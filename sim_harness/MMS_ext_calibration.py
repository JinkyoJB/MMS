"""
MMS Hand-Eye Calibration 시뮬레이션 - Isaac Sim 5.1.0 (Extension / Script Editor)

목적
----
실기(real)에서 쓰는 **Artec hand-eye 캘리브 파이프라인(ChArUco + cv2.solvePnP +
cv2.calibrateHandEye)** 이 진짜로 동작하는지를, ground-truth 를 아는 가상환경에서
먼저 검증한다. (cf. docs/main_flow.md §1.0, docs/artec_SDK/5_artec_hand_eye.md)

이 스크립트가 하는 일
---------------------
0. 스캔 대상(`MMS_SIM_OBJECT_PRIM`, 기본 /World/ScanTarget/TestObject) 비활성 + **ChArUco 보드를 얇은 박스 rigid body 로** 턴테이블
   위에서 **중력 낙하·안착**. 상단면에 ChArUco 텍스처.
1. **진짜 보드를 카메라가 렌더링**한 픽셀에서 cv2.aruco 검출 (점/포즈 주입 X).
2. 로봇을 artec home 에서 출발해 보드 위 **반구의 N 자세**로 이동(관절공간 구동).
3. 각 자세: get_rgba → ChArUco → solvePnP → T_C_tgt / rigid_ee FK → T_B_E.
4. cv2.calibrateHandEye → T_E_C 복원, USD GT 와 t_err/r_err 비교.

★ IK 는 pluggable — sim/real 괴리 최소화 (docs/main_flow.md §1.0)
   - 실물 로봇(192.168.1.210) 연결되면 **xArm SDK IK**(컨트롤러 계산, zero-gap)
   - 아니면 **utils/robot/xarm7_kinematics**(공칭 DH 해석 IK, USD 정합) fallback
   - PhysX 자코비안 크롤은 폐기(또아리 발생) → **artec home seed + seeded IK** 로 자연스런 분기 유지.

★ 카메라 프레임 규약 (빼먹으면 결과가 '틀린 것처럼' 보임)
   USD/Isaac 카메라: 광축 -Z, +Y up | OpenCV(solvePnP): 광축 +Z, +Y down
   → solvePnP 결과는 OpenCV 프레임. USD GT 비교 시 T_E_C_usd = T_E_C_ocv @ diag(1,-1,-1).

실행: VSCode Isaac 확장 코드러너 또는 GUI Script Editor (standalone 아님!).
필요: opencv-contrib-python (cv2.aruco).  결과/디버그 → captures_calib/
"""

import os
import sys
import math
import asyncio
import numpy as np
import omni.usd
from isaacsim.core.api import World
from isaacsim.core.utils.stage import open_stage
from isaacsim.core.api.robots import Robot
from isaacsim.core.prims import RigidPrim
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.sensors.camera import Camera
from pxr import Usd, UsdGeom, UsdShade, UsdPhysics, Sdf, Gf

# ── OpenCV (aruco) ───────────────────────────────────────────────────────────
try:
    import cv2
    _HAS_ARUCO = hasattr(cv2, "aruco")
except Exception:
    cv2 = None
    _HAS_ARUCO = False

# ── 해석 운동학·SDK import ────────────────────────────────────────────────────
# ── 경로 해석 (하드코딩 금지) ─────────────────────────────────────────────────
#   Isaac 트리에 복사/링크해 돌리면 __file__ 이 리포 밖일 수 있다 → 아래 순서로 탐색.
def _find_mms_repo():
    import os as _os
    cands = [_os.environ.get("MMS_ROOT"),
             _os.path.join(_os.path.dirname(_os.path.abspath(__file__)), ".."),
             _os.path.expanduser("~/workspace/4_인수인계서/A1_멀티모달스캔시스템_3D스캐닝경로생성/1_코드/MMS"),
             _os.path.expanduser("~/workspace/sync/2_Rapid_Digital_Twin/1_MMS/7_MMS_framework")]
    for c in cands:
        if c and _os.path.isfile(_os.path.join(c, "main_artec.py")):
            return _os.path.abspath(c)
    raise RuntimeError("MMS 리포를 찾지 못했다. export MMS_ROOT=/경로/MMS")


_MMS_REPO = _find_mms_repo()
if _MMS_REPO not in sys.path:
    sys.path.insert(0, _MMS_REPO)

# xArm SDK — pip 설치본(mms-env)이 있으면 그걸 쓰고, 없으면 소스 경로를 MMS_XARM_SDK 로
_XARM_SDK = os.environ.get("MMS_XARM_SDK", "")
if _XARM_SDK and _XARM_SDK not in sys.path:
    sys.path.insert(0, _XARM_SDK)

# 씬 USD — mms_paths 가 자산 루트를 해석한다 (MMS_ASSET_ROOT 로 override)
from mms_paths import asset as _asset  # noqa: E402

# 산출물 — 기본은 Isaac 트리, MMS_HARNESS_OUT 으로 변경 가능
_BASE_DIR = os.environ.get(
    "MMS_HARNESS_OUT",
    os.path.expanduser("~/isaacsim/standalone_examples/play/MMS"))
# ─────────────────────────────────────────────────────────────────────────────
_XARM_SDK = os.environ.get("MMS_XARM_SDK", "")
# xArm SDK 는 패키지(xarm.wrapper 등) → sys.path 로 추가
if _XARM_SDK not in sys.path:
    sys.path.insert(0, _XARM_SDK)
# ★ MMS 레포 모듈은 'utils' 패키지로 import 하면 Isaac 런타임의 동명 'utils' 와
#   충돌한다(과거 kin=None 버그). 아래 모듈들은 cv2/numpy/yaml 만 의존(레포 내부
#   import 없음)하므로 **파일 경로로 직접 로드**해 패키지명 충돌을 회피한다.
import importlib.util as _ilu


def _load_mod(name, relpath):
    """MMS 레포의 단일 모듈을 파일 경로로 직접 로드 ('utils' 패키지명 충돌 회피)."""
    path = os.path.join(_MMS_REPO, relpath)
    spec = _ilu.spec_from_file_location(name, path)
    mod = _ilu.module_from_spec(spec)
    sys.modules[name] = mod          # @dataclass/pickling 이 sys.modules 를 참조하므로 필수
    spec.loader.exec_module(mod)
    return mod


try:
    kin = _load_mod("mms_xarm7_kinematics", "utils/robot/xarm7_kinematics.py")
    _HAS_KIN = True
except Exception as _e:
    kin = None
    _HAS_KIN = False
    print(f"[CALIB][WARN] xarm7_kinematics 로드 실패: {_e}")

# ★ 실물과 공유하는 캘리브 코어 — sim 이 중복 구현하지 않고 MMS 모듈을 그대로 사용.
#   ArtecCharucoDetector: ChArUco 검출 + solvePnP → T_MC (실물/ sim 공용)
#   HandEyeCalibrator   : AX=XB 솔버 (실물/ sim 공용)
try:
    _chmod = _load_mod("mms_charuco_detector",
                       "mms_artec/utils/calibration/artec_charuco_detector.py")
    ArtecCharucoDetector = _chmod.ArtecCharucoDetector
    CharucoBoardSpec = _chmod.CharucoBoardSpec
    HandEyeCalibrator = _load_mod(
        "mms_hand_eye_calibrator", "utils/calibration/hand_eye_calibrator.py"
    ).HandEyeCalibrator
    # 기하 수학 + 자세생성 (MMS 라이브러리) — sim 은 이름만 바인딩해 그대로 사용
    geo = _load_mod("mms_handeye_geometry",
                    "mms_artec/utils/calibration/handeye_geometry.py")
    make_T = geo.make_T
    inv_T = geo.inv_T
    quat_wxyz_to_R = geo.quat_wxyz_to_R
    rot_angle_deg = geo.rot_angle_deg
    mat_to_pose6d_mm = geo.mat_to_pose6d_mm
    generate_hemisphere_poses = geo.generate_hemisphere_poses
    # pluggable IK provider (로봇제어) — MMS 라이브러리
    RobotIK = _load_mod("mms_ik_provider", "utils/robot/ik_provider.py").RobotIK
    _HAS_CALIB = True
except Exception as _e:
    ArtecCharucoDetector = CharucoBoardSpec = HandEyeCalibrator = None
    geo = RobotIK = None
    _HAS_CALIB = False
    print(f"[CALIB][WARN] MMS calibration/geometry/IK 모듈 로드 실패: {_e}")


# ── 상수 ───────────────────────────────────────────────────────────────────────
USD_PATH     = _asset("frame_xarm7_spider_turntable_v2/v3_scene.usd")
ROBOT_PRIM   = "/World/xarm7"
JOINTS_SCOPE = "/World/xarm7/joints"
CAMERA_PRIM  = "/World/xarm7/link7/tool/spider/Camera"
EE_LINK_NAME = "link7"
EE_LINK_PATH = f"{ROBOT_PRIM}/{EE_LINK_NAME}"

INITIAL_JOINT_POS = {f"joint{i}": 0.0 for i in range(1, 8)}

# Artec Space Spider 광학 (MMS_ext.py 와 동일)
SPIDER_HFOV_DEG         = 30.0
SPIDER_WORKING_DISTANCE = (0.2, 0.3)
SPIDER_FOCUS_DISTANCE   = 0.25
SCANNER_RESOLUTION      = (1280, 960)

# ── IK provider ───────────────────────────────────────────────────────────────
ROBOT_IP      = "192.168.1.210"     # 실물 xArm 컨트롤러
USE_XARM_SDK  = False               # 자체 해석 IK 사용(SDK IK 미사용)
# artec home (deg) — IsaacXArm.HOME_JOINTS_DEG["artec"]: 스캐너가 턴테이블 중심 위
# (~0.29m)에서 내려다보는 충돌-없는 자세. 캘리브 시작 seed.
ARTEC_HOME_JOINTS_DEG = [38.92, -48.70, -65.29, 21.22, 21.46, 72.70, -96.58]

# ── ChArUco 보드 (실기 spider 프리셋: 5×3, 20mm/15mm, DICT_4X4_50) ─────────────
#   CharucoBoardSpec 는 mm 규약 → solvePnP T_MC 가 mm (HandEyeCalibrator 기대치와 일치).
CH_SQUARES_X, CH_SQUARES_Y = 5, 3
CH_SQUARE_LEN_MM = 20.0
CH_MARKER_LEN_MM = 15.0
CH_ARUCO_DICT    = "DICT_4X4_50"
CH_BOARD_W_M     = CH_SQUARES_X * CH_SQUARE_LEN_MM / 1000.0   # 0.10
CH_BOARD_H_M     = CH_SQUARES_Y * CH_SQUARE_LEN_MM / 1000.0   # 0.06
CH_BOARD_THICK_M = 0.006
CH_IMG_PX        = (1000, 600)
BOARD_PRIM_PATH  = "/World/CharucoBoard"

# ── 보드 낙하/안착 (턴테이블 top=z 0.713, XY중심 (0.325,-0.022), 0.15×0.15m) ───
# v3 기준: 턴테이블 축 (0.365, 0), disc 상면 Z=0.665 (docs/hw_layout.md §2)
# 보드를 축 위에서 낙하시켜 disc 표면에 안착시킨다.
BOARD_DROP_XY      = (0.365, 0.0)
BOARD_DROP_Z       = 0.730          # 상면 +65mm 에서 낙하
BOARD_SETTLE_STEPS = 150
MARBLE_PRIM_PATH   = os.environ.get("MMS_SIM_OBJECT_PRIM", "/World/ScanTarget/TestObject")

# ── 캘리브 자세: 안착 보드 위 반구에서 내려다보기 ─────────────────────────────
CALIB_DISTANCE_M   = 0.25
CALIB_POLARS_DEG   = [0.0, 12.0, 22.0]
CALIB_AZIS_DEG     = [0.0, 72.0, 144.0, 216.0, 288.0]
CALIB_ROLLS_DEG    = [-20.0, 0.0, 20.0]
CALIB_DIST_JITTER  = [-0.02, 0.0, 0.02]
# look-at 기준 x축(이미지 up). 손목 roll 결정 — 관절공간 구동에선 IK 가 분기를 고르므로
# 또아리와 무관하나, 이미지 방향 일관성을 위해 유지.
CALIB_UP_HINT      = (1.0, 0.0, 0.0)

# ── 상태머신 타이밍 ───────────────────────────────────────────────────────────
SETTLE_STABLE_N    = 15        # 관절 수렴 연속 N 스텝이면 캡처
MOVE_TIMEOUT_N     = 400       # 한 자세 구동 최대 스텝
JOINT_SETTLE_TOL   = 0.01      # 관절 수렴 허용오차 (rad, ≈0.57°)
MIN_CH_CORNERS     = 6

FIX_DRIVE_GAINS = True
DRIVE_STIFFNESS = 2000.0
DRIVE_DAMPING   = 200.0
WIDEN_JOINT1_LIMIT_DEG = 175.0

PHYSICS_CB_NAME = "mms_calib_step"

# ★ VSCode Isaac 코드러너는 __file__ 을 확장 자신의 디렉토리로 잡아 출력이 엉뚱한
#   곳(.../code_editor/vscode/)에 묻힌다 → 스크립트 위치를 절대경로로 고정.
OUT_DIR   = os.path.join(_BASE_DIR, "captures_calib")
ASSET_DIR = os.path.join(_BASE_DIR, "calib_assets")
BOARD_PNG = os.path.join(ASSET_DIR, "charuco_5x3_20_15.png")

# ── 콘솔 로그 tee → 파일 (Isaac 확장 콘솔이 안 보여서; utf-8, tail -f 용) ───────
os.makedirs(OUT_DIR, exist_ok=True)
LOG_PATH = os.path.join(OUT_DIR, "calib_log.txt")
try:
    open(LOG_PATH, "w").close()          # 새 실행마다 초기화
except Exception:
    pass
_builtin_print = print
def print(*args, **kwargs):              # noqa: A001 — 모듈 전역 print 를 tee 로 교체
    _builtin_print(*args, **kwargs)
    try:
        with open(LOG_PATH, "a", encoding="utf-8") as _f:
            _f.write(" ".join(str(a) for a in args) + "\n")
    except Exception:
        pass

print(f"[CALIB] ===== run start =====  (kin={_HAS_KIN}, calib_modules={_HAS_CALIB})")
print(f"[CALIB] log file: {LOG_PATH}")

R_FLIP = np.diag([1.0, -1.0, -1.0])
T_FLIP = np.eye(4); T_FLIP[:3, :3] = R_FLIP


# 수학 유틸(make_T/inv_T/quat_wxyz_to_R/rot_angle_deg/mat_to_pose6d_mm/자세생성)과
# IK provider(RobotIK)는 MMS 라이브러리로 이동 — 상단에서 로드해 이름 바인딩됨:
#   mms_artec/utils/calibration/handeye_geometry.py
#   utils/robot/ik_provider.py
# 이 파일에는 Isaac/USD 환경 구현 코드만 남긴다.


# ============================================================================
# USD: joint bake / 카메라 광학 / ChArUco 보드 prim
# ============================================================================
def bake_joint_initial_state(stage, joints_scope, joint_pos):
    for jn, ang in joint_pos.items():
        jp = stage.GetPrimAtPath(f"{joints_scope}/{jn}")
        if not jp.IsValid():
            continue
        deg = math.degrees(ang)
        for attr in ("state:angular:physics:position",
                     "drive:angular:physics:targetPosition"):
            a = jp.GetAttribute(attr)
            if not a.IsValid():
                a = jp.CreateAttribute(attr, Sdf.ValueTypeNames.Float)
            a.Set(float(deg))


def configure_scanner_camera(stage, cam_path):
    cam = stage.GetPrimAtPath(cam_path)
    if not cam.IsValid():
        print(f"[CALIB][WARN] camera prim not found: {cam_path}")
        return

    def _attr(name, vtype, value):
        a = cam.GetAttribute(name)
        if not a.IsValid():
            a = cam.CreateAttribute(name, vtype)
        a.Set(value)

    h_ap_a = cam.GetAttribute("horizontalAperture")
    h_ap = float(h_ap_a.Get()) if h_ap_a.IsValid() and h_ap_a.Get() else 20.5
    focal = h_ap / (2.0 * math.tan(math.radians(SPIDER_HFOV_DEG) / 2.0))
    w, h = SCANNER_RESOLUTION
    v_ap = h_ap * (h / w)
    _attr("focalLength", Sdf.ValueTypeNames.Float, float(focal))
    _attr("horizontalAperture", Sdf.ValueTypeNames.Float, float(h_ap))
    _attr("verticalAperture", Sdf.ValueTypeNames.Float, float(v_ap))
    _attr("clippingRange", Sdf.ValueTypeNames.Float2,
          Gf.Vec2f(float(SPIDER_WORKING_DISTANCE[0]), float(SPIDER_WORKING_DISTANCE[1])))
    _attr("focusDistance", Sdf.ValueTypeNames.Float, float(SPIDER_FOCUS_DISTANCE))
    print(f"[CALIB] camera optics: focal={focal:.2f} hAp={h_ap:.2f} vAp={v_ap:.2f}")


def make_board_spec():
    """실물과 동일한 CharucoBoardSpec (mm). 검출기·텍스처 생성 공용 단일 정의."""
    return CharucoBoardSpec(
        squares_x=CH_SQUARES_X, squares_y=CH_SQUARES_Y,
        square_length_mm=CH_SQUARE_LEN_MM, marker_length_mm=CH_MARKER_LEN_MM,
        aruco_dict=getattr(cv2.aruco, CH_ARUCO_DICT))


def write_board_texture(board):
    """cv2 보드(spec.make_board())로 USD 텍스처 PNG 생성 (marginSize=0 → 평면과 1:1)."""
    os.makedirs(ASSET_DIR, exist_ok=True)
    img = board.generateImage(CH_IMG_PX, marginSize=0, borderBits=1)
    cv2.imwrite(BOARD_PNG, img)
    print(f"[CALIB] ChArUco board image: {BOARD_PNG} ({img.shape[1]}x{img.shape[0]})")


def _bind_texture_material(stage, mesh, mat_root, png_path):
    mat = UsdShade.Material.Define(stage, mat_root)
    surf = UsdShade.Shader.Define(stage, mat_root + "/surf")
    surf.CreateIdAttr("UsdPreviewSurface")
    surf.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.9)
    surf.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
    reader = UsdShade.Shader.Define(stage, mat_root + "/stReader")
    reader.CreateIdAttr("UsdPrimvarReader_float2")
    reader.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
    tex = UsdShade.Shader.Define(stage, mat_root + "/tex")
    tex.CreateIdAttr("UsdUVTexture")
    tex.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(png_path)
    tex.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(
        reader.ConnectableAPI(), "result")
    tex.CreateInput("sourceColorSpace", Sdf.ValueTypeNames.Token).Set("sRGB")
    tex.CreateInput("wrapS", Sdf.ValueTypeNames.Token).Set("clamp")
    tex.CreateInput("wrapT", Sdf.ValueTypeNames.Token).Set("clamp")
    tex.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)
    surf.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(
        tex.ConnectableAPI(), "rgb")
    surf.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(
        tex.ConnectableAPI(), "rgb")
    surf.CreateOutput("surface", Sdf.ValueTypeNames.Token)
    mat.CreateSurfaceOutput().ConnectToSource(surf.ConnectableAPI(), "surface")
    UsdShade.MaterialBindingAPI(mesh).Bind(mat)


def create_charuco_board_rigid(stage, root_path, png_path, w_m, h_m, thick_m, position):
    """얇은 박스 ChArUco 보드 rigid body (중력 낙하). 상단면 텍스처."""
    root = UsdGeom.Xform.Define(stage, root_path)
    root.ClearXformOpOrder()
    root.AddTranslateOp().Set(Gf.Vec3d(*[float(v) for v in position]))
    root.AddOrientOp().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    rprim = root.GetPrim()
    UsdPhysics.RigidBodyAPI.Apply(rprim)
    UsdPhysics.MassAPI.Apply(rprim).CreateMassAttr(0.05)
    cube = UsdGeom.Cube.Define(stage, root_path + "/collision")
    cube.GetSizeAttr().Set(1.0)
    cube.AddScaleOp().Set(Gf.Vec3f(float(w_m), float(h_m), float(thick_m)))
    cube.CreateDisplayColorAttr([Gf.Vec3f(0.92, 0.92, 0.92)])
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    z = thick_m / 2.0 + 0.0008
    mesh = UsdGeom.Mesh.Define(stage, root_path + "/top")
    hw, hh = w_m / 2.0, h_m / 2.0
    mesh.CreatePointsAttr([Gf.Vec3f(-hw, -hh, z), Gf.Vec3f(hw, -hh, z),
                           Gf.Vec3f(hw, hh, z), Gf.Vec3f(-hw, hh, z)])
    mesh.CreateFaceVertexCountsAttr([4])
    mesh.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    mesh.CreateNormalsAttr([Gf.Vec3f(0, 0, 1)] * 4)
    mesh.SetNormalsInterpolation("vertex")
    UsdGeom.PrimvarsAPI(mesh).CreatePrimvar(
        "st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.varying).Set(
        [Gf.Vec2f(0, 0), Gf.Vec2f(1, 0), Gf.Vec2f(1, 1), Gf.Vec2f(0, 1)])
    _bind_texture_material(stage, mesh, root_path + "/mat", png_path)
    print(f"[CALIB] ChArUco 보드(rigid) @ {np.round(position,3).tolist()} "
          f"(낙하 {BOARD_SETTLE_STEPS} step)")


# ============================================================================
# 카메라 intrinsics / prim pose
# ============================================================================
def get_camera_K(camera, stage):
    try:
        K = np.asarray(camera.get_intrinsics_matrix(), dtype=np.float64)
        if K.shape == (3, 3) and K[0, 0] > 1.0:
            return K
    except Exception:
        pass
    cam = stage.GetPrimAtPath(CAMERA_PRIM)
    focal = float(cam.GetAttribute("focalLength").Get())
    h_ap = float(cam.GetAttribute("horizontalAperture").Get())
    v_ap = float(cam.GetAttribute("verticalAperture").Get())
    w, h = SCANNER_RESOLUTION
    return np.array([[w*focal/h_ap, 0, w/2.0],
                     [0, h*focal/v_ap, h/2.0], [0, 0, 1]], dtype=np.float64)


def get_prim_world_T(stage, xform_cache, path):
    prim = stage.GetPrimAtPath(path)
    if not prim.IsValid():
        return None
    m = xform_cache.GetLocalToWorldTransform(prim).RemoveScaleShear()
    R = np.array(m.ExtractRotationMatrix(), dtype=np.float64).T
    t = np.array(m.ExtractTranslation(), dtype=np.float64)
    return make_T(R, t)


# (검출은 MMS 의 ArtecCharucoDetector, 풀이는 HandEyeCalibrator 를 그대로 사용 —
#  sim 중복 구현 제거. 여기엔 sim 전용 'GT 대비 검증'만 남긴다.)


# ============================================================================
# Hand-eye 풀이(MMS HandEyeCalibrator) + sim 전용 GT 비교
# ============================================================================
def solve_and_report(ctx):
    cal = ctx["calibrator"]
    if cal.n_samples < 3:
        print(f"[CALIB][ERROR] 샘플 부족 ({cal.n_samples}). 최소 3. 자세/검출 확인.")
        return
    T_EC_gt = ctx["T_EC_gt"]
    print("\n[CALIB] ===== HAND-EYE 결과 (ground-truth 대비) =====")
    print(f"[CALIB] 유효 샘플: {cal.n_samples} | IK: {ctx['ik_mode']}")
    print(f"[CALIB] GT  T_E_C  t(mm)={np.round(T_EC_gt[:3,3]*1000,2).tolist()}")
    try:
        # HandEyeCalibrator: T_EC = E→C(EE-in-camera), OpenCV cam frame, m
        T_EC_ocv = cal.calibrate()
    except Exception as exc:
        print(f"[CALIB][ERROR] HandEyeCalibrator.calibrate 실패: {exc}")
        return
    # OpenCV cam → USD cam 환산. T_EC 는 카메라가 출력(행)측 → flip 은 왼쪽곱.
    T_EC_est = T_FLIP @ T_EC_ocv
    t_err = float(np.linalg.norm(T_EC_est[:3, 3] - T_EC_gt[:3, 3]) * 1000.0)
    r_err = rot_angle_deg(T_EC_est[:3, :3], T_EC_gt[:3, :3])
    print(f"[CALIB] >>> t_err={t_err:.2f} mm  r_err={r_err:.2f}°")
    print(f"[CALIB]     EST t(mm)={np.round(T_EC_est[:3,3]*1000,2).tolist()}")
    verdict = "PASS ✅" if (t_err < 5.0 and r_err < 2.0) else "CHECK ⚠"
    print(f"[CALIB]     판정: {verdict}")
    os.makedirs(OUT_DIR, exist_ok=True)
    np.savez(os.path.join(OUT_DIR, "handeye_result.npz"),
             T_EC_gt=T_EC_gt, T_EC_est=T_EC_est, t_err_mm=t_err, r_err_deg=r_err,
             n_samples=cal.n_samples, ik_mode=ctx["ik_mode"])
    print("[CALIB] ===== COMPLETE =====\n")


# ============================================================================
# 자세 생성 — MMS handeye_geometry.generate_hemisphere_poses 에 config 만 주입
# ============================================================================
def build_calibration_poses(board_center, board_normal):
    poses = generate_hemisphere_poses(
        board_center, board_normal, _ctx["T_EC_gt"],
        distance_m=CALIB_DISTANCE_M, polars_deg=CALIB_POLARS_DEG,
        azis_deg=CALIB_AZIS_DEG, rolls_deg=CALIB_ROLLS_DEG,
        dist_jitter=CALIB_DIST_JITTER, up_hint=CALIB_UP_HINT)
    print(f"[CALIB] 캘리브 자세: {len(poses)} (보드중심 {np.round(board_center,3).tolist()})")
    return poses


# ============================================================================
# 런타임 상태 + 관절공간 구동 상태머신
# ============================================================================
_ctx = {
    "world": None, "stage": None, "robot": None, "camera": None,
    "rigid_ee": None, "board_rigid": None, "provider": None, "ik_mode": "?",
    "dof_idx": None, "T_W_base": None, "K": None, "dist": None, "T_EC_gt": None,
    "detector": None, "calibrator": None,
    "poses": [], "pose_idx": 0, "seed_q": None, "target_q": None,
    "board_center": None, "phase": "BOARD_SETTLE", "phase_step": 0, "stable_n": 0,
    "step": 0,
}


def drive_joints(q_kin):
    """kin 관절각(rad,7) → sim 아티큘레이션 관절 타깃."""
    robot = _ctx["robot"]; dof_idx = _ctx["dof_idx"]
    q_full = np.asarray(robot.get_joint_positions(), dtype=float).copy()
    for i, di in enumerate(dof_idx):
        q_full[di] = q_kin[i]
    robot.apply_action(ArticulationAction(joint_positions=q_full))


def _joint_err_to_target():
    robot = _ctx["robot"]; dof_idx = _ctx["dof_idx"]; tq = _ctx["target_q"]
    q = np.asarray(robot.get_joint_positions(), dtype=float)
    return float(np.max([abs(q[di] - tq[i]) for i, di in enumerate(dof_idx)]))


def _ee_world_pose():
    pos, quat = _ctx["rigid_ee"].get_world_poses()
    return np.asarray(pos[0], float), np.asarray(quat[0], float)


def _finalize_board_and_poses():
    pos, quat = _ctx["board_rigid"].get_world_poses()
    p = np.asarray(pos[0], float)
    R = quat_wxyz_to_R(np.asarray(quat[0], float))
    normal = R[:, 2]
    center = p + R @ np.array([0.0, 0.0, CH_BOARD_THICK_M / 2.0 + 0.0008])
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, abs(normal[2])))))
    print(f"[CALIB] 보드 안착: pos={np.round(p,3).tolist()} top_z={center[2]:.3f} 기울기={tilt:.1f}°")
    if normal[2] < 0.5:
        print("[CALIB][WARN] 보드 뒤집힘/큰 기울기 — 검출 실패 가능")
    _ctx["board_center"] = center
    _ctx["poses"] = build_calibration_poses(center, normal)
    return len(_ctx["poses"]) > 0


def _start_pose(idx):
    """feasible 한 다음 자세를 찾아 IK 풀고 관절 구동. 없으면 SOLVE 로."""
    poses = _ctx["poses"]
    while idx < len(poses):
        T_W_E = poses[idx]
        T_base_E = inv_T(_ctx["T_W_base"]) @ T_W_E
        pose6d = mat_to_pose6d_mm(T_base_E)
        q, ok = _ctx["provider"].ik(pose6d, seed=_ctx["seed_q"])
        if ok:
            _ctx["pose_idx"] = idx
            _ctx["target_q"] = np.asarray(q, float)
            _ctx["seed_q"] = np.asarray(q, float)
            drive_joints(q)
            _ctx["phase"] = "SETTLE"; _ctx["phase_step"] = 0; _ctx["stable_n"] = 0
            print(f"[CALIB] → pose {idx+1}/{len(poses)} IK ok, 구동")
            return
        print(f"[CALIB]   pose {idx+1}: IK 실패(미도달) — skip")
        idx += 1
    print("[CALIB] 더 이상 도달 가능한 자세 없음 → SOLVE")
    _ctx["phase"] = "SOLVE"; _ctx["phase_step"] = 0


def _do_capture(idx):
    """현재 자세 렌더 → MMS ArtecCharucoDetector 검출 → HandEyeCalibrator 샘플 추가."""
    rgba = _ctx["camera"].get_rgba()
    os.makedirs(OUT_DIR, exist_ok=True)
    if rgba is None or getattr(rgba, "size", 0) == 0:
        print(f"[CALIB]   pose {idx+1}: 렌더 미준비 — skip")
        return
    img_rgb = np.asarray(rgba)[:, :, :3].astype(np.uint8)
    det = _ctx["detector"].detect(img_rgb, min_corners=MIN_CH_CORNERS,
                                  draw_debug=True, verbose=False)
    if det is None or det.n_corners < MIN_CH_CORNERS:
        n = 0 if det is None else det.n_corners
        print(f"[CALIB]   pose {idx+1}: 검출 실패 (corners={n}) — skip")
        if det is not None and det.debug_image is not None:
            cv2.imwrite(os.path.join(OUT_DIR, f"fail_{idx:02d}.png"), det.debug_image)
        return
    # T_MC: board→camera (mm, OpenCV cam) | T_EB: link7→base(=world) (m)
    p, q = _ee_world_pose()
    T_EB = make_T(quat_wxyz_to_R(q), p)
    _ctx["calibrator"].add_sample(T_EB, det.T_MC)
    if det.debug_image is not None:                       # 이미 BGR
        cv2.imwrite(os.path.join(OUT_DIR, f"ok_{idx:02d}.png"), det.debug_image)
    print(f"[CALIB]   pose {idx+1}: ✓ corners={det.n_corners} "
          f"reproj={det.procrustes_rmse_mm:.2f}px (샘플 {_ctx['calibrator'].n_samples})")


def _on_physics_step(step_size):
    phase = _ctx["phase"]
    _ctx["phase_step"] += 1

    if phase == "BOARD_SETTLE":
        if _ctx["phase_step"] >= BOARD_SETTLE_STEPS:
            if _finalize_board_and_poses():
                _start_pose(0)
            else:
                print("[CALIB][ERROR] 자세 생성 실패 → 중단")
                _ctx["phase"] = "DONE"

    elif phase == "SETTLE":
        if _joint_err_to_target() < JOINT_SETTLE_TOL:
            _ctx["stable_n"] += 1
        else:
            _ctx["stable_n"] = 0
        if _ctx["stable_n"] >= SETTLE_STABLE_N or _ctx["phase_step"] >= MOVE_TIMEOUT_N:
            if _ctx["phase_step"] >= MOVE_TIMEOUT_N:
                print(f"[CALIB]   pose {_ctx['pose_idx']+1}: 구동 타임아웃 — 현 자세 캡처")
            _do_capture(_ctx["pose_idx"])
            _start_pose(_ctx["pose_idx"] + 1)

    elif phase == "SOLVE":
        solve_and_report(_ctx)
        _ctx["phase"] = "DONE"

    _ctx["step"] += 1


# ============================================================================
# setup
# ============================================================================
def configure_joint_drives(robot):
    view = robot._articulation_view
    nd = view.num_dof
    view.set_gains(kps=np.full((1, nd), DRIVE_STIFFNESS, dtype=np.float32),
                   kds=np.full((1, nd), DRIVE_DAMPING, dtype=np.float32))


def build_dof_index(robot):
    names = list(robot.dof_names)
    idx = []
    for i in range(1, 8):
        nm = f"joint{i}"
        idx.append(names.index(nm) if nm in names else i - 1)
    return idx


async def setup_async():
    if not _HAS_ARUCO:
        print("[CALIB][ERROR] cv2.aruco 없음:")
        print("    <isaac>/python.sh -m pip install opencv-contrib-python")
        return
    if not _HAS_CALIB:
        print("[CALIB][ERROR] MMS calibration 모듈(ArtecCharucoDetector/HandEyeCalibrator) "
              "로드 실패 — 위 WARN 확인. 중단.")
        return
    if not _HAS_KIN:
        print("[CALIB][ERROR] xarm7_kinematics 로드 실패 — 위 WARN 확인. 중단.")
        return

    if World.instance() is not None:
        try:
            World.instance().clear_all_callbacks()
        except Exception:
            pass
        World.clear_instance()

    print(f"[CALIB] Opening USD: {USD_PATH}")
    open_stage(usd_path=USD_PATH)
    stage = omni.usd.get_context().get_stage()

    if WIDEN_JOINT1_LIMIT_DEG is not None:
        j1 = stage.GetPrimAtPath(f"{JOINTS_SCOPE}/joint1")
        if j1.IsValid():
            j1.GetAttribute("physics:lowerLimit").Set(-float(WIDEN_JOINT1_LIMIT_DEG))
            j1.GetAttribute("physics:upperLimit").Set(float(WIDEN_JOINT1_LIMIT_DEG))

    bake_joint_initial_state(stage, JOINTS_SCOPE, INITIAL_JOINT_POS)
    configure_scanner_camera(stage, CAMERA_PRIM)

    marble = stage.GetPrimAtPath(MARBLE_PRIM_PATH)
    if marble.IsValid():
        marble.SetActive(False)
        print(f"[CALIB] 마블 비활성화: {MARBLE_PRIM_PATH}")

    # 보드: 실물과 동일한 spec → 텍스처 PNG + rigid body (검출기도 같은 spec 사용)
    board_spec = make_board_spec()
    write_board_texture(board_spec.make_board())
    create_charuco_board_rigid(stage, BOARD_PRIM_PATH, BOARD_PNG,
                               CH_BOARD_W_M, CH_BOARD_H_M, CH_BOARD_THICK_M,
                               (BOARD_DROP_XY[0], BOARD_DROP_XY[1], BOARD_DROP_Z))

    world = World(physics_dt=1.0/60.0, rendering_dt=1.0/60.0, stage_units_in_meters=1.0)
    if world.get_physics_context() is None:
        await world.initialize_simulation_context_async()
    robot = world.scene.add(Robot(prim_path=ROBOT_PRIM, name="xarm7"))
    scanner_cam = Camera(prim_path=CAMERA_PRIM, resolution=SCANNER_RESOLUTION)

    await world.reset_async()
    if FIX_DRIVE_GAINS:
        configure_joint_drives(robot)
    scanner_cam.initialize()

    # intrinsics / GT T_E_C / base / dof map
    xc = UsdGeom.XformCache(Usd.TimeCode.Default())
    T_W_E = get_prim_world_T(stage, xc, EE_LINK_PATH)
    T_W_C = get_prim_world_T(stage, xc, CAMERA_PRIM)
    T_W_base = get_prim_world_T(stage, xc, ROBOT_PRIM)
    # 시스템 T_EC 규약 = E→C = EE-in-camera (compute_T_CB: x_C = T_EC·x_E).
    # HandEyeCalibrator 가 반환하는 것과 동일 규약 → inv(T_W_C)@T_W_E.
    T_EC_gt = inv_T(T_W_C) @ T_W_E
    K = get_camera_K(scanner_cam, stage)

    board_rigid = RigidPrim(prim_paths_expr=BOARD_PRIM_PATH)
    board_rigid.initialize()

    provider = RobotIK(kin=kin, robot_ip=ROBOT_IP, use_sdk=USE_XARM_SDK,
                       xarm_sdk_path=_XARM_SDK, logger=print)
    dof_idx = build_dof_index(robot)
    home_q = np.radians(ARTEC_HOME_JOINTS_DEG)

    # 실물과 공유하는 검출기/솔버 (sim K 주입 → PnP 경로)
    detector = ArtecCharucoDetector(
        board_spec=board_spec, intrinsic={"K": K.tolist(), "dist": [0, 0, 0, 0, 0]})
    calibrator = HandEyeCalibrator()

    _ctx.update(world=world, stage=stage, robot=robot, camera=scanner_cam,
                rigid_ee=RigidPrim(prim_paths_expr=EE_LINK_PATH), board_rigid=board_rigid,
                provider=provider, ik_mode=provider.mode, dof_idx=dof_idx,
                T_W_base=T_W_base, K=K, dist=np.zeros(5), T_EC_gt=T_EC_gt,
                detector=detector, calibrator=calibrator,
                poses=[], pose_idx=0, seed_q=home_q.copy(), target_q=home_q.copy(),
                phase="BOARD_SETTLE", phase_step=0, stable_n=0, step=0)
    _ctx["rigid_ee"].initialize()

    # artec home 으로 이동 (보드 낙하 동안 함께 안정)
    drive_joints(home_q)

    print(f"[CALIB] K=\n{np.round(K,1)}")
    print(f"[CALIB] GT T_E_C trans(mm)={np.round(T_EC_gt[:3,3]*1000,2).tolist()}")
    print(f"[CALIB] dof_idx(joint1..7)={dof_idx} | home(deg)={ARTEC_HOME_JOINTS_DEG}")

    world.add_physics_callback(PHYSICS_CB_NAME, _on_physics_step)
    await world.play_async()
    print("[CALIB] 시작: home 이동 + 보드 안착 → 자세 순회(관절공간) → ChArUco hand-eye.")


def stop():
    world = _ctx.get("world") or World.instance()
    if world is not None:
        try:
            world.remove_physics_callback(PHYSICS_CB_NAME)
        except Exception:
            pass
        world.stop()
    print("[CALIB] Stopped.")


asyncio.ensure_future(setup_async())
