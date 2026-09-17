"""
MMS lookaround → nbv 시뮬레이션 (부족면 NBV 보강) - Isaac Sim 5.1.0 (Extension)

목적
----
lookaround(5면 GT 누적) → **nbv(부족면 NBV 보강)** 를 sim 에서 end-to-end 검증.
docs/4_nbv.md. lookaround 은 MMS_ext_lookaround.py 와 동일(턴테이블 회전·로봇 고정·−θ 누적).

★ "특정 위치에서 부분적으로 스캔 안되는 환경" = **입사각(incidence) 필터**.
   구조광 스캐너는 표면을 grazing(스침)으로 보면 실패한다. lookaround 은 로봇이 **낮은
   측면 자세**로 고정 → **윗면**을 grazing 으로만 봄 → 윗면 점이 안 쌓임 = **윗면 gap**.
   (옆면은 정면으로 보여 잘 쌓임.) 이 gap 은 viewpoint 의존 — 로봇을 들어올려 내려다보면 잡힌다.

흐름
----
0. 로봇 낮은 측면 자세 고정, 대상물 box 세움.
1. [lookaround] 턴테이블 θ 회전(0..350°,10°) → 캡처 → **입사각 필터** → −θ 누적. 옆면 4 OK, 윗면 gap.
2. [lookaround 판정] 윗면 점 희박(gap) 확인.
3. [nbv] 누적점에서 **윗면 gap 검출** → **NBV 포즈**(들어올려 내려다봄) 후보 생성 →
   **해석 IK + 궤적(swept) 충돌검사** 통과 + **관절이동 최소** 자세 선택 → 로봇 이동.
4. [nbv 캡처] 윗면을 정면(낮은 입사각)으로 봄 → 윗면 점 누적 = **gap 메움**.
5. [nbv 판정] 윗면 점 급증 → PASS.

자기완결 모듈(kin, geo)만 로드(Isaac utils 충돌 회피). 입사각필터·gap검출·NBV·swept충돌은
인라인(프로덕션 = utils/nbv/nbv_core.py + robot_collision.swept_pose_collision, 별도 단위검증).
실행: VSCode Isaac 확장/Script Editor. 결과 → captures_nbv/.
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
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.sensors.camera import Camera
from pxr import Usd, UsdGeom, Sdf, Gf

# ── MMS 공유 코어 (파일경로 로드, 자기완결 모듈만) ────────────────────────────
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
import importlib.util as _ilu


def _load_mod(name, relpath):
    spec = _ilu.spec_from_file_location(name, os.path.join(_MMS_REPO, relpath))
    mod = _ilu.module_from_spec(spec)
    sys.modules[name] = mod
    spec.loader.exec_module(mod)
    return mod


_XARM_SDK = os.environ.get("MMS_XARM_SDK", "")
if _XARM_SDK not in sys.path:
    sys.path.insert(0, _XARM_SDK)
try:
    kin = _load_mod("mms_xarm7_kinematics", "utils/robot/xarm7_kinematics.py")
    geo = _load_mod("mms_handeye_geometry", "mms_artec/utils/calibration/handeye_geometry.py")
    cgeom = _load_mod("mms_collision_geometry", "utils/collision/geometry.py")  # seg_seg_distance
    make_T, inv_T = geo.make_T, geo.inv_T
    look_at_camera, mat_to_pose6d_mm = geo.look_at_camera, geo.mat_to_pose6d_mm
    RobotIK = _load_mod("mms_ik_provider", "utils/robot/ik_provider.py").RobotIK
    _HAS_CORE = True
except Exception as _e:
    _HAS_CORE = False
    print(f"[NBV][WARN] MMS 코어 로드 실패: {_e}")

try:
    from isaacsim.util.debug_draw import _debug_draw
    _draw = _debug_draw.acquire_debug_draw_interface()
except Exception:
    _draw = None


# ── 상수 ───────────────────────────────────────────────────────────────────────
USD_PATH     = _asset("frame_xarm7_spider_turntable_v2/v3_scene.usd")
ROBOT_PRIM   = "/World/xarm7"
JOINTS_SCOPE = "/World/xarm7/joints"
CAMERA_PRIM  = "/World/xarm7/link7/tool/spider/Camera"
SCANNER_PRIM = "/World/xarm7/link7/Artec_Space_Spider_mm"     # 자가충돌용 실 mesh
LINK7_PRIM   = "/World/xarm7/link7"
MARBLE_PRIM_PATH = os.environ.get("MMS_SIM_OBJECT_PRIM", "/World/ScanTarget/TestObject")
TURNTABLE_MESH = "/World/frame/turntable_disc"
OBJ_PATH     = "/World/nbvObject"

INITIAL_JOINT_POS = {f"joint{i}": 0.0 for i in range(1, 8)}

SPIDER_HFOV_DEG, SPIDER_CLIP = 30.0, (0.15, 0.45)
SPIDER_FOCUS_DISTANCE, SCANNER_RESOLUTION = 0.25, (1280, 960)
ROBOT_IP, USE_XARM_SDK = "192.168.1.210", False
ARTEC_HOME_JOINTS_DEG = [38.92, -48.70, -65.29, 21.22, 21.46, 72.70, -96.58]

# lookaround 측면 관측 — elevation 30°(팔 덜 뻗음=중력토크↓, 진동 억제). 윗면 gap 은
# 입사각 임계(아래)로 유도하므로 굳이 더 낮출 필요 없음. 도달 가능 azimuth 채택.
VIEW_EL_DEG    = 30.0
VIEW_STANDOFF  = 0.26
VIEW_AZIS_DEG  = [0.0, 30.0, -30.0, 60.0, -60.0, 90.0, -90.0, 180.0]
VOXEL_M        = 0.002

# ★ 입사각 필터 — 표면 법선과 카메라 시선의 각이 이보다 크면(grazing) 캡처 실패로 모델.
# el=30° 에서 윗면(≈60° 입사) 은 버리고 측면(≈30°) 은 살리도록 50° 로.
MAX_INCIDENCE_DEG = 50.0

# nbv NBV — 윗면 gap 을 **옆에서 비스듬히** 내려다봄. 곧장 위로 뻗는 무리한(특이점/접촉)
# 자세를 피하려 낮은 앙각 우선(50~65°, 윗면 입사각<50° 라 여전히 잡힘). min-motion 이 측면
# 자세(el=30)에 가까운 낮은 앙각을 자연히 선호. standoff 키워 스캐너-물체 여유 확보.
NBV_STANDOFF       = 0.27
NBV_APPROACH_ELS   = [55.0, 50.0, 60.0, 65.0]
NBV_APPROACH_AZIS  = [0.0, 30.0, -30.0, 60.0, -60.0, 90.0, -90.0, 180.0]
NBV_SWEPT_STEPS    = 12
# 인라인 충돌 (turntable 원기둥 + 대상물 box + 자가충돌). robot_collision 의 sim 미러.
DISC_RADIUS_M, BODY_H_M = 0.15, 0.20
# robot_collision.DEFAULT_LINK_RADII 기반 + 스캐너(마지막)는 Spider 부피 반영해 키움.
# (스캐너는 home 에서도 elbow 와 ~123mm 라 너무 키우면 정상자세 오검출 — 0.075 로 절충.)
LINK_RADII = np.array([0.060, 0.060, 0.055, 0.050, 0.050, 0.050, 0.075])
SELF_GAP, SELF_SCALE = 3, 0.7   # 비인접(≥3) 캡슐쌍만, 반경 scale (robot_collision.self_collision)
# ── 충돌 금지 원기둥 (turntable 위 특정 영역; 회피 + 시각화) ────────────────────
# ★ 이건 sim 검증용 **미러**. real/production 은 robot_collision.CollisionWorld.add_cylinder +
#   artec_multipass_scan_session(nbv_keepout_enable/radius_mm/height_mm/center_xy) 가 본체.
#   여기 값은 그 production 설정과 일치시킬 것(real 에 실제 적용되는 건 production 쪽).
KEEPOUT_ENABLE   = True
KEEPOUT_CENTER   = None         # (x,y) world. None = 턴테이블(disc) 중심 사용
KEEPOUT_MATCH_TURNTABLE = True  # True = 반경을 턴테이블 mesh 지름과 동일하게 (USD 에서 측정)
KEEPOUT_RADIUS_M = 0.06         # MATCH_TURNTABLE=False 일 때 또는 측정실패 시 fallback 반경
KEEPOUT_HEIGHT_M = 0.12         # disc top 위로 높이
KEEPOUT_COLOR    = (1.0, 0.35, 0.35)   # 빨강 wireframe
# 스캐너(실 mesh)는 **베이스 링크**에만 검사 — forearm 은 스캐너가 자연히 가까워(오검출).
# cap0=link1-2, cap1=link2-3. "스캐너↔link1 reach-over" 가 여기 잡힌다.
SCANNER_CHECK_CAPS = (0, 1)

DISC_TOP_Z   = 0.713
# ── 대상물 형상 (다양한 shape 테스트) — 한 번 RUN 당 하나. 값만 바꿔 재실행 ──
OBJECT_SHAPE = "lshape"            # box | cylinder | sphere | cone | lshape | stepped
ADD_MARKER_SHAPES = ("box", "cylinder", "stepped")   # 비대칭 marker 부착 형상
OBJ_W, OBJ_D, OBJ_H = 0.060, 0.040, 0.070            # _configure_shape() 가 형상별로 덮어씀(bbox)

N_THETA        = 36
WAIT_PER_THETA = 16             # physics_dt 1/120 기준 (이전 8 @ 1/60)
OBJ_Z_MIN_OFF  = 0.006
OBJ_Z_MAX_OFF  = 0.12
OBJ_XY_CROP    = 0.06
TOP_BAND_M     = 0.010          # 윗면 판정 밴드 (z_top - 이 값 ~ z_top)

WIDEN_JOINT1_LIMIT_DEG = 175.0
# ★ 덜덜거림 대책 — 핵심은 **물리 스텝을 잘게(PHYSICS_DT)** 해서 강성 PD 의 수치 불안정 제거.
#   그 위에서 댐핑은 중간값(800): 너무 낮으면 편 자세 언더댐핑, 너무 높으면(2000) 속도노이즈
#   증폭으로 chatter. USD 원본 게인은 약해 떨리므로 override 유지(stiffness 2000 필요).
FIX_DRIVE_GAINS, DRIVE_STIFFNESS, DRIVE_DAMPING = True, 2000.0, 800.0
PHYSICS_DT = 1.0 / 120.0        # 1/60→1/120: 강성 드라이브 수치 안정 (스텝상수도 ×2)
WARMUP_STEPS, SETTLE_STABLE_N, MOVE_TIMEOUT_N, JOINT_SETTLE_TOL = 120, 24, 800, 0.01
PHYSICS_CB_NAME = "mms_nbv_step"

OUT_DIR   = os.path.join(_BASE_DIR, "captures_nbv")
os.makedirs(OUT_DIR, exist_ok=True)
LOG_PATH  = os.path.join(OUT_DIR, "calib_log.txt")
try:
    open(LOG_PATH, "w").close()
except Exception:
    pass
_builtin_print = print
def print(*a, **k):                      # noqa: A001 — tee → 파일
    _builtin_print(*a, **k)
    try:
        with open(LOG_PATH, "a", encoding="utf-8") as f:
            f.write(" ".join(str(x) for x in a) + "\n")
    except Exception:
        pass

print(f"[NBV] ===== run start =====  (core={_HAS_CORE})")


# ── USD/Isaac 헬퍼 (lookaround 과 동일) ───────────────────────────────────────────
def bake_joint_initial_state(stage, scope, joint_pos):
    for jn, ang in joint_pos.items():
        jp = stage.GetPrimAtPath(f"{scope}/{jn}")
        if not jp.IsValid():
            continue
        for attr in ("state:angular:physics:position", "drive:angular:physics:targetPosition"):
            a = jp.GetAttribute(attr)
            if not a.IsValid():
                a = jp.CreateAttribute(attr, Sdf.ValueTypeNames.Float)
            a.Set(float(math.degrees(ang)))


def configure_scanner_camera(stage, cam_path):
    cam = stage.GetPrimAtPath(cam_path)
    if not cam.IsValid():
        return

    def _attr(n, vt, v):
        a = cam.GetAttribute(n)
        if not a.IsValid():
            a = cam.CreateAttribute(n, vt)
        a.Set(v)

    h_ap_a = cam.GetAttribute("horizontalAperture")
    h_ap = float(h_ap_a.Get()) if h_ap_a.IsValid() and h_ap_a.Get() else 20.5
    focal = h_ap / (2.0 * math.tan(math.radians(SPIDER_HFOV_DEG) / 2.0))
    w, h = SCANNER_RESOLUTION
    _attr("focalLength", Sdf.ValueTypeNames.Float, float(focal))
    _attr("horizontalAperture", Sdf.ValueTypeNames.Float, float(h_ap))
    _attr("verticalAperture", Sdf.ValueTypeNames.Float, float(h_ap * h / w))
    _attr("clippingRange", Sdf.ValueTypeNames.Float2,
          Gf.Vec2f(float(SPIDER_CLIP[0]), float(SPIDER_CLIP[1])))
    _attr("focusDistance", Sdf.ValueTypeNames.Float, float(SPIDER_FOCUS_DISTANCE))


def get_prim_world_T(stage, xc, path):
    prim = stage.GetPrimAtPath(path)
    if not prim.IsValid():
        return None
    m = xc.GetLocalToWorldTransform(prim).RemoveScaleShear()
    R = np.array(m.ExtractRotationMatrix(), dtype=np.float64).T
    return make_T(R, np.array(m.ExtractTranslation(), dtype=np.float64))


def _compute_scanner_corners_l7(stage, xc):
    """스캐너 mesh 의 world AABB 8 corner 를 **link7 로컬 프레임**으로 변환해 반환 (8,3).
    candidate 자세마다 link7 FK 로 다시 world 로 보내 충돌검사에 쓴다(실 부피 반영)."""
    try:
        sp = stage.GetPrimAtPath(SCANNER_PRIM)
        if not sp.IsValid():
            print(f"[NBV][WARN] 스캐너 prim 없음: {SCANNER_PRIM}")
            return None
        bbc = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                                [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
        wb = bbc.ComputeWorldBound(sp)            # GfBBox3d (oriented)
        rng = wb.GetRange()                       # 박스-로컬 범위(타이트)
        Mnp = np.array(wb.GetMatrix(), float)     # 박스-로컬 → world (Gf 행벡터 규약)
        mn = np.array(rng.GetMin(), float)
        mx = np.array(rng.GetMax(), float)
        lc = np.array([[x, y, z] for x in (mn[0], mx[0])
                       for y in (mn[1], mx[1]) for z in (mn[2], mx[2])])
        wc = (np.hstack([lc, np.ones((8, 1))]) @ Mnp)[:, :3]   # 타이트 oriented box (world)
        T_l7_W = inv_T(get_prim_world_T(stage, xc, LINK7_PRIM))
        cl = (wc @ T_l7_W[:3, :3].T) + T_l7_W[:3, 3]          # link7 로컬
        ext = (wc.max(0) - wc.min(0)) * 1000.0
        print(f"[NBV] 스캐너 bbox(mm) ≈ {np.round(ext,0).tolist()} (oriented, 타이트) → 자가충돌 실 mesh")
        return cl
    except Exception as e:
        print(f"[NBV][WARN] 스캐너 bbox 계산 실패 ({e}) — 캡슐 근사로 fallback")
        return None


def _turntable_radius(stage, xc):
    """턴테이블 mesh 의 world AABB 수평 범위 → 반경(=지름/2). keepout 원기둥 지름 일치용."""
    try:
        tp = stage.GetPrimAtPath(TURNTABLE_MESH)
        if not tp.IsValid():
            print(f"[NBV][WARN] 턴테이블 prim 없음: {TURNTABLE_MESH}")
            return None
        bbc = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                                [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
        rng = bbc.ComputeWorldBound(tp).ComputeAlignedRange()
        ext = np.array(rng.GetMax(), float) - np.array(rng.GetMin(), float)
        r = float(max(ext[0], ext[1]) / 2.0)
        print(f"[NBV] 턴테이블 반경 ≈ {r*1000:.0f}mm (keepout 원기둥 지름 일치)")
        return r
    except Exception as e:
        print(f"[NBV][WARN] 턴테이블 반경 계산 실패 ({e})")
        return None


def configure_joint_drives(robot):
    v = robot._articulation_view
    v.set_gains(kps=np.full((1, v.num_dof), DRIVE_STIFFNESS, dtype=np.float32),
                kds=np.full((1, v.num_dof), DRIVE_DAMPING, dtype=np.float32))


def build_dof_index(robot):
    names = list(robot.dof_names)
    return [names.index(f"joint{i}") if f"joint{i}" in names else i - 1 for i in range(1, 8)]


def drive_joints(q):
    robot, dof_idx = _ctx["robot"], _ctx["dof_idx"]
    qf = np.asarray(robot.get_joint_positions(), dtype=float).copy()
    for i, di in enumerate(dof_idx):
        qf[di] = q[i]
    robot.apply_action(ArticulationAction(joint_positions=qf))


def _joint_err(target_q):
    q = np.asarray(_ctx["robot"].get_joint_positions(), dtype=float)
    return float(np.max([abs(q[di] - target_q[i]) for i, di in enumerate(_ctx["dof_idx"])]))


def _configure_shape():
    """OBJECT_SHAPE 별 bounding box(OBJ_W/D/H) 전역 설정 (충돌·crop·NBV 타깃에 사용)."""
    global OBJ_W, OBJ_D, OBJ_H
    dims = {
        "box":      (0.060, 0.040, 0.070),
        "cylinder": (0.060, 0.060, 0.070),
        "sphere":   (0.060, 0.060, 0.060),
        "cone":     (0.060, 0.060, 0.080),
        "lshape":   (0.080, 0.050, 0.070),
        "stepped":  (0.070, 0.050, 0.080),
    }.get(OBJECT_SHAPE, (0.060, 0.040, 0.070))
    OBJ_W, OBJ_D, OBJ_H = dims
    print(f"[NBV] 대상물 형상='{OBJECT_SHAPE}'  bbox(cm)="
          f"{OBJ_W*100:.0f}×{OBJ_D*100:.0f}×{OBJ_H*100:.0f}")


def _cube(stage, path, tx, ty, tz, sx, sy, sz, color):
    g = UsdGeom.Cube.Define(stage, path)
    g.GetSizeAttr().Set(1.0)
    g.AddTranslateOp().Set(Gf.Vec3f(tx, ty, tz))
    g.AddScaleOp().Set(Gf.Vec3f(sx, sy, sz))
    g.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    return g


def create_object(stage, center_xy, top_z):
    """OBJECT_SHAPE 형상을 disc 중심에 생성 (root = 물체중심, +z 로 형상 구성)."""
    s = OBJECT_SHAPE
    col = (0.85, 0.35, 0.2)
    root = UsdGeom.Xform.Define(stage, OBJ_PATH)
    root.ClearXformOpOrder()
    root.AddTranslateOp().Set(Gf.Vec3d(float(center_xy[0]), float(center_xy[1]),
                                       float(top_z + OBJ_H / 2.0)))   # root = 물체 중심
    root.AddOrientOp().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    gp = OBJ_PATH + "/geom"
    if s == "box":
        _cube(stage, gp, 0, 0, 0, OBJ_W, OBJ_D, OBJ_H, col)
    elif s == "cylinder":
        g = UsdGeom.Cylinder.Define(stage, gp)
        g.GetRadiusAttr().Set(OBJ_W / 2); g.GetHeightAttr().Set(OBJ_H); g.GetAxisAttr().Set("Z")
        g.CreateDisplayColorAttr([Gf.Vec3f(*col)])
    elif s == "sphere":
        g = UsdGeom.Sphere.Define(stage, gp)
        g.GetRadiusAttr().Set(OBJ_H / 2); g.CreateDisplayColorAttr([Gf.Vec3f(*col)])
    elif s == "cone":
        g = UsdGeom.Cone.Define(stage, gp)
        g.GetRadiusAttr().Set(OBJ_W / 2); g.GetHeightAttr().Set(OBJ_H); g.GetAxisAttr().Set("Z")
        g.CreateDisplayColorAttr([Gf.Vec3f(*col)])
    elif s == "lshape":                      # 수직(왼) + 발(오른-하단) → 오목 step
        _cube(stage, gp + "A", -OBJ_W / 4, 0, 0, OBJ_W / 2, OBJ_D, OBJ_H, col)
        _cube(stage, gp + "B", OBJ_W / 4, 0, -OBJ_H / 4, OBJ_W / 2, OBJ_D, OBJ_H / 2, col)
    elif s == "stepped":                     # 하단 큰 블록 + 상단 작은 블록 → 어깨면 gap
        _cube(stage, gp + "A", 0, 0, -0.2 * OBJ_H, OBJ_W, OBJ_D, 0.6 * OBJ_H, col)
        _cube(stage, gp + "B", 0, 0, 0.3 * OBJ_H, 0.5 * OBJ_W, 0.5 * OBJ_D, 0.4 * OBJ_H, col)
    else:
        _cube(stage, gp, 0, 0, 0, OBJ_W, OBJ_D, OBJ_H, col)
    if s in ADD_MARKER_SHAPES:               # 비대칭 marker (PCA 법선/방위 확인)
        _cube(stage, OBJ_PATH + "/marker", OBJ_W * 0.5, 0.0, OBJ_H * 0.4,
              0.018, 0.018, 0.025, (0.2, 0.45, 0.9))
    print(f"[NBV] 대상물 '{s}' 생성 @ disc 중심 (marker={'O' if s in ADD_MARKER_SHAPES else 'X'})")


def set_object_theta(stage, theta):
    prim = stage.GetPrimAtPath(OBJ_PATH)
    o = prim.GetAttribute("xformOp:orient")
    if not o.IsValid():
        return
    qw, qz = math.cos(theta / 2.0), math.sin(theta / 2.0)
    cur = o.Get()
    o.Set(Gf.Quatf(qw, 0, 0, qz) if isinstance(cur, Gf.Quatf) else Gf.Quatd(qw, 0, 0, qz))


def _Rz(t):
    c, s = math.cos(t), math.sin(t)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=np.float64)


def voxel_ds(pts, v=VOXEL_M):
    pts = np.asarray(pts, dtype=float)
    if len(pts) == 0:
        return pts
    keys = np.floor(pts / v).astype(np.int64)
    _, idx = np.unique(keys, axis=0, return_index=True)
    return pts[idx]


def _save_ply(path, pts):
    pts = np.asarray(pts, float)
    with open(path, "w") as f:
        f.write(f"ply\nformat ascii 1.0\nelement vertex {len(pts)}\n")
        f.write("property float x\nproperty float y\nproperty float z\nend_header\n")
        for p in pts:
            f.write(f"{p[0]:.6f} {p[1]:.6f} {p[2]:.6f}\n")


# ── 입사각 필터 (gap 메커니즘) + 충돌 (인라인) ────────────────────────────────
def _obj_center_world():
    cx, cy = _ctx["center_xy"]
    return np.array([cx, cy, DISC_TOP_Z + OBJ_H / 2.0])


def _estimate_normals_pca(pts, k=12):
    """국소 PCA 법선 추정(numpy). marker 등 임의 형상 대응. 외향(물체중심 기준) 정렬.
    (해석적 box-면 법선은 튀어나온 marker 점을 틀리게 잡아 버리므로 PCA 로.)"""
    N = len(pts)
    if N < 4:
        return None
    k = min(k, N)
    c_obj = _obj_center_world()
    nrm = np.empty_like(pts)
    for i in range(N):
        dif = pts - pts[i]
        idx = np.argpartition(np.einsum('ij,ij->i', dif, dif), k - 1)[:k]
        nb = pts[idx] - pts[idx].mean(0)
        _, v = np.linalg.eigh(nb.T @ nb)
        n = v[:, 0]                                  # 최소 분산 방향 = 법선
        if n @ (pts[i] - c_obj) < 0:
            n = -n                                   # 외향
        nrm[i] = n
    return nrm


def _filter_incidence(pts_w, theta, cam_pos):
    """표면 법선 vs 카메라 시선 입사각 > MAX_INCIDENCE_DEG 면 제거(grazing→스캔실패).
    법선 = **국소 PCA 추정**(marker 포함 임의 형상 대응; theta 인자 미사용)."""
    if len(pts_w) < 4:
        return pts_w
    nrm = _estimate_normals_pca(pts_w)
    if nrm is None:
        return pts_w
    vd = cam_pos[None, :] - pts_w
    vd /= (np.linalg.norm(vd, axis=1, keepdims=True) + 1e-9)
    cos_inc = np.sum(nrm * vd, axis=1)
    return pts_w[cos_inc > math.cos(math.radians(MAX_INCIDENCE_DEG))]


def _cam_pos_world():
    xc = UsdGeom.XformCache(Usd.TimeCode.Default())
    T = get_prim_world_T(_ctx["stage"], xc, CAMERA_PRIM)
    return T[:3, 3] if T is not None else None


def _arm_capsules_base(q):
    """robot_collision.capsules_from_joints 미러 — (origins(8,3) base m, radii(7,)).
    origins = link1..link7(flange) + **스캐너 tip = T_EC 로 산출한 카메라 위치**.
    (직전 버전은 tip 을 링크축으로 근사해 스캐너 위치가 틀려 자가충돌을 놓침 → T_EC 로 교정.)"""
    qf = np.asarray(q, float)
    o = kin.fk_link_origins(qf)                       # (8,3) base m: [base, o1..o7(flange)]
    Tm = kin.fk_T(qf).copy(); Tm[:3, 3] /= 1000.0     # flange (base, m)
    tip = (Tm @ _ctx["T_EC_gt"])[:3, 3]               # 스캐너 끝 = 카메라 위치(base)
    return np.vstack([o[1:8], tip]), LINK_RADII       # (8,3): link1..flange + tip


def _arm_capsules_world(q):
    ob, radii = _arm_capsules_base(q)
    Twb = _ctx["T_W_base"]
    return (ob @ Twb[:3, :3].T) + Twb[:3, 3], radii


def _arm_hits_scene(q):
    """로봇 캡슐(링크1..스캐너)이 **턴테이블 원기둥 또는 대상물 box** 관통(world).
    per-캡슐 반경 사용. robot_collision.pose_collision(world) 의 sim 미러."""
    pw, radii = _arm_capsules_world(q)
    cx, cy = _ctx["center_xy"]
    for k in range(len(pw) - 1):
        rk = float(radii[k])
        p0, p1 = pw[k], pw[k + 1]
        for s in np.linspace(0.0, 1.0, 6):
            p = p0 + s * (p1 - p0)
            r = math.hypot(p[0] - cx, p[1] - cy)
            if (DISC_TOP_Z - BODY_H_M - rk < p[2] < DISC_TOP_Z + 0.005
                    and r < DISC_RADIUS_M + rk):                              # 턴테이블
                return True
            if (DISC_TOP_Z - rk < p[2] < DISC_TOP_Z + OBJ_H + rk
                    and abs(p[0] - cx) < OBJ_W / 2 + rk
                    and abs(p[1] - cy) < OBJ_D / 2 + rk):                     # 대상물
                return True
    return False


def _scanner_corners_world(q):
    """candidate q 의 스캐너 mesh bbox 8 corner(world). link7 FK 로 로컬→world."""
    Tm = kin.fk_T(np.asarray(q, float)).copy(); Tm[:3, 3] /= 1000.0   # link7 base(m)
    T_W_l7 = _ctx["T_W_base"] @ Tm
    cl = _ctx["scanner_corners_l7"]
    return (cl @ T_W_l7[:3, :3].T) + T_W_l7[:3, 3]


def _self_arm_capsule(q):
    """팔-팔 자가충돌(캡슐, 스캐너 캡슐 제외)."""
    o, radii = _arm_capsules_base(q)
    nseg = len(o) - 1                                 # 7 capsules (cap6=scanner) → 제외하고 6
    for i in range(nseg - 1):
        for j in range(i + SELF_GAP, nseg - 1):
            if cgeom.seg_seg_distance(o[i], o[i + 1], o[j], o[j + 1]) \
                    < (float(radii[i]) + float(radii[j])) * SELF_SCALE:
                return True
    return False


def _self_scanner_mesh(q):
    """스캐너 실 mesh(bbox corner) ↔ **베이스 링크**(SCANNER_CHECK_CAPS) 충돌.
    캡슐 중심선만으론 Spider 부피를 못 잡아 통과 오판 → 실 bbox corner 로 검사."""
    cl = _ctx.get("scanner_corners_l7")
    if cl is None:
        return False
    cw = _scanner_corners_world(q)
    ow, radii = _arm_capsules_world(q)
    for k in SCANNER_CHECK_CAPS:
        rk = float(radii[k]) + 0.005
        for c in cw:
            if cgeom.seg_seg_distance(c, c, ow[k], ow[k + 1]) < rk:
                return True
    return False


def _self_collision(q):
    """자가충돌 — 팔-팔(캡슐) OR 스캐너 실 mesh ↔ 베이스 링크."""
    return _self_arm_capsule(q) or _self_scanner_mesh(q)


def _keepout_center():
    if KEEPOUT_CENTER is not None:
        return np.asarray(KEEPOUT_CENTER, float)[:2]
    return np.asarray(_ctx["center_xy"], float)


def _keepout_radius():
    """MATCH_TURNTABLE 면 측정된 턴테이블 반경, 아니면 fallback."""
    if KEEPOUT_MATCH_TURNTABLE and _ctx.get("turntable_radius"):
        return float(_ctx["turntable_radius"])
    return KEEPOUT_RADIUS_M


def _arm_hits_keepout(q):
    """로봇 캡슐(+스캐너 mesh)이 **충돌 금지 원기둥**(turntable 위)에 들어오는가."""
    if not KEEPOUT_ENABLE:
        return False
    cx, cy = _keepout_center()
    R = _keepout_radius()
    zb, zt = DISC_TOP_Z, DISC_TOP_Z + KEEPOUT_HEIGHT_M
    pw, radii = _arm_capsules_world(q)
    for k in range(len(pw) - 1):
        rk = float(radii[k])
        p0, p1 = pw[k], pw[k + 1]
        for s in np.linspace(0.0, 1.0, 6):
            p = p0 + s * (p1 - p0)
            if zb - rk < p[2] < zt + rk and math.hypot(p[0] - cx, p[1] - cy) < R + rk:
                return True
    if _ctx.get("scanner_corners_l7") is not None:
        for c in _scanner_corners_world(q):
            if zb < c[2] < zt and math.hypot(c[0] - cx, c[1] - cy) < R:
                return True
    return False


def _pose_in_collision(q):
    """장면(turntable/대상물) 충돌 OR 자가충돌 OR 충돌금지 원기둥."""
    return _arm_hits_scene(q) or _self_collision(q) or _arm_hits_keepout(q)


def _self_min_clearance(q):
    """자가충돌 최소 여유(m, 음수=관통) 와 가장 가까운 쌍 (진단/로깅용).
    팔-팔(캡슐) + 스캐너 실 mesh ↔ 근위 링크 모두 고려."""
    o, radii = _arm_capsules_base(q)
    nseg = len(o) - 1
    mn, pair = 1e9, None
    for i in range(nseg - 1):
        for j in range(i + SELF_GAP, nseg - 1):
            gap = (cgeom.seg_seg_distance(o[i], o[i + 1], o[j], o[j + 1])
                   - (float(radii[i]) + float(radii[j])) * SELF_SCALE)
            if gap < mn:
                mn, pair = gap, (f"cap{i}", f"cap{j}")
    cl = _ctx.get("scanner_corners_l7")
    if cl is not None:
        cw = _scanner_corners_world(q)
        ow, _ = _arm_capsules_world(q)
        for k in SCANNER_CHECK_CAPS:
            for c in cw:
                gap = cgeom.seg_seg_distance(c, c, ow[k], ow[k + 1]) - (float(radii[k]) + 0.005)
                if gap < mn:
                    mn, pair = gap, ("scanner_mesh", f"cap{k}")
    return mn, pair


def _swept_free(q0, q1, n=NBV_SWEPT_STEPS):
    """q0→q1 관절보간 경로 전체가 충돌-free(장면+자가) 인가 (swept_pose_collision 미러)."""
    q0, q1 = np.asarray(q0, float), np.asarray(q1, float)
    for k in range(n + 1):
        if _pose_in_collision(q0 + (k / n) * (q1 - q0)):
            return False, k / n
    return True, -1.0


# ── 캡처/누적 ─────────────────────────────────────────────────────────────────
def _capture_object_points(theta, apply_incidence=True):
    """카메라 pointcloud → 대상물 점(world) 분리 + (옵션) 입사각 필터."""
    pc = _ctx["camera"].get_pointcloud()
    if pc is None or getattr(pc, "size", 0) == 0:
        return None
    pc = np.asarray(pc, dtype=float).reshape(-1, 3)
    c = _ctx["axis_point"]
    rxy = np.linalg.norm(pc[:, :2] - c[:2], axis=1)
    m = ((pc[:, 2] > DISC_TOP_Z + OBJ_Z_MIN_OFF) & (pc[:, 2] < DISC_TOP_Z + OBJ_Z_MAX_OFF)
         & (rxy < OBJ_XY_CROP))
    obj = voxel_ds(pc[m], VOXEL_M)
    if apply_incidence:
        cam_pos = _cam_pos_world()
        if cam_pos is not None:
            obj = _filter_incidence(obj, theta, cam_pos)
    return obj


def _lookaround_capture(i):
    th = float(_ctx["thetas"][i])
    obj_w = _capture_object_points(th, apply_incidence=True)
    if obj_w is None or len(obj_w) < 10:
        print(f"[NBV]   θ={math.degrees(th):4.0f}°: 대상물 점 부족")
        return
    c = _ctx["axis_point"]
    obj_canon = (obj_w - c) @ _Rz(-th).T + c                  # −θ 역회전 → canonical
    _ctx["accum"].append(obj_canon)
    print(f"[NBV]   θ={math.degrees(th):4.0f}°: {len(obj_w)}점 (총 "
          f"{sum(len(a) for a in _ctx['accum'])})")


def _top_count(pts):
    """**윗 면** 점만 카운트 — z≈z_top 이면서 **xy 내부**(가장자리=옆면 위쪽 제외)."""
    if len(pts) == 0:
        return 0
    z_top = DISC_TOP_Z + OBJ_H
    cx, cy = _ctx["center_xy"]
    r = np.hypot(pts[:, 0] - cx, pts[:, 1] - cy)
    r_in = min(OBJ_W, OBJ_D) / 2.0 - 0.006          # 내부(가장자리 6mm 제외)
    return int(np.count_nonzero((pts[:, 2] > z_top - 0.004) & (r < r_in)))


def _finish_lookaround():
    pts = np.vstack(_ctx["accum"]) if _ctx["accum"] else np.zeros((0, 3))
    c = _ctx["axis_point"]
    side = pts[pts[:, 2] < DISC_TOP_Z + OBJ_H - 0.012]
    az = np.arctan2(side[:, 1] - c[1], side[:, 0] - c[0])
    quad = set((np.round(az / (np.pi / 2.0)).astype(int) % 4).tolist())
    n_top = _top_count(pts)
    _ctx["n_top_p1"] = n_top
    print("\n[NBV] ===== lookaround 결과 =====")
    print(f"[NBV] 총 점={len(pts)}  옆면4 점유={len(quad)}/4  윗면 점={n_top}")
    gap_ok = (len(quad) == 4 and n_top < 25)
    msg = "확인 ✅ (희박)" if gap_ok else "CHECK ⚠ (윗면 과다 — VIEW_EL_DEG↓/MAX_INCIDENCE_DEG↓)"
    print(f"[NBV] 윗면 gap {msg}  (옆면4={len(quad)==4}, top={n_top})")
    _save_ply(os.path.join(OUT_DIR, "lookaround.ply"), pts)
    _draw_pts(pts, (0.0, 1.0, 1.0, 1.0))
    return gap_ok


# ── nbv — NBV 계획 ────────────────────────────────────────────────────────
def _plan_nbv():
    """윗면 gap → NBV 카메라 포즈 후보 생성 → 해석 IK + swept 충돌 통과 + 관절이동 최소 선택.
    프로덕션 경로 = nbv_core.detect_gaps/nbv_pose_from_candidate + _rank_nbv_candidates
    (open3d). 여기선 gap=윗면(엔지니어링됨)이라 윗면중심+상향법선으로 직접 NBV 포즈 산출."""
    O_top = _obj_center_world() + np.array([0, 0, OBJ_H / 2.0])   # 윗면 중심
    Twb, T_EC = _ctx["T_W_base"], _ctx["T_EC_gt"]
    home_q, view_q = _ctx["home_q"], _ctx["view_q"]
    W = np.array([2.0, 2.0, 1.5, 1.0, 1.0, 1.0, 1.0])            # DEFAULT_JOINT_WEIGHTS
    best = None
    n_ikfail = n_collide = 0
    n_scene = n_sscan = n_scap = n_keep = 0
    for eld in NBV_APPROACH_ELS:
        el = math.radians(eld)
        for azd in NBV_APPROACH_AZIS:
            az = math.radians(azd)
            d = np.array([math.cos(el) * math.cos(az), math.cos(el) * math.sin(az), math.sin(el)])
            eye = O_top + NBV_STANDOFF * d
            T_WC = make_T(look_at_camera(eye, O_top, (0.0, 0.0, 1.0)), eye)
            pose6d = mat_to_pose6d_mm(inv_T(Twb) @ (T_WC @ T_EC))
            q, ok = _ctx["provider"].ik(pose6d, seed=view_q)
            if not ok:
                n_ikfail += 1
                continue
            q = np.asarray(q, float)
            free, s_hit = _swept_free(view_q, q)
            if not free:
                n_collide += 1
                # 사유 분류(endpoint 기준; path 중간충돌이면 아래 셋 다 0)
                if _arm_hits_scene(q):
                    n_scene += 1
                elif _arm_hits_keepout(q):
                    n_keep += 1
                elif _self_scanner_mesh(q):
                    n_sscan += 1
                elif _self_arm_capsule(q):
                    n_scap += 1
                continue
            dq = q - view_q
            cost = float(np.sum(W * dq * dq))                    # 관절이동 최소
            if best is None or cost < best[0]:
                best = (cost, q, T_WC, eld, azd)
    n_path = n_collide - n_scene - n_keep - n_sscan - n_scap
    print(f"[NBV] NBV 후보탐색: IK실패={n_ikfail} 충돌reject={n_collide} "
          f"[scene={n_scene} keepout={n_keep} scanner={n_sscan} arm={n_scap} path중간={n_path}]  "
          f"feasible={'있음' if best else '없음'}")
    if best is None:
        print("[NBV][ERROR] NBV: feasible(IK+swept충돌) 포즈 없음")
        return None
    cost, q, T_WC, eld, azd = best
    clr, pair = _self_min_clearance(q)
    print(f"[NBV] NBV 선택: el={eld:.0f}° az={azd:.0f}°  관절이동cost={cost:.4f}")
    print(f"[NBV]   q(deg)={np.round(np.degrees(q),1).tolist()}")
    print(f"[NBV]   자가충돌 최소여유={clr*1000:+.0f}mm (가까운 쌍 {pair[0]}↔{pair[1]}; "
          f"cap0=link1..cap5=link6, scanner_mesh=실 스캐너)")
    return q


# ── 시각화 ─────────────────────────────────────────────────────────────────────
def _draw_keepout():
    """충돌 금지 원기둥을 viewport 에 wireframe 으로 표시 (debug_draw — 카메라 캡처 안 됨)."""
    if _draw is None or not KEEPOUT_ENABLE or _ctx.get("center_xy") is None:
        return
    cx, cy = _keepout_center()
    zb, zt = DISC_TOP_Z, DISC_TOP_Z + KEEPOUT_HEIGHT_M
    R, N = _keepout_radius(), 36
    ang = np.linspace(0.0, 2 * np.pi, N + 1)
    cb = [(cx + R * math.cos(a), cy + R * math.sin(a), zb) for a in ang]
    ct = [(cx + R * math.cos(a), cy + R * math.sin(a), zt) for a in ang]
    starts, ends = [], []
    for i in range(N):
        starts += [cb[i], ct[i]]
        ends += [cb[i + 1], ct[i + 1]]                 # 상/하 원
    for i in range(0, N, 3):
        starts.append(cb[i]); ends.append(ct[i])       # 수직선
    col = (KEEPOUT_COLOR[0], KEEPOUT_COLOR[1], KEEPOUT_COLOR[2], 1.0)
    _draw.draw_lines([tuple(map(float, s)) for s in starts],
                     [tuple(map(float, e)) for e in ends],
                     [col] * len(starts), [2.0] * len(starts))
    print(f"[NBV] 충돌금지 원기둥 표시 — 중심=({cx:.3f},{cy:.3f}) R={R*1000:.0f}mm H={KEEPOUT_HEIGHT_M*1000:.0f}mm")


def _draw_pts(pts, color):
    if _draw is None or len(pts) == 0:
        return
    viz = pts[:: max(1, len(pts) // 3000)]
    pts_l = [tuple(float(v) for v in p) for p in viz]
    _draw.draw_points(pts_l, [color] * len(pts_l), [4] * len(pts_l))


def _finish_nbv():
    pts = np.vstack(_ctx["accum"]) if _ctx["accum"] else np.zeros((0, 3))
    n_top2 = _top_count(pts)
    n_top1 = _ctx.get("n_top_p1", 0)
    print("\n[NBV] ===== nbv 결과 (NBV 보강 후) =====")
    print(f"[NBV] 윗면(상부 gap) 점: lookaround={n_top1} → nbv={n_top2}  (총 점={len(pts)})")
    # 형상별 상부 면적이 달라(cone/sphere=작음) 절대치 대신 증가량 기준.
    filled = (n_top2 >= max(30, n_top1 + 30))
    verdict = "PASS ✅ (윗면 gap 메움)" if filled else "CHECK ⚠ (윗면 보강 부족 — NBV 자세/입사각 확인)"
    print(f"[NBV] 판정: {verdict}")
    _save_ply(os.path.join(OUT_DIR, "nbv_filled.ply"), pts)
    if _draw is not None:
        _draw.clear_points()
    _draw_pts(pts, (1.0, 0.55, 0.0, 1.0))
    print("[NBV] viewport: 주황점=NBV 보강 후 누적 (nbv_filled.ply 저장)")
    print("[NBV] ===== COMPLETE =====\n")


# ── 런타임 상태 + 상태머신 ────────────────────────────────────────────────────
_ctx = {
    "world": None, "stage": None, "robot": None, "camera": None, "dof_idx": None,
    "view_q": None, "home_q": None, "nbv_q": None, "axis_point": None, "center_xy": None,
    "T_W_base": None, "T_EC_gt": None, "provider": None,
    "thetas": None, "theta_idx": 0, "accum": [], "n_top_p1": 0,
    "phase": "SETTLE", "phase_step": 0, "stable_n": 0, "step": 0,
}


def _on_physics_step(step_size):
    ph = _ctx["phase"]
    _ctx["phase_step"] += 1

    if ph == "SETTLE":
        conv = _joint_err(_ctx["view_q"]) < JOINT_SETTLE_TOL
        _ctx["stable_n"] = _ctx["stable_n"] + 1 if conv else 0
        if (_ctx["stable_n"] >= SETTLE_STABLE_N and _ctx["phase_step"] >= WARMUP_STEPS) \
                or _ctx["phase_step"] >= MOVE_TIMEOUT_N:
            _ctx["phase"], _ctx["phase_step"] = "SCAN", 0

    elif ph == "SCAN":
        i = _ctx["theta_idx"]
        if _ctx["phase_step"] == 1:
            set_object_theta(_ctx["stage"], float(_ctx["thetas"][i]))
        elif _ctx["phase_step"] >= WAIT_PER_THETA:
            _lookaround_capture(i)
            if i + 1 >= len(_ctx["thetas"]):
                _finish_lookaround()
                _ctx["phase"], _ctx["phase_step"] = "P2_PLAN", 0
            else:
                _ctx["theta_idx"], _ctx["phase_step"] = i + 1, 0

    elif ph == "P2_PLAN":
        print("\n[NBV] ====== nbv — 부족면 NBV 보강 ======")
        nbv_q = _plan_nbv()
        if nbv_q is None:
            _ctx["phase"] = "DONE"
        else:
            _ctx["nbv_q"] = nbv_q
            set_object_theta(_ctx["stage"], 0.0)                 # canonical 로 복귀
            drive_joints(nbv_q)                                  # NBV 자세로 이동
            _ctx["phase"], _ctx["phase_step"], _ctx["stable_n"] = "P2_SETTLE", 0, 0

    elif ph == "P2_SETTLE":
        conv = _joint_err(_ctx["nbv_q"]) < JOINT_SETTLE_TOL
        _ctx["stable_n"] = _ctx["stable_n"] + 1 if conv else 0
        if (_ctx["stable_n"] >= SETTLE_STABLE_N and _ctx["phase_step"] >= WARMUP_STEPS) \
                or _ctx["phase_step"] >= MOVE_TIMEOUT_N:
            _ctx["phase"], _ctx["phase_step"] = "P2_CAPTURE", 0

    elif ph == "P2_CAPTURE":
        if _ctx["phase_step"] >= WAIT_PER_THETA:
            obj_w = _capture_object_points(0.0, apply_incidence=True)   # θ=0 canonical
            if obj_w is not None and len(obj_w) > 0:
                _ctx["accum"].append(obj_w)
                print(f"[NBV] NBV 캡처: 윗면 포함 {len(obj_w)}점 누적")
            _finish_nbv()
            _ctx["phase"] = "DONE"

    elif ph == "DONE":
        pass
    _ctx["step"] += 1


# ── setup ─────────────────────────────────────────────────────────────────────
async def setup_async():
    if not _HAS_CORE:
        print("[NBV][ERROR] MMS 코어 로드 실패 — 중단.")
        return
    if World.instance() is not None:
        try:
            World.instance().clear_all_callbacks()
        except Exception:
            pass
        World.clear_instance()
    if _draw is not None:
        _draw.clear_points()
        _draw.clear_lines()

    print(f"[NBV] Opening USD: {USD_PATH}")
    open_stage(usd_path=USD_PATH)
    stage = omni.usd.get_context().get_stage()

    j1 = stage.GetPrimAtPath(f"{JOINTS_SCOPE}/joint1")
    if WIDEN_JOINT1_LIMIT_DEG and j1.IsValid():
        j1.GetAttribute("physics:lowerLimit").Set(-float(WIDEN_JOINT1_LIMIT_DEG))
        j1.GetAttribute("physics:upperLimit").Set(float(WIDEN_JOINT1_LIMIT_DEG))

    bake_joint_initial_state(stage, JOINTS_SCOPE, INITIAL_JOINT_POS)
    configure_scanner_camera(stage, CAMERA_PRIM)

    marble = stage.GetPrimAtPath(MARBLE_PRIM_PATH)
    if marble.IsValid():
        marble.SetActive(False)

    xc0 = UsdGeom.XformCache(Usd.TimeCode.Default())
    T_tt = get_prim_world_T(stage, xc0, TURNTABLE_MESH)
    center_xy = np.array([T_tt[0, 3], T_tt[1, 3]], dtype=float)
    turntable_radius = _turntable_radius(stage, xc0)     # keepout 원기둥 지름 = 턴테이블
    _configure_shape()                                   # OBJECT_SHAPE → bbox(OBJ_W/D/H)
    create_object(stage, center_xy, DISC_TOP_Z)

    world = World(physics_dt=PHYSICS_DT, rendering_dt=1.0/60.0, stage_units_in_meters=1.0)
    if world.get_physics_context() is None:
        await world.initialize_simulation_context_async()
    robot = world.scene.add(Robot(prim_path=ROBOT_PRIM, name="xarm7"))
    cam = Camera(prim_path=CAMERA_PRIM, resolution=SCANNER_RESOLUTION)

    await world.reset_async()
    if FIX_DRIVE_GAINS:
        configure_joint_drives(robot)
    cam.initialize()
    cam.add_distance_to_image_plane_to_frame()
    cam.add_pointcloud_to_frame()

    xc = UsdGeom.XformCache(Usd.TimeCode.Default())
    T_W_E = get_prim_world_T(stage, xc, f"{ROBOT_PRIM}/link7")
    T_W_C = get_prim_world_T(stage, xc, CAMERA_PRIM)
    T_W_base = get_prim_world_T(stage, xc, ROBOT_PRIM)
    T_EC_gt = inv_T(T_W_C) @ T_W_E
    scanner_corners_l7 = _compute_scanner_corners_l7(stage, xc)   # 자가충돌용 실 mesh bbox
    O = np.array([center_xy[0], center_xy[1], DISC_TOP_Z + OBJ_H / 2.0])
    provider = RobotIK(kin=kin, robot_ip=ROBOT_IP, use_sdk=USE_XARM_SDK,
                       xarm_sdk_path=_XARM_SDK, logger=print)
    home_q = np.radians(ARTEC_HOME_JOINTS_DEG)

    # lookaround 측면 관측 자세 (낮은 elevation → 윗면 grazing → gap)
    el = math.radians(VIEW_EL_DEG)
    view_q = None
    for azd in VIEW_AZIS_DEG:
        az = math.radians(azd)
        d = np.array([math.cos(el)*math.cos(az), math.cos(el)*math.sin(az), math.sin(el)])
        eye = O + VIEW_STANDOFF * d
        T_WC = make_T(look_at_camera(eye, O, (0.0, 0.0, 1.0)), eye)
        pose6d = mat_to_pose6d_mm(inv_T(T_W_base) @ (T_WC @ T_EC_gt))
        q, ok = provider.ik(pose6d, seed=home_q)
        if ok:
            view_q = np.asarray(q, float)
            print(f"[NBV] lookaround 측면 자세: az={azd:.0f}° el={VIEW_EL_DEG:.0f}° IK ok")
            break
    if view_q is None:
        print("[NBV][WARN] 측면 자세 IK 실패 — home fallback")
        view_q = home_q

    _ctx.update(world=world, stage=stage, robot=robot, camera=cam,
                dof_idx=build_dof_index(robot), view_q=view_q, home_q=home_q,
                center_xy=center_xy, T_W_base=T_W_base, T_EC_gt=T_EC_gt, provider=provider,
                scanner_corners_l7=scanner_corners_l7, turntable_radius=turntable_radius,
                axis_point=np.array([center_xy[0], center_xy[1], DISC_TOP_Z], dtype=float),
                thetas=np.linspace(0.0, 2*np.pi, N_THETA, endpoint=False),
                theta_idx=0, accum=[], n_top_p1=0,
                phase="SETTLE", phase_step=0, stable_n=0, step=0)
    drive_joints(view_q)
    _draw_keepout()                              # 충돌금지 원기둥 wireframe 표시
    print(f"[NBV] disc 중심={np.round(center_xy,4).tolist()} | "
          f"lookaround(측면 GT 누적, 윗면 gap) → nbv(NBV 보강)")
    world.add_physics_callback(PHYSICS_CB_NAME, _on_physics_step)
    await world.play_async()
    print("[NBV] 시작.")


def stop():
    world = _ctx.get("world") or World.instance()
    if world is not None:
        try:
            world.remove_physics_callback(PHYSICS_CB_NAME)
        except Exception:
            pass
        world.stop()
    print("[NBV] Stopped.")


asyncio.ensure_future(setup_async())
