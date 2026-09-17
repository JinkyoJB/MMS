"""
MMS lookaround 시뮬레이션 (5면 스캐닝, ground-truth 누적) - Isaac Sim 5.1.0 (Extension)

목적
----
lookaround(턴테이블 360° 회전으로 윗면+옆면4=5면 취득)의 **누적 로직**을 sim 에서 검증.
sim 엔 Artec SLAM 이 없으므로, 턴테이블 θ(ground-truth)+회전축으로 점군을 누적한다.
(cf. docs/3_lookaround.md §7) real 의 streaming/SLAM 은 이 검증된 누적 위에 그대로 올린다.

흐름
----
0. 마블 비활성, 카메라 광학, 로봇 home(턴테이블 비스듬히 내려다봄) 고정.
1. 턴테이블 위 대상물(box)을 **known θ 로 회전**(키네마틱) — 0..350° step 10°.
2. 매 θ: 카메라 포인트클라우드 → 대상물 점 분리 → 축 둘레 **−θ 역회전** → 대상물 프레임 누적.
3. 누적 점군 = 5면(윗면+옆면4) 재구성. 옆면 방위 커버리지로 검증 + viewport 표시.

핵심: p_obj = Rz(−θ)·(p_world − axis_point) + axis_point   (axis = 턴테이블 회전축).
      θ 가 정확하면(sim GT) 모든 옆면이 정확히 겹쳐 쌓인다 = "SLAM 대신 GT 누적".

★ 로봇은 lookaround 동안 고정(home). 대상물만 회전. real 도 회전 중 robot 고정(불변식).
실행: VSCode Isaac 확장/Script Editor. 결과 → captures_lookaround/ (calib_log.txt tail, scan.ply).
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

# ── MMS 공유 코어 (파일경로 로드) ─────────────────────────────────────────────
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
    make_T, inv_T = geo.make_T, geo.inv_T
    look_at_camera, mat_to_pose6d_mm = geo.look_at_camera, geo.mat_to_pose6d_mm
    RobotIK = _load_mod("mms_ik_provider", "utils/robot/ik_provider.py").RobotIK
    _HAS_CORE = True
except Exception as _e:
    _HAS_CORE = False
    print(f"[LOOKAROUND][WARN] MMS 코어 로드 실패: {_e}")

# ── debug_draw (누적 결과 시각화) ─────────────────────────────────────────────
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
MARBLE_PRIM_PATH = os.environ.get("MMS_SIM_OBJECT_PRIM", "/World/ScanTarget/TestObject")
TURNTABLE_MESH = "/World/frame/turntable_disc"
OBJ_PATH     = "/World/lookaroundObject"

INITIAL_JOINT_POS = {f"joint{i}": 0.0 for i in range(1, 8)}

SPIDER_HFOV_DEG, SPIDER_CLIP = 30.0, (0.15, 0.45)
SPIDER_FOCUS_DISTANCE, SCANNER_RESOLUTION = 0.25, (1280, 960)
ROBOT_IP, USE_XARM_SDK = "192.168.1.210", False  # 자체 해석 IK 사용(SDK IK 미사용)
ARTEC_HOME_JOINTS_DEG = [38.92, -48.70, -65.29, 21.22, 21.46, 72.70, -96.58]

# 측면 관측 자세 — 옆면이 보이도록 낮은 elevation(top-down 금지). 도달 가능한 azimuth 채택.
VIEW_EL_DEG    = 35.0            # 수평에서의 올려본 각(낮을수록 옆면 위주)
VIEW_STANDOFF  = 0.26           # 대상물 중심까지 작동거리 (Spider 0.2~0.3)
VIEW_AZIS_DEG  = [0.0, 30.0, -30.0, 60.0, -60.0, 90.0, -90.0, 180.0]
VOXEL_M        = 0.002          # 누적 전 다운샘플(점수 폭발 방지)

# 턴테이블 disc (USD 검증값)
DISC_TOP_Z   = 0.713
DISC_CENTER  = None             # setup 에서 USD 로 채움 (x,y)

# 대상물 (box, 디스크 중심에 세움 — 윗면+옆면4 = 5면). 비대칭 marker 로 방위 구분.
OBJ_W, OBJ_D, OBJ_H = 0.060, 0.040, 0.070    # x,y,z (m)

# 스캔
N_THETA        = 36             # 360° / 10°
WAIT_PER_THETA = 8              # θ 설정 후 렌더 정착 대기 스텝
# 대상물 점 분리 (디스크 위 + 중심 부근)
OBJ_Z_MIN_OFF  = 0.006          # disc top 위 이만큼부터 (디스크 제외)
OBJ_Z_MAX_OFF  = 0.12
OBJ_XY_CROP    = 0.06

WIDEN_JOINT1_LIMIT_DEG = 175.0
FIX_DRIVE_GAINS, DRIVE_STIFFNESS, DRIVE_DAMPING = True, 2000.0, 200.0
WARMUP_STEPS, SETTLE_STABLE_N, MOVE_TIMEOUT_N, JOINT_SETTLE_TOL = 60, 12, 400, 0.01
PHYSICS_CB_NAME = "mms_lookaround_step"

OUT_DIR   = os.path.join(_BASE_DIR, "captures_lookaround")
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

print(f"[LOOKAROUND] ===== run start =====  (core={_HAS_CORE})")
print(f"[LOOKAROUND] log file: {LOG_PATH}")


# ── USD/Isaac 헬퍼 ────────────────────────────────────────────────────────────
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


# ── 대상물(box) 생성 + 회전 ───────────────────────────────────────────────────
def create_object(stage, center_xy, top_z):
    """디스크 중심에 세운 box(비대칭 marker 포함). 키네마틱(물리 없음) — 직접 회전."""
    root = UsdGeom.Xform.Define(stage, OBJ_PATH)
    root.ClearXformOpOrder()
    root.AddTranslateOp().Set(Gf.Vec3d(float(center_xy[0]), float(center_xy[1]),
                                       float(top_z + OBJ_H / 2.0)))
    root.AddOrientOp().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    box = UsdGeom.Cube.Define(stage, OBJ_PATH + "/box")
    box.GetSizeAttr().Set(1.0)
    box.AddScaleOp().Set(Gf.Vec3f(OBJ_W, OBJ_D, OBJ_H))
    box.CreateDisplayColorAttr([Gf.Vec3f(0.85, 0.35, 0.2)])
    mk = UsdGeom.Cube.Define(stage, OBJ_PATH + "/marker")     # 비대칭 표식
    mk.GetSizeAttr().Set(1.0)
    mk.AddTranslateOp().Set(Gf.Vec3f(OBJ_W * 0.5, 0.0, OBJ_H * 0.5))
    mk.AddScaleOp().Set(Gf.Vec3f(0.018, 0.018, 0.025))
    mk.CreateDisplayColorAttr([Gf.Vec3f(0.2, 0.45, 0.9)])
    print(f"[LOOKAROUND] 대상물 box {OBJ_W*100:.0f}×{OBJ_D*100:.0f}×{OBJ_H*100:.0f}cm @ disc 중심")


def set_object_theta(stage, theta):
    """대상물을 회전축(수직, 중심 통과) 둘레로 θ 회전 (자기 Z 둘레 = 축 둘레)."""
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
    """voxel 다운샘플 (점수 폭발 방지, numpy)."""
    pts = np.asarray(pts, dtype=float)
    if len(pts) == 0:
        return pts
    keys = np.floor(pts / v).astype(np.int64)
    _, idx = np.unique(keys, axis=0, return_index=True)
    return pts[idx]


# ── 런타임 상태 + 상태머신 ────────────────────────────────────────────────────
_ctx = {
    "world": None, "stage": None, "robot": None, "camera": None, "dof_idx": None,
    "view_q": None, "axis_point": None, "thetas": None, "theta_idx": 0, "accum": [],
    "phase": "SETTLE", "phase_step": 0, "stable_n": 0, "step": 0,
}


def _capture_accumulate(i):
    th = float(_ctx["thetas"][i])
    pc = _ctx["camera"].get_pointcloud()
    if pc is None or getattr(pc, "size", 0) == 0:
        print(f"[LOOKAROUND]   θ={math.degrees(th):.0f}°: 렌더 미준비 — skip")
        return
    pc = np.asarray(pc, dtype=float).reshape(-1, 3)
    c = _ctx["axis_point"]
    rxy = np.linalg.norm(pc[:, :2] - c[:2], axis=1)
    m = ((pc[:, 2] > DISC_TOP_Z + OBJ_Z_MIN_OFF) & (pc[:, 2] < DISC_TOP_Z + OBJ_Z_MAX_OFF)
         & (rxy < OBJ_XY_CROP))
    obj_w = voxel_ds(pc[m], VOXEL_M)         # 다운샘플 (점수 폭발 방지)
    if len(obj_w) < 20:
        print(f"[LOOKAROUND]   θ={math.degrees(th):.0f}°: 대상물 점 부족({len(obj_w)})")
        return
    # 축(수직, 중심 통과) 둘레 −θ 역회전 → 대상물 프레임(θ=0) 누적
    obj_canon = (obj_w - c) @ _Rz(-th).T + c
    _ctx["accum"].append(obj_canon)
    print(f"[LOOKAROUND]   θ={math.degrees(th):4.0f}°: 대상물 {len(obj_w)}점 누적 "
          f"(총 {sum(len(a) for a in _ctx['accum'])})")


def _finish():
    if not _ctx["accum"]:
        print("[LOOKAROUND][ERROR] 누적 점 없음")
        return
    pts = np.vstack(_ctx["accum"])
    c = _ctx["axis_point"]
    bb_lo, bb_hi = pts.min(0), pts.max(0)
    size = (bb_hi - bb_lo)
    # 5면 = 옆면4 + 윗면. 옆면은 z < (disc+H) 인 점, 윗면은 z 최상단 근처.
    z_top = DISC_TOP_Z + OBJ_H
    side = pts[pts[:, 2] < z_top - 0.012]
    top = pts[pts[:, 2] > z_top - 0.008]
    # 옆면 4면: 방위 4분면(0/90/180/270° 중심) 각각 점유 확인
    az = np.arctan2(side[:, 1] - c[1], side[:, 0] - c[0])
    quad = set((np.round(az / (np.pi / 2.0)).astype(int) % 4).tolist())
    cov_bins = len(np.unique(((np.arctan2(pts[:, 1]-c[1], pts[:, 0]-c[0]) + np.pi)
                              / (2*np.pi) * 36).astype(int) % 36))
    print("\n[LOOKAROUND] ===== 누적 결과 =====")
    print(f"[LOOKAROUND] 총 점={len(pts)}  θ뷰={len(_ctx['accum'])}")
    print(f"[LOOKAROUND] 크기(cm)≈ {size[0]*100:.1f}×{size[1]*100:.1f}×{size[2]*100:.1f} "
          f"(대상물 {OBJ_W*100:.0f}×{OBJ_D*100:.0f}×{OBJ_H*100:.0f})")
    print(f"[LOOKAROUND] 옆면 4면 점유={len(quad)}/4  윗면 점={len(top)}  방위커버리지={cov_bins}/36")
    # bbox xy 가 대상물 크기와 ~1cm 이내 일치 = 전 옆면 도달
    bbox_ok = (abs(size[0] - OBJ_W) < 0.012 and abs(size[1] - OBJ_D) < 0.012)
    ok = (len(quad) == 4 and len(top) > 30 and bbox_ok)
    verdict = "PASS ✅ (5면 재구성)" if ok else "CHECK ⚠"
    print(f"[LOOKAROUND] 판정: {verdict}  (옆면4={len(quad)==4}, 윗면={len(top)>30}, bbox={bbox_ok})")
    _save_ply(os.path.join(OUT_DIR, "scan.ply"), pts)
    np.savez(os.path.join(OUT_DIR, "lookaround_result.npz"), pts=pts, coverage=cov_bins,
             bbox_lo=bb_lo, bbox_hi=bb_hi)
    _ctx["viz"] = pts[:: max(1, len(pts) // 3000)]
    _draw_accum()
    print("[LOOKAROUND] viewport: 청록점=누적 재구성 (scan.ply 도 저장)")
    print("[LOOKAROUND] ===== COMPLETE =====\n")


def _save_ply(path, pts):
    pts = np.asarray(pts, float)
    with open(path, "w") as f:
        f.write(f"ply\nformat ascii 1.0\nelement vertex {len(pts)}\n")
        f.write("property float x\nproperty float y\nproperty float z\nend_header\n")
        for p in pts:
            f.write(f"{p[0]:.6f} {p[1]:.6f} {p[2]:.6f}\n")


def _draw_accum():
    if _draw is None or _ctx.get("viz") is None:
        return
    _draw.clear_points()
    pts = [tuple(float(v) for v in p) for p in _ctx["viz"]]
    _draw.draw_points(pts, [(0.0, 1.0, 1.0, 1.0)] * len(pts), [4] * len(pts))


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
            set_object_theta(_ctx["stage"], float(_ctx["thetas"][i]))    # θ 설정
        elif _ctx["phase_step"] >= WAIT_PER_THETA:
            _capture_accumulate(i)
            if i + 1 >= len(_ctx["thetas"]):
                _finish()
                _ctx["phase"] = "DONE"
            else:
                _ctx["theta_idx"], _ctx["phase_step"] = i + 1, 0
    elif ph == "DONE":
        if _draw is not None and _ctx["step"] % 60 == 0:
            _draw_accum()
    _ctx["step"] += 1


# ── setup ─────────────────────────────────────────────────────────────────────
async def setup_async():
    if not _HAS_CORE:
        print("[LOOKAROUND][ERROR] MMS 코어 로드 실패 — 중단.")
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

    print(f"[LOOKAROUND] Opening USD: {USD_PATH}")
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

    # disc 중심 → 대상물 생성
    xc0 = UsdGeom.XformCache(Usd.TimeCode.Default())
    T_tt = get_prim_world_T(stage, xc0, TURNTABLE_MESH)
    center_xy = np.array([T_tt[0, 3], T_tt[1, 3]], dtype=float)
    create_object(stage, center_xy, DISC_TOP_Z)

    world = World(physics_dt=1.0/60.0, rendering_dt=1.0/60.0, stage_units_in_meters=1.0)
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

    # ── 측면 관측 자세 선정 (옆면이 보이도록 낮은 elevation) ──
    xc = UsdGeom.XformCache(Usd.TimeCode.Default())
    T_W_E = get_prim_world_T(stage, xc, f"{ROBOT_PRIM}/link7")
    T_W_C = get_prim_world_T(stage, xc, CAMERA_PRIM)
    T_W_base = get_prim_world_T(stage, xc, ROBOT_PRIM)
    T_EC_gt = inv_T(T_W_C) @ T_W_E                          # E→C (hand-eye 와 동일)
    O = np.array([center_xy[0], center_xy[1], DISC_TOP_Z + OBJ_H / 2.0])   # 대상물 중심
    provider = RobotIK(kin=kin, robot_ip=ROBOT_IP, use_sdk=USE_XARM_SDK,
                       xarm_sdk_path=_XARM_SDK, logger=print)
    home_q = np.radians(ARTEC_HOME_JOINTS_DEG)
    el = math.radians(VIEW_EL_DEG)
    view_q = None
    for azd in VIEW_AZIS_DEG:
        az = math.radians(azd)
        d = np.array([math.cos(el) * math.cos(az), math.cos(el) * math.sin(az), math.sin(el)])
        eye = O + VIEW_STANDOFF * d
        T_WC = make_T(look_at_camera(eye, O, (0.0, 0.0, 1.0)), eye)
        pose6d = mat_to_pose6d_mm(inv_T(T_W_base) @ (T_WC @ T_EC_gt))
        q, ok = provider.ik(pose6d, seed=home_q)
        if ok:
            view_q = np.asarray(q, float)
            print(f"[LOOKAROUND] 측면 자세: az={azd:.0f}° el={VIEW_EL_DEG:.0f}° IK ok")
            break
    if view_q is None:
        print("[LOOKAROUND][WARN] 측면 자세 IK 전부 실패 — home fallback (옆면 불충분 가능)")
        view_q = home_q

    _ctx.update(world=world, stage=stage, robot=robot, camera=cam,
                dof_idx=build_dof_index(robot), view_q=view_q,
                axis_point=np.array([center_xy[0], center_xy[1], DISC_TOP_Z], dtype=float),
                thetas=np.linspace(0.0, 2*np.pi, N_THETA, endpoint=False),
                theta_idx=0, accum=[], phase="SETTLE", phase_step=0, stable_n=0, step=0)
    drive_joints(view_q)            # 측면 자세로 이동 후 고정 (lookaround 동안 불변)
    print(f"[LOOKAROUND] disc 중심={np.round(center_xy,4).tolist()} | θ {N_THETA}스텝(10°) GT 누적")
    world.add_physics_callback(PHYSICS_CB_NAME, _on_physics_step)
    await world.play_async()
    print("[LOOKAROUND] 시작: home 고정 → 대상물 θ 회전 → 캡처·−θ 누적 → 5면 재구성.")


def stop():
    world = _ctx.get("world") or World.instance()
    if world is not None:
        try:
            world.remove_physics_callback(PHYSICS_CB_NAME)
        except Exception:
            pass
        world.stop()
    print("[LOOKAROUND] Stopped.")


asyncio.ensure_future(setup_async())
