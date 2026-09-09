"""
MMS Phase 1 Recovery 시뮬레이션 [버전1: 빗나감] - Isaac Sim 5.1.0 (Extension)

(버전2 윗면 미포착은 MMS_ext_phase1_recovery2.py)

목적
----
tracking-lost → **자동 recovery**(safe-back + 자세 재탐색 + 재개) 메커니즘을 sim 에서 검증.
(cf. docs/2_phase1.md §6, docs/main_flow.md §6)

★ trigger 설계 (왜 이렇게?)
   실물 recovery 는 SLAM tracking-lost(overlap/feature/framing 불량)로 발동. sim 엔 SLAM 이
   없으므로 **"캡처된 대상물 점 수"를 trackability 프록시**로 쓴다(점 부족=정합불가=lost).
   trigger 는 **나쁜 로봇 자세**로 건다: 일부러 대상물을 빗나가게 조준 → 점≈0 → lost.
   그러면 recovery 가 대상물 중심을 여러 elevation 으로 조준·preview → 점 최대 자세 선택 →
   재이동 + θ safe-back → 재개. (실물 _adaptive_prescan_position 와 동일 메커니즘.)
   ※ 형상이 어려울 필요 없음 — 자세가 나쁘면 lost 나고 recovery(자세 재선정)가 고친다.

흐름
----
0. box 대상물, 로봇을 **빗나간 자세**(대상물 FOV 밖)로 시작.
1. SCAN: θ 회전하며 캡처. 대상물 점 < 임계 가 N 연속 → tracking lost.
2. RECOVERY: θ safe-back → elevation 후보들(대상물 중심 조준)을 preview → 점 최대 자세 선택 → 재이동.
3. 재개 → 5면 재구성. recovery 발생/성공 보고.

실행: VSCode Isaac 확장/Script Editor. 결과 → captures_phase1_recovery1/.
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

# ── MMS 공유 코어 ─────────────────────────────────────────────────────────────
# ── 경로 해석 (하드코딩 금지) ─────────────────────────────────────────────────
#   이 스크립트는 Isaac 트리에 복사/링크해서 돌리므로 __file__ 이 리포 밖일 수 있다.
#   따라서 리포 위치를 아래 순서로 찾는다.  ※ 자세히는 sim_harness/README.md
def _find_mms_repo():
    import os as _os
    cands = [_os.environ.get("MMS_ROOT"),
             _os.path.join(_os.path.dirname(_os.path.abspath(__file__)), ".."),  # 리포 안에서 실행
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


try:
    kin = _load_mod("mms_xarm7_kinematics", "utils/robot/xarm7_kinematics.py")
    geo = _load_mod("mms_handeye_geometry", "mms_artec/utils/calibration/handeye_geometry.py")
    make_T, inv_T = geo.make_T, geo.inv_T
    look_at_camera, mat_to_pose6d_mm = geo.look_at_camera, geo.mat_to_pose6d_mm
    RobotIK = _load_mod("mms_ik_provider", "utils/robot/ik_provider.py").RobotIK
    # ★ 충돌검사는 **실물과 동일한** utils/collision/robot_collision.py 를 그대로 사용.
    #   그 모듈은 내부적으로 from utils.collision.geometry / utils.robot.xarm7_kinematics 를
    #   import 하므로, Isaac 'utils' 충돌 회피 위해 canonical 이름으로 dep 를 주입 후 로드.
    import types as _types
    _colgeo = _load_mod("mms_col_geometry", "utils/collision/geometry.py")
    for _pkg in ("utils", "utils.collision", "utils.robot"):   # 없을 때만(기존 것 보존)
        if _pkg not in sys.modules:
            _m = _types.ModuleType(_pkg); _m.__path__ = []; sys.modules[_pkg] = _m
    sys.modules["utils.collision.geometry"] = _colgeo
    sys.modules["utils.robot.xarm7_kinematics"] = kin
    _rc = _load_mod("mms_robot_collision", "utils/collision/robot_collision.py")
    CollisionWorld = _rc.CollisionWorld
    pose_collision_lib = _rc.pose_collision           # q → (충돌, 사유) [real/sim 공용]
    _HAS_CORE = True
except Exception as _e:
    _HAS_CORE = False
    print(f"[RECOV][WARN] MMS 코어 로드 실패: {_e}")

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
OBJ_PATH     = "/World/Phase1Object"

INITIAL_JOINT_POS = {f"joint{i}": 0.0 for i in range(1, 8)}
SPIDER_HFOV_DEG, SPIDER_CLIP = 30.0, (0.15, 0.45)
SPIDER_FOCUS_DISTANCE, SCANNER_RESOLUTION = 0.25, (1280, 960)
ROBOT_IP, USE_XARM_SDK = "192.168.1.210", False  # 자체 해석 IK 사용(SDK IK 미사용)
ARTEC_HOME_JOINTS_DEG = [38.92, -48.70, -65.29, 21.22, 21.46, 72.70, -96.58]

DISC_TOP_Z = 0.713
OBJ_W, OBJ_D, OBJ_H = 0.060, 0.040, 0.080

# 관측 자세 (대상물 중심 조준)
VIEW_STANDOFF = 0.26
VIEW_AZIS_DEG = [0.0, 30.0, -30.0, 60.0, -60.0, 90.0, -90.0, 180.0]
GOOD_EL_DEG   = 35.0
# ★ 나쁜 초기 자세 — 시선에 **수직(측면)** 으로 빗나가게 조준 → 대상물 FOV 밖 → lost.
#   (광축 방향으로 빗나가면 깊이만 바뀌어 시야에 남으므로 반드시 측면 오프셋)
BAD_AIM_LATERAL = 0.16          # 시선 수직 오프셋(m). FOV 반각(15°@0.26m≈70mm)보다 커야 함
# recovery elevation 후보 — 대상물 중심 조준, 대상물 점 최대 선택
RECOVERY_EL_DEG = [25.0, 35.0, 45.0]

# 스캔 + watchdog (대상물 점 수 = trackability 프록시)
N_THETA, WAIT_PER_THETA, VOXEL_M = 36, 8, 0.002
TRACK_MIN_PTS = 400            # 대상물 점이 이보다 적으면 lost 후보 frame
CONSEC_LOST   = 3              # 연속 N frame → tracking lost
MAX_RECOVERY  = 3
OBJ_Z_MIN_OFF, OBJ_Z_MAX_OFF, OBJ_XY_CROP = 0.006, 0.14, 0.06
DISC_RADIUS = 0.075

# ── 충돌 검사 (로봇 캡슐 vs 장애물 + self-collision) — 캡슐/판정은 라이브러리 ──
COL_MARGIN    = 0.005           # 여유(m)
TABLE_FLOOR_Z = 0.685           # 이 z 아래로 로봇 링크 금지(테이블 위 유지)
TT_BODY_H, TT_MARGIN = 0.20, 0.01
FRAME_PRIM    = "/World/Frame/frame_structure"
INCLUDE_FRAME = True            # 프레임 구조물 장애물 포함 (과도하게 reject 되면 False)
INCLUDE_SELF  = True            # self-collision (팔 링크끼리, 스캐너 제외, 3칸+)
SELF_SCALE    = 0.7             # self-collision 반경 축소(접힌 팔 오탐 방지). 검증: home 깨끗

WIDEN_JOINT1_LIMIT_DEG = 175.0
# 드라이브 게인 — 과소감쇠/극단 자세에서 부들부들(진동) 방지. stiffness↑(중력 처짐↓)·
# damping↑(진동 억제). (USD 원본 damping≈0 + stiffness=10000 이 과소감쇠 원인)
FIX_DRIVE_GAINS, DRIVE_STIFFNESS, DRIVE_DAMPING = True, 4000.0, 700.0
WARMUP_STEPS, SETTLE_STABLE_N, MOVE_TIMEOUT_N, JOINT_SETTLE_TOL = 50, 12, 400, 0.01
PROBE_WAIT = 30                # recovery preview 정착 대기
PHYSICS_CB_NAME = "mms_recov1_step"

_BASE_DIR = os.environ.get("MMS_HARNESS_OUT",
    os.path.expanduser("~/isaacsim/standalone_examples/play/MMS"))
OUT_DIR = os.path.join(_BASE_DIR, "captures_phase1_recovery1")
os.makedirs(OUT_DIR, exist_ok=True)
LOG_PATH = os.path.join(OUT_DIR, "calib_log.txt")
try:
    open(LOG_PATH, "w").close()
except Exception:
    pass
_builtin_print = print
def print(*a, **k):                      # noqa: A001
    _builtin_print(*a, **k)
    try:
        with open(LOG_PATH, "a", encoding="utf-8") as f:
            f.write(" ".join(str(x) for x in a) + "\n")
    except Exception:
        pass

print(f"[RECOV] ===== run start =====  (core={_HAS_CORE})")
print(f"[RECOV] log file: {LOG_PATH}")


# ── USD/Isaac 헬퍼 ────────────────────────────────────────────────────────────
def bake_joint_initial_state(stage, scope, jp_pos):
    for jn, ang in jp_pos.items():
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


# ── 대상물 ────────────────────────────────────────────────────────────────────
def create_object(stage, center_xy, top_z):
    root = UsdGeom.Xform.Define(stage, OBJ_PATH)
    root.ClearXformOpOrder()
    root.AddTranslateOp().Set(Gf.Vec3d(float(center_xy[0]), float(center_xy[1]),
                                       float(top_z + OBJ_H / 2.0)))
    root.AddOrientOp().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    box = UsdGeom.Cube.Define(stage, OBJ_PATH + "/box")
    box.GetSizeAttr().Set(1.0)
    box.AddScaleOp().Set(Gf.Vec3f(OBJ_W, OBJ_D, OBJ_H))
    box.CreateDisplayColorAttr([Gf.Vec3f(0.85, 0.35, 0.2)])
    mk = UsdGeom.Cube.Define(stage, OBJ_PATH + "/marker")
    mk.GetSizeAttr().Set(1.0)
    mk.AddTranslateOp().Set(Gf.Vec3f(OBJ_W * 0.5, 0.0, OBJ_H * 0.5))
    mk.AddScaleOp().Set(Gf.Vec3f(0.018, 0.018, 0.025))
    mk.CreateDisplayColorAttr([Gf.Vec3f(0.2, 0.45, 0.9)])


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
    _, idx = np.unique(np.floor(pts / v).astype(np.int64), axis=0, return_index=True)
    return pts[idx]


def aim_pose_q(target, el_deg, az_deg, standoff, seed):
    """target 을 (el,az,standoff)에서 조준하는 EE 자세 IK + **충돌 검사** → (q, ok)."""
    el, az = math.radians(el_deg), math.radians(az_deg)
    d = np.array([math.cos(el) * math.cos(az), math.cos(el) * math.sin(az), math.sin(el)])
    eye = np.asarray(target, float) + standoff * d
    T_WC = make_T(look_at_camera(eye, target, (0.0, 0.0, 1.0)), eye)
    pose6d = mat_to_pose6d_mm(inv_T(_ctx["T_W_base"]) @ (T_WC @ _ctx["T_EC_gt"]))
    q, ok = _ctx["provider"].ik(pose6d, seed=seed)
    if not ok:
        return None, False
    reason = pose_collides(q)               # ★ 충돌하는 자세는 reject (IK 실패처럼 취급)
    if reason:
        print(f"[RECOV]   [충돌회피] el={el_deg:.0f}° az={az_deg:.0f}° reject ({reason})")
        return None, False
    return np.asarray(q, float), True


# ── 충돌 검사 — 실물과 동일한 utils/collision/robot_collision.py 사용 ──────────
#   robot 캡슐: capsules_from_joints(q, T_EC) [base 프레임, 해석 FK] (라이브러리 내부)
#   판정     : pose_collision(world, q, ...) [월드 + self-collision] (라이브러리)
#   sim 전용 부분 = 월드 빌드(장애물 geometry 가 USD 에서 옴). 좌표는 robot **base** 프레임.
def pose_collides(q):
    """충돌하면 사유 문자열, 아니면 None. (실물 공용 pose_collision 호출)."""
    W = _ctx.get("col_world")
    if W is None:
        return None
    col, reason = pose_collision_lib(
        W, q, T_EC=_ctx["T_EC_gt"], margin=COL_MARGIN,
        self_scale=(SELF_SCALE if INCLUDE_SELF else 0.0))
    return reason if col else None


def _world_aabb(xc, prim):
    try:
        rng = UsdGeom.Imageable(prim).ComputeWorldBound(
            Usd.TimeCode.Default(), "default").ComputeAlignedRange()
        mn = np.array(rng.GetMin(), float); mx = np.array(rng.GetMax(), float)
    except Exception:
        return None
    if not (np.all(np.isfinite(mn)) and np.all(np.isfinite(mx))) or np.any(mx - mn <= 1e-4):
        return None
    return mn, mx


def build_collision_world(stage, xc, disc_center_w, T_W_base):
    """충돌 월드(**robot base 프레임**). 턴테이블(calib) + 테이블평면 + (옵션)프레임 구조물.
    실물도 동일 CollisionWorld 를 calibration·셀 측정치로 만든다(좌표만 base)."""
    T_Bw = inv_T(T_W_base)
    Rbw, tbw = T_Bw[:3, :3], T_Bw[:3, 3]
    def w2b(p):  return Rbw @ np.asarray(p, float) + tbw          # world point → base
    def w2bv(v): return Rbw @ np.asarray(v, float)               # world vec → base
    W = CollisionWorld.from_turntable(w2b(disc_center_w), w2bv([0., 0., 1.]),
                                      DISC_RADIUS, body_height=TT_BODY_H, margin=TT_MARGIN)
    W.add_halfspace("table", w2b([0., 0., TABLE_FLOOR_Z]), w2bv([0., 0., 1.]))
    if INCLUDE_FRAME:
        fr = stage.GetPrimAtPath(FRAME_PRIM)
        if fr.IsValid():
            for ch in fr.GetChildren():
                bb = _world_aabb(xc, ch)
                if bb is None:
                    continue
                lo, hi = bb
                cb = np.array([w2b([x, y, z]) for x in (lo[0], hi[0])
                               for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])
                W.add_box(f"frame:{ch.GetName()[:12]}", cb.min(0), cb.max(0))
    return W


# ── 대상물 점 캡처 (watchdog/누적 공용) ───────────────────────────────────────
def _object_points():
    pc = _ctx["camera"].get_pointcloud()
    if pc is None or getattr(pc, "size", 0) == 0:
        return np.zeros((0, 3))
    pc = np.asarray(pc, dtype=float).reshape(-1, 3)
    c = _ctx["axis_point"]
    rxy = np.linalg.norm(pc[:, :2] - c[:2], axis=1)
    m = ((pc[:, 2] > DISC_TOP_Z + OBJ_Z_MIN_OFF) & (pc[:, 2] < DISC_TOP_Z + OBJ_Z_MAX_OFF)
         & (rxy < OBJ_XY_CROP))
    return pc[m]


# ── 런타임 상태 + 상태머신 ────────────────────────────────────────────────────
_ctx = {
    "world": None, "stage": None, "robot": None, "camera": None, "dof_idx": None,
    "provider": None, "T_W_base": None, "T_EC_gt": None, "obj_center": None, "col_world": None,
    "axis_point": None, "scan_q": None, "thetas": None, "theta_idx": 0,
    "last_good_idx": 0, "consec_lost": 0, "recov_count": 0, "accum": [],
    # recovery 서브상태
    "rec_cands": None, "rec_idx": 0, "rec_scores": None, "rec_seed": None,
    "phase": "SETTLE", "phase_step": 0, "stable_n": 0, "step": 0, "viz": None,
}


def _accumulate(i, obj_w):
    th = float(_ctx["thetas"][i])
    obj = voxel_ds(obj_w, VOXEL_M)
    _ctx["accum"].append((obj - _ctx["axis_point"]) @ _Rz(-th).T + _ctx["axis_point"])
    _ctx["last_good_idx"] = i


def _enter_recovery():
    if _ctx["recov_count"] >= MAX_RECOVERY:
        print(f"[RECOV] recovery {MAX_RECOVERY}회 초과 — 포기(실물은 user-prompt) → 종료")
        _finish()
        _ctx["phase"] = "DONE"
        return
    _ctx["recov_count"] += 1
    sb = int(_ctx["last_good_idx"])
    print(f"\n[RECOV] ⚠ 대상물 빗나감 @ θ={math.degrees(_ctx['thetas'][_ctx['theta_idx']]):.0f}° "
          f"(점 부족 {CONSEC_LOST}연속) → recovery #{_ctx['recov_count']} (자세 재탐색)")
    print(f"[RECOV]   safe-back: θ idx {_ctx['theta_idx']} → {sb} "
          f"(θ={math.degrees(_ctx['thetas'][sb]):.0f}°)")
    # elevation 후보 자세 IK (대상물 중심 조준)
    cands = []
    for el in RECOVERY_EL_DEG:
        q, ok = aim_pose_q(_ctx["obj_center"], el, _ctx["good_az"], VIEW_STANDOFF, _ctx["scan_q"])
        if ok:
            cands.append((el, np.asarray(q, float)))
    if not cands:
        print("[RECOV][ERROR] 후보 IK 전부 실패 → 종료")
        _finish(); _ctx["phase"] = "DONE"; return
    _ctx["rec_cands"], _ctx["rec_idx"], _ctx["rec_scores"] = cands, 0, []
    _ctx["theta_idx"] = sb
    set_object_theta(_ctx["stage"], float(_ctx["thetas"][sb]))   # safe-back θ
    drive_joints(cands[0][1])
    _ctx["phase"], _ctx["phase_step"], _ctx["stable_n"] = "REC_MOVE", 0, 0


def _finish():
    if not _ctx["accum"]:
        print("[RECOV][ERROR] 누적 점 없음 (recovery 실패)")
        return
    pts = np.vstack(_ctx["accum"])
    c = _ctx["axis_point"]
    size = pts.max(0) - pts.min(0)
    z_top = DISC_TOP_Z + OBJ_H
    side = pts[pts[:, 2] < z_top - 0.012]
    top = pts[pts[:, 2] > z_top - 0.008]
    az = np.arctan2(side[:, 1] - c[1], side[:, 0] - c[0])
    quad = set((np.round(az / (np.pi / 2.0)).astype(int) % 4).tolist())
    print("\n[RECOV] ===== 누적 결과 =====")
    print(f"[RECOV] recovery 발생 = {_ctx['recov_count']}회")
    print(f"[RECOV] 총 점={len(pts)}  크기(cm)≈ {size[0]*100:.1f}×{size[1]*100:.1f}×{size[2]*100:.1f} "
          f"(대상물 {OBJ_W*100:.0f}×{OBJ_D*100:.0f}×{OBJ_H*100:.0f})")
    print(f"[RECOV] 옆면 4면 점유={len(quad)}/4  윗면 점={len(top)}")
    bbox_ok = abs(size[0] - OBJ_W) < 0.012 and abs(size[1] - OBJ_D) < 0.012
    ok = (len(quad) == 4 and len(top) > 30 and bbox_ok and _ctx["recov_count"] >= 1)
    print(f"[RECOV] 판정: {'PASS ✅ (recovery 후 5면 재구성)' if ok else 'CHECK ⚠'}")
    _save_ply(os.path.join(OUT_DIR, "scan.ply"), pts)
    np.savez(os.path.join(OUT_DIR, "recovery_result.npz"), pts=pts,
             recov_count=_ctx["recov_count"], n_quad=len(quad))
    _ctx["viz"] = pts[:: max(1, len(pts) // 3000)]
    _draw_accum()
    print("[RECOV] viewport: 청록점=recovery 후 누적 재구성")
    print("[RECOV] ===== COMPLETE =====\n")


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

    if ph == "SETTLE":          # 현재 scan_q 로 이동·정착 → SCAN
        conv = _joint_err(_ctx["scan_q"]) < JOINT_SETTLE_TOL
        _ctx["stable_n"] = _ctx["stable_n"] + 1 if conv else 0
        if (_ctx["stable_n"] >= SETTLE_STABLE_N and _ctx["phase_step"] >= WARMUP_STEPS) \
                or _ctx["phase_step"] >= MOVE_TIMEOUT_N:
            _ctx["phase"], _ctx["phase_step"] = "SCAN", 0

    elif ph == "SCAN":
        i = _ctx["theta_idx"]
        if _ctx["phase_step"] == 1:
            set_object_theta(_ctx["stage"], float(_ctx["thetas"][i]))
        elif _ctx["phase_step"] >= WAIT_PER_THETA:
            obj = _object_points()
            n = len(obj)
            if n < TRACK_MIN_PTS:                       # 대상물 빗나감(점 부족)
                _ctx["consec_lost"] += 1
                print(f"[RECOV]   θ={math.degrees(_ctx['thetas'][i]):4.0f}°: 대상물 점 {n} "
                      f"< {TRACK_MIN_PTS} — 빗나감 {_ctx['consec_lost']}/{CONSEC_LOST}")
                if _ctx["consec_lost"] >= CONSEC_LOST:
                    _ctx["consec_lost"] = 0
                    _enter_recovery()
                    return
            else:
                _ctx["consec_lost"] = 0
                _accumulate(i, obj)
                print(f"[RECOV]   θ={math.degrees(_ctx['thetas'][i]):4.0f}°: 대상물 {n}점 누적")
            # 다음 θ
            if i + 1 >= len(_ctx["thetas"]):
                _finish(); _ctx["phase"] = "DONE"
            else:
                _ctx["theta_idx"], _ctx["phase_step"] = i + 1, 0

    elif ph == "REC_MOVE":      # 후보 자세로 이동·정착 → preview
        q = _ctx["rec_cands"][_ctx["rec_idx"]][1]
        conv = _joint_err(q) < JOINT_SETTLE_TOL
        _ctx["stable_n"] = _ctx["stable_n"] + 1 if conv else 0
        if (_ctx["stable_n"] >= SETTLE_STABLE_N and _ctx["phase_step"] >= PROBE_WAIT) \
                or _ctx["phase_step"] >= MOVE_TIMEOUT_N:
            n = len(_object_points())                    # 대상물 점 = 점수
            el = _ctx["rec_cands"][_ctx["rec_idx"]][0]
            _ctx["rec_scores"].append(n)
            print(f"[RECOV]   probe el={el:.0f}°: 대상물 {n}점")
            if _ctx["rec_idx"] + 1 < len(_ctx["rec_cands"]):
                _ctx["rec_idx"] += 1
                drive_joints(_ctx["rec_cands"][_ctx["rec_idx"]][1])
                _ctx["phase_step"], _ctx["stable_n"] = 0, 0
            else:               # 후보 평가 끝 → 대상물 점 최대 자세 선택
                best = int(np.argmax(_ctx["rec_scores"]))
                el, q = _ctx["rec_cands"][best]
                if _ctx["rec_scores"][best] < TRACK_MIN_PTS:
                    print("[RECOV][ERROR] 모든 후보 점 부족 → 종료")
                    _finish(); _ctx["phase"] = "DONE"; return
                print(f"[RECOV]   → best el={el:.0f}° (대상물 {_ctx['rec_scores'][best]}점) 재이동, θ 재개")
                _ctx["scan_q"] = q
                drive_joints(q)
                _ctx["phase"], _ctx["phase_step"], _ctx["stable_n"] = "SETTLE", 0, 0

    elif ph == "DONE":
        if _draw is not None and _ctx["step"] % 60 == 0:
            _draw_accum()
    _ctx["step"] += 1


# ── setup ─────────────────────────────────────────────────────────────────────
async def setup_async():
    if not _HAS_CORE:
        print("[RECOV][ERROR] MMS 코어 로드 실패 — 중단.")
        return
    if World.instance() is not None:
        try:
            World.instance().clear_all_callbacks()
        except Exception:
            pass
        World.clear_instance()
    if _draw is not None:
        _draw.clear_points(); _draw.clear_lines()

    print(f"[RECOV] Opening USD: {USD_PATH}")
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

    xc = UsdGeom.XformCache(Usd.TimeCode.Default())
    T_W_E = get_prim_world_T(stage, xc, f"{ROBOT_PRIM}/link7")
    T_W_C = get_prim_world_T(stage, xc, CAMERA_PRIM)
    obj_center = np.array([center_xy[0], center_xy[1], DISC_TOP_Z + OBJ_H / 2.0])
    provider = RobotIK(kin=kin, robot_ip=ROBOT_IP, use_sdk=USE_XARM_SDK,
                       xarm_sdk_path=_XARM_SDK, logger=print)
    _ctx.update(stage=stage, T_W_base=get_prim_world_T(stage, xc, ROBOT_PRIM),
                T_EC_gt=inv_T(T_W_C) @ T_W_E, provider=provider, obj_center=obj_center,
                robot=robot, dof_idx=build_dof_index(robot))
    # 충돌 월드 (IK 후보 자세를 실행 전 거름) — base 프레임, 실물 CollisionWorld 그대로
    _ctx["col_world"] = build_collision_world(
        stage, xc, np.array([center_xy[0], center_xy[1], DISC_TOP_Z]), _ctx["T_W_base"])
    W = _ctx["col_world"]
    print(f"[RECOV] 충돌 월드(base): 장애물 캡슐 {len(W.obstacles)}개 + 평면 {len(W.halfspaces)}개 "
          f"(self-collision {'ON' if INCLUDE_SELF else 'OFF'})")

    home_q = np.radians(ARTEC_HOME_JOINTS_DEG)
    # 도달 가능한 azimuth 1개 확정 (대상물 중심 조준, good elevation)
    good_az, good_q = None, None
    for azd in VIEW_AZIS_DEG:
        q, ok = aim_pose_q(obj_center, GOOD_EL_DEG, azd, VIEW_STANDOFF, home_q)
        if ok:
            good_az, good_q = azd, np.asarray(q, float)
            break
    if good_az is None:
        print("[RECOV][ERROR] 도달 가능한 관측 azimuth 없음 — 중단"); return
    # ★ 나쁜 초기 자세 — 시선에 수직(측면)으로 빗나간 점 조준 → 대상물 FOV 밖 → lost
    gar = math.radians(good_az)
    perp = np.array([-math.sin(gar), math.cos(gar), 0.0])
    bad_q, ok = aim_pose_q(obj_center + BAD_AIM_LATERAL * perp,
                           GOOD_EL_DEG, good_az, VIEW_STANDOFF, good_q)
    if not ok:
        print("[RECOV][WARN] 나쁜 자세 IK 실패 — good 자세로 시작(lost 안 날 수 있음)")
        bad_q = good_q
    else:
        print(f"[RECOV] 나쁜 초기 자세: 측면 {BAD_AIM_LATERAL*100:.0f}cm 빗나감 (대상물 FOV 밖 예상)")

    _ctx.update(camera=cam, world=world, good_az=good_az, scan_q=bad_q,
                axis_point=np.array([center_xy[0], center_xy[1], DISC_TOP_Z], dtype=float),
                thetas=np.linspace(0.0, 2*np.pi, N_THETA, endpoint=False),
                theta_idx=0, last_good_idx=0, consec_lost=0, recov_count=0, accum=[],
                phase="SETTLE", phase_step=0, stable_n=0, step=0)
    drive_joints(bad_q)
    print(f"[RECOV] 관측 azimuth={good_az:.0f}° | 시작=측면 빗나감(대상물 FOV 밖) → recovery 유발 예정")
    world.add_physics_callback(PHYSICS_CB_NAME, _on_physics_step)
    await world.play_async()
    print("[RECOV] 시작: 빗나감(점≈0) → recovery(자세 재탐색) → 대상물 재조준 → 5면.")


def stop():
    world = _ctx.get("world") or World.instance()
    if world is not None:
        try:
            world.remove_physics_callback(PHYSICS_CB_NAME)
        except Exception:
            pass
        world.stop()
    print("[RECOV] Stopped.")


asyncio.ensure_future(setup_async())
