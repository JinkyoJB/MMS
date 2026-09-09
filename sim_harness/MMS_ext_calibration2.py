"""
MMS Turntable Calibration 시뮬레이션 (rim 방법) - Isaac Sim 5.1.0 (Extension/Script Editor)

목적
----
실기 턴테이블 rim 캘리브(disc rim 점 → 3D 원피팅 → T_B_F0)를 ground-truth 를 아는
가상환경에서 검증. (cf. docs/1_calibration.md Part 2, docs/main_flow.md §1.1)

★ 왜 멀티뷰인가
   Spider FOV(작동거리 0.2~0.3m)는 disc(반경 75mm)보다 작아 **한 번에 rim 전체가 안 보임**.
   disc 중심을 내려다보면 진짜 가장자리가 시야 밖 → 반경 과소. 그래서 카메라를 **rim(가장자리)을
   겨냥**해 여러 방위에서 **진짜 rim arc** 를 모아 합쳐 원피팅한다(검증: 멀티뷰 radius_err 0.02mm).

흐름
----
0. 마블 비활성, 카메라 광학(클립 확대), GT(축=+Z, 중심=disc center) 읽기.
1. rim 위 N개 방위를 겨냥하는 카메라 자세 생성 → IK(RobotIK) 로 이동.
2. 각 자세: 포인트클라우드 → disc 표면(auto z) → 그 방위 섹터의 **외곽=rim arc** 추출·누적.
3. 누적 rim 점 → 실물 공유 코어 `fit_circle_3d` → `build_T_B_F0`. GT 비교.

★ 검출/피팅 수학은 `utils/calibration/turntable_frame.py`(실물 동일)를 파일경로 로드해 사용.
★ base = world (로봇 base 고정) → 포인트클라우드가 곧 base 점, T_CB=I.

실행: VSCode Isaac 확장/Script Editor. 결과/로그 → captures_turntable/ (calib_log.txt tail).
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
from pxr import Usd, UsdGeom, Sdf, Gf

# ── MMS 공유 코어 (파일경로 로드 — Isaac 'utils' 패키지명 충돌 회피) ──────────────
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
    _tf = _load_mod("mms_turntable_frame", "utils/calibration/turntable_frame.py")
    fit_circle_3d, fit_plane, build_T_B_F0 = _tf.fit_circle_3d, _tf.fit_plane, _tf.build_T_B_F0
    RobotIK = _load_mod("mms_ik_provider", "utils/robot/ik_provider.py").RobotIK
    _HAS_CORE = True
except Exception as _e:
    _HAS_CORE = False
    print(f"[TTCAL][WARN] MMS 코어 로드 실패: {_e}")


# ── 상수 ───────────────────────────────────────────────────────────────────────
USD_PATH     = _asset("frame_xarm7_spider_turntable/v2.usd")
ROBOT_PRIM   = "/World/xarm7"
JOINTS_SCOPE = "/World/xarm7/joints"
CAMERA_PRIM  = "/World/xarm7/link7/Artec_Space_Spider_mm/Camera"
EE_LINK_PATH = f"{ROBOT_PRIM}/link7"
TURNTABLE_MESH = "/World/ScanTarget/turntable_demo/turntable/turntable"
MARBLE_PRIM_PATH = "/World/ScanTarget/Solid_Marble"

INITIAL_JOINT_POS = {f"joint{i}": 0.0 for i in range(1, 8)}

# Artec Spider 광학. ★ 클립을 (0.15,0.40)로 확대 — rim 겨냥 시 disc 가 잘리지 않게.
SPIDER_HFOV_DEG, SPIDER_CLIP = 30.0, (0.15, 0.40)
SPIDER_FOCUS_DISTANCE, SCANNER_RESOLUTION = 0.25, (1280, 960)

ROBOT_IP, USE_XARM_SDK = "192.168.1.210", False  # 자체 해석 IK 사용(SDK IK 미사용)
ARTEC_HOME_JOINTS_DEG = [38.92, -48.70, -65.29, 21.22, 21.46, 72.70, -96.58]

# disc 기하 (USD 검증값: 원형 반경 0.075, top z 0.713, 중심 (0.325,-0.022), 축=+Z)
DISC_TOP_Z       = 0.713
DISC_RADIUS      = 0.075          # GT 검증용 참값
DISC_RADIUS_HINT = 0.075          # rim 겨냥용(대략값이면 됨 — 진짜 edge 가 시야에 들어오게)
# 멀티뷰 rim 겨냥
N_VIEWS        = 8
VIEW_UP        = 0.22             # aim 위 카메라 높이 (m)
VIEW_OUT       = 0.06             # aim 바깥쪽 오프셋 (m) — edge 프로파일 보이게 기울임
VIEW_UP_HINT   = (1.0, 0.0, 0.0)
# rim arc 추출
NEAR_Z_BAND    = 0.03            # disc 부근 z 1차 필터 반폭
SURF_Z_TOL     = 0.004           # disc 표면 평면 두께
NEAR_XY_CROP   = 0.13            # disc 중심 부근만 (로봇/배경 컷)
AZ_HALF_DEG    = 32.0            # 겨냥 방위 섹터 반폭
RIM_FINE_BIN_DEG = 1.0          # 세부 방위 bin 폭 — bin별 최외곽 1점 = rim (점수 제한)

WIDEN_JOINT1_LIMIT_DEG = 175.0
FIX_DRIVE_GAINS, DRIVE_STIFFNESS, DRIVE_DAMPING = True, 2000.0, 200.0

WARMUP_STEPS     = 50
SETTLE_STABLE_N  = 12
MOVE_TIMEOUT_N   = 400
JOINT_SETTLE_TOL = 0.01
PHYSICS_CB_NAME  = "mms_ttcal_step"

_BASE_DIR = os.environ.get("MMS_HARNESS_OUT",
    os.path.expanduser("~/isaacsim/standalone_examples/play/MMS"))
OUT_DIR   = os.path.join(_BASE_DIR, "captures_turntable")
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

print(f"[TTCAL] ===== run start =====  (core={_HAS_CORE})")
print(f"[TTCAL] log file: {LOG_PATH}")

# ── 결과 시각화용 debug_draw (RGB 좌표축) ─────────────────────────────────────
try:
    from isaacsim.util.debug_draw import _debug_draw
    _draw = _debug_draw.acquire_debug_draw_interface()
except Exception:
    try:
        from omni.isaac.debug_draw import _debug_draw
        _draw = _debug_draw.acquire_debug_draw_interface()
    except Exception:
        _draw = None
        print("[TTCAL][WARN] debug_draw 없음 — 좌표축 시각화 skip")


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
        print(f"[TTCAL][WARN] camera prim not found: {cam_path}")
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


# ── rim 겨냥 자세 생성 ────────────────────────────────────────────────────────
def build_rim_view_poses(center, top_z, hint_r, T_EC):
    """N 방위에서 rim 점을 위·바깥에서 내려다보는 EE world pose 들 + 겨냥 방위."""
    poses, aims = [], []
    for az in np.linspace(0.0, 2 * np.pi, N_VIEWS, endpoint=False):
        dirxy = np.array([math.cos(az), math.sin(az), 0.0])
        aim = np.array([center[0], center[1], top_z]) + hint_r * dirxy
        eye = aim + VIEW_OUT * dirxy + np.array([0.0, 0.0, VIEW_UP])
        R_WC = look_at_camera(eye, aim, VIEW_UP_HINT)
        T_WC = make_T(R_WC, eye)
        poses.append(T_WC @ T_EC)          # 카메라 pose → EE world (시스템 T_EC=E→C)
        aims.append(float(az))
    return poses, aims


# ── rim arc 추출 (겨냥 섹터에서 세부 방위별 최외곽 1점 = 진짜 edge) ───────────
def extract_rim_arc(pts, center_xy, aim_az):
    pts = np.asarray(pts, dtype=float).reshape(-1, 3)
    rxy = np.linalg.norm(pts[:, :2] - center_xy, axis=1)
    near = pts[(np.abs(pts[:, 2] - DISC_TOP_Z) < NEAR_Z_BAND) & (rxy < NEAR_XY_CROP)]
    if len(near) < 100:
        return None
    # disc 표면 z: 위쪽 밀집(상위 70pct) 중앙값 → 그 두께 밴드
    z_surf = float(np.median(near[near[:, 2] > np.percentile(near[:, 2], 70), 2]))
    surf = near[np.abs(near[:, 2] - z_surf) < SURF_Z_TOL]
    if len(surf) < 50:
        return None
    d = surf[:, :2] - center_xy
    r = np.linalg.norm(d, axis=1)
    az = np.arctan2(d[:, 1], d[:, 0])
    daz = (az - aim_az + np.pi) % (2 * np.pi) - np.pi
    sel = np.abs(daz) < math.radians(AZ_HALF_DEG)
    if int(sel.sum()) < 20:
        return None
    s_r, s_daz, s_p = r[sel], daz[sel], surf[sel]
    # 세부 방위 bin(1°) 별 **최외곽 1점** → 얇은 고리(진짜 edge), 점수 폭발 방지
    nfb = max(8, int(2 * AZ_HALF_DEG / RIM_FINE_BIN_DEG))
    fb = ((s_daz + math.radians(AZ_HALF_DEG))
          / (2 * math.radians(AZ_HALF_DEG)) * nfb).astype(int)
    fb = np.clip(fb, 0, nfb - 1)
    rim = [s_p[idx[np.argmax(s_r[idx])]]
           for k in range(nfb) for idx in [np.where(fb == k)[0]] if len(idx)]
    return np.asarray(rim) if len(rim) >= 3 else None


# ── 런타임 상태 + 상태머신 ────────────────────────────────────────────────────
_ctx = {
    "world": None, "stage": None, "robot": None, "camera": None, "provider": None,
    "dof_idx": None, "seed_q": None, "T_W_base": None, "gt_center": None, "gt_dir": None,
    "views": [], "aims": [], "view_idx": 0, "target_q": None, "rim_pts": [],
    "phase": "MOVE", "phase_step": 0, "stable_n": 0, "step": 0, "ik_mode": "?",
    "T_W_F": None, "T_W_F_gt": None, "rim_viz": None,
}


def _start_view(i):
    views = _ctx["views"]
    while i < len(views):
        T_base_E = inv_T(_ctx["T_W_base"]) @ views[i]
        q, ok = _ctx["provider"].ik(mat_to_pose6d_mm(T_base_E), seed=_ctx["seed_q"])
        if ok:
            _ctx["view_idx"] = i
            _ctx["target_q"] = np.asarray(q, float)
            _ctx["seed_q"] = np.asarray(q, float)
            drive_joints(q)
            _ctx["phase"], _ctx["phase_step"], _ctx["stable_n"] = "MOVE", 0, 0
            print(f"[TTCAL] → view {i+1}/{len(views)} (az={math.degrees(_ctx['aims'][i]):.0f}°) IK ok")
            return
        print(f"[TTCAL]   view {i+1}: IK 실패 — skip")
        i += 1
    _ctx["phase"] = "SOLVE"


def _grab_view(i):
    pc = _ctx["camera"].get_pointcloud()
    if pc is None or getattr(pc, "size", 0) == 0:
        print(f"[TTCAL]   view {i+1}: 렌더 미준비 — skip")
        return
    arc = extract_rim_arc(pc, _ctx["gt_center"][:2], _ctx["aims"][i])
    if arc is None:
        print(f"[TTCAL]   view {i+1}: rim arc 추출 실패 (시야 확인)")
        return
    _ctx["rim_pts"].append(arc)
    rr = np.linalg.norm(arc[:, :2] - _ctx["gt_center"][:2], axis=1)
    print(f"[TTCAL]   view {i+1}: rim arc {len(arc)}점  r={rr.mean()*1000:.1f}±{rr.std()*1000:.1f}mm")


def _solve():
    if not _ctx["rim_pts"]:
        print("[TTCAL][ERROR] rim 점 없음 — 중단")
        return
    rim = np.vstack(_ctx["rim_pts"])
    if len(rim) < 6:
        print(f"[TTCAL][ERROR] rim 점 부족 {len(rim)}")
        return
    center, normal, radius, residual = fit_circle_3d(rim)
    if normal[2] < 0:
        normal = -normal
    T_B_F0 = build_T_B_F0(center, normal)
    gt_c, gt_dir = _ctx["gt_center"], _ctx["gt_dir"]
    dir_err = math.degrees(math.acos(max(-1.0, min(1.0, abs(float(normal @ gt_dir))))))
    center_xy_err = float(np.linalg.norm(center[:2] - gt_c[:2]) * 1000.0)
    radius_err = float(abs(radius - DISC_RADIUS) * 1000.0)

    print("\n[TTCAL] ===== TURNTABLE 결과 (ground-truth 대비) =====")
    print(f"[TTCAL] rim 점 합계={len(rim)} (뷰 {len(_ctx['rim_pts'])}개) | IK={_ctx['ik_mode']}")
    print(f"[TTCAL] fit: center_xy={np.round(center[:2],4).tolist()} z={center[2]:.4f} "
          f"radius={radius*1000:.2f}mm residual={residual*1000:.3f}mm")
    print(f"[TTCAL] GT : center_xy={np.round(gt_c[:2],4).tolist()} radius={DISC_RADIUS*1000:.1f}mm")
    print(f"[TTCAL] >>> dir_err={dir_err:.3f}°  center_xy_err={center_xy_err:.2f}mm  "
          f"radius_err={radius_err:.2f}mm")
    verdict = "PASS ✅" if (dir_err < 1.0 and center_xy_err < 3.0 and radius_err < 3.0) else "CHECK ⚠"
    print(f"[TTCAL]     판정: {verdict}")

    np.savez(os.path.join(OUT_DIR, "turntable_result.npz"),
             T_B_F0=T_B_F0, rim=rim, center=center, normal=normal, radius=radius,
             gt_center=gt_c, gt_dir=gt_dir, dir_err_deg=dir_err,
             center_xy_err_mm=center_xy_err, radius_err_mm=radius_err)
    _save_ply(os.path.join(OUT_DIR, "rim_points.ply"), rim)

    # ── viewport 시각화: fit / GT 프레임을 RGB 좌표축으로 ──
    _ctx["T_W_F"] = inv_T(T_B_F0)                            # F 의 world pose (base=world)
    _ctx["T_W_F_gt"] = inv_T(build_T_B_F0(gt_c, gt_dir))
    _ctx["rim_viz"] = rim[:: max(1, len(rim) // 400)]
    _draw_result()
    print("[TTCAL] viewport 좌표축: 굵은선=fit, 가는선=GT, 청록점=rim  (x빨강 y초록 z파랑)")
    print("[TTCAL] ===== COMPLETE =====\n")


def _save_ply(path, pts):
    pts = np.asarray(pts, float)
    with open(path, "w") as f:
        f.write(f"ply\nformat ascii 1.0\nelement vertex {len(pts)}\n")
        f.write("property float x\nproperty float y\nproperty float z\nend_header\n")
        for p in pts:
            f.write(f"{p[0]:.6f} {p[1]:.6f} {p[2]:.6f}\n")


def _axes_lines(T_W_F, length):
    """프레임 → 3 좌표축 선분 (x빨강/y초록/z파랑)."""
    o = T_W_F[:3, 3].astype(float)
    starts, ends, cols = [], [], []
    for ax, col in ((0, (1.0, 0.0, 0.0, 1.0)), (1, (0.0, 1.0, 0.0, 1.0)),
                    (2, (0.0, 0.0, 1.0, 1.0))):
        starts.append(tuple(o))
        ends.append(tuple(o + length * T_W_F[:3, ax].astype(float)))
        cols.append(col)
    return starts, ends, cols


def _draw_result():
    """fit 프레임(굵게) + GT 프레임(가늘게·길게) + rim 점 을 viewport 에 그린다."""
    if _draw is None or _ctx.get("T_W_F") is None:
        return
    _draw.clear_lines()
    _draw.clear_points()
    s1, e1, c1 = _axes_lines(_ctx["T_W_F"], 0.08)       # fit (굵게)
    _draw.draw_lines(s1, e1, c1, [7, 7, 7])
    s2, e2, c2 = _axes_lines(_ctx["T_W_F_gt"], 0.115)   # GT (가늘게·길게 — 정렬 확인)
    _draw.draw_lines(s2, e2, c2, [2, 2, 2])
    rim = _ctx.get("rim_viz")
    if rim is not None and len(rim):
        pts = [tuple(float(v) for v in p) for p in rim]
        _draw.draw_points(pts, [(0.0, 1.0, 1.0, 1.0)] * len(pts), [5] * len(pts))


def _on_physics_step(step_size):
    ph = _ctx["phase"]
    _ctx["phase_step"] += 1
    if ph == "MOVE":
        conv = _joint_err(_ctx["target_q"]) < JOINT_SETTLE_TOL
        _ctx["stable_n"] = _ctx["stable_n"] + 1 if conv else 0
        if (_ctx["stable_n"] >= SETTLE_STABLE_N and _ctx["phase_step"] >= WARMUP_STEPS) \
                or _ctx["phase_step"] >= MOVE_TIMEOUT_N:
            _grab_view(_ctx["view_idx"])
            _start_view(_ctx["view_idx"] + 1)
    elif ph == "SOLVE":
        _solve()
        _ctx["phase"] = "DONE"
    elif ph == "DONE":
        if _draw is not None and _ctx["step"] % 60 == 0:
            _draw_result()                       # 좌표축 지속(스테이지 업데이트로 지워짐 방지)
    _ctx["step"] += 1


# ── setup ─────────────────────────────────────────────────────────────────────
async def setup_async():
    if not _HAS_CORE:
        print("[TTCAL][ERROR] MMS 코어 로드 실패 — 위 WARN 확인. 중단.")
        return
    if World.instance() is not None:
        try:
            World.instance().clear_all_callbacks()
        except Exception:
            pass
        World.clear_instance()

    if _draw is not None:                        # 이전 실행 좌표축 제거
        _draw.clear_lines()
        _draw.clear_points()

    print(f"[TTCAL] Opening USD: {USD_PATH}")
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
        print(f"[TTCAL] 마블 비활성화: {MARBLE_PRIM_PATH}")

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
    T_W_E = get_prim_world_T(stage, xc, EE_LINK_PATH)
    T_W_C = get_prim_world_T(stage, xc, CAMERA_PRIM)
    T_W_base = get_prim_world_T(stage, xc, ROBOT_PRIM)
    T_EC_gt = inv_T(T_W_C) @ T_W_E                 # 시스템 규약 E→C (hand-eye 와 동일)
    T_tt = get_prim_world_T(stage, xc, TURNTABLE_MESH)
    gt_center = np.array([T_tt[0, 3], T_tt[1, 3], DISC_TOP_Z], dtype=float)
    gt_dir = np.array([0.0, 0.0, 1.0], dtype=float)

    provider = RobotIK(kin=kin, robot_ip=ROBOT_IP, use_sdk=USE_XARM_SDK,
                       xarm_sdk_path=_XARM_SDK, logger=print)
    home_q = np.radians(ARTEC_HOME_JOINTS_DEG)
    views, aims = build_rim_view_poses(gt_center, DISC_TOP_Z, DISC_RADIUS_HINT, T_EC_gt)

    _ctx.update(world=world, stage=stage, robot=robot, camera=cam, provider=provider,
                dof_idx=build_dof_index(robot), seed_q=home_q.copy(), T_W_base=T_W_base,
                gt_center=gt_center, gt_dir=gt_dir, views=views, aims=aims,
                view_idx=0, rim_pts=[], ik_mode=provider.mode,
                phase="MOVE", phase_step=0, stable_n=0, step=0)

    print(f"[TTCAL] GT disc center={np.round(gt_center,4).tolist()} radius={DISC_RADIUS*1000:.1f}mm | "
          f"뷰 {N_VIEWS}개 rim 겨냥")
    world.add_physics_callback(PHYSICS_CB_NAME, _on_physics_step)
    drive_joints(home_q)
    _start_view(0)
    await world.play_async()
    print("[TTCAL] 시작: rim 겨냥 멀티뷰 캡처 → arc 합산 → fit_circle_3d → T_B_F0 → GT 비교.")


def stop():
    world = _ctx.get("world") or World.instance()
    if world is not None:
        try:
            world.remove_physics_callback(PHYSICS_CB_NAME)
        except Exception:
            pass
        world.stop()
    print("[TTCAL] Stopped.")


asyncio.ensure_future(setup_async())
