"""
MMS Extension Script - Isaac Sim 5.1.0 (VSCode Extension / Script Editor 버전)
USD: 자산 루트의 frame_xarm7_spider_turntable/v2.usd (mms_paths 가 해석)

⚠️ 이 파일은 standalone(python.sh)이 아니라 **이미 실행 중인 Isaac Sim** 안에서
   돌리는 버전이다. 다음 둘 중 하나로 실행한다:
     - VSCode 의 Isaac Sim 확장(코드 러너)으로 이 파일 실행
     - Isaac Sim GUI > Window > Script Editor 에 붙여넣고 실행

   standalone(MMS.py)과의 차이:
     1. SimulationApp 을 만들지 않는다 (GUI 가 이미 app 을 제공).
     2. UI 를 멈추는 블로킹 while 루프를 쓰지 않는다.
        → async 셋업 + world.add_physics_callback() 으로 매 물리스텝 처리.
     3. world.reset_async() / world.play_async() 사용.
     4. python.sh 로 직접 실행하면 안 된다.
"""

# ── 1. import (SimulationApp 없음 — app 은 이미 실행 중) ───────────────────────
import os
import math
import asyncio
import numpy as np
import omni.usd
from isaacsim.core.api import World
from isaacsim.core.utils.stage import open_stage
from isaacsim.core.api.robots import Robot
from isaacsim.core.api.objects import VisualCuboid
from isaacsim.core.prims import RigidPrim
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.sensors.camera import Camera
from pxr import Usd, UsdGeom, Sdf, Gf

# ── 2. 상수 ───────────────────────────────────────────────────────────────────
USD_PATH     = _asset("frame_xarm7_spider_turntable/v2.usd")
ROBOT_PRIM   = "/World/xarm7"
JOINTS_SCOPE = "/World/xarm7/joints"

# 초기 joint 각도 (radians)
INITIAL_JOINT_POS = {
    "joint1":  0.0,
    "joint2":  0.0,
    "joint3":  0.0,
    "joint4":  0.0,
    "joint5":  0.0,
    "joint6":  0.0,
    "joint7":  0.0,
}

# ── 스캐너 카메라 (Artec Space Spider 모사) ──────────────────────────────────
# 카메라 prim 경로 (xarm7 EE의 스캐너 하위)
CAMERA_PRIM = "/World/xarm7/link7/Artec_Space_Spider_mm/Camera"
# 스캔 대상물 (정렬 확인용)
TARGET_PRIM = "/World/ScanTarget/Solid_Marble"

# Artec Space Spider 실제 스펙 기반 광학값
#   - 작동거리 0.2~0.3 m, 작동거리 중앙(≈0.25 m)에서 선형 FOV ≈ 135×105 mm
#     → 수평 FOV ≈ 30°, 수직 FOV ≈ 23° (4:3 센서)
#   - 3D 텍스처 센서 약 1.3 MP → 1280×960
SPIDER_HFOV_DEG          = 30.0          # 목표 수평 시야각
SPIDER_WORKING_DISTANCE  = (0.2, 0.3)    # (near, far) m — 클리핑 범위
SPIDER_FOCUS_DISTANCE    = 0.25          # m — 작동거리 중앙
SCANNER_RESOLUTION       = (1280, 960)   # (width, height), 4:3

# 캡처 설정 — get_rgba/get_depth/get_pointcloud 결과를 주기적으로 저장
CAPTURE_ENABLED  = True
CAPTURE_INTERVAL = 300                    # N 스텝마다 1회 저장
# ★ VSCode Isaac 코드러너는 __file__ 을 확장 자신의 디렉토리로 잡아 출력이 엉뚱한
#   곳(.../code_editor/vscode/)에 묻힌다 → 스크립트 위치를 절대경로로 고정.
_BASE_DIR        = "/home/keti/isaacsim/standalone_examples/play/MMS"
CAPTURE_DIR      = os.path.join(_BASE_DIR, "captures")
# 정렬 상태 리포트 주기 (스텝)
REPORT_INTERVAL  = 120

# 물리 콜백 이름 (재실행 시 중복 등록 방지에 사용)
PHYSICS_CB_NAME = "mms_scan_step"

# ── IK 추종 (GUI 에서 타깃 프레임을 드래그하면 로봇 EE 가 따라감) ─────────────
# GUI 의 Move/Rotate gizmo 로 아래 prim 을 움직이면, 자코비안 DLS IK 로 EE(link7)
# 가 그 6-DOF 포즈를 추종한다. (xarm7 디스크립터/URDF 불필요 — PhysX 자코비안 사용)
IK_FOLLOW_ENABLED = True
TARGET_PRIM_PATH  = "/World/ee_target"   # 사용자가 GUI 에서 움직이는 타깃 프레임
EE_LINK_NAME      = "link7"              # IK 가 맞출 EE 링크 (= 스캐너 마운트)
EE_LINK_PATH      = f"{ROBOT_PRIM}/{EE_LINK_NAME}"

IK_DAMPING   = 0.08    # DLS 감쇠 λ (특이점 안정화)
IK_STEP_GAIN = 0.7     # 스텝당 보정 비율 (0~1)
IK_MAX_DQ    = 0.04    # 스텝당 joint 변화 상한 (rad) — dq 벡터를 방향보존 스케일
IK_POS_TOL   = 0.002   # 위치 수렴 허용오차 (m) — 이하이면 위치항 무시
IK_ROT_TOL   = 0.01    # 자세 수렴 허용오차 (rad)
# 스텝당 "쫓는" 오차 상한 — 큰 목표를 한 번에 쫓지 않고 점진 접근시켜
# 자코비안 선형화를 유효하게 유지(없으면 큰 회전에서 해가 엉뚱한 자세로 빠져
# joint1 이 한계에 핀되어 얼어붙음 → 자세가 거의 안 바뀜).
IK_POS_CLAMP = 0.03    # (m)
IK_ROT_CLAMP = 0.15    # (rad)

# joint1 한계 정상화: 원본 USD 는 ±1.0 rad(±57°)로 비정상적으로 좁음
# (xArm7 베이스 실제 ±360°). 좁은 한계 탓에 IK 가 막힘 → 합리적 값으로 확대.
# None 이면 USD 원본 유지.
WIDEN_JOINT1_LIMIT_DEG = 175.0

# ── 관절 드라이브 게인 보정 (트램블링 방지) ──────────────────────────────────
# USD 의 drive damping 이 ~1e-4(거의 0) 라 stiffness(10000)와 합쳐져 심하게
# 과소감쇠 → 목표 주변에서 진동(부들부들). reset 후 런타임 게인으로 덮어쓴다.
# (검증: 현재 EE std 6.7mm → 아래 값 적용 시 0.04mm)
FIX_DRIVE_GAINS = True
DRIVE_STIFFNESS = 2000.0
DRIVE_DAMPING   = 200.0


def bake_joint_initial_state(stage, joints_scope: str, joint_pos: dict):
    """
    각 RevoluteJoint prim에 대해:
      1. state:angular:physics:position  → PhysX 초기 각도 (degrees)
      2. drive:angular:physics:targetPosition → Drive 목표각도 (degrees)
         (이걸 설정 안 하면 Drive가 0도로 당겨서 떨림 발생)
    두 값을 동일하게 설정해야 Play 직후 떨림이 없음.
    """
    baked = []
    for joint_name, angle_rad in joint_pos.items():
        joint_path = f"{joints_scope}/{joint_name}"
        joint_prim = stage.GetPrimAtPath(joint_path)

        if not joint_prim.IsValid():
            print(f"[MMS][WARN] joint prim not found: {joint_path}")
            continue

        angle_deg = math.degrees(angle_rad)

        # ── (A) 초기 state 위치 ──────────────────────────────────────────
        state_attr = joint_prim.GetAttribute("state:angular:physics:position")
        if not state_attr.IsValid():
            state_attr = joint_prim.CreateAttribute(
                "state:angular:physics:position",
                Sdf.ValueTypeNames.Float
            )
        state_attr.Set(float(angle_deg))

        # ── (B) Drive targetPosition — 이게 없으면 Drive가 0도로 당김 ────
        drive_attr = joint_prim.GetAttribute("drive:angular:physics:targetPosition")
        if drive_attr.IsValid():
            drive_attr.Set(float(angle_deg))
        else:
            # Drive 속성이 없으면 생성 (Drive가 아예 없는 joint는 무시)
            print(f"[MMS][INFO] {joint_name}: drive:angular:physics:targetPosition 없음 — 생성 시도")
            drive_attr = joint_prim.CreateAttribute(
                "drive:angular:physics:targetPosition",
                Sdf.ValueTypeNames.Float
            )
            drive_attr.Set(float(angle_deg))

        baked.append((joint_name, round(angle_deg, 2)))

    print(f"[MMS] Baked state+drive targetPos: {baked}")


def configure_scanner_camera(stage, cam_path: str):
    """
    스캐너 카메라(USD Camera prim)의 광학·작동거리 속성을 Artec Space Spider에
    맞춰 설정한다. (마운트 transform: translate/orient/scale 은 건드리지 않음)

      - focalLength : 기존 horizontalAperture 를 유지한 채 목표 수평 FOV(SPIDER_HFOV_DEG)
                      가 나오도록 계산. FOV = 2·atan(aperture / (2·focalLength))
      - clippingRange / focusDistance : 작동거리(0.2~0.3 m)에 맞춤
      - verticalAperture : 해상도(4:3)에 맞춰 정사각 픽셀이 되도록 설정
                           → Camera 센서가 set_resolution 시 자동 보정도 하지만
                             USD 단계에서도 일관성 있게 맞춰 둔다.
    """
    cam = stage.GetPrimAtPath(cam_path)
    if not cam.IsValid():
        print(f"[MMS][WARN] camera prim not found: {cam_path}")
        return

    def _attr(name, vtype, value):
        a = cam.GetAttribute(name)
        if not a.IsValid():
            a = cam.CreateAttribute(name, vtype)
        a.Set(value)
        return a

    # 현재 수평 조리개(센서 폭) 유지 — 없으면 USD 기본값 사용
    h_ap_attr = cam.GetAttribute("horizontalAperture")
    h_aperture = float(h_ap_attr.Get()) if h_ap_attr.IsValid() and h_ap_attr.Get() else 20.5

    # 목표 수평 FOV → focalLength 역산
    hfov_rad = math.radians(SPIDER_HFOV_DEG)
    focal = h_aperture / (2.0 * math.tan(hfov_rad / 2.0))

    # 정사각 픽셀을 위한 수직 조리개 (해상도 종횡비 적용)
    w, h = SCANNER_RESOLUTION
    v_aperture = h_aperture * (h / w)
    vfov_deg = math.degrees(2.0 * math.atan(v_aperture / (2.0 * focal)))

    _attr("focalLength",        Sdf.ValueTypeNames.Float, float(focal))
    _attr("horizontalAperture", Sdf.ValueTypeNames.Float, float(h_aperture))
    _attr("verticalAperture",   Sdf.ValueTypeNames.Float, float(v_aperture))
    _attr("clippingRange",      Sdf.ValueTypeNames.Float2,
          Gf.Vec2f(float(SPIDER_WORKING_DISTANCE[0]), float(SPIDER_WORKING_DISTANCE[1])))
    _attr("focusDistance",      Sdf.ValueTypeNames.Float, float(SPIDER_FOCUS_DISTANCE))

    print(f"[MMS] Scanner camera configured: "
          f"focal={focal:.2f}, hAperture={h_aperture:.2f}, vAperture={v_aperture:.2f} "
          f"→ HFOV={SPIDER_HFOV_DEG:.1f}°, VFOV={vfov_deg:.1f}° | "
          f"clip={SPIDER_WORKING_DISTANCE} m, focus={SPIDER_FOCUS_DISTANCE} m")


def _world_pos_and_forward(stage, xform_cache, prim_path):
    """prim 의 world 위치와 카메라 광축(-Z) world 방향(정규화)을 반환."""
    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        return None, None
    m = xform_cache.GetLocalToWorldTransform(prim)
    pos = m.ExtractTranslation()
    rot = m.ExtractRotationMatrix()          # scale 포함될 수 있음 → 정규화 필요
    fwd = Gf.Vec3d(0, 0, -1) * rot
    if fwd.GetLength() > 1e-9:
        fwd = fwd.GetNormalized()
    return pos, fwd


def report_scan_geometry(stage, xform_cache):
    """
    런타임 조준 상태 리포트:
      카메라는 EE에 강체 고정이므로 조준은 joint 각도가 결정한다.
      대상물까지의 거리와 광축 오프셋 각도를 측정해, 작동거리/FOV 안에 들어오는지 확인.
    """
    xform_cache.Clear()
    cam_pos, cam_fwd = _world_pos_and_forward(stage, xform_cache, CAMERA_PRIM)
    tgt_pos, _       = _world_pos_and_forward(stage, xform_cache, TARGET_PRIM)
    if cam_pos is None or tgt_pos is None:
        return

    to_tgt = tgt_pos - cam_pos
    dist = to_tgt.GetLength()
    if dist < 1e-9:
        return
    to_tgt_n = to_tgt / dist
    dot = max(-1.0, min(1.0, cam_fwd[0] * to_tgt_n[0]
                            + cam_fwd[1] * to_tgt_n[1]
                            + cam_fwd[2] * to_tgt_n[2]))
    off_axis_deg = math.degrees(math.acos(dot))

    near, far = SPIDER_WORKING_DISTANCE
    in_wd  = near <= dist <= far
    in_fov = off_axis_deg <= (SPIDER_HFOV_DEG / 2.0)
    wd_tag  = "OK" if in_wd  else f"OUT(작동거리 {near}~{far} m)"
    fov_tag = "OK" if in_fov else f"OUT(±{SPIDER_HFOV_DEG/2:.1f}°)"
    print(f"[MMS][AIM] dist={dist:.3f} m [{wd_tag}] | "
          f"off-axis={off_axis_deg:.1f}° [{fov_tag}]")


def capture_scanner_frame(camera, step):
    """카메라 센서에서 RGB/Depth/PointCloud 를 받아 디스크에 저장."""
    os.makedirs(CAPTURE_DIR, exist_ok=True)
    rgba = camera.get_rgba()
    depth = camera.get_depth()
    pc = camera.get_pointcloud()

    saved = []
    if rgba is not None and getattr(rgba, "size", 0) > 0:
        np.save(os.path.join(CAPTURE_DIR, f"rgb_{step:06d}.npy"), rgba)
        saved.append("rgb")
        try:
            from PIL import Image
            Image.fromarray(rgba[:, :, :3].astype(np.uint8)).save(
                os.path.join(CAPTURE_DIR, f"rgb_{step:06d}.png"))
            saved.append("png")
        except Exception:
            pass
    if depth is not None and getattr(depth, "size", 0) > 0:
        np.save(os.path.join(CAPTURE_DIR, f"depth_{step:06d}.npy"), depth)
        saved.append("depth")
    if pc is not None and getattr(pc, "size", 0) > 0:
        np.save(os.path.join(CAPTURE_DIR, f"pointcloud_{step:06d}.npy"), pc)
        saved.append(f"pc[{len(pc)}]")
    print(f"[MMS][CAPTURE] step={step} → {CAPTURE_DIR} ({', '.join(saved) or 'no data yet'})")


# ── IK 유틸 (자코비안 DLS) ────────────────────────────────────────────────────
def _quat_to_wxyz(gf_rot):
    """Gf.Rotation → numpy (w, x, y, z)."""
    q = gf_rot.GetQuaternion()
    i = q.GetImaginary()
    return np.array([q.GetReal(), i[0], i[1], i[2]], dtype=np.float64)


def _orientation_error(q_cur, q_des):
    """
    현재→목표 자세 오차를 world 프레임의 회전벡터(axis*angle, 3-vec)로 반환.
    q = (w, x, y, z). e = vee(q_des ⊗ q_cur⁻¹).
    """
    w0, x0, y0, z0 = q_cur
    w1, x1, y1, z1 = q_des
    # q_cur⁻¹ = conj (단위 가정)
    cw, cx, cy, cz = w0, -x0, -y0, -z0
    # q_err = q_des ⊗ q_cur⁻¹
    ew = w1 * cw - x1 * cx - y1 * cy - z1 * cz
    ex = w1 * cx + x1 * cw + y1 * cz - z1 * cy
    ey = w1 * cy - x1 * cz + y1 * cw + z1 * cx
    ez = w1 * cz + x1 * cy - y1 * cx + z1 * cw
    if ew < 0.0:                      # 최단 회전 선택
        ew, ex, ey, ez = -ew, -ex, -ey, -ez
    v = np.array([ex, ey, ez], dtype=np.float64)
    nv = np.linalg.norm(v)
    if nv < 1e-9:
        return np.zeros(3)
    angle = 2.0 * math.atan2(nv, ew)
    return (v / nv) * angle


def solve_ik_step(ctx):
    """
    1 스텝 자코비안 DLS IK:
      EE(link7) 의 현재 6-DOF 포즈를 타깃 프레임 포즈로 한 스텝 보정.
      dq = Jᵀ (J Jᵀ + λ²I)⁻¹ · e,  e = [pos_err(3); rot_err(3)]
    """
    robot = ctx["robot"]
    view = ctx["view"]
    rigid_ee = ctx["rigid_ee"]
    xc = ctx["xform_cache"]

    # ── 타깃 프레임 world 포즈 (사용자가 GUI 에서 편집 → USD authored) ────────
    xc.Clear()
    tgt_prim = ctx["stage"].GetPrimAtPath(TARGET_PRIM_PATH)
    if not tgt_prim.IsValid():
        return
    m_t = xc.GetLocalToWorldTransform(tgt_prim)
    p_des = np.array(m_t.ExtractTranslation(), dtype=np.float64)
    # ★ 타깃 큐브에 scale(0.03)이 있어 그냥 ExtractRotation() 하면 회전이 왜곡됨
    #    (60° → ~118°). 반드시 scale 제거 후 추출해야 올바른 자세 목표가 나온다.
    q_des = _quat_to_wxyz(m_t.RemoveScaleShear().ExtractRotation())

    # ── 게이팅: 타깃이 움직였을 때만 IK 활성화 (정지 시 드라이브가 명령 유지) ─
    # 이게 없으면 중력 처짐을 매 스텝 보정하느라 미세 진동이 남는다.
    pp, qp = ctx.get("tgt_p_prev"), ctx.get("tgt_q_prev")
    if pp is None or np.linalg.norm(p_des - pp) > 1e-4 \
            or np.linalg.norm(_orientation_error(qp, q_des)) > 1e-3:
        ctx["ik_active"] = True
    ctx["tgt_p_prev"], ctx["tgt_q_prev"] = p_des, q_des
    if not ctx.get("ik_active", False):
        return                                       # 타깃 정지 + 수렴 완료 → 유지

    # ── EE 현재 world 포즈 (PhysX 텐서 — 자코비안과 동일 프레임) ──────────────
    pos, quat = rigid_ee.get_world_poses()
    p_cur = np.asarray(pos[0], dtype=np.float64)
    q_cur = np.asarray(quat[0], dtype=np.float64)

    # ── 6-DOF 오차 ───────────────────────────────────────────────────────────
    e_pos = p_des - p_cur
    e_rot = _orientation_error(q_cur, q_des)
    # 수렴 판정은 tol 로만. 오차 자체를 0 으로 만들지 말 것!
    # (작은 위치오차를 0 으로 만들면 회전 중 위치 제약이 사라져 IK 가 위치보존·손목
    #  사용 해를 잃고 base 로 발산함 → 자세가 거의 안 바뀌는 원인이었음)
    if np.linalg.norm(e_pos) < IK_POS_TOL and np.linalg.norm(e_rot) < IK_ROT_TOL:
        ctx["ik_active"] = False                 # 수렴 → 비활성(유지). 타깃 이동 시 재활성
        return
    # 오차 크기 제한 — 큰 목표를 점진 접근 (선형화 유효 유지, joint1 핀 방지)
    npos = np.linalg.norm(e_pos)
    if npos > IK_POS_CLAMP:
        e_pos = e_pos * (IK_POS_CLAMP / npos)
    nrot = np.linalg.norm(e_rot)
    if nrot > IK_ROT_CLAMP:
        e_rot = e_rot * (IK_ROT_CLAMP / nrot)
    e = np.concatenate([e_pos, e_rot])           # (6,)

    # ── 자코비안 (M, bodies, 6, gen_dof) → EE 링크의 (6, gen_dof) ────────────
    jac = view.get_jacobians()
    if jac is None:
        return
    jac = np.asarray(jac)[0]                      # (bodies, 6, gen_dof)
    J = jac[ctx["ee_jac_row"]]                    # (6, gen_dof)
    # floating-base 아티큘레이션은 앞쪽에 베이스 6-DOF 열이 붙음 → 실제 관절만 사용
    n = ctx["num_dof"]
    if J.shape[1] != n:
        J = J[:, -n:]                             # (6, num_dof)

    # ── DLS 해 ───────────────────────────────────────────────────────────────
    JT = J.T
    lam2 = IK_DAMPING ** 2
    dq = JT @ np.linalg.solve(J @ JT + lam2 * np.eye(6), e)   # (dof,)

    dq *= IK_STEP_GAIN
    # 방향 보존 스케일링 (요소별 클립은 DLS 해의 방향을 왜곡 → 수렴 방해)
    mx = float(np.max(np.abs(dq)))
    if mx > IK_MAX_DQ:
        dq *= IK_MAX_DQ / mx

    q_now = np.asarray(robot.get_joint_positions(), dtype=np.float64)
    q_cmd = q_now + dq
    lo, hi = ctx["dof_lower"], ctx["dof_upper"]
    q_cmd = np.clip(q_cmd, lo, hi)

    robot.apply_action(ArticulationAction(joint_positions=q_cmd))


# ── 3. 런타임 상태 (블로킹 루프 대신 물리 콜백에서 사용) ──────────────────────
_ctx = {
    "world": None,
    "stage": None,
    "robot": None,
    "camera": None,
    "xform_cache": None,
    "view": None,
    "rigid_ee": None,
    "ee_jac_row": None,
    "dof_lower": None,
    "dof_upper": None,
    "ik_active": False,      # 타깃 이동 시 True, 수렴 시 False (정지 떨림 방지)
    "tgt_p_prev": None,
    "tgt_q_prev": None,
    "step": 0,
}


def _on_physics_step(step_size):
    """매 물리스텝마다 호출 — standalone 의 while 루프 본문에 해당."""
    step = _ctx["step"]

    # 타깃 프레임 추종 IK (GUI gizmo 로 /World/ee_target 을 움직이면 EE 가 따라감)
    if IK_FOLLOW_ENABLED and _ctx.get("view") is not None:
        try:
            solve_ik_step(_ctx)
        except Exception as exc:           # IK 실패가 루프를 죽이지 않도록
            if step % 120 == 0:
                print(f"[MMS][IK][WARN] {exc}")

    # 정렬 상태 리포트 (카메라 광축 vs 대상물, 작동거리/FOV 확인)
    if REPORT_INTERVAL and step % REPORT_INTERVAL == 0:
        report_scan_geometry(_ctx["stage"], _ctx["xform_cache"])

    # 스캐너 프레임 캡처 (RGB/Depth/PointCloud 저장)
    if CAPTURE_ENABLED and step > 0 and step % CAPTURE_INTERVAL == 0:
        capture_scanner_frame(_ctx["camera"], step)

    _ctx["step"] = step + 1


def configure_joint_drives(robot):
    """
    관절 PD 드라이브 게인을 잘 감쇠된 값으로 덮어쓴다 (트램블링 방지).
    USD 원본 damping≈1e-4 + stiffness=10000 → 과소감쇠 진동.
    (world.reset() 이후, articulation view 가 초기화된 뒤 호출)
    """
    view = robot._articulation_view
    nd = view.num_dof
    kps = np.full((1, nd), DRIVE_STIFFNESS, dtype=np.float32)
    kds = np.full((1, nd), DRIVE_DAMPING, dtype=np.float32)
    view.set_gains(kps=kps, kds=kds)
    print(f"[MMS] Joint drive gains set: stiffness={DRIVE_STIFFNESS}, "
          f"damping={DRIVE_DAMPING} (트램블링 방지)")


def prepare_ik(stage, robot):
    """
    IK 추종에 필요한 상태를 구성하고 GUI 타깃 프레임을 현재 EE 포즈에 생성한다.
    (world.reset() 이후, 물리 핸들이 유효할 때 호출해야 함)
    """
    view = robot._articulation_view
    num_dof = view.num_dof
    num_bodies = view.num_bodies
    body_index = view.get_body_index(EE_LINK_NAME)

    # 자코비안의 body 차원이 num_bodies 면 floating-base(베이스 포함),
    # num_bodies-1 이면 fixed-base → EE row 인덱스 보정
    jac_shape = view.get_jacobian_shape()
    nb = int(np.asarray(jac_shape)[0]) if jac_shape is not None else num_bodies
    ee_jac_row = body_index if nb == num_bodies else body_index - 1

    lim = np.asarray(view.get_dof_limits())[0]      # (dof, 2)
    dof_lower = lim[:, 0].astype(np.float64)
    dof_upper = lim[:, 1].astype(np.float64)

    # EE(link7) 현재 world 포즈를 PhysX 에서 읽어 타깃 프레임 초기 위치로 사용
    rigid_ee = RigidPrim(prim_paths_expr=EE_LINK_PATH)
    rigid_ee.initialize()
    pos, quat = rigid_ee.get_world_poses()
    p0 = np.asarray(pos[0], dtype=np.float64)
    q0 = np.asarray(quat[0], dtype=np.float64)

    # GUI 에서 Move/Rotate gizmo 로 잡고 움직일 타깃 프레임 (작은 초록 큐브)
    VisualCuboid(
        prim_path=TARGET_PRIM_PATH,
        name="ee_target",
        position=p0,
        orientation=q0,
        scale=np.array([0.03, 0.03, 0.03]),
        color=np.array([0.0, 1.0, 0.0]),
    )

    _ctx.update(
        view=view,
        rigid_ee=rigid_ee,
        ee_jac_row=ee_jac_row,
        num_dof=num_dof,
        dof_lower=dof_lower,
        dof_upper=dof_upper,
    )
    print(f"[MMS][IK] ready: EE={EE_LINK_NAME} jac_row={ee_jac_row} "
          f"(jac_shape={tuple(np.asarray(jac_shape).tolist())}, num_dof={num_dof}) | "
          f"target='{TARGET_PRIM_PATH}' @ {p0.round(3).tolist()}")
    print("[MMS][IK] GUI 에서 초록 큐브(/World/ee_target)를 Move/Rotate 로 옮기면 "
          "EE 가 따라갑니다.")


async def setup_async():
    """GUI 안에서 비동기로 씬을 구성하고 시뮬레이션을 시작한다."""
    # ── 재실행 대비: 기존 World 싱글톤 정리 ──────────────────────────────────
    if World.instance() is not None:
        try:
            World.instance().clear_all_callbacks()
        except Exception:
            pass
        World.clear_instance()

    # ── USD Stage 열기 ───────────────────────────────────────────────────────
    print(f"[MMS] Opening USD: {USD_PATH}")
    open_stage(usd_path=USD_PATH)
    stage = omni.usd.get_context().get_stage()

    # ── joint1 한계 정상화 (IK 가 막히지 않도록) ─────────────────────────────
    if WIDEN_JOINT1_LIMIT_DEG is not None:
        j1 = stage.GetPrimAtPath(f"{JOINTS_SCOPE}/joint1")
        if j1.IsValid():
            j1.GetAttribute("physics:lowerLimit").Set(-float(WIDEN_JOINT1_LIMIT_DEG))
            j1.GetAttribute("physics:upperLimit").Set(float(WIDEN_JOINT1_LIMIT_DEG))
            print(f"[MMS] joint1 한계 확대: ±{WIDEN_JOINT1_LIMIT_DEG}° (원본 ±57°)")

    # ── joint 초기값 굽기 + 카메라 광학 설정 ─────────────────────────────────
    bake_joint_initial_state(stage, JOINTS_SCOPE, INITIAL_JOINT_POS)
    configure_scanner_camera(stage, CAMERA_PRIM)

    # ── World / 로봇 / 카메라 센서 ───────────────────────────────────────────
    world = World(
        physics_dt=1.0 / 60.0,
        rendering_dt=1.0 / 60.0,
        stage_units_in_meters=1.0,
    )

    # 익스텐션 워크플로우 대비: 실행 경로에 따라 World.__init__ 이 physics context 를
    # 동기 생성하지 않을 수 있음(ISAAC_LAUNCHED_FROM_TERMINAL 분기). 이 경우
    # reset_async → play_async 의 get_physics_context().warm_start() 가 None 으로 죽음.
    # → 현재 stage 위에 physics context 를 명시적으로 초기화 (stage 는 새로 안 만듦).
    if world.get_physics_context() is None:
        await world.initialize_simulation_context_async()

    robot = world.scene.add(Robot(prim_path=ROBOT_PRIM, name="xarm7"))
    scanner_cam = Camera(prim_path=CAMERA_PRIM, resolution=SCANNER_RESOLUTION)

    # ── 초기화 (extension 에서는 reset_async 사용) ───────────────────────────
    await world.reset_async()

    # 관절 드라이브 게인 보정 (트램블링 방지) — reset 직후 view 초기화된 뒤
    if FIX_DRIVE_GAINS:
        configure_joint_drives(robot)

    scanner_cam.initialize()
    scanner_cam.add_distance_to_image_plane_to_frame()   # depth
    scanner_cam.add_pointcloud_to_frame()                # point cloud
    print(f"[MMS] Scanner camera sensor ready: resolution={scanner_cam.get_resolution()}, "
          f"focal={scanner_cam.get_focal_length()}, clip={scanner_cam.get_clipping_range()}")

    # ── 런타임 상태 저장 ─────────────────────────────────────────────────────
    _ctx.update(
        world=world,
        stage=stage,
        robot=robot,
        camera=scanner_cam,
        xform_cache=UsdGeom.XformCache(Usd.TimeCode.Default()),
        step=0,
    )

    # ── IK 추종 준비 (타깃 프레임 생성 + 자코비안/limits 캐시) ────────────────
    if IK_FOLLOW_ENABLED:
        prepare_ik(stage, robot)

    # ── 물리 콜백 등록 ───────────────────────────────────────────────────────
    world.add_physics_callback(PHYSICS_CB_NAME, _on_physics_step)

    print(f"[MMS] DOF count : {robot.num_dof}")
    print(f"[MMS] DOF names : {robot.dof_names}")
    print(f"[MMS] Joint pos : {robot.get_joint_positions()}")

    # ── 재생 시작 (블로킹 X — GUI 업데이트 루프가 물리 콜백을 구동) ──────────
    await world.play_async()
    print("[MMS] Simulation running (extension mode). Timeline 의 Stop 으로 중지.")


def stop():
    """정지 + 콜백/싱글톤 정리 (Script Editor 에서 수동 호출용)."""
    world = _ctx.get("world") or World.instance()
    if world is not None:
        try:
            world.remove_physics_callback(PHYSICS_CB_NAME)
        except Exception:
            pass
        world.stop()
    print("[MMS] Stopped.")


# ── 4. 진입점: 이미 실행 중인 app 의 이벤트 루프에 셋업 코루틴 예약 ───────────
asyncio.ensure_future(setup_async())
