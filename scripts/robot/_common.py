#!/usr/bin/env python
# scripts/robot/_common.py
#
# scripts/robot/* 공용 헬퍼 — 연결·안전검사·확인 프롬프트.
# 로봇을 움직이는 스크립트는 전부 여기의 precheck() 를 거친다.

import sys
from pathlib import Path

import numpy as np

_PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(_PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(_PROJECT_ROOT))

from utils.robot.xarm_interface import XArmInterface          # noqa: E402
from utils.robot import xarm7_kinematics as kin               # noqa: E402

ROBOT_IP = "192.168.1.210"

# 특이점 경고 임계. sigma_min 은 스케일된 야코비안의 최소 특이값 —
# 0 에 가까울수록 그 방향으로 관절속도가 발산한다.
SIGMA_MIN_WARN = 0.05

# xArm7 관절 가동범위 (deg). UFACTORY xArm7 사양 값이다.
#
# ★ utils/robot/xarm7_kinematics.py 의 해석 IK 는 관절 한계를 **강제하지 않는다**
#   (수치 DLS 라 한계 밖 해도 수렴할 수 있다). 그대로 set_servo_angle 에 넣으면
#   컨트롤러가 거부하거나 예상 밖 자세로 간다 → 이동 전에 여기서 거른다.
JOINT_LIMITS_DEG = [
    (-360.0, 360.0),   # J1
    (-118.0, 120.0),   # J2
    (-360.0, 360.0),   # J3
    ( -11.0, 225.0),   # J4
    (-360.0, 360.0),   # J5
    ( -97.0, 180.0),   # J6
    (-360.0, 360.0),   # J7
]


def connect(ip: str = ROBOT_IP, *, require_no_error: bool = True) -> XArmInterface:
    """
    로봇 연결 + 에러 상태 확인.

    ★ enable_motion() 은 부르지 않는다 — 그 안의 set_state(0) 이 컨트롤러를
      **이전 명령 위치로 resume** 시켜 로봇이 실제로 움직인다.
      모션이 필요한 스크립트만 직접 호출한다.
    """
    robot = XArmInterface(ip)
    if not robot.arm.connected:
        raise SystemExit(f"✘ 연결 실패: {ip} — 전원·네트워크 확인 (ping {ip})")

    err, warn = robot.arm.error_code, robot.arm.warn_code
    print(f"연결됨  {ip}  fw={robot.arm.version_number}  "
          f"state={robot.arm.state}  mode={robot.arm.mode}  err={err}  warn={warn}")
    if err and require_no_error:
        robot.disconnect()
        raise SystemExit(
            f"✘ error_code={err} — 움직이기 전에 해소해야 한다.\n"
            f"   python scripts/robot/recover.py   또는 http://{ip}:18333"
        )
    return robot


def describe(robot: XArmInterface) -> None:
    """현재 관절각·TCP 출력 (읽기 전용)."""
    q = robot.get_joint_angles(is_radian=True)
    p = robot.get_pose(is_radian=True)
    print(f"  joint(deg) [{', '.join(f'{a:7.2f}' for a in np.degrees(q))}]")
    print(f"  TCP  x={p[0]:8.1f}  y={p[1]:8.1f}  z={p[2]:8.1f} mm   "
          f"rpy=({p[3]:+.3f}, {p[4]:+.3f}, {p[5]:+.3f}) rad")


def sdk_ik(robot, pose6d_rad, *, seed=None):
    """
    목표 pose6d(mm, rad) → 관절각(rad,7).  **컨트롤러 IK 를 쓴다.**

    ★ utils.robot 의 해석 IK(xarm7_kinematics)를 실물 모션에 쓰면 안 된다.
      그 모델은 sim USD(xarm7_spider/v2.usd)에서 추출한 것이고 이 실물과
      플랜지 위치가 z 약 291mm 어긋난다(측정값). 컨트롤러 pose 에 해석 IK 를
      먹이면 의도와 전혀 다른 곳으로 간다 — 실측: dz=+20mm 요청이
      (-140, -3, +277)mm 로 나왔다.  docs/robot_control.md §함정 참고.
    """
    code, q = robot.arm.get_inverse_kinematics(
        list(np.asarray(pose6d_rad, float)[:6]),
        input_is_radian=True, return_is_radian=True)
    if code != 0 or q is None:
        raise RuntimeError(f"컨트롤러 IK 실패 (code={code}) — 도달 불가 자세일 수 있다")
    return np.asarray(q[:7], dtype=float)


def sdk_fk(robot, q):
    """관절각(rad,7) → pose6d(mm, rad). 컨트롤러 FK — 로봇은 움직이지 않는다."""
    code, p = robot.arm.get_forward_kinematics(
        list(np.asarray(q, float)[:7]), input_is_radian=True, return_is_radian=True)
    if code != 0 or p is None:
        raise RuntimeError(f"컨트롤러 FK 실패 (code={code})")
    return np.asarray(p[:6], dtype=float)


def verify_target(robot, q_target, pose_want, *, tol_mm: float = 1.0) -> bool:
    """IK 해가 정말 목표로 가는지 컨트롤러 FK 로 되짚는다 (움직이지 않음)."""
    got = sdk_fk(robot, q_target)
    err = float(np.linalg.norm(got[:3] - np.asarray(pose_want, float)[:3]))
    print(f"  IK 검산(컨트롤러 FK): 위치 오차 {err:.3f} mm"
          + ("   ✘ 목표와 다르다 — 중단" if err > tol_mm else "   ✓"))
    return err <= tol_mm



# 작은 직교 이동인데 관절이 크게 튀면 IK 가 **다른 해 분기**를 고른 것이다.
# 7축은 여유자유도가 있어 같은 TCP 에 무한히 많은 자세가 대응한다 —
# 그대로 set_servo_angle 하면 팔이 크게 휘둘린다. 그래서 직교 이동은
# set_position(컨트롤러가 연속성을 유지) 으로 보내고, 이 값은 경고용으로만 쓴다.
JOINT_JUMP_WARN_DEG = 30.0


def joint_jump(q_from, q_to):
    """관절별 변화량(deg) 최대값."""
    return float(np.max(np.abs(np.degrees(np.asarray(q_to, float)[:7]
                                          - np.asarray(q_from, float)[:7]))))


def move_cartesian(robot, pose6d_rad, *, speed_mm_s: float, wait: bool = True) -> int:
    """
    직교 공간 이동. **컨트롤러가 IK 와 경로 연속성을 처리한다.**

    set_servo_angle(관절 목표) 대신 이걸 쓰는 이유: 위 JOINT_JUMP_WARN_DEG 주석 참고.
    x,y,z 는 mm, rpy 는 rad (README §단위 — set_position 은 mm 를 받는다).
    """
    p6 = np.asarray(pose6d_rad, float)[:6]
    robot.enable_motion()
    return robot.arm.set_position(
        x=float(p6[0]), y=float(p6[1]), z=float(p6[2]),
        roll=float(p6[3]), pitch=float(p6[4]), yaw=float(p6[5]),
        speed=speed_mm_s, is_radian=True, wait=wait)


def precheck(q_target: np.ndarray, *, label: str = "목표 자세") -> bool:
    """
    이동 **전** 자세 검사. 통과해야 움직인다.

      1. 관절 한계 — xArm7 범위 밖이면 컨트롤러가 거부한다
      2. self-collision — 링크끼리 겹치는 자세
      3. 특이점 근접 — sigma_min 이 작으면 그 방향 관절속도가 발산

    ⚠ 이건 **자세** 검사일 뿐 **경로** 검사가 아니다. 현재 자세에서 목표까지
      관절 직선 보간이 무엇을 쓸고 가는지는 `move_joint_safe()` 가 본다.
      로봇을 움직이는 스크립트는 그쪽을 쓸 것.

    ※ 2026-09-21 정정 — 옛 주석은 2·3 이 "sim USD 기반이라 플랜지가 z 약
      291mm 어긋나 참고용" 이라고 했는데, 그 뒤 `scripts/robot/calib_dh.py` 로
      DH 를 실물 교정했다(`config/calibration/xarm7_dh.yaml`). 현장 실측
      대조에서 해석 FK 와 컨트롤러 FK 차이는 **1.2mm** 였다 — 모델을 믿어도
      된다. 환경 충돌도 이제 검사한다(`collision_gate`, 실측 셀 레이아웃).
    """
    q = np.asarray(q_target, dtype=float)[:7]
    ok = True

    print(f"\n── {label} 검사 ──")
    print(f"  joint(deg) [{', '.join(f'{a:7.2f}' for a in np.degrees(q))}]")

    deg = np.degrees(q)
    over = [(i, deg[i]) for i in range(7)
            if not (JOINT_LIMITS_DEG[i][0] <= deg[i] <= JOINT_LIMITS_DEG[i][1])]
    if over:
        print("  ✘ 관절 한계 초과: " + ", ".join(
            f"J{i+1}={d:.1f}° (허용 {JOINT_LIMITS_DEG[i][0]:.0f}~{JOINT_LIMITS_DEG[i][1]:.0f})"
            for i, d in over))
        ok = False
    else:
        print("  ✓ 관절 한계 이내")

    # 자세 안전성은 **단일 게이트**로 본다 (링크 메시 + 실측 셀 SDF + 특이점).
    # 옛 캡슐 근사(capsules_from_joints)는 오탐·미탐이 모두 있어 참고도 못 됐다.
    try:
        cm = collision_gate(required=False)
    except Exception:                                    # noqa: BLE001
        cm = None
    if cm is None:
        print("  ⚠ 충돌 게이트 없음 — 자가/환경 충돌 **미검사**")
    else:
        s, e, who = cm.clearance(q)
        safe, why = cm.is_pose_safe(q)
        print(f"  {'✓' if safe else '✘'} 자세 충돌검사: 자가 {s*1000:.0f}mm · "
              f"환경 {e*1000:.0f}mm ({who})" + ("" if safe else f"  → {why}"))
        if not safe:
            ok = False

    s = kin.sigma_min(q)
    if s < SIGMA_MIN_WARN:
        print(f"  ⚠ 특이점 근접 (sigma_min={s:.4f} < {SIGMA_MIN_WARN}) — 관절속도 발산 주의")
    else:
        print(f"  ✓ 특이점 여유 (sigma_min={s:.4f})")

    return ok


def collision_gate(*, required: bool = True):
    """단일 충돌 게이트 (sim·real 공용 `utils.collision.collision_model`).

    `required=True` 면 게이트를 못 얻었을 때 **None 이 아니라 예외**다 —
    "게이트가 없으면 무검사로 움직인다" 는 조용한 폴백이 곧 파손이다.
    (활성 레이아웃은 `utils/collision/data/ACTIVE_LAYOUT.txt` 가 가리킨다.)
    """
    from utils.collision import collision_model as _cmod
    cm = _cmod.get_default()
    if cm is None and required:
        raise SystemExit(
            "✘ 충돌 게이트를 못 열었다 — 무검사 이동은 하지 않는다.\n"
            "   캐시 생성: scripts/sim/export_link_meshes.py, "
            "scripts/sim/export_env_mesh.py")
    return cm


def plan_safe_joint_path(q0, q1, cm, *, log=print):
    """q0 → q1 의 **충돌 없는** 관절 경유점 목록. 불가능하면 None.

    직선(관절 보간)이 통과하면 [q1] 하나, 막히면 우회를 계획한다.
    `artec_multipass_scan_session._move_robot_to_q` 와 같은 구조 — 실물에서
    검증된 경로를 스크립트도 그대로 쓴다.
    """
    q0 = np.asarray(q0, float)[:7]
    q1 = np.asarray(q1, float)[:7]

    ok, why, n = cm.is_path_safe(q0, q1)
    if ok:
        log(f"  ✓ 직선 경로 안전 (검사 {n}회)")
        return [q1]

    if why.startswith("start"):
        log(f"  ✘ **현재 자세**가 이미 충돌/특이점 — {why}")
        log("     로봇을 수동으로 살짝 빼낸 뒤 다시 시도할 것 "
            "(웹 UI 수동 모드: http://192.168.1.210:18333)")
        return None
    if why.startswith("goal"):
        log(f"  ✘ **목표 자세**가 충돌/특이점 — {why}")
        return None

    log(f"  직선 막힘({why}) — 우회 경로 계획 중 …")
    from utils.control.joint_path_planner import plan_joint_path
    path = plan_joint_path(
        q0, q1, lambda a, b: cm.is_path_safe(a, b)[:2],
        lower=kin.JOINT_LOWER, upper=kin.JOINT_UPPER,
        step=0.3, max_iter=600, shortcut_iters=60,
        log=lambda m: log(f"  [plan] {m}"))
    if path is None:
        log("  ✘ 우회 경로 없음 — 이동하지 않는다")
        return None
    way = [np.asarray(w, float)[:7] for w in path[1:]]
    log(f"  ✓ 우회 경로 {len(way)}개 경유점")
    return way


def move_joint_safe(robot, q_target, *, speed_deg_s: float,
                    label: str = "목표 자세", cm=None) -> bool:
    """충돌 게이트를 거쳐 관절 이동. 경로가 없으면 **움직이지 않고** False.

    ★ `robot.go_home()`/`set_servo_angle()` 직접 호출과의 차이가 이것이다 —
      그쪽은 현재 자세에서 목표까지 관절을 **직선 보간**하므로, 그 사이에
      턴테이블·테이블이 있으면 그냥 쓸고 지나간다 (2026-09-21 현장 사고).
    """
    q1 = np.asarray(q_target, float)[:7]
    cm = cm if cm is not None else collision_gate()
    q0 = np.asarray(robot.get_joint_angles(is_radian=True), float)[:7]

    print(f"\n── {label} 경로 검사 ──")
    s, e, who = cm.clearance(q0)
    print(f"  현재 여유: 자가 {s*1000:.0f}mm · 환경 {e*1000:.0f}mm ({who})")
    way = plan_safe_joint_path(q0, q1, cm)
    if way is None:
        return False

    robot.enable_motion()
    for i, w in enumerate(way, 1):
        if len(way) > 1:
            print(f"  → 경유점 {i}/{len(way)}")
        code = robot.arm.set_servo_angle(
            angle=w.tolist(), speed=float(speed_deg_s), is_radian=True, wait=True)
        if code:
            print(f"  ✘ set_servo_angle 실패 (code={code})")
            return False
    return True


def confirm(msg: str = "이동") -> bool:
    """
    모션 직전 확인. Enter 가 아니라 'y' 를 **입력**하게 한다 —
    Enter 는 실수로 눌리기 쉽다.
    """
    print(f"\n⚠ {msg}합니다. 경로에 사람·장애물이 없는지, 비상정지가 손에 닿는지 확인하세요.")
    return input("   진행하려면 y 입력: ").strip().lower() == "y"
