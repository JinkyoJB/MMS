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
from utils.collision import capsules_from_joints              # noqa: E402
from utils.collision.robot_collision import self_collision    # noqa: E402

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

    ⚠ 2·3 은 **참고용**이다. 둘 다 utils.robot.xarm7_kinematics 의 링크 원점을
      쓰는데, 그 모델은 sim USD 기반이라 이 실물과 플랜지가 z 약 291mm 어긋난다.
      자세가 안전하다는 보장으로 받아들이면 안 된다. 1(관절 한계)만 사양 기반이라
      그대로 신뢰할 수 있다.

    ⚠ 턴테이블·주변 환경과의 충돌은 검사하지 않는다. 그건 CollisionWorld 가
      필요하고 turntable_frame.yaml 이 재캘리브 대기 상태다 (README §알려진 한계).
      → 실제 이동 전에 반드시 눈으로 경로를 확인할 것.
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

    hits = self_collision(capsules_from_joints(q))
    if hits:
        print("  ⚠ (참고) self-collision: " + ", ".join(f"{a}↔{b} ({d*1000:.0f}mm)" for a, b, d in hits))
        # 모델 불일치 때문에 ok 를 내리지 않는다 — 경고만 한다
    else:
        print("  ✓ (참고) self-collision 없음")

    s = kin.sigma_min(q)
    if s < SIGMA_MIN_WARN:
        print(f"  ⚠ 특이점 근접 (sigma_min={s:.4f} < {SIGMA_MIN_WARN}) — 관절속도 발산 주의")
    else:
        print(f"  ✓ 특이점 여유 (sigma_min={s:.4f})")

    return ok


def confirm(msg: str = "이동") -> bool:
    """
    모션 직전 확인. Enter 가 아니라 'y' 를 **입력**하게 한다 —
    Enter 는 실수로 눌리기 쉽다.
    """
    print(f"\n⚠ {msg}합니다. 경로에 사람·장애물이 없는지, 비상정지가 손에 닿는지 확인하세요.")
    return input("   진행하려면 y 입력: ").strip().lower() == "y"
