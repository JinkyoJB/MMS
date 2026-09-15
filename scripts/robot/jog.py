#!/usr/bin/env python
# scripts/robot/jog.py
#
# 현재 TCP 기준 **상대 이동**. ★ 로봇이 움직인다.
#
#   python scripts/robot/jog.py --dz 20              # +Z 로 20mm
#   python scripts/robot/jog.py --dx -10 --dy 5
#   python scripts/robot/jog.py --dyaw 15            # yaw +15°
#   python scripts/robot/jog.py --dz 20 --dry-run    # 계산·검증만, 안 움직임
#
# 좌표계는 B(로봇 베이스). 회전 인자는 **deg** 로 받는다.
#
# ★ XArmInterface.move_relative() 를 쓰지 않는다 — 그건 컨트롤러 pose 에
#   해석 IK(sim USD 모델)를 먹여서 실물에서 엉뚱한 곳으로 간다.
#   docs/robot_control.md §함정 참고.

import argparse
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from _common import (ROBOT_IP, connect, describe, precheck, confirm,   # noqa: E402
                     sdk_ik, verify_target, move_cartesian, joint_jump,
                     JOINT_JUMP_WARN_DEG)


def main() -> int:
    ap = argparse.ArgumentParser(description="xArm7 상대 이동 (B 프레임)")
    ap.add_argument("--ip", default=ROBOT_IP)
    for a in ("dx", "dy", "dz"):
        ap.add_argument(f"--{a}", type=float, default=0.0, help="mm")
    for a in ("droll", "dpitch", "dyaw"):
        ap.add_argument(f"--{a}", type=float, default=0.0, help="deg")
    ap.add_argument("--speed", type=float, default=20.0, help="mm/s (기본 20 — 느리게)")
    ap.add_argument("--dry-run", action="store_true", help="검증만 하고 움직이지 않는다")
    args = ap.parse_args()

    delta = np.array([args.dx, args.dy, args.dz,
                      *np.radians([args.droll, args.dpitch, args.dyaw])])
    if not delta.any():
        ap.error("이동량이 전부 0 이다 — --dx/--dy/--dz/--droll/--dpitch/--dyaw 중 하나는 지정")

    robot = connect(args.ip)
    try:
        print("\n현재:")
        describe(robot)

        target = robot.get_pose(is_radian=True) + delta
        print(f"\n목표 TCP  x={target[0]:.1f}  y={target[1]:.1f}  z={target[2]:.1f} mm   "
              f"rpy=({target[3]:+.3f}, {target[4]:+.3f}, {target[5]:+.3f}) rad")

        q0 = robot.get_joint_angles(is_radian=True)
        try:
            q_target = sdk_ik(robot, target, seed=q0)
        except RuntimeError as e:
            print(f"\n✘ {e}")
            return 1

        if not verify_target(robot, q_target, target):
            return 1
        jump = joint_jump(q0, q_target)
        print(f"  IK 해의 관절 변화 최대 {jump:.1f}°"
              + ("   ⚠ IK 가 다른 해 분기를 골랐다 — 목표가 현재 자세에서 멀다는 신호."
                 if jump > JOINT_JUMP_WARN_DEG else ""))

        if not precheck(q_target):
            print("\n✘ 사전검사 실패 — 이동하지 않는다")
            return 1

        if args.dry_run:
            print("\n--dry-run : 여기까지. 움직이지 않는다.")
            return 0

        if not confirm(f"dx={args.dx:+.1f} dy={args.dy:+.1f} dz={args.dz:+.1f} mm 이동"):
            print("취소")
            return 0

        code = move_cartesian(robot, target, speed_mm_s=args.speed)
        print()
        print(f"set_position code={code}")
        print("결과:")
        describe(robot)
        return 0
    finally:
        robot.disconnect()


if __name__ == "__main__":
    raise SystemExit(main())
