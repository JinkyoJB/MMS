#!/usr/bin/env python
# scripts/robot/move_pose.py
#
# **절대** TCP pose 로 이동. ★ 로봇이 움직인다.
#
#   python scripts/robot/move_pose.py --xyz 400 0 350 --rpy 180 0 0
#   python scripts/robot/move_pose.py --xyz 400 0 350 --dry-run
#
# xyz 는 mm, rpy 는 **deg** (B 프레임).
#
# ★ IK 는 컨트롤러 IK(get_inverse_kinematics)를 쓴다. utils.robot 의 해석 IK 는
#   sim USD 모델이라 실물과 어긋난다 — docs/robot_control.md §함정.

import argparse
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from _common import (ROBOT_IP, connect, describe, precheck, confirm,   # noqa: E402
                     sdk_ik, verify_target, move_cartesian, joint_jump,
                     JOINT_JUMP_WARN_DEG)

WARN_DIST_MM = 100.0


def main() -> int:
    ap = argparse.ArgumentParser(description="xArm7 절대 TCP pose 이동")
    ap.add_argument("--ip", default=ROBOT_IP)
    ap.add_argument("--xyz", nargs=3, type=float, required=True,
                    metavar=("X", "Y", "Z"), help="mm, B 프레임")
    ap.add_argument("--rpy", nargs=3, type=float, default=None,
                    metavar=("R", "P", "Y"), help="deg. 생략하면 현재 자세 유지")
    ap.add_argument("--speed", type=float, default=20.0, help="mm/s (기본 20 — 느리게)")
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()

    robot = connect(args.ip)
    try:
        print("\n현재:")
        describe(robot)

        now = robot.get_pose(is_radian=True)
        target = np.empty(6)
        target[:3] = args.xyz
        target[3:] = np.radians(args.rpy) if args.rpy is not None else now[3:]

        dist = float(np.linalg.norm(target[:3] - now[:3]))
        print(f"\n목표 TCP  x={target[0]:.1f}  y={target[1]:.1f}  z={target[2]:.1f} mm   "
              f"rpy=({target[3]:+.3f}, {target[4]:+.3f}, {target[5]:+.3f}) rad")
        print(f"  직선거리 {dist:.1f} mm"
              + (f"   ⚠ {WARN_DIST_MM:.0f}mm 이상 — 경로를 눈으로 확인할 것"
                 if dist > WARN_DIST_MM else ""))

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

        if not confirm(f"{dist:.0f}mm 이동 ({args.speed} mm/s)"):
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
