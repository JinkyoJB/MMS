#!/usr/bin/env python
# scripts/robot/home.py
#
# 안전 home 자세로 이동. ★ 로봇이 움직인다.
#
#   python scripts/robot/home.py                 # artec (기본)
#   python scripts/robot/home.py --sensor phoxi
#   python scripts/robot/home.py --speed 10
#
# home 은 wrist singularity 를 피한 중립 자세라 IK seed 로도 쓴다.
# 센서마다 EE 마운트가 달라 J7 회전이 다르다 (XArmInterface.HOME_JOINTS_DEG).

import argparse
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from _common import ROBOT_IP, connect, describe, precheck, confirm  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from utils.robot.xarm_interface import XArmInterface               # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser(description="xArm7 home 이동")
    ap.add_argument("--ip", default=ROBOT_IP)
    ap.add_argument("--sensor", default="artec",
                    choices=sorted(XArmInterface.HOME_JOINTS_DEG))
    ap.add_argument("--speed", type=float, default=20.0, help="deg/s")
    args = ap.parse_args()

    home_deg = XArmInterface.HOME_JOINTS_DEG[args.sensor]
    robot = connect(args.ip)
    try:
        print("\n현재:")
        describe(robot)

        if not precheck(np.radians(home_deg), label=f"home ({args.sensor})"):
            print("\n✘ 사전검사 실패 — 이동하지 않는다")
            return 1

        if not confirm(f"home({args.sensor}) 로 {args.speed} deg/s 로 이동"):
            print("취소")
            return 0

        # go_home 은 내부에서 enable_motion() 을 부른다
        robot.go_home(sensor=args.sensor, speed=args.speed, confirm=False)
        print("\n결과:")
        describe(robot)
        return 0
    finally:
        robot.disconnect()


if __name__ == "__main__":
    raise SystemExit(main())
