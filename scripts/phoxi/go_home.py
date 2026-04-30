#!/usr/bin/env python3
# scripts/phoxi/go_home.py
#
# PhoXi 마운트용 home pose 로 이동.
# Joints (deg): [0, -30, 0, 60, 0, 90, 0]   (XArmInterface.HOME_JOINTS_DEG['phoxi'])

import argparse
import sys
from pathlib import Path

_PROJECT_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(_PROJECT_ROOT))

from utils.robot.xarm_interface import XArmInterface

ROBOT_IP = "192.168.1.210"


def main():
    parser = argparse.ArgumentParser(description="xArm7 → PhoXi home pose")
    parser.add_argument("--speed", type=float, default=5.0)
    args = parser.parse_args()

    robot = XArmInterface(ROBOT_IP)
    try:
        robot.go_home(sensor="phoxi", speed=args.speed, confirm=True)
    finally:
        robot.disconnect()


if __name__ == "__main__":
    main()
