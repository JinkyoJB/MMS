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
from _common import (ROBOT_IP, collision_gate, confirm, connect,   # noqa: E402
                      describe, move_joint_safe, precheck)

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from utils.robot.xarm_interface import XArmInterface               # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser(description="xArm7 home 이동")
    ap.add_argument("--ip", default=ROBOT_IP)
    ap.add_argument("--sensor", default="artec",
                    choices=sorted(XArmInterface.HOME_JOINTS_DEG))
    ap.add_argument("--speed", type=float, default=20.0, help="deg/s")
    ap.add_argument("--no-collision", action="store_true",
                    help="충돌 게이트 없이 직선 이동 (권장하지 않음)")
    args = ap.parse_args()

    home_deg = XArmInterface.HOME_JOINTS_DEG[args.sensor]
    q_home = np.radians(home_deg)
    robot = connect(args.ip)
    try:
        print("\n현재:")
        describe(robot)

        if not precheck(q_home, label=f"home ({args.sensor})"):
            print("\n✘ 사전검사 실패 — 이동하지 않는다")
            return 1

        # ★ 경로 계획을 **확인 프롬프트보다 먼저** 한다. 사용자가 y 를 누를 때는
        #   로봇이 실제로 어떤 경로로 갈지(직선인지 우회 N점인지) 이미 화면에
        #   나와 있어야 한다 — 눈으로 지켜볼 구간을 알고 손을 올려둘 수 있다.
        cm = None if args.no_collision else collision_gate()
        way = None
        if cm is not None:
            from _common import plan_safe_joint_path
            q0 = np.asarray(robot.get_joint_angles(is_radian=True), float)[:7]
            print(f"\n── home 경로 검사 ──")
            s, e, who = cm.clearance(q0)
            print(f"  현재 여유: 자가 {s*1000:.0f}mm · 환경 {e*1000:.0f}mm ({who})")
            way = plan_safe_joint_path(q0, q_home, cm)
            if way is None:
                print("\n✘ 안전한 경로가 없다 — 이동하지 않는다")
                return 1
        else:
            print("\n⚠ --no-collision: 턴테이블·테이블 충돌을 **검사하지 않는다**")

        n = len(way) if way else 1
        if not confirm(f"home({args.sensor}) 로 {args.speed} deg/s 로 이동"
                       f" (경유점 {n}개)"):
            print("취소")
            return 0

        if way is None:                      # --no-collision 경로
            robot.go_home(sensor=args.sensor, speed=args.speed, confirm=False)
        else:
            robot.enable_motion()
            for i, w in enumerate(way, 1):
                if n > 1:
                    print(f"  → 경유점 {i}/{n}")
                # ★ xArm SDK: is_radian=True 이면 speed/mvacc 도 rad/s·rad/s² 로 해석한다 — deg 값을 그대로 넘기면 π rad/s(180°/s)로 클램프돼 설정과 무관하게 최고속이 된다(2026-09-22 발견).
                code = robot.arm.set_servo_angle(
                    angle=w.tolist(), speed=float(np.radians(args.speed)),
                    mvacc=float(np.radians(args.speed * 4.0)),
                    is_radian=True, wait=True)
                if code:
                    print(f"\n✘ set_servo_angle 실패 (code={code})")
                    return 1
        print("\n결과:")
        describe(robot)
        return 0
    finally:
        robot.disconnect()


if __name__ == "__main__":
    raise SystemExit(main())
