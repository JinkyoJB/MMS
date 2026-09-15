#!/usr/bin/env python
# scripts/robot/recover.py
#
# 에러 클리어 + 모션 활성화 준비.
#
#   python scripts/robot/recover.py
#
# ⚠ clear_error 후 set_state(0) 은 컨트롤러를 **이전 명령 위치로 resume** 시킨다 —
#   즉 로봇이 곧바로 움직일 수 있다. 그래서 확인을 받는다.

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from _common import ROBOT_IP, connect, describe, confirm  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser(description="xArm7 에러 클리어")
    ap.add_argument("--ip", default=ROBOT_IP)
    ap.add_argument("--enable", action="store_true",
                    help="클리어 후 motion_enable 까지 수행 (로봇이 움직일 수 있다)")
    args = ap.parse_args()

    robot = connect(args.ip, require_no_error=False)
    try:
        arm = robot.arm
        print(f"\n현재  err={arm.error_code}  warn={arm.warn_code}  state={arm.state}")

        if arm.error_code or arm.warn_code:
            arm.clean_error()
            arm.clean_warn()
            print(f"클리어 후  err={arm.error_code}  warn={arm.warn_code}")
            if arm.error_code:
                print("✘ 에러가 남아 있다 — 물리적 원인일 수 있다 "
                      f"(충돌/한계 초과). http://{args.ip}:18333 에서 확인.")
                return 1
        else:
            print("클리어할 에러 없음")

        if args.enable:
            if not confirm("motion_enable — 컨트롤러가 이전 명령 위치로 resume 되어 로봇이 움직일 수 있습니다. 활성화"):
                print("취소")
                return 0
            robot.enable_motion()
            print(f"motion_enable 완료  state={arm.state}  mode={arm.mode}")

        print()
        describe(robot)
        return 0
    finally:
        robot.disconnect()


if __name__ == "__main__":
    raise SystemExit(main())
