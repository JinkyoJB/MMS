#!/usr/bin/env python
# scripts/robot/status.py
#
# 로봇 상태 조회 — **읽기 전용, 절대 움직이지 않는다.**
#
#   python scripts/robot/status.py
#   python scripts/robot/status.py --ip 192.168.1.211

import argparse
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from _common import ROBOT_IP, connect, describe, JOINT_LIMITS_DEG  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from utils.robot import xarm7_kinematics as kin                     # noqa: E402
from utils.collision import capsules_from_joints                    # noqa: E402
from utils.collision.robot_collision import self_collision          # noqa: E402

# xArm 컨트롤러 state 코드 (SDK 문서)
STATE_DESC = {
    0: "READY (모션 가능)",
    1: "SUSPEND",
    2: "STOP",
    3: "PAUSE",
    4: "STOPPED (motion_enable 전 대기 — 정상)",
    5: "SYSTEM RESET",
}


def main() -> int:
    ap = argparse.ArgumentParser(description="xArm7 상태 조회 (읽기 전용)")
    ap.add_argument("--ip", default=ROBOT_IP)
    args = ap.parse_args()

    # require_no_error=False : 에러가 있어도 진단할 수 있어야 한다
    robot = connect(args.ip, require_no_error=False)
    try:
        arm = robot.arm
        print(f"\n== 컨트롤러 ==")
        print(f"  state = {arm.state}  → {STATE_DESC.get(arm.state, '?')}")
        print(f"  mode  = {arm.mode}   (0=position)")
        print(f"  error = {arm.error_code}   warn = {arm.warn_code}")
        if arm.error_code:
            print("  ⚠ 에러 상태 — 움직이기 전에 scripts/robot/recover.py")

        print(f"\n== 현재 자세 ==")
        describe(robot)

        q = robot.get_joint_angles(is_radian=True)
        deg = np.degrees(q)

        print(f"\n== 관절 여유 ==")
        for i in range(7):
            lo, hi = JOINT_LIMITS_DEG[i]
            room = min(deg[i] - lo, hi - deg[i])
            mark = "⚠" if room < 10 else " "
            print(f"  {mark} J{i+1}  {deg[i]:8.2f}°   한계까지 {room:7.2f}°")

        print(f"\n== 특이점 / 충돌 ==")
        print(f"  sigma_min      = {kin.sigma_min(q):.4f}   (작을수록 특이점 근접)")
        print(f"  manipulability = {kin.manipulability(q):.4f}")
        print(f"  condition      = {kin.condition_number(q):.1f}")
        hits = self_collision(capsules_from_joints(q))
        print("  self-collision : " +
              (", ".join(f"{a}↔{b}" for a, b, _ in hits) if hits else "없음"))

        # 자체 해석 FK 가 컨트롤러 값과 맞는지 — 캘리브 전 sanity check
        print(f"\n== FK 검증 (해석 FK vs 컨트롤러) ==")
        p_ctrl = robot.get_pose(is_radian=True)
        p_fk = robot.fk(q)
        d = np.abs(p_fk[:3] - p_ctrl[:3])
        print(f"  위치 차이  dx={d[0]:.2f}  dy={d[1]:.2f}  dz={d[2]:.2f} mm")
        print("  " + ("✓ 일치 (해석 FK 신뢰 가능)" if d.max() < 1.0
                      else "⚠ 불일치 — DH 파라미터/툴 오프셋 확인 필요"))
        return 0
    finally:
        robot.disconnect()


if __name__ == "__main__":
    raise SystemExit(main())
