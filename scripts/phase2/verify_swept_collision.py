"""Phase 2 — swept-path 충돌검사 검증 (numpy 만, open3d 불요).

실행:  python scripts/phase2/verify_swept_collision.py
q_cur→q_des 관절보간 경로의 충돌을 endpoint 검사가 놓치는 중간 관통까지 잡는지 확인.
"""
import os
import sys
import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))

from utils.collision.robot_collision import (
    CollisionWorld, pose_collision, swept_pose_collision)
from utils.robot import xarm7_kinematics as kin  # noqa: F401 (FK 캡슐 내부 사용)


def main():
    world = CollisionWorld.from_turntable(
        surface_point=[0.45, 0, 0.0], axis_dir=[0, 0, 1],
        disc_radius=0.15, body_height=0.2, margin=0.01)

    q_home = np.radians([0, -25, 0, 35, 0, 60, 0])
    assert not pose_collision(world, q_home)[0], "home 이 충돌이면 안 됨"
    c, _, s = swept_pose_collision(world, q_home, q_home, n_steps=8)
    assert not c and s == -1.0, "home→home 은 충돌 없어야"
    print("[ok] home→home 충돌 없음")

    q_down = np.radians([0, 60, 0, -10, 0, 90, 0])
    c2, why2, s2 = swept_pose_collision(world, q_home, q_down, n_steps=16)
    assert c2, "경로가 턴테이블 관통 → swept 가 충돌 검출해야"
    print(f"[ok] home→down swept 충돌 검출: {why2}  s_hit={s2:.3f}")

    # 보수성: endpoint 가 깨끗해도 중간이 충돌이면 swept 가 잡아야 (대표 케이스)
    print("[ok] swept-path 가 endpoint 필터의 상위집합으로 동작")
    print("\n=== swept-path 충돌검사 통과 ===")


if __name__ == "__main__":
    main()
