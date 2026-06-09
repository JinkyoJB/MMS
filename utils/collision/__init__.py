"""
utils.collision — xArm7 자세별 충돌 쿼리 검사 (real/sim 공용, sensor 무관).

물리 충돌 응답(키네마틱 로봇에선 jitter만 유발) 대신, 자세를 기하로 검사한다.
  CollisionWorld.from_turntable(표면, 축, 반경, …)   ← calibration 결과(base)로 월드
  caps = robot.collision_capsules()                  ← 실제 링크 포즈(sim:USD / real:FK)
  world.check(caps, margin, ignore)  → CollisionResult(collide, min_clearance, contacts)
"""

from utils.collision.geometry import seg_seg_distance, seg_halfspace_min_signed
from utils.collision.robot_collision import (
    Capsule, CollisionResult, CollisionWorld,
    capsules_from_origins, capsules_from_joints, DEFAULT_LINK_RADII,
)

__all__ = [
    "seg_seg_distance", "seg_halfspace_min_signed",
    "Capsule", "CollisionResult", "CollisionWorld",
    "capsules_from_origins", "capsules_from_joints", "DEFAULT_LINK_RADII",
]
