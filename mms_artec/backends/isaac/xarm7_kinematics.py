"""
xarm7_kinematics — utils.robot.xarm7_kinematics 로 이동(real/sim/충돌검사 공용).

하위호환 re-export shim. 신규 코드는 `from utils.robot import xarm7_kinematics` 사용.
"""
from utils.robot.xarm7_kinematics import *          # noqa: F401,F403
from utils.robot.xarm7_kinematics import (          # 명시 re-export
    JOINT_LOWER, JOINT_UPPER, fk_T, fk_pose6d, fk_link_origins,
    pose6d_to_T_mm, pose6d_to_T_m, euler_xyz_to_R, R_to_euler_xyz,
)
