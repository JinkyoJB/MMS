"""
IsaacXArm — xArm7 robot 백엔드 (Isaac Sim).

`utils.robot.xarm_interface.XArmInterface` 와 덕타이핑 호환:
  get_joint_angles / get_pose / get_ee_pose_mat / fk / ik / move_relative /
  go_home / enable_motion / disconnect  + raw `.arm` shim.

상태(get_*)는 sim 아티큘레이션에서 읽고, fk/ik 는 해석적 xArm7 운동학
(xarm7_kinematics)으로 계산한다. 모션은 ik → 관절각 → sim 아티큘레이션 관절공간
구동(IsaacWorld.drive_to_joints)으로 실행한다.

좌표/단위 규약은 실물과 동일:
  pose6d = [x,y,z (mm), roll,pitch,yaw (rad)]  (B 기준 플랜지)
  T_EB (get_ee_pose_mat) : 병진 m
"""

from __future__ import annotations

import numpy as np

from utils.robot import xarm7_kinematics as kin


# 센서별 home joint (deg).
#   phoxi: XArmInterface.HOME_JOINTS_DEG 와 동일.
#   artec: ★ sim 충돌회피 홈. 실물 기본값 [0,-18.4,0,70.6,0,60,-45] 은 스캐너가
#     턴테이블에 박혀(bbox 3축 겹침) 시작부터 충돌한다. sim ground-truth 로 스캐너를
#     턴테이블 중심 위(약 0.29m)에서 내려다보는 자세를 IK 로 산출 → 모든 링크가
#     턴테이블 위(min z≈0.96 ≫ 턴테이블 top 0.71)에 있어 충돌 없음. 여기서 calibration
#     시작(이후 fixture 조준은 작은 보정). 산출: scripts/sim 의 홈 탐색(아래 주석 참고).
HOME_JOINTS_DEG = {
    "phoxi": [0.0, -30.0, 0.0, 60.0, 0.0, 90.0, 0.0],
    "artec": [38.92, -48.70, -65.29, 21.22, 21.46, 72.70, -96.58],
}


class _ArmShim:
    """xArm SDK(XArmAPI) 부분집합 shim — orchestration 이 robot.arm.* 로 직접 호출."""

    def __init__(self, xarm: "IsaacXArm"):
        self._x = xarm

    # ── 상태 조회 ───────────────────────────────────────────────────────────
    def get_servo_angle(self, is_radian: bool = True, **_):
        q = self._x._sim_joints()
        return 0, (q.tolist() if is_radian else np.degrees(q).tolist())

    def get_position(self, is_radian: bool = True, **_):
        pose = self._x.get_pose(is_radian=is_radian)
        return 0, pose.tolist()

    def get_forward_kinematics(self, angles, input_is_radian: bool = True,
                               return_is_radian: bool = True, **_):
        q = np.asarray(angles, dtype=float)[:7]
        if not input_is_radian:
            q = np.radians(q)
        pose = kin.fk_pose6d(q)
        if not return_is_radian:
            pose = pose.copy(); pose[3:] = np.degrees(pose[3:])
        return 0, pose.tolist()

    def get_inverse_kinematics(self, pose, input_is_radian: bool = True,
                               return_is_radian: bool = True, **_):
        p = np.asarray(pose, dtype=float)[:6].copy()
        if not input_is_radian:
            p[3:] = np.radians(p[3:])
        q, ok = kin.ik(p, seed=self._x._sim_joints())
        if not ok:
            return 1, [0.0] * 7              # code!=0 → 미도달
        return 0, (q.tolist() if return_is_radian else np.degrees(q).tolist())

    # ── 모션 ────────────────────────────────────────────────────────────────
    def set_servo_angle(self, angle, speed=None, is_radian: bool = False,
                        wait: bool = True, **_):
        q = np.asarray(angle, dtype=float)[:7]
        if not is_radian:
            q = np.radians(q)
        self._x._move(q)
        return 0

    def set_position(self, x=0.0, y=0.0, z=0.0, roll=0.0, pitch=0.0, yaw=0.0,
                     is_radian: bool = True, speed=None, wait: bool = True, **_):
        rpy = np.array([roll, pitch, yaw], dtype=float)
        if not is_radian:
            rpy = np.radians(rpy)
        pose6d = np.array([x, y, z, rpy[0], rpy[1], rpy[2]], dtype=float)
        q, ok = kin.ik(pose6d, seed=self._x._sim_joints())
        if not ok:
            return 1                          # IK 실패 → code!=0
        self._x._move(q)
        return 0

    # ── 컨트롤러 상태 (sim 에선 no-op) ───────────────────────────────────────
    def motion_enable(self, *a, **k):
        return 0

    def set_mode(self, *a, **k):
        return 0

    def set_state(self, *a, **k):
        return 0

    def disconnect(self, *a, **k):
        return 0


class IsaacXArm:
    """XArmInterface 호환 robot 백엔드 (Isaac Sim)."""

    HOME_JOINTS_DEG = HOME_JOINTS_DEG

    def __init__(self, world):
        self._world = world
        self.arm = _ArmShim(self)
        # 명령(commanded) 관절각을 robot 의 논리 상태로 추적한다.
        # sim 아티큘레이션은 충돌/접촉으로 명령값과 수 도 어긋날 수 있어(물리 정착),
        # 제어 로직이 보는 robot 은 '이상적 키네마틱 로봇'으로 두고(get_joint_angles=
        # 명령값), sim 은 시각화로 따라가게 한다. fk/ik/get_pose 가 자기일관됨.
        self._q = np.asarray(world.get_joint_positions()[:7], dtype=float).copy()

    # ── 내부 ────────────────────────────────────────────────────────────────
    def _sim_joints(self) -> np.ndarray:
        """현재(명령) 관절각 (rad, 7)."""
        return self._q

    def _move(self, q: np.ndarray) -> None:
        """명령 관절각 갱신 + sim 아티큘레이션 구동(시각화)."""
        self._q = np.asarray(q, dtype=float)[:7].copy()
        self._world.drive_to_joints(self._q)

    # ── 상태 조회 ───────────────────────────────────────────────────────────
    def get_joint_angles(self, is_radian: bool = True) -> np.ndarray:
        q = self._sim_joints()
        return q if is_radian else np.degrees(q)

    def get_pose(self, is_radian: bool = True) -> np.ndarray:
        pose = kin.fk_pose6d(self._sim_joints())
        if not is_radian:
            pose = pose.copy(); pose[3:] = np.degrees(pose[3:])
        return pose

    def get_ee_pose_mat(self) -> np.ndarray:
        """T_EB (4x4), 병진 m."""
        return kin.pose6d_to_T_m(self.get_pose(is_radian=True))

    # ── 운동학 ──────────────────────────────────────────────────────────────
    def fk(self, joints: np.ndarray, input_is_radian: bool = True) -> np.ndarray:
        q = np.asarray(joints, dtype=float)[:7]
        if not input_is_radian:
            q = np.radians(q)
        return kin.fk_pose6d(q)

    def ik(self, pose: np.ndarray, input_is_radian: bool = True,
           seed_joints=None) -> np.ndarray:
        p = np.asarray(pose, dtype=float)[:6].copy()
        if not input_is_radian:
            p[3:] = np.radians(p[3:])
        seed = self._sim_joints() if seed_joints is None else np.asarray(seed_joints, float)
        q, ok = kin.ik(p, seed=seed)
        if not ok:
            raise RuntimeError("IK failed (목표 미도달)")
        return q

    # ── 모션 ────────────────────────────────────────────────────────────────
    def enable_motion(self) -> None:
        pass                                  # sim 에선 불필요

    def move_relative(self, dx=0, dy=0, dz=0, d_roll=0, d_pitch=0, d_yaw=0,
                      speed=10, confirm=True) -> bool:
        pose_now = self.get_pose(is_radian=True)
        target = pose_now.copy()
        target[0] += dx; target[1] += dy; target[2] += dz
        target[3] += d_roll; target[4] += d_pitch; target[5] += d_yaw
        try:
            q = self.ik(target, input_is_radian=True)
        except RuntimeError:
            print("[IsaacXArm] move_relative IK 실패 — 도달 불가")
            return False
        if confirm:
            print(f"[IsaacXArm] (sim) move_relative → "
                  f"d=[{dx:+.1f},{dy:+.1f},{dz:+.1f}]mm")
        self._move(q)
        return True

    def go_home(self, sensor: str = "artec", speed: float = 20,
                confirm: bool = True) -> None:
        try:
            home_deg = self.HOME_JOINTS_DEG[sensor]
        except KeyError:
            raise ValueError(f"unknown sensor {sensor!r} — "
                             f"choices: {list(self.HOME_JOINTS_DEG)}")
        print(f"\n[IsaacXArm] (sim) Home joints ({sensor}, deg): {home_deg}")
        if confirm:
            input("Enter 누르면 home 으로 이동...")
        self._move(np.radians(home_deg))
        pose = self.get_pose(is_radian=True)
        print(f"[IsaacXArm] Home 완료  TCP(mm): "
              f"x={pose[0]:.1f} y={pose[1]:.1f} z={pose[2]:.1f}")

    def disconnect(self):
        # 공유 IsaacWorld 는 닫지 않는다 (turntable/scanner 가 공유).
        pass

    # ── 충돌 캡슐 (현재 자세, base 프레임) ───────────────────────────────────
    def collision_capsules(self, link_radii=None):
        """
        현재 자세의 충돌 캡슐 [(name, Capsule), …] (로봇 base 프레임).

        sim 은 USD 링크 prim 의 **실제** world 포즈를 base 로 변환해 쓴다(해석 DH 는
        이 USD 와 불일치하므로 실제 포즈가 정확). utils.collision.CollisionWorld.check
        에 그대로 넣는다. 가상 자세 질의는 그 자세로 이동(look_at 등) 후 호출.
        """
        from utils.collision import capsules_from_origins, DEFAULT_LINK_RADII
        from mms_artec.backends.isaac.isaac_world import ROBOT_PRIM, CAMERA_PRIM
        W = self._world
        bpos, bR = W.base_world_pose()
        def w2b(p):
            return bR.T @ (np.asarray(p, float) - bpos)
        origins = [w2b(W.prim_world_pose(f"{ROBOT_PRIM}/link{i}")[0]) for i in range(1, 8)]
        origins.append(w2b(W.prim_world_pose(CAMERA_PRIM)[0]))      # 스캐너 끝
        radii = DEFAULT_LINK_RADII if link_radii is None else link_radii
        names = ["link1", "link2", "link3", "link4", "link5", "link6", "scanner"]
        return capsules_from_origins(origins, radii, names)
