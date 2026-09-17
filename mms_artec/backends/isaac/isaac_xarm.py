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

import os
import numpy as np

from utils.robot import xarm7_kinematics as kin


# 센서별 home joint (deg).
#   phoxi: XArmInterface.HOME_JOINTS_DEG 와 동일.
#   artec: ★ sim 충돌회피 홈. 실물 기본값 [0,-18.4,0,70.6,0,60,-45] 은 스캐너가
#     턴테이블에 박혀(bbox 3축 겹침) 시작부터 충돌한다. sim ground-truth 로 스캐너를
#     턴테이블 중심 위(약 0.29m)에서 내려다보는 자세를 IK 로 산출 → 모든 링크가
#     턴테이블 위(min z≈0.96 ≫ 턴테이블 top 0.71)에 있어 충돌 없음. 여기서 calibration
#     시작(이후 fixture 조준은 작은 보정). 산출: scripts/sim 의 홈 탐색(아래 주석 참고).
#   ★ v3_scene(260811 신규 레이아웃)용으로 재산출(2026-08-12).
#     구값 [38.92,-48.70,-65.29,21.22,21.46,72.70,-96.58] 은 v2 배치(로봇 base
#     (0.538,0,1.407) / 턴테이블 (0.28,0,0.71)) 전용이라, v3(base (0.365,0,1.5) /
#     턴테이블 (0.365,0,0.665))에서는 카메라가 대상에서 **499mm** 떨어져 preview 점이
#     0 이 된다. 신규값은 대상 상단을 고도각 65°·작동거리 250mm 로 내려다보며
#     최저 링크 z=0.912 (상판 0.510 보다 한참 위) 라 충돌이 없다.
#     산출: scripts/sim/find_home_pose.py (USD 실측 base·link7→Camera 기반 IK 탐색)
#   ★ 2026-09-17 — **실물과 같은 값으로 통일.**
#     실물 셀은 v2 배치인데 sim 만 v3 씬을 띄우고 있었다(짓기로 했다가 안 지은
#     레이아웃). 실물 배치를 그대로 재현한 씬을 만들었으므로
#     (`scripts/sim/build_scene_v2_real.py`), home 도 `XArmInterface.HOME_JOINTS_DEG`
#     와 같아야 sim 이 실물을 미러링한다.
#     실측 셀(`v2_real_260917`)에서: 충돌여유 +51.5mm · sigma_min 0.147 ·
#     계획 격자 **12/12** 에 충돌-free 경로 (v3 용으로 뽑았던 값은 11/12).
#
#     ⚠ 구 v3 씬(`v3_scene.usd`)을 돌릴 때는 이 값이 맞지 않는다. 그 씬은
#       턴테이블이 로봇 base 바로 아래라 기하가 전혀 다르다.
#       `MMS_SIM_HOME_DEG="0.8,-14.65,-7.64,21.0,122.43,107.37,-88.46"` 로 덮어쓸 것
#       (그 값도 오늘 충돌 게이트를 제약으로 재산출한 것이다 — 원래 값
#        [-7.65,-75.61,...] 은 손목이 디스크 27mm 위를 스쳐 **모든 이동이 거부**됐다).
_HOME_ENV = os.environ.get("MMS_SIM_HOME_DEG", "").strip()
HOME_JOINTS_DEG = {
    "phoxi": [0.0, -30.0, 0.0, 60.0, 0.0, 90.0, 0.0],
    "artec": ([float(v) for v in _HOME_ENV.split(",")] if _HOME_ENV
              else [0.0, -18.4, 0.0, 70.6, 0.0, 60.0, -45.0]),
}


#: **USD 아티큘레이션 ↔ 해석 FK 영점 오프셋** (deg, 관절 1~7).
#
#  ★ 2026-09-17 — 이게 없으면 **sim 로봇이 명령과 다른 자세로 간다.**
#    2026-09-15 에 `xarm7_kinematics` 를 **실물 컨트롤러 기준**으로 재교정했는데
#    (J2 영점 29.4° · J7 영점 9.77°), v2.usd 아티큘레이션은 그 꺾인 자세가 관절
#    프레임에 녹아든 채 그대로다. 그래서 같은 q 를 줘도
#        Isaac link7  vs  kin.fk_T(q)  →  128.7mm / 29.0° 차이
#    가 난다. `IsaacXArm` 은 제어 로직에 **해석 FK 를 보여주고**(get_pose 가
#    kin.fk_pose6d) Isaac 은 시각화로만 쓰는데, **카메라는 Isaac 아티큘레이션에
#    붙어 있다.** 결국 플래너는 카메라를 정확히 조준했다고 믿지만 실제 카메라는
#    29° 딴 데를 본다 — 실측: 대상물이 세로 화각 28.3°(한계 14.3°)로 밀려나
#    `raw=110592 crop=0`, 즉 **캡처 0점**.
#
#  구동할 때 `q + Δ` 를 Isaac 에 주고, 읽을 때 `−Δ` 한다. 제어 로직이 보는 관절각은
#  그대로 **컨트롤러/해석 FK 공간**에 남는다(실물과 같은 값).
#
#  산출: Isaac 에서 12자세를 찍어 `P_isaac(q) == kin_FK(q − Δ)` 로 최소자승 피팅.
#    잔차 위치 RMS **260.3mm → 4.6mm** (최대 8.1mm). 지배항은 J2·J7 로 재교정
#    커밋의 기록(29.4° / 9.77°)과 일치한다. 남은 4.6mm 는 USD CAD 와 보정 DH 의
#    링크 치수 차이가 관절각으로 흡수된 것 — 카메라 발자국(~108mm)에 비해 작다.
#  끄려면 `MMS_SIM_JOINT_OFFSET=0` (A/B 대조용).
JOINT_ZERO_OFFSET_DEG = [0.0931, 29.4555, -0.3383, -0.3181, -1.0364, -0.7730, -9.7748]
JOINT_ZERO_OFFSET = (
    np.radians(JOINT_ZERO_OFFSET_DEG)
    if os.environ.get("MMS_SIM_JOINT_OFFSET", "1") == "1" else np.zeros(7))


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
        # Isaac 관절각 → 컨트롤러 공간 (−Δ)
        self._q = (np.asarray(world.get_joint_positions()[:7], dtype=float)
                   - JOINT_ZERO_OFFSET).copy()

    # ── 내부 ────────────────────────────────────────────────────────────────
    def _sim_joints(self) -> np.ndarray:
        """현재(명령) 관절각 (rad, 7)."""
        return self._q

    def _move(self, q: np.ndarray) -> None:
        """명령 관절각 갱신 + sim 아티큘레이션 구동(시각화)."""
        self._q = np.asarray(q, dtype=float)[:7].copy()
        # 컨트롤러 공간 → Isaac 아티큘레이션 (+Δ). 이래야 **카메라**가 해석 FK 가
        # 말하는 자리에 온다 (카메라는 Isaac link7 의 자식이다).
        self._world.drive_to_joints(self._q + JOINT_ZERO_OFFSET)

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
