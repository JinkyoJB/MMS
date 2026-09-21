from xarm.wrapper import XArmAPI
import numpy as np
from typing import Optional

from utils.transforms import pose6d_to_mat


class XArmInterface:
    """
    xArm7 로봇 인터페이스.

    Coordinate frames
    -----------------
    - B (Base frame): xArm7 로봇 베이스 = 전역 좌표계 (B ≡ W)
    - E (End-Effector frame): 로봇 플랜지 / TCP 프레임

    Pose convention
    ---------------
    [x(mm), y(mm), z(mm), roll(rad), pitch(rad), yaw(rad)]  — B 기준 TCP pose
    """

    def __init__(self, ip: str = "192.168.1.210"):
        self.arm = XArmAPI(ip)

    def enable_motion(self) -> None:
        """
        모션 제어 활성화. 실제로 로봇을 움직이기 전에만 호출한다.

        주의: set_state(0)은 컨트롤러를 이전 명령 위치로 resume시키므로
        포즈 읽기(get_pose, get_joint_angles)만 할 때는 호출하지 않는다.
        """
        self.arm.motion_enable(True)
        self.arm.set_mode(0)
        self.arm.set_state(0)

    # ------------------------------------------------------------------
    # State queries
    # ------------------------------------------------------------------

    def get_joint_angles(self, is_radian: bool = True) -> np.ndarray:
        """
        현재 joint angles 반환.

        Parameters
        ----------
        is_radian : bool
            True → rad, False → deg

        Returns
        -------
        np.ndarray, shape (7,)
        """
        code, angles = self.arm.get_servo_angle(is_radian=is_radian)
        if code != 0:
            raise RuntimeError(f"get_servo_angle failed (code={code})")
        return np.array(angles[:7])

    def get_pose(self, is_radian: bool = True) -> np.ndarray:
        """
        현재 TCP pose 반환 (B 프레임 기준).

        Parameters
        ----------
        is_radian : bool
            True → roll/pitch/yaw 단위 rad, False → deg

        Returns
        -------
        np.ndarray, shape (6,)  [x(mm), y(mm), z(mm), roll, pitch, yaw]
        """
        code, pose = self.arm.get_position(is_radian=is_radian)
        if code != 0:
            raise RuntimeError(f"get_position failed (code={code})")
        return np.array(pose[:6])

    def get_ee_pose_mat(self) -> np.ndarray:
        """
        현재 EE pose를 (4,4) 동차 변환 행렬로 반환 (B 프레임 기준).

        get_pose()의 [x(mm), y(mm), z(mm), roll, pitch, yaw]를
        T_E_B (4,4) SE3 행렬로 변환한다. Translation 단위는 m.

        Returns
        -------
        T_E_B : np.ndarray, shape (4,4)
        """
        pose6d = self.get_pose(is_radian=True)
        return pose6d_to_mat(pose6d)

    # ------------------------------------------------------------------
    # Kinematics
    # ------------------------------------------------------------------

    def fk(self, joints: np.ndarray, input_is_radian: bool = True) -> np.ndarray:
        """
        Forward Kinematics: joint angles → TCP pose (B 프레임 기준).

        Parameters
        ----------
        joints : np.ndarray, shape (7,)
        input_is_radian : bool

        Returns
        -------
        np.ndarray, shape (6,)  [x(mm), y(mm), z(mm), roll(rad), pitch(rad), yaw(rad)]

        ※ **자체 해석 FK**(xarm7_kinematics) 사용 — 컨트롤러/하드웨어 무관·오프라인 안정.
          SDK FK(get_forward_kinematics) 미사용.
        """
        from utils.robot import xarm7_kinematics as _kin
        q = np.asarray(joints, dtype=float)[:7]
        if not input_is_radian:
            q = np.radians(q)
        return _kin.fk_pose6d(q)

    def ik(
        self,
        pose: np.ndarray,
        input_is_radian: bool = True,
        seed_joints: Optional[np.ndarray] = None,
    ) -> np.ndarray:
        """
        Inverse Kinematics: TCP pose → joint angles.

        Parameters
        ----------
        pose : np.ndarray, shape (6,)
            [x(mm), y(mm), z(mm), roll, pitch, yaw]
        input_is_radian : bool
        seed_joints : np.ndarray or None
            초기 추정 joint angles (rad). None이면 현재 joint 사용.

        Returns
        -------
        np.ndarray, shape (7,)  [rad]

        ※ **자체 해석 IK**(xarm7_kinematics, 수치 DLS) 사용 — 컨트롤러/하드웨어 무관·안정.
          SDK IK(get_inverse_kinematics, 컨트롤러 통신 → 연결 필요·불안정) 미사용.
        """
        from utils.robot import xarm7_kinematics as _kin
        if seed_joints is None:
            seed_joints = self.get_joint_angles(is_radian=True)
        p = np.asarray(pose, dtype=float)[:6].copy()
        if not input_is_radian:
            p[3:] = np.radians(p[3:])
        q, ok = _kin.ik(p, seed=np.asarray(seed_joints, dtype=float))
        if not ok:
            raise RuntimeError("IK failed (analytic 수렴 실패 — 목표 미도달 가능)")
        return q

    # ------------------------------------------------------------------
    # Motion
    # ------------------------------------------------------------------

    def move_relative(
        self,
        dx: float = 0,
        dy: float = 0,
        dz: float = 0,
        d_roll: float = 0,
        d_pitch: float = 0,
        d_yaw: float = 0,
        speed: float = 10,
        confirm: bool = True,
    ) -> bool:
        """
        현재 TCP pose 기준 상대 이동 (B 프레임).

        Parameters
        ----------
        dx, dy, dz          : mm 단위 이동량
        d_roll, d_pitch, d_yaw : rad 단위 자세 변화량
        speed               : deg/s
        confirm             : True → Enter 확인 후 이동

        Returns
        -------
        bool : 성공 여부
        """
        pose_now = self.get_pose(is_radian=True)

        print(f"\n현재 TCP:  x={pose_now[0]:.1f}  y={pose_now[1]:.1f}  z={pose_now[2]:.1f} mm")
        print(f"이동 delta: dx={dx:+.1f}  dy={dy:+.1f}  dz={dz:+.1f} mm")

        target_pose = pose_now.copy()
        target_pose[0] += dx
        target_pose[1] += dy
        target_pose[2] += dz
        target_pose[3] += d_roll
        target_pose[4] += d_pitch
        target_pose[5] += d_yaw

        print(f"목표 TCP:  x={target_pose[0]:.1f}  y={target_pose[1]:.1f}  z={target_pose[2]:.1f} mm")

        try:
            ik_joints_rad = self.ik(target_pose, input_is_radian=True)   # 자체 해석 IK
        except RuntimeError:
            print("IK 실패 (analytic) — 도달 불가능한 위치")
            return False

        if confirm:
            input("\nEnter 누르면 이동...")

        self.enable_motion()
        self.arm.set_servo_angle(angle=ik_joints_rad.tolist(), speed=speed,
                                 is_radian=True, wait=True)

        pose_after = self.get_pose(is_radian=True)
        print(
            f"\n이동 완료  "
            f"dx={pose_after[0]-pose_now[0]:+.1f}  "
            f"dy={pose_after[1]-pose_now[1]:+.1f}  "
            f"dz={pose_after[2]-pose_now[2]:+.1f} mm"
        )
        return True

    # 센서별 home joint 위치 (deg).
    # PhoXi 와 Artec 은 EE 에 다른 마운트로 설치돼 있어 J7 회전이 다르다 —
    # Artec 은 J7 기준 -45° 회전 마운트로 설치돼 있음.
    #: ⚠ sim 과 **같은 값**이어야 한다 — `mms_artec/backends/isaac/isaac_xarm.py`.
    #   둘이 갈라지면 sim 이 실물을 미러링하지 못한다.
    #
    #   artec 2026-09-21 변경 (J2 −18.4→−6.98 · J6 60→35.53):
    #     로봇 시스템을 옮긴(9/19) 뒤 실물에서 잡은 자세다. 옛 home 은 턴테이블을
    #     **319mm** 에서 봐 Spider 작동거리(170~330mm) 상한에 붙어 있었다 — 초점이
    #     흐려 캘리브 보드 인식이 나빴다. 새 값은 **260mm**(보드면까지 210mm)로
    #     최적대역(200~250mm) 안이다. 실측: 재투영 1.23px · verts 20,200.
    #     충돌 여유도 더 낫다 (환경 90 → 136mm). sigma_min 0.147 → 0.120 으로
    #     줄지만 임계 0.05 의 2.4배라 여유가 있다.
    #
    #   home 은 IK seed 이자 자세 생성의 경로 검사 기준점이므로, 바꾼 뒤에는
    #   캘리브 자세를 다시 만들 것 (`gen_calib_poses.py --from-view --write`).
    HOME_JOINTS_DEG = {
        "phoxi": [0.0, -30.0,  0.0, 60.0, 0.0, 90.0,   0.0],
        "artec": [0.0,  -6.98, 0.0, 70.6, 0.0, 35.53, -45.0],
    }

    def go_home(
        self,
        sensor: str = "artec",
        speed: float = 20,
        confirm: bool = True,
    ) -> None:
        """
        안전한 home joint 위치로 이동.

        - Wrist singularity 회피, IK solver seed 로 중립적인 자세.
        - `sensor` 별로 J7 마운트 회전이 다르다 (`HOME_JOINTS_DEG` 참고).
        """
        try:
            home_deg = self.HOME_JOINTS_DEG[sensor]
        except KeyError:
            raise ValueError(
                f"unknown sensor {sensor!r} — "
                f"choices: {list(self.HOME_JOINTS_DEG)}"
            )

        print(f"\nHome joints ({sensor}, deg): {home_deg}")
        if confirm:
            input("Enter 누르면 home으로 이동...")

        self.enable_motion()
        self.arm.set_servo_angle(angle=home_deg, speed=speed, is_radian=False, wait=True)

        joints = self.get_joint_angles(is_radian=False)
        pose = self.get_pose(is_radian=True)
        print(f"Home 완료")
        print(f"  joints (deg): {np.round(joints, 2)}")
        print(f"  TCP (mm): x={pose[0]:.1f}  y={pose[1]:.1f}  z={pose[2]:.1f}")

    def disconnect(self):
        """로봇 연결 해제."""
        self.arm.disconnect()

    # ── 충돌 캡슐 (base 프레임) ──────────────────────────────────────────────
    def collision_capsules(self, q=None, link_radii=None, T_EC=None):
        """
        충돌 캡슐 [(name, Capsule), …] (로봇 base 프레임).

        실물 경로 — **공칭 xArm7 해석 FK**(utils.robot.xarm7_kinematics)로 링크 원점을
        구해 캡슐화한다(실물은 공칭 DH 와 일치 가정). q 미지정 시 현재 관절각.
        가상 자세 사전질의(pre-move)에 q 를 직접 주면 된다.

        sim 과 동일 인터페이스(IsaacXArm.collision_capsules) — CollisionWorld.check 공용.
        """
        from utils.collision import capsules_from_joints, DEFAULT_LINK_RADII
        if q is None:
            q = self.get_joint_angles(is_radian=True)
        radii = DEFAULT_LINK_RADII if link_radii is None else link_radii
        return capsules_from_joints(q, link_radii=radii, T_EC=T_EC)


if __name__ == "__main__":
    robot = XArmInterface("192.168.1.210")

    # 읽기만 할 때는 enable_motion() 불필요
    print(" 현재 joint angles (rad) : ", robot.get_joint_angles())
    print(" 현재 TCP pose (B 프레임, mm+rad) : ", robot.get_pose())
    print(" ----------------------------------------------------")

    # try:
    #     robot.go_home(speed=5)  # 내부에서 enable_motion() 호출

    #     print("\n=== move_relative 테스트: +20mm in X, Y, Z ===")
    #     robot.move_relative(dx=20, dy=20, dz=20, speed=5)

    #     input("\nEnter → 원위치 복귀...")
    #     robot.move_relative(dx=-20, dy=-20, dz=-20, speed=5)
    # finally:
    #     robot.disconnect()

    target_pose = [280, 0, 350, 180, -58, 0]
    try :
        robot.enable_motion()  # 모션 제어 활성화
        code = robot.arm.set_position(
            x=target_pose[0], y=target_pose[1], z=target_pose[2],
            roll=target_pose[3], pitch=target_pose[4], yaw=target_pose[5],
            is_radian=False, speed=30, wait=True
        )
        print('code:', code)
        print('실제 TCP:', robot.get_pose(is_radian=False))
        robot.disconnect()
    except Exception as e:
        print("Error:", e)
        robot.disconnect()
