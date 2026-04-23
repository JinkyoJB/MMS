import msvcrt
import numpy as np

from mms import PROJECT_ROOT
from mms.system import MMS, MMSConfig
from mms.sensor.phoxi.phoxi_client import PhoxiConfig
from mms.robot.xarm_interface import XArmInterface
from mms.turntable import Turntable
from mms.utils.diagnostics import print_pcd_stats, suggest_roi
from mms.utils.visualization import visualize

ROBOT_IP        = "192.168.1.210"
TURNTABLE_IP    = "192.168.0.10"
TURNTABLE_BD_ID = 0

CFG = MMSConfig(
    phoxi=PhoxiConfig(
        serial_number=None,
        trigger_timeout_s=15.0,
        target_interval_s=1.0,
    ),
    turntable_frame_yaml=str(PROJECT_ROOT / "config/calibration/turntable_frame.yaml"),
    sensor_frames_yaml=str(PROJECT_ROOT / "config/calibration/hand_eye_phoxi.yaml"),
    T_EC_key="T_E_C",
)

# NBV / 모션 파라미터
DISTANCE_M              = 0.414                 # 표면 → 카메라 거리
CAMERA_DEFAULT_ROLL_DEG = 90.0                  # 광축(z_cam) 기준 roll —
                                                # 90°면 x_cam ↔ y_cam 스왑(portrait ↔ landscape)
ROBOT_SPEED_DEG_S       = 20.0
TURNTABLE_VEL_RAD_S     = np.radians(30.0)
PREPROC_VOXEL_M         = 0.003

# 스캔 시작 자세 — 홈 TCP xyz 유지 + rpy 만 변경하여 턴테이블 조준
# (홈 자세에서는 PhoXi 가 턴테이블 위를 못 보므로 캡처 전에 한 번 돌려준다)
SCAN_START_RPY_DEG  = (178.5, -20.5, 0.0)


def _flush_stdin() -> None:
    """캡처 대기 중 눌린 잔류 Enter 를 stdin 버퍼에서 제거."""
    while msvcrt.kbhit():
        msvcrt.getch()


def go_to_scan_start(robot: XArmInterface, speed: float = 10.0, confirm: bool = True) -> None:
    """
    홈 자세의 xyz 는 유지하고 rpy 만 SCAN_START_RPY_DEG 로 재설정.
    이 자세에서 PhoXi 광축이 턴테이블을 향하므로 여기서 캡처해야 한다.
    """
    pose = robot.get_pose(is_radian=True)  # [x(mm), y(mm), z(mm), r, p, y(rad)]
    r, p, y = SCAN_START_RPY_DEG
    print(f"\n[scan_start] 목표 TCP  xyz=[{pose[0]:.1f},{pose[1]:.1f},{pose[2]:.1f}] mm  "
          f"rpy=({r:+.1f},{p:+.1f},{y:+.1f}) deg")
    if confirm:
        input("Enter 누르면 scan_start 포즈로 이동...")

    robot.enable_motion()
    code = robot.arm.set_position(
        x=float(pose[0]), y=float(pose[1]), z=float(pose[2]),
        roll=r, pitch=p, yaw=y,
        is_radian=False, speed=float(speed), wait=True,
    )
    if code != 0:
        raise RuntimeError(f"scan_start 이동 실패 (code={code})")

    pose_now = robot.get_pose(is_radian=False)
    print(f"[scan_start] 도착 TCP  xyz=[{pose_now[0]:.1f},{pose_now[1]:.1f},{pose_now[2]:.1f}] mm  "
          f"rpy=({pose_now[3]:+.1f},{pose_now[4]:+.1f},{pose_now[5]:+.1f}) deg")


def connect_turntable() -> Turntable:
    """턴테이블 연결 + 서보 ON."""
    tt = Turntable(bd_id=TURNTABLE_BD_ID, ip=TURNTABLE_IP, pulses_per_rev=50000)
    tt.connect(comm_type=0)
    tt.check_drive_info()
    tt.check_drive_err()
    tt.set_servo_on(True)
    tt.set_acceleration(np.radians(180), np.radians(180))
    return tt


def control_step(
    mms: MMS,
    robot: XArmInterface,
    turntable: Turntable,
    theta_target: float,
    work_in_O: bool = True,
) -> dict:
    """
    한 스텝 사이클 (docs/1_control_layers.md §5):

      ① 캡처 → ② 상위 레이어 (수동 pick → T_CO_des) → ③ 하위 레이어 (θ, T_EB_des 실행)

    Parameters
    ----------
    theta_target : float
        이번 스텝의 턴테이블 절대 목표 각도 (rad).
    work_in_O : bool
        True 면 O 프레임에서 picking (반환 T_CO_des).
        turntable_transform 이 설정되어 있어야 함.

    Returns
    -------
    dict  — execute_target() 결과
    """
    # ── ① 캡처 ─────────────────────────────────────────────────────────
    _flush_stdin()
    input("\n[control_step] 캡처를 시작하려면 Enter... ")
    batch = mms.capture_frames(1, ee_pose_fn=robot.get_ee_pose_mat)
    if not batch:
        raise RuntimeError("캡처 실패 — 프레임이 수집되지 않았습니다.")

    print_pcd_stats(batch)

    # ROI / voxel / normals 전처리 — picker 로컬 노말 계산 정확도를 위해 권장
    roi = suggest_roi(batch)
    print(f"[control_step] 자동 ROI: "
          f"x=[{roi[0]:+.3f},{roi[1]:+.3f}] "
          f"y=[{roi[2]:+.3f},{roi[3]:+.3f}] "
          f"z=[{roi[4]:+.3f},{roi[5]:+.3f}]")
    mms.preprocess(
        frames=batch,
        roi_bbox=roi,
        voxel_size=PREPROC_VOXEL_M,
        enable_voxel=True,
        enable_denoise=True,
        enable_normals=True,
    )

    # ── ② 상위 레이어: 수동 pick → T_CO_des ──────────────────────────
    theta_now = float(turntable.getActualPos()) if turntable.is_connected else 0.0
    print(f"\n[control_step] 현재 θ (turntable) = {np.degrees(theta_now):+.2f}°")

    # IK 실패 시 재picking 루프. 매 반복마다 상위 레이어에서 다른 표면점을 골라
    # 새 T_CO_des 를 만들고, 하위 레이어에서 IK 를 다시 확인한다.
    attempt = 0
    while True:
        attempt += 1
        print(f"\n[control_step] 타겟 선택 시도 #{attempt}")

        T_CO_des = mms.nbv_pick_target(
            frames=batch,
            theta_current=theta_now,
            distance_m=DISTANCE_M,
            work_in_O=work_in_O,
            preview=True,
            default_roll_deg=CAMERA_DEFAULT_ROLL_DEG,
            # IK reachability 프리컴퓨트: 초록=가능, 빨강=불가
            check_ik=True,
            robot=robot,
            theta_target=theta_target,
            ik_voxel_m=0.025,
            # ik_roll_candidates=None → default_roll_deg 한 샘플만 검사
            ik_keep_only_reachable=False,
        )

        # ── ③ 하위 레이어: θ_target 으로 턴테이블+로봇 이동 ───────────
        result = mms.execute_target(
            T_CO_des=T_CO_des,
            theta=theta_target,
            robot=robot,
            turntable=turntable,
            robot_speed=ROBOT_SPEED_DEG_S,
            turntable_vel_rad_s=TURNTABLE_VEL_RAD_S,
            move_turntable=True,
            move_robot=True,
            confirm=True,
        )

        if result["ik_ok"]:
            return result

        _flush_stdin()
        ans = input("\n  IK 실패 — 다른 표면점으로 재선택할까요? "
                    "(Enter=재선택, q=중단): ").strip().lower()
        if ans == "q":
            print("[control_step] 사용자 중단.")
            return result


def main() -> None:
    robot = XArmInterface(ROBOT_IP)
    turntable = connect_turntable()

    try:
        with MMS(CFG) as mms:
            # 홈 자세 — IK seed 로도 쓰이는 중립 자세
            robot.go_home(speed=5, confirm=True)

            # 홈에서는 PhoXi 가 턴테이블 위를 못 보므로 rpy 만 돌려 조준
            go_to_scan_start(robot, speed=10, confirm=True)

            # 한 스텝: 현재 θ 를 유지 (정책 대신 호출자 지정)
            theta_target = float(turntable.getActualPos())
            result = control_step(
                mms=mms,
                robot=robot,
                turntable=turntable,
                theta_target=theta_target,
                work_in_O=True,
            )

            # 재캡처 — 이동 후의 뷰 확인 (선택)
            _flush_stdin()
            ans = input("\n[main] 이동 후 재캡처? (y/N): ").strip().lower()
            if ans == "y":
                batch2 = mms.capture_frames(1, ee_pose_fn=robot.get_ee_pose_mat)
                if batch2:
                    visualize("After move — PhoXi", batch2)

            print(f"\n[main] 완료.  θ={np.degrees(result['theta']):+.2f}°  "
                  f"robot_moved={result['robot_moved']}  "
                  f"turntable_moved={result['turntable_moved']}")
    finally:
        robot.disconnect()
        turntable.disconnect()


if __name__ == "__main__":
    main()
    
