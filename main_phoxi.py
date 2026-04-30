# main_phoxi.py
#
# Photoneo PhoXi 3D 센서용 MMS entry point.
# (현재 PhoXi 하드웨어 반납 상태 — `scripts/phoxi/phase1_from_dataset.py` 로 dataset
# replay 가 주 사용 경로. 이 파일은 라이브 PhoXi 가 다시 연결됐을 때 사용.)

import msvcrt
import traceback

import numpy as np

from utils import PROJECT_ROOT
from mms_phoxi.system import MMS, MMSConfig
from mms_phoxi.sensor.phoxi_client import PhoxiConfig
from mms_phoxi.nbv.scan_session import ScanSession, ScanSessionSettings
from mms_phoxi.utils.diagnostics import print_pcd_stats
from mms_phoxi.utils.visualization import visualize
from utils.robot.xarm_interface import XArmInterface
from utils.turntable import Turntable


# ── 하드웨어 ──────────────────────────────────────────────────────────
ROBOT_IP        = "192.168.1.210"
TURNTABLE_IP    = "192.168.0.10"
TURNTABLE_BD_ID = 0

# ── 모션 ──────────────────────────────────────────────────────────────
DISTANCE_M              = 0.414
CAMERA_DEFAULT_ROLL_DEG = 90.0
ROBOT_SPEED_DEG_S       = 15.0
TURNTABLE_VEL_RAD_S     = np.radians(10.0)
SCAN_INIT_JOINTS_DEG    = [-0.5, -43.5, 0.2, 46.2, -0.2, 69.2, 1.0]

RUN_MODE = "scan_session"      # "manual" | "scan_session"

CFG = MMSConfig(
    phoxi=PhoxiConfig(
        serial_number="SEA-023",            # PhoXi 3D Scanner Gen3 (S)
        trigger_timeout_s=15.0,
        target_interval_s=1.0,
    ),
    turntable_frame_yaml=str(PROJECT_ROOT / "config/calibration/turntable_frame.yaml"),
    sensor_frames_yaml=str(PROJECT_ROOT / "config/calibration/hand_eye_phoxi.yaml"),
    T_EC_key="T_E_C",
)

SCAN_SETTINGS = ScanSessionSettings(
    distance_m=DISTANCE_M,
    tsdf_voxel_length=0.002,
    tsdf_sdf_trunc=0.006,
    mesh_backend="pcd_accumulate",
    phase1_enabled=True,
    phase1_theta_step_deg=15.0,
    phase1_dwell_s=0.40,
    phase1_show_progress=True,
    phase1_wait_window_close=True,
    phase1_export_pcd_path=str(PROJECT_ROOT / "output/phase1_merged.ply"),
    phase1_export_mesh_path=str(PROJECT_ROOT / "output/phase1_mesh.ply"),
    phase1_poisson_backend="open3d",
    phase2_enabled=False,
    K_max=5,
    confirm_each_move=True,
    robot_speed_deg_s=ROBOT_SPEED_DEG_S,
    turntable_vel_rad_s=TURNTABLE_VEL_RAD_S,
)


def _flush_stdin() -> None:
    while msvcrt.kbhit():
        msvcrt.getch()


def go_to_scan_init(robot: XArmInterface, speed: float = 10.0, confirm: bool = True) -> None:
    print(f"\n[scan_init] 목표 joints (deg): {SCAN_INIT_JOINTS_DEG}")
    if confirm:
        input("Enter 누르면 scan_init 포즈로 이동...")
    robot.enable_motion()
    code = robot.arm.set_servo_angle(
        angle=list(SCAN_INIT_JOINTS_DEG),
        speed=float(speed), is_radian=False, wait=True,
    )
    if code != 0:
        raise RuntimeError(f"scan_init 이동 실패 (code={code})")


def connect_turntable() -> Turntable:
    tt = Turntable(bd_id=TURNTABLE_BD_ID, ip=TURNTABLE_IP, pulses_per_rev=50000)
    tt.connect(comm_type=0)
    tt.check_drive_info()
    tt.check_drive_err()
    try:
        tt.stop()
    except Exception:
        pass
    tt.set_servo_on(True)
    tt.set_acceleration(np.radians(180), np.radians(180))
    return tt


def _run_scan_session(mms: MMS, robot: XArmInterface, turntable: Turntable) -> None:
    print(f"\n[main_phoxi] Scan Session 시작  K_max={SCAN_SETTINGS.K_max}")
    session = ScanSession(mms, robot, turntable, settings=SCAN_SETTINGS)
    try:
        session.run()
    except KeyboardInterrupt:
        print("\n[main_phoxi] ⚠ KeyboardInterrupt")
    except Exception as e:
        print(f"\n[main_phoxi] ✘ {type(e).__name__}: {e}")
        traceback.print_exc()


def main() -> None:
    robot = XArmInterface(ROBOT_IP)
    turntable = connect_turntable()
    try:
        with MMS(CFG) as mms:
            go_to_scan_init(robot, speed=10, confirm=True)
            if RUN_MODE == "scan_session":
                _run_scan_session(mms, robot, turntable)
            else:
                raise ValueError(f"Unknown RUN_MODE: {RUN_MODE!r}")
    finally:
        try: turntable.stop()
        except Exception: pass
        try: turntable.set_servo_on(False)
        except Exception: pass
        try: turntable.disconnect()
        except Exception: pass
        try: robot.disconnect()
        except Exception: pass


if __name__ == "__main__":
    main()
