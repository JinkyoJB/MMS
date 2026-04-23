import sys
from pathlib import Path

_PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_PROJECT_ROOT))

from mms import PROJECT_ROOT
from mms.system import MMS, MMSConfig
from mms.sensor.phoxi.phoxi_client import PhoxiConfig
from mms.robot.xarm_interface import XArmInterface
from mms.utils.diagnostics import print_pcd_stats
from mms.utils.visualization import visualize, visualize_hand_eye_calibration

ROBOT_IP      = "192.168.1.210"
THETA         = 0.0
N_CALIB_POSES = 3

cfg = MMSConfig(
    phoxi=PhoxiConfig(
        sensor_frames_yaml=str(PROJECT_ROOT / "config/sensor_frames.yaml"),
        T_EC_key="T_EC_phoxi",
        serial_number=None,
        trigger_timeout_s=15.0,
        target_interval_s=1.0,
    ),
    turntable_frame_yaml=str(PROJECT_ROOT / "config/object_frame.yaml"),
)


def main():
    robot = XArmInterface(ROBOT_IP)
    with MMS(cfg) as mms:

        robot.go_home(speed=5, confirm=True)
        # check_hand_eye(mms, robot, n_poses=N_CALIB_POSES)
    robot.disconnect()


if __name__ == "__main__":
    main()
