import msvcrt
import numpy as np
from mms import PROJECT_ROOT
from mms.system import MMS, MMSConfig
from mms.sensor.phoxi_client import PhoxiConfig
from mms.robot.xarm_interface import XArmInterface
from mms.utils.diagnostics import print_pcd_stats
from mms.utils.visualization import visualize, visualize_hand_eye_calibration

ROBOT_IP      = "192.168.1.210"
THETA         = 0.0
N_CALIB_POSES = 3

cfg = MMSConfig(
    phoxi=PhoxiConfig(
        sensor_frames_yaml=str(PROJECT_ROOT / "config/sensor_frames.yaml"),
        T_E_S_key="T_E_S_phoxi",
        serial_number=None,
        trigger_timeout_s=15.0,
        target_interval_s=1.0,
    ),
    object_frame_yaml=str(PROJECT_ROOT / "config/object_frame.yaml"),
)


def verify_world_transform(mms):
    wt = mms.world_transform
    if wt is None:
        return
    x_O = np.array([0.1, 0.0, 0.0, 1.0])
    for deg in [0, 45, 90, 180]:
        theta    = np.radians(deg)
        x_B      = wt.T_O_B(theta) @ x_O
        x_O_back = wt.T_B_O(theta) @ x_B
        err      = np.max(np.abs(x_O - x_O_back))
        print(f"  theta={deg}  err={err:.2e}")


def _flush_stdin():
    """캡처 대기 중 눌린 잔류 Enter를 stdin 버퍼에서 제거한다."""
    while msvcrt.kbhit():
        msvcrt.getch()


def check_hand_eye(mms, robot, n_poses=3):
    calib_frames = []
    for i in range(n_poses):
        _flush_stdin()
        input(f"[{i+1}/{n_poses}] 로봇 이동 후 Enter... ")
        batch = mms.capture_frames(1, ee_pose_fn=robot.get_ee_pose_mat)
        if batch:
            calib_frames.append(batch[0])
    if len(calib_frames) >= 2:
        visualize_hand_eye_calibration(
            calib_frames, T_E_S=mms.sensor.T_E_S, frame_size=0.05,
            title="Hand-Eye Calibration Verification")


def main():
    robot = XArmInterface(ROBOT_IP)
    with MMS(cfg) as mms:
        # verify_world_transform(mms)
        # batch = mms.capture_frames(1, ee_pose_fn=robot.get_ee_pose_mat)
        # if not batch:
        #     robot.disconnect()
        #     return
        # print_pcd_stats(batch)
        # visualize("PhoXi — Raw PCD", batch)
        # mms.visualize_with_object_frame(batch, theta=THETA, frame_size=0.05,
        #                                  title="PhoXi — Frame Transform Verification")
        check_hand_eye(mms, robot, n_poses=N_CALIB_POSES)
    robot.disconnect()


if __name__ == "__main__":
    main()
