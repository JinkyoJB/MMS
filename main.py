# main.py
from mms.robot.xarm_ros_client import RobotInterface
from mms.turntable.controller import TurntableInterface
from mms.sensor.harvester_controller import SensorInterface


def main():
    robot = RobotInterface()
    turntable = TurntableInterface()
    sensor = SensorInterface()

    # 라이프사이클
    robot.connect()
    turntable.connect()
    sensor.connect()

    # 예시 스캔 시퀀스
    scan_angles = [0, 90, 180, 270]
    for angle in scan_angles:
        turntable.rotate(angle)
        # 여기서 나중에 angle -> robot pose 매핑함수 사용할 예정
        # robot.move_linear(pose_for_angle(angle))
        frame = sensor.fetch_frame()
        # frame 저장/처리 함수는 추후 구현

    sensor.disconnect()
    robot.disconnect()
    turntable.disconnect()


if __name__ == "__main__":
    main()
