# mms/robot/xarm_client.py

class RobotInterface:
    def connect(self):
        """ROS 노드 초기화, 서비스/토픽 클라이언트 준비."""
        raise NotImplementedError

    def disconnect(self):
        """필요 시 shutdown 작업."""
        raise NotImplementedError

    def move_joint(self, joints, speed=None, acc=None):
        """관절 값 리스트로 이동."""
        raise NotImplementedError

    def move_linear(self, pose, speed=None, acc=None):
        """TCP pose(x, y, z, rx, ry, rz) 기준 선형 이동."""
        raise NotImplementedError

    def go_home(self):
        """홈 포즈로 이동."""
        raise NotImplementedError

    def stop(self):
        """모션 정지."""
        raise NotImplementedError
