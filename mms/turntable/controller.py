# mms/turntable/controller.py

class TurntableInterface:
    def connect(self):
        """시리얼/TCP 등 연결."""
        raise NotImplementedError

    def disconnect(self):
        raise NotImplementedError

    def home(self):
        """원점 복귀."""
        raise NotImplementedError

    def rotate(self, angle_deg):
        """지정 각도로 회전."""
        raise NotImplementedError

    def get_angle(self):
        """현재 각도 반환."""
        raise NotImplementedError
