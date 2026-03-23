# mms/sensor/harvester_controller.py

class SensorInterface:
    def connect(self):
        """
        - Harvester 생성
        - GenTL Producer(.cti) 파일 로드
        - device 선택 및 ImageAcquirer 생성/시작
        """
        raise NotImplementedError

    def disconnect(self):
        """ImageAcquirer stop, Harvester 정리."""
        raise NotImplementedError

    def fetch_frame(self):
        """
        한 프레임 취득 후 numpy 배열로 반환.
        color / mono 여부는 나중에 정의.
        """
        raise NotImplementedError
