# mms/sensor/harvester_controller.py

import os
import sys
import numpy as np
from pathlib import Path
from harvesters.core import Harvester


# GENICAM_GENTL64_PATH 환경변수에서 .cti/.so 경로 자동 탐색
# 없으면 fallback으로 mvIMPACT 기본 경로 사용
_GENTL_ENV = "GENICAM_GENTL64_PATH"
_PRODUCER_FILE = "libmvGenTLProducer.so"  # Linux
if sys.platform == "win32":
    _PRODUCER_FILE = "mvGenTLProducer.cti"


def _find_producer_path() -> Path:
    """GENICAM_GENTL64_PATH 환경변수에서 GenTL Producer 파일 탐색."""
    env_val = os.getenv(_GENTL_ENV)
    if env_val is None:
        # 환경변수 없을 때 fallback
        fallback = Path("/opt/mvIMPACT_Acquire/lib/x86_64") / _PRODUCER_FILE
        print(f"[Sensor] WARNING: {_GENTL_ENV} not set. Trying fallback: {fallback}")
        return fallback

    for p in env_val.split(os.pathsep):
        candidate = Path(p) / _PRODUCER_FILE
        if candidate.exists():
            return candidate

    raise FileNotFoundError(
        f"[Sensor] '{_PRODUCER_FILE}' not found in {_GENTL_ENV}={env_val}"
    )


class HarvesterController:
    """
    GenICam Harvesters 기반 GigE-V 센서 제어 클래스.
    GENICAM_GENTL64_PATH 환경변수를 통해 GenTL Producer를 자동 탐색한다.

    Usage
    -----
    # 환경변수 필수 (예: ~/.bashrc에 추가)
    # export GENICAM_GENTL64_PATH=/opt/mvIMPACT_Acquire/lib/x86_64

    sensor = HarvesterController(serial_number="XXXX")
    sensor.connect()
    frame = sensor.fetch_frame()
    sensor.disconnect()
    """

    def __init__(self, serial_number: str = None, device_index: int = 0):
        """
        Parameters
        ----------
        serial_number : str, optional
            장치 시리얼 번호로 특정 디바이스 지정. None이면 device_index 사용.
        device_index : int
            serial_number 없을 때 사용할 디바이스 목록 인덱스 (default: 0)
        """
        self._serial_number = serial_number
        self._device_index = device_index
        self._harvester = None
        self._ia = None  # ImageAcquirer

    def connect(self) -> None:
        """
        GenTL Producer 로드 → 디바이스 탐색 → 연결 → 취득 시작.
        """
        producer_path = _find_producer_path()
        print(f"[Sensor] Using producer: {producer_path}")

        self._harvester = Harvester()
        self._harvester.add_file(str(producer_path), check_existence=True, check_validity=True)
        self._harvester.update()

        devices = self._harvester.device_info_list
        if len(devices) == 0:
            raise RuntimeError("[Sensor] No GenICam devices found. Check network/device connection.")

        print(f"[Sensor] Found {len(devices)} device(s):")
        for i, dev in enumerate(devices):
            print(f"  [{i}] {dev}")

        # 시리얼 번호 지정 시 해당 장치로 연결, 없으면 인덱스로 연결
        search_key = {"serial_number": self._serial_number} if self._serial_number else self._device_index
        self._ia = self._harvester.create(search_key)

        features = self._ia.remote_device.node_map
        fw_ver = getattr(features, "DeviceFirmwareVersion", None)
        if fw_ver:
            print(f"[Sensor] Firmware version: {fw_ver.value}")

        # Default UserSet 로드 (continuous acquisition mode 복원)
        try:
            features.UserSetSelector.value = "Default"
            features.UserSetLoad.execute()
        except Exception as e:
            print(f"[Sensor] UserSetLoad skipped: {e}")

        self._ia.start()
        print("[Sensor] Acquisition started.")

    def disconnect(self) -> None:
        """
        취득 중지 및 리소스 해제.
        """
        if self._ia is not None:
            self._ia.stop()
            self._ia.destroy()
            self._ia = None

        if self._harvester is not None:
            self._harvester.reset()
            self._harvester = None

        print("[Sensor] Disconnected.")

    def fetch_frame(self, timeout: float = 15.0) -> np.ndarray:
        """
        단일 프레임 취득 후 numpy 배열로 반환.

        Parameters
        ----------
        timeout : float
            프레임 대기 최대 시간 (초). default: 15.0

        Returns
        -------
        np.ndarray
            이미지 데이터 배열.
        """
        if self._ia is None:
            raise RuntimeError("[Sensor] Not connected. Call connect() first.")

        with self._ia.fetch(timeout=timeout) as buffer:
            component = buffer.payload.components[0]
            frame = component.data.copy()  # buffer 반환 전 반드시 copy

        return frame

    def get_device_info(self) -> list:
        """연결된 디바이스 정보 목록 반환."""
        if self._harvester is None:
            raise RuntimeError("[Sensor] Not connected. Call connect() first.")
        return self._harvester.device_info_list

    @property
    def node_map(self):
        """remote_device의 NodeMap에 직접 접근. 파라미터 읽기/쓰기에 사용."""
        if self._ia is None:
            raise RuntimeError("[Sensor] Not connected.")
        return self._ia.remote_device.node_map


# --------------------------
# 기본 기능 테스트
# --------------------------
if __name__ == "__main__":
    # 환경변수 미설정 시 자동 안내
    if not os.getenv(_GENTL_ENV):
        print(f"[WARN] {_GENTL_ENV} is not set.")
        print("  Run: export GENICAM_GENTL64_PATH=/opt/mvIMPACT_Acquire/lib/x86_64")

    # serial_number=None → 첫 번째 발견된 장치에 연결
    sensor = HarvesterController(serial_number=None, device_index=0)

    try:
        print("=== [TEST 1] connect ===")
        sensor.connect()

        print("\n=== [TEST 2] device_info ===")
        for info in sensor.get_device_info():
            print(f"  {info}")

        print("\n=== [TEST 3] fetch_frame ===")
        frame = sensor.fetch_frame(timeout=15.0)
        print(f"  shape   : {frame.shape}")
        print(f"  dtype   : {frame.dtype}")
        print(f"  min/max : {frame.min()} / {frame.max()}")

        print("\n=== [TEST 4] fetch x5 FPS 측정 ===")
        for i in range(5):
            sensor.fetch_frame()
            print(f"  Frame {i+1}  FPS: {sensor._ia.statistics.fps:.2f}", end="\r")
        print()

    except Exception as e:
        print(f"\n[ERROR] {e}", file=sys.stderr)

    finally:
        print("\n=== [TEST 5] disconnect ===")
        sensor.disconnect()
