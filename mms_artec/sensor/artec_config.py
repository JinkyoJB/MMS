# mms_artec/sensor/artec_config.py
#
# ArtecConfig — 스캐너 클라이언트 설정 (바인딩 비의존, 경량).
#
# artec_client.py 는 Artec C++ 바인딩(Windows 전용)을 로드하므로 Linux/Isaac 에선
# import 가 실패한다. 설정 dataclass 만 별도 모듈로 분리해 양쪽(real/isaac) 에서
# 안전하게 import 한다. artec_client.py 는 하위호환을 위해 이 클래스를 re-export 한다.

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional


@dataclass
class ArtecConfig:
    """
    Artec 3D 스캐너 클라이언트 설정.

    serial_number : str | None
        스캐너 시리얼 번호. None → 인덱스 0 스캐너.
    capture_texture : bool
        True → RGB 텍스처 포함 재구성. False → 지오메트리만 (빠름).
    fps : float | None
        스캐너 FPS. None → 기본값 유지.
    target_interval_s : float
        capture() 호출당 최소 대기 시간(초). 0.0 이면 대기 없음.
    scan_range_near_mm / scan_range_far_mm : float | None
        **작동거리 창**(mm). None → SDK 기본값 그대로.

        ★ 여기가 **유일한 손잡이**다. SDK 는 스캔 범위를 두 객체가 각각 들고 있다 —
          `IFrameProcessor`(단일 캡처·플래너가 조회하는 쪽)와
          `IScanningProcedure`(스트리밍 스캔이 쓰는 쪽). 예전에는 스트리밍 세션만
          자기 설정으로 세션 쪽을 건드려서, **플래너가 믿는 창과 실제 스캔 창이
          따로 놀 수 있었다.** 이제 이 값 하나가 둘 다를 정한다:
            · `ArtecClient.initialize()` → processor 에 적용
            · 스트리밍 세션 → 자기 설정이 None 이면 이 값을 따라감
          그리고 `SensorModel.dof`(밴드 산정 입력)는 `sensor.scanning_range()` 로
          **조회해서** 쓰므로, 설정과 계획 가정이 정의상 같아진다.
    """
    serial_number: Optional[str] = None
    capture_texture: bool = True
    fps: Optional[float] = None
    target_interval_s: float = 0.0
    scan_range_near_mm: Optional[float] = None
    scan_range_far_mm: Optional[float] = None
