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
    """
    serial_number: Optional[str] = None
    capture_texture: bool = True
    fps: Optional[float] = None
    target_interval_s: float = 0.0
