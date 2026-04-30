# mms/sensor/scan_result.py
#
# 센서 캡처 결과 공통 데이터클래스.
# 모든 센서 클라이언트는 이 클래스를 반환한다.
#
# 좌표 단위: points / depth 모두 mm, 센서(S) 프레임.
# 시스템 레이어(MMS)에서 T_E_S, T_E_B를 적용해 Base(B) 프레임으로 변환한다.

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional

import numpy as np


@dataclass
class ScanResult:
    """
    센서 1회 캡처 결과.

    모든 좌표는 센서(S) 프레임, mm 단위.
    좌표계 변환(B 프레임 변환)은 시스템 레이어에서 수행.

    Attributes
    ----------
    sensor_type : str
        'artec' | 'phoxi' | 'orbbec'
    points : (N, 3) float32
        포인트 클라우드, mm, 센서 프레임.
    normals : (N, 3) float32 | None
        단위 법선 벡터.
    triangles : (M, 3) int32 | None
        메쉬 삼각형 인덱스 (Artec 전용).
    img : (H, W, 3) uint8 | None
        RGB 이미지.
    depth : (H, W) float32 | None
        깊이 이미지, mm.
    colors : (N, 3) float32 | None
        per-point RGB [0, 1] (Orbbec XYZRGB 모드 전용).
    frame_id : int
    timestamp : float
        단조 시간, 초.
    """

    sensor_type: str
    points: np.ndarray                       # (N, 3) float32, mm, sensor frame
    normals: Optional[np.ndarray] = None     # (N, 3) float32
    triangles: Optional[np.ndarray] = None   # (M, 3) int32, mesh faces (Artec)
    img: Optional[np.ndarray] = None         # (H, W, 3) uint8 RGB
    depth: Optional[np.ndarray] = None       # (H, W) float32, mm
    colors: Optional[np.ndarray] = None      # (N, 3) float32 [0,1]
    frame_id: int = 0
    timestamp: float = 0.0

    def __post_init__(self) -> None:
        if self.points.dtype != np.float32:
            self.points = self.points.astype(np.float32)
