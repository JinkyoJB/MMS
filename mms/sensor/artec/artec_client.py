# mms/sensor/artec/artec_client.py
#
# Artec 3D 스캐너 클라이언트.
# pybind11 확장 모듈 artec_sdk_py를 통해 Artec Capture SDK를 래핑.
#
# 빌드 방법
# ---------
#   pip install pybind11
#   $cmake = "...\CMake\bin\cmake.exe"
#   & $cmake -B build -G "Visual Studio 18 2026" -A x64
#   & $cmake --build build --config Release

from __future__ import annotations

import os
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

if __name__ == "__main__":
    sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

import numpy as np

from mms.sensor.scan_result import ScanResult

_ARTEC_SDK_BIN = Path(r"C:\Program Files\Artec\Artec 3D Scanning SDK\bin-x64")


def _load_artec_sdk_py():
    if _ARTEC_SDK_BIN.exists():
        os.add_dll_directory(str(_ARTEC_SDK_BIN))
        os.environ["PATH"] = str(_ARTEC_SDK_BIN) + ";" + os.environ.get("PATH", "")

    _here = Path(__file__).parent
    if str(_here) not in sys.path:
        sys.path.insert(0, str(_here))

    try:
        import artec_sdk_py as _m
        return _m
    except ImportError as e:
        raise ImportError(
            "[ArtecClient] artec_sdk_py 모듈 로드 실패.\n"
            "  CMakeLists.txt로 빌드 후 artec_sdk_py.pyd를 mms/sensor/artec/에 배치.\n"
            f"  원인: {e}"
        ) from e


# ------------------------------------------------------------------
# Config
# ------------------------------------------------------------------

@dataclass
class ArtecConfig:
    """
    Artec 3D 스캐너 클라이언트 설정.

    serial_number : str | None
        스캐너 시리얼 번호. None → 첫 번째 발견된 스캐너.
    capture_texture : bool
        True → RGB 텍스처 포함 재구성. False → 지오메트리만 (빠름).
    fps : float | None
        스캐너 FPS. None → 기본값 유지.
    target_interval_s : float
        캡처 호출당 최소 대기 시간(초). 0.0 이면 대기 없음.
    """
    serial_number: Optional[str] = None
    capture_texture: bool = True
    fps: Optional[float] = None
    target_interval_s: float = 0.0


# ------------------------------------------------------------------
# ArtecClient
# ------------------------------------------------------------------

class ArtecClient:
    """
    Artec 3D 스캐너 클라이언트.

    capture() 는 센서(S) 프레임의 ScanResult를 반환한다.
    좌표 단위: points mm, 센서 프레임.
    """

    def __init__(self, cfg: ArtecConfig) -> None:
        self.cfg = cfg
        self._sdk = None
        self._scanner = None
        self._initialized: bool = False

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def initialize(self) -> None:
        self._sdk = _load_artec_sdk_py()

        serial = self.cfg.serial_number or ""
        self._scanner = self._sdk.ArtecScanner(serial)
        self._scanner.initialize()

        if self.cfg.fps is not None:
            fps = min(self.cfg.fps, self._scanner.get_max_fps())
            self._scanner.set_fps(fps)
            print(f"[ArtecClient] FPS={fps:.1f}")

        self._initialized = True
        print(
            f"[ArtecClient] Connected: {self._scanner.get_name()}"
            f"  serial={self._scanner.get_serial()}"
        )

    def shutdown(self) -> None:
        if self._scanner is not None:
            self._scanner.shutdown()
            self._scanner = None
        self._initialized = False
        print("[ArtecClient] Disconnected.")

    # ------------------------------------------------------------------
    # Capture
    # ------------------------------------------------------------------

    def capture(self, frame_id: int = 0, timestamp: Optional[float] = None) -> Optional[ScanResult]:
        """
        1회 스캔.

        Returns
        -------
        ScanResult
            points    : (N, 3) float32, mm, 센서(S) 프레임 (IFrameMesh 버텍스)
            normals   : (N, 3) float32 스무스 법선 | None
            triangles : (M, 3) int32 메쉬 face 인덱스
            img       : (H, W, 3) uint8 RGB 텍스처 | None
        """
        if not self._initialized:
            raise RuntimeError("[ArtecClient] Not initialized.")

        if timestamp is None:
            timestamp = time.perf_counter()

        _t0 = time.perf_counter()
        result = self._scanner.capture(self.cfg.capture_texture)
        _t1 = time.perf_counter()

        pts = np.asarray(result.points,    dtype=np.float32)
        if pts.shape[0] == 0:
            return None

        nrm = np.asarray(result.normals,   dtype=np.float32)
        tri = np.asarray(result.triangles, dtype=np.int32)
        tex = np.asarray(result.texture_image, dtype=np.uint8)

        scan = ScanResult(
            sensor_type="artec",
            points=pts,
            normals=nrm   if nrm.shape[0] > 0 else None,
            triangles=tri if tri.shape[0] > 0 else None,
            img=tex       if tex.shape[0]  > 0 else None,
            frame_id=frame_id,
            timestamp=timestamp,
        )

        elapsed = time.perf_counter() - _t0
        print(
            f"  [ArtecClient] capture={_t1-_t0:.3f}s  total={elapsed:.3f}s"
            f"  pts={len(scan.points):,}"
        )

        remaining = self.cfg.target_interval_s - elapsed
        if remaining > 0.0:
            time.sleep(remaining)

        return scan


# ------------------------------------------------------------------
# 테스트
# ------------------------------------------------------------------
if __name__ == "__main__":
    _PROJECT_ROOT = Path(__file__).resolve().parents[3]

    import open3d as o3d

    sdk = _load_artec_sdk_py()
    print("=== 연결된 스캐너 목록 ===")
    scanners = sdk.enumerate_scanners()
    if not scanners:
        print("  스캐너 없음")
        sys.exit(1)
    for s in scanners:
        print(f"  [{s['index']}] {s['name']}  serial={s['serial']}")

    cfg = ArtecConfig(
        serial_number=None,
        capture_texture=True,
    )
    client = ArtecClient(cfg)

    try:
        print("\n=== initialize ===")
        client.initialize()

        print("\n=== capture ===")
        result = client.capture(frame_id=0)

        if result is not None:
            print(
                f"  pts={len(result.points):,}"
                f"  normals={result.normals is not None}"
                f"  triangles={result.triangles is not None}"
                f"  img={result.img is not None}"
            )
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(result.points.astype(np.float64))
            o3d.visualization.draw_geometries([pcd], window_name="ArtecClient",
                                              width=1280, height=720)
        else:
            print("  capture 실패 (점 없음)")

    except Exception as e:
        print(f"\n[ERROR] {e}", file=sys.stderr)
        raise
    finally:
        client.shutdown()
