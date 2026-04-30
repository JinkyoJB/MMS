# mms/sensor/orbbec/orbbec_client.py
#
# Orbbec Femto Bolt 센서 클라이언트.

from __future__ import annotations

import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import cv2
import numpy as np

from pyorbbecsdk import (
    Context, Pipeline, Config,
    OBSensorType, OBStreamType, OBFormat,
    AlignFilter, PointCloudFilter, OBError,
)

from utils.scan_result import ScanResult


@dataclass
class OrbbecConfig:
    """
    Orbbec Femto Bolt 클라이언트 설정.

    enable_color
        True → 컬러+뎁스 동시 스트림, per-point RGB 포함.
    frame_timeout_ms
        wait_for_frames 타임아웃 (ms).
    target_interval_s
        캡처 호출당 최소 대기 시간(초). 0.0 이면 대기 없음.
    """
    enable_color: bool = True
    frame_timeout_ms: int = 2000
    target_interval_s: float = 0.2


class OrbbecClient:
    """
    Orbbec Femto Bolt 센서 클라이언트.

    capture() 는 센서(S) 프레임의 ScanResult를 반환한다.
    좌표 단위: points / depth 모두 mm.
    """

    def __init__(self, cfg: OrbbecConfig) -> None:
        self.cfg = cfg
        self._pipeline: Optional[Pipeline] = None
        self._align_filter: Optional[AlignFilter] = None
        self._pc_filter: Optional[PointCloudFilter] = None
        self._has_color: bool = False
        self._initialized: bool = False

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def initialize(self) -> None:
        pipeline = Pipeline()
        config = Config()

        # DEPTH (필수)
        depth_profile_list = pipeline.get_stream_profile_list(OBSensorType.DEPTH_SENSOR)
        if depth_profile_list is None:
            raise RuntimeError("No depth profile available.")
        config.enable_stream(depth_profile_list.get_default_video_stream_profile())

        # COLOR (선택)
        has_color = False
        if self.cfg.enable_color:
            try:
                color_list = pipeline.get_stream_profile_list(OBSensorType.COLOR_SENSOR)
                if color_list is not None:
                    config.enable_stream(color_list.get_default_video_stream_profile())
                    has_color = True
            except OBError:
                pass
        self._has_color = has_color

        pipeline.enable_frame_sync()
        pipeline.start(config)

        camera_param = pipeline.get_camera_param()
        align_to = OBStreamType.COLOR_STREAM if has_color else OBStreamType.DEPTH_STREAM
        align_filter = AlignFilter(align_to_stream=align_to)

        pc_filter = PointCloudFilter()
        pc_filter.set_camera_param(camera_param)

        # 워밍업
        print("[OrbbecClient] warming up...", end="", flush=True)
        for _ in range(30):
            f = pipeline.wait_for_frames(1000)
            if f is None:
                continue
            d = f.get_depth_frame()
            c = f.get_color_frame() if has_color else True
            if d is not None and (not has_color or c is not None):
                break
            print(".", end="", flush=True)
        print(" done")

        self._pipeline = pipeline
        self._align_filter = align_filter
        self._pc_filter = pc_filter
        self._initialized = True

    def shutdown(self) -> None:
        if self._pipeline is not None:
            self._pipeline.stop()
        self._pipeline = None
        self._align_filter = None
        self._pc_filter = None
        self._initialized = False

    # ------------------------------------------------------------------
    # Capture
    # ------------------------------------------------------------------

    def capture(self, frame_id: int = 0, timestamp: Optional[float] = None) -> Optional[ScanResult]:
        """
        1회 프레임 캡처.

        Returns
        -------
        ScanResult
            points : (N, 3) float32, mm, 센서(S) 프레임
            depth  : (H, W) float32, mm
            img    : (H, W, 3) uint8 RGB | None
            colors : (N, 3) float32 [0,1] | None
        """
        if not self._initialized:
            raise RuntimeError("[OrbbecClient] Not initialized.")

        if timestamp is None:
            timestamp = time.perf_counter()

        _t0 = time.perf_counter()

        frames = self._pipeline.wait_for_frames(self.cfg.frame_timeout_ms)
        if frames is None:
            return None

        depth_frame = frames.get_depth_frame()
        if depth_frame is None:
            return None
        if self._has_color and frames.get_color_frame() is None:
            return None

        frames_aligned = self._align_filter.process(frames)
        if frames_aligned is None:
            return None
        frames_aligned = frames_aligned.as_frame_set()

        depth_frame = frames_aligned.get_depth_frame()
        if depth_frame is None:
            return None

        depth_scale = depth_frame.get_depth_scale()  # → meters
        self._pc_filter.set_position_data_scaled(depth_scale)

        # 깊이 이미지 → mm
        raw_depth_mm: Optional[np.ndarray] = None
        try:
            h = depth_frame.get_height()
            w = depth_frame.get_width()
            depth_data = np.frombuffer(depth_frame.get_data(), dtype=np.uint16)
            raw_depth_mm = depth_data.reshape((h, w)).astype(np.float32) * depth_scale * 1000.0
        except Exception:
            pass

        # 컬러 이미지
        raw_img: Optional[np.ndarray] = None
        if self._has_color:
            color_frame = frames_aligned.get_color_frame()
            if color_frame is not None:
                buf = np.frombuffer(color_frame.get_data(), dtype=np.uint8)
                img_bgr = cv2.imdecode(buf, cv2.IMREAD_COLOR)
                if img_bgr is not None:
                    raw_img = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
            self._pc_filter.set_create_point_format(OBFormat.RGB_POINT)
        else:
            self._pc_filter.set_create_point_format(OBFormat.POINT)

        # 포인트 클라우드 (센서 프레임, meters) → mm 변환
        pc_frame = self._pc_filter.process(frames_aligned)
        if pc_frame is None:
            return None

        points_raw = np.asarray(self._pc_filter.calculate(pc_frame), dtype=np.float32)
        points_m = points_raw[:, :3]
        colors_01: Optional[np.ndarray] = (
            points_raw[:, 3:6] / 255.0
            if self._has_color and points_raw.shape[1] >= 6
            else None
        )

        # 유효점 필터링 후 m → mm
        valid = ~np.all(points_m == 0.0, axis=1)
        points_mm = points_m[valid] * 1000.0
        colors_valid = colors_01[valid] if colors_01 is not None else None

        _t1 = time.perf_counter()
        print(f"  [OrbbecClient] capture={_t1-_t0:.3f}s  pts={len(points_mm):,}")

        result = ScanResult(
            sensor_type="orbbec",
            points=points_mm,
            img=raw_img,
            depth=raw_depth_mm,
            colors=colors_valid,
            frame_id=frame_id,
            timestamp=timestamp,
        )

        elapsed = time.perf_counter() - _t0
        remaining = self.cfg.target_interval_s - elapsed
        if remaining > 0.0:
            time.sleep(remaining)

        return result


# ------------------------------------------------------------------
# 테스트
# ------------------------------------------------------------------
if __name__ == "__main__":
    if __package__ is None:
        sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

    import open3d as o3d

    cfg = OrbbecConfig(enable_color=True)
    client = OrbbecClient(cfg)

    try:
        client.initialize()
        result = client.capture(frame_id=0)

        if result is not None:
            print(f"pts={len(result.points):,}  depth={'yes' if result.depth is not None else 'no'}")
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(result.points.astype(np.float64))
            if result.colors is not None:
                pcd.colors = o3d.utility.Vector3dVector(result.colors.astype(np.float64))
            o3d.visualization.draw_geometries([pcd], window_name="OrbbecClient")
    finally:
        client.shutdown()
