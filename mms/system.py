# mms/system.py

from __future__ import annotations

import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, Optional, Union

import numpy as np

from mms.core.frames import Frame
from mms.core.stream import Stream
from mms.core.transforms import WorldTransformConfig, load_transform
from mms.sensor.orbbec_client import OrbbecClient, OrbbecConfig


@dataclass
class MMSConfig:
    """
    Top-level MMS configuration.

    Attributes
    ----------
    orbbec : OrbbecConfig
        Orbbec Femto Bolt sensor configuration.
    object_frame_yaml : str, optional
        Path to config/object_frame.yaml (key: T_B_O0).
        If provided, MMS.world_transform is populated on init.
    stream_max_size : int, default=500
        Maximum number of frames retained in MMS.stream (Stream).
        Oldest frames are evicted automatically when the buffer is full.
    """
    orbbec: OrbbecConfig
    object_frame_yaml: Optional[str] = None
    stream_max_size: int = 500
    # 추후 추가 예정
    # robot: RobotConfig
    # turntable: TurntableConfig

    def __post_init__(self):
        if self.object_frame_yaml is not None:
            self.object_frame_yaml = str(Path(self.object_frame_yaml).resolve())


class MMS:
    """
    Top-level MMS (Multi Modal Scanning System) orchestrator.

    Manages sensor lifecycle, frame capture, and preprocessing pipeline.

    Notation
    --------
    T_A^B maps frame A to frame B:  x_B = T_A^B @ x_A

    Attributes
    ----------
    world_transform : WorldTransformConfig or None
        Base <-> Object frame transforms. Populated when
        MMSConfig.object_frame_yaml is provided.

    Example
    -------
    >>> with MMS(cfg) as mms:
    ...     batch = mms.capture_frames(10)
    ...     mms.preprocess(batch, roi_bbox=(-0.5, 0.5, -0.5, 0.5, 0.1, 1.5))
    ...     x_B = mms.world_transform.T_O_B(theta) @ x_O
    """

    def __init__(self, cfg: MMSConfig) -> None:
        self.cfg = cfg
        self.sensor = OrbbecClient(cfg.orbbec)
        self.stream: Stream = Stream(max_size=cfg.stream_max_size)

        if cfg.object_frame_yaml is not None:
            T_B_O0 = load_transform(cfg.object_frame_yaml, "T_B_O0")
            self.world_transform: Optional[WorldTransformConfig] = WorldTransformConfig(T_B_O0)
        else:
            self.world_transform = None

    # ── lifecycle ──────────────────────────────────────────────────────────

    def initialize(self) -> None:
        """Initialize all hardware clients."""
        self.sensor.initialize()

    def shutdown(self) -> None:
        """Shutdown all hardware clients and release resources."""
        self.sensor.shutdown()

    def __enter__(self) -> MMS:
        self.initialize()
        return self

    def __exit__(self, *_) -> None:
        self.shutdown()

    # ── capture ────────────────────────────────────────────────────────────

    def capture_frames(
        self,
        n: int,
        ee_pose_fn: Optional[Callable[[], np.ndarray]] = None,
    ) -> list[Frame]:
        """
        Capture n frames from the sensor and append them to self.stream.

        Frame pacing is controlled by OrbbecConfig.target_interval_s:
        after each capture, sleeps max(0, target - elapsed) so that
        successive frames are spaced target_interval_s apart.

        Parameters
        ----------
        n : int
            Number of frames to capture.
        ee_pose_fn : () -> (4,4) np.ndarray, optional
            Callable that returns the current T_E^B from the robot.
            If None, identity matrix is used (robot not connected).

        Returns
        -------
        batch : list[Frame]
            Newly captured frames (also appended to self.stream Stream).
        """
        target = self.cfg.orbbec.target_interval_s
        batch: list[Frame] = []

        print(f"[MMS] capture_frames: {n} 프레임 수집  "
              f"(target_interval={target:.3f}s)")
        for i in range(n):
            ee_pose_mat_B = (
                ee_pose_fn() if ee_pose_fn is not None
                else np.eye(4, dtype=np.float64)
            )

            t_start = time.perf_counter()

            frame = self.sensor.capture_frame(
                ee_pose_mat_B=ee_pose_mat_B,
                frame_id=len(self.stream) + len(batch),
                timestamp=time.time(),
            )

            elapsed = time.perf_counter() - t_start

            if frame is None:
                print(f"  [{i:02d}] 캡처 실패 (None) — 스킵  elapsed={elapsed:.3f}s")
            else:
                print(f"  [{i:02d}] frame_id={frame.frame_id}  "
                      f"pts={len(frame.points):>8,}  "
                      f"img={'있음' if frame.img is not None else '없음'}  "
                      f"elapsed={elapsed:.3f}s")
                batch.append(frame)

            if i < n - 1:
                remaining = target - elapsed
                if remaining > 0:
                    time.sleep(remaining)

        self.stream.extend(batch)
        print(f"[MMS] 캡처 완료: {len(batch)}/{n}  "
              f"(누적 {len(self.stream)} 프레임)")
        return batch

    # ── preprocessing ──────────────────────────────────────────────────────

    def preprocess(
        self,
        frames: Optional[Union[list[Frame], Stream]] = None,
        roi_bbox: Optional[tuple[float, float, float, float, float, float]] = None,
        voxel_size: float = 0.005,
        enable_voxel: bool = True,
        nb_neighbors: int = 20,
        std_ratio: float = 2.0,
        enable_denoise: bool = True,
        normal_radius: float = 0.01,
        normal_max_nn: int = 30,
        enable_normals: Optional[bool] = None,
    ) -> None:
        """
        Run preprocessing pipeline on frames.

        Pipeline: [roi_crop] → [voxel_downsample] → [denoise] → [estimate_normals]

        ROI crop은 roi_bbox가 주어지면 항상 실행된다.
        나머지 스텝은 enable_* 플래그로 개별 활성화/비활성화할 수 있다.

        Parameters
        ----------
        frames : list[Frame] or Stream, optional
            Frames to process. Defaults to self.stream (entire Stream buffer).
        roi_bbox : (min_x, max_x, min_y, max_y, min_z, max_z) or None
            Axis-aligned bounding box in base (B) frame (meters).
            Skipped if None.
        voxel_size : float, default=0.005
        enable_voxel : bool, default=True
            Voxel downsampling 실행 여부.
        nb_neighbors : int, default=20
        std_ratio : float, default=2.0
        enable_denoise : bool, default=True
            Statistical outlier removal 실행 여부.
        normal_radius : float, default=0.01
        normal_max_nn : int, default=30
        enable_normals : bool or None, default=None
            Normal estimation 실행 여부.
            None이면 cfg.orbbec가 설정된 경우 자동으로 True, 아니면 False.
        """
        if enable_normals is None:
            enable_normals = self.cfg.orbbec is not None

        targets = frames if frames is not None else self.stream

        print(f"[MMS] preprocess: {len(targets)} 프레임  "
              f"(voxel={enable_voxel}, denoise={enable_denoise}, normals={enable_normals})")

        total_start = time.perf_counter()

        for f in targets:
            before = len(f.points)
            t0 = time.perf_counter()

            if roi_bbox is not None:
                f.roi_crop(roi_bbox)
            after_roi = len(f.points)
            t_roi = time.perf_counter()

            if enable_voxel:
                f.voxel_downsample(voxel_size)
            after_vox = len(f.points)
            t_vox = time.perf_counter()

            if enable_denoise:
                f.denoise(nb_neighbors=nb_neighbors, std_ratio=std_ratio)
            after_dn = len(f.points)
            t_dn = time.perf_counter()

            if enable_normals:
                f.estimate_normals(radius=normal_radius, max_nn=normal_max_nn)
            t_norm = time.perf_counter()

            timing_parts = []
            if roi_bbox is not None:
                timing_parts.append(f"roi={t_roi - t0:.3f}s")
            if enable_voxel:
                timing_parts.append(f"voxel={t_vox - t_roi:.3f}s")
            if enable_denoise:
                timing_parts.append(f"denoise={t_dn - t_vox:.3f}s")
            if enable_normals:
                timing_parts.append(f"normals={t_norm - t_dn:.3f}s")
            timing_str = "  [" + "  ".join(timing_parts) + f"  total={t_norm - t0:.3f}s]"

            normals_count = len(f.normals) if f.normals is not None else 0
            print(f"  frame {f.frame_id}: {before:>8,}"
                  + (f" → roi {after_roi:>7,}" if roi_bbox else "")
                  + (f" → voxel {after_vox:>7,}" if enable_voxel else "")
                  + (f" → denoise {after_dn:>7,}" if enable_denoise else "")
                  + (f"  normals={normals_count:>7,}" if enable_normals else "")
                  + timing_str)

        total_elapsed = time.perf_counter() - total_start
        print(f"[MMS] preprocess 완료: {total_elapsed:.3f}s (전체 {len(targets)} 프레임)")

    # ── buffer management ──────────────────────────────────────────────────

    def clear_stream(self) -> None:
        """Clear the internal frame stream buffer."""
        self.stream.clear()
        print("[MMS] stream buffer cleared.")
