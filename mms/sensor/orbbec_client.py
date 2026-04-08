# mms/sensor/orbbec_client.py

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import cv2
import numpy as np

from pyorbbecsdk import (
    Context,
    Pipeline,
    Config,
    OBSensorType,
    OBStreamType,
    OBFormat,
    AlignFilter,
    PointCloudFilter,
    OBError,
)

from mms.core.frames import Frame
from mms.utils.transforms import load_transform, pose_mat_to_6d, compute_T_S_B, transform_points


@dataclass
class OrbbecConfig:
    """
    Orbbec Femto Bolt client configuration.

    target_interval_s
        Desired wall-clock time (seconds) per capture call.
        MMS.capture_frames() sleeps max(0, target - elapsed) after each
        capture so that successive frames are spaced this far apart.
        Set to 0.0 to capture as fast as possible.
        Femto Bolt (1920×1080, depth+color, PointCloudFilter) typically
        takes ~0.15 s per frame; 0.2 s gives a small headroom buffer.

    normal_radius, normal_max_nn
        Parameters for surface normal estimation performed at capture time.
        Normals are always computed so that every captured Frame is fully
        populated (img, depth, points, colors, normals).
    """
    sensor_frames_yaml: str      # path to config/sensor_frames.yaml
    T_E_S_key: str               # e.g. 'T_E_S_femto'
    enable_color: bool = True
    frame_timeout_ms: int = 2000  # wait_for_frames timeout (ms)
    target_interval_s: float = 0.2  # desired seconds per capture

    def __post_init__(self):
        self.sensor_frames_yaml = str(Path(self.sensor_frames_yaml).resolve())


class OrbbecClient:
    """
    Wraps Orbbec Femto Bolt (pyorbbecsdk) and produces Frame objects in base (B) frame.

    Notation
    --------
    T_A^B maps frame A to frame B:  x_B = T_A^B @ x_A

    Responsibilities
    ----------------
    - Initialize Femto pipeline (pyorbbecsdk).
    - Grab synchronized depth (+ color) frames.
    - Use AlignFilter to align depth to color, when color enabled.
    - Use PointCloudFilter to compute point cloud (XYZ or XYZRGB) in sensor frame.
    - Transform point cloud to base (B) frame using T_E^B and T_E^S.
    - Pack into mms.core.frames.Frame.
    """

    def __init__(self, cfg: OrbbecConfig):
        self.cfg = cfg
        self.T_E_S: np.ndarray = load_transform(cfg.sensor_frames_yaml, cfg.T_E_S_key)

        self._context: Optional[Context] = None
        self._pipeline: Optional[Pipeline] = None
        self._config: Optional[Config] = None
        self._align_filter: Optional[AlignFilter] = None
        self._pc_filter: Optional[PointCloudFilter] = None
        self._has_color: bool = False
        self._initialized: bool = False

    def initialize(self) -> None:
        """
        Initialize Orbbec SDK pipeline:
        - create Context, Pipeline, Config
        - enable DEPTH stream (required)
        - optionally enable COLOR stream (if cfg.enable_color and sensor supports it)
        - enable frame sync
        - start pipeline
        - create AlignFilter (align to COLOR if has_color else DEPTH)
        - create PointCloudFilter and set camera_param

        Raises
        ------
        RuntimeError
            If no suitable depth profile is found.
        """
        context = Context()
        pipeline = Pipeline()
        config = Config()

        # DEPTH (required)
        depth_profile_list = pipeline.get_stream_profile_list(OBSensorType.DEPTH_SENSOR)
        if depth_profile_list is None:
            raise RuntimeError("No depth profile available for Orbbec camera.")
        depth_profile = depth_profile_list.get_default_video_stream_profile()
        config.enable_stream(depth_profile)

        # COLOR (optional)
        has_color = False
        if self.cfg.enable_color:
            try:
                color_profile_list = pipeline.get_stream_profile_list(OBSensorType.COLOR_SENSOR)
                if color_profile_list is not None:
                    color_profile = color_profile_list.get_default_video_stream_profile()
                    config.enable_stream(color_profile)
                    has_color = True
            except OBError:
                has_color = False
        self._has_color = has_color

        pipeline.enable_frame_sync()
        pipeline.start(config)

        camera_param = pipeline.get_camera_param()
        align_to = OBStreamType.COLOR_STREAM if has_color else OBStreamType.DEPTH_STREAM
        align_filter = AlignFilter(align_to_stream=align_to)

        pc_filter = PointCloudFilter()
        pc_filter.set_camera_param(camera_param)

        # Warmup: wait until a valid synced frameset arrives
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

        self._context = context
        self._pipeline = pipeline
        self._config = config
        self._align_filter = align_filter
        self._pc_filter = pc_filter
        self._initialized = True

    def shutdown(self) -> None:
        """
        Stop pipeline and release resources.
        """
        if self._pipeline is not None:
            self._pipeline.stop()
        self._pipeline = None
        self._config = None
        self._align_filter = None
        self._pc_filter = None
        self._context = None
        self._initialized = False

    def capture_frame(
        self,
        ee_pose_mat_B: np.ndarray,
        frame_id: int,
        timestamp: float,
    ) -> Optional[Frame]:
        """
        Capture one frame from Orbbec Femto Bolt and return a unified Frame.

        Parameters
        ----------
        ee_pose_mat_B : (4,4) np.ndarray
            T_E^B — EE-to-Base transform at capture time (from xArm FK).
            x_B = ee_pose_mat_B @ x_E
        frame_id : int
        timestamp : float
            Monotonic or ROS time in seconds.

        Returns
        -------
        frame : Frame or None
            None if capture failed or depth frame not available.
        """
        if not self._initialized:
            raise RuntimeError("OrbbecClient not initialized. Call initialize() first.")

        import time as _time
        _t0 = _time.perf_counter()

        frames = self._pipeline.wait_for_frames(self.cfg.frame_timeout_ms)
        _t1 = _time.perf_counter()
        if frames is None:
            return None

        depth_frame = frames.get_depth_frame()
        if depth_frame is None:
            return None

        color_frame = frames.get_color_frame() if self._has_color else None
        if self._has_color and color_frame is None:
            return None

        # Align depth to color (or keep depth-only)
        frames_aligned = self._align_filter.process(frames)
        _t2 = _time.perf_counter()
        if frames_aligned is None:
            return None
        frames_aligned = frames_aligned.as_frame_set()

        depth_frame = frames_aligned.get_depth_frame()
        if depth_frame is None:
            return None

        # Depth scale for metric conversion
        depth_scale = depth_frame.get_depth_scale()
        self._pc_filter.set_position_data_scaled(depth_scale)

        # Decode depth image to (H, W) float32 in meters
        raw_depth: Optional[np.ndarray] = None
        try:
            h = depth_frame.get_height()
            w = depth_frame.get_width()
            depth_data = np.frombuffer(depth_frame.get_data(), dtype=np.uint16)
            raw_depth = depth_data.reshape((h, w)).astype(np.float32) * depth_scale
        except Exception:
            raw_depth = None
        _t3 = _time.perf_counter()

        # Decode color image if available
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
        _t4 = _time.perf_counter()

        # Compute point cloud in sensor frame S
        pc_frame = self._pc_filter.process(frames_aligned)
        if pc_frame is None:
            return None

        points_raw = np.asarray(self._pc_filter.calculate(pc_frame), dtype=np.float32)
        points_S = points_raw[:, :3]
        colors_S: Optional[np.ndarray] = (
            points_raw[:, 3:6] / 255.0 if self._has_color and points_raw.shape[1] >= 6
            else None
        )
        _t5 = _time.perf_counter()

        frame = self._build_frame_from_sensor_pcd(
            sensor_type="orbbec",
            raw_img=raw_img,
            raw_depth=raw_depth,
            points_S=points_S,
            colors_S=colors_S,
            ee_pose_mat_B=ee_pose_mat_B,
            T_E_S=self.T_E_S,
            frame_id=frame_id,
            timestamp=timestamp,
        )
        _t6 = _time.perf_counter()

        print(f"  [timing] wait={_t1-_t0:.3f}s  align={_t2-_t1:.3f}s  "
              f"depth_decode={_t3-_t2:.3f}s  color_decode={_t4-_t3:.3f}s  "
              f"pc_calc={_t5-_t4:.3f}s  build_frame={_t6-_t5:.3f}s  "
              f"total={_t6-_t0:.3f}s  pts={len(frame.points):,}")

        return frame

    @staticmethod
    def _build_frame_from_sensor_pcd(
        sensor_type: str,
        raw_img: Optional[np.ndarray],
        raw_depth: Optional[np.ndarray],
        points_S: np.ndarray,
        colors_S: Optional[np.ndarray],
        ee_pose_mat_B: np.ndarray,
        T_E_S: np.ndarray,
        frame_id: int,
        timestamp: float,
    ) -> Frame:
        """
        Convert sensor-frame point cloud to Frame in base (B) frame.

        Stores results as NumPy arrays only — no Open3D object created here.

        Notation: T_A^B maps A→B, i.e. x_B = T_A^B @ x_A

        Parameters
        ----------
        sensor_type : str
        raw_img : (H,W,3) uint8 or None
        raw_depth : (H,W) float32 or None
        points_S : (N,3) float32
            Point cloud in sensor frame S (includes invalid zero-depth points).
        colors_S : (N,3) float32 [0,1] or None
            Per-point RGB in sensor frame (same indexing as points_S).
        ee_pose_mat_B : (4,4) np.ndarray  T_E^B
        T_E_S : (4,4) np.ndarray  T_E^S
        frame_id : int
        timestamp : float
        """
        # T_S_B = T_E_B @ inv(T_E_S)  →  x_B = T_S_B @ x_S
        T_S_B = compute_T_S_B(ee_pose_mat_B, T_E_S)

        # Filter out zero-depth (invalid) points before transform
        valid = ~np.all(points_S == 0.0, axis=1)
        pts = points_S[valid]  # (M, 3) float32, M << N

        points_B = transform_points(T_S_B, pts).astype(np.float32)

        colors_B = colors_S[valid].astype(np.float32) if colors_S is not None else None

        return Frame(
            sensor_type=sensor_type,
            img=raw_img,
            depth=raw_depth,
            points=points_B,
            normals=None,
            colors=colors_B,
            frame_id=frame_id,
            timestamp=timestamp,
            ee_pose_mat_B=ee_pose_mat_B.astype(np.float64),
        )
