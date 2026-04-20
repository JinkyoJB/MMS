# mms/system.py

from __future__ import annotations

import time
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Optional, Union

import numpy as np

from mms.core.frames import Frame
from mms.core.stream import Stream
from mms.sensor.scan_result import ScanResult
from mms.utils.transforms import (
    WorldTransformConfig,
    load_transform,
    compute_T_S_B,
    transform_points,
    transform_normals,
)
from mms.sensor.orbbec import OrbbecClient, OrbbecConfig
from mms.sensor.phoxi import PhoxiClient, PhoxiConfig
from mms.sensor.artec.artec_client import ArtecClient, ArtecConfig


@dataclass
class MMSConfig:
    """
    Top-level MMS configuration.

    센서는 orbbec / phoxi / artec 중 하나만 설정한다.

    Attributes
    ----------
    orbbec : OrbbecConfig, optional
        Orbbec Femto Bolt 센서 설정.
    phoxi : PhoxiConfig, optional
        Photoneo PhoXi 3D 센서 설정.
    artec : ArtecConfig, optional
        Artec 3D 스캐너 설정.
    sensor_frames_yaml : str, optional
        Path to config/sensor_frames.yaml.
        Required to load T_E_S for sensor→base frame conversion.
    T_E_S_key : str, optional
        Key in sensor_frames_yaml for the T_E_S transform of the active sensor.
        e.g. "T_E_S_femto", "T_E_S_phoxi", "T_E_S_artec".
        Required when sensor_frames_yaml is provided.
    object_frame_yaml : str, optional
        Path to config/object_frame.yaml (key: T_B_O0).
        If provided, MMS.world_transform is populated on init.
    stream_max_size : int, default=500
        Maximum number of frames retained in MMS.stream (Stream).
        Oldest frames are evicted automatically when the buffer is full.
    """
    orbbec: Optional[OrbbecConfig] = None
    phoxi: Optional[PhoxiConfig] = None
    artec: Optional[ArtecConfig] = None
    sensor_frames_yaml: Optional[str] = None
    T_E_S_key: Optional[str] = None
    object_frame_yaml: Optional[str] = None
    stream_max_size: int = 500

    def __post_init__(self):
        active = sum([
            self.orbbec is not None,
            self.phoxi is not None,
            self.artec is not None,
        ])
        if active == 0:
            raise ValueError("MMSConfig: orbbec, phoxi, artec 중 하나를 설정해야 합니다.")
        if active > 1:
            raise ValueError("MMSConfig: 센서는 하나만 설정할 수 있습니다.")

        if self.sensor_frames_yaml is not None:
            self.sensor_frames_yaml = str(Path(self.sensor_frames_yaml).resolve())
            if self.T_E_S_key is None:
                raise ValueError(
                    "MMSConfig: sensor_frames_yaml을 지정하면 T_E_S_key도 함께 지정해야 합니다."
                )
        if self.object_frame_yaml is not None:
            self.object_frame_yaml = str(Path(self.object_frame_yaml).resolve())


class MMS:
    """
    Top-level MMS (Multi Modal Scanning System) orchestrator.

    Manages sensor lifecycle, frame capture, and the sensor-to-base
    coordinate transform pipeline.

    Notation
    --------
    T_A_B maps frame A to frame B:  x_B = T_A_B @ x_A

    Sensor pipeline
    ---------------
    1. sensor.capture() → ScanResult   (S frame, mm)
    2. _scan_to_frame()                (S → B transform, mm → m)
        T_S_B = T_E_B @ inv(T_E_S)
        pts_B = T_S_B @ (pts_S / 1000)
    3. Frame                           (B frame, m)

    Attributes
    ----------
    world_transform : WorldTransformConfig or None
        Base <-> Object frame transforms. Populated when
        MMSConfig.object_frame_yaml is provided.

    Frame transform convenience methods
    ------------------------------------
    T_O_B(theta)      Object → Base at turntable angle theta (rad)
    T_B_O(theta)      Base → Object at turntable angle theta (rad)
    T_S_B(T_E_B)      Sensor → Base  (uses self._T_E_S loaded from yaml)

    Example
    -------
    >>> with MMS(cfg) as mms:
    ...     batch = mms.capture_frames(10)
    ...     mms.preprocess(batch, roi_bbox=(-0.5, 0.5, -0.5, 0.5, 0.1, 1.5))
    ...     x_B = mms.T_O_B(theta) @ x_O
    ...     x_O = mms.T_B_O(theta) @ x_B
    ...     T = mms.T_S_B(robot.get_ee_pose())
    """

    def __init__(self, cfg: MMSConfig) -> None:
        self.cfg = cfg

        # Sensor
        if cfg.phoxi is not None:
            self.sensor: OrbbecClient | PhoxiClient | ArtecClient = PhoxiClient(cfg.phoxi)
        elif cfg.artec is not None:
            self.sensor = ArtecClient(cfg.artec)
        else:
            self.sensor = OrbbecClient(cfg.orbbec)

        # T_E_S: EE → Sensor transform (hand-eye calibration result)
        if cfg.sensor_frames_yaml is not None:
            self._T_E_S: Optional[np.ndarray] = load_transform(
                cfg.sensor_frames_yaml, cfg.T_E_S_key
            )
        else:
            self._T_E_S = None

        self.stream: Stream = Stream(max_size=cfg.stream_max_size)

        # World transform (B <-> O)
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

    # ── sensor → frame conversion ──────────────────────────────────────────

    def _scan_to_frame(
        self,
        scan: ScanResult,
        ee_pose_mat_B: np.ndarray,
    ) -> Frame:
        """
        Convert a ScanResult (sensor frame, mm) to a Frame (base frame, m).

        Transform chain:
            T_S_B = T_E_B @ inv(T_E_S)
            pts_B = T_S_B @ (pts_S_mm / 1000)

        If T_E_S is not configured (sensor_frames_yaml not set),
        points are converted mm→m but remain in the sensor frame (no rotation).
        A warning is printed in this case.

        Parameters
        ----------
        scan : ScanResult
            Raw capture from sensor.capture() — S frame, mm.
        ee_pose_mat_B : (4,4) np.ndarray
            T_E_B — EE → Base transform at capture time (robot FK).
            Pass np.eye(4) when the robot is not connected.

        Returns
        -------
        Frame
            Points in base (B) frame, meters.
        """
        if self._T_E_S is not None:
            T_S_B = compute_T_S_B(ee_pose_mat_B, self._T_E_S)
        else:
            print("[MMS] ⚠ T_E_S 미설정 (sensor_frames_yaml 없음). "
                  "포인트가 센서(S) 프레임 기준으로 유지됩니다.")
            T_S_B = ee_pose_mat_B  # identity T_E_S assumed

        pts_m = (scan.points / 1000.0).astype(np.float64)
        pts_B = transform_points(T_S_B, pts_m).astype(np.float32)

        normals_B: Optional[np.ndarray] = None
        if scan.normals is not None:
            normals_B = transform_normals(
                T_S_B, scan.normals.astype(np.float64)
            ).astype(np.float32)

        depth_m = scan.depth / 1000.0 if scan.depth is not None else None

        return Frame(
            sensor_type=scan.sensor_type,
            img=scan.img,
            depth=depth_m,
            points=pts_B,
            normals=normals_B,
            colors=scan.colors,
            frame_id=scan.frame_id,
            timestamp=scan.timestamp,
            ee_pose_mat_B=ee_pose_mat_B.astype(np.float64),
        )

    # ── capture ────────────────────────────────────────────────────────────

    def capture_frames(
        self,
        n: int,
        ee_pose_fn: Optional[Callable[[], np.ndarray]] = None,
    ) -> list[Frame]:
        """
        Capture n frames from the sensor and append them to self.stream.

        Each ScanResult (S frame, mm) is converted to a Frame (B frame, m)
        via _scan_to_frame() using T_E_S loaded from sensor_frames_yaml.

        Frame pacing is controlled by sensor config target_interval_s:
        after each capture, sleeps max(0, target - elapsed) so that
        successive frames are spaced target_interval_s apart.

        Parameters
        ----------
        n : int
            Number of frames to capture.
        ee_pose_fn : () -> (4,4) np.ndarray, optional
            Callable that returns the current T_E_B from the robot.
            If None, identity matrix is used (robot not connected).

        Returns
        -------
        batch : list[Frame]
            Newly captured frames (also appended to self.stream Stream).
        """
        target = self.sensor.cfg.target_interval_s
        batch: list[Frame] = []

        print(f"[MMS] capture_frames: {n} 프레임 수집  "
              f"(target_interval={target:.3f}s)")
        for i in range(n):
            ee_pose_mat_B = (
                ee_pose_fn() if ee_pose_fn is not None
                else np.eye(4, dtype=np.float64)
            )

            t_start = time.perf_counter()
            frame_id = len(self.stream) + len(batch)

            scan: Optional[ScanResult] = self.sensor.capture(
                frame_id=frame_id,
                timestamp=time.time(),
            )

            elapsed = time.perf_counter() - t_start

            if scan is None:
                print(f"  [{i:02d}] 캡처 실패 (None) — 스킵  elapsed={elapsed:.3f}s")
            else:
                frame = self._scan_to_frame(scan, ee_pose_mat_B)
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
            None이면 True로 동작.
        """
        if enable_normals is None:
            enable_normals = True

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

    # ── frame transforms ───────────────────────────────────────────────────

    def T_O_B(self, theta: float) -> np.ndarray:
        """
        Object → Base transform at turntable angle theta (rad).

        x_B = mms.T_O_B(theta) @ x_O

        Requires MMSConfig.object_frame_yaml to be set.
        """
        if self.world_transform is None:
            raise RuntimeError(
                "world_transform not configured. Provide object_frame_yaml in MMSConfig."
            )
        return self.world_transform.T_O_B(theta)

    def T_B_O(self, theta: float) -> np.ndarray:
        """
        Base → Object transform at turntable angle theta (rad).

        x_O = mms.T_B_O(theta) @ x_B

        Requires MMSConfig.object_frame_yaml to be set.
        """
        if self.world_transform is None:
            raise RuntimeError(
                "world_transform not configured. Provide object_frame_yaml in MMSConfig."
            )
        return self.world_transform.T_B_O(theta)

    def T_S_B(self, T_E_B: np.ndarray) -> np.ndarray:
        """
        Sensor → Base transform.

        T_S_B = T_E_B @ inv(T_E_S)
        x_B   = mms.T_S_B(T_E_B) @ x_S

        Uses self._T_E_S loaded from MMSConfig.sensor_frames_yaml.

        Parameters
        ----------
        T_E_B : (4,4) np.ndarray
            EE → Base transform at capture time (from robot FK).
        """
        if self._T_E_S is None:
            raise RuntimeError(
                "T_E_S not configured. Provide sensor_frames_yaml and T_E_S_key in MMSConfig."
            )
        return compute_T_S_B(T_E_B, self._T_E_S)

    # ── visualization / diagnostics ────────────────────────────────────────

    def visualize_with_object_frame(
        self,
        frames: Union[list[Frame], "Stream"],
        theta: float = 0.0,
        frame_size: float = 0.1,
        max_pts: int = 500_000,
        title: str = "Frame Transform Verification",
    ) -> None:
        """
        Captured point cloud에 Base(B)·Object(O) 좌표계 축을 겹쳐 시각화한다.

        프레임 변환이 올바른지 눈으로 확인하는 용도:
        - O 축 원점이 턴테이블 중심에 위치하는지
        - O 축 z축이 위쪽을 가리키는지
        - 포인트 클라우드가 올바른 위치에 있는지

        좌표계 표시 색상 (Open3D 기본)
        --------------------------------
        빨간축(X)  초록축(Y)  파란축(Z)

        Parameters
        ----------
        frames : list[Frame] or Stream
            시각화할 프레임. points는 Base(B) 프레임 기준이어야 한다.
        theta : float, default=0.0
            현재 턴테이블 각도 (rad). T_O_B(theta)로 Object 축 위치를 결정한다.
        frame_size : float, default=0.1
            좌표계 축 표시 길이 (m).
        max_pts : int, default=500_000
            렌더링할 최대 포인트 수. 초과 시 랜덤 서브샘플링.
        title : str
            Open3D 창 제목.
        """
        import open3d as o3d
        from scipy.spatial.transform import Rotation as R

        frames_list = list(frames)
        geoms = []

        # ── 로봇 미연결 경고 ──────────────────────────────────────────────
        _identity = np.eye(4)
        no_robot = all(
            np.allclose(f.ee_pose_mat_B, _identity) for f in frames_list
        )
        if no_robot:
            print("[MMS] ⚠ T_E_B = identity (로봇 미연결). "
                  "포인트 클라우드가 EE 프레임 기준으로 표시됩니다. "
                  "B·O 축과 일치하지 않을 수 있습니다.")

        # ── Base(B) 좌표계 (원점) ──────────────────────────────────────────
        base_axes = o3d.geometry.TriangleMesh.create_coordinate_frame(
            size=frame_size, origin=[0, 0, 0]
        )
        geoms.append(base_axes)

        # ── Object(O) 좌표계 ──────────────────────────────────────────────
        if self.world_transform is not None:
            T_obj = self.T_O_B(theta)
            obj_axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=frame_size)
            obj_axes.transform(T_obj)
            geoms.append(obj_axes)

            origin_B = T_obj[:3, 3]
            rpy_deg = R.from_matrix(T_obj[:3, :3]).as_euler("xyz", degrees=True)
            print(f"[MMS] Object(O) frame  theta={np.degrees(theta):.1f}°")
            print(f"  origin in B : [{origin_B[0]:.4f}, {origin_B[1]:.4f}, {origin_B[2]:.4f}] m")
            print(f"  RPY in B    : [{rpy_deg[0]:.2f}, {rpy_deg[1]:.2f}, {rpy_deg[2]:.2f}] deg")
        else:
            print("[MMS] world_transform 미설정 — Object 축 표시 생략.")

        # ── Sensor(S) 좌표계 (첫 프레임 기준) ────────────────────────────
        if frames_list and self._T_E_S is not None:
            T_E_B_0 = frames_list[0].ee_pose_mat_B
            T_S_B_0 = compute_T_S_B(T_E_B_0, self._T_E_S)
            sensor_axes = o3d.geometry.TriangleMesh.create_coordinate_frame(
                size=frame_size * 0.6
            )
            sensor_axes.transform(T_S_B_0)
            geoms.append(sensor_axes)

            s_origin = T_S_B_0[:3, 3]
            print(f"[MMS] Sensor(S) frame (frame 0 기준)")
            print(f"  origin in B : [{s_origin[0]:.4f}, {s_origin[1]:.4f}, {s_origin[2]:.4f}] m")

        # ── 포인트 클라우드 ────────────────────────────────────────────────
        _COLORS = [
            [0.85, 0.30, 0.25],
            [0.25, 0.65, 0.30],
            [0.25, 0.45, 0.85],
            [0.85, 0.70, 0.15],
            [0.65, 0.25, 0.80],
        ]

        total_pts = sum(len(f.points) for f in frames_list)
        ratio = min(1.0, max_pts / max(total_pts, 1))

        for i, frame in enumerate(frames_list):
            pts = frame.points
            cols = frame.colors

            if ratio < 1.0:
                idx = np.random.choice(len(pts), int(len(pts) * ratio), replace=False)
                pts = pts[idx]
                cols = cols[idx] if cols is not None else None

            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(pts.astype(np.float64))
            if cols is not None:
                pcd.colors = o3d.utility.Vector3dVector(cols.astype(np.float64))
            else:
                pcd.paint_uniform_color(_COLORS[i % len(_COLORS)])
            geoms.append(pcd)

        print(f"[MMS] visualize_with_object_frame: {len(frames_list)} frame(s)  "
              f"total_pts={total_pts:,}  rendered≤{max_pts:,}")
        print("  축 범례: B=큰축(원점)  O=큰축(턴테이블)  S=작은축(센서)  "
              "빨간=X  초록=Y  파란=Z")

        o3d.visualization.draw_geometries(
            geoms,
            window_name=title,
            width=1280,
            height=720,
        )
