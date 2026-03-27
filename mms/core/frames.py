# mms/core/frames.py

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np
import open3d as o3d

from mms.core.transforms import pose_mat_to_6d


@dataclass
class Frame:
    """
    Unified sensor frame in base (B) coordinates.

    Convention: T_A^B maps frame A -> frame B, i.e. x_B = T_A^B @ x_A.

    Attributes
    ----------
    sensor_type : str
        'orbbec' or 'phoxi'.
    img : np.ndarray | None
        (H, W, 3) uint8 RGB image, or None if not available.
    depth : np.ndarray | None
        (H, W) float32 depth in meters (sensor frame), or None.
    points : np.ndarray
        (N, 3) float32 point cloud in base (B) frame.
        Zero-depth / invalid points are pre-filtered at capture time.
    normals : np.ndarray | None
        (N, 3) float32 unit normals in base (B) frame.
        Populated by estimate_normals(). Invalidated by voxel_downsample().
    colors : np.ndarray | None
        (N, 3) float32 per-point RGB in [0, 1]. Available when sensor
        provides color (e.g. Orbbec with enable_color=True).
    frame_id : int
    timestamp : float
        Monotonic or ROS time in seconds.
    ee_pose_mat_B : np.ndarray
        (4, 4) float64, T_E^B — EE-to-Base transform at capture time.
        x_B = ee_pose_mat_B @ x_E

    권장 전처리 순서: roi_crop() → voxel_downsample() → denoise() → estimate_normals()
    """

    sensor_type: str
    img: Optional[np.ndarray]
    depth: Optional[np.ndarray]
    points: np.ndarray
    normals: Optional[np.ndarray]
    colors: Optional[np.ndarray]
    frame_id: int
    timestamp: float
    ee_pose_mat_B: np.ndarray

    def __post_init__(self) -> None:
        if self.ee_pose_mat_B.shape != (4, 4):
            raise ValueError(
                f"ee_pose_mat_B must be (4,4), got {self.ee_pose_mat_B.shape}"
            )
        if self.ee_pose_mat_B.dtype != np.float64:
            self.ee_pose_mat_B = self.ee_pose_mat_B.astype(np.float64)
        if self.points.dtype != np.float32:
            self.points = self.points.astype(np.float32)

    @property
    def ee_pose_6d_B(self) -> np.ndarray:
        """
        (x, y, z, rx, ry, rz) derived from ee_pose_mat_B.

        Returns
        -------
        np.ndarray, shape (6,)
            [x, y, z, r1, r2, r3] with xyz Euler angles in radians.
        """
        return pose_mat_to_6d(self.ee_pose_mat_B)

    # ----- Open3D interop -----

    def to_pcd(self) -> o3d.geometry.PointCloud:
        """
        Build an Open3D PointCloud from the current numpy state.

        Called at visualization time — not stored as a field.
        colors and normals are included if available.

        Returns
        -------
        o3d.geometry.PointCloud
        """
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(self.points)
        if self.colors is not None:
            pcd.colors = o3d.utility.Vector3dVector(self.colors)
        if self.normals is not None:
            pcd.normals = o3d.utility.Vector3dVector(self.normals)
        return pcd

    # ----- 전처리 메서드 -----

    def roi_crop(
        self,
        bbox_B: tuple[float, float, float, float, float, float],
    ) -> None:
        """
        ROI Crop (pure NumPy, no Open3D).

        Parameters
        ----------
        bbox_B : (min_x, max_x, min_y, max_y, min_z, max_z) in meters,
            axis-aligned 3D bounding box in base (B) frame.

        Effects
        -------
        Filters self.points (and self.normals / self.colors if present)
        in-place to keep only points inside the given 3D ROI.
        """
        min_x, max_x, min_y, max_y, min_z, max_z = bbox_B
        p = self.points
        mask = (
            (p[:, 0] >= min_x) & (p[:, 0] <= max_x) &
            (p[:, 1] >= min_y) & (p[:, 1] <= max_y) &
            (p[:, 2] >= min_z) & (p[:, 2] <= max_z)
        )
        self.points = self.points[mask]
        if self.normals is not None:
            self.normals = self.normals[mask]
        if self.colors is not None:
            self.colors = self.colors[mask]

    def voxel_downsample(self, voxel_size: float = 0.005) -> None:
        """
        Voxel grid downsampling.

        Builds a temporary Open3D PointCloud, downsamples, then extracts
        results back to numpy. self.normals is cleared (must re-estimate
        after downsampling).

        Parameters
        ----------
        voxel_size : float, default=0.005
            Voxel grid size in meters.

        Effects
        -------
        Updates self.points (and self.colors if present) in-place.
        Clears self.normals.
        """
        if len(self.points) == 0:
            return
        pcd = self.to_pcd()
        pcd_down = pcd.voxel_down_sample(voxel_size=voxel_size)
        self.points = np.asarray(pcd_down.points, dtype=np.float32)
        self.colors = (
            np.asarray(pcd_down.colors, dtype=np.float32)
            if pcd_down.has_colors() and self.colors is not None
            else None
        )
        self.normals = None  # invalidated by resampling

    def denoise(self,
                nb_neighbors: int = 20,
                std_ratio: float = 2.0) -> None:
        """
        Statistical outlier removal.

        Builds a temporary Open3D PointCloud for SOR, then applies the
        inlier index mask back to all numpy arrays.

        Parameters
        ----------
        nb_neighbors : int, default=20
            Number of nearest neighbors for local statistics.
        std_ratio : float, default=2.0
            Points with mean distance > (mean + std_ratio * std)
            are considered outliers.

        Effects
        -------
        Filters self.points / self.normals / self.colors in-place.
        """
        if len(self.points) == 0:
            return
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(self.points)
        _, ind = pcd.remove_statistical_outlier(
            nb_neighbors=nb_neighbors,
            std_ratio=std_ratio,
        )
        ind = np.asarray(ind)
        self.points = self.points[ind]
        if self.normals is not None:
            self.normals = self.normals[ind]
        if self.colors is not None:
            self.colors = self.colors[ind]

    def estimate_normals(self,
                         radius: float = 0.01,
                         max_nn: int = 30) -> None:
        """
        Estimate surface normals in B frame.

        Builds a temporary Open3D PointCloud, estimates normals, then
        stores results in self.normals as (N, 3) float32.

        Parameters
        ----------
        radius : float, default=0.01
            Search radius in meters.
        max_nn : int, default=30
            Maximum number of neighbors per point.

        Effects
        -------
        Populates self.normals with unit normals in base (B) frame.
        """
        if len(self.points) == 0:
            return
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(self.points)
        pcd.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(
                radius=radius,
                max_nn=max_nn,
            )
        )
        pcd.normalize_normals()
        self.normals = np.asarray(pcd.normals, dtype=np.float32)
