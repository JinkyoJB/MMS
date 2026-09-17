# mms/core/frames.py

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np
import open3d as o3d

from utils.transforms import pose_mat_to_6d


@dataclass
class Frame:
    """
    Unified sensor frame in base (B) coordinates.

    Convention: T_AB maps frame A to frame B, i.e. x_B = T_AB @ x_A.

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
        (4, 4) float64, T_EB — E-to-B transform at capture time.
        x_B = ee_pose_mat_B @ x_E

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
    mesh: Optional[o3d.geometry.TriangleMesh] = None

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

    # NOTE: 전처리 메서드(roi_crop, voxel_downsample, denoise, estimate_normals,
    #       reconstruct_mesh)는 lookaround 파이프라인에서 사용하지 않으므로 제거됨.
    #       lookaround 은 sensor._last_organized_pts 를 직접 사용하고,
    #       모든 cleanup 은 PcdAccumulateVolume 내부에서 처리한다.
    #       필요해지면 Open3D API 를 직접 호출하자 (pcd.voxel_down_sample 등).
