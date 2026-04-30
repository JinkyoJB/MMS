# mms/nbv/tsdf_volume.py
#
# Open3D ScalableTSDFVolume 래퍼 (docs/2_control_layers.md §6).
#
# - 모든 프레임을 full-frame 으로 integrate (§3.2 결정: crop 은 mesh 단계에서).
# - Intrinsic 은 PhoxiClient.get_intrinsic() 로 받아 세션 동안 재사용.
# - Extrinsic 은 Open3D 규약 inv(T_CO) = T_OC.

from __future__ import annotations

from typing import Optional

import numpy as np
import open3d as o3d

from mms_phoxi.core.frames import Frame


class TSDFScanVolume:
    """
    Scalable TSDF 볼륨 래퍼.

    Parameters
    ----------
    intrinsic : o3d.camera.PinholeCameraIntrinsic
        세션 시작 시 PhoXi 에서 한 번 받아 고정.
    voxel_length : float, default=0.002 (Q5)
    sdf_trunc    : float, default=0.006  (= 3×voxel_length)
    depth_scale  : float, default=1.0    (Frame.depth 는 이미 meter)
    depth_trunc  : float, default=2.0    (m)
    """

    def __init__(
        self,
        intrinsic: o3d.camera.PinholeCameraIntrinsic,
        voxel_length: float = 0.002,
        sdf_trunc: float = 0.006,
        depth_scale: float = 1.0,
        depth_trunc: float = 2.0,
    ):
        self.intrinsic = intrinsic
        self.voxel_length = float(voxel_length)
        self.sdf_trunc = float(sdf_trunc)
        self.depth_scale = float(depth_scale)
        self.depth_trunc = float(depth_trunc)
        self._vol = o3d.pipelines.integration.ScalableTSDFVolume(
            voxel_length=self.voxel_length,
            sdf_trunc=self.sdf_trunc,
            color_type=o3d.pipelines.integration.TSDFVolumeColorType.NoColor,
        )
        self._n_integrated: int = 0

    # ── integrate ──────────────────────────────────────────────────────

    def integrate_frame(self, frame: Frame, T_CO: np.ndarray) -> None:
        """
        Full-frame integrate — crop 은 적용하지 않음.

        Parameters
        ----------
        frame : Frame
            `frame.depth` (H, W) float32, meter, 센서(C) 프레임 필수.
            `frame.img`   (H, W, 3) uint8 — 없으면 zeros 대체.
        T_CO : (4,4) np.ndarray
            C → O 변환. Open3D extrinsic = inv(T_CO) = T_OC.
        """
        if frame.depth is None:
            raise ValueError("Frame.depth 가 없습니다 — TSDF integrate 불가")
        if T_CO.shape != (4, 4):
            raise ValueError(f"T_CO shape must be (4,4), got {T_CO.shape}")

        depth_m = frame.depth.astype(np.float32)
        H, W = depth_m.shape

        if frame.img is not None and frame.img.shape[:2] == (H, W):
            color = frame.img.astype(np.uint8)
            if color.ndim == 2:
                color = np.stack([color] * 3, axis=-1)
        else:
            color = np.zeros((H, W, 3), dtype=np.uint8)

        o3d_color = o3d.geometry.Image(color)
        o3d_depth = o3d.geometry.Image(depth_m)
        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
            o3d_color, o3d_depth,
            depth_scale=self.depth_scale,
            depth_trunc=self.depth_trunc,
            convert_rgb_to_intensity=False,
        )

        extrinsic = np.linalg.inv(T_CO)    # T_OC
        self._vol.integrate(rgbd, self.intrinsic, extrinsic)
        self._n_integrated += 1

    # ── extract ────────────────────────────────────────────────────────

    def extract_mesh(self) -> o3d.geometry.TriangleMesh:
        mesh = self._vol.extract_triangle_mesh()
        mesh.compute_vertex_normals()
        return mesh

    # ── status ─────────────────────────────────────────────────────────

    @property
    def n_integrated(self) -> int:
        return self._n_integrated

    def reset(self) -> None:
        """Re-create empty volume (keep intrinsic)."""
        self._vol = o3d.pipelines.integration.ScalableTSDFVolume(
            voxel_length=self.voxel_length,
            sdf_trunc=self.sdf_trunc,
            color_type=o3d.pipelines.integration.TSDFVolumeColorType.NoColor,
        )
        self._n_integrated = 0
