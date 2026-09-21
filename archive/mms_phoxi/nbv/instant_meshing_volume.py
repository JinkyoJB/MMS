# mms/nbv/instant_meshing_volume.py
#
# Photoneo PhoXiInstantMeshing 백엔드 래퍼 — `TSDFScanVolume` 와 **동일한 API** 를
# 노출해 ScanSession 이 backend 만 바꿔도 나머지 파이프라인이 그대로 동작하도록.
#
# 설계 메모
# --------
# - PhoxiInstantMeshing 은 전부 **mm 단위** 로 돌아간다 (voxel/depth/translation).
# - 입력 포인트는 센서(C) 프레임, **organized grid** (H×W×3) 원본 그대로.
# - 외부 pose 는 `T_CO` (C → O) 의 translation 만 mm 로 스케일해 전달.
# - 출력 mesh 는 "world 프레임" (여기선 O) + mm 단위 → meter 로 변환해 반환.
#
# 이유: MMS 파이프라인은 O 프레임 meter 기준이라 Open3D TSDF 래퍼와 호환.

from __future__ import annotations

import time
from typing import Optional, TYPE_CHECKING

import numpy as np
import open3d as o3d

from mms_phoxi.core.frames import Frame
from mms_phoxi.sensor.phoxi_instant_meshing import PhoxiInstantMeshingWrapper

if TYPE_CHECKING:
    from mms_phoxi.sensor.phoxi_client import PhoxiClient


class InstantMeshingScanVolume:
    """
    PhoxiInstantMeshing 기반 TSDF 볼륨 — API 는 `TSDFScanVolume` 와 동일.

    차이점
    ------
    - integrate_frame 이 `Frame.depth` + Open3D intrinsic 대신 **센서의
      organized (H,W,3) Range + Intensity** 원본을 직접 사용한다.
    - 따라서 constructor 에 `sensor` (PhoxiClient) 를 넘겨야 organized 데이터
      접근이 가능하다.

    Parameters
    ----------
    sensor : PhoxiClient
        `_last_organized_pts` / `_last_intensity` 를 읽기 위한 참조.
    voxel_length : float, default=0.002 (m)
        voxel 크기 — 내부적으로 mm 로 환산.
    depth_trunc : float, default=2.0 (m)
        max_depth — mm 로 환산.
    min_depth_m : float, default=0.05 (m)
    ortho_size_m : float, default=0.5 (m)
        PhoXi 직교 투영 볼륨 (width=height) — mm 로 환산.
    min_voxel_consensus : int, default=2
    max_gpu_memory_pct : int, default=85
    device_id : str, default="phoxi_0"
    """

    def __init__(
        self,
        sensor: "PhoxiClient",
        voxel_length: float = 0.002,
        depth_trunc: float = 2.0,
        min_depth_m: float = 0.05,
        ortho_size_m: float = 0.5,
        min_voxel_consensus: int = 2,
        max_gpu_memory_pct: int = 85,
        device_id: str = "phoxi_0",
        use_perspective: bool = True,
        **unused,                       # TSDFScanVolume 의 인자(intrinsic/sdf_trunc 등) 흡수
    ):
        self.sensor = sensor
        self.voxel_length = float(voxel_length)
        self.depth_trunc = float(depth_trunc)
        self.use_perspective = bool(use_perspective)

        # Perspective 모드 intrinsics — sensor 에서 organized Range 기반으로 fit.
        # capture() 가 최소 1회는 이미 실행됐어야 organized pts 가 있음.
        intr_tuple: Optional[tuple] = None
        if self.use_perspective:
            if not hasattr(sensor, "get_intrinsic"):
                raise RuntimeError(
                    "use_perspective=True 이면 sensor.get_intrinsic() 이 필요합니다."
                )
            intr = sensor.get_intrinsic()
            K = intr.intrinsic_matrix
            intr_tuple = (
                float(K[0, 0]), float(K[1, 1]), float(K[0, 2]), float(K[1, 2]),
            )

        self._wrapper = PhoxiInstantMeshingWrapper(
            device_id=device_id,
            voxel_size_mm=float(voxel_length) * 1000.0,
            min_voxel_consensus=int(min_voxel_consensus),
            max_gpu_memory_pct=int(max_gpu_memory_pct),
            min_depth_mm=float(min_depth_m) * 1000.0,
            max_depth_mm=float(depth_trunc) * 1000.0,
            ortho_width_mm=float(ortho_size_m) * 1000.0,
            ortho_height_mm=float(ortho_size_m) * 1000.0,
            tracking_enabled=False,
            orthogonal_projection=not self.use_perspective,
            camera_intrinsics=intr_tuple,
        )
        self._n_integrated: int = 0
        self._diag_printed: bool = False

    # ── integrate ──────────────────────────────────────────────────────

    def integrate_frame(self, frame: Frame, T_CO: np.ndarray) -> None:
        """
        센서 organized Range + Intensity 를 가져와 PhoxiInstantMeshing 에 스캔 추가.

        `T_CO` (meters) → `T_CO_mm` 로 translation 스케일 후 전달.
        `sensor._last_organized_pts` 가 이 시점에 갱신돼 있어야 함
        (caller 가 `capture()` 직후에 호출하는 걸 전제).
        """
        organized = self.sensor._last_organized_pts
        if organized is None:
            raise RuntimeError("sensor._last_organized_pts 가 없습니다 — capture 누락?")

        intensity = self.sensor._last_intensity
        H, W = organized.shape[:2]
        pts_flat = np.ascontiguousarray(
            organized.reshape(-1, 3), dtype=np.float32,
        )
        if intensity is None:
            tex_flat = np.zeros(H * W, dtype=np.float32)
        else:
            tex_flat = np.ascontiguousarray(
                intensity.astype(np.float32).reshape(-1),
            )

        # T_CO 는 meter 기반 — mm 로 스케일해서 전달
        T_CO_mm = np.asarray(T_CO, dtype=np.float64).copy()
        T_CO_mm[:3, 3] *= 1000.0

        if not self._diag_printed:
            # 첫 스캔 진단 로그 (데이터/pose 검증용)
            valid = ~np.all(pts_flat == 0, axis=1)
            v = pts_flat[valid]
            print(f"  [InstantMesh-diag] frame 해상도 {W}×{H}  "
                  f"valid pts = {int(valid.sum()):,}/{len(pts_flat):,}")
            if len(v) > 0:
                print(f"    points_C_mm  X∈[{v[:,0].min():+.1f}, {v[:,0].max():+.1f}]  "
                      f"Y∈[{v[:,1].min():+.1f}, {v[:,1].max():+.1f}]  "
                      f"Z∈[{v[:,2].min():+.1f}, {v[:,2].max():+.1f}] mm")
            t = T_CO_mm[:3, 3]
            print(f"    T_CO_mm translation = [{t[0]:+.1f}, {t[1]:+.1f}, {t[2]:+.1f}] mm")
            print(f"    T_CO_mm columns (C axes in O):")
            for c in range(3):
                v3 = T_CO_mm[:3, c]
                print(f"      col{c}: [{v3[0]:+.3f}, {v3[1]:+.3f}, {v3[2]:+.3f}]")
            self._diag_printed = True

        ok = self._wrapper.add_scan(
            points_S_flat=pts_flat,
            texture_flat=tex_flat,
            T_CB=T_CO_mm,                # 실제로는 "C → world" — world = O
            timestamp=float(time.perf_counter()),
            width=W,
            height=H,
        )
        if not ok:
            print("  [InstantMesh] ⚠ tracking lost — 이 스캔은 TSDF 에 누락될 수 있음")
        self._n_integrated += 1

    # ── extract ────────────────────────────────────────────────────────

    def extract_mesh(self) -> o3d.geometry.TriangleMesh:
        """
        PhoxiInstantMeshing 출력 (mm) → meter 스케일로 변환해 반환.
        """
        mesh_mm = self._wrapper.get_mesh()
        return _rescale_mesh_mm_to_m(mesh_mm)

    # ── status ─────────────────────────────────────────────────────────

    @property
    def n_integrated(self) -> int:
        return self._n_integrated

    def reset(self) -> None:
        self._wrapper.reset()
        self._n_integrated = 0

    def cleanup(self) -> None:
        self._wrapper.cleanup()


# ─────────────────────────────────────────────────────────────────────────────
# 유틸
# ─────────────────────────────────────────────────────────────────────────────

def _rescale_mesh_mm_to_m(mesh: o3d.geometry.TriangleMesh) -> o3d.geometry.TriangleMesh:
    if len(mesh.vertices) == 0:
        return mesh
    v = np.asarray(mesh.vertices, dtype=np.float64) / 1000.0
    mesh.vertices = o3d.utility.Vector3dVector(v)
    if not mesh.has_vertex_normals():
        mesh.compute_vertex_normals()
    return mesh
