# mms/nbv/pcd_accumulate_volume.py
#
# "센서 원본 pcd 를 O 프레임에 누적 → 마지막에 Poisson" 백엔드.
#
# Phoxi/Artec 네이티브 알고리즘 이슈를 우회해 포즈(T_CO) 만 검증되면 mesh 가 나오는
# 가장 단순한 파이프라인. TSDFScanVolume 과 동일한 API 를 노출.
#
# 핵심
# ----
# 1. sensor organized Range (H,W,3) mm, C 프레임에서 invalid(0) + depth 범위 필터.
# 2. T_CO (C→O, meters) 로 점들을 O 프레임으로 옮겨 리스트에 append.
# 3. extract_mesh() 시:
#     - 모든 프레임 점군 vstack
#     - voxel_down_sample
#     - estimate_normals + orient_consistent
#     - Poisson surface reconstruction
#
# Notation: T_AB : A → B  (x_B = T_AB @ x_A)  — README.md / CLAUDE.md 준수.

from __future__ import annotations

from typing import List, Optional, TYPE_CHECKING

import numpy as np
import open3d as o3d

from mms_phoxi.core.frames import Frame

if TYPE_CHECKING:
    from mms_phoxi.sensor.phoxi_client import PhoxiClient


class PcdAccumulateVolume:
    """
    센서 organized pcd 를 O 프레임에 누적, 마지막에 Poisson 으로 mesh 생성.

    Parameters
    ----------
    sensor : PhoxiClient
        `_last_organized_pts` 읽기용.
    voxel_length : float, default=0.002 (m)
        voxel downsample 크기 — 누적 후 다운샘플.
    depth_trunc : float, default=2.0 (m)
        센서 depth 상한. 이보다 먼 점 제거.
    min_depth_m : float, default=0.05 (m)
        depth 하한.
    poisson_depth : int, default=9
        Poisson octree depth.
    poisson_density_quantile : float, default=0.01
        Poisson 결과에서 density 하위 q 분위 아래 vertex 제거 (외곽 artifact).
    orient_normals_k : int, default=15
        consistent tangent plane orientation k.
    color_fallback : bool, default=True
        Intensity 를 grayscale 색으로 사용.
    """

    def __init__(
        self,
        sensor: "PhoxiClient",
        voxel_length: float = 0.002,
        depth_trunc: float = 2.0,
        min_depth_m: float = 0.05,
        poisson_depth: int = 9,
        poisson_density_quantile: float = 0.0,    # ★ 기본 off — raw Poisson 유지
        orient_normals_k: int = 15,
        color_fallback: bool = True,
        intensity_auto_stretch: bool = True,      # 2-98 percentile 로 밝기 확장
        intensity_percentile: tuple = (2.0, 98.0),
        # cleanup — 기본 모두 off (센서 raw 품질 유지). 필요하면 명시적으로 on.
        sor_enabled: bool = False,                # ★ 기본 off
        sor_nb_neighbors: int = 30,
        sor_std_ratio: float = 2.0,
        remove_turntable_plane: bool = False,     # ★ 기본 off
        plane_z_search_max: float = 0.030,
        plane_distance_thresh: float = 0.003,
        plane_ransac_iters: int = 1000,
        **unused,
    ):
        self.sensor = sensor
        self.voxel_length = float(voxel_length)
        self.depth_trunc = float(depth_trunc)
        self.min_depth_m = float(min_depth_m)
        self.poisson_depth = int(poisson_depth)
        self.poisson_density_quantile = float(poisson_density_quantile)
        self.orient_normals_k = int(orient_normals_k)
        self.color_fallback = bool(color_fallback)
        self.intensity_auto_stretch = bool(intensity_auto_stretch)
        self.intensity_percentile = tuple(intensity_percentile)
        self.sor_enabled = bool(sor_enabled)
        self.sor_nb_neighbors = int(sor_nb_neighbors)
        self.sor_std_ratio = float(sor_std_ratio)
        self.remove_turntable_plane = bool(remove_turntable_plane)
        self.plane_z_search_max = float(plane_z_search_max)
        self.plane_distance_thresh = float(plane_distance_thresh)
        self.plane_ransac_iters = int(plane_ransac_iters)

        self._pts_O: List[np.ndarray] = []          # 각 프레임의 (N_i, 3) O-frame, meters
        self._cols: List[Optional[np.ndarray]] = [] # 각 프레임의 (N_i, 3) rgb [0,1] or None
        self._cached_merged_pcd: Optional[o3d.geometry.PointCloud] = None
        self._cached_dirty: bool = True

    # ── integrate ──────────────────────────────────────────────────────

    def integrate_frame(self, frame: Frame, T_CO: np.ndarray) -> None:
        """
        센서 organized pts → O 프레임으로 옮겨 누적.

        Parameters
        ----------
        frame : Frame (사용 X — sensor 가 organized 데이터 보유)
        T_CO  : (4,4) C → O, translation meters
        """
        organized = self.sensor._last_organized_pts
        if organized is None:
            raise RuntimeError("sensor._last_organized_pts 없음 — capture 누락?")

        flat_mm = np.ascontiguousarray(organized.reshape(-1, 3), dtype=np.float64)
        # invalid (0,0,0) 및 depth 범위 필터
        z = flat_mm[:, 2]
        valid = (
            (z > self.min_depth_m * 1000.0) &
            (z < self.depth_trunc * 1000.0) &
            np.isfinite(flat_mm).all(axis=1)
        )
        pts_C_m = flat_mm[valid] / 1000.0

        # 점 없으면 skip
        if len(pts_C_m) == 0:
            return

        # C → O 변환
        R = T_CO[:3, :3]
        t = T_CO[:3, 3]
        pts_O = pts_C_m @ R.T + t            # (N, 3) in O, meters

        # 색 — intensity (H, W) grayscale. 누락 프레임은 gray fallback 으로 항상 채움
        # (한 프레임이라도 None 이면 merged_pcd 전체에서 colors 가 사라지던 기존 버그 수정).
        col_valid: np.ndarray
        if self.color_fallback:
            intensity = getattr(self.sensor, "_last_intensity", None)
            if intensity is not None:
                intensity_flat = np.asarray(intensity).astype(np.float32).reshape(-1)
                if intensity_flat.size == flat_mm.shape[0]:
                    vi = intensity_flat[valid]
                    if self.intensity_auto_stretch and vi.size > 100:
                        lo = float(np.percentile(vi, float(self.intensity_percentile[0])))
                        hi = float(np.percentile(vi, float(self.intensity_percentile[1])))
                        if hi - lo > 1e-6:
                            g = np.clip((vi - lo) / (hi - lo), 0.0, 1.0).astype(np.float32)
                        else:
                            g = np.clip(vi / 255.0, 0.0, 1.0).astype(np.float32)
                    else:
                        g = np.clip(vi / 255.0, 0.0, 1.0).astype(np.float32)
                    col_valid = np.stack([g, g, g], axis=1).astype(np.float32)
                else:
                    col_valid = np.full((int(valid.sum()), 3), 0.5, dtype=np.float32)
            else:
                col_valid = np.full((int(valid.sum()), 3), 0.5, dtype=np.float32)
        else:
            col_valid = np.full((int(valid.sum()), 3), 0.5, dtype=np.float32)

        self._pts_O.append(pts_O.astype(np.float32))
        self._cols.append(col_valid)
        self._cached_dirty = True

    # ── 누적 pcd (voxel_down) ─────────────────────────────────────────

    def merged_pcd(
        self,
        voxel_m: Optional[float] = None,
        use_cache: bool = True,
    ) -> o3d.geometry.PointCloud:
        """
        O 프레임 기준 누적 pcd — voxel_down 까지만. 빠른 시각화용.
        """
        if use_cache and not self._cached_dirty and self._cached_merged_pcd is not None:
            return self._cached_merged_pcd

        if not self._pts_O:
            empty = o3d.geometry.PointCloud()
            return empty

        all_pts = np.vstack(self._pts_O)
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(all_pts.astype(np.float64))

        # 색 — 항상 포함 (integrate_frame 에서 누락 프레임도 gray fallback 으로 채움)
        valid_cols = [c for c in self._cols if c is not None]
        if valid_cols and len(valid_cols) == len(self._cols):
            all_cols = np.vstack(self._cols)
            if len(all_cols) == len(all_pts):
                pcd.colors = o3d.utility.Vector3dVector(all_cols.astype(np.float64))

        v = self.voxel_length if voxel_m is None else float(voxel_m)
        if v > 0:
            pcd = pcd.voxel_down_sample(v)

        self._cached_merged_pcd = pcd
        self._cached_dirty = False
        return pcd

    # ── 노이즈 / 턴테이블 평면 제거 ──────────────────────────────────

    def _remove_turntable_plane(
        self,
        pcd: o3d.geometry.PointCloud,
        verbose: bool = False,
    ) -> o3d.geometry.PointCloud:
        """
        z < plane_z_search_max 범위에서 RANSAC 으로 평면 fit → 그 평면 근방 점 제거.

        두 단계:
          1) z_O < `plane_z_search_max` 인 "턴테이블 후보 영역" 추출
          2) 거기서 `segment_plane` 으로 평면 모델 a·x + b·y + c·z + d = 0 획득
          3) 전체 pcd 에서 |a·x + b·y + c·z + d| < `plane_distance_thresh` 인 점 제거
        """
        if len(pcd.points) == 0:
            return pcd

        pts = np.asarray(pcd.points)
        cand_mask = pts[:, 2] < self.plane_z_search_max
        if int(cand_mask.sum()) < 200:
            if verbose:
                print("  [plane-remove] 후보 점 부족 — skip")
            return pcd

        cand_pcd = pcd.select_by_index(np.where(cand_mask)[0].tolist())
        try:
            plane_model, inliers = cand_pcd.segment_plane(
                distance_threshold=float(self.plane_distance_thresh),
                ransac_n=3,
                num_iterations=int(self.plane_ransac_iters),
            )
        except Exception as e:
            if verbose:
                print(f"  [plane-remove] segment_plane 실패: {e}")
            return pcd

        a, b, c, d = plane_model
        denom = float(np.sqrt(a * a + b * b + c * c))
        if denom < 1e-9:
            return pcd

        # 전체 점에 대해 signed distance 계산
        dist = np.abs(pts @ np.array([a, b, c]) + d) / denom
        keep_mask = dist > float(self.plane_distance_thresh)
        n_removed = int(pts.shape[0] - int(keep_mask.sum()))
        if verbose:
            print(f"  [plane-remove] plane: [{a:+.3f}, {b:+.3f}, {c:+.3f}]·p + {d:+.4f}  "
                  f"removed {n_removed:,} / {pts.shape[0]:,} pts")
        if n_removed == 0:
            return pcd
        return pcd.select_by_index(np.where(keep_mask)[0].tolist())

    def _denoise_sor(
        self,
        pcd: o3d.geometry.PointCloud,
        verbose: bool = False,
    ) -> o3d.geometry.PointCloud:
        if not self.sor_enabled:
            return pcd
        if len(pcd.points) < max(self.sor_nb_neighbors, 50):
            return pcd
        try:
            clean, ind = pcd.remove_statistical_outlier(
                nb_neighbors=self.sor_nb_neighbors,
                std_ratio=self.sor_std_ratio,
            )
            n_removed = len(pcd.points) - len(ind)
            if verbose:
                print(f"  [SOR] removed {n_removed:,} / {len(pcd.points):,}  "
                      f"(nb={self.sor_nb_neighbors}, std={self.sor_std_ratio})")
            return clean
        except Exception as e:
            if verbose:
                print(f"  [SOR] 실패: {e}")
            return pcd

    def _clean_pcd_for_mesh(
        self,
        pcd: o3d.geometry.PointCloud,
        verbose: bool = False,
    ) -> o3d.geometry.PointCloud:
        """Poisson 전 cleanup: plane 제거 + SOR."""
        if self.remove_turntable_plane:
            pcd = self._remove_turntable_plane(pcd, verbose=verbose)
        pcd = self._denoise_sor(pcd, verbose=verbose)
        return pcd

    # ── extract (Poisson) ──────────────────────────────────────────────

    def extract_mesh(self, verbose: bool = False) -> o3d.geometry.TriangleMesh:
        """
        누적 pcd → plane 제거 → SOR → normals → Poisson.
        """
        pcd = self.merged_pcd()
        if len(pcd.points) < 100:
            return o3d.geometry.TriangleMesh()

        # clean: 턴테이블 평면 제거 + SOR
        pcd = self._clean_pcd_for_mesh(pcd, verbose=verbose)
        if len(pcd.points) < 100:
            return o3d.geometry.TriangleMesh()

        # 노말 추정 + 일관성 있게 orient
        pcd.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(
                radius=self.voxel_length * 4.0, max_nn=30,
            )
        )
        try:
            pcd.orient_normals_consistent_tangent_plane(self.orient_normals_k)
        except Exception:
            # 이 단계에서 실패해도 Poisson 은 대부분 돌아감
            pass

        # Poisson
        mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
            pcd,
            depth=self.poisson_depth,
            scale=1.1,
            linear_fit=False,
        )
        if len(mesh.vertices) == 0:
            return mesh

        # density 하위 quantile 아래 vertex 제거 (외곽 확장 artifact)
        densities = np.asarray(densities)
        if self.poisson_density_quantile > 0 and len(densities) > 0:
            q = float(np.quantile(densities, self.poisson_density_quantile))
            mask_rm = densities < q
            if np.any(mask_rm):
                mesh.remove_vertices_by_mask(mask_rm.tolist())

        mesh.compute_vertex_normals()
        return mesh

    # ── status ─────────────────────────────────────────────────────────

    @property
    def n_integrated(self) -> int:
        return len(self._pts_O)

    def reset(self) -> None:
        self._pts_O.clear()
        self._cols.clear()
        self._cached_merged_pcd = None
        self._cached_dirty = True

    # ── 디스크 저장 ────────────────────────────────────────────────────

    def save_pcd(
        self,
        path: str,
        with_normals: bool = True,
    ) -> int:
        """
        누적된 pcd 를 `.ply` 로 저장 — Photoneo/MeshLab/CloudCompare 비교용.

        Returns
        -------
        int — 저장된 포인트 수
        """
        from pathlib import Path as _P
        pcd = self.merged_pcd()
        # 턴테이블 평면 제거 + SOR
        pcd = self._clean_pcd_for_mesh(pcd, verbose=True)
        if with_normals and not pcd.has_normals():
            pcd.estimate_normals(
                search_param=o3d.geometry.KDTreeSearchParamHybrid(
                    radius=max(self.voxel_length * 4.0, 0.008), max_nn=30,
                )
            )
            try:
                pcd.orient_normals_consistent_tangent_plane(self.orient_normals_k)
            except Exception:
                pass
        out = _P(path)
        out.parent.mkdir(parents=True, exist_ok=True)
        o3d.io.write_point_cloud(str(out), pcd, write_ascii=False)
        n = len(pcd.points)
        print(f"  [PcdAccumulate] saved {n:,} pts → {out}")
        return n

    # ── Photoneo PoissonRecon.exe 백엔드 ───────────────────────────────

    def extract_mesh_photoneo(
        self,
        depth: int = 9,
        scale: float = 1.1,
        verbose: bool = False,
    ) -> o3d.geometry.TriangleMesh:
        """
        Photoneo `PoissonRecon.exe` 를 호출해 메쉬 재구성 (비교용).
        """
        from mms_phoxi.sensor.phoxi_meshing import reconstruct_photoneo_exe

        pcd = self.merged_pcd()
        if len(pcd.points) < 100:
            return o3d.geometry.TriangleMesh()

        # clean: 턴테이블 평면 제거 + SOR (Open3D 백엔드와 공정 비교)
        pcd = self._clean_pcd_for_mesh(pcd, verbose=verbose)
        if len(pcd.points) < 100:
            return o3d.geometry.TriangleMesh()

        pcd.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(
                radius=self.voxel_length * 4.0, max_nn=30,
            )
        )
        try:
            pcd.orient_normals_consistent_tangent_plane(self.orient_normals_k)
        except Exception:
            pass

        pts = np.asarray(pcd.points)
        nms = np.asarray(pcd.normals) if pcd.has_normals() else None

        mesh = reconstruct_photoneo_exe(
            points=pts, normals=nms, depth=depth, scale=scale,
        )
        if len(mesh.vertices) == 0:
            return mesh

        if verbose:
            v = np.asarray(mesh.vertices)
            print(f"  [photoneo] verts={len(v):,}  z∈[{v[:,2].min()*1000:+.1f},"
                  f" {v[:,2].max()*1000:+.1f}]mm")
        mesh.compute_vertex_normals()
        return mesh
