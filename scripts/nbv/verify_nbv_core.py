"""nbv Step 2/3 검증 — nbv_core 코어 (pcd→mesh→구멍검출→커버리지→NBV 포즈).

open3d 필요 (MMS 런타임 환경에서 실행).
실행:  python scripts/nbv/verify_nbv_core.py
구-구멍 합성 pcd 로 frontier 구멍 검출 / 커버리지 지표 / NBV 카메라 포즈 타당성 검증.
"""
import os
import sys
import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))

import open3d as o3d
from utils.nbv import nbv_core as p2


def sphere_pcd(center, radius, n=8000, hole_axis=None, hole_halfangle_deg=35.0, seed=0):
    rng = np.random.default_rng(seed)
    v = rng.normal(size=(n, 3))
    v /= np.linalg.norm(v, axis=1, keepdims=True)
    ax = None
    if hole_axis is not None:
        ax = np.asarray(hole_axis, float)
        ax /= np.linalg.norm(ax)
        v = v[(v @ ax) < np.cos(np.radians(hole_halfangle_deg))]   # 캡 제거 = 구멍
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(center + radius * v)
    pcd.normals = o3d.utility.Vector3dVector(v)
    return pcd, ax


def main():
    C = np.array([0.45, 0.0, 0.15])
    R = 0.05
    gap_kw = dict(min_seg_vertices=6, min_seg_length=0.005, max_seg_length=0.05)

    pcd_hole, ax = sphere_pcd(C, R, hole_axis=[0.3, 0.2, 1.0])
    mesh = p2.pcd_to_mesh_poisson(pcd_hole, depth=7, density_quantile=0.08)
    cov = p2.coverage_state(mesh, **gap_kw)
    gaps = p2.detect_gaps(mesh, **gap_kw)
    print(f"[구멍有] tris={len(mesh.triangles)} boundary={cov.boundary_len_m*1000:.1f}mm "
          f"cov={cov.angular_cov:.3f} gaps={cov.n_gaps}")
    assert len(mesh.triangles) > 100 and cov.n_gaps >= 1 and cov.boundary_len_m > 0.005

    biggest = max(gaps, key=lambda c: c.L)
    d = biggest.p_O - C
    d /= np.linalg.norm(d)
    print(f"         최대구멍 vs 실제구멍축 = {np.degrees(np.arccos(np.clip(d@ax,-1,1))):.1f}°")
    assert np.degrees(np.arccos(np.clip(d @ ax, -1, 1))) < 45.0

    T_CB = p2.nbv_pose_from_candidate(biggest, distance_m=0.225)
    to_surf = biggest.p_O - T_CB[:3, 3]
    assert abs(np.linalg.norm(to_surf) - 0.225) < 1e-3
    assert float(T_CB[:3, 2] @ (to_surf / np.linalg.norm(to_surf))) > 0.99
    print("[ok] 구멍 검출 + NBV 포즈 타당")

    pcd_full, _ = sphere_pcd(C, R, hole_axis=None)
    mesh_f = p2.pcd_to_mesh_poisson(pcd_full, depth=7, density_quantile=0.08)
    cov_f = p2.coverage_state(mesh_f, **gap_kw)
    print(f"[구멍無] boundary={cov_f.boundary_len_m*1000:.1f}mm cov={cov_f.angular_cov:.3f}")
    assert cov_f.angular_cov > cov.angular_cov
    assert cov_f.boundary_len_m < cov.boundary_len_m
    print("[ok] 커버리지 지표가 구멍 유무 구분")
    print("\n=== Step 2/3 통과 ===")


if __name__ == "__main__":
    main()
