"""
방법 1·2 — Open3D 기반 classic global registration.

  1. FPFH + RANSAC   (feature matching + RANSAC)
  2. Fast Global Registration (FGR)

공통: coarse voxel downsample → FPFH 특징 → global → 공통 ICP refine.
입력 src/ref 는 common.load_sessions() 의 미터-점군(색 포함 가능).
출력: T_global, T_final(ICP 후), runtime.
"""
from __future__ import annotations

import time
import numpy as np
import open3d as o3d

from scripts.artec.reg_benchmark.common import (
    FEATURE_VOXEL_M, NORMAL_RADIUS_M, icp_refine,
)


def _preprocess_for_feature(pcd: o3d.geometry.PointCloud, voxel: float):
    """coarse downsample + normal + FPFH."""
    d = pcd.voxel_down_sample(voxel)
    d.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(
        radius=voxel * 2.5, max_nn=30))
    fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        d, o3d.geometry.KDTreeSearchParamHybrid(radius=voxel * 5.0, max_nn=100))
    return d, fpfh


def fpfh_ransac(src, ref, voxel: float = FEATURE_VOXEL_M, seed: int = 0):
    """방법 1 — FPFH + RANSAC global → ICP refine."""
    t0 = time.perf_counter()
    s_d, s_f = _preprocess_for_feature(src, voxel)
    r_d, r_f = _preprocess_for_feature(ref, voxel)
    dist = voxel * 1.5
    res = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        s_d, r_d, s_f, r_f, mutual_filter=True, max_correspondence_distance=dist,
        estimation_method=o3d.pipelines.registration.
        TransformationEstimationPointToPoint(False),
        ransac_n=3,
        checkers=[
            o3d.pipelines.registration.
            CorrespondenceCheckerBasedOnEdgeLength(0.9),
            o3d.pipelines.registration.
            CorrespondenceCheckerBasedOnDistance(dist),
        ],
        criteria=o3d.pipelines.registration.RANSACConvergenceCriteria(
            4_000_000, 0.999),
    )
    T_global = res.transformation
    T_final = icp_refine(src, ref, T_global)
    return T_global, T_final, time.perf_counter() - t0


def fast_global(src, ref, voxel: float = FEATURE_VOXEL_M):
    """방법 2 — Fast Global Registration → ICP refine."""
    t0 = time.perf_counter()
    s_d, s_f = _preprocess_for_feature(src, voxel)
    r_d, r_f = _preprocess_for_feature(ref, voxel)
    dist = voxel * 1.5
    res = o3d.pipelines.registration.registration_fgr_based_on_feature_matching(
        s_d, r_d, s_f, r_f,
        o3d.pipelines.registration.FastGlobalRegistrationOption(
            maximum_correspondence_distance=dist))
    T_global = res.transformation
    T_final = icp_refine(src, ref, T_global)
    return T_global, T_final, time.perf_counter() - t0
