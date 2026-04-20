# mms/sensor/phoxi/phoxi_meshing.py
#
# PhoXi3D 포인트 클라우드 → 메쉬 변환 유틸리티.
#
# 백엔드:
#   1. open3d  : Screened Poisson (기본)
#   2. PoissonRecon.exe : Photoneo 3D Meshing

from __future__ import annotations

import subprocess
import tempfile
from pathlib import Path
from typing import Optional

import numpy as np
import open3d as o3d

_POISSON_EXE = Path(r"C:\Program Files\Photoneo\3DMeshing\2.3.0\PoissonRecon.exe")


def reconstruct_open3d(
    points: np.ndarray,
    normals: Optional[np.ndarray] = None,
    depth: int = 9,
    scale: float = 1.1,
    linear_fit: bool = False,
    n_threads: int = -1,
) -> o3d.geometry.TriangleMesh:
    """
    open3d Screened Poisson Surface Reconstruction.

    Parameters
    ----------
    points  : (N, 3) float32/64, 임의 단위 (mm or m).
    normals : (N, 3) float32/64 | None. 없으면 자동 추정.
    """
    if len(points) == 0:
        raise ValueError("points is empty.")

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points.astype(np.float64))

    if normals is not None and len(normals) == len(points):
        pcd.normals = o3d.utility.Vector3dVector(normals.astype(np.float64))
    else:
        pcd.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=10.0, max_nn=30)
        )
        pcd.normalize_normals()

    mesh, _ = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
        pcd, depth=depth, scale=scale, linear_fit=linear_fit, n_threads=n_threads,
    )
    return mesh


def reconstruct_photoneo_exe(
    points: np.ndarray,
    normals: Optional[np.ndarray] = None,
    depth: int = 9,
    scale: float = 1.1,
    exe_path: Path = _POISSON_EXE,
) -> o3d.geometry.TriangleMesh:
    """
    Photoneo PoissonRecon.exe 를 subprocess로 호출하여 메쉬 생성.
    """
    if not exe_path.exists():
        raise FileNotFoundError(f"PoissonRecon.exe not found: {exe_path}")
    if len(points) == 0:
        raise ValueError("points is empty.")

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points.astype(np.float64))
    if normals is not None and len(normals) == len(points):
        pcd.normals = o3d.utility.Vector3dVector(normals.astype(np.float64))
    else:
        pcd.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=10.0, max_nn=30)
        )

    with tempfile.TemporaryDirectory() as tmpdir:
        tmp = Path(tmpdir)
        in_ply  = tmp / "input.ply"
        out_ply = tmp / "output.ply"

        o3d.io.write_point_cloud(str(in_ply), pcd, write_ascii=False)

        cmd = [str(exe_path), "--in", str(in_ply), "--out", str(out_ply),
               "--depth", str(depth), "--scale", str(scale)]
        result = subprocess.run(cmd, capture_output=True, text=True)

        if result.returncode != 0:
            raise RuntimeError(
                f"PoissonRecon.exe failed (code={result.returncode})\n{result.stderr}"
            )
        mesh = o3d.io.read_triangle_mesh(str(out_ply))

    return mesh
