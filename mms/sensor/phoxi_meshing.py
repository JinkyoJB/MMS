# mms/sensor/phoxi_meshing.py
#
# PhoXi3D 포인트 클라우드 → 메쉬 변환 유틸리티.
#
# 두 가지 백엔드 제공:
#   1. open3d  : Screened Poisson (Python-native, 기본)
#   2. PoissonRecon.exe : Photoneo 3D Meshing 설치본 (subprocess)

from __future__ import annotations

import subprocess
import tempfile
from pathlib import Path
from typing import Optional

import numpy as np
import open3d as o3d

from mms.core.frames import Frame

# Photoneo 3DMeshing 설치 경로
_POISSON_EXE = Path(
    r"C:\Program Files\Photoneo\3DMeshing\2.3.0\PoissonRecon.exe"
)


# ------------------------------------------------------------------
# Backend 1: open3d  (기본)
# ------------------------------------------------------------------

def reconstruct_open3d(
    frame: Frame,
    depth: int = 9,
    scale: float = 1.1,
    linear_fit: bool = False,
    n_threads: int = -1,
) -> o3d.geometry.TriangleMesh:
    """
    open3d Screened Poisson Surface Reconstruction.

    Parameters
    ----------
    frame : Frame
        points / normals 이 있어야 함. normals 없으면 자동 추정.
    depth : int
        Octree depth (8~11 권장, 높을수록 세밀/느림).
    scale : float
        Bounding box 여유 배율.
    linear_fit : bool
        True → 선형 보간 iso-surface (메모리 절약).
    n_threads : int
        -1 = 전체 코어 사용.

    Returns
    -------
    o3d.geometry.TriangleMesh
    """
    if len(frame.points) == 0:
        raise ValueError("Frame has no points.")

    if frame.normals is None:
        frame.estimate_normals()

    pcd = frame.to_pcd()
    mesh, _ = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
        pcd,
        depth=depth,
        scale=scale,
        linear_fit=linear_fit,
        n_threads=n_threads,
    )
    return mesh


# ------------------------------------------------------------------
# Backend 2: PoissonRecon.exe  (Photoneo 3DMeshing)
# ------------------------------------------------------------------

def reconstruct_photoneo_exe(
    frame: Frame,
    depth: int = 9,
    scale: float = 1.1,
    exe_path: Path = _POISSON_EXE,
) -> o3d.geometry.TriangleMesh:
    """
    Photoneo의 PoissonRecon.exe 를 subprocess로 호출하여 메쉬 생성.

    포인트 클라우드를 임시 PLY로 저장 → PoissonRecon.exe 실행 →
    출력 PLY를 open3d TriangleMesh로 반환.

    Parameters
    ----------
    frame : Frame
    depth : int
        --depth 인자 (Octree depth, 기본 8~9).
    scale : float
        --scale 인자.
    exe_path : Path
        PoissonRecon.exe 경로.

    Returns
    -------
    o3d.geometry.TriangleMesh

    Raises
    ------
    FileNotFoundError
        PoissonRecon.exe 가 없을 때.
    RuntimeError
        PoissonRecon.exe 실행 실패 시.
    """
    if not exe_path.exists():
        raise FileNotFoundError(
            f"PoissonRecon.exe not found: {exe_path}\n"
            "Photoneo 3D Meshing 2.3.0 이 설치되어 있는지 확인하세요."
        )

    if len(frame.points) == 0:
        raise ValueError("Frame has no points.")

    if frame.normals is None:
        frame.estimate_normals()

    with tempfile.TemporaryDirectory() as tmpdir:
        tmp = Path(tmpdir)
        in_ply = tmp / "input.ply"
        out_ply = tmp / "output.ply"

        # PLY 저장 (points + normals)
        pcd = frame.to_pcd()
        o3d.io.write_point_cloud(str(in_ply), pcd, write_ascii=False)

        # PoissonRecon.exe 실행
        cmd = [
            str(exe_path),
            "--in", str(in_ply),
            "--out", str(out_ply),
            "--depth", str(depth),
            "--scale", str(scale),
        ]
        result = subprocess.run(cmd, capture_output=True, text=True)

        if result.returncode != 0:
            raise RuntimeError(
                f"PoissonRecon.exe failed (code={result.returncode})\n"
                f"stderr: {result.stderr}"
            )

        mesh = o3d.io.read_triangle_mesh(str(out_ply))

    return mesh


# ------------------------------------------------------------------
# 편의 함수: Frame.mesh 에 바로 저장
# ------------------------------------------------------------------

def add_mesh_to_frame(
    frame: Frame,
    backend: str = "open3d",
    depth: int = 9,
    scale: float = 1.1,
) -> None:
    """
    메쉬를 생성하여 frame.mesh 에 저장.

    Parameters
    ----------
    frame : Frame
    backend : str
        'open3d' (기본) 또는 'photoneo'
    depth : int
        Poisson octree depth.
    scale : float
        Bounding box 여유 배율.
    """
    if backend == "photoneo":
        frame.mesh = reconstruct_photoneo_exe(frame, depth=depth, scale=scale)
    else:
        frame.mesh = reconstruct_open3d(frame, depth=depth, scale=scale)

    verts = np.asarray(frame.mesh.vertices)
    tris = np.asarray(frame.mesh.triangles)
    print(
        f"[Meshing] backend={backend}  depth={depth}  "
        f"vertices={len(verts):,}  triangles={len(tris):,}"
    )


# ------------------------------------------------------------------
# 테스트
# ------------------------------------------------------------------
if __name__ == "__main__":
    import sys
    import time
    from pathlib import Path

    _PROJECT_ROOT = Path(__file__).resolve().parents[2]
    sys.path.insert(0, str(_PROJECT_ROOT))

    from mms.sensor.phoxi_client import PhoxiClient, PhoxiConfig

    cfg = PhoxiConfig(
        sensor_frames_yaml=str(_PROJECT_ROOT / "config" / "sensor_frames.yaml"),
        T_E_S_key="T_E_S_phoxi",
        serial_number="SEA-023",
        trigger_timeout_s=15.0,
    )
    client = PhoxiClient(cfg)

    try:
        client.initialize()
        frame = client.capture_frame(
            ee_pose_mat_B=np.eye(4),
            frame_id=0,
            timestamp=time.perf_counter(),
        )

        if frame is not None:
            print(f"Points: {frame.points.shape}")

            # open3d 백엔드 (기본)
            add_mesh_to_frame(frame, backend="open3d", depth=9)
            o3d.visualization.draw_geometries([frame.mesh], window_name="Mesh (open3d)")

            # Photoneo PoissonRecon.exe 백엔드 (선택)
            # add_mesh_to_frame(frame, backend="photoneo", depth=9)

    finally:
        client.shutdown()
