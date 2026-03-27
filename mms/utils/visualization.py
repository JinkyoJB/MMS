# mms/utils/visualization.py
"""Open3D 기반 포인트 클라우드 시각화 유틸."""

import numpy as np
import open3d as o3d

from mms.core.frames import Frame

FRAME_COLORS = [
    [1.0, 0.2, 0.2], [0.2, 0.8, 0.2], [0.2, 0.4, 1.0],
    [1.0, 0.8, 0.1], [1.0, 0.2, 1.0], [0.2, 1.0, 1.0],
    [1.0, 0.5, 0.1], [0.6, 0.2, 1.0], [0.3, 0.8, 0.5],
    [0.8, 0.8, 0.8],
]


def visualize(
    title: str,
    frames: list[Frame],
    max_pts_per_frame: int = 200_000,
) -> None:
    """
    프레임 리스트의 포인트 클라우드를 Open3D로 시각화한다.

    Open3D PointCloud는 이 함수 안에서만 생성된다 (Frame.to_pcd() 호출).
    포인트 수가 max_pts_per_frame을 초과하면 랜덤 서브샘플링한다.

    Parameters
    ----------
    title : str
        창 제목.
    frames : list[Frame]
        시각화할 프레임 리스트.
    max_pts_per_frame : int, default=200_000
        프레임당 최대 포인트 수 (초과 시 랜덤 샘플링).
    """
    geoms: list = [o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)]
    non_empty = 0

    for i, f in enumerate(frames):
        if len(f.points) == 0:
            continue

        pts = f.points
        colors = f.colors

        if len(pts) > max_pts_per_frame:
            idx = np.random.choice(len(pts), max_pts_per_frame, replace=False)
            pts = pts[idx]
            colors = colors[idx] if colors is not None else None

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pts)
        if colors is not None:
            pcd.colors = o3d.utility.Vector3dVector(colors)
        else:
            pcd.paint_uniform_color(FRAME_COLORS[i % len(FRAME_COLORS)])

        geoms.append(pcd)
        non_empty += 1

    if non_empty == 0:
        print(f"  [{title}] 시각화할 포인트 없음")
        return

    print(f"\n[시각화: {title}]  프레임={non_empty}  Q/ESC=닫기")
    o3d.visualization.draw_geometries(
        geoms, window_name=title, width=1280, height=720,
    )
