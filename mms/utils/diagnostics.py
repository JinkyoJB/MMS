# mms/utils/diagnostics.py
"""포인트 클라우드 진단 및 ROI 제안 유틸."""

import numpy as np

from mms.core.frames import Frame


def print_pcd_stats(frames: list[Frame]) -> None:
    """
    각 프레임의 포인트 클라우드 좌표 통계를 출력한다.

    Parameters
    ----------
    frames : list[Frame]
        통계를 출력할 프레임 리스트.
    """
    print("\n[Raw PCD 좌표 통계]")
    for f in frames:
        pts = f.points
        print(f"  frame {f.frame_id}: 포인트={len(pts):,}")
        if len(pts) == 0:
            print("    → 포인트 없음 (depth 스케일/연결 확인)")
            continue
        for ax, name in enumerate("xyz"):
            print(f"    {name}: {pts[:, ax].min():+.3f} ~ "
                  f"{pts[:, ax].max():+.3f}  mean={pts[:, ax].mean():+.3f}")


def suggest_roi(frames: list[Frame]) -> tuple:
    """
    2~98 percentile 기반으로 ROI bbox를 자동 제안한다.

    Parameters
    ----------
    frames : list[Frame]
        ROI를 계산할 프레임 리스트.

    Returns
    -------
    bbox : (min_x, max_x, min_y, max_y, min_z, max_z)
        자동 계산된 axis-aligned bounding box.
    """
    all_pts = [f.points for f in frames if len(f.points) > 0]
    if not all_pts:
        return (-1.0, 1.0, -1.0, 1.0, 0.1, 2.0)
    combined = np.concatenate(all_pts, axis=0)
    x0, x1 = np.percentile(combined[:, 0], [2, 98])
    y0, y1 = np.percentile(combined[:, 1], [2, 98])
    z0, z1 = np.percentile(combined[:, 2], [2, 98])
    return (x0, x1, y0, y1, max(z0, 0.01), z1)
