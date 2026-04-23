# mms/utils/visualization.py
"""Open3D 기반 포인트 클라우드 시각화 유틸."""

import numpy as np
import open3d as o3d

from mms.core.frames import Frame
from mms.utils.transforms import compute_T_CB

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


def visualize_hand_eye_calibration(
    frames: list[Frame],
    T_EC: np.ndarray,
    frame_size: float = 0.05,
    max_pts_per_frame: int = 100_000,
    title: str = "Hand-Eye Calibration Verification",
    # backwards-compatible parameter alias
    T_E_S: np.ndarray = None,
) -> None:
    """
    핸드-아이 캘리브레이션(T_EC) 검증 시각화.

    검증 원리
    ---------
    T_EC가 정확하면, 로봇이 어느 자세에 있든 동일한 물체를 B 프레임으로
    변환한 PCD가 서로 겹쳐야 한다.
    각 프레임의 PCD를 다른 색으로 오버레이했을 때:
      - 잘 겹침  → T_EC 정확
      - 어긋남   → T_EC에 오차 있음, 재캘리브레이션 필요

    표시 요소
    ---------
    큰 축    : B 프레임 (로봇 베이스 원점)
    중간 축  : 각 캡처 시점의 EE 자세 (T_EB)
    작은 축  : 각 캡처 시점의 Camera 자세 (T_CB)
    컬러 구체: EE 위치 마커 (프레임마다 다른 색)
    PCD      : 프레임마다 다른 색 → 겹칠수록 캘리브레이션 양호

    정량 지표
    ---------
    각 프레임 PCD의 무게중심(centroid)을 B 프레임에서 계산하여 편차를 출력한다.
      centroid 편차 < 5 mm  → 양호
      centroid 편차 > 10 mm → 재캘리브레이션 권장

    Parameters
    ----------
    frames : list[Frame]
        서로 다른 EE 자세에서 캡처한 프레임 목록 (최소 2개 권장).
        각 frame.ee_pose_mat_B에 캡처 시점의 T_EB가 기록되어 있어야 한다.
    T_EC : (4,4) np.ndarray
        E → C 변환 (핸드-아이 캘리브레이션 결과, config/calibration/hand_eye_phoxi.yaml).
    frame_size : float, default=0.05
        좌표계 축 표시 길이 (m).
    max_pts_per_frame : int, default=100_000
        프레임당 최대 렌더링 포인트 수.
    title : str
        Open3D 창 제목.
    """
    if T_E_S is not None and T_EC is None:
        T_EC = T_E_S  # backwards-compatible alias

    geoms: list = []
    centroids: list[np.ndarray] = []

    # ── B 프레임 축 (원점) ────────────────────────────────────────────────
    geoms.append(
        o3d.geometry.TriangleMesh.create_coordinate_frame(size=frame_size * 1.5)
    )

    # ── 헤더 출력 ─────────────────────────────────────────────────────────
    print(f"\n{'─'*80}")
    print(f"  Hand-Eye Calibration Verification  ({len(frames)} frames)")
    print(f"{'─'*80}")
    print(f"  {'#':>3}  {'EE origin (m)':^36}  {'PCD centroid (m)':^36}  {'pts':>8}")
    print(f"  {'─'*3}  {'─'*36}  {'─'*36}  {'─'*8}")

    for i, frame in enumerate(frames):
        color = FRAME_COLORS[i % len(FRAME_COLORS)]

        T_EB = frame.ee_pose_mat_B          # (4,4)  E → B
        T_CB = compute_T_CB(T_EB, T_EC)    # (4,4)  C → B

        # ── EE 자세 축 (중간 크기) ────────────────────────────────────────
        ee_axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=frame_size)
        ee_axes.transform(T_EB)
        geoms.append(ee_axes)

        # ── EE 위치 마커 구체 (프레임 색 식별용) ──────────────────────────
        sphere = o3d.geometry.TriangleMesh.create_sphere(radius=frame_size * 0.25)
        sphere.translate(T_EB[:3, 3])
        sphere.paint_uniform_color(color)
        sphere.compute_vertex_normals()
        geoms.append(sphere)

        # ── Camera 자세 축 (작은 크기) ────────────────────────────────────
        c_axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=frame_size * 0.5)
        c_axes.transform(T_CB)
        geoms.append(c_axes)

        # ── 포인트 클라우드 ───────────────────────────────────────────────
        pts = frame.points
        if len(pts) > max_pts_per_frame:
            idx = np.random.choice(len(pts), max_pts_per_frame, replace=False)
            pts = pts[idx]

        centroid = pts.mean(axis=0)
        centroids.append(centroid)

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pts.astype(np.float64))
        pcd.paint_uniform_color(color)
        geoms.append(pcd)

        ee_p = T_EB[:3, 3]
        print(f"  [{i:>2}]  "
              f"[{ee_p[0]:+.3f}, {ee_p[1]:+.3f}, {ee_p[2]:+.3f}]  "
              f"[{centroid[0]:+.3f}, {centroid[1]:+.3f}, {centroid[2]:+.3f}]  "
              f"{len(frame.points):>8,}")

    # ── 정량 지표: centroid 편차 ──────────────────────────────────────────
    print(f"  {'─'*80}")
    if len(centroids) >= 2:
        arr = np.array(centroids)               # (N, 3)
        mean_c = arr.mean(axis=0)
        deviations = np.linalg.norm(arr - mean_c, axis=1)  # (N,)

        max_dev_mm  = deviations.max()  * 1000
        mean_dev_mm = deviations.mean() * 1000

        verdict = "✔ 양호" if max_dev_mm < 5.0 else (
                  "△ 주의" if max_dev_mm < 10.0 else
                  "✘ 재캘리브레이션 권장")

        print(f"  평균 centroid     : [{mean_c[0]:+.4f}, {mean_c[1]:+.4f}, {mean_c[2]:+.4f}] m")
        print(f"  centroid 편차 max : {max_dev_mm:6.2f} mm")
        print(f"  centroid 편차 mean: {mean_dev_mm:6.2f} mm")
        print(f"  판정 : {verdict}  (기준: <5mm 양호 / <10mm 주의 / ≥10mm 재캘리브레이션)")
    else:
        print("  (정량 판정을 위해 최소 2개 프레임 필요)")

    print(f"\n  축 범례: 큰축=B(원점)  중간축=EE  작은축=S(센서)  "
          f"구체=EE위치  PCD색=프레임 구분  Q/ESC=닫기")
    print(f"{'─'*80}\n")

    o3d.visualization.draw_geometries(
        geoms,
        window_name=title,
        width=1280,
        height=720,
    )
