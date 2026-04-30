# mms/nbv/manual_picker.py
#
# Upper-layer (3.1 NBV / 재구성 레이어) placeholder.
#
# 진짜 NBV 알고리즘을 붙이기 전, 사용자가 Open3D 창에서 직접 표면을
# picking 하게 하고 → 해당 로컬 노말을 따라 distance_m 떨어진 위치에
# 카메라 목표 포즈(6DoF) 를 생성한다. 카메라의 roll 각도는 키보드로 입력받는다.
#
# Frame convention (README.md)
# ----------------------------
# 이 모듈은 "하나의 프레임 좌표계" 기준으로만 동작한다. 입력 points/normals가
# 어떤 프레임에 있든 그 프레임 기준으로 camera 목표 포즈를 반환한다.
#
# 예) O 프레임 기준 입력 → 반환값은 T_CO_des (C → O).
#     B 프레임 기준 입력 → 반환값은 T_CB_des (C → B).
#
# Camera convention (OpenCV/PhoXi 스타일)
# ---------------------------------------
# x_cam = right,  y_cam = down,  z_cam = forward (광축, scene 쪽)
#
# 즉 camera는 선택된 표면을 바라본다:
#     p_cam     = p_surface + distance * n_out    (n_out: 표면 바깥 노말)
#     z_cam     = -n_out                           (광축이 표면을 향함)
#     x_cam, y_cam = world-up 기준으로 결정 후 roll만큼 z_cam 축 회전

from __future__ import annotations

from typing import Optional

import numpy as np

from utils.transforms import rotz


# ─────────────────────────────────────────────────────────────────────────────
# 수학: normal + roll → 카메라 포즈
# ─────────────────────────────────────────────────────────────────────────────

def compute_camera_pose_from_normal(
    surface_point: np.ndarray,
    normal: np.ndarray,
    distance_m: float,
    roll_rad: float,
    world_up: np.ndarray = np.array([0.0, 0.0, 1.0]),
) -> np.ndarray:
    """
    표면 점 + 바깥 노말 + roll 각도로부터 카메라 목표 포즈 T_CX 를 생성한다.

    입력 좌표계 X (예: O 또는 B) 기준 4×4 SE3 를 반환한다.
    반환값 T_CX 는 C → X 변환이며, T_CX[:3, 3] 은 X 프레임에서의 카메라 원점이다.

    카메라 규약: OpenCV — x=right, y=down, z=forward(광축).

    Parameters
    ----------
    surface_point : (3,) np.ndarray   표면 선택 점 (X 프레임, m)
    normal        : (3,) np.ndarray   표면 바깥 노말 (X 프레임, 단위벡터 권장)
    distance_m    : float             표면에서 카메라까지 거리 (m). PhoXi 권장: 0.384
    roll_rad      : float             카메라 z축(광축) 기준 roll (rad)
    world_up      : (3,) np.ndarray   X 프레임에서의 "위쪽" 방향. 기본 [0,0,1].

    Returns
    -------
    T_CX : (4,4) np.ndarray  —  C → X
    """
    n = np.asarray(normal, dtype=float).reshape(3)
    norm = np.linalg.norm(n)
    if norm < 1e-9:
        raise ValueError("normal 의 크기가 0 입니다.")
    n = n / norm

    z_cam = -n

    up = np.asarray(world_up, dtype=float).reshape(3)
    up = up / np.linalg.norm(up)
    if abs(float(z_cam @ up)) > 0.95:
        up = np.array([0.0, 1.0, 0.0])
        if abs(float(z_cam @ up)) > 0.95:
            up = np.array([1.0, 0.0, 0.0])

    right = np.cross(up, z_cam)
    right = right / np.linalg.norm(right)
    down = np.cross(z_cam, right)

    R = np.column_stack([right, down, z_cam])
    R = R @ rotz(roll_rad)

    p_cam = np.asarray(surface_point, dtype=float).reshape(3) + distance_m * n

    T = np.eye(4, dtype=float)
    T[:3, :3] = R
    T[:3, 3] = p_cam
    return T


# ─────────────────────────────────────────────────────────────────────────────
# 시각화 + picking
# ─────────────────────────────────────────────────────────────────────────────

def _make_arrow(
    origin: np.ndarray,
    direction: np.ndarray,
    length: float,
    color: tuple[float, float, float],
    shaft_radius_ratio: float = 0.02,
):
    """표면 노말 표시용 arrow mesh (origin → origin + length*direction)."""
    import open3d as o3d
    from scipy.spatial.transform import Rotation as R

    shaft_radius = max(length * shaft_radius_ratio, 1e-4)
    cone_radius = shaft_radius * 2.0
    cylinder_h = length * 0.8
    cone_h = length * 0.2

    arrow = o3d.geometry.TriangleMesh.create_arrow(
        cylinder_radius=shaft_radius,
        cone_radius=cone_radius,
        cylinder_height=cylinder_h,
        cone_height=cone_h,
    )
    arrow.paint_uniform_color(color)
    arrow.compute_vertex_normals()

    d = np.asarray(direction, dtype=float).reshape(3)
    d = d / (np.linalg.norm(d) + 1e-12)
    z = np.array([0.0, 0.0, 1.0])
    axis = np.cross(z, d)
    s = np.linalg.norm(axis)
    c = float(z @ d)
    if s < 1e-9:
        rot = np.eye(3) if c > 0 else R.from_rotvec(np.pi * np.array([1.0, 0.0, 0.0])).as_matrix()
    else:
        axis = axis / s
        angle = float(np.arctan2(s, c))
        rot = R.from_rotvec(axis * angle).as_matrix()

    T = np.eye(4)
    T[:3, :3] = rot
    T[:3, 3] = np.asarray(origin, dtype=float).reshape(3)
    arrow.transform(T)
    return arrow


def pick_camera_target_in_frame(
    points: np.ndarray,
    normals: Optional[np.ndarray] = None,
    colors: Optional[np.ndarray] = None,
    distance_m: float = 0.384,
    knn: int = 30,
    world_up: np.ndarray = np.array([0.0, 0.0, 1.0]),
    frame_label: str = "O",
    preview: bool = True,
    default_roll_deg: float = 0.0,
) -> np.ndarray:
    """
    Open3D 창에서 사용자가 표면 포인트를 pick → 로컬 노말 기준 카메라 타겟 포즈 반환.

    Notation
    --------
    입력 points/normals 는 임의의 단일 좌표계 X (예: O 또는 B) 기준.
    반환값은 `T_CX_des` (C → X) 이며 frame_label 로 문서화 용도의 라벨만 제공한다.

    Workflow
    --------
    1. VisualizerWithEditing 창 — Shift+좌클릭으로 표면 포인트 선택, Q로 종료
    2. 선택된 인덱스들에 대해 평균 위치/노말 계산 (kNN 로 노말 재추정)
    3. 터미널 프롬프트 — 노말 뒤집기 여부 + 카메라 roll (deg)
    4. (preview=True) 결과 확인용 창 — 선택 점/노말/카메라 축 중첩 표시
    5. 4×4 T_CX_des 반환

    Parameters
    ----------
    points : (N,3) np.ndarray
    normals : (N,3) np.ndarray, optional
        없으면 pcd.estimate_normals() 로 추정.
    colors : (N,3) np.ndarray, optional
        [0,1] 범위 RGB.
    distance_m : float, default=0.384
        표면에서 카메라까지 거리 (m). PhoXi 최적거리.
    knn : int, default=30
        선택된 점 주변 k-NN 으로 로컬 노말 refine.
    world_up : (3,), default=[0,0,1]
        입력 프레임에서의 world-up 방향.
    frame_label : str, default="O"
        로그/창 제목용 라벨.
    preview : bool, default=True
        True 면 결과 확인용 두 번째 창 표시.

    Returns
    -------
    T_CX_des : (4,4) np.ndarray
    """
    import open3d as o3d

    pts = np.asarray(points, dtype=np.float64)
    if pts.ndim != 2 or pts.shape[1] != 3:
        raise ValueError(f"points must be (N,3), got {pts.shape}")
    if len(pts) == 0:
        raise ValueError("points is empty.")

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pts)
    if colors is not None:
        pcd.colors = o3d.utility.Vector3dVector(np.asarray(colors, dtype=np.float64))
    if normals is not None and len(normals) == len(pts):
        pcd.normals = o3d.utility.Vector3dVector(np.asarray(normals, dtype=np.float64))
    else:
        pcd.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.01, max_nn=30)
        )
        pcd.normalize_normals()

    print("\n" + "─" * 72)
    print(f"[NBV] 표면 선택 — 프레임: {frame_label}")
    print("─" * 72)
    print("  · Shift + 좌클릭 으로 표면 포인트를 1개 이상 선택")
    print("  · Shift + 우클릭 으로 선택 취소")
    print("  · Q 또는 창 닫기 로 선택 종료")
    print("─" * 72)

    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window(
        window_name=f"NBV pick (frame={frame_label}) — Shift+LMB pick, Q close",
        width=1280, height=720,
    )
    vis.add_geometry(pcd)
    vis.run()
    vis.destroy_window()

    picked = vis.get_picked_points()
    if not picked:
        raise RuntimeError("선택된 포인트가 없습니다. 다시 실행해주세요.")

    picked = np.asarray(picked, dtype=int)
    sel_p = pts[picked].mean(axis=0)

    kdt = o3d.geometry.KDTreeFlann(pcd)
    _, knn_idx, _ = kdt.search_knn_vector_3d(sel_p.tolist(), int(knn))
    knn_idx = np.asarray(knn_idx, dtype=int)

    nrm_arr = np.asarray(pcd.normals)
    local_n = nrm_arr[knn_idx].mean(axis=0)
    n_norm = np.linalg.norm(local_n)
    if n_norm < 1e-9:
        raise RuntimeError("노말 추정 실패 (로컬 kNN 평균 ≈ 0).")
    local_n = local_n / n_norm

    print(f"\n  선택 점 수   : {len(picked)}  (kNN={knn} 이웃으로 노말 refine)")
    print(f"  surface_{frame_label} : [{sel_p[0]:+.4f}, {sel_p[1]:+.4f}, {sel_p[2]:+.4f}] m")
    print(f"  normal_{frame_label}  : [{local_n[0]:+.4f}, {local_n[1]:+.4f}, {local_n[2]:+.4f}]")
    print(f"  world_up({frame_label}) · normal = {float(world_up @ local_n):+.3f}  "
          f"(양수면 노말이 world-up 쪽을 향함)")

    # ── 자동 정규화 ─────────────────────────────────────────────────────
    # 1) 노말을 항상 world-up(z) 성분이 음수가 되도록 강제 (결정적)
    # 2) roll 은 default_roll_deg 를 그대로 사용
    if local_n[2] > 0:
        local_n = -local_n
        print(f"  → normal_{frame_label} (z<0 강제): "
              f"[{local_n[0]:+.4f}, {local_n[1]:+.4f}, {local_n[2]:+.4f}]")

    roll_deg = float(default_roll_deg)
    roll_rad = np.radians(roll_deg)
    print(f"  → roll = {roll_deg:+.1f}° (자동)")

    T_CX_des = compute_camera_pose_from_normal(
        surface_point=sel_p,
        normal=local_n,
        distance_m=distance_m,
        roll_rad=roll_rad,
        world_up=world_up,
    )

    p_cam = T_CX_des[:3, 3]
    print(f"\n  camera origin in {frame_label} : "
          f"[{p_cam[0]:+.4f}, {p_cam[1]:+.4f}, {p_cam[2]:+.4f}] m  "
          f"(표면에서 {distance_m*1000:.0f} mm)")
    print(f"  roll = {roll_deg:+.1f}°")
    print("─" * 72)

    if preview:
        _preview_target(
            pcd=pcd,
            surface_point=sel_p,
            normal=local_n,
            T_CX_des=T_CX_des,
            distance_m=distance_m,
            frame_label=frame_label,
        )

    return T_CX_des


def _preview_target(
    pcd,
    surface_point: np.ndarray,
    normal: np.ndarray,
    T_CX_des: np.ndarray,
    distance_m: float,
    frame_label: str,
) -> None:
    """결과 확인용 창 — pcd + surface sphere + normal arrow + camera axes."""
    import open3d as o3d

    geoms = [pcd]

    world_axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)
    geoms.append(world_axes)

    sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.008)
    sphere.translate(surface_point)
    sphere.paint_uniform_color([1.0, 0.2, 0.2])
    sphere.compute_vertex_normals()
    geoms.append(sphere)

    arrow = _make_arrow(
        origin=surface_point,
        direction=normal,
        length=distance_m,
        color=(0.1, 0.9, 0.2),
    )
    geoms.append(arrow)

    cam_axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.06)
    cam_axes.transform(T_CX_des)
    geoms.append(cam_axes)

    print(f"[NBV] 미리보기 창 — 빨간구=선택점, 초록화살=표면 노말({distance_m*1000:.0f}mm), "
          f"축=카메라 목표 ({frame_label} 프레임). Q로 종료.")
    o3d.visualization.draw_geometries(
        geoms,
        window_name=f"NBV target preview (frame={frame_label})",
        width=1280, height=720,
    )
