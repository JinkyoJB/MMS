# mms/nbv/_progress_vis.py
#
# Non-blocking Open3D 프로그레스 뷰어 — Phase 1 (턴테이블 회전 스캔) 중
# 각 프레임 누적 결과를 실시간으로 갱신한다.

from __future__ import annotations

from typing import List, Optional

import numpy as np
import open3d as o3d


class ProgressVisualizer:
    """
    `Visualizer.poll_events/update_renderer` 기반 non-blocking 뷰어.

    워크플로우
    ----------
    vis = ProgressVisualizer(title)
    try:
        for step in steps:
            ... capture / integrate ...
            vis.update_mesh(current_mesh)
            vis.update_camera_trajectory(T_CO_list)
    finally:
        vis.run_until_closed()   # 또는 vis.close()
    """

    def __init__(
        self,
        window_title: str = "Scan Progress",
        mesh_color: tuple = (0.85, 0.30, 0.25),
        show_reference_disc: bool = True,
        reference_disc_radius: float = 0.12,
    ):
        self.mesh_color = tuple(mesh_color)
        self._vis = o3d.visualization.Visualizer()
        self._vis.create_window(window_name=window_title, width=1280, height=720)

        # O 프레임 축
        axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.05)
        self._vis.add_geometry(axes)

        # 턴테이블 z=0 디스크 (순수 시각 레퍼런스 — 크롭 아님)
        if show_reference_disc:
            disc = o3d.geometry.TriangleMesh.create_cylinder(
                radius=float(reference_disc_radius), height=0.001, resolution=64,
            )
            disc.paint_uniform_color([0.20, 0.35, 0.80])
            disc.compute_vertex_normals()
            self._vis.add_geometry(disc)

        self._mesh: Optional[o3d.geometry.TriangleMesh] = None
        self._pcd: Optional[o3d.geometry.PointCloud] = None
        self._traj_spheres: List[o3d.geometry.TriangleMesh] = []
        self._first_geom = True
        self._closed = False

        # 기본 시점 — O 프레임 +z 가 위쪽으로 보이도록 비스듬히 내려다봄
        self._apply_default_view()

    # ── Mesh 갱신 ──────────────────────────────────────────────────────

    def update_mesh(self, new_mesh: o3d.geometry.TriangleMesh) -> None:
        if self._closed:
            return
        if new_mesh is None or len(new_mesh.vertices) == 0:
            self._pump()
            return

        if self._mesh is not None:
            self._vis.remove_geometry(self._mesh, reset_bounding_box=False)

        self._mesh = o3d.geometry.TriangleMesh(new_mesh)
        self._mesh.paint_uniform_color(list(self.mesh_color))
        self._mesh.compute_vertex_normals()
        # 첫 geom 만 bounding box 재설정 — 이후 프레임은 시점 유지
        self._vis.add_geometry(self._mesh, reset_bounding_box=self._first_geom)
        if self._first_geom:
            self._apply_default_view()
            self._first_geom = False
        self._pump()

    # ── PointCloud 갱신 (pcd_accumulate 백엔드용) ──────────────────────

    def update_pcd(self, new_pcd: o3d.geometry.PointCloud) -> None:
        if self._closed:
            return
        if new_pcd is None or len(new_pcd.points) == 0:
            self._pump()
            return

        if self._pcd is not None:
            self._vis.remove_geometry(self._pcd, reset_bounding_box=False)

        self._pcd = o3d.geometry.PointCloud()
        self._pcd.points = new_pcd.points
        if new_pcd.has_colors():
            self._pcd.colors = new_pcd.colors
        else:
            self._pcd.paint_uniform_color(list(self.mesh_color))

        self._vis.add_geometry(self._pcd, reset_bounding_box=self._first_geom)
        if self._first_geom:
            self._apply_default_view()
            self._first_geom = False
        self._pump()

    # ── 카메라 궤적 갱신 ────────────────────────────────────────────────

    def update_camera_trajectory(self, T_CO_list: List[np.ndarray]) -> None:
        if self._closed:
            return
        for s in self._traj_spheres:
            self._vis.remove_geometry(s, reset_bounding_box=False)
        self._traj_spheres.clear()

        n = len(T_CO_list)
        for i, T in enumerate(T_CO_list):
            sp = o3d.geometry.TriangleMesh.create_sphere(radius=0.004)
            sp.translate(T[:3, 3])
            t = i / max(n - 1, 1)
            sp.paint_uniform_color([1.0 - t, 0.3, t])
            sp.compute_vertex_normals()
            self._vis.add_geometry(sp, reset_bounding_box=False)
            self._traj_spheres.append(sp)
        self._pump()

    # ── 기본 시점 (O 프레임 +z 를 위로, 대각선 위에서 내려다봄) ─────────

    def _apply_default_view(self) -> None:
        try:
            ctl = self._vis.get_view_control()
            # front = 카메라가 바라보는 방향 (viewer → scene)
            # up    = world 에서 "위" 방향
            # +z 를 위로, 카메라는 (+x,+y,+z) 쪽에서 원점을 내려다봄
            ctl.set_up([0.0, 0.0, 1.0])
            ctl.set_front([-0.4, -0.4, -0.8])
            ctl.set_lookat([0.0, 0.0, 0.03])
            ctl.set_zoom(0.7)
        except Exception:
            pass   # 일부 Open3D 버전에서 빈 scene 초기화 시 예외 가능

    # ── Event pump ─────────────────────────────────────────────────────

    def _pump(self) -> None:
        if self._closed:
            return
        still = self._vis.poll_events()
        self._vis.update_renderer()
        if not still:
            self._closed = True

    def run_until_closed(self) -> None:
        """창이 닫힐 때까지 poll 루프."""
        if self._closed:
            return
        while self._vis.poll_events():
            self._vis.update_renderer()
        self._closed = True
        self._vis.destroy_window()

    def close(self) -> None:
        if not self._closed:
            self._vis.destroy_window()
            self._closed = True
