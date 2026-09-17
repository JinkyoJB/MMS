"""
utils/viz.py — 스캔 결과 open3d 시각화 헬퍼 (main/스크립트 공용).

Artec/sim 스캔 결과(`ArtecProcessResult`-호환: `.model`, `.ctx`)를 표시한다.
real/sim·main_artec·스크립트 어디서나 재사용. open3d 는 **함수 안에서 lazy import**
하므로(이 모듈 import 자체는 open3d 불필요), 호출 시점에만 open3d 가 필요하다.

Notation: 좌표 m, mesh 정점은 mm→m 변환. 파란 원판 = 턴테이블 z=0 레퍼런스.
"""
from __future__ import annotations

from typing import Optional

import numpy as np


def show_textured_obj(obj_path: str, title: str) -> bool:
    """
    Texturize 된 export OBJ(.obj + .mtl + 텍스처 png)를 텍스처 그대로 표시.

    성공하면 True. 파일 없음 / 텍스처 없음 / 로드 실패면 False
    (호출측이 단색 fallback 으로 넘어감).
    """
    import os

    import open3d as o3d

    if not obj_path or not os.path.isfile(obj_path):
        print(f"[viz] textured OBJ 없음 ({obj_path}) — 단색 fallback")
        return False
    try:
        mesh = o3d.io.read_triangle_mesh(obj_path, enable_post_processing=True)
    except Exception as e:
        print(f"[viz] OBJ 로드 실패 ({type(e).__name__}: {e}) — 단색 fallback")
        return False

    if len(mesh.triangles) == 0:
        print("[viz] OBJ 에 삼각형 없음 — 단색 fallback")
        return False

    mesh.compute_vertex_normals()
    has_tex = mesh.has_textures() and mesh.has_triangle_uvs()
    print(f"[viz] textured OBJ: verts={len(mesh.vertices):,} "
          f"faces={len(mesh.triangles):,} textured={has_tex}")
    if not has_tex:
        return False                      # 텍스처 없으면 단색 fallback 이 더 깔끔

    axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.05)
    disc = o3d.geometry.TriangleMesh.create_cylinder(
        radius=0.12, height=0.001, resolution=64)
    disc.paint_uniform_color([0.20, 0.35, 0.80])
    disc.compute_vertex_normals()

    print("[viz] 텍스처 입은 최종 mesh 표시 — Q/ESC 로 닫기.")
    o3d.visualization.draw_geometries(            # legacy draw = triangle-uv 텍스처 렌더
        [mesh, axis, disc], window_name=f"{title} (textured)",
        width=1280, height=720, mesh_show_back_face=True,
    )
    return True


def show_composite_mesh(result, title: str = "Artec lookaround",
                        obj_path: Optional[str] = None) -> None:
    """
    스캔 결과(`result.model`)를 표시. 우선순위:
      1) texturize 된 export OBJ 를 텍스처 그대로 (`show_textured_obj`)
      2) composite final mesh (단색)
      3) IModel frame transformations 으로 vertices 누적 PointCloud (단색)
    """
    import open3d as o3d

    if obj_path is not None and show_textured_obj(obj_path, title):
        return

    model = result.model
    geoms: list = []

    if model.has_final_mesh():
        verts = model.final_vertices().astype(np.float64) / 1000.0   # mm → m
        faces = model.final_faces().astype(np.int32)
        print(f"\n[viz] composite mesh  verts={len(verts):,}  faces={len(faces):,}")
        mesh = o3d.geometry.TriangleMesh()
        mesh.vertices = o3d.utility.Vector3dVector(verts)
        mesh.triangles = o3d.utility.Vector3iVector(faces)
        mesh.compute_vertex_normals()
        mesh.paint_uniform_color([0.85, 0.30, 0.25])
        geoms.append(mesh)
    else:
        # composite 없음 → IModel 안 frame transformations 으로 누적
        print("\n[viz] composite mesh 없음 — frame transformations 으로 누적 표시")
        all_pts = []
        for s_i in range(model.scan_count()):
            scan = model.get_scan(s_i)
            for f_i in range(scan.frame_count()):
                v_mm = scan.get_frame(f_i).vertices().astype(np.float64)
                try:                                  # SDK SLAM frame transform (streaming)
                    T = scan.get_frame_transformation(f_i)
                    v_m = (v_mm @ T[:3, :3].T + T[:3, 3]) / 1000.0
                except Exception:
                    v_m = v_mm / 1000.0
                all_pts.append(v_m)
        if all_pts:
            all_pts = np.vstack(all_pts)
            print(f"  누적 pts (O 프레임) = {len(all_pts):,}")
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(all_pts)
            pcd = pcd.voxel_down_sample(0.001)        # 가볍게
            pcd.paint_uniform_color([0.85, 0.30, 0.25])
            geoms.append(pcd)

    if not geoms:
        print("[viz] 표시할 데이터 없음 — 시각화 skip")
        return

    # O 프레임 축 + 턴테이블 z=0 디스크
    geoms.append(o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.05))
    disc = o3d.geometry.TriangleMesh.create_cylinder(radius=0.12, height=0.001, resolution=64)
    disc.paint_uniform_color([0.20, 0.35, 0.80])
    disc.compute_vertex_normals()
    geoms.append(disc)

    print("[viz] 붉음=mesh  /  파란 원판=turntable z=0 레퍼런스. Q/ESC 로 닫기.")
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name=title, width=1280, height=720)
    for g in geoms:
        vis.add_geometry(g)
    try:
        ctl = vis.get_view_control()
        ctl.set_up([0.0, 0.0, 1.0])
        ctl.set_front([-0.4, -0.4, -0.8])
        ctl.set_lookat([0.0, 0.0, 0.03])
        ctl.set_zoom(0.7)
    except Exception:
        pass
    vis.run()
    vis.destroy_window()
