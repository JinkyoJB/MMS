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


def show_composite_mesh(result, title: str = "Artec 스캔 결과",
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


def show_preview_result(pv: dict, title: str = "preview 결과") -> None:
    """preview 산출(점군·턴테이블 축·밴드 플랜)을 한 창에 그린다.

    왜 따로 있나 — `--until preview` 는 **캡처를 안 한다.** 모델이 비어 있어
    `show_composite_mesh` 가 보여줄 것이 없다. 그런데 preview 야말로 눈으로
    확인해야 하는 단계다: 물체를 제대로 봤는지, 밴드를 왜 그렇게 나눴는지가
    로그 숫자만으로는 안 보인다 (2026-09-21 실물 요청).

    좌표는 전부 **로봇 base 프레임(m)**. 천장 마운트라 +Z 가 아래이므로 뷰의
    up 을 −Z 로 잡는다 — 안 그러면 물체가 거꾸로 서 보인다.

    색: 회색=preview 점군 · 파랑=턴테이블 원판 · 청록=회전축 ·
        주황=밴드 카메라 위치(크기순: 첫 밴드가 크다) · 노랑=밴드 조준선
    """
    import open3d as o3d

    P = np.asarray(pv.get("points_B"), float)
    if P.size == 0:
        print("[viz] preview 점군이 비었다 — 시각화 skip")
        return
    axis_pt = np.asarray(pv["axis_pt_B"], float)
    axis_dir = np.asarray(pv["axis_dir_B"], float)
    axis_dir = axis_dir / (np.linalg.norm(axis_dir) + 1e-12)
    up_sign = float(pv.get("up_sign", 1.0))
    geoms = []

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(P)
    pcd.paint_uniform_color([0.45, 0.45, 0.48])
    geoms.append(pcd)

    # 턴테이블 원판 — 축에 수직으로 눕힌다.
    disc = o3d.geometry.TriangleMesh.create_cylinder(
        radius=0.1176, height=0.002, resolution=64)
    z = np.array([0.0, 0.0, 1.0])
    v = np.cross(z, axis_dir); s = float(np.linalg.norm(v)); c = float(z @ axis_dir)
    if s > 1e-9:
        K = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
        R = np.eye(3) + K + K @ K * ((1 - c) / (s * s))
    else:
        R = np.eye(3) if c > 0 else -np.eye(3)
    disc.rotate(R, center=(0, 0, 0)); disc.translate(axis_pt)
    disc.paint_uniform_color([0.20, 0.35, 0.80]); disc.compute_vertex_normals()
    geoms.append(disc)

    # 회전축 — 원판에서 '위'(up_sign) 로 30cm
    up = -axis_dir if up_sign < 0 else axis_dir
    ax = o3d.geometry.LineSet(
        points=o3d.utility.Vector3dVector([axis_pt, axis_pt + up * 0.30]),
        lines=o3d.utility.Vector2iVector([[0, 1]]))
    ax.colors = o3d.utility.Vector3dVector([[0.0, 0.8, 0.8]])
    geoms.append(ax)

    # 밴드 카메라 — eye 위치에 구, 조준점까지 선. 첫 밴드가 가장 크다.
    vps = pv.get("view_poses") or []
    for i, vp in enumerate(vps):
        eye = getattr(vp, "eye_w", None)
        if eye is None:
            continue
        eye = np.asarray(eye, float)
        # 조준점 = 축 위의 target_z (플래너가 밴드마다 정한 높이).
        tgt = np.array([axis_pt[0], axis_pt[1],
                        float(getattr(vp, "target_z", axis_pt[2]))])
        sp = o3d.geometry.TriangleMesh.create_sphere(
            radius=max(0.012 - 0.002 * i, 0.005), resolution=12)
        sp.translate(eye); sp.paint_uniform_color([0.95, 0.55, 0.15])
        sp.compute_vertex_normals(); geoms.append(sp)
        ln = o3d.geometry.LineSet(
            points=o3d.utility.Vector3dVector([eye, tgt]),
            lines=o3d.utility.Vector2iVector([[0, 1]]))
        ln.colors = o3d.utility.Vector3dVector([[0.9, 0.85, 0.2]])
        geoms.append(ln)

    # 콘솔 요약 — 창을 닫아도 남는다.
    zc = (P - axis_pt) @ up
    rr = np.linalg.norm((P - axis_pt) - np.outer(zc, up), axis=1)
    print(f"\n[viz] preview 결과 — {pv.get('note', '')}")
    print(f"  점군        : {len(P):,}점")
    print(f"  물체 높이   : {(zc.max() - max(zc.min(), 0.0)) * 1000:.0f} mm "
          f"(원판 위 {zc.min()*1000:+.0f} ~ {zc.max()*1000:+.0f} mm)")
    print(f"  최대 반경   : {rr.max()*1000:.0f} mm")
    print(f"  밴드 자세   : {len(vps)}개")
    for i, vp in enumerate(vps, 1):
        print(f"    {i}. el={getattr(vp,'el_deg',0):.0f}°  "
              f"standoff={getattr(vp,'standoff',0)*1000:.0f}mm  "
              f"tz={getattr(vp,'target_z',0)*1000:.0f}mm")
    print("  회색=점군 · 파랑=원판 · 청록=회전축 · 주황=밴드 카메라(첫 밴드가 큼)")
    print("  Q/ESC 로 닫기.")

    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name=title, width=1280, height=760)
    for g in geoms:
        vis.add_geometry(g)
    try:
        ctl = vis.get_view_control()
        ctl.set_up(list(up))                 # 천장 마운트: '위' 는 −Z
        ctl.set_front([-0.6, -0.6, 0.0])
        ctl.set_lookat(list(axis_pt + up * 0.06))
        ctl.set_zoom(0.55)
    except Exception:
        pass
    vis.run()
    vis.destroy_window()
