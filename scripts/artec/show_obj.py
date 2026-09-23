# scripts/artec/show_obj.py
#
# OBJ 메시(텍스처 없어도)를 창으로 띄운다 — 붉은 메시 + 파란 턴테이블 원판(z=0 기준) + 축.
# `utils.viz.show_textured_obj` 는 **텍스처가 없으면 창을 열지 않고 False 를 돌려준다**
# (단색 fallback 을 호출측에 맡기는 설계). fusion 결과처럼 텍스처 없는 OBJ 를 그냥 보려면 이걸 쓴다.
#
#   python scripts/artec/show_obj.py output/registration_test/<RUN>/post_H129/03_fusion.obj [--mm]
#
# --mm : OBJ 좌표가 mm 면(SDK save_obj 는 mm) m 로 바꿔 원판과 스케일을 맞춘다 (기본 on).
# 조작: 마우스 회전/줌, Q/ESC 닫기.  (legacy draw_geometries — 이 PC 에서 메시는 렌더된다)

from __future__ import annotations

import argparse
import sys
from pathlib import Path


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("obj")
    ap.add_argument("--title", default=None)
    ap.add_argument("--no-mm", action="store_true", help="OBJ 가 이미 m 단위일 때")
    a = ap.parse_args()
    import numpy as np
    import open3d as o3d

    p = Path(a.obj)
    if not p.is_file():
        print(f"✘ 없음: {p}")
        return 1
    mesh = o3d.io.read_triangle_mesh(str(p), enable_post_processing=True)
    if len(mesh.triangles) == 0:
        print("✘ 삼각형 없음")
        return 1
    if not a.no_mm:
        mesh.scale(0.001, center=(0, 0, 0))
    mesh.compute_vertex_normals()
    if not (mesh.has_textures() and mesh.has_triangle_uvs()):
        mesh.paint_uniform_color([0.85, 0.30, 0.25])
    v = np.asarray(mesh.vertices)
    c = v.mean(axis=0)
    print(f"[show_obj] {p.name}: {len(v):,} v / {len(mesh.triangles):,} f  "
          f"중심 ({c[0]:+.3f},{c[1]:+.3f},{c[2]:+.3f}) m  크기 {(v.max(0)-v.min(0))*1000}")
    axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.05, origin=c)
    disc = o3d.geometry.TriangleMesh.create_cylinder(radius=0.12, height=0.001, resolution=64)
    disc.paint_uniform_color([0.20, 0.35, 0.80])
    disc.compute_vertex_normals()
    # 원판은 메시 아래(가장 낮은 z 쪽)에 참고용으로만 놓는다 — 프레임을 모르는 OBJ 도 있어서
    disc.translate([c[0], c[1], float(v[:, 2].max())])
    print("[show_obj] Q/ESC 로 닫기")
    o3d.visualization.draw_geometries(
        [mesh, axis, disc], window_name=a.title or p.name,
        width=1280, height=720, mesh_show_back_face=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
