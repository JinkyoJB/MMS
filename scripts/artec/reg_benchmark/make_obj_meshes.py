"""
6개 방법의 정합 병합본 → Poisson fusion 으로 watertight 메시 → OBJ 저장 + 음영 렌더.

입력: results3d/real_{method}_1_0_textured.ply (ref+src_aligned, 실제 색).
처리: normal 추정·정렬 → Poisson → 저밀도(extrapolation) crop → 최대성분.
출력: results3d/mesh/{method}.obj (지오메트리), {method}_color.ply (정점색 메시),
      grid_meshes.png (Lambert 음영 비교).
"""
from __future__ import annotations
import sys
from pathlib import Path
import numpy as np
import open3d as o3d
from PIL import Image, ImageDraw

PROJECT_ROOT = Path(__file__).resolve().parents[3]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))
from scripts.artec.reg_benchmark.common import OUT_DIR
from scripts.artec.reg_benchmark._render import PLANES

OUT3D = OUT_DIR / "results3d"
MESH_DIR = OUT3D / "mesh"
NAMED6 = ["fpfh_ransac", "fgr", "artec_gr", "geotransformer", "predator",
          "image_match"]
GTROT = {"fpfh_ransac": 0.37, "fgr": 0.37, "artec_gr": 0.90,
         "geotransformer": 1.48, "predator": 6.85, "image_match": 97.49}


def poisson_mesh(pcd, depth=9, density_q=0.08):
    pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
    pcd.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(
        radius=0.01, max_nn=30))
    pcd.orient_normals_consistent_tangent_plane(30)
    mesh, dens = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
        pcd, depth=depth)
    dens = np.asarray(dens)
    keep = dens > np.quantile(dens, density_q)
    mesh.remove_vertices_by_mask(~keep)
    mesh.remove_unreferenced_vertices()
    # 떠다니는 잡음 제거 — 최대 연결성분만
    ti, n_t, _ = mesh.cluster_connected_triangles()
    ti = np.asarray(ti); n_t = np.asarray(n_t)
    if len(n_t) > 1:
        biggest = int(n_t.argmax())
        mesh.remove_triangles_by_mask(ti != biggest)
        mesh.remove_unreferenced_vertices()
    # 정점색: 입력 점군 최근접 색 전이
    kt = o3d.geometry.KDTreeFlann(pcd)
    pcol = np.asarray(pcd.colors)
    mv = np.asarray(mesh.vertices)
    vc = np.zeros((len(mv), 3))
    for i, v in enumerate(mv):
        _, idx, _ = kt.search_knn_vector_3d(v, 1)
        vc[i] = pcol[idx[0]]
    mesh.vertex_colors = o3d.utility.Vector3dVector(vc)
    mesh.compute_vertex_normals()
    return mesh


def shade_render(mesh, plane="XZ", S=680, splat=2):
    """Lambert 음영(정점 normal·광원) 라스터."""
    V = np.asarray(mesh.vertices)
    N = np.asarray(mesh.vertex_normals)
    au, av, ad, flip = PLANES[plane]
    L = np.array([0.3, 0.4, 0.85]); L = L / np.linalg.norm(L)
    lam = np.clip(N @ L, 0, 1) * 0.8 + 0.2
    cols = np.repeat(lam[:, None], 3, axis=1)
    u, v, d = V[:, au], V[:, av], V[:, ad]
    span = max(np.ptp(u), np.ptp(v)) * 1.08
    uc, vc = (u.min()+u.max())/2, (v.min()+v.max())/2
    px = np.clip((((u-uc)/span+0.5)*(S-1)).round().astype(int), 0, S-1)
    py = np.clip((((v-vc)/span+0.5)*(S-1)).round().astype(int), 0, S-1)
    if flip:
        py = (S-1)-py
    img = np.ones((S, S, 3), np.uint8)*255
    zbuf = np.full((S, S), np.inf)
    for i in np.argsort(-d):
        x, y = px[i], py[i]
        for dx in range(-splat, splat+1):
            for dy in range(-splat, splat+1):
                xx, yy = x+dx, y+dy
                if 0 <= xx < S and 0 <= yy < S and d[i] < zbuf[yy, xx]:
                    zbuf[yy, xx] = d[i]
                    img[yy, xx] = (np.clip(cols[i], 0, 1)*255).astype(np.uint8)
    return img


def _panel(arr, label, sub):
    im = Image.fromarray(arr).convert("RGB")
    bar = Image.new("RGB", (im.width, 46), (245, 245, 245))
    d = ImageDraw.Draw(bar); d.text((10, 6), label, fill=(0, 0, 0))
    d.text((10, 26), sub, fill=(120, 30, 30))
    out = Image.new("RGB", (im.width, im.height+46), (255, 255, 255))
    out.paste(bar, (0, 0)); out.paste(im, (0, 46))
    return out


def main():
    MESH_DIR.mkdir(parents=True, exist_ok=True)
    panels = []
    for mth in NAMED6:
        src = OUT3D / f"real_{mth}_1_0_textured.ply"
        if not src.exists():
            continue
        pcd = o3d.io.read_point_cloud(str(src))
        mesh = poisson_mesh(pcd)
        obj = MESH_DIR / f"{mth}.obj"
        o3d.io.write_triangle_mesh(str(obj), mesh, write_vertex_colors=False)
        o3d.io.write_triangle_mesh(str(MESH_DIR / f"{mth}_color.ply"), mesh)
        gr = GTROT.get(mth, float('nan'))
        ok = "OK" if gr < 5 else "FAIL" if gr > 30 else "~"
        print(f"  {mth:<14} verts={len(mesh.vertices):>6} "
              f"faces={len(mesh.triangles):>6} -> {obj.name}  [{ok}]")
        panels.append(_panel(shade_render(mesh), mth,
                             f"gtRot={gr:.1f} deg  [{ok}]"))
    # grid 3x2
    w = max(p.width for p in panels); h = max(p.height for p in panels)
    g = Image.new("RGB", (3*w, 2*h), (255, 255, 255))
    for i, p in enumerate(panels):
        g.paste(p, ((i % 3)*w, (i//3)*h))
    g.save(OUT3D / "grid_meshes.png")
    print(f"[mesh] grid -> {OUT3D/'grid_meshes.png'}")


if __name__ == "__main__":
    main()
