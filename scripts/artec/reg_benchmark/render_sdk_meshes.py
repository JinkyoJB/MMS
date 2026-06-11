"""
SDK 텍스처 메시(results3d/sdk_mesh/{method}.obj) 6개를 텍스처 색 + 지오메트리
음영으로 렌더 → 비교 그리드 PNG. (filament 헤드리스 불가 → numpy/PIL 정사영.)
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
from scripts.artec.reg_benchmark.common import OUT_DIR, MESH_DIR, FIG_DIR

SDK_MESH = MESH_DIR
NAMED6 = ["fpfh_ransac", "fgr", "artec_gr", "geotransformer", "predator",
          "image_match"]
GTROT = {"fpfh_ransac": 0.37, "fgr": 0.37, "artec_gr": 0.90,
         "geotransformer": 1.48, "predator": 6.85, "image_match": 97.49}


def vertex_texcolor(m):
    V = np.asarray(m.vertices)
    col = np.full((len(V), 3), 0.6)
    if not (m.has_triangle_uvs() and m.has_textures()):
        return col
    uvs = np.asarray(m.triangle_uvs)
    tv = np.asarray(m.triangles).reshape(-1)
    texs = [np.asarray(t) for t in m.textures if np.asarray(t).size]
    if not texs:
        return col
    tex = texs[-1]; H, W = tex.shape[:2]
    px = np.clip((uvs[:, 0] * (W - 1)).astype(int), 0, W - 1)
    py = np.clip(((1 - uvs[:, 1]) * (H - 1)).astype(int), 0, H - 1)
    samp = tex[py, px, :3].astype(np.float64) / 255.0
    acc = np.zeros((len(V), 3)); cnt = np.zeros(len(V))
    np.add.at(acc, tv, samp); np.add.at(cnt, tv, 1)
    nz = cnt > 0; col[nz] = acc[nz] / cnt[nz, None]
    return col


def rasterize(V, N, base_col, plane, S=620, splat=2, shade=True):
    au, av, ad = {"XZ": (0, 2, 1), "YZ": (1, 2, 0), "XY": (0, 1, 2)}[plane]
    u, v, d = V[:, au], V[:, av], V[:, ad]
    span = max(np.ptp(u), np.ptp(v)) * 1.08
    uc, vc = (u.min()+u.max())/2, (v.min()+v.max())/2
    px = np.clip((((u-uc)/span+0.5)*(S-1)).round().astype(int), 0, S-1)
    py = np.clip((((v-vc)/span+0.5)*(S-1)).round().astype(int), 0, S-1)
    py = (S-1)-py
    if shade:
        L = np.array([.3, .4, .85]); L /= np.linalg.norm(L)
        lam = (np.clip(np.abs(N @ L), 0, 1)*0.55+0.45)[:, None]
        cols = np.clip(base_col*lam, 0, 1)
    else:
        cols = base_col
    img = np.ones((S, S, 3), np.uint8)*255
    zb = np.full((S, S), np.inf)
    for i in np.argsort(-d):
        x0, y0 = px[i], py[i]
        c = (cols[i]*255).astype(np.uint8)
        for dx in range(-splat, splat+1):
            for dy in range(-splat, splat+1):
                x, y = x0+dx, y0+dy
                if 0 <= x < S and 0 <= y < S and d[i] < zb[y, x]:
                    zb[y, x] = d[i]; img[y, x] = c
    return img


def _panel(arr, label, sub):
    im = Image.fromarray(arr).convert("RGB")
    bar = Image.new("RGB", (im.width, 44), (245, 245, 245))
    dd = ImageDraw.Draw(bar); dd.text((8, 5), label, fill=(0, 0, 0))
    dd.text((8, 25), sub, fill=(120, 30, 30))
    out = Image.new("RGB", (im.width, im.height+44), (255, 255, 255))
    out.paste(bar, (0, 0)); out.paste(im, (0, 44))
    return out


def _grid(panels, cols, path):
    w = max(p.width for p in panels); h = max(p.height for p in panels)
    rows = (len(panels)+cols-1)//cols
    g = Image.new("RGB", (cols*w, rows*h), (255, 255, 255))
    for i, p in enumerate(panels):
        g.paste(p, ((i % cols)*w, (i//cols)*h))
    g.save(path); print(f"saved {path.name}")


def main():
    tex_panels, geo_panels = [], []
    for mth in NAMED6:
        obj = SDK_MESH / f"{mth}.obj"
        if not obj.exists():
            print(f"  (missing) {mth}"); continue
        m = o3d.io.read_triangle_mesh(str(obj), enable_post_processing=True)
        m.compute_vertex_normals()
        V = np.asarray(m.vertices); N = np.asarray(m.vertex_normals)
        col = vertex_texcolor(m)
        gr = GTROT.get(mth, float('nan'))
        ok = "OK" if gr < 5 else "FAIL" if gr > 30 else "~"
        sub = f"gtRot={gr:.1f}deg [{ok}]  v={len(V)//1000}k"
        tex_panels.append(_panel(rasterize(V, N, col, "XZ", shade=True),
                                 mth, sub))
        geo_panels.append(_panel(rasterize(
            V, N, np.full_like(V, 0.7), "XZ", shade=True), mth, sub))
        print(f"  rendered {mth}  v={len(V)}")
    if tex_panels:
        _grid(tex_panels, 3, FIG_DIR/"grid_sdk_textured.png")
        _grid(geo_panels, 3, FIG_DIR/"grid_sdk_geometry.png")


if __name__ == "__main__":
    main()
