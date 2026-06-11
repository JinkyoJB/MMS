"""
헤드리스 정사영 렌더 (numpy + PIL). filament/EGL 불가 환경용.

각 점군을 직교 투영해 픽셀 그리드에 z-buffer(가까운 점 우선)로 색칠.
세션 텍스처 색 그대로 + 오버레이 PLY 를 여러 평면에서 PNG 저장.
"""
from __future__ import annotations
import sys
from pathlib import Path
import numpy as np
from PIL import Image

PROJECT_ROOT = Path(__file__).resolve().parents[3]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))
import open3d as o3d
from scripts.artec.reg_benchmark.common import CACHE_DIR, OUT_DIR

RENDER_DIR = OUT_DIR / "renders"
PLANES = {  # name: (axis_u, axis_v, axis_depth, flip_v)
    "XY": (0, 1, 2, True),
    "XZ": (0, 2, 1, True),
    "YZ": (1, 2, 0, True),
}


def rasterize(pts, cols, plane, S=700, pad=0.08, splat=2, depth_shade=False):
    au, av, ad, flip = PLANES[plane]
    u = pts[:, au]; v = pts[:, av]; d = pts[:, ad]
    umin, umax = u.min(), u.max(); vmin, vmax = v.min(), v.max()
    span = max(umax - umin, vmax - vmin) * (1 + pad)
    uc = (umin + umax) / 2; vc = (vmin + vmax) / 2
    px = np.clip((((u - uc) / span + 0.5) * (S - 1)).round().astype(int), 0, S - 1)
    py = np.clip((((v - vc) / span + 0.5) * (S - 1)).round().astype(int), 0, S - 1)
    if flip:
        py = (S - 1) - py
    if depth_shade:
        dn = (d - d.min()) / (np.ptp(d) + 1e-9)      # 0(가까움)~1(멈)
        shade = (0.25 + 0.65 * (1 - dn))           # 가까울수록 밝게
        cols = np.repeat(shade[:, None], 3, axis=1)
    img = np.ones((S, S, 3), np.uint8) * 255
    zbuf = np.full((S, S), np.inf)
    order = np.argsort(-d)
    for i in order:
        x0, y0 = px[i], py[i]
        c = (np.clip(cols[i], 0, 1) * 255).astype(np.uint8)
        for dx in range(-splat, splat + 1):
            for dy in range(-splat, splat + 1):
                x, y = x0 + dx, y0 + dy
                if 0 <= x < S and 0 <= y < S and d[i] < zbuf[y, x]:
                    zbuf[y, x] = d[i]
                    img[y, x] = c
    return img


def render_ply(path: Path, out_stem: str):
    pc = o3d.io.read_point_cloud(str(path))
    pts = np.asarray(pc.points)
    if pc.has_colors():
        cols = np.asarray(pc.colors)
    else:
        cols = np.tile([0.4, 0.4, 0.4], (len(pts), 1))
    if len(pts) == 0:
        print(f"  (empty) {path.name}")
        return
    shade = "_shade" in out_stem
    for plane in PLANES:
        img = rasterize(pts, cols, plane, depth_shade=shade)
        outp = RENDER_DIR / f"{out_stem}_{plane}.png"
        Image.fromarray(img).save(outp)
    print(f"  rendered {out_stem} ({len(pts):,} pts)")


def main():
    RENDER_DIR.mkdir(parents=True, exist_ok=True)
    for i in range(3):
        p = CACHE_DIR / f"session_{i}.ply"
        if p.exists():
            render_ply(p, f"session{i}")
            render_ply(p, f"session{i}_shade")
    for tag in ["identity_1_0", "hint_1_0", "fpfh_ransac_1_0",
                "identity_2_0", "hint_2_0", "fpfh_ransac_2_0"]:
        from scripts.artec.reg_benchmark.common import OVERLAY_DIR as _OV; p = _OV / f"{tag}.ply"
        if p.exists():
            render_ply(p, f"ov_{tag}")


if __name__ == "__main__":
    main()
