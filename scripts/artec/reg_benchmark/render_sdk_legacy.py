"""
SDK 텍스처 메시 6개를 legacy Visualizer 로 실제 렌더(텍스처 그대로) → 비교 그리드.
(이 PC 에선 filament/EGL 헤드리스 불가하지만 legacy Visualizer 는 메시 렌더 가능 —
 [[project_open3d_must_use_filament_o3dvisualizer]] 의 예외: 정적 메시는 됨.)
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
RDIR = FIG_DIR / "legacy"
NAMED6 = ["fpfh_ransac", "fgr", "artec_gr", "geotransformer", "predator",
          "image_match"]
GTROT = {"fpfh_ransac": 0.37, "fgr": 0.37, "artec_gr": 0.90,
         "geotransformer": 1.48, "predator": 6.85, "image_match": 97.49}


def render(mesh_path, out_png, S=760):
    m = o3d.io.read_triangle_mesh(str(mesh_path), enable_post_processing=True)
    m.compute_vertex_normals()
    vis = o3d.visualization.Visualizer()
    vis.create_window(width=S, height=S, visible=False)
    vis.add_geometry(m)
    opt = vis.get_render_option()
    opt.mesh_show_back_face = True
    opt.background_color = np.array([1, 1, 1])
    ctr = vis.get_view_control()
    ctr.set_front([0.35, -0.45, -0.82]); ctr.set_up([0, -1, 0])
    ctr.set_zoom(0.62)
    for _ in range(6):
        vis.poll_events(); vis.update_renderer()
    vis.capture_screen_image(str(out_png), do_render=True)
    vis.destroy_window()


def _panel(png, label, sub):
    im = Image.open(png).convert("RGB")
    bar = Image.new("RGB", (im.width, 46), (245, 245, 245))
    d = ImageDraw.Draw(bar); d.text((10, 6), label, fill=(0, 0, 0))
    d.text((10, 26), sub, fill=(120, 30, 30))
    out = Image.new("RGB", (im.width, im.height+46), (255, 255, 255))
    out.paste(bar, (0, 0)); out.paste(im, (0, 46))
    return out


def main():
    RDIR.mkdir(parents=True, exist_ok=True)
    panels = []
    for mth in NAMED6:
        obj = SDK_MESH / f"{mth}.obj"
        if not obj.exists():
            continue
        png = RDIR / f"{mth}.png"
        render(obj, png)
        gr = GTROT.get(mth, float('nan'))
        ok = "OK" if gr < 5 else "FAIL" if gr > 30 else "~"
        panels.append(_panel(png, mth, f"gtRot={gr:.1f}deg  [{ok}]"))
        print(f"  rendered {mth}")
    if panels:
        w = max(p.width for p in panels); h = max(p.height for p in panels)
        g = Image.new("RGB", (3*w, 2*h), (255, 255, 255))
        for i, p in enumerate(panels):
            g.paste(p, ((i % 3)*w, (i//3)*h))
        out = FIG_DIR/"grid_sdk_legacy.png"
        g.save(out); print(f"saved {out}")


if __name__ == "__main__":
    main()
