"""
6개 방법의 정합 결과물(병합 3D)을 텍스처 색 입혀 PLY 저장 + 비교 이미지 렌더.

- real 1->0: 6개 방법 모두 적용 → 실제 색 병합본(ref=session0 색 + src_aligned=session1 색).
  overlay PLY(빨강/파랑)에서 src_aligned 좌표를 떼어내고 캐시의 실제 색을 재부여.
- syn_rot180_partial: 점 기반 4방법(fpfh/fgr/geotransformer/predator) 빨강/파랑 오버레이
  → flip 복원 성공/실패가 한눈에.

출력: output/reg_benchmark/results3d/*.ply + grid_*.png
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
from scripts.artec.reg_benchmark.common import OUT_DIR, CACHE_DIR, OVERLAY_DIR, FIG_DIR
from scripts.artec.reg_benchmark._render import rasterize

OUT3D = FIG_DIR
NAMED6 = ["fpfh_ransac", "fgr", "artec_gr", "geotransformer", "predator",
          "image_match"]
# real 1->0 GT 대비 gtRot (라벨용; RESULTS 에서)
GTROT_REAL = {"fpfh_ransac": 0.37, "fgr": 0.37, "artec_gr": 0.90,
              "geotransformer": 1.48, "predator": 6.85, "image_match": 97.49}


def textured_merge(method, pair="1_0", si=1):
    """overlay PLY → src_aligned 좌표 + 실제 색 병합본."""
    ov = OVERLAY_DIR / f"real_{method}_{pair}.ply"
    if not ov.exists():
        return None
    pc = o3d.io.read_point_cloud(str(ov))
    P = np.asarray(pc.points)
    s0 = o3d.io.read_point_cloud(str(CACHE_DIR / "session_0.ply"))
    si_pc = o3d.io.read_point_cloud(str(CACHE_DIR / f"session_{si}.ply"))
    n0 = len(s0.points)
    ref_xyz, ref_col = np.asarray(s0.points), np.asarray(s0.colors)
    src_xyz = P[n0:]                       # src_aligned (session_i 순서)
    src_col = np.asarray(si_pc.colors)
    m = min(len(src_xyz), len(src_col))
    merged = o3d.geometry.PointCloud()
    merged.points = o3d.utility.Vector3dVector(
        np.vstack([ref_xyz, src_xyz[:m]]))
    merged.colors = o3d.utility.Vector3dVector(
        np.vstack([ref_col, src_col[:m]]))
    return merged


def _panel(img_arr, label, sub):
    """라스터 이미지에 라벨 바 추가."""
    im = Image.fromarray(img_arr).convert("RGB")
    bar = Image.new("RGB", (im.width, 46), (245, 245, 245))
    d = ImageDraw.Draw(bar)
    d.text((10, 6), label, fill=(0, 0, 0))
    d.text((10, 26), sub, fill=(120, 30, 30))
    out = Image.new("RGB", (im.width, im.height + 46), (255, 255, 255))
    out.paste(bar, (0, 0)); out.paste(im, (0, 46))
    return out


def _grid(panels, cols, path):
    w = max(p.width for p in panels); h = max(p.height for p in panels)
    rows = (len(panels) + cols - 1) // cols
    g = Image.new("RGB", (cols * w, rows * h), (255, 255, 255))
    for i, p in enumerate(panels):
        g.paste(p, ((i % cols) * w, (i // cols) * h))
    g.save(path)
    print(f"  saved {path.name}")


def main():
    OUT3D.mkdir(parents=True, exist_ok=True)

    # 1) real 1->0 — 6방법 텍스처 병합
    panels = []
    for mth in NAMED6:
        mg = textured_merge(mth, "1_0", si=1)
        if mg is None:
            continue
        o3d.io.write_point_cloud(str(OUT3D / f"real_{mth}_1_0_textured.ply"), mg)
        img = rasterize(np.asarray(mg.points), np.asarray(mg.colors), "XZ",
                        S=520, splat=2)
        gr = GTROT_REAL.get(mth, float("nan"))
        ok = "OK" if gr < 5 else "FAIL" if gr > 30 else "~"
        panels.append(_panel(img, f"{mth}", f"gtRot={gr:.1f} deg  [{ok}]"))
    _grid(panels, 3, OUT3D / "grid_real_1_0_textured.png")

    # 2) syn_rot180_partial — 점기반 4방법 오버레이(빨강=src,파랑=ref)
    panels2 = []
    gtrot_syn = {"fpfh_ransac": 0.8, "fgr": 0.1, "geotransformer": 20.5,
                 "predator": 174.5}
    for mth in ["fpfh_ransac", "fgr", "geotransformer", "predator"]:
        ov = OVERLAY_DIR / f"syn_{mth}_syn_rot180_partial.ply"
        if not ov.exists():
            continue
        pc = o3d.io.read_point_cloud(str(ov))
        img = rasterize(np.asarray(pc.points), np.asarray(pc.colors), "XZ",
                        S=520, splat=2)
        gr = gtrot_syn.get(mth, float("nan"))
        ok = "OK" if gr < 5 else "FAIL" if gr > 30 else "~"
        panels2.append(_panel(img, f"{mth}", f"gtRot={gr:.1f} deg  [{ok}]"))
    _grid(panels2, 2, OUT3D / "grid_syn180partial_overlay.png")


if __name__ == "__main__":
    main()
