"""
평면 제거 후: 각 방법의 변환으로 3세션을 병합(점군) → GT 정렬 → 커버리지 측정.
SDK 메시 없이 빠르게 "어느 방법이 GT 를 가장 잘 덮나(특히 윗/아랫면)" 판정.

변환 복원: real_{method}_{1_0,2_0}.ply 오버레이의 src_aligned vs 캐시 세션(1:1) Umeyama.
병합: session0 + T1·session1 + T2·session2 (평면제거 컬러 캐시).
정렬: 병합→GT FPFH+ICP. 지표: accuracy, completeness<2mm, bottom/top 캡.
출력: results3d/grid_merge_vs_gt.png + results_merge_vs_gt.csv
"""
from __future__ import annotations
import csv
import sys
from pathlib import Path
import numpy as np
import open3d as o3d
from PIL import Image, ImageDraw

PROJECT_ROOT = Path(__file__).resolve().parents[3]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))
from scripts.artec.reg_benchmark.common import OUT_DIR, CACHE_DIR, OVERLAY_DIR, FIG_DIR, RESULTS_DIR
from scripts.artec.reg_benchmark.methods_artec import _umeyama_rigid
from scripts.artec.reg_benchmark.compare_to_gt import align_to_gt, cap_coverage, GT_OBJ
from scripts.artec.reg_benchmark._render import rasterize

R3 = FIG_DIR
METHODS = ["fpfh_ransac", "fgr", "geotransformer", "predator", "image_match"]
NPTS = 120000


def recover_T(method, si):
    ov = OVERLAY_DIR / f"real_{method}_{si}_0.ply"
    if not ov.exists():
        return None
    P = np.asarray(o3d.io.read_point_cloud(str(ov)).points)
    s0 = np.asarray(o3d.io.read_point_cloud(str(CACHE_DIR/"session_0.ply")).points)
    sip = np.asarray(o3d.io.read_point_cloud(str(CACHE_DIR/f"session_{si}.ply")).points)
    src_al = P[len(s0):]
    m = min(len(src_al), len(sip))
    return _umeyama_rigid(sip[:m], src_al[:m])


def merged_cloud(method):
    s = [o3d.io.read_point_cloud(str(CACHE_DIR/f"session_{i}.ply")) for i in range(3)]
    out = o3d.geometry.PointCloud(s[0])
    for i in (1, 2):
        T = recover_T(method, i)
        if T is None:
            continue
        out += o3d.geometry.PointCloud(s[i]).transform(T)
    return out


def _panel(arr, label, sub):
    im = Image.fromarray(arr).convert("RGB")
    bar = Image.new("RGB", (im.width, 46), (245, 245, 245))
    d = ImageDraw.Draw(bar); d.text((8, 5), label, fill=(0, 0, 0))
    d.text((8, 26), sub, fill=(120, 30, 30))
    out = Image.new("RGB", (im.width, im.height+46), (255, 255, 255))
    out.paste(bar, (0, 0)); out.paste(im, (0, 46))
    return out


def main():
    gt = o3d.io.read_triangle_mesh(str(GT_OBJ), enable_post_processing=True)
    gt_pts = gt.sample_points_uniformly(NPTS)
    gp = np.asarray(gt_pts.points)
    # GT 패널 (지오메트리, uniform)
    gtcol = np.tile([0.6, 0.6, 0.6], (len(gp), 1))
    panels = [_panel(rasterize(gp, gtcol, "XZ", S=460, splat=2),
                     "GROUND TRUTH", "Artec ref")]
    rows = []
    for mth in METHODS:
        mg = merged_cloud(mth)
        mp = mg.voxel_down_sample(0.002)
        mp_s = mp.farthest_point_down_sample(min(NPTS, len(mp.points))) \
            if len(mp.points) > NPTS else mp
        T, icp = align_to_gt(o3d.geometry.PointCloud(mp_s), gt_pts)
        mp_a = o3d.geometry.PointCloud(mp_s).transform(T)
        acc = np.asarray(mp_a.compute_point_cloud_distance(gt_pts))*1000
        comp = np.asarray(gt_pts.compute_point_cloud_distance(mp_a))*1000
        caps = cap_coverage(gt_pts, mp_a, axis=0)
        rows.append(dict(method=mth, acc_med=round(np.median(acc), 2),
                         compl_2mm=round((comp < 2).mean()*100, 1),
                         compl_mean=round(comp.mean(), 2),
                         bottom=round(caps["bottom"], 1), top=round(caps["top"], 1)))
        sub = (f"acc {np.median(acc):.1f}mm compl {(comp<2).mean()*100:.0f}% "
               f"bot {caps['bottom']:.0f}% top {caps['top']:.0f}%")
        col = np.asarray(mp_a.colors) if mp_a.has_colors() else \
            np.tile([0.6, 0.6, 0.6], (len(mp_a.points), 1))
        panels.append(_panel(rasterize(np.asarray(mp_a.points), col, "XZ",
                                       S=460, splat=2), mth, sub))
        print(f"  {mth}: {sub}")
    w0 = max(p.width for p in panels); h0 = max(p.height for p in panels)
    g = Image.new("RGB", (3*w0, 2*h0), (255, 255, 255))
    for i, p in enumerate(panels):
        g.paste(p, ((i % 3)*w0, (i//3)*h0))
    g.save(R3/"grid_merge_vs_gt.png"); print("saved grid_merge_vs_gt.png")
    with open(RESULTS_DIR/"results_merge_vs_gt.csv", "w", newline="") as f:
        wr = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        wr.writeheader(); [wr.writerow(r) for r in rows]


if __name__ == "__main__":
    main()
