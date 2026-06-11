"""
6개 방법의 SDK 병합 메시를 Ground-Truth(Artec Studio 레퍼런스)와 비교.

- 각 방법 메시(mm) → m 스케일 → 턴테이블 평면 제거 → FPFH+ICP 로 GT 에 정렬.
- 지표: accuracy(method→GT dist), completeness(GT→method, %<2mm),
        top/bottom 캡 커버리지(아랫면 open 정량화).
- 그리드: 텍스처(legacy) / 지오메트리 음영 / GT편차 히트맵 — GT + 6 방법 같은 카메라.
- results_vs_gt.csv 저장.

실행(mms-env): python -m scripts.artec.reg_benchmark.compare_to_gt
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
from scripts.artec.reg_benchmark.common import OUT_DIR, MESH_DIR, FIG_DIR, RESULTS_DIR

GT_OBJ = OUT_DIR / "ground_truth" / "r_0017_[17]ChocolateProtein.obj"
SDK = MESH_DIR
R3 = FIG_DIR
NAMED6 = ["fpfh_ransac", "fgr", "artec_gr", "geotransformer", "predator",
          "image_match"]
GTROT = {"fpfh_ransac": 0.37, "fgr": 0.37, "artec_gr": 0.90,
         "geotransformer": 1.48, "predator": 6.85, "image_match": 97.49}
# 공통 카메라 (GT 프레임). 카톤 높이축 = X.
CAM = dict(front=[-0.55, -0.5, -0.65], up=[-1, 0, 0], zoom=0.62)
NPTS = 120000


def _feat(p, v):
    d = p.voxel_down_sample(v)
    d.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=v*2.5, max_nn=30))
    f = o3d.pipelines.registration.compute_fpfh_feature(
        d, o3d.geometry.KDTreeSearchParamHybrid(radius=v*5, max_nn=100))
    return d, f


def align_to_gt(method_pts, gt_pts, v=0.004):
    """턴테이블 평면 제거 → FPFH RANSAC → ICP. 반환 (T_align, icp)."""
    pl, inl = method_pts.segment_plane(0.003, 3, 800)
    carton = (method_pts.select_by_index(inl, invert=True)
              if len(inl) > 0.15*len(method_pts.points) else method_pts)
    s, sf = _feat(carton, v); t, tf = _feat(gt_pts, v)
    res = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        s, t, sf, tf, True, v*1.5,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(False), 3,
        [o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(v*1.5)],
        o3d.pipelines.registration.RANSACConvergenceCriteria(4000000, 0.999))
    icp = o3d.pipelines.registration.registration_icp(
        carton, gt_pts, 0.004, res.transformation,
        o3d.pipelines.registration.TransformationEstimationPointToPoint())
    return icp.transformation, icp


def cap_coverage(gt_pts, method_pts, axis=0, frac=0.06, tau=0.002):
    """높이축 양 끝 캡(아랫면/윗면)에서 method 가 GT 를 덮은 비율."""
    g = np.asarray(gt_pts.points)
    lo, hi = g[:, axis].min(), g[:, axis].max()
    band = (hi - lo) * frac
    out = {}
    dist = np.asarray(gt_pts.compute_point_cloud_distance(method_pts))
    for nm, mask in [("bottom", g[:, axis] < lo + band),
                     ("top", g[:, axis] > hi - band)]:
        if mask.sum() == 0:
            out[nm] = float("nan")
        else:
            out[nm] = float((dist[mask] < tau).mean()*100)
    return out


def render_legacy(geoms, out_png, S=720, lit=True):
    vis = o3d.visualization.Visualizer()
    vis.create_window(width=S, height=S, visible=False)
    for g in geoms:
        vis.add_geometry(g)
    opt = vis.get_render_option()
    opt.mesh_show_back_face = True
    opt.background_color = np.array([1, 1, 1])
    opt.light_on = lit            # heat 맵은 조명 off → 정점색 그대로
    ctr = vis.get_view_control()
    ctr.set_front(CAM["front"]); ctr.set_up(CAM["up"]); ctr.set_zoom(CAM["zoom"])
    for _ in range(6):
        vis.poll_events(); vis.update_renderer()
    vis.capture_screen_image(str(out_png), do_render=True)
    vis.destroy_window()


def heat_colors(mesh, gt_pts, tmax=0.005):
    """정점 dist-to-GT → green(0)~red(tmax) 컬러."""
    vp = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(
        np.asarray(mesh.vertices)))
    d = np.asarray(vp.compute_point_cloud_distance(gt_pts))
    t = np.clip(d / tmax, 0, 1)
    col = np.zeros((len(d), 3)); col[:, 0] = t; col[:, 1] = 1 - t
    return col


def _panel(png, label, sub):
    im = Image.open(png).convert("RGB")
    bar = Image.new("RGB", (im.width, 46), (245, 245, 245))
    dd = ImageDraw.Draw(bar); dd.text((10, 6), label, fill=(0, 0, 0))
    dd.text((10, 26), sub, fill=(120, 30, 30))
    out = Image.new("RGB", (im.width, im.height+46), (255, 255, 255))
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
    cmp_dir = FIG_DIR / "vs_gt"; cmp_dir.mkdir(parents=True, exist_ok=True)
    gt = o3d.io.read_triangle_mesh(str(GT_OBJ), enable_post_processing=True)
    gt.compute_vertex_normals()
    gt_pts = gt.sample_points_uniformly(NPTS)

    # GT 패널 렌더
    render_legacy([gt], cmp_dir/"GT_tex.png")
    gt_geo = o3d.geometry.TriangleMesh(gt); gt_geo.textures = []
    gt_geo.paint_uniform_color([0.7, 0.7, 0.7]); gt_geo.compute_vertex_normals()
    render_legacy([gt_geo], cmp_dir/"GT_geo.png", lit=True)

    rows = []
    tex_p = [_panel(cmp_dir/"GT_tex.png", "GROUND TRUTH", "Artec Studio ref")]
    geo_p = [_panel(cmp_dir/"GT_geo.png", "GROUND TRUTH", "watertight")]
    heat_p = []
    for mth in NAMED6:
        obj = SDK / f"{mth}.obj"
        if not obj.exists():
            continue
        mesh = o3d.io.read_triangle_mesh(str(obj), enable_post_processing=True)
        mesh.scale(0.001, center=np.zeros(3))           # mm -> m
        mpts = mesh.sample_points_uniformly(NPTS)
        T, icp = align_to_gt(mpts, gt_pts)
        mesh.transform(T)
        mpts_a = o3d.geometry.PointCloud(mpts).transform(T)
        # 지표
        acc = np.asarray(mpts_a.compute_point_cloud_distance(gt_pts))*1000
        comp = np.asarray(gt_pts.compute_point_cloud_distance(mpts_a))*1000
        caps = cap_coverage(gt_pts, mpts_a, axis=0)
        rows.append(dict(method=mth, gtRot=GTROT.get(mth, np.nan),
                         icp_fit=round(icp.fitness, 3),
                         acc_med_mm=round(np.median(acc), 2),
                         acc_mean_mm=round(acc.mean(), 2),
                         compl_2mm=round((comp < 2).mean()*100, 1),
                         compl_mean_mm=round(comp.mean(), 2),
                         bottom_cov=round(caps["bottom"], 1),
                         top_cov=round(caps["top"], 1),
                         verts=len(mesh.vertices)))
        sub = (f"acc med {np.median(acc):.1f}mm | compl {(comp<2).mean()*100:.0f}%"
               f" | bottom {caps['bottom']:.0f}% top {caps['top']:.0f}%")
        # 텍스처 렌더 (GT 프레임)
        render_legacy([mesh], cmp_dir/f"{mth}_tex.png")
        tex_p.append(_panel(cmp_dir/f"{mth}_tex.png", mth, sub))
        # 지오메트리
        g2 = o3d.geometry.TriangleMesh(mesh); g2.textures = []
        g2.paint_uniform_color([0.7, 0.7, 0.7]); g2.compute_vertex_normals()
        render_legacy([g2], cmp_dir/f"{mth}_geo.png", lit=True)
        geo_p.append(_panel(cmp_dir/f"{mth}_geo.png", mth, sub))
        # 히트맵
        hm = o3d.geometry.TriangleMesh(mesh); hm.textures = []
        hm.vertex_colors = o3d.utility.Vector3dVector(heat_colors(hm, gt_pts))
        render_legacy([hm], cmp_dir/f"{mth}_heat.png", lit=False)
        heat_p.append(_panel(cmp_dir/f"{mth}_heat.png", mth, sub))
        print(f"  {mth}: acc_med={np.median(acc):.2f}mm compl={ (comp<2).mean()*100:.1f}% "
              f"bottom={caps['bottom']:.1f}% top={caps['top']:.1f}%")

    _grid(tex_p, 3, R3/"grid_vs_gt_textured.png")
    _grid(geo_p, 3, R3/"grid_vs_gt_geometry.png")
    _grid(heat_p, 3, R3/"grid_vs_gt_heatmap.png")
    with open(RESULTS_DIR/"results_vs_gt.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader(); [w.writerow(r) for r in rows]
    print(f"[compare_to_gt] csv + 3 grids -> {R3}")


if __name__ == "__main__":
    main()
