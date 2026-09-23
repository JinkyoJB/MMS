# scripts/artec/reg_hint_fusion.py
#
# 저장된 sproj(예: reg_hint_post.py 의 01_outliers.sproj) 를 열어 **GR 없이** PoissonFusion 만
# 여러 설정으로 돌리고, 결과 메시가 watertight 인지·몇 조각인지 잰다.
#
#   python scripts/artec/reg_hint_fusion.py output/registration_test/<RUN>/post_H129/01_outliers.sproj
#       [--variants byradius5,all] [--small-objects]
#
# 산출: 같은 폴더에 fusion_<variant>.obj + fusion_report.txt

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np

_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(_ROOT))
for _s in (sys.stdout, sys.stderr):
    try:
        _s.reconfigure(encoding="utf-8", errors="replace")
    except Exception:                                   # noqa: BLE001
        pass


def mesh_stats(path):
    import open3d as o3d
    m = o3d.io.read_triangle_mesh(str(path))
    if len(m.triangles) == 0:
        return "삼각형 없음"
    cl = np.asarray(m.cluster_connected_triangles()[0])
    sz = np.sort(np.bincount(cl))[::-1]
    # 가장 큰 덩어리만 따로 watertight 인지 — 작은 조각(잡음)은 어차피 SmallObjects 몫
    big = o3d.geometry.TriangleMesh(m)
    big.remove_triangles_by_mask(cl != int(np.argmax(np.bincount(cl))))
    big.remove_unreferenced_vertices()
    return (f"{len(m.vertices):,} v / {len(m.triangles):,} f · 덩어리 {len(sz)}개 (최대 {sz[0]/len(m.triangles)*100:.0f}%)"
            f" · 전체 watertight={m.is_watertight()} · 최대덩어리 watertight={big.is_watertight()}"
            f" · 최대덩어리 경계엣지 {len(np.asarray(big.get_non_manifold_edges(allow_boundary_edges=False)))}nm/"
            f"{int((np.asarray(big.compute_adjacency_list() and 0) or 0))}")


def boundary_edges(path):
    """최대 덩어리의 열린 경계 엣지 수 — 0 이면 닫힌 면."""
    import open3d as o3d
    m = o3d.io.read_triangle_mesh(str(path))
    cl = np.asarray(m.cluster_connected_triangles()[0])
    big = o3d.geometry.TriangleMesh(m)
    big.remove_triangles_by_mask(cl != int(np.argmax(np.bincount(cl))))
    big.remove_unreferenced_vertices()
    tri = np.asarray(big.triangles)
    e = np.sort(np.vstack([tri[:, [0, 1]], tri[:, [1, 2]], tri[:, [2, 0]]]), axis=1)
    _, cnt = np.unique(e, axis=0, return_counts=True)
    return int((cnt == 1).sum()), len(tri), bool(big.is_watertight())


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("sproj")
    ap.add_argument("--variants", default="byradius5,all")
    ap.add_argument("--small-objects", action="store_true", help="fusion 전에 SmallObjectsFilter")
    a = ap.parse_args()
    from mms_artec.sensor.artec_client import ArtecClient
    from mms_artec.sensor import artec_base, artec_algorithm as A

    src = Path(a.sproj)
    OUT = src.parent
    rep = []

    def log(m=""):
        print(m, flush=True); rep.append(m)

    t0 = time.perf_counter()
    entries = ArtecClient.load_project(str(src))
    model = artec_base.create_model()
    n = 0
    for e in entries:
        sc = getattr(e, "scan", None)
        if sc is not None and sc.frame_count() > 0:
            model.add_scan(sc); n += 1
    log(f"{src.name}: IScan {n}개 로드 ({time.perf_counter()-t0:.0f}s)  — GR 없이 fusion 만")

    if a.small_objects:
        t1 = time.perf_counter()
        try:
            model = A.Algorithms.small_objects_filter(model)
            log(f"SmallObjectsFilter 완료 {time.perf_counter()-t1:.0f}s")
        except Exception as ex:                                   # noqa: BLE001
            log(f"SmallObjectsFilter 실패({type(ex).__name__}: {ex}) — 건너뜀")

    for var in [v.strip() for v in a.variants.split(",") if v.strip()]:
        st = A.PoissonFusionSettingsDTO.default()
        if var == "all":
            st.fill_type = A.FillHolesType.ALL
        elif var.startswith("byradius"):
            st.fill_type = A.FillHolesType.BY_RADIUS
            st.max_hole_radius = float(var[len("byradius"):] or st.max_hole_radius)
        t1 = time.perf_counter()
        try:
            fused = A.Algorithms.poisson_fusion(model, st)
            obj = OUT / f"fusion_{var}.obj"
            fused.save_obj(str(obj))
            nb, ntri, wt = boundary_edges(obj)
            import open3d as o3d
            m = o3d.io.read_triangle_mesh(str(obj)); cl = np.asarray(m.cluster_connected_triangles()[0]); sz = np.sort(np.bincount(cl))[::-1]
            log(f"[{var:10s}] fill={A.FillHolesType(st.fill_type).name} r={st.max_hole_radius:.0f}mm  "
                f"{time.perf_counter()-t1:.0f}s → {obj.name}: {len(m.triangles):,} f · 덩어리 {len(sz)}개 (최대 {sz[0]/len(m.triangles)*100:.0f}%)"
                f" · 최대덩어리 watertight={wt} (열린 경계엣지 {nb:,} / {ntri:,} f)")
        except Exception as ex:                                   # noqa: BLE001
            log(f"[{var:10s}] ✘ 실패({type(ex).__name__}: {ex})")
    (OUT / "fusion_report.txt").write_text("\n".join(rep) + "\n", encoding="utf-8")
    log(f"→ {OUT}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
