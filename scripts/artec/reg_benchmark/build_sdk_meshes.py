"""
각 방법의 정합 변환을 **풀해상도 스캔**에 적용 → Artec SDK Cleaning→Fusion→
Texturize → 텍스처 OBJ. (벤치마크의 거친 Poisson 대신 SDK 품질 재현.)

변환 복원: real_{method}_{1_0,2_0}.ply 오버레이의 src_aligned 와 캐시 세션 점군
(1:1 index)으로 Umeyama → T_{i->0}(미터). 풀해상도 프레임엔 translation×1000(mm).

출력: output/reg_benchmark/results3d/sdk_mesh/{method}.obj (+ .mtl/.png)

실행(mms-env): python -m scripts.artec.reg_benchmark.build_sdk_meshes [--methods fpfh_ransac,...]
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np
import open3d as o3d

PROJECT_ROOT = Path(__file__).resolve().parents[3]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))
for _s in (sys.stdout, sys.stderr):
    try:
        _s.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass

from scripts.artec.reg_benchmark.common import OUT_DIR, CACHE_DIR, DEFAULT_SPROJ, OVERLAY_DIR, MESH_DIR
from scripts.artec.reg_benchmark.methods_artec import _umeyama_rigid

SDK_MESH = MESH_DIR
NAMED6 = ["fpfh_ransac", "fgr", "artec_gr", "geotransformer", "predator",
          "image_match"]


def recover_T(method, si):
    """real_{method}_{si}_0.ply 오버레이 → T_{si->0} (미터)."""
    ov = OVERLAY_DIR / f"real_{method}_{si}_0.ply"
    if not ov.exists():
        return None
    P = np.asarray(o3d.io.read_point_cloud(str(ov)).points)
    s0 = np.asarray(o3d.io.read_point_cloud(
        str(CACHE_DIR / "session_0.ply")).points)
    si_pts = np.asarray(o3d.io.read_point_cloud(
        str(CACHE_DIR / f"session_{si}.ply")).points)
    n0 = len(s0)
    src_aligned = P[n0:]
    m = min(len(src_aligned), len(si_pts))
    return _umeyama_rigid(si_pts[:m], src_aligned[:m])      # si -> 0 (m)


def apply_T_to_scan(scan, T_m):
    """T_m(미터) 를 scan 의 모든 frame_transformation 에 좌측 곱(translation mm)."""
    T_mm = T_m.copy()
    T_mm[:3, 3] = T_mm[:3, 3] * 1000.0
    for j in range(scan.frame_count()):
        old = np.asarray(scan.get_frame_transformation(j), float)
        scan.set_frame_transformation(j, T_mm @ old)


def build_one(method, ArtecClient, artec_base):
    print(f"\n══════ {method} ══════")
    t0 = time.perf_counter()
    entries = ArtecClient.load_project(str(DEFAULT_SPROJ))
    scans = [getattr(e, "scan", None) for e in entries]
    scans = [s for s in scans if s is not None and s.frame_count() > 0]
    print(f"  loaded {len(scans)} scans  ({time.perf_counter()-t0:.1f}s)")

    # scan0=ref 그대로, scan1/2 에 방법 변환 적용
    for si in (1, 2):
        if si >= len(scans):
            continue
        T = recover_T(method, si)
        if T is None:
            print(f"  scan{si}: 변환 없음 — skip")
            continue
        apply_T_to_scan(scans[si], T)
        ang = np.degrees(np.arccos(np.clip((np.trace(T[:3, :3])-1)/2, -1, 1)))
        print(f"  scan{si}: T 적용 (rot {ang:.1f}°)")

    model = artec_base.create_model()
    for s in scans:
        model.add_scan(s)

    def step(name, fn):
        tt = time.perf_counter()
        try:
            out = fn(model)
            print(f"  {name} ok ({time.perf_counter()-tt:.0f}s)")
            return out
        except Exception as e:
            print(f"  {name} 실패: {e}")
            return model

    # Cleaning: outliers_removal(~86s) 사용. small_objects_filter 는 이 입력에서
    # 0x80010201 로 실패하므로 안 씀. outliers 가 stray/fuzz 점 제거 → 깨끗한 fusion.
    m2 = step("OutliersRemoval", ArtecClient.outliers_removal)
    m2 = step("PoissonFusion", lambda mm: ArtecClient.poisson_fusion(m2))
    m2 = step("Texturize", lambda mm: ArtecClient.texturize(m2))

    SDK_MESH.mkdir(parents=True, exist_ok=True)
    obj = SDK_MESH / f"{method}.obj"
    try:
        m2.save_obj(str(obj))
        print(f"  saved → {obj}  (total {time.perf_counter()-t0:.0f}s)")
    except Exception as e:
        print(f"  save_obj 실패: {e}")
        return
    try:
        crop_turntable(obj, method)
    except Exception as e:
        print(f"  turntable crop 실패(무시): {e}")


CW = OUT_DIR / "cache_withplane"


def _session_plane_removed(si):
    """★턴테이블 평면을 '제거된 점'(cache_withplane − cache)에 피팅 → 카톤 면을
    잘못 잡을 수 없음(제거된 점은 정의상 턴테이블). 세션 프레임(미터), 카톤+ 방향."""
    F = np.asarray(o3d.io.read_point_cloud(str(CW / f"session_{si}.ply")).points)
    keep = o3d.io.read_point_cloud(str(CACHE_DIR / f"session_{si}.ply"))
    tree = o3d.geometry.KDTreeFlann(keep)
    rem = [p for p in F if tree.search_knn_vector_3d(p, 1)[2][0] > 0.001 ** 2]
    rem = np.asarray(rem)
    tp = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(rem))
    pl, _ = tp.segment_plane(0.004, 3, 1000)
    n = np.array(pl[:3], float); d = float(pl[3])
    nn = np.linalg.norm(n) + 1e-12; n /= nn; d /= nn
    K = np.asarray(keep.points)
    if np.mean(K @ n + d) < 0:           # 카톤(keep) 이 + 가 되도록
        n, d = -n, -d
    return n, d


def crop_turntable(obj_path, method, margin=0.004, min_keep=0.30):
    """제거된-점 평면(robust)을 각 변환으로 병합프레임에 옮겨 슬랩(+아래) 제거.
    텍스처 유지. flip 등으로 과다제거(<min_keep)면 session0 평면만, 그래도 과하면 skip.
    (단일평면 RANSAC 재검출은 카톤 면을 잘못 잡아 윗면을 지우는 버그 → 사용 금지.)"""
    planes = [_session_plane_removed(0)]              # session0 = 병합 프레임
    for si in (1, 2):
        T = recover_T(method, si)
        if T is None:
            continue
        n, d = _session_plane_removed(si)
        R, t = T[:3, :3], T[:3, 3]
        planes.append((R @ n, d - (R @ n) @ t))       # 카톤+ 보존 (재정렬 금지)

    m = o3d.io.read_triangle_mesh(str(obj_path), enable_post_processing=True)
    V = np.asarray(m.vertices) * 0.001                # SDK mm → m
    tri = np.asarray(m.triangles)

    def below_mask(pls):
        b = np.zeros(len(V), bool)
        for n, d in pls:
            b |= (V @ n + d) < margin
        return b

    below = below_mask(planes)
    if (~below[tri].any(axis=1)).mean() < min_keep:   # 과다제거 → session0 만
        print(f"  ⚠ 과다제거 감지 → session0 평면만 사용")
        below = below_mask(planes[:1])
        if (~below[tri].any(axis=1)).mean() < min_keep:
            print(f"  ⚠ 그래도 과다 → turntable crop 생략(원본 유지)")
            return
    rm = below[tri].any(axis=1)
    nb = len(tri)
    m.remove_triangles_by_mask(rm)
    m.remove_unreferenced_vertices()
    o3d.io.write_triangle_mesh(str(obj_path), m)
    print(f"  turntable crop: tris {nb} → {len(m.triangles)} "
          f"({100*(1-rm.mean()):.0f}% kept, 텍스처 유지)")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--methods", type=str, default=None)
    args = ap.parse_args()
    methods = (args.methods.split(",") if args.methods else NAMED6)

    from mms_artec.sensor.artec_client import ArtecClient
    from mms_artec.sensor import artec_base
    for mth in methods:
        mth = mth.strip()
        try:
            build_one(mth, ArtecClient, artec_base)
        except Exception as e:
            import traceback
            print(f"  ✘ {mth}: {e}")
            traceback.print_exc()
    print("\n[build_sdk_meshes] done.")


if __name__ == "__main__":
    main()
