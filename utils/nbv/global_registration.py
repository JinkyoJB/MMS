"""global_registration.py — **초기값 없는** 점군 정합. sim·real 공용.

언제 쓰나 — 두 정합 문제의 구분
------------------------------
    문제                        초기값        중첩    방법
    Phase 1·2 패스 간 (수 mm)   있음(FK+θ)    높음    ICP  ← utils/nbv/icp_strategy
    **Phase 3 flip 후**         **없음**      낮음    **이 모듈**

flip 은 사람이(또는 sim 이) 물체를 뒤집으므로 상대자세를 신뢰할 수 없다.

왜 '명목 회전 hint + ICP' 가 아닌가
----------------------------------
그 경로는 **이미 폐기됐다.** 회전 벤치마크(`scripts/artec/reg_benchmark/README.md`)에서
`meta_hint` 는 아예 **"오답 확인용" baseline** 으로 분류돼 있다:

    "meta.npz 의 recorded hint(90/180°)는 이 데이터엔 **오답**(적용 시 오히려 어긋남)"

사람 손회전은 ±10° 이상 틀어지고, ICP 는 그 정도 초기오차에서 국소최소에 빠진다.
그래서 **특징 기반 전역 정합**으로 초기값을 만들고 ICP 로 마무리한다.

방법 (벤치마크에서 검증된 두 가지)
    1. FPFH + RANSAC — 특징 대응 + RANSAC. 견고하지만 느리다.
    2. FGR (Fast Global Registration) — 대응 없이 최적화. 빠르다.
둘 다 실패하면 호출자가 폴백(예: hint)을 결정하도록 `None` 을 돌려준다.
"""
from __future__ import annotations

import time

import numpy as np

FEATURE_VOXEL_M = 0.004     # FPFH/FGR 용 coarse downsample (벤치마크와 동일)
NORMAL_RADIUS_M = 0.006
ICP_THRESH_M = 0.004


def _prep(pcd, voxel):
    import open3d as o3d
    d = pcd.voxel_down_sample(voxel)
    d.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(
        radius=max(NORMAL_RADIUS_M, voxel * 2.0), max_nn=30))
    f = o3d.pipelines.registration.compute_fpfh_feature(
        d, o3d.geometry.KDTreeSearchParamHybrid(radius=voxel * 5.0, max_nn=100))
    return d, f


def _icp_refine(src, ref, T_init, thresh=ICP_THRESH_M):
    import open3d as o3d
    if not ref.has_normals():
        ref.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(
            radius=NORMAL_RADIUS_M, max_nn=30))
    r = o3d.pipelines.registration.registration_icp(
        src, ref, thresh, T_init,
        o3d.pipelines.registration.TransformationEstimationPointToPlane(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=60))
    return np.asarray(r.transformation, float), float(r.fitness), float(r.inlier_rmse)


def fpfh_ransac(src, ref, voxel=FEATURE_VOXEL_M, seed=0):
    import open3d as o3d
    s, sf = _prep(src, voxel)
    r, rf = _prep(ref, voxel)
    res = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        s, r, sf, rf, True, voxel * 1.5,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(False), 3,
        [o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
         o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(voxel * 1.5)],
        o3d.pipelines.registration.RANSACConvergenceCriteria(400000, 0.999))
    return np.asarray(res.transformation, float)


def fast_global(src, ref, voxel=FEATURE_VOXEL_M):
    import open3d as o3d
    s, sf = _prep(src, voxel)
    r, rf = _prep(ref, voxel)
    res = o3d.pipelines.registration.registration_fgr_based_on_feature_matching(
        s, r, sf, rf,
        o3d.pipelines.registration.FastGlobalRegistrationOption(
            maximum_correspondence_distance=voxel * 1.5))
    return np.asarray(res.transformation, float)


def register_no_init(src_pts, ref_pts, *, voxel=FEATURE_VOXEL_M,
                     min_fitness=0.25, max_rmse_m=0.004, log=None):
    """(T, info) — 초기값 없이 src→ref 정합. 실패 시 (None, info).

    FGR → FPFH+RANSAC 순으로 시도하고 각각 ICP 로 마무리한 뒤 **더 나은 쪽**을 고른다.
    (FGR 이 빠르므로 먼저. 저중첩에서는 RANSAC 이 더 견고할 때가 있어 둘 다 본다.)
    게이트: fitness ≥ min_fitness 이고 rmse ≤ max_rmse_m.
    """
    import open3d as o3d
    src = o3d.geometry.PointCloud()
    src.points = o3d.utility.Vector3dVector(np.asarray(src_pts, float))
    ref = o3d.geometry.PointCloud()
    ref.points = o3d.utility.Vector3dVector(np.asarray(ref_pts, float))

    best, info = None, {"tried": []}
    for name, fn in (("fgr", fast_global), ("fpfh_ransac", fpfh_ransac)):
        t0 = time.perf_counter()
        try:
            T0 = fn(src, ref, voxel)
            T, fit, rmse = _icp_refine(src, ref, T0)
        except Exception as e:                       # noqa: BLE001
            info["tried"].append((name, "예외", str(e)[:40], 0.0))
            continue
        dt = time.perf_counter() - t0
        info["tried"].append((name, fit, rmse, dt))
        if log:
            log(f"[greg] {name}: fitness={fit:.3f} rmse={rmse*1000:.2f}mm {dt:.1f}s")
        if fit >= min_fitness and rmse <= max_rmse_m:
            if best is None or fit > best[1]:
                best = (T, fit, rmse, name)
    if best is None:
        return None, info
    info["method"], info["fitness"], info["rmse"] = best[3], best[1], best[2]
    return best[0], info


def _rot_angle_deg(A, B):
    R = np.asarray(A, float)[:3, :3] @ np.asarray(B, float)[:3, :3].T
    return float(np.degrees(np.arccos(np.clip((np.trace(R) - 1) / 2.0, -1.0, 1.0))))


def register_consensus(src_pts, ref_pts, *, voxel=FEATURE_VOXEL_M,
                       agree_rot_deg=5.0, agree_trans_m=0.005,
                       bbox_tol_m=0.012, inflation_tol_m=0.005, log=None):
    """(T, info) — **fitness 를 믿지 않는** 전역 정합.

    왜 fitness 가 게이트가 못 되나 (실측)
    ------------------------------------
        중첩 49%  fitness 0.795  ΔR **170°**   ← 완전히 틀렸는데 fitness 높음
        중첩 86%  fitness 0.864  ΔR **0.0°**   ← 정답
    회전대칭 표면은 어긋난 각도로도 잘 겹쳐 fitness 가 올라간다(face-merging).
    그래서 fitness 대신 **독립 근거 두 가지**로 판정한다:

      1. **방법 간 합의** — FGR 과 FPFH+RANSAC 은 원리가 다르다. 둘이 같은 답을
         내면 우연히 같은 오답에 빠졌을 가능성이 낮다.
      2. **형상 일치(AABB)** — 같은 물체이므로 정합 후 bbox 가 master 와 맞아야 한다.
         회전이 틀리면 bbox 가 어긋난다(대칭축 둘레 회전은 예외지만, 그 경우는
         표면이 실제로 일치하므로 무해하다).

    ※ 전제: **전회전(360°) 스윕**으로 중첩이 충분할 것. 부분 관측(중첩 20~50%)에서는
      두 방법이 나란히 실패해 합의가 성립할 수 있다(실측 확인) — 이 함수는 flip 후
      전회전 스캔처럼 중첩이 큰 경우에만 신뢰한다.
    """
    import open3d as o3d
    S = np.asarray(src_pts, float)
    Rf = np.asarray(ref_pts, float)
    src = o3d.geometry.PointCloud(); src.points = o3d.utility.Vector3dVector(S)
    ref = o3d.geometry.PointCloud(); ref.points = o3d.utility.Vector3dVector(Rf)

    cand = {}
    for name, fn in (("fgr", fast_global), ("fpfh_ransac", fpfh_ransac)):
        try:
            T0 = fn(src, ref, voxel)
            T, fit, rmse = _icp_refine(src, ref, T0)
            cand[name] = (T, fit, rmse)
            if log:
                log(f"[greg] {name}: fitness={fit:.3f} rmse={rmse*1000:.2f}mm")
        except Exception as e:                       # noqa: BLE001
            if log:
                log(f"[greg] {name} 예외: {type(e).__name__}")
    info = {"cand": {k: (v[1], v[2]) for k, v in cand.items()}}
    if len(cand) < 2:
        info["reason"] = "두 방법 중 하나만 성공 — 합의 불가"
        return None, info

    (Ta, fa, _), (Tb, fb, _) = cand["fgr"], cand["fpfh_ransac"]
    dR = _rot_angle_deg(Ta, Tb)
    dt = float(np.linalg.norm(Ta[:3, 3] - Tb[:3, 3]))
    info.update(agree_rot_deg=dR, agree_trans_mm=dt * 1000.0)
    if dR > agree_rot_deg or dt > agree_trans_m:
        info["reason"] = f"방법 간 불일치 ΔR={dR:.1f}° Δt={dt*1000:.1f}mm"
        return None, info

    T = Ta if fa >= fb else Tb
    moved = S @ T[:3, :3].T + T[:3, 3]
    d_bbox = np.abs((moved.max(0) - moved.min(0)) - (Rf.max(0) - Rf.min(0)))
    info["bbox_diff_mm"] = (d_bbox * 1000.0).round(1).tolist()
    if float(d_bbox.max()) > bbox_tol_m:
        info["reason"] = f"형상 불일치 bbox Δ={d_bbox.max()*1000:.1f}mm"
        return None, info
    # ★ 3. **위치** 일치 — 위 검사는 bbox '크기'만 본다. 대칭축 방향 평행이동은
    #    크기를 바꾸지 않아 통과해 버린다(실측: protein_drink 가 긴 축으로 10mm
    #    밀린 채 합의·형상 게이트를 모두 통과 → F@1mm 75.5→56.5% 회귀).
    from utils.nbv.icp_strategy import union_inflation_m
    infl = union_inflation_m(moved, Rf)
    info["inflation_mm"] = round(infl * 1000.0, 1)
    if infl > inflation_tol_m:
        info["reason"] = (f"정합 후 물체가 {infl*1000:.1f}mm 커짐 "
                          f"(한계 {inflation_tol_m*1000:.0f}mm — 평행이동 오류)")
        return None, info
    info["method"] = "fgr" if fa >= fb else "fpfh_ransac"
    info["fitness"] = max(fa, fb)
    return T, info
