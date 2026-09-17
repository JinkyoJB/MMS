# utils/nbv/nbv_core.py
#
# nbv (부족면 NBV 보강) 의 **하드웨어 무관 코어** (docs/4_nbv.md §3).
#
# master B-프레임 colored pcd  →  Poisson mesh  →  frontier(구멍) 검출  →
# NBV 목표 카메라 포즈 생성  +  커버리지 수렴 지표.
#
# 로봇/턴테이블/Artec 무의존 → sim·offline pcd 로 단독 검증 가능.
# θ-feasibility(IK+충돌) 는 calib·robot 의존이라 session 에서 wiring
# (`theta_planner.plan_min_motion_theta_analytic`).
#
# Notation: 모든 점/포즈는 단일 프레임(B, meter) 기준. README.md 좌표규약.

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional

import numpy as np
import open3d as o3d

from utils.nbv.frontier import (
    FrontierCandidate,
    extract_boundary_edges,
    extract_frontier_candidates,
)
from utils.nbv.manual_picker import compute_camera_pose_from_normal


# ─────────────────────────────────────────────────────────────────────────────
# pcd → mesh (Poisson)
# ─────────────────────────────────────────────────────────────────────────────

def pcd_to_mesh_poisson(
    pcd: o3d.geometry.PointCloud,
    depth: int = 8,
    normal_radius_m: float = 0.01,
    normal_max_nn: int = 30,
    density_quantile: float = 0.04,
    crop_margin_m: float = 0.01,
) -> o3d.geometry.TriangleMesh:
    """
    B-프레임 colored pcd → Poisson mesh. 저밀도 vertex 제거 + pcd bbox 로 crop
    (Poisson 의 풍선 artifact 차단). 부족면은 **경계(boundary)** 로 남아 frontier 가 잡는다.

    normals 없으면 추정 + 카메라 일관 방향(외향) 으로 orient.
    """
    if len(pcd.points) < 100:
        return o3d.geometry.TriangleMesh()

    if not pcd.has_normals():
        pcd.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(
                radius=normal_radius_m, max_nn=normal_max_nn)
        )
    # 외향 일관화 — centroid 기준 (대상물은 볼록에 가까움)
    c = np.asarray(pcd.points).mean(axis=0)
    pcd.orient_normals_towards_camera_location(camera_location=c)
    pcd.normals = o3d.utility.Vector3dVector(-np.asarray(pcd.normals))  # centroid 바깥쪽

    mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
        pcd, depth=depth, linear_fit=False)
    densities = np.asarray(densities)
    if len(densities) and 0.0 < density_quantile < 1.0:
        thr = np.quantile(densities, density_quantile)
        mesh.remove_vertices_by_mask(densities < thr)

    # pcd bbox(+margin) 밖 풍선 제거
    aabb = pcd.get_axis_aligned_bounding_box()
    aabb.min_bound = np.asarray(aabb.min_bound) - crop_margin_m
    aabb.max_bound = np.asarray(aabb.max_bound) + crop_margin_m
    mesh = mesh.crop(aabb)

    mesh.compute_vertex_normals()
    return mesh


# ─────────────────────────────────────────────────────────────────────────────
# 구멍 검출 + 커버리지 지표
# ─────────────────────────────────────────────────────────────────────────────

def detect_gaps(
    mesh: o3d.geometry.TriangleMesh,
    min_seg_vertices: int = 10,
    min_seg_length: float = 0.01,
    max_seg_length: float = 0.08,
) -> List[FrontierCandidate]:
    """누적 mesh 의 부족면(경계 세그먼트) → frontier 후보. (frontier.py 재사용)"""
    return extract_frontier_candidates(
        mesh,
        min_seg_vertices=min_seg_vertices,
        min_seg_length=min_seg_length,
        max_seg_length=max_seg_length,
    )


def boundary_length(mesh: o3d.geometry.TriangleMesh) -> float:
    """mesh 경계(open edge) 총 길이 (m). watertight → 0."""
    if len(mesh.triangles) == 0:
        return 0.0
    b = extract_boundary_edges(mesh)
    if len(b) == 0:
        return 0.0
    v = np.asarray(mesh.vertices)
    d = v[b[:, 0]] - v[b[:, 1]]
    return float(np.linalg.norm(d, axis=1).sum())


def _fibonacci_sphere(n: int) -> np.ndarray:
    """단위구 위 n 방향 (거의 균일)."""
    k = np.arange(n) + 0.5
    phi = np.arccos(1.0 - 2.0 * k / n)
    gold = np.pi * (1.0 + 5.0 ** 0.5)
    theta = gold * k
    return np.column_stack([
        np.sin(phi) * np.cos(theta),
        np.sin(phi) * np.sin(theta),
        np.cos(phi),
    ])


def angular_coverage(
    mesh: o3d.geometry.TriangleMesh,
    n_dirs: int = 64,
    parallel_thresh_deg: float = 40.0,
) -> float:
    """
    "모든 방향에서 충분히 봤나" 정량 지표 (docs §3.6-4).
    Fibonacci 구의 각 방향에 대해, 그 방향과 거의 평행한 (관측) vertex normal 이
    존재하는 방향의 비율 ∈ [0,1]. 1 에 가까울수록 전 방향 커버.
    """
    if len(mesh.triangles) == 0:
        return 0.0
    if not mesh.has_vertex_normals():
        mesh.compute_vertex_normals()
    nrm = np.asarray(mesh.vertex_normals)
    if len(nrm) == 0:
        return 0.0
    dirs = _fibonacci_sphere(n_dirs)
    cos_thr = np.cos(np.radians(parallel_thresh_deg))
    # (n_dirs, n_vert) dot — vert 많으면 메모리 큼 → vert 다운샘플
    if len(nrm) > 4000:
        idx = np.random.default_rng(0).choice(len(nrm), 4000, replace=False)
        nrm = nrm[idx]
    dots = dirs @ nrm.T                       # (n_dirs, n_vert)
    covered = (dots.max(axis=1) >= cos_thr)   # 각 방향에 평행 normal 존재?
    return float(covered.mean())


@dataclass
class CoverageState:
    boundary_len_m: float
    angular_cov: float
    n_gaps: int


def coverage_state(
    mesh: o3d.geometry.TriangleMesh,
    n_dirs: int = 64,
    parallel_thresh_deg: float = 40.0,
    **gap_kw,
) -> CoverageState:
    return CoverageState(
        boundary_len_m=boundary_length(mesh),
        angular_cov=angular_coverage(mesh, n_dirs, parallel_thresh_deg),
        n_gaps=len(detect_gaps(mesh, **gap_kw)),
    )


def is_converged(
    state: CoverageState,
    boundary_stop_m: float = 0.01,
    coverage_tau: float = 0.95,
) -> bool:
    """boundary 길이→0 AND 각도 커버리지 τ 이상 이면 수렴(완벽 근접)."""
    return (state.boundary_len_m <= boundary_stop_m
            and state.angular_cov >= coverage_tau)


# ─────────────────────────────────────────────────────────────────────────────
# 후보 → NBV 카메라 목표 포즈
# ─────────────────────────────────────────────────────────────────────────────

def nbv_pose_from_candidate(
    cand: FrontierCandidate,
    distance_m: float = 0.225,
    roll_rad: float = 0.0,
    world_up: np.ndarray = np.array([0.0, 0.0, 1.0]),
) -> np.ndarray:
    """
    frontier 후보(대표점 p, 바깥 법선 n) → 카메라 목표 포즈 T_CB_des (C→B).
    광축이 표면을 정면으로 보고, 표면에서 distance_m 떨어진 위치.
    (`compute_camera_pose_from_normal` 재사용; 입력이 B 프레임이면 반환도 C→B.)
    """
    return compute_camera_pose_from_normal(
        surface_point=cand.p_O,
        normal=cand.n_O,
        distance_m=distance_m,
        roll_rad=roll_rad,
        world_up=world_up,
    )


def joint_motion_cost(
    q: np.ndarray,
    q_cur: np.ndarray,
    weights: np.ndarray,
    L: float,
    L_max: float,
    delta: float = 0.3,
) -> float:
    """
    ★ 기본 cost — **로봇 관절 이동 최소** + 큰 구멍 우선 (docs §3.3).
    cost = Σ_i w_i·(q_i − q_cur,i)²  − δ·(L / L_max).
    base 관절 가중↑ (DEFAULT_JOINT_WEIGHTS) → 같은 목표면 손목 우선.
    """
    dq = np.asarray(q, float) - np.asarray(q_cur, float)
    L_hat = L / max(L_max, 1e-9)
    return float(np.sum(np.asarray(weights, float) * dq * dq)) - delta * L_hat


def geometric_cost(
    cand: FrontierCandidate,
    T_CB_des: np.ndarray,
    p_cam_prev: np.ndarray,
    L_max: float,
    delta: float = 0.3,
) -> float:
    """
    대안(Cartesian) — 관절 q 를 못 구할 때의 카메라 이동거리 기준.
    cost = ‖Δp_cam‖² − δ·L̂. **기본은 `joint_motion_cost`** (관절공간이 직접 목표).
    """
    dp = T_CB_des[:3, 3] - np.asarray(p_cam_prev)
    L_hat = cand.L / max(L_max, 1e-9)
    return float(dp @ dp) - delta * L_hat


def gap_normal_elevations_deg(gaps, top_k: int = 8,
                              up_sign: float = +1.0) -> List[float]:
    """큰 gap 들의 바깥법선 elevation(수평 위 각도, deg). 윗면 gap≈90°, 측면≈0°.

    `up_sign` — 작업 프레임에서 어느 z 방향이 '위'인가 (+1 = +Z 가 위).
      sim 은 world 프레임이라 +1. real 은 천장 마운트 base 라 **−1**
      (`docs/collision.md` §6.1) — 부호 없이 n_z 를 그대로 쓰면 윗면 gap 이
      elevation **−90°** 로 나와 윗면/아랫면 판정이 통째로 뒤집힌다.
    """
    s = float(np.sign(up_sign)) or 1.0
    big = sorted(gaps, key=lambda c: -c.L)[:top_k]
    out = []
    for c in big:
        n = np.asarray(c.n_O, float)
        n = n / (np.linalg.norm(n) + 1e-12)
        out.append(math.degrees(math.asin(float(np.clip(s * n[2], -1.0, 1.0)))))
    return out


def plan_nbv_elevation_pose(
    gaps, q_cur, pose_q_fn, swept_free_fn, *,
    joint_weights, el_floor_deg: float = 30.0,
    view_azis_deg=(0., 30., -30., 60., -60., 90., -90., 180.),
    el_extra_deg=(65., 55., 45.), el_cap_deg: float = 88.0,
    visited=(), ensure_els=(), up_sign: float = +1.0,
):
    """★ 공용 NBV 자세선택 (real/sim 동일) — **부족면을 덮을 관측 elevation 자세** 선택.

    캡처가 "로봇 한 자세 + 턴테이블 전회전"으로 통일됐으므로(2026-06-30), NBV = **어느 관측
    elevation 으로 추가 전회전 스캔할지** 고르는 문제. gap 법선 elevation → 필요 el, **도달·
    충돌-free 중 가장 높은 el** 채택(윗면 보강). az 는 도달 가능한 것 중 관절이동 최소.

    하드웨어/프레임/카메라규약(USD vs OpenCV) 차이는 **주입 함수**가 캡슐화:
      pose_q_fn(el_deg, az_deg) -> q | None : 턴테이블축을 (el,az,standoff)에서 보는 카메라→IK q.
      swept_free_fn(q_cur, q)  -> bool      : 이동(swept-path) 충돌-free 여부.

    `visited` = 이미 스캔한 (el, az) 목록. **반드시 넘길 것** — az 를 "관절이동 최소"로
    고르므로, 넘기지 않으면 직전 자세의 이동비용이 0 이라 **같은 자세를 무한 반복**한다
    (실측: el=65 az=-30 을 4회 연속 선택, gap 18→18→20→19 로 안 줄었다).

    ★ 단, 재방문 판정은 **el 로만** 한다(az 는 무시). 이 함수의 전제가 "az 는
      관측 조건을 바꾸지 않는다 — 회전은 턴테이블이 담당한다" 이므로, 같은 el 을
      az 만 바꿔 다시 도는 것은 **새 정보가 0 인데 로봇만 크게 움직이는** 일이다.
      (el, az) 로 판정하면 el 하나당 az 후보 수만큼 재방문이 허용돼 정확히 그
      낭비가 생긴다.
      실측 2026-09-17 (같은 코드·같은 격자, 실측 T_B_F0 vs sim v3 씬,
      seed = 직전 자세 = 런타임과 동일):
        az 를 0°→+30° 로만 바꾸는 데 드는 **최대 관절이동**
          sim  : el45 23.3° · el55 27.7° · el65 23.9°
          real : el45 51.3° · el55 55.5° · el65 47.7°     (≈ 2배)
      real 셀은 턴테이블이 base 에서 수평 799mm·전체 1055mm(도달한계 1090mm)라
      팔이 거의 다 펴진 자세다. 그 근처에서는 야코비안이 나빠 같은 az 변화가
      sim 의 약 2배 관절이동을 요구한다 — 사용자가 본 "로봇이 크게 움직인다" 가
      이것이다. el 로만 판정하면 로봇은 **자기가 편한 az 에 머물고** 고도각만 바꾼다.

      ⚠ 이 수치는 **IK seed 에 민감하다.** 옛 home 값을 seed 로 쓰면 real 이
        6/12 만 풀리고 el65 가 103° 로 나온다 — 해석 IK 가 국소해라서 그렇지
        기하의 한계가 아니다. 실물 home(`XArmInterface.HOME_JOINTS_DEG`)으로
        재면 도달은 12/12 다. 비교할 때 seed 를 반드시 맞출 것.

    Returns (q, el_deg, az_deg) or None(도달 가능 관측자세 없음 = 윗면 도달한계 등).
    """
    if not gaps:
        return None
    # ★ el 로만 재방문 판정 (az 는 커버리지 무관 — 위 docstring 참고)
    seen_el = {round(float(e), 1) for e, _a in visited}
    needs = gap_normal_elevations_deg(gaps, up_sign=up_sign)
    el_need = float(np.clip(np.median(needs), el_floor_deg + 5.0, el_cap_deg))
    # ★ **el_need 에 가까운 순**으로 시도한다. 예전에는 내림차순(=가장 높은 el 우선)이라
    #   el_extra_deg 의 65 가 항상 이겨 el_need 가 사실상 무시됐다. 그 결과 gap 이
    #   측면 위주(법선 elevation 낮음)여도 계속 el=65 를 골라 측면이 안 메워졌다
    #   (실측: NBV 4회 모두 el=65, 측면 gap 11→15 로 오히려 증가).
    #   윗면 gap 이 많으면 el_need 가 높아지므로 "윗면 보강" 의도는 그대로 유지된다.
    el_cands = sorted(set([el_need, el_need - 10.0, el_need - 20.0, el_need - 30.0,
                           *el_extra_deg]), key=lambda e: abs(e - el_need))
    # ★ `ensure_els` — gap 과 무관하게 **최소 한 번** 시도할 고도각을 맨 앞에 놓는다.
    #   오목 물체(컵 등) 내부는 **미관측이라 메시에 없고, 없으면 경계(gap)로도 안 잡힌다.**
    #   그래서 el_need 가 올라갈 근거가 없어 NBV 가 계속 낮은 el 만 고른다(닭·달걀).
    #   실측: el=55° az=0° standoff 0.25 한 자세 + 전회전으로 컵 내벽·내부바닥 100% 커버.
    todo = [float(e) for e in ensure_els
            if not any(abs(float(v[0]) - float(e)) < 1e-6 for v in visited)]
    if todo:
        el_cands = todo + [e for e in el_cands if e not in todo]
    W = np.asarray(joint_weights, float)
    qc = np.asarray(q_cur, float)
    for el in el_cands:
        if el < el_floor_deg or el > el_cap_deg:
            continue
        if round(float(el), 1) in seen_el:
            continue                           # 이 고도각은 이미 전회전 스캔했다
        best = None
        for az in view_azis_deg:
            q = pose_q_fn(float(el), float(az))
            if q is None:
                continue
            if not swept_free_fn(qc, q):
                continue
            dq = np.asarray(q, float) - qc
            cost = float(np.sum(W * dq * dq))
            if best is None or cost < best[0]:
                best = (cost, q, az)
        if best is not None:
            return best[1], float(el), float(best[2])
    return None
