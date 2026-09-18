# mms/nbv/icp_strategy.py
#
# Roll 전략 (docs/2_control_layers.md §5) + ICP triple gate (§6.3).

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation as R

from utils.nbv.manual_picker import compute_camera_pose_from_normal


# ─────────────────────────────────────────────────────────────────────────────
# Roll 전략
# ─────────────────────────────────────────────────────────────────────────────

def pick_icp_roll(
    p_i_O: np.ndarray,
    n_i_O: np.ndarray,
    p_cam_prev_O: np.ndarray,
    distance_m: float,
    world_up: np.ndarray = np.array([0.0, 0.0, 1.0]),
    R_CO_prev: Optional[np.ndarray] = None,
    fallback_thresh: float = 0.2,
) -> float:
    """
    ICP 친화 roll 선택 (§5.2).

    전략 (C): 새 카메라에서 이전 카메라 중심을 향하는 방향 `u_O` 가 새 카메라 프레임의
              y 축(아래) 과 일치하도록 roll* 을 결정.
                  roll* = atan2(u_cam_0[0], u_cam_0[1])
              (u_cam_0 = R_CO(roll=0).T @ u_O)

    Fallback (A): (C) 의 광축 투영이 `‖u_cam_xy‖/‖u_cam‖ < fallback_thresh` 이면
                  이전 프레임 y_cam 과 새 y_cam 을 맞추는 roll 로 전환.

    Parameters
    ----------
    p_i_O        : (3,) 후보 surface 점 (O)
    n_i_O        : (3,) 후보 바깥 normal (O)  — z<0 강제 여부는 caller 가 결정
    p_cam_prev_O : (3,) 이전 카메라 원점 (O)
    distance_m   : float — 후보 camera 가 p_i 에서 떨어지는 거리 (일관성 유지용)
    world_up     : (3,)
    R_CO_prev    : (3,3) 이전 카메라 방향 행렬 (C_prev → O). (A) fallback 에서 사용.
    fallback_thresh : float

    Returns
    -------
    roll_rad : float
    """
    # roll=0 기준 카메라 포즈
    T_CO_0 = compute_camera_pose_from_normal(
        surface_point=p_i_O,
        normal=n_i_O,
        distance_m=distance_m,
        roll_rad=0.0,
        world_up=world_up,
    )
    R_CO_0 = T_CO_0[:3, :3]
    p_cam_new_O = T_CO_0[:3, 3]

    # u_O : 새 카메라 → 이전 카메라 방향 (O)
    u = np.asarray(p_cam_prev_O, dtype=float) - p_cam_new_O
    u_norm = np.linalg.norm(u)
    if u_norm < 1e-9:
        # 이전/현재 원점이 사실상 동일 — fallback (A), R_CO_prev 없으면 0
        if R_CO_prev is None:
            return 0.0
        return _pick_roll_match_up(R_CO_0, R_CO_prev, world_up)

    u = u / u_norm

    # 새 카메라 frame 으로 투영
    u_cam = R_CO_0.T @ u                                         # (3,)
    xy_norm = float(np.linalg.norm(u_cam[:2]))
    tot_norm = float(np.linalg.norm(u_cam))
    if tot_norm < 1e-9 or xy_norm / tot_norm < float(fallback_thresh):
        # degenerate — (C) 가 수치 불안정 → (A) fallback
        if R_CO_prev is None:
            return 0.0
        return _pick_roll_match_up(R_CO_0, R_CO_prev, world_up)

    # 전략 (C) — y_cam(down) 이 u_cam 방향과 일치하도록
    roll = float(np.arctan2(u_cam[0], u_cam[1]))
    return roll


def _pick_roll_match_up(
    R_CO_0: np.ndarray,
    R_CO_prev: np.ndarray,
    world_up: np.ndarray,
) -> float:
    """
    Fallback (A) — 이전 y_cam 을 새 y_cam 에 맞춘다.
        roll* = atan2(y_prev_cam_0[0], y_prev_cam_0[1])
    """
    y_prev_O = R_CO_prev[:, 1]                  # 이전 카메라의 y 축 (O)
    y_prev_cam_0 = R_CO_0.T @ y_prev_O           # 새 카메라 (roll=0) 에 투영
    return float(np.arctan2(y_prev_cam_0[0], y_prev_cam_0[1]))


# ─────────────────────────────────────────────────────────────────────────────
# ICP triple gate (§6.3)
# ─────────────────────────────────────────────────────────────────────────────

def union_inflation_m(src_moved: np.ndarray, ref: np.ndarray) -> float:
    """정합 후 **합집합 bbox 가 기준보다 얼마나 커지는가** (축별 최대, m).

    같은 물체의 다른 관측이므로 올바로 정합되면 0 에 가깝다. 새로 붙는 면(바닥
    등)은 이미 master 가 본 실루엣 **안쪽**을 채우므로 bbox 를 넓히지 않는다.

    왜 필요한가 — bbox **크기**만 비교하면 대칭축 방향 평행이동을 못 잡는다
    (크기는 그대로다). 실측(2026-08-19 9종 스윕):

        정상 7종      팽창 −0.1 ~ 3.8mm   F@1mm 67~99%
        protein_drink 팽창 10.0mm         F@1mm 56.5%  (긴 축으로 10mm 밀림)
        laundry       팽창  8.5mm         F@1mm 45.1%

    두 회귀 모두 기존 게이트(방법 간 합의·bbox 크기·drift)를 통과했다.
    """
    src_moved = np.asarray(src_moved, float)
    ref = np.asarray(ref, float)
    lo = np.minimum(src_moved.min(0), ref.min(0))
    hi = np.maximum(src_moved.max(0), ref.max(0))
    return float(((hi - lo) - (ref.max(0) - ref.min(0))).max())


#: 패치(nbv 부분 스윕·밴드) 와 flip 패스의 이동 허용치. flip 은 사람 손회전
#  ±10° 오차를 안고 들어오므로 더 넓다. **완화하면 안 된다**(sim 실측 2026-08-18:
#  RMSE 임계 2→4mm 로 풀자 뒤집힌 바닥면이 옆면에 미끄러져 붙었다).
REFINE_SCALES_M = (0.020, 0.008, 0.004)     # coarse→fine 대응거리
#  ★ patch 한계는 **초기값(기구학+hand-eye)의 그럴듯한 오차 크기**여야 한다. 그보다
#    큰 "보정" 은 오차를 고친 게 아니라 다른 최소점으로 미끄러진 것이다. 30mm 였을 때
#    (2026-09-18 sim, 기구학 정확): 세제 바닥 밴드 패치가 fitness 0.99·RMSE 1.9mm 로
#    18.6~28.9mm 접선 방향으로 밀려 붙었고 게이트 전부 통과 — 겹침이 얇은 무늬 없는
#    띠라 접선 이동이 관측 불가(bbox 도 안 부푼다). 8mm/2° 면 그런 해는 기각되고
#    기구학이 남는다. 실물 hand-eye 가 8mm 를 넘으면 정합이 영영 안 걸리는데,
#    그건 캘리브를 고칠 일이지 여기를 풀 일이 아니다.
REFINE_DRIFT = {"patch": (0.008, 2.0), "flip": (0.080, 15.0)}   # (m, deg)
REFINE_INFLATION_M = 0.005                  # 정합이 물체를 부풀리는 한계
REFINE_FITNESS_MIN = 0.20
REFINE_MIN_PTS = 300
#: ★ 겹침 영역 트리밍 — 초기 자세에서 master 점이 이 반경 안에 있는 src 점만 ICP 에
#  넣는다. nbv 패치는 **대부분이 새 면**이라(그게 목적이다) 전체를 넣으면 새 면이
#  가까운 기존 면으로 미끄러져 붙는다. 실측 2026-09-18 sim(기구학 정확, 세제 바닥
#  밴드 패치): 전체 ICP 가 18.6mm 위로 끌어올렸고 fitness 0.99·RMSE 1.9mm·drift·팽창
#  게이트를 **전부 통과**했다 — 새 면이 master 의 최하단 점들에 맞춰진 것. 같은
#  기록의 다른 반복은 306mm 폭주(drift 게이트만 잡음). 반경은 예상 초기 오차(real
#  hand-eye 수 mm)보다 크고 "새 면" 간격(15mm+)보다 작아야 한다.
REFINE_OVERLAP_R_M = 0.008
REFINE_OVERLAP_MIN_FRAC = 0.10       # 겹침 점이 패치의 이 비율 미만이면 정합하지 않는다


def refine_to_master(src_pts_m, tgt_pts_m, T_init=None, *, mode: str = "patch",
                     voxel_m: float = 0.002, scales_m=REFINE_SCALES_M,
                     max_iter: int = 50, on_step=None, log=None) -> Optional[IcpResult]:
    """새 관측(src)을 누적 master(tgt)에 붙이는 **sim·real 공용** 정합.

    단위 m, 프레임은 둘이 같기만 하면 된다(sim 은 base, real 은 B 로 내린 PCD).
    `T_init` = 기구학이 준 초기값(없으면 identity). 반환 IcpResult 의 `T_refined`
    는 src→tgt 변환(m). 점이 모자라면 None.

    방법 = coarse→fine point-to-plane ICP + 3중 게이트(RMSE·fitness·drift) +
    **팽창 게이트**(합집합 bbox 가 커지면 평행이동 오류로 보고 기각). 게이트에
    걸리면 `ok=False` 이고 호출자는 초기값(기구학)을 그대로 쓴다.

    왜 공용인가 (2026-09-18) — 그때까지 sim 은 이 다단 ICP, real 은 colored ICP
    (색 가중 0.5, 단일 대응거리 30mm)로 **다른 방법**이었고, real 의 nbv 병합은
    정합 자체가 없었다(기구학 T_pre 만). sim 으로 real 을 검증하려면 같은 함수여야
    한다. 색은 쓰지 않는다 — sim 점군에 색이 없어서 같은 동작이 안 된다.
    `on_step` 은 단계 사이에 부르는 훅(sim 이 Isaac UI 를 펌핑하는 데 쓴다).
    """
    S = np.asarray(src_pts_m, float); Tt = np.asarray(tgt_pts_m, float)
    if len(S) < REFINE_MIN_PTS or len(Tt) < REFINE_MIN_PTS:
        return None
    def _pcd(P):
        pc = o3d.geometry.PointCloud()
        pc.points = o3d.utility.Vector3dVector(P)
        return pc.voxel_down_sample(float(voxel_m)) if voxel_m else pc
    # ── 겹침 영역만 정합에 쓴다 (REFINE_OVERLAP_R_M 주석) ───────────────────
    T0 = np.eye(4) if T_init is None else np.asarray(T_init, float)
    S0 = S @ T0[:3, :3].T + T0[:3, 3]
    from scipy.spatial import cKDTree
    d_near, _ = cKDTree(Tt).query(S0, k=1, distance_upper_bound=REFINE_OVERLAP_R_M)
    ovl = np.isfinite(d_near)
    n_ovl, frac = int(ovl.sum()), float(ovl.mean())
    if n_ovl < REFINE_MIN_PTS or frac < REFINE_OVERLAP_MIN_FRAC:
        if log:
            log(f"(icp/{mode}) 겹침 점 {n_ovl}({frac*100:.0f}%) — 정합 안 함(초기값 유지)")
        return IcpResult(False, f"겹침 부족 {n_ovl}점/{frac*100:.0f}% "
                                f"(≥{REFINE_MIN_PTS}점·{REFINE_OVERLAP_MIN_FRAC*100:.0f}% 필요)",
                         T0, float("inf"), 0.0, 0.0, 0.0)
    if log:
        log(f"(icp/{mode}) 겹침 영역 {n_ovl}/{len(S)}점({frac*100:.0f}%) 으로 정합")
    src, tgt = _pcd(S[ovl]), _pcd(Tt)
    drift_t, drift_r = REFINE_DRIFT.get(mode, REFINE_DRIFT["patch"])
    T = np.eye(4) if T_init is None else np.asarray(T_init, float).copy()
    res = None
    for corr in scales_m:
        if on_step is not None:
            on_step()
        res = icp_with_gates(src, tgt, T, max_correspondence_distance=float(corr),
                             max_iter=int(max_iter),
                             rmse_thresh=float(corr) / 2.0,
                             fitness_thresh=REFINE_FITNESS_MIN,
                             drift_trans_m=drift_t, drift_rot_deg=drift_r,
                             max_inflation_m=REFINE_INFLATION_M)
        if log:
            log(f"(icp/{mode}) corr={corr*1000:.0f}mm ok={res.ok} "
                f"fitness={res.fitness:.3f} rmse={res.rmse*1000:.2f}mm "
                f"Δt={res.delta_translation_m*1000:.1f}mm "
                f"Δr={res.delta_rotation_deg:.1f}° [{res.reason}]")
        T = res.T_refined
    if res is not None and res.ok:
        # ★ 누적 drift 게이트 — 단계별 게이트만 있으면 30mm 한계를 20mm 씩 세 번
        #   나눠 넘어간다(단위 검사 2026-09-18: 60mm 오차가 ok 로 통과). 초기값
        #   대비 **총 이동**으로 다시 건다.
        dT = np.linalg.inv(np.eye(4) if T_init is None else np.asarray(T_init, float)) @ T
        dt = float(np.linalg.norm(dT[:3, 3]))
        try:
            dr = float(np.degrees(np.linalg.norm(R.from_matrix(dT[:3, :3]).as_rotvec())))
        except Exception:
            dr = 0.0
        if dt > drift_t or dr > drift_r:
            res = IcpResult(False, f"누적 drift t={dt*1000:.1f}mm / r={dr:.1f}° "
                                   f"(limit {drift_t*1000:.0f}mm / {drift_r:.0f}°)",
                            T, res.rmse, res.fitness, dt, dr)
        else:
            res = IcpResult(True, "ok", T, res.rmse, res.fitness, dt, dr)
    return res


@dataclass
class IcpResult:
    ok: bool
    reason: str
    T_refined: np.ndarray           # (4,4)
    rmse: float
    fitness: float
    delta_translation_m: float
    delta_rotation_deg: float


def icp_with_gates(
    source_pcd: o3d.geometry.PointCloud,
    target_pcd: o3d.geometry.PointCloud,
    init_T: np.ndarray,
    max_correspondence_distance: float = 0.005,
    max_iter: int = 50,
    rmse_thresh: float = 0.001,
    fitness_thresh: float = 0.3,
    drift_trans_m: float = 0.020,
    drift_rot_deg: float = 10.0,
    max_inflation_m: Optional[float] = None,
) -> IcpResult:
    """
    Point-to-plane ICP + triple gate (RMSE / fitness / drift).

    source → target 방향. `init_T` 는 source 가 target 좌표계로 이동하는 초기 추정 T.

    모든 게이트 통과 시 `ok=True`. 하나라도 실패 시 `ok=False` + `reason`.
    """
    if not target_pcd.has_normals():
        target_pcd.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.01, max_nn=30)
        )

    result = o3d.pipelines.registration.registration_icp(
        source_pcd, target_pcd,
        max_correspondence_distance=float(max_correspondence_distance),
        init=init_T,
        estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPlane(),
        criteria=o3d.pipelines.registration.ICPConvergenceCriteria(
            max_iteration=int(max_iter),
        ),
    )

    T_ref = np.asarray(result.transformation, dtype=float)
    delta_T = np.linalg.inv(init_T) @ T_ref
    dt = float(np.linalg.norm(delta_T[:3, 3]))
    try:
        rot = R.from_matrix(delta_T[:3, :3])
        dr_deg = float(np.degrees(np.linalg.norm(rot.as_rotvec())))
    except Exception:
        dr_deg = 0.0

    rmse = float(result.inlier_rmse)
    fitness = float(result.fitness)

    if not np.isfinite(rmse) or rmse > rmse_thresh:
        return IcpResult(False, f"RMSE {rmse*1000:.2f}mm > {rmse_thresh*1000:.1f}mm",
                         T_ref, rmse, fitness, dt, dr_deg)
    if fitness < fitness_thresh:
        return IcpResult(False, f"fitness {fitness:.3f} < {fitness_thresh:.2f}",
                         T_ref, rmse, fitness, dt, dr_deg)
    if dt > drift_trans_m or dr_deg > drift_rot_deg:
        return IcpResult(False,
                         f"drift t={dt*1000:.1f}mm / r={dr_deg:.1f}° "
                         f"(limit {drift_trans_m*1000:.0f}mm / {drift_rot_deg:.1f}°)",
                         T_ref, rmse, fitness, dt, dr_deg)
    if max_inflation_m is not None:
        S = np.asarray(source_pcd.points, float)
        infl = union_inflation_m(S @ T_ref[:3, :3].T + T_ref[:3, 3],
                                 np.asarray(target_pcd.points, float))
        if infl > max_inflation_m:
            return IcpResult(False,
                             f"정합 후 물체가 {infl*1000:.1f}mm 커짐 "
                             f"(한계 {max_inflation_m*1000:.0f}mm — 평행이동 오류)",
                             T_ref, rmse, fitness, dt, dr_deg)

    return IcpResult(True, "ok", T_ref, rmse, fitness, dt, dr_deg)
