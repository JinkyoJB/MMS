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
