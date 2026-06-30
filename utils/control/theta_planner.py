# mms/control/theta_planner.py
#
# 하위 레이어 내 "planner" 서브레이어.
#
#   입력 : 상위(NBV) 출력 T_CO_des + 현재 턴테이블 θ + 현재 로봇 joint q
#   출력 : θ*, q_des*, T_EB*    — 로봇 관절 이동을 최소화하는 회전각
#
# 그리드 샘플링 방식:
#   각 θ 후보에 대해 solve_T_EB → xArm IK → weighted joint-space cost 계산,
#   feasible 한 후보 중 argmin 선택.
#
# Cost (docs/1_control_layers.md §4 참조)
#   cost(θ) = Σ_i  w_i · (q_des_i(θ) − q_current_i)²   +   w_tt · |θ − θ_current|
#
# Notation: T_AB 는 A → B  (x_B = T_AB @ x_A). README.md / CLAUDE.md 준수.

from __future__ import annotations

import time
from typing import Optional, Sequence, TYPE_CHECKING

import numpy as np

from utils.transforms import TurntableTransformConfig, solve_T_EB
from utils.control.hardware_layer import pose_mat_to_xarm6d

if TYPE_CHECKING:
    from utils.robot.xarm_interface import XArmInterface


# xArm7 기준 joint-space cost 가중치.
# 베이스 쪽 큰 관절(J1, J2)을 손목(J4~J7)보다 비싸게 두어, 같은 목표라면 손목을
# 우선 사용하도록 유도한다. J3 는 중간.
DEFAULT_JOINT_WEIGHTS = np.array(
    [2.0, 2.0, 1.5, 1.0, 1.0, 1.0, 1.0],
    dtype=float,
)


def plan_min_motion_theta(
    T_CO_des: np.ndarray,
    T_OF: np.ndarray,
    T_EC: np.ndarray,
    tt: TurntableTransformConfig,
    robot: "XArmInterface",
    theta_current: float,
    q_current: Optional[np.ndarray] = None,
    theta_samples: Optional[Sequence[float]] = None,
    theta_range: tuple = (-np.pi, np.pi),
    n_samples: int = 72,
    joint_weights: np.ndarray = DEFAULT_JOINT_WEIGHTS,
    w_tt: float = 0.0,
    verbose: bool = True,
) -> dict:
    """
    T_CO_des 를 만족하는 θ 후보 그리드 중 로봇 관절 이동이 최소인 θ* 를 선택.

    Parameters
    ----------
    T_CO_des : (4,4) np.ndarray      상위 레이어 출력 (C → O)
    T_OF, T_EC, tt                   기존 하드웨어 상수
    robot : XArmInterface            xArm IK 호출용
    theta_current : float            현재 턴테이블 각도 (rad). w_tt 적용에 사용.
    q_current : (7,), optional       현재 로봇 joint (rad). None → robot.get_joint_angles()
    theta_samples : array-like, optional
        None 이면 np.linspace(theta_range, n_samples, endpoint=False) 생성.
    theta_range : (lo, hi)
    n_samples : int                  그리드 샘플 수 (기본 72 ≈ 5° 간격)
    joint_weights : (7,)             관절별 가중치 (베이스 쪽이 크게)
    w_tt : float, default=0.0        턴테이블 회전 비용 가중치 (rad 당)
    verbose : bool                   진행/결과 로그

    Returns
    -------
    dict
        "theta"         : float or None  — feasible 없으면 None
        "joints"        : (7,) or None   — q_des*
        "cost"          : float
        "T_EB"          : (4,4) or None
        "pose6d"        : (6,)  [x(mm), y(mm), z(mm), r, p, y(rad)] or None
        "n_feasible"    : int
        "n_samples"     : int
        "all_thetas"    : (N,) np.ndarray
        "all_costs"     : (N,) np.ndarray  — infeasible 은 np.inf
        "all_feasible"  : (N,) bool
    """
    if q_current is None:
        q_current = np.asarray(robot.get_joint_angles(is_radian=True), dtype=float)
    else:
        q_current = np.asarray(q_current, dtype=float)
    if q_current.shape != (7,):
        raise ValueError(f"q_current must be (7,), got {q_current.shape}")

    w = np.asarray(joint_weights, dtype=float)
    if w.shape != (7,):
        raise ValueError(f"joint_weights must be (7,), got {w.shape}")

    if theta_samples is None:
        thetas = np.linspace(
            float(theta_range[0]), float(theta_range[1]),
            int(n_samples), endpoint=False,
        )
    else:
        thetas = np.asarray(theta_samples, dtype=float)
    N = len(thetas)

    costs = np.full(N, np.inf)
    feasible = np.zeros(N, dtype=bool)
    joints_arr = np.full((N, 7), np.nan)

    if verbose:
        print("\n" + "─" * 72)
        print("[Planner] θ 최적화 — 로봇 관절 이동 최소화")
        print("─" * 72)
        print(f"  그리드   : {N} 샘플  range=[{np.degrees(thetas.min()):+.1f}, "
              f"{np.degrees(thetas.max()):+.1f}]°")
        print(f"  현재 θ   : {np.degrees(theta_current):+.2f}°")
        print(f"  현재 q   : {np.round(np.degrees(q_current), 1).tolist()} deg")
        print(f"  weights  : {w.tolist()}  w_tt={w_tt}")

    t0 = time.perf_counter()
    n_calls = 0
    for i, theta in enumerate(thetas):
        T_EB = solve_T_EB(float(theta), T_CO_des, T_OF, T_EC, tt)
        pose6d = pose_mat_to_xarm6d(T_EB)
        code, q = robot.arm.get_inverse_kinematics(
            pose=pose6d.tolist(),
            input_is_radian=True,
            return_is_radian=True,
        )
        n_calls += 1
        if code != 0:
            continue
        q_arr = np.asarray(q[:7], dtype=float)
        dq = q_arr - q_current
        cost_robot = float(np.sum(w * dq * dq))
        cost_tt = float(w_tt * abs(float(theta) - float(theta_current)))
        costs[i] = cost_robot + cost_tt
        feasible[i] = True
        joints_arr[i] = q_arr

    n_feasible = int(feasible.sum())
    elapsed = time.perf_counter() - t0

    if n_feasible == 0:
        if verbose:
            print(f"  ✘ feasible θ 없음  (IK 성공 0/{N})  elapsed={elapsed:.2f}s")
            print("─" * 72)
        return {
            "theta": None, "joints": None, "cost": float("inf"),
            "T_EB": None, "pose6d": None,
            "n_feasible": 0, "n_samples": N,
            "all_thetas": thetas, "all_costs": costs, "all_feasible": feasible,
        }

    i_best = int(np.argmin(costs))
    theta_best = float(thetas[i_best])
    q_best = joints_arr[i_best].copy()
    T_EB_best = solve_T_EB(theta_best, T_CO_des, T_OF, T_EC, tt)
    pose6d_best = pose_mat_to_xarm6d(T_EB_best)

    if verbose:
        print(f"  ✓ θ* = {np.degrees(theta_best):+.2f}°   "
              f"Δθ = {np.degrees(theta_best - theta_current):+.2f}°   "
              f"cost = {costs[i_best]:.4f}")
        dq_best = q_best - q_current
        print(f"    Δq (deg) : {np.round(np.degrees(dq_best), 2).tolist()}")
        print(f"    feasible {n_feasible}/{N}  elapsed={elapsed:.2f}s "
              f"({1000*elapsed/max(n_calls,1):.1f} ms/call)")
        # 상위 3 후보
        order = np.argsort(costs)
        tops = [(int(order[k]), costs[int(order[k])]) for k in range(min(3, N))
                if costs[int(order[k])] < np.inf]
        if tops:
            print("    top-3: " + ",  ".join(
                f"θ={np.degrees(thetas[j]):+6.1f}° (cost={c:.3f})" for j, c in tops
            ))
        print("─" * 72)

    return {
        "theta": theta_best,
        "joints": q_best,
        "cost": float(costs[i_best]),
        "T_EB": T_EB_best,
        "pose6d": pose6d_best,
        "n_feasible": n_feasible,
        "n_samples": N,
        "all_thetas": thetas,
        "all_costs": costs,
        "all_feasible": feasible,
    }


def plan_min_motion_theta_analytic(
    T_CO_des: np.ndarray,
    T_OF: np.ndarray,
    T_EC: np.ndarray,
    tt: TurntableTransformConfig,
    ik_fn,
    theta_current: float,
    q_current: np.ndarray,
    collision_fn=None,
    theta_samples: Optional[Sequence[float]] = None,
    theta_range: tuple = (-np.pi, np.pi),
    n_samples: int = 72,
    joint_weights: np.ndarray = DEFAULT_JOINT_WEIGHTS,
    w_tt: float = 0.0,
    seed_from_current: bool = True,
    pose_from_T=None,
    verbose: bool = True,
) -> dict:
    """
    `plan_min_motion_theta` 의 **해석 IK + 충돌검사** 변형 (결정 2026-06: SDK IK 미사용).

    xArm SDK 핸들(`robot.arm.get_inverse_kinematics`) 대신 주입된 `ik_fn` 으로 풀고,
    `collision_fn` 으로 충돌 자세를 거른다. real·sim 동일 호출 (backend-agnostic).
    반환 dict 스키마는 `plan_min_motion_theta` 와 동일 (drop-in).

    Parameters
    ----------
    ik_fn : callable  pose6d([x,y,z mm, rpy rad]) → (q(7,), ok: bool)
        예: `utils.robot.xarm7_kinematics.ik` (seed kwarg 지원 시 warm-start 사용).
    q_current : (7,)  현재 관절각 (rad) — cost 기준 + IK seed.
    collision_fn : callable|None  q(7,) → bool (True=충돌). None 이면 충돌검사 skip.
        예: `lambda q: pose_collision(world, q, T_EC)[0]`.
    seed_from_current : bool  True 면 각 θ 의 IK 를 q_current 로 warm-start
        (가장 가까운 해 branch → min-motion + 수치 IK 수렴 가속).
    pose_from_T : callable|None  T_EB(4x4, m) → pose6d([x,y,z mm, rpy rad]).
        None 이면 **kin 규약**(`xarm7_kinematics.R_to_euler_xyz`) 으로 변환. ⚠ kin 의 euler
        규약은 scipy `as_euler('xyz')`(=`pose_mat_to_xarm6d`)와 **불일치**하므로, ik_fn 이
        해석 IK(kin) 일 때 scipy 변환을 쓰면 엉뚱한 해로 수렴한다. 기본 kin 변환을 쓸 것.
    """
    if pose_from_T is None:
        from utils.robot import xarm7_kinematics as _kin

        def pose_from_T(T_EB):                       # noqa: E306 (지역 기본)
            return np.concatenate([T_EB[:3, 3] * 1000.0,
                                   _kin.R_to_euler_xyz(T_EB[:3, :3])])

    q_current = np.asarray(q_current, dtype=float)
    if q_current.shape != (7,):
        raise ValueError(f"q_current must be (7,), got {q_current.shape}")
    w = np.asarray(joint_weights, dtype=float)
    if w.shape != (7,):
        raise ValueError(f"joint_weights must be (7,), got {w.shape}")

    if theta_samples is None:
        thetas = np.linspace(
            float(theta_range[0]), float(theta_range[1]),
            int(n_samples), endpoint=False,
        )
    else:
        thetas = np.asarray(theta_samples, dtype=float)
    N = len(thetas)

    costs = np.full(N, np.inf)
    feasible = np.zeros(N, dtype=bool)
    joints_arr = np.full((N, 7), np.nan)
    n_collision = 0

    def _solve_ik(pose6d):
        # ik_fn 은 (q, ok) 또는 q|None 둘 다 허용.
        if seed_from_current:
            try:
                out = ik_fn(pose6d, seed=q_current)
            except TypeError:
                out = ik_fn(pose6d)
        else:
            out = ik_fn(pose6d)
        if out is None:
            return None
        if isinstance(out, tuple):
            q_sol, ok = out
            return np.asarray(q_sol, dtype=float) if ok else None
        return np.asarray(out, dtype=float)

    t0 = time.perf_counter()
    for i, theta in enumerate(thetas):
        T_EB = solve_T_EB(float(theta), T_CO_des, T_OF, T_EC, tt)
        pose6d = pose_from_T(T_EB)
        q_arr = _solve_ik(pose6d)
        if q_arr is None:
            continue
        if collision_fn is not None and bool(collision_fn(q_arr)):
            n_collision += 1
            continue
        dq = q_arr - q_current
        cost_robot = float(np.sum(w * dq * dq))
        cost_tt = float(w_tt * abs(float(theta) - float(theta_current)))
        costs[i] = cost_robot + cost_tt
        feasible[i] = True
        joints_arr[i] = q_arr

    n_feasible = int(feasible.sum())
    elapsed = time.perf_counter() - t0

    if verbose:
        print("\n" + "─" * 72)
        print("[Planner/analytic] θ 최적화 — 해석 IK + 충돌검사")
        print(f"  그리드 {N}  feasible {n_feasible}  collision-reject {n_collision}  "
              f"elapsed={elapsed:.2f}s")

    if n_feasible == 0:
        return {
            "theta": None, "joints": None, "cost": float("inf"),
            "T_EB": None, "pose6d": None,
            "n_feasible": 0, "n_samples": N, "n_collision": n_collision,
            "all_thetas": thetas, "all_costs": costs, "all_feasible": feasible,
        }

    i_best = int(np.argmin(costs))
    theta_best = float(thetas[i_best])
    q_best = joints_arr[i_best].copy()
    T_EB_best = solve_T_EB(theta_best, T_CO_des, T_OF, T_EC, tt)
    pose6d_best = pose_from_T(T_EB_best)

    if verbose:
        print(f"  ✓ θ* = {np.degrees(theta_best):+.2f}°  "
              f"Δθ = {np.degrees(theta_best - theta_current):+.2f}°  "
              f"cost = {costs[i_best]:.4f}")
        print("─" * 72)

    return {
        "theta": theta_best,
        "joints": q_best,
        "cost": float(costs[i_best]),
        "T_EB": T_EB_best,
        "pose6d": pose6d_best,
        "n_feasible": n_feasible,
        "n_samples": N,
        "n_collision": n_collision,
        "all_thetas": thetas,
        "all_costs": costs,
        "all_feasible": feasible,
    }
