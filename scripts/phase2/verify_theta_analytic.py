"""Phase 2 Step 1 검증 — plan_min_motion_theta_analytic (해석 IK + 충돌검사).

하드웨어 무관. scipy/numpy 만 필요 (open3d 불요).
실행:  python scripts/phase2/verify_theta_analytic.py
라운드트립 일관성(solve_T_EB↔ik↔fk) + min-motion θ 선택 + 충돌필터 동작 검증.
"""
import os
import sys
import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))

from utils.transforms import TurntableTransformConfig, solve_T_EB
from utils.control.theta_planner import plan_min_motion_theta_analytic
from utils.robot import xarm7_kinematics as kin


def se3(rpy, t_m):
    T = np.eye(4)
    T[:3, :3] = kin.euler_xyz_to_R(np.array(rpy))
    T[:3, 3] = t_m
    return T


def fk_T_m(q):
    T = kin.fk_T(q).copy()
    T[:3, 3] /= 1000.0
    return T


def main():
    T_BF0 = se3([0, 0, 0.3], [0.45, 0.0, 0.10])
    tt = TurntableTransformConfig(T_BF0)
    T_OF = se3([0.05, -0.03, 0.2], [0.02, 0.01, 0.05])
    T_EC = se3([0.0, 0.0, 0.0], [0.0, 0.0, 0.06])

    q0 = np.radians([0.0, -25.0, 0.0, 35.0, 0.0, 60.0, 0.0])
    theta0 = np.radians(12.0)
    T_EB0 = fk_T_m(q0)
    T_CO_des = (np.linalg.inv(T_OF) @ np.linalg.inv(tt.T_FB(theta0))
                @ T_EB0 @ np.linalg.inv(T_EC))

    assert np.allclose(solve_T_EB(theta0, T_CO_des, T_OF, T_EC, tt), T_EB0, atol=1e-9)
    print("[ok] solve_T_EB 역산 일치")

    samples = np.union1d(np.linspace(-np.pi, np.pi, 72, endpoint=False), [theta0])
    res = plan_min_motion_theta_analytic(
        T_CO_des, T_OF, T_EC, tt, ik_fn=kin.ik,
        theta_current=theta0, q_current=q0, collision_fn=None,
        theta_samples=samples, w_tt=0.0, verbose=True)
    assert res["theta"] is not None
    assert abs(res["theta"] - theta0) < np.radians(2.0), "θ* 가 θ0 근처가 아님"
    assert np.max(np.abs(np.degrees(res["joints"] - q0))) < 2.0, "min-motion 실패"
    T_star = solve_T_EB(res["theta"], T_CO_des, T_OF, T_EC, tt)
    err_mm = np.linalg.norm((fk_T_m(res["joints"])[:3, 3] - T_star[:3, 3]) * 1000)
    assert err_mm < 1.0
    print(f"[ok] 충돌無: θ*={np.degrees(res['theta']):+.2f}° fk오차={err_mm:.3f}mm")

    res2 = plan_min_motion_theta_analytic(
        T_CO_des, T_OF, T_EC, tt, ik_fn=kin.ik,
        theta_current=theta0, q_current=q0, collision_fn=lambda q: True,
        theta_samples=samples, verbose=False)
    assert res2["theta"] is None and res2["n_feasible"] == 0
    print(f"[ok] 전부 충돌 → feasible 0 (reject={res2['n_collision']})")
    print("\n=== Step 1 통과 ===")


if __name__ == "__main__":
    main()
