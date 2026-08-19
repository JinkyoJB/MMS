"""eval_azimuth_cost.py — 로봇이 방위(az)를 만들 때의 비용 실측 (논문 4.1).

주장: 방위 커버리지는 턴테이블 전회전이 제공하므로 로봇이 az 를 만들 이유가 없다.

프로토콜
  · 각 (el, az)에 대해 실제 파이프라인과 같은 수용 탐색(ψ 6후보 × 시드 8,
    σ_min ≥ 0.05, 충돌 여유)을 수행해 수용 자세를 얻는다.
  · az = 0 자세를 기준으로, 로봇만으로 해당 az 로 옮길 때의 관절 이동량
    Σ|Δq| (deg) 를 측정한다. 턴테이블은 같은 상대 관측을 관절 이동 0 으로 제공한다.
순수 numpy — Isaac 불필요.
"""
from __future__ import annotations
import os, sys, json
import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))
from utils.robot import xarm7_kinematics as kin          # noqa: E402
from utils.robot import view_pose as vp                  # noqa: E402
from utils.collision.collision_model import CollisionModel   # noqa: E402

DATA = os.path.join(os.path.dirname(__file__), "log", "testset_points")
AZS = (0., 30., -30., 60., -60., 90., -90., 180.)
ELS = (40., 55.)
STANDOFF = 0.313
HOME = np.radians([-7.65, -75.61, -8.95, 78.64, 2.81, 126.71, -69.63])


def accepted_pose(cm, tgt, el, az, T_WB, T_EC):
    """실제 파이프라인과 동일: ψ 후보를 순회하며 수용 판정 통과 첫 해."""
    for psi in vp.DEFAULT_ROLLS_DEG:
        q, _, _ = vp.solve_view_q(kin, tgt, el, az, STANDOFF, HOME, T_EC,
                                  T_WB=T_WB, convention=vp.CAM_USD,
                                  rolls_deg=(psi,), min_sigma=0.05)
        if q is None:
            continue
        ok, why = cm.is_pose_safe(q)
        if ok:
            return q, psi
    return None, None


def main():
    sc = json.load(open(os.path.join(DATA, "scene.json")))
    T_WB = np.array(sc["T_WB"], float); T_EC = np.array(sc["T_EC"], float)
    tgt = np.array([sc["axis_xy"][0], sc["axis_xy"][1],
                    sc["disc_top_z"] + 0.06], float)
    cm = CollisionModel(min_sigma=0.05, log=lambda *a: None)

    print(f"{'el':>4} {'az':>5} {'수용':>4} {'ψ':>5} {'Δq(az=0 기준)deg':>16}")
    n_rej = 0; travels = {}
    for el in ELS:
        q0, _ = accepted_pose(cm, tgt, el, 0., T_WB, T_EC)
        for az in AZS:
            q, psi = accepted_pose(cm, tgt, el, az, T_WB, T_EC)
            if q is None:
                print(f"{el:>4.0f} {az:>5.0f}    ✘"); n_rej += 1; continue
            d = (float(np.degrees(np.sum(np.abs(q - q0))))
                 if (q0 is not None and az != 0.) else 0.0)
            travels.setdefault(el, {})[az] = d
            print(f"{el:>4.0f} {az:>5.0f}    O {psi:>5.0f} {d:>16.0f}")

    N = len(ELS) * len(AZS)
    print(f"\n수용 불가: {n_rej}/{N}")
    for el in ELS:
        tv = travels.get(el, {})
        small = [v for a, v in tv.items() if a != 0. and abs(a) <= 30.]
        big = [v for a, v in tv.items() if abs(a) >= 60.]
        if small and big:
            print(f"el={el:.0f}: az ±30° 이동 평균 {np.mean(small):.0f}°  ↔  "
                  f"|az| ≥ 60° 이동 평균 {np.mean(big):.0f}° "
                  f"({np.mean(big)/np.mean(small):.1f}배)   [턴테이블은 0°]")


if __name__ == "__main__":
    main()
