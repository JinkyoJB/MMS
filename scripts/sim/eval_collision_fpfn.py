"""eval_collision_fpfn.py — 충돌 판정 방법별 오탐/미탐율 비교 (논문 4.3).

같은 스캔 후보 자세 집합에 대해
  구(old) : 캡슐 근사 자가충돌 + 턴테이블 캡슐 (환경 미등록)   ← 2026-08 이전 파이프라인
  신(new) : 표면 점군 자가충돌 + 셀 전체 거리장 + 턴테이블 캡슐
을 각각 적용하고, **메시(표면 점군) 거리**를 기준으로

  오탐(FP) : old 가 기각했으나 메시 여유가 충분한 자세  → 자유도 손실
  미탐(FN) : old 가 수용했으나 셀 구조물과의 메시 거리가 여유 미만  → 장비 위험

을 센다. 순수 numpy — Isaac 불필요. 특이점 판정은 끄고(min_sigma=0) 충돌만 비교한다.

사용:  python scripts/sim/eval_collision_fpfn.py
"""
from __future__ import annotations
import os, sys, json
import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))
from utils.robot import xarm7_kinematics as kin          # noqa: E402
from utils.robot import view_pose as vp                  # noqa: E402
from utils.collision.robot_collision import (            # noqa: E402
    CollisionWorld, pose_collision, DEFAULT_LINK_RADII)
from utils.collision.collision_model import CollisionModel   # noqa: E402

DATA = os.path.join(os.path.dirname(__file__), "log", "testset_points")
ELS = (30., 40., 50., 60., 70., 80.)
AZS = (0., 30., -30.)
PSIS = (0., -45., 45., -90., 90., 180.)
TZ_OFF = (0.05, 0.10)
STANDOFF = 0.25
SEED = np.radians([-7.65, -75.61, -8.95, 78.64, 2.81, 126.71, -69.63])


def main():
    sc = json.load(open(os.path.join(DATA, "scene.json")))
    T_WB = np.array(sc["T_WB"], float); T_EC = np.array(sc["T_EC"], float)
    ax_w = np.array([sc["axis_xy"][0], sc["axis_xy"][1], sc["disc_top_z"]], float)
    T_BW = np.linalg.inv(T_WB)
    ax_b = (T_BW @ np.append(ax_w, 1.0))[:3]
    dir_b = T_BW[:3, :3] @ np.array([0., 0., 1.])

    # 구 파이프라인 재현: isaac_scan_session._build_world + _pose_collision
    world = CollisionWorld.from_turntable(
        surface_point=ax_b, axis_dir=dir_b,
        disc_radius=0.15, body_height=0.20, margin=0.01)
    cm = CollisionModel(min_sigma=0.0, log=lambda *a: None)

    # 후보 생성 (IK 수렴만 요구 — 충돌 판정은 이후 비교 대상)
    cands = []
    for tz in TZ_OFF:
        tgt = ax_w + np.array([0., 0., tz])
        for el in ELS:
            for az in AZS:
                for psi in PSIS:
                    q, _, _ = vp.solve_view_q(
                        kin, tgt, el, az, STANDOFF, SEED, T_EC,
                        T_WB=T_WB, convention=vp.CAM_USD,
                        rolls_deg=(psi,), n_seed_alt=7, min_sigma=0.0)
                    if q is not None:
                        cands.append((el, az, psi, tz, q))
    N = len(cands)
    print(f"IK 수렴 후보: {N} / {len(TZ_OFF)*len(ELS)*len(AZS)*len(PSIS)}")

    fp, fn, rows = [], [], []
    n_old_rej = n_new_rej = 0
    for el, az, psi, tz, q in cands:
        old_col, old_why = pose_collision(world, q, T_EC=T_EC,
                                          link_radii=DEFAULT_LINK_RADII)
        s_min, e_min, who = cm.clearance(q)
        scene_col, _ = pose_collision(world, q, T_EC=T_EC,
                                      link_radii=DEFAULT_LINK_RADII, self_scale=0.0)
        new_col = scene_col or (s_min < cm.self_margin) or (e_min < cm.env_margin)
        n_old_rej += old_col; n_new_rej += new_col
        if old_col and not new_col:
            fp.append((el, az, psi, s_min, e_min, old_why))
        if (not old_col) and (e_min < cm.env_margin):
            fn.append((el, az, psi, s_min, e_min))
        rows.append((el, az, psi, tz, old_col, new_col, s_min, e_min))

    print(f"\n구(캡슐) 기각 {n_old_rej}/{N} ({100*n_old_rej/N:.0f}%)   "
          f"신(점군+거리장) 기각 {n_new_rej}/{N} ({100*n_new_rej/N:.0f}%)")
    print(f"\n오탐(FP): 구가 기각했으나 메시 기준 안전  {len(fp)}/{N} "
          f"({100*len(fp)/N:.1f}%)")
    if fp:
        d = np.array([f[3] for f in fp]) * 1000
        print(f"   그 자세들의 실제 자가 여유: 중앙 {np.median(d):.0f} mm, "
              f"최소 {d.min():.0f} mm  (자가 margin 20 mm)")
        why = {}
        for f in fp: why[f[5]] = why.get(f[5], 0) + 1
        top = sorted(why.items(), key=lambda kv: -kv[1])[:3]
        print(f"   기각 사유 상위: " + ", ".join(f"{k}×{v}" for k, v in top))
    print(f"\n미탐(FN): 구가 수용했으나 셀 구조물 여유 미달  {len(fn)}/{N} "
          f"({100*len(fn)/N:.1f}%)")
    if fn:
        d = np.array([f[4] for f in fn]) * 1000
        print(f"   그 자세들의 실제 환경 여유: 중앙 {np.median(d):.0f} mm, "
              f"최소 {d.min():.0f} mm  (환경 margin 25 mm)")

    # el 대역별 (오탐이 낮은 el 을 막았는지)
    print("\nel별  후보  구기각  신기각  FP  FN")
    for el in ELS:
        r = [x for x in rows if x[0] == el]
        if not r: continue
        o = sum(x[4] for x in r); n_ = sum(x[5] for x in r)
        f1 = sum(1 for x in fp if x[0] == el); f2 = sum(1 for x in fn if x[0] == el)
        print(f"  {el:>4.0f} {len(r):>5} {o:>6} {n_:>6} {f1:>4} {f2:>4}")


if __name__ == "__main__":
    main()
