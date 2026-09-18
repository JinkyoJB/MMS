#!/usr/bin/env python
"""validate_real_cell.py — **실물 셀 기하로** 계획 로직을 점검한다 (장비 불요).

왜 필요한가
-----------
"sim 에서 개발·검증하고 real 에 그대로 올린다" 는 전략은 **자세 선정에 대해서는
성립하지 않는다.** 두 셀의 기하가 다르기 때문이다 (2026-09-17 실측):

    턴테이블 축 (로봇 base 프레임)      수평거리    전체거리
      sim  (v3_scene.usd)   [0, 0, 0.835]      0 mm     835 mm
      real (실측 T_B_F0)    [0.799, 0.005, 0.688]  799 mm    1055 mm   (도달한계 1090mm)

real 은 팔이 거의 다 펴진 자세라 야코비안이 나쁘다. 같은 코드·같은 후보 격자인데
az 를 0°→+30° 바꾸는 최대 관절이동이 sim 20°, real 48~103° 다. sim 에서 "짧게 짧게"
움직이던 것이 real 에서 크게 움직이는 이유가 이것이고, 플래너를 고쳐서 될 일이 아니다.

그래서 **실물에 가기 전에 real 상수로** 계획기를 돌려 본다. 스캐너·로봇·턴테이블
없이 캘리브 yaml + 충돌 캐시만으로 돈다.

    env -u PYTHONPATH ~/miniconda3/envs/mms-env/bin/python \
        scripts/artec/validate_real_cell.py

검사 항목
---------
  1. 셀 기하 요약 (축 위치·도달여유·up 방향)
  2. home 자세가 충돌 게이트를 통과하는가 + 계획 격자로 **경로가 있는가**
  3. lookaround 계획 자세(el × az × standoff)의 도달·충돌 통과율
  4. nbv 축-고도각 NBV 가 몇 번 · 얼마나 움직이며 도는가
"""
from __future__ import annotations

import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))

from utils.transforms import load_transform, TurntableTransformConfig  # noqa: E402
from utils.robot import xarm7_kinematics as kin                        # noqa: E402
from utils.robot import view_pose as vp                                # noqa: E402
from utils.collision import collision_model as cmod                    # noqa: E402
from utils.control.theta_planner import DEFAULT_JOINT_WEIGHTS as JW     # noqa: E402
from utils.nbv import nbv_core as p2                                 # noqa: E402

TT_YAML = "config/calibration/turntable_frame.yaml"
EC_YAML = "config/sensor_frames.yaml"
# 실물 home (XArmInterface.HOME_JOINTS_DEG["artec"] 와 같은 값이어야 한다)
from utils.robot.xarm_interface import XArmInterface as _XI            # noqa: E402
HOME_DEG = _XI.HOME_JOINTS_DEG["artec"]

ROLLS = (0.0, -45.0, 45.0, 90.0, -90.0, 180.0)


class _Gap:
    """법선 고도각만 쓰는 가짜 gap (plan_nbv_elevation_pose 입력용)."""

    def __init__(self, n, L=0.02):
        self.p_O = np.zeros(3)
        self.n_O = np.asarray(n, float)
        self.L = L


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--standoff", type=float, default=0.225, help="NBV standoff m")
    ap.add_argument("--els", type=float, nargs="+", default=[30., 45., 55., 65.])
    ap.add_argument("--azis", type=float, nargs="+", default=[0., 30., -30.])
    ap.add_argument("--layout", default=None,
                    help="충돌 셀 레이아웃 별칭 (기본 = 활성본)")
    ap.add_argument("--iters", type=int, default=8, help="NBV 반복 모사 횟수")
    a = ap.parse_args()
    if a.layout:
        os.environ["MMS_COLLISION_LAYOUT"] = a.layout

    # ── 1. 셀 기하 ────────────────────────────────────────────────────
    tt = TurntableTransformConfig(load_transform(TT_YAML, "T_B_F0"))
    T_EC = load_transform(EC_YAML, "T_EC_artec")
    axis = tt.axis_point_B                      # ★ inv(T_BF0) — 열을 쓰면 안 된다
    d = tt.axis_dir_B
    up = -d if float(d[2]) > 0 else d           # _view_up_B() 와 같은 유도
    q_home = np.radians(HOME_DEG)

    print("═" * 72)
    print("1. 셀 기하 (로봇 base 프레임)")
    print(f"   턴테이블 축   {np.round(axis, 4)} m")
    print(f"     수평 {np.linalg.norm(axis[:2])*1000:7.1f} mm · "
          f"전체 {np.linalg.norm(axis)*1000:7.1f} mm · "
          f"도달한계 {kin.MAX_REACH_M*1000:.0f} mm "
          f"(여유 {(kin.MAX_REACH_M - np.linalg.norm(axis))*1000:+.0f} mm)")
    print(f"   축 방향       {np.round(d, 4)}   '위'(카메라 쪽) {np.round(up, 4)}")
    print(f"   home (deg)    {HOME_DEG}")

    cm = cmod.get_default(self_margin_m=0.020, env_margin_m=0.025)
    if cm is None:
        print("   ⚠ 충돌 캐시 없음 — 충돌 항목은 건너뛴다")
    else:
        # ★ **어느 셀로 검사했는지 찍는다.** 활성 레이아웃이 sim 것으로 바뀌어
        #   있으면 아래 숫자가 통째로 남의 셀 이야기가 된다 — 조용히 틀리면 안 된다.
        from utils.collision.layout import active_name
        print(f"   충돌 셀       {os.path.basename(cm.env_npz)} "
              f"(활성 별칭 '{active_name()}')")
        if "v3" in str(cm.env_npz):
            print("   ⚠ **sim 레이아웃으로 검사하고 있다.** 실물 셀은 v2 다 — "
                  "`MMS_COLLISION_LAYOUT` 를 비우거나 --layout v2_layout_real 로 줄 것")

    # ── 2. home ───────────────────────────────────────────────────────
    print("\n" + "═" * 72)
    print("2. home 자세")
    if cm is not None:
        ok, why = cm.is_pose_safe(q_home)
        sl, who = cm.slack(q_home)
        print(f"   충돌 게이트  {'통과' if ok else '✘ ' + why}  "
              f"여유 {sl*1000:+.1f} mm ({who})  sigma_min {kin.sigma_min(q_home):.4f}")
        if not ok:
            print("   ⚠ home 이 게이트를 못 넘으면 `is_path_safe` 의 start 검사에서 "
                  "**모든 이동이 거부**된다. 먼저 이것부터 고칠 것.")

    # ── 3. lookaround 계획 자세 도달성 ───────────────────────────────────
    print("\n" + "═" * 72)
    print("3. lookaround 계획 자세 — 도달(IK) · 충돌 · home 에서 경로")
    print(f"   {'el':>5} | " + "".join(f"{f'az{az:+.0f}':>10}" for az in a.azis))
    tot_ik = tot_ok = tot_path = n_cell = 0
    for el in a.els:
        cells = []
        for az in a.azis:
            q, _, eye = vp.solve_view_q(kin, axis, el, az, a.standoff, q_home, T_EC,
                                        T_WB=None, convention=vp.CAM_OPENCV,
                                        up=up, world_up=up, az_ref=-axis,
                                        rolls_deg=ROLLS, n_seed_alt=3)
            n_cell += 1
            if q is None:
                cells.append("IK✘"); continue
            tot_ik += 1
            if cm is None:
                cells.append("IK✓"); continue
            ok, _ = cm.is_pose_safe(q)
            if not ok:
                cells.append("충돌✘"); continue
            tot_ok += 1
            p_ok, _, _ = cm.is_path_safe(q_home, q)
            tot_path += int(p_ok)
            cells.append("✓" if p_ok else "경로✘")
        print(f"   {el:>5.0f} | " + "".join(f"{c:>10}" for c in cells))
    print(f"   합계 {n_cell}칸 — IK {tot_ik} · 충돌통과 {tot_ok} · home 에서 경로 {tot_path}")

    # ── 4. nbv 모사 ───────────────────────────────────────────
    print("\n" + "═" * 72)
    print(f"4. nbv 축-고도각 NBV {a.iters}회 모사 (가짜 gap · 충돌 게이트 적용)")
    gaps = [_Gap(np.array([0.77, 0., 0.64])), _Gap(np.array([0., 0.77, 0.64])),
            _Gap(np.array([-0.77, 0., 0.64]))]
    qc = q_home.copy()
    visited, total, steps = [], 0.0, []

    def pose_q(el, az):
        q, _, _ = vp.solve_view_q(kin, axis, el, az, a.standoff, qc, T_EC,
                                  T_WB=None, convention=vp.CAM_OPENCV,
                                  up=up, world_up=up, az_ref=-axis,
                                  rolls_deg=ROLLS, n_seed_alt=3)
        return q

    def swept(q0, q1):
        if cm is None:
            return True
        return bool(cm.is_path_safe(q0, q1)[0])

    for _ in range(a.iters):
        res = p2.plan_nbv_elevation_pose(
            gaps, qc, pose_q, swept, joint_weights=JW, el_floor_deg=30.0,
            view_azis_deg=tuple(a.azis), visited=visited,
            ensure_els=(55.,), up_sign=-1.0)
        if res is None:
            break
        q, el, az = res
        mv = float(np.abs(np.degrees(q - qc)).max())
        steps.append((el, az, mv)); total += mv
        visited.append((el, az)); qc = q
    if not steps:
        print("   ✘ 관측 자세를 하나도 못 찾았다 — 위 3번 표부터 확인할 것")
    else:
        print(f"   전회전 패스 {len(steps)}회 · 총 최대관절이동 {total:.1f}° · "
              f"방문 az {sorted({az for _, az, _ in steps})}")
        for i, (el, az, mv) in enumerate(steps, 1):
            print(f"     {i}. el={el:>4.0f}° az={az:+4.0f}°  최대관절이동 {mv:6.1f}°")
        print("   ※ 방위가 하나로 모여 있어야 정상이다 — az 는 커버리지를 안 바꾸므로"
              "\n     (회전은 턴테이블 담당) 방위를 옮기는 건 정보 0 인 이동이다.")
    print("═" * 72)


if __name__ == "__main__":
    main()
