"""
analyze_base_height.py — 로봇 베이스 높이(ΔH) vs NBV 후보 feasibility 스윕.

목적: Phase2 고도각(윗면 보강) 자세에서 링크꼬임/스캐너-손목 근접이 문제 →
베이스를 얼마나 올리면 해결되는지 정량화해 하드웨어팀에 ΔH 전달.
(real 도 동일하게 올릴 예정 — sim 선행 검증, 2026-07-08)

방법 (오프라인, Isaac 불필요):
  ΔH ∈ {0..0.25m}: T_WB z+=ΔH (턴테이블/축은 world 고정)
  후보 = el 20..70° × az 8종, look=축, standoff=0.225+r_obj (nominal r=6cm)
  각 후보: 해석 IK → 수정된 충돌검사(스캐너↔link1~5 포함) →
    feasible 수 / 최대 feasible el / 스캐너↔손목(4·5) raw 여유(전반경, 스케일무)

실행: (env_isaacsim) python scripts/sim/analyze_base_height.py
"""
import os
import sys
import json
import math

import numpy as np

_REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, _REPO)

from utils.robot import xarm7_kinematics as kin                    # noqa: E402
from utils.collision.robot_collision import (                      # noqa: E402
    CollisionWorld, pose_collision, capsules_from_joints,
    DEFAULT_LINK_RADII, seg_seg_distance)
from utils.nbv.phase1_viewpoint import look_at_R                   # noqa: E402

DATA = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                    "log", "testset_points")
HOME_Q = np.radians([38.92, -48.70, -65.29, 21.22, 21.46, 72.70, -96.58])
AZS = [0.0, 30.0, -30.0, 60.0, -60.0, 90.0, -90.0, 180.0]
ELS = list(range(20, 75, 5))
OBJ_R, OBJ_H = 0.06, 0.15                     # nominal 물체 (testset 중간값)
NBV_STANDOFF = 0.225 + OBJ_R                  # sim NBV_DISTANCE_M + r
DHS = [0.0, 0.05, 0.10, 0.15, 0.20, 0.25]


def main():
    sc = json.load(open(os.path.join(DATA, "scene.json")))
    T_WB0 = np.array(sc["T_WB"])
    T_EC = np.array(sc["T_EC"])
    axis_w = np.array([sc["axis_xy"][0], sc["axis_xy"][1], sc["disc_top_z"]])
    tz = axis_w[2] + OBJ_H * 0.6              # look 타깃(물체 상부)

    print(f"{'ΔH(cm)':>7} {'feasible':>9} {'max el':>7} {'el≥60 수':>8} "
          f"{'scanner-손목 여유(mm)':>22} {'저각(20-35°) 수':>14}")
    print("─" * 78)
    for dh in DHS:
        T_WB = T_WB0.copy()
        T_WB[2, 3] += dh
        T_WB_inv = np.linalg.inv(T_WB)
        # 충돌 world (base 프레임): 턴테이블 축·표면을 새 base 로 변환
        sp_b = (T_WB_inv @ np.append(axis_w, 1.0))[:3]
        ax_b = T_WB_inv[:3, :3] @ np.array([0.0, 0.0, 1.0])
        world = CollisionWorld.from_turntable(
            sp_b, ax_b, disc_radius=0.15, body_height=0.20,
            object_radius=OBJ_R, object_height=OBJ_H, margin=0.01)

        n_ok, max_el, n_hi, n_lo = 0, None, 0, 0
        min_wrist = np.inf
        for el in ELS:
            for az in AZS:
                e, a = math.radians(el), math.radians(az)
                target = np.array([axis_w[0], axis_w[1], tz])
                eye = target + NBV_STANDOFF * np.array(
                    [math.cos(e) * math.cos(a), math.cos(e) * math.sin(a),
                     math.sin(e)])
                T_WC = np.eye(4)
                T_WC[:3, :3] = look_at_R(eye, target)
                T_WC[:3, 3] = eye
                T_EB = T_WB_inv @ T_WC @ T_EC
                pose6d = np.concatenate([T_EB[:3, 3] * 1000.0,
                                         kin.R_to_euler_xyz(T_EB[:3, :3])])
                q, ok = kin.ik(pose6d, seed=HOME_Q)
                if not ok:
                    continue
                col, _ = pose_collision(world, q, T_EC=T_EC)
                if col:
                    continue
                n_ok += 1
                max_el = el if (max_el is None or el > max_el) else max_el
                if el >= 60:
                    n_hi += 1
                if el <= 35:
                    n_lo += 1
                caps = capsules_from_joints(q, DEFAULT_LINK_RADII, T_EC=T_EC)
                names = [n for n, _ in caps]
                cs = caps[names.index("scanner")][1]
                for li in ("link4", "link5"):
                    c = caps[names.index(li)][1]
                    d = seg_seg_distance(cs.p0, cs.p1, c.p0, c.p1)
                    min_wrist = min(min_wrist, d - (cs.r + c.r))
        mw = f"{min_wrist*1000:+.0f}" if np.isfinite(min_wrist) else "-"
        print(f"{dh*100:7.0f} {n_ok:9d} {str(max_el):>7} {n_hi:8d} "
              f"{mw:>22} {n_lo:14d}")


if __name__ == "__main__":
    main()
