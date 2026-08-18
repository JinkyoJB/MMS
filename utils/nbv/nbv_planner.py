"""nbv_planner.py — Phase 2 관측자세 선택 래퍼. **sim·real 공용**.

왜 공용인가
----------
`phase2_nbv.plan_nbv_elevation_pose` 는 이미 공용이었지만, **그것을 감싸는 판단들**이
sim 쪽에만 쌓였다. 2026-08 시점의 실측 격차:

    항목                        sim   real
    gap 부호 분류(위/아래/측면)  O     X
    아랫면 gap 제외(flip 몫)     O     X
    visited(같은 자세 반복 방지) O     X   ← real 은 무한 반복 버그가 그대로
    gap 방향 roll 정렬           O     X
    실패 원인 진단(IK vs 충돌)   O     X

전부 **순수 기하·기록 로직**이라 sim 의존성이 없다. 여기로 모아 양쪽이 같은 판단을
쓰게 한다. 백엔드가 주입하는 것은 (1) 충돌 판정, (2) 좌표계·hand-eye 뿐이다.

핵심 판단 세 가지 (근거: docs/hw_layout.md, docs/4_collision.md)
--------------------------------------------------------------
1. **아랫면 gap 은 NBV 대상이 아니다** — 턴테이블에 가려 어떤 관측 elevation 으로도
   못 본다(Phase 3 flip 의 몫). 넘기면 el_need 중앙값만 끌어내려 **측면 보강까지
   방해**한다(실측: 중앙값이 -3° 까지 내려갔다).
2. **visited 를 반드시 넘긴다** — az 를 '관절이동 최소'로 고르므로, 직전 자세의
   이동비용이 0 이라 같은 자세를 무한 반복한다(실측: el=65 az=-30 을 4회 연속,
   gap 18→18→20→19 로 안 줄었다).
3. **실패는 IK/충돌로 갈라 기록한다** — "feasible 없음"만 찍으면 도달한계인지 충돌
   오탐인지 알 수 없다. 실측에서 이 구분이 `IK실패 0, swept충돌 32` 를 드러내
   캡슐 오탐을 잡아냈다.
"""
from __future__ import annotations

import numpy as np

from utils.nbv import phase2_nbv as p2

DOWN_NZ = -0.6      # 바깥법선 z 가 이보다 작으면 '아랫면'
UP_NZ = 0.6


def classify_gaps(gaps):
    """(n_up, n_down, n_side, nz list). 위/아래를 **부호로** 가른다.

    `abs(n_z)>0.6` 로 뭉뚱그리면 아래를 향한 gap 까지 '윗면'으로 세어, 바닥면
    미스캔이 윗면 부족처럼 보인다(그러면 더 높은 el 을 시도하게 되어 역효과).
    """
    nz = []
    for c in gaps:
        n = np.asarray(c.n_O, float)
        nz.append(float((n / (np.linalg.norm(n) + 1e-12))[2]))
    n_up = sum(1 for z in nz if z > UP_NZ)
    n_dn = sum(1 for z in nz if z < DOWN_NZ)
    return n_up, n_dn, len(gaps) - n_up - n_dn, nz


class NbvPlanner:
    """Phase 2 자세 선택기. **visited 를 세션 동안 유지**한다.

    solve_pose_fn(el, az, rolls) -> (q | None, roll_used | None)
    swept_free_fn(q0, q1)        -> (ok: bool, why: str)
    roll_order_fn(el, az, gaps)  -> rolls 순서 (None 이면 기본 순서)
    """

    def __init__(self, *, joint_weights, el_floor_deg, view_azis_deg=None,
                 log=print, tag=""):
        self.joint_weights = joint_weights
        self.el_floor_deg = float(el_floor_deg)
        self.view_azis_deg = view_azis_deg
        self.log = log
        self.tag = f"{tag} " if tag else ""
        self.visited = []                     # 이미 전회전 스캔한 (el, az)

    def plan(self, gaps, q_cur, solve_pose_fn, swept_free_fn, *,
             roll_order_fn=None):
        """(q, el, az, roll) 또는 None."""
        if not gaps:
            return None
        n_up, n_dn, n_side, nz = classify_gaps(gaps)
        needs = p2.gap_normal_elevations_deg(gaps)
        self.log(f"{self.tag}gap유형: 윗면={n_up} 아랫면={n_dn} 측면/뒷면={n_side}  "
                 f"필요 el(중앙값)={float(np.median(needs)):.0f}°")

        usable = [c for c, z in zip(gaps, nz) if z >= DOWN_NZ]
        if n_dn:
            self.log(f"{self.tag}  ↳ 아랫면 {n_dn}개는 NBV 대상 제외 (Phase 3 flip 필요)")
        if not usable:
            self.log(f"{self.tag}NBV: 남은 gap 이 전부 아랫면 — flip 으로만 해결 가능")
            return None

        stat = {"ik": 0, "swept": 0, "ok": 0, "why": {}}
        rolls_used = {}

        def pose_q(el, az):
            rolls = roll_order_fn(el, az, usable) if roll_order_fn else None
            q, roll = solve_pose_fn(el, az, rolls)
            if q is None:
                stat["ik"] += 1
            else:
                rolls_used[(round(float(el), 1), round(float(az), 1))] = roll
            return q

        def swept(q0, q1):
            ok, why = swept_free_fn(q0, q1)
            if ok:
                stat["ok"] += 1
            else:
                stat["swept"] += 1
                k = str(why)
                stat["why"][k] = stat["why"].get(k, 0) + 1
            return ok

        kw = dict(joint_weights=self.joint_weights,
                  el_floor_deg=self.el_floor_deg, visited=self.visited)
        if self.view_azis_deg is not None:
            kw["view_azis_deg"] = tuple(self.view_azis_deg)
        res = p2.plan_nbv_elevation_pose(usable, q_cur, pose_q, swept, **kw)

        if res is None:
            self.log(f"{self.tag}NBV: feasible 관측자세 없음 — "
                     f"IK실패 {stat['ik']}, swept충돌 {stat['swept']}, 통과 {stat['ok']}")
            top = sorted(stat["why"].items(), key=lambda kv: -kv[1])[:3]
            if top:
                self.log(f"{self.tag}  충돌사유: "
                         + ", ".join(f"{k}×{v}" for k, v in top))
            return None

        q, el, az = res
        self.visited.append((el, az))
        roll = rolls_used.get((round(float(el), 1), round(float(az), 1)))
        self.log(f"{self.tag}NBV 관측자세 el={el:.0f}° az={az:.0f}° "
                 f"roll={'-' if roll is None else f'{roll:.0f}°'} — 전회전 스캔 "
                 f"(누적 {len(self.visited)}자세)")
        return q, el, az, roll
