"""nbv_planner.py — nbv 관측자세 선택 래퍼. **sim·real 공용**.

왜 공용인가
----------
`nbv_core.plan_nbv_elevation_pose` 는 이미 공용이었지만, **그것을 감싸는 판단들**이
sim 쪽에만 쌓였다. 2026-08 시점의 실측 격차:

    항목                        sim   real
    gap 부호 분류(위/아래/측면)  O     X
    아랫면 gap 제외(flip 몫)     O     X
    visited(같은 자세 반복 방지) O     X   ← real 은 무한 반복 버그가 그대로
    gap 방향 roll 정렬           O     X
    실패 원인 진단(IK vs 충돌)   O     X

전부 **순수 기하·기록 로직**이라 sim 의존성이 없다. 여기로 모아 양쪽이 같은 판단을
쓰게 한다. 백엔드가 주입하는 것은 (1) 충돌 판정, (2) 좌표계·hand-eye 뿐이다.

핵심 판단 세 가지 (근거: docs/hw_layout.md, docs/collision.md)
--------------------------------------------------------------
1. **아랫면 gap 은 NBV 대상이 아니다** — 턴테이블에 가려 어떤 관측 elevation 으로도
   못 본다(flip flip 의 몫). 넘기면 el_need 중앙값만 끌어내려 **측면 보강까지
   방해**한다(실측: 중앙값이 -3° 까지 내려갔다).
2. **visited 를 반드시 넘긴다** — az 를 '관절이동 최소'로 고르므로, 직전 자세의
   이동비용이 0 이라 같은 자세를 무한 반복한다(실측: el=65 az=-30 을 4회 연속,
   gap 18→18→20→19 로 안 줄었다).
3. **실패는 IK/충돌로 갈라 기록한다** — "feasible 없음"만 찍으면 도달한계인지 충돌
   오탐인지 알 수 없다. 실측에서 이 구분이 `IK실패 0, swept충돌 32` 를 드러내
   캡슐 오탐을 잡아냈다.
"""
from __future__ import annotations

import math
import time as _time

import numpy as np

from utils.nbv import nbv_core as p2

DOWN_NZ = -0.6      # 바깥법선 z 가 이보다 작으면 '아랫면'
UP_NZ = 0.6
# 바깥법선이 회전축 쪽(안쪽)을 향하면 **오목·내부면**. 컵 내벽이 대표적이다.
INWARD_DOT = -0.2


def classify_gaps(gaps, up_sign: float = +1.0):
    """(n_up, n_down, n_side, nz list). 위/아래를 **부호로** 가른다.

    `abs(n_z)>0.6` 로 뭉뚱그리면 아래를 향한 gap 까지 '윗면'으로 세어, 바닥면
    미스캔이 윗면 부족처럼 보인다(그러면 더 높은 el 을 시도하게 되어 역효과).

    `up_sign` — 작업 프레임에서 어느 z 방향이 '위'인가 (+1 = +Z 가 위).
      real 의 base 는 천장 마운트라 +Z 가 **아래**(−1). 부호를 안 맞추면 윗면과
      아랫면이 통째로 뒤바뀐다 — 2026-09-16 실물: 뚜껑 gap 3개가 '아랫면'으로
      분류돼 NBV 대상에서 제외되고, 정작 볼 수 없는 바닥면 gap 을 겨냥했다.
    반환하는 nz 는 **up_sign 적용 후**(+ = 위) — 호출측 usable 필터가 그대로 맞다.
    """
    s = float(np.sign(up_sign)) or 1.0
    nz = []
    for c in gaps:
        n = np.asarray(c.n_O, float)
        nz.append(s * float((n / (np.linalg.norm(n) + 1e-12))[2]))
    n_up = sum(1 for z in nz if z > UP_NZ)
    n_dn = sum(1 for z in nz if z < DOWN_NZ)
    return n_up, n_dn, len(gaps) - n_up - n_dn, nz


def split_inward(gaps, axis_xy):
    """(inward, outward) — 바깥법선이 **축 쪽**을 향하는 gap 을 가른다.

    왜 갈라야 하나 — 축-고도각 방식(`plan_nbv_elevation_pose`)은 카메라를 늘
    **물체 바깥 구면**에 놓고 축을 겨눈다. 그래서 컵 내벽처럼 법선이 안쪽을 향하는 면은
    (1) 정면으로 볼 수 없고, (2) standoff 가 외곽 기준이라 내부가 작동거리(0.2~0.3m)
    **far clip 밖**으로 밀려 아예 캡처되지 않는다.
    이런 면은 표면점 기준으로 카메라를 놓는 **프론티어 방식**이어야 한다.
    """
    inw, outw = [], []
    ax = np.asarray(axis_xy, float)
    for c in gaps:
        p = np.asarray(c.p_O, float)
        n = np.asarray(c.n_O, float)
        n = n / (np.linalg.norm(n) + 1e-12)
        radial = p[:2] - ax                      # 축 → 표면점 (수평)
        rn = float(np.linalg.norm(radial))
        if rn < 1e-6:
            outw.append(c)
            continue
        (inw if float(n[:2] @ (radial / rn)) < INWARD_DOT else outw).append(c)
    return inw, outw


class NbvPlanner:
    """nbv 자세 선택기. **visited 를 세션 동안 유지**한다.

    solve_pose_fn(el, az, rolls) -> (q | None, roll_used | None)
    swept_free_fn(q0, q1)        -> (ok: bool, why: str)
    roll_order_fn(el, az, gaps)  -> rolls 순서 (None 이면 기본 순서)
    """

    def __init__(self, *, joint_weights, el_floor_deg, view_azis_deg=None,
                 ensure_els=(), log=print, tag="", up_sign: float = +1.0):
        self.joint_weights = joint_weights
        self.el_floor_deg = float(el_floor_deg)
        self.view_azis_deg = view_azis_deg
        # 작업 프레임의 '위' 부호 (+1 = +Z 가 위). real(천장 마운트 base)은 −1 —
        # gap 의 윗면/아랫면 분류·필요 elevation·frontier 기울임 방향이 전부 따른다.
        self.up_sign = float(np.sign(up_sign)) or 1.0
        self.log = log
        self.tag = f"{tag} " if tag else ""
        self.visited = []                     # 이미 전회전 스캔한 (el, az)
        self.visited_frontier = []            # 이미 겨냥한 내부 gap 대표점
        # ── 비생산 gap 회계 ──────────────────────────────────────────────
        # 패치가 새 관측을 못 얻으면(신규복셀↓) 그 겨냥점 주변은 **도달 불가이거나
        # 이미 촘촘한** 영역이다. dry 로 표시해 그 근방(dry_sep_m)의 후보를 건너뛴다.
        # 왜 gap 단위인가 — NBV 는 패치마다 다른 gap 을 겨냥하므로 "직전 패치가
        # 조용했다 = 끝났다" 는 전역 판정은 틀린다(실측 2026-08-19 hand_drill:
        # 4패치째 신규복셀 1.2% 로 조용 → 6패치째 +3.3%p 이득). 조용함은 그 gap
        # 의 속성이지 스캔 전체의 속성이 아니다.
        self.dry_frontier = []                # 겨냥했지만 신규 관측이 없던 지점
        self.dry_sep_m = 0.04                 # dry 점 주변 후보 제외 반경
        self._last_target = None              # 직전 채택 겨냥점 (report_patch 용)
        # gap 과 무관하게 **최소 한 번** 시도할 고도각. 오목 물체 내부는 미관측이라
        # gap 으로 잡히지 않아 el_need 가 올라갈 근거가 없다(닭·달걀) → 사전지식으로 보완.
        self.ensure_els = tuple(float(e) for e in (ensure_els or ()))

    def plan_frontier(self, gaps, q_cur, solve_lookat_fn, swept_free_fn, *,
                      standoff_m=0.25, min_sep_m=0.02,
                      tilt_degs=(0.0, 30.0, 45.0, 60.0, 75.0),
                      up=None, axis_xy=None, az_pref_deg=(0.0, 30.0, -30.0)):
        """gap 을 **직접 겨냥**한다. (q, cand, roll, theta) 또는 None.

        축-고도각 방식은 카메라가 늘 턴테이블 축을 봐서 **gap 위치를 통째로 버린다**
        (gap 정보가 '법선 고도각 중앙값' 하나로 압축). 손잡이·내벽 같은 국소 결손을
        원리적으로 겨냥할 수 없었다. 여기서는 표면점 p 를 정면으로 본다.

        **턴테이블이 gap 을 로봇 앞으로 가져온다** — gap 의 방위각을 로봇이 편한
        방위(az_pref)로 만드는 θ 를 구하고, 그 θ 에서의 위치를 겨냥한다. 로봇은
        고도각·거리만 담당하므로 팔이 크게 움직이지 않는다(충돌 위험 ↓).

        법선 정면이 막히면(작동거리가 물체보다 큰 오목면 등) 개구부 쪽으로 기울인다:
            d(θ_t) = normalize(n̂·cos θ_t + ẑ·sin θ_t)
        """
        # `up` 미지정이면 planner 의 up_sign 을 따른다 — tilt 는 gap 법선에서
        # '위' 쪽으로 기울여 내려다보게 만드는 것이라 부호가 뒤집히면 카메라가
        # 아래에서 올려다보는 자세를 시도한다 (real 은 대부분 도달 불가·충돌).
        if up is None:
            up = (0.0, 0.0, self.up_sign)
        u = np.asarray(up, float); u = u / (np.linalg.norm(u) + 1e-12)
        ax = None if axis_xy is None else np.asarray(axis_xy, float)
        # ★ 아랫면 gap 은 여기서도 제외한다 — `plan()`(축-고도각)에는 이 필터가
        #   있었는데 frontier 경로에는 없었다. 아래를 향한 면은 밑에서 올려다봐야
        #   해서 후보 조합(방위×기울임)이 **전부** IK 불능인데, 도달상한 사전필터는
        #   위치만 보므로 못 거른다 → 조합마다 수치 IK 를 끝까지 소진한다.
        #   실측 2026-09-16: 후보 25개 중 아랫면 14개, 후보당 ~20s = 5분 낭비.
        #   아랫면은 flip flip 의 몫이다.
        _, _, _, nz = classify_gaps(gaps, self.up_sign)
        n_dn = sum(1 for z in nz if z < DOWN_NZ)
        if n_dn:
            self.log(f"{self.tag}  ↳ 아랫면 {n_dn}개 frontier 제외 (flip 필요)")
        cands = [c for c, z in zip(gaps, nz) if z >= DOWN_NZ]
        cands = sorted(cands, key=lambda c: -float(c.L))
        if not cands:
            return None
        self.log(f"{self.tag}gap 겨냥 — 후보 {len(cands)}개 "
                 f"(최대 L={float(cands[0].L)*1000:.0f}mm)")
        tried = {"near": 0, "dry": 0, "ik": 0, "swept": 0}
        # ★ 진행 로그. 이 루프는 후보×방위×기울임 조합마다 IK+swept 를 돌아서
        #   조건이 나쁘면 수 분간 출력이 없다 — 2026-09-16 실물에서 gap 좌표가
        #   틀려 전 조합이 도달 불가가 되자 로그 한 줄 없이 14분을 돌았다.
        #   멈춘 것인지 도는 것인지 구분할 수 있어야 한다.
        _t0 = _time.perf_counter()
        _t_last = _t0
        for _i, c in enumerate(cands):
            if _time.perf_counter() - _t_last > 5.0:
                _t_last = _time.perf_counter()
                self.log(f"{self.tag}  gap 겨냥 진행 {_i}/{len(cands)} "
                         f"({_t_last - _t0:.0f}s) — IK실패 {tried['ik']}, "
                         f"충돌 {tried['swept']}")
            p0 = np.asarray(c.p_O, float)
            if any(np.linalg.norm(p0 - v) < min_sep_m for v in self.visited_frontier):
                tried["near"] += 1
                continue
            if any(np.linalg.norm(p0 - v) < self.dry_sep_m for v in self.dry_frontier):
                tried["dry"] += 1
                continue
            n0 = np.asarray(c.n_O, float); n0 = n0 / (np.linalg.norm(n0) + 1e-12)
            # 턴테이블 각 후보: gap 방위를 로봇 편한 방위로 보내는 θ
            thetas = [0.0]
            if ax is not None:
                g = math.atan2(p0[1] - ax[1], p0[0] - ax[0])
                thetas = [math.radians(a) - g for a in az_pref_deg]
            for th in thetas:
                ct, st = math.cos(th), math.sin(th)
                if ax is not None:
                    d0 = p0[:2] - ax
                    p = np.array([ax[0] + ct*d0[0] - st*d0[1],
                                  ax[1] + st*d0[0] + ct*d0[1], p0[2]])
                    n = np.array([ct*n0[0] - st*n0[1], st*n0[0] + ct*n0[1], n0[2]])
                else:
                    p, n = p0, n0
                for tilt in tilt_degs:
                    t = math.radians(float(tilt))
                    d = n * math.cos(t) + u * math.sin(t)
                    nd = float(np.linalg.norm(d))
                    if nd < 1e-9:
                        continue
                    q, roll = solve_lookat_fn(p + (d / nd) * float(standoff_m), p)
                    if q is None:
                        tried["ik"] += 1
                        continue
                    ok, _why = swept_free_fn(q_cur, q)
                    if not ok:
                        tried["swept"] += 1
                        continue
                    self.visited_frontier.append(p0)
                    self._last_target = p0.copy()
                    self.log(f"{self.tag}gap 겨냥 채택 — L={float(c.L)*1000:.0f}mm "
                             f"θ={math.degrees(th) % 360:.0f}° 기울임={tilt:.0f}° "
                             f"roll={'-' if roll is None else f'{roll:.0f}°'} "
                             f"(누적 {len(self.visited_frontier)}곳)")
                    return q, c, roll, float(th)
        self.log(f"{self.tag}gap 겨냥 실패 — 근접중복 {tried['near']}, "
                 f"dry {tried['dry']}, IK {tried['ik']}, 충돌 {tried['swept']}")
        return None

    def report_patch(self, productive: bool) -> None:
        """직전 frontier 패치의 결과 회계. 백엔드가 패치 후 신규 관측량으로 부른다.

        productive=False 면 겨냥점을 dry 목록에 넣어 근방 후보를 건너뛴다 —
        도달 불가하거나 이미 촘촘한 영역을 반복 겨냥하지 않게. **루프 종료가
        아니라 후보 제외**라서, 판단이 틀려도 그 영역 하나를 잃을 뿐 스캔이
        일찍 끝나지 않는다(전역 조기종료보다 실패 비용이 훨씬 싸다).

        frontier 가 아닌 패스(ensure el / 축-고도각) 뒤에는 no-op — _last_target
        은 plan_frontier 채택 때만 설정된다. sim 은 누적점군 신규복셀로, real 은
        _merge_into_master 의 n_added 비율로 productive 를 판정하면 된다.
        """
        p0, self._last_target = self._last_target, None
        if p0 is None or productive:
            return
        self.dry_frontier.append(np.asarray(p0, float))
        self.log(f"{self.tag}겨냥점 dry 처리 — 신규 관측 없음 "
                 f"(누적 {len(self.dry_frontier)}곳, 반경 {self.dry_sep_m*1000:.0f}mm 제외)")

    def plan(self, gaps, q_cur, solve_pose_fn, swept_free_fn, *,
             roll_order_fn=None):
        """(q, el, az, roll) 또는 None."""
        if not gaps:
            return None
        n_up, n_dn, n_side, nz = classify_gaps(gaps, self.up_sign)
        needs = p2.gap_normal_elevations_deg(gaps, up_sign=self.up_sign)
        self.log(f"{self.tag}gap유형: 윗면={n_up} 아랫면={n_dn} 측면/뒷면={n_side}  "
                 f"필요 el(중앙값)={float(np.median(needs)):.0f}°")

        usable = [c for c, z in zip(gaps, nz) if z >= DOWN_NZ]
        if n_dn:
            self.log(f"{self.tag}  ↳ 아랫면 {n_dn}개는 NBV 대상 제외 (flip flip 필요)")
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
                  el_floor_deg=self.el_floor_deg, visited=self.visited,
                  ensure_els=self.ensure_els, up_sign=self.up_sign)
        if self.view_azis_deg is not None:
            kw["view_azis_deg"] = tuple(self.view_azis_deg)
        todo = [e for e in self.ensure_els
                if not any(abs(float(v[0]) - float(e)) < 1e-6 for v in self.visited)]
        if todo:
            self.log(f"{self.tag}보장 고도각 우선 시도: {todo} (오목 내부 대비)")
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
