"""joint_path_planner.py — 관절공간 **우회 경로** 계획 (RRT-Connect). sim·real 공용.

왜 필요한가
----------
지금까지의 충돌 대응은 **"검사 후 거부"** 뿐이었다 — 직선 보간이 막히면 그 자세를
포기한다. 그래서 Phase 2 가 "feasible 관측자세 없음"으로 끝나는 경우 중 일부는
'도달 불가'가 아니라 **'직선으로 못 갈 뿐'** 이었다. 돌아가면 되는 자세를 버린 것이다.

여기서는 막혔을 때 **우회 경로**를 찾는다. 순수 numpy — hpp-fcl·pinocchio 는
sim(env_isaacsim)에만 있고 실물(mms-env)에는 없어, 쓰면 sim/real 격차가 되살아난다.

알고리즘
-------
**RRT-Connect** (양방향 RRT). 시작·목표에서 트리를 하나씩 키우고 서로를 향해
greedy 하게 뻗어(connect) 만나면 종료. 단방향 RRT 보다 훨씬 빨리 붙는다.

  1. **직선 먼저** — 대부분의 이동은 직선이 안전하다. 되면 즉시 반환(계획 비용 0).
  2. 실패 시 RRT-Connect 로 우회 경로 탐색.
  3. **shortcut 평활화** — 무작위 두 웨이포인트를 직선으로 이어보고 되면 사이를 삭제.
     RRT 원경로는 지그재그라 그대로 쓰면 동작이 거칠다.

충돌 판정은 **주입**받는다(`is_path_safe`) — `utils/collision/collision_model` 의
단일 게이트를 그대로 쓰면 Phase 무관하게 같은 기준이 적용된다.
"""
from __future__ import annotations

import numpy as np


def _unit_step(a, b, step):
    """a 에서 b 방향으로 최대 step(rad, 최대성분 기준) 만큼."""
    d = b - a
    m = float(np.max(np.abs(d)))
    if m <= step:
        return b.copy(), True
    return a + d * (step / m), False


class _Tree:
    def __init__(self, root):
        self.nodes = [np.asarray(root, float)]
        self.parent = [-1]

    def nearest(self, q):
        d = np.max(np.abs(np.asarray(self.nodes) - q), axis=1)
        return int(np.argmin(d))

    def add(self, q, parent):
        self.nodes.append(np.asarray(q, float))
        self.parent.append(int(parent))
        return len(self.nodes) - 1

    def path_to(self, i):
        out = []
        while i >= 0:
            out.append(self.nodes[i])
            i = self.parent[i]
        return out[::-1]


def plan_joint_path(q0, q1, is_path_safe, *, lower, upper,
                    step=0.20, max_iter=3000, seed=0,
                    shortcut_iters=120, log=None):
    """q0 → q1 의 충돌-free 웨이포인트 리스트. 실패 시 None.

    is_path_safe(a, b) -> (bool, ...) 또는 bool  — 두 자세 사이 **직선 구간**의 안전성.
    lower/upper : 관절 한계(rad).
    step        : RRT 확장 폭(rad, 최대성분). 작을수록 정밀·느림.

    반환 경로는 [q0, ..., q1] 이며 **인접 쌍마다 is_path_safe 가 참**임이 보장된다.
    """
    def safe(a, b):
        r = is_path_safe(a, b)
        return bool(r[0]) if isinstance(r, tuple) else bool(r)

    q0 = np.asarray(q0, float)
    q1 = np.asarray(q1, float)
    lower = np.asarray(lower, float)
    upper = np.asarray(upper, float)

    # 1) 직선이 되면 계획할 필요가 없다 (대부분의 경우)
    if safe(q0, q1):
        return [q0, q1]
    if log:
        log("[path] 직선 막힘 — RRT-Connect 우회 탐색")

    rng = np.random.default_rng(seed)
    ta, tb = _Tree(q0), _Tree(q1)
    lo = np.maximum(lower, np.minimum(q0, q1) - np.pi)
    hi = np.minimum(upper, np.maximum(q0, q1) + np.pi)

    for it in range(int(max_iter)):
        q_rand = rng.uniform(lo, hi)
        # ── extend(ta) ──
        i = ta.nearest(q_rand)
        q_new, _ = _unit_step(ta.nodes[i], q_rand, step)
        q_new = np.clip(q_new, lower, upper)
        if not safe(ta.nodes[i], q_new):
            ta, tb = tb, ta                       # 실패해도 번갈아 성장
            continue
        ia = ta.add(q_new, i)
        # ── connect(tb → q_new) : 될 때까지 greedy ──
        j = tb.nearest(q_new)
        q_cur, jb = tb.nodes[j], j
        while True:
            q_next, reached = _unit_step(q_cur, q_new, step)
            q_next = np.clip(q_next, lower, upper)
            if not safe(q_cur, q_next):
                break
            jb = tb.add(q_next, jb)
            q_cur = q_next
            if reached:
                pa, pb = ta.path_to(ia), tb.path_to(jb)
                # ta 가 q0 쪽인지 확인(스왑 때문에 뒤바뀔 수 있다)
                if np.allclose(ta.nodes[0], q0):
                    path = pa + pb[::-1][1:]
                else:
                    path = pb + pa[::-1][1:]
                if log:
                    log(f"[path] 우회 발견 — {it+1}회 시도, 웨이포인트 {len(path)}개")
                return _shortcut(path, safe, rng, shortcut_iters, log)
        ta, tb = tb, ta                           # 번갈아
    if log:
        log(f"[path] ⚠ 우회 실패 ({max_iter}회) — 이 목표는 포기")
    return None


def _shortcut(path, safe, rng, iters, log=None):
    """무작위 구간을 직선으로 대체해 지그재그를 편다."""
    path = [np.asarray(p, float) for p in path]
    n0 = len(path)
    for _ in range(int(iters)):
        if len(path) <= 2:
            break
        i = int(rng.integers(0, len(path) - 2))
        j = int(rng.integers(i + 2, len(path)))
        if safe(path[i], path[j]):
            path = path[:i + 1] + path[j:]
    if log and len(path) != n0:
        log(f"[path] 평활화 — 웨이포인트 {n0} → {len(path)}개")
    return path
