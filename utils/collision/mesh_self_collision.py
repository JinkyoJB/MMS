"""mesh_self_collision.py — 실제 링크 메시로 자가충돌을 판정한다(캡슐 근사 대체).

왜 필요한가
----------
`robot_collision` 의 캡슐 근사는 이 팔(xArm7 + Spider 툴체인)에서 **오탐이 심하다**:

  · 링크 캡슐이 관절원점을 잇는 **직선**이라 꺾인 link4(길이 351mm, r=50mm)를 잘못 덮는다
    → 툴↔link4 축간 58mm 로 계산되지만 **실제 메시 거리는 93mm 이상**
  · 툴 캡슐이 플랜지~카메라를 균일 반경 95mm 로 덮어 플랜지 쪽을 38mm 과대평가하고,
    정작 265mm 까지 뻗는 Spider 끝단은 174mm 에서 잘려 과소평가된다

그 결과 Phase2 NBV 가 **후보 32자세를 IK 로 전부 풀고도 캡슐 자가충돌로 전부 기각**해
윗면 보강이 불가능했다(2026-08-13 실측: `IK실패 0, swept충돌 32, 통과 0 / self×32`).

원리
----
링크 메시를 **링크 로컬 좌표**로 한 번 샘플링해 KD-tree 를 짓고, 자세마다 툴 점을 각
링크 로컬로 **역변환**해 질의한다. 트리를 다시 짓지 않으므로 자세당 수 ms 다.
프레임은 해석 FK(`xarm7_kinematics._fk_frames_m`) 를 쓴다 — USD 링크 프레임 대비
위치 1.5~8.6mm·회전 1.4° 로 일치함을 실측 확인했다(로봇을 sim 에서 구동할 필요 없음).

캐시 생성: `scripts/sim/export_link_meshes.py`
"""
from __future__ import annotations

import os

import numpy as np

DEFAULT_NPZ = os.path.join(os.path.dirname(__file__), "data",
                           "xarm7_spider_links.npz")

# 툴이 스칠 수 있는 링크. link6/7 은 툴이 직접 붙은 인접 링크라 항상 붙어 있다 → 제외.
TOOL_VS_LINKS = (1, 2, 3, 4, 5)
# 손목 뭉치(link6)가 접근할 수 있는 몸통 링크.
LINK6_VS_LINKS = (1, 2, 3)


class MeshSelfCollision:
    """실제 메시 기반 자가충돌 판정기.

    clearance(q) -> m 단위 최소거리. margin 보다 작으면 충돌로 본다.
    """

    def __init__(self, npz_path: str = DEFAULT_NPZ, margin_m: float = 0.02):
        from scipy.spatial import cKDTree

        d = np.load(npz_path)
        self.margin = float(margin_m)
        self.links = {i: np.asarray(d[f"link{i}"], float) for i in range(1, 8)}
        self.tool = np.asarray(d["tool"], float)          # link7 로컬
        self._trees = {i: (cKDTree(v) if len(v) else None)
                       for i, v in self.links.items()}

    # ── 내부 ────────────────────────────────────────────────────────────────
    @staticmethod
    def _to_local(P, T):
        """world/base 점 P 를 프레임 T 의 로컬로."""
        return (P - T[:3, 3]) @ T[:3, :3]

    def _min_to(self, P_base, frames, idxs):
        best, who = 1e9, ""
        for i in idxs:
            t = self._trees.get(i)
            if t is None:
                continue
            d = float(t.query(self._to_local(P_base, frames[i]), k=1)[0].min())
            if d < best:
                best, who = d, f"link{i}"
        return best, who

    # ── 공개 ────────────────────────────────────────────────────────────────
    def clearance(self, q, tool_extra=None):
        """자세 q 의 최소 자가거리(m)와 그 상대 링크 이름.

        tool_extra: 툴을 교체한 경우(예: 그리퍼) link7 로컬 점군으로 대체.
        """
        from utils.robot import xarm7_kinematics as kin

        F = kin._fk_frames_m(np.asarray(q, float))        # (8,4,4) base 기준
        tool = self.tool if tool_extra is None else np.asarray(tool_extra, float)
        Pt = tool @ F[7][:3, :3].T + F[7][:3, 3]
        best, who = self._min_to(Pt, F, TOOL_VS_LINKS)
        if len(self.links[6]):
            P6 = self.links[6] @ F[6][:3, :3].T + F[6][:3, 3]
            d6, w6 = self._min_to(P6, F, LINK6_VS_LINKS)
            if d6 < best:
                best, who = d6, w6 + "(link6)"
        return best, who

    def collides(self, q, tool_extra=None):
        d, who = self.clearance(q, tool_extra)
        return (d < self.margin), who


_cached = {}


def get_default(npz_path: str = DEFAULT_NPZ, margin_m: float = 0.02):
    """캐시가 있으면 판정기를, 없으면 None 을 준다(호출부는 캡슐로 폴백)."""
    key = (npz_path, margin_m)
    if key not in _cached:
        try:
            _cached[key] = MeshSelfCollision(npz_path, margin_m)
        except Exception as e:                       # 캐시 없음/scipy 없음 등
            print(f"[mesh_self_collision] 캐시 사용 불가({type(e).__name__}: {e}) "
                  f"— 캡슐 근사로 폴백. 생성: scripts/sim/export_link_meshes.py")
            _cached[key] = None
    return _cached[key]
