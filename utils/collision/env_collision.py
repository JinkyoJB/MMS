"""env_collision.py — 셀 구조물(벽·상판·저울·툴스탠드)을 **실제 메시**로 충돌검사.

왜 필요한가
----------
기존 `CollisionWorld` 의 장애물은 **세 개뿐**이었다:
    turntable(캡슐) · tt_frame(AABB 박스) · keepout(원기둥, 기본 OFF)
즉 **벽·상판(Z=0.510)·저울·툴스탠드가 전혀 등록돼 있지 않았다.**

이건 다른 충돌 이슈들과 성격이 다르다:
  · roll/시드/캡슐 문제 = **오탐**(갈 수 있는데 못 간다) → 손해는 성능
  · 셀 구조물 누락      = **미탐**(칠 수 있는데 안전하다고 한다) → 손해는 **파손**
el 30° 처럼 낮은 관측자세는 팔이 아래로 뻗으므로 상판과 실제로 간섭할 수 있다.

원리 (mesh_self_collision 과 대칭)
---------------------------------
환경 메시를 **로봇 base 프레임**으로 한 번 구워 KD-tree 를 만든다. 자세마다 링크/툴
점을 해석 FK 로 base 에 배치해 질의한다. base 프레임에 저장하므로 sim·real 공용이며,
로봇을 구동하지 않고 판정할 수 있다(자세당 수 ms).

캐시 생성: `scripts/sim/export_env_mesh.py`
"""
from __future__ import annotations

import os

import numpy as np

#: `None` = `utils/collision/layout.py` 가 백엔드에 맞는 레이아웃을 고른다.
#  (전역 활성본 하나를 쓰면 sim 이 실측 셀로 검사된다 — layout.py 주석 참고)
DEFAULT_NPZ = None

# 검사할 링크. link1/2 는 천장 마운트에 붙어 있어 프레임과 상시 근접 → 제외.
ENV_VS_LINKS = (3, 4, 5, 6, 7)


class EnvCollision:
    """셀 구조물 ↔ 로봇 최소거리 판정기 (base 프레임)."""

    def __init__(self, npz_path: str = DEFAULT_NPZ, margin_m: float = 0.02,
                 links_npz: str | None = None):
        from scipy.spatial import cKDTree
        from utils.collision.mesh_self_collision import DEFAULT_NPZ as LINKS_NPZ

        if npz_path is None:                       # 백엔드별 레이아웃 해석
            from utils.collision.layout import resolve_env_npz
            npz_path = resolve_env_npz()
        self.npz_path = npz_path
        d = np.load(npz_path)
        self.margin = float(margin_m)
        self.env = np.asarray(d["env"], float)          # base 프레임 점군
        self.names = list(d["names"]) if "names" in d else []
        self._tree = cKDTree(self.env)
        L = np.load(links_npz or LINKS_NPZ)
        self.links = {i: np.asarray(L[f"link{i}"], float) for i in range(1, 8)}
        self.tool = np.asarray(L["tool"], float)        # link7 로컬

    def clearance(self, q, tool_extra=None):
        """자세 q 에서 로봇 ↔ 환경 최소거리(m), 그리고 가장 가까운 링크 이름."""
        from utils.robot import xarm7_kinematics as kin

        F = kin._fk_frames_m(np.asarray(q, float))
        best, who = 1e9, ""
        for i in ENV_VS_LINKS:
            V = self.links[i]
            if not len(V):
                continue
            T = F[i]
            d = float(self._tree.query(V @ T[:3, :3].T + T[:3, 3], k=1)[0].min())
            if d < best:
                best, who = d, f"link{i}"
        tool = self.tool if tool_extra is None else np.asarray(tool_extra, float)
        if len(tool):
            T = F[7]
            d = float(self._tree.query(tool @ T[:3, :3].T + T[:3, 3], k=1)[0].min())
            if d < best:
                best, who = d, "tool"
        return best, who

    def collides(self, q, tool_extra=None):
        d, who = self.clearance(q, tool_extra)
        return (d < self.margin), who


_cached = {}


def get_default(npz_path: str = DEFAULT_NPZ, margin_m: float = 0.02):
    """캐시가 있으면 판정기를, 없으면 None(호출부는 기존 캡슐 world 로 폴백)."""
    if npz_path is None:
        from utils.collision.layout import resolve_env_npz
        npz_path = resolve_env_npz()
    key = (npz_path, margin_m)
    if key not in _cached:
        try:
            _cached[key] = EnvCollision(npz_path, margin_m)
        except Exception as e:
            print(f"[env_collision] 캐시 사용 불가({type(e).__name__}: {e}) — "
                  f"기존 장애물(캡슐)만 사용. 생성: scripts/sim/export_env_mesh.py")
            _cached[key] = None
    return _cached[key]
