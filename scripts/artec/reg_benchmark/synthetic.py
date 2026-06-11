"""
합성 controlled-GT 시나리오 생성.

실제 데이터(153357)의 세 세션은 이미 정렬돼 있어 'flip 정합' 난이도를 안 담는다.
여기선 session0 에 **알려진 변환 T_gt** 를 적용하고(+부분 crop 으로 중첩 조절)
저중첩 flip 을 인위적으로 만든다 → 각 방법이 T_gt 를 얼마나 정확히 복원하는지
ΔR/Δt 로 엄밀 평가.

시나리오
  syn_rot90_full   : 90° 회전, 전체(중첩 100%)  — 전역 수렴/대칭 견고성
  syn_rot180_full  : 180° 회전, 전체
  syn_rot90_partial: 90° 회전 + 상보 crop(중첩 ~40%) — 저중첩 난이도
  syn_rot180_partial

좌표계: ref = session0 그대로. src = crop_src 를 T_gt 로 옮긴 것.
       따라서 방법이 찾아야 할 정답(src→ref)은 inv(T_gt). 평가에선 inv(T_gt) 를 GT.
"""
from __future__ import annotations

from dataclasses import dataclass
import numpy as np
import open3d as o3d


def _rot(axis: str, deg: float) -> np.ndarray:
    a = np.radians(deg)
    c, s = np.cos(a), np.sin(a)
    R = {"x": np.array([[1, 0, 0], [0, c, -s], [0, s, c]]),
         "y": np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]]),
         "z": np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])}[axis]
    T = np.eye(4); T[:3, :3] = R
    return T


def _crop_band(pcd: o3d.geometry.PointCloud, axis_idx: int,
               lo_pct: float, hi_pct: float) -> o3d.geometry.PointCloud:
    """axis_idx 좌표의 [lo_pct, hi_pct] 백분위 밴드만 남김.

    회전축과 같은 축으로 자르면(회전이 그 좌표를 보존) 회전 후에도 밴드가
    유지돼 ref/src 가 겹치는 부분뷰를 만들 수 있다."""
    pts = np.asarray(pcd.points)
    c = pts[:, axis_idx]
    lo, hi = np.percentile(c, [lo_pct, hi_pct])
    mask = (c >= lo) & (c <= hi)
    return pcd.select_by_index(np.where(mask)[0])


@dataclass
class SynScenario:
    name: str
    ref: o3d.geometry.PointCloud
    src: o3d.geometry.PointCloud
    T_gt: np.ndarray            # 정답 src→ref ( = inv(applied) )
    expected_deg: float


def build_scenarios(session0: o3d.geometry.PointCloud,
                    pivot_axis="y") -> list[SynScenario]:
    """합성 시나리오:
      - full(중첩 100%): 회전 30/60/90/180° — 전역 수렴/대칭 견고성.
      - partial(중첩 ~40%): 회전 90/180° + 회전축 밴드 crop(ref=[0,70]%,
        src=[30,100]%) → 회전이 축좌표 보존하므로 [30,70]% 밴드가 실제 공유 표면.
    """
    base = o3d.geometry.PointCloud(session0)
    c = base.get_center()
    ax_idx = {"x": 0, "y": 1, "z": 2}[pivot_axis]
    out = []

    def applied(deg):
        R = _rot(pivot_axis, deg)
        Tc = np.eye(4); Tc[:3, 3] = c
        Tnc = np.eye(4); Tnc[:3, 3] = -c
        return Tc @ R @ Tnc

    # full 중첩 — 회전각 스윕
    for deg in (30.0, 60.0, 90.0, 180.0):
        Ta = applied(deg)
        src_full = o3d.geometry.PointCloud(base).transform(Ta)
        out.append(SynScenario(f"syn_rot{int(deg)}_full", base, src_full,
                               np.linalg.inv(Ta), deg))

    # partial 중첩 — 회전축 밴드 crop (공유 밴드 [30,70]%)
    for deg in (90.0, 180.0):
        Ta = applied(deg)
        ref_b = _crop_band(base, ax_idx, 0.0, 70.0)
        src_b = _crop_band(base, ax_idx, 30.0, 100.0)
        src_b = o3d.geometry.PointCloud(src_b).transform(Ta)
        ref_b.estimate_normals(); src_b.estimate_normals()
        out.append(SynScenario(f"syn_rot{int(deg)}_partial", ref_b, src_b,
                               np.linalg.inv(Ta), deg))
    return out
