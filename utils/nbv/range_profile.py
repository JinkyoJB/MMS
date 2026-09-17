"""range_profile.py — 거리별 반환점 히스토그램으로 **적정 작업거리**를 찾는다.

왜 필요한가
-----------
Artec Studio 로 손스캔할 때 하는 일을 코드로 옮긴 것이다 — 스캐너를 가까이·멀리
움직이며 **어느 거리에서 점이 잘 들어오는지** 보고 그 거리를 유지한다. 추적이
끊기는 대부분의 원인이 "작동거리를 벗어나 프레임이 비는 것" 이라서, 자세를 고를 때
이 창 안에 들어가게 하면 tracking-lost 가 줄어든다.

지금까지 `SensorModel.dof = (0.20, 0.30)` 은 **스펙에서 가져온 가정**이었다.
그런데 주석에 남은 실측은 다르다 — "이 값을 (0.17, 0.35) 로 넓히자 모델이 단일
자세로 173mm 를 덮는다고 봤지만 실물에서 자세당 실제 캡처는 ~87mm 였다".
즉 **진짜 창은 더 좁다.** 이 모듈은 그걸 가정하지 않고 잰다.

무엇을 재나
-----------
두 가지가 다르다. 둘 다 같은 원시값(점별 카메라까지 거리)에서 나온다.

  ① **깊이 히스토그램** (자세 하나) — "이 센서는 어느 깊이에서 점을 주나"
     → 센서 특성. `SensorModel.dof` 를 여기서 정한다. 한 번 재면 된다.
  ② **스탠드오프 스윕** — "축에서 얼마 떨어져야 점이 제일 잘 들어오나"
     → 운용 지점. 물체 반경에 따라 달라지므로 대상마다 잴 수 있다.

sim·real 공용: 백엔드는 **캡처 콜백 하나만** 주입한다. 판단은 전부 여기서 한다
(`collect_planning_points` 와 같은 구조).

⚠ 거리는 **카메라 원점 기준 3D 거리**다. 광축 깊이(-Z)가 아니다 — Artec Studio 가
  보여주는 것도, 작동거리 스펙이 말하는 것도 표면까지의 거리다.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable, List, Optional

import numpy as np


@dataclass
class RangeHistogram:
    """점별 카메라 거리 분포."""
    edges: np.ndarray          # (nb+1,) m
    counts: np.ndarray         # (nb,)
    n_total: int

    @property
    def centers(self) -> np.ndarray:
        return 0.5 * (self.edges[:-1] + self.edges[1:])

    def peak_m(self) -> float:
        """점이 가장 많이 들어오는 거리 (m). 비었으면 nan."""
        if not self.counts.any():
            return float("nan")
        return float(self.centers[int(np.argmax(self.counts))])

    def window(self, frac: float = 0.25):
        """반환이 **피크의 frac 이상**인 연속 구간 (near, far) m.

        `frac` 은 "쓸 만하다" 의 기준이다. 0.25 면 피크 대비 1/4 이상 들어오는 대역.
        꼬리를 잘라내야 스펙(0.2~0.3)처럼 낙관적인 창이 안 나온다.
        """
        if not self.counts.any():
            return (float("nan"), float("nan"))
        thr = self.counts.max() * float(frac)
        ok = self.counts >= thr
        i = int(np.argmax(self.counts))
        lo = i
        while lo > 0 and ok[lo - 1]:
            lo -= 1
        hi = i
        while hi < len(ok) - 1 and ok[hi + 1]:
            hi += 1
        return (float(self.edges[lo]), float(self.edges[hi + 1]))


@dataclass
class StandoffSample:
    standoff: float            # 축에서 카메라까지 (m)
    n_points: int              # 물체로 분류된 점 수
    hist: Optional[RangeHistogram] = None
    note: str = ""


@dataclass
class RangeProfile:
    samples: List[StandoffSample] = field(default_factory=list)

    def best(self) -> Optional[StandoffSample]:
        """점이 가장 많이 들어온 스탠드오프."""
        ok = [s for s in self.samples if s.n_points > 0]
        return max(ok, key=lambda s: s.n_points) if ok else None


def point_ranges(pts, cam_pos) -> np.ndarray:
    """점별 **카메라까지 3D 거리** (m). pts·cam_pos 는 같은 프레임."""
    P = np.asarray(pts, float)
    if P.size == 0:
        return np.zeros(0)
    return np.linalg.norm(P - np.asarray(cam_pos, float)[None, :], axis=1)


def range_histogram(pts, cam_pos, bin_m: float = 0.005,
                    lo: float = 0.10, hi: float = 0.50) -> RangeHistogram:
    r = point_ranges(pts, cam_pos)
    edges = np.arange(lo, hi + bin_m * 0.5, bin_m)
    counts, _ = np.histogram(r, bins=edges)
    return RangeHistogram(edges=edges, counts=counts, n_total=int(len(r)))


def render(hist: RangeHistogram, width: int = 52, frac: float = 0.25,
           only_nonzero: bool = True) -> str:
    """터미널용 가로 막대 히스토그램. GUI 없이 SSH 로도 본다."""
    if not hist.counts.any():
        return "   (점 없음)"
    mx = int(hist.counts.max())
    near, far = hist.window(frac)
    out = []
    for c, lo, hiv in zip(hist.counts, hist.edges[:-1], hist.edges[1:]):
        if only_nonzero and c == 0:
            continue
        bar = "█" * max(1, int(round(c / mx * width))) if c else ""
        mark = " ◀ 창" if (lo >= near - 1e-9 and hiv <= far + 1e-9) else ""
        out.append(f"   {lo*1000:4.0f}~{hiv*1000:4.0f}mm {c:>7,} |{bar}{mark}")
    out.append(f"   피크 {hist.peak_m()*1000:.0f}mm · "
               f"쓸만한 창 {near*1000:.0f}~{far*1000:.0f}mm "
               f"(피크의 {frac*100:.0f}% 이상) · 총 {hist.n_total:,}점")
    return "\n".join(out)


def sweep_standoff(capture_at: Callable[[float], tuple], standoffs,
                   bin_m: float = 0.005, log=None) -> RangeProfile:
    """스탠드오프를 훑으며 거리 히스토그램을 모은다.

      capture_at(standoff) -> (pts, cam_pos) | None
          그 스탠드오프로 **구동한 뒤** 캡처해 (물체점, 카메라위치) 를 같은 프레임으로.
          이동이 거부되면 None 을 돌려준다 — 그 샘플은 건너뛴다.

    백엔드가 주입하는 것은 이 콜백 하나뿐이다. sim·real 이 같은 판단을 쓴다.
    """
    prof = RangeProfile()
    for d in standoffs:
        got = capture_at(float(d))
        if got is None:
            prof.samples.append(StandoffSample(float(d), 0, note="이동 거부"))
            if log:
                log(f"  standoff {d*1000:3.0f}mm — 이동 거부")
            continue
        pts, cam = got
        pts = np.asarray(pts, float)
        h = range_histogram(pts, cam, bin_m=bin_m) if len(pts) else None
        prof.samples.append(StandoffSample(float(d), int(len(pts)), hist=h))
        if log:
            pk = "-" if h is None else f"{h.peak_m()*1000:.0f}mm"
            log(f"  standoff {d*1000:3.0f}mm — {len(pts):>7,}점  피크거리 {pk}")
    return prof


def merged_histogram(prof: RangeProfile) -> Optional[RangeHistogram]:
    """스윕 전체를 합친 히스토그램 = **센서의 깊이 특성**.

    스탠드오프가 달라도 '카메라에서 표면까지 거리' 축은 같으므로 그대로 더할 수 있다.
    이렇게 모아야 `SensorModel.dof` 를 정할 만큼 표본이 쌓인다.
    """
    hs = [s.hist for s in prof.samples if s.hist is not None]
    if not hs:
        return None
    return RangeHistogram(edges=hs[0].edges,
                          counts=np.sum([h.counts for h in hs], axis=0),
                          n_total=int(sum(h.n_total for h in hs)))
