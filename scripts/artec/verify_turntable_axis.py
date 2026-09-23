#!/usr/bin/env python
"""verify_turntable_axis.py — 턴테이블 축을 **rim 클릭과 무관하게** 검증한다.

왜 필요한가
-----------
`T_B_F0` 는 rim 을 클릭해 원을 피팅해서 얻는다. 그런데 그 결과가 맞는지 확인할
독립 수단이 없었다:

  · 잔차·반경은 **점들끼리의 일관성**만 말한다. 중심이 통째로 밀려도 작게 나온다.
  · `check_calibration.py` 의 셀 캐시 교차검증은 **자기충족**이다 —
    `calibrate.py` 4단계가 캐시를 `T_B_F0` 에 맞춰 옮기므로 둘은 늘 일치한다.

여기서는 **물리를 쓴다.** 물체는 턴테이블 축 둘레로 돈다. preview 가 0/90/180/270°
에서 찍은 패치를 각각 −θ 로 되돌렸을 때, **참 축이면 패치들이 같은 표면에 겹친다.**
어긋난 축이면 안 겹친다. 그래서 "패치가 가장 잘 겹치는 축"을 찾으면 그게 참 축이고,
rim 캘리브 값과 비교할 수 있다.

⚠ **한계 — 물체가 축에서 치우치면 이 추정이 편향된다.**
  패치는 늘 카메라 쪽 면만 담는데, 물체가 치우쳐 있으면 회전각마다 카메라와의
  관계가 달라져 겹치는 영역이 줄고, 최적점이 물체 쪽으로 끌린다.
  합성 실측(2026-09-21, 정답 축을 아는 상태):

      물체 치우침   0mm → 추정 오차  0.4mm   (정합 1.4 → 1.4mm)
      물체 치우침  40mm → 추정 오차 13.7mm   (정합 20.8 → 14.1mm)
      물체 치우침  60mm → 추정 오차 21.6mm   (정합 30.7 → 17.4mm)

  그래서 이 도구는 **"축이 맞다"를 확인하는 데는 강하고**(치우침이 없으면
  0.4mm), "얼마나 틀렸다"를 단정하는 데는 약하다. 큰 차이가 나오면 물체를
  **원판 한가운데**에 놓고 다시 재는 것이 먼저다.

입력은 `--until preview` 가 남기는 `output/<RUN>/debug/preview_points_*.npz` 다
(방향별 패치가 들어 있다). 장비도 로봇도 건드리지 않는다.

    python scripts/artec/verify_turntable_axis.py              # 최신 파일
    python scripts/artec/verify_turntable_axis.py <npz 경로>
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(_ROOT))

#: 이만큼 차이 나면 rim 캘리브를 의심한다 (mm).
WARN_MM, FAIL_MM = 10.0, 25.0


def _rot(P, ap, ad, ang):
    a = np.asarray(ad, float); a = a / np.linalg.norm(a)
    c, s = np.cos(ang), np.sin(ang)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    R = np.eye(3) * c + np.outer(a, a) * (1 - c) + K * s
    return (np.asarray(P, float) - ap) @ R.T + ap


def _fit_score(patches, ap0, ad0, ap, ad):
    """후보 축으로 다시 합쳤을 때 패치 간 최근접거리 중앙값 (mm)."""
    from scipy.spatial import cKDTree
    P = [_rot(_rot(pp, ap0, ad0, +t), ap, ad, -t) for t, pp in patches]
    ds = []
    for i in range(len(P)):
        for j in range(i + 1, len(P)):
            if len(P[i]) < 50 or len(P[j]) < 50:
                continue
            a = np.median(cKDTree(P[j]).query(P[i])[0])
            b = np.median(cKDTree(P[i]).query(P[j])[0])
            ds.append(0.5 * (a + b))
    return float(np.median(ds)) * 1000.0 if ds else float("nan")


def main() -> int:
    from scipy.optimize import minimize

    if len(sys.argv) > 1:
        npz = Path(sys.argv[1])
    else:
        # 최근 run 부터 — output/<RUN>/debug/preview_points_*.npz (utils/run_paths.py)
        cand = sorted((_ROOT / "output").glob("*/debug/preview_points_*.npz"),
                      key=lambda q: (q.parents[1].name, q.name))
        if not cand:
            print("✘ output/<RUN>/debug/preview_points_*.npz 가 없다.")
            print("   먼저:  python main_artec.py --until preview")
            return 2
        npz = cand[-1]
    d = np.load(npz)
    ap0 = np.asarray(d["axis_pt_B"], float)
    ad0 = np.asarray(d["axis_dir_B"], float); ad0 /= np.linalg.norm(ad0)

    patches, i = [], 0
    while f"patch{i}_pts" in d.files:
        patches.append((float(d[f"patch{i}_theta"][0]),
                        np.asarray(d[f"patch{i}_pts"], float)))
        i += 1
    print("=" * 64)
    print("  턴테이블 축 독립 검증 — preview 패치 정합 (장비 무동작)")
    print("=" * 64)
    print(f"  입력: {npz.name}")
    if len(patches) < 3:
        print(f"  ✘ 방향별 패치가 {len(patches)}개뿐 — 최소 3개 필요")
        print("     (옛 npz 는 병합본만 들어 있다. 다시 preview 를 돌릴 것)")
        return 2
    print(f"  패치 {len(patches)}개: "
          + ", ".join(f"θ={np.degrees(t):+.0f}°({len(p):,}pt)" for t, p in patches))

    s0 = _fit_score(patches, ap0, ad0, ap0, ad0)

    def obj(v):
        return _fit_score(patches, ap0, ad0,
                          ap0 + np.array([v[0], v[1], 0.0]),
                          np.array([v[2], v[3], 1.0]))

    r = minimize(obj, [0.0, 0.0, 0.0, 0.0], method="Nelder-Mead",
                 options={"xatol": 2e-4, "fatol": 0.02, "maxiter": 400})
    dx, dy = float(r.x[0]), float(r.x[1])
    ap1 = ap0 + np.array([dx, dy, 0.0])
    ad1 = np.array([r.x[2], r.x[3], 1.0]); ad1 /= np.linalg.norm(ad1)
    off = float(np.hypot(dx, dy)) * 1000.0

    print()
    print(f"  rim 캘리브 축 : [{ap0[0]:+.4f}, {ap0[1]:+.4f}]  "
          f"기울기 {np.degrees(np.arccos(abs(ad0[2]))):.1f}°   정합 {s0:.1f}mm")
    print(f"  패치정합 최적 : [{ap1[0]:+.4f}, {ap1[1]:+.4f}]  "
          f"기울기 {np.degrees(np.arccos(abs(ad1[2]))):.1f}°   정합 {r.fun:.1f}mm")
    print(f"  차이          : {off:.0f} mm  (Δx {dx*1000:+.0f}, Δy {dy*1000:+.0f})")
    print()

    # 개선이 없으면 애초에 판별력이 없는 데이터다 — 그걸 먼저 말한다.
    if not (s0 > r.fun + 0.15 * s0):
        print("  [판정 보류] 최적화해도 정합이 거의 안 는다 — 이 데이터로는")
        print("             축을 구분할 수 없다 (패치가 겹치는 영역이 부족하거나")
        print("             물체가 회전대칭).")
        return 0
    if off < WARN_MM:
        print(f"  [OK]   rim 캘리브 축이 물체 회전과 맞는다 ({off:.0f}mm < {WARN_MM:.0f}mm)")
        return 0

    # ★ 물체 치우침을 함께 잰다 — 이 값이 크면 위 차이를 **믿으면 안 된다**
    #   (도구 docstring 의 편향 표 참조).
    allp = np.vstack([p for _t, p in patches])
    ecc = float(np.hypot(*(allp[:, :2].mean(0) - ap0[:2]))) * 1000.0
    lvl = "FAIL" if off >= FAIL_MM else "WARN"
    print(f"  [{lvl}] rim 축과 패치정합 최적이 {off:.0f}mm 다르다")
    print(f"         물체 치우침(패치 중심 ↔ rim 축) {ecc:.0f}mm")
    print()
    if ecc > 25.0:
        print("  ⚠ 물체가 축에서 많이 치우쳐 있다 — **이 차이를 축 오차로 읽으면 안 된다.**")
        print("    치우친 물체는 추정을 물체 쪽으로 끌어당긴다(도구 설명의 편향 표).")
        print()
        print("  먼저 할 일:")
        print("   1) 물체를 **원판 한가운데**로 옮기고 preview 재실행 → 이 검증 반복")
        print("      치우침이 작아졌는데도 차이가 남으면 그때가 진짜 축 오차다")
    else:
        print("  물체는 축 근처에 있다 — 이 차이는 축 오차로 볼 만하다.")
        print()
        print("  다음 할 일:")
        print("   1) rim 재캘리브 — 점을 **원주 전체에 고르게** (호 120° 이상):")
        print("        python scripts/artec/calibrate.py --only 3")
        print("      끝나면 `호 길이` 출력과 output/debug/rim_points_*.npz 를 확인")
    print("   2) 다시 preview 를 돌리고 이 스크립트로 재검증")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
