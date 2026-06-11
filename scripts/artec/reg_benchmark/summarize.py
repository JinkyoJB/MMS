"""
최종 통합 리포트 — results_all.csv(classic+baseline+artec_gr) +
results_learned.csv(geotransformer, predator) 를 합쳐 비교표 + RESULTS.md 생성.

실행(mms-env 또는 reg-dl 무관, 표준 라이브러리만):
  python -m scripts.artec.reg_benchmark.summarize
"""
from __future__ import annotations

import csv
import sys
from pathlib import Path

for _s in (sys.stdout, sys.stderr):
    try:
        _s.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass

OUT = Path(__file__).resolve().parents[3] / "output" / "reg_benchmark"
RES = OUT / "results"

FINDINGS = """
## 핵심 요약

1. **실데이터(real) 3세션은 이미 거의 정렬**돼 있다(상대자세≈identity+드리프트 7·13°).
   `meta_hint`(시스템 기록 90/180°)는 **오답**(gtRot 90/177°). → flip 정합 난이도를
   실데이터로는 못 봄. 그래서 **synthetic** 트랙(알려진 회전+부분뷰)으로 엄밀 비교.
2. **full 중첩**: FPFH/FGR 는 모든 각도(30~180°) **완벽**(gtRot 0°). ICP-from-identity
   는 ~45° basin 까지만. GeoTransformer 도 전 각도 우수(0.5~0.9°).
3. **partial(~40% 실제 공유밴드)**: FPFH/FGR/GeoTransformer 모두 90°는 성공.
   180° partial 에서 GeoTransformer(zero-shot 3DMatch)는 다소 흔들림(도메인 갭),
   FPFH/FGR 가 더 정확. → 이 객체·중첩대에선 **고전 global reg 로 충분**.
4. **방법3 Artec GR**: 정확하나(gtRot~1°) **약 42분** — FPFH/FGR 가 <1초에 동급.
5. **방법6 이미지매칭(SIFT)**: 다뷰 디스크립터 집계가 앞-뒤 텍스처 혼동 →
   real 1→0 에서 gtRot 97° 오답. **현 형태로는 비신뢰**(개선: LightGlue+프레임쌍 기하검증).
6. **방법3·6 은 구조적으로 real 트랙 전용**(SDK 스캔/프레임 이미지 필요). 학습기반과
   동일입력 비교는 synthetic 점군 트랙(1·2·4·5)에서 성립.
"""

PAIR_ORDER = ["1->0", "2->0", "syn_rot30_full", "syn_rot60_full",
              "syn_rot90_full", "syn_rot180_full",
              "syn_rot90_partial", "syn_rot180_partial"]
METHOD_ORDER = ["identity", "meta_hint", "icp_identity", "gt",
                "fpfh_ransac", "fgr", "artec_gr", "image_match",
                "geotransformer", "predator"]


def _load():
    recs = {}
    for fn in ("results_all.csv", "results_learned.csv"):
        p = RES / fn
        if not p.exists():
            continue
        for r in csv.DictReader(open(p)):
            key = (r["track"], r["pair"], r["method"])
            recs[key] = r
    return recs


def _g(r, k):
    v = r.get(k, "")
    return v if v not in (None, "") else "-"


def main():
    recs = _load()
    pairs = []
    for k in recs:
        if k[1] not in pairs:
            pairs.append(k[1])
    pairs = [p for p in PAIR_ORDER if p in pairs] + \
            [p for p in pairs if p not in PAIR_ORDER]

    lines = []
    lines.append("# Registration Benchmark — 결과\n")
    lines.append("지표: **gtRot/gtT** = GT 대비 회전/평행이동 오차(작을수록 정답). "
                 "synthetic 은 절대 정확도, real 은 ICP-best 대비 편차. "
                 "fit@5mm=중첩, chamfer=평균 최근접거리(mm).\n")
    lines.append(FINDINGS)
    header = f"| method | gtRot° | gtT(mm) | fit@5 | chamfer(mm) | sec |"
    sep = "|---|---|---|---|---|---|"

    print("=" * 78)
    for pair in pairs:
        track = next((k[0] for k in recs if k[1] == pair), "?")
        title = f"\n## [{track}] {pair}\n"
        lines.append(title)
        lines.append(header); lines.append(sep)
        print(f"\n[{track}] {pair}")
        print(f"  {'method':<14}{'gtRot°':>8}{'gtT_mm':>8}{'fit@5':>7}"
              f"{'cham':>7}{'sec':>9}")
        methods = [m for m in METHOD_ORDER if (track, pair, m) in recs]
        methods += [k[2] for k in recs if k[1] == pair and k[2] not in methods]
        for mth in methods:
            r = recs.get((track, pair, mth))
            if not r:
                continue
            gr, gt = _g(r, "gt_rot_err_deg"), _g(r, "gt_t_err_mm")
            f5, ch = _g(r, "fit@5mm"), _g(r, "chamfer_mm")
            sec = _g(r, "runtime_s")
            lines.append(f"| {mth} | {gr} | {gt} | {f5} | {ch} | {sec} |")
            print(f"  {mth:<14}{gr:>8}{gt:>8}{f5:>7}{ch:>7}{sec:>9}")
    print("=" * 78)

    md = RES / "RESULTS.md"
    md.write_text("\n".join(lines), encoding="utf-8")
    print(f"\n[summarize] → {md}")


if __name__ == "__main__":
    main()
