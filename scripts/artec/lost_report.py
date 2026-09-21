"""lost_report.py — run 이벤트 로그(`output/events_*.jsonl`)에서 tracking-lost 를 집계한다.

    python scripts/artec/lost_report.py                 # 최근 run 5개
    python scripts/artec/lost_report.py -n 20           # 최근 20개
    python scripts/artec/lost_report.py output/events_20260921_162322.jsonl

run 마다: 어디서(stage/band/θ) 잃었는지, 밴드 전환이 몇 번 살고 죽었는지(잃은 α),
되돌아가기·자동 복구가 먹었는지. 마지막에 전체 합계.
"""
from __future__ import annotations

import argparse
import json
import sys
from collections import Counter, defaultdict
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


def load(p: Path):
    out = []
    for line in p.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if line:
            try:
                out.append(json.loads(line))
            except json.JSONDecodeError:
                pass
    return out


def report(p: Path, agg: dict) -> None:
    ev = load(p)
    tag = p.stem.replace("events_", "")
    lost = [e for e in ev if e["kind"] == "lost"]
    moves = [e for e in ev if e["kind"] == "band_move" and e.get("outcome")]
    recov = [e for e in ev if e["kind"] == "recovery" and "ok" in e]
    merges = [e for e in ev if e["kind"] == "merge"]
    print(f"\n━━ run {tag}  (이벤트 {len(ev)}개) ━━")
    if lost:
        print(f"  lost {len(lost)}회:")
        for e in lost:
            print(f"    t={e['t']:7.1f}s  {e.get('stage','?'):10s} band {e.get('band','?')}/{e.get('n_bands','?')}"
                  f"  θ={e.get('theta_deg', 0):+7.1f}°  ok프레임={e.get('frames_ok','?')}  {str(e.get('reason',''))[:60]}")
    else:
        print("  lost 없음")
    if moves:
        ok = [m for m in moves if m["outcome"] == "ok"]
        rel = [m for m in moves if m["outcome"] == "relocalized"]
        bad = [m for m in moves if m["outcome"] == "failed"]
        print(f"  밴드 전환 {len(moves)}회: 바로 성공 {len(ok)} · 되돌아가 성공 {len(rel)} · 실패 {len(bad)}")
        for m in moves:
            if m["outcome"] != "ok":
                print(f"    {m.get('stage','?')} band {m.get('from_band')}→{m.get('to_band')}: {m['outcome']}"
                      f"  잃은 α={m.get('lost_alpha')}  되찾은 α={m.get('found_alpha')}  시도={m.get('tries')}")
    if recov:
        print(f"  자동 복구 {len(recov)}회: 성공 {sum(1 for r in recov if r['ok'])}")
    if merges:
        c = Counter(m.get("method", "?") for m in merges)
        print(f"  병합 {len(merges)}회: " + ", ".join(f"{k} {v}" for k, v in c.items()))
    agg["runs"] += 1
    agg["lost"] += len(lost)
    for e in lost:
        agg["lost_by_stage"][e.get("stage", "?")] += 1
    for m in moves:
        agg["moves"][m["outcome"]] += 1
        if m.get("lost_alpha") is not None:
            agg["lost_alpha"].append(float(m["lost_alpha"]))
    for r in recov:
        agg["recovery"][bool(r["ok"])] += 1


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("files", nargs="*")
    ap.add_argument("-n", type=int, default=5)
    a = ap.parse_args()
    files = [Path(f) for f in a.files] or sorted((ROOT / "output").glob("events_*.jsonl"))[-a.n:]
    if not files:
        print("events_*.jsonl 이 없다 — run 을 한 번 돌리면 output/ 에 생긴다")
        return 1
    agg = {"runs": 0, "lost": 0, "lost_by_stage": Counter(), "moves": Counter(),
           "lost_alpha": [], "recovery": Counter()}
    for f in files:
        report(f, agg)
    print(f"\n━━ 합계 ({agg['runs']} run) ━━")
    print(f"  lost {agg['lost']}회  — 단계별: " + ", ".join(f"{k} {v}" for k, v in agg["lost_by_stage"].items()))
    if agg["moves"]:
        n = sum(agg["moves"].values())
        print(f"  밴드 전환 {n}회 — " + ", ".join(f"{k} {v}" for k, v in agg["moves"].items()))
    if agg["lost_alpha"]:
        al = sorted(agg["lost_alpha"])
        print(f"  전환 중 잃은 α: n={len(al)} 중앙값={al[len(al)//2]:.2f} 범위 {al[0]:.2f}~{al[-1]:.2f}")
    if agg["recovery"]:
        print(f"  자동 복구: 성공 {agg['recovery'][True]} / 실패 {agg['recovery'][False]}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
