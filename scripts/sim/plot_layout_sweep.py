"""plot_layout_sweep.py — eval_turntable_layout.py 의 CSV 를 그림으로.

    python scripts/sim/plot_layout_sweep.py
    #  --csv scripts/sim/log/turntable_layout.csv --out docs/figures/turntable_layout.png
"""
from __future__ import annotations

import argparse
import csv
import os
from collections import defaultdict

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

ELS = (30, 40, 50, 60, 70, 80)
PICK_ELS = (90, 75, 60, 45)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", default="scripts/sim/log/turntable_layout.csv")
    ap.add_argument("--out", default="docs/figures/turntable_layout.png")
    args = ap.parse_args()

    by_z = defaultdict(list)
    with open(args.csv) as f:
        for r in csv.DictReader(f):
            by_z[float(r["base_z"])].append({k: float(v) for k, v in r.items()})
    for z in by_z:
        by_z[z].sort(key=lambda r: r["x"])

    zs = sorted(by_z, reverse=True)
    fig, axes = plt.subplots(2, len(zs), figsize=(7.0 * len(zs), 9.0), squeeze=False)
    fig.suptitle("턴테이블 X 위치 스윕 — 스캔 각도 다양성과 파지 가능성 "
                 "(로봇 base 고정, 메시 충돌 판정)", fontsize=13)

    for c, z in enumerate(zs):
        rows = by_z[z]
        xs = [r["x"] for r in rows]

        # (상) 스캔 el 대역별 통과 az 수 — 스택 막대 + el 대역 수 라인
        ax = axes[0][c]
        bottom = [0.0] * len(xs)
        cmap = plt.get_cmap("viridis")
        for i, e in enumerate(ELS):
            v = [r[f"el{e}"] for r in rows]
            ax.bar(xs, v, width=0.04, bottom=bottom,
                   color=cmap(i / (len(ELS) - 1)), label=f"el {e}°")
            bottom = [b + y for b, y in zip(bottom, v)]
        ax2 = ax.twinx()
        ax2.plot(xs, [r["n_el"] for r in rows], "r.-", lw=2, ms=9,
                 label="el 대역 수 (6=30~80 전부)")
        ax2.set_ylim(0, 6.6)
        ax2.set_ylabel("el 대역 수", color="r")
        ax2.axhline(6, color="r", ls=":", lw=1)
        ax.set_title(f"스캔 — 로봇 base Z = {z:.3f} m")
        ax.set_xlabel("턴테이블 X (m)")
        ax.set_ylabel("통과 자세 수 (el×az)")
        ax.legend(fontsize=8, ncol=2, loc="upper left")
        ax2.legend(fontsize=8, loc="upper right")
        ax.grid(alpha=0.3, axis="y")

        # (하) 파지 접근각별 통과 수 — el=90(수직) 가능 여부가 핵심
        ax = axes[1][c]
        bottom = [0.0] * len(xs)
        cmap2 = plt.get_cmap("plasma")
        for i, e in enumerate(PICK_ELS):
            v = [r[f"pick{e}"] for r in rows]
            ax.bar(xs, v, width=0.04, bottom=bottom,
                   color=cmap2(i / (len(PICK_ELS) - 1)),
                   label=f"접근 el {e}°" + (" (수직)" if e == 90 else ""))
            bottom = [b + y for b, y in zip(bottom, v)]
        ax.set_title(f"F* 그리퍼 파지 — 로봇 base Z = {z:.3f} m")
        ax.set_xlabel("턴테이블 X (m)")
        ax.set_ylabel("통과 자세 수 (접근각×az)")
        ax.legend(fontsize=8, loc="upper left")
        ax.grid(alpha=0.3, axis="y")

        for a in (axes[0][c], axes[1][c]):
            a.axvline(0.365, color="k", ls="--", lw=1)
            a.annotate("현재 v3", (0.365, a.get_ylim()[1] * 0.96),
                       fontsize=8, ha="right", rotation=90, va="top")
            a.axvspan(-0.192, 0.192, color="tab:orange", alpha=0.08)

    fig.text(0.5, 0.005, "주황 음영 = 저울 상부 구간(X −0.192~0.192) — "
             "이 위에 놓으려면 철판이 필요하다", ha="center", fontsize=9)
    fig.tight_layout(rect=(0, 0.02, 1, 0.96))
    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    fig.savefig(args.out, dpi=130)
    print(f"저장: {args.out}")


if __name__ == "__main__":
    main()
