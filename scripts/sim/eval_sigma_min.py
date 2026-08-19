"""eval_sigma_min.py — xArm7 σ_min 분포 실측 + 논문 그림 생성.

특이점 임계 `view_pose.DEFAULT_MIN_SIGMA = 0.05` 의 **근거 수치를 재현**한다.
순수 numpy — Isaac 불필요.

  σ_min = 단위정규화 자코비안 J̃ 의 최소 특이값 = 특이점까지의 거리
  J̃ = [ J_v/1000 ; L_c·J_ω ],  L_c = 0.30 m   (utils/robot/xarm7_kinematics)

사용:
  python scripts/sim/eval_sigma_min.py [--n 1500] [--seed 0] [--out <png>]

그림은 KSMTE 규정에 맞춰 **라벨 전부 영문 · 9pt 이하**로 그린다.
"""
from __future__ import annotations

import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))

from utils.robot import xarm7_kinematics as kin          # noqa: E402
from utils.robot.view_pose import DEFAULT_MIN_SIGMA      # noqa: E402

D = np.radians

# 참조 자세 — 라벨은 그림·표에 그대로 쓰이므로 영문
REF_POSES = {
    "Home (real)":        D([0.0, -18.4, 0.0, 70.6, 0.0, 60.0, -45.0]),
    "Home (sim)":         D([-7.65, -75.61, -8.95, 78.64, 2.81, 126.71, -69.63]),
    "Elbow extended (q4=0)":  D([0.0, -18.4, 0.0, 0.0, 0.0, 60.0, -45.0]),
    "q4 = 140 deg":       D([0.0, -18.4, 0.0, 140.0, 0.0, 60.0, -45.0]),
    "Wrist aligned (q5=0)": D([0.0, -18.4, 0.0, 70.6, 0.0, 0.0, -45.0]),
}


def sample(n, seed):
    rng = np.random.default_rng(seed)
    lo, hi = kin.JOINT_LOWER, kin.JOINT_UPPER
    Q = rng.uniform(lo, hi, size=(n, 7))
    return Q, np.array([kin.sigma_min(q) for q in Q])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=1500)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--out", default=os.path.join(
        os.path.dirname(__file__), "..", "..", "..", "8_paper",
        "2_논문작성", "figures", "fig_sigma_min_distribution.png"))
    a = ap.parse_args()

    Q, s = sample(a.n, a.seed)
    pcts = [0, 1, 5, 25, 50, 75, 100]
    vals = np.percentile(s, pcts)

    print(f"=== xArm7 sigma_min — random poses (n={a.n}, seed={a.seed}) ===")
    print(f"{'percentile':>12} {'sigma_min':>10}")
    for p, v in zip(pcts, vals):
        print(f"{p:>11}% {v:>10.4f}")
    print(f"\n threshold sigma_th = {DEFAULT_MIN_SIGMA:.3f}"
          f"  -> rejects {100.0*np.mean(s < DEFAULT_MIN_SIGMA):.1f}% of random poses")

    print("\n=== reference poses ===")
    for name, q in REF_POSES.items():
        sg = kin.sigma_min(q)
        print(f"  {name:<24} sigma_min={sg:.4f}   "
              f"w={kin.manipulability(q):.3e}  cond={kin.condition_number(q):.1f}"
              f"   {'REJECT' if sg < DEFAULT_MIN_SIGMA else 'accept'}")

    # ── 그림 (KSMTE: 라벨 전부 영문, 9pt 이하) ──────────────────────────
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 8,
                         "axes.labelsize": 8, "xtick.labelsize": 7.5,
                         "ytick.labelsize": 7.5, "legend.fontsize": 7.5})

    fig, ax = plt.subplots(figsize=(3.35, 2.25), dpi=300)   # 85mm = 1단 폭
    ax.hist(s, bins=45, color="0.72", edgecolor="0.35", linewidth=0.4)
    ax.axvline(DEFAULT_MIN_SIGMA, color="k", linestyle="--", linewidth=1.0,
               label=f"threshold $\\sigma_{{th}}$ = {DEFAULT_MIN_SIGMA:.2f}")
    for name, mark in (("Home (real)", "o"), ("Wrist aligned (q5=0)", "^"),
                       ("q4 = 140 deg", "s")):
        ax.plot(kin.sigma_min(REF_POSES[name]), 0, mark, color="k",
                markersize=4, clip_on=False, label=name)
    ax.set_xlabel("$\\sigma_{min}$ of scaled Jacobian")
    ax.set_ylabel("Number of poses")
    ax.legend(frameon=False, loc="upper right")
    ax.spines[["top", "right"]].set_visible(False)
    fig.tight_layout(pad=0.3)
    out = os.path.abspath(a.out)
    os.makedirs(os.path.dirname(out), exist_ok=True)
    fig.savefig(out, bbox_inches="tight")
    print(f"\n[fig] {out}")


if __name__ == "__main__":
    main()
