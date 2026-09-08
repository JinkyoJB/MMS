"""make_paper_fig_algorithm.py — 논문 Fig. 4 (자세 탐색 의사코드 + 수용 판정).

3.3절의 이중 순회 탐색과 Accept 판정을 한 상자에 담은 의사코드 그림.
값은 코드와 일치시킨다: Ψ = {0, ±45, ±90, 180}° (isaac_scan_session.VIEW_ROLLS_DEG),
σ_min ≥ 0.05, self ≥ 20 mm, env ≥ 25 mm (collision_model.CollisionModel).

산출  8_paper/2_논문작성/figures/fig_algorithm.png  (78 mm 폭 기준)
"""
import os
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

OUT = os.path.join(os.path.dirname(__file__), "..", "..", "..", "8_paper",
                   "2_논문작성", "figures", "fig_algorithm.png")

LINES = [
    ("Input:", 0, True),
    ("p : surface point,  n : outward normal", 1, False),
    ("\u03a8 = {0\u00b0, \u00b145\u00b0, \u00b190\u00b0, 180\u00b0}, |\u03c8| ascending", 1, False),
    ("Q\u2080 : IK seeds", 1, False),
    ("", 0, False),
    ("for \u03c8 \u2208 \u03a8:", 0, True),
    ("for q\u2080 \u2208 Q\u2080:", 1, True),
    ("q \u2190 IK(T(p, n, \u03c8), q\u2080)", 2, False),
    ("if converged(q) \u2227 Accept(q):", 2, False),
    ("return q", 3, False),
    ("return failure", 0, True),
    ("", 0, False),
    ("Accept(q) :=  \u03c3_min(J\u0303(q)) \u2265 0.05", 0, True),
    ("\u2227 self-clearance(q) \u2265 20 mm", 3, False),
    ("\u2227 env-clearance(q)  \u2265 25 mm", 3, False),
]

FS = 7.2
fig, ax = plt.subplots(figsize=(3.07, 1.98), dpi=300)
ax.axis("off")
y = 0.925
for text, ind, bold in LINES:
    ax.text(0.065 + 0.048 * ind, y, text, family="monospace", fontsize=FS,
            fontweight="bold" if bold else "normal",
            va="top", ha="left", transform=ax.transAxes)
    y -= 0.0585
ax.add_patch(Rectangle((0.004, 0.006), 0.992, 0.988, transform=ax.transAxes,
                       fill=False, lw=0.8, edgecolor="black", clip_on=False))
fig.subplots_adjust(0.005, 0.01, 0.995, 0.99)
fig.savefig(os.path.abspath(OUT), facecolor="white")
print("saved", os.path.abspath(OUT))
