"""make_paper_fig1.py — 논문 Fig. 1 (스캔 파이프라인). 1단 폭(84mm)·라벨 전부 영문."""
import os, matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch

OUT = os.path.join(os.path.dirname(__file__), "..", "..", "..", "8_paper",
                   "2_논문작성", "figures", "fig_pipeline.png")
plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 7})
fig, ax = plt.subplots(figsize=(3.25, 2.85), dpi=300)     # 82 mm
ax.set_xlim(0, 100); ax.set_ylim(0, 100); ax.axis("off")

BX, BW, BH = 6.0, 88.0, 22.0
TOPS = (76.0, 42.0, 8.0)

def box(y, title, lines):
    ax.add_patch(FancyBboxPatch((BX, y), BW, BH,
                                boxstyle="round,pad=0.4,rounding_size=1.4",
                                fc="0.95", ec="0.25", lw=0.8))
    ax.text(BX + 4.5, y + BH - 6.0, title, ha="left", va="center",
            fontsize=7.5, fontweight="bold")
    for i, t in enumerate(lines):
        ax.text(BX + 4.5, y + BH - 12.0 - i*5.4, t, ha="left", va="center", fontsize=6.6)

def arrow(y0, y1, label):
    ax.add_patch(FancyArrowPatch((BX + 12, y0), (BX + 12, y1), arrowstyle="-|>",
                                 mutation_scale=8, lw=0.9, color="0.25"))
    ax.text(BX + 15, (y0 + y1) / 2, label, ha="left", va="center",
            fontsize=6.3, color="0.35")

box(TOPS[0], "Phase 1  first sweep",
    ["target: the whole object", "split into overlapping height bands"])
box(TOPS[1], "Phase 2  gap-driven sweeps",
    ["target: boundary of the current model", "repeat until no upward gap remains"])
box(TOPS[2], "Phase 3  bottom face",
    ["flip the object, sweep, and merge", "with the model from Phases 1-2"])
arrow(TOPS[0] - 0.5, TOPS[1] + BH + 0.5, "gaps remain")
arrow(TOPS[1] - 0.5, TOPS[2] + BH + 0.5, "downward gaps only")
ax.text(50, 2.0, "The pose of every sweep is selected by the same search (Section 3.3).",
        ha="center", va="center", fontsize=6.2, color="0.4", style="italic")
fig.tight_layout(pad=0.1)
out = os.path.abspath(OUT); os.makedirs(os.path.dirname(out), exist_ok=True)
fig.savefig(out, bbox_inches="tight")
print("[fig]", out)
