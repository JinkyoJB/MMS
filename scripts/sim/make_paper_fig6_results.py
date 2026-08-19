"""make_paper_fig6_results.py — 논문 Fig. 6 (GT 대비 복원 결과, 대표 4종).

행: GT(원본 USD 메시) / front(복원, 정면) / bottom(복원, 아랫면 — Phase 3 반전의 성패)
1단 폭(78 mm) 배치 기준으로 캔버스를 3.1 in 으로 잡아 글자가 9 pt 를 넘지 않게 한다.
"""
import os, matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.image as mpimg

FIG = os.path.join(os.path.dirname(__file__), "..", "..", "..", "8_paper",
                   "2_논문작성", "figures")
RES, GT = os.path.join(FIG, "results"), os.path.join(FIG, "gt")
OBJS = [("mustard", "mustard"), ("laundry_detergent", "detergent"),
        ("alarm_clock", "alarm clock"), ("mug", "mug")]
ROWS = [("GT", None), ("front", "정면"), ("bottom", "아랫면")]
FS = 8

plt.rcParams.update({"font.family": "DejaVu Sans"})
fig, axs = plt.subplots(3, 4, figsize=(3.1, 3.1*0.80), dpi=300)
for j, (key, label) in enumerate(OBJS):
    for i, (rname, kview) in enumerate(ROWS):
        ax = axs[i][j]
        path = (os.path.join(GT, f"{key}_{'front' if rname=='GT' else rname}.png")
                if rname == "GT" else os.path.join(RES, f"{key}_{kview}.jpg"))
        ax.imshow(mpimg.imread(path)); ax.axis("off")
        if i == 0:
            ax.set_title(label, fontsize=FS, pad=2.5)
for i, (rname, _) in enumerate(ROWS):
    axs[i][0].text(-0.10, 0.5, rname, transform=axs[i][0].transAxes, rotation=90,
                   ha="center", va="center", fontsize=FS)
fig.subplots_adjust(left=0.045, right=0.998, top=0.925, bottom=0.005,
                    wspace=0.03, hspace=0.03)
out = os.path.abspath(os.path.join(FIG, "fig_results.png"))
fig.savefig(out, bbox_inches="tight", pad_inches=0.015)
print("[fig]", out)
