"""make_paper_fig_cell.py — 논문 Fig. 1 (계측 셀 구성). 1단 폭(78 mm) 배치용.

단폭에서는 이름표를 그림 안에 넣으면 인쇄 시 5 pt 이하로 뭉개진다. 그래서 그림에는
**번호 콜아웃만** 두고 항목명은 캡션 범례로 뺀다. 번호는 12 pt 로 그려 78 mm 배치에서
약 9 pt 로 인쇄된다(숫자 하나라 판독에 문제 없음).
"""
import os, matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle
import matplotlib.image as mpimg

FIG = os.path.join(os.path.dirname(__file__), "..", "..", "..", "8_paper",
                   "2_논문작성", "figures")
img = mpimg.imread(os.path.join(FIG, "MMS_img.png"))
H, W = img.shape[:2]
img = img[70:H-14, 215:W-30]                       # UI 제거 + 여백 타이트 crop
h, w = img.shape[:2]

plt.rcParams.update({"font.family": "DejaVu Sans"})
fig, ax = plt.subplots(figsize=(4.0, 4.0*h/w), dpi=300)
ax.imshow(img); ax.set_xlim(0, w); ax.set_ylim(h, 0); ax.axis("off")
ax.add_patch(Rectangle((0, h-45), 70, 45, fc="white", ec="none", zorder=3))

R = 20
def call(n, cx, cy, px, py):
    """번호 원 + 지시선."""
    ax.plot([cx, px], [cy, py], lw=1.2, color="0.05", zorder=4,
            solid_capstyle="round")
    ax.add_patch(Circle((cx, cy), R, fc="white", ec="0.05", lw=1.4, zorder=5))
    ax.text(cx, cy, str(n), ha="center", va="center", fontsize=12,
            fontweight="bold", color="black", zorder=6)

call(1, 300,  35, 405,  28)     # ceiling mount
call(2, 560, 120, 480, 140)     # 7-DOF manipulator
call(3, 600, 262, 495, 278)     # 3D scanner
call(4,  55, 150,  90, 210)     # cell frame
call(5, 128, 318, 195, 330)     # tool stand
call(6, 545, 372, 440, 362)     # turntable
call(7,  60, 430, 285, 418)     # weighing unit

fig.tight_layout(pad=0.03)
out = os.path.abspath(os.path.join(FIG, "fig_cell.png"))
fig.savefig(out, bbox_inches="tight")
print("[fig]", out, " crop:", (w, h))
