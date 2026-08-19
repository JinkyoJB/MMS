"""make_paper_fig_params.py — 논문 Fig. 3 (허용 자세 집합의 네 매개변수). 1단 폭.

식 (1)의 ψ(광축 회전), δ(입사각), d(표면까지 거리), θ(턴테이블 각)를 실제 장면 위에 표시.

색으로 **자유도의 소속**을 구분한다 — 논문 3.1 의 역할 분담과 같은 구분이다.
    초록  : 스캐너(로봇)가 만드는 자유도  ψ, δ, d  (+ 표면점 p, 법선 n)
    주황  : 턴테이블이 만드는 자유도      θ
대상물이 검은색이라 검정 주석은 묻힌다. 라벨 12 pt → 78 mm 배치에서 약 9 pt.
"""
import os, math, matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Arc, FancyArrowPatch, Circle
import matplotlib.image as mpimg

FIG = os.path.join(os.path.dirname(__file__), "..", "..", "..", "8_paper",
                   "2_논문작성", "figures")
img = mpimg.imread(os.path.join(FIG, "MMS_params_src.png"))
img = img[35:520, 60:780]
h, w = img.shape[:2]

G = "#0E8A1E"      # 스캐너 측 자유도
O = "#E2660A"      # 턴테이블 자유도
FS = 12
plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": FS})
fig, ax = plt.subplots(figsize=(4.0, 4.0*h/w), dpi=300)
ax.imshow(img); ax.set_xlim(0, w); ax.set_ylim(h, 0); ax.axis("off")

CAM  = (255, 70)
PT   = (405, 272)
NRM  = (312, 256)
DISC = (414, 306)          # 1 mm(≈9 px) 오른쪽으로

def lab(x, y, s, col, fs=FS):
    ax.text(x, y, s, ha="center", va="center", fontsize=fs, color=col,
            zorder=9, fontweight="bold",
            bbox=dict(boxstyle="round,pad=0.16", fc="white", ec="none", alpha=0.92))

# ── 시선 + d ────────────────────────────────────────────────────────────
ax.add_patch(FancyArrowPatch(CAM, PT, arrowstyle="-|>", mutation_scale=11,
                             lw=1.8, color=G, zorder=6))
mid = ((CAM[0]+PT[0])/2, (CAM[1]+PT[1])/2)
lab(mid[0]+30, mid[1]-16, "$d$", G, FS+1)

# ── p, n ────────────────────────────────────────────────────────────────
ax.add_patch(Circle(PT, 5, fc="white", ec=G, lw=1.6, zorder=8))
ax.add_patch(FancyArrowPatch(PT, NRM, arrowstyle="-|>", mutation_scale=10,
                             lw=1.7, color=G, ls=(0, (3, 1.4)), zorder=6))
lab(NRM[0]-24, NRM[1]+2, "$n$", G)
lab(PT[0]+22, PT[1]+20, "$p$", G)

# ── δ  (각도는 데이터 좌표계 기준 — y축이 반전돼 있으므로 부호를 뒤집지 않는다) ──
a_cam = math.degrees(math.atan2(CAM[1]-PT[1], CAM[0]-PT[0])) % 360
a_nrm = math.degrees(math.atan2(NRM[1]-PT[1], NRM[0]-PT[0])) % 360
ax.add_patch(Arc(PT, 130, 130, angle=0, theta1=min(a_cam, a_nrm),
                 theta2=max(a_cam, a_nrm), lw=1.8, color=G, zorder=6))
am = math.radians((a_cam+a_nrm)/2)
lab(PT[0]+86*math.cos(am), PT[1]+86*math.sin(am), r"$\delta$", G, FS+1)

# ── ψ  (광축에 수직인 원 → 장축이 광축과 직교하는 타원) ──────────────
ux, uy = PT[0]-CAM[0], PT[1]-CAM[1]
L = math.hypot(ux, uy); ux, uy = ux/L, uy/L
c = (CAM[0]+ux*56, CAM[1]+uy*56)
ang = math.degrees(math.atan2(uy, ux)) + 90.0
RX_P, RY_P = 76.0, 26.0
ax.add_patch(Arc(c, RX_P, RY_P, angle=ang, theta1=0, theta2=360,
                 lw=1.9, color=G, zorder=6))
ca, sa = math.cos(math.radians(ang)), math.sin(math.radians(ang))
def _ell(t):
    x, y = RX_P/2*math.cos(math.radians(t)), RY_P/2*math.sin(math.radians(t))
    return (c[0]+x*ca-y*sa, c[1]+x*sa+y*ca)
ax.add_patch(FancyArrowPatch(_ell(200), _ell(168), arrowstyle="-|>",
                             mutation_scale=12, lw=2.0, color=G, zorder=7))
lab(_ell(0)[0]-14, _ell(0)[1]+12, r"$\psi$", G, FS+1)

# ── θ — 턴테이블 평면. 더 둥글게(높이 74→112) ──────────────────────────
RX, RY = 250.0, 112.0
ax.add_patch(Arc(DISC, RX, RY, angle=0, theta1=178, theta2=362,
                 lw=2.1, color=O, zorder=6))
t0 = math.radians(178)
p0 = (DISC[0] + RX/2*math.cos(t0), DISC[1] + RY/2*math.sin(t0))
ax.add_patch(FancyArrowPatch((p0[0]+3, p0[1]-13), (p0[0]-1, p0[1]+7),
                             arrowstyle="-|>", mutation_scale=12, lw=2.0,
                             color=O, zorder=7))
lab(DISC[0]-104, DISC[1]+52, r"$\theta$", O, FS+1)

fig.tight_layout(pad=0.04)
out = os.path.abspath(os.path.join(FIG, "fig_params.png"))
fig.savefig(out, bbox_inches="tight")
print("[fig]", out, " crop:", (w, h))
