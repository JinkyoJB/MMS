"""make_paper_fig3_pipeline.py — 논문 Fig. 3 (가로 3연 파이프라인, 실제 장면).

  (a) lookaround : 스윕 궤적(300° 타원 화살표)
  (b) nbv : 복셀 모델 + 'voxel' 지시선
  (c) flip : 반전된 컵 + 아래→위 'flip' 화살표
세 장면을 가로로 나란히 배치하고 사이에 → 화살표. **2단 전체 폭(175 mm) 배치** 기준으로
캔버스를 6.9 in 으로 잡아 그림 내 글자가 인쇄 시 9 pt 를 넘지 않게 한다.
"""
import os, math, matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from matplotlib.patches import Arc, FancyArrowPatch

FIG = os.path.join(os.path.dirname(__file__), "..", "..", "..", "8_paper",
                   "2_논문작성", "figures")
SRC = [("test_home.png",                   (430, 230, 960, 720)),
       ("스크린샷 2026-08-19 12-42-05.png", (140, 150, 800, 700)),
       ("스크린샷 2026-08-19 16-22-15.png", (130, 45, 700, 545))]
FS = 9

imgs = []
for name,(x0,y0,x1,y1) in SRC:
    imgs.append(mpimg.imread(os.path.join(FIG,name))[y0:y1, x0:x1])

H = 480.0                                   # 공통 표시 높이(축 단위)
ws = [H*im.shape[1]/im.shape[0] for im in imgs]
GAP = 46.0
Wtot = sum(ws) + GAP*2

plt.rcParams.update({"font.family":"DejaVu Sans"})
fig = plt.figure(figsize=(6.9, 6.9*H/Wtot), dpi=300)
axs=[]; x=0.0
for im,w in zip(imgs,ws):
    ax=fig.add_axes([x/Wtot, 0.0, w/Wtot, 1.0])
    ax.imshow(im); ax.set_xlim(0,im.shape[1]); ax.set_ylim(im.shape[0],0)
    ax.axis("off"); axs.append(ax); x+=w+GAP

def tag(ax,s):
    ax.text(14,34,s,fontsize=FS,fontweight="bold",va="top",
            bbox=dict(boxstyle="round,pad=0.25",fc="white",ec="0.3",lw=0.5))

# (a) 스윕 궤적 — 300° 호 + 진행 화살촉
a=axs[0]; tag(a,"(a) lookaround")
cx,cy=215,345
a.add_patch(Arc((cx,cy),380,110,angle=0,theta1=120,theta2=60,
                lw=1.8,color="0.05",zorder=10))
p1=(cx+190*math.cos(math.radians(48)), cy+55*math.sin(math.radians(48)))
p2=(cx+190*math.cos(math.radians(60)), cy+55*math.sin(math.radians(60)))
a.add_patch(FancyArrowPatch(p1,p2,arrowstyle="-|>",mutation_scale=12,
                            lw=1.8,color="0.05",zorder=11))

# (b) voxel 지시선
b=axs[1]; tag(b,"(b) nbv")
b.annotate("voxel",xy=(470,250),xytext=(575,158),fontsize=FS,va="center",zorder=6,
           bbox=dict(boxstyle="round,pad=0.22",fc="white",ec="none",alpha=0.9),
           arrowprops=dict(arrowstyle="-",lw=1.1,color="black",shrinkA=1,shrinkB=2))

# (c) flip — 아래→위
c=axs[2]; tag(c,"(c) flip")
mx,my=250,310
c.add_patch(Arc((mx,my),190,200,angle=0,theta1=-75,theta2=75,
                lw=1.8,color="0.05",zorder=10))
q1=(mx+95*math.cos(math.radians(-58)), my+100*math.sin(math.radians(-58)))
q2=(mx+95*math.cos(math.radians(-75)), my+100*math.sin(math.radians(-75)))
c.add_patch(FancyArrowPatch(q1,q2,arrowstyle="-|>",mutation_scale=12,
                            lw=1.8,color="0.05",zorder=11))
c.text(mx+62,my-126,"flip",fontsize=FS,fontweight="bold",
       bbox=dict(boxstyle="round,pad=0.22",fc="white",ec="none",alpha=0.9))

# 그림 사이 → 화살표 (figure 좌표)
x=0.0
for w in ws[:-1]:
    x+=w
    fx0=(x+GAP*0.14)/Wtot; fx1=(x+GAP*0.86)/Wtot
    fig.patches.append(FancyArrowPatch((fx0,0.5),(fx1,0.5),
        transform=fig.transFigure,arrowstyle="-|>",mutation_scale=13,
        lw=1.8,color="0.15"))
    x+=GAP

out=os.path.abspath(os.path.join(FIG,"fig_pipeline.png"))
fig.savefig(out,bbox_inches="tight",pad_inches=0.02)
print("[fig]",out)
