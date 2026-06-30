# -*- coding: utf-8 -*-
"""MMS 과제계획용 설명 figure 합성.
[Sim 궤적 합성] -(현실화)-> [실물 MMS] | [MMS main flow 다이어그램]
"""
import math
import numpy as np
from PIL import Image, ImageDraw, ImageFont

SIM_A = '/home/keti/Pictures/스크린샷/스크린샷 2026-06-22 13-57-18.png'  # EE~(520,301)
SIM_B = '/home/keti/Pictures/스크린샷/스크린샷 2026-06-22 13-56-14.png'  # EE~(378,323)+법선
REAL  = '/home/keti/Pictures/스크린샷/스크린샷 2026-06-22 14-20-50.png'
OUT   = '/home/keti/workspace/MMS/MMS/docs/figures/mms_real2sim_overview.png'

FREG = '/usr/share/fonts/truetype/nanum/NanumGothic.ttf'
FBLD = '/usr/share/fonts/truetype/nanum/NanumGothicBold.ttf'
def font(sz, bold=False):
    try: return ImageFont.truetype(FBLD if bold else FREG, sz)
    except Exception: return ImageFont.truetype('/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc', sz)

# ---------- colors ----------
NAVY=(28,38,66); ACC=(38,108,210); ORANGE=(232,126,24); GREEN=(33,150,90)
GREY=(120,128,140); LGREY=(208,213,222); BORDER=(176,182,194)
HL=(255,243,224); HLB=(232,126,24)

# ---------- 1. sim composite (full res, then scale) ----------
A=np.array(Image.open(SIM_A).convert('RGB')).astype(np.float32)
B=np.array(Image.open(SIM_B).convert('RGB')).astype(np.float32)
R_SPLIT=470
comp=B.copy()
# 상단(팔 영역): 현재 포즈 B는 진하게(0.6), 이전 포즈 A는 옅은 ghost(0.4). 하단(물체+법선)은 B 그대로.
comp[:R_SPLIT]=np.round(0.6*B[:R_SPLIT]+0.4*A[:R_SPLIT])
comp=Image.fromarray(comp.astype(np.uint8))

F=0.62
simW,simH=int(915*F),int(903*F)
comp=comp.resize((simW,simH),Image.LANCZOS)
def s(x,y): return (int(x*F),int(y*F))
EE_A=s(520,301); EE_B=s(378,323); NRM=s(523,600)

# ---------- canvas ----------
M=44; TITLE=88; PTOP=120; PH=simH; CAPH=70
realImg=Image.open(REAL).convert('RGB')
rF=PH/realImg.height; realW=int(realImg.width*rF)
realImg=realImg.resize((realW,PH),Image.LANCZOS)
AR1=120; AR2=92; flowW=372
x_sim=M
x_ar1=x_sim+simW
x_real=x_ar1+AR1
x_ar2=x_real+realW
x_flow=x_ar2+AR2
W=x_flow+flowW+M
H=TITLE+PTOP-TITLE+PH+CAPH+30
H=PTOP+PH+CAPH+24
cv=Image.new('RGB',(W,H),(255,255,255))
d=ImageDraw.Draw(cv,'RGBA')

def ctext(d,cx,y,txt,fnt,fill,anchor='mm'):
    d.text((cx,y),txt,font=fnt,fill=fill,anchor=anchor)
def rrect(d,box,r,fill=None,outline=None,width=1):
    d.rounded_rectangle(box,radius=r,fill=fill,outline=outline,width=width)
def arrow(d,p0,p1,color,width=6,head=16,alpha=255):
    col=color+(alpha,) if len(color)==3 else color
    d.line([p0,p1],fill=col,width=width)
    ang=math.atan2(p1[1]-p0[1],p1[0]-p0[0])
    for a in (ang+math.radians(150),ang-math.radians(150)):
        d.line([p1,(p1[0]+head*math.cos(a),p1[1]+head*math.sin(a))],fill=col,width=width)
def curved_arrow(d,p0,p1,bend,color,width=6,head=18):
    mx,my=(p0[0]+p1[0])/2,(p0[1]+p1[1])/2
    cx,cy=mx,my+bend
    pts=[]
    for t in [i/24 for i in range(25)]:
        x=(1-t)**2*p0[0]+2*(1-t)*t*cx+t*t*p1[0]
        y=(1-t)**2*p0[1]+2*(1-t)*t*cy+t*t*p1[1]
        pts.append((x,y))
    d.line(pts,fill=color,width=width,joint='curve')
    ang=math.atan2(pts[-1][1]-pts[-2][1],pts[-1][0]-pts[-2][0])
    for a in (ang+math.radians(150),ang-math.radians(150)):
        d.line([pts[-1],(pts[-1][0]+head*math.cos(a),pts[-1][1]+head*math.sin(a))],fill=color,width=width)

# ---------- title ----------
ctext(d,W//2,30,'MMS 능동 3D 스캐닝 — Real2Sim 물리속성 추정 & 능동 탐색 경로 생성',font(30,True),NAVY)
ctext(d,W//2,62,'Simulation(GT 기반 로직 개발·검증)  →  Real 현실화  →  통합 파이프라인',font(19),GREY)

# ---------- panel 1: sim ----------
cv.paste(comp,(x_sim,PTOP))
d.rectangle([x_sim,PTOP,x_sim+simW,PTOP+PH],outline=BORDER,width=2)
# EE 두 포즈 궤적 화살표 (pose1 -> pose2)
e0=(x_sim+EE_A[0],PTOP+EE_A[1]); e1=(x_sim+EE_B[0],PTOP+EE_B[1])
curved_arrow(d,e0,e1,-46,ORANGE,width=7,head=20)
for (ex,ey),lab in [(e0,'view pose 1'),(e1,'view pose 2')]:
    d.ellipse([ex-7,ey-7,ex+7,ey+7],fill=ORANGE+(255,),outline=(255,255,255),width=2)
ctext(d,(e0[0]+e1[0])//2,min(e0[1],e1[1])-30,'스캐닝 궤적 (Active NBV)',font(18,True),ORANGE)
# 법선/관측방향 콜아웃
nx,ny=x_sim+NRM[0],PTOP+NRM[1]
lx,ly=x_sim+simW-150,PTOP+PH-150
d.line([(nx,ny),(lx,ly)],fill=ACC+(255,),width=2)
d.ellipse([nx-6,ny-6,nx+6,ny+6],outline=ACC,width=3)
rrect(d,[lx-6,ly-22,lx+196,ly+18],8,fill=(255,255,255,235),outline=ACC,width=2)
ctext(d,lx+95,ly-2,'표면 법선 / 관측 방향 벡터',font(16,True),ACC)
ctext(d,x_sim+simW//2,PTOP+PH+26,'① Simulation: 능동 시점계획 + 표면 법선 추정',font(20,True),NAVY)
ctext(d,x_sim+simW//2,PTOP+PH+50,'(불확실 영역 우선 탐색 · GT로 로직 검증)',font(16),GREY)

# ---------- arrow 1: sim -> real ----------
ay=PTOP+PH//2
arrow(d,(x_ar1+16,ay),(x_real-14,ay),GREEN,width=9,head=22)
ctext(d,(x_ar1+x_real)//2,ay-30,'현실화',font(22,True),GREEN)
ctext(d,(x_ar1+x_real)//2,ay+30,'Sim → Real',font(16),GREEN)

# ---------- panel 2: real ----------
cv.paste(realImg,(x_real,PTOP))
d.rectangle([x_real,PTOP,x_real+realW,PTOP+PH],outline=BORDER,width=2)
ctext(d,x_real+realW//2,PTOP+PH+26,'② 실물 MMS 시스템',font(20,True),NAVY)
ctext(d,x_real+realW//2,PTOP+PH+50,'(xArm7 + Artec Spider + 턴테이블)',font(16),GREY)

# ---------- arrow 2 -> flow ----------
arrow(d,(x_ar2+12,ay),(x_flow-12,ay),GREY,width=7,head=18)

# ---------- panel 3: main flow diagram ----------
fx0=x_flow; fx1=x_flow+flowW
ctext(d,(fx0+fx1)//2,PTOP-2,'③ MMS Main Flow',font(20,True),NAVY)
steps=[
 ('Auto-Calibration','Hand-eye T_E_C · 턴테이블 축',False),
 ('Active View Planning','Frontier 추출 → Next-Best-View',True),
 ('Streaming Scan','Artec SLAM · 점군 실시간 누적',False),
 ('NBV 보충 루프','불확실/부족 표면 우선 재탐색',True),
 ('데이터 병합','5면 + 아랫면(180° flip)',False),
 ('후처리','Water-tight Mesh + Texture',False),
]
n=len(steps); gap=14
bx0=fx0+14; bx1=fx1-14
top=PTOP+30; avail=PH-30-46
bh=(avail-gap*(n-1))//n
fT=font(17,True); fS=font(13)
boxes=[]
for i,(t,sub,hl) in enumerate(steps):
    y0=top+i*(bh+gap); y1=y0+bh
    boxes.append((y0,y1))
    fill=HL if hl else (244,246,250)
    oc=HLB if hl else LGREY
    rrect(d,[bx0,y0,bx1,y1],10,fill=fill,outline=oc,width=3 if hl else 2)
    ctext(d,(bx0+bx1)//2,y0+bh*0.34,t,fT,NAVY if not hl else (180,90,10))
    ctext(d,(bx0+bx1)//2,y0+bh*0.70,sub,fS,GREY)
# down arrows between boxes
for i in range(n-1):
    y1=boxes[i][1]; ynext=boxes[i+1][0]
    cxm=(bx0+bx1)//2
    arrow(d,(cxm,y1+2),(cxm,ynext-2),NAVY,width=4,head=10)
# loop-back arrow (NBV 루프 idx3 -> Active View Planning idx1)
lyA=boxes[3][0]+ (boxes[3][1]-boxes[3][0])//2
lyB=boxes[1][0]+ (boxes[1][1]-boxes[1][0])//2
loopx=bx1+0  # along right edge outside
ox=bx1+0
lx2=bx1+ (flowW-(bx1-fx0))//2 - 2
d.line([(bx1,lyA),(bx1+16,lyA),(bx1+16,lyB),(bx1+2,lyB)],fill=ORANGE,width=4,joint='curve')
arrow(d,(bx1+16,lyB+0),(bx1+2,lyB),ORANGE,width=4,head=10)
# final output
fy=boxes[-1][1]+10
ctext(d,(bx0+bx1)//2,fy+14,'→ 산출물: 전면 3D 모델 (mesh+texture)',font(15,True),GREEN)

cv.save(OUT)
print('saved',OUT,cv.size)
