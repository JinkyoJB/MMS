"""render_gt_meshes.py — testset USD 원본(GT) 메시를 스캔 렌더와 같은 스타일로 렌더.

Axes3D 없이 동작한다 — 삼각형을 직접 투영하고 깊이 정렬(painter) + 평면 셰이딩으로
2D PolyCollection 에 그린다. 스캔 결과 렌더와 같은 회색 균일 재질/조명을 흉내낸다.

산출: 8_paper/2_논문작성/figures/gt/<obj>_<view>.png   (view = front | bottom)
"""
from __future__ import annotations
import os, sys, glob
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection
from pxr import Usd, UsdGeom

TESTSET = "/home/keti/isaacsim/standalone_examples/play/MMS/testset"
OUT = os.path.join(os.path.dirname(__file__), "..", "..", "..", "8_paper",
                   "2_논문작성", "figures", "gt")
MAX_TRI = 600000          # 렌더 속도용 상한
# 물체별 Z축 방위 보정(deg) — USD 로컬 자세가 턴테이블 위 자세와 달라서
# 스캔 결과 렌더와 같은 면이 보이도록 맞춘다.
# 축 문자 + 각도(deg) 목록을 순서대로 적용한다.
ROT = {"alarm_clock": [("z", 90.0)], "protein_drink": [("y", -90.0)]}


def _rot(axis, deg):
    t = np.radians(deg); c, s = np.cos(t), np.sin(t)
    if axis == "x":
        return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
    if axis == "y":
        return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def load_mesh(path):
    st = Usd.Stage.Open(path)
    xc = UsdGeom.XformCache()
    V, F = [], []
    for prim in st.Traverse():
        if not prim.IsA(UsdGeom.Mesh):
            continue
        m = UsdGeom.Mesh(prim)
        pts = np.asarray(m.GetPointsAttr().Get(), dtype=np.float64)
        fc = np.asarray(m.GetFaceVertexCountsAttr().Get())
        fi = np.asarray(m.GetFaceVertexIndicesAttr().Get())
        M = np.asarray(xc.GetLocalToWorldTransform(prim)).T
        pts = (M[:3, :3] @ pts.T).T + M[:3, 3]
        off = len(np.vstack(V)) if V else 0
        V.append(pts)
        starts = np.concatenate([[0], np.cumsum(fc)[:-1]])
        for c, s in zip(fc, starts):
            idx = fi[s:s + c]
            for t in range(1, c - 1):
                F.append((off + idx[0], off + idx[t], off + idx[t + 1]))
    return np.vstack(V), np.asarray(F, dtype=np.int64)


def render(V, F, view, ax, rots=()):
    C = V.mean(0)
    P = V - C
    for axis, deg in rots:
        P = (_rot(axis, deg) @ P.T).T
    if view == "front":                      # +Y 에서 -Y 방향으로 본다, Z 위
        u, v, w = P[:, 0], P[:, 2], -P[:, 1]
    else:                                    # bottom: 아래에서 위로
        u, v, w = P[:, 0], -P[:, 1], -P[:, 2]
    tri = F
    if len(tri) > MAX_TRI:                   # 균등 솎기
        tri = tri[np.linspace(0, len(tri) - 1, MAX_TRI).astype(int)]
    a, b, c = P[tri[:, 0]], P[tri[:, 1]], P[tri[:, 2]]
    n = np.cross(b - a, c - a)
    ln = np.linalg.norm(n, axis=1, keepdims=True); ln[ln == 0] = 1
    n = n / ln
    L = np.array([0.35, -0.75, 0.55]); L = L / np.linalg.norm(L)
    if view == "bottom":
        L = np.array([0.35, -0.55, -0.75]); L = L / np.linalg.norm(L)
    shade = np.clip(np.abs(n @ L), 0, 1) * 0.55 + 0.42
    depth = (w[tri[:, 0]] + w[tri[:, 1]] + w[tri[:, 2]]) / 3.0
    order = np.argsort(depth)
    polys = np.stack([np.stack([u[tri[:, k]], v[tri[:, k]]], -1) for k in range(3)], 1)
    pc = PolyCollection(polys[order], facecolors=plt.cm.gray(shade[order]),
                        edgecolors="none", antialiaseds=False)
    ax.add_collection(pc)
    r = max(float(np.ptp(u)), float(np.ptp(v))) * 0.62
    cu = (float(u.min()) + float(u.max())) / 2.0   # 정점 평균은 조밀한 부위로
    cv = (float(v.min()) + float(v.max())) / 2.0   # 쏠린다 — 경계상자 중심 사용
    ax.set_xlim(cu - r, cu + r); ax.set_ylim(cv - r, cv + r)
    ax.set_aspect("equal"); ax.axis("off")
    ax.set_facecolor("#EAEAEA")


def main():
    os.makedirs(os.path.abspath(OUT), exist_ok=True)
    want = sys.argv[1:] or None
    for path in sorted(glob.glob(os.path.join(TESTSET, "*.usd"))):
        name = os.path.basename(path)[:-4]
        short = name.split("_", 1)[1] if "_" in name else name
        if want and short not in want:
            continue
        V, F = load_mesh(path)
        for view in ("front", "bottom"):
            fig, ax = plt.subplots(figsize=(2.2, 2.2), dpi=200)
            fig.patch.set_facecolor("#EAEAEA")
            render(V, F, view, ax, ROT.get(short, ()))
            fig.subplots_adjust(0, 0, 1, 1)
            out = os.path.join(os.path.abspath(OUT), f"{short}_{view}.png")
            fig.savefig(out, facecolor="#EAEAEA", pad_inches=0)
            plt.close(fig)
        print(f"  {short:<20} tris={len(F)}")


if __name__ == "__main__":
    main()
