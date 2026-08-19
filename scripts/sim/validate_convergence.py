"""validate_convergence.py — 런타임 수렴 지표를 GT 로 검증하고 임계값을 고른다.

런타임에는 GT 를 못 쓰므로 판정은 GT-free 신호(누적 점군의 **신규 점유 복셀**)로 한다.
GT 는 그 신호가 맞는지 **검증·보정**하는 데만 쓴다 — sim 을 쓰는 이유가 이것이다.

각 (eps, N) 조합에 대해:
    정지단계   = 신규복셀 < eps 가 N 회 연속된 첫 지점
    손실       = 최고 completeness − 정지시점 completeness   (덜 얻고 멈춘 대가)
    절약       = 건너뛴 패치 수                                (아낀 비용)
최대손실이 작으면서 절약이 큰 조합을 고른다.

    python scripts/sim/validate_convergence.py --dirs scripts/sim/log/conv_val/*
"""
from __future__ import annotations
import argparse, glob, json, os, re
import numpy as np
import open3d as o3d
from scipy.spatial import cKDTree

GT_DIR = "scripts/sim/log/gt"


def novelty(dirpath: str, vox: float):
    """누적 점군 덤프에서 패치별 신규 점유 복셀 비율."""
    fs = sorted(glob.glob(os.path.join(dirpath, "*_accum.npz")))
    out, seen = [], None
    for f in fs:
        p = np.load(f)["points"].astype(np.float64)
        cur = set(map(tuple, np.unique(np.floor(p / vox).astype(np.int64), axis=0)))
        out.append(None if seen is None else len(cur - seen) / max(len(seen), 1))
        seen = cur
    return fs, out


def completeness(dirpath: str, gt: np.ndarray, tau: float, n: int):
    fs = sorted(glob.glob(os.path.join(dirpath, "*.ply")))
    tree = None
    out = []
    for f in fs:
        m = o3d.io.read_triangle_mesh(f)
        pts = np.asarray(m.sample_points_uniformly(n).points)
        d, _ = cKDTree(pts).query(gt, k=1, workers=-1)
        out.append(float((d < tau).mean()) * 100)
    return fs, out


def stop_at(nov, eps, N):
    flat = 0
    for i, v in enumerate(nov):
        if v is None:
            continue
        flat = flat + 1 if v < eps else 0
        if flat >= N:
            return i
    return len(nov) - 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dirs", nargs="+", required=True)
    ap.add_argument("--vox", type=float, default=0.004)
    ap.add_argument("--tau", type=float, default=0.001)
    ap.add_argument("--n", type=int, default=150_000)
    args = ap.parse_args()

    data = {}
    for d in args.dirs:
        name = os.path.basename(d.rstrip("/"))
        gtf = os.path.join(GT_DIR, f"{name}.npz")
        if not os.path.isfile(gtf):
            print(f"  ⚠ GT 없음, 건너뜀: {name}"); continue
        gt = np.load(gtf)["points"].astype(np.float64)
        _, nov = novelty(d, args.vox)
        _, C = completeness(d, gt, args.tau, args.n)
        if len(nov) != len(C):
            print(f"  ⚠ {name}: accum {len(nov)}개 vs mesh {len(C)}개 — 건너뜀"); continue
        data[name] = (nov, C)
        print(f"\n=== {name} ===")
        print(f"{'단계':>4}{'compl%':>9}{'Δcompl':>8}{'신규복셀%':>11}")
        for i in range(len(C)):
            nv = "—" if nov[i] is None else f"{nov[i]*100:.2f}"
            dc = "—" if i == 0 else f"{C[i]-C[i-1]:+.2f}"
            print(f"{i:>4}{C[i]:>9.1f}{dc:>8}{nv:>11}")

    if not data:
        return
    print(f"\n{'eps%':>6}{'N':>3} | " +
          " | ".join(f"{k[:12]:>12}" for k in data) + " | 최대손실  총절약")
    best = None
    for eps in (0.005, 0.01, 0.02, 0.03, 0.05, 0.08):
        for N in (1, 2, 3):
            cells, worst, saved = [], 0.0, 0
            for nov, C in data.values():
                s = stop_at(nov, eps, N)
                loss = max(C) - C[s]; sv = len(C) - 1 - s
                cells.append(f"{s}단계 -{loss:.2f}"); worst = max(worst, loss); saved += sv
            print(f"{eps*100:>6.1f}{N:>3} | " + " | ".join(f"{c:>12}" for c in cells)
                  + f" | {worst:>7.2f}  {saved:>5}")
            if best is None or (worst, -saved) < best[0]:
                best = ((worst, -saved), eps, N)
    print(f"\n권장(최대손실 최소 → 절약 최대): eps={best[1]*100:.1f}%  N={best[2]}")
    print("⚠ 물체 수가 적으면 과적합이다. 대상을 늘려 다시 돌릴 것.")


if __name__ == "__main__":
    main()
