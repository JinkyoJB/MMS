"""eval_vs_gt.py — 재구성 메시를 GT 표면과 비교해 3D 재구성 **표준 지표**를 낸다.

    completeness(τ) = GT 점 중 재구성 표면에서 τ 이내인 비율   ("빠짐없이 얻었나")
    accuracy(τ)     = 재구성 점 중 GT 표면에서 τ 이내인 비율   ("헛것을 만들지 않았나")
    F-score(τ)      = 둘의 조화평균                            (MVS 벤치마크 표준)
    Chamfer         = 양방향 평균거리

왜 필요한가 — 런타임 대용 지표 `boundary_len` 은 **새 표면을 추가하면 반드시 올라가**
확장과 손상을 구분하지 못한다. 실측(hand_drill): 파편 3개가 1개로 합쳐지고 표면적이
12% 늘어 명백히 좋아지는 동안 boundary 는 519→800mm 로 '악화'했다.

    python scripts/sim/eval_vs_gt.py --scan <obj|ply> --gt scripts/sim/log/gt/<name>.npz
    python scripts/sim/eval_vs_gt.py --stages scripts/sim/log/stage_dump/hand_drill \
                                     --gt scripts/sim/log/gt/0002_hand_drill.npz
"""
from __future__ import annotations
import argparse, glob, json, os, re
import numpy as np
import open3d as o3d
from scipy.spatial import cKDTree

TAUS = (0.0005, 0.001, 0.002)          # 0.5 / 1 / 2 mm


def surface_points(path: str, n: int = 200_000) -> np.ndarray:
    m = o3d.io.read_triangle_mesh(path)
    if len(m.triangles) == 0:
        raise SystemExit(f"삼각형 0: {path}")
    # ★ 정점이 아니라 **면 위 균일 샘플**. 정점 밀도는 Poisson depth 에 따라 달라져
    #   accuracy 가 메시 해상도에 좌우된다.
    return np.asarray(m.sample_points_uniformly(n).points)


def evaluate(scan_pts: np.ndarray, gt_pts: np.ndarray) -> dict:
    d_s2g, _ = cKDTree(gt_pts).query(scan_pts, k=1, workers=-1)     # accuracy 용
    d_g2s, _ = cKDTree(scan_pts).query(gt_pts, k=1, workers=-1)     # completeness 용
    out = {"chamfer_mm": float((d_s2g.mean() + d_g2s.mean()) / 2 * 1000),
           "acc_rmse_mm": float(np.sqrt((d_s2g ** 2).mean()) * 1000)}
    for t in TAUS:
        a = float((d_s2g < t).mean())
        c = float((d_g2s < t).mean())
        f = 0.0 if a + c == 0 else 2 * a * c / (a + c)
        k = f"{t*1000:g}mm"
        out[f"accuracy@{k}"] = a
        out[f"completeness@{k}"] = c
        out[f"fscore@{k}"] = f
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scan"); ap.add_argument("--stages")
    ap.add_argument("--gt", required=True)
    ap.add_argument("--n", type=int, default=200_000)
    ap.add_argument("--json")
    args = ap.parse_args()

    gt = np.load(args.gt)["points"].astype(np.float64)
    files = (sorted(glob.glob(os.path.join(args.stages, "*.ply")))
             if args.stages else [args.scan])
    rows = []
    print(f"GT {len(gt):,}점 · {os.path.basename(args.gt)}")
    print(f"{'단계':<18}{'boundary':>9}{'compl@1mm':>11}{'acc@1mm':>9}"
          f"{'F@1mm':>8}{'F@2mm':>8}{'chamfer':>9}")
    for f in files:
        pts = surface_points(f, args.n)
        r = evaluate(pts, gt)
        b = re.search(r"_b(\d+)_g(\d+)", os.path.basename(f))
        r["name"] = os.path.basename(f)
        r["boundary_mm"] = int(b.group(1)) if b else None
        r["gaps"] = int(b.group(2)) if b else None
        rows.append(r)
        print(f"{os.path.basename(f)[:17]:<18}{str(r['boundary_mm'] or '-'):>9}"
              f"{r['completeness@1mm']*100:>10.1f}%{r['accuracy@1mm']*100:>8.1f}%"
              f"{r['fscore@1mm']*100:>7.1f}%{r['fscore@2mm']*100:>7.1f}%"
              f"{r['chamfer_mm']:>8.2f}mm")
    if args.json:
        json.dump(rows, open(args.json, "w"), ensure_ascii=False, indent=1)
        print("\n저장:", args.json)


if __name__ == "__main__":
    main()
