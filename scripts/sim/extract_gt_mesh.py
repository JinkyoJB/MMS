"""extract_gt_mesh.py — 합성 씬에서 스캔 대상의 **정답(GT) 표면**을 월드 좌표로 추출.

sim 의 존재 이유가 GT 를 안다는 것인데, 지금까지 평가는 GT 없이 `boundary_len`
대용 지표로만 했다. 그 지표는 **새 표면을 추가하면 반드시 올라가** 확장과 손상을
구분하지 못한다(실측 2026-08-19 hand_drill: 파편 3개→1개로 좋아지는 동안 519→800mm).

여기서 뽑은 GT 로 completeness / accuracy / F-score 를 계산한다(3D 재구성 표준 지표).

    env -u PYTHONPATH ~/miniconda3/envs/step2usd/bin/python \
        scripts/sim/extract_gt_mesh.py --scene <v3_ts_*.usd> --out <gt.npz>
    #  --all 이면 v3_ts_*.usd 전부
"""
from __future__ import annotations
import argparse, glob, os, sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))

import os, sys; sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))
from mms_paths import asset, testset_dir

ASSET = asset("frame_xarm7_spider_turntable_v2")
PRIM = "/World/ScanTarget/TestObject"


def extract(scene: str, prim_path: str, spacing: float):
    import numpy as np
    from pxr import Usd, UsdGeom
    from utils.collision.mesh_sampling import sample_surface

    stage = Usd.Stage.Open(scene)
    root = stage.GetPrimAtPath(prim_path)
    if not root or not root.IsValid():
        raise SystemExit(f"prim 없음: {prim_path} in {scene}")
    xc = UsdGeom.XformCache()
    rng = np.random.default_rng(0)
    pred = Usd.TraverseInstanceProxies(Usd.PrimAllPrimsPredicate)

    P = []
    for d in Usd.PrimRange(root, pred):
        if not d.IsA(UsdGeom.Mesh):
            continue
        m = UsdGeom.Mesh(d)
        q = m.GetPointsAttr().Get()
        if not q or not len(q):
            continue
        # ★ 표면 샘플링. 정점만 쓰면 평평한 면의 한가운데가 비어 completeness 가
        #   실제보다 높게 나온다(빈 곳에 GT 점이 없으니 '덮였다'고 판정).
        V = sample_surface(np.asarray(q, float),
                           m.GetFaceVertexCountsAttr().Get() or [],
                           m.GetFaceVertexIndicesAttr().Get() or [],
                           spacing_m=spacing, rng=rng)
        A = np.array(xc.GetLocalToWorldTransform(d)).T     # USD 는 row-major
        P.append(V @ A[:3, :3].T + A[:3, 3])
    if not P:
        raise SystemExit(f"메시 없음: {prim_path}")
    return np.vstack(P)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scene")
    ap.add_argument("--all", action="store_true")
    ap.add_argument("--prim", default=PRIM)
    ap.add_argument("--spacing", type=float, default=0.0008, help="표면 샘플 간격 m")
    ap.add_argument("--out-dir", default="scripts/sim/log/gt")
    args = ap.parse_args()

    import numpy as np
    scenes = (sorted(glob.glob(os.path.join(ASSET, "v3_ts_*.usd")))
              if args.all else [args.scene])
    os.makedirs(args.out_dir, exist_ok=True)
    for sc in scenes:
        name = os.path.basename(sc)[len("v3_ts_"):-len(".usd")]
        pts = extract(sc, args.prim, args.spacing)
        f = os.path.join(args.out_dir, f"{name}.npz")
        np.savez_compressed(f, points=pts.astype(np.float32))
        ext = (pts.max(0) - pts.min(0)) * 1000
        print(f"  ✔ {name}: {len(pts):8,d}점  bbox={np.round(ext,1).tolist()}mm  → {f}")


if __name__ == "__main__":
    main()
