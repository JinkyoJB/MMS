"""
방법 5 — GeoTransformer 추론 러너 (reg-dl env, torch cu128 + GPU).

export_pairs.py 가 만든 npy 쌍을 읽어 3DMatch 사전학습 GeoTransformer 로 정합.
13cm 객체 → SCALE 배 키워(3DMatch voxel 0.025m 스케일) 추론, 평행이동은 역스케일.
공통 지표(common.evaluate_pair)로 평가 → results_learned.csv 에 append.

실행(reg-dl):
  reg-dl/python.exe -m scripts.artec.reg_benchmark.run_geotransformer
"""
from __future__ import annotations

import csv
import json
import sys
import time
from pathlib import Path

import numpy as np
import open3d as o3d
import torch

MMS_ROOT = Path(__file__).resolve().parents[3]
GEOT = MMS_ROOT / "third_party" / "GeoTransformer"
EXP = GEOT / "experiments" / "geotransformer.3dmatch.stage4.gse.k3.max.oacl.stage2.sinkhorn"
for p in (str(MMS_ROOT), str(GEOT), str(EXP)):
    if p not in sys.path:
        sys.path.insert(0, p)

from scripts.artec.reg_benchmark.common import OUT_DIR, OVERLAY_DIR, RESULTS_DIR, evaluate_pair  # noqa: E402

PAIRS_DIR = OUT_DIR / "pairs"
WEIGHTS = GEOT / "weights" / "geotransformer-3dmatch.pth.tar"
NEIGHBOR_LIMITS = [38, 36, 36, 38]      # 3DMatch 기본
METHOD = "geotransformer"


def _load_model():
    from config import make_cfg
    from model import create_model
    cfg = make_cfg()
    model = create_model(cfg).cuda()
    state = torch.load(str(WEIGHTS), map_location="cuda", weights_only=False)
    model.load_state_dict(state["model"])
    model.eval()
    return model, cfg


def _infer(model, cfg, src_pts, ref_pts):
    """src/ref (이미 스케일된 미터) → estimated_transform (4x4)."""
    from geotransformer.utils.data import registration_collate_fn_stack_mode
    from geotransformer.utils.torch import to_cuda, release_cuda
    data = {
        "ref_points": ref_pts.astype(np.float32),
        "src_points": src_pts.astype(np.float32),
        "ref_feats": np.ones((len(ref_pts), 1), np.float32),
        "src_feats": np.ones((len(src_pts), 1), np.float32),
        "transform": np.eye(4, dtype=np.float32),
    }
    data = registration_collate_fn_stack_mode(
        [data], cfg.backbone.num_stages, cfg.backbone.init_voxel_size,
        cfg.backbone.init_radius, NEIGHBOR_LIMITS)
    data = to_cuda(data)
    with torch.no_grad():
        out = model(data)
    out = release_cuda(out)
    return np.asarray(out["estimated_transform"], np.float64)


def main():
    model, cfg = _load_model()
    rows = []
    pair_dirs = sorted([d for d in PAIRS_DIR.iterdir() if d.is_dir()])
    for d in pair_dirs:
        meta = json.load(open(d / "meta.json"))
        scale = meta["scale"]
        src = np.load(d / "src.npy").astype(np.float64)
        ref = np.load(d / "ref.npy").astype(np.float64)
        T_gt = np.load(d / "gt.npy").astype(np.float64)

        t0 = time.perf_counter()
        try:
            T_s = _infer(model, cfg, src * scale, ref * scale)
        except Exception as e:
            print(f"  [{d.name}] FAIL {type(e).__name__}: {e}")
            import traceback; traceback.print_exc()
            continue
        rt = time.perf_counter() - t0
        T_est = T_s.copy()
        T_est[:3, 3] /= scale            # 평행이동 역스케일

        src_pc = o3d.geometry.PointCloud(
            o3d.utility.Vector3dVector(src))
        ref_pc = o3d.geometry.PointCloud(
            o3d.utility.Vector3dVector(ref))
        ref_pc.estimate_normals()
        src_al = o3d.geometry.PointCloud(src_pc).transform(T_est)
        m = evaluate_pair(src_al, ref_pc, meta["pair"], T_est,
                          meta["expected_deg"], T_gt=T_gt)
        rows.append((meta["track"], m, rt))
        # 정합 결과 PLY (빨강=src_aligned, 파랑=ref) + 변환 npy 저장
        a = o3d.geometry.PointCloud(ref_pc); a.paint_uniform_color([.2, .45, .85])
        b = o3d.geometry.PointCloud(src_al); b.paint_uniform_color([.85, .3, .25])
        tag = (f"real_{METHOD}_{meta['pair'].replace('->', '_')}"
               if meta["track"] == "real" else f"syn_{METHOD}_{meta['pair']}")
        o3d.io.write_point_cloud(str(OVERLAY_DIR / f"{tag}.ply"), a + b)
        (OVERLAY_DIR / "transforms").mkdir(parents=True, exist_ok=True)
        np.save(OVERLAY_DIR / "transforms" / f"{tag}.npy", T_est)
        print(f"  [{d.name:<24}] gtRot={m.gt_rot_err_deg:6.1f}° "
              f"gtT={m.gt_t_err_mm:6.1f}mm fit@5={m.fitness[0.005]:.3f} "
              f"cham={m.chamfer_m*1000:.2f}mm {rt:.2f}s")

    out_csv = RESULTS_DIR / "results_learned.csv"
    new = not out_csv.exists()
    with open(out_csv, "a", newline="") as f:
        w = csv.writer(f)
        if new:
            w.writerow(["track", "method", "pair", "fit@5mm", "rmse@5mm_mm",
                        "chamfer_mm", "rot_deg", "gt_rot_err_deg",
                        "gt_t_err_mm", "runtime_s"])
        for track, m, rt in rows:
            w.writerow([track, METHOD, m.pair, f"{m.fitness[0.005]:.4f}",
                        f"{m.rmse[0.005]*1000:.3f}", f"{m.chamfer_m*1000:.3f}",
                        f"{m.rot_deg:.2f}", f"{m.gt_rot_err_deg:.2f}",
                        f"{m.gt_t_err_mm:.2f}", f"{rt:.2f}"])
    print(f"[geotransformer] → {out_csv}")


if __name__ == "__main__":
    main()
