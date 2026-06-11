"""
방법 4 — Predator (OverlapPredator) 추론 러너 (reg-dl env, GPU).

export_pairs.py 의 npy 쌍을 읽어 3DMatch 사전학습 Predator 로 정합.
13cm 객체 → SCALE 배 확대(3DMatch voxel 0.025m), 평행이동 역스케일.
C++ cpp_wrappers 는 순수 Python 으로 대체됨(빌드 우회).
공통 지표(common.evaluate_pair) → results_learned.csv 에 append.

실행(reg-dl):
  reg-dl/python.exe -m scripts.artec.reg_benchmark.run_predator
"""
from __future__ import annotations

import csv
import json
import sys
import tempfile
import time
from pathlib import Path

import numpy as np
import open3d as o3d
import torch

MMS_ROOT = Path(__file__).resolve().parents[3]
PRED = MMS_ROOT / "third_party" / "OverlapPredator"
for p in (str(MMS_ROOT), str(PRED)):
    if p not in sys.path:
        sys.path.insert(0, p)

from scripts.artec.reg_benchmark.common import OUT_DIR, OVERLAY_DIR, RESULTS_DIR, evaluate_pair  # noqa: E402

PAIRS_DIR = OUT_DIR / "pairs"
CONFIG = PRED / "configs" / "test" / "indoor.yaml"
WEIGHTS = PRED / "weights" / "indoor.pth"
METHOD = "predator"


def _build_config():
    from easydict import EasyDict as edict
    from lib.utils import load_config
    cfg = edict(load_config(str(CONFIG)))
    cfg.device = torch.device("cuda")
    cfg.pretrain = str(WEIGHTS)
    cfg.src_pcd = ""
    cfg.tgt_pcd = ""
    # architecture (demo 와 동일)
    cfg.architecture = ['simple', 'resnetb']
    for _ in range(cfg.num_layers - 1):
        cfg.architecture += ['resnetb_strided', 'resnetb', 'resnetb']
    for _ in range(cfg.num_layers - 2):
        cfg.architecture += ['nearest_upsample', 'unary']
    cfg.architecture += ['nearest_upsample', 'last_unary']
    return cfg


def _load_model(cfg):
    from models.architectures import KPFCNN
    model = KPFCNN(cfg).to(cfg.device)
    state = torch.load(cfg.pretrain, map_location=cfg.device, weights_only=False)
    sd = state.get("state_dict", state) if isinstance(state, dict) else state
    model.load_state_dict(sd)
    model.eval()
    return model


class _PairSet(torch.utils.data.Dataset):
    """ThreeDMatchDemo 형식 — src/tgt npy(스케일됨) 한 쌍."""
    def __init__(self, src, tgt, config=None):
        self.src = src.astype(np.float32)
        self.tgt = tgt.astype(np.float32)
        self.config = config              # get_dataloader 가 dataset.config 참조

    def __len__(self):
        return 1

    def __getitem__(self, _):
        sf = np.ones((len(self.src), 1), np.float32)
        tf = np.ones((len(self.tgt), 1), np.float32)
        rot = np.eye(3, dtype=np.float32)
        trans = np.ones((3, 1), np.float32)
        corr = torch.ones(1, 2).long()
        return (self.src, self.tgt, sf, tf, rot, trans, corr,
                self.src, self.tgt, torch.ones(1))


def _ransac_o3d(src_xyz, tgt_xyz, src_feat, tgt_feat, dist=0.05):
    """feature-based RANSAC (open3d 0.19 API) — Predator descriptors 사용."""
    sp = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(
        np.asarray(src_xyz, np.float64)))
    tp = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(
        np.asarray(tgt_xyz, np.float64)))
    sf = o3d.pipelines.registration.Feature(); sf.data = np.asarray(src_feat).T
    tf = o3d.pipelines.registration.Feature(); tf.data = np.asarray(tgt_feat).T
    res = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        sp, tp, sf, tf, mutual_filter=False, max_correspondence_distance=dist,
        estimation_method=o3d.pipelines.registration.
        TransformationEstimationPointToPoint(False),
        ransac_n=4,
        checkers=[o3d.pipelines.registration.
                  CorrespondenceCheckerBasedOnDistance(dist)],
        criteria=o3d.pipelines.registration.RANSACConvergenceCriteria(50000, 1000))
    return res.transformation


def _infer(cfg, model, nbhd, src, tgt):
    from datasets.dataloader import get_dataloader
    ds = _PairSet(src, tgt, config=cfg)
    loader, _ = get_dataloader(ds, batch_size=1, shuffle=False, num_workers=0,
                              neighborhood_limits=nbhd)
    inputs = next(iter(loader))
    for k, v in inputs.items():
        inputs[k] = ([it.to(cfg.device) for it in v] if isinstance(v, list)
                     else v.to(cfg.device))
    with torch.no_grad():
        feats, s_ov, s_sal = model(inputs)
    pcd = inputs['points'][0]
    len_src = inputs['stack_lengths'][0][0]
    src_pcd, tgt_pcd = pcd[:len_src], pcd[len_src:]
    sf, tf = feats[:len_src].cpu(), feats[len_src:].cpu()
    so = (s_ov[:len_src] * s_sal[:len_src]).cpu()
    to = (s_ov[len_src:] * s_sal[len_src:]).cpu()
    n = cfg.n_points
    if src_pcd.size(0) > n:
        idx = np.random.choice(src_pcd.size(0), n, replace=False,
                               p=(so / so.sum()).numpy().flatten())
        src_pcd, sf = src_pcd[idx], sf[idx]
    if tgt_pcd.size(0) > n:
        idx = np.random.choice(tgt_pcd.size(0), n, replace=False,
                               p=(to / to.sum()).numpy().flatten())
        tgt_pcd, tf = tgt_pcd[idx], tf[idx]
    tsfm = _ransac_o3d(src_pcd.cpu().numpy(), tgt_pcd.cpu().numpy(),
                       sf.numpy(), tf.numpy())
    return np.asarray(tsfm, np.float64)


def main():
    from datasets.dataloader import calibrate_neighbors, collate_fn_descriptor
    cfg = _build_config()
    model = _load_model(cfg)

    pair_dirs = sorted([d for d in PAIRS_DIR.iterdir() if d.is_dir()])
    # neighborhood_limits 캘리브: 첫 쌍으로 1회.
    m0 = json.load(open(pair_dirs[0] / "meta.json"))
    s0 = np.load(pair_dirs[0] / "src.npy").astype(np.float32) * m0["scale"]
    t0 = np.load(pair_dirs[0] / "ref.npy").astype(np.float32) * m0["scale"]
    nbhd = calibrate_neighbors(_PairSet(s0, t0, cfg), cfg, collate_fn_descriptor)
    print(f"[predator] neighborhood_limits={list(nbhd)}")

    rows = []
    for d in pair_dirs:
        meta = json.load(open(d / "meta.json"))
        scale = meta["scale"]
        src = np.load(d / "src.npy").astype(np.float64)
        ref = np.load(d / "ref.npy").astype(np.float64)
        T_gt = np.load(d / "gt.npy").astype(np.float64)
        t0 = time.perf_counter()
        try:
            T_s = _infer(cfg, model, nbhd, (src * scale).astype(np.float32),
                         (ref * scale).astype(np.float32))
        except Exception as e:
            print(f"  [{d.name}] FAIL {type(e).__name__}: {e}")
            import traceback; traceback.print_exc()
            continue
        rt = time.perf_counter() - t0
        T_est = T_s.copy(); T_est[:3, 3] /= scale
        src_pc = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(src))
        ref_pc = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(ref))
        ref_pc.estimate_normals()
        src_al = o3d.geometry.PointCloud(src_pc).transform(T_est)
        m = evaluate_pair(src_al, ref_pc, meta["pair"], T_est,
                          meta["expected_deg"], T_gt=T_gt)
        rows.append((meta["track"], m, rt))
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
    print(f"[predator] → {out_csv}")


if __name__ == "__main__":
    main()
