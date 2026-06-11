"""
벤치마크 정합 쌍(real + synthetic)을 npy 로 내보낸다 — 학습기반 러너(reg-dl)용.

각 쌍: src.npy (Ns,3), ref.npy (Nr,3) [미터, 원 스케일], gt.npy (4x4, src→ref),
meta.json (track, pair, expected_deg, scale).

학습 모델(3DMatch, voxel 0.025m)에 맞추려면 13cm 객체를 키워야 하므로 러너 쪽에서
SCALE 을 곱한다(여기선 원 스케일 저장, scale 값만 meta 에 기록).

common+synthetic 만 사용 → SDK 불필요, mms-env/reg-dl 양쪽에서 실행 가능.
"""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import open3d as o3d

from scripts.artec.reg_benchmark.common import (
    OUT_DIR, load_sessions, icp_refine, _rot_angle_deg,
)
from scripts.artec.reg_benchmark import synthetic as syn

PAIRS_DIR = OUT_DIR / "pairs"
SCALE = 10.0          # 학습 모델 입력 스케일(13cm→1.3m). 러너가 사용.


def _best_icp_gt(src, ref):
    T = np.eye(4)
    for th in (0.02, 0.01, 0.005, 0.003):
        T = icp_refine(src, ref, T, thresh_m=th)
    return T


def _save_pair(track, name, src, ref, T_gt, expected_deg):
    d = PAIRS_DIR / f"{track}_{name.replace('->', '_')}"
    d.mkdir(parents=True, exist_ok=True)
    np.save(d / "src.npy", np.asarray(src.points, np.float32))
    np.save(d / "ref.npy", np.asarray(ref.points, np.float32))
    np.save(d / "gt.npy", np.asarray(T_gt, np.float32))
    json.dump({"track": track, "pair": name,
               "expected_deg": float(expected_deg), "scale": SCALE},
              open(d / "meta.json", "w"), indent=2)
    print(f"  exported {d.name}  src={len(src.points)} ref={len(ref.points)}")


def main():
    sessions = load_sessions(use_cache=True)
    PAIRS_DIR.mkdir(parents=True, exist_ok=True)

    # real
    ref = sessions[0]
    for i in (1, 2):
        if i >= len(sessions):
            continue
        src = sessions[i]
        T_gt = _best_icp_gt(src, ref)
        _save_pair("real", f"{i}->0", src, ref, T_gt,
                   _rot_angle_deg(T_gt[:3, :3]))

    # synthetic
    for sc in syn.build_scenarios(sessions[0]):
        _save_pair("syn", sc.name, sc.src, sc.ref, sc.T_gt, sc.expected_deg)

    print(f"[export] → {PAIRS_DIR}")


if __name__ == "__main__":
    main()
