"""
reg_benchmark 오케스트레이터 (real + synthetic 두 트랙).

추상화: 모든 방법은 (src, ref) → T(src→ref). 두 트랙을 RegPair 리스트로 통일.
  - real      : ref=session0, src=session_i. GT = identity 에서 다단 ICP 정합
                (이 데이터는 세션이 이미 거의 정렬 → 그 ICP 결과가 best 정렬).
  - synthetic : session0 에 알려진 변환+crop. GT = 정확한 inv(applied).

평가: fitness@{3,5}mm, RMSE@5mm, chamfer, rot°, GT 대비 ΔR(gtRot)/Δt(gtT).
      synthetic 에선 gtRot/gtT 가 절대 정확도. real 에선 ICP-best 대비 편차.

사용:
  python -m scripts.artec.reg_benchmark.run_benchmark --track both
  python -m scripts.artec.reg_benchmark.run_benchmark --track synthetic \
         --methods fpfh_ransac,fgr
"""
from __future__ import annotations

import argparse
import csv
import time
import traceback
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Optional

import numpy as np
import open3d as o3d

from scripts.artec.reg_benchmark.common import (
    OUT_DIR, OVERLAY_DIR, RESULTS_DIR, load_sessions, hint_transforms, evaluate_pair, icp_refine,
    PairMetric, _rot_angle_deg,
)
from scripts.artec.reg_benchmark import methods_classic as mc
from scripts.artec.reg_benchmark import synthetic as syn
from scripts.artec.reg_benchmark import methods_artec as ma
from scripts.artec.reg_benchmark import methods_image as mi
from scripts.artec.reg_benchmark.common import DEFAULT_SPROJ

_ARTEC_GR_CACHE: dict = {}
_IMG_CACHE: dict = {}


def _m_image_match(src, ref, ctx):
    """방법 6 — real 트랙 전용 (텍스처 이미지 필요)."""
    if ctx.get("track") != "real":
        raise RuntimeError("image_match 는 real 트랙 전용 (프레임 이미지 필요)")
    if not _IMG_CACHE:
        _IMG_CACHE.update(mi.image_match_transforms(DEFAULT_SPROJ))
    i = ctx.get("pair_idx")
    T, n_inl = _IMG_CACHE.get(i, (np.eye(4), 0))
    return T, T, _IMG_CACHE.get("runtime", 0.0)


def _m_artec_gr(src, ref, ctx):
    """방법 3 — real 트랙 전용. 합성/비-real 이면 N/A(예외로 skip)."""
    if ctx.get("track") != "real":
        raise RuntimeError("artec_gr 는 real 트랙 전용 (SDK 스캔 필요)")
    if not _ARTEC_GR_CACHE:
        _ARTEC_GR_CACHE.update(ma.artec_gr_transforms(DEFAULT_SPROJ))
    i = ctx.get("pair_idx")
    T = _ARTEC_GR_CACHE.get(i)
    if T is None:
        raise RuntimeError(f"artec_gr: pair_idx={i} 변환 없음")
    return T, T, _ARTEC_GR_CACHE.get("runtime", 0.0)


@dataclass
class RegPair:
    track: str
    name: str
    src: o3d.geometry.PointCloud
    ref: o3d.geometry.PointCloud
    T_gt: Optional[np.ndarray]
    expected_deg: float


# 방법: fn(src, ref, ctx) -> (T_global, T_final, runtime). ctx 에 T_gt 등.
def _m_identity(src, ref, ctx):
    return np.eye(4), np.eye(4), 0.0


def _m_icp_identity(src, ref, ctx):
    t0 = time.perf_counter()
    Tf = icp_refine(src, ref, np.eye(4))
    return np.eye(4), Tf, time.perf_counter() - t0


def _m_gt(src, ref, ctx):
    T = ctx.get("T_gt")
    T = np.eye(4) if T is None else T
    return T, T, 0.0


def _m_meta_hint(src, ref, ctx):
    T = ctx.get("T_hint")
    T = np.eye(4) if T is None else T
    return T, T, 0.0


METHODS: dict[str, Callable] = {
    "identity":     _m_identity,
    "icp_identity": _m_icp_identity,
    "gt":           _m_gt,
    "meta_hint":    _m_meta_hint,
    "artec_gr":     _m_artec_gr,
    "image_match":  _m_image_match,
    "fpfh_ransac":  lambda s, r, c: mc.fpfh_ransac(s, r),
    "fgr":          lambda s, r, c: mc.fast_global(s, r),
}
DEFAULT_METHODS = ["identity", "icp_identity", "artec_gr", "image_match",
                   "fpfh_ransac", "fgr"]
REAL_ONLY = {"artec_gr", "image_match"}


# ─────────────────────────────────────────────────────────────────────
# 트랙별 RegPair 구성
# ─────────────────────────────────────────────────────────────────────

def _best_icp_gt(src, ref) -> np.ndarray:
    """identity 에서 다단(coarse→fine) point-to-plane ICP — real 트랙 GT 기준."""
    T = np.eye(4)
    for th in (0.02, 0.01, 0.005, 0.003):
        T = icp_refine(src, ref, T, thresh_m=th)
    return T


def build_real_pairs(sessions, hints) -> list[RegPair]:
    pairs = []
    ref = sessions[0]
    for i in (1, 2):
        if i >= len(sessions):
            continue
        src = sessions[i]
        T_gt = _best_icp_gt(src, ref)
        pairs.append(RegPair("real", f"{i}->0", src, ref, T_gt,
                             _rot_angle_deg(T_gt[:3, :3])))
    return pairs


def build_syn_pairs(sessions) -> list[RegPair]:
    pairs = []
    for sc in syn.build_scenarios(sessions[0]):
        pairs.append(RegPair("syn", sc.name, sc.src, sc.ref, sc.T_gt,
                             sc.expected_deg))
    return pairs


# ─────────────────────────────────────────────────────────────────────

def _merge_colored(ref, src_aligned, out: Path):
    a = o3d.geometry.PointCloud(ref); b = o3d.geometry.PointCloud(src_aligned)
    a.paint_uniform_color([0.20, 0.45, 0.85])
    b.paint_uniform_color([0.85, 0.30, 0.25])
    o3d.io.write_point_cloud(str(out), a + b)


def run_one(name, fn, pair: RegPair, ctx, rows, out_dir):
    try:
        T_global, T_final, rt = fn(pair.src, pair.ref, ctx)
    except Exception as e:
        print(f"  [{name} {pair.track}:{pair.name}] FAIL "
              f"{type(e).__name__}: {e}")
        traceback.print_exc()
        return
    src_aligned = o3d.geometry.PointCloud(pair.src).transform(T_final)
    m: PairMetric = evaluate_pair(src_aligned, pair.ref, pair.name, T_final,
                                  pair.expected_deg, T_gt=pair.T_gt)
    rows.append((pair.track, name, m, rt))
    tag = f"{pair.track}_{name}_{pair.name.replace('->', '_')}"
    _merge_colored(pair.ref, src_aligned, OVERLAY_DIR / f"{tag}.ply")
    print(f"  [{name:<12} {pair.track}:{pair.name:<16}] "
          f"fit@5={m.fitness[0.005]:.3f} cham={m.chamfer_m*1000:5.2f}mm "
          f"rot={m.rot_deg:6.1f}° gtRot={m.gt_rot_err_deg:6.1f}° "
          f"gtT={m.gt_t_err_mm:6.1f}mm {rt:.1f}s")


def print_table(rows):
    print("\n" + "=" * 104)
    print(f"{'track':<6}{'method':<13}{'pair':<17}{'fit@5':>7}{'rmse5':>7}"
          f"{'cham':>7}{'rot°':>7}{'gtRot°':>8}{'gtT_mm':>8}{'sec':>7}")
    print("-" * 104)
    for track, name, m, rt in rows:
        print(f"{track:<6}{name:<13}{m.pair:<17}{m.fitness[0.005]:>7.3f}"
              f"{m.rmse[0.005]*1000:>7.2f}{m.chamfer_m*1000:>7.2f}"
              f"{m.rot_deg:>7.1f}{m.gt_rot_err_deg:>8.1f}"
              f"{m.gt_t_err_mm:>8.1f}{rt:>7.1f}")
    print("=" * 104)
    print("synthetic: gtRot/gtT = 절대 정확도(작을수록 정답).  "
          "real: ICP-best 대비 편차.")


def write_csv(rows, path: Path):
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["track", "method", "pair", "fit@3mm", "fit@5mm",
                    "rmse@5mm_mm", "chamfer_mm", "rot_deg", "expected_deg",
                    "gt_rot_err_deg", "gt_t_err_mm", "runtime_s"])
        for track, name, m, rt in rows:
            w.writerow([track, name, m.pair, f"{m.fitness[0.003]:.4f}",
                        f"{m.fitness[0.005]:.4f}", f"{m.rmse[0.005]*1000:.3f}",
                        f"{m.chamfer_m*1000:.3f}", f"{m.rot_deg:.2f}",
                        f"{m.expected_deg:.1f}", f"{m.gt_rot_err_deg:.2f}",
                        f"{m.gt_t_err_mm:.2f}", f"{rt:.2f}"])
    print(f"[run] CSV → {path}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--track", choices=["real", "synthetic", "both"],
                    default="both")
    ap.add_argument("--methods", type=str, default=None)
    ap.add_argument("--no-cache", action="store_true")
    args = ap.parse_args()

    OUT_DIR.mkdir(parents=True, exist_ok=True)
    sessions = load_sessions(use_cache=not args.no_cache)
    hints = hint_transforms()

    pairs: list[RegPair] = []
    if args.track in ("real", "both"):
        pairs += build_real_pairs(sessions, hints)
    if args.track in ("synthetic", "both"):
        pairs += build_syn_pairs(sessions)
    print(f"[run] pairs: {[(p.track, p.name) for p in pairs]}")

    names = (args.methods.split(",") if args.methods else DEFAULT_METHODS)
    names = [n.strip() for n in names]
    # synthetic 트랙엔 oracle 'gt' 도 자동 포함(상단 sanity). real 엔 meta_hint.
    rows = []
    for pair in pairs:
        is_real = pair.track == "real"
        pidx = int(pair.name[0]) if pair.name[0].isdigit() else None
        ctx = {"T_gt": pair.T_gt, "track": pair.track, "pair_idx": pidx,
               "T_hint": hints.get(pidx) if is_real else None}
        extra = (["gt"] if pair.track == "syn" else
                 (["meta_hint"] if is_real else []))
        # artec_gr / image_match 는 real 트랙에서만 (SDK 스캔/이미지 필요).
        sel = [n for n in names if not (n in REAL_ONLY and not is_real)]
        print(f"\n── pair {pair.track}:{pair.name} (expected {pair.expected_deg:.0f}°) ──")
        for name in extra + sel:
            if name not in METHODS:
                print(f"  unknown method '{name}' — skip")
                continue
            run_one(name, METHODS[name], pair, ctx, rows, OUT_DIR)

    if rows:
        print_table(rows)
        write_csv(rows, RESULTS_DIR / "results.csv")
        _merge_all(rows, RESULTS_DIR / "results_all.csv")


def _merge_all(rows, path: Path):
    """(track,method,pair) 키로 results_all.csv 에 누적 병합(dedup)."""
    cols = ["track", "method", "pair", "fit@3mm", "fit@5mm", "rmse@5mm_mm",
            "chamfer_mm", "rot_deg", "expected_deg", "gt_rot_err_deg",
            "gt_t_err_mm", "runtime_s"]
    store: dict = {}
    if path.exists():
        with open(path, newline="") as f:
            for r in csv.DictReader(f):
                store[(r["track"], r["method"], r["pair"])] = r
    for track, name, m, rt in rows:
        store[(track, name, m.pair)] = {
            "track": track, "method": name, "pair": m.pair,
            "fit@3mm": f"{m.fitness[0.003]:.4f}", "fit@5mm": f"{m.fitness[0.005]:.4f}",
            "rmse@5mm_mm": f"{m.rmse[0.005]*1000:.3f}",
            "chamfer_mm": f"{m.chamfer_m*1000:.3f}", "rot_deg": f"{m.rot_deg:.2f}",
            "expected_deg": f"{m.expected_deg:.1f}",
            "gt_rot_err_deg": f"{m.gt_rot_err_deg:.2f}",
            "gt_t_err_mm": f"{m.gt_t_err_mm:.2f}", "runtime_s": f"{rt:.2f}"}
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=cols)
        w.writeheader()
        for v in store.values():
            w.writerow(v)
    print(f"[run] merged → {path}")


if __name__ == "__main__":
    main()
