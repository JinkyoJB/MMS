"""reg_offline.py — run 산출물로 **정합만** 오프라인에서 다시 돌린다 (로봇·스캐너 불필요).

    python scripts/artec/reg_offline.py                       # 최신 run
    python scripts/artec/reg_offline.py --run 20260922_101530
    python scripts/artec/reg_offline.py --run ... --sub 13 --methods hint,img,greg

입력 (run 마다 자동으로 남는 것)
    output/artec_lookaround_<RUN>_raw/*.sproj    IScan 원본(정점·사진·uv, 페이로드는 옆 scans/).
                                                 프레임 변환에는 run 때 적용한 T_pre 가 **이미 박혀** 있다.
    output/scan_dumps/<RUN>/scanNN_<stage>_poseK.npz  pass 별 메타: 적용 T_pre_mm, stage, pose_idx,
                                                 R_phys, master_T_CB, T_BC_new, S 등.
    output/events_<RUN>.jsonl                    어느 병합이 어떤 방법(img/greg/hint)이었나.

하는 일
    sub 스캔(기본: 마지막 flip 스캔)을 npz 의 T_pre 로 **되돌려** 원래 세션 좌표로 만들고,
    master(그 이전 스캔 전부)에 대해 방법별로 정합 → 방법별 PLY + 지표(겹침 NN 거리) 출력.
    새 정합 아이디어는 `METHODS` 에 함수 하나 추가하면 같은 데이터로 비교된다.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))


def _latest_run() -> str:
    runs = sorted(p.stem.replace("events_", "") for p in (ROOT / "output").glob("events_*.jsonl"))
    if not runs:
        raise SystemExit("output/events_*.jsonl 이 없다 — run 을 먼저 돌릴 것")
    return runs[-1]


def _scan_pts_W(scan, stride: int = 4, undo_T_mm=None):
    """IScan → (pts m, colors) 세션 좌표. undo_T_mm 를 주면 그 역을 곱해 run 이전 좌표로."""
    inv = np.linalg.inv(undo_T_mm) if undo_T_mm is not None else None
    P = []
    for j in range(0, scan.frame_count(), max(1, stride)):
        f = scan.get_frame(j)
        v = np.asarray(f.vertices(), float)
        if v.size == 0:
            continue
        T = np.asarray(scan.get_frame_transformation(j), float)
        w = v @ T[:3, :3].T + T[:3, 3]
        if inv is not None:
            w = w @ inv[:3, :3].T + inv[:3, 3]
        P.append(w)
    return (np.vstack(P) / 1000.0) if P else np.zeros((0, 3))


def _nn_metric(src_m, ref_m, r=0.004):
    from scipy.spatial import cKDTree
    d, _ = cKDTree(ref_m).query(src_m)
    return float(np.median(d) * 1000), float((d < r).mean())


def m_hint(ctx):
    """run 때 적용한 T_pre 그대로 (= 현재 결과)."""
    return ctx["T_pre_run_mm"]


def m_img(ctx):
    from utils.nbv import image_match as im
    T, n, info = im.register_models(ctx["sub_model"], ctx["master_model"], log=print)
    if T is None:
        print("   img 실패:", info); return None
    T = np.asarray(T, float).copy(); T[:3, 3] *= 1000.0
    # sub_model 의 프레임 변환에는 run 의 T_pre 가 박혀 있으므로 그 위에서 구한 T 를 합성
    return T @ ctx["T_pre_run_mm"]


def m_greg(ctx):
    from utils.nbv.global_registration import register_consensus
    T, info = register_consensus(ctx["sub_pts_orig"], ctx["master_pts"], log=print)
    if T is None:
        print("   greg 실패:", info); return None
    T = np.asarray(T, float).copy(); T[:3, 3] *= 1000.0
    return T


METHODS = {"hint": m_hint, "img": m_img, "greg": m_greg}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--run", default=None)
    ap.add_argument("--sub", type=int, default=None, help="master 안 sub 스캔 인덱스 (기본: 마지막 flip)")
    ap.add_argument("--methods", default="hint,img,greg")
    ap.add_argument("--stride", type=int, default=4)
    ap.add_argument("--all", action="store_true",
                    help="모든 scan k 를 scan 0..k-1 에 대해 run 의 배치(hint) 그대로 채점만 (PLY 없음)")
    a = ap.parse_args()
    run = a.run or _latest_run()
    sproj = ROOT / "output" / f"artec_lookaround_{run}_raw" / f"artec_lookaround_{run}_raw.sproj"
    scans_dir = ROOT / "output" / "scan_dumps" / run
    # 2026-09-22 이전 레이아웃(프로젝트 전용 폴더 도입 전) 폴백
    if not sproj.exists() and (ROOT / "output" / f"artec_lookaround_{run}_raw.sproj").exists():
        sproj = ROOT / "output" / f"artec_lookaround_{run}_raw.sproj"
    if not scans_dir.exists() and (ROOT / "output" / "scans" / run).exists():
        scans_dir = ROOT / "output" / "scans" / run
    if not sproj.exists():
        raise SystemExit(f"raw sproj 없음: {sproj}  (--no-sproj 로 돌렸거나 후처리 전에 죽었다)")
    from mms_artec.sensor.artec_client import ArtecClient
    from mms_artec.sensor import artec_base
    entries = ArtecClient.load_project(str(sproj))
    scans = [e.scan for e in entries if getattr(e, "scan", None) is not None and e.scan.frame_count() > 0]
    metas = sorted(scans_dir.glob("scan*.npz"))
    print(f"run {run}: IScan {len(scans)}개, 메타 {len(metas)}개")
    by_idx = {}
    for m in metas:
        z = np.load(m, allow_pickle=True)
        by_idx[int(m.name[4:6])] = dict(path=m, T_pre_mm=np.asarray(z["T_pre_mm"], float),
                                        stage=str(z["stage"]), pose_idx=int(z["pose_idx"]))
    if a.all:
        print(f"\n{'scan':>4s} {'stage':10s} {'frames':>6s} {'NN중앙값(mm)':>12s} {'겹침(<4mm)':>10s}")
        acc = _scan_pts_W(scans[0], a.stride)
        for k in range(1, len(scans)):
            P = _scan_pts_W(scans[k], a.stride)          # run 이 배치한 그대로(T_pre 박힘)
            if len(P) < 50:
                print(f"{k:4d} {by_idx.get(k, {}).get('stage', '?'):10s} {scans[k].frame_count():6d}   (점 부족)")
                continue
            med, frac = _nn_metric(P, acc)
            print(f"{k:4d} {by_idx.get(k, {}).get('stage', '?'):10s} {scans[k].frame_count():6d} {med:12.2f} {frac*100:9.1f}%")
            acc = np.vstack([acc, P])
        return 0
    if a.sub is None:
        flips = [i for i, v in by_idx.items() if v["stage"] == "flip"]
        a.sub = flips[-1] if flips else len(scans) - 1
    meta = by_idx.get(a.sub, dict(T_pre_mm=np.eye(4), stage="?", pose_idx=0))
    print(f"sub = scan {a.sub} ({meta['stage']}, pose {meta['pose_idx']}) vs master = scan 0..{a.sub-1}")
    T_run = meta["T_pre_mm"]
    master_pts = np.vstack([_scan_pts_W(scans[i], a.stride) for i in range(a.sub)])
    sub_pts_orig = _scan_pts_W(scans[a.sub], a.stride, undo_T_mm=T_run)      # run 이전(세션) 좌표
    # image_match 용 모델 핸들: master = 0..sub-1, sub = 그 스캔 하나
    master_model = sub_model = None
    try:
        master_model = artec_base.create_model(); sub_model = artec_base.create_model()
        for i in range(a.sub):
            master_model.add_scan(scans[i])
        sub_model.add_scan(scans[a.sub])
    except Exception as e:                                       # noqa: BLE001
        print(f"  ⚠ 모델 핸들 구성 실패({e}) — img 방법은 건너뜀")
    ctx = dict(T_pre_run_mm=T_run, sub_pts_orig=sub_pts_orig, master_pts=master_pts,
               sub_model=sub_model, master_model=master_model)
    out = ROOT / "output" / "reg_offline" / run
    out.mkdir(parents=True, exist_ok=True)
    import open3d as o3d
    def save(name, pts):
        pc = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(pts))
        o3d.io.write_point_cloud(str(out / f"{name}.ply"), pc)
    save("master", master_pts)
    print(f"\n{'method':8s} {'NN중앙값(mm)':>12s} {'겹침(<4mm)':>10s}")
    for name in a.methods.split(","):
        fn = METHODS.get(name.strip())
        if fn is None:
            continue
        if name == "img" and sub_model is None:
            continue
        try:
            T = fn(ctx)
        except Exception as e:                                   # noqa: BLE001
            print(f"{name:8s} 예외 {type(e).__name__}: {e}"); continue
        if T is None:
            print(f"{name:8s} —"); continue
        aligned = (sub_pts_orig * 1000.0) @ T[:3, :3].T + T[:3, 3]
        aligned /= 1000.0
        med, frac = _nn_metric(aligned, master_pts)
        save(f"sub_{name}", aligned)
        print(f"{name:8s} {med:12.2f} {frac*100:9.1f}%")
    print(f"\nPLY → {out}  (master.ply + sub_<method>.ply 를 겹쳐 보면 된다)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
