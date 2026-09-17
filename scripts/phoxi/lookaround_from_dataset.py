#!/usr/bin/env python
# scripts/lookaround_from_dataset.py
#
# 저장된 dataset 을 live PhoXi 대신 `PhoxiDatasetReplay` 로 사용해
# **ScanSession** 을 그대로 실행.
#
# lookaround 로직은 `mms/nbv/scan_session.py::_rule_based_scan` 에 있고
# 이 스크립트는 단순 wrapper 역할만 — 중복 로직 없음.
#
# 사용
# ----
#   python scripts/lookaround_from_dataset.py --dataset datasets/phoxi_20260424_141529
#   python scripts/lookaround_from_dataset.py --dataset ... --no-icp
#   python scripts/lookaround_from_dataset.py --dataset ... --poisson both

from __future__ import annotations

import argparse
import sys
from pathlib import Path

_PROJECT_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(_PROJECT_ROOT))

from mms_phoxi.system import MMS, MMSConfig
from mms_phoxi.sensor.phoxi_client import PhoxiConfig
from mms_phoxi.sensor.phoxi_dataset_replay import (
    PhoxiDatasetReplay, PhoxiDatasetConfig,
)
from mms_phoxi.nbv.scan_session import ScanSession, ScanSessionSettings


def _resolve_dataset(arg: str) -> Path:
    p = Path(arg)
    if not p.is_absolute():
        p = (_PROJECT_ROOT / p).resolve()
    if not p.exists():
        sys.exit(f"dataset 폴더 없음: {p}")
    return p


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Dataset 으로 lookaround 실행 (ScanSession offline)"
    )
    parser.add_argument("--dataset", required=True,
                        help="datasets/<session> 경로")
    parser.add_argument("--out", default=None,
                        help="출력 폴더 (기본: <dataset>/out)")

    # Volume
    parser.add_argument("--voxel", type=float, default=0.002)
    parser.add_argument("--depth-trunc", type=float, default=2.0)

    # ICP — ScanSessionSettings 기본값과 동기화 (middle-third)
    parser.add_argument("--icp-refine", dest="icp_refine", action="store_true",
                        default=True)
    parser.add_argument("--no-icp", dest="icp_refine", action="store_false")
    parser.add_argument("--icp-max-corr", type=float, default=0.007)
    parser.add_argument("--icp-rmse",     type=float, default=0.0015)
    parser.add_argument("--icp-fitness",  type=float, default=0.25)
    parser.add_argument("--icp-drift-trans", type=float, default=0.023)
    parser.add_argument("--icp-drift-rot",   type=float, default=12.0)

    # 시각화
    parser.add_argument("--show-progress", dest="show_progress",
                        action="store_true", default=True)
    parser.add_argument("--no-progress", dest="show_progress",
                        action="store_false")

    # Poisson
    parser.add_argument("--poisson",
                        choices=["open3d", "photoneo_exe", "both", "none"],
                        default="open3d")

    args = parser.parse_args()

    # ── 경로 세팅 ─────────────────────────────────────────────────────
    dataset_path = _resolve_dataset(args.dataset)
    if args.out:
        out_arg = Path(args.out)
        out_dir = out_arg.resolve() if out_arg.is_absolute() \
                  else (_PROJECT_ROOT / out_arg).resolve()
    else:
        out_dir = dataset_path / "out"
    out_dir.mkdir(parents=True, exist_ok=True)

    calib = dataset_path / "calibration"
    he = calib / "hand_eye_phoxi.yaml"
    tt = calib / "turntable_frame.yaml"
    if not (he.exists() and tt.exists()):
        sys.exit(f"calibration 파일 없음: {calib}")

    print(f"[lookaround] dataset    = {dataset_path}")
    print(f"[lookaround] output dir = {out_dir}")

    # ── MMS 구성 (dataset 의 calibration yaml 로) ──────────────────────
    cfg = MMSConfig(
        phoxi=PhoxiConfig(serial_number=None, target_interval_s=0.0),
        turntable_frame_yaml=str(tt),
        sensor_frames_yaml  =str(he),
        T_EC_key            ="T_E_C",
    )
    mms = MMS(cfg)
    # 생성된 PhoxiClient 를 replay 로 교체 — live hardware 연결 없음
    replay = PhoxiDatasetReplay(PhoxiDatasetConfig(dataset_dir=str(dataset_path)))
    mms.sensor = replay

    # ── lookaround settings — ScanSessionSettings 그대로 사용 ────────────
    settings = ScanSessionSettings(
        # voxel
        tsdf_voxel_length = args.voxel,
        tsdf_depth_trunc  = args.depth_trunc,

        # lookaround
        lookaround_enabled           = True,
        lookaround_show_progress     = args.show_progress,
        lookaround_wait_window_close = True,
        lookaround_icp_refine        = args.icp_refine,
        lookaround_poisson_backend   = args.poisson,

        # Export
        lookaround_export_pcd_path   = str(out_dir / "lookaround_merged.ply"),
        lookaround_export_mesh_path  = str(out_dir / "lookaround_mesh.ply"),

        # ICP gate
        icp_max_correspondence_m = args.icp_max_corr,
        icp_rmse_thresh_m        = args.icp_rmse,
        icp_fitness_thresh       = args.icp_fitness,
        icp_drift_trans_m        = args.icp_drift_trans,
        icp_drift_rot_deg        = args.icp_drift_rot,

        # Offline 에서는 nbv / confirm 불필요
        nbv_enabled       = False,
        confirm_each_move    = False,
    )

    # ── ScanSession 실행 (offline) ────────────────────────────────────
    with mms:                         # replay.initialize/shutdown 은 no-op
        session = ScanSession(
            mms=mms, robot=None, turntable=None,
            settings=settings,
            replay=replay,
        )
        session.run()

    n = len(session.state.T_CO_list)
    print(f"\n[lookaround] 완료 — {n} 프레임 통합  outputs in {out_dir}")


if __name__ == "__main__":
    main()
