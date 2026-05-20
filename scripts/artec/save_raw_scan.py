"""
Raw scan 1회만 돌려서 IScan 데이터와 hint 메타를 저장.

병합 알고리즘을 여러 번 비교하고 싶을 때 scan 을 매번 다시 돌리지 않도록.
이 스크립트로 한 번 캡처 → 저장하면, merge_compare.py 에 `--load <dir>` 로
넘겨서 후처리만 반복 가능.

저장 파일 (output/scan_raw/<TS>/):
  master.sproj   — IScan 들 (apply_hints=False 로 raw frame_transformations 보존)
  meta.npz       — recorded_hints (scan_indices, T_pres) + T_BC + T_CB + n_scans

사용:
  python scripts/artec/save_raw_scan.py
"""

from __future__ import annotations

import sys
import traceback
from datetime import datetime
from pathlib import Path

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT_ROOT))

import main_artec  # noqa: F401  (SDK 라이브러리 load 효과)
from main_artec import CFG, MULTIPASS_SETTINGS, ROBOT_IP, connect_turntable
from mms_artec.nbv.artec_multipass_scan_session import ArtecMultiPassScanSession
from mms_artec.sensor.artec_client import ArtecClient
from mms_artec.system import ArtecMMS
from utils.robot.xarm_interface import XArmInterface


def save_meta(
    out_dir: Path,
    recorded_hints: list,
    T_BC: np.ndarray | None,
    T_CB: np.ndarray | None,
    n_scans: int,
) -> Path:
    """recorded_hints + T_BC/T_CB → meta.npz."""
    if recorded_hints:
        scan_indices = np.asarray(
            [int(idx) for idx, _ in recorded_hints], dtype=np.int32)
        T_pres = np.stack(
            [np.asarray(T, float) for _, T in recorded_hints], axis=0)
    else:
        scan_indices = np.zeros((0,), dtype=np.int32)
        T_pres = np.zeros((0, 4, 4), dtype=np.float64)
    meta_path = out_dir / "meta.npz"
    np.savez(
        meta_path,
        scan_indices=scan_indices,
        T_pres=T_pres,
        T_BC=(np.asarray(T_BC, float) if T_BC is not None
              else np.eye(4)),
        T_CB=(np.asarray(T_CB, float) if T_CB is not None
              else np.eye(4)),
        n_scans=np.int32(n_scans),
        has_T_BC=np.bool_(T_BC is not None),
    )
    return meta_path


def main() -> None:
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    out_dir = PROJECT_ROOT / "output" / "scan_raw" / ts
    out_dir.mkdir(parents=True, exist_ok=True)

    robot = XArmInterface(ROBOT_IP)
    turntable = connect_turntable()

    try:
        with ArtecMMS(CFG) as mms:
            robot.go_home(sensor="artec", speed=10, confirm=True)

            # raw 보존 모드 — hint 는 record 만, IScan 에 박지 않음
            MULTIPASS_SETTINGS.apply_hints_to_frame_transformations = False

            print(f"\n[save_raw_scan] === raw scan 캡처 + 저장 ===")
            print(f"  output dir : {out_dir}")
            print(f"  apply_hints_to_frame_transformations = False")

            session = ArtecMultiPassScanSession(
                mms, robot, turntable, MULTIPASS_SETTINGS,
            )
            scan_result = session.run()

            n_scans = scan_result.model.scan_count()
            print(f"\n[save_raw_scan] scan 완료 — passes={scan_result.n_passes} "
                  f"master scan_count={n_scans} "
                  f"recorded_hints={len(scan_result.recorded_hints)}")
            if n_scans == 0:
                print("[save_raw_scan] ✘ master 비어있음 — abort")
                return

            # ── 저장: master.sproj ────────────────────────────────────
            sproj_path = out_dir / "master.sproj"
            sproj_path.unlink(missing_ok=True)
            try:
                ArtecClient.save_project(scan_result.model, str(sproj_path))
                print(f"[save_raw_scan] saved sproj → {sproj_path}")
            except Exception as e:
                print(f"[save_raw_scan] ✘ sproj 저장 실패: {e}")
                return

            # ── 저장: meta.npz ────────────────────────────────────────
            T_BC = getattr(session, "_T_BC", None)
            T_CB = getattr(session, "_T_CB", None)
            meta_path = save_meta(
                out_dir, scan_result.recorded_hints, T_BC, T_CB, n_scans,
            )
            print(f"[save_raw_scan] saved meta → {meta_path}")

            print(f"\n══════════════════════ 완료 ══════════════════════")
            print(f"  scans            : {n_scans}")
            print(f"  recorded_hints   : {len(scan_result.recorded_hints)}")
            print(f"  T_BC available   : {T_BC is not None}")
            print(f"  saved to         : {out_dir}")
            print(f"\n  다음: 후처리만 반복하려면")
            print(f"        python scripts/artec/merge_compare.py --load {out_dir}")

    except Exception as e:
        print(f"\n[save_raw_scan] ✘ {type(e).__name__}: {e}")
        traceback.print_exc()
    finally:
        try:
            turntable.stop()
            print("[save_raw_scan] turntable.stop() OK")
        except Exception as e:
            print(f"[save_raw_scan] ⚠ turntable.stop() 예외: {e}")
        try:
            turntable.set_servo_on(False)
        except Exception:
            pass
        try:
            turntable.disconnect()
        except Exception:
            pass
        try:
            robot.disconnect()
        except Exception:
            pass


if __name__ == "__main__":
    main()
