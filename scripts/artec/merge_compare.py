"""
4-variant 병합 비교 — 같은 raw scan 데이터에 네 가지 후처리 파이프라인을
적용해 결과 OBJ 를 비교.

Variants:
  noHint        — hint 적용 안 함, GlobalReg(GEOMETRY)
  textureBased  — hint 적용 안 함, GlobalReg(GEOMETRY_AND_TEXTURE) — SDK 텍스처 기반
  hintRefine    — hint 를 initial estimate 로 적용 후 GlobalReg(GEOMETRY) refine
  hintIcpRefine — hint 를 init 으로 Open3D colored ICP 로 *측정* 후 적용 → GlobalReg(GEOMETRY)
                  사용자 손회전 ±10° 오차 흡수 + 캔처럼 회전대칭 객체의 앞-뒤
                  ambiguity 를 init 으로 깸 (docs face-merging 회피)

사용:
  # scan + save raw + 4 variant 후처리 (기본)
  python scripts/artec/merge_compare.py

  # 저장된 raw 로 4 variant 만 (scan 스킵, 하드웨어 불필요)
  python scripts/artec/merge_compare.py --load output/scan_raw/<TS>

  # scan + raw 저장만 (variants 스킵)
  python scripts/artec/merge_compare.py --save-only

  # scan + variants 하되 raw 는 저장 안 함
  python scripts/artec/merge_compare.py --no-save

raw 저장 형식 (output/scan_raw/<TS>/):
  master.sproj   — IScan 들 (apply_hints=False 로 raw frame_transformations 보존)
  meta.npz       — recorded_hints + T_BC + T_CB + n_scans

raw 저장만 단독으로 하려면 scripts/artec/save_raw_scan.py 도 동일 결과.
"""

from __future__ import annotations

import argparse
import sys
import traceback
from datetime import datetime
from pathlib import Path

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT_ROOT))

import main_artec
from main_artec import (
    CFG,
    MULTIPASS_SETTINGS,
    ROBOT_IP,
    connect_turntable,
)
from mms_artec.nbv.artec_multipass_scan_session import ArtecMultiPassScanSession
from mms_artec.sensor import artec_algorithm, artec_base
from mms_artec.sensor.artec_algorithm import (
    GlobalRegistrationSettingsDTO,
    GlobalRegistrationType,
    ScannerType,
)
from mms_artec.sensor.artec_client import ArtecClient
from mms_artec.system import ArtecMMS
from utils.robot.xarm_interface import XArmInterface


# (name, hint_mode, global_reg_type)
# hint_mode: "none"  → hint 적용 0
#            "raw"   → recorded_hints 그대로 적용
#            "icp"   → recorded_hints 를 init 으로 colored ICP refine 후 적용
VARIANTS = [
    ("noHint",        "none", GlobalRegistrationType.GEOMETRY),
    ("textureBased",  "none", GlobalRegistrationType.GEOMETRY_AND_TEXTURE),
    ("hintRefine",    "raw",  GlobalRegistrationType.GEOMETRY),
    ("hintIcpRefine", "icp",  GlobalRegistrationType.GEOMETRY),
]


def _backup_frame_transformations(model) -> dict:
    """master IModel 의 각 IScan 의 모든 frame_transformation 백업.
    Returns: {scan_idx: [T_0, T_1, ...]} (numpy 4x4 list)."""
    backup = {}
    for s in range(model.scan_count()):
        scan = model.get_scan(s)
        n = scan.frame_count()
        backup[s] = [
            np.asarray(scan.get_frame_transformation(i), float).copy()
            for i in range(n)
        ]
    return backup


def _restore_frame_transformations(model, backup: dict) -> None:
    """백업된 frame_transformations 로 모든 IScan 복원."""
    for s in range(model.scan_count()):
        scan = model.get_scan(s)
        ts = backup.get(s, [])
        for i, T in enumerate(ts):
            scan.set_frame_transformation(i, T)


def save_raw(
    out_dir: Path, model, recorded_hints: list,
    T_BC: np.ndarray | None, T_CB: np.ndarray | None,
) -> tuple:
    """raw scan 결과를 master.sproj + meta.npz 로 저장."""
    out_dir.mkdir(parents=True, exist_ok=True)
    sproj_path = out_dir / "master.sproj"
    sproj_path.unlink(missing_ok=True)
    ArtecClient.save_project(model, str(sproj_path))
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
        T_BC=(np.asarray(T_BC, float) if T_BC is not None else np.eye(4)),
        T_CB=(np.asarray(T_CB, float) if T_CB is not None else np.eye(4)),
        n_scans=np.int32(model.scan_count()),
        has_T_BC=np.bool_(T_BC is not None),
    )
    return sproj_path, meta_path


def load_raw(load_dir: Path) -> tuple:
    """저장된 master.sproj + meta.npz → (model, recorded_hints, T_BC, T_CB).

    Note: ArtecClient.load_project 는 LoadedProjectEntry 리스트 반환 — 각
    entry.scan 으로 IScan 접근. 새 model 만들어서 그 scan 들 add.
    """
    sproj_path = load_dir / "master.sproj"
    meta_path = load_dir / "meta.npz"
    if not sproj_path.exists():
        raise FileNotFoundError(f"master.sproj 없음: {sproj_path}")
    if not meta_path.exists():
        raise FileNotFoundError(f"meta.npz 없음: {meta_path}")

    print(f"[load_raw] sproj 로드: {sproj_path}")
    entries = ArtecClient.load_project(str(sproj_path))
    model = artec_base.create_model()
    for e in entries:
        scan = getattr(e, "scan", None)
        if scan is not None and scan.frame_count() > 0:
            model.add_scan(scan)
    print(f"[load_raw] master scan_count={model.scan_count()}")

    print(f"[load_raw] meta 로드: {meta_path}")
    meta = np.load(meta_path, allow_pickle=False)
    scan_indices = meta["scan_indices"]
    T_pres = meta["T_pres"]
    recorded_hints = [(int(scan_indices[i]), T_pres[i])
                      for i in range(len(scan_indices))]
    has_T_BC = bool(meta["has_T_BC"])
    T_BC = meta["T_BC"] if has_T_BC else None
    T_CB = meta["T_CB"] if has_T_BC else None
    print(f"[load_raw] recorded_hints={len(recorded_hints)} "
          f"has_T_BC={has_T_BC}")
    return model, recorded_hints, T_BC, T_CB


def _apply_recorded_hints(model, recorded_hints: list) -> int:
    """recorded_hints = [(scan_idx, T_pre), ...] 를 model 의 해당 IScan 에
    좌측 곱. Variant C(hintRefine) 의 'hint as initial estimate' 단계."""
    applied = 0
    for scan_idx, T_pre in recorded_hints:
        if scan_idx < 0 or scan_idx >= model.scan_count():
            continue
        scan = model.get_scan(scan_idx)
        for i in range(scan.frame_count()):
            T_old = scan.get_frame_transformation(i)
            scan.set_frame_transformation(i, T_pre @ T_old)
        applied += 1
    return applied


def _apply_icp_refined_hints(
    raw_master_model, recorded_hints: list, T_BC, T_CB, icp_kwargs: dict,
) -> int:
    """Variant D(hintIcpRefine): raw master 의 IScan 들을 순서대로 처리하며,
    각 IScan i 의 hint T_pre 를 init 으로 colored ICP 돌려 측정된 T 로 교체
    후 그 IScan 의 frame_transformations 에 박음. master 는 순차적으로 자라
    ICP 의 target 이 점점 풍부.

    raw_master_model 자체를 mutate (frame_transformations 박음).
    동시에 'progressive master' 를 별도 ModelHandle 로 빌드해 ICP target 으로
    사용.

    Returns: ICP refine 이 적용된 scan 수.
    """
    hint_dict = dict(recorded_hints)
    applied = 0
    progressive_master = artec_base.create_model()
    for src_idx in range(raw_master_model.scan_count()):
        scan = raw_master_model.get_scan(src_idx)
        T_pre = hint_dict.get(src_idx)
        if T_pre is None or progressive_master.scan_count() == 0:
            # 첫 scan 또는 hint 없음 → raw 그대로 progressive master 에 추가
            progressive_master.add_scan(scan)
            continue
        # 단일 IScan 으로 sub_model 만들어 ICP refine
        single = artec_base.create_model()
        single.add_scan(scan)
        print(f"  [icp_refine] scan_idx={src_idx} ICP target 점 수 "
              f"≈{progressive_master.scan_count()} scan(s)")
        T_refined, fit, rmse = ArtecMultiPassScanSession.hint_icp_refine_static(
            single, T_pre, progressive_master, T_BC=T_BC, T_CB=T_CB,
            **icp_kwargs,
        )
        # 측정된 T 를 IScan frame 에 박음
        for i in range(scan.frame_count()):
            T_old = scan.get_frame_transformation(i)
            scan.set_frame_transformation(i, T_refined @ T_old)
        progressive_master.add_scan(scan)
        applied += 1
    return applied


def _run_variant(name: str, source_model, recorded_hints: list,
                 hint_mode: str, global_reg_type: GlobalRegistrationType,
                 T_BC, T_CB, icp_kwargs: dict,
                 out_dir: Path, ts: str) -> Path | None:
    """한 variant 의 후처리 파이프라인 → OBJ/sproj 저장."""
    print(f"\n══════════════════════ Variant: {name} ══════════════════════")
    print(f"  hint_mode={hint_mode}  GlobalReg={global_reg_type.name}")

    # 1. (옵션) hint 적용 — 모드별
    if hint_mode == "raw":
        n = _apply_recorded_hints(source_model, recorded_hints)
        print(f"  hint(raw) 적용: {n}/{len(recorded_hints)} scan(s)")
    elif hint_mode == "icp":
        n = _apply_icp_refined_hints(
            source_model, recorded_hints, T_BC, T_CB, icp_kwargs)
        print(f"  hint(icp refined) 적용: {n}/{len(recorded_hints)} scan(s)")
    else:
        print(f"  hint 적용 0 (mode='none')")

    # 2. GlobalRegistration
    gr_settings = GlobalRegistrationSettingsDTO.default(ScannerType.UNKNOWN)
    gr_settings.registration_type = int(global_reg_type)
    print(f"  GlobalRegistration ({global_reg_type.name}) ...")
    try:
        model = ArtecClient.global_registration(source_model, gr_settings)
        print(f"    ok  scan_count={model.scan_count()}")
    except RuntimeError as e:
        print(f"    ⚠ GlobalReg 실패: {e} — source 그대로 진행")
        model = source_model

    # 3. SmallObjectsFilter (Outliers 는 dev_mode 라 skip 정책 따름)
    try:
        print(f"  SmallObjectsFilter ...")
        model = ArtecClient.small_objects_filter(model)
        print(f"    ok")
    except RuntimeError as e:
        print(f"    ⚠ SmallObjectsFilter 실패: {e} — skip")

    # 4. PoissonFusion
    try:
        print(f"  PoissonFusion ...")
        model = ArtecClient.poisson_fusion(model)
        print(f"    ok  composite={model.has_final_mesh()}")
    except RuntimeError as e:
        print(f"    ✘ PoissonFusion 실패: {e}")
        return None

    # 5. Texturize
    try:
        print(f"  Texturize ...")
        model = ArtecClient.texturize(model)
        print(f"    ok")
    except RuntimeError as e:
        print(f"    ⚠ Texturize 실패: {e} — 그대로 export")

    # 6. Save OBJ + sproj
    obj_path = out_dir / f"{name}_{ts}.obj"
    sproj_path = out_dir / f"{name}_{ts}.sproj"
    try:
        model.save_obj(str(obj_path))
        print(f"  saved OBJ → {obj_path}")
    except Exception as e:
        print(f"  ✘ OBJ 저장 실패: {e}")
        obj_path = None
    try:
        sproj_path.unlink(missing_ok=True)
        ArtecClient.save_project(model, str(sproj_path))
        print(f"  saved sproj → {sproj_path}")
    except Exception as e:
        print(f"  sproj 저장 실패: {e}")

    return obj_path


def _run_variants(model, recorded_hints, T_BC, T_CB,
                  out_dir: Path, ts: str) -> dict:
    """4 variant 후처리 루프 — 같은 raw model 위에 매 variant 전 복원."""
    out_dir.mkdir(parents=True, exist_ok=True)
    backup = _backup_frame_transformations(model)
    total_frames = sum(len(v) for v in backup.values())
    print(f"[merge_compare] raw frame_transformations 백업 — "
          f"{total_frames} frames across {len(backup)} scans")

    icp_kwargs = dict(
        voxel_mm=MULTIPASS_SETTINGS.icp_voxel_mm,
        max_iter=MULTIPASS_SETTINGS.icp_max_iter,
        color_weight=MULTIPASS_SETTINGS.icp_color_weight,
        corr_dist_mm=MULTIPASS_SETTINGS.icp_corr_dist_mm,
    )
    if T_BC is None or T_CB is None:
        print(f"[merge_compare] ⚠ T_BC/T_CB 없음 — hintIcpRefine 결과 부정확")

    obj_paths = {}
    for name, hint_mode, gr_type in VARIANTS:
        _restore_frame_transformations(model, backup)
        try:
            obj_paths[name] = _run_variant(
                name, model, recorded_hints, hint_mode, gr_type,
                T_BC, T_CB, icp_kwargs, out_dir, ts,
            )
        except Exception as e:
            print(f"  ✘ variant '{name}' 예외: {type(e).__name__}: {e}")
            traceback.print_exc()
            obj_paths[name] = None

    print("\n══════════════════════ 요약 ══════════════════════")
    for name, _, _ in VARIANTS:
        p = obj_paths.get(name)
        if p and Path(p).exists():
            sz_kb = Path(p).stat().st_size / 1024
            print(f"  {name:<15} → {p}  ({sz_kb:.0f} KB)")
        else:
            print(f"  {name:<15} → (실패)")
    print(f"\n  비교: CloudCompare/MeshLab 으로 네 OBJ 동시 로드")
    print(f"        또는 Artec Studio 에서 sproj 열기")
    return obj_paths


def main_scan(save_raw_dir: Path | None, do_variants: bool,
              out_dir: Path, ts: str) -> None:
    """scan + (옵션) raw 저장 + (옵션) variants 후처리."""
    robot = XArmInterface(ROBOT_IP)
    turntable = connect_turntable()
    try:
        with ArtecMMS(CFG) as mms:
            robot.go_home(sensor="artec", speed=10, confirm=True)

            # raw 보존 모드 — hint 는 record 만
            MULTIPASS_SETTINGS.apply_hints_to_frame_transformations = False

            print("\n[merge_compare] === scan + (save+) variants ===")
            print(f"  variants run : {do_variants}")
            print(f"  raw save dir : {save_raw_dir or '(skip)'}")
            print(f"  apply_hints_to_frame_transformations = False (raw 보존)")

            session = ArtecMultiPassScanSession(
                mms, robot, turntable, MULTIPASS_SETTINGS,
            )
            scan_result = session.run()
            n_scans = scan_result.model.scan_count()
            print(f"\n[merge_compare] scan 완료 — passes={scan_result.n_passes} "
                  f"master scan_count={n_scans} "
                  f"recorded_hints={len(scan_result.recorded_hints)}")
            for scan_idx, T_pre in scan_result.recorded_hints:
                t = T_pre[:3, 3]
                print(f"  recorded_hint scan_idx={scan_idx} "
                      f"t=({t[0]:+.1f},{t[1]:+.1f},{t[2]:+.1f})mm")
            if n_scans == 0:
                print("[merge_compare] ✘ master 비어있음 — abort")
                return

            T_BC = getattr(session, "_T_BC", None)
            T_CB = getattr(session, "_T_CB", None)

            # ── raw 저장 ─────────────────────────────────────────
            if save_raw_dir is not None:
                try:
                    sp, mp = save_raw(
                        save_raw_dir, scan_result.model,
                        scan_result.recorded_hints, T_BC, T_CB,
                    )
                    print(f"[merge_compare] saved raw → {save_raw_dir}")
                    print(f"  sproj : {sp}")
                    print(f"  meta  : {mp}")
                    print(f"\n  ⓘ 후처리 재실행:")
                    print(f"    python scripts/artec/merge_compare.py "
                          f"--load {save_raw_dir}")
                except Exception as e:
                    print(f"[merge_compare] ✘ raw 저장 실패: {e}")

            # ── variants 실행 ───────────────────────────────────
            if do_variants:
                _run_variants(
                    scan_result.model, scan_result.recorded_hints,
                    T_BC, T_CB, out_dir, ts,
                )
    finally:
        try:
            turntable.stop()
            print("[merge_compare] turntable.stop() OK")
        except Exception as e:
            print(f"[merge_compare] ⚠ turntable.stop() 예외: {e}")
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


def main_load(load_dir: Path, out_dir: Path, ts: str) -> None:
    """저장된 raw scan 로드 → variants 후처리만. 하드웨어 불필요."""
    print(f"\n[merge_compare] === load + variants (no scan) ===")
    print(f"  load dir   : {load_dir}")
    model, recorded_hints, T_BC, T_CB = load_raw(load_dir)
    if model.scan_count() == 0:
        print("[merge_compare] ✘ 로드된 master 비어있음 — abort")
        return
    _run_variants(model, recorded_hints, T_BC, T_CB, out_dir, ts)


def parse_args():
    p = argparse.ArgumentParser(
        description="4-variant 병합 비교 — scan 또는 저장된 raw 로드")
    g = p.add_mutually_exclusive_group()
    g.add_argument("--load", type=Path, metavar="DIR",
                   help="저장된 raw scan 디렉터리 (master.sproj + meta.npz). "
                        "지정 시 scan 스킵, 하드웨어 불필요.")
    g.add_argument("--save-only", action="store_true",
                   help="scan + raw 저장만 (variants 스킵).")
    p.add_argument("--no-save", action="store_true",
                   help="scan 모드일 때 raw 자동 저장 끔. (기본은 자동 저장)")
    return p.parse_args()


def main() -> None:
    args = parse_args()
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    out_dir = PROJECT_ROOT / "output" / "merge_compare"

    if args.load is not None:
        # --load 모드: 하드웨어 없이 후처리만
        main_load(args.load.resolve(), out_dir, ts)
        return

    # scan 모드
    if args.save_only:
        save_dir = PROJECT_ROOT / "output" / "scan_raw" / ts
        main_scan(save_raw_dir=save_dir, do_variants=False,
                  out_dir=out_dir, ts=ts)
    else:
        save_dir = (None if args.no_save
                    else PROJECT_ROOT / "output" / "scan_raw" / ts)
        main_scan(save_raw_dir=save_dir, do_variants=True,
                  out_dir=out_dir, ts=ts)


if __name__ == "__main__":
    main()
