#!/usr/bin/env python
# scripts/phoxi_dataset_capture.py
#
# PhoXi + xArm + Turntable 을 돌려 **센서가 주는 raw 데이터** 를 그대로 저장.
#
# - 각 프레임마다 PhoXi organized Range/Normal/Intensity (H,W,3/3/1) 을 디스크에 저장.
# - Robot FK (T_EB), 턴테이블 encoder θ, timestamp 도 함께 기록.
# - 캘리브레이션 YAML 경로 + PhoXi intrinsic (least-squares fit) 도 세션 meta 에 박아
#   나중에 이 데이터셋만으로 `mms.T_CO(θ, T_EB)` 를 재현 가능하도록 함.
#
# 출력 구조:
#   datasets/phoxi_YYYYMMDD_HHMMSS/
#     session_meta.json                 — n_frames, intrinsic, hand-eye, turntable_frame
#     frames/
#       frame_000/
#         range.npy        (H, W, 3) float32, mm   — PhoXi CalibratedABC_Grid 원본
#         intensity.npy    (H, W)    uint8        — PhoXi Intensity 원본 (grayscale)
#         intensity.png                            — 미리보기
#         normals.npy      (H, W, 3) float32  (Normal 컴포넌트 있을 때만)
#         meta.json        — idx, θ_target/actual, T_EB, timestamp
#       frame_001/
#       ...
#
# 사용:
#   python scripts/phoxi_dataset_capture.py                  # 24 프레임 (15°)
#   python scripts/phoxi_dataset_capture.py --step 10        # 36 프레임
#   python scripts/phoxi_dataset_capture.py --out mydataset  # datasets/mydataset

from __future__ import annotations

import argparse
import json
import shutil
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Optional

import numpy as np

# PROJECT ROOT import 설정
_PROJECT_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(_PROJECT_ROOT))

from mms_phoxi.sensor.phoxi_client import PhoxiClient, PhoxiConfig
from utils.robot.xarm_interface import XArmInterface
from utils.turntable import Turntable


# ─── 실험 설정 (main.py 와 일관) ──────────────────────────────────────────
ROBOT_IP             = "192.168.1.210"
TURNTABLE_IP         = "192.168.0.10"
TURNTABLE_BD_ID      = 0
PHOXI_SERIAL         = "SEA-023"

SCAN_INIT_JOINTS_DEG = [-0.5, -43.5, 0.2, 46.2, -0.2, 69.2, 1.0]
ROBOT_SPEED_DEG_S    = 15.0
TURNTABLE_VEL_RAD_S  = float(np.radians(10.0))
DWELL_S              = 0.40

HAND_EYE_YAML   = _PROJECT_ROOT / "config/calibration/hand_eye_phoxi.yaml"
TURNTABLE_YAML  = _PROJECT_ROOT / "config/calibration/turntable_frame.yaml"
DEFAULT_OUT_ROOT = _PROJECT_ROOT / "datasets"


# ─── I/O 헬퍼 ───────────────────────────────────────────────────────────

def _save_intensity_png(path: Path, intensity: np.ndarray) -> None:
    """PIL 없이 Open3D 로 PNG 저장 (의존성 최소)."""
    try:
        import open3d as o3d
        if intensity.ndim == 2:
            rgb = np.stack([intensity] * 3, axis=-1)
        else:
            rgb = intensity
        o3d.io.write_image(str(path), o3d.geometry.Image(rgb.astype(np.uint8)))
    except Exception as e:
        # 실패하면 raw npy 만 남음 — 치명적 아님
        print(f"  [warn] PNG 저장 실패({path.name}): {e}")


def _safe_read_theta(tt: Turntable, fallback: float) -> float:
    val = tt.getActualPos()
    if isinstance(val, bool) or val is None:
        return float(fallback)
    return float(val)


def _reset_turntable_to_zero(tt: Turntable, vel: float) -> bool:
    cur = _safe_read_theta(tt, 0.0)
    if abs(cur) < np.radians(0.5):
        print(f"[turntable] 이미 θ≈0 ({np.degrees(cur):+.3f}°) — reset 생략")
        return True
    print(f"[turntable] θ=0 리셋 (현재 {np.degrees(cur):+.2f}°)")
    return _move_with_recovery(tt, 0.0, vel, n_retries=2)


def _recover_turntable(tt: Turntable) -> bool:
    """
    드라이버 fault (FAS_GetAxisStatus 연속 실패) 시 복구 시도.
    - check_drive_err (alarm 자동 reset)
    - servo off → on
    """
    print("[turntable] 복구 시도: alarm reset + servo 재기동")
    try:
        tt.check_drive_err()
        time.sleep(0.3)
        tt.set_servo_on(False)
        time.sleep(0.3)
        tt.set_servo_on(True)
        time.sleep(0.5)
        # 재확인
        status = tt.GetAxisStatus()
        ok = isinstance(status, tuple)
        print(f"[turntable] 복구 {'성공' if ok else '실패'}")
        return ok
    except Exception as e:
        print(f"[turntable] 복구 예외: {e}")
        return False


def _move_with_recovery(
    tt: Turntable,
    theta_rad: float,
    vel: float,
    n_retries: int = 2,
) -> bool:
    """
    move_abs + wait_motion_done 에 복구 재시도 래핑.
    Return True on confirmed arrival, False on give-up.
    """
    for attempt in range(1, n_retries + 2):
        ok = tt.move_abs(float(theta_rad), float(vel))
        if ok is False:
            print(f"  [retry {attempt}] move_abs 거부 — 복구 시도")
            if not _recover_turntable(tt):
                continue
            continue
        done = tt.wait_motion_done()
        if done:
            return True
        print(f"  [retry {attempt}] wait_motion_done 실패 — 복구 시도")
        _recover_turntable(tt)
    return False


def _copy_calib_file(src: Path, dst_dir: Path) -> Optional[str]:
    if not src.exists():
        return None
    dst = dst_dir / src.name
    shutil.copy2(src, dst)
    return dst.name


# ─── 프레임 저장 ────────────────────────────────────────────────────────

def save_frame(
    frame_dir: Path,
    idx: int,
    organized_range: np.ndarray,              # (H, W, 3) float32 mm
    intensity: Optional[np.ndarray],           # (H, W) or (H, W, 3) uint8
    intensity_is_rgb: bool,                    # True 면 intensity 가 RGB per-point
    organized_normals: Optional[np.ndarray],   # (H, W, 3) float32
    color_image: Optional[np.ndarray],         # (H_c, W_c, 3) uint8 — ColorCamera 컴포넌트
    theta_target_rad: float,
    theta_actual_rad: float,
    T_EB: np.ndarray,
    timestamp: float,
    capture_elapsed_s: float,
) -> None:
    frame_dir.mkdir(parents=True, exist_ok=True)

    # Range — 필수
    np.save(frame_dir / "range.npy", organized_range.astype(np.float32))

    # Intensity — grayscale 또는 RGB per-point
    if intensity is not None:
        np.save(frame_dir / "intensity.npy", intensity.astype(np.uint8))
        _save_intensity_png(frame_dir / "intensity.png", intensity)

    # Normal (organized)
    if organized_normals is not None:
        np.save(frame_dir / "normals.npy", organized_normals.astype(np.float32))

    # ColorCamera 2D RGB 이미지 (선택)
    if color_image is not None:
        np.save(frame_dir / "color_image.npy", color_image.astype(np.uint8))
        _save_intensity_png(frame_dir / "color_image.png", color_image)

    # Meta
    H, W = organized_range.shape[:2]
    valid = ~np.all(organized_range == 0, axis=2)
    meta = {
        "idx":                int(idx),
        "theta_target_deg":   float(np.degrees(theta_target_rad)),
        "theta_target_rad":   float(theta_target_rad),
        "theta_actual_deg":   float(np.degrees(theta_actual_rad)),
        "theta_actual_rad":   float(theta_actual_rad),
        "T_EB":               [[float(v) for v in row] for row in T_EB.tolist()],
        "timestamp":          float(timestamp),
        "capture_elapsed_s":  float(capture_elapsed_s),
        "resolution":         [int(H), int(W)],
        "n_valid_pixels":     int(valid.sum()),
        "intensity_is_rgb":   bool(intensity_is_rgb),
        "color_image_shape":  (list(color_image.shape) if color_image is not None else None),
    }
    (frame_dir / "meta.json").write_text(
        json.dumps(meta, indent=2, ensure_ascii=False),
        encoding="utf-8",
    )


# ─── 메인 ──────────────────────────────────────────────────────────────

def main() -> None:
    parser = argparse.ArgumentParser(description="PhoXi raw dataset capture")
    parser.add_argument("--step", type=float, default=15.0,
                        help="θ 간격 (deg, 기본 15 → 24 프레임)")
    parser.add_argument("--out", type=str, default=None,
                        help="세션 폴더명 label. 지정 시 `<out>_YYYYMMDD_HHMMSS`."
                             " 기본: `phoxi_YYYYMMDD_HHMMSS`. `--resume` 이면 정확히 이 이름.")
    parser.add_argument("--skip-scan-init", action="store_true",
                        help="로봇 이동 생략 (이미 적절한 자세일 때)")
    parser.add_argument("--confirm-each", action="store_true",
                        help="각 프레임마다 Enter 확인 (디버그용)")
    parser.add_argument("--resume", action="store_true",
                        help="동일 --out 폴더에서 누락된 frame_XXX 만 다시 캡처")
    parser.add_argument("--color", action="store_true",
                        help="ColorCamera 컴포넌트 활성화 — 2D RGB 이미지도 저장 (Gen3)")
    parser.add_argument("--texture-source", default="LED",
                        choices=["LED", "Laser", "LaserEnhanced", "Color",
                                 "Computed", "ComputedEnhanced", "Focus"],
                        help='TextureSource. "Color" 로 두면 Intensity 가 RGB per-3D-point')
    parser.add_argument("--color-resolution", default=None,
                        help='ColorSettings_Resolution (예: Resolution_2576x1460)')
    args = parser.parse_args()

    # 출력 폴더 — 항상 생성시간 포함해서 덮어쓰기 방지.
    #   - 새 세션: `<out-or-"phoxi">_YYYYMMDD_HHMMSS`
    #   - --resume: args.out 에 지정된 이름을 그대로 찾아가 이어붙임
    session_stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    if args.resume:
        if not args.out:
            sys.exit("--resume 에는 --out <기존 세션 폴더명> 지정 필요")
        out_name = args.out
    else:
        label = args.out if args.out else "phoxi"
        out_name = f"{label}_{session_stamp}"
    out_root = DEFAULT_OUT_ROOT / out_name
    frames_dir = out_root / "frames"
    calib_dir = out_root / "calibration"
    if args.resume and not out_root.exists():
        sys.exit(f"--resume 대상 폴더 없음: {out_root}")
    frames_dir.mkdir(parents=True, exist_ok=True)
    calib_dir.mkdir(parents=True, exist_ok=True)
    print(f"[dataset] 출력: {out_root}")

    # 캘리브레이션 파일 복사 (self-contained)
    hand_eye_copy  = _copy_calib_file(HAND_EYE_YAML,   calib_dir)
    turntable_copy = _copy_calib_file(TURNTABLE_YAML,  calib_dir)

    # 하드웨어 연결
    print("[conn] 로봇 연결")
    robot = XArmInterface(ROBOT_IP)

    print("[conn] 턴테이블 연결")
    tt = Turntable(bd_id=TURNTABLE_BD_ID, ip=TURNTABLE_IP, pulses_per_rev=50000)
    tt.connect(comm_type=0)
    tt.check_drive_info()
    tt.check_drive_err()
    tt.set_servo_on(True)
    tt.set_acceleration(np.radians(180), np.radians(180))

    print("[conn] PhoXi 연결")
    sensor = PhoxiClient(PhoxiConfig(
        serial_number=PHOXI_SERIAL,
        trigger_timeout_s=15.0,
        target_interval_s=0.5,
        enable_color_camera=args.color,
        texture_source=args.texture_source,
        color_resolution=args.color_resolution,
    ))
    sensor.initialize()

    captured: list = []

    try:
        # 1) 로봇 scan_init 자세로
        if not args.skip_scan_init:
            print(f"\n[scan_init] joints={SCAN_INIT_JOINTS_DEG}")
            input("Enter 로 scan_init 이동... ")
            robot.enable_motion()
            code = robot.arm.set_servo_angle(
                angle=list(SCAN_INIT_JOINTS_DEG),
                speed=ROBOT_SPEED_DEG_S,
                is_radian=False, wait=True,
            )
            if code != 0:
                raise RuntimeError(f"scan_init 이동 실패 (code={code})")

        # 2) 턴테이블 θ=0 리셋
        _reset_turntable_to_zero(tt, TURNTABLE_VEL_RAD_S)

        # 3) θ 스케줄
        step_rad = float(np.radians(args.step))
        n = int(round(2.0 * np.pi / step_rad))
        print(f"\n[schedule] step={args.step}°  → {n} 프레임")

        # 4) 워밍업 캡처 1회 (intrinsic fit 용)
        print("\n[warmup] intrinsic fit 용 캡처 1회")
        _ = sensor.capture(frame_id=-1, timestamp=time.time())
        try:
            intrinsic = sensor.get_intrinsic()
            K = intrinsic.intrinsic_matrix
            intrinsic_meta = {
                "fx": float(K[0, 0]), "fy": float(K[1, 1]),
                "cx": float(K[0, 2]), "cy": float(K[1, 2]),
                "width": int(intrinsic.width),
                "height": int(intrinsic.height),
            }
        except Exception as e:
            print(f"  ⚠ intrinsic fit 실패: {e}")
            intrinsic_meta = None

        # 5) 세션 meta 기록
        session_meta = {
            "session":              session_stamp,
            "n_frames":             int(n),
            "theta_step_deg":       float(args.step),
            "turntable_vel_rad_s":  TURNTABLE_VEL_RAD_S,
            "dwell_s":              DWELL_S,
            "scan_init_joints_deg": SCAN_INIT_JOINTS_DEG,
            "robot_ip":             ROBOT_IP,
            "turntable_ip":         TURNTABLE_IP,
            "phoxi_serial":         PHOXI_SERIAL,
            "intrinsic":            intrinsic_meta,
            "texture_source":       args.texture_source,
            "color_camera_enabled": bool(args.color),
            "color_resolution":     args.color_resolution,
            "calibration": {
                "hand_eye_yaml":       hand_eye_copy,          # relative to calibration/
                "turntable_frame_yaml": turntable_copy,
                "T_OF":                "identity (O ≡ F at θ=0)",
            },
            "frames_dir":           "frames",
            "created_at":           datetime.now().isoformat(),
        }
        (out_root / "session_meta.json").write_text(
            json.dumps(session_meta, indent=2, ensure_ascii=False),
            encoding="utf-8",
        )

        # 6) 회전 스캔 루프
        existing_frames = set()
        if args.resume:
            for d in frames_dir.iterdir():
                if d.is_dir() and d.name.startswith("frame_"):
                    try:
                        idx = int(d.name.split("_")[1])
                        if (d / "range.npy").exists():
                            existing_frames.add(idx)
                    except Exception:
                        pass
            print(f"\n[resume] 이미 저장된 프레임: {sorted(existing_frames)}")

        for i in range(n):
            theta_target = i * step_rad

            if args.resume and i in existing_frames:
                print(f"─── [{i+1}/{n}] θ_target = {np.degrees(theta_target):+.1f}° "
                      f"— 이미 저장됨, skip")
                continue

            print(f"\n─── [{i+1}/{n}] θ_target = {np.degrees(theta_target):+.1f}° ───")

            # i==0 이어도 resume 모드면 실제 이동 필요 (이전 세션 종료 후 θ 위치 모름)
            need_move = i > 0 or (args.resume and i == 0)
            if need_move:
                arrived = _move_with_recovery(tt, theta_target, TURNTABLE_VEL_RAD_S, n_retries=2)
                if not arrived:
                    print("  ✘ 턴테이블 이동 실패(복구 2회 포함) — skip")
                    continue

            time.sleep(DWELL_S)
            theta_actual = _safe_read_theta(tt, theta_target)

            if args.confirm_each:
                input("  Enter 로 캡처... ")

            t_cap = time.perf_counter()
            result = sensor.capture(frame_id=i, timestamp=time.time())
            elapsed = time.perf_counter() - t_cap

            if result is None:
                print("  ⚠ 캡처 실패")
                continue

            organized_range   = sensor._last_organized_pts
            organized_normals = sensor._last_organized_normals
            intensity         = sensor._last_intensity
            intensity_is_rgb  = bool(sensor._last_intensity_is_rgb)
            color_image       = sensor._last_color_image

            T_EB = robot.get_ee_pose_mat()

            save_frame(
                frame_dir=frames_dir / f"frame_{i:03d}",
                idx=i,
                organized_range=organized_range,
                intensity=intensity,
                intensity_is_rgb=intensity_is_rgb,
                organized_normals=organized_normals,
                color_image=color_image,
                theta_target_rad=theta_target,
                theta_actual_rad=theta_actual,
                T_EB=T_EB,
                timestamp=result.timestamp,
                capture_elapsed_s=elapsed,
            )
            n_valid = int(np.sum(~np.all(organized_range == 0, axis=2)))
            print(f"  saved frame_{i:03d}  pts={n_valid:,}  "
                  f"θ_act={np.degrees(theta_actual):+.2f}°  "
                  f"capture={elapsed:.2f}s")
            captured.append(i)

        print(f"\n[dataset] 완료 — {len(captured)}/{n} 프레임 저장")
        print(f"  {out_root}")

    finally:
        try: sensor.shutdown()
        except Exception: pass
        try: robot.disconnect()
        except Exception: pass
        try: tt.disconnect()
        except Exception: pass


if __name__ == "__main__":
    main()
