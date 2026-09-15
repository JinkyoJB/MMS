#!/usr/bin/env python3
"""
scripts/artec_hand_eye_calib.py

xArm7 + Artec Spider Hand-Eye 캘리브레이션 (`docs/5_artec_hand_eye.md` 참조).

검출 방식
--------
- ChArUco 보드 (DICT_5X5_100, 7×5, 30/22mm) 를 Artec 텍스처 이미지에서 검출
- 각 corner pixel → vertex UV 매핑 → 3D 점 (camera frame, mm)
- Procrustes 로 T_M_C 산출
- HandEyeCalibrator (cv2.calibrateHandEye 5-method 시도) 가 T_E_C 산출

실행 모드
--------
1) `--poses <yaml>` 가 존재하면 yaml 의 포즈로 자동 순회 (PhoXi 방식과 동일)
2) 그 외 / `--interactive` → 사용자가 manual mode 로 로봇을 움직이고 Enter 로 캡처
                          (감지 성공 자세는 yaml 에 기록)

Usage
-----
  # 보드 인쇄 후 (1회): scripts/artec_make_charuco.py
  # 첫 실행 - manual 모드 (poses yaml 없음)
  python scripts/artec_hand_eye_calib.py
  # 이후: 같은 yaml 로 재실행
  python scripts/artec_hand_eye_calib.py --poses config/calibration/artec_calibration_poses.yaml
"""

from __future__ import annotations

import argparse
import logging
import sys
import time
from pathlib import Path
from typing import Optional

import cv2
import numpy as np
import yaml
from scipy.spatial.transform import Rotation as ScipyR

_PROJECT_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(_PROJECT_ROOT))

from utils.calibration.hand_eye_calibrator import HandEyeCalibrator
from mms_artec.utils.calibration.artec_charuco_detector import (
    ArtecCharucoDetector, CharucoBoardSpec,
)
from utils.robot.xarm_interface import XArmInterface
from mms_artec.sensor.artec_client import ArtecClient, ArtecConfig


# ─── 기본 설정 ─────────────────────────────────────────────────────────

ROBOT_IP        = "192.168.1.210"
DEFAULT_POSES_YAML = _PROJECT_ROOT / "config" / "calibration" / "artec_calibration_poses.yaml"
DEFAULT_INTRINSIC_YAML = _PROJECT_ROOT / "config" / "calibration" / "artec_intrinsic.yaml"
OUTPUT_YAML     = _PROJECT_ROOT / "config" / "calibration" / "hand_eye_artec.yaml"
DEBUG_DIR       = _PROJECT_ROOT / "debug_calib_artec"

MOVE_SPEED_DEG  = 10
SETTLE_TIME_S   = 1.5
MIN_SAMPLES     = 5

# ChArUco 보드 프리셋 — Artec Spider FOV 작아서 작은 보드 필요
# (Spider 작동거리 170-350mm, FOV 90-180mm)
BOARD_PRESETS = {
    # 큰 보드 — PhoXi/광각 카메라용 (참고용, Spider 에는 부적합)
    "a4": CharucoBoardSpec(7, 5, 30.0, 22.0, cv2.aruco.DICT_5X5_100),
    # Spider 추천 — 100×60mm, FOV 안에 충분히 들어옴
    "spider": CharucoBoardSpec(5, 3, 20.0, 15.0, cv2.aruco.DICT_4X4_50),
    # 더 작은 — 80×48mm
    "spider_small": CharucoBoardSpec(5, 3, 16.0, 12.0, cv2.aruco.DICT_4X4_50),
}
DEFAULT_BOARD_NAME = "spider"


# ─── 헬퍼 ─────────────────────────────────────────────────────────────

def _read_T_EB(robot: XArmInterface) -> np.ndarray:
    """xArm 현재 TCP 포즈 → T_EB (E→B, translation meters)."""
    pose6 = robot.get_pose(is_radian=False)
    R_EB = ScipyR.from_euler("xyz", np.radians(pose6[3:6])).as_matrix()
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R_EB
    T[:3, 3] = np.array(pose6[:3]) / 1000.0
    return T


def _capture_artec_frame(client: ArtecClient):
    """1회 capture → FrameMeshHandle 반환. SDK 에러 시 None."""
    try:
        return client.capture_frame(capture_texture=True)
    except RuntimeError as e:
        # Artec SDK 가 reconstructAndTexturizeMesh 실패하면 0x80070803 등
        # RuntimeError 가 올라옴 — 다음 자세에서 재시도 가능.
        print(f"  [warn] capture 실패: {e}")
        time.sleep(0.5)
        return None


def _detect_and_log(
    detector: ArtecCharucoDetector,
    fmh,
    pose_name: str,
):
    """FrameMeshHandle 에서 ChArUco 검출 → CharucoDetection 또는 None."""
    if fmh is None:
        print(f"  [{pose_name}] capture 실패")
        return None
    if not fmh.has_image() or not fmh.is_textured():
        print(f"  [{pose_name}] 텍스처 없음 — capture_texture=True 필요")
        return None

    image = fmh.image()                  # (H, W, 3) uint8 RGB
    vertices = fmh.vertices()            # (N, 3) float32 mm
    uv = fmh.uv()                        # (N, 2) float32 [0..1]

    if image is None or vertices is None or uv is None:
        print(f"  [{pose_name}] image/vertices/uv 누락")
        return None

    print(f"  [{pose_name}] mesh verts={len(vertices):,}  "
          f"image={image.shape[1]}×{image.shape[0]}")

    # 검출 실패해도 raw texture 는 저장 — 보드가 시야에 들어왔는지 확인용
    DEBUG_DIR.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(
        str(DEBUG_DIR / f"{pose_name}_raw.png"),
        cv2.cvtColor(image, cv2.COLOR_RGB2BGR),
    )

    # ArUco 만 먼저 감지해서 보드 가시성 체크
    n_aruco_only = 0
    try:
        ch_c, ch_i, n_aruco_only = detector.detect_charuco_corners(image)
    except Exception:
        pass

    det = detector.detect(image, vertices, uv)
    if det is None:
        # ArUco 마커만이라도 어디 검출됐는지 시각화 — 인접성/위치 디버그용
        try:
            mc, mi = detector.detect_aruco_only(image)
            vis = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            if mc is not None and mi is not None and len(mi) > 0:
                cv2.aruco.drawDetectedMarkers(vis, mc, mi)
            cv2.putText(vis, f"aruco={n_aruco_only} (charuco failed)",
                        (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            cv2.imwrite(str(DEBUG_DIR / f"{pose_name}_aruco.png"), vis)
        except Exception:
            pass
        print(f"  [{pose_name}] ChArUco 검출 실패  "
              f"(aruco markers seen = {n_aruco_only})  "
              f"→ {DEBUG_DIR / (pose_name + '_aruco.png')}  확인")
        return None
    print(f"  [{pose_name}] ✓ corners={det.n_corners}  "
          f"rmse={det.procrustes_rmse_mm:.2f}mm  "
          f"t_M_C=({det.T_MC[0,3]:.0f},{det.T_MC[1,3]:.0f},{det.T_MC[2,3]:.0f})mm")
    return det


def _save_debug(name: str, det) -> None:
    if det is None or det.debug_image is None:
        return
    DEBUG_DIR.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(DEBUG_DIR / f"{name}_detect.png"), det.debug_image)


def _append_pose_yaml(yaml_path: Path, name: str, pose6_mm_deg) -> None:
    """yaml 의 poses 리스트에 하나 추가 (이미 있으면 skip)."""
    yaml_path.parent.mkdir(parents=True, exist_ok=True)
    if yaml_path.exists():
        data = yaml.safe_load(yaml_path.read_text(encoding="utf-8")) or {}
    else:
        data = {}
    poses = data.get("poses", []) or []
    if any(p.get("name") == name for p in poses):
        return
    poses.append({
        "name": name,
        "ee_pose": [round(float(v), 2) for v in pose6_mm_deg],
    })
    data["poses"] = poses
    yaml_path.write_text(
        yaml.dump(data, default_flow_style=None, allow_unicode=True),
        encoding="utf-8",
    )


def _next_manual_idx(yaml_path: Path) -> int:
    """yaml 의 기존 'manual_NN' 중 가장 큰 NN+1 을 반환 (없으면 0)."""
    if not yaml_path.exists():
        return 0
    data = yaml.safe_load(yaml_path.read_text(encoding="utf-8")) or {}
    poses = data.get("poses", []) or []
    max_n = -1
    for p in poses:
        n = str(p.get("name", ""))
        if n.startswith("manual_"):
            try:
                idx = int(n.split("_", 1)[1])
                if idx > max_n:
                    max_n = idx
            except ValueError:
                pass
    return max_n + 1


# ─── 모드 1: yaml 의 포즈로 자동 순회 ──────────────────────────────────

def _run_scripted(
    poses_yaml: Path,
    robot: XArmInterface,
    sensor: ArtecClient,
    detector: ArtecCharucoDetector,
    calibrator: HandEyeCalibrator,
    via_home: bool = True,
    sensor_name: str = "artec",
    start_from: int = 0,
) -> None:
    data = yaml.safe_load(poses_yaml.read_text(encoding="utf-8")) or {}
    poses = data.get("poses", []) or []
    if not poses:
        raise RuntimeError(f"{poses_yaml} 에 poses 가 비어 있습니다.")

    print(f"\n[scripted] {len(poses)} 포즈 순회 시작 ({poses_yaml})  "
          f"via_home={via_home}  start_from={start_from}")
    robot.enable_motion()

    for i, p in enumerate(poses):
        if i < start_from:
            continue
        name = p.get("name", f"pose_{i}")

        # 자세 간 충돌 회피 — 매 이동 전 home 경유
        if via_home:
            print(f"  [via-home] {sensor_name} home 으로 복귀")
            try:
                robot.go_home(sensor=sensor_name, speed=MOVE_SPEED_DEG, confirm=False)
            except Exception as e:
                print(f"  [warn] go_home 실패: {e} — 직접 이동 시도")

        if "joints" in p:
            joints_deg = list(p["joints"])
            print(f"\n[{i+1}/{len(poses)}] {name}  joints={joints_deg}")
        elif "ee_pose" in p:
            ee = p["ee_pose"]
            ee_rad = [ee[0], ee[1], ee[2],
                      np.radians(ee[3]), np.radians(ee[4]), np.radians(ee[5])]
            code_ik, ik_joints = robot.arm.get_inverse_kinematics(
                pose=ee_rad, input_is_radian=True, return_is_radian=False,
            )
            if code_ik != 0:
                print(f"\n[{i+1}/{len(poses)}] {name}  IK 실패 ({code_ik}) — skip")
                continue
            joints_deg = ik_joints[:7]
            _, cur = robot.arm.get_servo_angle(is_radian=False)
            joints_deg = [
                j - 360 * round((j - c) / 360)
                for j, c in zip(joints_deg, cur[:7])
            ]
            print(f"\n[{i+1}/{len(poses)}] {name}  ee={ee}  IK→{joints_deg}")
        else:
            print(f"\n[{i+1}/{len(poses)}] {name}  joints/ee_pose 키 없음 — skip")
            continue

        _, state = robot.arm.get_state()
        if state not in (0, 1, 2):
            robot.arm.clean_error(); robot.arm.clean_warn()
            robot.enable_motion(); time.sleep(0.5)

        code = robot.arm.set_servo_angle(
            angle=joints_deg, speed=MOVE_SPEED_DEG, is_radian=False, wait=True,
        )
        if code != 0:
            print(f"  이동 실패 ({code}) — skip")
            robot.arm.clean_error(); robot.arm.clean_warn(); robot.enable_motion()
            continue
        time.sleep(SETTLE_TIME_S)

        T_EB = _read_T_EB(robot)
        fmh = _capture_artec_frame(sensor)
        det = _detect_and_log(detector, fmh, name)
        _save_debug(name, det)
        if det is None:
            continue
        calibrator.add_sample(T_EB, det.T_MC)


# ─── 모드 2: interactive — 사용자가 manual move ─────────────────────────

def _run_interactive(
    poses_yaml: Path,
    robot: XArmInterface,
    sensor: ArtecClient,
    detector: ArtecCharucoDetector,
    calibrator: HandEyeCalibrator,
    target_n: int = 12,
) -> None:
    """
    사용자가 robot 을 manual mode 로 직접 움직이며 Enter 로 캡처.
    감지 성공한 자세는 `poses_yaml` 에 기록.
    """
    # 기존 yaml 의 manual_NN 다음 번호부터 시작 — 재실행 시 collision 회피
    start_idx = _next_manual_idx(poses_yaml)
    if start_idx > 0:
        print(f"\n[interactive] yaml 에 기존 자세 {start_idx}개 있음 — "
              f"이번 세션은 manual_{start_idx:02d} 부터 추가")
    print("\n[interactive] manual mode — 사용자가 직접 로봇을 움직이세요.")
    print("  Enter      : 현재 자세에서 캡처 + ChArUco 검출 시도")
    print("  q + Enter  : 종료 (현재까지 샘플로 calibrate 시도)")
    print(f"  목표 샘플 수: {target_n} (이번 세션 추가)\n")

    # manual mode (gravity-comp). xArm SDK: set_mode(2) = teach mode.
    try:
        robot.arm.motion_enable(enable=True)
        robot.arm.set_mode(2)
        robot.arm.set_state(0)
    except Exception as e:
        print(f"  [warn] manual mode 진입 실패: {e}")

    saved = 0
    while True:
        cmd = input(f"\n  [{saved}/{target_n}] Enter 캡처 / q 종료 > ").strip().lower()
        if cmd == "q":
            break

        T_EB = _read_T_EB(robot)
        fmh = _capture_artec_frame(sensor)
        idx = start_idx + saved
        name = f"manual_{idx:02d}"
        det = _detect_and_log(detector, fmh, name)
        _save_debug(name, det)
        if det is None:
            continue

        # 검출 성공 → calibrator 에 추가 + yaml 에 기록
        calibrator.add_sample(T_EB, det.T_MC)
        pose6 = robot.get_pose(is_radian=False)
        _append_pose_yaml(poses_yaml, name, pose6)
        saved += 1
        if saved >= target_n:
            break

    # 다시 position mode 로
    try:
        robot.arm.set_mode(0)
        robot.arm.set_state(0)
    except Exception:
        pass


# ─── main ─────────────────────────────────────────────────────────────

def main() -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )

    parser = argparse.ArgumentParser(description="Artec hand-eye 캘리브레이션")
    parser.add_argument("--poses", type=str, default=str(DEFAULT_POSES_YAML),
                        help="포즈 yaml 경로 (없으면 interactive 모드)")
    parser.add_argument("--interactive", action="store_true",
                        help="yaml 존재해도 interactive 모드 강제")
    parser.add_argument("--target-n", type=int, default=12,
                        help="interactive 모드 목표 샘플 수")
    parser.add_argument("--board", type=str, default=DEFAULT_BOARD_NAME,
                        choices=list(BOARD_PRESETS.keys()),
                        help="보드 프리셋 (spider 권장)")
    parser.add_argument("--squares-x", type=int, default=None,
                        help="(고급) 가로 칸 수 — 프리셋 덮어쓰기")
    parser.add_argument("--squares-y", type=int, default=None,
                        help="(고급) 세로 칸 수")
    parser.add_argument("--square-mm", type=float, default=None,
                        help="ChArUco 한 칸 길이 (인쇄 후 실측 보정 — 프리셋 덮어쓰기)")
    parser.add_argument("--marker-mm", type=float, default=None,
                        help="ArUco 마커 한 변 길이 — 프리셋 덮어쓰기")
    parser.add_argument("--serial", type=str, default=None,
                        help="Artec 시리얼 (None → 첫 번째 스캐너)")
    parser.add_argument("--no-texture-flash", action="store_true",
                        help="텍스처용 화이트 LED 끄기 (광택 보드 glare 회피, "
                             "구조광 flash 는 그대로 — geometry 정상)")
    parser.add_argument("--no-auto-exposure", action="store_true",
                        help="auto exposure 끄기 (기본 on)")
    parser.add_argument("--no-via-home", action="store_true",
                        help="자세 간 home 경유 비활성 (기본: 매 이동 전 home 으로 복귀)")
    parser.add_argument("--start-from", type=int, default=0,
                        help="scripted 모드에서 N번째 자세부터 시작 (resume 용)")
    parser.add_argument("--intrinsic", type=str, default=str(DEFAULT_INTRINSIC_YAML),
                        help="texture camera intrinsic yaml — 있으면 solvePnP 경로, "
                             "없으면 UV→3D fallback")
    parser.add_argument("--no-intrinsic", action="store_true",
                        help="intrinsic yaml 무시하고 UV→3D 강제")
    args = parser.parse_args()

    poses_yaml = Path(args.poses)
    use_interactive = args.interactive or not poses_yaml.exists()

    base = BOARD_PRESETS[args.board]
    board_spec = CharucoBoardSpec(
        squares_x=int(args.squares_x) if args.squares_x is not None else base.squares_x,
        squares_y=int(args.squares_y) if args.squares_y is not None else base.squares_y,
        square_length_mm=float(args.square_mm) if args.square_mm is not None else base.square_length_mm,
        marker_length_mm=float(args.marker_mm) if args.marker_mm is not None else base.marker_length_mm,
        aruco_dict=base.aruco_dict,
    )

    # intrinsic 로드 → solvePnP 경로 활성
    intrinsic = None
    intr_path = Path(args.intrinsic)
    if not args.no_intrinsic and intr_path.exists():
        intr_data = yaml.safe_load(intr_path.read_text(encoding="utf-8")) or {}
        intrinsic = {
            "K": intr_data["K"],
            "dist": intr_data.get("dist", [0, 0, 0, 0, 0]),
            "image_size": intr_data.get("image_size", None),
        }
        print(f"  [info] intrinsic 로드: {intr_path}  "
              f"reproj_err={intr_data.get('reprojection_error_px', '?'):.3f}px  "
              f"→ solvePnP 경로 사용")
        # ★ 낡은 intrinsic 을 조용히 쓰면 T_EC 가 통째로 틀린다.
        #   K 는 **카메라 개체 종속**이라 스캐너를 바꾸면 무효다. 2026-09-15 현장에서
        #   intrinsic 단계가 frame 부족으로 실패했는데 exit 0 이라 그냥 넘어갔고,
        #   여기서 **구 스캐너(SP.10.36181288)의 K** 를 그대로 써버렸다.
        _age_days = None
        _when = str(intr_data.get("date", ""))
        if _when:
            try:
                from datetime import date as _date
                _age_days = (_date.today() - _date.fromisoformat(_when)).days
            except ValueError:
                pass
        if _age_days is not None and _age_days > 90:
            print(f"\n  ⚠ 이 intrinsic 은 {_when} 측정 ({_age_days}일 전) 이다.")
            print( "     스캐너를 교체·탈착했다면 **무효**다 — K 는 개체 종속.")
            print( "     다시 잡으려면:  python scripts/artec/intrinsic_calib.py")
            if input("     그래도 이 값으로 진행? (y/N) > ").strip().lower() != "y":
                print("  중단. intrinsic 을 먼저 다시 잡을 것.")
                return 1
    else:
        print(f"  [info] intrinsic 없음 — UV→3D fallback (정확도 낮음)")

    detector = ArtecCharucoDetector(board_spec, intrinsic=intrinsic)

    print("=" * 70)
    print("  Artec Hand-Eye Calibration  (ChArUco + UV→3D Procrustes)")
    print("=" * 70)
    print(f"  ChArUco        : {board_spec.squares_x}×{board_spec.squares_y}  "
          f"sq={board_spec.square_length_mm}mm  mk={board_spec.marker_length_mm}mm")
    print(f"  poses yaml     : {poses_yaml}  (exists={poses_yaml.exists()})")
    print(f"  output yaml    : {OUTPUT_YAML}")
    print(f"  debug dir      : {DEBUG_DIR}")
    print(f"  mode           : {'interactive' if use_interactive else 'scripted'}")

    robot = XArmInterface(ip=ROBOT_IP)
    sensor = ArtecClient(ArtecConfig(
        serial_number=args.serial,
        capture_texture=True,
        target_interval_s=0.0,
    ))
    sensor.initialize()

    # 광원 / 노출 제어 — Spider LED glare 줄이기
    try:
        if args.no_texture_flash:
            sensor._scanner.enable_texture_flash(False)
            print("  [info] texture flash OFF (ambient 조명으로 텍스처 캡처)")
        else:
            print(f"  [info] texture flash ON  "
                  f"(현재: {sensor._scanner.texture_flash_enabled()})")
        if not args.no_auto_exposure:
            sensor._scanner.enable_auto_exposure(True)
            print("  [info] auto exposure ON")
    except Exception as e:
        print(f"  [warn] 광원/노출 설정 실패: {e}")

    calibrator = HandEyeCalibrator()
    DEBUG_DIR.mkdir(parents=True, exist_ok=True)

    print("\n준비되면 Enter (Artec 가 부드럽게 capture 시작합니다)...")
    input()

    try:
        if use_interactive:
            _run_interactive(
                poses_yaml, robot, sensor, detector, calibrator,
                target_n=args.target_n,
            )
        else:
            _run_scripted(
                poses_yaml, robot, sensor, detector, calibrator,
                via_home=not args.no_via_home,
                sensor_name="artec",
                start_from=int(args.start_from),
            )
    finally:
        sensor.shutdown()
        robot.disconnect()

    # ── solve & save ──────────────────────────────────────────────────
    print(f"\n수집된 샘플: {calibrator.n_samples}")
    if calibrator.n_samples < MIN_SAMPLES:
        print(f"⚠ 샘플 부족 — 최소 {MIN_SAMPLES} 필요. 종료.")
        return 1

    print("calibrate() 실행...")
    calibrator.calibrate()
    calibrator.save_yaml(
        OUTPUT_YAML,
        sensor="artec_spider",
        method="charuco_uv3d_procrustes",
    )
    print(f"\n[done] {OUTPUT_YAML}")
    print("  → config/sensor_frames.yaml 의 T_EC_artec 를 위 값으로 갱신하세요.")


if __name__ == "__main__":
    # ★ 실패를 exit code 로 알린다 — intrinsic_calib.py 와 같은 이유.
    #   조용히 exit 0 으로 끝나면 calibrate.py 가 다음 단계로 넘어가고,
    #   **낡은 값을 그대로 쓴 결과물**이 만들어진다 (2026-09-15 현장 사고).
    sys.exit(main() or 0)
