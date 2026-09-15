#!/usr/bin/env python3
"""
scripts/artec_intrinsic_calib.py

Artec texture camera intrinsic 캘리브레이션 (`docs/5_artec_hand_eye.md` §6 참조).

목적
----
hand-eye 캘리브에서 UV→3D 양자화 오차를 우회하기 위해 텍스처 카메라의
intrinsic (fx, fy, cx, cy + distortion) 을 측정한다.

방법
----
- 같은 ChArUco 보드를 다양한 위치/각도에서 캡처
- `cv2.aruco.calibrateCameraCharuco` 로 K, dist 산출
- `config/calibration/artec_intrinsic.yaml` 에 저장

저장된 yaml 은 `artec_hand_eye_calib.py` 가 자동 로드해서 solvePnP 경로 활성.

Usage
-----
  # 기존 hand-eye 자세 yaml 재사용 (권장 — 다양성 충분)
  python scripts/artec_intrinsic_calib.py

  # 또는 interactive (manual move)
  python scripts/artec_intrinsic_calib.py --interactive --target-n 15
"""

from __future__ import annotations

import argparse
import logging
import sys
import time
from pathlib import Path
from typing import List, Optional

import cv2
import numpy as np
import yaml
from scipy.spatial.transform import Rotation as ScipyR

_PROJECT_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(_PROJECT_ROOT))

from mms_artec.utils.calibration.artec_charuco_detector import (
    ArtecCharucoDetector, CharucoBoardSpec,
)
from utils.robot.xarm_interface import XArmInterface
from mms_artec.sensor.artec_client import ArtecClient, ArtecConfig


ROBOT_IP        = "192.168.1.210"
DEFAULT_POSES_YAML = _PROJECT_ROOT / "config" / "calibration" / "artec_calibration_poses.yaml"
OUTPUT_YAML     = _PROJECT_ROOT / "config" / "calibration" / "artec_intrinsic.yaml"
DEBUG_DIR       = _PROJECT_ROOT / "debug_intrinsic_artec"

MOVE_SPEED_DEG  = 10
SETTLE_TIME_S   = 1.5
MIN_FRAMES      = 8


# ─── 보드 ─────────────────────────────────────────────────────────────

BOARD_PRESETS = {
    "spider": CharucoBoardSpec(5, 3, 20.0, 15.0, cv2.aruco.DICT_4X4_50),
    "a4":     CharucoBoardSpec(7, 5, 30.0, 22.0, cv2.aruco.DICT_5X5_100),
    "spider_small": CharucoBoardSpec(5, 3, 16.0, 12.0, cv2.aruco.DICT_4X4_50),
}


# ─── 캡처 + 검출 누적 ─────────────────────────────────────────────────

def _capture(client: ArtecClient):
    try:
        return client.capture_frame(capture_texture=True)
    except RuntimeError as e:
        print(f"  [warn] capture: {e}")
        time.sleep(0.5)
        return None


def _detect_for_intrinsic(
    detector: ArtecCharucoDetector,
    image: np.ndarray,
    name: str,
):
    """
    cv2.aruco.calibrateCameraCharuco 입력용 — ChArUco corner 만 사용 (sub-pixel).

    Returns
    -------
    (corners (n,1,2), ids (n,1)) or (None, None)
    """
    ch_c, ch_i, n_aruco = detector.detect_charuco_corners(image)
    if ch_c is None or ch_i is None or len(ch_i) < 6:
        print(f"  [{name}] charuco 부족 (aruco={n_aruco}, charuco="
              f"{0 if ch_i is None else len(ch_i)}) — skip")
        return None, None
    print(f"  [{name}] aruco={n_aruco}  charuco corners={len(ch_i)}")
    return ch_c, ch_i


def _save_debug(name: str, image: np.ndarray, corners, ids) -> None:
    DEBUG_DIR.mkdir(parents=True, exist_ok=True)
    vis = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    if corners is not None and ids is not None:
        try:
            cv2.aruco.drawDetectedCornersCharuco(vis, corners, ids)
        except Exception:
            for (u, v) in corners.reshape(-1, 2).astype(int):
                cv2.circle(vis, (int(u), int(v)), 5, (0, 255, 0), 1)
    cv2.imwrite(str(DEBUG_DIR / f"{name}.png"), vis)


# ─── 모드 1: scripted (yaml 의 자세 순회) ──────────────────────────────

def _run_scripted(
    poses_yaml: Path,
    robot: XArmInterface,
    sensor: ArtecClient,
    detector: ArtecCharucoDetector,
    via_home: bool = True,
    sensor_name: str = "artec",
    start_from: int = 0,
):
    data = yaml.safe_load(poses_yaml.read_text(encoding="utf-8")) or {}
    poses = data.get("poses", []) or []
    if not poses:
        raise RuntimeError(f"{poses_yaml}: poses 비어 있음")

    print(f"\n[scripted] {len(poses)} 포즈 순회 ({poses_yaml})  via_home={via_home}")
    robot.enable_motion()

    all_corners: List[np.ndarray] = []
    all_ids: List[np.ndarray] = []
    image_size = None

    for i, p in enumerate(poses):
        if i < start_from:
            continue
        name = p.get("name", f"pose_{i}")

        if via_home:
            try:
                robot.go_home(sensor=sensor_name, speed=MOVE_SPEED_DEG, confirm=False)
            except Exception as e:
                print(f"  [warn] go_home: {e}")

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
                print(f"\n[{i+1}/{len(poses)}] {name}  IK 실패 — skip")
                continue
            joints_deg = ik_joints[:7]
            _, cur = robot.arm.get_servo_angle(is_radian=False)
            joints_deg = [j - 360 * round((j - c) / 360)
                          for j, c in zip(joints_deg, cur[:7])]
            print(f"\n[{i+1}/{len(poses)}] {name}  ee={ee}")
        else:
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

        fmh = _capture(sensor)
        if fmh is None or not fmh.has_image():
            print(f"  [{name}] capture 실패")
            continue
        image = fmh.image()
        if image_size is None:
            image_size = (image.shape[1], image.shape[0])
        elif (image.shape[1], image.shape[0]) != image_size:
            print(f"  [{name}] image size 다름 — skip")
            continue

        c, ids = _detect_for_intrinsic(detector, image, name)
        _save_debug(name, image, c, ids)
        if c is None:
            continue
        all_corners.append(c)
        all_ids.append(ids)

    return all_corners, all_ids, image_size


# ─── 모드 2: interactive ───────────────────────────────────────────────

def _run_interactive(
    robot: XArmInterface,
    sensor: ArtecClient,
    detector: ArtecCharucoDetector,
    target_n: int = 15,
):
    print(f"\n[interactive] manual mode — Enter 캡처 / q 종료, 목표 {target_n}장")
    try:
        robot.arm.motion_enable(enable=True)
        robot.arm.set_mode(2); robot.arm.set_state(0)
    except Exception as e:
        print(f"  [warn] manual mode: {e}")

    all_corners: List[np.ndarray] = []
    all_ids: List[np.ndarray] = []
    image_size = None
    saved = 0
    while True:
        cmd = input(f"\n  [{saved}/{target_n}] Enter / q > ").strip().lower()
        if cmd == "q":
            break
        fmh = _capture(sensor)
        if fmh is None or not fmh.has_image():
            print("  capture 실패")
            continue
        image = fmh.image()
        if image_size is None:
            image_size = (image.shape[1], image.shape[0])
        name = f"intrinsic_{saved:02d}"
        c, ids = _detect_for_intrinsic(detector, image, name)
        _save_debug(name, image, c, ids)
        if c is None:
            continue
        all_corners.append(c); all_ids.append(ids)
        saved += 1
        if saved >= target_n:
            break

    try:
        robot.arm.set_mode(0); robot.arm.set_state(0)
    except Exception:
        pass
    return all_corners, all_ids, image_size


# ─── main ─────────────────────────────────────────────────────────────

def main() -> None:
    logging.basicConfig(level=logging.INFO)
    parser = argparse.ArgumentParser(description="Artec texture camera intrinsic 캘리브")
    parser.add_argument("--poses", type=str, default=str(DEFAULT_POSES_YAML))
    parser.add_argument("--interactive", action="store_true")
    parser.add_argument("--target-n", type=int, default=15)
    parser.add_argument("--board", type=str, default="spider",
                        choices=list(BOARD_PRESETS.keys()))
    parser.add_argument("--square-mm", type=float, default=None)
    parser.add_argument("--marker-mm", type=float, default=None)
    parser.add_argument("--no-via-home", action="store_true")
    parser.add_argument("--start-from", type=int, default=0)
    parser.add_argument("--no-texture-flash", action="store_true")
    parser.add_argument("--fix-k3", action="store_true", default=True,
                        help="distortion k3 = 0 강제 (limited data 에서 overfitting 방지, 기본 on)")
    parser.add_argument("--full-distortion", action="store_true",
                        help="--fix-k3 끄고 5-param distortion 전체 fit")
    args = parser.parse_args()
    if args.full_distortion:
        args.fix_k3 = False

    base = BOARD_PRESETS[args.board]
    spec = CharucoBoardSpec(
        squares_x=base.squares_x, squares_y=base.squares_y,
        square_length_mm=float(args.square_mm) if args.square_mm else base.square_length_mm,
        marker_length_mm=float(args.marker_mm) if args.marker_mm else base.marker_length_mm,
        aruco_dict=base.aruco_dict,
    )
    detector = ArtecCharucoDetector(spec)    # intrinsic 없이 — 검출만

    poses_yaml = Path(args.poses)
    use_interactive = args.interactive or not poses_yaml.exists()

    print("=" * 70)
    print("  Artec Intrinsic Calibration  (cv2.aruco.calibrateCameraCharuco)")
    print("=" * 70)
    print(f"  보드: {spec.squares_x}×{spec.squares_y}  sq={spec.square_length_mm}  mk={spec.marker_length_mm}")
    print(f"  poses: {poses_yaml}  (exists={poses_yaml.exists()})")
    print(f"  output: {OUTPUT_YAML}")
    print(f"  mode: {'interactive' if use_interactive else 'scripted'}")

    robot = XArmInterface(ip=ROBOT_IP)
    sensor = ArtecClient(ArtecConfig(serial_number=None, capture_texture=True))
    sensor.initialize()
    if args.no_texture_flash:
        try:
            sensor._scanner.enable_texture_flash(False)
        except Exception:
            pass
    try:
        sensor._scanner.enable_auto_exposure(True)
    except Exception:
        pass

    DEBUG_DIR.mkdir(parents=True, exist_ok=True)
    print("\nEnter 누르면 시작...")
    input()

    try:
        if use_interactive:
            corners, ids, img_sz = _run_interactive(
                robot, sensor, detector, target_n=args.target_n,
            )
        else:
            corners, ids, img_sz = _run_scripted(
                poses_yaml, robot, sensor, detector,
                via_home=not args.no_via_home,
                start_from=int(args.start_from),
            )
    finally:
        sensor.shutdown()
        robot.disconnect()

    print(f"\n수집 frames: {len(corners)}")
    if len(corners) < MIN_FRAMES:
        print(f"⚠ 최소 {MIN_FRAMES} frame 필요. 종료.")
        return 1

    # ── calibrateCamera (신 API: matchImagePoints → calibrateCamera) ───
    print("\ncalibrateCamera 실행...")
    obj_pts_list: List[np.ndarray] = []
    img_pts_list: List[np.ndarray] = []
    for c, ids_arr in zip(corners, ids):
        try:
            obj_pts, img_pts = detector.board.matchImagePoints(c, ids_arr)
        except Exception as e:
            print(f"  matchImagePoints 실패: {e}")
            continue
        if obj_pts is None or img_pts is None or len(obj_pts) < 4:
            continue
        obj_pts_list.append(np.asarray(obj_pts, dtype=np.float32).reshape(-1, 1, 3))
        img_pts_list.append(np.asarray(img_pts, dtype=np.float32).reshape(-1, 1, 2))

    if len(obj_pts_list) < MIN_FRAMES:
        print(f"⚠ 매칭 후 frame {len(obj_pts_list)} < {MIN_FRAMES} — 종료")
        return 1

    flags = 0
    if args.fix_k3:
        flags |= cv2.CALIB_FIX_K3
    print(f"  flags: fix_k3={bool(flags & cv2.CALIB_FIX_K3)}")

    try:
        ret, K, dist, rvecs, tvecs = cv2.calibrateCamera(
            objectPoints=obj_pts_list,
            imagePoints=img_pts_list,
            imageSize=img_sz,
            cameraMatrix=None,
            distCoeffs=None,
            flags=flags,
        )
    except Exception as e:
        print(f"⚠ calibrate 실패: {e}")
        return 1

    print(f"\nreproj rmse = {ret:.3f} px  (used {len(obj_pts_list)} frames)")
    print(f"K =\n{K}")
    print(f"dist = {dist.flatten()}")

    # ── 저장 ────────────────────────────────────────────────────────────
    import datetime
    OUTPUT_YAML.parent.mkdir(parents=True, exist_ok=True)
    OUTPUT_YAML.write_text(yaml.dump({
        "sensor": "artec_spider",
        "date": datetime.date.today().isoformat(),
        "method": "charuco_calibrateCameraCharuco",
        "n_frames": len(corners),
        "image_size": [int(img_sz[0]), int(img_sz[1])],
        "reprojection_error_px": float(ret),
        "K": [[float(v) for v in row] for row in K],
        "dist": [float(v) for v in dist.flatten().tolist()],
    }, default_flow_style=None, allow_unicode=True), encoding="utf-8")
    print(f"\n[done] {OUTPUT_YAML}")
    return 0


if __name__ == "__main__":
    # ★ 실패는 **exit code 로** 알려야 한다. 예전엔 return 만 해서 frame 부족으로
    #   죽어도 exit 0 이었고, calibrate.py 가 "✓ 완료" 로 보고 다음 단계로 넘어갔다
    #   → hand-eye 가 **구 스캐너의 intrinsic** 을 그대로 써버렸다 (2026-09-15 현장).
    #   조용한 실패가 가장 위험하다.
    sys.exit(main() or 0)
