#!/usr/bin/env python3
"""
scripts/hand_eye_calib.py

xArm7 + PhoXi 3D 스캐너 Hand-Eye 캘리브레이션 실행 스크립트.

동작 순서
---------
1. config/calibration_poses.yaml 의 포즈로 로봇 순차 이동
2. 각 포즈에서 PhoxiClient.detect_marker_transform() 호출
   → Photoneo 내장 RecognizeMarkers 로 T_M_S (Marker→Sensor) 획득
3. HandEyeCalibrator.add_sample(T_E_B, T_M_S) 로 샘플 누적
4. HandEyeCalibrator.calibrate() → T_E_S 계산 (AX=XB)
5. config/sensor_frames.yaml 에 T_E_S_phoxi 저장

Usage
-----
  python scripts/hand_eye_calib.py
"""

from __future__ import annotations

import logging
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import yaml
from scipy.spatial.transform import Rotation as ScipyR

_PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_PROJECT_ROOT))

from mms.calibration.hand_eye_calibrator import HandEyeCalibrator
from mms.robot.xarm_interface import XArmInterface
from mms.sensor.phoxi_client import PhoxiClient, PhoxiConfig

# A4-REV-23A board 좌표 파일 (마커 프레임, mm)
_POSITIONS_FILE = Path(
    r"C:\Program Files\Photoneo\PhoXiControl-1.16.5"
    r"\MarkerPatterns\patterns_with_metadata\A4-REV-23A_positions.txt"
)


def _load_board_pts(path: Path) -> np.ndarray:
    """A4-REV-23A_positions.txt → (N, 3) float64, mm, board frame."""
    pts = []
    with open(path) as f:
        for line in f:
            parts = line.strip().split()
            if len(parts) >= 4:
                pts.append([float(parts[1]), float(parts[2]), float(parts[3])])
    return np.array(pts, dtype=np.float64)


def _draw_marker_detections(
    img_gray: np.ndarray,
    marker_pts: np.ndarray,   # (H, W, 3) float32, mm, marker frame
    board_pts: np.ndarray,    # (N, 3) float64, mm, marker frame
    max_dist_mm: float = 8.0,
) -> np.ndarray:
    """
    board_pts(마커 프레임 3D)를 조직화된 포인트 클라우드에서 가장 가까운 픽셀로
    매핑해 intensity 이미지에 초록 원으로 표시한다.

    CoordinateSpace=MarkerSpace일 때 Range 컴포넌트와 board positions.txt 가
    같은 마커 프레임을 공유하므로 직접 비교 가능.
    """
    vis = cv2.cvtColor(img_gray, cv2.COLOR_GRAY2BGR)
    H, W = img_gray.shape[:2]

    flat_pts = marker_pts.reshape(-1, 3)                      # (H*W, 3)
    valid_mask = ~np.all(flat_pts == 0, axis=1)
    valid_pts  = flat_pts[valid_mask]                         # (M, 3)
    valid_idx  = np.where(valid_mask)[0]                      # (M,)

    if len(valid_pts) == 0:
        return vis

    found = 0
    for pt in board_pts:
        dists = np.linalg.norm(valid_pts - pt, axis=1)
        nearest = int(dists.argmin())
        if dists[nearest] > max_dist_mm:
            continue
        flat_i = valid_idx[nearest]
        v, u = divmod(int(flat_i), W)
        cv2.circle(vis, (u, v), radius=6, color=(0, 255, 0), thickness=2)
        found += 1

    cv2.putText(
        vis,
        f"detected {found}/{len(board_pts)} markers",
        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2,
    )
    return vis

# ==============================================================================
# CONFIG
# ==============================================================================

ROBOT_IP       = "192.168.1.210"
POSES_YAML     = _PROJECT_ROOT / "config" / "calibration_poses.yaml"
SENSOR_YAML    = str(_PROJECT_ROOT / "config" / "sensor_frames.yaml")
OUTPUT_YAML    = _PROJECT_ROOT / "config" / "sensor_frames.yaml"
OUTPUT_KEY     = "T_E_S_phoxi"
MOVE_SPEED_DEG = 10     # deg/s
SETTLE_TIME_S  = 2.0    # 이동 후 진동 정착 대기 (s)
MIN_SAMPLES    = 3
DEBUG_DIR      = _PROJECT_ROOT / "debug_calib"


# ==============================================================================
# main
# ==============================================================================

def main() -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )

    # ── 캘리브레이션 포즈 로드 ──────────────────────────────────────
    with open(POSES_YAML, "r", encoding="utf-8") as f:
        poses_data = yaml.safe_load(f)
    pose_list = poses_data.get("poses", [])
    if not pose_list:
        raise RuntimeError(f"poses가 없습니다: {POSES_YAML}")

    print("=" * 60)
    print("  Hand-Eye Calibration  (A4-REV-23A, Eye-in-Hand)")
    print("  마커 감지: Photoneo RecognizeMarkers (GenTL 내장)")
    print("=" * 60)
    print(f"  포즈 파일  : {POSES_YAML}  ({len(pose_list)}개)")
    print(f"  출력 파일  : {OUTPUT_YAML}  key={OUTPUT_KEY}")
    print(f"  이동 속도  : {MOVE_SPEED_DEG} deg/s")
    print(f"  정착 대기  : {SETTLE_TIME_S} s")

    # ── 초기화 ──────────────────────────────────────────────────────
    robot = XArmInterface(ip=ROBOT_IP)
    sensor = PhoxiClient(PhoxiConfig(
        sensor_frames_yaml=SENSOR_YAML,
        T_E_S_key=OUTPUT_KEY,
        serial_number="SEA-023",
        trigger_timeout_s=15.0,
    ))
    sensor.initialize()
    calibrator = HandEyeCalibrator()
    DEBUG_DIR.mkdir(exist_ok=True)
    board_pts = _load_board_pts(_POSITIONS_FILE) if _POSITIONS_FILE.exists() else None
    print(f"  디버그 이미지 → {DEBUG_DIR}")

    print("\n로봇이 각 포즈로 자동 이동합니다.")
    input("준비되면 Enter... ")

    # ── 포즈 순회 ───────────────────────────────────────────────────
    failed: list[str] = []

    try:
        robot.enable_motion()

        for idx, pose_entry in enumerate(pose_list):
            name = pose_entry.get("name", f"pose_{idx}")

            # 포즈 → joint angles (deg)
            if "joints" in pose_entry:
                joints_deg = pose_entry["joints"]
                print(f"\n[{idx+1}/{len(pose_list)}] {name}  joints={joints_deg}")

            elif "ee_pose" in pose_entry:
                ee = pose_entry["ee_pose"]
                ee_rad = [ee[0], ee[1], ee[2],
                          np.radians(ee[3]), np.radians(ee[4]), np.radians(ee[5])]
                print(f"\n[{idx+1}/{len(pose_list)}] {name}  "
                      f"pos=({ee[0]:.1f},{ee[1]:.1f},{ee[2]:.1f})mm  "
                      f"rpy=({ee[3]:.1f},{ee[4]:.1f},{ee[5]:.1f})°")

                code_ik, ik_joints = robot.arm.get_inverse_kinematics(
                    pose=ee_rad, input_is_radian=True, return_is_radian=False,
                )
                if code_ik != 0:
                    print(f"  [!] IK 실패 (code={code_ik}) — 건너뜀")
                    failed.append(name)
                    continue
                joints_deg = ik_joints[:7]
                print(f"  IK  joints={[round(j,1) for j in joints_deg]}")

            else:
                print(f"\n[{idx+1}/{len(pose_list)}] {name}  joints/ee_pose 키 없음 — 건너뜀")
                failed.append(name)
                continue

            # 로봇 에러 복구
            _, state = robot.arm.get_state()
            if state not in (0, 1, 2):
                print(f"  [복구] state={state} → 에러 클리어")
                robot.arm.clean_error()
                robot.arm.clean_warn()
                robot.enable_motion()
                time.sleep(0.5)

            # 이동
            code = robot.arm.set_servo_angle(
                angle=joints_deg, speed=MOVE_SPEED_DEG,
                is_radian=False, wait=True,
            )
            if code != 0:
                print(f"  [!] 이동 실패 (code={code}) — 건너뜀")
                robot.arm.clean_error()
                robot.arm.clean_warn()
                robot.enable_motion()
                failed.append(name)
                continue

            time.sleep(SETTLE_TIME_S)

            # T_E_B (EE→Base, meters)
            pose6 = robot.get_pose(is_radian=False)
            R_EB = ScipyR.from_euler("xyz", np.radians(pose6[3:6])).as_matrix()
            T_E_B = np.eye(4)
            T_E_B[:3, :3] = R_EB
            T_E_B[:3, 3]  = pose6[:3] / 1000.0
            print(f"  TCP  ({pose6[0]:.1f},{pose6[1]:.1f},{pose6[2]:.1f}) mm")

            # Photoneo 마커 감지 → T_M_S (mm)
            # detect_marker_transform()은 내부에서 intensity도 _last_intensity에 저장
            T_M_S = sensor.detect_marker_transform()

            # 디버그: intensity 이미지 저장 (감지 성공/실패 무관)
            if sensor._last_intensity is not None:
                cv2.imwrite(
                    str(DEBUG_DIR / f"{name}_intensity.png"),
                    sensor._last_intensity,
                )

            if T_M_S is None:
                print("  [!] 마커 감지 실패 — 건너뜀")
                failed.append(name)
                continue

            t = T_M_S[:3, 3]

            # 디버그: board 마커 위치를 픽셀로 매핑해 초록 원으로 표시
            if (sensor._last_intensity is not None
                    and sensor._last_marker_pts is not None
                    and board_pts is not None):
                vis = _draw_marker_detections(
                    sensor._last_intensity, sensor._last_marker_pts, board_pts
                )
            elif sensor._last_intensity is not None:
                vis = cv2.cvtColor(sensor._last_intensity, cv2.COLOR_GRAY2BGR)
                cv2.putText(vis, f"T_M_S t=({t[0]:.0f},{t[1]:.0f},{t[2]:.0f})mm",
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            else:
                vis = None
            if vis is not None:
                cv2.imwrite(str(DEBUG_DIR / f"{name}_detected.png"), vis)

            calibrator.add_sample(T_E_B, T_M_S)
            print(f"  ✓ 샘플 {calibrator.n_samples}  "
                  f"marker=({t[0]:.0f},{t[1]:.0f},{t[2]:.0f}) mm")

    finally:
        sensor.shutdown()
        robot.disconnect()

    # ── 결과 ────────────────────────────────────────────────────────
    print(f"\n{'='*60}")
    print(f"  수집: {calibrator.n_samples}/{len(pose_list)} 샘플")
    if failed:
        print(f"  실패: {failed}")

    if calibrator.n_samples < MIN_SAMPLES:
        print(f"\n샘플 부족 ({calibrator.n_samples}/{MIN_SAMPLES}). "
              "calibration_poses.yaml의 포즈를 확인하세요.")
        return

    print(f"\n캘리브레이션 실행 ({calibrator.n_samples} 샘플)...")
    calibrator.calibrate()
    calibrator.save_yaml(OUTPUT_YAML, key=OUTPUT_KEY)
    print("\n완료. config/sensor_frames.yaml 업데이트됨.")


if __name__ == "__main__":
    main()
