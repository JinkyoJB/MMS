#!/usr/bin/env python3
"""
scripts/phoxi_hand_eye_validate.py

Hand-Eye 캘리브레이션 결과 검증 스크립트 (PhoXi).

원리
----
캘리브레이션이 정확하면, 고정된 마커 보드를 여러 로봇 자세에서 촬영했을 때
Base 프레임 기준 마커 포즈 T_B_M(i) 가 모든 i에서 일정해야 한다.

  T_B_M(i) = T_B_E(i) · T_E_S · inv(T_M_S(i))

일관성 오차(Consistency Error):
  t_err : T_B_M translation 의 std dev (mm)  — 목표 < 1.0 mm
  r_err : T_B_M rotation 의 angle std dev (deg)

Usage
-----
  python scripts/phoxi_hand_eye_validate.py

  로봇을 수동으로 원하는 자세로 이동시킨 후 Enter → 샘플 캡처.
  'q' + Enter → 결과 출력.
"""

from __future__ import annotations

import logging
import sys
from pathlib import Path

import numpy as np
import yaml
from scipy.spatial.transform import Rotation as ScipyR

_PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_PROJECT_ROOT))

from mms.robot.xarm_interface import XArmInterface
from mms.sensor.phoxi.phoxi_client import PhoxiClient, PhoxiConfig

# ==============================================================================
# CONFIG
# ==============================================================================

ROBOT_IP    = "192.168.1.210"
CALIB_YAML  = _PROJECT_ROOT / "config" / "calibration" / "hand_eye_phoxi.yaml"


# ==============================================================================
# helpers
# ==============================================================================

def _load_T_E_S(yaml_path: Path) -> np.ndarray:
    data = yaml.safe_load(yaml_path.read_text(encoding="utf-8"))
    mat = data["T_E_C"]["matrix"]
    T = np.array(mat, dtype=np.float64)
    assert T.shape == (4, 4), "matrix 필드가 4x4가 아닙니다."
    return T


def _fk_matrix(robot: XArmInterface) -> np.ndarray:
    """현재 EE 포즈 → T_B_E (4×4, translation in meters)."""
    pose6 = robot.get_pose(is_radian=False)   # [x,y,z mm, roll,pitch,yaw deg]
    R = ScipyR.from_euler("xyz", np.radians(pose6[3:6])).as_matrix()
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3]  = np.array(pose6[:3]) / 1000.0
    return T


def _rotation_angle(R: np.ndarray) -> float:
    """회전행렬 R 의 회전각 (deg)."""
    cos_val = np.clip((np.trace(R) - 1) / 2, -1.0, 1.0)
    return float(np.degrees(np.arccos(cos_val)))


def _mean_rotation(Rs: list[np.ndarray]) -> np.ndarray:
    """회전행렬 목록의 평균 회전 (Chordal mean)."""
    M = np.zeros((3, 3))
    for R in Rs:
        M += R
    U, _, Vt = np.linalg.svd(M / len(Rs))
    R_mean = U @ Vt
    if np.linalg.det(R_mean) < 0:
        U[:, -1] *= -1
        R_mean = U @ Vt
    return R_mean


# ==============================================================================
# main
# ==============================================================================

def main() -> None:
    logging.basicConfig(
        level=logging.WARNING,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )

    if not CALIB_YAML.exists():
        print(f"[ERROR] 캘리브레이션 파일 없음: {CALIB_YAML}")
        print("  먼저 scripts/phoxi_hand_eye_calib.py 를 실행하세요.")
        return

    T_E_S = _load_T_E_S(CALIB_YAML)
    calib_meta = yaml.safe_load(CALIB_YAML.read_text(encoding="utf-8"))
    print("=" * 60)
    print("  Hand-Eye Calibration Validator  (PhoXi)")
    print("=" * 60)
    print(f"  캘리브레이션 파일 : {CALIB_YAML}")
    print(f"  sensor={calib_meta.get('sensor')}  "
          f"date={calib_meta.get('date')}  "
          f"method={calib_meta.get('method')}  "
          f"n_poses={calib_meta.get('n_poses')}")
    print(f"  T_E_S translation : {(T_E_S[:3,3]*1000).round(2).tolist()} mm")
    print()

    robot  = XArmInterface(ip=ROBOT_IP)
    sensor = PhoxiClient(PhoxiConfig(serial_number="SEA-023", trigger_timeout_s=15.0))
    sensor.initialize()

    T_B_M_list: list[np.ndarray] = []

    print("로봇을 마커 보드가 보이는 자세로 수동 이동 후 Enter → 캡처.")
    print("권장: 10개 이상, 다양한 방향(틸트/좌우/거리)에서 캡처.")
    print("'q' + Enter → 검증 결과 출력.\n")

    try:
        while True:
            key = input(f"[{len(T_B_M_list)+1}] Enter 캡처 / q 종료 > ").strip().lower()
            if key == "q":
                break

            T_B_E = _fk_matrix(robot)
            T_M_S = sensor.detect_marker_transform()

            if T_M_S is None:
                print("  [!] 마커 감지 실패 — 건너뜀\n")
                continue

            T_M_S_m = T_M_S.copy()
            T_M_S_m[:3, 3] /= 1000.0                       # mm → m

            # T(M→B) = FK @ T(S→E) @ T(M→S) = FK @ inv(T_E_S) @ T_M_S
            T_B_M = T_B_E @ np.linalg.inv(T_E_S) @ T_M_S_m
            T_B_M_list.append(T_B_M)

            t_mm = T_B_M[:3, 3] * 1000
            print(f"  T_B_M t=({t_mm[0]:.1f}, {t_mm[1]:.1f}, {t_mm[2]:.1f}) mm\n")

    finally:
        sensor.shutdown()
        robot.disconnect()

    # ── 결과 ────────────────────────────────────────────────────────
    n = len(T_B_M_list)
    print(f"\n{'='*60}")
    print(f"  캡처 샘플 수: {n}")

    if n < 2:
        print("  샘플 2개 이상 필요합니다.")
        return

    translations = np.array([T[:3, 3] for T in T_B_M_list]) * 1000  # mm
    rotations    = [T[:3, :3] for T in T_B_M_list]

    t_mean  = translations.mean(axis=0)
    t_std   = translations.std(axis=0)
    t_err   = float(np.linalg.norm(t_std))    # 3축 합산 std (mm)

    R_mean  = _mean_rotation(rotations)
    r_angles = [_rotation_angle(R_mean.T @ R) for R in rotations]
    r_err   = float(np.std(r_angles))

    print(f"\n  [Translation consistency]")
    print(f"    mean  : ({t_mean[0]:.2f}, {t_mean[1]:.2f}, {t_mean[2]:.2f}) mm")
    print(f"    std   : ({t_std[0]:.3f}, {t_std[1]:.3f}, {t_std[2]:.3f}) mm")
    print(f"    t_err : {t_err:.3f} mm  (목표 < 1.0 mm)")
    print(f"\n  [Rotation consistency]")
    print(f"    angles from mean : {[round(a,3) for a in r_angles]} deg")
    print(f"    r_err            : {r_err:.3f} deg  (목표 < 0.5 deg)")

    print()
    if t_err < 1.0 and r_err < 0.5:
        print("  결과: PASS ✓")
    else:
        flags = []
        if t_err >= 1.0:
            flags.append(f"t_err={t_err:.2f}mm ≥ 1.0mm")
        if r_err >= 0.5:
            flags.append(f"r_err={r_err:.3f}deg ≥ 0.5deg")
        print(f"  결과: FAIL — {', '.join(flags)}")
        print("  → 포즈 다양성 부족 또는 마커 감지 품질 확인 필요")


if __name__ == "__main__":
    main()
