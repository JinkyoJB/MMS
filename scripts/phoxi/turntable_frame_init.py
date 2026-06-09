#!/usr/bin/env python3
"""
scripts/phoxi/turntable_frame_init.py

T_B_F0 캘리브레이션 (PhoXi): 턴테이블 rim 클릭 → 3D 원 피팅 → ^B T_F(0) 추정.

★ 알고리즘 코어는 utils/calibration 으로 분리되어 PhoXi·Artec 공용이다:
    utils.calibration.turntable_frame  — fit_circle_3d / build_T_B_F0 / save_*
    utils.calibration.rim_picker       — RimPicker / run_picker / show_3d_result
  이 스크립트는 **PhoXi 전용 부분**(robot/sensor 초기화 + 조직화 점군 캡처)만 갖는다.
  Artec 용은 같은 코어에 Artec 의 (intensity, organized_pts, T_CB) 만 주면 된다.

사전 조건
---------
- hand-eye 완료: config/calibration/hand_eye_phoxi.yaml
- 턴테이블 각도 = 0, 로봇이 rim 이 잘 보이는 위치

조작법: 좌클릭=점추가  우클릭=취소  Enter=피팅  r=재캡처  d=깊이뷰  q=종료
출력: config/calibration/turntable_frame.yaml
"""

from __future__ import annotations

import logging
import sys
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation as ScipyR

_PROJECT_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(_PROJECT_ROOT))

from utils.robot.xarm_interface import XArmInterface
from mms_phoxi.sensor.phoxi_client import PhoxiClient, PhoxiConfig
from utils.transforms import load_transform
from utils.calibration.turntable_frame import (
    fit_circle_3d, build_T_B_F0, save_turntable_frame_yaml,
)
from utils.calibration.rim_picker import RimPicker, run_picker, show_3d_result

# ── CONFIG ─────────────────────────────────────────────────────────────────────
ROBOT_IP       = "192.168.1.210"
HAND_EYE_YAML  = _PROJECT_ROOT / "config" / "calibration" / "hand_eye_phoxi.yaml"
OUTPUT_YAML    = _PROJECT_ROOT / "config" / "calibration" / "turntable_frame.yaml"
WARN_RESIDUAL  = 5.0    # mm — 이 이상이면 품질 경고


def main() -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s")

    print("=" * 60)
    print("  Turntable Frame Init (PhoXi)  —  ^B T_F(0)  rim fitting")
    print("=" * 60)
    print(f"  hand-eye YAML : {HAND_EYE_YAML}")
    print(f"  출력 YAML     : {OUTPUT_YAML}")
    print("\n사전 확인:  1. 턴테이블 각도=0   2. 로봇이 rim 전체가 보이는 위치")
    input("\nEnter 누르면 시작... ")

    if not HAND_EYE_YAML.exists():
        raise FileNotFoundError(f"hand-eye YAML 없음: {HAND_EYE_YAML}")
    T_E_C = load_transform(str(HAND_EYE_YAML), "T_E_C")   # E→C, meters
    T_CE = np.linalg.inv(T_E_C)
    t_mm = T_E_C[:3, 3] * 1000.0
    print(f"\n[TurntableInit] T_E_C 로드  t=({t_mm[0]:.1f},{t_mm[1]:.1f},{t_mm[2]:.1f}) mm")

    robot = XArmInterface(ip=ROBOT_IP)
    sensor = PhoxiClient(PhoxiConfig(serial_number="SEA-023", trigger_timeout_s=15.0))
    sensor.initialize()

    try:
        while True:
            # T_CB: C→B  (C→E→B = T_EB @ T_CE)
            T_E_B = robot.get_ee_pose_mat()
            T_CB = T_E_B @ T_CE
            ee_mm = T_E_B[:3, 3] * 1000.0
            print(f"\n[TurntableInit] 현재 EE  t=({ee_mm[0]:.1f},{ee_mm[1]:.1f},{ee_mm[2]:.1f}) mm")

            print("[TurntableInit] PhoXi 캡처 중...")
            sensor.capture()
            if sensor._last_organized_pts is None or sensor._last_intensity is None:
                print("[!] 조직화된 포인트 클라우드를 얻지 못했습니다")
                input("Enter 누르면 재시도..."); continue

            org_pts = sensor._last_organized_pts      # (H,W,3) float32, mm, 센서 프레임
            intensity = sensor._last_intensity         # (H,W) uint8
            print(f"[TurntableInit] 캡처 완료  {org_pts.shape[1]}×{org_pts.shape[0]} px")

            # ── rim 클릭 (utils 공용 피커) ──────────────────────────────────
            state = RimPicker(intensity, org_pts, T_CB)
            action = run_picker(state)
            if action == "quit":
                print("[TurntableInit] 종료."); return
            if action == "recapture":
                print("[TurntableInit] 재캡처..."); continue

            pts_B = np.array(state.pts_B, dtype=np.float64)   # (N,3) meters

            # ── 원 피팅 (utils 공용) ─────────────────────────────────────────
            print(f"\n[TurntableInit] 3D 원 피팅 ({len(pts_B)}점)...")
            try:
                center_B, normal_B, radius_m, residual_m = fit_circle_3d(pts_B)
            except Exception as e:
                print(f"[!] 피팅 실패: {e} — 점 재선택")
                input("Enter 누르면 계속..."); continue

            radius_mm, residual_mm = radius_m * 1000.0, residual_m * 1000.0
            c_mm = center_B * 1000.0
            print(f"\n  중심(B)={tuple(round(float(v),2) for v in c_mm)} mm  "
                  f"반지름={radius_mm:.2f}mm  RMS={residual_mm:.3f}mm"
                  + ("  ← [경고]" if residual_mm > WARN_RESIDUAL else ""))

            # ── T_B_F0 구성 + 검증 출력 ─────────────────────────────────────
            T_B_F0 = build_T_B_F0(center_B, normal_B)
            T_F_B = np.linalg.inv(T_B_F0)
            z_axis_in_B = T_F_B[:3, 2]
            tilt_deg = float(np.degrees(np.arccos(np.clip(abs(z_axis_in_B[2]), 0.0, 1.0))))
            rpy = ScipyR.from_matrix(T_B_F0[:3, :3]).as_euler("xyz", degrees=True)
            print(f"  F원점(B)={tuple(round(float(v)*1000,2) for v in T_F_B[:3,3])} mm  "
                  f"z틸트={tilt_deg:.2f}°  RPY=({rpy[0]:.2f},{rpy[1]:.2f},{rpy[2]:.2f})")
            if tilt_deg > 5.0:
                print(f"  [경고] z축 틸트 {tilt_deg:.1f}° — 턴테이블 수평 확인")

            # ── 3D 결과 뷰 (utils 공용) ──────────────────────────────────────
            try:
                show_3d_result(org_pts, intensity, T_CB, center_B, normal_B,
                               radius_m, T_B_F0, pts_B)
            except Exception as e:
                print(f"[3D View] 오류: {e}")

            # ── 저장 확인 ────────────────────────────────────────────────────
            ans = input("\n저장할까요? [y=저장 / r=재선택 / n=종료]: ").strip().lower()
            if ans == "y":
                save_turntable_frame_yaml(OUTPUT_YAML, T_B_F0, len(pts_B),
                                          radius_mm, residual_mm)
                print(f"\n[TurntableInit] 완료. {OUTPUT_YAML}"); return
            elif ans == "r":
                print("[TurntableInit] 점 재선택..."); continue
            else:
                print("[TurntableInit] 저장하지 않고 종료."); return
    finally:
        sensor.shutdown()
        robot.disconnect()


if __name__ == "__main__":
    main()
