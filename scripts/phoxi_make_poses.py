#!/usr/bin/env python3
"""
scripts/phoxi_make_poses.py

후보 EE 포즈로 로봇을 이동시켜 PhoXi로 마커 가시성을 자동 검증한다.
통과 조건: Photoneo RecognizeMarkers 성공 (T_M_S 반환)
통과 포즈는 config/calibration/calibration_poses.yaml에 ee_pose 형식으로 자동 추가.

마커 감지: PhoxiClient.detect_marker_transform() — Photoneo GenTL 내장 사용.

Usage
-----
  python scripts/phoxi_make_poses.py

설정 (아래 CONFIG 섹션 참고)
-----------------------------
  ROBOT_IP      : xArm7 IP
  MARKER_POS_MM : 마커보드 중심 위치 [x, y, z] mm (Base frame)
  BASE_RPY_DEG  : 마커보드면을 바라볼 때의 EE 방향 [roll, pitch, yaw] deg
  Z_EE_MM       : 기준 EE 높이 (마커까지 수직거리 = MARKER_POS_MM[2] - Z_EE_MM)
"""

from __future__ import annotations

import sys
import time
from pathlib import Path

import numpy as np
import yaml
from scipy.spatial.transform import Rotation as R

_PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_PROJECT_ROOT))

from mms.robot.xarm_interface import XArmInterface
from mms.sensor.phoxi.phoxi_client import PhoxiClient, PhoxiConfig

# ==============================================================================
# CONFIG
# ==============================================================================

ROBOT_IP      = "192.168.1.210"
SENSOR_YAML   = str(_PROJECT_ROOT / "config" / "sensor_frames.yaml")
POSES_YAML    = _PROJECT_ROOT / "config" / "calibration" / "calibration_poses.yaml"

MARKER_POS_MM = np.array([600.0, 3.7, 635.0])   # 마커보드 중심 (Base frame, mm)
BASE_RPY_DEG  = [163, -83.4, 20.9]              # 마커보드면을 바라보는 EE 방향
Z_EE_MM       = 300.0                            # 기준 EE 높이 (mm from base/ceiling)

MOVE_SPEED    = 8        # deg/s
SETTLE_S      = 1.0      # 이동 후 진동 대기 (s)


# ==============================================================================
# 후보 포즈 생성
# ==============================================================================
# 기준 방향 R_base에 세계 프레임 delta 회전을 합성해 다양한 EE 방향을 생성한다.
# R_new = R_delta_world * R_base
# tilt(dx,dy)에 비례해 EE x,y 위치를 오프셋해 마커 방향을 유지한다.

_TILTS = [           # (delta_Rx°, delta_Ry°, delta_Rz°)
    (  0,   0,   0),
    ( 20,   0,   0), (-20,   0,   0),
    (  0,  20,   0), (  0, -20,   0),
    (  0,   0,  25), (  0,   0, -25),
    ( 30,  15,  15), (-30,  15, -15),
    ( 20, -20,  20), (-20, -20, -20),
    ( 35,   0,   0), (-35,   0,   0),
    (  0,  30,   0), (  0, -30,   0),
]
_Z_OFFSETS = [-60, 0, 60]   # EE 높이 변형 (mm)


def _build_candidates() -> list[dict]:
    R_base = R.from_euler("xyz", BASE_RPY_DEG, degrees=True)
    cands = []
    idx = 0
    for dz_off in _Z_OFFSETS:
        z = float(Z_EE_MM + dz_off)
        dist = float(MARKER_POS_MM[2] - z)
        for (dx, dy, dz_spin) in _TILTS:
            R_delta = R.from_euler("xyz", [dx, dy, dz_spin], degrees=True)
            R_new = R_delta * R_base
            rpy_new = R_new.as_euler("xyz", degrees=True)
            x_off = -np.sin(np.radians(dy)) * dist
            y_off =  np.sin(np.radians(dx)) * dist
            # numpy scalar → Python float 명시 변환 (yaml.safe_dump 호환)
            pos = [
                round(float(MARKER_POS_MM[0] + x_off), 1),
                round(float(MARKER_POS_MM[1] + y_off), 1),
                round(z, 1),
            ]
            rpy = [round(float(v), 1) for v in rpy_new]
            cands.append({"name": f"scan_{idx:02d}", "ee_pose": pos + rpy})
            idx += 1
    return cands


# ==============================================================================
# YAML 저장
# ==============================================================================

def _append_to_yaml(yaml_path: Path, entries: list[dict]) -> None:
    if yaml_path.exists():
        with open(yaml_path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
    else:
        data = {}
    existing_names = {p["name"] for p in data.get("poses", [])}
    poses = data.get("poses", [])
    added = 0
    for entry in entries:
        if entry["name"] in existing_names:
            print(f"  [skip] {entry['name']} — 이미 존재")
            continue
        poses.append(entry)
        existing_names.add(entry["name"])
        added += 1
    data["poses"] = poses
    with open(yaml_path, "w", encoding="utf-8") as f:
        yaml.dump(data, f, default_flow_style=None, allow_unicode=True)
    print(f"\n[저장] {added}개 포즈 추가 → {yaml_path}")


# ==============================================================================
# main
# ==============================================================================

def main():
    candidates = _build_candidates()

    print("=" * 60)
    print("  scan_calib_poses  —  마커 가시성 스캔")
    print("  마커 감지: Photoneo RecognizeMarkers (GenTL 내장)")
    print("=" * 60)
    print(f"  마커 위치     : {MARKER_POS_MM.tolist()} mm")
    print(f"  기준 EE 방향  : {BASE_RPY_DEG}")
    print(f"  기준 EE 높이  : {Z_EE_MM} mm  "
          f"(마커까지 {MARKER_POS_MM[2]-Z_EE_MM:.0f} mm)")
    print(f"  후보 포즈 수  : {len(candidates)}")
    print()

    robot  = XArmInterface(ip=ROBOT_IP)
    sensor = PhoxiClient(PhoxiConfig(
        sensor_frames_yaml=SENSOR_YAML,
        T_EC_key="T_EC_phoxi",
        serial_number="SEA-023",
        trigger_timeout_s=15.0,
    ))
    sensor.initialize()

    input("\n준비되면 Enter (로봇이 자동 이동합니다)... ")

    passed: list[dict] = []
    results: list[dict] = []

    try:
        robot.enable_motion()

        for idx, cand in enumerate(candidates):
            name = cand["name"]
            ee   = cand["ee_pose"]
            print(f"\n[{idx+1}/{len(candidates)}] {name}  "
                  f"({ee[0]:.0f},{ee[1]:.0f},{ee[2]:.0f})mm  "
                  f"rpy=({ee[3]:.1f},{ee[4]:.1f},{ee[5]:.1f})°")

            # IK
            ee_rad = [ee[0], ee[1], ee[2],
                      np.radians(ee[3]), np.radians(ee[4]), np.radians(ee[5])]
            code_ik, ik_joints = robot.arm.get_inverse_kinematics(
                pose=ee_rad, input_is_radian=True, return_is_radian=False,
            )
            if code_ik != 0:
                print(f"  IK 실패 (code={code_ik})")
                results.append({"name": name, "status": "IK_FAIL"})
                continue

            # 이동 전 에러 상태 확인 및 복구
            _, state = robot.arm.get_state()
            if state not in (0, 1, 2):
                print(f"  [복구] 로봇 state={state} → 에러 클리어 후 재활성화")
                robot.arm.clean_error()
                robot.arm.clean_warn()
                robot.enable_motion()
                time.sleep(0.5)

            # 이동
            code = robot.arm.set_servo_angle(
                angle=ik_joints[:7], speed=MOVE_SPEED,
                is_radian=False, wait=True,
            )
            if code != 0:
                print(f"  이동 실패 (code={code})")
                results.append({"name": name, "status": "MOVE_FAIL"})
                robot.arm.clean_error()
                robot.arm.clean_warn()
                robot.enable_motion()
                continue

            time.sleep(SETTLE_S)

            # ── Photoneo 내장 마커 감지 ──────────────────────────────────
            T_M_S = sensor.detect_marker_transform()
            if T_M_S is None:
                print("  → 마커 감지 실패")
                results.append({"name": name, "status": "DETECT_FAIL"})
                continue

            t_mm = T_M_S[:3, 3]
            print(f"  [PASS]  t=({t_mm[0]:.0f},{t_mm[1]:.0f},{t_mm[2]:.0f})mm")
            passed.append({"name": name, "ee_pose": ee})
            results.append({"name": name, "status": "PASS",
                             "t_mm": t_mm.tolist()})

    finally:
        sensor.shutdown()
        robot.disconnect()

    # ── 결과 요약 ────────────────────────────────────────────────
    print("\n" + "=" * 60)
    print("  스캔 결과 요약")
    print("=" * 60)
    print(f"  {'포즈':10s}  {'상태':14s}  {'마커 위치 (mm)':>30s}")
    print(f"  {'-'*10}  {'-'*14}  {'-'*30}")
    for r in results:
        t_s = (f"({r['t_mm'][0]:.0f},{r['t_mm'][1]:.0f},{r['t_mm'][2]:.0f})"
               if "t_mm" in r else "")
        print(f"  {r['name']:10s}  {r['status']:14s}  {t_s:>30s}")

    print(f"\n  통과: {len(passed)} / {len(candidates)}")

    if not passed:
        print("\n  통과 포즈가 없습니다. 확인 사항:")
        print(f"  1. Z_EE_MM={Z_EE_MM}  (마커까지 {MARKER_POS_MM[2]-Z_EE_MM:.0f}mm)")
        print("     → 마커가 PhoXi 작동 범위(100~600mm)를 벗어났을 수 있음")
        print(f"  2. BASE_RPY_DEG={BASE_RPY_DEG}")
        print("     → 실제로 마커를 바라보는 방향인지 확인")
        print("  3. trigger_timeout_s=15.0 — 마커 미인식 시 fetch가 이 시간 동안 대기함")
        return

    # YAML 저장
    print(f"\n통과 포즈 {len(passed)}개:")
    for p in passed:
        e = p["ee_pose"]
        print(f"  {p['name']}: pos=({e[0]},{e[1]},{e[2]})  rpy=({e[3]},{e[4]},{e[5]})")

    ans = input("\nYAML에 저장하시겠습니까? [y/N]: ").strip().lower()
    if ans == "y":
        _append_to_yaml(POSES_YAML, passed)
    else:
        print("저장 취소.")


if __name__ == "__main__":
    main()
