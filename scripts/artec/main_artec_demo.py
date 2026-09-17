"""
🎬 Artec lookaround 데모 — 중간 보고용 영상 촬영 스크립트.

평소 알고리즘은 "tracking lost 가 안 나면 robot 이 거의 안 움직이는" 게 의도
(docs §3 rule). 영상 보고용으로 보여주려면 동선이 부족해서, 이 스크립트가
**의도적인 헐리우드 액션 인트로** 를 앞에 붙임:

  ① Hollywood intro  (≈ 10초)
     home → 5개 waypoint 큰 호로 swoosh → home 복귀
     → "robot 이 객체를 둘러보며 자세 잡는 듯" 한 cinematic motion

  ② Adaptive pre-position  (≈ 1분, **알고리즘 진짜 동작**)
     - turntable 회전 차분 probe (객체 envelope + (r,z) profile 산출)
     - elevation search **full grid** (-10° / -5° / 0° / +5° / +10° + fine)
       → 7~9 자세 visit + 매 자세 preview·score → best 선택
     → 영상 시청자에게는 "robot 이 객체를 평가하고 best 자세 결정하는"
        퍼포먼스로 보임

  ③ lookaround scan + 후처리
     - turntable 360° 회전 (~30초, robot 고정)
     - GlobalReg / Cleaning / Poisson / Texturize → OBJ
     - 최종 textured mesh viewer

사용:
  python scripts/artec/main_artec_demo.py

기존 main_artec.py 의 CFG / MULTIPASS_SETTINGS / PROCESS_SETTINGS / 안전
정리 흐름을 그대로 재활용. 본 파이프라인 동작에는 영향 없음 (별도 스크립트).
"""

from __future__ import annotations

import sys
import time
import traceback
from pathlib import Path

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT_ROOT))

import main_artec  # noqa: F401  (SDK lib load 효과)
from main_artec import (
    CFG,
    MULTIPASS_SETTINGS,
    PROCESS_SETTINGS,
    ROBOT_IP,
    _show_composite_mesh,
    connect_turntable,
)
from mms_artec.nbv.artec_multipass_scan_session import ArtecMultiPassScanSession
from mms_artec.system import ArtecMMS
from utils.robot.xarm_interface import XArmInterface
from utils.transforms import pose_mat_to_6d


# ── 헐리우드 인트로 설정 ─────────────────────────────────────────────
# TCP delta (mm) from home pose. orientation 은 home 유지 — TCP 위치만 변동.
# 객체를 두고 "갸우뚱하며 살피는" cinematic sweep.
HOLLYWOOD_TCP_DELTAS_MM: list[tuple[int, int, int]] = [
    ( +90,   0, +60),     # 오른쪽 위로 swoosh
    (   0, +90, +30),     # 정면 위
    ( -90,   0, +30),     # 왼쪽 위
    (   0, -90, -20),     # 뒤쪽 살짝 아래
    (   0,   0,   0),     # home 복귀 (adaptive prescan 시작 자세)
]
HOLLYWOOD_SPEED_DEG_S = 25.0     # 빠르고 dynamic
HOLLYWOOD_PAUSE_S = 0.4          # 각 waypoint 에서 잠깐 정지 (영상 호흡)


def hollywood_intro(robot: XArmInterface) -> None:
    """영상미용 grand orbit — home 둘레 큰 호로 swoosh."""
    print("\n══════════════════ 🎬 Hollywood Intro ══════════════════")
    T_EB_home = robot.get_ee_pose_mat()
    p6 = pose_mat_to_6d(T_EB_home)
    x0, y0, z0 = p6[0] * 1000.0, p6[1] * 1000.0, p6[2] * 1000.0
    roll, pitch, yaw = float(p6[3]), float(p6[4]), float(p6[5])
    print(f"  home TCP : x={x0:.0f}  y={y0:.0f}  z={z0:.0f}mm  "
          f"speed={HOLLYWOOD_SPEED_DEG_S:.0f}°/s")
    print(f"  waypoints: {len(HOLLYWOOD_TCP_DELTAS_MM)} ("
          f"각 {HOLLYWOOD_PAUSE_S:.1f}s pose hold)\n")

    for k, (dx, dy, dz) in enumerate(HOLLYWOOD_TCP_DELTAS_MM, 1):
        tag = "home 복귀" if (dx == 0 and dy == 0 and dz == 0) else \
              f"Δ=({dx:+d},{dy:+d},{dz:+d})mm"
        print(f"  WP {k}/{len(HOLLYWOOD_TCP_DELTAS_MM)} : {tag}")
        try:
            robot.enable_motion()
            code = robot.arm.set_position(
                x=x0 + dx, y=y0 + dy, z=z0 + dz,
                roll=roll, pitch=pitch, yaw=yaw,
                is_radian=True, speed=HOLLYWOOD_SPEED_DEG_S, wait=True,
            )
            if code != 0:
                print(f"    ⚠ set_position code={code} — skip")
                continue
            time.sleep(HOLLYWOOD_PAUSE_S)
        except Exception as e:
            print(f"    ✘ 예외: {e}")

    print("\n  ✓ Hollywood intro 완료 — home 자세 복귀\n")


def adaptive_preposition_for_demo(
    mms: ArtecMMS, robot: XArmInterface, turntable,
) -> None:
    """probe + full-grid elevation search.

    평소엔 recovery 흐름에서만 호출되지만 데모용으로 직접 발동
    (recovery=False → 5 coarse + fine = 7~9 candidate visit).
    """
    print("══════════════════ 🔍 Adaptive Pre-position ══════════════════")
    print("  ① turntable 회전 차분 probe — 객체 envelope·(r,z) profile 산출")
    print("  ② elevation search full grid [-10,-5,0,+5,+10]° + fine refine")
    print("  ③ best 자세로 robot 이동 → scan 준비 완료\n")

    # 임시 session — _adaptive_prescan_position 만 호출 (run() 안 함).
    # 그 session 의 결과 robot 자세가 다음 mms.artec_process 의 scan 시작
    # 자세가 됨 (artec_process 가 자기 session 새로 만들 때 _T_BC 재캡처).
    tmp = ArtecMultiPassScanSession(mms, robot, turntable, MULTIPASS_SETTINGS)
    tmp._adaptive_prescan_position(recovery=False)
    print("\n  ✓ Adaptive pre-position 완료 — scan 자세 결정\n")


def main() -> None:
    robot = XArmInterface(ROBOT_IP)
    turntable = connect_turntable()

    try:
        with ArtecMMS(CFG) as mms:
            robot.go_home(sensor="artec", speed=10, confirm=True)

            print("\n╔══════════════════════════════════════════════╗")
            print("║   🎥 Artec lookaround — 데모 모드 (영상 보고용)    ║")
            print("╚══════════════════════════════════════════════╝")
            print(f"  T_EC      : {CFG.T_EC_key}")
            print(f"  fusion    : {PROCESS_SETTINGS.fusion}")
            print(f"  export OBJ: {PROCESS_SETTINGS.export_obj_path}")
            print(f"\n  📋 흐름:")
            print(f"     ① Hollywood intro  (~10s, robot orbit)")
            print(f"     ② Adaptive prescan (~1min, probe + elevation search)")
            print(f"     ③ lookaround scan     (~30s, turntable 360°)")
            print(f"     ④ 후처리 + export OBJ\n")

            # ① 영상미용 orbit
            hollywood_intro(robot)

            # ② probe + elevation search (실제 알고리즘)
            adaptive_preposition_for_demo(mms, robot, turntable)

            # ③+④ 평소 scan + 후처리
            print("══════════════════ 📸 lookaround Scan ══════════════════")
            result = None
            try:
                result = mms.artec_process(
                    robot, turntable, settings=PROCESS_SETTINGS,
                )
                n_frames = sum(
                    result.model.get_scan(i).frame_count()
                    for i in range(result.model.scan_count()))
                print(f"\n[demo] 완료 — frames={n_frames} "
                      f"scans={result.model.scan_count()}")
            except KeyboardInterrupt:
                print("\n[demo] ⚠ KeyboardInterrupt — 현재까지 보존")
            except Exception as e:
                print(f"\n[demo] ✘ {type(e).__name__}: {e}")
                traceback.print_exc()

            if result is not None:
                _show_composite_mesh(
                    result, mms,
                    obj_path=PROCESS_SETTINGS.export_obj_path,
                )

    finally:
        # 안전 정리 (main_artec 와 동일 — turntable stop 필수)
        try:
            turntable.stop()
            print("[demo] turntable.stop() OK")
        except Exception as e:
            print(f"[demo] ⚠ turntable.stop() 예외: {e}")
            try:
                if hasattr(turntable, "reconnect"):
                    turntable.reconnect()
                turntable.stop()
                print("[demo] turntable.stop() 재시도 OK")
            except Exception as e2:
                print(f"[demo] ✘ 최종 stop 실패: {e2} — 물리 전원 차단 필요")
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
