#!/usr/bin/env python
# scripts/turntable/phase1_speed_rotation.py
#
# Phase 1 과 동일한 속도(rotation_duration_s=30s, vel=2π/30 rad/s)로
# 턴테이블을 한 바퀴(+overshoot) 회전시키는 standalone 스크립트.
#
# 용도: Artec Studio 의 Recording 을 띄워둔 채로 이 스크립트를 실행하면,
# Phase 1 과 똑같은 회전 조건에서 Studio 의 'tracking lost' 검출 동작을
# 비교 확인할 수 있음. 회전 중 손으로 아이템을 가리거나 빼서 Studio 가
# tracking lost 를 잡는지 테스트.
#
# 실행: python scripts/turntable/phase1_speed_rotation.py
# 중단: Ctrl+C — 즉시 stop() + servo OFF + disconnect.

import sys
import time
from pathlib import Path

import numpy as np

_PROJECT_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(_PROJECT_ROOT))

from utils.turntable import Turntable

# ── 하드웨어 ──────────────────────────────────────────────────────────
TURNTABLE_IP    = "192.168.0.10"
TURNTABLE_BD_ID = 0

# ── Phase 1 과 동일 회전 파라미터 (main_artec.py 와 일치) ──────────────
ROTATION_DURATION_S    = 30.0       # 360° 한 바퀴 기대 시간
ROTATION_OVERSHOOT_DEG = 5.0        # 마지막 frame 보장용 여유
ACCEL_DEG_S2           = 180.0      # 가속도/감속도
RESET_VEL_DEG_S        = 30.0       # 0° 복귀 속도
POLL_S                 = 0.05       # 위치 폴링 주기


def connect() -> Turntable:
    tt = Turntable(bd_id=TURNTABLE_BD_ID, ip=TURNTABLE_IP, pulses_per_rev=50000)
    tt.connect(comm_type=0)
    tt.check_drive_info()
    tt.check_drive_err()
    try:
        tt.stop()                                    # 잔여 회전 정리
    except Exception:
        pass
    tt.set_servo_on(True)
    tt.set_acceleration(np.radians(ACCEL_DEG_S2), np.radians(ACCEL_DEG_S2))
    return tt


def _read_pos(tt: Turntable, fallback: float = 0.0) -> float:
    v = tt.getActualPos()
    if isinstance(v, bool) or v is None:
        return float(fallback)
    return float(v)


def reset_to_zero(tt: Turntable) -> None:
    """현재 위치 → 0° 이동. 실패 시 clearpos() 로 현재 위치를 0 기준으로 재설정."""
    cur = _read_pos(tt)
    if abs(cur) < np.radians(0.5):
        print(f"  [reset] 이미 θ≈0° (cur={np.degrees(cur):+.2f}°) — skip")
        return
    print(f"  [reset] cur={np.degrees(cur):+.2f}° → 0°  vel={RESET_VEL_DEG_S}°/s")
    ok = tt.move_abs(0.0, np.radians(RESET_VEL_DEG_S))
    if ok is False:
        print("  [reset] move_abs 거부 — clean_drive_err 후 재시도")
        try:
            tt.check_drive_err()
        except Exception:
            pass
        ok = tt.move_abs(0.0, np.radians(RESET_VEL_DEG_S))
    if ok is False:
        # 누적 카운터가 한참 벗어나 있으면 drive 가 거부 — clearpos 로 0 재설정.
        print("  [reset] move_abs 계속 실패 — clearpos() 로 현재 위치를 0 기준으로 강제 설정")
        tt.clearpos()
        return
    timeout = max(8.0, abs(np.degrees(cur)) / RESET_VEL_DEG_S * 2.0 + 3.0)
    tt.wait_motion_done(timeout_s=timeout)
    final = _read_pos(tt, fallback=cur)
    if abs(final) > np.radians(0.5):
        print(f"  [reset] move 미완료 (최종={np.degrees(final):+.2f}°) — "
              f"clearpos() 로 강제 재설정")
        tt.clearpos()
    else:
        print(f"  [reset] 최종 = {np.degrees(final):+.2f}°")


def main() -> None:
    vel_rad_s   = (2.0 * np.pi) / ROTATION_DURATION_S
    target_rad  = 2.0 * np.pi * (1.0 + ROTATION_OVERSHOOT_DEG / 360.0)
    vel_deg_s   = np.degrees(vel_rad_s)
    target_deg  = np.degrees(target_rad)

    print("═══════════════ Phase1-speed turntable test ═══════════════")
    print(f"  rotation_duration_s : {ROTATION_DURATION_S:.1f}")
    print(f"  vel                 : {vel_deg_s:.2f} °/s ({vel_rad_s:.4f} rad/s)")
    print(f"  target              : {target_deg:.1f}° (overshoot {ROTATION_OVERSHOOT_DEG}°)")
    print(f"  Artec Studio 에서 Recording 시작한 뒤 Enter 눌러 회전 시작.")

    tt = connect()
    try:
        reset_to_zero(tt)

        try:
            input("  >> Studio 에서 Recording 시작했으면 Enter — 회전 시작 ... ")
        except EOFError:
            pass

        # 시작 위치 기록 — 종료 판정은 절대 위치가 아니라 '시작점에서의
        # 이동 거리' 로 한다. drive position 카운터가 누적돼 있어도 안전.
        start_pos = _read_pos(tt)
        print(f"  start_pos = {np.degrees(start_pos):+.2f}°")
        print(f"  [{0:>5.1f}s] move_velocity({vel_rad_s:.4f} rad/s, dir=0)")
        if not tt.move_velocity(vel_rad_s, direction=0):
            print("  ✘ move_velocity 거부 — 종료")
            return

        t0 = time.time()
        last_print_s = -1
        while True:
            elapsed = time.time() - t0
            pos = tt.getActualPos()
            if isinstance(pos, bool) or pos is None:
                time.sleep(POLL_S)
                continue
            pos = float(pos)
            traveled = abs(pos - start_pos)

            if int(elapsed) != last_print_s:
                last_print_s = int(elapsed)
                print(f"  [{elapsed:>5.1f}s] θ={np.degrees(pos):+7.1f}°  "
                      f"traveled={np.degrees(traveled):+6.1f}°")

            if traveled >= target_rad:
                print(f"  ✓ traveled {target_deg:.1f}° 도달 — "
                      f"duration={elapsed:.1f}s")
                break

            # 안전 timeout (정상보다 1.5배)
            if elapsed > ROTATION_DURATION_S * 1.5 + 5.0:
                print(f"  ⚠ timeout {elapsed:.1f}s — 강제 종료 "
                      f"(traveled={np.degrees(traveled):+.1f}°)")
                break

            time.sleep(POLL_S)

    except KeyboardInterrupt:
        print("\n  ⚠ Ctrl+C — 즉시 정지")
    finally:
        try:
            tt.stop()
            print("  [end] turntable.stop() OK")
        except Exception as e:
            print(f"  [end] ⚠ stop 예외: {e}")
        try:
            print("  [end] post-rotation 0° 복귀 ...")
            reset_to_zero(tt)
        except Exception as e:
            print(f"  [end] ⚠ reset 예외 (무시): {e}")
        try:
            tt.set_servo_on(False)
        except Exception:
            pass
        try:
            tt.disconnect()
        except Exception:
            pass


if __name__ == "__main__":
    main()
