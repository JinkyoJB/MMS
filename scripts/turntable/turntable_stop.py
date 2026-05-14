#!/usr/bin/env python
# scripts/artec/turntable_stop.py
#
# 턴테이블 즉시 정지 + 서보 OFF + 연결 해제 (Artec 파이프라인용).
# main_artec.py 가 비정상 종료해서 모터가 계속 돌 때 사용.

import sys
from pathlib import Path

_PROJECT_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(_PROJECT_ROOT))

from utils.turntable import Turntable

TURNTABLE_IP = "192.168.0.10"
TURNTABLE_BD_ID = 0


def main():
    tt = Turntable(bd_id=TURNTABLE_BD_ID, ip=TURNTABLE_IP, pulses_per_rev=50000)
    print("[stop] connect ...")
    if not tt.connect(comm_type=0):
        print("[stop] ✘ connect 실패. 드라이버 전원 확인.")
        return
    try:
        print(f"[stop] turntable.stop() → {tt.stop()}")
    except Exception as e:
        print(f"[stop] ⚠ stop 예외: {e}")
    try:
        tt.set_servo_on(False); print("[stop] servo OFF")
    except Exception as e:
        print(f"[stop] ⚠ servo off 실패: {e}")
    tt.disconnect()
    print("[stop] 완료")


if __name__ == "__main__":
    main()
