#!/usr/bin/env python
"""
실물 장비 3종 연결 점검 — 읽기 전용.

    python scripts/check_devices.py

로봇을 움직이지 않는다. xArm 은 motion_enable/set_state 를 부르지 않고
상태만 읽는다 (set_state(0) 은 컨트롤러를 이전 명령 위치로 resume 시켜
로봇이 실제로 움직일 수 있다 — utils/robot/xarm_interface.py 주석 참고).
턴테이블도 servo on / 모션 명령 없이 상태만 읽는다.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

ROBOT_IP     = os.environ.get("MMS_ROBOT_IP", "192.168.1.210")
TURNTABLE_IP = os.environ.get("MMS_TURNTABLE_IP", "192.168.0.10")
TURNTABLE_BD = int(os.environ.get("MMS_TURNTABLE_BD", "0"))

OK, FAIL = "  [OK]  ", "  [FAIL]"


def check_scanner() -> bool:
    print("\n== Artec Spider ==")
    try:
        from mms_artec.sensor import artec_base
        artec_base._load()                      # SDK bin-x64 를 DLL 경로에 등록
        import artec_sdk_py
        found = artec_sdk_py.enumerate_scanners()
    except Exception as e:
        print(f"{FAIL} 바인딩 로드 실패: {type(e).__name__}: {e}")
        print("        → docs/install.md §3 (cmake 빌드)")
        return False
    if not found:
        print(f"{FAIL} 스캐너를 찾지 못했다 (USB 케이블 / 전원 확인)")
        return False
    for s in found:
        print(f"{OK} {s['serial']}  texture={s['has_texture_camera']}  license={s['license']}")
    return True


def check_robot() -> bool:
    print(f"\n== xArm7 @ {ROBOT_IP} ==")
    try:
        from xarm.wrapper import XArmAPI
    except Exception as e:
        print(f"{FAIL} xArm SDK import 실패: {e}")
        return False
    arm = None
    try:
        arm = XArmAPI(ROBOT_IP, is_radian=True)
        if not arm.connected:
            print(f"{FAIL} 연결 안 됨 (IP / 전원 / 네트워크 확인)")
            return False
        print(f"{OK} connected   firmware={arm.version_number}")

        # 상태 — 움직이지 않는다
        print(f"         state={arm.state}  mode={arm.mode}  "
              f"error={arm.error_code}  warn={arm.warn_code}")
        if arm.error_code:
            print(f"         ⚠ error_code={arm.error_code} — 움직이기 전에 clear_error() 필요")

        code, angles = arm.get_servo_angle(is_radian=True)
        if code == 0:
            print(f"{OK} joint(rad) = [{', '.join(f'{a:.3f}' for a in angles[:7])}]")
        else:
            print(f"{FAIL} get_servo_angle code={code}")
            return False

        code, pose = arm.get_position(is_radian=True)
        if code == 0:
            print(f"{OK} TCP        = x={pose[0]:.1f} y={pose[1]:.1f} z={pose[2]:.1f} mm  "
                  f"rpy=({pose[3]:.3f}, {pose[4]:.3f}, {pose[5]:.3f}) rad")
        else:
            print(f"{FAIL} get_position code={code}")
            return False
        return True
    except Exception as e:
        print(f"{FAIL} {type(e).__name__}: {e}")
        return False
    finally:
        if arm is not None:
            try:
                arm.disconnect()
            except Exception:
                pass


def check_turntable() -> bool:
    print(f"\n== 턴테이블 Ezi-SERVO @ {TURNTABLE_IP} ==")
    cwd = os.getcwd()
    try:
        from utils.turntable.turntable_interface import Turntable
    except Exception as e:
        print(f"{FAIL} 벤더 라이브러리 로드 실패: {type(e).__name__}: {e}")
        return False
    finally:
        os.chdir(cwd)        # 모듈이 import 시 chdir 한다

    tt = Turntable(bd_id=TURNTABLE_BD, ip=TURNTABLE_IP)
    # ★ UDP(1). TCP 는 지속 polling 시 socket 이 막힌다 (README §환경)
    if not tt.connect(comm_type=1):
        print(f"{FAIL} 연결 실패 — IP / 전원 / 공유기 확인")
        return False
    try:
        tt.check_drive_info()
        res = tt.GetAxisStatus()
        if not res:
            print(f"{FAIL} GetAxisStatus 실패")
            return False
        _, status = res
        from utils.turntable.Eziservo_x64.MOTION_EziSERVO2_DEFINE import EZISERVO2_AXISSTATUS as S
        flags = [n for n in dir(S) if n.startswith("FFLAG_") and status & getattr(S, n)]
        print(f"{OK} status=0x{status:08X}  {' '.join(flags) or '(none)'}")
        if status & S.FFLAG_ERRORALL:
            print("         ⚠ 드라이브 에러 — ServoAlarmReset 필요")
            return False
        print(f"{OK} pos={tt.getActualPos():.4f} rad  vel={tt.getActualVel():.4f} rad/s")
        return True
    finally:
        tt.disconnect()


if __name__ == "__main__":
    results = {
        "scanner":   check_scanner(),
        "robot":     check_robot(),
        "turntable": check_turntable(),
    }
    print("\n" + "=" * 46)
    for k, v in results.items():
        print(f"  {k:<10} {'OK' if v else 'FAIL'}")
    print("=" * 46)
    sys.exit(0 if all(results.values()) else 1)
