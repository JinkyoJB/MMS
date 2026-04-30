import os
import sys
import platform
import time
import numpy as np

current_dir = os.path.dirname(os.path.abspath(__file__))
arch = platform.architecture()[0]

library_path = os.path.join(current_dir, "Eziservo_x64")

if library_path not in sys.path:
    sys.path.append(library_path)
    os.chdir(current_dir) 

try:
    from FAS_EziMOTIONPlusE import *
    from MOTION_DEFINE import *
    from ReturnCodes_Define import *
    from MOTION_EziSERVO2_DEFINE import *
    print(f"[Info] Library load complete. {library_path}")
except ImportError as e:
    print(f"[Error] Library load fail: {e}")


def check_connection(func):
    def wrapper(self, *args, **kwargs):
        if not self.is_connected:
            print(f"[Error] Board ID {self.bd_id}: Turntable is not connected.")
            return False
        return func(self, *args, **kwargs)
    return wrapper

class Turntable:
    def __init__(self, bd_id: int, ip: str = "192.168.0.10", pulses_per_rev: int = 50000):
        self.bd_id = bd_id
        self.ip = ip
        self.is_connected = False
        self.pulses_per_rev = pulses_per_rev
        self.TCP = 0
        self.UDP = 1
        # 모션 파라미터 (단위: rad/s^2)
        self.default_accel = np.pi  # 기본 가속도 (3.14 rad/s^2)
        self.default_decel = np.pi  # 기본 감속도
        
        # MOTION_OPTION_EX 구조체 초기화
        self.motion_opt = MOTION_OPTION_EX()
        self.motion_opt.BIT_USE_CUSTOMACCEL = 1  # 가속 시간 커스텀 플래그 활성화
        self.motion_opt.BIT_USE_CUSTOMDECEL = 1  # 감속 시간 커스텀 플래그 활성화
        self.motion_opt.BIT_IGNOREEXSTOP = 0
        
        # 속도 운전용 옵션 (Velocity/Jog)
        self.vel_opt = VELOCITY_OPTION_EX()
        self.vel_opt.BIT_USE_CUSTOMACCDEC = 1

    def set_acceleration(self, accel_rad_s2: float, decel_rad_s2: float = None):
        # 사용자가 원하는 물리적 가속도와 감속도를 설정 (rad/s^2)
        self.default_accel = accel_rad_s2
        self.default_decel = decel_rad_s2 if decel_rad_s2 is not None else accel_rad_s2
        print(f"Set Target Acceleration: {self.default_accel} rad/s^2")
    
    def _calculate_accel_time_ms(self, target_vel_rad_s: float, accel_rad_s2: float) -> int:
        # v = a * t 공식을 이용하여 ms 단위 가속 시간을 계산
        if accel_rad_s2 <= 0 or target_vel_rad_s <= 0:
            return 100 # 안전을 위한 기본값
            
        # t (sec) = v / a
        time_sec = abs(target_vel_rad_s) / accel_rad_s2
        # ms로 변환 (최소 1ms, 최대 9999ms 제한)
        return max(1, min(9999, int(time_sec * 1000)))

    def _rad_to_pulse(self, rad: float) -> int:
        return int((self.pulses_per_rev * rad) / (np.pi * 2))

    def _rads_to_pps(self, rad_s: float) -> int:
        # 각속도(rad/s)를 PPS(Pulse Per Second)로 변환
        return int((self.pulses_per_rev * rad_s) / (np.pi * 2))

    def set_accel_time(self, ms: int):
        # 위치 이동 시 목표 속도까지 도달하는 가속 시간 설정 (1~9999ms) [cite: 198]
        self.motion_opt.wCustomAccelTime = max(1, min(9999, int(ms)))
        print(f"Accel Time set to {self.motion_opt.wCustomAccelTime} ms")

    def set_decel_time(self, ms: int):
        # 위치 이동 시 정지까지 걸리는 감속 시간 설정 (1~9999ms) [cite: 198]
        self.motion_opt.wCustomDecelTime = max(1, min(9999, int(ms)))
        print(f"Decel Time set to {self.motion_opt.wCustomDecelTime} ms")

    def set_jog_accel_time(self, ms: int):
        # 속도 운전(Jog) 시 가감속 시간 설정 [cite: 126]
        self.vel_opt.wCustomAccDecTime = max(1, min(9999, int(ms)))
        print(f"Jog Acc/Dec Time set to {self.vel_opt.wCustomAccDecTime} ms")

    def connect(self, comm_type: int = 0) -> bool:
        # 모터 드라이브에 연결 (0: TCP, 1: UDP)
        ip_parts = list(map(int, self.ip.split('.')))
        
        if comm_type == self.TCP:
            result = FAS_ConnectTCP(ip_parts[0], ip_parts[1], ip_parts[2], ip_parts[3], self.bd_id)
        else:
            result = FAS_Connect(ip_parts[0], ip_parts[1], ip_parts[2], ip_parts[3], self.bd_id)
            
        if result != 0:
            self.is_connected = True
            print(f"Board ID {self.bd_id}: Connect Success ({self.ip})")
            return True
        else:
            print(f"Board ID {self.bd_id}: Connect fail")
            return False

    def disconnect(self):
        # 연결 해제
        FAS_Close(self.bd_id)
        self.is_connected = False
        print(f"Board ID {self.bd_id}: Disconnect")

    def reconnect(self, settle_s: float = 0.5) -> bool:
        """
        TCP 통신 timeout / drive 일시 fault 후 회복용.
        Disconnect → wait → Connect → ServoOn → ErrorClear.
        """
        print(f"[Turntable] reconnect 시도 ...")
        try:
            FAS_Close(self.bd_id)
        except Exception:
            pass
        self.is_connected = False
        time.sleep(settle_s)
        if not self.connect(comm_type=self.TCP):
            print(f"[Turntable] reconnect 실패")
            return False
        time.sleep(settle_s)
        try:
            self.set_servo_on(True)
        except Exception:
            pass
        try:
            self.check_drive_err()
        except Exception:
            pass
        print(f"[Turntable] reconnect OK")
        return True

    @check_connection
    def check_drive_info(self):
        result, byType, version = FAS_GetSlaveInfo(self.bd_id)
        if result == FMM_OK:
            print(f"Board ID {self.bd_id}: TYPE={byType}, Version={version}")
            return True
        return False

    @check_connection
    def check_drive_err(self) -> bool:
        # 드라이브 이상 유무 확인
        res, status = FAS_GetAxisStatus(self.bd_id)
        if res != FMM_OK: return False
        if status & EZISERVO2_AXISSTATUS.FFLAG_ERRORALL:
            print(f"Board ID {self.bd_id}: Error detected. Try reset...")
            if FAS_ServoAlarmReset(self.bd_id) == FMM_OK:
                time.sleep(0.5)
                return True
            return False
        return True

    @check_connection
    def set_servo_on(self, state: bool = True) -> bool:
        # 모터 서보 온
        val = 1 if state else 0
        if FAS_ServoEnable(self.bd_id, val) != FMM_OK:
            return False
        res, status = FAS_GetAxisStatus(self.bd_id)
        if (status & EZISERVO2_AXISSTATUS.FFLAG_SERVOON) == (val << 0): # 비트 확인 logic
            print(f"Board ID {self.bd_id}: Servo {'ON' if state else 'OFF'}")
            return True
        time.sleep(0.001)

    @check_connection
    def GetAxisStatus(self):
        # 모터 모션 수행 중인지 확인용
        status_result, axis_status = FAS_GetAxisStatus(self.bd_id)
        if status_result != FMM_OK:
            print("Function(FAS_GetAxisStatus) was failed.")
            return False
        return status_result, axis_status
    
    @check_connection
    def wait_motion_done(
        self,
        timeout_s: float = 30.0,
        start_timeout_s: float = 1.0,
        stable_reads: int = 5,
        stable_poll_s: float = 0.010,
    ):
        """
        2단계 대기.

        Stage 1 — 모션이 실제로 '시작' 될 때까지 기다림
            move_abs 직후 드라이버가 명령을 반영하기 전 첫 GetAxisStatus 가 불리면
            (MOTIONING=0, INPOSITION=1) 이라 즉시 "done" 으로 오판하는 경우를 방지.
            MOTIONING=1 또는 INPOSITION=0 이 보일 때까지 대기.

        Stage 2 — 정지 (MOTIONING=0 AND INPOSITION=1) 까지 대기.

        Parameters
        ----------
        timeout_s       : 전체 타임아웃 (s)
        start_timeout_s : stage 1 타임아웃 (s). 초과 시 stage 2 로 진행.
        """
        t_start = time.perf_counter()

        # ── Stage 1: 모션 시작 확인 ────────────────────────────────────
        started = False
        while (time.perf_counter() - t_start) < start_timeout_s:
            time.sleep(0.005)
            status = self.GetAxisStatus()
            if not isinstance(status, tuple):
                continue
            _, axis_status = status
            moving = bool(axis_status & EZISERVO2_AXISSTATUS.FFLAG_MOTIONING)
            in_pos = bool(axis_status & EZISERVO2_AXISSTATUS.FFLAG_INPOSITION)
            if moving or (not in_pos):
                started = True
                break
        if not started:
            print(f"[Turntable] motion 시작 안 됨 ({start_timeout_s:.2f}s) — "
                  f"이미 target 에 있거나 명령 미반영 가능")

        # ── Stage 2: 정지 + 연속 안정성 확인 ───────────────────────────
        # "INPOSITION=1 AND MOTIONING=0" 이 `stable_reads` 회 연속 관측돼야 통과.
        # 한 번만 보고 리턴하면 기계적 settling 중인 상태로 다음 단계 진행돼 캡처 흔들림.
        fail_streak = 0
        stable_count = 0
        while True:
            time.sleep(stable_poll_s)
            status = self.GetAxisStatus()
            if not isinstance(status, tuple):
                fail_streak += 1
                if fail_streak >= 20 or (time.perf_counter() - t_start) > timeout_s:
                    print(f"[Turntable] wait_motion_done: axis status 실패 "
                          f"({fail_streak}회) — 포기")
                    return False
                time.sleep(0.02)
                continue
            fail_streak = 0
            _, axis_status = status

            if (not (axis_status & EZISERVO2_AXISSTATUS.FFLAG_MOTIONING) and
                (axis_status & EZISERVO2_AXISSTATUS.FFLAG_INPOSITION)):
                stable_count += 1
                if stable_count >= stable_reads:
                    print(f"Move Done. (stable={stable_count}×{stable_poll_s*1000:.0f}ms)")
                    return True
            else:
                stable_count = 0

            if (time.perf_counter() - t_start) > timeout_s:
                print(f"[Turntable] wait_motion_done: timeout {timeout_s:.1f}s")
                return False
    
    # --- 데이터 읽기 및 초기화 ---

    @check_connection
    def getActualPos(self):
        # Position 읽기
        status_result, lActualPos = FAS_GetActualPos(self.bd_id)
        if status_result != FMM_OK:
            print("Funtion(FAS_GetActualPos) was failed.")
            return False
        return (lActualPos/self.pulses_per_rev)*(np.pi*2)
    
    @check_connection
    def getActualVel(self):
        # 각속도 읽기
        status_result, lActualVel = FAS_GetActualVel(self.bd_id)
        if status_result != FMM_OK:
            print("Funtion(FAS_GetActualPos) was failed.")
            return False
        return (lActualVel/self.pulses_per_rev)*(np.pi*2)
    @check_connection
    def clearpos(self):
        # 현재 Position을 기준으로 0으로 초기화
        status_result=FAS_ClearPosition(self.bd_id)
        if status_result != FMM_OK:
            print("Funtion(FAS_ClearPosition) was failed.")
            return False
        print("Cleared position value.")
        return True
    
    # --- 동작 함수 ---
    @check_connection
    def move_velocity(self, vel_rad_s: float, direction: int) -> bool:
        # 설정된 가속도를 유지하며 지정한 속도로 회전 시작
        acc_ms = self._calculate_accel_time_ms(vel_rad_s, self.default_accel)
        self.vel_opt.wCustomAccDecTime = acc_ms
        
        pps = int((self.pulses_per_rev * abs(vel_rad_s)) / (np.pi * 2))
        
        if FAS_MoveVelocityEx(self.bd_id, pps, direction, self.vel_opt) == FMM_OK:
            return True
        return False

    @check_connection
    def move_abs(self, pos_rad: float, vel_rad_s: float) -> bool:
        # 설정된 가속도를 유지하며 절대 위치로 이동
        acc_ms = self._calculate_accel_time_ms(vel_rad_s, self.default_accel)
        dec_ms = self._calculate_accel_time_ms(vel_rad_s, self.default_decel)

        self.motion_opt.wCustomAccelTime = acc_ms
        self.motion_opt.wCustomDecelTime = dec_ms

        pos_pulse = int((self.pulses_per_rev * pos_rad) / (np.pi * 2))
        vel_pps = int((self.pulses_per_rev * vel_rad_s) / (np.pi * 2))

        # Alarm 자동 리셋 시도 — 이전 명령에서 오류가 쌓여 있을 수 있음
        self.check_drive_err()

        ret = FAS_MoveSingleAxisAbsPosEx(
            self.bd_id, pos_pulse, vel_pps, self.motion_opt,
        )
        if ret != FMM_OK:
            print(f"[Turntable] move_abs 실패 (return={ret}) — "
                  f"target={np.degrees(pos_rad):+.2f}°  vel={np.degrees(vel_rad_s):.1f}°/s")
            return False
        return True

    
    @check_connection
    def move_inc(self, pos_rad: float, vel_rad_s: float) -> bool:
        # 설정된 가속도를 유지하며 상대 위치로 이동
        acc_ms = self._calculate_accel_time_ms(vel_rad_s, self.default_accel)
        dec_ms = self._calculate_accel_time_ms(vel_rad_s, self.default_decel)
        
        # 하드웨어 옵션의 속성명을 정확히 입력 (wCustomAccelTime 등)
        self.motion_opt.wCustomAccelTime = acc_ms
        self.motion_opt.wCustomDecelTime = dec_ms
        
        pos_pulse = int((self.pulses_per_rev * pos_rad) / (np.pi * 2))
        vel_pps = int((self.pulses_per_rev * vel_rad_s) / (np.pi * 2))

        FAS_MoveSingleAxisIncPosEx(self.bd_id, pos_pulse, vel_pps, self.motion_opt)

        # # 예제 코드 110페이지 참조: Ex 함수 호출
        # if FAS_MoveSingleAxisIncPosEx(self.bd_id, pos_pulse, vel_pps, self.motion_opt) == FMM_OK:
        #     return self.wait_motion_done()


    @check_connection
    def stop(self):
        # 현재 운전 중인 모터의 정지 요청
        return FAS_MoveStop(self.bd_id) == FMM_OK
    

if __name__ == "__main__":
    import numpy as np
    import time


    RAD2DEG = 180.0 / np.pi
    DEG2RAD = np.pi / 180.0
    # 1. 초기화 (ID 0번, 192.168.0.X 대역 고정IP 사용해야 하므로 참고)
    motor = Turntable(bd_id=0, ip="192.168.0.10", pulses_per_rev=50000) # 50000pulse 당 1바퀴

    # 2. 연결
    motor.connect(comm_type=0) # 0:TCP, 1:UDP


    # 3. 준비 (정보 확인 및 에러 리셋)
    motor.check_drive_info()
    motor.check_drive_err()
    motor.set_servo_on(True) # 서보 온
    # motor.set_acceleration(360*DEG2RAD,360*DEG2RAD) 세팅하지 않을 경우 기본 가감속은 3.14 rad/s^2

    print("상대 각도 이동 테스트")
    rpose=20*DEG2RAD
    rvel=20*DEG2RAD
    motor.set_acceleration(180*DEG2RAD,180*DEG2RAD) # 각가속/감속 설정 0.5pi rad/s^2
    motor.move_inc(rpose, rvel)
    motor.disconnect()