# mms_artec/nbv/artec_streaming_scan_session.py
#
# Artec **IScanningProcedure** (streaming) 기반 Phase 1.
#
# 핵심 설계 — Spider ↔ Turntable 양방향 피드백
# --------------------------------------------
#  - Spider 의 frame_callback 이 매 frame 의 FrameState 를 TrackingState 에 기록
#  - Tracking 잃으면 즉시 stop_event 셋 → turntable thread 가 즉시 정지
#  - Main loop 가 session.poll_events() 를 정기적으로 drain (SDK 큐 freeze 방지)
#  - 정상 360° 도달 시 자연스럽게 종료
#
# 이전 버전의 두 문제 해결:
#  - Spider 가 20s 후 멈춤 → poll_events drain 누락이 원인. 이젠 50ms 마다 drain.
#  - Turntable 이 timeout 무시 → 별도 thread + stop_event 로 즉시 차단.
#
# Notation: T_AB : A → B  (x_B = T_AB @ x_A) — README.md / CLAUDE.md 준수.

from __future__ import annotations

import threading
import time
from dataclasses import dataclass, field
from typing import List, Optional, TYPE_CHECKING

import numpy as np

from mms_artec.sensor import artec_base
from mms_artec.sensor import artec_scanning
from mms_artec.sensor.artec_client import ArtecClient

if TYPE_CHECKING:
    from mms_artec.system import ArtecMMS as MMS
    from utils.robot.xarm_interface import XArmInterface
    from utils.turntable.turntable_interface import Turntable


# ─────────────────────────────────────────────────────────────────────────────
# Settings
# ─────────────────────────────────────────────────────────────────────────────

@dataclass
class ArtecStreamingScanSessionSettings:
    # ── 회전 ──────────────────────────────────────────────────────────
    rotation_duration_s: float = 30.0      # 360° 한 바퀴 기대 시간
    rotation_overshoot_deg: float = 5.0    # 360° + 여유 (마지막 frame 보장)

    # ── 시계열 로그 ───────────────────────────────────────────────────
    # poll cycle 마다 (t, theta, scanning_flag, ...) 기록 → 사후 분석.
    # CSV path 지정 시 종료 시 dump.
    timeline_csv_path: Optional[str] = None

    # ── 스캐너 ────────────────────────────────────────────────────────
    target_fps: Optional[float] = None     # None = max_fps
    capture_texture: bool = True

    # ── 등록 / 파이프라인 ─────────────────────────────────────────────
    # Hybrid (geometry+texture) — Artec Studio 기본값. ICP 만 쓰면 아이템 빠진
    # 뒤에도 빈 턴테이블/배경에 정합 성공시켜 tracking lost 가 안 잡힘.
    registration_type: int = int(artec_scanning.RegistrationType.HYBRID)
    pipeline_flags: int = (
        int(artec_scanning.ScanningPipelineFlags.REGISTER_FRAME) |
        int(artec_scanning.ScanningPipelineFlags.FIND_GEOMETRY_KEYFRAME) |
        int(artec_scanning.ScanningPipelineFlags.MAP_TEXTURE) |
        int(artec_scanning.ScanningPipelineFlags.CONVERT_TEXTURES)
    )
    sensitivity: Optional[float] = None
    scan_range_near_mm: Optional[float] = None
    scan_range_far_mm:  Optional[float] = None
    ignore_registration_errors: bool = True

    # ── 트래킹 손실 정책 ──────────────────────────────────────────────
    # (1) 연속 N 프레임 정합 실패 → tracking_lost.
    consecutive_loss_threshold: int = 8

    # (2) **Frame stall watchdog** — callback 자체가 N초 안 들어오면 lost 판정.
    # SDK silent auto-stop / USB freeze / 하드웨어 fault 등 정합 실패와 다른
    # 종류의 멈춤 (FAILED 도 OK 도 안 옴) 도 잡기 위함.
    # 기본 2.0s — 일반적 FPS=3-15 면 매 프레임이 0.07-0.33s 간격, 2s 무반응이면 freeze.
    stale_threshold_s: float = 2.0

    # (3) **Native registration_error watchdog** — Artec Studio 가 'tracking
    # lost' 표시할 때 보는 SDK 시그널 그 자체.
    #   RegistrationInfo.registrationError < 0  →  registration 실패
    # ignoreRegistrationErrors=True 면 frame 은 scan 에 추가되지만 error 는
    # 그대로 음수로 들어옴. 연속 N 프레임 음수면 lost 처리.
    consecutive_reg_err_threshold: int = 5

    # (4) **양수 임계 워치독** — Hybrid 라도 가끔 SDK 가 빈 배경에 작은 양수
    # error 로 정합 성공을 내릴 수 있음. healthy 구간 reg_err 가 보통
    # 0.05~0.40 정도이니 여기서 명백히 벗어나면 lost.
    # 0 이면 비활성. 기본 1.5 — 1mm 단위 (mean p2plane dist) 근거.
    max_acceptable_reg_error: float = 1.5
    consecutive_high_err_threshold: int = 8

    # ── 프레임 수 제한 ────────────────────────────────────────────────
    # SDK default 100 — 도달하면 silent auto-stop. 0 = unlimited.
    # docs/7_artec_phase1.md §12 의 freeze 원인.
    max_frame_count: int = 0

    # ── 타이밍 ────────────────────────────────────────────────────────
    preview_settle_s: float = 1.5
    post_record_settle_s: float = 0.5
    poll_interval_s: float = 0.05         # main loop 의 event drain 주기

    # ── 기타 ──────────────────────────────────────────────────────────
    reset_to_zero_first: bool = True


# ─────────────────────────────────────────────────────────────────────────────
# TrackingState — Spider/Turntable 공유 상태
# ─────────────────────────────────────────────────────────────────────────────

@dataclass
class TrackingState:
    """
    Frame callback 이 갱신하는 공유 상태. Turntable controller 가 이걸 보고
    rotation 계속 / 정지 결정.

    세 가지 lost 검출:
      (1) 연속 정합/재구성 실패 N회 (FrameState 기반) → tracking_lost
      (2) callback 무반응 N초 (stall) → tracking_lost
          ← SDK silent auto-stop / USB freeze 같은 callback 자체가 끊긴 케이스
      (3) registration_error 가 연속 N회 음수 → tracking_lost
          ← Artec Studio 의 'tracking lost' 시그널과 동일.
            ignoreRegistrationErrors=True 라 FrameState 가 OK 로 들어와도
            error<0 이면 정합 실패로 간주.
    """
    frames_ok: int = 0
    frames_failed: int = 0
    consecutive_lost: int = 0           # 연속 정합/재구성 실패 (FrameState)
    consecutive_reg_err: int = 0        # 연속 registration_error < 0
    consecutive_high_err: int = 0       # 연속 registration_error > max_acceptable
    last_state: Optional[artec_scanning.FrameState] = None
    last_reg_error: float = 0.0         # 가장 최근 frame 의 registration error
    state_counts: dict = field(default_factory=dict)   # FrameState → count
    tracking_lost: bool = False         # 한 번이라도 임계 초과면 True
    last_loss_reason: str = ""
    last_frame_time: float = 0.0        # ★ 마지막 callback 시각 (state 무관)
    started_time: float = 0.0           # record 시작 시각 (stall 검사 기준)
    tracking_established: bool = False  # reg_err>=0 한 번이라도 본 뒤 True
                                        # — 이전엔 SDK warm-up 으로 -1 이 정상

    _lock: threading.Lock = field(default_factory=threading.Lock, repr=False)
    _stop_event: threading.Event = field(default_factory=threading.Event, repr=False)
    _max_reg_error: float = field(default=0.0, repr=False)

    def on_frame(self, event: artec_scanning.FrameEvent) -> None:
        """frame_callback 으로 매 frame 호출 — state 무관 시각 항상 갱신."""
        FS = artec_scanning.FrameState
        with self._lock:
            self.last_frame_time = time.time()
            self.last_state = event.frame_state
            self.last_reg_error = float(event.registration_error)
            name = event.frame_state.name if event.frame_state is not None else "NONE"
            self.state_counts[name] = self.state_counts.get(name, 0) + 1

            # SDK native: registrationError < 0 → registration 실패
            # (Artec Studio 'tracking lost' 와 동일 시그널)
            # 단, scan 시작 직후엔 직전 keyframe 이 없어서 SDK 가 -1.0 을
            # warm-up sentinel 로 내보냄. 한 번이라도 reg_err>=0 (= 정상
            # 정합 성공) 본 뒤부터만 lost 카운트한다.
            if self.last_reg_error >= 0.0:
                self.tracking_established = True
                self.consecutive_reg_err = 0
            elif self.tracking_established:
                self.consecutive_reg_err += 1
            # else: warm-up 단계 — 무시

            # 양수 임계 워치독 — 정합 "성공" 했지만 error 가 너무 크면 lost.
            # warm-up 끝난 뒤에만 카운트.
            if (self.tracking_established and self._max_reg_error > 0.0
                    and self.last_reg_error > self._max_reg_error):
                self.consecutive_high_err += 1
            else:
                self.consecutive_high_err = 0

            if event.frame_state == FS.OK:
                self.frames_ok += 1
                self.consecutive_lost = 0
            else:
                self.frames_failed += 1
                if event.frame_state in (FS.REGISTRATION_FAILED,
                                         FS.RECONSTRUCTION_FAILED,
                                         FS.ADD_TO_SCAN_FAILED):
                    self.consecutive_lost += 1

    def mark_started(self) -> None:
        with self._lock:
            self.started_time = time.time()
            self.last_frame_time = time.time()    # stall watchdog 시작점

    def should_stop(
        self,
        threshold: int,
        stale_s: float = 2.0,
        reg_err_threshold: int = 0,
        high_err_threshold: int = 0,
    ) -> bool:
        with self._lock:
            if self.tracking_lost:
                return True
            # (0) External stop_event — turntable thread 등 외부에서 set 한 경우.
            # 이게 없으면 turntable abort 가 set 한 stop_event 를 main loop 가
            # 못 보고 빈 캡처 계속함.
            if self._stop_event.is_set():
                self.tracking_lost = True
                if not self.last_loss_reason:
                    self.last_loss_reason = "external stop_event (turntable abort 등)"
                return True
            # (1) 연속 정합 실패 (FrameState 기반)
            if self.consecutive_lost >= threshold:
                self.tracking_lost = True
                self.last_loss_reason = (
                    f"연속 {self.consecutive_lost} 프레임 정합 실패 "
                    f"(last={self.last_state.name if self.last_state else '?'})"
                )
                self._stop_event.set()
                return True
            # (2) Frame stall — callback 자체가 N초 안 들어옴
            if self.last_frame_time > 0:
                silence = time.time() - self.last_frame_time
                if silence > stale_s:
                    self.tracking_lost = True
                    self.last_loss_reason = (
                        f"frame stall {silence:.1f}s (callback 무반응) — "
                        f"SDK auto-stop 또는 capture freeze 가능"
                    )
                    self._stop_event.set()
                    return True
            # (3) registration_error < 0 연속 N회 (Artec Studio 'tracking lost')
            if (reg_err_threshold > 0 and
                    self.consecutive_reg_err >= reg_err_threshold):
                self.tracking_lost = True
                self.last_loss_reason = (
                    f"연속 {self.consecutive_reg_err} 프레임 "
                    f"registration_error<0 (last={self.last_reg_error:.3f}) — "
                    f"Artec Studio 'tracking lost' 시그널"
                )
                self._stop_event.set()
                return True
            # (4) registration_error > max_acceptable 연속 N회
            if (high_err_threshold > 0 and
                    self.consecutive_high_err >= high_err_threshold):
                self.tracking_lost = True
                self.last_loss_reason = (
                    f"연속 {self.consecutive_high_err} 프레임 "
                    f"registration_error>{self._max_reg_error:.2f} "
                    f"(last={self.last_reg_error:.3f}) — 정합 품질 급락"
                )
                self._stop_event.set()
                return True
        return False

    @property
    def stop_event(self) -> threading.Event:
        return self._stop_event

    @property
    def scanning_flag(self) -> bool:
        """
        '지금 스캐닝이 건강한가' boolean.
        True  : tracking 살아있고 callback 받는 중
        False : tracking_lost (consec failure 또는 stall) 또는 stop_requested
        """
        with self._lock:
            return not self.tracking_lost and not self._stop_event.is_set()

    def request_stop(self, reason: str = "") -> None:
        with self._lock:
            if reason:
                self.last_loss_reason = reason
        self._stop_event.set()


# ─────────────────────────────────────────────────────────────────────────────
# TurntableController — 백그라운드 thread 로 회전 + stop_event 감시
# ─────────────────────────────────────────────────────────────────────────────

class TurntableController:
    """
    별도 thread 로 턴테이블 연속 회전. `stop_event` 가 set 되면 즉시 stop.
    """

    def __init__(
        self,
        turntable: "Turntable",
        vel_rad_s: float,
        target_rad: float,
        stop_event: threading.Event,
        poll_s: float = 0.05,
    ):
        self.turntable = turntable
        self.vel_rad_s = float(vel_rad_s)
        self.target_rad = float(target_rad)
        self.stop_event = stop_event
        self.poll_s = float(poll_s)

        self.thread: Optional[threading.Thread] = None
        self.actual_pos_rad: float = 0.0
        self.completed: bool = False
        self.aborted_reason: str = ""
        self._started_ok = False
        self._start_pos_rad: float = 0.0    # _run 시작 시 측정

    def start(self) -> None:
        self.thread = threading.Thread(target=self._run, daemon=True, name="ArtecTurntable")
        self.thread.start()

    def _run(self) -> None:
        try:
            # 시작 위치 측정 — 종료 판정은 절대값이 아니라 '시작점 대비 이동
            # 거리'. drive position counter 가 이전 run 잔여로 누적돼있어도
            # 정확히 한 바퀴 후 종료. 시작 위치 못 읽으면 0 으로 가정.
            start_pos = self.turntable.getActualPos()
            if isinstance(start_pos, bool) or start_pos is None:
                start_pos = 0.0
            start_pos = float(start_pos)
            self._start_pos_rad = start_pos

            ok = self.turntable.move_velocity(self.vel_rad_s, direction=0)
            self._started_ok = bool(ok)
            if not ok:
                self.aborted_reason = "move_velocity 거부"
                return

            consecutive_pos_fail = 0
            # drive 통신 사망 watchdog — N회 연속 getActualPos 실패하면
            # turntable 사망으로 간주하고 abort. poll_s 0.1s × 30 = 3s 허용.
            POS_FAIL_LIMIT = 30

            while True:
                if self.stop_event.is_set():
                    self.aborted_reason = "stop_event 감지"
                    break
                pos = self.turntable.getActualPos()
                if isinstance(pos, bool) or pos is None:
                    consecutive_pos_fail += 1
                    if consecutive_pos_fail >= POS_FAIL_LIMIT:
                        self.aborted_reason = (
                            f"getActualPos 연속 {consecutive_pos_fail}회 실패 — "
                            f"drive 통신 사망 또는 alarm trip 추정"
                        )
                        # 모터가 마지막 move_velocity 로 계속 도는 중일 수 있음.
                        # stop() 만으론 부족 — emergency_stop 시도 (servo_off).
                        try:
                            r = self.turntable.emergency_stop()
                            print(f"\n  [Turntable] emergency_stop tried: {r}")
                        except Exception as e:
                            print(f"\n  [Turntable] emergency_stop 예외: {e}")
                        # 다른 thread 들도 끊기게 stop_event set
                        self.stop_event.set()
                        break
                    time.sleep(self.poll_s)
                    continue
                consecutive_pos_fail = 0
                self.actual_pos_rad = float(pos)
                traveled = abs(self.actual_pos_rad - start_pos)
                if traveled >= self.target_rad:
                    self.completed = True
                    break
                time.sleep(self.poll_s)
        except Exception as e:
            self.aborted_reason = f"thread 예외: {e}"
        finally:
            # **반드시** 정지 — 어떤 경로로 나가든
            try:
                self.turntable.stop()
            except Exception as e:
                print(f"  [Turntable] thread 종료 시 stop 실패: {e}")

    def join(self, timeout: float = 2.0) -> None:
        if self.thread is not None:
            self.thread.join(timeout)


# ─────────────────────────────────────────────────────────────────────────────
# Result
# ─────────────────────────────────────────────────────────────────────────────

@dataclass
class ArtecStreamingScanResult:
    model: artec_base.ModelHandle
    n_frames: int = 0
    rotation_actual_deg: float = 0.0
    duration_s: float = 0.0
    fps_actual: float = 0.0
    tracking_lost: bool = False
    loss_reason: str = ""
    frames_ok: int = 0
    frames_failed: int = 0
    # Recovery 용: tracking 이 살아있었던 (reg_err >= 0) 마지막 timeline sample 의
    # turntable 각도 (rad). lost 발생 시 safe-back rollback 기준점.
    # tracking 이 한 번도 안 잡혔으면 0.0.
    last_good_theta_rad: float = 0.0


# ─────────────────────────────────────────────────────────────────────────────
# Session
# ─────────────────────────────────────────────────────────────────────────────

class ArtecStreamingScanSession:
    """
    연속 회전 + 실시간 SLAM Phase 1.
    Spider frame_callback ↔ Turntable thread 양방향 피드백.
    """

    def __init__(
        self,
        mms: "MMS",
        robot: Optional["XArmInterface"],
        turntable: "Turntable",
        settings: Optional[ArtecStreamingScanSessionSettings] = None,
    ):
        self.mms = mms
        self.robot = robot
        self.turntable = turntable
        self.s = settings or ArtecStreamingScanSessionSettings()

        if not isinstance(mms.sensor, ArtecClient):
            raise RuntimeError("ArtecClient 만 지원.")
        if turntable is None:
            raise RuntimeError("turntable 필수.")

    # ── Entry ──────────────────────────────────────────────────────────

    def run(self) -> ArtecStreamingScanResult:
        s = self.s
        scanner = self.mms.sensor._scanner

        # 1. FPS
        max_fps = float(scanner.max_fps())
        target_fps = min(s.target_fps if s.target_fps else max_fps, max_fps)
        try:
            scanner.set_fps(target_fps)
        except Exception as e:
            print(f"  [warn] set_fps 실패: {e}")
            target_fps = max_fps

        reg_name = artec_scanning.RegistrationType(s.registration_type).name
        print(f"\n═══════════════ Artec Streaming Phase 1 ═══════════════")
        print(f"  fps                : {target_fps:.1f} / max {max_fps:.1f}")
        print(f"  rotation_duration_s: {s.rotation_duration_s:.1f}")
        print(f"  registration       : {reg_name}")
        print(f"  loss thresholds    : "
              f"frameState={s.consecutive_loss_threshold}  "
              f"reg<0:{s.consecutive_reg_err_threshold}  "
              f"reg>{s.max_acceptable_reg_error:.2f}:{s.consecutive_high_err_threshold}")
        print(f"  expected frames    : ~{int(target_fps * s.rotation_duration_s)}")

        # 2. ScanSession 설정
        settings = artec_scanning.ScanSessionSettings.default()
        settings.set_max_frame_count(int(s.max_frame_count))    # ★ 0 = unlimited
        settings.set_registration_type(artec_scanning.RegistrationType(s.registration_type))
        settings.set_pipeline(s.pipeline_flags)
        settings.set_initial_state(artec_scanning.ScanningState.PREVIEW)
        if s.capture_texture:
            settings.set_capture_texture(artec_scanning.CaptureTextureMethod.ALWAYS)
        else:
            settings.set_capture_texture(artec_scanning.CaptureTextureMethod.NONE)
        settings.set_ignore_registration_errors(s.ignore_registration_errors)

        session = artec_scanning.ScanSession.create(scanner, settings)
        if s.sensitivity is not None:
            try: session.set_sensitivity(float(s.sensitivity))
            except Exception: pass
        if s.scan_range_near_mm is not None and s.scan_range_far_mm is not None:
            try:
                session.set_scanning_range(
                    float(s.scan_range_near_mm), float(s.scan_range_far_mm),
                )
            except Exception: pass

        # 3. TrackingState + frame callback
        tracking = TrackingState()
        tracking._max_reg_error = float(s.max_acceptable_reg_error)
        session.set_frame_callback(tracking.on_frame)

        # 4. 턴테이블 logical 0 reset — clearpos 로 카운터만 리셋.
        #    실패하면 drive alarm trip — 회전 시도 자체를 안 하고 빈 결과 반환.
        if s.reset_to_zero_first and self.turntable.is_connected:
            reset_ok = self._reset_turntable_to_zero(reset_vel_rad_s=np.radians(30.0))
            if not reset_ok:
                print(f"\n  ⚠ Turntable reset 실패 — 회전 진입 중단 (drive 복구 필요)")
                model = artec_base.create_model()
                return ArtecStreamingScanResult(
                    model=model,
                    n_frames=0,
                    rotation_actual_deg=0.0,
                    duration_s=0.0,
                    fps_actual=0.0,
                    tracking_lost=True,
                    loss_reason="turntable drive alarm — clearpos failed",
                    frames_ok=0,
                    frames_failed=0,
                )

        # 5. Preview → settle → drain (preview 단계 frame 은 카운트 안 함)
        print(f"\n  start_preview (settle {s.preview_settle_s:.1f}s)")
        session.start_preview()
        time.sleep(s.preview_settle_s)
        session.poll_events()           # preview 이벤트 비움
        with tracking._lock:
            tracking.frames_ok = 0
            tracking.frames_failed = 0
            tracking.consecutive_lost = 0

        # 6. Record + 회전 시작 (turntable 별도 thread)
        print(f"  start_record + 턴테이블 thread 시작")
        session.start_record()
        tracking.mark_started()                # ★ stall watchdog 기준점
        t_start = time.time()

        vel_rad_s = (2.0 * np.pi) / s.rotation_duration_s
        target_rad = 2.0 * np.pi * (1.0 + s.rotation_overshoot_deg / 360.0)
        # Drive 친화적 polling — 50ms (20Hz) 는 EziSERVO TCP queue 에 부하.
        # 100ms (10Hz) 면 30s 회전 동안 ~300 polls 로 충분 (회전 정밀도엔 영향 없음).
        tt_poll_s = max(0.1, s.poll_interval_s * 2.0)
        tt_ctrl = TurntableController(
            self.turntable, vel_rad_s, target_rad,
            tracking.stop_event, poll_s=tt_poll_s,
        )
        tt_ctrl.start()

        # 7. Main loop — event drain + 종료 조건 검사 + 시계열 로그
        timeout_s = s.rotation_duration_s + 10.0
        end_reason = "unknown"
        timeline: List[dict] = []      # (t, theta, scanning_flag, ...) 기록
        last_good_theta_rad: float = 0.0   # reg_err >= 0 였던 마지막 sample 의 θ
        try:
            while True:
                now = time.time()
                elapsed = now - t_start
                if elapsed > timeout_s:
                    end_reason = f"timeout ({timeout_s:.1f}s)"
                    tracking.request_stop("timeout")
                    break

                # Spider event drain — SDK 큐 비움 (freeze 방지)
                session.poll_events()

                # ── Time-synchronized 기록 ──────────────────────────────
                theta_rad = float(tt_ctrl.actual_pos_rad)
                flag = tracking.scanning_flag
                timeline.append({
                    "t": elapsed,
                    "theta_rad": theta_rad,
                    "theta_deg": float(np.degrees(theta_rad)),
                    "scanning_flag": int(flag),
                    "frames_ok": tracking.frames_ok,
                    "frames_failed": tracking.frames_failed,
                    "consec_lost": tracking.consecutive_lost,
                    "last_state": tracking.last_state.name if tracking.last_state else "",
                })
                # last-good θ — tracking 이 SDK 기준 살아있는 동안의 마지막 각도.
                # multipass recovery 가 safe-back rollback 시 기준으로 사용.
                if (tracking.tracking_established
                        and tracking.last_reg_error >= 0.0
                        and tracking.consecutive_reg_err == 0):
                    last_good_theta_rad = theta_rad

                # tracking lost 체크 — 네 종류
                #   (1) FrameState 기반 정합 실패 연속
                #   (2) callback stall
                #   (3) reg_err<0 연속
                #   (4) reg_err>max_acceptable 연속
                if tracking.should_stop(
                    s.consecutive_loss_threshold,
                    s.stale_threshold_s,
                    s.consecutive_reg_err_threshold,
                    s.consecutive_high_err_threshold,
                ):
                    end_reason = f"tracking lost: {tracking.last_loss_reason}"
                    print(f"\n  ⚠ {end_reason} — turntable 즉시 정지")
                    break

                # rotation 정상 완료 체크
                if tt_ctrl.completed:
                    end_reason = "rotation 정상 완료"
                    break

                # turntable thread 자체 abort (move_velocity 실패, drive 통신
                # 사망 등). aborted_reason 만 보고 break — stop_event 가 이미
                # set 됐을 수도 있음 (turntable 의 watchdog 가 set).
                if tt_ctrl.aborted_reason and not tt_ctrl.completed:
                    end_reason = f"turntable abort: {tt_ctrl.aborted_reason}"
                    # 항상 last_loss_reason 을 turntable abort 사유로 override —
                    # multipass 가 "drive 통신 사망/alarm trip" 키워드 보고
                    # retry 안 띄우고 즉시 종료하도록.
                    tracking.request_stop(end_reason)
                    print(f"\n  ⚠ {end_reason} — scanning 즉시 중단")
                    break

                # 진행률 1초마다 표시
                if int(elapsed) != int(elapsed - s.poll_interval_s):
                    estab = "EST" if tracking.tracking_established else "warm"
                    last_name = (tracking.last_state.name
                                 if tracking.last_state is not None else "?")
                    print(f"  [{elapsed:5.1f}s] θ={np.degrees(theta_rad):6.1f}°  "
                          f"flag={'ON ' if flag else 'OFF'}  "
                          f"ok={tracking.frames_ok}  fail={tracking.frames_failed}  "
                          f"consec={tracking.consecutive_lost}  "
                          f"regErr={tracking.last_reg_error:+.3f}  "
                          f"errLo={tracking.consecutive_reg_err}  "
                          f"errHi={tracking.consecutive_high_err}  "
                          f"trk={estab}  last={last_name}")

                time.sleep(s.poll_interval_s)
        finally:
            # 8. 항상 정지
            tracking.stop_event.set()
            tt_ctrl.join(timeout=3.0)
            try:
                self.turntable.stop()        # 안전 차원 한 번 더
            except Exception:
                pass

            # 마지막 event drain (record 도중 큐에 남은 것)
            time.sleep(s.post_record_settle_s)
            session.poll_events()

            print(f"  session.stop() ...")
            try:
                model = session.stop()
            except Exception as e:
                print(f"  ⚠ session.stop() 예외: {e}")
                model = None

        duration = time.time() - t_start

        if model is None:
            print(f"  ⚠ session.stop() 결과 None → 빈 IModel 반환")
            model = artec_base.create_model()

        n_frames = sum(model.get_scan(i).frame_count() for i in range(model.scan_count()))
        fps_actual = n_frames / max(duration, 1e-6)

        print(f"\n[StreamingScan 결과]")
        print(f"  종료 사유      : {end_reason}")
        print(f"  duration       : {duration:.1f} s")
        print(f"  rotation       : {np.degrees(tt_ctrl.actual_pos_rad):.1f}°")
        print(f"  scans          : {model.scan_count()}")
        print(f"  total frames   : {n_frames}  (ok={tracking.frames_ok}  fail={tracking.frames_failed})")
        print(f"  effective fps  : {fps_actual:.1f}")
        print(f"  tracking_lost  : {tracking.tracking_lost}")
        if tracking.state_counts:
            dist = "  ".join(f"{k}={v}" for k, v in
                             sorted(tracking.state_counts.items(),
                                    key=lambda kv: -kv[1]))
            print(f"  state 분포     : {dist}")

        # 9. Timeline CSV dump (옵션)
        if s.timeline_csv_path and timeline:
            try:
                from pathlib import Path as _P
                p = _P(s.timeline_csv_path)
                p.parent.mkdir(parents=True, exist_ok=True)
                with open(p, "w", encoding="utf-8") as f:
                    keys = ["t", "theta_rad", "theta_deg", "scanning_flag",
                            "frames_ok", "frames_failed", "consec_lost", "last_state"]
                    f.write(",".join(keys) + "\n")
                    for row in timeline:
                        f.write(",".join(f"{row[k]}" for k in keys) + "\n")
                print(f"  [timeline] {len(timeline)} samples → {p}")
            except Exception as e:
                print(f"  [timeline] dump 실패: {e}")

        # 10. 0° 복귀 (정상 종료 / lost 무관) — 회전 한바퀴 + 마진 timeout
        print(f"\n[Turntable] post-scan 0° 복귀 ...")
        try:
            self._reset_turntable_to_zero(reset_vel_rad_s=np.radians(30.0))
        except Exception as e:
            print(f"  [Turntable] post-reset 예외 (무시): {e}")

        return ArtecStreamingScanResult(
            model=model,
            n_frames=n_frames,
            rotation_actual_deg=float(np.degrees(tt_ctrl.actual_pos_rad)),
            duration_s=duration,
            fps_actual=fps_actual,
            tracking_lost=tracking.tracking_lost,
            loss_reason=tracking.last_loss_reason,
            frames_ok=tracking.frames_ok,
            frames_failed=tracking.frames_failed,
            last_good_theta_rad=last_good_theta_rad,
        )

    # ── 유틸 ──────────────────────────────────────────────────────────

    def _read_theta(self, fallback: float) -> tuple[float, bool]:
        """현재 turntable 각도와 'read 성공 여부' 를 반환.
        getActualPos 가 False/None 이면 (fallback, False)."""
        if not self.turntable.is_connected:
            return float(fallback), False
        v = self.turntable.getActualPos()
        if isinstance(v, bool) or v is None:
            return float(fallback), False
        return float(v), True

    def _reset_turntable_to_zero(self, reset_vel_rad_s: float = np.radians(30.0)) -> bool:
        """
        Logical 0° reset — drive position counter 를 0 으로 재정의.

        절차:
          1. stop() — 모션 중일 수 있으니 정지
          2. check_drive_err() — alarm latch 있으면 ServoAlarmReset 으로 clear
          3. clearpos() — position counter 0 으로 강제 설정
        """
        # 1. 모션 정지
        try:
            self.turntable.stop()
        except Exception:
            pass
        time.sleep(0.2)

        # 2. ★ Alarm latch clear — 이전 run 에서 누적된 alarm 가 있을 수 있음.
        #    check_drive_err 가 내부에서 FAS_ServoAlarmReset 호출.
        try:
            self.turntable.check_drive_err()
        except Exception as e:
            print(f"  [Turntable] check_drive_err 예외 (무시): {e}")
        time.sleep(0.2)

        # 3. 현재 위치 read (참고용)
        cur, ok_read = self._read_theta(fallback=0.0)
        if ok_read:
            print(f"\n  [Turntable] clearpos: {np.degrees(cur):+.2f}° → logical 0°")
        else:
            print(f"\n  [Turntable] clearpos: 위치 read 실패 → logical 0°")

        # 4. clearpos
        try:
            ok = self.turntable.clearpos()
        except Exception as e:
            print(f"  [Turntable] ✘ clearpos 예외: {e}")
            ok = False

        if not ok:
            print(f"  [Turntable] ✘ clearpos 실패 — drive alarm trip 의심")
            # 모터가 계속 도는 중일 수 있음 — emergency_stop 시도
            try:
                r = self.turntable.emergency_stop()
                print(f"  [Turntable] emergency_stop tried: {r}")
            except Exception as e:
                print(f"  [Turntable] emergency_stop 예외: {e}")
            print(f"  [Turntable]   복구: EziSERVO drive 물리 전원 OFF→5s 대기→ON")
            return False

        # 5. ★ Servo 재활성화 — emergency_stop / drive 재연결 후엔 servo OFF
        # 상태일 수 있어 다음 move_velocity 가 거부됨. 항상 ON 강제.
        try:
            self.turntable.set_servo_on(True)
        except Exception as e:
            print(f"  [Turntable] set_servo_on(True) 예외 (무시): {e}")
        return True
