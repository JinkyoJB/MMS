# mms_artec/nbv/artec_streaming_scan_session.py
#
# Artec **IScanningProcedure** (streaming) 기반 lookaround.
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
from utils.nbv.event_log import log_event as _ev      # run 이벤트(JSONL) — lost 분석용

if TYPE_CHECKING:
    from mms_artec.system import ArtecMMS as MMS
    from mms_artec.nbv.live_scan_viewer import LiveScanViewer
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
    # ★ 부분 스윕 (rad). None = 전회전(기본). nbv gap 겨냥처럼 목표 각 구간만
    #   돌 때 설정한다 — sim(`NBV_PATCH_SPAN_DEG`)과 같은 구조. 각속도는 전회전과
    #   동일(2π/rotation_duration_s)하고 목표각·타임아웃만 구간에 비례한다.
    #   ⚠ sim 실측: 60° 이하로 좁히면 프레임 간 중첩 부족으로 정합이 무너진다.
    sweep_rad: Optional[float] = None
    #: 이 캡처가 **어느 단계**를 위한 것인지 (배너 표기용). 예전엔 배너가
    #  "lookaround" 로 박혀 있어, nbv·flip 에서도 lookaround 라고 찍혀 로그만
    #  보면 지금 어느 단계인지 알 수 없었다 (2026-09-21 실물 혼동).
    stage_label: str = "lookaround"

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
    # ★ False: SDK 가 reg_err<0 프레임을 IScan 에 안 넣음 → 노이즈 감소.
    # True 였을 땐 정합 실패 프레임도 IScan 에 우겨넣어 cloud 형상 망가짐.
    # 단점: tracking_lost 가 정직하게 더 자주 trip 될 수 있음 (그게 맞는 신호).
    ignore_registration_errors: bool = False

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
    # docs/7_artec_lookaround.md §12 의 freeze 원인.
    max_frame_count: int = 0

    # ── 타이밍 ────────────────────────────────────────────────────────
    preview_settle_s: float = 1.5
    post_record_settle_s: float = 0.5
    poll_interval_s: float = 0.05         # main loop 의 event drain 주기
    # 밴드 전환: 로봇이 멈춘 뒤 회전을 시작하기 전 SLAM 재정착 대기.
    band_settle_s: float = 2.0

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
    # ── 밴드 회계 ─────────────────────────────────────────────────────────
    # 밴드 3개 중 2개가 전회전을 마치고 3번째에서 lost 된 경우, `tracking_lost`
    # 만 보면 "전부 실패"와 구분이 안 된다. 그러면 호출자가 이미 성공한 밴드까지
    # 처음부터 다시 돌린다 — 2026-09-16 실물에서 band1·2 완주분을 버리고
    # pass 2·3 을 낭비하고 nbv 에 도달조차 못 했다.
    n_bands: int = 1                   # 계획된 밴드 수
    n_bands_done: int = 0              # 전회전을 끝낸 밴드 수
    band_reasons: List[str] = field(default_factory=list)


# ─────────────────────────────────────────────────────────────────────────────
# Session
# ─────────────────────────────────────────────────────────────────────────────

class ArtecStreamingScanSession:
    """
    연속 회전 + 실시간 SLAM lookaround.
    Spider frame_callback ↔ Turntable thread 양방향 피드백.
    """

    def __init__(
        self,
        mms: "MMS",
        robot: Optional["XArmInterface"],
        turntable: "Turntable",
        settings: Optional[ArtecStreamingScanSessionSettings] = None,
        live_viewer: Optional["LiveScanViewer"] = None,
        band_poses=None,
        move_robot_fn=None,
        retarget_fn=None,
        standoff_of=None,
        scan_range=None,
        band_path_fn=None,
    ):
        self.mms = mms
        self.robot = robot
        self.turntable = turntable
        self.s = settings or ArtecStreamingScanSessionSettings()
        #: 밴드 관절해 목록. 주면 **한 IScan 안에서** 밴드마다 전회전한다
        #  (세션을 열어둔 채 로봇만 옮기므로 SLAM 추적이 이어진다).
        #  None/빈 목록이면 기존 단일 회전과 완전히 동일하다.
        self.band_poses = list(band_poses) if band_poses else None
        #: 밴드 이동 콜백 `fn(q) -> None`. 녹화 중 호출되므로 **느리게** 움직여야
        #  추적이 유지된다 (호출자가 속도를 정한다).
        self.move_robot_fn = move_robot_fn
        #: 거리추종 — `retarget_fn(q_cur, d_new) -> q_new|None` 은 **같은 el/az/tz**
        #  를 새 축거리로 다시 푼다. `standoff_of(q) -> m` 은 그 밴드의 계획 거리.
        #  판단(얼마나·언제 움직일지)은 sim 과 공유하는 `StandoffTracker` 가 하고,
        #  여기는 "표면거리를 재서 넘기고, 결과대로 옮긴다" 만 한다.
        self.retarget_fn = retarget_fn
        self.standoff_of = standoff_of
        self.scan_range = scan_range
        #: 밴드 전환 경로 `fn(q_from, q_to, step_m, alpha_from, seed) -> [(q, α)]`
        #  — 카메라 조준을 유지한 소보간. 주면 `_band_transition` 이 스텝마다
        #  폴링·regErr 감시하고, 잃으면 되돌아가 되찾는다. 없으면 예전 한 방 이동.
        self.band_path_fn = band_path_fn
        # 선택적 라이브 뷰어 — multipass 가 pass 들 사이에 재사용하라고 넘김.
        # None 이면 모든 viewer 경로 no-op. 절대 스캔을 깨뜨리지 않음.
        self.live_viewer = live_viewer

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
        print(f"\n═══════════════ Artec Streaming {s.stage_label} ═══════════════")
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

        # 3. TrackingState (frame callback 은 세션 생성 루프에서 바인딩)
        tracking = TrackingState()
        tracking._max_reg_error = float(s.max_acceptable_reg_error)

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

        # 5. 세션 생성 + preview 헬스체크 (+1회 재초기화 회복)
        #    ★ preview settle 동안 이벤트가 **0개**면 SDK 스캐닝 파이프라인이
        #      웨지된 것이다 — 단일 프레임 preview(`capture_frame_handle`)는
        #      다른 경로라 멀쩡해 보여도, record 를 시작하면 콜백이 영원히 안
        #      온다 (2026-09-16 실물: pass 1·2 모두 frames 0 → "frame stall
        #      2.0s"). 이때는 `_robust_capture` 와 같은 처방 — scanner
        #      shutdown → initialize 로 세션을 새로 연다.
        session = None
        for _sess_try in (1, 2):
            session = artec_scanning.ScanSession.create(scanner, settings)
            if s.sensitivity is not None:
                try: session.set_sensitivity(float(s.sensitivity))
                except Exception: pass
            # 작동거리 창 — 세션 설정이 없으면 **스캐너(ArtecConfig)의 값을 따른다.**
            # 두 SDK 객체(IScanningProcedure / IFrameProcessor)가 따로 노는 것을 막는다.
            _near, _far = s.scan_range_near_mm, s.scan_range_far_mm
            if _near is None or _far is None:
                try:
                    _near, _far = self.mms.sensor.scanning_range()
                except Exception:                        # noqa: BLE001
                    _near = _far = None
            if _near is not None and _far is not None:
                try:
                    session.set_scanning_range(float(_near), float(_far))
                    print(f"  스캔 범위 {float(_near):.0f}~{float(_far):.0f}mm")
                except Exception: pass
            session.set_frame_callback(tracking.on_frame)

            print(f"\n  start_preview (settle {s.preview_settle_s:.1f}s)"
                  + (f" — 재초기화 후 재시도" if _sess_try > 1 else ""))
            session.start_preview()
            time.sleep(s.preview_settle_s)
            _prev_ev = session.poll_events()        # preview 이벤트 비움 + 카운트
            _n_prev = len(_prev_ev) if _prev_ev else 0
            if _n_prev > 0:
                break                               # 파이프라인 정상
            print(f"  ⚠ preview {s.preview_settle_s:.1f}s 동안 이벤트 0개 — "
                  f"스캐닝 파이프라인 무반응")
            try:
                session.stop()
            except Exception:
                pass
            if _sess_try == 1:
                print(f"  → 스캐너 세션 재초기화 (shutdown → initialize)")
                try:
                    self.mms.sensor.shutdown()
                except Exception as e:              # noqa: BLE001
                    print(f"    shutdown 경고: {e}")
                time.sleep(1.0)
                try:
                    self.mms.sensor.initialize()
                except Exception as e:              # noqa: BLE001
                    print(f"    ✘ 재초기화 실패: {e}")
                    break
                scanner = self.mms.sensor._scanner
            else:
                print(f"  ✘ 재초기화 후에도 무반응 — 스캐너 USB 재연결/전원 "
                      f"재시작이 필요할 수 있다")
                model = artec_base.create_model()
                return ArtecStreamingScanResult(
                    model=model, n_frames=0, rotation_actual_deg=0.0,
                    duration_s=0.0, fps_actual=0.0, tracking_lost=True,
                    loss_reason="scanner pipeline 무반응 (재초기화 후에도 "
                                "preview 이벤트 0개)",
                    frames_ok=0, frames_failed=0)
        with tracking._lock:
            tracking.frames_ok = 0
            tracking.frames_failed = 0
            tracking.consecutive_lost = 0

        # 6. Record + 회전 시작 (turntable 별도 thread)
        print(f"  start_record + 턴테이블 thread 시작")
        session.start_record()
        tracking.mark_started()                # ★ stall watchdog 기준점
        t_start = time.time()

        # ── 밴드 루프 ────────────────────────────────────────────────
        #  ★ **한 IScan 안에서** 밴드마다 전회전한다. 밴드마다 세션을 새로 열면
        #    IScan 이 분리돼 SLAM 추적이 끊기고, 밴드끼리는 후처리
        #    GlobalRegistration 에 의존하게 된다(2026-09-16 실물에서 band1/band2
        #    가 정합되지 않음). 세션을 열어둔 채 로봇만 옮기면 추적이 이어진다.
        #  `band_poses` 가 없으면 기존 단일 회전과 **완전히 동일**하다.
        bands = list(self.band_poses) if self.band_poses else [None]
        timeline: List[dict] = []      # (t, theta, ...) — 밴드 간 누적
        band_reasons: List[str] = []   # 밴드별 종료 사유 — 잘린 밴드를 숨기지 않는다
        n_bands_done = 0               # 전회전을 끝낸 밴드 수
        tt_ctrl = None
        model = None
        if len(bands) > 1:
            print(f'  밴드 {len(bands)}개를 한 IScan 에서 연속 스캔한다')
        try:                                    # outer — session.stop() 보장
            for _bi, _band_q in enumerate(bands):
                if _bi > 0:
                    if tracking.tracking_lost:
                        print('  ⚠ tracking lost — 남은 밴드 중단')
                        break
                    print('')
                    print(f'  ── band {_bi+1}/{len(bands)} — 로봇 이동 (녹화 유지) ──')
                    try:
                        if self.move_robot_fn is not None and _band_q is not None:
                            if self.band_path_fn is not None and bands[_bi - 1] is not None:
                                self._band_transition(session, tracking,
                                                      bands[_bi - 1], _band_q, band_i=_bi)
                            else:
                                self.move_robot_fn(_band_q)
                                _ev("band_move", stage=s.stage_label, from_band=_bi,
                                    to_band=_bi + 1, steps=1, outcome="direct")
                    except Exception as e:
                        print(f'  ⚠ 밴드 이동 실패({e}) — 남은 밴드 중단')
                        break
                    # 턴테이블을 0 으로 되돌리고 stop_event 를 푼다.
                    # ★ 이전 밴드 finally 에서 set 된 채면 다음 회전이 즉시 중단된다.
                    try:
                        self._reset_turntable_to_zero(
                            reset_vel_rad_s=np.radians(30.0))
                    except Exception as e:
                        print(f'  ⚠ 턴테이블 0 복귀 실패({e})')
                    tracking.stop_event.clear()
                    # ★ 이동 직후 바로 회전시키지 않는다. 로봇이 멈춘 뒤 SLAM 이
                    #   새 시점에서 정합을 다시 잡을 시간을 준다 — 이동+회전이
                    #   겹치면 프레임 간 변화가 커져 추적이 엉뚱한 자세로 수렴할 수
                    #   있다(SDK 는 reg_err 양수를 내며 '성공'으로 보고한다).
                    if s.band_settle_s > 0:
                        print(f"  이동 완료 — SLAM 재정착 {s.band_settle_s:.1f}s 대기")
                        _t_settle = time.time()
                        while time.time() - _t_settle < s.band_settle_s:
                            session.poll_events()      # 큐 비움 (freeze 방지)
                            time.sleep(s.poll_interval_s)
                    tracking.mark_started()
                # ★ 거리추종 — 회전을 시작하기 **전에**, 턴테이블이 멈춰 있는 동안
                #   표면거리를 재서 축거리만 고친다. 회전 중이 아니라 여기서 하는
                #   이유: 밴드 경계는 이미 로봇이 움직이고 settle 하는 자리라
                #   추가 위험이 0 이다(회전+이동이 겹치면 SLAM 이 흔들린다).
                _live = self._track_standoff(session, _band_q, _bi, len(bands))
                # 회전 중 거리추종(live) 상태 — 트래커·현재 축거리·재겨냥 콜백.
                _live_trk, _live_d, _live_rt = (_live if _live else (None, 0.0, None))
                _live_since = 0            # 마지막 판정 이후 OK 프레임 수
                _last_ok_v = None          # 최근 OK 프레임 정점 (스캐너 프레임, m)
                vel_rad_s = (2.0 * np.pi) / s.rotation_duration_s
                if s.sweep_rad is not None:            # 부분 스윕 (nbv gap 겨냥)
                    target_rad = float(abs(s.sweep_rad)) \
                        + np.radians(s.rotation_overshoot_deg)
                    print(f"  부분 스윕 {np.degrees(abs(s.sweep_rad)):.0f}°"
                          f" (전회전 아님)")
                else:
                    target_rad = 2.0 * np.pi * (1.0 + s.rotation_overshoot_deg / 360.0)
                # Drive 친화적 polling — 50ms (20Hz) 는 EziSERVO TCP queue 에 부하.
                # 100ms (10Hz) 면 30s 회전 동안 ~300 polls 로 충분 (회전 정밀도엔 영향 없음).
                tt_poll_s = max(0.1, s.poll_interval_s * 2.0)
                tt_ctrl = TurntableController(
                    self.turntable, vel_rad_s, target_rad,
                    tracking.stop_event, poll_s=tt_poll_s,
                )
                tt_ctrl.start()

                # 6.5 라이브 뷰어 — Artec SDK 정합행렬(FrameEvent.transformation)을
                #     그대로 누적. θ / hand-eye / turntable_frame.yaml 의존 없음.
                live = self.live_viewer
                live_ok = live is not None
                if live_ok:
                    print(f"  [live] viewer 활성 — SDK 정합행렬 기반 누적")

                # 7. Main loop — event drain + 종료 조건 검사 + 시계열 로그
                #    타임아웃은 실제 목표각에 비례 (부분 스윕이면 그만큼 짧게).
                timeout_s = (s.rotation_duration_s
                             * (target_rad / (2.0 * np.pi))) + 10.0
                # ★ 타임아웃은 **밴드마다** 새로 잰다. `t_start` 는 세션 전체(=IScan)
                #   기준이라 밴드 2·3 에서는 이미 앞 밴드 시간이 쌓여 있다 —
                #   2026-09-16: 밴드 3개를 한 scan 에 담은 뒤 band 3 이 시작 2.7초
                #   만에 "timeout (40.0s)" 로 잘렸다(추적은 정상이었다).
                t_band = time.time()
                end_reason = "unknown"
                band_ok = False            # 이 밴드가 전회전을 끝냈는가
                # timeline 은 밴드 간 누적 — 루프 밖에서 선언한다
                last_good_theta_rad: float = 0.0   # reg_err >= 0 였던 마지막 sample 의 θ
                # 라이브 뷰어 진단 카운터 — 검은 화면 디버깅용
                _lv_ev = 0          # 받은 총 이벤트
                _lv_okmesh = 0      # OK + frame_mesh + reg_err>=0 (= viewer 에 공급)
                _lv_feederr = ""    # 첫 feed 예외 메시지
                try:
                    while True:
                        now = time.time()
                        elapsed = now - t_band          # 이 밴드의 경과
                        if elapsed > timeout_s:
                            end_reason = (f"timeout ({timeout_s:.1f}s, "
                                          f"band {_bi + 1}/{len(bands)})")
                            tracking.request_stop("timeout")
                            break

                        # Spider event drain — SDK 큐 비움 (freeze 방지)
                        events = session.poll_events()

                        # ── Time-synchronized 기록 ──────────────────────────────
                        theta_rad = float(tt_ctrl.actual_pos_rad)
                        flag = tracking.scanning_flag

                        # ── 라이브 뷰어 공급 (메인 스레드, race 없음) ───────────
                        # OK 프레임의 frame_mesh + SDK 정합행렬(ev.transformation)을
                        # scan-world 로 누적. 예외는 viewer 내부에서 삼킴 → 스캔 영향 0.
                        #
                        # ★ 필터는 SDK 가 IScan 에 넣는 기준과 동일하게만 — viewer 는
                        #   IScan 을 그대로 비추는 거울이어야 함 ([[feedback_live_viewer
                        #   _must_mirror_scan]]). ignore_registration_errors=False 일 때
                        #   SDK 가 reg_err<0 프레임을 IScan 에 안 넣음 → viewer 도 동일
                        #   기준으로 거름. 그 외 high-error / warm-up 같은 추가 게이트는
                        #   둘 다 IScan 에 들어가니 viewer 에서도 통과시켜야 거짓말 없음.
                        if events:
                            _lv_ev += len(events)
                        # ── 거리추종 live — 회전 중에도 축거리를 창 중앙으로 ──────
                        #  sim 과 같은 `StandoffTracker.update()`. 측정은 최근 OK
                        #  프레임의 정점(스캐너 프레임, 원점=카메라)이라 hand-eye 가
                        #  안 낀다. 이동은 밴드 경계와 같은 `move_robot_fn`(녹화 유지,
                        #  느린 속도)이고 1회 ≤ TRACK_MAX_STEP_M(25mm). 이동 중엔
                        #  프레임이 비므로 stall watchdog 기준점을 다시 찍는다.
                        #  ⚠ 회전+이동이 겹칠 때 SLAM 이 붙어 있는지는 실기 확인
                        #    항목이다 — 깨지면 MMS_STANDOFF_TRACK_STEP_MM·_EVERY 를
                        #    조이고, 그래도 안 되면 MMS_STANDOFF_TRACK=band.
                        if events and _live_trk is not None and _live_trk.mode == "live":
                            try:
                                FS = artec_scanning.FrameState
                                for ev in events:
                                    if ev.frame_state == FS.OK and ev.frame_mesh is not None:
                                        _live_since += 1
                                        _v = ev.frame_mesh.vertices()
                                        if _v is not None and len(_v):
                                            _last_ok_v = np.asarray(_v, float) / 1000.0
                            except Exception:                    # noqa: BLE001
                                pass
                            if _live_since >= _live_trk.every and _last_ok_v is not None:
                                _live_since = 0
                                try:
                                    _d_new = _live_trk.update(
                                        int(tracking.frames_ok), _live_d, _last_ok_v,
                                        np.zeros(3), _live_rt)
                                except Exception as e:           # noqa: BLE001
                                    print(f"  [거리추종] live 판정 실패({e}) — 유지")
                                    _d_new = _live_d
                                if _d_new != _live_d:
                                    _live_d = _d_new
                                    tracking.mark_started()      # 이동 공백 = stall 아님
                                    # ★ 이동(블로킹) 중 큐에 쌓인 프레임은 이동 전
                                    #   것이다 — 다음 판정까지 한 주기 더 흘려보내
                                    #   새 자세의 프레임만 재게 한다.
                                    _live_since = -int(_live_trk.every)
                                    _last_ok_v = None
                        if live is not None and live_ok and events:
                            try:
                                FS = artec_scanning.FrameState
                                for ev in events:
                                    if (ev.frame_state == FS.OK
                                            and ev.frame_mesh is not None
                                            and ev.transformation is not None
                                            and float(ev.registration_error) >= 0.0):
                                        _lv_okmesh += 1
                                        live.add_frame(ev.frame_mesh,
                                                       ev.transformation)
                            except Exception as e:
                                if not _lv_feederr:
                                    _lv_feederr = f"{type(e).__name__}: {e}"
                        if live is not None and live_ok:
                            try:
                                live.tick(flag)
                            except Exception:
                                pass
                        timeline.append({
                            "t": now - t_start,     # 세션 전체 축 (밴드마다 되감기면 안 됨)
                            "band": _bi + 1,
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
                            _ev("lost", stage=s.stage_label, band=_bi + 1, n_bands=len(bands),
                                theta_deg=float(np.degrees(theta_rad)),
                                frames_ok=int(tracking.frames_ok),
                                reason=str(tracking.last_loss_reason))
                            break

                        # rotation 정상 완료 체크
                        if tt_ctrl.completed:
                            end_reason = "rotation 정상 완료"
                            band_ok = True
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
                            print(f"  [b{_bi+1} {elapsed:5.1f}s] θ={np.degrees(theta_rad):6.1f}°  "
                                  f"flag={'ON ' if flag else 'OFF'}  "
                                  f"ok={tracking.frames_ok}  fail={tracking.frames_failed}  "
                                  f"consec={tracking.consecutive_lost}  "
                                  f"regErr={tracking.last_reg_error:+.3f}  "
                                  f"errLo={tracking.consecutive_reg_err}  "
                                  f"errHi={tracking.consecutive_high_err}  "
                                  f"trk={estab}  last={last_name}")
                            if live is not None:
                                _pts = getattr(live, "_n_buffered", -1)
                                _ing = getattr(live, "_frames_ingested", -1)
                                _dead = getattr(live, "_dead", "?")
                                _rej = getattr(live, "_reject", "")
                                _bb = getattr(live, "_bbox_str", "")
                                print(f"           [live] ok={live_ok} ev={_lv_ev} "
                                      f"okmesh={_lv_okmesh} ingest={_ing} "
                                      f"pts={_pts} dead={_dead}"
                                      + (f"  AABB[{_bb}]" if _bb else "")
                                      + (f"  reject={_rej}" if _rej else "")
                                      + (f"  feedErr={_lv_feederr}"
                                         if _lv_feederr else ""))

                        time.sleep(s.poll_interval_s)
                finally:
                    # 밴드 종료 — 턴테이블만 정리한다. session.stop() 은 바깥에서.
                    # 밴드별 사유를 모은다. end_reason 하나만 두면 마지막 밴드의
                    # 사유가 앞 밴드의 실패(잘린 회전)를 덮어버린다.
                    _th = ""
                    if tt_ctrl is not None:
                        _th = f" (θ={np.degrees(float(tt_ctrl.actual_pos_rad)):.1f}°)"
                    band_reasons.append(
                        f"band {_bi + 1}/{len(bands)}: {end_reason}{_th}")
                    if band_ok:
                        n_bands_done += 1
                    tracking.stop_event.set()
                    if tt_ctrl is not None:
                        tt_ctrl.join(timeout=3.0)
                    try:
                        self.turntable.stop()
                    except Exception:
                        pass
                if tracking.tracking_lost:
                    break
        finally:
            # 모든 밴드가 끝난 뒤 **한 번만** 세션을 닫는다 → IScan 1개.
            time.sleep(s.post_record_settle_s)
            try:
                session.poll_events()
            except Exception:
                pass
            print(f'  session.stop() ...')
            try:
                model = session.stop()
            except Exception as e:
                print(f'  ⚠ session.stop() 예외: {e}')
                model = None

        duration = time.time() - t_start

        if model is None:
            print(f"  ⚠ session.stop() 결과 None → 빈 IModel 반환")
            model = artec_base.create_model()

        n_frames = sum(model.get_scan(i).frame_count() for i in range(model.scan_count()))
        fps_actual = n_frames / max(duration, 1e-6)

        print(f"\n[StreamingScan 결과]")
        print(f"  종료 사유      : {end_reason}")
        if len(bands) > 1:
            print(f"  밴드 완주       : {n_bands_done}/{len(bands)}")
            for _r in band_reasons:
                print(f"    · {_r}")
        print(f"  duration       : {duration:.1f} s")
        print(f"  rotation       : {np.degrees(tt_ctrl.actual_pos_rad):.1f}°  (마지막 밴드)")
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
            n_bands=len(bands),
            n_bands_done=n_bands_done,
            band_reasons=list(band_reasons),
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

    # ── 거리추종 (el·az·tz 고정 · 축거리만 창 중앙으로) ──────────────────
    def _latest_frame_ranges_m(self, session, timeout_s: float = 1.5,
                               fresh_settle_s: float = 0.0):
        """최근 OK 프레임의 **스캐너 프레임 정점**으로 표면거리 배열(m)을 만든다.

        `frame_mesh.vertices()` 는 스캐너 좌표계 mm 라 원점이 곧 카메라다 —
        거리 = 정점의 노름. 좌표변환·hand-eye 가 전혀 끼지 않아, 거리추종이
        캘리브 오차에 오염되지 않는다(이게 이 경로를 고른 이유다).

        `fresh_settle_s` > 0 이면 **큐를 비우고 그 시간 동안 오는 프레임도 버린 뒤**
        그다음 OK 프레임을 쓴다. 로봇 이동(블로킹) 직후엔 큐가 이동 전 프레임으로
        차 있어서, 그냥 "첫 묶음의 마지막 OK" 를 쓰면 이동 전 값을 읽는다
        (2026-09-21 run_150720: 세 번 물러나도 213→218mm — 그 뒤 lost).
        """
        import time as _t
        best = None
        t0 = _t.time()
        FS = artec_scanning.FrameState
        if fresh_settle_s > 0:
            t_s = _t.time()
            while _t.time() - t_s < fresh_settle_s:
                try:
                    session.poll_events()               # 이동 전/중 프레임 버림
                except Exception:                       # noqa: BLE001
                    break
                _t.sleep(self.s.poll_interval_s)
            t0 = _t.time()
        while _t.time() - t0 < timeout_s:
            try:
                events = session.poll_events()
            except Exception:                                   # noqa: BLE001
                break
            for ev in (events or []):
                try:
                    if ev.frame_state == FS.OK and ev.frame_mesh is not None:
                        v = ev.frame_mesh.vertices()
                        if v is not None and len(v):
                            best = np.asarray(v, float)
                except Exception:                               # noqa: BLE001
                    continue
            if best is not None:
                break
            time.sleep(self.s.poll_interval_s)
        if best is None:
            return None
        return best / 1000.0                     # mm → m (원점 = 카메라)

    # ── 밴드 전환: 조준 유지 소보간 + relocalization ─────────────────────
    #  왜 — 2026-09-21 여섯 run 전부 밴드 전환 직후 θ=0° 에서 lost(회전 중은 0회).
    #  이동 거리·고도각·fill 과 무관해 경로 의존으로 판단. Artec SDK 스트리밍은
    #  한번 잃으면 마지막 키프레임과 겹치는 시야로 돌아가기 전엔 재정합하지 않는다.
    BAND_STEP_M = 0.005          # 스텝당 카메라 이동 (m)
    BAND_STEP_POLL_S = 0.15      # 스텝 후 프레임 폴링 시간 (~2 프레임)
    BAND_RELOC_TRIES = 2         # 되돌아가기 시도 횟수
    BAND_RELOC_WAIT_S = 1.5      # 되찾음 판정 대기 (스텝당)

    def _reg_ok(self, tracking) -> bool:
        """마지막 프레임이 정합됐고 연속 reg<0 가 끊겼는가."""
        with tracking._lock:
            return (tracking.last_reg_error >= 0.0
                    and tracking.consecutive_reg_err == 0)

    BAND_LOST_N = 3              # 스텝 뒤 연속 reg<0 가 이만큼이면 '잃었다' 로 보고 멈춤

    def _wait_reg(self, session, tracking, wait_s: float) -> bool:
        """정지 상태로 최대 wait_s 동안 폴링하며 정합 회복을 기다린다."""
        t0 = time.time()
        while time.time() - t0 < wait_s:
            session.poll_events()
            if self._reg_ok(tracking):
                return True
            time.sleep(self.s.poll_interval_s)
        return self._reg_ok(tracking)

    def _walk_path(self, session, tracking, path, tag: str, mode: str = "fwd",
                   wait_s: float = None) -> int:
        """`path`=[(q, α)] 를 순서대로 밟는다. 스텝마다 폴링해 regErr 를 찍는다.

        mode="fwd"  : 잃으면(연속 reg<0 ≥ BAND_LOST_N, 잠깐 기다려도 회복 없음)
                      **그 자리에서 멈추고** 그 인덱스를 돌려준다. 끝까지 가면 -1.
        mode="back" : 되찾은 스텝에서 멈추고 그 인덱스를 돌려준다. 못 찾으면 -1.
        """
        wait_s = self.BAND_STEP_POLL_S if wait_s is None else float(wait_s)
        n = len(path)
        for i, (q, a) in enumerate(path):
            self.move_robot_fn(q)
            n_ok0 = tracking.frames_ok
            t0 = time.time()
            ok_seen = False
            while time.time() - t0 < wait_s:
                session.poll_events()
                if mode == "back" and tracking.frames_ok > n_ok0 and self._reg_ok(tracking):
                    ok_seen = True
                    break
                time.sleep(self.s.poll_interval_s)
            with tracking._lock:
                re, ce = tracking.last_reg_error, tracking.consecutive_reg_err
            print(f"    [{tag}] {i+1}/{n} α={a:.2f}  regErr={re:+.3f}  consec<0={ce}")
            if mode == "back" and ok_seen:
                return i
            if mode == "fwd" and ce >= self.BAND_LOST_N:
                if self._wait_reg(session, tracking, self.BAND_RELOC_WAIT_S):
                    continue                      # 잠깐 멈추니 다시 붙었다
                print(f"    [{tag}] ⚠ α={a:.2f} 에서 정합 끊김 — 여기서 멈춤")
                return i
        return -1

    def _band_transition(self, session, tracking, q_from, q_to, band_i: int = 0) -> None:
        """밴드 q_from → q_to. 소보간 전진, 잃으면 그 자리에서 멈춰 **되돌아가**
        되찾은 뒤 더 잘게 재전진(최대 BAND_RELOC_TRIES). 끝까지 못 찾으면 목표로
        가서 나간다(이후 should_stop 이 lost 로 처리 — 밴드 부분 성공 경로).
        결과는 이벤트 로그 `band_move` 로 남긴다(outcome ok/relocalized/failed)."""
        step = self.BAND_STEP_M
        path = self.band_path_fn(q_from, q_to, step)
        evb = dict(stage=self.s.stage_label, from_band=band_i, to_band=band_i + 1)
        if not path:
            print("  [밴드이동] 경로 맥락 없음 — 한 방 이동(예전 방식)")
            self.move_robot_fn(q_to)
            _ev("band_move", steps=1, outcome="direct", **evb)
            return
        print(f"  [밴드이동] 조준 유지 소보간 {len(path)}스텝 (스텝 {step*1000:.0f}mm)")
        lost_alpha = None; found_alpha = None
        for attempt in range(0, self.BAND_RELOC_TRIES + 1):
            k_lost = self._walk_path(session, tracking, path, "전진" if attempt == 0 else "재전진",
                                     mode="fwd")
            if k_lost >= 0 and lost_alpha is None:
                lost_alpha = float(path[k_lost][1])
            if k_lost < 0 and self._wait_reg(session, tracking, self.BAND_RELOC_WAIT_S):
                if attempt > 0:
                    print(f"  [밴드이동] ✓ 되찾고 도착 (되돌아가기 {attempt}회)")
                _ev("band_move", steps=len(path), step_mm=step * 1000,
                    outcome=("ok" if attempt == 0 else "relocalized"), tries=attempt,
                    lost_alpha=lost_alpha, found_alpha=found_alpha, **evb)
                return
            if attempt == self.BAND_RELOC_TRIES:
                break
            # 되돌아가기: 잃은 자리(또는 끝)에서 q_from 쪽으로, 되찾을 때까지.
            k_lost = len(path) - 1 if k_lost < 0 else k_lost
            with tracking._lock:
                ce = tracking.consecutive_reg_err
            print(f"  [밴드이동] ⚠ 정합 없음 (연속 reg<0 {ce}) — 되돌아가 되찾기 "
                  f"{attempt+1}/{self.BAND_RELOC_TRIES}")
            back = [path[j] for j in range(k_lost - 1, -1, -1)] + [(np.asarray(q_from, float), 0.0)]
            k = self._walk_path(session, tracking, back, "후진", mode="back",
                                wait_s=self.BAND_RELOC_WAIT_S)
            if k < 0:
                print("  [밴드이동] ✘ 직전 밴드 자세까지 돌아가도 정합 없음 — 포기")
                break
            a_found = back[k][1]
            found_alpha = float(a_found)
            step *= 0.5
            path = self.band_path_fn(q_from, q_to, step, alpha_from=a_found, seed=back[k][0])
            if not path:
                break
            print(f"  [밴드이동] α={a_found:.2f} 에서 되찾음 — 스텝 {step*1000:.1f}mm 로 재전진 "
                  f"{len(path)}스텝")
        print("  [밴드이동] ✘ 되찾기 실패 — 목표 자세로 가서 진행 (이 밴드는 lost 로 처리될 수 있다)")
        _ev("band_move", steps=len(path), step_mm=step * 1000, outcome="failed",
            tries=self.BAND_RELOC_TRIES, lost_alpha=lost_alpha, found_alpha=found_alpha, **evb)
        self.move_robot_fn(q_to)

    def _track_standoff(self, session, band_q, band_i: int, n_bands: int):
        """밴드 시작에서 축거리를 작동거리 창 중앙으로 보정하고, **트래커를 돌려준다**.

        반환 = (tracker, 현재 축거리 m, retarget(d)->bool) 또는 None.
        판단은 sim 과 **같은** `StandoffTracker` 가 한다. 실패(측정 없음·IK·충돌)는
        전부 무시하고 계획 거리 그대로 간다 — 밴드를 버리는 것보다 낫다.

        ★ 2026-09-18 까지 real 은 여기 **밴드 시작 1회**뿐이었고 `live` 모드는
          sim 에만 배선돼 있었다. 이제 회전 루프가 반환된 트래커로 `TRACK_EVERY`
          OK 프레임마다 `update()` 를 부른다(아래 밴드 루프 "거리추종 live").
        """
        if (self.retarget_fn is None or self.standoff_of is None
                or self.move_robot_fn is None or band_q is None):
            return None
        from utils.nbv.standoff import (StandoffTracker, core_mask_camera_frame,
                                        TRACK_FRESH_SETTLE_S)
        d0 = self.standoff_of(band_q)
        if d0 is None:
            return None
        dof = self.scan_range or (0.20, 0.30)
        # ★ 표면거리는 **밴드 핵심 높이대**(광축 세로 ±9.5°)의 점으로 잰다.
        #   프레임 전체 중앙값은 내려다보는 자세에서 가까운 윗부분에 끌려 카메라를
        #   물리고, 그러면 창의 먼 끝이 이 밴드가 맡은 아랫부분을 자른다
        #   (2026-09-18 sim 실측: tz=32mm 밴드가 50~138mm 만 잡음). sim 은 같은
        #   것을 base 높이 ±40mm 로 넘긴다 — 정의는 `standoff.TRACK_CORE_HALF_*`.
        trk = StandoffTracker(dof, max(0.05, d0 - 0.12), d0 + 0.12,
                              log=lambda m: print(f" {m}"),
                              core=core_mask_camera_frame)
        if not trk.enabled:
            return None
        if self._latest_frame_ranges_m(session) is None:
            print(f"  [거리추종] band {band_i+1}/{n_bands} — OK 프레임이 없어 건너뜀")
            return None

        def _retarget(d_new) -> bool:
            qn = self.retarget_fn(band_q, float(d_new))
            if qn is None:
                return False
            try:
                code = self.move_robot_fn(qn)
            except Exception as e:                              # noqa: BLE001
                print(f"  [거리추종] 이동 실패({e})")
                return False
            # 게이트 거부(-1)·드라이버 오류코드는 "안 움직였다" — 거리를 갱신하면 안 된다.
            if isinstance(code, (int, np.integer)) and int(code) != 0:
                print(f"  [거리추종] 이동 거부(code={int(code)}) — 유지")
                return False
            return True

        cam0 = np.zeros(3)                       # 스캐너 프레임 원점 = 카메라
        # 회전 전(정지)이라 제한을 유지한 채 여러 번 수렴시킨다 — sim 과 동일.
        # ★ 매 측정은 **이동 뒤 새 프레임**이어야 한다(`fresh_settle_s`) — 큐에 남은
        #   이동 전 프레임을 읽으면 같은 값으로 세 번 연속 밀어낸다(run_150720).
        d1 = trk.converge(float(d0),
                          lambda: (self._latest_frame_ranges_m(
                              session, fresh_settle_s=TRACK_FRESH_SETTLE_S), cam0),
                          _retarget)
        if d1 != d0 and self.s.band_settle_s > 0:
            # 움직였으면 SLAM 이 새 시점에서 다시 정착할 시간을 준다 — 밴드
            # 경계 이동에 settle 을 두는 것과 같은 이유다.
            t0 = time.time()
            while time.time() - t0 < self.s.band_settle_s:
                session.poll_events()
                time.sleep(self.s.poll_interval_s)
        return trk, float(d1), _retarget

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
