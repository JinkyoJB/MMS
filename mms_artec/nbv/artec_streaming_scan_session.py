# mms/nbv/artec_streaming_scan_session.py
#
# Artec **IScanningProcedure** (streaming) 기반 Phase 1.
#
# 기존 `artec_scan_session.py` 의 discrete (15° × 24) 방식과 다르게:
#   - 턴테이블을 **연속 회전** (천천히, 정속)
#   - Spider 가 **최대 FPS** 로 실시간 capture + reconstruct + register
#   - SDK 가 frame-to-frame 정합 자동 수행 (RegistrationType=ICP)
#   - 결과 IModel 은 이미 register 된 상태 → SerialReg 생략 가능
#
# 우리 시스템에서의 의미:
#   - 카메라는 B frame 에 고정, 턴테이블이 객체 회전
#   - Artec SDK 입장: 객체가 정지, 카메라가 객체 둘레를 도는 것으로 해석 (등가 motion)
#   - 15° step 의 discrete capture 보다 frame 간 overlap 큼 (SLAM 친화적)
#
# Notation: T_AB : A → B (x_B = T_AB @ x_A) — README.md 준수.

from __future__ import annotations

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
    rotation_duration_s: float = 30.0      # 360° 한 바퀴 시간 (s)
    rotation_overshoot_deg: float = 5.0    # 360° + 여유 (frame 마지막까지 capture 보장)

    # ── 스캐너 ────────────────────────────────────────────────────────
    target_fps: Optional[float] = None     # None = max_fps 사용
    capture_texture: bool = True           # 텍스처 매핑 활성

    # ── 등록 / 파이프라인 ─────────────────────────────────────────────
    registration_type: int = int(artec_scanning.RegistrationType.ICP)
    pipeline_flags: int = (
        int(artec_scanning.ScanningPipelineFlags.REGISTER_FRAME) |
        int(artec_scanning.ScanningPipelineFlags.FIND_GEOMETRY_KEYFRAME) |
        int(artec_scanning.ScanningPipelineFlags.MAP_TEXTURE) |
        int(artec_scanning.ScanningPipelineFlags.CONVERT_TEXTURES)
    )
    sensitivity: Optional[float] = None    # None = SDK 기본값
    scan_range_near_mm: Optional[float] = None   # None = SDK 기본값
    scan_range_far_mm:  Optional[float] = None   # None = SDK 기본값
    ignore_registration_errors: bool = True      # 일부 frame 등록 실패해도 계속

    # ── 타이밍 ────────────────────────────────────────────────────────
    preview_settle_s: float = 1.5         # preview 진입 후 안정화 대기
    post_record_settle_s: float = 0.5     # 회전 끝난 뒤 record 끝내기 전 대기

    # ── 기타 ──────────────────────────────────────────────────────────
    reset_to_zero_first: bool = True      # 시작 전 θ=0 reset


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


# ─────────────────────────────────────────────────────────────────────────────
# Session
# ─────────────────────────────────────────────────────────────────────────────

class ArtecStreamingScanSession:
    """
    연속 회전 + 실시간 SLAM 기반 Artec Phase 1.

    Usage
    -----
    >>> sess = ArtecStreamingScanSession(mms, robot, turntable, settings)
    >>> result = sess.run()
    >>> model = result.model    # 등록된 frames 가 들어있는 IModel
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
            raise RuntimeError("ArtecStreamingScanSession 는 ArtecClient 만 지원.")
        if turntable is None:
            raise RuntimeError("turntable 필수.")

    # ── Entry ──────────────────────────────────────────────────────────

    def run(self) -> ArtecStreamingScanResult:
        s = self.s

        # 1. Scanner FPS — 최대로
        scanner = self.mms.sensor._scanner
        max_fps = float(scanner.max_fps())
        target_fps = s.target_fps if s.target_fps else max_fps
        target_fps = min(target_fps, max_fps)
        try:
            scanner.set_fps(target_fps)
        except Exception as e:
            print(f"  [warn] set_fps({target_fps}) 실패: {e}")
            target_fps = max_fps

        print(f"\n═══════════════ Artec Streaming Phase 1 ═══════════════")
        print(f"  scanner_fps         : {target_fps:.1f} / max {max_fps:.1f}")
        print(f"  rotation_duration_s : {s.rotation_duration_s:.1f}")
        print(f"  registration_type   : {artec_scanning.RegistrationType(s.registration_type).name}")
        print(f"  capture_texture     : {s.capture_texture}")
        print(f"  expected frames     : ~{int(target_fps * s.rotation_duration_s)}")

        # 2. ScanSession 설정
        settings = artec_scanning.ScanSessionSettings.default()
        settings.set_registration_type(artec_scanning.RegistrationType(s.registration_type))
        settings.set_pipeline(s.pipeline_flags)
        settings.set_initial_state(artec_scanning.ScanningState.PREVIEW)
        if s.capture_texture:
            settings.set_capture_texture(artec_scanning.CaptureTextureMethod.ALWAYS)
        else:
            settings.set_capture_texture(artec_scanning.CaptureTextureMethod.NONE)
        settings.set_ignore_registration_errors(s.ignore_registration_errors)

        session = artec_scanning.ScanSession.create(scanner, settings)

        # sensitivity / scan range — 옵션
        if s.sensitivity is not None:
            try:
                session.set_sensitivity(float(s.sensitivity))
            except Exception:
                pass
        if s.scan_range_near_mm is not None and s.scan_range_far_mm is not None:
            try:
                session.set_scanning_range(
                    float(s.scan_range_near_mm), float(s.scan_range_far_mm),
                )
            except Exception:
                pass

        # 3. 턴테이블 θ=0 reset
        if s.reset_to_zero_first and self.turntable.is_connected:
            cur = self._read_theta(0.0)
            if abs(cur) > np.radians(0.5):
                print(f"  reset turntable to θ=0 (현재 {np.degrees(cur):+.2f}°)")
                self.turntable.move_abs(0.0, np.radians(15.0))
                self.turntable.wait_motion_done(timeout_s=15.0)

        # 4. Preview → settle
        print(f"\n  start_preview ...")
        session.start_preview()
        time.sleep(s.preview_settle_s)

        # 5. Record 시작 + 연속 회전 (try/finally 로 stop 보장)
        print(f"  start_record + 턴테이블 연속 회전 시작")
        session.start_record()
        t_start = time.time()
        last_pos = 0.0

        # 회전 속도: 360° / duration_s
        vel_rad_s = (2.0 * np.pi) / s.rotation_duration_s
        rotation_started = False

        try:
            ok = self.turntable.move_velocity(vel_rad_s, direction=0)
            rotation_started = ok
            if not ok:
                print("  ⚠ move_velocity 실패 — record 만 진행")

            # 6. 회전 완료까지 polling
            target_rad = 2.0 * np.pi * (1.0 + s.rotation_overshoot_deg / 360.0)
            timeout_s = s.rotation_duration_s + 5.0
            while True:
                elapsed = time.time() - t_start
                if elapsed >= timeout_s:
                    print(f"  rotation timeout {timeout_s:.1f}s — 강제 종료")
                    break
                pos = self.turntable.getActualPos()
                if isinstance(pos, bool) or pos is None:
                    time.sleep(0.1)
                    continue
                last_pos = float(pos)
                if last_pos >= target_rad - np.radians(1.0):
                    print(f"  rotation 완료: {np.degrees(last_pos):.1f}°  ({elapsed:.1f}s)")
                    break
                time.sleep(0.05)
        finally:
            # 7. **반드시** 턴테이블 정지 — 예외 / KeyboardInterrupt 시에도
            if rotation_started:
                self._safe_stop_turntable()

        # 8. Record 마저 진행 (last frames 캡처) → stop
        time.sleep(s.post_record_settle_s)
        print(f"  session.stop() ...")
        try:
            model = session.stop()
        except Exception as e:
            print(f"  ⚠ session.stop() 예외: {e}")
            model = None
        t_end = time.time()
        duration = t_end - t_start

        if model is None:
            print(f"  ⚠ session.stop() 결과 None")
            empty = artec_base.create_model()
            return ArtecStreamingScanResult(model=empty, n_frames=0, duration_s=duration)

        # 결과 통계
        n_frames = 0
        for i in range(model.scan_count()):
            n_frames += model.get_scan(i).frame_count()
        fps_actual = n_frames / max(duration, 1e-6)

        print(f"\n[StreamingScan 완료]")
        print(f"  scans          : {model.scan_count()}")
        print(f"  total frames   : {n_frames}")
        print(f"  duration       : {duration:.1f} s")
        print(f"  effective fps  : {fps_actual:.1f}")
        print(f"  rotation       : {np.degrees(last_pos):.1f}°")

        # 회전축 0 으로 복귀 (사용자 편의)
        try:
            self.turntable.move_abs(0.0, np.radians(15.0))
            self.turntable.wait_motion_done(timeout_s=10.0)
        except Exception:
            pass

        return ArtecStreamingScanResult(
            model=model,
            n_frames=n_frames,
            rotation_actual_deg=float(np.degrees(last_pos)),
            duration_s=duration,
            fps_actual=fps_actual,
        )

    # ── 유틸 ──────────────────────────────────────────────────────────

    def _read_theta(self, fallback: float) -> float:
        if not self.turntable.is_connected:
            return float(fallback)
        v = self.turntable.getActualPos()
        if isinstance(v, bool) or v is None:
            return float(fallback)
        return float(v)

    def _safe_stop_turntable(self) -> None:
        """예외/인터럽트 시에도 안전하게 턴테이블 정지 — 여러 번 시도."""
        for attempt in range(3):
            try:
                ok = self.turntable.stop()
                if ok:
                    print(f"  [Turntable] stop OK (시도 {attempt+1})")
                    return
            except Exception as e:
                print(f"  [Turntable] stop 예외 ({attempt+1}/3): {e}")
            time.sleep(0.2)
        # 마지막 수단: reconnect 후 stop
        try:
            if hasattr(self.turntable, "reconnect"):
                print("  [Turntable] reconnect 후 stop 재시도")
                self.turntable.reconnect()
                self.turntable.stop()
        except Exception as e:
            print(f"  [Turntable] 최종 stop 실패: {e}  ⚠ 수동 정지 필요")
