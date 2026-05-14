# mms_artec/nbv/artec_multipass_scan_session.py
#
# Phase 1 + Phase 2 통합 오케스트레이터.
# ArtecStreamingScanSession 한 번 = 한 IScan = 한 회전.
# 이 클래스는 그것을 N 번 반복하며 모든 IScan 을 master IModel 에 누적한다.
#
# Phase 정의 (docs/7_artec_phase.md):
#  - Phase 1: 객체 canonical 자세에서 turntable 360° → 윗면 + 옆면 4 (= 5면) 캡처.
#  - Phase 2: 객체 자세를 바꿔 (예: Ry+90°, Ry+180°) 바닥면 + overlap 옆면 캡처
#             후 Phase 1 데이터와 정합. centroid-pivot pre-rotation hint 사용
#             (docs/8_artec_phase2_pose_disambiguation.md §5.3).
#
# 동기:
#  - SDK 의 ScanningState_ContinueRecord 는 미지원이라 한 IScan 안에서 pause/resume 불가.
#  - 대신 multi-IScan 으로 분할 → master IModel 에 누적, hint 로 정합.
#  - 이는 Artec Studio 의 표준 multi-scan 워크플로우와 동일.
#
# 두 가지 흐름이 같은 메커니즘으로 처리됨:
#  - Tracking lost recovery: 같은 pose 로 retry, hint 동일.
#  - Phase 2 의 자세 변경: 다음 pose advance, 새 hint.
#
# Notation: T_AB : A → B  (CLAUDE.md 준수).

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import List, Optional, TYPE_CHECKING

import numpy as np

from mms_artec.sensor import artec_base
from mms_artec.nbv.artec_streaming_scan_session import (
    ArtecStreamingScanSession,
    ArtecStreamingScanSessionSettings,
    ArtecStreamingScanResult,
)
from mms_artec.nbv.recovery_pose_selector import (
    RecoveryPoseSelector,
    master_points_in_base_frame,
)
from utils.transforms import pose_mat_to_6d


# ─────────────────────────────────────────────────────────────────────────────
# Pose hint helpers — Pre-rotation hint 의 사용자 친화 빌더.
# 사용자는 "객체의 물리 회전" 을 입력. 내부적으로 inverse 가 IScan frame
# transformation 에 좌측 곱돼 GlobalReg 의 local-minimum 함정 회피.
# ─────────────────────────────────────────────────────────────────────────────

def _R_axis(axis: str, deg: float) -> np.ndarray:
    """4x4 rotation matrix around axis ∈ {x, y, z}."""
    c, s = np.cos(np.radians(deg)), np.sin(np.radians(deg))
    T = np.eye(4)
    if axis == "x":
        T[:3, :3] = [[1, 0, 0], [0, c, -s], [0, s, c]]
    elif axis == "y":
        T[:3, :3] = [[c, 0, s], [0, 1, 0], [-s, 0, c]]
    elif axis == "z":
        T[:3, :3] = [[c, -s, 0], [s, c, 0], [0, 0, 1]]
    else:
        raise ValueError(f"axis must be x/y/z, got {axis!r}")
    return T


def make_axis_physical_rotations(
    axis: str, angles_deg: List[float],
) -> List[np.ndarray]:
    """
    Pose 별 사용자 물리 회전 (단일 축 기준). **base frame B** (world 좌표,
    Z up) 에서 정의된 회전.

    Example
    -------
    >>> make_axis_physical_rotations("y", [0, 90, 180])
    # [I, Ry_B(90°), Ry_B(180°)] — base Y 축 기준 0/90/180° 회전.
    # Right-hand convention. base Y 축 기준 +90° 는 -x_B → +z_B
    # (= 옆면이 윗자리로) 회전.

    Notes
    -----
    회전축은 **base 프레임** 에서 의미: world Z 가 up, X/Y 가 horizontal.
    Multipass session 의 __init__ 이 T_BC 를 캡처해 scan world frame 으로
    내부 변환. Spider 가 EE 에 마운트돼있어 camera 의 Y 축과 base 의 Y 축이
    다를 수 있어서 명시적 변환 필수.
    """
    return [_R_axis(axis, deg) for deg in angles_deg]

if TYPE_CHECKING:
    from mms_artec.system import ArtecMMS as MMS
    from utils.robot.xarm_interface import XArmInterface
    from utils.turntable.turntable_interface import Turntable


# ─────────────────────────────────────────────────────────────────────────────
# Settings / Result
# ─────────────────────────────────────────────────────────────────────────────

@dataclass
class ArtecMultiPassScanSessionSettings:
    """Multi-pass scan orchestrator settings."""

    # 단일 pass 의 설정. None 이면 기본값.
    streaming_settings: Optional[ArtecStreamingScanSessionSettings] = None

    # 안전 한계 — 사용자가 'q' 누르지 않아도 이 횟수에서 자동 종료.
    max_passes: int = 6

    # 첫 pass 전 prompt (확인용). False = 즉시 시작.
    prompt_before_first_pass: bool = True

    # Pass 정상 완료 후 다음 pass 진행 여부 prompt.
    # False 면 한 pass 성공 즉시 종료 — 단일 회전 모드와 동일.
    prompt_between_passes: bool = True

    # Pass 도중 tracking lost 발생 시 retry prompt.
    # False 면 lost 즉시 종료 — 자동화 친화 모드.
    prompt_on_tracking_lost: bool = True

    # ── Pre-rotation hint (docs/8) ──────────────────────────────────────
    # 사용자가 각 pose 에서 객체에 가한 물리 회전 (4x4 행렬 list).
    # GlobalReg 의 local-minimum 함정 회피 위해, IScan 의 frame transformation
    # 에 inv() 를 좌측 곱. 빈 list = hint 없음 (기존 동작).
    # 길이 = pose 개수. Tracking lost 재시도는 pose_idx 유지하므로 retry 도
    # 같은 hint 사용.
    # 자주 쓰는 preset 은 make_axis_physical_rotations() helper 사용.
    pose_physical_rotations: List[Optional[np.ndarray]] = field(default_factory=list)

    # ── Tracking-lost auto-recovery ─────────────────────────────────────
    # None 이면 기존 동작 (user prompt). 인스턴스 주면 자동 복구:
    #   1. last-good 각도 + safe_back_margin_deg 까지 turntable 역회전
    #   2. selector 가 다음 카메라 pose 결정 → robot 이동
    #   3. 다음 streaming pass 진행 (pose_idx 유지)
    # 같은 pose 안에서 연속 max_recovery_retries 회까지 자동 시도, 초과하면
    # user prompt 로 fallback.
    recovery_selector: Optional[RecoveryPoseSelector] = None
    max_recovery_retries: int = 3
    safe_back_margin_deg: float = 10.0
    # Recovery 시 robot 이동 속도 (deg/s) — collision 위험 최소화 위해 보수적.
    recovery_robot_speed_deg_s: float = 10.0
    # Recovery turntable 역회전 속도 (rad/s).
    recovery_turntable_vel_rad_s: float = float(np.radians(30.0))

    def __post_init__(self):
        if self.streaming_settings is None:
            self.streaming_settings = ArtecStreamingScanSessionSettings()


@dataclass
class ArtecMultiPassScanResult:
    model: artec_base.ModelHandle                   # master with N IScans
    n_passes: int = 0
    n_total_frames: int = 0
    pass_results: List[ArtecStreamingScanResult] = field(default_factory=list)
    user_quit: bool = False                         # 사용자가 'q' 눌러 종료
    aborted_reason: str = ""
    hints_applied: bool = False                     # pose_physical_rotations 가
                                                    # 실제로 한 번이라도 적용됐는가
    master_center_mm: Optional[np.ndarray] = None   # Pass 1 의 centroid (mm)
    n_recovery_attempts: int = 0                    # 자동 recovery 가 trigger 된 횟수
    n_recovery_succeeded: int = 0                   # recovery 후 정상 완료된 pass 수


# ─────────────────────────────────────────────────────────────────────────────
# Session
# ─────────────────────────────────────────────────────────────────────────────

class ArtecMultiPassScanSession:
    """
    Run multiple Phase 1 rotations, accumulating all IScans into a master IModel.

    Each pass:
      1. Prompt 사용자 (필요시).
      2. ArtecStreamingScanSession.run() — 한 회전.
      3. Sub-model 의 IScan 들을 master_model 에 추가 (SDK addRef).
      4. Tracking lost 였으면 retry prompt, 정상이면 next-pass prompt.

    종료 조건:
      - 사용자 'q' 입력
      - max_passes 도달
      - prompt_* 플래그 False 인 경우 해당 분기에서 즉시 종료

    Returns ArtecMultiPassScanResult — master IModel 은 N IScans 보유.
    이후 ArtecMMS.artec_process 가 GlobalRegistration 으로 정합.
    """

    def __init__(
        self,
        mms: "MMS",
        robot: Optional["XArmInterface"],
        turntable: "Turntable",
        settings: Optional[ArtecMultiPassScanSessionSettings] = None,
    ):
        self.mms = mms
        self.robot = robot
        self.turntable = turntable
        self.s = settings or ArtecMultiPassScanSessionSettings()

        # ── T_BC 캡처 (B → C = scan world W) ─────────────────────────────
        # pose_physical_rotations 는 base frame B 에서 정의됨 (사용자 직관:
        # world Z up). hint 를 scan world W 에 적용하려면 R_W = T_BC @ R_B @ T_CB
        # 로 변환 필요. Spider 가 EE 에 마운트되어 home pose 에서 T_EC 만큼
        # 회전돼있어 base Y ≠ scan world Y. 변환 없으면 회전축이 잘못 적용돼
        # mesh 가 '바람개비' 처럼 회전됨.
        self._T_BC: Optional[np.ndarray] = None
        self._T_CB: Optional[np.ndarray] = None
        self._T_EC: Optional[np.ndarray] = None     # E → C (fixed, hand-eye)
        try:
            T_EC = getattr(self.mms, "_T_EC", None)
            if T_EC is None or self.robot is None:
                raise RuntimeError("T_EC 또는 robot 미설정")
            self._T_EC = np.asarray(T_EC, dtype=float).copy()
            T_EB = self.robot.get_ee_pose_mat()       # E → B (xarm)
            T_BE = np.linalg.inv(T_EB)                 # B → E
            self._T_BC = T_EC @ T_BE                   # B → C
            self._T_CB = np.linalg.inv(self._T_BC)
            # T_BE 가 m 단위 (xarm pose6d_to_mat 의 mm→m 자동 변환) 이므로
            # T_BC translation 도 m. 회전부 (3x3) 만 hint 에 사용 — translation
            # 은 unit 무관.
            t_m = self._T_BC[:3, 3]
            print(f"  [multipass] T_BC captured — hints in base frame B "
                  f"(translation = ({t_m[0]*1000:+.1f}, {t_m[1]*1000:+.1f}, "
                  f"{t_m[2]*1000:+.1f}) mm)")
        except Exception as e:
            print(f"  [multipass] ⚠ T_BC 캡처 실패 ({e}) — "
                  f"hints scan-world frame 그대로 적용 (좌표축 안 맞을 가능성)")

    # ── Entry ──────────────────────────────────────────────────────────

    def run(self) -> ArtecMultiPassScanResult:
        s = self.s
        master_model = artec_base.create_model()
        pass_results: List[ArtecStreamingScanResult] = []
        user_quit = False
        aborted_reason = ""
        n_pass = 0
        pose_idx = 0   # 현재 pose 의 hint 인덱스. 정상 완료 시만 advance.
        master_center: Optional[np.ndarray] = None  # Pass 1 후 lock
        hints_applied = False                       # 한 번이라도 적용?
        # Recovery 상태 — auto-recovery 가 set 하면 다음 iteration 의
        # 1) streaming session pre-scan clearpos skip
        # 2) merge 단계 camera-motion T_pre override
        # 에 사용. 정상 완료된 pass 후 reset.
        recovery_retry_count = 0          # 같은 pose 안에서 누적 (성공 시 0)
        n_recovery_attempts = 0           # 총 시도 (전체 run)
        n_recovery_succeeded = 0          # 총 성공
        next_T_BC_pending: Optional[np.ndarray] = None
        next_skip_clearpos = False

        while n_pass < s.max_passes:
            self._print_pass_banner(n_pass + 1, s.max_passes, pose_idx)

            # 첫 pass 만 따로 prompt — Studio 시작 위치 확인 등.
            if n_pass == 0 and s.prompt_before_first_pass:
                self._print_pose_hint_for_user(pose_idx)
                print("  [Enter] 회전 시작 / [q]+Enter 종료")
                if self._wait_user_quit():
                    user_quit = True
                    aborted_reason = "사용자 종료 (첫 pass 전)"
                    break

            # ── Recovery 직후면 streaming 의 pre-scan clearpos skip ──
            # 우리가 이미 turntable 을 safe-back 으로 이동시켰음. clearpos 가
            # 그 위치를 새 0° 로 redefine 하면 의도와 어긋남.
            _orig_reset_to_zero = s.streaming_settings.reset_to_zero_first
            if next_skip_clearpos:
                s.streaming_settings.reset_to_zero_first = False

            # ── 단일 회전 ─────────────────────────────────────────────
            try:
                single = ArtecStreamingScanSession(
                    self.mms, self.robot, self.turntable, s.streaming_settings,
                )
                sub_result = single.run()
            finally:
                # streaming_settings 는 multipass 인스턴스 외부에서 공유될 수
                # 있으므로 반드시 원복.
                s.streaming_settings.reset_to_zero_first = _orig_reset_to_zero
                next_skip_clearpos = False
            pass_results.append(sub_result)
            n_pass += 1

            # ── Pose hint 계산 (centroid-aware + base-frame aware) ─────
            # 1. R_phys 는 base frame B 에서 정의 (사용자 직관)
            # 2. T_BC 있으면 R_W = T_BC @ R_B @ T_CB 로 scan world 로 변환
            # 3. T_pre = Translate(c_master) @ inv(R_W) @ Translate(-c_pass)
            #    객체 centroid 를 pivot 으로 회전 → camera 원점 기준 30cm
            #    translation 오류 제거.
            T_pre = None

            # Recovery override: 직전 iteration 에서 robot 이 움직였다면
            # camera-motion 만 보정 (object 회전 무관, pose_idx 그대로). 다른
            # R_phys hint 보다 우선.
            #   x_W_master = T_BC_master @ T_CB_new @ x_W_new
            #              = T_BC_master @ inv(T_BC_new) @ x_W_new
            # SDK frame_transformation 은 mm 라 translation 만 m→mm scale.
            if next_T_BC_pending is not None:
                if self._T_BC is None:
                    print(f"  [recovery hint] ⚠ T_BC_master 미설정 — override skip")
                else:
                    try:
                        T_pre_cam = self._T_BC @ np.linalg.inv(next_T_BC_pending)
                        T_pre = T_pre_cam.copy()
                        T_pre[:3, 3] *= 1000.0          # m → mm (SDK 단위)
                        print(f"\n  [recovery hint] camera-motion correction "
                              f"applied (Δtrans = ({T_pre[0,3]:+.1f}, "
                              f"{T_pre[1,3]:+.1f}, {T_pre[2,3]:+.1f}) mm)")
                    except np.linalg.LinAlgError as e:
                        print(f"  [recovery hint] ⚠ inv 실패 ({e}) — override skip")
                next_T_BC_pending = None

            R_phys = (s.pose_physical_rotations[pose_idx]
                      if pose_idx < len(s.pose_physical_rotations) else None)
            if T_pre is None and R_phys is not None and not np.allclose(R_phys, np.eye(4), atol=1e-9):
                try:
                    R_phys_inv_B = np.linalg.inv(R_phys)[:3, :3]
                except np.linalg.LinAlgError:
                    print(f"  [hint] ⚠ pose_physical_rotations[{pose_idx}] inverse 실패 — skip")
                    R_phys_inv_B = None
                if R_phys_inv_B is not None:
                    # B → W 변환 (T_BC 있으면)
                    if self._T_BC is not None and self._T_CB is not None:
                        R_phys_inv_W = (self._T_BC[:3, :3] @ R_phys_inv_B
                                        @ self._T_CB[:3, :3])
                        frame_tag = "base"
                    else:
                        R_phys_inv_W = R_phys_inv_B
                        frame_tag = "scan_world (fallback)"

                    c_pass = self._compute_model_centroid(sub_result.model)
                    c_ref = master_center if master_center is not None else c_pass
                    T_pre = np.eye(4)
                    T_pre[:3, :3] = R_phys_inv_W
                    T_pre[:3, 3] = c_ref - R_phys_inv_W @ c_pass
                    hints_applied = True
                    print(f"\n  [hint pose {pose_idx}] frame={frame_tag}")
                    print(f"  [hint pose {pose_idx}] c_pass = ({c_pass[0]:+.1f}, "
                          f"{c_pass[1]:+.1f}, {c_pass[2]:+.1f}) mm")
                    print(f"  [hint pose {pose_idx}] c_master = ({c_ref[0]:+.1f}, "
                          f"{c_ref[1]:+.1f}, {c_ref[2]:+.1f}) mm")
                    print(f"  [hint pose {pose_idx}] translation = ({T_pre[0,3]:+.1f}, "
                          f"{T_pre[1,3]:+.1f}, {T_pre[2,3]:+.1f}) mm")

            # Sub-model 의 IScan 들 → master_model (hint 적용 후)
            n_added = self._merge_into_master(
                sub_result.model, master_model, T_pre,
            )
            print(f"\n  [pass {n_pass} / pose {pose_idx}] {n_added} scan(s) → master "
                  f"(total scans={master_model.scan_count()})")

            # Pass 1 (또는 첫 성공한 IScan) 후 master_center lock
            if master_center is None and master_model.scan_count() > 0:
                master_center = self._compute_model_centroid(master_model)
                print(f"  [master_center] locked at "
                      f"({master_center[0]:+.1f}, {master_center[1]:+.1f}, "
                      f"{master_center[2]:+.1f}) mm")

            # ── 분기: tracking lost 였나 정상 완료였나 ────────────────
            if sub_result.tracking_lost:
                # Drive 통신 사망 / alarm trip — retry 해도 못 풀림. 즉시 종료.
                # TurntableController 가 watchdog 으로 emergency_stop 시도하지만
                # 그래도 안 멈출 수 있어 사용자에게 물리 전원 차단 안내.
                lr = (sub_result.loss_reason or "").lower()
                drive_dead = ("drive alarm" in lr
                              or "drive 통신 사망" in lr
                              or "alarm trip" in lr
                              or "getactualpos" in lr)
                if drive_dead:
                    aborted_reason = (
                        f"turntable drive alarm / 통신 사망 — "
                        f"EziSERVO 전원 OFF→5s→ON 필요"
                    )
                    print(f"\n  ✘ {aborted_reason}")
                    print(f"  ✘ 모터가 안 멈추면 emergency_stop 도 실패한 것 — "
                          f"물리 전원 차단해야 함.")
                    print(f"  ✘ 복구 후 main_artec.py 재실행.")
                    break

                # ── 자동 recovery 시도 ─────────────────────────────────
                # selector 있고, max_recovery_retries 미만이면 turntable
                # safe-back + scanner 재배치로 자동 재시도.
                recovery_initiated = False
                if (s.recovery_selector is not None
                        and recovery_retry_count < s.max_recovery_retries):
                    ok, T_BC_new = self._attempt_recovery(
                        sub_result, master_model, recovery_retry_count,
                    )
                    if ok:
                        recovery_retry_count += 1
                        n_recovery_attempts += 1
                        # robot 이 움직였으면 다음 merge 의 T_pre override 용
                        if T_BC_new is not None:
                            next_T_BC_pending = T_BC_new
                        # turntable 을 safe-back 으로 manual 이동했으므로
                        # 다음 streaming session 은 pre-scan clearpos skip.
                        next_skip_clearpos = True
                        recovery_initiated = True
                        print(f"  → recovery #{recovery_retry_count}/"
                              f"{s.max_recovery_retries} 진입 — 자동 재시도")
                elif (s.recovery_selector is not None
                        and recovery_retry_count >= s.max_recovery_retries):
                    print(f"\n  ⓘ recovery 한계 도달 "
                          f"({recovery_retry_count}/{s.max_recovery_retries}) "
                          f"— user prompt 로 fallback")

                if recovery_initiated:
                    continue   # 다음 iteration 으로 (pose_idx 유지)

                # ── User-prompt fallback (기존 동작) ────────────────
                if not s.prompt_on_tracking_lost:
                    aborted_reason = (
                        f"tracking lost (auto, no prompt): {sub_result.loss_reason}"
                    )
                    print(f"  ⚠ {aborted_reason}")
                    break
                print(f"\n  ⚠ Pass {n_pass} tracking lost: {sub_result.loss_reason}")
                print(f"  → 같은 pose (pose {pose_idx}) 그대로 두고 [Enter] 재시도 / [q]+Enter 종료")
                if self._wait_user_quit():
                    user_quit = True
                    aborted_reason = "사용자 종료 (lost 직후)"
                    break
                # User 가 수동 retry 한 경우 — recovery_retry_count 는 reset
                # (수동 개입은 새 시작으로 간주).
                recovery_retry_count = 0
                # 다음 iteration 으로 진행 — pose_idx 유지 (재시도)
            else:
                # 정상 완료 — recovery 카운터 reset.
                if recovery_retry_count > 0:
                    n_recovery_succeeded += 1
                    print(f"\n  ✓ recovery 후 정상 완료 — retry 카운터 reset "
                          f"(누적 성공: {n_recovery_succeeded})")
                    recovery_retry_count = 0
                if not s.prompt_between_passes:
                    aborted_reason = "single-pass mode (no inter-pass prompt)"
                    break
                print(f"\n  ✓ Pass {n_pass} (pose {pose_idx}) 완료 — frames={sub_result.n_frames}")
                # 다음 pose 가 정의돼있으면 안내
                next_pose = pose_idx + 1
                if next_pose < len(s.pose_physical_rotations):
                    print(f"  → 다음 pose ({next_pose}) 자세로 아이템 회전 후 [Enter]")
                    self._print_pose_hint_for_user(next_pose)
                else:
                    print(f"  → 추가 pass 진행하려면 [Enter] (모든 정의된 pose 완료)")
                print(f"    종료하려면 [q]+Enter")
                if self._wait_user_quit():
                    user_quit = True
                    aborted_reason = "사용자 종료 (정상 완료 후)"
                    break
                pose_idx += 1   # 정상 완료 시만 advance

        if n_pass >= s.max_passes:
            aborted_reason = aborted_reason or f"max_passes={s.max_passes} 도달"
            print(f"\n  ⓘ {aborted_reason}")

        n_total_frames = sum(
            master_model.get_scan(i).frame_count()
            for i in range(master_model.scan_count())
        )

        print(f"\n═══ Multi-pass 종료 ═══")
        print(f"  passes              : {n_pass}")
        print(f"  master scan_count   : {master_model.scan_count()}")
        print(f"  master total frames : {n_total_frames}")
        print(f"  user_quit           : {user_quit}")
        if aborted_reason:
            print(f"  reason              : {aborted_reason}")

        if n_recovery_attempts > 0:
            print(f"  recovery            : {n_recovery_succeeded}/"
                  f"{n_recovery_attempts} 성공")

        return ArtecMultiPassScanResult(
            model=master_model,
            n_passes=n_pass,
            n_total_frames=n_total_frames,
            pass_results=pass_results,
            user_quit=user_quit,
            aborted_reason=aborted_reason,
            hints_applied=hints_applied,
            master_center_mm=master_center,
            n_recovery_attempts=n_recovery_attempts,
            n_recovery_succeeded=n_recovery_succeeded,
        )

    # ── Recovery ───────────────────────────────────────────────────────

    def _attempt_recovery(
        self,
        sub_result: ArtecStreamingScanResult,
        master_model: "artec_base.ModelHandle",
        retry_idx: int,
    ) -> tuple[bool, Optional[np.ndarray]]:
        """
        Tracking lost 발생 시 자동 복구 시도.

        흐름:
          1. last-good θ + safe_back_margin_deg 만큼 turntable 역회전 (move_abs).
          2. master point cloud → recovery_selector 가 다음 카메라 pose 결정.
          3. xArm 으로 robot 이동, 새 T_BC 캡처.

        Returns
        -------
        (ok, T_BC_new)
          ok        — recovery 진입 성공 여부. False 면 호출측이 user-prompt 로 fallback.
          T_BC_new  — robot 이동 후 새 B → C transform (next merge 의 T_pre 계산용).
                      master 가 비었거나 selector 가 None 반환하면 None
                      (turntable 만 safe-back, robot 미이동 의미).
        """
        s = self.s
        sel = s.recovery_selector
        if sel is None or self._T_EC is None or self._T_BC is None or self.robot is None:
            print(f"  ⚠ recovery 조건 미충족: selector/T_EC/robot 확인")
            return False, None

        # ── 1. safe-back 각도 ──────────────────────────────────────────
        # post-scan clearpos 가 이미 일어났으므로 현재 logical = 0.
        # 실패 scan 좌표계에서: last_good_θ, final_θ (= rotation_actual_deg).
        # delta_to_lastgood = last_good - final  (in failed coords)
        # 새 logical 좌표계 (post-clearpos) 에서 same physical offset = delta.
        # scan 방향 반대로 margin 만큼 더: target = delta + (-scan_dir_sign)*margin.
        final_rad = float(np.radians(sub_result.rotation_actual_deg))
        last_good_rad = float(sub_result.last_good_theta_rad)
        if abs(last_good_rad) < 1e-6 and abs(final_rad) > 1e-3:
            # tracking 한 번도 안 잡힘 (= last-good 없음). safe-back 의미 없음.
            print(f"  ⚠ recovery skip: tracking 초기에 실패해 last-good θ 없음")
            return False, None
        delta_to_lastgood = last_good_rad - final_rad
        scan_dir_sign = np.sign(final_rad) if abs(final_rad) > 1e-6 else -1.0
        margin_rad = float(np.radians(s.safe_back_margin_deg))
        safe_back_target_rad = float(delta_to_lastgood - scan_dir_sign * margin_rad)

        print(f"\n  ━━━━━━━━━━ Recovery #{retry_idx + 1}/"
              f"{s.max_recovery_retries} ━━━━━━━━━━")
        print(f"  failed scan: last-good θ = {np.degrees(last_good_rad):+.1f}°  "
              f"final θ = {np.degrees(final_rad):+.1f}°")
        print(f"  safe-back target (new coords): "
              f"{np.degrees(safe_back_target_rad):+.1f}° "
              f"(margin={s.safe_back_margin_deg:.1f}°)")

        # ── 2. Turntable safe-back ─────────────────────────────────────
        try:
            self.turntable.stop()
            time.sleep(0.1)
            self.turntable.check_drive_err()
            self.turntable.set_servo_on(True)
            time.sleep(0.1)
            ok = self.turntable.move_abs(
                safe_back_target_rad, s.recovery_turntable_vel_rad_s,
            )
            if not ok:
                print(f"  ✘ turntable.move_abs 실패 — fallback")
                return False, None
            # 완료 대기
            if hasattr(self.turntable, "wait_motion_done"):
                self.turntable.wait_motion_done(timeout_s=15.0)
            else:
                # fallback — drive 가 못 따라가도 좋게 충분히 sleep
                est_t = abs(safe_back_target_rad) / max(
                    s.recovery_turntable_vel_rad_s, 0.1,
                ) + 1.0
                time.sleep(min(est_t, 10.0))
            cur = self.turntable.getActualPos()
            if not isinstance(cur, bool) and cur is not None:
                print(f"  turntable @ {np.degrees(float(cur)):+.1f}° (target "
                      f"{np.degrees(safe_back_target_rad):+.1f}°)")
        except Exception as e:
            print(f"  ✘ safe-back 예외: {e}")
            return False, None

        # ── 3. Master 점운 → selector ──────────────────────────────────
        try:
            master_pts_B = master_points_in_base_frame(
                master_model, self._T_BC, max_points=30_000,
            )
        except Exception as e:
            print(f"  ⚠ master_points_in_base_frame 예외: {e} — selector skip")
            master_pts_B = np.zeros((0, 3), dtype=np.float64)

        if master_pts_B.shape[0] == 0:
            print(f"  ⚠ master 비어있음 — 같은 자리에서 turntable safe-back 만으로 재시도")
            return True, None

        # 현재 camera pose
        try:
            T_EB_now = self.robot.get_ee_pose_mat()
            T_BE_now = np.linalg.inv(T_EB_now)
            T_BC_now = self._T_EC @ T_BE_now
            T_CB_now = np.linalg.inv(T_BC_now)
        except Exception as e:
            print(f"  ✘ 현재 robot pose 읽기 실패: {e} — selector skip")
            return True, None

        decision = sel.select(T_CB_now, master_pts_B)
        if decision is None:
            print(f"  ⚠ selector '{sel.name}' 후보 없음 — 같은 자리 재시도")
            return True, None
        print(f"  selector       : {sel.name}")
        print(f"  decision       : {decision.debug_info}")
        print(f"  score          : {decision.score:.0f}")

        # 현재 pose 와 거의 같으면 robot 이동 skip
        delta_t_m = float(np.linalg.norm(
            decision.T_CB_target[:3, 3] - T_CB_now[:3, 3],
        ))
        if delta_t_m < 0.002:        # 2mm 미만 = no-op
            print(f"  → decision == 현재 pose (Δ={delta_t_m*1000:.1f}mm) "
                  f"— robot 미이동")
            return True, None

        # ── 4. xArm 이동 ───────────────────────────────────────────────
        # T_CB_target (C → B) → T_EB_target = T_CB_target @ T_EC.
        # pose_mat_to_6d 는 translation m, euler rad → xArm 은 mm + rad.
        try:
            T_EB_target = decision.T_CB_target @ self._T_EC
            pose6d = pose_mat_to_6d(T_EB_target)
            x_mm = float(pose6d[0] * 1000.0)
            y_mm = float(pose6d[1] * 1000.0)
            z_mm = float(pose6d[2] * 1000.0)
            roll, pitch, yaw = (
                float(pose6d[3]), float(pose6d[4]), float(pose6d[5]),
            )
            print(f"  → robot move   : x={x_mm:.1f}  y={y_mm:.1f}  z={z_mm:.1f}mm  "
                  f"rpy=({np.degrees(roll):.1f}, {np.degrees(pitch):.1f}, "
                  f"{np.degrees(yaw):.1f})°")
            self.robot.enable_motion()
            code = self.robot.arm.set_position(
                x=x_mm, y=y_mm, z=z_mm,
                roll=roll, pitch=pitch, yaw=yaw,
                is_radian=True,
                speed=float(s.recovery_robot_speed_deg_s),
                wait=True,
            )
            if code != 0:
                print(f"  ✘ robot.set_position 실패 (code={code}) — robot 미이동")
                return True, None
        except Exception as e:
            print(f"  ✘ robot 이동 예외: {e}")
            return False, None

        # ── 5. 이동 후 T_BC 재캡처 ─────────────────────────────────────
        try:
            T_EB_after = self.robot.get_ee_pose_mat()
            T_BE_after = np.linalg.inv(T_EB_after)
            T_BC_recovery = self._T_EC @ T_BE_after
            t_after_m = T_BC_recovery[:3, 3]
            print(f"  T_BC re-captured: trans = ({t_after_m[0]*1000:+.1f}, "
                  f"{t_after_m[1]*1000:+.1f}, {t_after_m[2]*1000:+.1f}) mm")
            return True, T_BC_recovery
        except Exception as e:
            print(f"  ⚠ post-move T_BC 캡처 실패: {e} — hint override 없이 진행")
            return True, None

    # ── 내부 ───────────────────────────────────────────────────────────

    @staticmethod
    def _merge_into_master(
        sub_model: artec_base.ModelHandle,
        master_model: artec_base.ModelHandle,
        T_pre: Optional[np.ndarray] = None,
    ) -> int:
        """
        Sub-model 의 (비어있지 않은) IScan 을 master 에 추가. 반환: 추가된 개수.
        T_pre 가 주어지면 IScan 의 모든 frame transformation 에 좌측 곱 — 객체
        body frame (= Pass 1 의 좌표계) 으로 가져오는 pre-rotation hint.
        SDK IModel::add 는 내부에서 addRef 하므로 sub_model 가 GC 돼도 안전.
        """
        n_added = 0
        for i in range(sub_model.scan_count()):
            scan = sub_model.get_scan(i)
            if scan.frame_count() == 0:
                continue
            if T_pre is not None:
                ArtecMultiPassScanSession._apply_pre_rotation(scan, T_pre)
            master_model.add_scan(scan)
            n_added += 1
        return n_added

    @staticmethod
    def _apply_pre_rotation(scan, T_pre: np.ndarray) -> None:
        """Left-multiply T_pre to all frame transformations in scan."""
        n = scan.frame_count()
        for i in range(n):
            T_old = scan.get_frame_transformation(i)
            T_new = T_pre @ T_old
            scan.set_frame_transformation(i, T_new)

    @staticmethod
    def _compute_model_centroid(model) -> np.ndarray:
        """
        Model 안 모든 IScan 의 vertex centroid (scan world 좌표, mm).
        frame_transformation 적용 후 좌표.
        성능: scan 당 frame 50개로 subsample.
        """
        sum_xyz = np.zeros(3, dtype=np.float64)
        count = 0
        for s_i in range(model.scan_count()):
            scan = model.get_scan(s_i)
            n_frames = scan.frame_count()
            if n_frames == 0:
                continue
            n_sample = min(50, n_frames)
            indices = np.linspace(0, n_frames - 1, n_sample, dtype=int)
            for i in indices:
                frame = scan.get_frame(int(i))
                v = frame.vertices()
                if v.shape[0] == 0:
                    continue
                T = scan.get_frame_transformation(int(i))
                v_world = v @ T[:3, :3].T + T[:3, 3]
                sum_xyz += v_world.sum(axis=0)
                count += v_world.shape[0]
        if count == 0:
            return np.zeros(3, dtype=np.float64)
        return sum_xyz / count

    def _get_pre_rotation_for_pose(self, pose_idx: int) -> Optional[np.ndarray]:
        """
        pose_physical_rotations[pose_idx] (사용자가 객체에 가한 물리 회전) 의
        inverse 를 반환. 그 inverse 가 IScan frame transformation 에 좌측 곱돼
        Pass N 의 scan 좌표계 → 객체 body frame (Pass 1 좌표계) 로 가져옴.
        """
        rots = self.s.pose_physical_rotations
        if not rots or pose_idx >= len(rots):
            return None
        R_phys = rots[pose_idx]
        if R_phys is None:
            return None
        # Identity 면 굳이 곱 안 해도 OK (성능 + 부동소수 안전).
        if np.allclose(R_phys, np.eye(4), atol=1e-9):
            return None
        try:
            return np.linalg.inv(R_phys)
        except np.linalg.LinAlgError:
            print(f"  [multipass] ⚠ pose_physical_rotations[{pose_idx}] 의 "
                  f"inverse 계산 실패 — hint 무시")
            return None

    def _print_pose_hint_for_user(self, pose_idx: int) -> None:
        """현재 pose 에 대해 사용자가 어떤 자세로 두어야 하는지 출력."""
        rots = self.s.pose_physical_rotations
        if not rots or pose_idx >= len(rots) or rots[pose_idx] is None:
            print(f"  [pose {pose_idx}] hint 미정 — canonical 자세로 두기")
            return
        R3 = rots[pose_idx][:3, :3]
        # Rotation 행렬에서 axis-angle 추출 (Rodrigues' formula 역계산)
        cos_t = (np.trace(R3) - 1.0) / 2.0
        cos_t = float(np.clip(cos_t, -1.0, 1.0))
        angle_deg = float(np.degrees(np.arccos(cos_t)))

        if abs(angle_deg) < 0.1:
            print(f"  [pose {pose_idx}] canonical 자세 (회전 0°)")
            return

        # 축 추출: 180° 근처는 sin_t≈0 singularity. (R+I)/2 의 대각 원소로 axis 복원.
        if abs(180.0 - angle_deg) < 1.0:
            M = (R3 + np.eye(3)) / 2.0
            diag = np.diag(M)
            i = int(np.argmax(diag))
            ax = np.zeros(3)
            ax[i] = float(np.sqrt(max(diag[i], 0.0)))
            for j in range(3):
                if j != i:
                    ax[j] = M[i, j] / ax[i] if ax[i] > 1e-9 else 0.0
            n = float(np.linalg.norm(ax))
            if n > 1e-9:
                ax = ax / n
        else:
            sin_t = np.sin(np.radians(angle_deg))
            ax = np.array([
                R3[2, 1] - R3[1, 2],
                R3[0, 2] - R3[2, 0],
                R3[1, 0] - R3[0, 1],
            ]) / (2.0 * sin_t)

        print(f"  [pose {pose_idx}] 객체 회전 ≈ {angle_deg:.0f}° "
              f"around axis ({ax[0]:+.2f}, {ax[1]:+.2f}, {ax[2]:+.2f})")

    @staticmethod
    def _print_pass_banner(idx: int, total: int, pose_idx: int) -> None:
        bar = "═" * 46
        print(f"\n╔{bar}╗")
        print(f"║   Multi-pass scan — Pass {idx} / max {total:<3} (pose {pose_idx})    ║")
        print(f"╚{bar}╝")

    @staticmethod
    def _wait_user_quit() -> bool:
        """Return True if user typed 'q' (case-insensitive), else False."""
        try:
            inp = input("  >> ").strip().lower()
        except EOFError:
            return False
        return inp == "q"
