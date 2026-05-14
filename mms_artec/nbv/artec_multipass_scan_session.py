# mms_artec/nbv/artec_multipass_scan_session.py
#
# Multi-pass orchestrator. ArtecStreamingScanSession 한 번 = 한 IScan = 한 회전.
# 이 클래스는 그것을 N 번 반복하며 모든 IScan 을 master IModel 에 누적한다.
#
# 동기 (docs/7_artec_phase1.md §0, §15 참조):
#  - SDK 의 ScanningState_ContinueRecord 는 미지원이라 한 IScan 안에서 pause/resume 불가.
#  - 대신 multi-IScan 으로 분할 → 마지막에 GlobalRegistration 이 정합.
#  - 이는 Artec Studio 의 표준 multi-scan 워크플로우와 동일.
#
# 두 가지 use case:
#  - **Phase 1 재진행 (tracking lost recovery)**:
#      한 pass 도중 tracking lost → session 종료 → 사용자 위치 복원 → 다음 pass.
#      두 IScan 은 같은 좌표계(아이템이 동일 위치) 이므로 GlobalReg 가 자연 정합.
#  - **Phase 2 (바닥면 스캐닝)**:
#      한 pass 정상 완료 → 사용자 아이템 뒤집음 → 다음 pass.
#      두 IScan 은 다른 자세이지만 옆면 overlap 으로 GlobalReg 가 정합.
#
# Notation: T_AB : A → B  (CLAUDE.md 준수).

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional, TYPE_CHECKING

import numpy as np

from mms_artec.sensor import artec_base
from mms_artec.nbv.artec_streaming_scan_session import (
    ArtecStreamingScanSession,
    ArtecStreamingScanSessionSettings,
    ArtecStreamingScanResult,
)


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
    Pose 별 사용자 물리 회전 (단일 축 기준).

    Example
    -------
    >>> make_axis_physical_rotations("y", [0, 90, 180])
    # [Ry(0)=I, Ry(90°), Ry(180°)] — 0/90/180 도 회전을 객체에 가하는 시퀀스.
    # Right-hand convention. y 축 기준 +90° 는 -x 방향이 +z 로 가는 회전
    # ('face5 가 face1 자리로') — docs/8 §1 참조.
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

    # ── Entry ──────────────────────────────────────────────────────────

    def run(self) -> ArtecMultiPassScanResult:
        s = self.s
        master_model = artec_base.create_model()
        pass_results: List[ArtecStreamingScanResult] = []
        user_quit = False
        aborted_reason = ""
        n_pass = 0
        pose_idx = 0   # 현재 pose 의 hint 인덱스. 정상 완료 시만 advance.

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

            # ── 단일 회전 ─────────────────────────────────────────────
            single = ArtecStreamingScanSession(
                self.mms, self.robot, self.turntable, s.streaming_settings,
            )
            sub_result = single.run()
            pass_results.append(sub_result)
            n_pass += 1

            # Sub-model 의 IScan 들 → master_model (pose hint 적용 후)
            T_pre = self._get_pre_rotation_for_pose(pose_idx)
            n_added = self._merge_into_master(
                sub_result.model, master_model, T_pre,
            )
            print(f"\n  [pass {n_pass} / pose {pose_idx}] {n_added} scan(s) → master "
                  f"(total scans={master_model.scan_count()})")

            # ── 분기: tracking lost 였나 정상 완료였나 ────────────────
            if sub_result.tracking_lost:
                # Drive 알람은 retry 해도 못 풀음 — 즉시 종료
                if "drive alarm" in (sub_result.loss_reason or "").lower():
                    aborted_reason = (
                        f"turntable drive alarm — EziSERVO 전원 OFF/ON 후 재실행"
                    )
                    print(f"\n  ✘ {aborted_reason}")
                    break
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
                # 다음 iteration 으로 진행 — pose_idx 유지 (재시도)
            else:
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

        return ArtecMultiPassScanResult(
            model=master_model,
            n_passes=n_pass,
            n_total_frames=n_total_frames,
            pass_results=pass_results,
            user_quit=user_quit,
            aborted_reason=aborted_reason,
        )

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
        R = rots[pose_idx]
        # Rotation 행렬에서 axis-angle 추출 (Rodrigues' formula 역계산)
        cos_t = (np.trace(R[:3, :3]) - 1.0) / 2.0
        cos_t = float(np.clip(cos_t, -1.0, 1.0))
        angle_deg = float(np.degrees(np.arccos(cos_t)))
        if abs(angle_deg) < 0.1:
            print(f"  [pose {pose_idx}] canonical 자세 (회전 0°)")
            return
        # 회전축 추출
        sin_t = np.sin(np.radians(angle_deg))
        if abs(sin_t) > 1e-6:
            ax = np.array([
                R[2, 1] - R[1, 2],
                R[0, 2] - R[2, 0],
                R[1, 0] - R[0, 1],
            ]) / (2.0 * sin_t)
        else:
            ax = np.array([0.0, 0.0, 0.0])
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
