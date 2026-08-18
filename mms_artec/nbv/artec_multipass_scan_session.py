# mms_artec/nbv/artec_multipass_scan_session.py
#
# Artec(real) 스캔 backend — 공용 Phase 컨트롤러의 ScanBackend 프리미티브 구현.
# ArtecStreamingScanSession 한 번 = 한 IScan = 한 회전 (= _do_one_rotation).
# Phase 1→2→3 순서/게이팅/NBV 수렴 루프는 **공용 컨트롤러**
# (utils/nbv/scan_phase_controller.run_scan_phases) 소유 — sim IsaacScanSession
# 과 동일. 이 파일은 real Artec 캡처 하드웨어 프리미티브만 제공한다.
# (2026-07-01: 단일 pass 루프 → phase 함수 분리 → 공용 컨트롤러 위임.
#  prompt 는 confirm_start(Phase 1) / next_flip(Phase 3)에만 — Phase 1·2 무인.)
#
# Phase 정의 (★ 재정의: docs/3_phase2.md §8. 이전 docs/7 의 "Phase 2(flip 바닥면)" 는
#               본 설계에서 **Phase 3** 로 분리되고, Phase 2 는 NBV 보강으로 재정의됨):
#  - Phase 1: 데이터 잘 잡히는 포즈로 로봇 고정 → turntable 360° → 윗면 + 옆면 4 (= 5면).
#  - Phase 2: 5면 중 부족 영역을 **로봇이 최소이동 NBV 로 보강**(공용 _run_phase2_nbv +
#             utils/nbv/phase2_nbv + robot_collision). 로봇이 움직인다.
#  - Phase 3: 물체를 **외부에서 flip**(Ry+90/180°) → 턴테이블만 회전 → **바닥면(윗면)**
#             + overlap 옆면 캡처 후 centroid-pivot pre-rotation hint 로 병합
#             (docs/8_artec_phase2_pose_disambiguation.md §5.3). = 이전의 "Phase 2(flip)".
#  ★ phase_mode=N 으로 **순차 누적** 실행: 1=Phase1, 2=Phase1→2, 3=Phase1→2→3.
#    (Phase 2→3 전환 시 NBV 로 움직인 robot 을 go_home 으로 복귀시킨 뒤 flip.)
#
# 동기:
#  - SDK 의 ScanningState_ContinueRecord 는 미지원이라 한 IScan 안에서 pause/resume 불가.
#  - 대신 multi-IScan 으로 분할 → master IModel 에 누적, hint 로 정합.
#  - 이는 Artec Studio 의 표준 multi-scan 워크플로우와 동일.
#
# 두 가지 흐름이 같은 메커니즘으로 처리됨:
#  - Tracking lost recovery: 같은 pose 로 retry, hint 동일.
#  - Phase 3(flip) 의 자세 변경: 다음 pose advance, 새 hint.
#
# Notation: T_AB : A → B  (CLAUDE.md 준수).

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import List, Optional, TYPE_CHECKING

import numpy as np

from mms_artec.sensor import artec_algorithm, artec_base
from mms_artec.nbv.artec_streaming_scan_session import (
    ArtecStreamingScanSession,
    ArtecStreamingScanSessionSettings,
    ArtecStreamingScanResult,
)
from mms_artec.nbv.recovery_pose_selector import (
    look_at_axes,
    calibrate_camera_axes_from_preview,
    SPIDER_HALF_FOV_H,
    SPIDER_HALF_FOV_V,
    SPIDER_NEAR_MM,
    SPIDER_FAR_MM,
)
from utils.transforms import pose_mat_to_6d
from utils.nbv.scan_phase_controller import run_scan_phases, AT_CURRENT


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
    # 2026-05-20 재설계: selector(LocalJitter/CentroidVector) 폐기.
    # tracking lost 발생 시:
    #   1. last-good 각도 + safe_back_margin_deg 까지 turntable 역회전
    #   2. _adaptive_prescan_position(recovery=True) 호출:
    #        - fresh probe → envelope + (r,z) profile
    #        - elevation search (recovery 축소 후보) → best 자세로 robot 이동
    #   3. 다음 streaming pass 진행 (pose_idx 유지)
    # 같은 pose 안에서 연속 max_recovery_retries 회까지 자동 시도, 초과하면
    # user prompt 로 fallback.
    auto_recovery_enabled: bool = True
    max_recovery_retries: int = 3
    safe_back_margin_deg: float = 10.0
    # Recovery 시 robot 이동 속도 (deg/s) — collision 위험 최소화 위해 보수적.
    recovery_robot_speed_deg_s: float = 10.0
    # Recovery turntable 역회전 속도 (rad/s).
    recovery_turntable_vel_rad_s: float = float(np.radians(30.0))
    # Recovery 시 elevation search 축소 — 시간 단축 (~1분 → ~30초).
    # 첫 시작은 elevation 호출 안 함(home 그대로). recovery 일 때만 이 후보로.
    # docs/artec_scanning_pipeline.md §6.3.
    recovery_elevation_offsets_deg: List[float] = field(
        default_factory=lambda: [-5.0, 0.0, 5.0])
    recovery_elevation_fine_search_enabled: bool = False

    # ── 라이브 뷰어 ───────────────────────────────────────────────────
    # True 면 모든 pass(Phase1 + Phase2) 동안 누적 컬러 포인트클라우드를
    # 실시간 표시. open3d 필요. 창을 닫아도 스캔은 계속됨. 절대 스캔
    # 성능/안정성에 영향 주지 않도록 모든 viewer 경로가 예외 격리됨.
    enable_live_viewer: bool = False

    # ── 물체-적응 자세 재선정 (recovery 전용, 2026-05-20 rule) ──────────
    # 첫 시작은 robot=home 그대로. tracking lost 시 recovery 흐름이
    # _adaptive_prescan_position(recovery=True) 를 호출 → fresh probe +
    # elevation search → 새 robot 자세. 상세: docs/artec_scanning_pipeline.md §6.
    # 아래 키들은 그 알고리즘의 공통 파라미터.
    adaptive_target_standoff_mm: float = 225.0   # 최적대역 중앙 목표 거리
    adaptive_min_preview_verts: int = 1500       # preview 유효 최소 정점
    adaptive_robot_speed_deg_s: float = 15.0

    # ── 고도각(elevation) 탐색 공통 파라미터 ───────────────────────────
    # recovery 호출 시의 elevation search 범위·기본 offsets. 재조준은
    # hardcoded +Z 가정의 look_at 이 아니라 probe preview 로 경험적 캘리브된
    # 광축(look_at_axes)을 써서 mis-aim bug 회피.
    # 상세: docs/artec_scanning_pipeline.md §3.0 / docs/artec_phase1_view_score.md.
    elevation_range_deg: tuple = (-10.0, 10.0)        # 탐색범위 clamp
    elevation_fine_step_deg: float = 2.5              # fine 활성 시 ±
    # v1.5 스코어: 분리된 '물체점' 중 카메라 FOV 안 + Gaussian 거리가중 합.
    # band 끝(200/250mm)은 약 e⁻¹ 가중, d* (225mm) 가 최대.
    elevation_optimal_band_mm: tuple = (200.0, 250.0)

    # ── 턴테이블 회전 차분 물체 probe (docs §3.0 선행1) ─────────────────
    # 단일뷰 RANSAC isolation 폐기 (Spider 협FOV → 부분 패치라 원리적
    # 불가, memory project_spider_partial_view_no_scene_segmentation).
    # 대신: robot 고정, 턴테이블을 step 회전하며 preview 캡처 →
    # voxel frame-support 로 static(회전대칭 디스크·정적 배경) 제거 →
    # moving = 물체. 명령각으로 회전축·envelope(수직 실린더) 자동 산출.
    # scan 데이터 무영향 (조준 결정용 preview 전용).
    # recovery 흐름에서 _adaptive_prescan_position 호출 시 매번 fresh probe.
    probe_total_deg: float = 150.0            # probe 총 회전각
    probe_steps: int = 6                      # 분할 수(프레임 K=steps+1)
    probe_turntable_vel_rad_s: float = float(np.radians(20.0))
    probe_settle_s: float = 0.4               # 정지 후 캡처 전 안정화
    probe_vox_mm: float = 4.0                 # static/moving voxel 크기
    probe_static_support_frac: float = 0.7    # voxel 점유 step ≥ frac·K → static
    probe_cluster_eps_mm: float = 8.0         # moving DBSCAN 반경
    probe_cluster_min_pts: int = 20           # moving DBSCAN 최소점
    probe_min_moving_pts: int = 300           # 미만이면 probe 실패→fallback
    probe_axis_resid_max_mm: float = 25.0     # centroid-circle 잔차 한계
    env_r_margin_mm: float = 8.0              # envelope 반경 여유
    env_z_margin_mm: float = 6.0              # envelope z 여유
    # ── 물체/턴테이블 분리 (2026-05-20, docs §3.0 step 6) ───────────────
    # cylinder envelope 만으로는 z_bot≈z_table 부근 디스크 표면이 "물체점"
    # 으로 새어 잘못된 best 가 뽑히는 회귀가 있어 두 가지를 추가:
    #  (i) table_clear_mm: z ≤ z_table + table_clear_mm 점은 무조건 제외.
    #      평평한 물체 바닥 일부 잘림은 Phase 3(바닥면 flip)가 별도 캡처해 보완.
    #  (ii) (r,z) 축대칭 profile: moving voxel 의 (반경,높이) 2D 점유 맵을
    #      만들어 cylinder lookup 대신 실제 단면을 본다. 축대칭화 되어 있어
    #      probe 150° 만 돌아도 모든 θ candidate 에 멤버십 성립.
    table_clear_mm: float = 8.0
    profile_bin_mm: float = 0.0               # 0 = probe_vox_mm 재사용
    profile_r_margin_mm: float = 6.0          # (r,z) 점유 반경 dilation
    profile_z_margin_mm: float = 6.0          # (r,z) 점유 높이 dilation
    # 디버그: probe step별 static/moving + 최종 object + elevation 후보별
    # (통과 녹/턴테이블floor 빨강/profile 밖 회) 색상 PLY 덤프.
    # 기본 OFF — scan/성능 무영향. CloudCompare 로 검증. docs §3.0.
    probe_debug_dump: bool = False
    iso_debug_dir: str = "output/iso_debug"

    # 병합 비교용: False 면 _merge_into_master 가 IScan frame_transformations 에
    # T_pre 를 set 하지 않고 result.recorded_hints 에 (scan_index, T_pre) list 로만
    # 기록. 후처리 단계에서 hint 적용 여부 / GlobalReg 타입을 swap 해가며 같은
    # raw scan 데이터로 여러 variant 비교 가능 (scripts/artec/merge_compare.py).
    # 기본 True = 기존 동작 (hint 즉시 적용).
    apply_hints_to_frame_transformations: bool = True

    # ── Hint ICP refine (2026-05-20, face-merging 회피용) ───────────────
    # True 면 centroid-pivot T_pre 를 *init* 으로 Open3D colored ICP 를 돌려
    # **측정된** T 를 얻고 그걸 IScan frame 에 박음. 사용자 손회전의 ±10° 오차를
    # 흡수하면서 init 이 ambiguity 를 깨는 역할. 캔처럼 앞-뒤 유사한 객체에서
    # SDK GlobalReg 가 두 면을 합치는 face-merging 문제 회피.
    # apply_hints_to_frame_transformations=True 일 때만 의미. False (record-only)
    # 모드에선 recorded_hints 에는 raw init T_pre 기록 + 후처리에서 별도 ICP 가능.
    hint_icp_refine_mode: bool = False
    icp_voxel_mm: float = 4.0           # downsample voxel (속도 ↔ 정확)
    icp_max_iter: int = 60
    icp_color_weight: float = 0.5       # colored ICP 의 색상 가중 (Open3D 0.6)
    icp_corr_dist_mm: float = 30.0      # 대응 거리 한계 (init 이 좋으면 작게)

    # ── Phase 2 — 부족면 NBV 보강 (docs/3_phase2.md) ────────────────────
    # phase_mode: Phase 1 부터 **순차 누적**으로 어디까지 실행할지 (docs/3_phase2.md §8).
    # phase_mode=N → Phase 1..N 을 순서대로:
    #   1 = Phase 1 (5면 streaming)
    #   2 = Phase 1 → **Phase 2** (부족면 NBV 보강, 로봇 이동, _phase2_nbv_loop)
    #   3 = Phase 1 → Phase 2 → **Phase 3** (바닥면 180° flip, 사용자 손회전 + centroid hint)
    # 기본 3 = 전 파이프라인. ★ 첫 real 테스트는 1 부터 단계적으로 올릴 것(2·3 미검증).
    phase_mode: int = 3
    nbv_distance_mm: float = 225.0          # NBV 카메라-표면 거리 (Artec 최적대역)
    nbv_min_seg_vertices: int = 8           # frontier 세그먼트 최소 정점
    nbv_min_seg_length_mm: float = 6.0      # frontier 최소 길이
    nbv_max_seg_length_mm: float = 60.0     # frontier 재분할 한계
    nbv_poisson_depth: int = 8              # master pcd→mesh Poisson depth
    nbv_density_quantile: float = 0.04      # 저밀도 vertex trim 분위수
    nbv_master_voxel_mm: float = 2.0        # master→pcd voxel
    nbv_K_max: int = 12                     # NBV 최대 반복
    nbv_boundary_stop_mm: float = 12.0      # 수렴: 경계 총길이 < 이 값
    nbv_coverage_tau: float = 0.92          # 수렴: 각도 커버리지 ≥ τ
    nbv_coverage_dirs: int = 64
    nbv_coverage_parallel_deg: float = 40.0
    nbv_robot_speed_deg_s: float = 12.0     # NBV 로봇 이동 속도 (보수적)
    nbv_sweep_deg: float = 15.0             # 캡처 시 턴테이블 ±스윕 (overlap)
    nbv_theta_assist: bool = False          # True=턴테이블 회전 보조(θ planner). 기본 robot-only.
    nbv_theta_n_samples: int = 72           # θ assist 그리드
    nbv_cost_delta: float = 0.3             # 큰 구멍 우선 가중
    nbv_swept_steps: int = 12               # 궤적(q_cur→q_des) 충돌검사 보간 스텝 수
    nbv_el_floor_deg: float = 30.0          # NBV 관측 elevation 하한(=Phase1 측면각). 그 이상에서 보강
    # ── 충돌 world (turntable calib 기반, robot_collision) ──────────────
    nbv_collision_enabled: bool = True      # False=충돌검사 skip(자세만 IK)
    nbv_turntable_radius_mm: float = 150.0  # disc(+프레임) 반경
    nbv_turntable_body_height_mm: float = 200.0  # 표면 아래 몸체 높이
    nbv_collision_margin_mm: float = 10.0   # 보수적 여유
    # 충돌 금지 원기둥 (turntable 위 금지구역; robot_collision.add_cylinder, real/sim 공용).
    nbv_keepout_enable: bool = False        # True 면 아래 원기둥을 충돌 world 에 추가
    nbv_keepout_radius_mm: Optional[float] = None  # None = 턴테이블 disc 반경과 동일
    nbv_keepout_height_mm: float = 120.0    # disc 표면 위 높이
    nbv_keepout_center_xy: Optional[tuple] = None  # None = 턴테이블 축 중심
    # relocalization: True 면 캡처가 기존 master 에 재고정 시도(§3.4.1). 미구현 경로는
    # camera-motion T_pre fallback(R3)로 자동 대체. 실기 검증 후 R1/R2 배선(§6.5).
    nbv_use_relocalization: bool = True

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
    # 병합 비교용 (apply_hints_to_frame_transformations=False 일 때만 채워짐).
    # 각 entry = (master 내 scan_index, T_pre 4x4). 후처리에서 선택적 적용.
    recorded_hints: List[tuple] = field(default_factory=list)
    # Hint ICP refine 진단 (hint_icp_refine_mode=True 일 때 채워짐).
    # 각 entry = (scan_index, T_init, T_measured, fitness, inlier_rmse_mm).
    icp_refine_log: List[tuple] = field(default_factory=list)


# ─────────────────────────────────────────────────────────────────────────────
# run() 내부 상태 — 회전 루프가 이어서 나르는 loop-carried 가변 변수 묶음.
# _do_one_rotation 이 mutate 하고, 공용 컨트롤러가 호출하는 프리미티브
# (confirm_start / capture_rotation / next_flip / finalize) 들이 self._st 로 공유한다.
# (회전 단위 상태를 한 곳에 모아 프리미티브 시그니처를 얇게 유지)
# ─────────────────────────────────────────────────────────────────────────────

# _do_one_rotation 반환 코드
_ROT_OK = "ok"          # 정상 완료 — 다음 phase / pose 로
_ROT_RETRY = "retry"    # tracking-lost — 같은 pose 재시도 (recovery / 수동)
_ROT_ABORT = "abort"    # 중단 (user_quit / 치명적 drive alarm)


@dataclass
class _RunState:
    """run() 회전 루프의 loop-carried 상태 (2026-07-01 phase 분리 리팩터)."""
    master_model: "artec_base.ModelHandle"
    live_viewer: object = None
    pass_results: List[ArtecStreamingScanResult] = field(default_factory=list)
    n_pass: int = 0
    pose_idx: int = 0                    # 현재 pose hint 인덱스. 정상 완료 시만 advance.
    master_center: Optional[np.ndarray] = None   # Pass 1 후 lock
    hints_applied: bool = False
    last_n_frames: int = 0               # 직전 회전 프레임 수 (완료 로그용)
    recovery_retry_count: int = 0        # 같은 pose 안에서 누적 (성공 시 0)
    n_recovery_attempts: int = 0
    n_recovery_succeeded: int = 0
    next_T_BC_pending: Optional[np.ndarray] = None
    next_skip_clearpos: bool = False
    recorded_hints: List[tuple] = field(default_factory=list)
    icp_refine_log: List[tuple] = field(default_factory=list)
    user_quit: bool = False
    aborted_reason: str = ""


# ─────────────────────────────────────────────────────────────────────────────
# Session
# ─────────────────────────────────────────────────────────────────────────────

class ArtecMultiPassScanSession:
    """
    Artec(real) 스캔 backend — 공용 Phase 컨트롤러의 ScanBackend 프리미티브 구현.

    Phase 1→2→3 순서·게이팅·NBV 수렴 루프는 공용
    utils/nbv/scan_phase_controller.run_scan_phases 가 소유(sim IsaacScanSession
    과 동일 컨트롤러). 이 클래스는 real Artec 캡처 하드웨어만 제공:
      - confirm_start / pick_phase1_pose(=AT_CURRENT) : Phase 1 시작(사람 확인 후 home 고정)
      - capture_rotation : 턴테이블 전회전 캡처. AT_CURRENT=현재포즈 streaming
        (+cleanup/hint/master 병합/tracking-lost recovery = _do_one_rotation 재시도),
        q 지정=NBV 자세로 구동 후 streaming(_capture_nbv_pose).
      - build_coverage_mesh / is_converged / plan_nbv_pose : Phase 2 NBV 보강.
      - supports_phase3=True / next_flip : Phase 3 (사람이 물체 flip → 윗면).
      - finalize : master IModel(N IScans) → ArtecMultiPassScanResult.

    prompt(사람 개입)는 confirm_start(Phase 1) + next_flip(Phase 3)에만. Phase 1·2
    회전은 무인. 반환된 master IModel 은 이후 ArtecMMS.artec_process 가
    GlobalRegistration 으로 정합.
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

    # ── 물체-적응 사전 포지셔닝 (Phase 1, run() 시작 1회) ──────────────

    def _capture_preview_verts(self, sensor, min_verts: int):
        """PREVIEW 최대 3회, 정점 최다 프레임 채택. C 프레임 mm (N,3) or None."""
        v_mm = None
        for _ in range(3):
            try:
                fmh = sensor.capture_frame()
                vv = fmh.vertices() if fmh is not None else None
            except Exception:
                vv = None
            if vv is not None and vv.shape[0] > 0:
                if v_mm is None or vv.shape[0] > v_mm.shape[0]:
                    v_mm = vv
            if v_mm is not None and v_mm.shape[0] >= min_verts:
                break
        if v_mm is None or v_mm.shape[0] < min_verts:
            return None
        return v_mm.astype(np.float64)

    @staticmethod
    def _est_from_points(xo: np.ndarray) -> dict:
        """물체점 (B,m) → center/h/r robust 통계."""
        center_xy = np.median(xo[:, :2], axis=0)
        zlo, zhi = np.percentile(xo[:, 2], 5), np.percentile(xo[:, 2], 95)
        return dict(
            cx=float(center_xy[0]), cy=float(center_xy[1]),
            z_lo=float(zlo), z_hi=float(zhi),
            z_mid=float(0.5 * (zlo + zhi)),
            h_obj=float(zhi - zlo),
            r_obj=float(np.percentile(
                np.linalg.norm(xo[:, :2] - center_xy, axis=1), 95)),
        )

    def _robust_center_from_preview(self, sensor, T_CB):
        """fallback 전용 — preview 한 장 전체 robust 통계 (배경 편향 감수)."""
        vC = self._capture_preview_verts(
            sensor, self.s.adaptive_min_preview_verts)
        if vC is None:
            return None
        xB = ((vC / 1000.0) @ np.asarray(T_CB[:3, :3], float).T
              + np.asarray(T_CB[:3, 3], float))
        return self._est_from_points(xB)

    @staticmethod
    def _fit_circle_center(P):
        """프레임별 물체 수평 centroid 가 그리는 원의 중심 = 회전축(c_x,c_y).
        반환 (center(2,), resid_m). 대칭 물체(centroid≈축)면 mean 으로 안정화."""
        P = np.asarray(P, float)
        if P.shape[0] < 3:
            c = P.mean(axis=0) if P.shape[0] else np.zeros(2)
            r = (float(np.std(np.linalg.norm(P - c, axis=1)))
                 if P.shape[0] else 0.0)
            return c, r
        spread = float(np.mean(np.std(P, axis=0)))
        if spread < 0.003:                  # centroid 거의 정지 → 축≈mean
            c = P.mean(axis=0)
            return c, float(np.std(np.linalg.norm(P - c, axis=1)))
        x, y = P[:, 0], P[:, 1]
        A = np.c_[2 * x, 2 * y, np.ones(len(P))]
        b = x ** 2 + y ** 2
        try:
            sol, *_ = np.linalg.lstsq(A, b, rcond=None)
            c = np.array([sol[0], sol[1]])
            rr = np.sqrt(max(sol[2] + c @ c, 0.0))
            if not np.isfinite(c).all():
                raise ValueError("nonfinite center")
            resid = float(np.sqrt(np.mean(
                (np.linalg.norm(P - c, axis=1) - rr) ** 2)))
            return c, resid
        except Exception:
            c = P.mean(axis=0)
            return c, float(np.std(np.linalg.norm(P - c, axis=1)))

    def _in_object_envelope(self, xB, env) -> np.ndarray:
        """envelope(수직 실린더) 멤버십 bool mask. cylinder pre-clip 용 —
        candidate scoring 은 `_in_object_profile` (turntable floor +
        (r,z) profile) 를 써야 한다."""
        s = self.s
        ax = np.asarray(env["axis_xy"], float)
        rad = np.linalg.norm(xB[:, :2] - ax, axis=1)
        return (
            (rad <= env["r_obj"] + s.env_r_margin_mm / 1000.0)
            & (xB[:, 2] >= env["z_lo"] - s.env_z_margin_mm / 1000.0)
            & (xB[:, 2] <= env["z_hi"] + s.env_z_margin_mm / 1000.0)
        )

    def _in_object_profile(self, xB, env) -> tuple:
        """
        물체 멤버십 + 턴테이블 평면 hard floor. docs §3.0 step 6.
          (i) cylinder pre-clip — 빠르고 보수적.
          (ii) `z > z_table + table_clear_mm` hard floor — 디스크 표면
               (z_bot≈z_table) 이 cylinder 안에서 "물체점"으로 새던 회귀를
               차단. 평평한 물체 바닥 일부는 비용으로 수용 (Phase 3 바닥면 flip 이 따로).
          (iii) probe (r,z) 점유 맵 lookup (`env['rz_occ']`) — moving voxel
               의 실제 축대칭 단면. cylinder bounding 보다 정확
               (overhang/패임 그대로). 맵 없으면 cylinder + floor 만.

        Returns (mask_obj, mask_excluded_table) — 두 mask 가 disjoint.
          mask_obj: 후보 스코어 산입 대상.
          mask_excluded_table: cylinder 안인데 floor 에 잘린 점 (진단용,
            "그 자세는 턴테이블을 얼마나 봤나" 지표).
        """
        s = self.s
        ax = np.asarray(env["axis_xy"], float)
        z = xB[:, 2]
        rad = np.linalg.norm(xB[:, :2] - ax, axis=1)
        in_cyl = (
            (rad <= float(env["r_obj"]) + s.env_r_margin_mm / 1000.0)
            & (z >= float(env["z_lo"]) - s.env_z_margin_mm / 1000.0)
            & (z <= float(env["z_hi"]) + s.env_z_margin_mm / 1000.0)
        )
        z_floor = float(env["z_table"]) + s.table_clear_mm / 1000.0
        above = z > z_floor
        mask_excl_table = in_cyl & (~above)
        rz_occ = env.get("rz_occ")
        if rz_occ is None:
            return in_cyl & above, mask_excl_table
        bin_m = float(env["bin_mm"]) / 1000.0
        r0 = float(env["r_origin_m"])
        z0 = float(env["z_origin_m"])
        nr, nz = rz_occ.shape
        ri = np.floor((rad - r0) / bin_m).astype(np.int64)
        zi = np.floor((z - z0) / bin_m).astype(np.int64)
        in_b = (ri >= 0) & (ri < nr) & (zi >= 0) & (zi < nz)
        cand = in_cyl & above & in_b
        mask_obj = np.zeros(len(xB), dtype=bool)
        if cand.any():
            mask_obj[cand] = rz_occ[ri[cand], zi[cand]]
        return mask_obj, mask_excl_table

    def _dump_probe_ply(self, valid, static, vox, obj_all, stage) -> None:
        """probe 디버그 PLY — static(회)·moving(파)·object(초). 예외 격리."""
        try:
            import os
            import open3d as o3d
            d = getattr(self.s, "iso_debug_dir", "output/iso_debug")
            if not os.path.isabs(d):                 # cwd 가 utils/turntable
                try:                                  # 로 바뀌므로 절대경로
                    from utils import PROJECT_ROOT
                    d = os.path.join(str(PROJECT_ROOT), d)
                except Exception:
                    d = os.path.abspath(d)
            os.makedirs(d, exist_ok=True)
            pts, cols = [], []
            for (_k, xB, _v) in valid:
                keys = np.floor(xB / vox).astype(np.int64)
                isst = np.fromiter(
                    (tuple(r) in static for r in keys),
                    dtype=bool, count=len(keys))
                c = np.where(isst[:, None],
                             np.array([0.5, 0.5, 0.5]),
                             np.array([0.2, 0.4, 0.9]))
                pts.append(xB)
                cols.append(c)
            if obj_all is not None and len(obj_all):
                pts.append(np.asarray(obj_all, float))
                cols.append(np.tile([0.1, 0.95, 0.2],
                                    (len(obj_all), 1)))
            P = np.vstack(pts)
            C = np.vstack(cols)
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(P)
            pcd.colors = o3d.utility.Vector3dVector(C)
            self._probe_n = getattr(self, "_probe_n", 0) + 1
            fn = os.path.join(
                d, f"probe_{self._probe_n:02d}_{stage}.ply")
            okw = bool(o3d.io.write_point_cloud(fn, pcd))
            sz = os.path.getsize(fn) if os.path.isfile(fn) else 0
            print(f"  [probe.dbg] {'OK' if okw and sz > 0 else '✘ FAIL'} "
                  f"{fn} ({sz}B) static={len(static)}vox "
                  f"obj={0 if obj_all is None else len(obj_all)}")
        except Exception as e:
            print(f"  [probe.dbg] dump 실패 ({type(e).__name__}: {e})")

    def _dump_candidate_ply(
        self, verts_C_mm, T_CB, env, seq, tag,
    ) -> None:
        """elevation 후보 preview 를 통과(녹)/turntable floor(빨강)/profile
        밖(회) 으로 색칠해 PLY 덤프. probe_debug_dump=True 일 때만 호출.
        scan/성능 무영향 — 예외는 모두 격리. docs §3.0."""
        try:
            import os
            import open3d as o3d
            xB = ((np.asarray(verts_C_mm, float) / 1000.0)
                  @ np.asarray(T_CB[:3, :3], float).T
                  + np.asarray(T_CB[:3, 3], float))
            mask_obj, mask_excl = self._in_object_profile(xB, env)
            cols = np.tile([0.6, 0.6, 0.6], (len(xB), 1))
            cols[mask_excl] = [0.95, 0.15, 0.15]
            cols[mask_obj] = [0.15, 0.95, 0.25]
            d = getattr(self.s, "iso_debug_dir", "output/iso_debug")
            if not os.path.isabs(d):
                try:
                    from utils import PROJECT_ROOT
                    d = os.path.join(str(PROJECT_ROOT), d)
                except Exception:
                    d = os.path.abspath(d)
            os.makedirs(d, exist_ok=True)
            safe = (tag.replace("+", "p").replace("-", "m")
                    .replace(".", "d"))
            fn = os.path.join(d, f"cand_{seq:02d}_{safe}.ply")
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(xB)
            pcd.colors = o3d.utility.Vector3dVector(cols)
            okw = bool(o3d.io.write_point_cloud(fn, pcd))
            sz = os.path.getsize(fn) if os.path.isfile(fn) else 0
            print(f"  [elev.dbg] {'OK' if okw and sz > 0 else '✘ FAIL'} "
                  f"{fn} ({sz}B) obj={int(mask_obj.sum())} "
                  f"excl={int(mask_excl.sum())} tot={len(xB)}")
        except Exception as e:
            print(f"  [elev.dbg] dump 실패 ({type(e).__name__}: {e})")

    def _motion_probe_object(self, T_CB_home) -> dict:
        """
        턴테이블 회전 차분으로 물체 envelope 추정 (docs §3.0 선행1).
        robot=home 고정, 턴테이블 step 회전하며 preview 캡처 → voxel
        frame-support 로 static(회전대칭 디스크·정적 배경) 제거 → moving
        DBSCAN 최대 클러스터 = 물체 swept set → centroid-circle 로
        회전축·envelope(수직 실린더) 산출. probe 후 시작각 복귀.

        Returns {ok, center_B, axis_xy, z_lo/z_hi/z_mid, h_obj, r_obj,
                 z_table, obj_vC(C mm 캘리브용), n_moving, axis_resid_mm,
                 moving_centroid_B}. ok=False → caller fallback.
        """
        s = self.s
        out = {"ok": False}
        sensor = getattr(self.mms, "sensor", None)
        if sensor is None or not hasattr(sensor, "capture_frame"):
            print("  [probe] sensor 없음 — skip")
            return out
        try:
            import open3d as o3d
        except Exception as e:
            print(f"  [probe] open3d 없음 ({e}) — skip")
            return out

        R_CB = np.asarray(T_CB_home[:3, :3], float)
        t_CB = np.asarray(T_CB_home[:3, 3], float)
        K = int(s.probe_steps) + 1
        dstep = np.radians(float(s.probe_total_deg)
                           / max(int(s.probe_steps), 1))
        vel = float(s.probe_turntable_vel_rad_s)

        try:
            self.turntable.stop()
            time.sleep(0.1)
            self.turntable.check_drive_err()
            self.turntable.set_servo_on(True)
            time.sleep(0.1)
            th0 = self.turntable.getActualPos()
            th0 = (float(th0) if not isinstance(th0, bool)
                   and th0 is not None else 0.0)
        except Exception as e:
            print(f"  [probe] 턴테이블 준비 실패 ({e}) — skip")
            return out

        frames_B, frames_C = [], []
        for k in range(K):
            time.sleep(float(s.probe_settle_s))
            vC = self._capture_preview_verts(sensor, 300)
            if vC is None:
                print(f"  [probe] step {k}: preview 부족")
                frames_B.append(None)
                frames_C.append(None)
            else:
                frames_B.append((vC / 1000.0) @ R_CB.T + t_CB)
                frames_C.append(vC)
            if k < K - 1:
                try:
                    ok = self.turntable.move_abs(
                        th0 + (k + 1) * dstep, vel)
                    if not ok:
                        print(f"  [probe] move_abs 실패(step {k}) — 조기종료")
                        break
                    if hasattr(self.turntable, "wait_motion_done"):
                        self.turntable.wait_motion_done(timeout_s=15.0)
                    else:
                        time.sleep(abs(dstep) / max(vel, 0.1) + 0.5)
                except Exception as e:
                    print(f"  [probe] 회전 예외(step {k}): {e}")
                    break

        try:                                # 시작각 복귀
            self.turntable.stop()
            time.sleep(0.05)
            self.turntable.move_abs(th0, vel)
            if hasattr(self.turntable, "wait_motion_done"):
                self.turntable.wait_motion_done(timeout_s=20.0)
        except Exception as e:
            print(f"  [probe] ⚠ 시작각 복귀 실패 ({e}) "
                  f"— streaming reset 가 보정")

        valid = [(k, frames_B[k], frames_C[k])
                 for k in range(len(frames_B)) if frames_B[k] is not None]
        if len(valid) < 3:
            print(f"  [probe] 유효 프레임 {len(valid)} <3 — fallback")
            return out

        # voxel frame-support → static / moving
        vox = s.probe_vox_mm / 1000.0
        support = {}
        for (k, xB, _v) in valid:
            for key in {tuple(r) for r in
                        np.floor(xB / vox).astype(np.int64)}:
                support.setdefault(key, set()).add(k)
        Kv = len(valid)
        thr = int(np.ceil(s.probe_static_support_frac * Kv))
        static = {key for key, fs in support.items() if len(fs) >= thr}

        moving_chunks, moving_by_frame = [], []
        for (k, xB, vC) in valid:
            keys = np.floor(xB / vox).astype(np.int64)
            mv = np.fromiter((tuple(r) not in static for r in keys),
                             dtype=bool, count=len(keys))
            if mv.any():
                moving_chunks.append(xB[mv])
                moving_by_frame.append((k, vC, mv))
        if not moving_chunks:
            print("  [probe] moving 점 0 — fallback")
            return out
        moving_all = np.vstack(moving_chunks)
        if moving_all.shape[0] < s.probe_min_moving_pts:
            print(f"  [probe] moving {moving_all.shape[0]} < "
                  f"{s.probe_min_moving_pts} — fallback")
            if s.probe_debug_dump:
                self._dump_probe_ply(valid, static, vox, None, "lowmoving")
            return out

        pc = o3d.geometry.PointCloud()
        pc.points = o3d.utility.Vector3dVector(moving_all)
        lbl = np.asarray(pc.cluster_dbscan(
            s.probe_cluster_eps_mm / 1000.0, s.probe_cluster_min_pts))
        if lbl.size == 0 or lbl.max() < 0:
            print("  [probe] moving 클러스터 없음 — fallback")
            if s.probe_debug_dump:
                self._dump_probe_ply(valid, static, vox, None, "noclust")
            return out
        vals, cnts = np.unique(lbl[lbl >= 0], return_counts=True)
        obj_all = moving_all[lbl == vals[int(np.argmax(cnts))]]

        # 회전축 = 프레임별 물체 수평 centroid 의 원중심 (centroid-circle)
        cents = np.asarray([
            frames_B[k][mv][:, :2].mean(axis=0)
            for (k, _vC, mv) in moving_by_frame])
        axis_xy, axis_resid = self._fit_circle_center(cents)
        if axis_resid > s.probe_axis_resid_max_mm / 1000.0:
            print(f"  [probe] axis resid {axis_resid*1000:.1f}mm > "
                  f"{s.probe_axis_resid_max_mm:.0f} — fallback")
            if s.probe_debug_dump:
                self._dump_probe_ply(valid, static, vox, obj_all, "axisbad")
            return out

        rad = np.linalg.norm(obj_all[:, :2] - axis_xy, axis=1)
        r_obj = float(np.percentile(rad, 95))
        zlo = float(np.percentile(obj_all[:, 2], 5))
        zhi = float(np.percentile(obj_all[:, 2], 95))
        z_mid = 0.5 * (zlo + zhi)

        # 캘리브용: envelope 안 물체점 최다 프레임의 C verts
        best_k, best_n, best_vC = None, -1, None
        for (k, vC, mv) in moving_by_frame:
            xBk = frames_B[k][mv]
            inenv = (
                (np.linalg.norm(xBk[:, :2] - axis_xy, axis=1)
                 <= r_obj + s.env_r_margin_mm / 1000.0)
                & (xBk[:, 2] >= zlo - s.env_z_margin_mm / 1000.0)
                & (xBk[:, 2] <= zhi + s.env_z_margin_mm / 1000.0))
            if int(inenv.sum()) > best_n:
                best_n = int(inenv.sum())
                best_vC = vC[np.nonzero(mv)[0][inenv]]
                best_k = k
        if best_vC is None or best_vC.shape[0] < 50:
            print(f"  [probe] 캘리브용 C 물체점 부족 ({best_n}) — fallback")
            if s.probe_debug_dump:
                self._dump_probe_ply(valid, static, vox, obj_all, "calibfew")
            return out

        # (r,z) 축대칭 occupancy — moving voxel 만으로 빌드. cylinder 와
        # 별개의 "실제 단면" 멤버십 (docs §3.0 선행1, step 6).
        rz_pack = self._build_rz_profile(
            support, static, vox, axis_xy, r_obj, zlo, zhi)

        if s.probe_debug_dump:
            self._dump_probe_ply(valid, static, vox, obj_all, "ok")
        print(f"  [probe] frames={Kv} moving={moving_all.shape[0]} "
              f"obj={obj_all.shape[0]} axis=({axis_xy[0]:+.3f},"
              f"{axis_xy[1]:+.3f}) resid={axis_resid*1000:.1f}mm "
              f"r={r_obj*1000:.0f}mm z=[{zlo:+.3f},{zhi:+.3f}] "
              f"h={(zhi-zlo)*1000:.0f}mm calib_frame={best_k}")
        if rz_pack is not None:
            print(f"  [probe] (r,z)profile bin={rz_pack['bin_mm']:.1f}mm "
                  f"shape={rz_pack['rz_occ'].shape} "
                  f"occ_cells={int(rz_pack['rz_occ'].sum())} "
                  f"dilate=(r{s.profile_r_margin_mm:.0f},"
                  f"z{s.profile_z_margin_mm:.0f})mm")
        out.update(
            ok=True,
            center_B=np.array([axis_xy[0], axis_xy[1], z_mid]),
            axis_xy=np.asarray(axis_xy, float),
            z_lo=zlo, z_hi=zhi, z_mid=float(z_mid),
            h_obj=float(zhi - zlo), r_obj=r_obj, z_table=zlo,
            obj_vC=best_vC.astype(np.float64),
            n_moving=int(moving_all.shape[0]),
            axis_resid_mm=float(axis_resid * 1000.0),
            moving_centroid_B=moving_all.mean(axis=0),
        )
        if rz_pack is not None:
            out.update(rz_pack)
        return out

    def _build_rz_profile(
        self, support, static, vox, axis_xy, r_obj, zlo, zhi,
    ) -> Optional[dict]:
        """
        Moving voxel 들을 축 (axis_xy) 기준 (r-bin, z-bin) 2D 점유 맵으로
        reduce — 축 둘레 360° **회전대칭화** 되어있어 probe 가 일부 각만
        돌아도 모든 candidate θ 에서 멤버십 성립. docs §3.0 선행1.

        Returns dict(rz_occ(bool,(nr,nz)), bin_mm, r_origin_m, z_origin_m)
        또는 None (moving voxel 0 등 비정상).
        """
        s = self.s
        moving_keys = [k for k, fs in support.items()
                       if (k not in static) and len(fs) > 0]
        if not moving_keys:
            return None
        bin_mm = (float(s.profile_bin_mm)
                  if s.profile_bin_mm > 0 else float(s.probe_vox_mm))
        bin_m = bin_mm / 1000.0
        keys = np.asarray(moving_keys, dtype=np.int64)        # (M,3)
        centers = (keys.astype(np.float64) + 0.5) * vox       # m, B 좌표
        r = np.linalg.norm(centers[:, :2] - np.asarray(axis_xy), axis=1)
        z = centers[:, 2]
        r_origin = 0.0
        z_origin = float(zlo) - 0.05            # 50mm below z_bot 안전여유
        r_max = float(r_obj) + 0.030 + s.profile_r_margin_mm / 1000.0
        z_max = float(zhi) + 0.05 + s.profile_z_margin_mm / 1000.0
        nr = max(int(np.ceil((r_max - r_origin) / bin_m)), 1)
        nz = max(int(np.ceil((z_max - z_origin) / bin_m)), 1)
        ri = np.floor((r - r_origin) / bin_m).astype(np.int64)
        zi = np.floor((z - z_origin) / bin_m).astype(np.int64)
        in_b = (ri >= 0) & (ri < nr) & (zi >= 0) & (zi < nz)
        if not in_b.any():
            return None
        rz_occ = np.zeros((nr, nz), dtype=bool)
        rz_occ[ri[in_b], zi[in_b]] = True
        r_d = int(np.ceil(s.profile_r_margin_mm / bin_mm))
        z_d = int(np.ceil(s.profile_z_margin_mm / bin_mm))
        if r_d > 0 or z_d > 0:
            try:
                from scipy.ndimage import binary_dilation
                kernel = np.ones((2 * r_d + 1, 2 * z_d + 1), dtype=bool)
                rz_occ = binary_dilation(rz_occ, structure=kernel)
            except Exception:
                dil = rz_occ.copy()
                for dr in range(-r_d, r_d + 1):
                    for dz in range(-z_d, z_d + 1):
                        if dr == 0 and dz == 0:
                            continue
                        dil |= np.roll(
                            np.roll(rz_occ, dr, axis=0), dz, axis=1)
                rz_occ = dil
        return dict(
            rz_occ=rz_occ, bin_mm=float(bin_mm),
            r_origin_m=float(r_origin), z_origin_m=float(z_origin),
        )

    def _standoff_distance(self, h_obj: float, r_obj: float):
        """단일 band stand-off backoff → (d_m, partial). 거리만 적응."""
        s = self.s
        tan_hv = np.tan(float(SPIDER_HALF_FOV_V))
        margin = 0.010                                  # 10mm 여유
        near = SPIDER_NEAR_MM / 1000.0
        far = SPIDER_FAR_MM / 1000.0

        def _fits(d):
            return (h_obj + 2.0 * margin) <= (2.0 * d * tan_hv)

        d = max(s.adaptive_target_standoff_mm / 1000.0, near + r_obj)
        if not _fits(d):
            d_need = (0.5 * h_obj + margin) / max(tan_hv, 1e-6)
            d = min(max(d, d_need), far)
        d = float(min(max(d, near + r_obj), far))
        return d, (not _fits(d))

    def _move_robot_to_T_CB(self, T_CB_target: np.ndarray,
                            speed_deg_s: float) -> int:
        """T_CB → T_EB → xArm set_position. 반환 = xArm code (0=OK).
        실패 시 fault 를 clear (이후 robot 동작이 막히지 않게)."""
        T_EB_target = T_CB_target @ self._T_EC
        p = pose_mat_to_6d(T_EB_target)
        self.robot.enable_motion()
        code = self.robot.arm.set_position(
            x=float(p[0] * 1000.0), y=float(p[1] * 1000.0),
            z=float(p[2] * 1000.0),
            roll=float(p[3]), pitch=float(p[4]), yaw=float(p[5]),
            is_radian=True, speed=float(speed_deg_s), wait=True,
        )
        if code != 0:
            try:
                self.robot.enable_motion()
            except Exception:
                pass
        return int(code)

    def _recapture_T_BC(self, tag: str) -> None:
        """이동 후 T_BC/T_CB 재캡처 (recovery hint 일관성)."""
        T_EB_after = self.robot.get_ee_pose_mat()
        self._T_BC = self._T_EC @ np.linalg.inv(T_EB_after)
        self._T_CB = np.linalg.inv(self._T_BC)
        tb = self._T_BC[:3, 3]
        print(f"  [{tag}] ✓ 이동 완료 — T_BC 재캡처 "
              f"(t=({tb[0]*1000:+.1f},{tb[1]*1000:+.1f},"
              f"{tb[2]*1000:+.1f})mm)")

    @staticmethod
    def _rot_about_axis(axis: np.ndarray, ang_rad: float) -> np.ndarray:
        """Rodrigues — 단위축 기준 3x3 회전."""
        a = np.asarray(axis, float)
        a = a / max(np.linalg.norm(a), 1e-12)
        c, sn = np.cos(ang_rad), np.sin(ang_rad)
        K = np.array([[0.0, -a[2], a[1]],
                      [a[2], 0.0, -a[0]],
                      [-a[1], a[0], 0.0]])
        return np.eye(3) * c + np.outer(a, a) * (1.0 - c) + K * sn

    def _phase1_view_score(self, verts_C_mm, T_CB_cand,
                           fwd_C, up_C, env) -> tuple:
        """
        후보 preview 의 v1.5 품질 스코어 (docs §3.0 step 6).
        candidate C→B 변환 후 `_in_object_profile` (turntable floor +
        (r,z) profile) 로 물체점 추출 → 카메라 FOV 안 점들을 Gaussian
        거리가중으로 합산:

            score = Σ_p∈objpts ∩ FOV  exp(-((depth_p − d*)/σ_d)²)
            d*    = adaptive_target_standoff_mm (≈225mm)
            σ_d   = (band_hi − band_lo) / 2     (band 끝 ≈ e⁻¹)

        같은 개수라도 "최적 거리에 모인 각도" 가 이김.
        Returns (score(float), n_obj, n_excl_table) — 진단 카운트 동봉.
        """
        s = self.s
        v_all = np.asarray(verts_C_mm, float)
        xB = ((v_all / 1000.0) @ np.asarray(T_CB_cand[:3, :3], float).T
              + np.asarray(T_CB_cand[:3, 3], float))
        mask_obj, mask_excl = self._in_object_profile(xB, env)
        if not mask_obj.any():
            return 0.0, 0, int(mask_excl.sum())
        v = v_all[mask_obj]                              # 물체점, C mm
        # 캘리브된 광학 정규직교 기저 (fwd, up⟂, right)
        e1 = fwd_C / max(np.linalg.norm(fwd_C), 1e-12)
        e2 = up_C - (up_C @ e1) * e1
        e2 = e2 / max(np.linalg.norm(e2), 1e-12)
        e3 = np.cross(e1, e2)
        depth = v @ e1
        lat = v @ e3
        vert = v @ e2
        band_lo, band_hi = s.elevation_optimal_band_mm
        in_fov = (
            (depth > 1.0)
            & (np.abs(lat) <= depth * np.tan(SPIDER_HALF_FOV_H))
            & (np.abs(vert) <= depth * np.tan(SPIDER_HALF_FOV_V))
        )
        d_star = float(s.adaptive_target_standoff_mm)
        sigma_d = max((band_hi - band_lo) / 2.0, 1.0)
        w = np.exp(-((depth - d_star) / sigma_d) ** 2)
        score = float(np.sum(np.where(in_fov, w, 0.0)))
        return score, int(mask_obj.sum()), int(mask_excl.sum())

    def _distance_only_position(self, T_CB_home, est) -> None:
        """기존 거리-only 적응 — home orientation 유지 + 시선 ray 평행이동."""
        s = self.s
        cam_pos_home = np.asarray(T_CB_home[:3, 3], float)   # B, m
        center_B = np.array([est["cx"], est["cy"], est["z_mid"]])
        f = center_B - cam_pos_home
        nf = np.linalg.norm(f)
        if nf < 1e-6:
            print("  [adaptive] 카메라-물체 거리 ≈0 — skip (home 유지)")
            return
        f = f / nf
        d, partial = self._standoff_distance(est["h_obj"], est["r_obj"])
        T_CB_target = np.eye(4)
        T_CB_target[:3, :3] = np.asarray(T_CB_home[:3, :3], float)
        T_CB_target[:3, 3] = center_B - f * d
        print(f"  [adaptive] stand-off d={d*1000:.0f}mm "
              f"(target {s.adaptive_target_standoff_mm:.0f}, "
              f"range [{SPIDER_NEAR_MM:.0f},{SPIDER_FAR_MM:.0f}])"
              + ("  ⚠ 높이 초과 — 부분 커버(상/하단 일부 누락)"
                 if partial else ""))
        code = self._move_robot_to_T_CB(
            T_CB_target, s.adaptive_robot_speed_deg_s)
        if code != 0:
            print(f"  [adaptive] ✘ set_position 실패 (code={code}) "
                  f"— home 유지로 진행")
            return
        self._recapture_T_BC("adaptive")

    def _elevation_search(self, T_CB_home, env, offsets_deg,
                          fine_search_enabled: bool) -> bool:
        """
        probe envelope 기준 물체중심 피벗 zx평면 호 고도각을 탐색.
        - coarse: home_dist(비퇴행 baseline) + `offsets_deg` 후보
        - fine: `fine_search_enabled` True 일 때만 coarse best φ* 주변
                ±elevation_fine_step_deg 추가 평가
        후보별 preview 를 envelope 멤버십으로 물체점 판정 → v1.5 스코어 →
        best. 재조준 = probe 물체점 경험적 캘리브 광축(look_at_axes) →
        mis-aim 회피. 유효후보 없으면 home 복귀. docs §3.0 / §6.

        Returns
        -------
        moved : bool
            True = best 자세로 robot 이동 완료, self._T_BC 재캡처됨.
            False = home 유지 (또는 home 복귀).
        """
        s = self.s
        sensor = self.mms.sensor
        world_z = np.array([0.0, 0.0, 1.0])
        cam_home = np.asarray(T_CB_home[:3, 3], float)       # B, m
        center_B = np.asarray(env["center_B"], float)
        est_env = {"cx": center_B[0], "cy": center_B[1],
                   "z_mid": center_B[2], "h_obj": env["h_obj"],
                   "r_obj": env["r_obj"]}

        # 1. C-프레임 광축 경험적 캘리브 (probe 물체점)
        try:
            fwd_C, up_C, info = calibrate_camera_axes_from_preview(
                env["obj_vC"], T_CB_home, center_B, world_z)
        except Exception as e:
            print(f"  [elev] ⚠ C-frame 캘리브 실패 ({e}) — 거리-only fallback")
            self._distance_only_position(T_CB_home, est_env)
            return
        print(f"  [elev] C-frame 캘리브: fwd_C=({fwd_C[0]:+.3f},"
              f"{fwd_C[1]:+.3f},{fwd_C[2]:+.3f}) "
              f"crosscheck={info['crosscheck_deg']:.1f}° "
              f"closure={info['closure_rot_deg']:.1f}°")
        if (not np.isnan(info["crosscheck_deg"])
                and info["crosscheck_deg"] > 20.0):
            print(f"  [elev] ⚠ fwd_C 교차검증 {info['crosscheck_deg']:.1f}° "
                  f"(>20°) — preview 점군/조준 의심, 그래도 진행")
        if (not np.isnan(info["closure_rot_deg"])
                and info["closure_rot_deg"] > 15.0):
            print(f"  [elev] ⚠ closure {info['closure_rot_deg']:.1f}° "
                  f"(>15°) — 캘리브 신뢰도 낮음, 그래도 진행")

        # 2. 피벗축 = world_z 와 radial_horizontal 에 모두 ⟂ 인 수평축
        #    (네가 말한 zx평면 = 현재 시선을 품은 수직면. yaml 미사용.)
        radial = cam_home - center_B
        radial_h = radial.copy()
        radial_h[2] = 0.0
        if np.linalg.norm(radial_h) < 1e-6:
            print(f"  [elev] ⚠ 카메라가 물체 바로 위 — 호 정의 불가, "
                  f"거리-only fallback")
            self._distance_only_position(T_CB_home, est_env)
            return
        a_B = np.cross(world_z, radial_h)
        a_B = a_B / np.linalg.norm(a_B)

        d, partial = self._standoff_distance(
            env["h_obj"], env["r_obj"])
        print(f"  [elev] stand-off d={d*1000:.0f}mm"
              + ("  ⚠ 높이 초과 — 부분 커버" if partial else ""))

        # 3. 적응형 coarse→fine 후보 탐색 (docs §3.0 step 6).
        f_home = center_B - cam_home
        f_home = f_home / max(np.linalg.norm(f_home), 1e-12)
        T_home_dist = np.eye(4)
        T_home_dist[:3, :3] = np.asarray(T_CB_home[:3, :3], float)
        T_home_dist[:3, 3] = center_B - f_home * d

        def _pose_for_phi(phi_deg):
            R = self._rot_about_axis(a_B, np.radians(phi_deg))
            dir_phi = R @ radial
            dir_phi = dir_phi / max(np.linalg.norm(dir_phi), 1e-12)
            return look_at_axes(center_B + dir_phi * d,
                                center_B, fwd_C, up_C, world_z)

        score_min = max(300, s.adaptive_min_preview_verts // 3)
        best = None    # (score, tag, T_CB)
        cand_seq = 0   # 디버그 PLY 시퀀스

        def _eval(tag, T_cand):
            nonlocal best, cand_seq
            cand_seq += 1
            code = self._move_robot_to_T_CB(
                T_cand, s.adaptive_robot_speed_deg_s)
            if code != 0:
                print(f"  [elev] {tag}: set_position code={code} "
                      f"(도달불가) — skip")
                return
            vC = self._capture_preview_verts(sensor, score_min)
            if vC is None:
                print(f"  [elev] {tag}: preview 부족 — score 0")
                return
            sc, n_obj, n_excl = self._phase1_view_score(
                vC, T_cand, fwd_C, up_C, env)
            print(f"  [elev] {tag}: score={sc:.1f} "
                  f"(obj {n_obj}, table_excl {n_excl})")
            if s.probe_debug_dump:
                self._dump_candidate_ply(vC, T_cand, env, cand_seq, tag)
            if best is None or sc > best[0]:
                best = (sc, tag, T_cand)

        lo_r, hi_r = s.elevation_range_deg

        # 3a. coarse — home_dist(비퇴행 baseline) + offsets
        _eval("home_dist", T_home_dist)
        evaluated = []          # 평가한 φ (fine 중복 방지)
        for p in offsets_deg:
            phi = float(np.clip(float(p), lo_r, hi_r))
            if any(abs(phi - q) < 0.5 for q in evaluated):
                continue
            _eval(f"c{phi:+.1f}", _pose_for_phi(phi))
            evaluated.append(phi)

        # 3b. fine (옵션) — coarse best φ*(≠home_dist) 주변 ±fine_step
        if (fine_search_enabled and best is not None
                and best[1] != "home_dist" and best[0] > 0):
            phi_star = float(best[1][1:])     # "c-10.0" → -10.0
            step = float(s.elevation_fine_step_deg)
            for phi in (phi_star - step, phi_star + step):
                phi = float(np.clip(phi, lo_r, hi_r))
                if any(abs(phi - q) < 0.5 for q in evaluated):
                    continue
                _eval(f"f{phi:+.1f}", _pose_for_phi(phi))
                evaluated.append(phi)

        # 4. best 선택·이동. 유효후보 없으면 home 복귀 (비퇴행).
        if best is None or best[0] <= 0:
            print(f"  [elev] 유효 후보 없음 — home 복귀")
            self._move_robot_to_T_CB(
                T_CB_home, s.adaptive_robot_speed_deg_s)
            return False
        print(f"  [elev] ★ 선택 = {best[1]} (score={best[0]:.1f})")
        code = self._move_robot_to_T_CB(
            best[2], s.adaptive_robot_speed_deg_s)
        if code != 0:
            print(f"  [elev] ✘ best 이동 실패 (code={code}) — home 복귀")
            self._move_robot_to_T_CB(
                T_CB_home, s.adaptive_robot_speed_deg_s)
            return False
        self._recapture_T_BC("elev")
        return True

    def _adaptive_prescan_position(self, recovery: bool = False) -> bool:
        """
        현재 robot 자세에서 PREVIEW 로 물체 크기/위치 추정 → 스캐너(EE)를
        최적 작업거리·고도각으로 이동.

        2026-05-20 rule: scan 첫 시작에서는 호출되지 않음. tracking-lost
        recovery 흐름에서만 호출 (`recovery=True`). recovery 시 elevation
        후보 수를 축소(`recovery_elevation_offsets_deg` + fine skip)해 시간 단축.

        실패/예외는 모두 삼키고 home 유지. Returns True iff robot 이동 완료.
        docs/artec_scanning_pipeline.md §3.0 / §6.
        """
        s = self.s
        try:
            if self.robot is None or self._T_EC is None:
                print("  [adaptive] robot/T_EC 없음 — skip (home 유지)")
                return False
            sensor = getattr(self.mms, "sensor", None)
            if sensor is None or not hasattr(sensor, "capture_frame"):
                print("  [adaptive] sensor.capture_frame 없음 — skip")
                return False

            T_EB_home = self.robot.get_ee_pose_mat()        # E→B (m)
            T_CB_home = self.mms.T_CB(T_EB_home)            # C→B (m)

            # recovery 모드면 축소 offsets, 일반 모드면 풀그리드 (forward-compat).
            if recovery:
                offsets = list(s.recovery_elevation_offsets_deg)
                fine_enabled = bool(s.recovery_elevation_fine_search_enabled)
            else:
                offsets = [-10.0, -5.0, 0.0, 5.0, 10.0]
                fine_enabled = True

            # 턴테이블 회전 차분 probe 로 물체 envelope 추정 (매번 fresh).
            env = self._motion_probe_object(T_CB_home)
            if env.get("ok"):
                return self._elevation_search(
                    T_CB_home, env, offsets, fine_enabled)
            # probe 실패 → preview robust center + 거리-only fallback
            print(f"  [adaptive] probe 실패 — 거리-only fallback")
            est = self._robust_center_from_preview(sensor, T_CB_home)
            if est is None:
                print(f"  [adaptive] preview 부족 — skip (home 유지)")
                return False
            print(f"  [adaptive] fallback center=({est['cx']:+.3f},"
                  f"{est['cy']:+.3f}) h={est['h_obj']*1000:.0f}mm "
                  f"r={est['r_obj']*1000:.0f}mm")
            self._distance_only_position(T_CB_home, est)
            return True
        except Exception as e:
            print(f"  [adaptive] ⚠ 예외 ({type(e).__name__}: {e}) "
                  f"— home 유지로 진행")
            return False

    # ── Entry ──────────────────────────────────────────────────────────

    # ── ScanBackend 프리미티브 (공용 utils/nbv/scan_phase_controller) ────
    # 순서/게이팅/NBV 수렴 루프는 공용 컨트롤러 소유. 이 클래스는 real Artec
    # 캡처 하드웨어(streaming + relocalization + recovery + hint)만 제공.
    @property
    def phase_mode(self) -> int:
        return self.s.phase_mode

    @property
    def nbv_k_max(self) -> int:
        return self.s.nbv_K_max

    def run(self) -> ArtecMultiPassScanResult:
        # 2026-05-20 rule: scan 첫 시작은 robot=home 그대로. 사전 probe/elevation
        # 호출 없음. tracking-lost 시 _attempt_recovery 가 비로소
        # _adaptive_prescan_position(recovery=True) 를 발동. (docs §3 / §6)
        #
        # 구조 (2026-07-01): Phase 1→2→3 순서/NBV 루프는 공용
        # utils/nbv/scan_phase_controller.run_scan_phases 가 소유. 여기 run() 은
        # 상태(_RunState) + live viewer 만 준비하고 컨트롤러에 위임한다.
        self._st = _RunState(master_model=artec_base.create_model())

        # ── 라이브 뷰어 (옵션) — 모든 회전이 한 화면에 누적 ──────────
        # 생성/사용/종료 모두 예외 격리. 실패해도 스캔에 영향 없음.
        if self.s.enable_live_viewer:
            try:
                from mms_artec.nbv.live_scan_viewer import LiveScanViewer
                self._st.live_viewer = LiveScanViewer()
            except Exception as e:
                print(f"  [live] ⚠ viewer 생성 실패 "
                      f"({type(e).__name__}: {e}) — 라이브 표시 없이 진행")
                self._st.live_viewer = None

        return run_scan_phases(self)

    # ── 공통 ────────────────────────────────────────────────────────────
    def confirm_start(self) -> bool:
        # Phase 1 시작 전 1회 확인 — Studio 시작 위치 등. real 전용(사람 개입).
        s = self.s
        st = self._st
        if s.prompt_before_first_pass:
            self._print_pose_hint_for_user(st.pose_idx)
            print("  [Enter] 회전 시작 / [q]+Enter 종료")
            if self._wait_user_quit():
                st.user_quit = True
                st.aborted_reason = "사용자 종료 (첫 pass 전)"
                return False
        return True

    def pick_phase1_pose(self):
        # real: 스캔 첫 시작은 robot=home 그대로 → 구동 없이 현재 포즈에서 캡처.
        return AT_CURRENT

    def go_home(self) -> None:
        # Phase 2→3 전환 등 — NBV 로 움직인 robot 을 home 복귀 (Phase 3 는 robot 고정).
        try:
            self.robot.go_home(sensor="artec", confirm=False)
            print("  [phase] robot home 복귀")
        except Exception as e:
            print(f"  [phase] ⚠ go_home 실패({e}) — 자세 확인 필요")

    def capture_rotation(self, pose, label: str, phase: int) -> bool:
        """real 캡처 (턴테이블 전회전). 두 경로 모두 보존:
          - pose=AT_CURRENT (Phase 1/3): 현재 고정 포즈에서 streaming +
            cleanup/hint/master 병합 + tracking-lost recovery (_do_one_rotation
            재시도 루프). 반환 False=중단.
          - pose=q (Phase 2 NBV): 로봇을 q 로 구동 후 streaming + camera-motion
            T_pre 병합 (_capture_nbv_pose). 반환 False=캡처 실패(루프 종료).
        """
        st = self._st
        if pose is AT_CURRENT:
            while st.n_pass < self.s.max_passes:
                status = self._do_one_rotation(st)
                if status == _ROT_ABORT:
                    return False
                if status == _ROT_RETRY:
                    continue
                return True                     # 정상 완료
            return False                        # max_passes 소진
        # Phase 2 — NBV 자세로 로봇 구동 후 streaming 캡처 + 병합.
        sub, T_pre = self._capture_nbv_pose(None, pose)
        if sub is None or sub.model.scan_count() == 0:
            print("  [nbv] 캡처 실패 — 종료.")
            return False
        n = self._merge_into_master(sub.model, st.master_model, T_pre)
        print(f"  [nbv] {n} scan 병합 (master scans={st.master_model.scan_count()})")
        return True

    def finalize(self) -> ArtecMultiPassScanResult:
        st = self._st
        s = self.s
        # phase_mode<3 → Phase 3 미실행 완료 사유. 단 phase1 이 정상 완료한 경우만
        # (사용자 종료 / max_passes 소진 시엔 아래 max_passes 블록·기존 사유가 우선).
        if (s.phase_mode < 3 and not st.aborted_reason
                and not st.user_quit and st.n_pass < s.max_passes):
            st.aborted_reason = f"phase_mode={s.phase_mode} 완료 (Phase 1..{s.phase_mode})"

        if st.live_viewer is not None:
            try:
                st.live_viewer.close()
            except Exception:
                pass

        if st.n_pass >= s.max_passes:
            st.aborted_reason = st.aborted_reason or f"max_passes={s.max_passes} 도달"
            print(f"\n  ⓘ {st.aborted_reason}")

        n_total_frames = sum(
            st.master_model.get_scan(i).frame_count()
            for i in range(st.master_model.scan_count())
        )

        print(f"\n═══ Multi-pass 종료 ═══")
        print(f"  passes              : {st.n_pass}")
        print(f"  master scan_count   : {st.master_model.scan_count()}")
        print(f"  master total frames : {n_total_frames}")
        print(f"  user_quit           : {st.user_quit}")
        if st.aborted_reason:
            print(f"  reason              : {st.aborted_reason}")

        if st.n_recovery_attempts > 0:
            print(f"  recovery            : {st.n_recovery_succeeded}/"
                  f"{st.n_recovery_attempts} 성공")

        return ArtecMultiPassScanResult(
            model=st.master_model,
            n_passes=st.n_pass,
            n_total_frames=n_total_frames,
            pass_results=st.pass_results,
            user_quit=st.user_quit,
            aborted_reason=st.aborted_reason,
            hints_applied=st.hints_applied,
            master_center_mm=st.master_center,
            n_recovery_attempts=st.n_recovery_attempts,
            n_recovery_succeeded=st.n_recovery_succeeded,
            recorded_hints=st.recorded_hints,
            icp_refine_log=st.icp_refine_log,
        )

    # ── Phase 1 ─────────────────────────────────────────────────────────
    # (Phase 1 캡처는 pick_phase1_pose=AT_CURRENT + capture_rotation 이 담당)

    # ── Phase 3 (외부 flip → 윗면) ──────────────────────────────────────
    def supports_phase3(self) -> bool:
        return True                             # real: 사용자 손회전으로 flip 가능

    def next_flip(self) -> bool:
        """Phase 3 — 사용자에게 물체를 다음 flip pose 로 뒤집도록 안내(+pose_idx
        advance). 정상 완료 후 호출되어 직전 pass 완료 로그도 출력. 더 진행 불가
        (single-pass / 사용자 종료 / max_passes)면 False."""
        st = self._st
        s = self.s
        if st.n_pass >= s.max_passes:
            return False
        # prompt_between_passes=False → Phase 3(flip)는 사람 개입 전제라 진행 불가.
        if not s.prompt_between_passes:
            st.aborted_reason = "single-pass mode (no inter-pass prompt)"
            return False
        print(f"\n  ✓ Pass {st.n_pass} (pose {st.pose_idx}) 완료 "
              f"— frames={st.last_n_frames}")
        next_pose = st.pose_idx + 1
        if next_pose < len(s.pose_physical_rotations):
            print(f"  → 다음 pose ({next_pose}) 자세로 아이템 회전 후 [Enter]")
            self._print_pose_hint_for_user(next_pose)
        else:
            print(f"  → 추가 pass 진행하려면 [Enter] (모든 정의된 pose 완료)")
        print(f"    종료하려면 [q]+Enter")
        if self._wait_user_quit():
            st.user_quit = True
            st.aborted_reason = "사용자 종료 (정상 완료 후)"
            return False
        st.pose_idx += 1                        # 정상 완료 시만 advance
        return True

    # ── 회전 1회 (Phase 1·3 공유) ───────────────────────────────────────

    def _do_one_rotation(self, st: "_RunState") -> str:
        """회전 1회: streaming → cleanup → hint → merge → viewer → master_center,
        그리고 tracking-lost recovery. Phase 1·3 이 공유한다.

        Returns _ROT_OK(정상), _ROT_RETRY(같은 pose 재시도), _ROT_ABORT(중단).
        """
        s = self.s
        self._print_pass_banner(st.n_pass + 1, s.max_passes, st.pose_idx)
        if st.live_viewer is not None:
            try:
                st.live_viewer.new_pass(
                    f"Pass {st.n_pass + 1} (pose {st.pose_idx})")
            except Exception:
                pass

        # ── Recovery 직후면 streaming 의 pre-scan clearpos skip ──
        # 우리가 이미 turntable 을 safe-back 으로 이동시켰음. clearpos 가
        # 그 위치를 새 0° 로 redefine 하면 의도와 어긋남.
        _orig_reset_to_zero = s.streaming_settings.reset_to_zero_first
        if st.next_skip_clearpos:
            s.streaming_settings.reset_to_zero_first = False

        # ── 단일 회전 ─────────────────────────────────────────────
        try:
            single = ArtecStreamingScanSession(
                self.mms, self.robot, self.turntable, s.streaming_settings,
                live_viewer=st.live_viewer,
            )
            sub_result = single.run()
        finally:
            # streaming_settings 는 multipass 인스턴스 외부에서 공유될 수
            # 있으므로 반드시 원복.
            s.streaming_settings.reset_to_zero_first = _orig_reset_to_zero
            st.next_skip_clearpos = False
        st.pass_results.append(sub_result)
        st.n_pass += 1
        st.last_n_frames = sub_result.n_frames

        # ── Pass cleanup: SerialReg + OutlierRemoval ───────────────
        # 멀티패스 끝까지 기다리지 말고 회전마다 즉시 정합/이상점 제거.
        # 이유: (a) hint 계산 (centroid) 이 깨끗한 데이터로 안정,
        #       (b) live viewer 가 정합된 IScan 을 그대로 비춰서 사용자가
        #           회전별 형상 / 정합 품질을 즉시 검증 가능
        #           ([[feedback_live_viewer_must_mirror_scan]]).
        # Outliers 는 Fusion 전에 와야 함 ([[feedback_artec_pipeline_order]]).
        # tracking_lost 면 partial IScan 이라 SerialReg 가 깨질 수 있어 skip.
        if (not sub_result.tracking_lost
                and sub_result.model.scan_count() > 0):
            try:
                cleaned = artec_algorithm.Algorithms.serial_registration(
                    sub_result.model,
                )
                cleaned = artec_algorithm.Algorithms.outliers_removal(
                    cleaned,
                )
                sub_result.model = cleaned
                print(f"\n  [pass cleanup] SerialReg + OutlierRemoval 완료")
            except Exception as e:
                print(f"\n  [pass cleanup] ⚠ 실패 "
                      f"({type(e).__name__}: {e}) — raw IScan 사용")

        # ── Pose hint 계산 (centroid-aware + base-frame aware) ─────
        # 1. R_phys 는 base frame B 에서 정의 (사용자 직관)
        # 2. T_BC 있으면 R_W = T_BC @ R_B @ T_CB 로 scan world 로 변환
        # 3. T_pre = Translate(c_master) @ inv(R_W) @ Translate(-c_pass)
        #    객체 centroid 를 pivot 으로 회전 → camera 원점 기준 30cm
        #    translation 오류 제거.
        T_pre = None

        # Recovery override: 직전 회전에서 robot 이 움직였다면 camera-motion
        # 만 보정 (object 회전 무관, pose_idx 그대로). 다른 R_phys hint 보다 우선.
        #   x_W_master = T_BC_master @ T_CB_new @ x_W_new
        #              = T_BC_master @ inv(T_BC_new) @ x_W_new
        # SDK frame_transformation 은 mm 라 translation 만 m→mm scale.
        if st.next_T_BC_pending is not None:
            if self._T_BC is None:
                print(f"  [recovery hint] ⚠ T_BC_master 미설정 — override skip")
            else:
                try:
                    T_pre_cam = self._T_BC @ np.linalg.inv(st.next_T_BC_pending)
                    T_pre = T_pre_cam.copy()
                    T_pre[:3, 3] *= 1000.0          # m → mm (SDK 단위)
                    print(f"\n  [recovery hint] camera-motion correction "
                          f"applied (Δtrans = ({T_pre[0,3]:+.1f}, "
                          f"{T_pre[1,3]:+.1f}, {T_pre[2,3]:+.1f}) mm)")
                except np.linalg.LinAlgError as e:
                    print(f"  [recovery hint] ⚠ inv 실패 ({e}) — override skip")
            st.next_T_BC_pending = None

        R_phys = (s.pose_physical_rotations[st.pose_idx]
                  if st.pose_idx < len(s.pose_physical_rotations) else None)
        if T_pre is None and R_phys is not None and not np.allclose(R_phys, np.eye(4), atol=1e-9):
            try:
                R_phys_inv_B = np.linalg.inv(R_phys)[:3, :3]
            except np.linalg.LinAlgError:
                print(f"  [hint] ⚠ pose_physical_rotations[{st.pose_idx}] inverse 실패 — skip")
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
                c_ref = st.master_center if st.master_center is not None else c_pass
                T_pre = np.eye(4)
                T_pre[:3, :3] = R_phys_inv_W
                T_pre[:3, 3] = c_ref - R_phys_inv_W @ c_pass
                st.hints_applied = True
                print(f"\n  [hint pose {st.pose_idx}] frame={frame_tag}")
                print(f"  [hint pose {st.pose_idx}] c_pass = ({c_pass[0]:+.1f}, "
                      f"{c_pass[1]:+.1f}, {c_pass[2]:+.1f}) mm")
                print(f"  [hint pose {st.pose_idx}] c_master = ({c_ref[0]:+.1f}, "
                      f"{c_ref[1]:+.1f}, {c_ref[2]:+.1f}) mm")
                print(f"  [hint pose {st.pose_idx}] translation = ({T_pre[0,3]:+.1f}, "
                      f"{T_pre[1,3]:+.1f}, {T_pre[2,3]:+.1f}) mm")

        # Sub-model 의 IScan 들 → master_model.
        # apply_hints_to_frame_transformations=False 일 때 hint 는 IScan 의
        # frame_transformations 에 박지 않고 recorded_hints 에 기록만 (병합
        # 비교용). 같은 raw scan 데이터로 후처리 단계에서 hint on/off 를
        # swap 가능.
        if T_pre is not None and not s.apply_hints_to_frame_transformations:
            idx_before = st.master_model.scan_count()
            n_added = self._merge_into_master(
                sub_result.model, st.master_model, None,  # hint 안 박음
            )
            for idx in range(idx_before, st.master_model.scan_count()):
                st.recorded_hints.append((idx, T_pre.copy()))
            if n_added > 0:
                print(f"\n  [hint record] T_pre → scan_idx "
                      f"{idx_before}..{st.master_model.scan_count() - 1} "
                      f"(apply_hints=False, 후처리에서 선택 적용)")
        else:
            # Hint ICP refine — centroid-pivot T_pre 를 init 으로 colored
            # ICP 돌려 측정된 T 로 교체. 사용자 손회전의 ±10° 오차 흡수.
            # docs §4.2 face-merging 회피.
            T_pre_to_apply = T_pre
            if (T_pre is not None and s.hint_icp_refine_mode
                    and st.master_model.scan_count() > 0):
                print(f"  [icp_refine] hint refine 시작 "
                      f"(voxel={s.icp_voxel_mm}mm, "
                      f"corr={s.icp_corr_dist_mm}mm, "
                      f"color_w={s.icp_color_weight})")
                T_pre_refined, fit, rmse = self._hint_icp_refine(
                    sub_result.model, T_pre, st.master_model)
                idx_for_log = st.master_model.scan_count()  # 곧 추가될 인덱스
                st.icp_refine_log.append(
                    (idx_for_log, T_pre.copy(), T_pre_refined.copy(),
                     fit, rmse))
                T_pre_to_apply = T_pre_refined
            n_added = self._merge_into_master(
                sub_result.model, st.master_model, T_pre_to_apply,
            )
        print(f"\n  [pass {st.n_pass} / pose {st.pose_idx}] {n_added} scan(s) → master "
              f"(total scans={st.master_model.scan_count()})")

        # Viewer 를 cleaned master_model 로 재구성 — 회전 중 누적된 raw
        # 점들 (정합 전 위치) 을 cleanup + T_pre 적용된 점들로 교체.
        # viewer = scan 일관성 ([[feedback_live_viewer_must_mirror_scan]]).
        if st.live_viewer is not None:
            try:
                st.live_viewer.rebuild_from_model(st.master_model)
            except Exception as e:
                print(f"  [live] ⚠ rebuild_from_model 호출 실패 "
                      f"({type(e).__name__}: {e})")

        # Pass 1 (또는 첫 성공한 IScan) 후 master_center lock
        if st.master_center is None and st.master_model.scan_count() > 0:
            st.master_center = self._compute_model_centroid(st.master_model)
            print(f"  [master_center] locked at "
                  f"({st.master_center[0]:+.1f}, {st.master_center[1]:+.1f}, "
                  f"{st.master_center[2]:+.1f}) mm")

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
                st.aborted_reason = (
                    f"turntable drive alarm / 통신 사망 — "
                    f"EziSERVO 전원 OFF→5s→ON 필요"
                )
                print(f"\n  ✘ {st.aborted_reason}")
                print(f"  ✘ 모터가 안 멈추면 emergency_stop 도 실패한 것 — "
                      f"물리 전원 차단해야 함.")
                print(f"  ✘ 복구 후 main_artec.py 재실행.")
                return _ROT_ABORT

            # ── 자동 recovery 시도 ─────────────────────────────────
            # auto_recovery_enabled 이고 max_recovery_retries 미만이면
            # turntable safe-back + (probe + 축소 elevation search) 로
            # 새 robot 자세 결정 후 자동 재시도. docs §6.
            recovery_initiated = False
            if (s.auto_recovery_enabled
                    and st.recovery_retry_count < s.max_recovery_retries):
                ok, T_BC_new = self._attempt_recovery(
                    sub_result, st.master_model, st.recovery_retry_count,
                )
                if ok:
                    st.recovery_retry_count += 1
                    st.n_recovery_attempts += 1
                    # robot 이 움직였으면 다음 merge 의 T_pre override 용
                    if T_BC_new is not None:
                        st.next_T_BC_pending = T_BC_new
                    # turntable 을 safe-back 으로 manual 이동했으므로
                    # 다음 streaming session 은 pre-scan clearpos skip.
                    st.next_skip_clearpos = True
                    recovery_initiated = True
                    print(f"  → recovery #{st.recovery_retry_count}/"
                          f"{s.max_recovery_retries} 진입 — 자동 재시도")
            elif (s.auto_recovery_enabled
                    and st.recovery_retry_count >= s.max_recovery_retries):
                print(f"\n  ⓘ recovery 한계 도달 "
                      f"({st.recovery_retry_count}/{s.max_recovery_retries}) "
                      f"— user prompt 로 fallback")

            if recovery_initiated:
                return _ROT_RETRY   # 같은 pose 재시도

            # ── User-prompt fallback (기존 동작) ────────────────
            if not s.prompt_on_tracking_lost:
                st.aborted_reason = (
                    f"tracking lost (auto, no prompt): {sub_result.loss_reason}"
                )
                print(f"  ⚠ {st.aborted_reason}")
                return _ROT_ABORT
            print(f"\n  ⚠ Pass {st.n_pass} tracking lost: {sub_result.loss_reason}")
            print(f"  → 같은 pose (pose {st.pose_idx}) 그대로 두고 [Enter] 재시도 / [q]+Enter 종료")
            if self._wait_user_quit():
                st.user_quit = True
                st.aborted_reason = "사용자 종료 (lost 직후)"
                return _ROT_ABORT
            # User 가 수동 retry 한 경우 — recovery_retry_count 는 reset
            # (수동 개입은 새 시작으로 간주).
            st.recovery_retry_count = 0
            return _ROT_RETRY       # 같은 pose 재시도

        # 정상 완료 — recovery 카운터 reset.
        if st.recovery_retry_count > 0:
            st.n_recovery_succeeded += 1
            print(f"\n  ✓ recovery 후 정상 완료 — retry 카운터 reset "
                  f"(누적 성공: {st.n_recovery_succeeded})")
            st.recovery_retry_count = 0
        return _ROT_OK

    # ── Recovery ───────────────────────────────────────────────────────

    def _attempt_recovery(
        self,
        sub_result: ArtecStreamingScanResult,
        master_model: "artec_base.ModelHandle",
        retry_idx: int,
    ) -> tuple[bool, Optional[np.ndarray]]:
        """
        Tracking lost 발생 시 자동 복구 시도 (2026-05-20 재설계).

        흐름 (docs §6):
          1. last-good θ + safe_back_margin_deg 만큼 turntable 역회전.
          2. _adaptive_prescan_position(recovery=True) 호출 — fresh probe +
             축소 elevation search 로 새 robot 자세 결정·이동. master point
             cloud 비의존.

        Returns
        -------
        (ok, T_BC_new)
          ok        — recovery 진입 성공 여부 (False 면 user-prompt fallback).
          T_BC_new  — robot 이동 후 새 B → C transform (next merge 의
                      T_pre 계산용). 자세 변경 없으면 None.
        """
        s = self.s
        if self._T_EC is None or self._T_BC is None or self.robot is None:
            print(f"  ⚠ recovery 조건 미충족: T_EC/robot 확인")
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

        # ── 3. 적응형 자세 재선정 (fresh probe + 축소 elevation) ───────
        print(f"  → adaptive 자세 재선정 (recovery 모드)")
        moved = self._adaptive_prescan_position(recovery=True)
        if not moved:
            print(f"  ⓘ 자세 미변경 (probe 실패 또는 home 유지) — 같은 자리 재시도")
            return True, None
        # _adaptive_prescan_position 이 _recapture_T_BC 까지 끝냈으므로
        # self._T_BC 가 새 값. 이걸 next merge 의 T_pre 계산용으로 반환.
        T_BC_recovery = self._T_BC.copy() if self._T_BC is not None else None
        if T_BC_recovery is not None:
            t = T_BC_recovery[:3, 3]
            print(f"  T_BC re-captured: trans = ({t[0]*1000:+.1f}, "
                  f"{t[1]*1000:+.1f}, {t[2]*1000:+.1f}) mm")
        return True, T_BC_recovery

    # ── Hint ICP refine 헬퍼 (2026-05-20) ──────────────────────────────

    @staticmethod
    def _frame_vertex_colors(frame) -> Optional[np.ndarray]:
        """FrameMeshHandle 의 (N,3) RGB 색상 [0,1] 추정 — uv·image 활용.
        텍스처 없거나 uv 없으면 None."""
        try:
            uv = frame.uv()
            img = frame.image()
        except Exception:
            return None
        if uv is None or img is None:
            return None
        if uv.ndim != 2 or uv.shape[1] != 2:
            return None
        H, W = img.shape[:2]
        u_px = np.clip((uv[:, 0] * W).astype(np.int32), 0, W - 1)
        # v 좌표가 top-down 인지 bottom-up 인지 SDK 가 명세 없음 — 기본
        # bottom-up 가정 (OpenGL convention). 시각 결과 이상하면 1-v 로 뒤집기.
        v_norm = uv[:, 1]
        v_px = np.clip(((1.0 - v_norm) * H).astype(np.int32), 0, H - 1)
        cols = img[v_px, u_px].astype(np.float64) / 255.0
        if cols.shape[1] >= 3:
            return cols[:, :3]
        return None

    @staticmethod
    def _iscan_to_pcd_B(scan, T_CB_master: np.ndarray, voxel_mm: float,
                        max_frames: int = 50):
        """단일 IScan 의 frame vertices+colors 를 B 프레임 Open3D PointCloud 로
        변환 (voxel downsample). T_CB_master = 첫 pass C→B (translation m).

        IScan vertices 는 mm, scan-world W (≈ C_first). C→B 후 base frame.
        반환: o3d.geometry.PointCloud (translation mm 단위) 또는 None.
        """
        try:
            import open3d as o3d
        except Exception:
            return None
        n = scan.frame_count()
        if n == 0:
            return None
        # subsample frames for speed
        if n > max_frames:
            step = max(1, n // max_frames)
            idxs = list(range(0, n, step))[:max_frames]
        else:
            idxs = list(range(n))
        R_CB = np.asarray(T_CB_master[:3, :3], float)
        t_CB_mm = np.asarray(T_CB_master[:3, 3], float) * 1000.0
        all_v, all_c = [], []
        for i in idxs:
            frame = scan.get_frame(i)
            v = frame.vertices()
            if v is None or v.shape[0] == 0:
                continue
            T_f = scan.get_frame_transformation(i)
            v_W = v @ np.asarray(T_f[:3, :3], float).T + np.asarray(T_f[:3, 3], float)
            v_B = v_W @ R_CB.T + t_CB_mm
            all_v.append(v_B)
            c = ArtecMultiPassScanSession._frame_vertex_colors(frame)
            if c is not None and c.shape[0] == v.shape[0]:
                all_c.append(c)
            else:
                all_c.append(np.full((v.shape[0], 3), 0.5))   # gray fallback
        if not all_v:
            return None
        V = np.vstack(all_v)
        C = np.vstack(all_c)
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(V)
        pcd.colors = o3d.utility.Vector3dVector(np.clip(C, 0.0, 1.0))
        if voxel_mm > 0:
            pcd = pcd.voxel_down_sample(voxel_mm)
        return pcd

    @staticmethod
    def _master_to_pcd_B(master_model, T_CB_master: np.ndarray,
                         voxel_mm: float, max_frames_per_scan: int = 30):
        """master IModel 의 모든 IScan 을 B 프레임 colored PCD 로 합치기."""
        try:
            import open3d as o3d
        except Exception:
            return None
        merged = o3d.geometry.PointCloud()
        for s in range(master_model.scan_count()):
            scan = master_model.get_scan(s)
            pcd = ArtecMultiPassScanSession._iscan_to_pcd_B(
                scan, T_CB_master, voxel_mm, max_frames=max_frames_per_scan)
            if pcd is not None:
                merged += pcd
        if len(merged.points) == 0:
            return None
        if voxel_mm > 0:
            merged = merged.voxel_down_sample(voxel_mm)
        return merged

    # ─────────────────────────────────────────────────────────────────────
    # Phase 2 — 부족면 NBV 보강 루프 (docs/3_phase2.md §3). phase_mode=2.
    # 하드웨어 무관 코어는 utils/nbv/phase2_nbv.py + theta_planner(해석 IK).
    # ⚠ 캡처는 v1 에서 기존 streaming(풀 회전) + camera-motion T_pre(R3) 병합.
    #   relocalization R1/R2(§3.4.1)·짧은 스윕은 실기 검증 후 배선(§6.5).
    # ─────────────────────────────────────────────────────────────────────

    def _build_collision_world(self):
        """turntable calibration(T_BF0) → CollisionWorld (disc+몸체). 없으면 None.
        대상물 자체는 장애물로 넣지 않음(225mm standoff 로 접근 대상). 충돌검사 전제."""
        if not self.s.nbv_collision_enabled:
            return None
        tt = getattr(self.mms, "turntable_transform", None)
        T_BF0 = getattr(tt, "T_BF0", None) if tt is not None else None
        if T_BF0 is None:
            print("  [nbv] ⚠ turntable_transform/T_BF0 없음 — 충돌 world 미구성(검사 skip)")
            return None
        from utils.collision.robot_collision import CollisionWorld
        T_BF0 = np.asarray(T_BF0, float)
        world = CollisionWorld.from_turntable(
            surface_point=T_BF0[:3, 3], axis_dir=T_BF0[:3, 2],
            disc_radius=self.s.nbv_turntable_radius_mm / 1000.0,
            body_height=self.s.nbv_turntable_body_height_mm / 1000.0,
            margin=self.s.nbv_collision_margin_mm / 1000.0)
        print(f"  [nbv] 충돌 world 구성 (disc r={self.s.nbv_turntable_radius_mm:.0f}mm, "
              f"margin={self.s.nbv_collision_margin_mm:.0f}mm)")
        # ── 충돌 금지 원기둥 (turntable 위 금지구역) ──
        if self.s.nbv_keepout_enable:
            cxy = (self.s.nbv_keepout_center_xy
                   if self.s.nbv_keepout_center_xy is not None else T_BF0[:2, 3])
            R = (self.s.nbv_keepout_radius_mm / 1000.0
                 if self.s.nbv_keepout_radius_mm is not None
                 else self.s.nbv_turntable_radius_mm / 1000.0)  # 기본=턴테이블 지름
            z0 = float(T_BF0[2, 3])                             # disc 표면
            world.add_cylinder("keepout", cxy, z0, z0 + self.s.nbv_keepout_height_mm / 1000.0,
                               R, margin=self.s.nbv_collision_margin_mm / 1000.0)
            print(f"  [nbv] keep-out 원기둥 추가 (r={R*1000:.0f}mm, "
                  f"h={self.s.nbv_keepout_height_mm:.0f}mm)")
        return world

    def _build_master_mesh_B(self, master_model):
        """master IModel → B 프레임 pcd → Poisson mesh (frontier 입력)."""
        from utils.nbv import phase2_nbv as _p2
        pcd = self._master_to_pcd_B(
            master_model, self._T_CB, voxel_mm=self.s.nbv_master_voxel_mm)
        if pcd is None or len(pcd.points) < 200:
            return None
        return _p2.pcd_to_mesh_poisson(
            pcd, depth=self.s.nbv_poisson_depth,
            density_quantile=self.s.nbv_density_quantile)

    def _nbv_feasible_q(self, T_CB_des, q_seed):
        """T_CB_des → T_EB → 해석 IK q. 충돌 world 있으면 **궤적 전체(swept-path)**
        충돌검사(q_seed→q 관절보간). 실패=None.
        IK 결정 2026-06: SDK get_inverse_kinematics 미사용 — 해석 IK(kin).
        q_seed = 현재 관절각(모션 시작점) — IK seed 겸 swept 시작."""
        from utils.robot import xarm7_kinematics as _kin
        T_EB = T_CB_des @ self._T_EC                      # E→B (m)
        pose6d = np.concatenate([T_EB[:3, 3] * 1000.0,    # mm
                                 _kin.R_to_euler_xyz(T_EB[:3, :3])])
        q, ok = _kin.ik(pose6d, seed=q_seed)
        if not ok:
            return None
        world = getattr(self, "_collision_world", None)
        if world is not None:
            from utils.collision.robot_collision import swept_pose_collision
            col, _why, _s = swept_pose_collision(
                world, q_seed, q, T_EC=self._T_EC, n_steps=self.s.nbv_swept_steps)
            if col:
                return None
        return q

    def _axis_view_q(self, look_target, el_deg, az_deg, standoff, q_seed, rolls=None):
        """턴테이블축(look_target, **base**)을 (el,az,standoff)에서 보는 카메라 → 해석 IK q.

        ★ **sim·real 공용** `utils/robot/view_pose.solve_view_q` 위임(2026-08).
          예전에는 sim 만 roll 6방향·IK 시드 8개를 쓰고 real 은 각각 1개였다. 같은
          solver(`xarm7_kinematics.ik`)를 쓰면서도 **sim 에서 검증한 자세가 real 에서
          '도달 불가'로 버려지는** 상태였다(sim 검증이 실물을 보장하지 못함).

          - roll 은 자유 DOF — 스캐너를 광축 둘레로 돌려도 같은 면을 본다.
            실측: 고정하면 el30 이 0/8, 풀면 5/8(시드를 12개로 늘려도 고정이면 0/8).
          - DLS IK 는 국소해라 시드 하나면 도달 가능한 자세를 놓친다(0/8 ↔ 8/8).
          - roll=0·주어진 시드를 **먼저** 쓰므로 기존에 풀리던 해는 그대로다.

          카메라 규약은 real 그대로 **OpenCV**(`T_EC_artec` 와 짝) — 규약은 데이터로만
          넘기고 캘리브레이션은 건드리지 않는다. USD 규약과는 카메라 로컬 Y축 180°
          회전 하나 차이임을 실측 확인(모든 el/az 편차 1.8e-12).

        충돌검사는 하지 않는다(공용 plan_nbv_elevation_pose 의 swept_free_fn 담당).
        """
        from utils.robot import xarm7_kinematics as _kin
        from utils.robot import view_pose as _vp
        rolls = rolls or getattr(self.s, "view_rolls_deg", None) or _vp.DEFAULT_ROLLS_DEG
        q, _roll, _eye = _vp.solve_view_q(
            _kin, look_target, el_deg, az_deg, standoff, q_seed, self._T_EC,
            T_WB=None,                       # look_target 이 이미 base 프레임
            convention=_vp.CAM_OPENCV, rolls_deg=rolls)
        return q

    def _plan_nbv_pose(self, q_cur, gaps, mesh):
        """Phase 2 관측자세 — 판단은 **sim·real 공용** `utils/nbv/nbv_planner` 가 한다.

        real 이 주입하는 것은 두 가지뿐이다:
          · 자세 생성 = OpenCV 규약(+Z 광축) + base 프레임 타깃 (`_axis_view_q`)
          · 충돌 판정 = `swept_pose_collision`

        ★ 2026-08 이전에는 이 함수가 공용 알고리즘을 **얇게만** 감싸서, sim 에만 있던
          판단들이 real 에 없었다 — 특히 `visited` 를 안 넘겨 **같은 자세를 무한 반복**
          했다(az 를 '관절이동 최소'로 고르므로 직전 자세 비용이 0). 아랫면 gap 제외·
          실패 원인 진단도 없었다. 이제 sim 과 같은 코드를 쓴다.
        """
        from utils.control.theta_planner import DEFAULT_JOINT_WEIGHTS
        from utils.collision.robot_collision import swept_pose_collision
        from utils.nbv.nbv_planner import NbvPlanner

        tt = getattr(self.mms, "turntable_transform", None)
        T_BF0 = getattr(tt, "T_BF0", None) if tt is not None else None
        if T_BF0 is None:
            return None
        T_BF0 = np.asarray(T_BF0, float)
        verts = np.asarray(mesh.vertices)                  # 메쉬(B) mid z = 객체 중간높이
        look_target = np.array([T_BF0[0, 3], T_BF0[1, 3], float(verts[:, 2].mean())])
        standoff = self.s.nbv_distance_mm / 1000.0

        if getattr(self, "_nbv", None) is None:
            # visited 를 pass 사이에 유지해야 하므로 세션에 1회만 만든다.
            self._nbv = NbvPlanner(joint_weights=DEFAULT_JOINT_WEIGHTS,
                                   el_floor_deg=self.s.nbv_el_floor_deg,
                                   log=lambda m: print(f"  [nbv] {m}"))

        def solve_pose(el, az, rolls):
            q = self._axis_view_q(look_target, el, az, standoff, q_cur, rolls=rolls)
            return q, None

        def swept(q0, q1):
            world = getattr(self, "_collision_world", None)
            if world is None:
                return True, ""
            col, why, _s = swept_pose_collision(
                world, q0, q1, T_EC=self._T_EC, n_steps=self.s.nbv_swept_steps)
            return (not col), (why or "")

        res = self._nbv.plan(gaps, q_cur, solve_pose, swept)
        return None if res is None else res[0]

    def _rank_nbv_candidates(self, cands, q_cur):
        """frontier 후보 → (cand, T_CB_des, q, cost) feasible만, cost 오름차순.
        cost = **관절이동 최소**(Σ w_i·Δq_i²) − δ·L̂ (docs §3.3). q 는 IK 결과 재사용."""
        from utils.nbv import phase2_nbv as _p2
        from utils.control.theta_planner import DEFAULT_JOINT_WEIGHTS
        if not cands:
            return []
        L_max = max(c.L for c in cands)
        out = []
        for c in cands:
            T_CB = _p2.nbv_pose_from_candidate(
                c, distance_m=self.s.nbv_distance_mm / 1000.0)
            q = self._nbv_feasible_q(T_CB, q_cur)        # 충돌-free 해석 IK
            if q is None:
                continue
            cost = _p2.joint_motion_cost(
                q, q_cur, DEFAULT_JOINT_WEIGHTS, c.L, L_max,
                delta=self.s.nbv_cost_delta)
            out.append((c, T_CB, q, cost))
        out.sort(key=lambda x: x[3])
        return out

    def _move_robot_to_q(self, q, speed_deg_s: float) -> int:
        """해석 IK q 로 관절구동 (IK 결정: set_position 대신 set_servo_angle)."""
        self.robot.enable_motion()
        code = self.robot.arm.set_servo_angle(
            angle=np.asarray(q, float).tolist(),
            speed=float(speed_deg_s), is_radian=True, wait=True)
        return int(code) if code is not None else 0

    def _capture_nbv_pose(self, T_CB_des, q):
        """로봇을 NBV pose 로 관절구동 후 streaming 캡처 → (sub_result, T_pre).

        v1: 기존 streaming(풀 회전) 후 camera-motion T_pre(case ③, R3)로 병합.
        master 의 self._T_BC 는 보존(덮어쓰지 않음)."""
        code = self._move_robot_to_q(q, self.s.nbv_robot_speed_deg_s)
        if code != 0:
            print(f"  [nbv] ⚠ 관절구동 실패 code={code}")
            return None, None
        T_EB_after = self.robot.get_ee_pose_mat()
        T_BC_nbv = self._T_EC @ np.linalg.inv(T_EB_after)
        T_pre = self._T_BC @ np.linalg.inv(T_BC_nbv)       # camera-motion (case ③)

        st = self.s.streaming_settings
        orig_reset = st.reset_to_zero_first
        st.reset_to_zero_first = False                     # 현재 위치 유지
        try:
            sub = ArtecStreamingScanSession(
                self.mms, self.robot, self.turntable, st).run()
        finally:
            st.reset_to_zero_first = orig_reset
        return sub, T_pre

    # ── Phase 2 (NBV hole-fill) 프리미티브 — 수렴 루프는 공용 컨트롤러 소유 ──
    #   (이전 _rank_nbv_candidates per-gap 정면 캡처 → 캡처통일로 대체, 2026-06-30.
    #    이전 _phase2_nbv_loop → 공용 _run_phase2_nbv 로 통합, 2026-07-01.)
    def _gap_kw(self) -> dict:
        s = self.s
        return dict(
            min_seg_vertices=s.nbv_min_seg_vertices,
            min_seg_length=s.nbv_min_seg_length_mm / 1000.0,
            max_seg_length=s.nbv_max_seg_length_mm / 1000.0)

    def build_coverage_mesh(self):
        # Phase 2 진입 첫 호출 시 swept-path 검사용 충돌 world 준비(1회).
        if getattr(self, "_collision_world", None) is None:
            print("\n═══════════════ Phase 2 — 부족면 NBV 보강 ═══════════════")
            self._collision_world = self._build_collision_world()
        mesh = self._build_master_mesh_B(self._st.master_model)
        if mesh is None or len(mesh.triangles) == 0:
            print("  [nbv] master mesh 비어있음 — 종료.")
            return None
        return mesh

    def is_converged(self, mesh) -> bool:
        from utils.nbv import phase2_nbv as _p2
        s = self.s
        cov = _p2.coverage_state(
            mesh, n_dirs=s.nbv_coverage_dirs,
            parallel_thresh_deg=s.nbv_coverage_parallel_deg, **self._gap_kw())
        print(f"  [nbv] boundary={cov.boundary_len_m*1000:.1f}mm "
              f"cov={cov.angular_cov:.3f} gaps={cov.n_gaps}")
        if _p2.is_converged(cov, s.nbv_boundary_stop_mm / 1000.0, s.nbv_coverage_tau):
            print("  [nbv] 커버리지 수렴 — 완료.")
            return True
        return False

    def plan_nbv_pose(self, mesh):
        # ★ 공용 자세선택(sim 과 동일): 부족면 덮을 관측 elevation 자세 → streaming 전회전.
        from utils.nbv import phase2_nbv as _p2
        cands = _p2.detect_gaps(mesh, **self._gap_kw())
        q_cur = np.asarray(self.robot.get_joint_angles(is_radian=True), float)
        q = self._plan_nbv_pose(q_cur, cands, mesh)
        if q is None:
            print("  [nbv] feasible 관측자세 없음 — 종료(윗면 도달한계 등).")
        return q

    @staticmethod
    def hint_icp_refine_static(
        sub_model, T_pre_init: np.ndarray, master_model,
        T_BC: np.ndarray, T_CB: np.ndarray,
        voxel_mm: float = 4.0, max_iter: int = 60,
        color_weight: float = 0.5, corr_dist_mm: float = 30.0,
    ) -> tuple:
        """centroid-pivot T_pre 를 init 으로 colored ICP → 측정된 T_pre.

        scan-world frame 에서 T_pre 가 동작 (IScan frame_transformations
        앞에 좌측 곱). 그러나 ICP 는 B 프레임 PCD 로 돌리는 게 자연 →
        결과를 W frame T_pre 로 역변환: T_pre_W = T_CB · T_pre_B · T_BC.
        translation 단위 — IScan frame_transformations 는 mm, T_BC/T_CB 는 m.

        Static method 라 후처리 단계(scripts/artec/merge_compare.py)에서도
        session instance 없이 호출 가능.

        Returns (T_pre_refined, fitness, rmse_mm). 실패 시 (init, 0.0, inf).
        """
        try:
            import open3d as o3d
        except Exception as e:
            print(f"  [icp_refine] open3d 없음 ({e}) — init 그대로")
            return T_pre_init, 0.0, float("inf")
        T_CB = np.asarray(T_CB, float)
        T_BC = np.asarray(T_BC, float)
        master_pcd = ArtecMultiPassScanSession._master_to_pcd_B(
            master_model, T_CB, voxel_mm)
        if master_pcd is None or len(master_pcd.points) < 100:
            print(f"  [icp_refine] master PCD 부족 — init 그대로")
            return T_pre_init, 0.0, float("inf")
        new_pcd = ArtecMultiPassScanSession._master_to_pcd_B(
            sub_model, T_CB, voxel_mm)
        if new_pcd is None or len(new_pcd.points) < 100:
            print(f"  [icp_refine] new IScan PCD 부족 — init 그대로")
            return T_pre_init, 0.0, float("inf")

        # init: T_pre_init (W frame, mm) → B frame, m → mm.
        # W ≈ C. T_pre_W transforms W coords.
        # y_B = T_CB·y_W = T_CB·T_pre_W·T_BC·x_B → T_pre_B = T_CB·T_pre_W·T_BC.
        T_pre_init_m = T_pre_init.copy()
        T_pre_init_m[:3, 3] = T_pre_init[:3, 3] / 1000.0
        init_B_m = T_CB @ T_pre_init_m @ T_BC
        init_B_mm = init_B_m.copy()
        init_B_mm[:3, 3] = init_B_m[:3, 3] * 1000.0

        radius_normal_mm = voxel_mm * 2.5
        master_pcd.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(
                radius=radius_normal_mm, max_nn=30))
        new_pcd.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(
                radius=radius_normal_mm, max_nn=30))
        try:
            criteria = o3d.pipelines.registration.ICPConvergenceCriteria(
                max_iteration=int(max_iter),
                relative_fitness=1e-6, relative_rmse=1e-6)
            result = o3d.pipelines.registration.registration_colored_icp(
                source=new_pcd, target=master_pcd,
                max_correspondence_distance=float(corr_dist_mm),
                init=init_B_mm,
                estimation_method=
                o3d.pipelines.registration.TransformationEstimationForColoredICP(
                    lambda_geometric=1.0 - float(color_weight)),
                criteria=criteria,
            )
        except Exception as e:
            print(f"  [icp_refine] colored ICP 예외 ({e}) — geom-only 재시도")
            try:
                result = o3d.pipelines.registration.registration_icp(
                    source=new_pcd, target=master_pcd,
                    max_correspondence_distance=float(corr_dist_mm),
                    init=init_B_mm,
                    estimation_method=
                    o3d.pipelines.registration.TransformationEstimationPointToPlane(),
                    criteria=criteria,
                )
            except Exception as e2:
                print(f"  [icp_refine] geom ICP 도 실패 ({e2}) — init 그대로")
                return T_pre_init, 0.0, float("inf")

        T_measured_B_mm = np.asarray(result.transformation, float).copy()
        fitness = float(result.fitness)
        rmse_mm = float(result.inlier_rmse)
        if fitness <= 0.0 or not np.all(np.isfinite(T_measured_B_mm)):
            print(f"  [icp_refine] fitness=0 — init 그대로")
            return T_pre_init, fitness, rmse_mm

        # B → W: T_pre_W = T_BC · T_pre_B · T_CB (init 식의 역)
        T_measured_B_m = T_measured_B_mm.copy()
        T_measured_B_m[:3, 3] = T_measured_B_mm[:3, 3] / 1000.0
        T_pre_refined_m = T_BC @ T_measured_B_m @ T_CB
        T_pre_refined = T_pre_refined_m.copy()
        T_pre_refined[:3, 3] = T_pre_refined_m[:3, 3] * 1000.0

        di = T_pre_init[:3, 3]
        dm = T_pre_refined[:3, 3]
        delta = np.linalg.norm(dm - di)
        print(f"  [icp_refine] fitness={fitness:.3f}  rmse={rmse_mm:.1f}mm  "
              f"Δtrans={delta:.1f}mm  "
              f"init=({di[0]:+.1f},{di[1]:+.1f},{di[2]:+.1f}) "
              f"→ refined=({dm[0]:+.1f},{dm[1]:+.1f},{dm[2]:+.1f})")
        return T_pre_refined, fitness, rmse_mm

    def _hint_icp_refine(self, sub_model, T_pre_init: np.ndarray,
                         master_model) -> tuple:
        """Instance wrapper — settings + self._T_BC/self._T_CB 를 static 함수에 위임."""
        s = self.s
        if self._T_BC is None or self._T_CB is None:
            print(f"  [icp_refine] T_BC/T_CB 미설정 — init 그대로")
            return T_pre_init, 0.0, float("inf")
        return ArtecMultiPassScanSession.hint_icp_refine_static(
            sub_model, T_pre_init, master_model,
            T_BC=self._T_BC, T_CB=self._T_CB,
            voxel_mm=s.icp_voxel_mm, max_iter=s.icp_max_iter,
            color_weight=s.icp_color_weight,
            corr_dist_mm=s.icp_corr_dist_mm,
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
