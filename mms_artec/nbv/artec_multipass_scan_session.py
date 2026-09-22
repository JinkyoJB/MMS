# mms_artec/nbv/artec_multipass_scan_session.py
#
# Artec(real) 스캔 backend — 공용 단계 컨트롤러의 ScanBackend 프리미티브 구현.
# ArtecStreamingScanSession 한 번 = 한 IScan = 한 회전 (= _do_one_rotation).
#
# ── 용어 (로그에서 셋이 같이 나와 헷갈리기 쉽다) ─────────────────────────────
#   pass  (`n_pass`)    회전 **1회 캡처** = IScan 1개. 재시도·복구도 pass 를 태운다.
#                       상한 = `max_passes`.
#   band                lookaround 플래너가 나눈 **높이 구간**. 밴드마다 로봇 관측
#                       자세가 다르고, 밴드 1개 = pass 1개(성공 시).
#   flip  (`pose_idx`)  **물체를 손으로 뒤집은 상태** (flip). 로봇 자세가 아니다.
#                       `pose_physical_rotations` 목록의 인덱스이고, 정상 완료
#                       시에만 advance. lookaround·nbv 에서는 0 으로 고정.
#
#   ★ 내부 변수명은 `pose_idx` 지만 **출력은 `flip`** 으로 통일했다(2026-09-16).
#     "pose" 가 로봇 자세처럼 읽혀서, `Pass 1 / max 8 (pose 0)` 이 로봇 얘기로
#     오해됐다. 공용 단계 컨트롤러도 같은 것을 `next_flip()` 이라 부른다.
# lookaround→nbv→flip 순서/게이팅/NBV 수렴 루프는 **공용 컨트롤러**
# (utils/nbv/scan_stage_controller.run_scan_stages) 소유 — sim IsaacScanSession
# 과 동일. 이 파일은 real Artec 캡처 하드웨어 프리미티브만 제공한다.
# (2026-07-01: 단일 pass 루프 → 단계 함수 분리 → 공용 컨트롤러 위임.
#  prompt 는 confirm_start(lookaround) / next_flip(flip)에만 — lookaround·nbv 무인.)
#
# 단계 정의 (★ 재정의: docs/4_nbv.md §8. 이전 docs/7 의 "nbv(flip 바닥면)" 는
#               본 설계에서 **flip** 로 분리되고, nbv 는 NBV 보강으로 재정의됨):
#  - lookaround: 물체를 높이 밴드로 썰어, 밴드마다 그 높이 자세로 이동 → turntable 360°
#             → 윗면 + 옆면 4 (= 5면). 밴드 전체가 한 IScan(z 단조 순서·겹침 보장).
#             한 자세가 물체를 다 덮으면 밴드 1개 = 옛 "로봇 고정" 동작.
#  - nbv: 5면 중 부족 영역을 **로봇이 최소이동 NBV 로 보강**(공용 _run_nbv +
#             utils/nbv/nbv_core + robot_collision). 로봇이 움직인다.
#  - flip: 물체를 **외부에서 flip**(Ry+90/180°) → 턴테이블만 회전 → **바닥면(윗면)**
#             + overlap 옆면 캡처 후 centroid-pivot pre-rotation hint 로 병합
#             (docs/8_artec_nbv_pose_disambiguation.md §5.3). = 이전의 "nbv(flip)".
#  ★ stage_until 로 **순차 누적** 실행 — 값은 단계 이름이다:
#    "preview"=계획만 / "lookaround" / "nbv" / "flip"(전체). 앞 단계는 항상 포함된다.
#    (nbv→flip 전환 시 NBV 로 움직인 robot 을 go_home 으로 복귀시킨 뒤 flip.)
#
# 동기:
#  - SDK 의 ScanningState_ContinueRecord 는 미지원이라 한 IScan 안에서 pause/resume 불가.
#  - 대신 multi-IScan 으로 분할 → master IModel 에 누적, hint 로 정합.
#  - 이는 Artec Studio 의 표준 multi-scan 워크플로우와 동일.
#
# 두 가지 흐름이 같은 메커니즘으로 처리됨:
#  - Tracking lost recovery: 같은 pose 로 retry, hint 동일.
#  - flip(flip) 의 자세 변경: 다음 pose advance, 새 hint.
#
# Notation: T_AB : A → B  (CLAUDE.md 준수).

from __future__ import annotations

import os
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
from utils.nbv import lookaround as _p1_const
from utils.nbv import nbv_planner as _nbvp   # sim·real 공용 수렴·회계·IK 예산 상수
from utils.nbv.nbv_debug_dump import NbvDebugDump as _NbvDebugDump   # 반복마다 겨냥·수집 기록
from utils.nbv.event_log import log_event as _ev      # run 이벤트(JSONL) — lost/병합 분석용
from utils.nbv.scan_stage_controller import (run_scan_stages, AT_CURRENT,
                                             runs_stage as _runs_stage)


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

from utils.nbv.standoff import (WORK_STANDOFF_M as _WORK_STANDOFF_M,
                               axis_standoff as _axis_standoff)
_WORK_STANDOFF_MM = _WORK_STANDOFF_M * 1000.0


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
    # True 면 모든 pass(lookaround + nbv) 동안 누적 컬러 포인트클라우드를
    # 실시간 표시. open3d 필요. 창을 닫아도 스캔은 계속됨. 절대 스캔
    # 성능/안정성에 영향 주지 않도록 모든 viewer 경로가 예외 격리됨.
    enable_live_viewer: bool = False

    # ── 물체-적응 자세 재선정 (recovery 전용, 2026-05-20 rule) ──────────
    # 첫 시작은 robot=home 그대로. tracking lost 시 recovery 흐름이
    # _adaptive_prescan_position(recovery=True) 를 호출 → fresh probe +
    # elevation search → 새 robot 자세. 상세: docs/artec_scanning_pipeline.md §6.
    # 아래 키들은 그 알고리즘의 공통 파라미터.
    #: 카메라↔**표면** 목표 거리 (mm). 기본값은 **공용 상수**에서 온다
    #  (`utils/nbv/standoff.py::WORK_STANDOFF_M`) — sim 과 갈라지지 않게.
    adaptive_target_standoff_mm: float = _WORK_STANDOFF_MM
    #: preview 원시 정점 문턱(크롭 전). 공용 기본값 = `lookaround.RAW_MIN_VERTS`.
    #  세 문턱(raw / 크롭후 / 최종)의 관계는 그 상수 옆에 적혀 있다.
    adaptive_min_preview_verts: int = _p1_const.RAW_MIN_VERTS
    #: preview **원시** 정점 문턱(크롭 전). 이보다 적으면 "스캐너가 아무것도 못 봤다".
    #  ★ 1500(`adaptive_min_preview_verts`)을 여기에 쓰면 **작은 윗부분을 버린다** —
    #    2026-09-22 run_132655: 상단 확인 높이(디스크 위 179mm)에서 350~381점이
    #    들어왔는데 버려져 병 높이가 122mm(실제 192)로 잡히고 뚜껑 밴드가 빠졌다.
    #    빈 시야의 노이즈는 실측 200~450점이라 원시 개수만으로는 못 가른다 — 여기는
    #    바닥값만 두고, 진짜 판정은 **크롭(축 r<0.16m·디스크 위) 후** 점수
    #    `lookaround.MIN_USEFUL_PTS`(200)가 한다. 노이즈는 크롭에서 대부분 빠진다.
    preview_raw_min_verts: int = 150
    #: 크롭 후 응집도 게이트 — 4mm 이웃 ≥8 인 점이 이보다 적거나 비율이 낮으면 노이즈.
    #  실측(2026-09-22 run_144803 preview): 진짜 패치 230~9,600점/46~94%, 노이즈 5~62점/2~15%.
    preview_min_coherent_pts: int = 150
    preview_min_coherent_frac: float = 0.30
    #: preview 캡처용 **재구성 민감도**(0~1). None 이면 SDK 기본(0.5) 유지.
    #
    #  ★ 스트리밍 세션엔 `sensitivity` 설정이 있었지만 **preview 경로엔 없었다.**
    #    기본 0.5 는 저대비/광택 표면에서 정점을 거의 못 만든다 — 2026-09-16 실측
    #    (흰 턴테이블 상판): 0.5 → 67~116개, 0.9 → 698개, 1.0 → 1739개.
    #    `adaptive_min_preview_verts`(1500)를 못 넘으면 preview 가 조용히 실패해
    #    플래너가 "점 부족" 으로 포기한다. 올릴수록 노이즈도 섞이므로 계획용으로만 쓴다.
    preview_sensitivity: Optional[float] = 0.9
    adaptive_robot_speed_deg_s: float = 15.0   # 2026-08-19: 15→30, 2026-09-21: 실물 안전상 15 로 복귀
    #: 밴드 사이 이동 속도. **녹화 중** 움직이므로 느려야 SLAM 이 따라온다.
    #  (밴드를 한 IScan 에 담는 구조 — 2026-09-16)
    band_move_speed_deg_s: float = 8.0

    # ── lookaround 자세 선정 (sim·real 공용 플래너) ───────────────────────
    # 2026-09-09: real 을 sim 과 같은 코드로 통일. 그전까지 real 의 시작 자세는
    # home 고정(2026-05-20 rule)이었고, sim 에만 전회전 maximin 채점기가 있었다 —
    # sim 에서 검증한 자세 선정이 실물에 전혀 적용되지 않는 상태였다.
    # 계획 = utils/nbv/lookaround.plan_lookaround_viewpoints (sim 과 동일 함수).
    # 실패(preview 부족·IK 없음·예외)하면 조용히 home 고정으로 되돌아간다.
    lookaround_planner_enabled: bool = True
    lookaround_els_deg: tuple = (30.0, 40.0, 50.0, 60.0, 70.0)  # sim MMS_SIM_P1_ELS 와 동일
    lookaround_view_azis_deg: tuple = (0.0, 30.0, -30.0)        # sim VIEW_AZIS_DEG 와 동일
    preview_el_deg: float = 30.0                     # 계획용 preview 고도각
    #: 계획용 preview 축거리 2스텝.
    #
    #  ★ 0.38m 은 **Spider 작동거리(170~350mm) 밖**이다. 2026-09-16 실물에서
    #    d=0.30 은 정점 13k~30k 가 나오는데 d=0.38 은 매번 300~1700개로
    #    `adaptive_min_preview_verts`(1500)를 못 넘겨 전부 버려졌다.
    #    Spider v1 최적 구간은 ~200~250mm 이므로 그 안에서 두 스텝을 잡는다.
    #: preview 고정 거리격자 (축거리 m). **None = 작동거리 창에서 유도**
    #  (`standoff.preview_grid`) — sim 과 같은 값이 나온다. 예전엔 여기에
    #  (0.24, 0.30) 이 박혀 있어 sim (0.30, 0.38) 과 갈라졌고, 반경 40mm 만
    #  넘으면 첫 스텝이 근접한계 안쪽이라 반환이 아예 없었다.
    #  특정 물체에만 다른 격자를 쓰고 싶을 때만 값을 준다.
    preview_dists_m: tuple = None
    preview_turntable_vel_rad_s: float = float(np.radians(30.0))
    lookaround_min_plan_points: int = _p1_const.MIN_PLAN_PTS    # 미만이면 플래너 포기

    # ── 고도각(elevation) 탐색 공통 파라미터 ───────────────────────────
    # recovery 호출 시의 elevation search 범위·기본 offsets. 재조준은
    # hardcoded +Z 가정의 look_at 이 아니라 probe preview 로 경험적 캘리브된
    # 광축(look_at_axes)을 써서 mis-aim bug 회피.
    # 상세: docs/artec_scanning_pipeline.md §3.0 / docs/artec_lookaround_view_score.md.
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
    #      평평한 물체 바닥 일부 잘림은 flip(바닥면 flip)가 별도 캡처해 보완.
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
    hint_icp_refine_mode: bool = False      # flip/recovery 패스의 hint 정밀정합 (기록만이 기본)
    #: pass 별 정리에 OutlierRemoval 까지 할지. 큰 IScan 에서 1~3분 — 기본 끔(SerialReg 만).
    #  후처리 단계의 OutliersRemoval(dev_mode 아닐 때)이 어차피 한 번 더 돈다.
    pass_outlier_removal: bool = False
    #: nbv 부분 스윕 IScan 을 master 에 붙일 때 공용 `refine_to_master` 로 다듬는다.
    #  기구학 T_pre 가 초기값, 게이트 기각이면 T_pre 그대로. False = 2026-09-18 이전 동작.
    nbv_icp_refine: bool = True
    icp_voxel_mm: float = 4.0           # downsample voxel (속도 ↔ 정확)
    icp_max_iter: int = 60
    icp_color_weight: float = 0.5       # colored ICP 의 색상 가중 (Open3D 0.6)
    icp_corr_dist_mm: float = 30.0      # 대응 거리 한계 (init 이 좋으면 작게)

    # ── nbv — 부족면 NBV 보강 (docs/4_nbv.md) ────────────────────
    # stage_until: lookaround 부터 **순차 누적**으로 어디까지 실행할지 (docs/4_nbv.md §8).
    # stage_until=N → lookaround..N 을 순서대로:
    #   "preview"    거리탐색·실루엣만 (계획까지, 캡처 없음)
    #   "lookaround" preview → lookaround (5면 streaming)
    #   "nbv"        → **nbv** (부족면 NBV 보강, 로봇 이동, _nbv_loop)
    #   "flip"       → **flip** (바닥면 180° flip, 사용자 손회전 + centroid hint)
    # 기본 "flip" = 전 파이프라인. ★ 첫 real 테스트는 "lookaround" 부터 올릴 것.
    stage_until: str = "flip"
    #: NBV 카메라↔**표면** 거리 (mm). 축 기준이 필요한 곳은 `axis_standoff(r)` 로
    #  변환해서 쓴다 — 예전엔 같은 값이 축/표면 두 뜻으로 섞여 쓰였다.
    nbv_distance_mm: float = _WORK_STANDOFF_MM
    nbv_min_seg_vertices: int = 8           # frontier 세그먼트 최소 정점
    nbv_min_seg_length_mm: float = 6.0      # frontier 최소 길이
    nbv_max_seg_length_mm: float = 60.0     # frontier 재분할 한계
    # 루프용 메시는 **저비용**으로 — 여기서 필요한 건 '어디가 비었나' 판단이지
    # 최종 품질이 아니다(최종 메시는 ArtecMMS.artec_process 가 SDK 로 별도 생성).
    # Poisson 비용은 depth 에 급격히 증가 → sim(isaac_scan_session, depth 6)과 통일.
    nbv_poisson_depth: int = 6              # master pcd→mesh Poisson depth (루프 전용)
    nbv_density_quantile: float = 0.04      # 저밀도 vertex trim 분위수
    nbv_master_voxel_mm: float = 2.0        # master→pcd voxel
    nbv_K_max: int = _nbvp.K_MAX            # NBV 최대 반복 (sim 과 같은 공용 상한)
    nbv_boundary_stop_mm: float = _nbvp.CONV_BOUNDARY_M * 1000.0   # 수렴: 경계 총길이 < 이 값 (공용)
    nbv_coverage_tau: float = _nbvp.CONV_COVERAGE_TAU              # 수렴: 각도 커버리지 ≥ τ (공용)
    nbv_coverage_dirs: int = 64
    nbv_coverage_parallel_deg: float = 40.0
    #: NBV 로봇 이동 속도. 2026-08-19 에 12→24 로 올렸다가 2026-09-21 실물에서
    #  "예상치 못한 움직임에 대응할 수 없다" 로 10 으로 내림. `--speed-scale` 로 배율.
    nbv_robot_speed_deg_s: float = 5.0        # 2026-09-22: 10 → 5 (여전히 빠르다는 현장 판단)
    nbv_sweep_deg: float = 15.0             # 캡처 시 턴테이블 ±스윕 (overlap)
    nbv_theta_assist: bool = False          # True=턴테이블 회전 보조(θ planner). 기본 robot-only.
    nbv_theta_n_samples: int = 72           # θ assist 그리드
    nbv_cost_delta: float = 0.3             # 큰 구멍 우선 가중
    nbv_swept_steps: int = 12               # 궤적(q_cur→q_des) 충돌검사 보간 스텝 수
    nbv_el_floor_deg: float = 30.0          # NBV 관측 elevation 하한(=lookaround 측면각). 그 이상에서 보강
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
    # gap 직접 겨냥(plan_frontier) — 로봇이 편한 방위. 턴테이블이 gap 을 여기로 가져온다.
    # 넓히면 겨냥 성공률은 오르지만 팔이 물체를 감싸 충돌 위험이 커진다(docs/collision §5).
    nbv_frontier_az_pref_deg: tuple = (0.0, 30.0, -30.0)
    # ★ gap 겨냥 캡처의 부분 스윕 폭 (deg) — sim `NBV_PATCH_SPAN_DEG` 와 동일.
    #   gap 하나 채우려고 360° 를 돌면 이미 가진 면만 다시 본다(낭비). 목표 θ 를
    #   중심으로 이 폭만 돈다. ⚠ sim 실측: 60° 이하는 프레임 중첩 부족으로 정합
    #   붕괴 (boundary 963→13,927mm). 보장 고도각 패스는 전회전 유지.
    nbv_patch_span_deg: float = 90.0
    #: nbv 패치 병합 게이트 — 4mm 복셀 점수·OK 프레임이 이보다 적으면 빈 캡처로 보고 버린다.
    #  실측(2026-09-22): 정상 패치 6만~8만점/30~100프레임, 빈 캡처 5~8천점.
    nbv_min_patch_pts: int = 5000
    nbv_min_patch_frames: int = 10
    #: nbv 캡처 방식. "step" = 정지-촬영(턴테이블 K단계, 프레임마다 θ 로 배치, SDK 추적
    #  없음 — 기본, 2026-09-22). "sweep" = 예전 스트리밍 부분 스윕(SLAM 의존).
    nbv_capture_mode: str = "step"
    nbv_step_deg: float = 30.0            # 정지-촬영 θ 간격 (90° 스윕 → 4프레임, 전회전 → 12)
    #: nbv 부분 스윕의 턴테이블 회전 속도 (초/1회전). lookaround 는 30s/rev 인데 nbv
    #  스윕은 90° 라 30s 기준이면 7.5s+오버슛 ≈ 9s 가 매 반복에 든다(2026-09-22 실측).
    #  sim 도 같은 ±45° 스윕이지만 순간이라 "sim 은 한 장, real 은 한 바퀴" 로 보였다.
    #  스윕 자체는 필수가 아니지만(단일 프레임도 가능) SDK 가 프레임끼리 정합해 주는
    #  일관된 패치를 얻는 대가로 둔다. 15s/rev 면 90° 에 3.75s.
    nbv_rotation_duration_s: float = 15.0
    nbv_frontier_enabled: bool = True        # False = 축-고도각만 (2026-09-10 이전 동작)
    # gap 과 무관하게 최소 한 번은 시도할 관측 고도각. 오목 물체 내부는 미관측이라
    # 메시에 없고 → 경계(gap)로도 안 잡혀 el_need 가 올라갈 근거가 없다(닭·달걀).
    # 실측(sim): el 55° 한 자세 + 전회전으로 컵 내벽·내부바닥 100% 커버.
    nbv_ensure_els_deg: tuple = (55.0,)
    nbv_view_azis_deg: tuple = (0.0, 30.0, -30.0)   # 축-고도각 방위 후보

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
    #: preview 단계 산출 — 점군(B)·턴테이블 축·밴드 플랜. `--until preview` 는
    #  캡처를 안 해 `model` 이 비므로, 결과를 눈으로 보려면 이쪽이 필요하다.
    preview_result: Optional[dict] = None


# ─────────────────────────────────────────────────────────────────────────────
# run() 내부 상태 — 회전 루프가 이어서 나르는 loop-carried 가변 변수 묶음.
# _do_one_rotation 이 mutate 하고, 공용 컨트롤러가 호출하는 프리미티브
# (confirm_start / capture_rotation / next_flip / finalize) 들이 self._st 로 공유한다.
# (회전 단위 상태를 한 곳에 모아 프리미티브 시그니처를 얇게 유지)
# ─────────────────────────────────────────────────────────────────────────────

# _do_one_rotation 반환 코드
_ROT_OK = "ok"          # 정상 완료 — 다음 단계 / pose 로
_ROT_RETRY = "retry"    # tracking-lost — 같은 pose 재시도 (recovery / 수동)
_ROT_ABORT = "abort"    # 중단 (user_quit / 치명적 drive alarm)


@dataclass
class _RunState:
    """run() 회전 루프의 loop-carried 상태 (2026-07-01 단계 분리 리팩터)."""
    master_model: "artec_base.ModelHandle"
    live_viewer: object = None
    pass_results: List[ArtecStreamingScanResult] = field(default_factory=list)
    n_pass: int = 0
    pose_idx: int = 0                    # 현재 pose hint 인덱스. 정상 완료 시만 advance.
    master_center: Optional[np.ndarray] = None   # Pass 1 후 lock
    #: master IModel 의 좌표계 = **첫 회전 녹화 시작 시점의 카메라 프레임**.
    #  그 시점의 C→B 를 고정해 둔다. `self._T_CB` 는 로봇이 움직일 때마다
    #  갱신되므로(밴드 이동·NBV 자세) 나중에 쓰면 마스터를 엉뚱한 곳에 놓는다.
    master_T_CB: Optional[np.ndarray] = None
    hints_applied: bool = False
    last_n_frames: int = 0               # 직전 회전 프레임 수 (완료 로그용)
    recovery_retry_count: int = 0        # 같은 pose 안에서 누적 (성공 시 0)
    n_recovery_attempts: int = 0
    n_recovery_succeeded: int = 0
    next_T_BC_pending: Optional[np.ndarray] = None
    last_scan_theta0: float = 0.0        # 직전 스캔 시작 시점 turntable 논리각 (rad)
    #: 밴드 회계 — 콘솔은 'pass' 가 아니라 **밴드**로 말한다. pass 는 내부
    #  재시도 카운터일 뿐이라 사용자에게 의미가 없다 (2026-09-21 요청).
    bands_done: int = 0                  # 전회전을 끝낸 밴드 (누적)
    bands_total: int = 0                 # 계획된 밴드 (누적)
    merge_method: str = "none"           # 직전 병합이 쓴 정렬: none|hint|camera|greg|icp
    r_eff_hist: list = field(default_factory=list)   # lookaround 거리추종 실측 반경(d−p, m)
    #: lookaround 밴드 관절해 목록. 설정되면 **한 IScan 안에서** 밴드마다 전회전한다.
    #  첫 밴드는 호출자가 미리 이동시켜 두고, 2번째부터 세션이 콜백으로 옮긴다.
    band_poses: Optional[list] = None
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
    Artec(real) 스캔 backend — 공용 단계 컨트롤러의 ScanBackend 프리미티브 구현.

    lookaround→nbv→flip 순서·게이팅·NBV 수렴 루프는 공용
    utils/nbv/scan_stage_controller.run_scan_stages 가 소유(sim IsaacScanSession
    과 동일 컨트롤러). 이 클래스는 real Artec 캡처 하드웨어만 제공:
      - confirm_start / pick_lookaround_pose(=AT_CURRENT) : lookaround 시작(사람 확인 후 home 고정)
      - capture_rotation : 턴테이블 전회전 캡처. AT_CURRENT=현재포즈 streaming
        (+cleanup/hint/master 병합/tracking-lost recovery = _do_one_rotation 재시도),
        q 지정=NBV 자세로 구동 후 streaming(_capture_nbv_pose).
      - build_coverage_mesh / is_converged / plan_nbv_pose : nbv 보강.
      - supports_flip=True / next_flip : flip (사람이 물체 flip → 윗면).
      - finalize : master IModel(N IScans) → ArtecMultiPassScanResult.

    prompt(사람 개입)는 confirm_start(lookaround) + next_flip(flip)에만. lookaround·nbv
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

    # ── 물체-적응 사전 포지셔닝 (lookaround, run() 시작 1회) ──────────────

    def _apply_preview_sensitivity(self, sensor) -> None:
        """preview 재구성 민감도를 1회 적용 (설정 시). 실패는 경고만."""
        if getattr(self, "_preview_sens_done", False):
            return
        self._preview_sens_done = True
        v = getattr(self.s, "preview_sensitivity", None)
        if v is None:
            return
        proc = getattr(sensor, "_processor", None)
        if proc is None or not hasattr(proc, "set_sensitivity"):
            print("  [p1plan] ⚠ preview 민감도 설정 불가 (processor 없음)")
            return
        try:
            old = proc.sensitivity()
            proc.set_sensitivity(float(v))
            print(f"  [p1plan] preview 민감도 {old:.2f} → {float(v):.2f}")
        except Exception as e:                                  # noqa: BLE001
            print(f"  [p1plan] ⚠ preview 민감도 설정 실패: {e}")

    def _capture_preview_verts(self, sensor, min_verts: int,
                               to_color: bool = False):
        """PREVIEW 최대 3회, 정점 최다 프레임 채택. mm (N,3) or None.

        ★ `to_color=True` 면 **스캐너 3D → Color 카메라 프레임**으로 옮긴다.
          `T_EC`(hand-eye)가 solvePnP 로 풀려 Color 기준이므로, `T_CB` 로 base 로
          옮길 점은 반드시 Color 프레임이어야 한다.

          이 변환이 빠져 있어서 preview 점이 턴테이블 축에서 **0.44m** 떨어진
          곳에 찍혔고, 기하 크롭(r<0.16m)이 정점 3만개를 **전부 버렸다**
          (2026-09-16 실측: 변환 전 크롭통과 0 / 변환 후 29,608 전부 통과.
           변환 후 r중앙 0.042m · 디스크 위 0.054m = 원판 중앙의 물체).
          자세한 근거는 `mms_artec/utils/calibration/scanner_frames.py`.
        """
        self._apply_preview_sensitivity(sensor)
        v_mm = None
        best_uv = None
        best_wh = None
        # ★ 텍스처 캡처는 스캐너3D→Color 변환(`_T_scan_color`)을 **한 번** 풀 때만 필요하다.
        #   변환이 캐시된 뒤에도 매번 텍스처를 찍고 있었고, 빈 프레임(점 0)이면
        #   capture+reconstruct 실패에 ~3s 가 걸려 3회 재시도 = 탐침 하나에 9s
        #   (2026-09-22 run_131509: 성공 캡처는 0.2s). 빈 프레임이 두 번 연속이면
        #   시야에 물체가 없는 것이라 더 찍어도 같다 — 거기서 멈춘다.
        need_tex = bool(to_color) and getattr(self, "_T_scan_color", None) is None
        n_empty = 0
        t_tries = []
        for _ in range(3):
            _t0 = time.perf_counter()
            try:
                fmh = (sensor.capture_frame(capture_texture=True) if need_tex
                       else sensor.capture_frame())
                vv = fmh.vertices() if fmh is not None else None
            except Exception:
                fmh, vv = None, None
            t_tries.append(time.perf_counter() - _t0)
            if vv is None or vv.shape[0] == 0:
                n_empty += 1
                if n_empty >= 2:
                    break
            if vv is not None and vv.shape[0] > 0:
                if v_mm is None or vv.shape[0] > v_mm.shape[0]:
                    v_mm = vv
                    if to_color:
                        try:
                            best_uv = fmh.uv()
                            img = fmh.image()
                            best_wh = (img.shape[1], img.shape[0])
                            # ★ 디버그 스냅샷용으로 보관 — "스캐너가 그 순간 뭘
                            #   봤나" 는 실물에서 가장 빠른 단서다. sim 은 예전부터
                            #   남기는데 real 만 없었다(2026-09-17까지).
                            self._last_preview_img = img
                        except Exception:                       # noqa: BLE001
                            best_uv = best_wh = None
            if v_mm is not None and v_mm.shape[0] >= min_verts:
                break
        if v_mm is None or v_mm.shape[0] < min_verts:
            print(f"  [p1plan]   preview 캡처 {len(t_tries)}회 — 점 "
                  f"{0 if v_mm is None else v_mm.shape[0]} (<{min_verts}), "
                  + "·".join(f"{t:.1f}s" for t in t_tries))
            return None
        v = v_mm.astype(np.float64)
        if not to_color:
            return v
        if not need_tex:                          # 변환 캐시 있음 — 바로 적용
            from mms_artec.utils.calibration import scanner_frames as _sf
            return _sf.apply(self._T_scan_color, v)

        T = self._scanner_to_color(v, best_uv, best_wh)
        if T is None:
            print("  [p1plan] ⚠ 스캐너3D→Color 변환 실패 — 좌표가 0.4m 급으로 "
                  "어긋나 크롭이 전부 버릴 수 있다")
            return v
        from mms_artec.utils.calibration import scanner_frames as _sf
        return _sf.apply(T, v)

    def _scanner_to_color(self, verts_mm, uv, image_wh):
        """스캐너3D→Color 변환. **고정 외부파라미터라 1회만 풀고 캐시**한다."""
        cached = getattr(self, "_T_scan_color", None)
        if cached is not None:
            return cached
        if uv is None or image_wh is None:
            return None
        intr = getattr(self.mms, "intrinsic", None) or self._load_color_intrinsic()
        if intr is None:
            return None
        from mms_artec.utils.calibration import scanner_frames as _sf
        T = _sf.solve_scanner_to_color(
            verts_mm, uv, image_wh, intr["K"], intr["dist"],
            log=lambda m: print(f"  [frame] {m}"))
        if T is not None:
            self._T_scan_color = T
        return T

    @staticmethod
    def _load_color_intrinsic():
        """`config/calibration/artec_intrinsic.yaml` 로드 (없으면 None)."""
        import yaml
        from pathlib import Path as _Path
        p = (_Path(__file__).resolve().parents[2]
             / "config" / "calibration" / "artec_intrinsic.yaml")
        if not p.exists():
            print(f"  [frame] ⚠ {p.name} 없음 — 스캐너3D→Color 변환 불가")
            return None
        d = yaml.safe_load(p.read_text(encoding="utf-8")) or {}
        return {"K": np.asarray(d["K"], float),
                "dist": np.asarray(d.get("dist", [0] * 5), float).reshape(-1)}

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
               차단. 평평한 물체 바닥 일부는 비용으로 수용 (flip 바닥면 flip 이 따로).
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

        band_hi = float(s.elevation_optimal_band_mm[1]) / 1000.0

        d = max(s.adaptive_target_standoff_mm / 1000.0, near + r_obj)
        if not _fits(d):
            d_need = (0.5 * h_obj + margin) / max(tan_hv, 1e-6)
            if d_need > far:
                # ★ far 까지 물러나도 **어차피 안 들어온다** → 물러날 이유가 없다.
                #   recovery 의 목적은 물체 전체를 한 화면에 담는 게 아니라
                #   **추적을 다시 붙이는** 것이다. 게다가 밴드 스캔에서는 애초에
                #   한 자세가 물체 전체를 덮지 않는 것이 정상이다.
                #   far(=350mm) 는 Spider 의 절대 한계라 거기선 데이터가 거의
                #   안 나온다 — 실측 2026-09-16 (h_obj 215mm): 이 분기가 d 를
                #   350mm 로 밀어붙여 elevation 후보마다 preview 206~243 verts,
                #   결국 "[elev] 유효 후보 없음 — home 복귀" 로 recovery 0/1 실패.
                #   최적대역 상단(250mm)에 머무는 쪽이 재획득 확률이 높다.
                d = min(max(d, min(band_hi, far)), far)
            else:
                d = min(max(d, d_need), far)
        d = float(min(max(d, near + r_obj), far))
        return d, (not _fits(d))

    def _move_robot_to_T_CB(self, T_CB_target: np.ndarray,
                            speed_deg_s: float) -> int:
        """T_CB → T_EB → 해석 IK → **게이트 경유** 관절 이동. 반환 0=OK, 그 외 실패.

        ★ 예전엔 `set_position`(직선 이동)으로 충돌 게이트를 **건너뛰었다** — preview
          탐침·recovery 자세 이동이 물체·턴테이블 옆을 무검사로 지나갔다(2026-09-22
          flip go_home 사고와 같은 부류). 이제 세션의 모든 이동은 `_move_robot_to_q`
          하나로 모인다(게이트 + 동적 장애물 + 우회 계획 + 속도 단위).
        """
        from utils.robot import xarm7_kinematics as _kin
        T_EB_target = T_CB_target @ self._T_EC
        pose6d = np.concatenate([T_EB_target[:3, 3] * 1000.0,
                                 _kin.R_to_euler_xyz(T_EB_target[:3, :3])])
        try:
            seed = np.asarray(self.robot.get_joint_angles(is_radian=True), float)
        except Exception:                                       # noqa: BLE001
            seed = None
        q, ok = _kin.ik(pose6d, seed=seed)
        if not ok:
            print("  [move] ✘ 해석 IK 실패 — 이동 안 함")
            return -1
        return int(self._move_robot_to_q(q, speed_deg_s))

    def _read_turntable_theta(self) -> float:
        """현재 turntable 논리각 (rad). 실패 시 0.0."""
        try:
            v = self.turntable.getActualPos()
            if not isinstance(v, bool) and v is not None:
                return float(v)
        except Exception:                                   # noqa: BLE001
            pass
        return 0.0

    def _R_obj_B(self, theta0: float, tag: str = "merge") -> np.ndarray:
        """스캔 시작 시 turntable 논리각 θ0 만큼 돌아간 물체를 논리 0° 로 되돌리는
        base 프레임 4×4 (축 통과점 기준 회전). θ0≈0 이면 항등."""
        R_obj = np.eye(4)
        tt = getattr(self.mms, "turntable_transform", None)
        if abs(float(theta0)) > 1e-6 and tt is not None:
            p_ax = np.asarray(tt.axis_point_B, float)
            R3 = self._rot_about_axis(np.asarray(tt.axis_dir_B, float),
                                      -float(theta0))
            R_obj[:3, :3] = R3
            R_obj[:3, 3] = p_ax - R3 @ p_ax
            print(f"  [{tag}] 병합 보정: 물체 회전 θ0={np.degrees(theta0):.1f}° 포함")
        return R_obj

    def _S_m(self):
        """스캐너3D→Color 변환 S (m). 미캐시면 None."""
        S_mm = getattr(self, "_T_scan_color", None)
        if S_mm is None:
            return None
        S_m = np.asarray(S_mm, float).copy()
        S_m[:3, 3] /= 1000.0
        return S_m

    def _flip_unflip_B(self, R_phys: np.ndarray, T_BC_new: np.ndarray,
                       theta0: float, c_pass_W_mm: np.ndarray,
                       c_master_W_mm: np.ndarray) -> np.ndarray:
        """사람이 뒤집은 물체를 **base 프레임에서** 원래 자세로 되돌리는 4×4 (m).

        회전은 `R_phys⁻¹`(설정된 뒤집기 각의 역), 평행이동은 **무게중심 피벗** —
        이번 스캔의 무게중심(B)이 master 무게중심(B)으로 가도록 잡는다. 사람이
        뒤집으며 물체를 옮겨 놓는 것(수 cm)은 회전 힌트만으로는 못 잡기 때문이다
        (예전 scan-world 판 `Translate(c_master)·R⁻¹·Translate(−c_pass)` 와 같은
        발상을 B 로 옮긴 것 — 밴드 캡처의 카메라 이동 보정과 **합성**하려면 둘 다
        B 에서 표현돼야 한다).
        """
        S_m = self._S_m()
        if S_m is None:
            S_m = np.eye(4)
        T_CB_master = (self._st.master_T_CB if self._st.master_T_CB is not None
                       else np.linalg.inv(self._T_BC))
        A_new = self._R_obj_B(theta0, tag="flip") @ np.linalg.inv(T_BC_new) @ S_m
        c_pass_B = (A_new @ np.r_[np.asarray(c_pass_W_mm, float) / 1000.0, 1.0])[:3]
        c_master_B = (T_CB_master @ S_m
                      @ np.r_[np.asarray(c_master_W_mm, float) / 1000.0, 1.0])[:3]
        R_inv = np.linalg.inv(np.asarray(R_phys, float))[:3, :3]
        T = np.eye(4)
        T[:3, :3] = R_inv
        T[:3, 3] = c_master_B - R_inv @ c_pass_B
        return T

    def _flip_global_refine(self, sub_model, T_BC_new, theta0: float,
                            T_hint_B: np.ndarray):
        """flip 스캔을 master 에 맞추는 변환을 **힌트 대신** 구한다.
        반환 (frame, T, method):
            ("W", T_pre_mm, "img")   텍스처 특징점 매칭 — 스캔월드→master월드 직접
            ("B", T_extra_B, "greg") FGR·FPFH 합의 (base 프레임, 카메라 이동과 합성)
            ("B", 힌트, "hint(...)") 둘 다 실패

        순서가 왜 이런가 — 회전대칭 물체는 기하 정합이 높은 fitness 로 틀린 답을 내고,
        텍스처(라벨)만이 앞뒤·뒤집힘을 구분한다(벤치마크 2026-06-11: image_match 가
        수평축 flip 을 유일하게 찾음). 힌트(설정 각 + 무게중심 피벗)는 사람 손회전
        오차와 부분 스캔의 무게중심 가정 때문에 틀릴 수 있고, 틀리면 후처리 GlobalReg
        도 건너뛰어(hints_applied) 굳는다 — 2026-09-21 run_162322 정합 실패.
        """
        # ── 1순위: 텍스처 특징점 (프레임 원본 사진 — Texturize 불필요) ────────
        try:
            from utils.nbv import image_match as _im
            T_m, n_inl, info = _im.register_models(
                sub_model, self._st.master_model,
                log=lambda m: print(f"  [flip] {m}"))
            if T_m is not None:
                T_mm = np.asarray(T_m, float).copy()
                T_mm[:3, 3] *= 1000.0
                print(f"  [flip] ✓ 텍스처 정합 채택 (inlier {n_inl})")
                return "W", T_mm, "img"
            print(f"  [flip] 텍스처 정합 불가({info.get('reason', '?')}) — 기하 전역 정합으로")
        except Exception as e:                                   # noqa: BLE001
            print(f"  [flip] ⚠ 텍스처 정합 예외({type(e).__name__}: {e}) — 기하 전역 정합으로")
        # ── 2순위: 기하 전역 정합 합의 ─────────────────────────────────────
        try:
            from utils.nbv.global_registration import register_consensus
            T_CB_new = np.linalg.inv(np.asarray(T_BC_new, float))
            sub_pcd = self._master_to_pcd_B(
                sub_model, T_CB_new, voxel_mm=self.s.nbv_master_voxel_mm,
                T_sc_mm=getattr(self, "_T_scan_color", None))
            mp = self._master_pts_B_m(80_000)
            if sub_pcd is None or len(sub_pcd.points) < 500 or len(mp) < 500:
                print("  [flip] 전역 정합 — 점 부족, 힌트 사용")
                return "B", T_hint_B, "hint(점 부족)"
            S = np.asarray(sub_pcd.points, float) / 1000.0
            R_obj = self._R_obj_B(theta0, tag="flip-greg")
            S = S @ R_obj[:3, :3].T + R_obj[:3, 3]          # 논리 0° 기준으로
            T, info = register_consensus(S, mp, log=lambda m: print(f"  [flip] {m}"))
            if T is None:
                print(f"  [flip] 전역 정합 불합의({info.get('reason', '?')}) — 힌트 사용")
                return "B", T_hint_B, "hint(greg 불합의)"
            T = np.asarray(T, float)
            c = np.r_[S.mean(0), 1.0]
            d_c = float(np.linalg.norm((T @ c)[:3] - (np.asarray(T_hint_B) @ c)[:3])) * 1000.0
            dR, _ = self._axis_angle(T[:3, :3].T @ np.asarray(T_hint_B)[:3, :3])
            print(f"  [flip] ✓ 기하 전역 정합 채택 (ΔR 힌트대비 {dR:.1f}°, 무게중심 이동차 {d_c:.0f}mm)")
            return "B", T, "greg"
        except Exception as e:                                   # noqa: BLE001
            print(f"  [flip] ⚠ 전역 정합 예외({type(e).__name__}: {e}) — 힌트 사용")
            return "B", T_hint_B, "hint(예외)"

    @staticmethod
    def _trim_lost_tail(model, n_tail: int):
        """IScan 꼬리의 미정합 프레임 `n_tail` 개를 잘라낸 **새 모델**을 돌려준다.
        SDK 에 프레임 삭제가 없어 scan 을 다시 만든다(프레임·변환 복사). 실패면 원본."""
        if n_tail <= 0 or model is None:
            return model
        try:
            out = artec_base.create_model()
            n_cut = 0
            for si in range(model.scan_count()):
                scan = model.get_scan(si)
                n = scan.frame_count()
                keep = max(0, n - n_tail) if si == model.scan_count() - 1 else n
                if keep < 10:                       # 남는 게 없으면 자르지 않는다
                    out.add_scan(scan); continue
                ns = artec_base.create_scan()
                for i in range(keep):
                    ns.add_frame(scan.get_frame(i))
                    ns.set_frame_transformation(i, scan.get_frame_transformation(i))
                out.add_scan(ns); n_cut += n - keep
            if n_cut:
                print(f"  [정리] lost 꼬리 {n_cut} 프레임 제거")
            return out
        except Exception as e:                                   # noqa: BLE001
            print(f"  [정리] ⚠ 꼬리 제거 실패({type(e).__name__}: {e}) — 원본 사용")
            return model

    @staticmethod
    def _cleanup_model(model, tag: str = "정리", outliers: bool = False):
        """SerialReg(+선택 OutlierRemoval). 실패하면 원본 그대로(파이프라인을 깨지 않는다).

        소요(실측 2026-09-21): nbv 패치(60~100 프레임) 0.7~1.2s, 전회전 IScan(700~1300
        프레임) 62~160s — 큰 쪽은 OutlierRemoval(프레임별 이웃 탐색)이 지배한다. 그래서
        pass 마다 하는 정리는 **SerialReg 만** 기본이고, Outlier 는 설정으로 켠다.
        """
        try:
            cleaned = artec_algorithm.Algorithms.serial_registration(model)
            if outliers:
                cleaned = artec_algorithm.Algorithms.outliers_removal(cleaned)
            print(f"\n  [{tag}] SerialReg{' + Outlier' if outliers else ''} 완료")
            return cleaned
        except Exception as e:                                   # noqa: BLE001
            print(f"\n  [{tag}] ⚠ 실패({type(e).__name__}: {e}) — raw IScan 사용")
            return model

    def _dump_scan_raw(self, model, T_pre_mm, stage: str, **extra) -> None:
        """원시 IScan 점(스캔 월드, mm, 색) + 적용 T_pre 를 npz 로 남긴다 —
        `output/scan_dumps/<RUN_TS>/scanNN_<stage>_poseK.npz`. 정합을 SDK 없이 오프라인에서
        다시 돌리기 위한 것. master 월드 = scan00 (T_pre 항등). 실패는 무시."""
        try:
            from pathlib import Path as _P
            tag = os.environ.get("MMS_RUN_TS") or time.strftime("%Y%m%d_%H%M%S")
            # output/scans/ 는 Artec 프로젝트 페이로드 폴더 이름과 겹친다 → scan_dumps/
            d = _P(__file__).resolve().parents[2] / "output" / "scan_dumps" / tag
            d.mkdir(parents=True, exist_ok=True)
            pts, cols = [], []
            for si in range(model.scan_count()):
                scan = model.get_scan(si)
                n = scan.frame_count()
                for i in range(0, n, max(1, n // 60)):
                    fr = scan.get_frame(i)
                    v = fr.vertices()
                    if v is None or len(v) == 0:
                        continue
                    T = np.asarray(scan.get_frame_transformation(i), float)
                    pts.append(np.asarray(v, float) @ T[:3, :3].T + T[:3, 3])
                    c = self._frame_vertex_colors(fr)
                    cols.append(np.asarray(c) if c is not None and len(c) == len(v)
                                else np.zeros((len(v), 3), np.uint8))
            if not pts:
                return
            P = np.vstack(pts).astype(np.float32)
            C = np.vstack(cols)
            if len(P) > 400_000:
                idx = np.random.default_rng(0).choice(len(P), 400_000, replace=False)
                P, C = P[idx], C[idx]
            st = self._st
            f = d / f"scan{st.master_model.scan_count():02d}_{stage}_pose{st.pose_idx}.npz"
            # ★ `T_BC_used` — 병합이 **실제로** T_pre 에 쓴 T_BC. dump 는 병합 뒤에
            #   돌기 때문에 `next_T_BC_pending` 은 이미 None 이고, 그때 self._T_BC 로
            #   폴백하면 preview 탐침 자세의 낡은 값이 저장된다(오프라인 재현 불가).
            _T_BC_used = extra.pop("T_BC_used", None)
            meta = {k: np.asarray(v) for k, v in extra.items() if v is not None}
            np.savez_compressed(
                f, pts_W_mm=P, colors=C,
                T_pre_mm=np.asarray(T_pre_mm if T_pre_mm is not None else np.eye(4), float),
                stage=stage, pose_idx=int(st.pose_idx),
                master_T_CB=np.asarray(st.master_T_CB if st.master_T_CB is not None else np.eye(4), float),
                T_BC_new=np.asarray(
                    _T_BC_used if _T_BC_used is not None
                    else (st.next_T_BC_pending if st.next_T_BC_pending is not None
                          else (self._T_BC if self._T_BC is not None else np.eye(4))), float),
                T_BC_is_used=bool(_T_BC_used is not None),
                T_scan_color_mm=np.asarray(getattr(self, "_T_scan_color", None)
                                           if getattr(self, "_T_scan_color", None) is not None
                                           else np.eye(4), float),
                **meta)
            print(f"  [scan-dump] {f.relative_to(d.parents[2])} ({len(P):,}pt)")
        except Exception as e:                                   # noqa: BLE001
            print(f"  [scan-dump] ⚠ 실패({type(e).__name__}: {e})")

    def _compose_T_pre_W_mm(self, T_BC_new: np.ndarray, theta0: float,
                            tag: str = "merge", T_extra_B=None) -> np.ndarray:
        """sub scan-world → master scan-world 변환 (SDK mm 단위).

            T_pre = S⁻¹ · T_BC_master · [T_extra_B] · R_B(axis, −θ0) · T_CB_new · S

        `T_extra_B` — base 프레임에서 물체에 추가로 가할 변환(선택). flip 밴드
        캡처가 "사람이 뒤집은 것 되돌리기"(`_flip_unflip_B`)를 여기로 넣는다 —
        카메라 이동 보정과 뒤집기 힌트가 **한 식**으로 합쳐진다.

        구성요소 세 가지가 **모두** 필요하다 (2026-09-16 실물에서 각각 사고):
          · 카메라 이동 (T_BC_master·T_CB_new): 로봇이 움직인 만큼 — master 기준은
            `master_T_CB` 고정 스냅샷 (self._T_BC 는 recovery 가 덮어씀).
          · 물체 회전 R_B(−θ0): 스캔 시작 시 turntable 이 논리 0° 가 아니면 물체가
            그만큼 돌아간 채 찍힌다. gap 겨냥(θ=306°)에서 빼먹어 유령 geometry →
            다음 반복이 유령을 겨냥, 스캐너가 뚜껑 15mm 까지 접근했다.
            부호 검증: `p1.rot_about_axis(..., -theta)` (실기 검증된 계획 경로)와 동일.
          · S 켤레: scan-world 는 **스캐너3D 프레임**이지 Color 가 아니다
            ([[project_artec_3d_vs_color_frame]], 180°+41.8mm). hand-eye 는 Color
            기준이라 켤레 없이 B 왕복하면 병합이 어긋난다.
        """
        _T_BC_master = (np.linalg.inv(self._st.master_T_CB)
                        if self._st.master_T_CB is not None else self._T_BC)
        R_obj = self._R_obj_B(theta0, tag=tag)
        T_x = np.eye(4) if T_extra_B is None else np.asarray(T_extra_B, float)
        T_core = _T_BC_master @ T_x @ R_obj @ np.linalg.inv(T_BC_new)   # B 경유 (m)
        S_m = self._S_m()
        if S_m is not None:
            T_pre = np.linalg.inv(S_m) @ T_core @ S_m
        else:
            print(f"  [{tag}] ⚠ 스캐너3D→Color 미캐시 — S 켤레 없이 병합 (오차 가능)")
            T_pre = T_core.copy()
        T_pre = T_pre.copy()
        T_pre[:3, 3] *= 1000.0                              # m → mm (SDK 단위)
        return T_pre

    def _cam_pos_B(self):
        """카메라 원점의 **base(B) 좌표** (m). 없으면 None.

        `_T_CB` 는 C→B 라 그 translation 이 곧 B 에서 본 카메라 위치다. 이동할
        때마다 `_recapture_T_BC` 가 갱신하므로 캡처 시점과 일치한다. 생성자에서
        hand-eye 를 못 잡은 경우만 현재 FK 로 직접 만든다.
        """
        T = self._T_CB
        if T is None:
            try:
                T_BC = self._T_EC @ np.linalg.inv(self.robot.get_ee_pose_mat())
                T = np.linalg.inv(T_BC)
            except Exception as e:                              # noqa: BLE001
                if not getattr(self, "_cam_pos_warned", False):
                    self._cam_pos_warned = True
                    print(f"  [p1plan] ⚠ 카메라 위치를 못 구했다"
                          f"({type(e).__name__}: {e}) — 이 프레임 거리보정 건너뜀")
                return None
        return np.asarray(T, float)[:3, 3].copy()

    def _recapture_T_BC(self, tag: str, return_only: bool = False):
        """이동 후 T_BC/T_CB 재캡처 (recovery hint 일관성).

        `return_only=True` 면 **새 T_BC 를 돌려주기만 하고 `self._T_BC` 는 그대로
        둔다.** master 기준(첫 밴드의 T_BC)을 유지한 채 이동 후 값을 얻어야
        `T_pre = T_BC_master @ inv(T_BC_new)` 를 만들 수 있기 때문이다.
        """
        T_EB_after = self.robot.get_ee_pose_mat()
        T_BC_new = self._T_EC @ np.linalg.inv(T_EB_after)
        tb = T_BC_new[:3, 3]
        print(f"  [{tag}] ✓ 이동 완료 — T_BC {'측정' if return_only else '재캡처'} "
              f"(t=({tb[0]*1000:+.1f},{tb[1]*1000:+.1f},"
              f"{tb[2]*1000:+.1f})mm)")
        if return_only:
            return T_BC_new
        self._T_BC = T_BC_new
        self._T_CB = np.linalg.inv(self._T_BC)
        return T_BC_new

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

    def _lookaround_view_score(self, verts_C_mm, T_CB_cand,
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
            sc, n_obj, n_excl = self._lookaround_view_score(
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

        호출 시점: **tracking-lost recovery 전용**(`recovery=True`). 스캔 시작
        자세는 이 함수가 아니라 `pick_lookaround_pose` → `_pick_lookaround_planner`
        (sim 과 같은 공용 전회전 채점기)가 정한다. recovery 시에는 elevation
        후보를 축소(`recovery_elevation_offsets_deg` + fine skip)해 시간을 줄인다.

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

    # ── ScanBackend 프리미티브 (공용 utils/nbv/scan_stage_controller) ────
    # 순서/게이팅/NBV 수렴 루프는 공용 컨트롤러 소유. 이 클래스는 real Artec
    # 캡처 하드웨어(streaming + relocalization + recovery + hint)만 제공.
    @property
    def stage_until(self) -> int:
        return self.s.stage_until

    @property
    def nbv_k_max(self) -> int:
        return self.s.nbv_K_max

    def run(self) -> ArtecMultiPassScanResult:
        # 시작 자세: pick_lookaround_pose 가 공용 플래너로 고른다(2026-09-09, sim 과 통일).
        # 플래너 실패 시에만 예전 동작인 home 고정(AT_CURRENT)으로 폴백한다.
        # tracking-lost 시에는 _attempt_recovery 가
        # _adaptive_prescan_position(recovery=True) 를 발동. (docs 2_lookaround §3 / §6)
        #
        # 구조 (2026-07-01): lookaround→nbv→flip 순서/NBV 루프는 공용
        # utils/nbv/scan_stage_controller.run_scan_stages 가 소유. 여기 run() 은
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

        return run_scan_stages(self)

    # ── 공통 ────────────────────────────────────────────────────────────
    def confirm_start(self) -> bool:
        # lookaround 시작 전 1회 확인 — Studio 시작 위치 등. real 전용(사람 개입).
        s = self.s
        st = self._st
        if s.prompt_before_first_pass:
            self._print_pose_hint_for_user(st.pose_idx)
            print("  [Enter] 회전 시작    [q] 종료    (키 하나만)")
            if self._wait_user_quit():
                st.user_quit = True
                st.aborted_reason = "사용자 종료 (첫 pass 전)"
                return False
        return True

    # ── lookaround 자세 선정 (sim 과 같은 공용 플래너) ─────────────────────
    def _turntable_frame(self):
        """턴테이블 캘리브(T_BF0) → (axis_pt(3,) base, axis_dir(3,) base). 없으면 None.

        ★ `T_B_F0` 는 **B→F** 규약이다 (x_F = R·x_B + t, `build_T_B_F0` 참고).
          따라서 base 기준 값을 얻으려면 **역변환**이어야 한다:

              축 위의 점(F 원점)  = −Rᵀ·t
              축 방향(F 의 +z)    = Rᵀ 의 3열 = R 의 3행

          2026-09-16 까지 `T_BF0[:3,3]` 과 `T_BF0[:3,2]` 를 그대로 썼다. 그 값은
          base 좌표가 아니라 변환 성분이라, 실측 원판 `[0.799,0.005,0.688]` 대신
          **`[-0.862,0.080,-0.603]`** (2105mm 떨어진 base 원점 반대쪽)을 겨눴고
          축 방향도 18.1° 틀렸다. 그래서 lookaround 플래너가 로봇 자기 base 를
          바라보는 자세를 만들어 **EE 가 base 에 충돌했다**(실물 확인).
          같은 규약 실수가 `check_calibration.py` 교차검증에도 있었다.
        """
        tt = getattr(self.mms, "turntable_transform", None)
        T_BF0 = getattr(tt, "T_BF0", None) if tt is not None else None
        if T_BF0 is None:
            return None
        return tt.axis_point_B, tt.axis_dir_B

    def _move_turntable_abs(self, theta_rad: float, tol_deg: float = 3.0) -> bool:
        """계획용 실루엣 회전 — 절대각 이동 후 **실제로 도착했는지 확인**한다.

        ★ 예전엔 반환값을 무시하고 "실패는 삼킨다" 였다. 그런데 이 경로는 조용히
          실패하기 쉽다:
            · `move_abs` 가 FMP_RUNFAIL(133) 로 거부돼도 그냥 넘어갔다
            · `wait_motion_done` 은 Stage 1 에서 "motion 시작 안 됨" 을 **경고만**
              찍고 Stage 2 로 내려가, 정지+in-position 이면 `Move Done` + True 를
              돌려준다 — 안 움직인 것과 끝난 것을 구분하지 못한다
          그 결과 플래너는 90° 실루엣을 받았다고 믿고 실제로는 0° 점군을 한 번 더
          쓴다. 계획이 조용히 틀어지고 원인은 로그에 안 남는다 (2026-09-16).

        그래서 **위치로 검증**한다. 도착 못 했으면 False 를 돌려주고, 호출자가
        그 방향 실루엣을 버릴 수 있게 한다.
        """
        want = float(theta_rad)
        try:
            ok = self.turntable.move_abs(
                want, float(self.s.preview_turntable_vel_rad_s))
            if ok is False:
                print(f"  [p1plan] ⚠ move_abs 거부 — θ={np.degrees(want):+.1f}° 건너뜀")
                return False
            if hasattr(self.turntable, "wait_motion_done"):
                self.turntable.wait_motion_done(timeout_s=20.0)
        except Exception as e:                                  # noqa: BLE001
            print(f"  [p1plan] ⚠ 턴테이블 이동 실패({type(e).__name__}: {e})")
            return False

        # 실제 위치 확인 — wait_motion_done 만으로는 '안 움직임' 을 못 거른다.
        try:
            pos = self.turntable.getActualPos()
        except Exception:                                       # noqa: BLE001
            pos = None
        if pos is None or isinstance(pos, bool):
            print("  [p1plan] ⚠ 턴테이블 위치를 못 읽었다 — 도착 검증 불가")
            return True                                         # 판단 보류, 진행
        err_deg = abs(np.degrees(float(pos) - want))
        err_deg = min(err_deg, 360.0 - err_deg % 360.0) if err_deg > 180 else err_deg
        if err_deg > tol_deg:
            print(f"  [p1plan] ⚠ 턴테이블이 목표에 도달하지 못했다 — "
                  f"목표 {np.degrees(want):+.1f}° · 실제 {np.degrees(float(pos)):+.1f}° "
                  f"(오차 {err_deg:.1f}°). 이 방향 실루엣은 버린다")
            return False
        return True

    def _preview_object_points_B(self, axis_pt, tz: float, d: float):
        """조준높이 tz·축거리 d 로 구동 후 preview 캡처.

        반환 = **(base 프레임 물체 점군, base 프레임 카메라 위치)**.

        ★ 카메라 위치를 같이 주는 것이 **적응적 preview 의 전제**다. 거리 보정은
          "표면까지 거리"로 계산되는데, 그건 점군만으로는 못 구한다
          (`collect_planning_points` 는 콜백이 카메라 위치를 안 주면 적응을 끄고
          고정 격자로 떨어진다). 2026-09-17 이전에는 이 함수가 점군만 돌려줘서
          **real 은 적응 preview 가 한 번도 안 돌았다** — sim 에서만 돌고 있었다.
          게다가 첫 높이에서 이동·캡처를 한 번 해놓고 그 결과를 버린 뒤
          고정 격자를 다시 돌았다(이동 1회 순손실).

        sim `_pick_lookaround_planner.preview_at` 의 real 판. 캡처 수단만 다르고
        (Isaac 카메라 ↔ Artec preview) 크롭·self-filter 는 같은 공용 함수를 쓴다.
        """
        from utils.nbv import lookaround as p1
        from utils.collision.robot_collision import (
            capsules_from_joints, DEFAULT_LINK_RADII)
        s = self.s
        sensor = getattr(self.mms, "sensor", None)
        if sensor is None or not hasattr(sensor, "capture_frame"):
            return np.zeros((0, 3)), self._cam_pos_B()
        axis_xy = np.asarray(axis_pt, float)[:2]
        target = np.array([axis_xy[0], axis_xy[1], float(tz)], float)
        try:
            q_seed = np.asarray(self.robot.get_joint_angles(is_radian=True), float)
        except Exception:                                       # noqa: BLE001
            return np.zeros((0, 3)), self._cam_pos_B()
        # ★ 예전엔 이 루프가 **완전히 조용했다.** IK 실패·충돌 거부·정점 부족이
        #   전부 로그 없이 `continue`/빈 배열이라, 로봇이 안 움직이는데 화면엔
        #   아무것도 안 뜨는 상태가 됐다(2026-09-16 실물). 어느 단계에서 막혔는지
        #   알 수 없으면 디버깅이 불가능하므로 각 실패를 한 줄씩 남긴다.
        n_ik = n_move = 0
        for azd in s.lookaround_view_azis_deg:      # 도달 azimuth 스윕 (커버리지 무관)
            q = self._axis_view_q(target, s.preview_el_deg,
                                  float(azd), float(d), q_seed)
            if q is None:
                n_ik += 1
                continue
            code = self._move_robot_to_q(q, s.adaptive_robot_speed_deg_s)
            if code != 0:
                n_move += 1
                print(f"  [p1plan] az={azd:+.0f}° 이동 실패(code={code}) — 다음 방위")
                continue                        # 충돌 거부/구동 실패 → 다음 az
            self._recapture_T_BC("p1plan")
            vC = self._capture_preview_verts(sensor, s.preview_raw_min_verts,
                                             to_color=True)
            if vC is None:
                print(f"  [p1plan] tz={tz:.3f} d={d:.2f} az={azd:+.0f}° — "
                      f"원시 정점 부족(<{s.preview_raw_min_verts}) = 빈 시야")
                return np.zeros((0, 3)), self._cam_pos_B()
            print(f"  [p1plan] tz={tz:.3f} d={d:.2f} az={azd:+.0f}° — "
                  f"정점 {len(vC):,}")
            self._snap_preview(tz, d, azd, len(vC))
            T_CB = np.asarray(self._T_CB, float)
            xB = (vC / 1000.0) @ T_CB[:3, :3].T + T_CB[:3, 3]
            # ★ 로봇 자기점 제거 — 프레임에 걸린 링크/스캐너 점이 크롭 실린더를
            #   오염해 밴드 수를 폭주시킨다 (sim 과 동일 처리).
            try:
                q_now = np.asarray(self.robot.get_joint_angles(is_radian=True), float)
                xB = p1.filter_robot_points(
                    xB, capsules_from_joints(q_now, DEFAULT_LINK_RADII,
                                             T_EC=self._T_EC))
            except Exception:                                   # noqa: BLE001
                pass
            # ★ up_sign 필수 — base 는 +Z 가 아래라 기본 +1 이면 물체 쪽을
            #   전부 버린다(2026-09-16: 정점 30k → 크롭 후 0점).
            xo = p1.crop_object_points(
                xB, axis_xy, float(axis_pt[2]),
                up_sign=(-1.0 if float(self._view_up_B()[2]) < 0 else +1.0))
            print(f"  [p1plan]     크롭 {len(xB):,} → {len(xo):,}점 "
                  f"(축 r<0.16m · 디스크 위 0.004~0.45m)")
            # ★ 응집도 게이트 — 빈 시야의 노이즈(300~500점)는 원시 문턱도 크롭도 통과한다.
            #   run_144803: 병 위 허공(디스크 위 168·236mm)에서 423·283점이 "물체"로
            #   인정돼 거리를 300→404mm 로 밀고 높이 사다리를 380mm 까지 올렸다(그 자세는
            #   프레임 충돌로 게이트 거부). 진짜 표면은 4mm 이웃 중앙값 19~90·응집점 80~94%,
            #   노이즈는 중앙값 3·응집점 2~15% 로 갈린다(같은 run 의 preview 패치 실측).
            coh_n, coh_f = self._coherence(xo)
            if len(xo) and (coh_n < s.preview_min_coherent_pts
                            or coh_f < s.preview_min_coherent_frac):
                print(f"  [p1plan]     응집도 미달 — 4mm 이웃≥8 인 점 {coh_n} ({coh_f*100:.0f}%) "
                      f"< {s.preview_min_coherent_pts}·{s.preview_min_coherent_frac*100:.0f}% → 빈 시야로 봄")
                return np.zeros((0, 3)), self._cam_pos_B()
            return xo, self._cam_pos_B()
        print(f"  [p1plan] tz={tz:.3f} d={d:.2f} — 도달 가능한 방위 없음 "
              f"(IK 실패 {n_ik} · 이동 실패 {n_move} / "
              f"{len(s.lookaround_view_azis_deg)} 방위)")
        return np.zeros((0, 3)), self._cam_pos_B()

    @staticmethod
    def _coherence(pts, radius_m: float = 0.004, min_nn: int = 8):
        """(응집점 수, 비율) — `radius_m` 안 이웃이 `min_nn` 이상인 점. 노이즈 판별용."""
        P = np.asarray(pts, float)
        if len(P) < 3:
            return 0, 0.0
        try:
            from scipy.spatial import cKDTree
            nn = cKDTree(P).query_ball_point(P, radius_m, return_length=True) - 1
            n = int((nn >= min_nn).sum())
            return n, n / float(len(P))
        except Exception:                                        # noqa: BLE001
            return len(P), 1.0                                   # 판별 불가면 통과

    def _pick_lookaround_planner(self):
        """계획용 preview → 기하 크롭 → plan_lookaround_viewpoints → az 스윕 IK.

        sim(`isaac_scan_session._pick_lookaround_planner`)과 **같은 공용 함수**를 부른다
        (`p1.collect_planning_points` / `plan_lookaround_viewpoints` / `solve_plan_poses`).
        다른 것은 preview 캡처 수단과 IK 콜백뿐이다. 실패하면 None → home 고정.
        """
        from utils.nbv import lookaround as p1
        s = self.s
        frame = self._turntable_frame()
        if frame is None:
            print("  [p1plan] turntable_transform/T_BF0 없음 — home 고정")
            return None
        axis_pt, axis_dir = frame
        self._axis_pt_B = np.asarray(axis_pt, float)   # nbv 디버그 기록용(턴테이블 축·원판 상면, B)
        axis_xy = axis_pt[:2]
        # ★ `dof`(작동거리 창)는 **스캐너에게 물어본다.** 여태 하드코딩 추측이었고,
        #   그 값이 밴드 수·커버리지 판정을 직접 좌우한다. SDK 가 알려주는 값을 쓰면
        #   플래너의 가정 = 스캐너의 설정이 된다.
        sensor_model = p1.SensorModel()
        #  real·sim 둘 다 `scanning_range()` 를 같은 계약으로 제공한다 —
        #  백엔드를 가리지 않는다(예전엔 `sensor._processor` 를 직접 뒤졌다).
        try:
            _near, _far = self.mms.sensor.scanning_range()
            sensor_model = p1.sensor_from_scanning_range(
                _near, _far, sensor_model, log=lambda m: print(f"  [p1plan]{m}"))
        except Exception as e:                                  # noqa: BLE001
            print(f"  [p1plan] ⚠ 스캔 범위 조회 실패({type(e).__name__}: {e}) — dof 기본값")

        # ★ `up_sign` 을 **명시**한다. base 가 천장 마운트라 +Z 가 아래여서,
        #   기본값 +1 이면 시작 조준높이가 원판 표면 **아래**(테이블 속)가 되고
        #   높이를 올릴수록 더 파고든다 → 물체가 아니라 턴테이블을 겨눈다
        #   (2026-09-16 실물 관측). sim 은 world 프레임이라 +1 이 맞다.
        up_sign = -1.0 if float(self._view_up_B()[2]) < 0 else +1.0
        # ★ (a) preview 자체를 보호한다 — 물체를 등록하려면 먼저 봐야 하는데
        #   preview 가 그 '보는' 단계다. 그동안 충돌 모델에 대상이 없다.
        #   아직 형상을 모르니 **있을 수 있는 최대 반경**으로 원기둥을 건다.
        self._guard_preview_volume(axis_pt, up_sign, float(s.preview_el_deg))
        pts = p1.collect_planning_points(
            lambda tz, d: self._preview_object_points_B(axis_pt, tz, d),
            self._move_turntable_abs, axis_pt, axis_dir,
            d_steps=(tuple(s.preview_dists_m) if s.preview_dists_m else None),
            up_sign=up_sign,
            log=lambda m: print(f"  [p1plan] {m}"))
        pts = p1.voxel_downsample(pts, sensor_model.voxel_m)
        if len(pts) < s.lookaround_min_plan_points:
            print(f"  [p1plan] preview 점 부족({len(pts)}) — home 고정")
            return None

        # ★ 계획 점군을 **충돌 게이트의 동적 장애물**로 등록한다. 셀 CAD 에는
        #   스캔 대상이 없어서 자세 간 이동 경로가 물체를 관통해도 못 막았다
        #   (2026-09-16 실물: NBV 이동 중 대상을 치고 지나감). 지금부터의 모든
        #   이동(밴드·NBV·recovery)이 이 장애물을 회피한다. nbv 부터는
        #   `build_coverage_mesh` 가 더 정확한 메시 정점으로 갱신한다.
        self._register_object_obstacle(pts, "preview")
        nrm = p1.estimate_outward_normals(pts, axis_xy)
        # `up_sign` — 위에서 구한 것과 같은 값. 플래너가 '윗면(뚜껑)' 보강 자세를
        # 넣을 때 **어느 끝이 위인지** 알아야 한다. base 가 천장 마운트라 −1.
        plan = p1.plan_lookaround_viewpoints(pts, nrm, axis_xy, sensor_model,
                                         els=tuple(s.lookaround_els_deg),
                                         up_sign=up_sign)
        print(f"  [p1plan] 플랜: {plan.note} risk={plan.tracking_risk} "
              f"(preview {len(pts)}pt)")
        try:
            q_seed = np.asarray(self.robot.get_joint_angles(is_radian=True), float)
        except Exception:                                       # noqa: BLE001
            return None
        cm = self._collision_gate()
        qs, vps = p1.solve_plan_poses(
            plan, axis_xy, tuple(s.lookaround_view_azis_deg),
            solve_q=lambda tgt, el, az, so: self._axis_view_q(tgt, el, az, so, q_seed),
            is_safe=(cm.is_pose_safe if cm is not None else None),
            log=lambda m: print(f"  [p1plan] {m}"), return_poses=True)
        # ★ 캡처 중 거리추종용 맥락 — sim(`isaac_scan_session`)과 같은 구조다.
        #   q 만으로는 "이 밴드가 원래 el/az/tz 몇이었나" 를 되찾을 수 없고,
        #   그게 없으면 축거리만 고쳐 다시 푸는 것이 불가능하다.
        self._band_pairs = list(zip(qs or [], vps))
        self._band_seed = np.asarray(q_seed, float).copy()
        self._band_axis_xy = np.asarray(axis_xy, float).copy()
        self._band_sensor = sensor_model
        # ★ preview 결과를 **보관**한다. `--until preview` 는 캡처를 안 하므로
        #   모델이 비어 있어 기존 결과 창이 보여줄 것이 없다. 무엇을 보고
        #   밴드를 그렇게 정했는지 눈으로 확인할 수 있어야 한다
        #   ("모션은 맞는데 preview 가 잘 됐는지 모르겠다" — 2026-09-21).
        # ★ 점군을 **파일로도 남긴다.** 창을 닫으면 사라져서 "왜 이렇게 나왔나"
        #   를 나중에 못 따진다. 좌표계 문제는 눈보다 숫자로 재야 갈린다.
        try:
            import datetime as _dt
            from pathlib import Path as _Path
            _o = _Path(__file__).resolve().parents[2] / "output" / "debug"
            _o.mkdir(parents=True, exist_ok=True)
            _f = _o / f"preview_points_{_dt.datetime.now():%H%M%S}.npz"
            _extra = {}
            for _i, (_t, _P) in enumerate(
                    getattr(p1.collect_planning_points, "last_patches", []) or []):
                _extra[f"patch{_i}_theta"] = np.asarray([_t], float)
                _extra[f"patch{_i}_pts"] = _P          # 역회전 **후** (물체프레임)
            np.savez_compressed(_f, points_B=np.asarray(pts, float),
                                axis_pt_B=np.asarray(axis_pt, float),
                                axis_dir_B=np.asarray(axis_dir, float),
                                up_sign=float(up_sign), **_extra)
            print(f"  [p1plan] preview 점군 저장 → {_f.relative_to(_o.parents[1])}")
        except Exception as e:                                  # noqa: BLE001
            print(f"  [p1plan] ⚠ preview 점군 저장 실패({type(e).__name__})")
        self.preview_result = {
            "points_B": np.asarray(pts, float).copy(),
            "axis_pt_B": np.asarray(axis_pt, float).copy(),
            "axis_dir_B": np.asarray(axis_dir, float).copy(),
            "up_sign": float(up_sign),
            "plan": plan,
            "view_poses": list(vps or []),
            "note": plan.note,
        }
        return qs

    def _snap_preview(self, tz: float, d: float, azd: float, n_raw: int) -> None:
        """preview 스냅샷 저장 (sim `_snap(stage="preview")` 와 **같은 규칙**).

        파일명·폴더·캡션을 sim 과 맞춘다 — 두 쪽 그림을 나란히 놓고 비교하는 것이
        목적이므로 규칙이 갈라지면 의미가 없다. 저장 위치 `output/debug/preview/`.
        """
        img = getattr(self, "_last_preview_img", None)
        if img is None:
            return
        dbg = getattr(self, "_dbg", None)
        if dbg is None:
            try:
                from utils.debug_view import DebugViewSaver
                dbg = self._dbg = DebugViewSaver(log=lambda m: print(f"  [p1plan] {m}"))
            except Exception:                                   # noqa: BLE001
                self._dbg = False
                return
        if dbg is False:
            return
        try:
            dbg.save(img,
                     f"preview_tz{tz*1000:.0f}_d{d*1000:.0f}_az{azd:+.0f}",
                     f"preview tz={tz*1000:.0f}mm d={d*1000:.0f}mm "
                     f"az={azd:+.0f}deg raw={n_raw}", stage="preview")
        except Exception as e:                                  # noqa: BLE001
            print(f"  [p1plan] [dbgview] ⚠ 스냅 실패({type(e).__name__}: {e})")

    def _guard_preview_volume(self, axis_pt, up_sign: float,
                              el_deg: float) -> None:
        """preview 동안 축 둘레에 보수적 원기둥을 장애물로 걸어 둔다(sim 과 동일).

        근거·함정은 공용 `lookaround.guard_cylinder_points` 주석에 있다. 반경은
        **원판 반경**(물체가 그보다 넓으면 넘어진다), 마진은 **0**(원기둥 자체가
        이미 최대 가정이라 위에 또 얹으면 이중보정) 이다.
        """
        cm = self._collision_gate()
        if cm is None:
            return
        try:
            _R = min(_p1_const.PREVIEW_RADIUS_MAX_M,
                     float(self.s.nbv_turntable_radius_mm) / 1000.0)
            lo = _p1_const.probe_bounds(self._scanning_range_m())[0]
            blocks, d_min = _p1_const.guard_blocks_probe(_R, 0.0, el_deg, lo)
            if blocks:
                print(f"  [p1plan] ⚠ 보호 원기둥(r={_R*1000:.0f}mm)이 탐침을 막는다 "
                      f"— el={el_deg:.0f}° 에서 축거리 {d_min*1000:.0f}mm 아래로 못 간다")
            pts = _p1_const.guard_cylinder_points(axis_pt, up_sign, _R)
            cm.set_dynamic_obstacle(pts, margin_m=0.0)
            print(f"  [p1plan] preview 보호 원기둥 — r={_R*1000:.0f}mm 마진 0 "
                  f"({len(pts):,}pt, 최소 축거리 {d_min*1000:.0f}mm)")
        except Exception as e:                                  # noqa: BLE001
            print(f"  [p1plan] ⚠ preview 보호 원기둥 실패({type(e).__name__}: {e})")

    def _register_object_obstacle(self, pts_B, tag: str) -> None:
        """**base** 점군을 스캔 대상 장애물로 등록한다 (sim `_register_object_obstacle`).

        ★ swept 적용을 **여기 한 곳**에서만 한다. 예전엔 호출부 두 곳이 각각
          `swept_about_axis(...)` 를 불렀고, 한쪽엔 축을 못 구하면 swept 없이 그냥
          등록하는 폴백까지 있었다 — 그 경로로 가면 θ=0 점군이 그대로 걸려
          비대칭 물체에서 장애물이 엉뚱한 방향을 향한다.
        """
        cm = self._collision_gate()
        if cm is None or pts_B is None or len(pts_B) == 0:
            return
        fr = self._turntable_frame()
        if fr is None:
            print(f"  [p1plan] ⚠ 대상물 장애물({tag}) — 턴테이블 축 없음, "
                  f"회전체 적용 불가. 비대칭 물체면 방향이 틀릴 수 있다")
            cm.set_dynamic_obstacle(np.asarray(pts_B, float))
            return
        try:
            # 축 둘레 **회전체 외피** — sim 과 같은 함수, 같은 이유
            #   (`lookaround.revolution_envelope` 주석).
            swept = _p1_const.revolution_envelope(np.asarray(pts_B, float),
                                                  fr[0], fr[1])
            cm.set_dynamic_obstacle(swept)
            lo, hi = swept.min(0), swept.max(0)
            _r = float(np.linalg.norm(
                (swept - np.asarray(fr[0], float))[:, :2], axis=1).max())
            print(f"  [p1plan] 대상물 장애물({tag}) {len(swept):,}pt "
                  f"최대반경 {_r*1000:.0f}mm · base bbox "
                  f"x[{lo[0]:.3f},{hi[0]:.3f}] y[{lo[1]:.3f},{hi[1]:.3f}] "
                  f"z[{lo[2]:.3f},{hi[2]:.3f}]")
        except Exception as e:                                  # noqa: BLE001
            print(f"  [p1plan] ⚠ 대상물 장애물 등록 실패({type(e).__name__}: {e})")

    def _scanning_range_m(self):
        """작동거리 창 (near_m, far_m). 스캐너가 못 알려주면 플래너 기본값.

        거리추종의 **목표**가 이 창의 중앙이므로, 계획에 쓴 창과 캡처 중 쓰는 창이
        같아야 한다 — 그래서 여기서도 `scanning_range()` 한 곳에서만 가져온다.
        """
        from utils.nbv import lookaround as p1
        try:
            near_mm, far_mm = self.mms.sensor.scanning_range()
            near, far = float(near_mm) / 1000.0, float(far_mm) / 1000.0
            if 0.0 < near < far:
                return (near, far)
        except Exception:                                       # noqa: BLE001
            pass
        return tuple(p1.SensorModel().dof)

    # ── 거리추종 재겨냥 (el·az·tz 고정, 축거리만) ─────────────────────────
    def _band_retarget(self, q_cur, d_new):
        """밴드 자세 `q_cur` 를 **같은 el/az/tz, 새 축거리**로 다시 푼다.

        반환 = 새 q (IK·충돌 통과) 또는 None. 이동은 호출자(streaming session)가
        `move_robot_fn` 으로 한다 — 녹화 중 이동 속도를 그쪽이 쥐고 있어서다.
        """
        vp = None
        for _q, _vp in getattr(self, "_band_pairs", ()):
            if _q is q_cur:
                vp = _vp
                break
        if vp is None:
            return None
        axis_xy = getattr(self, "_band_axis_xy", None)
        seed = getattr(self, "_band_seed", None)
        if axis_xy is None or seed is None:
            return None
        # ★ seed 는 **지금 관절각** — 계획 seed 로 풀면 다른 IK 분기가 나와 녹화 중
        #   큰 관절 이동이 생길 수 있다. 소보간 경로로 도착한 뒤엔 특히 그렇다.
        try:
            seed = np.asarray(self.robot.get_joint_angles(is_radian=True), float)
        except Exception:                                       # noqa: BLE001
            pass
        tgt = np.array([axis_xy[0], axis_xy[1], vp.target_z], float)
        qn = self._axis_view_q(tgt, vp.el_deg, vp.az_deg, float(d_new), seed)
        if qn is None:
            return None
        cm = self._collision_gate()
        if cm is not None:
            ok, _why = cm.is_pose_safe(qn)
            if not ok:
                return None
        return qn

    def _band_vp_of(self, q):
        for _q, _vp in getattr(self, "_band_pairs", ()):
            if _q is q:
                return _vp
        return None

    def _band_path(self, q_from, q_to, step_m: float = 0.005,
                   alpha_from: float = 0.0, seed=None,
                   standoff_from: float = None, standoff_to: float = None):
        """밴드 A→B 를 **카메라 조준을 유지한 채** 잘게 나눈 관절해 목록 [(q, α)].

        관절공간 직선보간(예전 `move_robot_fn(q_to)` 한 방)은 두 IK 해 사이에서
        카메라가 물체를 벗어나거나 프레임 간 변화가 커질 수 있고, Artec SDK 는 한번
        잃으면 되찾지 않는다 — 2026-09-21 여섯 run 모두 밴드 전환 직후 lost, 어느
        기하 지표와도 상관 없음(비결정 = 경로 의존). 그래서 (el, az, tz, 축거리)를
        α 로 선형보간하고 각 스텝을 직전 해를 seed 로 다시 푼다: 카메라 이동이
        `step_m` 씩이라 손으로 천천히 옮기는 것과 같다.
        `alpha_from` 부터 시작(relocalization 후 재전진), 마지막은 α=1.
        IK·충돌 실패 스텝은 건너뛴다. 맥락이 없으면 None → 호출자가 예전 방식.
        """
        vp_a, vp_b = self._band_vp_of(q_from), self._band_vp_of(q_to)
        axis_xy = getattr(self, "_band_axis_xy", None)
        if vp_a is None or vp_b is None or axis_xy is None:
            return None
        up = -1.0 if float(self._view_up_B()[2]) < 0 else +1.0

        def _cam(vp_el, d, tz):                       # 카메라 위치 근사 (수평, z)
            e = np.radians(vp_el)
            return np.array([d * np.cos(e), tz + up * d * np.sin(e)])

        # ★ 축거리는 계획값이 아니라 **추종 후 실제값**에서 출발하고(standoff_from),
        #   목표도 같은 보정을 반영한 값(standoff_to)으로 — 2026-09-22 run_162620 참조.
        pa = (float(vp_a.el_deg), float(vp_a.az_deg), float(vp_a.target_z),
              float(standoff_from if standoff_from else vp_a.standoff))
        pb = (float(vp_b.el_deg), float(vp_b.az_deg), float(vp_b.target_z),
              float(standoff_to if standoff_to else vp_b.standoff))
        dist = float(np.linalg.norm(_cam(pa[0], pa[3], pa[2]) - _cam(pb[0], pb[3], pb[2])))
        # 방위 변화도 카메라를 옮긴다 — 호 길이로 더한다.
        dist += abs(np.radians(pb[1] - pa[1])) * pb[3] * np.cos(np.radians(pb[0]))
        n = max(1, int(np.ceil(dist * (1.0 - alpha_from) / max(step_m, 1e-3))))
        alphas = alpha_from + (1.0 - alpha_from) * (np.arange(1, n + 1) / n)
        cm = self._collision_gate()
        if seed is None:
            try:                                     # 지금 관절각 — 추종 후 실제 자세
                seed = np.asarray(self.robot.get_joint_angles(is_radian=True), float)
            except Exception:                        # noqa: BLE001
                seed = None
        q_seed = np.asarray(seed if seed is not None else q_from, float)
        out = []
        for a in alphas:
            el, az, tz, d = (pa[i] + a * (pb[i] - pa[i]) for i in range(4))
            tgt = np.array([axis_xy[0], axis_xy[1], tz], float)
            q = self._axis_view_q(tgt, el, az, d, q_seed)
            if q is None:
                continue
            if cm is not None and not cm.is_pose_safe(q)[0]:
                continue
            out.append((np.asarray(q, float), float(a)))
            q_seed = np.asarray(q, float)
        return out

    def _band_standoff_of(self, q_cur):
        """이 밴드의 계획 축거리 (m). 맥락이 없으면 None."""
        for _q, _vp in getattr(self, "_band_pairs", ()):
            if _q is q_cur:
                return float(_vp.standoff)
        return None

    def pick_lookaround_pose(self):
        """lookaround 시작 자세. 플래너가 성공하면 그 자세(밴드면 리스트),
        실패하면 AT_CURRENT(= home 고정, 2026-05-20 이전 동작)."""
        if not self.s.lookaround_planner_enabled:
            return AT_CURRENT
        try:
            qs = self._pick_lookaround_planner()
        except Exception as e:                                  # noqa: BLE001
            print(f"  [p1plan] ⚠ 예외({type(e).__name__}: {e}) — home 고정")
            qs = None
        if qs:
            print(f"  [p1plan] ✓ lookaround 자세 {len(qs)}개 확정")
            return qs
        return AT_CURRENT

    def go_home(self) -> None:
        """nbv→flip 전환 등 — 로봇을 home 으로. flip 은 사람이 물체에 손을 대므로
        **뒤집기마다** 먼저 home 으로 물러난다.

        ★ 반드시 `_move_robot_to_q`(충돌 게이트 + 동적 장애물 + 우회 계획)로 간다.
          2026-09-22 실물: `robot.go_home()` 을 직접 불러 게이트를 건너뛰었고, nbv
          자세에서 home 으로 관절 직선보간하다 턴테이블 위 물체를 칠 뻔해 비상정지.
          게이트가 거부하면 **움직이지 않고** 사람에게 넘긴다.
        """
        try:
            home_deg = self.robot.HOME_JOINTS_DEG["artec"]
        except Exception as e:                                   # noqa: BLE001
            print(f"  [stage] ⚠ home 관절 조회 실패({e}) — 이동 안 함")
            return
        q_home = np.radians(np.asarray(home_deg, float))
        print(f"  [stage] home 복귀 (게이트 경유, {self.s.recovery_robot_speed_deg_s:.0f}°/s)")
        code = self._move_robot_to_q(q_home, self.s.recovery_robot_speed_deg_s)
        if code != 0:
            print(f"  [stage] ✘ home 이동 거부/실패(code={code}) — 로봇을 그 자리에 둔다. "
                  f"물체를 치운 뒤 `python scripts/robot/home.py` 로 보낼 것")
            return
        print("  [stage] robot home 복귀")

    def plan_flip_poses(self):
        """뒤집힌 물체는 **새 형상**이다 — lookaround 와 같은 preview → 밴드 계획을
        다시 돌려 자세 목록(q)을 준다. 실패하면 None → 컨트롤러가 home 고정 캡처.

        예전(2026-09-21 이전) flip 은 `go_home` 후 AT_CURRENT 한 자세 전회전이라
        거리도 높이도 적응하지 않았다("눈으로 보기엔 home 고정" — 사용자 지적).
        정합은 `capture_bands` 가 카메라 이동 보정을 예약하고 `_do_one_rotation`
        이 뒤집기 되돌리기와 합성한다(`_flip_unflip_B`).
        """
        if not self.s.lookaround_planner_enabled:
            return None
        try:
            qs = self._pick_lookaround_planner()
        except Exception as e:                                  # noqa: BLE001
            print(f"  [flip] ⚠ preview/밴드 계획 예외({type(e).__name__}: {e}) — home 고정")
            return None
        if qs:
            print(f"  [flip] ✓ 뒤집힌 물체 밴드 자세 {len(qs)}개 확정")
            return list(qs)
        return None

    def capture_bands(self, poses, stage: str = "lookaround") -> bool:
        """lookaround 밴드 전체를 **한 IScan** 으로 캡처한다.

        ★ 예전엔 컨트롤러가 밴드마다 `capture_rotation` 을 불러 **밴드 = pass =
          IScan** 이 됐다. 그러면 SLAM 이 밴드마다 끊기고, 밴드끼리는 후처리
          GlobalRegistration 에 의존한다 — 2026-09-16 실물에서 band1/band2 가
          정합되지 않았다. 설계 의도는 **밴드가 한 scan 안에 이어지는 것**이다.

          여기서는 첫 밴드로 이동한 뒤 스트리밍 세션을 열고, 세션이 녹화를 유지한
          채 나머지 밴드로 로봇을 옮긴다(`band_poses` + `move_robot_fn`).
        """
        st = self._st
        qs = [q for q in (poses or []) if q is not None]
        if not qs:
            return False
        code = self._move_robot_to_q(qs[0], self.s.adaptive_robot_speed_deg_s)
        if code != 0:
            print(f"  [p1plan] ✘ 첫 밴드 이동 실패(code={code})")
            return False
        if self._T_BC is not None and st.master_model.scan_count() > 0:
            # master 가 이미 있다(flip 밴드 캡처) — 첫 밴드는 master 기준이 아니라
            # 카메라 이동 보정 대상이다(`capture_rotation` lookaround 경로와 같은 규약).
            st.next_T_BC_pending = self._recapture_T_BC("p1plan", return_only=True)
            print("  [band hint] 카메라 이동 보정 예약 (master 프레임으로 정렬)")
        else:
            self._recapture_T_BC("p1plan")      # master 기준 = 첫 밴드
        # ★ 밴드 **이어 붙이기**(2026-09-22). 전환에서 SDK 추적을 잃으면 남은 밴드를
        #   버리지 않고 **새 IScan** 으로 이어서 찍는다 — 새 자세에서 새로 시작한 세션은
        #   추적이 붙는다(전환 중 잃는 것과 다르다). 새 IScan 은 카메라 이동 보정(기구학)
        #   으로 초기 배치 후 인접 밴드 겹침(≥25%, 대개 90%)으로 ICP 다듬기.
        #   예전(2026-09-21~22 run 6회)은 "부분 성공 → 남은 밴드 nbv 몫" 이라 윗부분이
        #   통째로 빠졌고 nbv 는 그걸 못 메웠다. SDK 자체에는 relocalization 이 없다
        #   (ScanningState_ContinueRecord = "Not supported now").
        remaining = list(qs)
        n_total = len(qs)
        fresh_fail = 0
        try:
            while remaining and st.n_pass < self.s.max_passes:
                st.band_poses = remaining
                status = self._do_one_rotation(st, stage)
                if status == _ROT_ABORT:
                    return False
                res = st.pass_results[-1] if st.pass_results else None
                n_done = int(getattr(res, "n_bands_done", 0) or 0)
                if status == _ROT_OK:
                    return True
                for _r in getattr(res, "band_reasons", []) or []:
                    print(f"    · {_r}")
                if n_done > 0:
                    remaining = remaining[n_done:]
                    fresh_fail = 0
                    if not remaining:
                        return True
                    print(f"  [p1plan] 밴드 {n_total - len(remaining)}/{n_total} 완주 — "
                          f"남은 {len(remaining)}개는 새 IScan 으로 이어 찍는다")
                    _ev("band_continue", stage=stage, done=n_total - len(remaining), total=n_total)
                else:
                    fresh_fail += 1
                    if fresh_fail >= 2:
                        print(f"  [p1plan] 밴드 {n_total - len(remaining)}/{n_total} 에서 새 IScan 도 "
                              f"두 번 연속 실패 — 남은 {len(remaining)}개 포기 (nbv 몫)")
                        _ev("band_partial", stage=stage, done=n_total - len(remaining), total=n_total)
                        return (n_total - len(remaining)) > 0
                    print(f"  [p1plan] 밴드 0/{len(remaining)} — 같은 자세에서 새 IScan 재시도 "
                          f"({fresh_fail}/2)")
                code = self._move_robot_to_q(remaining[0], self.s.adaptive_robot_speed_deg_s)
                if code != 0:
                    print(f"  [p1plan] ✘ 밴드 이동 실패(code={code}) — 남은 밴드 포기")
                    return (n_total - len(remaining)) > 0
                st.next_T_BC_pending = self._recapture_T_BC("p1plan", return_only=True)
                print("  [band hint] 카메라 이동 보정 예약 (master 프레임으로 정렬)")
            print(f"  [p1plan] ✘ max_passes({self.s.max_passes}) 소진")
            return (n_total - len(remaining)) > 0
        finally:
            st.band_poses = None

    def capture_rotation(self, pose, label: str, stage: str) -> bool:
        """real 캡처 (턴테이블 전회전). 세 경로:
          - stage="lookaround" + pose=q: lookaround 플래너가 고른 자세로 **구동한 뒤**
            아래 AT_CURRENT 경로와 동일하게 진행. 이동이 거부되면 False.

            ⚠ **밴드가 2개 이상이면 여기로 오지 않는다.** 컨트롤러는 backend 에
              `capture_bands` 가 있으면 그쪽으로 보내고(밴드 전체 = 한 IScan),
              real 은 그것을 구현한다. 따라서 이 경로는 실질적으로 **밴드 1개**
              (= 한 자세가 물체를 다 덮는 경우)일 때만 탄다. 아래 T_pre 보정은
              master 에 이미 scan 이 있는 경우(재시도·이어스캔) 대비로 남긴다.
          - pose=AT_CURRENT (lookaround fallback / flip): 현재 고정 포즈에서
            streaming + cleanup/hint/master 병합 + tracking-lost recovery
            (_do_one_rotation 재시도 루프). 반환 False=중단.
          - stage="nbv" + pose=q: 로봇을 q 로 구동 후 streaming +
            camera-motion T_pre 병합 (_capture_nbv_pose). False=캡처 실패.
        """
        st = self._st
        if stage in ("lookaround", "flip") and pose is not AT_CURRENT:
            # lookaround 플래너가 고른 자세(밴드면 대역마다 1회). 로봇을 그 자세로
            # 옮긴 뒤부터는 AT_CURRENT 와 완전히 같은 경로 — streaming + cleanup +
            # hint + master 병합 + tracking-lost recovery.
            code = self._move_robot_to_q(pose, self.s.adaptive_robot_speed_deg_s)
            if code != 0:
                print(f"  [p1plan] ✘ 자세 이동 실패(code={code}) — 이 밴드 건너뜀")
                return False
            # ★ 밴드마다 **로봇이 움직이므로** 카메라 프레임이 바뀐다. 각 pass 는
            #   독립 IScan 이고 SLAM 은 scan 안에서만 유지되므로, master 에 붙일 때
            #   카메라 이동을 보정하지 않으면 밴드끼리 어긋난 채 쌓인다.
            #
            #   예전엔 이 보정이 **recovery 경로에서만** 걸렸다
            #   (`next_T_BC_pending` 을 거기서만 세팅). 계획된 밴드 이동에는
            #   T_pre 가 None 이라 band 2 가 자기 카메라 좌표로 병합됐고,
            #   GlobalRegistration 이 "ok" 를 내도 실제로는 정합이 안 됐다
            #   (2026-09-16 실물: band1/band2 어긋남).
            #
            #   규약은 recovery 와 동일: T_pre = T_BC_master @ inv(T_BC_new)
            #   — master(=첫 밴드) 카메라 프레임으로 되돌린다.
            if self._T_BC is not None and self._st.master_model.scan_count() > 0:
                self._st.next_T_BC_pending = self._recapture_T_BC(
                    "p1plan", return_only=True)
                print("  [band hint] 카메라 이동 보정 예약 "
                      "(master 프레임으로 정렬)")
            else:
                self._recapture_T_BC("p1plan")   # 첫 밴드 = master 기준
            pose = AT_CURRENT
        if pose is AT_CURRENT:
            while st.n_pass < self.s.max_passes:
                status = self._do_one_rotation(st, stage)
                if status == _ROT_ABORT:
                    return False
                if status == _ROT_RETRY:
                    continue
                return True                     # 정상 완료
            return False                        # max_passes 소진
        # nbv — NBV 자세로 로봇 구동 후 streaming 캡처 + 병합.
        sub, T_pre = self._capture_nbv_pose(None, pose)
        if sub is None or sub.model.scan_count() == 0:
            print("  [nbv] 캡처 실패 — 종료.")
            return False
        # ★ 기구학 T_pre 는 **초기값**이다. 2026-09-18 까지 real 의 nbv 는 이걸
        #   그대로 붙였다 — sim 은 패치를 master 에 ICP 로 붙이는데 real 만 정합이
        #   없었고, IScan 사이 정합을 후처리 GlobalRegistration 에 맡겼다(그게 못
        #   믿을 것이라는 실측이 2026-09-16 밴드 건에 있다). 이제 sim 과 같은
        #   공용 `refine_to_master` 로 다듬고, 게이트에 걸리면 T_pre 그대로.
        # ★ 병합 품질 게이트 — **빈 캡처는 master 에 넣지 않는다.** 2026-09-22 run_132655:
        #   nbv #2~#12 중 6개가 14~60 프레임에 점 5~8,000개(정상은 수만~수십만). 그 쓰레기가
        #   master 에 들어가 다음 반복의 메시·gap 을 오염시켰고(gap 이 허공을 가리킴 →
        #   또 빈 캡처), Studio 에서 "노이즈 수준 scan 11개" 로 보였다. 안 넣으면 신규복셀
        #   0 → StallTracker 가 nbv 를 일찍 끝낸다.
        try:
            _T_CB_g = self._T_CB if self._T_CB is not None else np.linalg.inv(self._T_BC)
            _pcd_g = self._master_to_pcd_B(sub.model, _T_CB_g, 4.0,
                                           T_sc_mm=getattr(self, "_T_scan_color", None))
            _n_pts = 0 if _pcd_g is None else len(_pcd_g.points)
        except Exception:                                        # noqa: BLE001
            _n_pts = -1
        _ok_frames = int(getattr(sub, "frames_ok", 0) or 0)
        if 0 <= _n_pts < self.s.nbv_min_patch_pts or _ok_frames < self.s.nbv_min_patch_frames:
            print(f"  [nbv] ✘ 빈 캡처 — 점 {_n_pts:,}(4mm 복셀) · OK 프레임 {_ok_frames} "
                  f"(기준 {self.s.nbv_min_patch_pts:,}점·{self.s.nbv_min_patch_frames}프레임) → 병합 안 함")
            _ev("merge", stage="nbv", pose_idx=st.pose_idx, method="rejected(empty)",
                n_scans_before=st.master_model.scan_count(), n_pts=_n_pts, frames_ok=_ok_frames)
            if getattr(self, "_nbv", None) is not None:
                self._nbv.report_patch(False)                    # dry 로 회계
            return True
        # nbv 패치도 같은 정리(꼬리 제거 + SerialReg + Outlier) — 예전엔 이 경로만 빠져 있었다.
        sub.model = self._cleanup_model(
            self._trim_lost_tail(sub.model, int(getattr(sub, "n_tail_lost", 0) or 0)),
            "nbv 정리", outliers=self.s.pass_outlier_removal)
        T_apply = T_pre
        if (self.s.nbv_icp_refine and T_pre is not None
                and st.master_model.scan_count() > 0):
            T_ref, fit, rmse = self._hint_icp_refine(
                sub.model, T_pre, st.master_model, mode="patch")
            st.icp_refine_log.append(
                (st.master_model.scan_count(), T_pre.copy(), T_ref.copy(), fit, rmse))
            T_apply = T_ref
        self._dump_scan_raw(sub.model, T_apply, "nbv", T_pre_kin=T_pre,
                            tracking_lost=bool(getattr(sub, "tracking_lost", False)),
                            n_frames=int(getattr(sub, "n_frames", 0)))
        _ev("merge", stage="nbv", pose_idx=st.pose_idx,
            method=("icp" if T_apply is not T_pre else "camera"),
            n_scans_before=st.master_model.scan_count(),
            tracking_lost=bool(getattr(sub, "tracking_lost", False)))
        n = self._merge_into_master(sub.model, st.master_model, T_apply)
        # 겨냥·수집·정합 기록 — B 프레임 점: sub scan-world 점을 T_pre(기구학) / T_apply(정합) 로
        try:
            if getattr(self, "_nbv_dbg", None) is not None and self._T_BC is not None:
                T_CB = self._T_CB if self._T_CB is not None else np.linalg.inv(self._T_BC)
                pcd = self._master_to_pcd_B(sub.model, T_CB, 4.0)
                P_mm = np.asarray(pcd.points, float) if pcd is not None else np.zeros((0, 3))
                def to_B(T_W_mm):
                    Tm = np.asarray(T_W_mm, float).copy(); Tm[:3, 3] /= 1000.0
                    TB = T_CB @ Tm @ self._T_BC; TB[:3, 3] *= 1000.0
                    return (P_mm @ TB[:3, :3].T + TB[:3, 3]) / 1000.0
                raw = to_B(T_pre) if T_pre is not None else P_mm / 1000.0
                ali = to_B(T_apply) if T_apply is not None else raw
                from types import SimpleNamespace as _NS
                res = None
                if T_apply is not None and T_pre is not None and T_apply is not T_pre:
                    dt = float(np.linalg.norm((np.asarray(T_apply)[:3, 3] - np.asarray(T_pre)[:3, 3]) / 1000.0))
                    res = _NS(ok=True, reason="refined", fitness=float(fit), rmse=float(rmse) / 1000.0,
                              delta_translation_m=dt, delta_rotation_deg=0.0)
                self._nbv_dbg.patch(raw_pts=raw, aligned_pts=ali, result=res)
        except Exception as e:                                   # noqa: BLE001
            print(f"  [nbv-dbg] 기록 실패({e})")
        print(f"  [nbv] {n} scan 병합 (master scans={st.master_model.scan_count()})")
        return True

    def finalize(self) -> ArtecMultiPassScanResult:
        st = self._st
        s = self.s
        # flip 까지 안 가는 설정 → flip 미실행 완료 사유. 단 lookaround 이 정상
        # 완료한 경우만 (사용자 종료 / max_passes 소진 시엔 기존 사유가 우선).
        if (not _runs_stage(s.stage_until, "flip") and not st.aborted_reason
                and not st.user_quit and st.n_pass < s.max_passes):
            st.aborted_reason = f"stage_until={s.stage_until!r} 완료"

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

        print(f"\n═══ 스캔 종료 ═══")
        print(f"  밴드 완주           : {st.bands_done}/{st.bands_total}")
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
            preview_result=getattr(self, "preview_result", None),
        )

    # ── lookaround ─────────────────────────────────────────────────────────
    # (lookaround 캡처는 pick_lookaround_pose=AT_CURRENT + capture_rotation 이 담당)

    # ── flip (외부 flip → 윗면) ──────────────────────────────────────
    def supports_flip(self) -> bool:
        return True                             # real: 사용자 손회전으로 flip 가능

    def next_flip(self) -> bool:
        """flip — 사용자에게 물체를 다음 flip pose 로 뒤집도록 안내(+pose_idx
        advance). 정상 완료 후 호출되어 직전 pass 완료 로그도 출력. 더 진행 불가
        (single-pass / 사용자 종료 / max_passes)면 False."""
        st = self._st
        s = self.s
        if st.n_pass >= s.max_passes:
            return False
        # prompt_between_passes=False → flip(flip)는 사람 개입 전제라 진행 불가.
        if not s.prompt_between_passes:
            st.aborted_reason = "single-pass mode (no inter-pass prompt)"
            return False
        print(f"\n  ✓ 뒤집기 {st.pose_idx}번째 수집 완료 "
              f"— 밴드 {st.bands_done}/{st.bands_total} · frames={st.last_n_frames}")
        next_pose = st.pose_idx + 1
        # 세장형 물체 권고 — sim 과 같은 정책(utils/nbv/flip_policy). 실제 뒤집기는
        # 사람이 하므로 **안내만** 한다: 설정된 pose 목록에 90°(눕히기)가 없다면
        # 끝면이 남는다는 것을 사용자에게 알려 준다.
        dims = getattr(self, "_obj_dims", None)
        if dims is not None:
            from utils.nbv.flip_policy import flip_angles_for, describe_flip
            angles, aspect = flip_angles_for(dims[0], dims[1])
            if len(angles) > 1:
                print(f"  ★ 세장형 물체(종횡비 {aspect:.1f}) — 권장 순서: "
                      + " → ".join(describe_flip(a) for a in angles))
        # ★ 뒤집기마다 **따로** 묻는다 — "90° 할래?" → [n] 이면 "180° 할래?" 로.
        #   예전 [n] 은 남은 뒤집기 전부 건너뛰기였다(2026-09-21 요청으로 변경).
        n_flips = max(0, len(s.pose_physical_rotations) - 1)   # pose 0 = 원래 자세
        while next_pose < len(s.pose_physical_rotations):
            print(f"\n  ── 뒤집기 {next_pose}/{n_flips} — 할까? ──")
            self._print_pose_hint_for_user(next_pose)
            print("  [Enter] 이렇게 뒤집어 놓았다, 계속    [n] 이 뒤집기는 건너뛴다    "
                  "[q] 종료    (키 하나만 누르면 된다)")
            key = self._read_key("nq")
            if key == "q":
                st.user_quit = True
                st.aborted_reason = "사용자 종료 (뒤집기 프롬프트)"
                return False
            if key == "n":
                print(f"  → 뒤집기 {next_pose} 건너뜀")
                next_pose += 1
                continue
            st.pose_idx = next_pose             # 이 자세의 힌트(R_phys)로 정합
            return True
        st.aborted_reason = st.aborted_reason or f"정의된 뒤집기 {n_flips}회 소진"
        print(f"  → 더 할 뒤집기가 없다 — 마무리로")
        return False

    # ── 회전 1회 (lookaround·flip 공유) ───────────────────────────────────────

    def _do_one_rotation(self, st: "_RunState", stage: str = "lookaround") -> str:
        """회전 1회: streaming → cleanup → hint → merge → viewer → master_center,
        그리고 tracking-lost recovery. lookaround·flip 이 공유한다.

        Returns _ROT_OK(정상), _ROT_RETRY(같은 pose 재시도), _ROT_ABORT(중단).
        """
        s = self.s
        # 배너가 어느 단계의 캡처인지 말하게 한다 (lookaround / flip).
        s.streaming_settings.stage_label = str(stage)
        self._print_pass_banner(st.n_pass + 1, s.max_passes, st.pose_idx)
        # ★ master IModel 의 좌표계는 **첫 회전의 녹화 시작 시점** 카메라 프레임이다.
        #   그 T_CB 를 여기서 고정한다. 이후 밴드 이동·NBV 자세마다 `self._T_CB` 가
        #   갱신되므로, 나중에 그걸로 마스터를 B 로 옮기면 그 사이 카메라가 움직인
        #   만큼 통째로 밀린다 — 2026-09-16 실물: nbv 조준점이 물체보다
        #   한참 위로 떠서 허공을 스캔했다.
        if st.master_T_CB is None and self._T_CB is not None:
            st.master_T_CB = np.asarray(self._T_CB, float).copy()
        if st.live_viewer is not None:
            try:
                st.live_viewer.new_pass(
                    f"scan {st.n_pass + 1} (뒤집기 {st.pose_idx})")
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
            # ★ `st.band_poses` 가 있으면 **한 IScan 안에서** 밴드마다 전회전한다.
            #   세션을 열어둔 채 로봇만 옮기므로 SLAM 추적이 이어지고, 밴드끼리
            #   후처리 GlobalRegistration 에 의존하지 않는다.
            #   이동은 느린 속도로 — 녹화 중이라 급하게 움직이면 추적을 잃는다.
            single = ArtecStreamingScanSession(
                self.mms, self.robot, self.turntable, s.streaming_settings,
                live_viewer=st.live_viewer,
                band_poses=st.band_poses,
                move_robot_fn=(
                    (lambda q: self._move_robot_to_q(
                        q, s.band_move_speed_deg_s))
                    if st.band_poses else None),
                # ★ 거리추종 — 밴드 시작에서 표면거리를 재고 축거리만 고친다.
                #   sim 과 **같은 컨트롤러**(`utils/nbv/standoff.StandoffTracker`)를
                #   쓰고, 여기서는 "그 밴드를 새 거리로 다시 푸는 법" 만 준다.
                retarget_fn=(self._band_retarget if st.band_poses else None),
                standoff_of=(self._band_standoff_of if st.band_poses else None),
                scan_range=self._scanning_range_m(),
                # ★ 밴드 전환은 조준 유지 소보간 + lost 시 되돌아가기(relocalization).
                band_path_fn=(self._band_path if st.band_poses else None),
            )
            # ★ 스캔 시작 시점의 turntable 논리각 — clearpos 를 건너뛰는 경우
            #   (recovery safe-back) 물체가 이 각만큼 돌아간 채 찍히므로, 병합
            #   보정(`_compose_T_pre_W_mm`)에 넣어야 한다. clearpos 경로면 0.
            st.last_scan_theta0 = (self._read_turntable_theta()
                                   if st.next_skip_clearpos else 0.0)
            sub_result = single.run()
            for _d, _p in (getattr(sub_result, "band_standoff_m", None) or []):
                if _d and _p and 0.0 < (_d - _p) < 0.20:
                    st.r_eff_hist.append(float(_d - _p))
            if getattr(sub_result, "band_standoff_m", None):
                print(f"  [거리추종] 밴드 실측 반경 r_eff=d−p: "
                      + ", ".join(f"{(d - p)*1000:.0f}mm" for d, p in sub_result.band_standoff_m)
                      + f"  (누적 중앙값 {np.median(st.r_eff_hist)*1000:.0f}mm)" if st.r_eff_hist else "")
        finally:
            # streaming_settings 는 multipass 인스턴스 외부에서 공유될 수
            # 있으므로 반드시 원복.
            s.streaming_settings.reset_to_zero_first = _orig_reset_to_zero
            st.next_skip_clearpos = False
        st.pass_results.append(sub_result)
        st.n_pass += 1
        st.last_n_frames = sub_result.n_frames
        st.bands_done += int(getattr(sub_result, "n_bands_done", 0) or 0)
        st.bands_total += int(getattr(sub_result, "n_bands", 0) or 0)

        # ── Pass cleanup: SerialReg + OutlierRemoval ───────────────
        # 멀티패스 끝까지 기다리지 말고 회전마다 즉시 정합/이상점 제거.
        # 이유: (a) hint 계산 (centroid) 이 깨끗한 데이터로 안정,
        #       (b) live viewer 가 정합된 IScan 을 그대로 비춰서 사용자가
        #           회전별 형상 / 정합 품질을 즉시 검증 가능
        #           ([[feedback_live_viewer_must_mirror_scan]]).
        # Outliers 는 Fusion 전에 와야 함 ([[feedback_artec_pipeline_order]]).
        # tracking_lost 면 partial IScan 이라 SerialReg 가 깨질 수 있어 skip.
        # ★ lost 로 끝난 IScan 도 **완주한 밴드는 멀쩡한 데이터**다. 예전엔 tracking_lost 면
        #   정리를 통째로 건너뛰어(2026-09-22 run_132655: 정리 0회) 완주 밴드 2개가
        #   SerialReg 없이 병합됐다. 꼬리의 미정합 프레임만 잘라내고 정리한다.
        if sub_result.model.scan_count() > 0:
            _nb_done = int(getattr(sub_result, "n_bands_done", 0) or 0)
            _usable = (not sub_result.tracking_lost) or _nb_done > 0 or sub_result.frames_ok >= 30
            if _usable:
                sub_result.model = self._trim_lost_tail(
                    sub_result.model, int(getattr(sub_result, "n_tail_lost", 0) or 0))
                sub_result.model = self._cleanup_model(
                    sub_result.model, "정리", outliers=self.s.pass_outlier_removal)
            else:
                print("  [정리] 쓸 만한 프레임이 없어 정리 생략")

        # ── Pose hint 계산 (centroid-aware + base-frame aware) ─────
        # 1. R_phys 는 base frame B 에서 정의 (사용자 직관)
        # 2. T_BC 있으면 R_W = T_BC @ R_B @ T_CB 로 scan world 로 변환
        # 3. T_pre = Translate(c_master) @ inv(R_W) @ Translate(-c_pass)
        #    객체 centroid 를 pivot 으로 회전 → camera 원점 기준 30cm
        #    translation 오류 제거.
        T_pre = None
        st.merge_method = "none"

        # Recovery override: 직전 회전에서 robot 이 움직였다면 camera-motion
        # 만 보정 (object 회전 무관, pose_idx 그대로). 다른 R_phys hint 보다 우선.
        #   x_W_master = T_BC_master @ T_CB_new @ x_W_new
        #              = T_BC_master @ inv(T_BC_new) @ x_W_new
        # SDK frame_transformation 은 mm 라 translation 만 m→mm scale.
        R_phys = (s.pose_physical_rotations[st.pose_idx]
                  if st.pose_idx < len(s.pose_physical_rotations) else None)
        _is_flip = (R_phys is not None
                    and not np.allclose(R_phys, np.eye(4), atol=1e-9))
        if st.next_T_BC_pending is not None:
            # ★ 공통 헬퍼(`_compose_T_pre_W_mm`) — master 스냅샷 기준 + 물체 회전
            #   θ0(clearpos 를 건너뛴 recovery 는 safe-back 각에서 시작) + S 켤레.
            #   예전 공식(self._T_BC @ inv(pending))은 recovery 가 self._T_BC 를
            #   덮어써 T_pre ≈ I 로 무효였고, θ0·S 도 빠져 있었다.
            # ★ flip 밴드 캡처(2026-09-21)는 **카메라 이동 + 뒤집기 되돌리기**가
            #   동시에 필요하다. 예전엔 둘 중 하나만 걸렸다(이동 보정이 있으면
            #   뒤집기 힌트를 건너뜀) — flip 이 home 고정이었을 때는 문제가 없었지만
            #   flip 도 preview→밴드 계획으로 로봇이 움직이면서 필요해졌다.
            if st.master_T_CB is None and self._T_BC is None:
                print(f"  [recovery hint] ⚠ T_BC_master 미설정 — override skip")
            else:
                try:
                    T_unflip = None
                    st.merge_method = "camera"
                    # ★ dump 용 — 이 T_BC 가 T_pre 를 만든 값이다. 아래에서
                    #   `next_T_BC_pending = None` 이 되므로 여기서 붙잡아 둔다
                    #   (2026-09-22: dump 가 낡은 self._T_BC 를 저장해 오프라인
                    #    재현이 70~130mm 어긋났다 — flip 정합 분석이 막혔다).
                    _T_BC_used = np.asarray(st.next_T_BC_pending, float).copy()
                    if _is_flip:
                        c_pass = self._compute_model_centroid(sub_result.model)
                        c_ref = (st.master_center if st.master_center is not None
                                 else c_pass)
                        T_unflip = self._flip_unflip_B(
                            R_phys, st.next_T_BC_pending, st.last_scan_theta0,
                            c_pass, c_ref)
                        st.hints_applied = True
                        print(f"\n  [hint flip {st.pose_idx}] 뒤집기 되돌리기(B) "
                              f"+ 카메라 이동 보정 합성 (힌트)")
                        # ★ 힌트는 초기값일 뿐 — 텍스처 매칭 → 기하 전역 정합 순으로 교체.
                        _fr, _T, st.merge_method = self._flip_global_refine(
                            sub_result.model, st.next_T_BC_pending,
                            st.last_scan_theta0, T_unflip)
                        if _fr == "W":
                            T_pre = np.asarray(_T, float)      # 스캔월드→master월드 (mm) 직접
                            T_unflip = None
                        else:
                            T_unflip = _T
                    if T_pre is None:
                        T_pre = self._compose_T_pre_W_mm(
                            st.next_T_BC_pending, st.last_scan_theta0,
                            tag="camera-motion", T_extra_B=T_unflip)
                    print(f"\n  [camera-motion] 이동 보정 적용 "
                          f"(Δtrans = ({T_pre[0,3]:+.1f}, "
                          f"{T_pre[1,3]:+.1f}, {T_pre[2,3]:+.1f}) mm) "
                          f"— master 프레임으로 정렬")
                except np.linalg.LinAlgError as e:
                    print(f"  [recovery hint] ⚠ inv 실패 ({e}) — override skip")
            st.next_T_BC_pending = None

        if T_pre is None and _is_flip:
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
                st.merge_method = "hint"
                print(f"\n  [hint flip {st.pose_idx}] frame={frame_tag}")
                print(f"  [hint flip {st.pose_idx}] c_pass = ({c_pass[0]:+.1f}, "
                      f"{c_pass[1]:+.1f}, {c_pass[2]:+.1f}) mm")
                print(f"  [hint flip {st.pose_idx}] c_master = ({c_ref[0]:+.1f}, "
                      f"{c_ref[1]:+.1f}, {c_ref[2]:+.1f}) mm")
                print(f"  [hint flip {st.pose_idx}] translation = ({T_pre[0,3]:+.1f}, "
                      f"{T_pre[1,3]:+.1f}, {T_pre[2,3]:+.1f}) mm")

        # Sub-model 의 IScan 들 → master_model.
        # apply_hints_to_frame_transformations=False 일 때 hint 는 IScan 의
        # frame_transformations 에 박지 않고 recorded_hints 에 기록만 (병합
        # 비교용). 같은 raw scan 데이터로 후처리 단계에서 hint on/off 를
        # swap 가능.
        # ★ 원시 스캔 덤프 — 정합을 **오프라인에서 다시 돌릴 수 있게** 스캔 월드 점과
        #   적용 T_pre 를 남긴다 (2026-09-21: flip 정합이 나빴는데 재현할 데이터가 없었다).
        self._dump_scan_raw(sub_result.model, T_pre, stage,
                            theta0=st.last_scan_theta0,
                            T_BC_used=locals().get("_T_BC_used"),
                            R_phys=(R_phys if _is_flip else None),
                            tracking_lost=bool(sub_result.tracking_lost),
                            n_frames=int(sub_result.n_frames))
        # ★ 빈 IScan 은 master 에 넣지 않는다 (nbv 패치와 같은 게이트, 2026-09-22 run_162620).
        #   밴드 4(빈 시야, 2,021점)와 flip 재시도 2개(29·31프레임, 2~3천점)가 그대로 master 에
        #   들어가 Studio 에서 물체 위 100~200mm 허공의 노이즈 구름으로 보였다. dump 는 남긴다.
        _empty = False
        try:
            _T_CB_g = self._T_CB if self._T_CB is not None else np.linalg.inv(self._T_BC)
            _pcd_g = self._master_to_pcd_B(sub_result.model, _T_CB_g, 4.0,
                                           T_sc_mm=getattr(self, "_T_scan_color", None))
            _n_pts_g = 0 if _pcd_g is None else len(_pcd_g.points)
        except Exception:                                        # noqa: BLE001
            _n_pts_g = -1
        _ok_frames_g = int(getattr(sub_result, "frames_ok", 0) or 0)
        if 0 <= _n_pts_g < self.s.nbv_min_patch_pts or _ok_frames_g < self.s.nbv_min_patch_frames:
            _empty = True
            print(f"  [병합] ✘ 빈 IScan — 점 {_n_pts_g:,}(4mm 복셀) · OK 프레임 {_ok_frames_g} "
                  f"(기준 {self.s.nbv_min_patch_pts:,}점·{self.s.nbv_min_patch_frames}프레임) → master 에 안 넣음")
            st.merge_method = "rejected(empty)"
        _ev("merge", stage=stage, pose_idx=st.pose_idx,
            method=getattr(st, "merge_method", "hint" if T_pre is not None else "none"),
            n_scans_before=st.master_model.scan_count(),
            tracking_lost=bool(sub_result.tracking_lost), n_pts=_n_pts_g, frames_ok=_ok_frames_g)
        if _empty:
            pass
        elif T_pre is not None and not s.apply_hints_to_frame_transformations:
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
            if (T_pre is not None and st.band_poses and not _is_flip
                    and st.master_model.scan_count() > 0):
                # 밴드 이어붙이기 — 인접 밴드 겹침이 커서 ICP 가 잘 조건화된다. 게이트에
                # 걸리면 기구학 배치 그대로.
                T_ref, fit, rmse = self._hint_icp_refine(
                    sub_result.model, T_pre, st.master_model, mode="patch")
                if T_ref is not T_pre:
                    print(f"  [band icp] 기구학 초기값 → ICP 보정 적용 "
                          f"(fitness {fit:.2f}, RMSE {rmse:.2f}mm)")
                    T_pre_to_apply = T_ref
                    st.merge_method = "camera+icp"
                else:
                    print("  [band icp] 게이트 기각 — 기구학 배치 유지")
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
        _nb_done = int(getattr(sub_result, "n_bands_done", 0) or 0)
        _nb_all = int(getattr(sub_result, "n_bands", 0) or 0)
        print(f"\n  [병합] 밴드 {_nb_done}/{_nb_all} → master "
              f"(누적 scan {st.master_model.scan_count()} · "
              f"밴드 {st.bands_done}/{st.bands_total})")

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
            # ★ 밴드 스윕에서 일부 밴드를 이미 끝냈으면 recovery 를 돌리지 않는다.
            #   `capture_bands` 가 어차피 "부분 성공 → 재시도 없이 nbv" 로 처리하는데,
            #   그 앞에서 probe·후보 탐색·home 복귀에 30s 를 쓰고 결과도 버렸다
            #   (2026-09-22 run_132655). 0 밴드면 첫 자세부터 틀린 것이라 recovery 가 맞다.
            _nb_done = int(getattr(sub_result, "n_bands_done", 0) or 0)
            if st.band_poses:
                # ★ 밴드 계획이 있으면 0 밴드여도 probe 기반 recovery 를 쓰지 않는다
                #   (2026-09-22 run_154059: probe 가 물체를 h=42mm 로 오판해 빈 시야
                #   자세로 옮겼고 이후 재시도 3회가 전부 그 자세에서 즉시 lost).
                #   계획된 밴드 자세는 preview 로 검증된 자세다 — `capture_bands` 가
                #   그 자세로 되돌아가 새 IScan 으로 재시도한다(최대 2회, 그 뒤 nbv 몫).
                print(f"  ⓘ 밴드 {_nb_done}/{len(st.band_poses)} 완주 — probe recovery 생략, "
                      f"계획 자세에서 새 IScan 으로 이어간다")
                _ev("recovery", stage=stage, attempt=0, ok=False, pose_changed=False,
                    reason=("skipped: partial band success" if _nb_done > 0
                            else "skipped: planned bands (retry at planned pose)"))
                return _ROT_RETRY
            if (s.auto_recovery_enabled
                    and st.recovery_retry_count < s.max_recovery_retries):
                ok, T_BC_new = self._attempt_recovery(
                    sub_result, st.master_model, st.recovery_retry_count,
                )
                _ev("recovery", stage=stage, attempt=st.recovery_retry_count + 1,
                    ok=bool(ok), pose_changed=(T_BC_new is not None),
                    reason=str(sub_result.loss_reason)[:80])
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
            print(f"\n  ⚠ tracking lost: {sub_result.loss_reason}")
            print(f"  → 물체는 그대로 두고  [Enter] 재시도    [q] 종료    (키 하나만)")
            if self._wait_user_quit():
                st.user_quit = True
                st.aborted_reason = "사용자 종료 (lost 직후)"
                return _ROT_ABORT
            # User 가 수동 retry 한 경우 — recovery_retry_count 는 reset
            # (수동 개입은 새 시작으로 간주).
            st.recovery_retry_count = 0
            return _ROT_RETRY       # 같은 pose 재시도

        # 정상 완료 — recovery 카운터 reset.
        _ev("rotation_ok", stage=stage, pose_idx=st.pose_idx,
            after_recovery=int(st.recovery_retry_count),
            bands_done=int(getattr(sub_result, "n_bands_done", 0) or 0),
            n_bands=int(getattr(sub_result, "n_bands", 1) or 1))
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
                        max_frames: int = 50, T_sc_mm=None):
        """단일 IScan 의 frame vertices+colors 를 B 프레임 Open3D PointCloud 로
        변환 (voxel downsample). T_CB_master = 첫 pass C→B (translation m).

        IScan vertices 는 mm, scan-world W = **3D 스캐너 프레임**(첫 프레임).
        ⚠ W ≈ C 가 **아니다** — Artec 3D 정점과 Color 카메라 프레임은 180° 회전
          + 41.8mm 강체변환으로 어긋난다([[project_artec_3d_vs_color_frame]]).
          hand-eye `T_CB` 는 Color(솔브PnP) 기준이므로 T_sc 없이 곧장 C→B 를
          걸면 점군이 카메라 반대편(≈2×작동거리 ≈ 0.5~0.6m)으로 미러링된다 —
          2026-09-16 실물: nbv 메시가 물체보다 ~0.5m 위에 떠서 NBV 가
          허공을 조준했다. preview 경로(`_capture_preview_verts(to_color=True)`)
          는 이 변환을 하는데 이 경로만 빠뜨렸던 것.

        T_sc_mm : (4,4) 스캐너3D→Color (mm), `_scanner_to_color()` 캐시.
                  None 이면 변환 없이 진행(호출측이 경고할 것).
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
        R_sc = t_sc = None
        if T_sc_mm is not None:
            R_sc = np.asarray(T_sc_mm[:3, :3], float)
            t_sc = np.asarray(T_sc_mm[:3, 3], float)
        all_v, all_c = [], []
        for i in idxs:
            frame = scan.get_frame(i)
            v = frame.vertices()
            if v is None or v.shape[0] == 0:
                continue
            T_f = scan.get_frame_transformation(i)
            v_W = v @ np.asarray(T_f[:3, :3], float).T + np.asarray(T_f[:3, 3], float)
            if R_sc is not None:                # 스캐너3D → Color (mm)
                v_W = v_W @ R_sc.T + t_sc
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
                         voxel_mm: float, max_frames_per_scan: int = 30,
                         T_sc_mm=None):
        """master IModel 의 모든 IScan 을 B 프레임 colored PCD 로 합치기.
        T_sc_mm = 스캐너3D→Color (mm) — `_iscan_to_pcd_B` 참조."""
        try:
            import open3d as o3d
        except Exception:
            return None
        merged = o3d.geometry.PointCloud()
        for s in range(master_model.scan_count()):
            scan = master_model.get_scan(s)
            pcd = ArtecMultiPassScanSession._iscan_to_pcd_B(
                scan, T_CB_master, voxel_mm, max_frames=max_frames_per_scan,
                T_sc_mm=T_sc_mm)
            if pcd is not None:
                merged += pcd
        if len(merged.points) == 0:
            return None
        if voxel_mm > 0:
            merged = merged.voxel_down_sample(voxel_mm)
        return merged

    # ─────────────────────────────────────────────────────────────────────
    # nbv — 부족면 NBV 보강 루프 (docs/4_nbv.md §3). stage_until="nbv".
    # 하드웨어 무관 코어는 utils/nbv/nbv_core.py + theta_planner(해석 IK).
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
        # ★ base 좌표는 `tt.axis_point_B/axis_dir_B` 로 받는다. T_BF0 의 열을
        #   그대로 쓰면 안 된다 — B→F 규약이라 base 좌표가 아니다
        #   (2026-09-16 EE-base 충돌 원인. `utils/transforms.py` 접근자 주석 참고).
        axis_pt_B, axis_dir_B = tt.axis_point_B, tt.axis_dir_B
        world = CollisionWorld.from_turntable(
            surface_point=axis_pt_B, axis_dir=axis_dir_B,
            disc_radius=self.s.nbv_turntable_radius_mm / 1000.0,
            body_height=self.s.nbv_turntable_body_height_mm / 1000.0,
            margin=self.s.nbv_collision_margin_mm / 1000.0)
        print(f"  [nbv] 충돌 world 구성 (disc r={self.s.nbv_turntable_radius_mm:.0f}mm, "
              f"margin={self.s.nbv_collision_margin_mm:.0f}mm)")
        # ── 충돌 금지 원기둥 (turntable 위 금지구역) ──
        if self.s.nbv_keepout_enable:
            cxy = (self.s.nbv_keepout_center_xy
                   if self.s.nbv_keepout_center_xy is not None else axis_pt_B[:2])
            R = (self.s.nbv_keepout_radius_mm / 1000.0
                 if self.s.nbv_keepout_radius_mm is not None
                 else self.s.nbv_turntable_radius_mm / 1000.0)  # 기본=턴테이블 지름
            z0 = float(axis_pt_B[2])                            # disc 표면
            world.add_cylinder("keepout", cxy, z0, z0 + self.s.nbv_keepout_height_mm / 1000.0,
                               R, margin=self.s.nbv_collision_margin_mm / 1000.0)
            print(f"  [nbv] keep-out 원기둥 추가 (r={R*1000:.0f}mm, "
                  f"h={self.s.nbv_keepout_height_mm:.0f}mm)")
        return world

    def _build_master_mesh_B(self, master_model):
        """master IModel → B 프레임 pcd → Poisson mesh (frontier 입력). **단위 = m.**

        ★ `_master_to_pcd_B` 는 Artec 정점 그대로 **mm** 를 돌려준다(ICP 병합
          경로가 mm 를 요구한다). 반면 이 메시를 먹는 하류는 전부 **m** 다:
            · `utils/nbv/nbv_core.detect_gaps(min/max_seg_length)` — m
              (`_gap_kw()` 가 mm 설정을 /1000 해서 넘긴다)
            · `nbv_core.is_converged(boundary_stop_m)` — m
            · `_plan_nbv_pose` 의 `look_target` — base 프레임 m (`axis_xy_B` 와 조합)
          sim(`isaac_scan_session.build_coverage_mesh`)도 m 로 넘긴다 — 공용
          `nbv_core` 를 쓰므로 단위가 같아야 한다.

          단위를 안 맞췄을 때의 증상 (실측 2026-09-16, run_20260916_211701):
            · 세그먼트 길이가 mm 스케일 → 전부 `max_seg_length=0.06` 초과로
              걸러져 `gaps=0`, 그런데 `boundary=943019mm` — 모순된 로그.
            · `look_target` z 가 mm 값(−290)을 m 로 해석 → 조준점이 base 에서
              290m → IK 전부 실패 → "feasible 관측자세 없음" 으로 nbv 즉시 종료.
        """
        from utils.nbv import nbv_core as _p2
        # ★ **마스터 프레임의** T_CB 를 쓴다 (`self._T_CB` 는 현재 자세라 틀림).
        T_CB_master = self._st.master_T_CB
        if T_CB_master is None:
            T_CB_master = self._T_CB
        # ★ 스캐너3D→Color 변환 — 없으면 점군이 ~0.5m 미러링된다. lookaround 계획의
        #   preview 경로가 풀어서 캐시해 두므로(`_capture_preview_verts(to_color=
        #   True)` → `_scanner_to_color`) 정상 플로우면 항상 있다.
        T_sc = getattr(self, "_T_scan_color", None)
        if T_sc is None:
            print("  [nbv] ⚠ 스캐너3D→Color 변환 미캐시 — 메시가 카메라 반대편으로 "
                  "미러링될 수 있다 (preview 경로가 안 돌았나?)")
        pcd = self._master_to_pcd_B(
            master_model, T_CB_master, voxel_mm=self.s.nbv_master_voxel_mm,
            T_sc_mm=T_sc)
        if pcd is None or len(pcd.points) < 200:
            return None
        pcd = pcd.scale(0.001, center=(0.0, 0.0, 0.0))     # mm → m
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
        # ★ el 의 기준축(up)을 **명시**한다. base 프레임을 world 처럼 넘기는데
        #   (T_WB=None) 이 셀의 base 는 천장 마운트라 **+Z 가 아래**다
        #   (`docs/collision.md` §6.1). 기본값 (0,0,1) 을 쓰면 el=+30° 가 카메라를
        #   원판 **아래**로 보내 도달 불가가 된다 — 2026-09-16 실물에서 eye 가
        #   base 로부터 1.38m(반경 0.70m)에 찍혀 3방위 IK 가 전부 실패했다.
        #   sim 은 world 프레임에서 풀어 +Z 가 위였으므로 이 버그가 안 드러났다.
        # ★ az=0 을 **로봇 쪽**으로 잡는다. 방위각은 커버리지와 무관하고(회전은
        #   턴테이블 담당) 도달성만 좌우하는데, 이 셀은 턴테이블이 base 에서
        #   1.09m 인데 팔 반경은 0.70m 라 카메라를 base 쪽에 놓아야만 닿는다.
        #   2026-09-16 실측: 기본 기준(+X)에서는 az 0/±30° 가 전부 도달 불가였고
        #   105~270° 만 해가 있었다(180° 최적). 기준을 돌리면 기존 스윕
        #   (0, ±30) 이 그대로 유효 구간에 들어온다.
        up = self._view_up_B()
        az_ref = -np.asarray(look_target, float)     # 타깃 → base 원점
        q, _roll, _eye = _vp.solve_view_q(
            _kin, look_target, el_deg, az_deg, standoff, q_seed, self._T_EC,
            T_WB=None,                       # look_target 이 이미 base 프레임
            convention=_vp.CAM_OPENCV, rolls_deg=rolls, up=up, world_up=up,
            az_ref=az_ref)
        self._last_roll = _roll              # 채택된 roll — 계획 로그·visited 용
        return q

    def _view_up_B(self):
        """base 프레임에서 '위'(= 카메라가 놓이는 쪽) 단위벡터.

        턴테이블 축 방향을 **로봇 쪽**으로 뒤집은 것이다. `build_T_B_F0` 가 F 의 +z 를
        `nz[2] > 0` 으로 정규화하므로(= base 기준 +Z, 천장 마운트에서 '아래'),
        관측 자세를 만들 때는 그 반대 방향이 필요하다.
        캘리브가 없으면 base 기준 '위' = −Z 로 둔다.
        """
        tt = getattr(self.mms, "turntable_transform", None)
        if tt is not None and hasattr(tt, "axis_dir_B"):
            d = np.asarray(tt.axis_dir_B, float)
            # base 원점(로봇) 쪽을 향하게
            return -d if float(d[2]) > 0 else d
        return np.array([0.0, 0.0, -1.0])

    def _plan_nbv_pose(self, q_cur, gaps, mesh):
        """nbv 관측자세 — 판단은 **sim·real 공용** `utils/nbv/nbv_planner` 가 한다.

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
        # ★ **함수 머리에서** 임포트한다. 아래 `roll_order` 가 `_vp` 를 참조하는데,
        #   예전엔 이 임포트가 ② gap 겨냥 절(節) 안에 있었다. ① 축-고도각 경로가
        #   먼저 타면 `roll_order` 호출 시점에 `_vp` 가 아직 바인딩되지 않아
        #   NameError 로 파이프라인이 죽었다 (2026-09-21 실물, nbv 진입 직후).
        #   함수 지역 임포트를 중간에 두면 그 앞에서 쓰는 클로저가 조용히 깨진다.
        from utils.robot import view_pose as _vp
        from utils.robot import xarm7_kinematics as _kin

        tt = getattr(self.mms, "turntable_transform", None)
        T_BF0 = getattr(tt, "T_BF0", None) if tt is not None else None
        if T_BF0 is None:
            return None
        # ★ 회전축의 base xy 는 `tt.axis_xy_B`. T_BF0 의 열이 아니다 —
        #   그걸 조준 타깃으로 쓰면 로봇이 자기 base 를 본다(2026-09-16 충돌).
        axis_xy_B = tt.axis_xy_B
        verts = np.asarray(mesh.vertices)                  # 메쉬(B, m) mid z = 객체 중간높이
        look_target = np.array([axis_xy_B[0], axis_xy_B[1], float(verts[:, 2].mean())])
        # ★ `_axis_view_q` 는 **축까지의 거리**를 받는다. `nbv_distance_mm` 은
        #   카메라↔**표면** 거리이므로 물체 반경을 더해야 한다.
        #   예전엔 표면 거리를 축 거리로 그냥 넘겼다 — r=97mm 물체면 표면까지
        #   225−97 = 128mm 로 Spider 근접한계(170mm) 안쪽이라 데이터가 안 나온다.
        #   (같은 상수가 `plan_frontier`/`nbv_pose_from_candidate` 에는 표면 거리로
        #    넘어가고 있었다 — 한 값이 두 뜻으로 쓰이던 것을 갈랐다.)
        #   ★ 반경은 **preview 점군**에서 잰다(p95). master 메시로 재면 안 된다 —
        #     2026-09-21 실물(run_145013): preview 는 r p95=67mm 로 깨끗했는데
        #     master 메시는 nbv 진입 시점에 이미 폭 326mm(p95 142mm → standoff
        #     367mm), 이후 정합 없이 붙는 nbv 패치가 쌓이며 421mm·높이 380mm 까지
        #     부풀었다. 190mm 병이 그럴 수 없다 — 메시는 반경 측정원으로 못 쓴다.
        #     preview 가 없을 때만 메시로 폴백하되, 디스크면 10mm 위의 점만 센다.
        _r_obj, _r_src = self._object_radius_m(axis_xy_B, verts)
        standoff = _axis_standoff(_r_obj, self.s.nbv_distance_mm / 1000.0)
        # ★ 조준점을 **찍는다**. 이 한 줄이 없어서 단위/프레임 오류(mm 를 m 로,
        #   현재 T_CB 를 마스터 T_CB 로)가 "허공을 스캔" 으로만 드러났다.
        #   디스크 상단 z 와 나란히 보이면 바로 이상을 알 수 있다.
        _disc_z = float(np.asarray(tt.axis_point_B, float)[2])
        print(f"  [nbv] 조준 target=({look_target[0]:+.3f}, {look_target[1]:+.3f}, "
              f"{look_target[2]:+.3f})m  standoff={standoff*1000:.0f}mm "
              f"(표면 {self.s.nbv_distance_mm:.0f} + 반경 {_r_obj*1000:.0f}, {_r_src})  "
              f"| 디스크상단 z={_disc_z:+.3f}m  메시 z={verts[:, 2].min():+.3f}"
              f"..{verts[:, 2].max():+.3f}m")

        if getattr(self, "_nbv", None) is None:
            # visited 를 pass 사이에 유지해야 하므로 세션에 1회만 만든다.
            # ★ up_sign — base 가 천장 마운트라 '위' = −Z (`docs/collision.md`
            #   §6.1, lookaround 플래너와 같은 유도식). 안 넘기면 gap 의 윗면/아랫면
            #   분류가 뒤집혀 뚜껑 gap 이 'flip flip 필요'로 제외된다
            #   (2026-09-16 실물).
            self._nbv = NbvPlanner(joint_weights=DEFAULT_JOINT_WEIGHTS,
                                   el_floor_deg=self.s.nbv_el_floor_deg,
                                   view_azis_deg=tuple(self.s.nbv_view_azis_deg),
                                   ensure_els=tuple(self.s.nbv_ensure_els_deg),
                                   log=lambda m: print(f"  [nbv] {m}"),
                                   up_sign=(-1.0 if float(self._view_up_B()[2]) < 0
                                            else +1.0))

        _mesh_v = np.asarray(mesh.vertices)                 # (B, m)
        if len(_mesh_v) > 3000:
            _mesh_v = _mesh_v[np.random.default_rng(0).choice(
                len(_mesh_v), 3000, replace=False)]
        # ★ 가드 점군 = 메시 ∪ preview ∪ master 점. 메시만 쓰면 preview 가 놓친 윗부분
        #   (뚜껑)이 빠져 카메라가 그 위로 파고든다 — run_132655 의 gap 겨냥 6회가 전부
        #   뚜껑에서 161~218mm(근접한계 170) 에 놓여 빈 캡처였다.
        _parts = [_mesh_v]
        _pv = (getattr(self, "preview_result", None) or {}).get("points_B")
        if _pv is not None and len(_pv):
            _parts.append(np.asarray(_pv, float))
        _mp = self._master_pts_B_m(20_000)
        if len(_mp):
            _parts.append(_mp)
        _guard_pts = np.vstack(_parts)
        if len(_guard_pts) > 6000:
            _guard_pts = _guard_pts[np.random.default_rng(1).choice(len(_guard_pts), 6000, replace=False)]
        # 근접한계는 스캐너가 말하는 값 + 40mm (기존 EYE_CLEAR_M=150mm 는 한계 아래였다)
        _eye_clear = float(self._scanning_range_m()[0]) + 0.04

        def solve_pose(el, az, rolls):
            # ★ 근접한계 가드 — 폴백(축-고도각)·윗면 보장 자세도 물체 최근점에서
            #   near+40mm 는 떨어져야 한다. 2026-09-22 run_132655: 윗면 보장 자세가
            #   뚜껑에서 183mm(근접한계 170) 라 3,458점뿐이었다.
            eye = _vp.eye_from_el_az(look_target, el, az, standoff, up=self._view_up_B())
            if _guard_pts is not None and len(_guard_pts):
                dmin = float(np.min(np.linalg.norm(_guard_pts - np.asarray(eye, float)[None, :], axis=1)))
                if dmin < _eye_clear:
                    return None, None
            q = self._axis_view_q(look_target, el, az, standoff, q_cur, rolls=rolls)
            return q, getattr(self, "_last_roll", None)

        def roll_order(el, az, gs):
            """gap 이 늘어선 방향에 FOV 넓은 축을 맞추는 roll 순서 — sim 과 같은
            공용 `view_pose.roll_order_for_gaps`. 규약만 OpenCV. 2026-09-18 까지
            real 은 이걸 안 넘겨 roll=0 부터 돌았다(nbv_planner 머리 격차표)."""
            rolls = (getattr(self.s, "view_rolls_deg", None) or _vp.DEFAULT_ROLLS_DEG)
            if not gs:
                return tuple(rolls)
            up = self._view_up_B()
            eye = _vp.eye_from_el_az(look_target, el, az, standoff, up=up)
            return _vp.roll_order_for_gaps(
                [c.p_O for c in gs], [c.L for c in gs], eye, look_target,
                convention=_vp.CAM_OPENCV, rolls_deg=rolls, world_up=up)

        def swept(q0, q1):
            cm = self._collision_gate()
            if cm is not None:                 # 메시+SDF, 보수적 전진(터널링 불가)
                ok, why, _ = cm.is_path_safe(q0, q1)
                return ok, why
            world = getattr(self, "_collision_world", None)
            if world is None:
                return True, ""
            col, why, _s = swept_pose_collision(
                world, q0, q1, T_EC=self._T_EC, n_steps=self.s.nbv_swept_steps)
            return (not col), (why or "")

        # ── ① 보장 고도각 — 오목 내부는 미관측이라 gap 으로 안 잡힌다(닭·달걀).
        #    아직 안 가본 ensure_el 이 있으면 그 축-고도각 자세를 먼저 쓴다.
        if getattr(self, "_axis_pt_B", None) is not None:      # 바닥 근처 gap 은 모든 경로에서 flip 몫
            self._nbv.disc_z = float(self._axis_pt_B[2])
        need = [e for e in self._nbv.ensure_els
                if not any(abs(float(v[0]) - float(e)) < 1e-6
                           for v in self._nbv.visited)]
        # ★ 윗면 개구부(경계 고리)가 있을 때만 돈다 — sim 과 같은 공용 `needs_ensure`.
        #   없는 물체에서 55° 전회전은 실물 30초 낭비다(2026-09-18).
        if need:
            _ens, _uplen = _nbvp.needs_ensure(gaps, self._nbv.up_sign)
            if not _ens:
                print(f"  [nbv] 보장 고도각 {need} 건너뜀 — 윗면 향한 gap "
                      f"{_uplen*1000:.0f}mm < {_nbvp.ENSURE_MIN_UP_LEN_M*1000:.0f}mm (개구부 없음)")
                for e in need:
                    self._nbv.visited.append((float(e), 0.0))
                need = []
        if need:
            res0 = self._nbv.plan(gaps, q_cur, solve_pose, swept,
                                  roll_order_fn=roll_order)
            if res0 is not None:
                self._next_theta = None            # 전회전 (기존 동작)
                self._nbv_dbg_plan(gaps, None, "ensure", res0[0], look_target, 0.0, 360.0,
                                   mesh, note=f"ensure el={res0[1]:.0f}° az={res0[2]:.0f}°")
                return res0[0]

        # ── ② gap 직접 겨냥 (주경로) ────────────────────────────────────
        #    축-고도각은 카메라가 늘 턴테이블 축을 봐서 gap 위치를 통째로 버린다
        #    (gap 정보가 '법선 고도각 중앙값' 하나로 압축). 손잡이·컵 내벽 같은
        #    국소 결손은 원리적으로 못 겨냥한다 → 표면점 p 를 정면으로 본다.
        #    2026-09-10: sim 에만 있던 경로를 real 에 배선. 규약만 OpenCV 로 바뀐다.
        #    (`_vp`·`_kin` 은 함수 머리에서 임포트한다 — 위 주석 참조.)

        # ★ 물체 근접 가드는 **공용** `plan_frontier(obj_pts=, eye_clear_m=)` 가
        #   건다(2026-09-18 공용화). 근거는 "충돌 world 에 대상물이 없어서" 가
        #   **더는 아니다** — 2026-09-17 부터 대상물 회전체가 동적 장애물로 들어가
        #   30mm 마진으로 부딪힘은 막는다. 이 가드가 남는 이유는 다른 것이다:
        #   Spider 는 근접한계(170mm) 아래에선 데이터가 안 나오므로 150mm 안쪽
        #   후보는 안 부딪혀도 **찍히지 않는다**(2026-09-16 실물: 뚜껑 15mm 까지
        #   접근). 충돌 가드 = 부딪히지 않기, 이 가드 = 헛걸음하지 않기.

        def solve_lookat(eye, tgt):
            return _vp.solve_look_at_q(
                _kin, eye, tgt, q_cur, self._T_EC,
                T_WB=None,                          # look_target 이 이미 base 프레임
                convention=_vp.CAM_OPENCV,
                rolls_deg=(getattr(self.s, "view_rolls_deg", None)
                           or _vp.DEFAULT_ROLLS_DEG),
                # 스크리닝 예산 — sim 과 같은 공용값(근거는 nbv_planner 상수 주석)
                n_seed_alt=_nbvp.FRONTIER_IK_SEEDS,
                ik_max_iter=_nbvp.FRONTIER_IK_MAX_ITER)

        fr = None
        if self.s.nbv_frontier_enabled:
            # ★ '로봇 편한 방위' 는 **축→로봇 base 방향에서 유도**한다. 설정값
            #   (0, ±30) 은 그 방향에 대한 오프셋이다. 예전엔 atan2 세계각 그대로
            #   써서 gap 을 로봇 **반대편**(+x, 축 너머)으로 돌렸다 — 카메라 eye
            #   가 x≈1.0m 로 도달한계(1.09m) 직전이라 IK 가 전멸했고
            #   (2026-09-16: "IK 150, 충돌 0" 후 실패), 매번 전회전 폴백만 돌았다.
            #   성공하던 el-NBV 카메라는 전부 x 0.63~0.69 (로봇 쪽) — 그게 답이다.
            #   (2026-09-18: 공용 `robot_side_azimuths` — sim 도 같은 걸 쓴다.)
            fr = self._nbv.plan_frontier(
                gaps, q_cur, solve_lookat, swept,
                standoff_m=self.s.nbv_distance_mm / 1000.0,
                axis_xy=axis_xy_B,
                az_pref_deg=_nbvp.robot_side_azimuths(
                    axis_xy_B, self.s.nbv_frontier_az_pref_deg),
                obj_pts=_guard_pts, eye_clear_m=_eye_clear,
                # 정합 겹침 안전장치 — master 등록 scan 점(없으면 메시 정점)
                known_pts=self._known_surface_pts_m(mesh))
        if fr is not None:
            # 턴테이블을 이 θ 로 보내면 gap 이 로봇 편한 방위에 온다.
            # 캡처는 sim 과 같이 이 θ 중심 부분 스윕(`nbv_patch_span_deg`) —
            # `_capture_nbv_pose` 가 θ+span/2 에서 시작해 span 만큼 돈다.
            self._next_theta = float(fr[3])
            self._nbv_dbg_plan(gaps, fr[1], "frontier", fr[0], np.asarray(fr[1].p_O, float),
                               float(fr[3]), self.s.nbv_patch_span_deg, mesh,
                               note=f"L={float(fr[1].L)*1000:.0f}mm")
            return fr[0]

        # ── ③ 폴백: 축-고도각 — gap 군집을 로봇 앞으로 가져와 **180° 부분 스윕** ──
        #   sim 과 같은 공용 `gap_cluster_theta`(2026-09-18). 군집 방위를 못 구하면 전회전.
        self._next_theta = None
        self._next_span = None
        res = self._nbv.plan(gaps, q_cur, solve_pose, swept,
                                  roll_order_fn=roll_order)
        if res is None:
            return None
        _th = _nbvp.gap_cluster_theta(
            gaps, axis_xy_B,
            _nbvp.robot_side_azimuths(axis_xy_B, self.s.nbv_frontier_az_pref_deg)[0],
            self._nbv.up_sign)
        if _th is not None:
            self._next_theta = float(_th)
            self._next_span = _nbvp.FALLBACK_SPAN_DEG
            print(f"  [nbv] 폴백 스윕 θ={np.degrees(_th) % 360:.0f}° 중심 "
                  f"±{_nbvp.FALLBACK_SPAN_DEG/2:.0f}° (gap 군집 방위)")
        self._nbv_dbg_plan(gaps, None, "fallback", res[0], look_target,
                           float(_th) if _th is not None else 0.0,
                           _nbvp.FALLBACK_SPAN_DEG if _th is not None else 360.0, mesh,
                           note=f"axis-el el={res[1]:.0f}° az={res[2]:.0f}°")
        return res[0]

    def _nbv_dbg_plan(self, gaps, chosen, mode, q, target_B, theta, span_deg, mesh, note=""):
        """nbv 디버그 기록(계획 시점) — sim 과 같은 `utils/nbv/nbv_debug_dump`. 실패해도 무시.
        카메라 위치는 해석 FK: (T_EB · T_EC⁻¹) 의 평행이동 = `_capture_nbv_pose` 의 식과 같다."""
        try:
            from utils.robot import xarm7_kinematics as _kin
            if getattr(self, "_nbv_dbg", None) is None:
                self._nbv_dbg = _NbvDebugDump()
            eye = (_kin._fk_frames_m(np.asarray(q, float))[7]
                   @ np.linalg.inv(self._T_EC))[:3, 3]
            idx = next((i for i, c in enumerate(gaps or []) if c is chosen), -1)
            axis_pt = getattr(self, "_axis_pt_B", None)
            if axis_pt is None:
                axis_pt = np.r_[np.asarray(target_B, float)[:2], 0.0]
            up = -1.0 if float(self._view_up_B()[2]) < 0 else +1.0
            self._nbv_dbg.plan(gaps=gaps, chosen=idx, mode=mode, eye=eye, target=target_B,
                               theta=theta, span_deg=span_deg,
                               master_pts=self._known_surface_pts_m(mesh),
                               axis_pt=axis_pt, up_sign=up, note=note)
        except Exception as e:                                   # noqa: BLE001
            print(f"  [nbv-dbg] plan 기록 실패({e})")

    def _collision_gate(self):
        """단일 충돌 게이트(sim 과 **같은 모듈**). 캐시가 없으면 None → 기존 캡슐 경로.

        ★ 캐시(`utils/collision/data/*.npz`)는 셀 CAD 를 **로봇 base 프레임**으로 구운
          것이라 sim·real 공용이다. 실물 셀을 개조하면 반드시 재생성할 것:
              scripts/sim/export_env_mesh.py / export_link_meshes.py
        """
        if not hasattr(self, "_cm_cache"):
            try:
                from utils.collision import collision_model as _cmod
                self._cm_cache = _cmod.get_default(
                    self_margin_m=getattr(self.s, "self_clear_m", 0.020),
                    env_margin_m=getattr(self.s, "env_clear_m", 0.025))
            except Exception as e:                      # noqa: BLE001
                print(f"  [collision] 게이트 사용 불가({type(e).__name__}) — 캡슐 경로")
                self._cm_cache = None
        return self._cm_cache

    def _move_robot_to_q(self, q, speed_deg_s: float) -> int:
        """해석 IK q 로 관절구동 (IK 결정: set_position 대신 set_servo_angle).

        ★ 이동 전 **단일 게이트**로 자세·경로를 검사하고, 직선이 막히면 우회 경로를
          계획해 웨이포인트로 따라간다(sim `_drive` 와 같은 구조).
          예전에는 real 이 NBV 후보 판정에만 캡슐 충돌을 썼고 **실제 이동은 무검사**였다.
          계획도 실패하면 움직이지 않고 실패코드를 돌려준다 — 실물에서는 '일단 가본다'가
          곧 파손이다.
        """
        q1 = np.asarray(q, float)
        cm = self._collision_gate()
        way = [q1]
        if cm is not None:
            try:
                q0 = np.asarray(self.robot.get_joint_angles(is_radian=True), float)
            except Exception:                           # 관절각을 못 읽으면 검사 생략
                q0 = None
            if q0 is not None:
                ok, why, _ = cm.is_path_safe(q0, q1)
                if not ok:
                    if why.startswith(("start", "goal")):
                        print(f"  [collision] ✘ 이동 거부 — {why} (자세 자체가 불가)")
                        return -1
                    print(f"  [collision] 직선 막힘({why}) — 우회 계획")
                    from utils.control.joint_path_planner import plan_joint_path
                    from utils.robot import xarm7_kinematics as _kin
                    path = plan_joint_path(
                        q0, q1, lambda a, b: cm.is_path_safe(a, b)[:2],
                        lower=_kin.JOINT_LOWER, upper=_kin.JOINT_UPPER,
                        step=0.3, max_iter=600, shortcut_iters=60,
                        log=lambda m: print(f"  [collision] {m}"))
                    if path is None:
                        print("  [collision] ✘ 이동 거부 — 우회 경로 없음")
                        return -1
                    way = [np.asarray(w, float) for w in path[1:]]
        self.robot.enable_motion()
        code = 0
        for w in way:
            # ★ xArm SDK: is_radian=True 이면 speed/mvacc 도 rad/s·rad/s² 로 해석한다 — deg 값을 그대로 넘기면 π rad/s(180°/s)로 클램프돼 설정과 무관하게 최고속이 된다(2026-09-22 발견).
            #   가속도도 같이 준다(속도의 4배/s² → 0.25s 램프): 안 주면 SDK 의 마지막
            #   값(기본 최대)이라 저속에서도 출발이 확 튄다.
            code = self.robot.arm.set_servo_angle(
                angle=w.tolist(), speed=float(np.radians(speed_deg_s)),
                mvacc=float(np.radians(speed_deg_s * 4.0)), is_radian=True, wait=True)
            code = int(code) if code is not None else 0
            if code != 0:
                return code
        return code

    def _capture_nbv_step(self, T_BC_nbv, th_center, sweep_rad):
        """nbv 패치를 **정지-촬영**으로 모은다 — SDK 추적 없음.

        왜 — 스트리밍 스윕은 SDK 가 프레임끼리 정합(SLAM)해야 하는데, 윗면·매끈한
        면을 내려다보는 nbv 자세에서는 5프레임 만에 잃는다(2026-09-22 run_132655 폴백
        5회 전부). 정지-촬영은 턴테이블을 K 단계로 세우고 프레임 하나씩 찍어 **알고
        있는 θ** 로 놓는다 — 추적이 없으니 잃을 것도 없고, 배치 정확도는 축 캘리브와
        기구학(그 뒤 ICP)에 걸린다. sim 의 `_scan_patch`(θ 단계별 프레임)와 같은 구조.
        프레임 k 의 변환(스텝 0 스캐너 프레임 기준):
            T_k = S⁻¹ · T_BC · R_B(axis, −Δθ_k) · T_CB · S,   Δθ_k = θ_k − θ_0
        반환 (ArtecStreamingScanResult 호환 객체, T_pre) 또는 (None, None).
        """
        sensor = getattr(self.mms, "sensor", None)
        if sensor is None or not hasattr(sensor, "capture_frame"):
            print("  [nbv] 정지-촬영 불가(sensor 없음)")
            return None, None
        full = sweep_rad is None
        span = 2.0 * np.pi if full else float(sweep_rad)
        step_deg = float(getattr(self.s, "nbv_step_deg", 30.0))
        K = max(3, int(round(np.degrees(span) / step_deg)) + (0 if full else 1))
        th_c = float(th_center) if th_center is not None else self._read_turntable_theta()
        th_start = th_c if full else th_c + span / 2.0
        thetas = [th_start - k * (span / (K if full else K - 1)) for k in range(K)]
        S_m = self._S_m()
        if S_m is None:
            S_m = np.eye(4)
        T_CB = np.linalg.inv(np.asarray(T_BC_nbv, float))
        tt = getattr(self.mms, "turntable_transform", None)
        p_ax = np.asarray(tt.axis_point_B, float)
        a_dir = np.asarray(tt.axis_dir_B, float)
        scan = artec_base.create_scan()
        theta_used = []
        n_ok = 0
        t0 = time.perf_counter()
        self._nbv_dbg_seq = int(getattr(self, "_nbv_dbg_seq", 0)) + 1
        print(f"  [nbv] 정지-촬영 {K}프레임 — {np.degrees(th_start) % 360:.0f}° 부터 "
              f"{step_deg:.0f}° 간격" + (" (전회전)" if full else ""))
        for k, thk in enumerate(thetas):
            if not self._move_turntable_abs(thk):
                continue
            th_act = self._read_turntable_theta()
            fmh = None
            for _try in range(2):
                try:
                    fmh = sensor.capture_frame(capture_texture=True)
                except Exception:                                # noqa: BLE001
                    fmh = None
                if fmh is not None and fmh.vertex_count() > 0:
                    break
            self._nbv_dbg_image(k, K, th_act, fmh, T_BC_nbv)
            if fmh is None or fmh.vertex_count() == 0:
                print(f"    [step {k+1}/{K}] θ={np.degrees(th_act) % 360:.0f}° — 빈 프레임")
                continue
            if not theta_used:
                theta_used.append(th_act)
            dth = th_act - theta_used[0]
            R3 = self._rot_about_axis(a_dir, -dth)
            R_undo = np.eye(4)
            R_undo[:3, :3] = R3
            R_undo[:3, 3] = p_ax - R3 @ p_ax
            T_k = np.linalg.inv(S_m) @ np.asarray(T_BC_nbv, float) @ R_undo @ T_CB @ S_m
            T_k[:3, 3] *= 1000.0
            scan.add_frame(fmh)
            scan.set_frame_transformation(n_ok, T_k)
            n_ok += 1
            print(f"    [step {k+1}/{K}] θ={np.degrees(th_act) % 360:.0f}° — {fmh.vertex_count():,}점")
        if n_ok == 0:
            print("  [nbv] 정지-촬영 — 프레임 0 (전부 빈 프레임)")
            _ev("nbv_step", frames=0, span_deg=float(np.degrees(span)))
            return None, None
        model = artec_base.create_model()
        model.add_scan(scan)
        theta0 = float(theta_used[0])
        T_pre = self._compose_T_pre_W_mm(T_BC_nbv, theta0, tag="nbv")
        sub = ArtecStreamingScanResult(model=model, n_frames=n_ok, frames_ok=n_ok,
                                       tracking_lost=False, n_bands=1, n_bands_done=1,
                                       rotation_actual_deg=float(np.degrees(span)),
                                       duration_s=time.perf_counter() - t0)
        print(f"  [nbv] 정지-촬영 완료 — {n_ok}/{K} 프레임, {sub.duration_s:.1f}s")
        _ev("nbv_step", frames=n_ok, planned=K, span_deg=float(np.degrees(span)), secs=sub.duration_s)
        return sub, T_pre

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
        # ★ EE/카메라 위치를 **찍는다** — 조준이 빗나가면 이 줄과 `[nbv] 조준
        #   target` 줄만 비교해도 어긋난 좌표계를 바로 특정할 수 있다.
        _p_ee = T_EB_after[:3, 3]
        _T_CB_nbv = np.linalg.inv(T_BC_nbv)
        _p_cam = _T_CB_nbv[:3, 3]
        print(f"  [nbv] 이동 완료 — TCP=({_p_ee[0]:+.3f}, {_p_ee[1]:+.3f}, "
              f"{_p_ee[2]:+.3f})m  카메라=({_p_cam[0]:+.3f}, {_p_cam[1]:+.3f}, "
              f"{_p_cam[2]:+.3f})m (B)")

        # gap 겨냥 자세면 **T_pre 계산 전에** 턴테이블을 목표 θ 로 보낸다. 그 각에서
        # gap 이 로봇 정면에 오도록 자세를 푼 것이다. sim 과 같이 스윕을 목표 θ 에
        # **중심 정렬**한다: 회전은 −방향이므로 θ+span/2 에서 시작해 θ−span/2 까지.
        th = getattr(self, "_next_theta", None)
        sweep_rad = None
        if th is not None:
            _span = getattr(self, "_next_span", None) or self.s.nbv_patch_span_deg
            self._next_span = None
            sweep_rad = float(np.radians(_span))
            th_start = float(th) + sweep_rad / 2.0
            try:
                self.turntable.stop()
                self.turntable.check_drive_err()
                self.turntable.set_servo_on(True)
                self.turntable.move_abs(
                    th_start, float(self.s.probe_turntable_vel_rad_s))
                if hasattr(self.turntable, "wait_motion_done"):
                    self.turntable.wait_motion_done(timeout_s=20.0)
                print(f"  [nbv] gap 겨냥 θ={np.degrees(th) % 360:.0f}° — "
                      f"스윕 {np.degrees(sweep_rad):.0f}° "
                      f"(시작 {np.degrees(th_start) % 360:.0f}°)")
            except Exception as e:                          # noqa: BLE001
                print(f"  [nbv] ⚠ θ 이동 실패({type(e).__name__}: {e}) — 현재 각에서 진행")
            finally:
                self._next_theta = None

        # ── T_pre: sub scan-world → master scan-world ────────────────────
        # ★ 캡처 시작 시점의 **실제 턴테이블 각 θ0** 를 잰다. gap 겨냥은 물체가
        #   master 기준(logical 0°)에서 θ0 만큼 회전된 상태로 스캔을 시작하므로,
        #   카메라 이동만 보정하면 그 스캔이 **회전된 채** master 에 합쳐진다 —
        #   2026-09-16 실물: θ=306° gap 스캔이 유령 geometry 를 만들고, 다음
        #   반복이 유령의 gap 을 겨냥해 스캐너가 뚜껑 15mm 까지 접근했다.
        #   또 scan-world 는 **스캐너3D 프레임**이지 Color(C)가 아니다
        #   ([[project_artec_3d_vs_color_frame]]) — S(스캐너→Color)로 켤레해야
        #   B 를 경유하는 변환이 옳다:
        #     T_pre = S⁻¹ · T_BC_master · R_B(axis, −θ0) · T_CB_new · S
        if str(getattr(self.s, "nbv_capture_mode", "step")).lower() == "step":
            return self._capture_nbv_step(T_BC_nbv, th, sweep_rad)

        theta0 = self._read_turntable_theta()
        T_pre = self._compose_T_pre_W_mm(T_BC_nbv, theta0, tag="nbv")

        st = self.s.streaming_settings
        orig_reset = st.reset_to_zero_first
        orig_sweep = st.sweep_rad
        st.reset_to_zero_first = False                     # 현재 위치 유지
        st.sweep_rad = sweep_rad                           # gap 겨냥 = 부분 스윕
        orig_label = st.stage_label
        st.stage_label = "nbv"                             # 배너가 단계를 말하게
        orig_dur = st.rotation_duration_s
        st.rotation_duration_s = float(self.s.nbv_rotation_duration_s)   # 부분 스윕은 빠르게
        try:
            sub = ArtecStreamingScanSession(
                self.mms, self.robot, self.turntable, st).run()
        finally:
            st.reset_to_zero_first = orig_reset
            st.sweep_rad = orig_sweep
            st.stage_label = orig_label
            st.rotation_duration_s = orig_dur
        return sub, T_pre

    # ── nbv (NBV hole-fill) 프리미티브 — 수렴 루프는 공용 컨트롤러 소유 ──
    #   (per-gap 정면 캡처(`_rank_nbv_candidates`, 2026-09-18 삭제) → 캡처통일로 대체, 2026-06-30.
    #    이전 _nbv_loop → 공용 _run_nbv 로 통합, 2026-07-01.)
    def flip_extra_poses(self):
        """flip 테두리 패스 — **real 은 아직 없다** (sim 은 `IsaacScanSession.flip_extra_poses`).

        막힌 이유: real 의 `capture_rotation(stage="flip", pose=q)` 는 nbv 경로
        (`_capture_nbv_pose`: 카메라 이동 T_pre 만 보정)로 빠지는데, flip 패스는
        사람이 뒤집은 회전(hint, `recorded_hints`/`hint_icp_refine`)까지 붙어야
        master 에 맞는다. 테두리 패스에 같은 flip hint 를 합성해 넘기는 배선이 필요
        하고, 그건 실기(사람 flip 정확도)와 같이 봐야 한다 — 2026-09-18 보류.
        기하(el·축거리)는 공용 `flip_policy.rim_standoff` 로 sim 과 같이 낼 수 있다.
        """
        return []

    def _master_pts_B_m(self, max_points: int = 80_000):
        """master 의 등록 scan 점 → **B 프레임, m**. 없으면 빈 배열.

        ★ 반드시 `_build_master_mesh_B` 와 **같은 변환**을 탄다 — 마스터 프레임의
          `master_T_CB` + 스캐너3D→Color `S`. 예전에는 여기(기지면 필터)와 신규복셀
          회계가 `master_points_in_base_frame(model, self._T_CB)` 를 썼다: **현재
          자세**의 T_CB 에 S 도 없어서 점군이 물체에서 ~600mm 떨어진 허공에 놓였다
          (2026-09-21 run_145013: nbv-dbg master 가 축에서 r≈590mm, 디스크 위
          300mm). 그 결과 gap 후보가 전부 '기지면부족' 으로 기각돼 매 반복이
          폴백 스윕으로 빠졌고, 신규복셀 판정도 무의미했다.
        """
        try:
            st = self._st
            if st.master_model is None or st.master_model.scan_count() == 0:
                return np.zeros((0, 3), float)
            T_CB = st.master_T_CB if st.master_T_CB is not None else self._T_CB
            pcd = self._master_to_pcd_B(
                st.master_model, T_CB, voxel_mm=self.s.nbv_master_voxel_mm,
                T_sc_mm=getattr(self, "_T_scan_color", None))
            if pcd is None or len(pcd.points) == 0:
                return np.zeros((0, 3), float)
            P = np.asarray(pcd.points, float) / 1000.0
            if len(P) > max_points:
                P = P[np.random.default_rng(0).choice(len(P), max_points, replace=False)]
            return P
        except Exception as e:                                   # noqa: BLE001
            print(f"  [nbv] ⚠ master 점 추출 실패({e})")
            return np.zeros((0, 3), float)

    def _known_surface_pts_m(self, mesh):
        """gap 겨냥 필터용 **기지 면** 점군 (B, m). master 의 등록 scan 점을 쓰고,
        못 꺼내면 메시 정점으로 폴백 — dry 회계와 같은 우선순위."""
        mp = self._master_pts_B_m(80_000)
        return mp if len(mp) else np.asarray(mesh.vertices, float)

    def _nbv_dbg_image(self, k: int, K: int, th_act: float, fmh, T_BC_nbv=None) -> None:
        """nbv 정지-촬영 프레임을 lookaround 와 같은 거리 이미지로 남긴다
        (`output/debug/nbv_<RUN_TS>/nbvNN_stepKK_thDDD.png`). 유효 점군이 모였는지,
        물체가 시야 어디에 있는지를 눈으로 본다. MMS_LOOKAROUND_DEBUG_IMG=0 으로 끈다."""
        if os.environ.get("MMS_LOOKAROUND_DEBUG_IMG", "1") == "0":
            return
        try:
            from utils.nbv.range_debug_image import save_range_image
            from utils.nbv.standoff import TRACK_CORE_HALF_DEG
            run_ts = os.environ.get("MMS_RUN_TS", "run")
            seq = int(getattr(self, "_nbv_dbg_seq", 0))
            V = (np.asarray(fmh.vertices(), float) / 1000.0
                 if fmh is not None and fmh.vertex_count() > 0 else np.zeros((0, 3)))
            tex = None
            if fmh is not None:
                try:
                    tex = fmh.image()
                except Exception:                                   # noqa: BLE001
                    tex = None
            dof = self._scanning_range_m()
            p = float(np.median(np.linalg.norm(V, axis=1))) if len(V) else float("nan")
            deg = int(round(np.degrees(float(th_act)) % 360))
            cam = ""
            if T_BC_nbv is not None:
                try:
                    tt = self.mms.turntable_transform
                    T_CB = np.linalg.inv(np.asarray(T_BC_nbv, float))
                    c = T_CB[:3, 3]
                    ax = np.asarray(tt.axis_point_B, float)
                    cam = f"  axis dist={np.hypot(*(c[:2] - ax[:2]))*1000:.0f}mm"
                except Exception:                                   # noqa: BLE001
                    cam = ""
            lines = [f"nbv{seq:02d} step{k+1:02d}/{K}  theta={deg}deg",
                     f"n={len(V)}  median={p*1000:.0f}mm  window {dof[0]*1000:.0f}-{dof[1]*1000:.0f}mm{cam}"]
            save_range_image(os.path.join("output", "debug", f"nbv_{run_ts}",
                                          f"nbv{seq:02d}_step{k+1:02d}_th{deg:03d}.png"),
                             V, dof, TRACK_CORE_HALF_DEG, lines, tex)
        except Exception as e:                                          # noqa: BLE001
            print(f"  [nbv] 디버그 이미지 실패({type(e).__name__}: {e})")

    def _object_radius_m(self, axis_xy_B, mesh_verts):
        """축거리 계산용 **물체 반경** (m, p95) 과 출처 문자열.

        1순위 preview 점군(`preview_result["points_B"]`) — 물체만 담겨 깨끗하다.
        2순위 메시 정점 — 디스크면에서 10mm 이상 떠 있는 점만(디스크·바닥 제외).
        둘 다 없으면 0 (표면거리 = 축거리, 예전 동작).
        """
        tt = getattr(self.mms, "turntable_transform", None)
        disc_z = float(np.asarray(tt.axis_point_B, float)[2]) if tt is not None else None
        up = -1.0 if float(self._view_up_B()[2]) < 0 else +1.0
        ax = np.asarray(axis_xy_B, float)[None, :]

        def _p95(P):
            P = np.asarray(P, float)
            if disc_z is not None and len(P):
                P = P[up * (P[:, 2] - disc_z) > 0.010]         # 디스크면 10mm 위만
            if len(P) < 30:
                return None
            return float(np.percentile(np.linalg.norm(P[:, :2] - ax, axis=1), 95))

        # 0순위 lookaround 거리추종 실측 — "축거리 = 표면거리 + r" 의 r 를 실제로 잰 값.
        #   preview p95 는 실루엣 잡음·디스크 가장자리에 끌려 과대하다(2026-09-22 run_162620:
        #   preview 92mm → nbv 축거리 317mm 로 프레임당 ~1,000점 "빈 캡처" 2회. 같은 run 의
        #   밴드 추종은 353→278mm 로 수렴, 즉 r_eff≈25~30mm).
        _hist = list(getattr(getattr(self, "_st", None), "r_eff_hist", []) or [])
        if len(_hist) >= 1:
            return float(np.median(_hist)), "lookaround 실측"
        pv = getattr(self, "preview_result", None) or {}
        r = _p95(pv.get("points_B", np.zeros((0, 3))))
        if r is not None:
            return r, "preview"
        r = _p95(mesh_verts)
        if r is not None:
            return r, "메시(폴백)"
        return 0.0, "없음"

    def _gap_kw(self) -> dict:
        s = self.s
        return dict(
            min_seg_vertices=s.nbv_min_seg_vertices,
            min_seg_length=s.nbv_min_seg_length_mm / 1000.0,
            max_seg_length=s.nbv_max_seg_length_mm / 1000.0)

    def build_coverage_mesh(self):
        # nbv 진입 첫 호출 시 swept-path 검사용 충돌 world 준비(1회).
        if getattr(self, "_collision_world", None) is None:
            print("\n═══════════════ nbv — 부족면 NBV 보강 ═══════════════")
            self._collision_world = self._build_collision_world()
        mesh = self._build_master_mesh_B(self._st.master_model)
        if mesh is None or len(mesh.triangles) == 0:
            print("  [nbv] master mesh 비어있음 — 종료.")
            return None
        # 동적 장애물 갱신 — 계획 점군보다 정확한 실측 메시로 (경로가 대상물을
        # 관통하지 않도록; 등록 자체의 근거는 _pick_lookaround_planner 쪽 주석 참조).
        self._register_object_obstacle(np.asarray(mesh.vertices), "mesh")
        return mesh

    def is_converged(self, mesh) -> bool:
        from utils.nbv import nbv_core as _p2
        s = self.s
        cov = _p2.coverage_state(
            mesh, n_dirs=s.nbv_coverage_dirs,
            parallel_thresh_deg=s.nbv_coverage_parallel_deg, **self._gap_kw())
        print(f"  [nbv] boundary={cov.boundary_len_m*1000:.1f}mm "
              f"cov={cov.angular_cov:.3f} gaps={cov.n_gaps}")
        # ── gap 단위 회계 (sim 과 같은 구조) ─────────────────────────────
        # 직전 frontier 패치가 새 관측을 얻었는지 master 메시 정점의 신규 점유
        # 복셀로 판정해 planner 에 보고한다. 비생산 겨냥점 주변은 후보에서 빠져
        # 도달 불가 영역을 반복 겨냥하지 않는다(루프 종료가 아니라 후보 제외 —
        # 판정이 틀려도 그 영역 하나를 잃을 뿐이다).
        # ★ 재는 대상 = master 에 **등록된 scan 의 점**(base 프레임). 점은 scan 이
        #   붙을 때마다 더해지기만 하므로 sim 의 누적 원시 점군과 같은 성질이다.
        #   예전에는 Poisson/fusion **메시 정점**으로 쟀다 — sim 주석이 이미
        #   "메시 기반 신규복셀은 머그에서 Δcompleteness 와 r=+0.43 에 그쳤다" 고
        #   폐기한 지표 위에 real 의 dry 판정이 서 있었다(2026-09-18 리뷰).
        #   master 점을 못 꺼내면(변환 없음 등) 메시 정점으로 **폴백**하되 로그에
        #   남긴다 — 그 경우 임계 0.015 는 검증 안 된 값이다.
        if os.environ.get("MMS_REAL_DRY_GAP", "1") == "1":
            try:
                v = np.asarray(mesh.vertices, float)
                # flip 정책 안내용 치수 — sim 과 **같은 공용 함수**로 잰다.
                # AABB 폭을 쓰면 손잡이 때문에 지름이 부풀어 종횡비가 낮게 나온다.
                from utils.nbv.flip_policy import dims_from_points
                self._obj_dims = dims_from_points(v)
                src, pts_m = "메시정점(폴백)", v
                mp = self._master_pts_B_m(60_000)       # 마스터 프레임 + S (메시와 동일)
                if len(mp):
                    src, pts_m = "master scan 점", mp
                new_frac, self._conv_vox = _nbvp.new_voxel_frac(
                    pts_m, getattr(self, "_conv_vox", None), _nbvp.CONV_VOX_M)
                if new_frac is not None:
                    print(f"  [nbv] 신규복셀={new_frac*100:.2f}% ({src})")
                    if getattr(self, "_nbv", None) is not None:
                        self._nbv.report_patch(new_frac >= _nbvp.DRY_EPS)
                    # 전역 백스톱 — sim 과 같은 공용 StallTracker. "새로 보이는
                    # 게 없다" 는 판단이 실물에는 없어서 반복 상한까지 돌았다.
                    stall = getattr(self, "_stall", None)
                    if stall is None:
                        stall = self._stall = _nbvp.StallTracker()
                    if stall.update(new_frac):
                        print("  [nbv] 수렴 — 새로 보이는 곳이 없다. 완료.")
                        return True
            except Exception as e:               # noqa: BLE001
                print(f"  [nbv] ⚠ 신규복셀 회계 실패({e}) — 건너뜀")
        if _p2.is_converged(cov, s.nbv_boundary_stop_mm / 1000.0, s.nbv_coverage_tau):
            print("  [nbv] 커버리지 수렴 — 완료.")
            return True
        return False

    def plan_nbv_pose(self, mesh):
        # ★ 공용 자세선택(sim 과 동일): 부족면 덮을 관측 elevation 자세 → streaming 전회전.
        from utils.nbv import nbv_core as _p2
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
        mode: str = "flip",
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

        # ★ 정합은 **sim·real 공용** `refine_to_master` (2026-09-18). 예전엔 여기가
        #   colored ICP(색 가중 0.5, 단일 대응거리 30mm)였고 sim 은 다단 point-to-plane
        #   이라 **방법이 달랐다** — sim 이 real 을 검증한다는 전제가 여기서 깨졌다.
        #   공용 함수 = coarse→fine ICP + RMSE/fitness/drift(단계별·누적) 게이트 +
        #   팽창 게이트. 팽창 게이트 근거(옛 주석): sim 9종 실측에서 대칭축 방향
        #   평행이동 오류가 fitness·drift 를 통과하고도 합집합 bbox 를 8.5~10mm
        #   부풀려 GT F-score 가 52.0→45.1%, 75.5→56.5% 로 무너졌고, 게이트 뒤
        #   58.9% / 68.9% 로 회복했다.
        #   색은 안 쓴다 — sim 점군에 색이 없어 같은 동작이 안 된다.
        #   단위: 공용 함수는 m. B 프레임 mm PCD 를 m 로 내려 넣고 결과를 mm 로 올린다.
        from utils.nbv.icp_strategy import refine_to_master
        S_m = np.asarray(new_pcd.points, float) / 1000.0
        M_m = np.asarray(master_pcd.points, float) / 1000.0
        res = refine_to_master(S_m, M_m, init_B_m, mode=mode, voxel_m=0.0,
                               max_iter=int(max_iter),
                               log=lambda m: print(f"  [icp_refine] {m}"))
        if res is None:
            print(f"  [icp_refine] 점 부족 — init 그대로")
            return T_pre_init, 0.0, float("inf")
        fitness = float(res.fitness)
        rmse_mm = float(res.rmse) * 1000.0
        if not res.ok:
            print(f"  [icp_refine] 게이트 기각({res.reason}) → hint 그대로")
            return T_pre_init, fitness, rmse_mm
        T_measured_B_mm = np.asarray(res.T_refined, float).copy()
        T_measured_B_mm[:3, 3] = res.T_refined[:3, 3] * 1000.0

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
                         master_model, mode: str = "flip") -> tuple:
        """Instance wrapper — settings + self._T_BC/self._T_CB 를 static 함수에 위임.
        `mode` = "flip"(사람 손회전 오차, 이동 허용 넓음) | "patch"(nbv 부분 스윕)."""
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
            mode=mode,
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

    @staticmethod
    def _axis_angle(R3: np.ndarray):
        """회전행렬 → (각 deg, 단위축). Rodrigues 역계산; 180° 근처는 (R+I)/2 로 축 복원."""
        cos_t = float(np.clip((np.trace(R3) - 1.0) / 2.0, -1.0, 1.0))
        angle_deg = float(np.degrees(np.arccos(cos_t)))
        if abs(angle_deg) < 0.1:
            return 0.0, np.zeros(3)
        if abs(180.0 - angle_deg) < 1.0:
            M = (R3 + np.eye(3)) / 2.0
            diag = np.diag(M)
            i = int(np.argmax(diag))
            ax = np.zeros(3)
            ax[i] = float(np.sqrt(max(diag[i], 0.0)))
            for j in range(3):
                if j != i:
                    ax[j] = M[i, j] / ax[i] if ax[i] > 1e-9 else 0.0
        else:
            sin_t = np.sin(np.radians(angle_deg))
            ax = np.array([R3[2, 1] - R3[1, 2],
                           R3[0, 2] - R3[2, 0],
                           R3[1, 0] - R3[0, 1]]) / (2.0 * sin_t)
        n = float(np.linalg.norm(ax))
        return angle_deg, (ax / n if n > 1e-9 else ax)

    def _print_pose_hint_for_user(self, pose_idx: int) -> None:
        """이 뒤집기에서 물체를 어떻게 놓아야 하는지 안내.

        ★ `pose_physical_rotations[k]` 는 **원래 자세(pose 0) 기준 절대 회전**이고
          정합 힌트로 그대로 쓰인다(`R_phys`). 그래서 사용자는 이 값을 **따라야**
          한다 — 마음대로 다른 각으로 돌리면 힌트가 틀려 flip 패스가 어긋난다.
          2/2 에서 "180°" 만 찍으면 "지금 자세에서 180°" 로 읽히므로(2026-09-21
          질문) 직전 자세 대비 **추가로 돌릴 각**도 같이 찍는다.
        """
        rots = self.s.pose_physical_rotations
        if not rots or pose_idx >= len(rots) or rots[pose_idx] is None:
            print(f"  안내 미정 — 원래 자세 그대로 둔다")
            return
        angle_deg, ax = self._axis_angle(rots[pose_idx][:3, :3])
        if angle_deg < 0.1:
            print(f"  원래 자세 그대로 둔다 (회전 0°)")
            return
        from utils.nbv.flip_policy import describe_flip
        # 축을 사람 말로: 턴테이블 회전축(수직)과 나란하면 '수직축', 아니면 '수평축'.
        tt = getattr(self.mms, "turntable_transform", None)
        ad = getattr(tt, "axis_dir_B", None) if tt is not None else None
        vertical = ad is not None and abs(float(np.dot(ax, np.asarray(ad, float)))) > 0.9
        ax_word = "수직축(턴테이블 축)" if vertical else "수평축"
        print(f"  ★ 원래 자세 기준 {angle_deg:.0f}° — {describe_flip(angle_deg)}")
        print(f"    회전축: base ({ax[0]:+.2f}, {ax[1]:+.2f}, {ax[2]:+.2f}) = {ax_word}")
        # 직전 자세에서 추가로 돌릴 각 (같은 축이면 단순 차, 아니면 상대회전의 각).
        prev = rots[pose_idx - 1] if pose_idx >= 1 and rots[pose_idx - 1] is not None else None
        if prev is not None:
            d_deg, _ = self._axis_angle(prev[:3, :3].T @ rots[pose_idx][:3, :3])
            if d_deg > 0.1 and abs(d_deg - angle_deg) > 0.1:
                print(f"    지금 자세에서는 같은 축으로 {d_deg:.0f}° **더** 돌리면 된다")
        print(f"    (이 각이 정합 힌트다 — 다른 각으로 놓으면 정합이 어긋난다)")

    @staticmethod
    def _print_pass_banner(idx: int, total: int, pose_idx: int) -> None:
        """재시도·뒤집기처럼 **기본이 아닐 때만** 한 줄로 알린다.

        예전엔 매 회전마다 3줄짜리 박스로 `Multi-pass scan — Pass 1 / max 8
        (flip 0)` 을 찍었다. 바로 위 `[stage] ═══ [2] lookaround ═══` 과 바로 아래
        `Artec Streaming lookaround` 가 이미 단계를 말하는데 그 사이에 끼어
        중복이었고, 옛 용어(`Multi-pass`/`Pass`)에 `(flip 0)` 까지 붙어 **flip
        단계로 오해**를 샀다 — 실제로는 flip *인덱스*(0 = 안 뒤집은 원래 자세)다
        (2026-09-21 사용자 혼동).

        첫 시도·원래 자세(idx=1, pose_idx=0)는 아무것도 안 찍는다 — 그게 기본이라
        알릴 것이 없다.
        """
        # `idx` 는 누적 scan 번호라 "재시도" 가 아니다 — flip 첫 캡처가 '재시도 2/8'
        # 로 찍혔다(2026-09-21 run_150720). 뒤집기 번호만 말한다.
        if pose_idx <= 0:
            return
        print(f"\n  ── 뒤집기 {pose_idx}번째 수집 (scan {idx}/{total}) ──")

    @staticmethod
    def _read_key(allowed: str = "q") -> str:
        """콘솔에서 **키 하나**를 읽는다 — Enter 면 "", `allowed` 의 글자면 그 글자.

        Windows 콘솔은 msvcrt 로 받아 **Enter 가 필요 없다**. 다른 키는 무시하고
        계속 기다린다. 콘솔이 아니거나(파이프·리다이렉트) msvcrt 가 없으면
        `input()` 폴백 — 그때는 Enter 가 필요하다.
        예전 `[n]+Enter / [q]+Enter` 는 "n 을 누르고 Enter" 인지 "n 또는 Enter"
        인지 읽히지 않았다(2026-09-21 사용자 피드백)."""
        import sys as _sys
        try:
            import msvcrt
            if _sys.stdin is not None and _sys.stdin.isatty():
                while msvcrt.kbhit():               # 밀린 키 버림 (nbv 중 누른 n 등)
                    msvcrt.getwch()
                while True:
                    ch = msvcrt.getwch()
                    if ch in ("\r", "\n"):
                        print()
                        return ""
                    if ch.lower() in allowed:
                        print(ch.lower())
                        return ch.lower()
        except ImportError:
            pass
        except Exception:                           # noqa: BLE001 — 콘솔 이상 시 폴백
            pass
        try:
            inp = input("  >> ").strip().lower()
        except EOFError:
            return ""
        return inp[:1] if inp[:1] in allowed else ""

    @classmethod
    def _wait_user_quit(cls) -> bool:
        """[Enter] 계속 / [q] 종료 — q 면 True."""
        return cls._read_key("q") == "q"
