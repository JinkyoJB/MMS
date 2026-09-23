"""
utils/nbv/lookaround.py — lookaround viewpoint selection (sim/real 공용 코어).

컨셉 (2026-07-03 논의):
  lookaround = 고정 스캐너 + 턴테이블 전회전. 회전하는 물체는 스캐너 입장에서
  "회전 포락면"으로 환원되고, **트래킹 생존은 평균 프레임이 아니라 최악
  프레임(maximin)** 이 결정한다 (납작한 물체의 edge-on 순간).

기존 방법(sphere sampling: el 고정 + standoff=r_max 스칼라 + 첫 IK-feasible
azimuth) 대비 강건화 포인트:
  1) 물체 분리 = **캘리브 기하 크롭** (디스크상단 평면 + 축 실린더).
     3D 클러스터링/모션차분 불필요 → "턴테이블 타게팅" 실패모드 구조적 차단.
  2) 명시적 센서모델 (frustum ∩ 작동거리대역 ∩ 입사각) 로 전회전 프레임별
     가시면적 곡선을 예측 → **최악 프레임 fill 을 최대화** 하는 (el, tz,
     standoff) 선택. azimuth 는 도달성(IK)만 좌우하므로 backend 가 스윕.
  3) 단일 자세로 z-대역을 못 덮으면(긴 물체) **겹침 있는 z-밴드 분할** —
     밴드 간 겹침이 곧 Artec relocalization 성립 조건.
  4) tracking-lost recovery = **같은 채점기 + overlap 항** (재정착하려면
     이미 스캔된 면이 보여야 함) 으로 재계획.

sim/real 공용: 입력은 점군(np) + 캘리브 상수뿐. 점군을 모으는 실행 루프도
`collect_planning_points` 로 공용화돼 있어(2026-09-09) sim·real 이 턴테이블
4방향 실루엣 + 조준높이 상승이라는 같은 전략을 쓴다. IK/충돌은 backend 몫.
카메라 규약 = USD look-at (광축 -Z, up=world+z) — real 이식 시 광축 규약만
backend 에서 맞춘다.
"""
from __future__ import annotations

import math
import os
from dataclasses import dataclass, field, replace
from typing import List, Optional

import numpy as np


# ── 센서 모델 (Artec Space Spider 기본값) ────────────────────────────────────
@dataclass
class SensorModel:
    # ⚠ 2026-09-16 정정 — 가로/세로가 **뒤집혀 있었다.**
    #   옛 값(hfov 30° / vfov 22.62°)은 가로가 넓다고 봤지만, 실측 K
    #   (`config/calibration/artec_intrinsic.yaml`: fx 2519 / fy 2513,
    #   **960×1280 세로형**, 재투영 0.484px)로 계산하면 정반대다:
    #       가로(960축) = 2·atan(960/2/2519) = 21.58°
    #       세로(1280축) = 2·atan(1280/2/2513) = 28.58°
    #   `scripts/artec/gen_calib_poses.py` 에 이미 정정 기록이 있었는데 이 모델에는
    #   전파되지 않았다. **세로 FOV 는 한 자세가 덮는 물체 '높이' 를 결정하는 축**
    #   이어서, 27% 과소평가가 곧 z-커버 과소평가 → 밴드 과다분할 → 밴드별
    #   minfill 과소평가로 이어졌다.
    hfov_deg: float = 21.58                # 수평 FOV (960축, 실측 K)
    vfov_deg: float = 28.58                # 수직 FOV (1280축, 실측 K)
    # 한 프레임의 **유효 심도** (m, 광축 깊이) — 스캐너 스펙의 작동거리
    # 170~350mm ([[project_artec_spider_v1]]) 와 혼동하지 말 것. 그건 스탠드오프를
    # 잡을 수 있는 범위고, 한 프레임에서 실제로 데이터가 나오는 깊이 창은 더 좁다.
    #   2026-09-16 실측으로 확인: 이 값을 (0.17, 0.35) 로 넓히자 모델이
    #   "단일 자세로 173mm 커버 (z_cover 0.88)" 라고 판단했지만, 실물에서 자세당
    #   실제 캡처는 ~87mm 였다. 넓히면 밴드가 사라져 커버리지가 더 나빠진다.
    dof: tuple = (0.20, 0.30)
    #: 품질 기여 입사각 한계 (누적/coverage 예측). ★ **캡처 필터(sim MAX_INCIDENCE_DEG
    #  50°)보다 5° 엄격**하게 둔다 — 같은 값이면 "모델상 겨우 덮는" 자세가 실제 캡처에서는
    #  경계에서 통째로 잘린다. 실측 2026-09-18 세제: 뚜껑 윗면(법선 +87°)을 el=40°
    #  밴드가 입사각 50.0° 로 "덮었다" 고 계산해 뚜껑 보강 자세를 빼먹었고, 캡처에서는
    #  전량 기각 → 최종 메시 뚜껑에 L=54mm 구멍. nbv 는 Poisson 이 그 작은 구멍을
    #  닫아 버려 gap 으로 못 봤다. 45° 면 같은 점군에서 뚜껑 자세가 다시 들어간다
    #  (밴드 수 4 그대로, 모델 커버 91.7→88.9% 는 예측이 정직해진 것).
    max_incidence_deg: float = 45.0
    track_incidence_deg: float = 75.0      # 트래킹 기여 한계 (grazing 도 추적엔 기여)
    voxel_m: float = 0.002                 # 면적 정규화 복셀 (fill 단위 산출)

    @property
    def tan_h(self):
        return math.tan(math.radians(self.hfov_deg) / 2.0)

    @property
    def tan_v(self):
        return math.tan(math.radians(self.vfov_deg) / 2.0)

    def cm2(self, n_pts: int) -> float:
        """복셀 점수 → 면적(cm²). 점군은 voxel_m 다운샘플 전제."""
        return float(n_pts) * (self.voxel_m * 100.0) ** 2


def sensor_from_scanning_range(near_mm, far_mm, base: "SensorModel" = None,
                               log=None) -> "SensorModel":
    """스캐너가 **실제로 쓰는 스캔 범위**로 `SensorModel.dof` 를 채운다.

    ★ `dof` 는 밴드 수·커버리지 판정을 좌우하는 값인데 여태 **하드코딩 추측**
      (0.20~0.30m)이었다. Artec SDK 는 이 창을 그대로 알려준다
      (`IScanningProcedure::getScanningRange` / `IFrameProcessor` 쪽도 동일) —
      추정하지 말고 **물어본 값**을 쓴다. 그래야 플래너의 가정과 스캐너의 설정이
      정의상 같아진다(예전엔 둘이 따로 놀았고, 맞는지 확인할 방법도 없었다).

    ⚠ SDK 는 **mm**, `SensorModel.dof` 는 **m** 다.
    반환: 범위를 못 읽으면 `base`(기본 SensorModel) 를 그대로 — 조용히 틀리지 않게 로그를 남긴다.
    """
    base = base or SensorModel()
    try:
        near, far = float(near_mm) / 1000.0, float(far_mm) / 1000.0
    except (TypeError, ValueError):
        if log:
            log(f"  ⚠ 스캔 범위를 못 읽음({near_mm!r}, {far_mm!r}) — dof={base.dof} 유지")
        return base
    if not (0.0 < near < far):
        if log:
            log(f"  ⚠ 스캔 범위가 이상하다 ({near*1000:.0f}~{far*1000:.0f}mm) — "
                f"dof={base.dof} 유지")
        return base
    import dataclasses as _dc
    out = _dc.replace(base, dof=(near, far))
    if log:
        log(f"  스캐너 스캔 범위 {near*1000:.0f}~{far*1000:.0f}mm 를 dof 로 사용 "
            f"(기본 가정 {base.dof[0]*1000:.0f}~{base.dof[1]*1000:.0f}mm)")
    return out


# ══════════════════════════════════════════════════════════════════════════
#  "점이 충분한가" — 세 단계의 문턱
# ══════════════════════════════════════════════════════════════════════════
#  단계가 달라 값이 다르다. 셋의 관계를 여기 한 곳에 적어 둔다(예전엔 세 파일에
#  흩어져 있고 관계가 어디에도 없었다):
#
#    ① RAW_MIN_VERTS  (1500)  스캐너가 준 **원시 정점**. 크롭 전.
#         못 넘으면 "스캐너가 물체를 못 봤다" — 그 캡처는 통째로 버린다.
#         real 은 `adaptive_min_preview_verts` 로 이미 하고 있었고 sim 은 없었다.
#    ② MIN_USEFUL_PTS (200)   **크롭 후** 물체 점. 거리 보정·실루엣 누적의 문턱.
#         ①을 넘겨도 여기서 걸릴 수 있다 — 스캐너는 뭔가 봤는데 그게 물체가
#         아닌 경우(자기점·배경·원판)다. 그때도 '반환 없음'으로 보고 탐침한다.
#    ③ MIN_PLAN_PTS   (100)   preview **전체 누적**. 이보다 적으면 플래너 포기.
#         ②를 통과한 캡처가 하나도 없거나 voxel 다운샘플 뒤 너무 적을 때다.
#
#  즉 ① → ② 는 매 캡처, ③ 은 수집이 다 끝난 뒤 한 번이다.
RAW_MIN_VERTS: int = 1500
MIN_USEFUL_PTS: int = 200
MIN_PLAN_PTS: int = 100
#: preview 에서 턴테이블을 세울 각도. **물체를 전 방위에서 봐야** 플래너가
#  "못 본 쪽" 을 "없는 면" 으로 오독하지 않는다(`collect_planning_points` 주석).
PREVIEW_THETAS = (0.0, math.pi / 2, math.pi, 3 * math.pi / 2)

#: 물체 꼭대기를 정할 때 "위에서 내려오며 처음으로 전체의 이 비율 이상을 담는 5mm 구간".
#  분위수(p1·max)는 성긴 잡음 꼬리에 끌린다 — 2026-09-23 run_102221: preview p1 = 119mm 인데
#  밀도 기준은 75~85mm, master 스캔도 80~85mm. 그 40mm 차이로 (1) 윗밴드가 허공(109·119mm)을
#  돌아 매번 비었고, (2) flip 되돌리기 반사면(H/2)이 20mm 위로 잡혀 flip 의 뒷면이 master 의
#  윗면 높이에 겹쳤고, (3) 뚜껑 보강 자세가 허공을 겨눴다.
TOP_DENSITY_FRAC = 0.02
TOP_BIN_M = 0.005


def robust_top_height(pts_B, axis_pt, axis_dir, frac: float = TOP_DENSITY_FRAC,
                      r_max: float = 0.16, bin_m: float = TOP_BIN_M):
    """디스크면(축점, h=0) 기준 **물체 꼭대기까지의 높이 H (m, 양수)** 를 밀도로 잰다.

    반환 (H, 방향부호). 방향부호 = 물체가 있는 쪽의 h 부호(-1 = 위가 −axis_dir).
    점이 부족하면 (None, None). 단순 분위수가 아니라 5mm 구간 히스토그램에서 위에서부터
    내려오며 처음으로 `frac` 이상을 담는 구간의 **먼 쪽 경계**를 꼭대기로 본다.
    """
    P = np.asarray(pts_B, float)
    if P.ndim != 2 or len(P) < 50:
        return None, None
    a = np.asarray(axis_dir, float)
    a = a / (np.linalg.norm(a) + 1e-12)
    v = P - np.asarray(axis_pt, float)
    h = v @ a
    r = np.linalg.norm(v - np.outer(h, a), axis=1)
    h = h[r < r_max]
    if len(h) < 50:
        return None, None
    sgn = -1.0 if float(np.median(h)) < 0.0 else +1.0
    u = h * sgn                                      # 클수록 위 (양수)
    top_max = float(u.max())
    edges = np.arange(0.0, top_max + bin_m, bin_m)
    cnt, _ = np.histogram(u, bins=edges)
    need = max(30, int(round(frac * len(u))))
    for i in range(len(cnt) - 1, -1, -1):
        if cnt[i] >= need:
            return float(edges[i + 1]), sgn
    return float(np.percentile(u, 99)), sgn

#: 물체 상단을 **이미 안 뒤** 그 위를 겨눌 때의 거리탐침 횟수.
#  높이 스윕은 `tz = 관측 상단 + rise_m` 로 설계상 물체 위를 겨눈다 — 거기가 비는
#  것은 "거리가 틀렸다" 가 아니라 "물체가 끝났다" 는 신호다. 앞 높이에서 창중앙에
#  맞춰 검증된 거리를 그대로 쓰므로 한 번만 확인하면 충분하다.
ABOVE_TOP_DIST_TRIES = 1
#: 높이 오르기(확인 탐침) 생략 조건 — **그 프레임**의 밀도 상단이 조준높이 위로 이 비율 ×
#  (d·tan(vfov/2)) 안이면 물체 끝이 시야 안에 보인 것이다. 시야 반높이는 d·tan(vfov/2)·cos(el)
#  인데 el 을 여기서 모르므로 0.5 로 보수적으로 잡는다(el≤60° 에서 cos≥0.5).
#  판정을 **누적이 아니라 프레임**으로 하는 이유: 누적은 아래 밴드 점이 압도적이라 2% 문턱을
#  윗부분이 못 넘어 키 큰 물체를 조기 종료시킨다(합성시험 250mm → 125mm 에서 멈춤).
#  run_125718(눕힌 병 h=55mm): 조준 49mm·d=300 → 허용 49+38=87mm, 프레임 밀도상단 70~75mm
#  → 오르기 불필요. 옛 규칙은 max(잡음 한 점 142mm)에 끌려 167mm 허공을 네 방위 모두 찍었다
#  (8 이동 중 4 낭비 · 약 30초).
TOP_SEEN_FOV_FRAC = 0.5


@dataclass
class ViewPose:
    """축 기준 시점 파라미터 + world 카메라 자세 (USD 규약)."""
    el_deg: float
    standoff: float                        # look 타깃(축 위 tz)까지 거리 (m)
    target_z: float                        # look 타깃 z (world)
    eye_w: np.ndarray = None               # (3,)
    R_wc: np.ndarray = None                # (3,3) world→cam 회전의 역 (cam축 world표현)
    #: 이 자세가 밴드가 아니라 **윗면(뚜껑) 보강**으로 추가된 것인가.
    #  캡처 순서 정렬과 "겹침 미달 → 밴드 추가" 판단이 이 둘을 구분해야 한다 —
    #  뚜껑 자세의 겹침 부족은 밴드를 더 쪼개도 해결되지 않는다(el 이 달라서다).
    is_cap: bool = False
    #: `solve_plan_poses` 가 **실제로 채택한** 방위각. 계획 단계에서는 0 이다
    #  (방위는 관측 조건이 아니라 도달성 문제라 나중에 정해진다). 캡처 중
    #  거리추종이 **같은 el·az·tz 로 축거리만** 다시 풀려면 이 값이 필요하다 —
    #  az 가 바뀌면 도달성·충돌이 달라져 재겨냥이 엉뚱한 자세로 갈 수 있다.
    az_deg: float = 0.0


@dataclass
class PoseEval:
    fill_curve_cm2: np.ndarray             # (nθ,) 프레임별 트래킹-가시 면적
    min_fill_cm2: float                    # ★ maximin 지표
    mean_fill_cm2: float
    covered_frac: float                    # 품질-가시로 한번이라도 본 점 비율
    z_cover_frac: float                    # 품질-가시가 닿은 z-bin 비율
    seen_mask: np.ndarray = None           # (N,) bool — 품질-가시 누적 (밴드겹침/recovery 용)
    #: 전회전 중 **preview 근거가 없어 판단을 보류한** 각도 비율 (0=전부 근거 있음).
    #  이 각도들은 min/mean 에서 빠진다 — `evaluate_viewpoint_sub` 주석.
    unknown_frac: float = 0.0


@dataclass
class LookaroundPlan:
    poses: List[ViewPose]
    evals: List[PoseEval]
    banded: bool
    #: **캡처 순서상 인접한** 두 자세의 seen 겹침 (뒤 자세 기준 비율).
    #  ⚠ 예전엔 "그 전까지 누적된 master 와의 겹침" 을 넣었다. 밴드가 각자
    #    IScan 이던 시절엔 그게 relocalization 앵커 가용성이라 맞았지만, 밴드가
    #    한 IScan 으로 이어진 뒤로는(2026-09-16) **직전 자세와의 겹침**이라야
    #    로봇이 옮겨가는 동안 SLAM 이 붙어 있는지를 잰다.
    band_overlap_frac: List[float] = field(default_factory=list)
    tracking_risk: bool = False            # 최선이어도 min_fill < 임계 (recovery 대비)
    note: str = ""


# ── 기본 유틸 ────────────────────────────────────────────────────────────────
def voxel_downsample(pts: np.ndarray, v: float) -> np.ndarray:
    pts = np.asarray(pts, float)
    if len(pts) == 0:
        return pts
    keys = np.floor(pts / v).astype(np.int64)
    _, idx = np.unique(keys, axis=0, return_index=True)
    return pts[np.sort(idx)]


def crop_object_points(pts_w: np.ndarray, axis_xy, disc_top_z: float,
                       r_max: float = 0.16, z_margin: float = 0.004,
                       z_max_above: float = 0.45,
                       up_sign: float = +1.0) -> np.ndarray:
    """★ 물체 분리 = 캘리브 기하 크롭 (세그먼트/클러스터링 금지).
    디스크상단 평면 위 + 축 실린더 안 = 전부 물체. 턴테이블을 물체로
    오인하는 실패모드(기존 adaptive prescan)가 구조적으로 불가능.

    r_max 는 **디스크 반경 기준**(물체는 디스크 위에만 존재 — 캘리브 사실).
    0.25 로 두면 천장마운트 로봇 몸통(축에서 ~0.21m)이 실린더에 들어와
    preview 상단이 폭주 → 밴드 과분할 (detergent 13밴드 사건, 2026-07-08)."""
    p = np.asarray(pts_w, float)
    if len(p) == 0:
        return p
    r = np.linalg.norm(p[:, :2] - np.asarray(axis_xy, float)[None, :], axis=1)
    # ★ '디스크 위' 를 `up_sign` 으로 정한다. +1 이면 z 증가 = 위(sim world).
    #   real 의 base 는 **천장 마운트라 +Z 가 아래**여서 −1 이어야 한다
    #   (`docs/collision.md` §6.1). +1 로 두면 물체가 있는 쪽을 전부 버리고
    #   테이블 속만 남겨 **0점**이 된다 — 2026-09-16 실물에서 정점 30,074개를
    #   캡처했는데 "계획용 preview 0pt" 가 나온 원인이다.
    s = float(np.sign(up_sign)) or 1.0
    h = s * (p[:, 2] - disc_top_z)          # 디스크 상단 기준 '높이'
    m = (h > z_margin) & (h < z_max_above) & (r < r_max)
    return p[m]
    # (z-연속 필터는 폐기 — 손잡이 구멍 물체(detergent)에서 가시점 희박 구간을
    #  유령으로 오인해 상단 절단. 유령(로봇 자기점)은 filter_robot_points 로
    #  근원 제거하는 것이 정공법, 2026-07-08.)


def filter_robot_points(pts: np.ndarray, capsules, margin: float = 0.025) -> np.ndarray:
    """캡처 점군에서 **로봇 자기 점 제거** (self-filter, sim/real 공용).

    preview 가 로봇 링크/스캐너를 프레임에 담으면 그 점들이 기하 크롭 실린더에
    들어와 물체 상단을 오염(detergent 13밴드 사건의 근원). 로봇은 자기 관절각을
    아므로 FK 캡슐(capsules_from_joints(q, T_EC)) 근방 점을 지운다.
    pts 와 capsules 는 **같은 프레임**(base 권장)이어야 한다."""
    p = np.asarray(pts, float)
    if len(p) == 0 or not capsules:
        return p
    keep = np.ones(len(p), dtype=bool)
    for _, c in capsules:
        d = _point_seg_dist(p, c.p0, c.p1)
        keep &= d > (c.r + margin)
    return p[keep]


def _point_seg_dist(pts, a, b):
    ab = np.asarray(b, float) - np.asarray(a, float)
    L2 = float(ab @ ab)
    if L2 < 1e-12:
        return np.linalg.norm(pts - a[None, :], axis=1)
    t = np.clip((pts - a[None, :]) @ ab / L2, 0.0, 1.0)
    proj = a[None, :] + t[:, None] * ab[None, :]
    return np.linalg.norm(pts - proj, axis=1)


def estimate_outward_normals(pts: np.ndarray, axis_xy, k: int = 16) -> np.ndarray:
    """법선 추정 (open3d PCA) + 축-바깥 방향 정렬. open3d 없으면 radial fallback.
    (오목부/손잡이는 근사 — 트래킹 fill 프록시 용도로 충분.)"""
    pts = np.asarray(pts, float)
    c_ref = np.array([axis_xy[0], axis_xy[1], float(np.mean(pts[:, 2]))])
    try:
        import open3d as o3d
        pc = o3d.geometry.PointCloud()
        pc.points = o3d.utility.Vector3dVector(pts)
        pc.estimate_normals(o3d.geometry.KDTreeSearchParamKNN(k))
        n = np.asarray(pc.normals)
    except Exception:
        n = pts - c_ref[None, :]
        n /= (np.linalg.norm(n, axis=1, keepdims=True) + 1e-12)
        return n
    flip = np.sum(n * (pts - c_ref[None, :]), axis=1) < 0
    n[flip] *= -1.0
    return n


def look_at_R(eye, target, up=(0.0, 0.0, 1.0)) -> np.ndarray:
    """USD 규약 look-at (광축 = cam -Z). handeye_geometry.look_at_camera 와 동일."""
    eye = np.asarray(eye, float)
    f = np.asarray(target, float) - eye
    f /= (np.linalg.norm(f) + 1e-12)
    z = -f
    x = np.cross(np.asarray(up, float), z)
    if np.linalg.norm(x) < 1e-6:
        x = np.cross(np.array([0.0, 1.0, 0.0]), z)
    x /= (np.linalg.norm(x) + 1e-12)
    y = np.cross(z, x)
    return np.column_stack([x, y, z])


def make_view_pose(axis_xy, target_z: float, el_deg: float, az_deg: float,
                   standoff: float, up_sign: float = +1.0) -> ViewPose:
    """축 위 `target_z` 를 (el, az, standoff) 에서 보는 **채점용** 자세.

    ★ `up_sign` — 작업 프레임에서 어느 z 방향이 '위'인가. el 의 기준축이다.
      **실행 경로(`view_pose.eye_from_el_az`)의 `up` 인자와 같은 뜻**이고,
      둘이 어긋나면 채점과 실행이 다른 자세를 본다.

      ⚠ 2026-09-17 이전에는 이 함수가 `+Z = 위` 를 **못 박고 있었다.** sim 은
        world 프레임이라 우연히 맞았지만, real 의 base 는 천장 마운트라 +Z 가
        아래다(`docs/collision.md` §6.1). 그래서 real 은
          채점  eye = 원판면 기준 **아래**(테이블 속)에서 올려다봄
          실행  eye = 원판면 기준 **위**에서 내려다봄
        로 **원판면 대칭인 거울상**이었다(실측: el=30°·standoff 284mm 에서
        두 eye 가 284mm 떨어짐). 원통형 물체는 옆면이 비슷하게 보여 z-커버 숫자가
        그럴듯하게 나오는 바람에 오래 안 드러났지만,
          · 입사각·backface 판정이 뒤집힌 면을 본다
          · `_augment_top_face` 의 '뚜껑' 자세가 실제로는 **바닥**을 겨눈다
        는 점에서 real 의 밴드 계획이 틀린 근거로 세워지고 있었다.
    """
    el, az = math.radians(el_deg), math.radians(az_deg)
    s_up = float(np.sign(up_sign)) or 1.0
    target = np.array([axis_xy[0], axis_xy[1], target_z])
    eye = target + standoff * np.array(
        [math.cos(el) * math.cos(az), math.cos(el) * math.sin(az),
         s_up * math.sin(el)])
    return ViewPose(el_deg=el_deg, standoff=standoff, target_z=target_z,
                    eye_w=eye, R_wc=look_at_R(eye, target), az_deg=az_deg)


def _rot_z(pts, axis_xy, theta):
    c, s = math.cos(theta), math.sin(theta)
    R = np.array([[c, -s], [s, c]])
    out = pts.copy()
    out[:, :2] = (pts[:, :2] - axis_xy) @ R.T + axis_xy
    return out, R


def visible_masks(pts_w, nrm_w, pose: ViewPose, sensor: SensorModel):
    """(track_mask, quality_mask): frustum ∩ DOF ∩ (backface/입사각)."""
    pc = (pts_w - pose.eye_w[None, :]) @ pose.R_wc         # cam 좌표 (광축 -Z)
    d = -pc[:, 2]                                          # 광축 깊이
    in_f = ((d > 1e-6)
            & (np.abs(pc[:, 0]) <= d * sensor.tan_h)
            & (np.abs(pc[:, 1]) <= d * sensor.tan_v)
            & (d >= sensor.dof[0]) & (d <= sensor.dof[1]))
    v = pose.eye_w[None, :] - pts_w
    v /= (np.linalg.norm(v, axis=1, keepdims=True) + 1e-12)
    cos_inc = np.sum(nrm_w * v, axis=1)
    track = in_f & (cos_inc > math.cos(math.radians(sensor.track_incidence_deg)))
    quality = in_f & (cos_inc > math.cos(math.radians(sensor.max_incidence_deg)))
    return track, quality


def _pose_eval_from(fill, seen, z, z_bins: int, known=None) -> PoseEval:
    """전회전 결과(fill 곡선 · seen 마스크) → PoseEval.

    `known` (nθ,) bool — 그 각도에 preview 근거가 있었는가. False 인 각도는
    min/mean 에서 **뺀다**. 전부 False 면(근거가 하나도 없다) 0 으로 둔다 —
    아무것도 모르는 자세를 좋다고 할 수는 없다.
    """
    edges = np.linspace(z.min(), z.max() + 1e-9, z_bins + 1)
    bin_id = np.clip(np.digitize(z, edges) - 1, 0, z_bins - 1)
    tot = np.bincount(bin_id, minlength=z_bins)
    hit = np.bincount(bin_id[seen], minlength=z_bins)
    nz = tot > 0
    z_cover = float(np.mean((hit[nz] / np.maximum(tot[nz], 1)) > 0.3)) if nz.any() else 0.0
    if known is None:
        known = np.ones(len(fill), dtype=bool)
    f_known = fill[known]
    return PoseEval(fill_curve_cm2=fill,
                    min_fill_cm2=float(f_known.min()) if len(f_known) else 0.0,
                    mean_fill_cm2=float(f_known.mean()) if len(f_known) else 0.0,
                    covered_frac=float(seen.mean()), z_cover_frac=z_cover,
                    seen_mask=seen,
                    unknown_frac=float(1.0 - known.mean()))


#: 전회전 시뮬에서 "이 각도엔 preview 근거가 있다" 로 칠 기준.
#  카메라를 향한 ±EVIDENCE_SECTOR_DEG 방위 섹터 안의 preview 점이
#  (섹터별 중앙값 × EVIDENCE_MIN_FRAC) 보다 적으면 그 각도는 **모름**이다.
#    · 섹터 폭: preview 한 컷이 물체 옆면에서 보는 방위 폭이 ~90°(실측 세제)라
#      그 절반. 너무 좁으면 실제로 본 곳도 "모름" 이 되고, 너무 넓으면 반쪽만
#      본 것을 "다 봤다" 로 친다.
#    · 문턱을 중앙값 대비 비율로 두는 이유: 진짜 구멍(손잡이)은 preview 에서
#      뒤쪽 몸통이 구멍 너머로 잡혀 점이 0 이 아니고, 안 본 쪽은 정말 0 이다.
#      절대값이면 점 밀도(거리·해상도)에 따라 흔들린다.
#    · 비율 0.5 = "섹터의 절반은 실제로 봤어야 근거로 친다". 0.1 로 두면 본 쪽
#      가장자리의 5° 조각만 걸쳐도 근거가 돼서, 반원만 본 원통이 25% 만
#      "모름" 으로 나왔다(진실은 50%). 0.5 면 ~40% — 한 컷 폭 45° 만큼의
#      과신은 남지만 그 방향은 낙관이 아니라 보수 쪽이다.
EVIDENCE_SECTOR_DEG = 45.0
EVIDENCE_MIN_FRAC = 0.5


def _evidence_by_theta(pts_obj, axis_xy, cam_az_rad: float, n_theta: int):
    """(nθ,) bool — 물체를 θ 돌렸을 때 카메라를 향하는 섹터에 preview 점이 있는가.

    `_rot_z` 는 점을 +θ 회전시키므로, 회전 후 카메라 방위 `cam_az` 를 향하는
    점은 원래 방위가 `cam_az − θ` 인 점이다.
    """
    d = pts_obj[:, :2] - np.asarray(axis_xy, float)[None, :]
    phi = np.arctan2(d[:, 1], d[:, 0])
    half = math.radians(EVIDENCE_SECTOR_DEG)
    thetas = np.linspace(0.0, 2 * np.pi, n_theta, endpoint=False)
    cnt = np.empty(n_theta, dtype=int)
    for i, th in enumerate(thetas):
        c = cam_az_rad - th
        dphi = np.abs((phi - c + np.pi) % (2 * np.pi) - np.pi)
        cnt[i] = int((dphi <= half).sum())
    ref = float(np.median(cnt))
    return cnt >= max(1.0, EVIDENCE_MIN_FRAC * ref)


def evaluate_viewpoint(pts_obj, nrm_obj, axis_xy, pose: ViewPose,
                       sensor: SensorModel, n_theta: int = 36,
                       z_bins: int = 24) -> PoseEval:
    """전회전 시뮬: 물체를 θ 회전시키며 프레임별 트래킹-가시 면적 곡선 산출.
    ★ min(fill_curve) = 최악 프레임 = 트래킹 생존 지표 (maximin)."""
    return evaluate_viewpoint_sub(pts_obj, nrm_obj, axis_xy, pose, sensor,
                                  None, n_theta, z_bins)[0]


def evaluate_viewpoint_sub(pts_obj, nrm_obj, axis_xy, pose: ViewPose,
                           sensor: SensorModel, sub_mask=None,
                           n_theta: int = 36, z_bins: int = 24):
    """전회전 시뮬 **한 번**으로 `(전체 점군 평가, sub_mask 안 평가)` 둘 다 낸다.

    왜 둘이 같이 필요한가 — 밴드 자세는 **두 기준으로 판단**된다:
      · 그 밴드를 얼마나 잘 덮나  → 밴드 안 점만 본 평가 (자세 선택)
      · 회전 내내 추적이 붙나     → **전체 점군** 최악 프레임 (버릴지 판정)
    예전에는 선택을 앞의 기준으로, 버림을 뒤의 기준으로 해서 **플래너가 스스로
    버릴 자세를 골랐다**. 실측 2026-09-18(세제 284mm): 7밴드 중 4밴드가 그렇게
    버려졌고, 같은 밴드도 다른 el·축거리를 고르면 전체 minfill 이 0.0 → 8~30cm²
    였다. 두 평가를 같은 루프에서 내면 선택이 버림 기준을 볼 수 있다.

    `sub_mask=None` 이면 둘째 값은 None 이다.

    ★ "못 봤다" ≠ "없다". preview 점군에 **근거가 없는 방위**는 min/mean 에서
      뺀다(`_evidence_by_theta`, `PoseEval.unknown_frac`). 예전에는 preview 가
      안 본 쪽이 "면이 없다" 로 채점돼 그 각도의 fill 이 0 이 됐고, maximin 이
      그걸 최악 프레임으로 잡아 자세를 통째로 기각했다(2026-09-18 세제:
      아랫밴드 전부 기각, 원인은 preview 2방향). preview 를 4방향으로 늘려
      근거 자체를 채우는 것이 1차 수정이고, 이것은 실물의 어두운/반사 면처럼
      그래도 남는 구멍에 대한 2차 방어다.
      ⚠ 근거 없는 각도를 빼는 것은 낙관이다 — 거기가 정말 빈 곳이면 추적이
        끊길 수 있다. 그 경우는 watchdog·recovery·nbv 가 받는다. 비율은
        `unknown_frac` 으로 남겨 계획 로그에 찍힌다.
    """
    axis_xy = np.asarray(axis_xy, float)
    N = len(pts_obj)
    fill = np.zeros(n_theta)
    seen = np.zeros(N, dtype=bool)
    sub = None if sub_mask is None else np.asarray(sub_mask, bool)
    fill_s = None if sub is None else np.zeros(n_theta)
    seen_s = None if sub is None else np.zeros(int(sub.sum()), dtype=bool)
    cam_az = math.atan2(pose.eye_w[1] - axis_xy[1], pose.eye_w[0] - axis_xy[0])
    known = _evidence_by_theta(pts_obj, axis_xy, cam_az, n_theta)
    known_s = (None if sub is None
               else _evidence_by_theta(pts_obj[sub], axis_xy, cam_az, n_theta))
    for i, th in enumerate(np.linspace(0.0, 2 * np.pi, n_theta, endpoint=False)):
        p_th, R2 = _rot_z(pts_obj, axis_xy, th)
        n_th = nrm_obj.copy()
        n_th[:, :2] = nrm_obj[:, :2] @ R2.T
        tr, qu = visible_masks(p_th, n_th, pose, sensor)
        fill[i] = sensor.cm2(int(tr.sum()))
        seen |= qu
        if sub is not None:
            fill_s[i] = sensor.cm2(int(tr[sub].sum()))
            seen_s |= qu[sub]
    z = pts_obj[:, 2]
    ev = _pose_eval_from(fill, seen, z, z_bins, known)
    ev_s = (None if sub is None
            else _pose_eval_from(fill_s, seen_s, z[sub], z_bins, known_s))
    return ev, ev_s


def contiguous_z_span(z, mask, z_bins: int = 48, frac: float = 0.3) -> float:
    """덮인 **연속** z 구간 길이 (m).

    ⚠ `np.ptp(z[mask])` 로 재면 안 된다 — 최대−최소라서 **위아래만 걸치고
      가운데가 빈** 자세도 "다 덮었다" 로 나온다. 그 경우를 걸러내려고 도입한
      지표인데 구현이 같은 함정을 갖고 있었다.
      실측 2026-09-17: hand_drill ptp 154mm vs 연속 126mm, protein_drink 는
      이것만으로 단일/밴드 판정이 뒤집힌다.

    bin 판정 기준(`frac`)은 `evaluate_viewpoint` 의 z_cover 와 같다 — 같은
    "덮였다" 를 두 곳이 다르게 정의하면 둘을 같이 못 읽는다.
    """
    z = np.asarray(z, float)
    mask = np.asarray(mask, bool)
    if int(np.count_nonzero(mask)) < 30 or len(z) == 0:
        return 0.0
    edges = np.linspace(z.min(), z.max() + 1e-9, z_bins + 1)
    b = np.clip(np.digitize(z, edges) - 1, 0, z_bins - 1)
    tot = np.bincount(b, minlength=z_bins)
    hit = np.bincount(b[mask], minlength=z_bins)
    ok = (tot > 0) & (hit / np.maximum(tot, 1) > frac)
    best = run = 0
    for v in ok:
        run = run + 1 if v else 0
        best = max(best, run)
    return float(best) * float(edges[1] - edges[0])


# ── 계획용 preview 캡처 전략 (거리스텝, sim/real 공용 로직) ─────────────────
def simulate_planning_captures(pts_obj, nrm_obj, axis_xy, disc_top_z,
                               sensor: SensorModel = None,
                               d_steps=None, azs=(0.0, 90.0),
                               el_deg=25.0, max_heights=4, up_sign: float = +1.0):
    """real 계획용 preview 캡처 전략의 sim 모사 (★ 엄격 실기 DOF 그대로).

    제약의 본질 = 스캐너→표면 거리(작동거리 창). 축에서 거리 d 캡처가 잡는
    표면 반경 = `[d−far, d−near]` → 창 폭 간격으로 스텝을 놓으면 반경 0~18cm 를
    덮는다(어느 스텝에 잡히는지 자체가 r 측정). 격자는 **`standoff.preview_grid`
    가 창에서 유도**한다 — 여기서 하드코딩하면 실기와 갈라진다. 키큰 물체는
    표면거리를 유지한 채 조준높이(tz)만 올려 추가 캡처 — 각 preview 는 FK
    좌표로 독립이라 SLAM/relocalization 불필요.

    종료 조건은 real 과 동일하게 GT 없이: "새 캡처가 상단을 더 못 늘리면 stop".
    반환 = 계획용으로 보인 점 마스크 (real 에선 이 점들이 preview 점군)."""
    sensor = sensor or SensorModel()
    if not d_steps:
        from utils.nbv.standoff import preview_grid
        d_steps = preview_grid(sensor.dof)
    axis_xy = np.asarray(axis_xy, float)
    z = pts_obj[:, 2]
    seen = np.zeros(len(pts_obj), dtype=bool)
    for az in azs:
        tz = float(disc_top_z) + 0.05          # 첫 조준: 디스크 위 5cm (prior)
        prev_top = -np.inf
        for _ in range(max_heights):
            for d in d_steps:
                pose = make_view_pose(axis_xy, tz, el_deg, az, d, up_sign)
                tr, _ = visible_masks(pts_obj, nrm_obj, pose, sensor)
                seen |= tr
            top = float(z[seen].max()) if seen.any() else tz
            if top - prev_top < 0.01:          # 상단이 안 늘면 종료 (GT 불요)
                break
            prev_top = top
            tz = top + 0.03                    # 다음 조준: 현재 상단 위 (겹침)
    return seen


# ── 계획용 preview 수집 / 계획 자세 IK 해결 (sim·real 공용 실행 루프) ─────────
# sim(isaac_scan_session) 과 real(artec_multipass_scan_session) 이 **같은 코드**를
# 쓰도록 백엔드 의존 부분만 콜백으로 뺐다. 예전에는 이 루프가 sim 에만 있었고
# real 은 home 고정이라, sim 에서 검증한 자세 선정이 실물에 전혀 적용되지 않았다.

# ── `standoff` 재수출 ──────────────────────────────────────────────────────
#  백엔드는 `lookaround` 를 `p1` 로 들고 다닌다. 거리 관련 상수·함수가 `standoff`
#  에 있다고 해서 백엔드가 두 모듈을 따로 import 하게 하면, 한쪽만 고쳐서 갈라질
#  자리가 하나 더 생긴다. preview 가 쓰는 것만 여기서 같이 내보낸다.
#  (`standoff` 는 `lookaround` 를 import 하지 않으므로 순환이 없다.)
from utils.nbv.standoff import (                              # noqa: E402,F401
    PREVIEW_RADIUS_MAX_M, probe_bounds, blind_probe, preview_grid,
    preview_start_distance, distance_correction)

#: 스캔 대상 장애물을 축 둘레로 몇 방향 쓸어 등록할지 (sim·real 공용).
#  1 = 끄기(θ=0 그대로). 예전엔 sim 만 env 로 바꿀 수 있어 조용히 갈라질 수 있었다.
OBSTACLE_SWEEP_N: int = int(os.environ.get("MMS_OBSTACLE_SWEEP", "12"))


def guard_cylinder_points(axis_pt, up_sign: float, radius_m: float,
                          height_m: float = None, n_z: int = 24,
                          n_r: int = 8, n_th: int = 24):
    """preview 동안 축 둘레를 막는 **보수적 원기둥** 점군 (sim·real 공용).

    왜 필요한가 — 물체를 장애물로 등록하려면 먼저 물체를 봐야 하는데 preview 가
    바로 그 '보는' 단계다. 그동안 충돌 모델에는 대상이 **없다.** preview 는 축거리
    200~480mm 를 훑으며 최대 48회 움직이므로 무방호로 두기엔 길다.

    `radius_m` 은 **원판 반경**을 넘지 않게 줄 것 — 물체는 원판 위에 서 있으므로
    그보다 넓을 수 없다(넘어진다). `PREVIEW_RADIUS_MAX_M`(180mm)은 거리격자가
    상정하는 최대 반경이지 물체가 실제로 가질 수 있는 최대가 아니다.

    ⚠ **반경이 크면 preview 자세를 막는다.** 카메라의 수평거리는 `d·cos(el)` 이라
      el=30° 면 축거리 220mm 가 수평 191mm 다. keepout(반경+마진)이 210mm 면 그
      자세가 기각되고, 탐침 사다리의 220mm 칸이 통째로 사라진다. 게다가 증상이
      **조용하다** — 이동이 거부되면 빈 배열이 돌아가고 코어는 그걸 "스캐너가 못
      봤다" 로 읽어 다음 탐침으로 넘어간다. 로봇은 가지도 않았는데 말이다.
      그래서 `guard_blocks_probe()` 로 미리 검사해 경고한다.
    """
    a = np.asarray(axis_pt, float)
    u = float(np.sign(up_sign)) or 1.0
    R = float(radius_m)
    H = float(radius_m * 2.0 if height_m is None else height_m)
    z0 = float(a[2]); z1 = z0 + u * H
    zs = np.linspace(min(z0, z1), max(z0, z1), int(n_z))
    th = np.linspace(0.0, 2 * math.pi, int(n_th), endpoint=False)
    rs = np.linspace(R / float(n_r), R, int(n_r))
    return np.array([[a[0] + r * math.cos(t), a[1] + r * math.sin(t), z]
                     for z in zs for r in rs for t in th], float)


def guard_blocks_probe(radius_m: float, margin_m: float, el_deg: float,
                       probe_lo_m: float):
    """보호 원기둥이 탐침 사다리를 막는가 → (막는가, 최소가능 축거리).

    카메라 수평거리 = `d·cos(el)`. keepout = 반경 + 마진. 그러므로 안전한 최소
    축거리는 `(R+M)/cos(el)` 이고, 그게 탐색 하한보다 크면 그 사이 구간을 못 쓴다.
    """
    c = max(math.cos(math.radians(float(el_deg))), 1e-6)
    d_min = (float(radius_m) + float(margin_m)) / c
    return (d_min > float(probe_lo_m) + 1e-9), d_min


def revolution_envelope(pts, axis_pt, axis_dir, n_z: int = 48, n_th: int = 36,
                        n_r: int = 4, pad_m: float = 0.0):
    """점군의 축 둘레 **회전체 외피** — 충돌 장애물용 (sim·real 공용).

    높이별로 최대 반경을 재서 그 반경의 원기둥면을 쌓는다. 즉 **점군을 축 둘레로
    연속 회전시킨 부피**다.

    왜 `swept_about_axis`(이산 N방향) 대신 이것인가
    ------------------------------------------------
    12방향 샘플은 **사이에 쐐기 모양 틈이 남는다.** 손잡이처럼 얇은 돌출부는 그
    틈으로 로봇이 지나갈 수 있다 — 30° 간격이면 반경 100mm 지점에서 이웃 샘플
    사이가 52mm 벌어진다. 회전체 외피는 그 틈이 **원리적으로** 없다.

    왜 고정 원기둥(원판 반경)이 아닌가
    ----------------------------------
    그쪽이 더 안전해 보이지만 **고도각 높은 자세를 전부 기각한다.** 카메라 수평거리는
    `d·cos(el)` 이라 el 이 클수록 축에 가까워진다 — 실측(축거리 259~284mm):

        lookaround el=30°  수평 246mm   원판원기둥(150+30) OK
        nbv        el=50°  수평 166mm   → 기각
        flip       el=70°  수평  89mm   → 기각
        뚜껑 보강   el=70°  수평  97mm   → 기각

    물체가 반경 34mm 인데 150mm 로 막으면 계획이 통째로 사라진다. 외피는 물체만큼만
    굵어서(34+여유) 이 자세들이 다 통과한다.

    `pad_m` 은 반경에 더하는 여유. 0 이면 실측 그대로 — 충돌 게이트의 마진
    (`SCAN_OBSTACLE_MARGIN_M`)이 따로 붙으므로 보통 0 으로 둔다.
    """
    P = np.asarray(pts, float)
    if len(P) == 0:
        return P
    a = np.asarray(axis_pt, float)
    d = np.asarray(axis_dir, float)
    d = d / (np.linalg.norm(d) + 1e-12)
    # 축에 수직한 정규직교 두 축 (프레임 무관하게 만든다)
    tmp = np.array([1.0, 0.0, 0.0]) if abs(d[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
    u = np.cross(d, tmp); u /= (np.linalg.norm(u) + 1e-12)
    v = np.cross(d, u)

    t = P - a
    h = t @ d                                   # 축 좌표
    rad = np.linalg.norm(t - np.outer(h, d), axis=1)
    h_lo, h_hi = float(h.min()), float(h.max())
    if h_hi - h_lo < 1e-6:                      # 평면에 가까우면 한 층으로
        h_hi = h_lo + 1e-3
    edges = np.linspace(h_lo, h_hi, int(n_z) + 1)
    idx = np.clip(np.digitize(h, edges) - 1, 0, int(n_z) - 1)

    th = np.linspace(0.0, 2 * math.pi, int(n_th), endpoint=False)
    ring = np.cos(th)[:, None] * u[None, :] + np.sin(th)[:, None] * v[None, :]
    out = []
    for k in range(int(n_z)):
        m = idx == k
        if not m.any():
            continue
        r_max = float(rad[m].max()) + float(pad_m)
        if r_max <= 1e-6:
            continue
        hc = 0.5 * (edges[k] + edges[k + 1])
        c = a + hc * d
        # 표면만이 아니라 안쪽으로 몇 겹 — SDF 가 내부를 '멀다'고 읽지 않게
        for r in np.linspace(r_max / max(1, int(n_r)), r_max, int(n_r)):
            out.append(c[None, :] + r * ring)
    return np.vstack(out) if out else P


def swept_about_axis(pts, axis_pt, axis_dir, n: int = None):
    """점군을 축 둘레로 `n` 방향 쓸어 합친 **회전체** (충돌 장애물용, sim·real 공용).

    왜 필요한가 — 스캔 점군은 `θ=0` canonical 로 쌓인다(`rot_about_axis(−θ)`).
    그런데 로봇이 움직이는 시점의 턴테이블 각은 0 이 아니다(nbv 는 부분 스윕이라
    특히). 비대칭 물체면 장애물이 **실제와 다른 방향을 향한 채** 걸려 있어,
    있지도 않은 곳을 막고 정작 손잡이가 있는 쪽은 뚫린다.

    회전체로 만들면 턴테이블이 어느 각도에 있든 실제 물체를 포함한다. 보수적이지만
    카메라는 축에서 200mm 밖에 서므로 관측 자세가 기각되지는 않는다.
    """
    if n is None:
        n = OBSTACLE_SWEEP_N
    P = np.asarray(pts, float)
    if len(P) == 0 or int(n) <= 1:
        return P
    return np.vstack([rot_about_axis(P, axis_pt, axis_dir, a)
                      for a in np.linspace(0.0, 2 * math.pi, int(n), endpoint=False)])


def rot_about_axis(pts, axis_pt, axis_dir, ang):
    """임의 축(axis_pt, axis_dir) 둘레로 점군을 ang(rad) 회전. sim·real 공용."""
    a = np.asarray(axis_dir, float)
    a = a / (np.linalg.norm(a) + 1e-12)
    c, sn = math.cos(ang), math.sin(ang)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    R = np.eye(3) * c + np.outer(a, a) * (1 - c) + K * sn
    return (np.asarray(pts, float) - np.asarray(axis_pt, float)) @ R.T \
        + np.asarray(axis_pt, float)


def _unpack_preview(got):
    """`preview_at` 반환을 **(pts, cam_pos, legacy)** 로 정규화.

    `legacy` 는 **계약**을 말한다 — 콜백이 `(점군, 카메라위치)` 가 아니라 점군만
    돌려주는 옛 구현인가. `cam_pos is None` 과 **반드시 구분해야 한다.**

    ⚠ 예전엔 이 둘을 `cam is None` 하나로 겸했다. 그래서
      · 빈 캡처로 카메라 위치를 못 준 프레임
      · `prim_world_pose` 가 한 번 튄 프레임
      이 전부 "옛 계약" 으로 오인돼 **실행 내내 적응이 영구히 꺼졌다.** 게다가 그
      판정이 `blind_probe` 앞에 있어서, 빈 캡처가 탐침에 도달조차 못 했다 —
      적응이 가장 필요한 순간에 정확히 꺼지는 구조였다.

      계약은 **모양**으로만 본다(2-tuple 인가). 값이 None 인 것은 그 프레임의
      사정일 뿐이다.
    """
    if got is None:                       # 콜백이 아무것도 못 냄 — 계약 문제 아님
        return None, None, False
    if isinstance(got, tuple) and len(got) == 2:
        pts, cam = got
        return ((None if pts is None else np.asarray(pts, float)),
                (None if cam is None else np.asarray(cam, float)),
                False)
    return np.asarray(got, float), None, True


def next_distance(d, pts, cam_pos, dof, lo, hi, blind_step=0.04):
    """적응적 preview 의 다음 축거리. **밴드 캡처 중 거리추종과 같은 식**을 쓴다.

    실제 계산은 `utils.nbv.standoff.distance_correction` 한 곳에만 있다 — preview 와
    밴드가 다른 기준으로 거리를 정하면 "preview 는 맞췄는데 본 스캔은 멀다" 가 된다.
    여기서는 이동량 제한을 걸지 않는다: preview 는 아직 녹화 중이 아니라 카메라가
    훌쩍 뛰어도 SLAM 이 끊길 게 없고, 오히려 한 번에 가는 편이 이동 횟수가 준다.
    """
    from utils.nbv.standoff import distance_correction
    return distance_correction(d, pts, cam_pos, dof, lo, hi,
                               blind_step=blind_step, max_step=None)


def collect_planning_points(preview_at, move_turntable, axis_pt, axis_dir,
                            d_steps=None, thetas=PREVIEW_THETAS,
                            max_heights: int = 4, rise_m: float = 0.03,
                            start_off_m: float = 0.05, top_eps_m: float = 0.01,
                            up_sign: float = +1.0, sensor: "SensorModel" = None,
                            adaptive: bool = True, max_dist_tries: int = 6,
                            min_pts: int = MIN_USEFUL_PTS, log=None):
    """계획용 preview 를 모아 물체 프레임(θ=0) 점군으로 돌려준다.

    백엔드는 두 콜백만 제공한다.
      preview_at(tz, d) -> (N,3) | ((N,3), cam_pos)
          조준높이 tz·축거리 d 로 구동 후 캡처하고 물체 점만 크롭해 돌려준다.
          **카메라 위치를 같이 주면 적응적 거리 조절이 켜진다**(아래).
      move_turntable(theta_rad)   턴테이블을 절대각으로 돌리고 완료까지 대기.

    실루엣은 턴테이블 **4방향(0/90/180/270°)** 에서 얻는다(`PREVIEW_THETAS`).
    각 각도의 점군은 -θ 로 역회전해 물체 프레임으로 통일한다. 조준높이는 "새
    캡처가 상단을 더 못 늘리면 종료"로 올리므로 물체 높이에 대한 사전지식(GT)이
    필요없다.

    ★ 왜 2방향(0/90°)이 아니라 4방향인가 — 2방향은 **물체의 반쪽만** 본다.
      실측 2026-09-18(세제, 반경 97mm): 0~120mm 높이에서 방위 0~150° 는 점이
      **0개**, 150~330° 만 1,100~1,500점. 가는 윗부분은 el=30 에서 넘어다보여
      전 방위가 잡히지만 넓은 아랫부분은 카메라 쪽 반원만 잡힌다. 플래너는 그
      빈 반쪽을 "면이 없다" 로 읽어 아랫밴드 전부를 "회전 중 60° 동안 추적
      끊김" 으로 기각했다 — 실제로는 손잡이 구멍도 뭣도 아닌, 안 본 쪽이었다.
      같은 점군의 빈 반쪽을 채워서 돌리면 품질 커버 58% → 94% 가 된다.
      추가 각도는 거리 적응 상태(`d_cur`)를 이어받아 높이 오르기만 하므로
      비용은 2배가 아니라 그보다 적다(세제: 이동 10회 → ~20회).

    ★ 거리 결정 — **적응적**(`adaptive=True`, 기본)
      예전에는 고정 격자(`d_steps`)를 매 높이마다 전부 돌았다.
      그 값은 "반경 0~18cm 를 창 안에 넣는다" 는 커버리지 기준이라, 작은 물체에선
      먼 스텝이 빈 캡처가 된다(sim 실측: d=380mm 에서 3,233점 = 사실상 0).
      그런데도 **매번 이동은 한다** — 실물에서 그냥 시간 낭비다.

      이제는 한 장 찍고 `next_distance()` 로 다음 거리를 **계산**한다. 표면까지
      거리 중앙값이 창 중앙에 오도록 옮기므로, 보통 1~2회면 수렴한다.
      `preview_at` 이 카메라 위치를 안 주면 옛 고정 격자로 되돌아간다(하위호환).

    `min_pts` — **이만큼 안 들어오면 "안 들어온 것"으로 친다** (기본 200).
      거리 보정은 표면거리의 **중앙값**에 기대는데, 점이 서너 개뿐이면 그 중앙값은
      물체가 아니라 자기점 잔여·배경 노이즈다. 그걸 믿으면 카메라가 엉뚱한 쪽으로
      가고, "점이 있다" 는 이유로 탐침(`blind_probe`)까지 건너뛰어 회복 수단이
      사라진다. 실물 preview 는 보통 수만 점이라 200 은 아주 낮은 문턱이다
      (sim 실측 2026-09-17: 최소 15,912점). 실루엣 누적 문턱도 겸한다 — 거리에
      못 믿을 점군이면 실루엣으로도 못 믿는다.

    `up_sign` — 작업 프레임에서 **어느 z 방향이 '위'인가** (+1 = +Z 가 위).
      sim 은 world 프레임이라 +1 이 맞다. real 은 base 프레임을 쓰는데 이 셀의
      base 는 **천장 마운트라 +Z 가 아래**여서 **−1** 이어야 한다
      (`docs/collision.md` §6.1).
      +1 로 두면 시작 조준높이가 `axis_pt.z + 0.05` = 원판 표면보다 50mm **아래**
      (테이블 속)가 되고, 높이를 올릴수록 더 파고든다. 그래서 물체가 아니라
      **턴테이블을 겨눈다** — 2026-09-16 실물에서 관측된 증상이다.
      상단 판정(`top`)도 같은 이유로 부호를 따른다: base 프레임에서 물체의
      '위' 는 z 최대가 아니라 **z 최소**다.
    """
    axis_pt = np.asarray(axis_pt, float)
    sensor = sensor or SensorModel()
    dof = sensor.dof
    # ★ 거리격자·탐색한계·시작거리 **전부 작동거리 창에서 유도**한다. 예전엔
    #   격자가 sim (300,380) / real (240,300) 으로 하드코딩돼 갈라져 있었고,
    #   real 쪽은 반경 40mm 만 넘어도 근접한계 안쪽이라 반환이 없었다.
    #   창이 바뀌면(스캐너 설정·기종) 하드코딩은 조용히 틀려진다.
    from utils.nbv.standoff import (probe_bounds, blind_probe, preview_grid,
                                    preview_start_distance)
    d_lo, d_hi = probe_bounds(dof)
    if not d_steps:
        d_steps = preview_grid(dof)
    d_cur = float(np.clip(preview_start_distance(dof), d_lo, d_hi))
    acc = []
    theta_of = []        # acc[i] 가 어느 턴테이블 각에서 왔나 (진단용)
    prev_top = -np.inf   # 확정된 물체 상단 (s·z 스칼라) — 방위 간 공유
    n_cap = n_move = 0
    for theta in thetas:
        # ★ 회전 성공 여부를 **확인한다.** 실패했는데 그대로 찍으면 다른 방위의
        #   실루엣이라고 믿으면서 실제로는 직전 각도의 점군을 한 번 더 쌓는다 —
        #   `rot_about_axis(..., -theta)` 가 있지도 않은 회전을 되돌리므로 점들이
        #   엉뚱한 곳으로 간다. 조용히 계획만 나빠져서 원인 추적이 어렵다
        #   (2026-09-16 real). 콜백이 bool 을 안 주는 구현(sim)은 None → 통과.
        moved = move_turntable(float(theta))
        if moved is False:
            if log:
                log(f"θ={math.degrees(theta):+.0f}° 회전 실패 — 이 방위 실루엣 건너뜀")
            continue
        # 높이는 **up_sign 방향으로** 올린다. 아래 h(·) 는 "위쪽 높이" 스칼라이고
        # up_sign=+1 이면 기존 식과 완전히 같다.
        s = float(np.sign(up_sign)) or 1.0
        # ★ 상단은 방위 간 공유 — 앞 방위에서 확정한 상단(prev_top)보다 이 방위의 점이
        #   top_eps 이상 높지 않으면 오르지 않는다. 예전엔 방위마다 −inf 로 초기화해
        #   같은 확인 탐침을 4번 반복했다.
        tz = float(axis_pt[2]) + s * start_off_m
        for _ in range(max_heights):
            frame_pts = None                   # 이 높이에서 마지막으로 쓸 만했던 프레임
            if adaptive:
                d, d_anchor, n_blind = d_cur, d_cur, 0
                # ★ **"물체가 없다" 와 "거리가 틀렸다" 를 구분한다.**
                #   거리탐침은 "점이 없다 → 가까운지 먼지 모른다" 를 전제로 양쪽을
                #   벌려 본다. 그런데 두 번째 높이부터는 이미 물체 상단을 알고
                #   (`prev_top`), 조준을 **일부러 그 위 `rise_m`** 로 올린 참이다.
                #   거기가 비는 것은 정상이지 거리 문제가 아니다. 게다가 거리는
                #   앞 높이에서 "창중앙에 맞다" 고 검증된 값을 그대로 쓴다.
                #   그걸 구분 못 해 매 높이마다 6회를 헛돌았다 — 실측 2026-09-21:
                #   **29 이동 중 4회만 유효**(가까운 220·260mm 에서도 0점이었다).
                #   상단을 아는 뒤에는 한 번만 확인하고 물러난다.
                _tries = (max_dist_tries if not np.isfinite(prev_top)
                          else ABOVE_TOP_DIST_TRIES)
                for _k in range(_tries):
                    got = preview_at(float(tz), float(d)); n_move += 1
                    pts, cam, legacy = _unpack_preview(got)
                    if legacy:              # 콜백이 점군만 준다 → 고정 격자로
                        adaptive = False
                        if log:
                            log("preview_at 이 (점군, 카메라위치) 를 안 준다 "
                                "— 고정 거리격자 사용")
                        break
                    n_pts = 0 if pts is None else len(pts)
                    # ★ **몇 점 들어온 것은 '안 들어온 것'으로 친다.**
                    #   `next_distance` 는 표면거리의 **중앙값**으로 다음 거리를
                    #   정하는데, 점이 서너 개뿐이면 그 중앙값은 물체가 아니라
                    #   자기점 잔여·배경 노이즈일 수 있다. 그걸 믿으면 카메라가
                    #   엉뚱한 쪽으로 가고, 게다가 "점이 있다" 는 이유로 아래
                    #   탐침(blind_probe)마저 건너뛴다 — 회복 수단이 사라진다.
                    #   실물에서는 근접한계 안쪽이 0점이 아니라 **노이즈 몇 점**
                    #   으로 나오므로 이 경로가 실재한다(docs/2_preview.md P3).
                    useful = n_pts >= min_pts
                    if useful:
                        acc.append(rot_about_axis(pts, axis_pt, axis_dir, -theta))
                        theta_of.append(float(theta))
                        n_cap += 1
                        frame_pts = pts            # 높이 판정은 이 프레임으로
                    if not useful:
                        # ★ 쓸 만한 반환이 없다 = **방향을 모른다.** 가까이·멀리를
                        #   번갈아 벌려 가며 찾는다. 한쪽만 믿고 가면 "너무 가까워서
                        #   비었는데 더 가까이" 처럼 탐색이 실패 방향으로 달려간다.
                        _why = ("반환 없음" if n_pts == 0
                                else f"{n_pts}점뿐 (<{min_pts}) — 노이즈로 보고 버림")
                        n_blind += 1
                        d_try = blind_probe(d_anchor, n_blind, d_lo, d_hi)
                        if d_try is None:
                            if log:
                                log(f"  d={d*1000:.0f}mm pts={n_pts} — {_why}, "
                                    f"탐색범위({d_lo*1000:.0f}~{d_hi*1000:.0f}mm) 소진")
                            break
                        if _k == _tries - 1:
                            # ★ 예산이 끝났다 — **가지도 않을 탐침을 찍지 않는다.**
                            #   "460mm 탐침" 을 찍어놓고 안 가면, 로그만 보고
                            #   "거기도 봤는데 없더라" 로 잘못 읽는다.
                            if log:
                                log(f"  d={d*1000:.0f}mm pts={n_pts} — {_why}, "
                                    f"시도 {_tries}회 소진 "
                                    f"(다음 후보 {d_try*1000:.0f}mm — 안 감)")
                            break
                        if log:
                            log(f"  d={d*1000:.0f}mm pts={n_pts} — {_why}, "
                                f"{d_try*1000:.0f}mm 탐침 (#{n_blind})")
                        d = d_try
                        continue
                    if cam is None:
                        # 점은 있는데 **이번 프레임만** 카메라 위치를 못 읽었다.
                        # 표면거리를 못 구하니 보정은 건너뛰지만, 계약 문제가
                        # 아니므로 **적응은 유지한다**(다음 높이에서 다시 시도).
                        if log:
                            log(f"  d={d*1000:.0f}mm pts={n_pts} — 카메라 위치 없음, "
                                f"이번 보정만 건너뜀")
                        d_cur = d
                        break
                    # 점이 들어왔다 = 방향을 안다. 여기부터는 창 중앙으로 수렴.
                    d_anchor = d
                    d_next, why = next_distance(d, pts, cam, dof, d_lo, d_hi)
                    if log:
                        log(f"  d={d*1000:.0f}mm pts={n_pts} — {why}")
                    if d_next is None:
                        d_cur = d                      # 다음 높이/방위의 출발점
                        break
                    d = d_next
                else:
                    d_cur = d
            if not adaptive:
                # ── 고정 거리격자 (하위호환 보험) ──────────────────────────
                # ★ **실제로는 도달하지 않는다.** 두 백엔드 모두 `(pts, cam)` 을
                #   돌려주고, `scripts/nbv/verify_preview_contract.py` 가 그것을
                #   AST 로 강제한다 → `legacy=True` 가 될 수 없다. 남겨 두는 이유는
                #   외부/옛 콜백이 붙었을 때 조용히 죽지 않게 하려는 것뿐이다.
                #   즉 이 분기는 **테스트에서만 커버되는 코드**다. 지우려면 위
                #   정적 검사를 CI 에 걸어 계약을 영구 보장한 뒤에 할 것.
                for d in d_steps:
                    got = preview_at(float(tz), float(d)); n_move += 1
                    pts = _unpack_preview(got)[0]
                    # 적응 경로와 **같은 문턱**을 쓴다 — 고정 격자라고 해서 노이즈를
                    # 실루엣으로 받아들일 이유는 없다.
                    if pts is not None and len(pts) >= min_pts:
                        acc.append(rot_about_axis(pts, axis_pt, axis_dir, -theta))
                        theta_of.append(float(theta))
                        n_cap += 1
                        frame_pts = pts
                    elif log and pts is not None and len(pts):
                        log(f"  d={d*1000:.0f}mm pts={len(pts)}점뿐 "
                            f"(<{min_pts}) — 노이즈로 보고 버림")
            # 물체 상단 — **그 프레임의 밀도 기준**(robust_top_height). max 는 잡음 한 점에
            #   끌려 허공을 겨눈다(run_125718: 실제 55mm 인데 142mm 로 보고 167mm 를 찍음).
            top = _frame_top(frame_pts, axis_pt, axis_dir, s, default=-np.inf)
            if not np.isfinite(top):            # 이 높이에서 쓸 만한 프레임이 없었다
                break
            if top - prev_top < top_eps_m:      # 상단이 안 늘면 종료 (GT 불요)
                break
            prev_top = top
            # ★ 물체 끝이 **이미 시야 안에** 보였으면 위를 한 번 더 찍을 이유가 없다.
            #   조준높이에서 상단까지가 시야 반높이(보수적 0.5·d·tan(vfov/2)) 안이면 끝.
            #   (방위마다 되풀이하던 확인 탐침이 여기서 빠진다.)
            half_v = TOP_SEEN_FOV_FRAC * float(d_cur) * sensor.tan_v
            if top - s * tz < half_v:
                if log:
                    log(f"  상단 {(top - s * float(axis_pt[2]))*1000:.0f}mm 가 시야 안"
                        f"(조준 {(s * tz - s * float(axis_pt[2]))*1000:.0f}mm "
                        f"+ 시야 {half_v*1000:.0f}mm) — 높이 오르기 생략")
                break
            tz = s * (top + rise_m)
    move_turntable(0.0)
    pts = np.vstack(acc) if acc else np.zeros((0, 3))
    if log:
        log(f"계획용 preview {len(pts)}pt ({n_cap} 유효 / {n_move} 이동"
            + (", 적응적" if adaptive else ", 고정격자") + ")")
        _log_theta_merge(acc, theta_of, axis_pt, axis_dir, up_sign, log)
    # ★ **방향별 원본**을 남긴다 (역회전 전). 병합본만 있으면 "합쳐진 게 맞나" 를
    #   사후에 못 따진다 — 방향별로 갈라야 부호·정합을 검증할 수 있다.
    collect_planning_points.last_patches = [
        (float(t), np.asarray(P, float).copy()) for t, P in zip(theta_of, acc)]
    return pts


def _frame_top(frame_pts, axis_pt, axis_dir, s: float, default: float) -> float:
    """그 프레임의 **밀도 상단**을 s·z 스칼라로. 점이 적으면 max 로(옛 동작)."""
    if frame_pts is None or len(frame_pts) == 0:
        return default
    H, _ = robust_top_height(frame_pts, axis_pt, axis_dir)
    if H is None:
        return float((s * np.asarray(frame_pts, float)[:, 2]).max())
    return s * float(axis_pt[2]) + float(H)


def _log_theta_merge(acc, theta_of, axis_pt, axis_dir, up_sign, log) -> None:
    """방향별 점군이 **물체 프레임에서 겹치는지** 찍는다.

    왜 필요한가 — 턴테이블을 4방향 돌려 찍고 `rot_about_axis(−θ)` 로 되돌리는데,
    그 역회전의 **부호나 축 방향이 틀리면** 점군이 겹치지 않고 방위별로 흩어진다.
    그래도 점 수·높이·반경은 그럴듯하게 나와서 로그만 봐서는 멀쩡해 보인다
    (2026-09-21: 눕힌 병인데 bbox 가 x·y 양축 ±125mm 로 대칭이라 의심이 갔다).

    겹치면 방위 히스토그램이 서로 **같은 구간**을 채우고, 안 겹치면 방향마다
    다른 사분면을 채운다 — 그 차이를 직접 센다.
    """
    if len(acc) < 2:
        return
    s = float(np.sign(up_sign)) or 1.0
    up = s * np.asarray(axis_dir, float)
    a = np.array([1.0, 0.0, 0.0])
    if abs(float(a @ up)) > 0.9:
        a = np.array([0.0, 1.0, 0.0])
    u = np.cross(a, up); u /= np.linalg.norm(u)
    v = np.cross(up, u)

    # 판별식 = **패치 중심의 방위각.**
    #
    #  카메라는 고정이고 턴테이블이 물체를 θ 만큼 돌린다. 그래서 각 캡처는 그때
    #  카메라를 향한 면 = 물체프레임 방위 −θ 의 면이다. 역회전이 맞으면 네 패치가
    #  물체프레임에서 **90°씩 벌어져** 물체를 둘러싼다. 역회전이 안 먹으면 전부
    #  카메라 쪽 한 방위에 겹쳐 쌓인다 — 물체의 **한쪽만** 본 셈이 된다.
    #
    #  ⚠ 주축(최대분산 방향)으로 보면 안 된다(2026-09-21 시행착오):
    #    ① 180° 회전은 주축을 mod 180 에서 안 바꿔 0°/180° 비교가 무의미하고
    #    ② 각 방향은 **부분 패치**라 길쭉함이 들쭉날쭉해 주축이 불안정하다.
    #    중심 방위각은 패치가 짧아도 안정적이다.
    rows = []
    for P, th in zip(acc, theta_of):
        w = np.asarray(P, float) - np.asarray(axis_pt, float)
        if len(w) < 30:
            continue
        cx, cy = float(np.mean(w @ u)), float(np.mean(w @ v))
        rows.append((th, len(P), np.degrees(math.atan2(cy, cx)) % 360.0,
                     float(np.hypot(cx, cy)) * 1000.0))
    if len(rows) < 2:
        return
    log("방향별 패치 중심 방위 (물체프레임):")
    for th, n, ang, rad in rows:
        log(f"    θ={math.degrees(th):+4.0f}°  {n:6,}pt  중심방위 {ang:5.1f}°  "
            f"중심반경 {rad:4.1f}mm")
    if max(r[3] for r in rows) < 8.0:
        log("    (패치 중심이 모두 축 근처 — 방위가 불분명해 판별 불가)")
        return

    # ★ **원형 통계**로 본다. 방위는 각도라 선형 min/max(ptp)로는 못 잰다 —
    #   359° 와 1° 는 2° 차이인데 선형으로는 358° 로 나온다(2026-09-21 오판).
    #   결과벡터 길이 R: 한쪽에 몰리면 1, 고르게 둘러싸면 0 에 가깝다.
    def _R(angles_deg):
        a = np.radians(np.asarray(angles_deg, float))
        return float(np.hypot(np.mean(np.cos(a)), np.mean(np.sin(a))))

    # ⚠ 이 지표로 **판별되는 것과 안 되는 것**을 분명히 해 둔다 (2026-09-21 검증):
    #   · 판별됨 — 역회전을 **아예 안 한** 경우. 그때는 네 패치가 카메라 쪽 한
    #     방위에 겹쳐 쌓여 R≈1 이 된다.
    #   · 판별 **안 됨** — 역회전 **부호**가 반대인 경우. 보이는 면은 항상
    #     카메라 쪽(월드 방위≈0)이고 역회전하면 −θ 로 가므로, 물체가 어느 쪽으로
    #     돌든 저장 방위는 똑같이 −θ 다. 부호 오류는 방위가 아니라 **패치의
    #     내용**(어느 면이 거기 놓이나)을 바꾼다 → 점군 자체를 봐야 한다.
    R = _R([r[2] for r in rows])
    log(f"    방위 집중도 R={R:.2f}  (0=고르게 둘러쌈 · 1=한쪽에 몰림)")
    if R >= 0.5:
        log("    → ⚠ 네 패치가 한쪽에 몰렸다 — **회전이 반영되지 않았다.** "
            "물체의 한쪽만 본 셈이라 반대쪽은 '면이 없다' 로 읽힌다")
        return
    log("    → 회전이 점군에 반영됐다 (네 방향이 서로 다른 면을 덮는다)")

    # ★ **부호는 '어느 쪽이 더 일관된 강체를 만드나' 로 가른다.**
    #   방위 분포로는 못 가른다(보이는 면은 늘 카메라 쪽이라 저장 방위가 같다).
    #   대신 **패치끼리 겹치는 영역에서 서로 붙는지**를 본다 — 부호가 맞으면 같은
    #   표면이 겹쳐 최근접거리가 작고, 틀리면 다른 면이 포개져 커진다.
    #   현재(−θ)와 반대부호(추가로 +2θ)를 같은 잣대로 재서 더 작은 쪽을 고른다.
    try:
        _log_sign_check(acc, theta_of, axis_pt, axis_dir, log)
    except Exception as e:                                   # noqa: BLE001
        log(f"    (부호 교차검증 생략: {type(e).__name__})")


def _log_sign_check(acc, theta_of, axis_pt, axis_dir, log) -> None:
    """현재 부호 vs 반대 부호 — 어느 쪽이 패치를 더 잘 겹치게 하나.

    지표 = 서로 다른 방향 패치 사이의 **양방향 최근접거리 중앙값**. 같은 물체
    표면이면 작고, 엉뚱한 면끼리 포개지면 크다. 물체가 회전대칭이면 둘이 비슷해
    판별이 안 되므로 그때는 그렇다고 말한다.
    """
    from scipy.spatial import cKDTree
    if len(acc) < 2:
        return

    def _score(P_list):
        ds = []
        for i in range(len(P_list)):
            for j in range(i + 1, len(P_list)):
                A, B = P_list[i], P_list[j]
                if len(A) < 50 or len(B) < 50:
                    continue
                a = np.median(cKDTree(B).query(A)[0])
                b = np.median(cKDTree(A).query(B)[0])
                ds.append(0.5 * (a + b))
        return float(np.median(ds)) * 1000.0 if ds else float("nan")

    cur = list(acc)
    alt = [rot_about_axis(P, axis_pt, axis_dir, 2.0 * t)
           for P, t in zip(acc, theta_of)]
    s_cur, s_alt = _score(cur), _score(alt)
    if not np.isfinite(s_cur) or not np.isfinite(s_alt):
        return
    log(f"    부호 교차검증 — 패치 간 최근접거리 중앙값: "
        f"현재(−θ) {s_cur:.1f}mm · 반대(+θ) {s_alt:.1f}mm")
    if abs(s_cur - s_alt) < 0.15 * max(s_cur, s_alt):
        log("      → 두 부호가 비슷하다 (물체가 회전대칭에 가까워 판별 불가)")
    elif s_cur < s_alt:
        log("      → 현재 부호가 맞다")
    else:
        log("      → ⚠ **반대 부호가 더 잘 맞는다.** collect_planning_points 의 "
            "`rot_about_axis(..., -theta)` 를 `+theta` 로 바꿔야 한다")


def solve_plan_poses(plan, axis_xy, azis_deg, solve_q, is_safe=None, log=None,
                     fill_min: float = None, return_poses: bool = False):
    """계획 자세(el, standoff, target_z)마다 az 를 스윕해 도달·충돌 통과하는 q 선택.

    방위각은 관측 조건을 바꾸지 않고(턴테이블이 회전을 담당) **도달성과 충돌만**
    좌우하므로 여기서 스윕한다. 충돌 게이트를 이 루프에 두는 것이 핵심이다 —
    예전에는 IK 해가 나오면 충돌을 안 보고 확정해서, 충돌 없는 다른 az 를 한 번도
    시도하지 못하고 이동 단계에서 거부돼 밴드가 통째로 날아갔다.

      solve_q(target_xyz, el_deg, az_deg, standoff) -> q or None
      is_safe(q) -> (ok: bool, why: str)            None 이면 충돌검사 생략

    ★ `fill_min` 게이트 — **채점에서 이미 못 쓴다고 나온 자세는 실행하지 않는다.**
      `min_fill_cm2` 는 최악 회전각에서 보이는 물체 면적이고 `FILL_MIN_CM2`(6cm²)는
      SLAM 추적 요구치다. 그런데 이 함수는 IK·충돌만 보고 fill 은 무시했다.
      2026-09-16 실물: 플래너가 `minfill=2cm²`·`0cm²` 자세를 내놨고 그대로 실행돼
      **밴드 4 에서 즉시 tracking lost**(40프레임), 이어서 recovery 3회가 전부
      실패하며 pass 5~8 을 태웠다. 계획 단계에서 이미 알던 정보다.

      단, **전부 걸러지면 최선 하나는 남긴다** — 아무 자세도 없으면 lookaround 이
      통째로 사라지고, 부족한 면은 nbv 가 메우는 것이 설계 의도다.
    """
    # 상수가 이 함수보다 아래에 정의돼 있어 기본인자로 못 쓴다 (def 시점 평가).
    #
    # ★ 배제 문턱은 `FILL_MIN_CM2`(6cm²)가 **아니다.** 그건 주석대로
    #   "tracking-**risk**" 경고선이지 실패선이 아니다. 실측(2026-09-16):
    #     minfill 18 → 220프레임 OK,  6 → 221프레임 OK,  2 → 224프레임 OK,
    #     0 → 40프레임 만에 tracking lost
    #   6cm² 를 배제선으로 쓰면 **멀쩡히 스캔되던 밴드까지 날아가서**, 215mm 짜리
    #   물체가 밴드 1개로 줄었다. 실제로 죽는 건 0cm² 뿐이므로 그 근처만 자른다.
    fill_hard = FILL_HARD_MIN_CM2 if fill_min is None else float(fill_min)
    fill_warn = FILL_MIN_CM2
    usable = [(vp, ev) for vp, ev in zip(plan.poses, plan.evals)
              if float(ev.min_fill_cm2) > fill_hard]
    risky = [ev.min_fill_cm2 for vp, ev in zip(plan.poses, plan.evals)
             if fill_hard < float(ev.min_fill_cm2) < fill_warn]
    if risky and log:
        log(f"  ⚠ fill<{fill_warn:.0f}cm² 인 밴드 {len(risky)}개는 추적이 불안할 수 "
            f"있다 (minfill={', '.join(f'{v:.1f}' for v in risky)}cm²) — 실행은 한다")
    if not usable and plan.poses:
        best = max(zip(plan.poses, plan.evals),
                   key=lambda pe: float(pe[1].min_fill_cm2))
        usable = [best]
        if log:
            log(f"  ⚠ 모든 밴드가 fill≤{fill_hard:.1f}cm² — 최선 1개만 남긴다 "
                f"(minfill={best[1].min_fill_cm2:.1f}cm²). 부족분은 nbv 몫")
    elif log and len(usable) < len(plan.poses):
        dropped = [f"{ev.min_fill_cm2:.1f}" for vp, ev in
                   zip(plan.poses, plan.evals)
                   if float(ev.min_fill_cm2) <= fill_hard]
        log(f"  fill≤{fill_hard:.1f}cm² 인 밴드 {len(dropped)}개 제외 "
            f"(minfill={', '.join(dropped)}cm²) — 볼 면이 없어 tracking lost 난다")

    qs, vps = [], []
    for vp, ev in usable:
        target = np.array([axis_xy[0], axis_xy[1], vp.target_z], float)
        q = None
        for azd in azis_deg:
            q = solve_q(target, vp.el_deg, azd, vp.standoff)
            if q is not None and is_safe is not None:
                ok, why = is_safe(q)
                if not ok:
                    if log:
                        log(f"  band tz={vp.target_z:.3f} az={azd:.0f}° "
                            f"충돌({why}) — 다음 az 시도")
                    q = None
            if q is not None:
                if log:
                    log(f"  자세 el={vp.el_deg:.0f}° s={vp.standoff:.3f} "
                        f"tz={vp.target_z:.3f} az={azd:.0f}° "
                        f"minfill={ev.min_fill_cm2:.0f}cm² IK ok")
                break
        if q is not None:
            qs.append(q)
            # ★ 채택된 az 를 자세에 새겨 둔다. 캡처 중 거리추종이 **같은 밴드를
            #   같은 방위로** 다시 풀어야 하기 때문이다(`StandoffTracker`).
            vps.append(replace(vp, az_deg=float(azd)))
        elif log:
            # 예전엔 조용히 빠졌다 — 낮은 el 후보를 넣은 뒤로는 "왜 그 자세가 없나" 를
            # 로그에서 바로 알아야 한다(2026-09-23).
            log(f"  ✘ 자세 el={vp.el_deg:.0f}° s={vp.standoff:.3f} "
                f"tz={vp.target_z:.3f} — 모든 방위({len(azis_deg)}개)에서 "
                f"도달 자세 없음(IK·충돌) → 이 자세 제외")
    if return_poses:
        return (qs or None), vps
    return qs or None


# ── 계획 (단일 자세 → 부족하면 겹침 밴드 분할) ────────────────────────────────
#: 플래너가 고르는 elevation 후보 (deg).
#
#  ★ 2026-09-23: **20·25° 를 다시 넣었다.** 30° 이상만 두면 전부 '내려다보는' 자세라
#    눕힌 물체의 **옆면**(법선이 수평)이 구조적으로 안 찍힌다 — run_125718 실측:
#    옆면 법선 점이 lookaround 26%·flip 21% 뿐이고 방위 12구간 중 5~6구간이 300점
#    미만, 디스크 위 0~40mm 는 전체의 5% 였다. 그 상태로는 두 패스가 공유할 면이
#    없어 정합이 성립하지 않는다.
#  ⚠ 예전 주석은 "20° 는 도달 자세가 없다(실측)" 였다. 그 관측이 맞을 수 있으므로
#    **후보로만 둔다** — 도달 못 하면 `solve_plan_poses` 의 IK·충돌 검사가 걸러 내고
#    로그에 남긴다(`도달 자세 없음`). 낮은 el 은 축거리가 오히려 멀어져
#    (d = standoff·cos el) 보호 원기둥에는 덜 걸린다.
DEFAULT_ELS = (20.0, 25.0, 30.0, 40.0, 50.0, 60.0, 70.0)

#: '옆면' 판정 — 법선이 수평에서 ±이 각도 안.
SIDE_NORMAL_DEG = 30.0
#: 옆면을 이 비율 미만으로 덮으면 "옆면이 빈 계획" 으로 본다.
SIDE_COV_MIN = 0.75
#: 그때 밴드가 옆면에서 이만큼 더 덮으면, 전체 면적 이득이 모자라도 밴드를 유지한다.
SIDE_GAIN_MIN = 0.04
#: 자세 채점의 **옆면 커버** 항 가중 (전체 커버 항 0.5 와 같은 크기).
#  ★ 왜 필요한가 — run_125718 preview 실측에서 el 15~30° 는 **전체 커버가 32.5% 로 같고**
#    minfill 도 둘 다 fill_target 위라 점수가 **정확히 동점**이었다. 그래서 채점기가
#    추적 여유가 큰 30° 를 골랐는데, 옆면 커버는 15° 64.7% · 20° 59.7% · 30° 46.7% 로
#    크게 갈린다. 동점을 가르는 기준이 '추적 여유' 뿐이면 옆면은 영원히 안 찍힌다.
SIDE_SCORE_W = 0.5

FILL_MIN_CM2 = 6.0            # 최악 프레임 이보다 작으면 tracking-risk (경고선)
#: **배제선** — 이보다 작으면 볼 면이 사실상 없어 tracking lost 가 난다.
#  실측(2026-09-16): minfill 2cm² 는 224프레임 정상, 0cm² 는 40프레임 만에 lost.
#  경고선(6cm²)을 배제에 쓰면 멀쩡한 밴드까지 날아간다 — 215mm 물체가 1밴드가 됐다.
FILL_HARD_MIN_CM2 = 1.0
#: 밴드 자세 선택에서 **추적 여유**를 동점 가르기로 얼마나 볼지 (0=안 봄).
#  주 목적은 커버리지다(`_score` 최대 3.5점). 이 항은 최대 이만큼만 더해져
#  커버가 비슷한 후보 사이에서 추적이 편한 쪽을 고르게 한다. 크게 두면 가파른
#  자세가 이겨 커버가 떨어진다(실측 2026-09-18: 단 나누기 = 사실상 w→∞, 커버
#  58.3%→46.7%).
TRACK_TIEBREAK_W = 0.5
#: 밴드 하나가 **스스로** 품질-가시로 보는 점이 전체의 이 비율에 못 미치면
#  계획에서 뺀다. 밴드 1개 = 턴테이블 전회전 1회(실물 ~30초, sim 240프레임)라,
#  얻는 면이 없으면 시간만 쓰고 이음매 겹침만 망친다.
#    실측 2026-09-18(세제): 죽은 아랫밴드 3개는 각각 148~192점(1.0~1.4%),
#    산 밴드는 2,277~6,324점(16~45%)을 봤다. 사이가 넓어 5% 로 가른다.
#  ⚠ "다른 밴드가 안 보는 면" 으로 재면 안 된다 — 인접 밴드는 **일부러**
#    60% 넘게 겹치게 만들었으므로(SLAM 연속) 그 기준으론 멀쩡한 밴드끼리
#    서로를 죽인다(실측: 8밴드 → 2밴드로 붕괴, 이음매 16%).
#  ⚠ 여기서 빠진 면은 사라지는 게 아니라 **nbv(부분 스윕)·flip 의 몫**이다.
#    lookaround 는 전회전 내내 추적이 붙어야 해서 제약이 가장 세다.
BAND_MIN_SEEN_FRAC = 0.05
ZCOVER_MIN = 0.75             # 단일 자세 z-커버 임계 (미달 → 밴드 분할)
#: 인접 밴드가 겹치는 비율. **밴드 밀도를 정하는 유일한 손잡이**다.
#
#  겹침은 밴드 사이 SLAM 이 이어지는 유일한 근거이면서(relocalization),
#  "FOV/캡처가 모델보다 좁다" 를 흡수하는 곳이기도 하다. 남는 쪽으로 틀리는
#  비용은 회전 1회(실물 ~30s)지만, 부족한 쪽으로 틀리면 tracking lost 다.
#
#  ⚠ 2026-09-17 정리 전에는 같은 일을 하는 계수가 **둘**이었다:
#        step_max = (실측 z폭 × BAND_H_SAFETY 0.60) × (1 − BAND_OVERLAP 0.35)
#                 =  실측 z폭 × 0.39                → 실효 겹침 **61%**
#    코드에는 35% 라고 적혀 있는데 실제로는 61% 로 돌고 있었다. 로그의
#    "공칭overlap" 도 그래서 실제와 달랐다. 둘을 곱해 하나로 합쳤다 —
#    0.61 은 새 값이 아니라 **이미 돌고 있던 값**이다(회귀 없음).
#
#  ★ `standoff.WORK_STANDOFF_M`(더 다가가기)도 같은 증상을 고칠 수 있지만
#    **둘 다 당기면 이중보정**이다. 물리값인 standoff 는 실측 작동거리 창의
#    중앙에 고정해 두고(= 사실), 모자람은 여기서만 흡수한다. 이유:
#      · standoff 는 근접한계(170mm)·충돌이라는 물리 하한이 있다
#      · standoff 를 당기면 band_h 도 같이 줄어(∝거리) 두 효과가 얽힌다
#      · 여기는 스캔 시간만 더 드는, 물리적으로 안전한 손잡이다
#
#  A/B 이력:
#    2026-09-17 (옛 플래너: 전체 r_max 축거리·2방향 preview, 세제 h=202mm)
#       0.61 → 밴드 4개 98,808점 / 0.35 → 밴드 3개, "이득 부족" 으로 단일 자세 44,360점.
#    2026-09-18 (지금 플래너: 밴드별 축거리·4방향 preview·측정 겹침 되먹임, 세제 h=284mm,
#       오프라인 재현 `plan_lookaround_viewpoints`):
#         overlap  밴드  전회전  측정 인접겹침 최소  품질커버
#          0.61     5     6         55%           93.6%
#          0.50     4     5         48%           92.9%
#          0.40     4     4         51%           91.9%   ← 채택
#          0.30     3     3         41%           90.9%
#       0.61 은 두 안전계수를 곱한 합병값이지 측정값이 아니었다. 요구 겹침 25% 에
#       0.40 이 2배 여유고, 모자라면 ④ 되먹임이 밴드를 늘린다. 실물 회전 1회=30s 라
#       6→4 회전은 물체당 약 1분이다. sim 실제 실행 확인은 이 커밋 이후 첫 런에서.
BAND_OVERLAP = float(os.environ.get("MMS_BAND_OVERLAP", "0.40"))

#: 단일 자세로 끝내려면 **물체 높이의 이 비율 이상**을 한 자세가 덮어야 한다.
#  `z_cover_frac` 은 "닿은 z-bin 의 *비율*" 이라 위·아래가 조금씩 걸치고 가운데가
#  비어도 높게 나온다. 실제로 필요한 건 연속된 **z 구간 길이**다.
#  2026-09-16: FOV 를 실측값으로 고치자 z_cover 가 0.75(=임계)로 올라가
#  "단일 자세" 판정이 났는데, 그 자세가 덮는 구간은 149mm/209mm(0.72)였다.
#  높이 기준을 따로 두지 않으면 긴 물체가 한 자세로 처리된다.
ZSPAN_MIN_FRAC = 0.90


#: 밴드 센터를 물체 양끝에서 얼마나 안쪽으로 들일지 — **센터 간격(step) 배수**.
#  반 밴드(band_h/2)씩 물러나면 도달 범위를 그냥 버린다. 왜냐면
#  (a) `h` 는 preview 가 잰 값이라 실제 물체보다 작고
#      — 실측 2026-09-16: 플래너 h=215mm 인데 결과 메시는 252mm 였다 —
#  (b) 끝 밴드는 공칭 반높이를 넘어서까지 캡처한다.
#  그래서 조금만 들인다: 센터 간격 45→58mm, 예상 커버 222→262mm.
#
#  ⚠ 기준이 `band_h` 가 아니라 `step` 인 이유 — 겹침(`BAND_OVERLAP`)을 올리면
#    band_h 는 그대로인데 센터가 촘촘해진다. 들이는 양이 band_h 에 묶여 있으면
#    겹침을 올릴수록 양끝을 **더 많이 버리는** 엉뚱한 결합이 생긴다.
#    0.385 는 합치기 전 실효값(0.25×band_h = 0.385×step)을 그대로 옮긴 것이다.
CENTER_INSET_FRAC = 0.385

#: 캡처 순서상 **인접한 두 자세**가 공유해야 할 최소 seen 겹침 (측정값).
#  밴드 전체가 한 IScan 으로 이어진 뒤로는(2026-09-16) 로봇이 다음 밴드로
#  옮겨가는 동안 SLAM 이 붙어 있어야 한다. 그 조건은 공칭 기하 overlap
#  (`BAND_OVERLAP`) 이 아니라 **직전 자세가 본 면을 다음 자세도 보는가** 다.
#  둘은 꽤 다르다 — 실측 2026-09-17 세제: 공칭 38% 인데 측정 20%.
#  미달이면 밴드를 늘려 다시 짠다(`BAND_MAX_EXTRA`).
BAND_ADJ_OVERLAP_MIN = 0.25
#: 겹침 미달로 추가할 수 있는 밴드 수 상한. 밴드 1개 = 전회전 1회(실물 30s)라
#  무한정 늘릴 수는 없다. 상한에 걸리면 계획은 내되 경고한다.
BAND_MAX_EXTRA = 3
#: 밴드로 쪼개서 **더 덮는 표면점 비율**이 이만큼도 안 되면 단일 자세로 되돌린다.
#  밴드 1개는 전회전 1회(실물 30s)라 공짜가 아니다. z-span 판정이 미달이라고
#  해서 원인이 늘 '높이' 인 것은 아니다 — 납작하고 넓은 물체는 **윗면이 낮은
#  el 에서 안 보여서** 미달이고, 그건 z 로 쪼개도 해결되지 않는다
#  (`_augment_top_face` 의 몫이다).
#
#  ★ 이득은 z-span 이 아니라 **덮은 점 비율**로 잰다. z-span 은 "위에서 아래까지
#    닿았나" 만 보므로 밴드가 같은 구간을 훨씬 촘촘히 덮어도 0mm 로 나온다
#    (실측 2026-09-17 alarm_clock: Δz-span 0mm 인데 Δ커버 +4.8%p).
#  실측 분포(testset 9종, Δ%p):
#    밴드가 필요 없는 쪽 = mug +0.1 · drug_bottle −2.7 · povidone +0.6 ·
#                          protein_drink +2.6
#    밴드가 버는 쪽     = alarm_clock +4.8 · hand_drill +10.3 · mustard +10.6 ·
#                          spray_can +17.4
#  두 무리 사이(2.6 ~ 4.8)를 가른다. drug_bottle 이 **음수**인 것에 주의 —
#  밴드는 각 밴드 안에서만 자세를 고르므로 더 나빠질 수도 있다.
BAND_GAIN_MIN_COV = 0.035

#: 물체 **윗면**(법선이 위를 향하는 수평면 = 뚜껑) 보강 기준.
#  측면 점이 점수를 지배하므로 스코어러는 낮은 el 을 고르는데, 수평 윗면은
#  입사각 제한(`max_incidence_deg`) 때문에 el 이 낮으면 **품질-가시가 0** 이다.
#  실측 2026-09-16 (r45/h209 + 뚜껑 r18): el=20/30/40 → 뚜껑 커버 0%,
#  el=50/60/70 → 100%. 그런데 플랜은 4밴드 전부 el=20 을 골라 뚜껑이 통째로
#  빠졌다. 이전 실행들이 뚜껑을 잡은 건 우연히 el=60/40 이 뽑혔기 때문이다.
CAP_COVER_MIN = 0.50          # 이 비율 미만이면 내려다보는 자세를 하나 더 넣는다
CAP_EL_MIN_DEG = 50.0         # 윗면 보강 자세의 최소 고도각
CAP_ZONE_M = 0.03             # '상단부' 로 볼 두께
CAP_NORMAL_COS = 0.866
#: 윗면 보강 자세를 고를 때, 최고점수에서 이만큼 안이면 **더 가파른 el** 을 택한다.
#  채점에 쓰는 cap 점은 preview 가 옆(el=30°)에서 본 **테두리**뿐이라, 정작 채워야 할
#  평평한 윗면 **중앙**은 점이 없어 점수에 안 들어간다. 수평면의 입사각은 90°−el 이므로
#  (el=50°→40°, 70°→20°) 가파를수록 실제 품질이 낫고, 실물에서도 el=70 으로 윗면이
#  잘 찍혔다(2026-09-19 사용자 관찰). 2026-09-22 run_184746 근거.
CAP_SCORE_MARGIN = 0.15
#: 상단부 점 중 위쪽 법선 점이 전체의 이 비율 이상이면 '넓은 수평 윗면' — 커버율과 무관하게
#  윗면 보강 자세를 넣는다 (근거는 `_augment_top_face` 주석).
CAP_FORCE_FRAC = 0.05        # 법선이 위와 이루는 각 ≤30° 를 '윗면' 으로 본다


def _augment_top_face(poses, evals, pts_obj, nrm_obj, axis_xy, z, sensor,
                      els, sos, n_theta, up_sign, quiet: bool = False):
    """윗면이 안 덮였으면 내려다보는 자세를 1개 추가한다 (in-place, 추가 여부 반환).

    `up_sign` — 작업 프레임에서 어느 z 방향이 '위'인가 (+1 = +Z 가 위).
      sim 은 world 프레임이라 +1, real 은 천장 마운트 base 라 −1
      (`docs/collision.md` §6.1). 부호를 모르면 어느 끝이 뚜껑인지 알 수 없다.
    """
    s_up = float(np.sign(up_sign)) or 1.0
    n_up = nrm_obj[:, 2] * s_up                 # +1 = 위를 향함
    z_up = z * s_up                             # 클수록 위
    # ★ 꼭대기는 max 가 아니라 **밀도 기준** — 잡음 몇 점이 위에 떠 있으면 뚜껑 자세가
    #   허공을 겨눈다(run_102221: max 119mm vs 실제 80mm).
    _z_bottom = float(np.quantile(z, 0.995 if s_up < 0 else 0.005))   # 디스크 쪽 끝
    _H, _ = robust_top_height(pts_obj, np.r_[axis_xy, _z_bottom], np.array([0.0, 0.0, 1.0]))
    top_up = float(z_up.max()) if _H is None else float(_z_bottom * s_up + _H)
    zone = z_up > top_up - CAP_ZONE_M
    cap = (n_up > CAP_NORMAL_COS) & zone
    # ★ 임계를 **점군 밀도에 비례**시킨다. 예전의 절대 50점은 preview 밀도(7천점)에
    #   비해 임의적이었고, 2026-09-22 run_184746 은 윗면 점이 **49개**여서 한 점 차이로
    #   '윗면 없음' 판정 → 밴드 3개가 전부 el=30° → 뚜껑에 구멍이 남았다.
    #   (상단부 131점의 법선 n_up 은 p50 +0.81 · max +1.00 — 분명히 윗면이 있었다.)
    #   구·원뿔처럼 진짜 평평한 윗면이 없는 물체는 이 문턱이 아니라 **아래 커버율
    #   판정**이 걸러 준다: 옆 밴드가 이미 그 면을 보므로 cap_frac 이 높게 나온다.
    n_cap_min = max(15, int(round(0.002 * len(pts_obj))))
    if int(cap.sum()) < n_cap_min:
        if not quiet:
            print(f"[lookaround] 윗면 보강 판단 — 상단 {CAP_ZONE_M*1000:.0f}mm 의 "
                  f"위쪽 법선 점 {int(cap.sum())} < {n_cap_min} → 윗면이라 할 면이 없다고 봄")
        return False                            # 윗면이라 할 면이 없음 (구·원뿔 등)
    cov = np.zeros(len(pts_obj), dtype=bool)
    for e in evals:
        cov |= e.seen_mask
    cap_frac = float(cov[cap].mean())
    # ★ 윗면이 **넓은 수평면**(위쪽 법선 점이 전체의 5% 이상)이면 커버율과 무관하게 넣는다.
    #   옆 밴드(el≤40°)는 수평면을 입사각 50°+ 로밖에 못 봐 실제 스캔에 구멍이 남는데,
    #   preview 점 기반 커버율은 그걸 못 잰다(run_102221: 라벨면 점 2,182개가 el=30 에서
    #   "70% 커버" 로 나와 보강이 빠졌고 실제 메시엔 라벨면에 구멍). 대가는 전회전 1회.
    _force = int(cap.sum()) >= max(n_cap_min, int(round(CAP_FORCE_FRAC * len(pts_obj))))
    if cap_frac >= CAP_COVER_MIN and not _force:
        return False
    cap_els = [float(e) for e in els if float(e) >= CAP_EL_MIN_DEG]
    if not cap_els:
        if not quiet:
            print(f"[lookaround] ⚠ 윗면 커버 {cap_frac*100:.0f}% 인데 "
                  f"el≥{CAP_EL_MIN_DEG:.0f}° 후보가 없어 보강 못 함")
        return False
    tz_cap = s_up * (top_up - CAP_ZONE_M)
    best = None
    # ★ **높은 el 부터** 본다. 수평 윗면의 법선은 위를 향하므로 입사각 = 90°−el 이다
    #   (el=50° → 40°, el=70° → 20°). 커버 점수가 같으면(성긴 preview 에서는 쉽게
    #   포화된다) 입사각이 나은 쪽이 실제 스캔 품질이 낫다 — 아래 비교가 strict `>`
    #   라 동점이면 먼저 본 **높은 el** 이 남는다. 실물 관찰과도 맞는다(el=70 으로
    #   윗면이 잘 찍혔다, 2026-09-19).
    cands = []
    for el in sorted(cap_els, reverse=True):
        for so in sos:
            pose = make_view_pose(axis_xy, float(tz_cap), el, 0.0, so,
                                  up_sign)
            ev = evaluate_viewpoint(pts_obj, nrm_obj, axis_xy, pose,
                                    sensor, n_theta=n_theta)
            cands.append((float(ev.seen_mask[cap].mean()), float(el), pose, ev))
    if not cands:
        return False
    sc_max = max(c[0] for c in cands)
    near = [c for c in cands if c[0] >= sc_max - CAP_SCORE_MARGIN]
    el_pick = max(c[1] for c in near)             # 동점·근소차면 가파른 쪽
    pick = max((c for c in near if c[1] == el_pick), key=lambda c: c[0])
    best = (pick[2], pick[3], pick[0])
    if best[2] <= cap_frac and not _force:
        return False
    best[0].is_cap = True                       # 밴드가 아님 — 정렬/겹침 판단에서 구분
    poses.append(best[0])
    evals.append(best[1])
    if not quiet:
        print(f"[lookaround] 윗면 보강 자세 추가 — el={best[0].el_deg:.0f}° "
              f"tz={best[0].target_z*1000:.0f}mm: 뚜껑 커버 "
              f"{cap_frac*100:.0f}% → {best[2]*100:.0f}%"
              + (f"  (넓은 수평 윗면: 위쪽 법선 점 {int(cap.sum())}/{len(pts_obj)} → 강제)" if _force else ""))
    return True


def _adjacent_overlap(evals) -> List[float]:
    """캡처 순서상 **인접한** 두 자세의 seen 겹침 (뒤 자세 기준 비율).

    밴드가 한 IScan 으로 이어지므로(2026-09-16), 로봇이 다음 밴드로 옮겨가는
    동안 SLAM 이 끊기지 않으려면 직전 자세가 본 면을 새 자세도 봐야 한다.
    "누적 master 와의 겹침" 이 아니다 — 누적과는 겹치는데 **직전 자세와는 안
    겹치는** 순서가 실제로 나왔다(아래 `_band_capture_order` 주석).
    """
    out = []
    for a, b in zip(evals[:-1], evals[1:]):
        inter = int(np.logical_and(a.seen_mask, b.seen_mask).sum())
        out.append(float(inter / max(int(b.seen_mask.sum()), 1)))
    return out


#: 인접 자세의 고도각이 이만큼 넘게 차이나면 그 이음매는 **밴드를 더 쪼개도
#  안 낫다**. 겹침이 낮은 원인이 z 간격이 아니라 "다른 면을 보고 있다" 이기
#  때문이다 — 뚜껑 보강 자세를 제외하는 것과 같은 이유고, 실측으로도 같은
#  현상이 밴드끼리 났다(2026-09-18 세제: el 30°↔70° 이음매 겹침 0%, 밴드를
#  8→10 개로 늘려도 그대로 0% 였다. 늘어난 2회전은 전부 헛돌았다).
BAND_SEAM_EL_JUMP_DEG = 15.0


def _prune_worthless_bands(poses, evals, n_pts: int):
    """**보는 면이 거의 없는** 밴드를 뺀다 (`BAND_MIN_SEEN_FRAC`).

    밴드 하나는 턴테이블 전회전 1회다. 추적은 되지만(배제선 위) 품질-가시가
    거의 없는 자세 — 가파른 고도각으로 옆면을 스치듯 보는 경우 — 는 시간만
    쓰고, 그 밴드가 다른 el 로 붙으면 이음매 겹침까지 망가뜨린다.

    기준은 그 밴드 **자체**의 품질-가시 점 수다. 다른 밴드와의 중복은 보지
    않는다(겹침은 의도된 것이다). 최소 1개는 남긴다.
    """
    if len(poses) <= 1:
        return poses, evals
    need = max(1, int(round(BAND_MIN_SEEN_FRAC * max(n_pts, 1))))
    keep_p, keep_e = [], []
    for vp, ev in zip(poses, evals):
        n_seen = int(ev.seen_mask.sum())
        if n_seen < need:
            print(f"[lookaround]   밴드 tz={vp.target_z*1000:.0f}mm "
                  f"(el={vp.el_deg:.0f}°) 제외 — 품질-가시 {n_seen}점 < {need}점. "
                  f"전회전 1회를 쓸 값이 없다 (그 면은 nbv·flip 몫)")
            continue
        keep_p.append(vp); keep_e.append(ev)
    if not keep_p:                              # 전부 빠지면 제일 많이 보는 1개
        k = int(np.argmax([int(e.seen_mask.sum()) for e in evals]))
        return [poses[k]], [evals[k]]
    return keep_p, keep_e


def _worst_adjacent(poses, ov):
    """(전체 최소 겹침, **밴드를 더 쪼개면 나아질 이음매만** 본 최소 겹침).

    제외 대상 두 가지 — 둘 다 "원인이 z 간격이 아니다":
      · 뚜껑 보강 자세가 낀 이음매 (el 이 다르다)
      · 인접 밴드의 el 이 `BAND_SEAM_EL_JUMP_DEG` 넘게 벌어진 이음매
    밴드 수를 늘릴지는 두 번째 값으로 판단한다.
    """
    if not ov:
        return 1.0, 1.0
    band_only = [v for v, a, b in zip(ov, poses[:-1], poses[1:])
                 if not (a.is_cap or b.is_cap)
                 and abs(float(a.el_deg) - float(b.el_deg)) <= BAND_SEAM_EL_JUMP_DEG]
    return min(ov), (min(band_only) if band_only else 1.0)


def _union_covered_frac(evals, mask=None) -> float:
    """여러 자세가 **합쳐서** 품질-가시로 덮는 점 비율. `mask` 를 주면 그 부분집합만."""
    if not evals:
        return 0.0
    u = np.zeros_like(evals[0].seen_mask, dtype=bool)
    for e in evals:
        u |= e.seen_mask
    if mask is not None:
        m = np.asarray(mask, bool)
        if not m.any():
            return float("nan")
        return float(u[m].mean())
    return float(u.mean())


def _side_normal_mask(nrm_obj, up_sign: float, deg: float = SIDE_NORMAL_DEG):
    """법선이 수평에서 ±`deg` 안인 점 = **옆면**. (N,) bool.

    왜 따로 보나 — 면적 비율(`covered_frac`)은 윗면이 넓으면 높게 나와서 "옆면이라는
    방향이 통째로 비었다" 를 못 잡는다. run_125718 이 그 경우다: 커버 85.5% 인데
    옆면 방위 12구간 중 5개가 사실상 비어 flip 정합이 성립하지 않았다.
    """
    N = np.asarray(nrm_obj, float)
    if N.ndim != 2 or len(N) == 0:
        return np.zeros(0, bool)
    nz = N[:, 2] / (np.linalg.norm(N, axis=1) + 1e-12)
    return np.abs(nz) < math.sin(math.radians(float(deg)))


def _band_capture_order(poses, evals) -> List[int]:
    """캡처 순서 = **z 단조**. 방향은 안전한 끝(minfill 큰 쪽)에서 시작.

    ★ 예전에는 minfill 내림차순(safe-first)이었다. 그건 **밴드 = 독립 IScan**
      이던 시절의 규칙이다 — 밴드가 끊겨도 각자 정합되니, 위험한 밴드를 뒤로
      미뤄 "그때까지 쌓인 master 를 복구 앵커로 남기는" 것이 이득이었다.
      2026-09-16 에 밴드 전체가 **한 IScan** 이 되면서 전제가 사라졌다: 로봇이
      다음 밴드로 옮겨가는 동안 추적이 이어져야 하고, 그러려면 **캡처 순서상
      인접한 두 자세가 같은 면을 봐야 한다.**

      minfill 순서는 z 를 널뛰게 한다 — 실측 2026-09-17(testset 9종, 실제
      점군): 세제가 tz 148→88→29→207→263mm 로 잡혀 3번째 이음매에서 겹침
      **0%**, 9종 중 4종(mustard·spray_can·detergent·alarm_clock)이 0~12% 였다.
      같은 자세를 z 로만 다시 세우면 같은 4종이 20~57% 가 된다. 실물에서 본
      "band1/band2 가 정합되지 않음 · 메시 2덩어리" 가 이 순서 문제다.

      safe-first 의 취지 중 단조성과 양립하는 부분(추적이 가장 잘 붙는 곳에서
      시작한다)은 **방향 선택**으로 남긴다.
    """
    idx = sorted(range(len(poses)), key=lambda i: float(poses[i].target_z))
    if (float(evals[idx[-1]].min_fill_cm2)
            > float(evals[idx[0]].min_fill_cm2)):
        idx = idx[::-1]
    return idx


def _score(ev: PoseEval, fill_target: float = 12.0, side_mask=None) -> float:
    sc = (2.0 * min(ev.min_fill_cm2 / fill_target, 1.0)
          + 1.0 * ev.z_cover_frac + 0.5 * ev.covered_frac)
    # 옆면(법선 수평) 커버 — 없으면 flip 패스와 공유할 면이 없어 정합이 성립하지 않는다.
    if (side_mask is not None and ev.seen_mask is not None
            and len(side_mask) == len(ev.seen_mask) and bool(np.any(side_mask))):
        sc += SIDE_SCORE_W * float(ev.seen_mask[side_mask].mean())
    return sc


#: **오차 주입용** 축거리 바이어스 (mm). 기본 0 = 꺼짐.
#
#  왜 있나 — 거리추종(`standoff.StandoffTracker`)이 실제로 되돌리는지 확인하려면
#  틀린 거리에서 출발시켜야 한다. sim 은 반경 추정이 정확해서 계획 거리가 늘
#  맞고, 그러면 추종은 "유지" 만 찍어 **루프가 도는지 안 도는지 구분이 안 된다.**
#  실물에서 추종이 이상할 때도 알려진 오차를 넣고 수렴을 보는 것이 가장 빠르다.
#    MMS_STANDOFF_BIAS_MM=60  → 계획보다 60mm 멀리 서서 시작
STANDOFF_BIAS_M = float(os.environ.get("MMS_STANDOFF_BIAS_MM", "0.0")) / 1000.0


def _standoff_candidates(pts_obj, axis_xy, sensor: SensorModel):
    r_max = float(np.max(np.linalg.norm(
        pts_obj[:, :2] - np.asarray(axis_xy, float)[None, :], axis=1)))
    mid = 0.5 * (sensor.dof[0] + sensor.dof[1])
    # near-편향 후보 포함: 지름이 DOF 대역(≈10cm)에 육박하는 큰 물체는
    # 최근접면을 near-clip 쪽에 붙여야 반대편이 far-clip 을 안 넘는다.
    cands = [sensor.dof[0] + 0.02 + r_max, mid + r_max, mid + r_max + 0.03]
    if STANDOFF_BIAS_M:
        cands = [c + STANDOFF_BIAS_M for c in cands]
    return cands, r_max


def plan_lookaround_viewpoints(pts_obj, nrm_obj, axis_xy, sensor: SensorModel = None,
                           els=DEFAULT_ELS, n_theta: int = 36,
                           fill_target: float = 12.0,
                           fill_min: float = FILL_MIN_CM2,
                           up_sign: float = +1.0) -> LookaroundPlan:
    """maximin 채점으로 단일 최적 자세 선택; z-커버 미달이면 겹침 밴드 분할.
    pts_obj = crop_object_points + voxel_downsample(sensor.voxel_m) 된 점군.
    fill_target/fill_min 은 SLAM 트래킹 요구치에 정렬해 넘길 것 (real 캘리브)."""
    sensor = sensor or SensorModel()
    z = pts_obj[:, 2]
    # 강건 z-범위 — 분위수(0.5%)로는 성긴 잡음 꼬리(preview 의 ~4%)를 못 거른다. 물체가
    # 있는 쪽(위) 끝은 **밀도 기준**(`robust_top_height`, 2%)으로 자른다. run_102221:
    # 80mm 물체에 밴드가 109·119mm 까지 잡혀 윗밴드가 허공을 돌았다.
    z_lo, z_hi = float(np.quantile(z, 0.005)), float(np.quantile(z, 0.995))
    # 기준점 = 물체 **바닥**(디스크 쪽 끝). up_sign<0 이면 위가 −z 라 바닥은 z 큰 쪽.
    _s_up = float(np.sign(up_sign)) or 1.0
    _z_bottom = z_hi if _s_up < 0 else z_lo
    _H, _ = robust_top_height(pts_obj, np.r_[axis_xy, _z_bottom], np.array([0.0, 0.0, 1.0]))
    if _H is not None:
        _z_top = _z_bottom + _s_up * _H
        if _s_up < 0:
            z_lo = max(z_lo, _z_top)
        else:
            z_hi = min(z_hi, _z_top)
        # ★ 꼬리 점은 **계획 점군에서 아예 뺀다** — z 범위만 자르면 seen-span·커버율·
        #   윗면 판정이 여전히 꼬리 점을 세어 서로 안 맞는다(zspan/h 가 1.76 로 나옴).
        _keep = ((z >= _z_top - 0.010) if _s_up < 0 else (z <= _z_top + 0.010))
        if 50 <= int(_keep.sum()) < len(z):
            pts_obj = pts_obj[_keep]
            nrm_obj = nrm_obj[_keep]
            z = pts_obj[:, 2]
    sos, _ = _standoff_candidates(pts_obj, axis_xy, sensor)
    tzs = [float(np.quantile(z, q)) for q in (0.35, 0.5, 0.65)]

    def search(tz_list, restrict=None):
        """(pose, 선택기준 평가, 전체 점군 평가) — 밴드면 셋째가 버림 판정값이다.

        ★ 밴드 탐색에서 두 가지가 **밴드마다** 달라진다:
          ① 축거리 후보를 **그 밴드의 반경**으로 다시 만든다. 예전에는 물체
             전체의 r_max 하나로 모든 밴드를 세웠다. 위로 갈수록 가늘어지는
             물체(세제: 아래 r=97mm → 위 r=64mm)는 윗밴드에서 카메라가
             33mm 멀어져 **작동거리 창 밖**으로 나갔고, 그 밴드는 전체 minfill
             0.0cm² 이 되어 통째로 버려졌다(실측 2026-09-18). 밴드 반경으로
             세우면 같은 밴드가 30.2cm² 가 된다.
          ② 후보를 **버림 기준(전체 점군 minfill)으로 걸러서** 고른다. 아래
             `evaluate_viewpoint_sub` 주석 참조 — 고르는 기준과 버리는 기준이
             달라서 플래너가 스스로 버릴 자세를 고르고 있었다.
        """
        pts_r, nrm_r, sos_r = pts_obj, nrm_obj, sos
        sub = None
        if restrict is not None:                       # 밴드 z-cover 는 밴드 내에서만
            m = (z >= restrict[0]) & (z <= restrict[1])
            if m.sum() >= 30:
                pts_r, nrm_r, sub = pts_obj[m], nrm_obj[m], m
                sos_r, _ = _standoff_candidates(pts_r, axis_xy, sensor)   # ①
        # 배제선은 **바닥**이지 목적함수가 아니다. 통과한 후보 중에서는 여전히
        # "그 밴드를 얼마나 잘 덮나"(_score)로 고르고, 추적 여유는 **동점을
        # 가르는 보조항**으로만 넣는다.
        #   ⚠ 2026-09-18 에 추적 여유로 단(tier)을 나눠 봤더니 **커버리지가
        #     58.3% → 46.7% 로 떨어졌다.** 추적 면적이 큰 자세는 고도각이 가팔라
        #     입사각이 커지고, 그러면 품질-가시(= 실제로 메시에 기여하는 면)는
        #     오히려 준다. 추적은 "끊기지만 않으면 된다" 는 제약이지 많을수록
        #     좋은 양이 아니다.
        best = fallback = None
        _side_m = _side_normal_mask(nrm_obj, up_sign)   # 채점의 옆면 항 (SIDE_SCORE_W)
        for el in els:
            for s in sos_r:
                for tz in tz_list:
                    pose = make_view_pose(axis_xy, tz, el, 0.0, s, up_sign)
                    if sub is None:                    # 단일 자세 탐색
                        ev = evaluate_viewpoint(pts_obj, nrm_obj, axis_xy, pose,
                                                sensor, n_theta=n_theta)
                        ev_all = ev
                    else:
                        ev_all, ev = evaluate_viewpoint_sub(
                            pts_obj, nrm_obj, axis_xy, pose, sensor,
                            sub_mask=sub, n_theta=n_theta)
                    cand = (pose, ev, ev_all)
                    if (fallback is None
                            or ev_all.min_fill_cm2 > fallback[2].min_fill_cm2):
                        fallback = cand      # 전부 배제선 이하일 때 낼 최선 1개
                    if sub is not None and ev_all.min_fill_cm2 <= FILL_HARD_MIN_CM2:
                        continue             # 어차피 버려질 자세는 고르지 않는다
                    sc = (_score(ev, fill_target, side_mask=_side_m)
                          + TRACK_TIEBREAK_W * min(
                              float(ev_all.min_fill_cm2) / max(fill_min, 1e-9), 1.0))
                    if best is None or sc > best[3]:
                        best = cand + (sc,)
        return best[:3] if best is not None else fallback

    pose, ev, _ = search(tzs)
    # 밴드 분할 여부를 가르는 값 — 높이가 아니라 **단일 자세가 덮는 범위**다.
    # standoff 가 물체 반경에 비례해 정해지므로 가는 물체일수록 카메라가 가까워
    # FOV 가 높이를 못 덮는다.
    # ⚠ 옛 주석은 "r=97mm/h=293mm 는 단일" 이라고 적고 있었는데, 그건 세로 FOV 가
    #   22.62° 로 뒤집혀 있던 시절의 결과다. 실측 FOV(28.58°)+zspan 기준에서는
    #   그 물체도 한 자세로 146mm/290mm 밖에 못 덮어 5밴드가 된다. "단일" 판정
    #   자체가 물체의 절반만 스캔하던 증상이었다. r=34mm/h=207mm 는 3밴드 유지.
    h = z_hi - z_lo
    # 자세가 덮는 **연속 z 구간 길이**. z_cover_frac(닿은 bin 비율)과 달리
    # "위아래만 조금 걸치고 가운데가 빈" 경우를 높게 봐주지 않는다.
    # ⚠ 예전엔 `np.ptp(z[seen])` 이었다 — 최대−최소라 바로 그 경우를 못 걸렀다
    #   (`contiguous_z_span` 주석 참조). 이 값은 밴드 판정과 `band_h` 둘 다의
    #   입력이라 과대평가가 곧 밴드 부족으로 이어진다.
    seen_span = contiguous_z_span(z, ev.seen_mask)
    span_frac = seen_span / max(h, 1e-6)
    print(f"[lookaround] z_cover={ev.z_cover_frac:.2f} (기준 {ZCOVER_MIN:.2f}) "
          f"zspan={seen_span*1000:.0f}mm={span_frac:.2f} (기준 {ZSPAN_MIN_FRAC:.2f}) "
          f"standoff={pose.standoff*1000:.0f}mm h={h*1000:.0f}mm "
          f"minfill={ev.min_fill_cm2:.0f}cm²")
    # ★ 두 조건을 **모두** 넘어야 단일 자세로 끝낸다. z_cover 만 보면 209mm 물체가
    #   149mm 만 덮는 자세 하나로 끝나버린다 (2026-09-16 실측).
    # ★ 단일 자세 + 윗면 보강 안을 **먼저** 만든다. 판정이 밴드로 가더라도 이것이
    #   비교 기준이 된다 — 밴드가 실제로 더 덮지 못하면 되돌아온다(아래).
    single_poses, single_evals = [pose], [ev]
    #  ★ 여기서는 **조용히** 만든다 — 아직 채택된 게 아니라 비교 기준일 뿐이다.
    #    안 그러면 밴드로 가는 경우에도 "윗면 보강 추가" 가 두 번 찍혀, 자세가
    #    2개 추가된 것처럼 읽힌다.
    single_cap = _augment_top_face(single_poses, single_evals, pts_obj, nrm_obj,
                                   axis_xy, z, sensor, els, sos, n_theta, up_sign,
                                   quiet=True)
    single_cov = _union_covered_frac(single_evals)

    def _single_plan(note):
        if single_cap:
            print(f"[lookaround] 윗면 보강 자세 포함 — el="
                  f"{single_poses[-1].el_deg:.0f}° "
                  f"tz={single_poses[-1].target_z*1000:.0f}mm")
        return LookaroundPlan(poses=single_poses, evals=single_evals,
                          banded=single_cap,
                          band_overlap_frac=_adjacent_overlap(single_evals),
                          tracking_risk=any(e.min_fill_cm2 < fill_min
                                            for e in single_evals),
                          note=note)

    if ev.z_cover_frac >= ZCOVER_MIN and span_frac >= ZSPAN_MIN_FRAC:
        return _single_plan("single+윗면" if single_cap else "single")

    # ── 밴드 분할: 자세가 실제로 덮은 z-대역 폭으로 밴드 수 산정 ──────────
    # `band_h` 는 **사실 그대로**다 — 한 자세가 실제 덮은 연속 z 폭(`seen_span`).
    # 여기에 안전계수를 곱하지 않는다. 모자람은 `BAND_OVERLAP` 한 곳에서만
    # 흡수한다(이중보정 방지). 2026-09-17 실측: 모델 예측 148mm vs 실제 141·145mm
    # — 모델은 정확하다. 예전의 0.60 계수는 '모델 보정' 이 아니라 겹침이었다.
    band_h = max(seen_span if seen_span > 0 else 0.06, 0.04)
    # ★ 밴드 수는 **센터 간격**과 맞물려야 한다. 센터는 아래 linspace 로
    #   [z_lo+inset, z_hi-inset] 을 m 등분하므로 실제 간격은 step = span/(m-1) 이고,
    #   공칭 겹침은 1 - step/band_h 다. 겹침을 BAND_OVERLAP 이상으로 두려면
    #       step ≤ band_h·(1-ov)  ⟺  m ≥ 1 + span/(band_h·(1-ov))
    #   이므로 **ceil** 이 맞다.
    #   ⚠ 2026-09-16 에 round 로 바꿨던 것은 과교정이었다. 그때 고쳐야 했던 진짜
    #     버그는 식이 `ceil(h/(band_h·(1-ov)))` 로 **센터 span 이 아니라 물체
    #     높이**를 쓴 것(+ inset 이 band_h/2 였던 것)이고, ceil 자체가 아니었다.
    #     round 는 m 을 한 칸 **내려서** step 을 키우므로 겹침이 기준 아래로
    #     떨어진다 — 실측 2026-09-17: 공칭 27%/29%/31% (기준 35%).
    #     겹침은 밴드 사이 SLAM 이 이어지는 유일한 근거라 **부족한 쪽으로 틀리면
    #     안 된다.** 남는 쪽으로 틀리는 비용은 회전 1회다.
    step_max = max(band_h * (1.0 - BAND_OVERLAP), 1e-9)
    inset = step_max * CENTER_INSET_FRAC
    lo_c, hi_c = z_lo + inset, z_hi - inset
    span = max(hi_c - lo_c, 0.0)
    m_min = max(2, 1 + int(math.ceil(span / step_max - 1e-9)))

    def _build(m):
        """센터 m 개로 밴드 자세를 풀고, 캡처 순서(z 단조)까지 확정해 돌려준다."""
        ps, es = [], []
        for c in np.linspace(lo_c, hi_c, m):
            # search 가 (자세, 밴드 안 평가, 전체 점군 평가) 를 같이 낸다.
            # 겹침/coverage/버림 판정은 **전체 점군 기준**이므로 셋째를 쓴다
            # (예전에는 여기서 전체 점군 평가를 한 번 더 돌렸다 — 같은 값이고
            #  전회전 시뮬이 밴드마다 한 번씩 더 도는 비용이었다).
            pose_b, _ev_band, ev_all = search(
                [float(c)], restrict=(c - band_h / 2, c + band_h / 2))
            ps.append(pose_b)
            es.append(ev_all)
        # ★ 전회전 1회를 쓸 값어치가 없는 밴드는 **여기서** 뺀다. 정렬·겹침·
        #   밴드 수 재시도가 전부 이 목록 위에서 돌기 때문에, 나중에 빼면
        #   "죽은 밴드와의 이음매" 를 메우려고 밴드를 계속 늘리게 된다.
        ps, es = _prune_worthless_bands(ps, es, len(pts_obj))
        # ★ 윗면(뚜껑) 보강은 **정렬 전**에 넣는다 — 정렬과 겹침 계산에 같이 들어가야
        #   뚜껑 자세로 건너뛰는 구간의 겹침도 보인다.
        n_band = len(ps)
        # 재시도 루프 안이라 조용히 — 채택된 계획의 자세 수는 note 로 나간다.
        _augment_top_face(ps, es, pts_obj, nrm_obj, axis_xy, z,
                          sensor, els, sos, n_theta, up_sign, quiet=True)
        idx = _band_capture_order(ps, es)
        return [ps[i] for i in idx], [es[i] for i in idx], len(ps) - n_band

    # ── 겹침을 **재서** 밴드 수를 정한다 ────────────────────────────────
    #  공칭 기하 겹침(위 식)은 "밴드 창이 얼마나 포개지나" 일 뿐, 실제로 두
    #  자세가 **같은 면을 보는지**는 아니다 — 곡률·입사각·가림 때문에 늘 더
    #  작게 나온다(실측 2026-09-17 세제: 공칭 38% → 측정 20%). 그래서 공칭식을
    #  시작점으로만 쓰고, 측정 겹침이 기준에 닿을 때까지 밴드를 늘린다.
    #  BAND_OVERLAP 을 일괄로 키우는 것보다 이쪽이 낫다 — 물체마다
    #  필요한 만큼만 늘어나고, 왜 늘었는지가 로그에 숫자로 남는다.
    poses = evals = None
    n_cap = 0
    m_bands = m_min
    for m_try in range(m_min, m_min + BAND_MAX_EXTRA + 1):
        poses, evals, n_cap = _build(m_try)
        m_bands = m_try
        ov = _adjacent_overlap(evals)
        worst, worst_band = _worst_adjacent(poses, ov)
        _step = span / max(m_try - 1, 1)
        print(f"[lookaround] 밴드 {m_try}개  band_h={band_h*1000:.0f}mm "
              f"센터간격={_step*1000:.0f}mm "
              f"공칭overlap={(1.0 - _step / band_h)*100:.0f}% "
              f"측정 인접겹침 최소={worst*100:.0f}% "
              f"(기준 {BAND_ADJ_OVERLAP_MIN*100:.0f}%) "
              f"센터span={span*1000:.0f}mm (물체 h={h*1000:.0f}mm)")
        # 뚜껑 자세와의 겹침이 부족한 것은 **밴드를 더 쪼개도 안 낫는다**
        #  (el 이 달라서 생긴 차이다) → 밴드끼리의 겹침만 보고 늘릴지 정한다.
        if worst_band >= BAND_ADJ_OVERLAP_MIN:
            break
        if m_try < m_min + BAND_MAX_EXTRA:
            print(f"[lookaround]   ↳ 밴드 간 겹침 {worst_band*100:.0f}% < "
                  f"{BAND_ADJ_OVERLAP_MIN*100:.0f}% — 밴드를 1개 늘려 다시 짠다 "
                  f"(한 IScan 안에서 SLAM 이 이어져야 한다)")
        else:
            print(f"[lookaround]   ⚠ 밴드 {m_try}개에서도 겹침 {worst_band*100:.0f}% "
                  f"— 상한({BAND_MAX_EXTRA}개 추가) 도달. 이 이음매에서 "
                  f"tracking lost 가능 (nbv 가 메운다)")

    # ── 밴드가 **실제로 더 덮는가** ─────────────────────────────────────
    #  z-span 미달의 원인이 높이가 아니라 윗면이면(납작·넓은 물체) z 로 쪼개도
    #  커버가 안 늘고 전회전 횟수만 늘어난다. 재서 확인하고 아니면 되돌린다.
    band_cov = _union_covered_frac(evals)
    gain = band_cov - single_cov
    # ★ **옆면 방향 커버리지**도 같이 본다 (2026-09-23). 면적 이득만 보면 윗면이 넓은
    #   물체에서 "이득 부족" 으로 단일 자세가 되는데, 그러면 옆면이 통째로 비어 flip
    #   정합이 성립하지 않는다(run_125718: 커버 85.5% 인데 옆면 방위 12구간 중 5개가 빔).
    side_m = _side_normal_mask(nrm_obj, up_sign)
    side_single = _union_covered_frac(single_evals, side_m)
    side_band = _union_covered_frac(evals, side_m)
    if side_m.any():
        print(f"[lookaround] 옆면(법선 수평±{SIDE_NORMAL_DEG:.0f}°) 커버 — "
              f"밴드 {side_band*100:.1f}% vs 단일+윗면 {side_single*100:.1f}% "
              f"(옆면 점 {int(side_m.sum()):,}/{len(side_m):,})")
    if gain < BAND_GAIN_MIN_COV:
        _side_rescue = (side_m.any()
                        and np.isfinite(side_single) and np.isfinite(side_band)
                        and side_single < SIDE_COV_MIN
                        and (side_band - side_single) >= SIDE_GAIN_MIN)
        if _side_rescue:
            print(f"[lookaround] 면적 이득은 {gain*100:+.1f}%p 로 모자라지만 "
                  f"**옆면**이 단일 {side_single*100:.1f}% < {SIDE_COV_MIN*100:.0f}% 이고 "
                  f"밴드가 {(side_band - side_single)*100:+.1f}%p 더 덮는다 → 밴드 유지 "
                  f"(옆면이 비면 flip 정합이 성립하지 않는다)")
        else:
            print(f"[lookaround] 밴드 {m_bands}개 커버 {band_cov*100:.1f}% vs "
                  f"단일+윗면 {single_cov*100:.1f}% — 이득 {gain*100:+.1f}%p "
                  f"(<{BAND_GAIN_MIN_COV*100:.1f}%p) → 단일 자세로 되돌린다 "
                  f"(높이가 아니라 윗면이 문제였다)")
            return _single_plan("single+윗면(밴드 무익)" if single_cap
                                else "single(밴드 무익)")

    ov = _adjacent_overlap(evals)
    risk = any(e.min_fill_cm2 < fill_min for e in evals)
    n_real = sum(1 for vp in poses if not vp.is_cap)     # 제외분 반영
    unk = max((float(e.unknown_frac) for e in evals), default=0.0)
    if unk > 0.0:
        print(f"[lookaround] ⚠ preview 근거가 없는 방위 최대 {unk*100:.0f}% — 그 각도는 "
              f"채점에서 뺐다(낙관). 크면 preview 가 물체를 다 못 본 것이다")
    return LookaroundPlan(poses=poses, evals=evals, banded=True,
                      band_overlap_frac=ov, tracking_risk=risk,
                      note=(f"{n_real} bands (band_h={band_h*1000:.0f}mm, "
                            f"z-monotonic"
                            + (f", {m_bands - n_real}개 제외" if m_bands != n_real else "")
                            + ")"
                            + (f" + 윗면 {n_cap}" if n_cap else "")
                            + (f" [근거없는 방위 {unk*100:.0f}%]" if unk > 0 else "")))


# ── Recovery 재계획 (같은 채점기 + overlap 항) ────────────────────────────────
def recovery_replan(master_pts, master_nrm, axis_xy, resume_theta: float,
                    sensor: SensorModel = None, els=DEFAULT_ELS,
                    n_theta: int = 36, w_overlap: float = 1.5,
                    min_overlap_cm2: float = 8.0, up_sign: float = +1.0):
    """lost 후 재계획: 재개 각도에서 **이미 스캔된 면(master)** 이 충분히
    보여야 relocalization 이 성립 → overlap 항 + 남은 회전 maximin.
    반환 (pose, overlap_cm2) — overlap 미달 후보는 탈락, 전부 미달이면 None."""
    sensor = sensor or SensorModel()
    axis_xy = np.asarray(axis_xy, float)
    sos, _ = _standoff_candidates(master_pts, axis_xy, sensor)
    z = master_pts[:, 2]
    tzs = [float(np.quantile(z, q)) for q in (0.35, 0.5, 0.65)]
    p_r, R2 = _rot_z(master_pts, axis_xy, resume_theta)
    n_r = master_nrm.copy()
    n_r[:, :2] = master_nrm[:, :2] @ R2.T
    best, best_ov = None, 0.0
    for el in els:
        for s in sos:
            for tz in tzs:
                pose = make_view_pose(axis_xy, tz, el, 0.0, s, up_sign)
                tr, _ = visible_masks(p_r, n_r, pose, sensor)
                ov = sensor.cm2(int(tr.sum()))          # 재개각에서 보이는 기존면
                if ov < min_overlap_cm2:
                    continue                            # relocalization 불가 후보
                ev = evaluate_viewpoint(master_pts, master_nrm, axis_xy, pose,
                                        sensor, n_theta=n_theta)
                sc = (_score(ev, side_mask=_side_normal_mask(master_nrm, up_sign))
                      + w_overlap * min(ov / (2 * min_overlap_cm2), 1.0))
                if best is None or sc > best[2]:
                    best, best_ov = (pose, ev, sc), ov
    if best is None:
        return None, 0.0
    return best[0], best_ov
