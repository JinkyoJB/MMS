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
0°/90° 실루엣 + 조준높이 상승이라는 같은 전략을 쓴다. IK/충돌은 backend 몫.
카메라 규약 = USD look-at (광축 -Z, up=world+z) — real 이식 시 광축 규약만
backend 에서 맞춘다.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
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
    max_incidence_deg: float = 50.0        # 품질 기여 입사각 한계 (누적/coverage)
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


@dataclass
class ViewPose:
    """축 기준 시점 파라미터 + world 카메라 자세 (USD 규약)."""
    el_deg: float
    standoff: float                        # look 타깃(축 위 tz)까지 거리 (m)
    target_z: float                        # look 타깃 z (world)
    eye_w: np.ndarray = None               # (3,)
    R_wc: np.ndarray = None                # (3,3) world→cam 회전의 역 (cam축 world표현)


@dataclass
class PoseEval:
    fill_curve_cm2: np.ndarray             # (nθ,) 프레임별 트래킹-가시 면적
    min_fill_cm2: float                    # ★ maximin 지표
    mean_fill_cm2: float
    covered_frac: float                    # 품질-가시로 한번이라도 본 점 비율
    z_cover_frac: float                    # 품질-가시가 닿은 z-bin 비율
    seen_mask: np.ndarray = None           # (N,) bool — 품질-가시 누적 (밴드겹침/recovery 용)


@dataclass
class LookaroundPlan:
    poses: List[ViewPose]
    evals: List[PoseEval]
    banded: bool
    band_overlap_frac: List[float] = field(default_factory=list)   # 인접 밴드 seen 겹침
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
                   standoff: float) -> ViewPose:
    el, az = math.radians(el_deg), math.radians(az_deg)
    target = np.array([axis_xy[0], axis_xy[1], target_z])
    eye = target + standoff * np.array(
        [math.cos(el) * math.cos(az), math.cos(el) * math.sin(az), math.sin(el)])
    return ViewPose(el_deg=el_deg, standoff=standoff, target_z=target_z,
                    eye_w=eye, R_wc=look_at_R(eye, target))


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


def evaluate_viewpoint(pts_obj, nrm_obj, axis_xy, pose: ViewPose,
                       sensor: SensorModel, n_theta: int = 36,
                       z_bins: int = 24) -> PoseEval:
    """전회전 시뮬: 물체를 θ 회전시키며 프레임별 트래킹-가시 면적 곡선 산출.
    ★ min(fill_curve) = 최악 프레임 = 트래킹 생존 지표 (maximin)."""
    axis_xy = np.asarray(axis_xy, float)
    N = len(pts_obj)
    fill = np.zeros(n_theta)
    seen = np.zeros(N, dtype=bool)
    for i, th in enumerate(np.linspace(0.0, 2 * np.pi, n_theta, endpoint=False)):
        p_th, R2 = _rot_z(pts_obj, axis_xy, th)
        n_th = nrm_obj.copy()
        n_th[:, :2] = nrm_obj[:, :2] @ R2.T
        tr, qu = visible_masks(p_th, n_th, pose, sensor)
        fill[i] = sensor.cm2(int(tr.sum()))
        seen |= qu
    # z-cover: 품질-가시가 닿은 z-bin 비율 (물체 z 범위 균등분할)
    z = pts_obj[:, 2]
    edges = np.linspace(z.min(), z.max() + 1e-9, z_bins + 1)
    bin_id = np.clip(np.digitize(z, edges) - 1, 0, z_bins - 1)
    tot = np.bincount(bin_id, minlength=z_bins)
    hit = np.bincount(bin_id[seen], minlength=z_bins)
    nz = tot > 0
    z_cover = float(np.mean((hit[nz] / np.maximum(tot[nz], 1)) > 0.3))
    return PoseEval(fill_curve_cm2=fill, min_fill_cm2=float(fill.min()),
                    mean_fill_cm2=float(fill.mean()),
                    covered_frac=float(seen.mean()), z_cover_frac=z_cover,
                    seen_mask=seen)


# ── 계획용 preview 캡처 전략 (거리스텝, sim/real 공용 로직) ─────────────────
def simulate_planning_captures(pts_obj, nrm_obj, axis_xy, disc_top_z,
                               sensor: SensorModel = None,
                               d_steps=(0.30, 0.38), azs=(0.0, 90.0),
                               el_deg=25.0, max_heights=4):
    """real 계획용 preview 캡처 전략의 sim 모사 (★ 엄격 실기 DOF 그대로).

    제약의 본질 = 스캐너→표면 거리(0.2~0.3m 창). 축에서 거리 d 캡처가 잡는
    표면 반경 = [d-0.30, d-0.20] → **거리 2스텝(0.30/0.38)이면 r 0~18cm 전부
    커버**(어느 스텝에 잡히는지 자체가 r 측정). 키큰 물체는 표면거리를 유지한
    채 조준높이(tz)만 올려 추가 캡처 — 각 preview 는 FK 좌표로 독립이라
    SLAM/relocalization 불필요.

    종료 조건은 real 과 동일하게 GT 없이: "새 캡처가 상단을 더 못 늘리면 stop".
    반환 = 계획용으로 보인 점 마스크 (real 에선 이 점들이 preview 점군)."""
    sensor = sensor or SensorModel()
    axis_xy = np.asarray(axis_xy, float)
    z = pts_obj[:, 2]
    seen = np.zeros(len(pts_obj), dtype=bool)
    for az in azs:
        tz = float(disc_top_z) + 0.05          # 첫 조준: 디스크 위 5cm (prior)
        prev_top = -np.inf
        for _ in range(max_heights):
            for d in d_steps:
                pose = make_view_pose(axis_xy, tz, el_deg, az, d)
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

def rot_about_axis(pts, axis_pt, axis_dir, ang):
    """임의 축(axis_pt, axis_dir) 둘레로 점군을 ang(rad) 회전. sim·real 공용."""
    a = np.asarray(axis_dir, float)
    a = a / (np.linalg.norm(a) + 1e-12)
    c, sn = math.cos(ang), math.sin(ang)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    R = np.eye(3) * c + np.outer(a, a) * (1 - c) + K * sn
    return (np.asarray(pts, float) - np.asarray(axis_pt, float)) @ R.T \
        + np.asarray(axis_pt, float)


def collect_planning_points(preview_at, move_turntable, axis_pt, axis_dir,
                            d_steps=(0.30, 0.38), thetas=(0.0, math.pi / 2),
                            max_heights: int = 4, rise_m: float = 0.03,
                            start_off_m: float = 0.05, top_eps_m: float = 0.01,
                            up_sign: float = +1.0, log=None):
    """계획용 preview 를 모아 물체 프레임(θ=0) 점군으로 돌려준다.

    `simulate_planning_captures` 가 sim 안에서 모사하던 전략을 **실제 장비로**
    수행하는 루프다. 백엔드는 두 콜백만 제공한다.
      preview_at(tz, d) -> (N,3)  조준높이 tz·축거리 d 로 구동 후 캡처하고
                                  물체 점만 크롭해 돌려준다(작업 프레임).
      move_turntable(theta_rad)   턴테이블을 절대각으로 돌리고 완료까지 대기.

    실루엣을 2방향(턴테이블 0°/90°)에서 얻는 이유는 로봇을 크게 돌리지 않고도
    비대칭 물체의 폭을 보기 위해서다. 90° 점군은 -θ 로 역회전해 물체 프레임으로
    통일한다. 조준높이는 "새 캡처가 상단을 더 못 늘리면 종료"로 올리므로 물체
    높이에 대한 사전지식(GT)이 필요없다.

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
    acc = []
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
        tz, prev_top = float(axis_pt[2]) + s * start_off_m, -np.inf
        for _ in range(max_heights):
            for d in d_steps:
                obj = preview_at(float(tz), float(d))
                if obj is not None and len(obj):
                    acc.append(rot_about_axis(obj, axis_pt, axis_dir, -theta))
            # 물체 상단 = up_sign 방향 최댓값 (base 프레임이면 z 최소)
            top = max(((s * a[:, 2]).max() for a in acc), default=s * tz)
            if top - prev_top < top_eps_m:      # 상단이 안 늘면 종료 (GT 불요)
                break
            prev_top = top
            tz = s * (top + rise_m)
    move_turntable(0.0)
    pts = np.vstack(acc) if acc else np.zeros((0, 3))
    if log:
        log(f"계획용 preview {len(pts)}pt ({len(acc)} 캡처)")
    return pts


def solve_plan_poses(plan, axis_xy, azis_deg, solve_q, is_safe=None, log=None,
                     fill_min: float = None):
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
      통째로 사라지고, 부족한 면은 nbv NBV 가 메우는 것이 설계 의도다.
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
                f"(minfill={best[1].min_fill_cm2:.1f}cm²). 부족분은 nbv NBV 몫")
    elif log and len(usable) < len(plan.poses):
        dropped = [f"{ev.min_fill_cm2:.1f}" for vp, ev in
                   zip(plan.poses, plan.evals)
                   if float(ev.min_fill_cm2) <= fill_hard]
        log(f"  fill≤{fill_hard:.1f}cm² 인 밴드 {len(dropped)}개 제외 "
            f"(minfill={', '.join(dropped)}cm²) — 볼 면이 없어 tracking lost 난다")

    qs = []
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
    return qs or None


# ── 계획 (단일 자세 → 부족하면 겹침 밴드 분할) ────────────────────────────────
DEFAULT_ELS = (20.0, 30.0, 40.0, 50.0)
FILL_MIN_CM2 = 6.0            # 최악 프레임 이보다 작으면 tracking-risk (경고선)
#: **배제선** — 이보다 작으면 볼 면이 사실상 없어 tracking lost 가 난다.
#  실측(2026-09-16): minfill 2cm² 는 224프레임 정상, 0cm² 는 40프레임 만에 lost.
#  경고선(6cm²)을 배제에 쓰면 멀쩡한 밴드까지 날아간다 — 215mm 물체가 1밴드가 됐다.
FILL_HARD_MIN_CM2 = 1.0
ZCOVER_MIN = 0.75             # 단일 자세 z-커버 임계 (미달 → 밴드 분할)
BAND_OVERLAP = 0.35           # 인접 밴드 겹침 (relocalization 성립 조건)

#: 단일 자세로 끝내려면 **물체 높이의 이 비율 이상**을 한 자세가 덮어야 한다.
#  `z_cover_frac` 은 "닿은 z-bin 의 *비율*" 이라 위·아래가 조금씩 걸치고 가운데가
#  비어도 높게 나온다. 실제로 필요한 건 연속된 **z 구간 길이**다.
#  2026-09-16: FOV 를 실측값으로 고치자 z_cover 가 0.75(=임계)로 올라가
#  "단일 자세" 판정이 났는데, 그 자세가 덮는 구간은 149mm/209mm(0.72)였다.
#  높이 기준을 따로 두지 않으면 긴 물체가 한 자세로 처리된다.
ZSPAN_MIN_FRAC = 0.90

#: `band_h`(= 한 자세가 덮는 z 폭) 에 곱하는 **안전계수**.
#  기하 모델은 frustum ∩ DOF ∩ 입사각만 보므로 실제 캡처량보다 낙관적이다.
#  실측 2026-09-16: 모델 149mm vs 실물 자세당 ~87mm (밴드 3개·센터간격 34mm 로
#  메시 155mm → 155 - 68 = 87mm). 비율 ≈ 0.58.
#  이 계수는 밴드 수를 늘리고 센터를 물체 양 끝에 더 붙이는 방향으로만 작용한다
#  (커버리지 보장 ↔ 스캔 시간 trade-off). 커버리지를 우선한다.
BAND_H_SAFETY = 0.60

#: 밴드 센터를 물체 양끝에서 얼마나 안쪽으로 들일지 (band_h 배수).
#  0.5 면 센터가 [z_lo+band_h/2, z_hi-band_h/2] 에 갇힌다. 그런데
#  (a) `h` 는 preview 가 잰 값이라 실제 물체보다 작고
#      — 실측 2026-09-16: 플래너 h=215mm 인데 결과 메시는 252mm 였다 —
#  (b) 끝 밴드는 공칭 반높이를 넘어서까지 캡처한다.
#  그래서 반 밴드씩 물러나면 도달 범위를 그냥 버린다. 1/4 로 줄여 센터를
#  양끝에 붙인다: 밴드 수(=스캔 시간)는 그대로인데 센터 간격 45→58mm,
#  실제 overlap 44%→27%, 예상 커버 222→262mm.
CENTER_INSET_FRAC = 0.25

#: 물체 **윗면**(법선이 위를 향하는 수평면 = 뚜껑) 보강 기준.
#  측면 점이 점수를 지배하므로 스코어러는 낮은 el 을 고르는데, 수평 윗면은
#  입사각 제한(`max_incidence_deg`) 때문에 el 이 낮으면 **품질-가시가 0** 이다.
#  실측 2026-09-16 (r45/h209 + 뚜껑 r18): el=20/30/40 → 뚜껑 커버 0%,
#  el=50/60/70 → 100%. 그런데 플랜은 4밴드 전부 el=20 을 골라 뚜껑이 통째로
#  빠졌다. 이전 실행들이 뚜껑을 잡은 건 우연히 el=60/40 이 뽑혔기 때문이다.
CAP_COVER_MIN = 0.50          # 이 비율 미만이면 내려다보는 자세를 하나 더 넣는다
CAP_EL_MIN_DEG = 50.0         # 윗면 보강 자세의 최소 고도각
CAP_ZONE_M = 0.03             # '상단부' 로 볼 두께
CAP_NORMAL_COS = 0.866        # 법선이 위와 이루는 각 ≤30° 를 '윗면' 으로 본다


def _augment_top_face(poses, evals, pts_obj, nrm_obj, axis_xy, z, sensor,
                      els, sos, n_theta, up_sign):
    """윗면이 안 덮였으면 내려다보는 자세를 1개 추가한다 (in-place, 추가 여부 반환).

    `up_sign` — 작업 프레임에서 어느 z 방향이 '위'인가 (+1 = +Z 가 위).
      sim 은 world 프레임이라 +1, real 은 천장 마운트 base 라 −1
      (`docs/collision.md` §6.1). 부호를 모르면 어느 끝이 뚜껑인지 알 수 없다.
    """
    s_up = float(np.sign(up_sign)) or 1.0
    n_up = nrm_obj[:, 2] * s_up                 # +1 = 위를 향함
    z_up = z * s_up                             # 클수록 위
    top_up = float(z_up.max())
    cap = (n_up > CAP_NORMAL_COS) & (z_up > top_up - CAP_ZONE_M)
    if int(cap.sum()) < 50:
        return False                            # 윗면이라 할 면이 없음 (구·원뿔 등)
    cov = np.zeros(len(pts_obj), dtype=bool)
    for e in evals:
        cov |= e.seen_mask
    cap_frac = float(cov[cap].mean())
    if cap_frac >= CAP_COVER_MIN:
        return False
    cap_els = [float(e) for e in els if float(e) >= CAP_EL_MIN_DEG]
    if not cap_els:
        print(f"[lookaround] ⚠ 윗면 커버 {cap_frac*100:.0f}% 인데 "
              f"el≥{CAP_EL_MIN_DEG:.0f}° 후보가 없어 보강 못 함")
        return False
    tz_cap = s_up * (top_up - CAP_ZONE_M)
    best = None
    for el in cap_els:
        for so in sos:
            pose = make_view_pose(axis_xy, float(tz_cap), el, 0.0, so)
            ev = evaluate_viewpoint(pts_obj, nrm_obj, axis_xy, pose,
                                    sensor, n_theta=n_theta)
            sc = float(ev.seen_mask[cap].mean())
            if best is None or sc > best[2]:
                best = (pose, ev, sc)
    if best is None or best[2] <= cap_frac:
        return False
    poses.append(best[0])
    evals.append(best[1])
    print(f"[lookaround] 윗면 보강 자세 추가 — el={best[0].el_deg:.0f}° "
          f"tz={best[0].target_z*1000:.0f}mm: 뚜껑 커버 "
          f"{cap_frac*100:.0f}% → {best[2]*100:.0f}%")
    return True


def _score(ev: PoseEval, fill_target: float = 12.0) -> float:
    return (2.0 * min(ev.min_fill_cm2 / fill_target, 1.0)
            + 1.0 * ev.z_cover_frac + 0.5 * ev.covered_frac)


def _standoff_candidates(pts_obj, axis_xy, sensor: SensorModel):
    r_max = float(np.max(np.linalg.norm(
        pts_obj[:, :2] - np.asarray(axis_xy, float)[None, :], axis=1)))
    mid = 0.5 * (sensor.dof[0] + sensor.dof[1])
    # near-편향 후보 포함: 지름이 DOF 대역(≈10cm)에 육박하는 큰 물체는
    # 최근접면을 near-clip 쪽에 붙여야 반대편이 far-clip 을 안 넘는다.
    return [sensor.dof[0] + 0.02 + r_max, mid + r_max, mid + r_max + 0.03], r_max


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
    # 강건 z-범위 (노이즈/유령점 소수가 밴드 수를 부풀리지 않게 분위수 사용)
    z_lo, z_hi = float(np.quantile(z, 0.005)), float(np.quantile(z, 0.995))
    sos, _ = _standoff_candidates(pts_obj, axis_xy, sensor)
    tzs = [float(np.quantile(z, q)) for q in (0.35, 0.5, 0.65)]

    def search(tz_list, restrict=None):
        best = None
        pts_r, nrm_r = pts_obj, nrm_obj
        if restrict is not None:                       # 밴드 z-cover 는 밴드 내에서만
            m = (z >= restrict[0]) & (z <= restrict[1])
            if m.sum() >= 30:
                pts_r, nrm_r = pts_obj[m], nrm_obj[m]
        for el in els:
            for s in sos:
                for tz in tz_list:
                    pose = make_view_pose(axis_xy, tz, el, 0.0, s)
                    ev = evaluate_viewpoint(pts_r, nrm_r, axis_xy, pose,
                                            sensor, n_theta=n_theta)
                    if best is None or (_score(ev, fill_target)
                                        > _score(best[1], fill_target)):
                        best = (pose, ev)
        return best

    pose, ev = search(tzs)
    # 밴드 분할 여부를 가르는 값 — 높이가 아니라 **단일 자세가 덮는 범위**다.
    # standoff 가 물체 반경에 비례해 정해지므로 가는 물체일수록 카메라가 가까워
    # FOV 가 높이를 못 덮는다.
    # ⚠ 옛 주석은 "r=97mm/h=293mm 는 단일" 이라고 적고 있었는데, 그건 세로 FOV 가
    #   22.62° 로 뒤집혀 있던 시절의 결과다. 실측 FOV(28.58°)+zspan 기준에서는
    #   그 물체도 한 자세로 146mm/290mm 밖에 못 덮어 5밴드가 된다. "단일" 판정
    #   자체가 물체의 절반만 스캔하던 증상이었다. r=34mm/h=207mm 는 3밴드 유지.
    h = z_hi - z_lo
    seen_z = z[ev.seen_mask]
    # 자세가 덮는 **연속 z 구간 길이**. z_cover_frac(닿은 bin 비율)과 달리
    # "위아래만 조금 걸치고 가운데가 빈" 경우를 높게 봐주지 않는다.
    seen_span = float(np.ptp(seen_z)) if len(seen_z) > 30 else 0.0
    span_frac = seen_span / max(h, 1e-6)
    print(f"[lookaround] z_cover={ev.z_cover_frac:.2f} (기준 {ZCOVER_MIN:.2f}) "
          f"zspan={seen_span*1000:.0f}mm={span_frac:.2f} (기준 {ZSPAN_MIN_FRAC:.2f}) "
          f"standoff={pose.standoff*1000:.0f}mm h={h*1000:.0f}mm "
          f"minfill={ev.min_fill_cm2:.0f}cm²")
    # ★ 두 조건을 **모두** 넘어야 단일 자세로 끝낸다. z_cover 만 보면 209mm 물체가
    #   149mm 만 덮는 자세 하나로 끝나버린다 (2026-09-16 실측).
    if ev.z_cover_frac >= ZCOVER_MIN and span_frac >= ZSPAN_MIN_FRAC:
        poses, evals = [pose], [ev]
        added = _augment_top_face(poses, evals, pts_obj, nrm_obj, axis_xy, z,
                                  sensor, els, sos, n_theta, up_sign)
        return LookaroundPlan(poses=poses, evals=evals, banded=added,
                          tracking_risk=any(e.min_fill_cm2 < fill_min
                                            for e in evals),
                          note="single+윗면" if added else "single")

    # ── 밴드 분할: 자세가 실제로 덮은 z-대역 폭으로 밴드 수 산정 ──────────
    # 기하 모델은 실제 캡처량보다 낙관적이므로 안전계수를 곱한다. 작아진 band_h 는
    # 밴드 수를 늘리고 센터를 물체 양 끝에 더 붙인다 = 실제 커버리지가 늘어난다.
    band_h_model = max(seen_span if seen_span > 0 else 0.06, 0.04)
    band_h = max(band_h_model * BAND_H_SAFETY, 0.04)
    # ★ 밴드 수는 **센터 간격**과 맞물려야 한다. 센터는 아래 linspace 로
    #   [z_lo+band_h/2, z_hi-band_h/2] 를 m 등분하므로 실제 간격은
    #       step = (h - band_h) / (m - 1)
    #   인데, 예전 식 `ceil(h / (band_h·(1-ov)))` 은 "step = band_h·(1-ov)" 를
    #   가정해 세운 것이라 m 을 과다 산정했다. m 이 커지면 step 이 더 작아져
    #   밴드가 서로를 덮기만 하고 **아래로 내려가지 않는다**.
    #   실측 2026-09-16: h=209mm·band_h=106mm 에서 의도 step 69mm 인데 m=4 →
    #   step 34mm. 조준점 3개가 68mm 안에 몰려 360° 전회전을 3번 하고도
    #   메시가 155mm 에 그쳤다.
    #   step = band_h·(1-ov) 를 m 에 대해 풀면 m = 1 + span/(band_h·(1-ov)).
    #   ★ ceil 대신 round 를 쓴다. ceil 은 m 을 한 칸 올려 step 을 의도보다
    #     작게 만들어 overlap 만 키운다 (h=215·band_h=80: 의도 52mm → 45mm,
    #     overlap 35% → 44%). round 가 |실제 step − 의도 step| 을 최소화한다.
    inset = band_h * CENTER_INSET_FRAC
    lo_c, hi_c = z_lo + inset, z_hi - inset
    span = max(hi_c - lo_c, 0.0)
    m_bands = max(2, int(round(
        1.0 + span / (band_h * (1.0 - BAND_OVERLAP)))))
    centers = np.linspace(lo_c, hi_c, m_bands)
    _step = span / max(m_bands - 1, 1)
    print(f"[lookaround] 밴드 {m_bands}개  band_h={band_h*1000:.0f}mm "
          f"센터간격={_step*1000:.0f}mm "
          f"overlap={(1.0 - _step / band_h)*100:.0f}% "
          f"센터span={span*1000:.0f}mm (물체 h={h*1000:.0f}mm)")
    poses, evals = [], []
    for c in centers:
        b = search([float(c)], restrict=(c - band_h / 2, c + band_h / 2))
        poses.append(b[0])
        # 겹침/coverage 는 전체 점군 기준으로 재평가
        evals.append(evaluate_viewpoint(pts_obj, nrm_obj, axis_xy, b[0],
                                        sensor, n_theta=n_theta))
    # ★ 윗면(뚜껑) 보강은 **정렬 전**에 넣는다. 그래야 safe-first 정렬과
    #   겹침(ov) 계산에 같이 들어간다.
    n_band_only = len(poses)
    _augment_top_face(poses, evals, pts_obj, nrm_obj, axis_xy, z,
                      sensor, els, sos, n_theta, up_sign)
    n_cap = len(poses) - n_band_only
    # ★ 안전한 밴드부터 스캔 (minfill 내림차순): 위험 밴드를 나중에 돌면
    #   lost 가 나도 이미 master 가 쌓여 있어 replan/relocalization 앵커 존재.
    order = sorted(range(len(evals)),
                   key=lambda i: -evals[i].min_fill_cm2)
    poses = [poses[i] for i in order]
    evals = [evals[i] for i in order]
    # 겹침 = 각 밴드의 seen 이 "그 전까지 누적된 master" 와 겹치는 비율
    #        (relocalization 앵커 가용성을 직접 측정)
    ov, acc = [], evals[0].seen_mask.copy()
    for e in evals[1:]:
        inter = np.logical_and(acc, e.seen_mask).sum()
        ov.append(float(inter / max(e.seen_mask.sum(), 1)))
        acc |= e.seen_mask
    risk = any(e.min_fill_cm2 < fill_min for e in evals)
    return LookaroundPlan(poses=poses, evals=evals, banded=True,
                      band_overlap_frac=ov, tracking_risk=risk,
                      note=(f"{m_bands} bands (band_h={band_h*1000:.0f}mm, "
                            f"safe-first)"
                            + (f" + 윗면 {n_cap}" if n_cap else "")))


# ── Recovery 재계획 (같은 채점기 + overlap 항) ────────────────────────────────
def recovery_replan(master_pts, master_nrm, axis_xy, resume_theta: float,
                    sensor: SensorModel = None, els=DEFAULT_ELS,
                    n_theta: int = 36, w_overlap: float = 1.5,
                    min_overlap_cm2: float = 8.0):
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
                pose = make_view_pose(axis_xy, tz, el, 0.0, s)
                tr, _ = visible_masks(p_r, n_r, pose, sensor)
                ov = sensor.cm2(int(tr.sum()))          # 재개각에서 보이는 기존면
                if ov < min_overlap_cm2:
                    continue                            # relocalization 불가 후보
                ev = evaluate_viewpoint(master_pts, master_nrm, axis_xy, pose,
                                        sensor, n_theta=n_theta)
                sc = _score(ev) + w_overlap * min(ov / (2 * min_overlap_cm2), 1.0)
                if best is None or sc > best[2]:
                    best, best_ov = (pose, ev, sc), ov
    if best is None:
        return None, 0.0
    return best[0], best_ov
