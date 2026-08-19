"""
utils/nbv/phase1_viewpoint.py — Phase 1 viewpoint selection (sim/real 공용 코어).

컨셉 (2026-07-03 논의):
  Phase 1 = 고정 스캐너 + 턴테이블 전회전. 회전하는 물체는 스캐너 입장에서
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

sim/real 공용: 입력은 점군(np) + 캘리브 상수뿐. sim 은 GT mesh 점군,
real 은 home 정적 캡처(0°/90°) 2장을 넣는다. IK/충돌은 backend 몫.
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
    hfov_deg: float = 30.0                 # 수평 FOV
    vfov_deg: float = 22.62                # 수직 FOV (1280×960 → hfov·3/4 tan비)
    dof: tuple = (0.20, 0.30)              # 작동거리 대역 (m, 광축 깊이)
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
class Phase1Plan:
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
                       z_max_above: float = 0.45) -> np.ndarray:
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
    m = ((p[:, 2] > disc_top_z + z_margin)
         & (p[:, 2] < disc_top_z + z_max_above)
         & (r < r_max))
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


# ── 계획 (단일 자세 → 부족하면 겹침 밴드 분할) ────────────────────────────────
DEFAULT_ELS = (20.0, 30.0, 40.0, 50.0)
FILL_MIN_CM2 = 6.0            # 최악 프레임 이보다 작으면 tracking-risk
ZCOVER_MIN = 0.75             # 단일 자세 z-커버 임계 (미달 → 밴드 분할)
BAND_OVERLAP = 0.35           # 인접 밴드 겹침 (relocalization 성립 조건)


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


def plan_phase1_viewpoints(pts_obj, nrm_obj, axis_xy, sensor: SensorModel = None,
                           els=DEFAULT_ELS, n_theta: int = 36,
                           fill_target: float = 12.0,
                           fill_min: float = FILL_MIN_CM2) -> Phase1Plan:
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
    # 밴드 분할 여부를 가르는 값 — 높이가 아니라 **단일 자세의 z-커버율**이다.
    # standoff 가 물체 반경에 비례해 정해지므로 가는 물체일수록 카메라가 가까워
    # FOV 가 높이를 못 덮는다(실측: r=97mm/h=293mm 는 단일, r=34mm/h=207mm 는 3밴드).
    print(f"[phase1] z_cover={ev.z_cover_frac:.2f} (기준 {ZCOVER_MIN:.2f}) "
          f"standoff={pose.standoff*1000:.0f}mm h={(z_hi-z_lo)*1000:.0f}mm "
          f"minfill={ev.min_fill_cm2:.0f}cm²")
    if ev.z_cover_frac >= ZCOVER_MIN:
        return Phase1Plan(poses=[pose], evals=[ev], banded=False,
                          tracking_risk=ev.min_fill_cm2 < fill_min,
                          note="single")

    # ── 밴드 분할: 자세가 실제로 덮은 z-대역 폭으로 밴드 수 산정 ──────────
    seen_z = z[ev.seen_mask]
    band_h = max(float(np.ptp(seen_z)) if len(seen_z) > 30 else 0.06, 0.04)
    h = z_hi - z_lo
    m_bands = max(2, int(math.ceil(h / (band_h * (1.0 - BAND_OVERLAP)))))
    centers = np.linspace(z_lo + band_h / 2, z_hi - band_h / 2, m_bands)
    poses, evals = [], []
    for c in centers:
        b = search([float(c)], restrict=(c - band_h / 2, c + band_h / 2))
        poses.append(b[0])
        # 겹침/coverage 는 전체 점군 기준으로 재평가
        evals.append(evaluate_viewpoint(pts_obj, nrm_obj, axis_xy, b[0],
                                        sensor, n_theta=n_theta))
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
    return Phase1Plan(poses=poses, evals=evals, banded=True,
                      band_overlap_frac=ov, tracking_risk=risk,
                      note=f"{m_bands} bands (band_h={band_h*1000:.0f}mm, safe-first)")


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
