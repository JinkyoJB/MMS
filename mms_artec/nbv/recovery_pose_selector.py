# mms_artec/nbv/recovery_pose_selector.py
#
# Tracking-lost recovery 시 스캐너 (= xArm EE) 를 어디로 이동시킬지 결정.
#
# 두 전략을 swap 가능한 인터페이스로 제공:
#   Method A — LocalJitterSelector:
#       현재 camera pose 주변 ±3cm translation, ±8° rotation 으로 N 개 candidate
#       샘플. 각 candidate 에서 master point cloud 를 카메라 frustum 에 raycast
#       (occlusion 무시 — 속도 우선) 해 가시 점 개수 최대인 후보 선택.
#       → exploitation: 마스터와 overlap 보장.
#
#   Method B — CentroidVectorSelector:
#       마스터 mesh 의 centroid 를 base frame 으로 변환. 마지막 실패 카메라 방향의
#       반대편 위치 (centroid 기준) 에서 centroid 를 향하도록 카메라 pose 계산.
#       Stand-off 거리는 Spider v1 working range (200–300mm) 중간값 250mm.
#       → exploration: 보지 못한 면 우선.
#
# Spider v1 spec (artec3d.com/portable-3d-scanners/old/spider):
#   - Working distance 170–350mm (optimal ~200–250mm)
#   - Angular FOV 30° × 21° (H × V)
#   - 3D resolution 0.1mm
#
# Notation: T_AB : A → B (x_B = T_AB @ x_A). CLAUDE.md 준수.

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Protocol

import numpy as np


# ─────────────────────────────────────────────────────────────────────────────
# Spider v1 광학 상수
# ─────────────────────────────────────────────────────────────────────────────

# Half-angles (rad) of the FOV cone, for frustum culling in raycast.
SPIDER_FOV_H_DEG = 30.0
SPIDER_FOV_V_DEG = 21.0
SPIDER_HALF_FOV_H = np.radians(SPIDER_FOV_H_DEG / 2.0)   # 15°
SPIDER_HALF_FOV_V = np.radians(SPIDER_FOV_V_DEG / 2.0)   # 10.5°

# Depth range — points outside [near, far] are not seen by Spider.
# Full working range 170–350mm (not the optimal sub-band) so that recovery
# scoring isn't starved by a too-thin depth shell.
SPIDER_NEAR_MM = 170.0
SPIDER_FAR_MM = 350.0

# Default stand-off for Method B — kept in the optimal ~200–250mm sub-band.
SPIDER_DEFAULT_STANDOFF_MM = 250.0


# ─────────────────────────────────────────────────────────────────────────────
# Helpers
# ─────────────────────────────────────────────────────────────────────────────

def master_points_in_base_frame(
    master_model,
    T_CB_master: np.ndarray,
    max_points: int = 30_000,
    rng: Optional[np.random.Generator] = None,
) -> np.ndarray:
    """
    Master IModel 안 모든 IScan 의 vertices 를 base frame B 로 변환해 반환 (mm).

    Master IScan 의 vertices 는 scan-world W1 frame (= 첫 pass 의 scan world).
    `T_CB_master` 는 첫 pass 시점의 C → B transform (= multipass `_T_CB`).
    SDK 가 W1 ≈ C_at_start_of_first_pass 로 잡는다는 가정하에 W1 ≈ C 로 취급.
    잘못된 가정이어도 raycast scoring 은 상대 비교라 영향 작음.

    Parameters
    ----------
    master_model : artec_base.ModelHandle
        IScans 누적된 master. scan_count() 가 0 이면 빈 배열 반환.
    T_CB_master : (4,4) np.ndarray
        첫 pass 시점의 C → B transform (= inv(T_BC)). translation 단위 m.
    max_points : int
        성능 위해 무작위 subsample 상한. 30k 면 raycast 빠름.
    rng : np.random.Generator or None
        재현 가능한 subsample 위해.

    Returns
    -------
    pts_B_mm : (M, 3) np.ndarray  —  base frame, mm.
    """
    pts_W = []
    for s_i in range(master_model.scan_count()):
        scan = master_model.get_scan(s_i)
        n_frames = scan.frame_count()
        if n_frames == 0:
            continue
        # frame_transformation 적용 — W frame 좌표 (mm)
        for f_i in range(n_frames):
            v = scan.get_frame(f_i).vertices()
            if v.shape[0] == 0:
                continue
            T = scan.get_frame_transformation(f_i)
            pts_W.append(v @ T[:3, :3].T + T[:3, 3])

    if not pts_W:
        return np.zeros((0, 3), dtype=np.float64)

    pts_W = np.vstack(pts_W).astype(np.float64)   # (N,3) in mm

    # Subsample (memory + raycast 속도)
    if pts_W.shape[0] > max_points:
        if rng is None:
            rng = np.random.default_rng(seed=0)
        idx = rng.choice(pts_W.shape[0], size=max_points, replace=False)
        pts_W = pts_W[idx]

    # W → C → B
    # 가정: SDK 가 첫 frame 의 카메라 frame 을 world 로 사용 → W ≈ C_first
    # 따라서 W frame 의 점 = C frame 좌표 그대로 (mm).
    # C → B 는 T_CB (= inv(T_BC)) 로:  x_B = R_CB @ x_C + t_CB.
    # x_B (mm) = T_CB_master[:3,:3] @ x_C (mm) + T_CB_master[:3,3] * 1000
    R_CB = T_CB_master[:3, :3]
    t_CB_mm = T_CB_master[:3, 3] * 1000.0     # m → mm
    pts_B_mm = pts_W @ R_CB.T + t_CB_mm
    return pts_B_mm


def visible_point_count(
    pts_B_mm: np.ndarray,
    T_CB_candidate: np.ndarray,
    near_mm: float = SPIDER_NEAR_MM,
    far_mm: float = SPIDER_FAR_MM,
    half_fov_h: float = SPIDER_HALF_FOV_H,
    half_fov_v: float = SPIDER_HALF_FOV_V,
) -> int:
    """
    Raycast (no occlusion): base frame 의 점들을 candidate 카메라 frame 으로
    투영, frustum (FOV + depth range) 안에 들어오는 개수 반환.

    카메라 convention: +Z 가 시선 방향 (Artec/OpenCV/Open3D 등 표준).
    point (x, y, z) in C frame is visible if:
        near <= z <= far
        atan(|x| / z) <= half_fov_h
        atan(|y| / z) <= half_fov_v
    빠르게: |x| <= z * tan(half_fov_h),  |y| <= z * tan(half_fov_v)

    Parameters
    ----------
    pts_B_mm : (M, 3)
    T_CB_candidate : (4,4)
        candidate 카메라 의 C → B transform (camera pose in base, translation m).
        x_B (m) = T_CB[:3,:3] @ x_C (m) + T_CB[:3,3]
        →  x_C (mm) = R_BC @ (x_B (mm) - t_CB_mm)  where R_BC = T_CB[:3,:3].T,
           t_CB_mm = T_CB[:3,3] * 1000.
    """
    if pts_B_mm.shape[0] == 0:
        return 0
    R_BC = T_CB_candidate[:3, :3].T          # B → C (rotation only)
    t_CB_mm = T_CB_candidate[:3, 3] * 1000.0  # m → mm
    pts_C = (pts_B_mm - t_CB_mm) @ R_BC.T

    z = pts_C[:, 2]
    in_depth = (z >= near_mm) & (z <= far_mm)
    # frustum: |x|/z <= tan(h), |y|/z <= tan(v). z>0 보장됐으므로 곱셈으로.
    tan_h = np.tan(half_fov_h)
    tan_v = np.tan(half_fov_v)
    in_h = np.abs(pts_C[:, 0]) <= z * tan_h
    in_v = np.abs(pts_C[:, 1]) <= z * tan_v
    return int(np.sum(in_depth & in_h & in_v))


def _rot_xyz(rx: float, ry: float, rz: float) -> np.ndarray:
    """Rotation matrix from intrinsic XYZ Euler angles (rad)."""
    cx, sx = np.cos(rx), np.sin(rx)
    cy, sy = np.cos(ry), np.sin(ry)
    cz, sz = np.cos(rz), np.sin(rz)
    Rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
    Ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    Rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
    return Rx @ Ry @ Rz


def look_at(eye_B: np.ndarray, target_B: np.ndarray,
            up_hint: np.ndarray = np.array([0.0, 0.0, 1.0])) -> np.ndarray:
    """
    Camera C → B transform such that camera at `eye_B` looks at `target_B`.
    +Z_C is forward (toward target), +Y_C is down-ish (consistent with image y).
    up_hint default = world Z (up).

    Returns T_CB (4,4), translation in m.
    """
    eye = np.asarray(eye_B, dtype=float).reshape(3)
    tgt = np.asarray(target_B, dtype=float).reshape(3)
    f = tgt - eye
    n = np.linalg.norm(f)
    if n < 1e-9:
        raise ValueError("eye == target — cannot look at self")
    z_axis = f / n                                # forward (+Z_C in B)
    # x_axis = right; pick so that y_axis ≈ -up_hint (image y down).
    up = np.asarray(up_hint, dtype=float).reshape(3)
    if abs(z_axis @ up) > 0.999:
        # z parallel to up — fall back to world X as up
        up = np.array([1.0, 0.0, 0.0])
    x_axis = np.cross(up, z_axis)
    x_axis /= np.linalg.norm(x_axis)
    y_axis = np.cross(z_axis, x_axis)
    T = np.eye(4)
    T[:3, 0] = x_axis
    T[:3, 1] = y_axis
    T[:3, 2] = z_axis
    T[:3, 3] = eye
    return T


def look_at_axes(
    eye_B: np.ndarray,
    target_B: np.ndarray,
    fwd_C: np.ndarray,
    up_C: np.ndarray,
    world_up_B: np.ndarray = np.array([0.0, 0.0, 1.0]),
) -> np.ndarray:
    """
    Generalized look-at that respects the *actual* Artec C-frame optical-axis
    convention instead of hardcoding `+Z_C = forward` (the documented mis-aim
    bug — `look_at` 의 +Z 가정이 `T_EC_artec` C 프레임과 안 맞아 스캐너가
    엉뚱한 면을 봄, 2026-05-19 실측; docs §3.0).

    카메라를 `eye_B` 에 두고, orientation 을
        R_CB @ fwd_C  →  unit(target_B - eye_B)     (광축이 target 을 향함)
        R_CB @ up_C   →  world_up_B (가능한 한, in-plane)
    이 되도록 잡는다. `fwd_C`, `up_C` 는 **C 프레임에서 표현된** 광축/이미지-up
    방향 (단위 무관, `calibrate_camera_axes_from_preview` 가 경험적으로 산출).

    Returns T_CB (4,4), translation in m.
    """
    eye = np.asarray(eye_B, dtype=float).reshape(3)
    tgt = np.asarray(target_B, dtype=float).reshape(3)
    g = tgt - eye
    ng = np.linalg.norm(g)
    if ng < 1e-9:
        raise ValueError("eye == target — cannot look at self")
    b1 = g / ng                                      # forward 가 향할 B 방향

    # ── C 프레임 정규직교 기저 (fwd_C, up_C 로부터) ──────────────────────
    e1 = np.asarray(fwd_C, dtype=float).reshape(3)
    n1 = np.linalg.norm(e1)
    if n1 < 1e-9:
        raise ValueError("fwd_C 영벡터")
    e1 = e1 / n1
    u = np.asarray(up_C, dtype=float).reshape(3)
    e2 = u - (u @ e1) * e1
    if np.linalg.norm(e2) < 1e-9:                    # up_C ∥ fwd_C → 임의 보정
        tmp = np.array([1.0, 0.0, 0.0])
        if abs(tmp @ e1) > 0.999:
            tmp = np.array([0.0, 1.0, 0.0])
        e2 = tmp - (tmp @ e1) * e1
    e2 = e2 / np.linalg.norm(e2)
    e3 = np.cross(e1, e2)

    # ── B 프레임 목표 기저 (b1=광축, b2≈world up) ────────────────────────
    w = np.asarray(world_up_B, dtype=float).reshape(3)
    b2 = w - (w @ b1) * b1
    if np.linalg.norm(b2) < 1e-9:                    # 광축 ∥ world up
        tmp = np.array([1.0, 0.0, 0.0])
        if abs(tmp @ b1) > 0.999:
            tmp = np.array([0.0, 1.0, 0.0])
        b2 = tmp - (tmp @ b1) * b1
    b2 = b2 / np.linalg.norm(b2)
    b3 = np.cross(b1, b2)

    # R_CB @ [e1 e2 e3] = [b1 b2 b3]  →  R_CB = B_basis @ C_basis^T
    C_basis = np.column_stack([e1, e2, e3])
    B_basis = np.column_stack([b1, b2, b3])
    R_CB = B_basis @ C_basis.T

    T = np.eye(4)
    T[:3, :3] = R_CB
    T[:3, 3] = eye
    return T


def calibrate_camera_axes_from_preview(
    verts_C_mm: np.ndarray,
    T_CB_home: np.ndarray,
    object_center_B_m: np.ndarray,
    world_up_B: np.ndarray = np.array([0.0, 0.0, 1.0]),
) -> tuple:
    """
    Artec C-프레임 광축 컨벤션을 home preview 로 **경험적** 산출.
    SDK intrinsic 의 +Z (실측 불일치) 도, turntable_frame.yaml (stale) 도
    의존하지 않음. 가정 = "home 은 사용자가 물체를 보도록 맞춰둔 검증된 자세".

    - fwd_C  (1차, 물리적): Spider 가 반환하는 점군은 광축 중심 frustum 을
      채우므로 `unit(median(verts_C))` ≈ 광축. 조준 품질·물체 위치와 무관해
      견고.
    - fwd_C 교차검증: `R_BC_home @ unit(center_B - cam_pos_home_B)`.
      물체중심이 광축 위에 있다는 (근사) 가정에서의 독립 추정. 두 추정의
      사잇각을 info 로 반환 (크면 경고 권장).
    - up_C : `R_BC_home @ world_up_B` 의 fwd_C ⟂ 성분. 이렇게 잡으면 home
      eye/target 에서 `look_at_axes` 가 home orientation 을 재현 (info 의
      closure rotation error 로 sanity check).

    Parameters
    ----------
    verts_C_mm : (N,3)  home preview 정점, C 프레임 mm.
    T_CB_home  : (4,4)  home 의 C → B (translation m).
    object_center_B_m : (3,) 물체중심, base frame m.

    Returns
    -------
    (fwd_C, up_C, info)
      fwd_C, up_C : (3,) 단위벡터, C 프레임.
      info : dict — crosscheck_deg, closure_rot_deg, n_verts.
    """
    v = np.asarray(verts_C_mm, dtype=np.float64)
    if v.ndim != 2 or v.shape[0] < 1:
        raise ValueError("verts_C_mm 비어있음 — 캘리브 불가")

    R_BC = np.asarray(T_CB_home[:3, :3], float).T          # B → C (rot)
    cam_pos_B = np.asarray(T_CB_home[:3, 3], float)         # B, m
    center_B = np.asarray(object_center_B_m, float).reshape(3)

    # 1차: frustum 축 ≈ 점군 median 방향
    med = np.median(v, axis=0)
    nm = np.linalg.norm(med)
    if nm < 1e-9:
        raise ValueError("preview median ≈ 원점 — 광축 추정 불가")
    fwd_C = med / nm

    # 교차검증: home 시선벡터를 C 로
    g_B = center_B - cam_pos_B
    ng = np.linalg.norm(g_B)
    crosscheck_deg = float("nan")
    if ng > 1e-9:
        fwd_C_chk = R_BC @ (g_B / ng)
        fwd_C_chk /= max(np.linalg.norm(fwd_C_chk), 1e-12)
        crosscheck_deg = float(np.degrees(
            np.arccos(np.clip(fwd_C @ fwd_C_chk, -1.0, 1.0))))

    # up_C = (R_BC @ world_up) 의 fwd_C ⟂ 성분
    u_C = R_BC @ np.asarray(world_up_B, float).reshape(3)
    up_C = u_C - (u_C @ fwd_C) * fwd_C
    if np.linalg.norm(up_C) < 1e-9:
        # world up 이 광축과 평행 — base X 로 대체
        u_C = R_BC @ np.array([1.0, 0.0, 0.0])
        up_C = u_C - (u_C @ fwd_C) * fwd_C
    up_C = up_C / np.linalg.norm(up_C)

    # closure: home eye/target 에서 look_at_axes 가 home R 을 재현하는가
    closure_rot_deg = float("nan")
    if ng > 1e-9:
        try:
            T_chk = look_at_axes(cam_pos_B, center_B, fwd_C, up_C, world_up_B)
            R_rel = T_chk[:3, :3].T @ np.asarray(T_CB_home[:3, :3], float)
            closure_rot_deg = float(np.degrees(np.arccos(
                np.clip((np.trace(R_rel) - 1.0) / 2.0, -1.0, 1.0))))
        except Exception:
            pass

    info = {
        "crosscheck_deg": crosscheck_deg,
        "closure_rot_deg": closure_rot_deg,
        "n_verts": int(v.shape[0]),
    }
    return fwd_C, up_C, info


# ─────────────────────────────────────────────────────────────────────────────
# Protocol
# ─────────────────────────────────────────────────────────────────────────────

@dataclass
class RecoveryPoseDecision:
    """selector 가 반환하는 결정."""
    T_CB_target: np.ndarray             # 4x4, camera C → B (translation m)
    score: float = 0.0                  # selector 별 의미 (높을수록 좋음)
    debug_info: str = ""                # 로그용


class RecoveryPoseSelector(Protocol):
    """Tracking-lost 발생 시 새 camera pose 를 결정하는 인터페이스."""

    name: str

    def select(
        self,
        T_CB_current: np.ndarray,       # 현재 (lost 직전) camera pose, C → B
        master_pts_B_mm: np.ndarray,    # master point cloud (M,3), base frame mm
    ) -> Optional[RecoveryPoseDecision]:
        """후보 없거나 평가 불가면 None — multipass 가 fallback 처리."""
        ...


# ─────────────────────────────────────────────────────────────────────────────
# Method A — Local jitter + raycast scoring
# ─────────────────────────────────────────────────────────────────────────────

@dataclass
class LocalJitterSelector:
    """
    현재 pose 주변에서 무작위 N 개 candidate 샘플 → raycast scoring → top-1.

    exploitation 계열: 마스터와 overlap 보장이 목적.

    Parameters
    ----------
    n_candidates : int
        무작위 샘플 개수. 큰 값은 더 좋은 후보 가능성 ↑ 이지만 raycast 시간 ∝.
    trans_mm : float
        translation jitter 의 균등분포 반경 (각 축, ±). default 30mm.
    rot_deg : float
        rotation jitter 의 균등분포 반경 (roll/pitch/yaw, ±). default 8°.
    include_current : bool
        candidate 에 "현재 pose 그대로" 도 포함할지. True 면 jitter 가 모두
        나쁜 경우 "이동 안 함" 선택지 보존.
    seed : Optional[int]
        재현 가능한 jitter 위해.
    """
    n_candidates: int = 9
    trans_mm: float = 30.0
    rot_deg: float = 8.0
    include_current: bool = True
    seed: Optional[int] = None
    name: str = "local_jitter"

    def select(
        self,
        T_CB_current: np.ndarray,
        master_pts_B_mm: np.ndarray,
    ) -> Optional[RecoveryPoseDecision]:
        if master_pts_B_mm.shape[0] == 0:
            return None

        rng = np.random.default_rng(self.seed)
        trans_m = self.trans_mm / 1000.0     # mm → m (T_CB translation 은 m)
        rot_rad = np.radians(self.rot_deg)

        # candidate 0 = 현재 pose (옵션)
        candidates = []
        if self.include_current:
            candidates.append(T_CB_current.copy())
        for _ in range(self.n_candidates):
            dx = rng.uniform(-trans_m, trans_m)
            dy = rng.uniform(-trans_m, trans_m)
            dz = rng.uniform(-trans_m, trans_m)
            rrx = rng.uniform(-rot_rad, rot_rad)
            rry = rng.uniform(-rot_rad, rot_rad)
            rrz = rng.uniform(-rot_rad, rot_rad)
            dT = np.eye(4)
            dT[:3, :3] = _rot_xyz(rrx, rry, rrz)
            dT[:3, 3] = [dx, dy, dz]
            # jitter 를 카메라 frame 에서 적용 (camera-local) → T_CB_new = T_CB @ dT
            candidates.append(T_CB_current @ dT)

        scores = [
            visible_point_count(master_pts_B_mm, T_cand)
            for T_cand in candidates
        ]
        best_idx = int(np.argmax(scores))
        best_score = scores[best_idx]
        if best_score == 0:
            return None

        # current 가 best 면 같은 자리 — multipass 가 처리 (no-op 이동)
        cand = candidates[best_idx]
        tag = "current" if (self.include_current and best_idx == 0) else f"j{best_idx}"
        return RecoveryPoseDecision(
            T_CB_target=cand,
            score=float(best_score),
            debug_info=f"local_jitter top={tag} visible={best_score} "
                       f"(of {len(candidates)} cand)",
        )


# ─────────────────────────────────────────────────────────────────────────────
# Method B — Centroid-vector, opposite of failed direction
# ─────────────────────────────────────────────────────────────────────────────

@dataclass
class CentroidVectorSelector:
    """
    Master centroid 를 기준으로 마지막 실패 방향의 반대편에 stand_off_mm 떨어진
    위치에서 centroid 를 바라보는 카메라 pose.

    exploration 계열: 보지 못한 면 우선.

    Parameters
    ----------
    stand_off_mm : float
        centroid → 카메라 거리. Spider working range 중간값 250mm.
    up_hint_B : (3,) np.ndarray
        look_at 의 up 힌트 (base frame). default = world Z up.
    fwd_C, up_C : Optional[(3,)] np.ndarray
        둘 다 주어지면 hardcoded +Z 가정의 `look_at` 대신 경험적 캘리브된
        축으로 `look_at_axes` 사용 → memory 의 centroid_vector latent
        bug(광축 mis-aim) 해소. default None = 기존 동작 그대로 (recovery
        기본 거동 불변). multipass 가 캘리브 후 주입.
    """
    stand_off_mm: float = SPIDER_DEFAULT_STANDOFF_MM
    up_hint_B: np.ndarray = None
    fwd_C: Optional[np.ndarray] = None
    up_C: Optional[np.ndarray] = None
    name: str = "centroid_vector"

    def __post_init__(self):
        if self.up_hint_B is None:
            self.up_hint_B = np.array([0.0, 0.0, 1.0])

    def select(
        self,
        T_CB_current: np.ndarray,
        master_pts_B_mm: np.ndarray,
    ) -> Optional[RecoveryPoseDecision]:
        if master_pts_B_mm.shape[0] == 0:
            return None

        centroid_B_mm = master_pts_B_mm.mean(axis=0)     # (3,) mm
        centroid_B_m = centroid_B_mm / 1000.0

        # 현재 카메라 위치 (B frame, m)
        cam_pos_B_m = T_CB_current[:3, 3]
        # 현재 카메라 → centroid 벡터 (= "실패했던 방향")
        v_cam_to_centroid = centroid_B_m - cam_pos_B_m
        d = np.linalg.norm(v_cam_to_centroid)
        if d < 1e-6:
            # 카메라가 centroid 위에 있음 — 위쪽으로 도망
            v_failed = np.array([0.0, 0.0, -1.0])
        else:
            v_failed = v_cam_to_centroid / d
        # 반대편 = centroid 에서 -v_failed 방향
        eye_B_m = centroid_B_m + (-v_failed) * (self.stand_off_mm / 1000.0)

        if self.fwd_C is not None and self.up_C is not None:
            # 경험적 캘리브된 광축 — mis-aim bug 회피
            T_CB_target = look_at_axes(
                eye_B_m, centroid_B_m, self.fwd_C, self.up_C,
                world_up_B=self.up_hint_B)
        else:
            T_CB_target = look_at(eye_B_m, centroid_B_m, up_hint=self.up_hint_B)

        # score — frustum 안 master 점 개수 (참고용, 결정엔 미사용)
        score = visible_point_count(master_pts_B_mm, T_CB_target)
        return RecoveryPoseDecision(
            T_CB_target=T_CB_target,
            score=float(score),
            debug_info=f"centroid_vector centroid_B=({centroid_B_mm[0]:+.1f}, "
                       f"{centroid_B_mm[1]:+.1f}, {centroid_B_mm[2]:+.1f})mm "
                       f"eye→stand_off={self.stand_off_mm:.0f}mm score={score}",
        )
