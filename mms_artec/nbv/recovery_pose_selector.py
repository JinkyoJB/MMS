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
#   - Working distance 200–300mm
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
SPIDER_NEAR_MM = 200.0
SPIDER_FAR_MM = 300.0

# Default stand-off for Method B (middle of working range).
SPIDER_DEFAULT_STANDOFF_MM = 250.0


# ─────────────────────────────────────────────────────────────────────────────
# Helpers
# ─────────────────────────────────────────────────────────────────────────────

def master_points_in_base_frame(
    master_model,
    T_BC_master: np.ndarray,
    max_points: int = 30_000,
    rng: Optional[np.random.Generator] = None,
) -> np.ndarray:
    """
    Master IModel 안 모든 IScan 의 vertices 를 base frame B 로 변환해 반환 (mm).

    Master IScan 의 vertices 는 scan-world W1 frame (= 첫 pass 의 scan world).
    `T_BC_master` 는 첫 pass 시점의 B → C transform (= multipass `_T_BC`).
    SDK 가 W1 ≈ C_at_start_of_first_pass 로 잡는다는 가정하에 W1 ≈ C 로 취급.
    잘못된 가정이어도 raycast scoring 은 상대 비교라 영향 작음.

    Parameters
    ----------
    master_model : artec_base.ModelHandle
        IScans 누적된 master. scan_count() 가 0 이면 빈 배열 반환.
    T_BC_master : (4,4) np.ndarray
        첫 pass 시점의 B → C transform. translation 단위 m.
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
    # x_B (mm) = T_BC_master[:3,:3] @ x_C (mm) + T_BC_master[:3,3] * 1000
    R_BC = T_BC_master[:3, :3]
    t_BC_mm = T_BC_master[:3, 3] * 1000.0     # m → mm
    pts_B_mm = pts_W @ R_BC.T + t_BC_mm
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
    """
    stand_off_mm: float = SPIDER_DEFAULT_STANDOFF_MM
    up_hint_B: np.ndarray = None
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
