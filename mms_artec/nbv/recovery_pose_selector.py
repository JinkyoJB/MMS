# mms_artec/nbv/recovery_pose_selector.py
#
# Spider v1 광학 상수 + 광축 캘리브/look-at 헬퍼.
#
# 2026-05-20 rule 변경: 옛 selector 패턴(LocalJitterSelector·CentroidVectorSelector)
# 폐기. tracking-lost recovery 는 `ArtecMultiPassScanSession._attempt_recovery`
# → `_adaptive_prescan_position(recovery=True)` (fresh probe + 축소 elevation
# search) 로 통합 처리. 자세한 흐름은 docs/artec_scanning_pipeline.md §6.
#
# 이 모듈은 그 통합 경로가 쓰는 공용 헬퍼·상수 보관소:
#   - Spider v1 광학 상수 (FOV, working range, default stand-off)
#   - look_at / look_at_axes : C 프레임 광축 컨벤션을 존중하는 look-at
#   - calibrate_camera_axes_from_preview : home preview 로 광축 경험적 캘리브
#   - master_points_in_base_frame / visible_point_count : raycast 유틸
#     (현 selector-less 경로에선 미사용이지만 호환·테스트용으로 보존)
#
# Spider v1 spec (artec3d.com/portable-3d-scanners/old/spider):
#   - Working distance 170–350mm (optimal ~200–250mm)
#   - Angular FOV 30° × 21° (H × V)
#   - 3D resolution 0.1mm
#
# Notation: T_AB : A → B (x_B = T_AB @ x_A). CLAUDE.md 준수.

from __future__ import annotations

from typing import Optional

import numpy as np


# ─────────────────────────────────────────────────────────────────────────────
# Spider v1 광학 상수
# ─────────────────────────────────────────────────────────────────────────────

# Half-angles (rad) of the FOV cone, for frustum culling in raycast.
# ⚠ 2026-09-16 정정 — 가로/세로가 **뒤집혀 있었다** (`utils/nbv/phase1_viewpoint.py`
#   의 SensorModel 과 같은 오류). 실측 K (`config/calibration/artec_intrinsic.yaml`:
#   fx 2519 / fy 2513, **960×1280 세로형**, 재투영 0.484px):
#       가로(960축)  = 2·atan(960/2/2519)  = 21.58°
#       세로(1280축) = 2·atan(1280/2/2513) = 28.58°
#   세로를 21° 로 보면 `_standoff_distance` 의 "물체 높이가 FOV 에 들어오나"
#   판정이 과하게 비관적이 되어 카메라를 far(350mm)까지 물러나게 만들었다.
SPIDER_FOV_H_DEG = 21.58
SPIDER_FOV_V_DEG = 28.58
SPIDER_HALF_FOV_H = np.radians(SPIDER_FOV_H_DEG / 2.0)   # 10.79°
SPIDER_HALF_FOV_V = np.radians(SPIDER_FOV_V_DEG / 2.0)   # 14.29°

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
# 옛 Selector 패턴 (RecoveryPoseDecision / RecoveryPoseSelector Protocol /
# LocalJitterSelector / CentroidVectorSelector) 은 2026-05-20 제거됨.
# tracking-lost recovery 는 ArtecMultiPassScanSession._attempt_recovery 가
# _adaptive_prescan_position(recovery=True) 를 호출하는 통합 경로로 일원화.
# ─────────────────────────────────────────────────────────────────────────────
