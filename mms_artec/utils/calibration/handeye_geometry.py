"""
Hand-eye 캘리브레이션 기하 유틸 — numpy 전용, side-effect 없음 (sim + real 공용).

`utils/robot/xarm7_kinematics.py` 와 동일 정신: scipy/레포 내부 import 없이 자기완결
→ Isaac 확장처럼 `utils` 패키지명이 충돌하는 환경에서도 **파일경로 로드**로 안전히 재사용.

좌표 규약
---------
- pose 행렬 `T_XY` : 프레임 Y 를 X 에 표현 (x_X = T_XY @ x_Y).
- 회전 quat = [w, x, y, z].
- 시스템 hand-eye `T_EC` : E→C, 즉 x_C = T_EC @ x_E (= EE-in-camera). `utils/transforms.compute_T_CB` 규약.
"""

from __future__ import annotations

import math
import numpy as np


# ── SE(3) 기본 ────────────────────────────────────────────────────────────────
def make_T(R, t) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = np.asarray(t, dtype=np.float64).reshape(3)
    return T


def inv_T(T) -> np.ndarray:
    R = T[:3, :3]
    t = T[:3, 3]
    Ti = np.eye(4)
    Ti[:3, :3] = R.T
    Ti[:3, 3] = -R.T @ t
    return Ti


def quat_wxyz_to_R(q) -> np.ndarray:
    w, x, y, z = q
    n = math.sqrt(w * w + x * x + y * y + z * z) + 1e-18
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
    ], dtype=np.float64)


def rot_about_axis(axis, ang_rad) -> np.ndarray:
    a = np.asarray(axis, dtype=np.float64)
    a = a / (np.linalg.norm(a) + 1e-12)
    x, y, z = a
    c, s = math.cos(ang_rad), math.sin(ang_rad)
    C = 1 - c
    return np.array([
        [c + x * x * C,     x * y * C - z * s, x * z * C + y * s],
        [y * x * C + z * s, c + y * y * C,     y * z * C - x * s],
        [z * x * C - y * s, z * y * C + x * s, c + z * z * C],
    ], dtype=np.float64)


def rot_angle_deg(Ra, Rb) -> float:
    """두 회전행렬 사이 각도(deg)."""
    c = (np.trace(Ra.T @ Rb) - 1.0) / 2.0
    return math.degrees(math.acos(max(-1.0, min(1.0, c))))


def R_to_euler_xyz(R) -> np.ndarray:
    """R = Rx(a)Ry(b)Rz(c) 분해 → [a,b,c] (scipy from_euler('xyz') 호환)."""
    sb = float(np.clip(R[0, 2], -1.0, 1.0))
    b = math.asin(sb)
    if abs(abs(sb) - 1.0) < 1e-6:                # gimbal lock
        a = math.atan2(-R[1, 0], R[1, 1])
        c = 0.0
    else:
        a = math.atan2(-R[1, 2], R[2, 2])
        c = math.atan2(-R[0, 1], R[0, 0])
    return np.array([a, b, c], dtype=np.float64)


def mat_to_pose6d_mm(T) -> np.ndarray:
    """동차변환(m) → pose6d [x,y,z mm, roll,pitch,yaw rad] (XArmInterface/kin 규약)."""
    return np.concatenate([T[:3, 3] * 1000.0, R_to_euler_xyz(T[:3, :3])])


# ── 카메라 look-at (USD/OpenGL 규약: 광축 -Z, +Y up) ─────────────────────────
def look_at_camera(eye, center, up_hint, roll_deg=0.0) -> np.ndarray:
    """
    eye 에서 center 를 바라보는 카메라 회전(world 축). USD/OpenGL 규약(광축 = -Z).
    up_hint 가 이미지 up 방향(roll)을 결정. roll_deg 는 광축 둘레 추가 회전.
    """
    eye = np.asarray(eye, dtype=np.float64)
    center = np.asarray(center, dtype=np.float64)
    f = center - eye
    f = f / (np.linalg.norm(f) + 1e-12)
    z = -f                                        # 카메라 +Z = 광축(-Z) 반대
    up = np.asarray(up_hint, dtype=np.float64)
    x = np.cross(up, z)
    if np.linalg.norm(x) < 1e-6:
        up = np.array([0.0, 1.0, 0.0])
        x = np.cross(up, z)
    x = x / (np.linalg.norm(x) + 1e-12)
    y = np.cross(z, x)
    R = np.column_stack([x, y, z])
    if abs(roll_deg) > 1e-9:
        R = R @ rot_about_axis([0.0, 0.0, 1.0], math.radians(roll_deg))
    return R


# ── 캘리브 자세 생성 (타깃 위 반구에서 내려다보기) ───────────────────────────
def generate_hemisphere_poses(
    center, normal, T_EC,
    distance_m, polars_deg, azis_deg, rolls_deg, dist_jitter,
    up_hint=(1.0, 0.0, 0.0),
) -> list:
    """
    안착한 평면 타깃(보드) 위 반구에서 center 를 내려다보는 카메라 자세들을 만들고,
    각 카메라 pose 를 mount `T_EC`(E→C, EE-in-camera)로 **EE(플랜지) world pose** 로 환산.

      T_W_E = T_W_C @ T_EC    (x_W = T_W_C·x_C, x_C = T_EC·x_E → x_W = T_W_C·T_EC·x_E)

    - nadir(polar 0): 수직, roll 만 다양화. tilted(polar>0): azimuth N 방향 둘러봄.
    - polar/azimuth/roll/거리는 호출자가 다양성·도달성을 보며 조정.

    Returns: list[np.ndarray (4,4)]  — EE world pose 들.
    """
    n = np.asarray(normal, dtype=np.float64)
    n = n / (np.linalg.norm(n) + 1e-12)
    ref = np.cross(n, np.array([1.0, 0.0, 0.0]))
    if np.linalg.norm(ref) < 1e-6:
        ref = np.cross(n, np.array([0.0, 1.0, 0.0]))
    ref = ref / np.linalg.norm(ref)
    up = np.asarray(up_hint, dtype=np.float64)
    center = np.asarray(center, dtype=np.float64)

    poses = []
    k = 0
    for pol in polars_deg:
        azis = [0.0] if abs(pol) < 1e-6 else azis_deg
        for az in azis:
            roll = rolls_deg[k % len(rolls_deg)]
            dist = distance_m + dist_jitter[k % len(dist_jitter)]
            k += 1
            axis = rot_about_axis(n, math.radians(az)) @ ref
            d = rot_about_axis(axis, math.radians(pol)) @ n
            eye = center + dist * d
            R_WC = look_at_camera(eye, center, up, roll_deg=roll)
            poses.append(make_T(R_WC, eye) @ T_EC)
    return poses
