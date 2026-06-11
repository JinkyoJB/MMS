"""
turntable_frame.py — 턴테이블 프레임(T_B_F0) 구성 수학 (sensor-agnostic).

rim(또는 구중심) 점들로 3D 원을 피팅 → 중심(원점)·법선(축) → T_B_F0(4×4) 구성/저장.
PhoXi·Artec 등 어느 센서든 base 프레임 3D 점만 주면 동일하게 쓴다.

cv2/open3d 비의존 (numpy + (저장 시) scipy/yaml). UI 피커는 rim_picker.py 참고.
회전축 자동추정(3구 회전)은 turntable_axis.py 참고.
"""

from __future__ import annotations

import datetime
from pathlib import Path
from typing import Tuple

import numpy as np


def fit_circle_3d(pts: np.ndarray) -> Tuple[np.ndarray, np.ndarray, float, float]:
    """
    평면 위 3D 점들에 원을 피팅.

    Parameters
    ----------
    pts : (N, 3) — 단위 임의(meters 권장). N ≥ 3.

    Returns
    -------
    center   : (3,)  원 중심
    normal   : (3,)  평면 법선(단위)
    radius   : float 반지름
    residual : float RMS 잔차
    """
    pts = np.asarray(pts, dtype=float)
    if len(pts) < 3:
        raise ValueError(f"최소 3점 필요 (현재 {len(pts)}점)")

    # 1. SVD 평면 피팅 → 법선
    # full_matrices=False: U 가 (N,3) — 기본 (N,N) 은 대량점에서 메모리 폭발(fit_plane 와 동일)
    centroid = pts.mean(axis=0)
    _, _, Vt = np.linalg.svd(pts - centroid, full_matrices=False)
    normal = Vt[-1]

    # 2. 평면 내 정규직교 기저
    ref = np.array([1.0, 0.0, 0.0]) if abs(normal[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
    e1 = ref - np.dot(ref, normal) * normal
    e1 /= np.linalg.norm(e1)
    e2 = np.cross(normal, e1)

    # 3. 평면 투영(2D)
    delta = pts - centroid
    u = delta @ e1
    v = delta @ e2

    # 4. 2D 대수 원피팅: u²+v² = 2·cx·u + 2·cy·v + d
    A = np.column_stack([2.0 * u, 2.0 * v, np.ones(len(u))])
    b = u**2 + v**2
    x, _, _, _ = np.linalg.lstsq(A, b, rcond=None)
    cx, cy = float(x[0]), float(x[1])
    r2 = float(x[2]) + cx**2 + cy**2
    if r2 <= 0:
        raise RuntimeError("원 피팅 실패: r²≤0 — 점이 직선에 가깝거나 너무 적음")
    radius = float(np.sqrt(r2))
    center = centroid + cx * e1 + cy * e2

    # 5. RMS 잔차
    pts_c = pts - center
    in_plane = pts_c - np.outer(pts_c @ normal, normal)
    residual = float(np.sqrt(np.mean((np.linalg.norm(in_plane, axis=1) - radius) ** 2)))
    return center, normal, radius, residual


def fit_plane(pts: np.ndarray) -> Tuple[np.ndarray, np.ndarray, float]:
    """
    3D 점들에 평면을 피팅 (SVD). 턴테이블 disc 표면 → 표면점·법선.

    Parameters
    ----------
    pts : (N, 3) — meters 권장. N ≥ 3.

    Returns
    -------
    point    : (3,)  평면 위 한 점 (= 점군 centroid)
    normal   : (3,)  평면 법선(단위, +Z 쪽으로 정렬)
    residual : float RMS 평면-점 거리
    """
    pts = np.asarray(pts, dtype=float)
    if len(pts) < 3:
        raise ValueError(f"평면 피팅 최소 3점 필요 (현재 {len(pts)}점)")
    centroid = pts.mean(axis=0)
    # full_matrices=False: U 가 N×3 (기본 N×N 은 대량점에서 메모리 폭발)
    _, _, Vt = np.linalg.svd(pts - centroid, full_matrices=False)
    normal = Vt[-1]
    normal = normal / (np.linalg.norm(normal) + 1e-12)
    if normal[2] < 0:
        normal = -normal
    residual = float(np.sqrt(np.mean(((pts - centroid) @ normal) ** 2)))
    return centroid, normal, residual


def build_T_B_F0(center_B: np.ndarray, nz_B: np.ndarray) -> np.ndarray:
    """
    B→F 변환 T_B_F0 (4×4). x_F = T_B_F0 @ x_B.

    F 프레임: 원점=원중심, z=평면법선(위쪽), x=B의 x축을 평면 투영(θ=0 기준), y=z×x.
    """
    nz = np.asarray(nz_B, float)
    nz = nz / np.linalg.norm(nz)
    if nz[2] < 0:
        nz = -nz
    bx = np.array([1.0, 0.0, 0.0])
    nx = bx - np.dot(bx, nz) * nz
    if np.linalg.norm(nx) < 1e-6:
        by = np.array([0.0, 1.0, 0.0])
        nx = by - np.dot(by, nz) * nz
    nx /= np.linalg.norm(nx)
    ny = np.cross(nz, nx)

    T_F_B = np.eye(4)
    T_F_B[:3, :3] = np.column_stack([nx, ny, nz])
    T_F_B[:3, 3] = np.asarray(center_B, float)
    return np.linalg.inv(T_F_B)


def save_turntable_frame_yaml(path, T_B_F0: np.ndarray, n_points: int,
                              radius_mm: float, residual_mm: float,
                              extra: dict | None = None) -> None:
    """T_B_F0 를 turntable_frame.yaml 로 저장 (translation m, rotation quat)."""
    import yaml
    from scipy.spatial.transform import Rotation as ScipyR

    path = Path(path)
    t = T_B_F0[:3, 3]
    q = ScipyR.from_matrix(T_B_F0[:3, :3]).as_quat()   # [qx,qy,qz,qw]
    data = {
        "date": datetime.date.today().isoformat(),
        "n_points": int(n_points),
        "rim_radius_mm": round(float(radius_mm), 2),
        "rim_residual_mm": round(float(residual_mm), 3),
        "T_B_F0": {
            "translation": [float(v) for v in t],          # meters
            "rotation_quat": [float(v) for v in q],         # [qx,qy,qz,qw]
            "matrix": [[float(v) for v in row] for row in T_B_F0.tolist()],
        },
    }
    if extra:
        data.update(extra)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(yaml.dump(data, default_flow_style=None, allow_unicode=True),
                    encoding="utf-8")
    print(f"[turntable_frame] 저장 완료: {path}")
