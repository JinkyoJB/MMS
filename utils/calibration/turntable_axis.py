"""
turntable_axis.py — 턴테이블 회전축 추정 (T_B_F0 calibration 핵심 수학).

배경
----
하드웨어팀이 턴테이블/로봇을 옮기면 calibration 된 T_B_F0(로봇 base 기준 턴테이블
프레임)가 무효화된다. 이를 **버튼 하나로 다시 잡는** 루틴의 수학부.

방법 (구 fixture 기반)
----------------------
턴테이블에 **반경이 알려진 구(sphere)** 를 off-axis 로 고정하고, 여러 회전각 θ 에서
스캐너로 점군을 얻는다. 각 구의 중심은 θ 에 따라 회전축에 수직인 **원**을 그린다.
  1. 각 프레임에서 구 cap 점군 → `fit_sphere_center`(known-R) 로 중심.
  2. 한 구의 중심 궤적 → `fit_circle`(평면법선 = 축방향, 원중심 = 축 위 한 점).
  3. 여러 구의 (법선·원중심) → `estimate_axis` 로 공통 축 = (point, direction).

★ known-R 가 핵심: 스캐너는 구의 앞면(cap)만 본다. 반경을 자유추정하면 보는 방향에
  따라 중심이 깊이 편향돼 궤적이 망가진다. 반경을 고정하면 cap 만으로 중심이 유일하게
  정해진다(검증: 자유R 76° 오차 → known-R 0.14° 오차).

프레임
------
입력 점군이 어느 프레임이든(world / 로봇 base) 그 프레임에서의 축을 반환한다.
실물에선 카메라 점군을 T_EC·FK 로 **로봇 base(B)** 로 옮겨 넣으면 결과가 곧 T_B_F0 축.
"""

from __future__ import annotations

from typing import List, Sequence, Tuple

import numpy as np


def _norm(v: np.ndarray) -> np.ndarray:
    return v / (np.linalg.norm(v) + 1e-12)


def fit_sphere_center(points: np.ndarray, radius: float,
                      iters: int = 50, tol: float = 1e-7) -> np.ndarray:
    """
    반경이 알려진 구의 중심을 부분 점군(cap)에서 추정 (Gauss-Newton).

    residual r_i = |p_i - c| - R 를 최소화. cap 만 봐도 R 을 고정하면 중심이 유일.

    Parameters
    ----------
    points : (N,3) 구 표면 점들
    radius : 구 반경 (m)

    Returns
    -------
    center : (3,)
    """
    P = np.asarray(points, dtype=float)
    c = P.mean(axis=0)
    for _ in range(iters):
        d = P - c
        dist = np.linalg.norm(d, axis=1) + 1e-12
        u = d / dist[:, None]                 # c→p 단위벡터 (= -∂r/∂c)
        r = dist - radius
        dc = np.linalg.solve(u.T @ u + 1e-6 * np.eye(3), u.T @ r)
        c = c + dc
        if np.linalg.norm(dc) < tol:
            break
    return c


def fit_circle(centers: Sequence[np.ndarray]) -> Tuple[np.ndarray, np.ndarray]:
    """
    3D 점들(한 구의 θ별 중심)이 이루는 원 → (평면법선, 원중심3D).

    평면법선이 회전축 방향, 원중심이 축 위의 한 점.
    원피팅 수학은 turntable_frame.fit_circle_3d 로 통합(중복 제거).
    """
    from utils.calibration.turntable_frame import fit_circle_3d
    center3d, normal, _r, _res = fit_circle_3d(np.asarray(centers, dtype=float))
    return _norm(normal), center3d


def estimate_axis(sphere_center_tracks: List[Sequence[np.ndarray]],
                  up_hint: np.ndarray = np.array([0.0, 0.0, 1.0])
                  ) -> Tuple[np.ndarray, np.ndarray]:
    """
    여러 구의 중심 궤적 → 공통 회전축.

    Parameters
    ----------
    sphere_center_tracks : 구별 [center(θ0), center(θ1), ...] 리스트들 (각 구 ≥3개 θ)
    up_hint : 축 부호 정렬용 대략 방향 (기본 +Z)

    Returns
    -------
    (axis_point, axis_dir) : 축 위 한 점, 단위 방향벡터
    """
    normals, circ_centers = [], []
    for track in sphere_center_tracks:
        if len(track) < 3:
            continue
        n, c3 = fit_circle(track)
        if n @ up_hint < 0:
            n = -n
        normals.append(n)
        circ_centers.append(c3)
    if len(normals) == 0:
        raise ValueError("축 추정 실패: 유효한 구 궤적이 없음 (구별 θ≥3 필요).")
    axis_dir = _norm(np.mean(normals, axis=0))
    # 원중심들은 축 위에 일렬 → 평균점을 통과점으로, 방향으로 라인 정의
    axis_point = np.mean(circ_centers, axis=0)
    return axis_point, axis_dir


def axis_error(axis_point_est, axis_dir_est,
               axis_point_gt, axis_dir_gt) -> Tuple[float, float]:
    """
    추정 축 vs 기준 축 오차.

    Returns
    -------
    (dir_err_deg, pos_err_m) :
        dir_err_deg : 방향 각오차(도)
        pos_err_m   : 기준축 통과점에서 추정축 라인까지의 수직거리(m)
    """
    d_e = _norm(np.asarray(axis_dir_est, float))
    d_g = _norm(np.asarray(axis_dir_gt, float))
    dir_err = np.degrees(np.arccos(np.clip(abs(d_e @ d_g), -1.0, 1.0)))
    # 점(axis_point_gt)에서 추정축 라인까지 수직거리
    p = np.asarray(axis_point_gt, float) - np.asarray(axis_point_est, float)
    perp = p - (p @ d_e) * d_e
    return float(dir_err), float(np.linalg.norm(perp))
