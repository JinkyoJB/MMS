"""
geometry.py — 충돌검사용 기초 기하 (순수 numpy, sensor/backend 무관).

캡슐(선분+반경) 근사로 충돌을 본다. 핵심은 **선분-선분 최단거리**:
두 캡슐은 (선분거리 < r1+r2) 이면 충돌. 평면(반평면)은 선분 최소 부호거리로 본다.
"""

from __future__ import annotations

import numpy as np

_EPS = 1e-9


def seg_seg_distance(p1, q1, p2, q2) -> float:
    """선분 [p1,q1] 와 [p2,q2] 사이 최단거리 (Ericson, Real-Time Collision Detection)."""
    p1 = np.asarray(p1, float); q1 = np.asarray(q1, float)
    p2 = np.asarray(p2, float); q2 = np.asarray(q2, float)
    d1 = q1 - p1            # 선분1 방향
    d2 = q2 - p2            # 선분2 방향
    r = p1 - p2
    a = float(d1 @ d1)      # |d1|^2
    e = float(d2 @ d2)      # |d2|^2
    f = float(d2 @ r)

    if a <= _EPS and e <= _EPS:               # 둘 다 점
        return float(np.linalg.norm(p1 - p2))
    if a <= _EPS:                             # 선분1 이 점
        s = 0.0
        t = np.clip(f / e, 0.0, 1.0)
    else:
        c = float(d1 @ r)
        if e <= _EPS:                         # 선분2 가 점
            t = 0.0
            s = np.clip(-c / a, 0.0, 1.0)
        else:                                 # 일반
            b = float(d1 @ d2)
            denom = a * e - b * b
            s = np.clip((b * f - c * e) / denom, 0.0, 1.0) if denom > _EPS else 0.0
            t = (b * s + f) / e
            if t < 0.0:
                t = 0.0; s = np.clip(-c / a, 0.0, 1.0)
            elif t > 1.0:
                t = 1.0; s = np.clip((b - c) / a, 0.0, 1.0)
    c1 = p1 + d1 * s
    c2 = p2 + d2 * t
    return float(np.linalg.norm(c1 - c2))


def seg_halfspace_min_signed(p0, p1, point, normal) -> float:
    """
    선분 [p0,p1] 의 점들 중, 평면(point, 단위 normal)에 대한 **최소 부호거리**.

    부호거리 = (x - point)·normal. normal 은 '허용영역(keep-in)' 쪽을 가리킨다고 보고,
    이 값이 음수면 선분이 평면 너머(금지영역)로 넘어간 것.
    """
    p0 = np.asarray(p0, float); p1 = np.asarray(p1, float)
    point = np.asarray(point, float); normal = np.asarray(normal, float)
    d0 = float((p0 - point) @ normal)
    d1 = float((p1 - point) @ normal)
    return min(d0, d1)
