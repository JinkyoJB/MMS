"""거리추종 디버그 이미지 — "카메라가 정말 멀어졌나/가까워졌나" 를 눈으로 본다.

스캐너 프레임 정점(원점 = 카메라, m)을 **각도 좌표**(광축 기준 가로·세로 각)로
펼쳐 2D 이미지에 찍고, 색은 카메라까지의 거리다. 작동거리 창(dof)을 색 범위로
써서 창 안(초록~노랑)·너무 가까움(파랑)·너무 멂(빨강)이 한눈에 보인다.
핵심 높이대(광축 세로 ±TRACK_CORE_HALF_DEG)는 흰 가로선 두 줄로 표시하고,
그 안 점의 중앙값(= 거리추종이 쓰는 값)을 제목에 적는다.

텍스처 프레임(`frame_mesh.image()`)이 있으면 오른쪽에 나란히 붙인다 — 3D 점이
정말 그 장면에서 나온 것인지 대조용.

왜 이미지인가 — 2026-09-22 run_160447: 축거리 -25mm 이동에 표면거리 측정이 +3mm
만 변해 거리추종이 스스로 꺼졌다. 로그 숫자만으로는 "로봇이 안 움직였나 / 프레임이
이동 전 것인가 / 다른 점을 재고 있나" 를 가를 수 없었다. 이동 전·후 이미지 두 장이
있으면 즉시 가려진다.
"""
from __future__ import annotations

import math
import os
from typing import Optional, Sequence

import numpy as np

# 이미지 크기 · 각도 범위 (Spider 가로 FOV ≈ 38°, 세로 ≈ 29° — 여유 있게)
_W, _H = 480, 360
_HALF_X_DEG, _HALF_Y_DEG = 22.0, 17.0


def _colormap(t: np.ndarray) -> np.ndarray:
    """t∈[0,1] → BGR (0=파랑 가까움 · 0.5=초록 창중앙 · 1=빨강 멂)."""
    t = np.clip(t, 0.0, 1.0)
    r = np.clip(2.0 * t - 1.0, 0, 1)                   # 0.5→1 로 증가
    b = np.clip(1.0 - 2.0 * t, 0, 1)                   # 0→0.5 로 감소
    g = 1.0 - np.abs(2.0 * t - 1.0)                    # 0.5 에서 1
    return (np.stack([b, g, r], axis=1) * 255).astype(np.uint8)


def render_range_image(verts_m: np.ndarray, dof: Sequence[float],
                       core_half_deg: float, title_lines: Sequence[str],
                       tex_img: Optional[np.ndarray] = None) -> Optional[np.ndarray]:
    """BGR 이미지(np.uint8) 를 돌려준다. cv2 가 없으면 None."""
    try:
        import cv2
    except Exception:                                   # noqa: BLE001
        return None
    P = np.asarray(verts_m, float)
    img = np.full((_H, _W, 3), 24, np.uint8)
    if P.ndim == 2 and P.shape[0] > 0:
        z = np.maximum(np.abs(P[:, 2]), 1e-6)     # Artec 스캐너 프레임: 광축 −z
        ax = np.degrees(np.arctan2(P[:, 0], z))
        ay = np.degrees(np.arctan2(P[:, 1], z))
        u = ((ax / _HALF_X_DEG) * 0.5 + 0.5) * (_W - 1)
        v = ((ay / _HALF_Y_DEG) * 0.5 + 0.5) * (_H - 1)
        ok = (u >= 0) & (u < _W) & (v >= 0) & (v < _H)
        rng = np.linalg.norm(P, axis=1)
        lo, hi = float(dof[0]) - 0.03, float(dof[1]) + 0.03
        col = _colormap((rng - lo) / max(hi - lo, 1e-6))
        ui, vi = u[ok].astype(np.int32), v[ok].astype(np.int32)
        # 먼 점부터 찍어 가까운 점이 위에 남게
        order = np.argsort(-rng[ok])
        img[vi[order], ui[order]] = col[ok][order]
    # 핵심 높이대 (세로 ±core_half_deg) — 흰 가로선
    for sgn in (-1.0, 1.0):
        vy = int(((sgn * core_half_deg / _HALF_Y_DEG) * 0.5 + 0.5) * (_H - 1))
        cv2.line(img, (0, vy), (_W - 1, vy), (255, 255, 255), 1)
    # 광축 십자
    cv2.drawMarker(img, (_W // 2, _H // 2), (200, 200, 200), cv2.MARKER_CROSS, 12, 1)
    # 색 범례 (하단)
    bar = _colormap(np.linspace(0, 1, _W))
    img[_H - 8:_H, :, :] = bar[None, :, :]
    cv2.putText(img, f"{(float(dof[0]) - 0.03) * 1000:.0f}", (2, _H - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 255, 255), 1)
    cv2.putText(img, f"{(float(dof[1]) + 0.03) * 1000:.0f}mm", (_W - 60, _H - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 255, 255), 1)
    y = 14
    for ln in title_lines:
        cv2.putText(img, str(ln), (4, y), cv2.FONT_HERSHEY_SIMPLEX, 0.42,
                    (255, 255, 255), 1, cv2.LINE_AA)
        y += 15
    if tex_img is not None:
        try:
            t = np.asarray(tex_img)
            if t.ndim == 2:
                t = cv2.cvtColor(t, cv2.COLOR_GRAY2BGR)
            elif t.shape[2] == 4:
                t = t[:, :, :3]
            scale = _H / float(t.shape[0])
            t = cv2.resize(t, (int(t.shape[1] * scale), _H))
            img = np.hstack([img, t])
        except Exception:                               # noqa: BLE001
            pass
    return img


def save_range_image(path: str, verts_m: np.ndarray, dof: Sequence[float],
                     core_half_deg: float, title_lines: Sequence[str],
                     tex_img: Optional[np.ndarray] = None) -> bool:
    img = render_range_image(verts_m, dof, core_half_deg, title_lines, tex_img)
    if img is None:
        return False
    try:
        import cv2
        os.makedirs(os.path.dirname(path), exist_ok=True)
        return bool(cv2.imwrite(path, img))
    except Exception:                                   # noqa: BLE001
        return False
