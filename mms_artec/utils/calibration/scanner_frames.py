"""스캐너 3D 프레임 ↔ Color(텍스처) 카메라 프레임.

왜 이 모듈이 있나
----------------
Artec Spider 는 **3D 카메라 3대 + Color 카메라 1대**(+프로젝터)다.

  · `fmh.vertices()` → 3D 카메라들이 만든 **스캐너 프레임**
  · `fmh.image()` / `K` / `solvePnP` → **Color 카메라 프레임**

둘 사이에 고정 강체변환이 있다. 2026-09-16 실측(`SP.10.79103441`):

    회전 179.9° · 이동 [-0.2, 41, -5] mm · 재투영 0.5px · inlier 97%
    6프레임 표준편차 t [0.23, 0.04, 0.01] mm  → 고정 외부파라미터

그런데 `T_EC`(hand-eye)는 **solvePnP**(2D 픽셀만)로 풀어서 **Color 기준**이다.
그러므로 `vertices()` 를 `T_CB = T_EB·inv(T_EC)` 로 base 로 옮기려면 **먼저 이
변환을 적용해야 한다.** 빠뜨리면 180° 회전 + 41mm 가 통째로 오차가 된다.

실측된 증상 두 가지 (둘 다 2026-09-16):
  · rim 캘리브 — 피팅 원이 실제 테두리에서 23~43mm 어긋남
  · lookaround preview — 점이 턴테이블 축에서 **0.44m** 떨어진 곳에 찍혀
    기하 크롭(r<0.16m)이 **전부 버림**(정점 3만 → 0점) → 플래너가 계획 실패

하드코딩하지 않는 이유
--------------------
이 값은 `T_EC` 처럼 **개체 종속**이라 스캐너를 바꾸면 달라진다. 필요한 데이터
(vertices + uv)가 이미 매 프레임에 있으므로 별도 캘리브 없이 풀 수 있다.
"""
from __future__ import annotations

from typing import Optional, Tuple

import cv2
import numpy as np


def solve_scanner_to_color(vertices_mm, uv, image_wh: Tuple[int, int], K, dist,
                           n_sample: int = 6000, seed: int = 0,
                           max_reproj_px: float = 3.0,
                           log=None) -> Optional[np.ndarray]:
    """프레임 데이터에서 **스캐너3D → Color** 강체변환을 푼다.

    3D 점(vertices, 스캐너 프레임)과 그 UV 픽셀은 같은 대응쌍이므로 PnP 로
    Color 카메라 기준 스캐너 프레임 자세가 나온다.

    Returns: (4,4) T — `x_color = T @ x_scanner` (mm). 실패하면 None.
    """
    W, H = image_wh
    V = np.asarray(vertices_mm, float)
    if len(V) < 200:
        return None
    px = np.asarray(uv, float).copy()
    px[:, 0] *= float(W)
    px[:, 1] *= float(H)

    idx = np.random.default_rng(seed).choice(
        len(V), size=min(n_sample, len(V)), replace=False)
    ok, rv, tv, inl = cv2.solvePnPRansac(
        V[idx], px[idx], np.asarray(K, float), np.asarray(dist, float).reshape(-1),
        flags=cv2.SOLVEPNP_ITERATIVE, reprojectionError=max_reproj_px,
        iterationsCount=300)
    if not ok or inl is None or len(inl) < 100:
        if log:
            log("스캐너3D→Color 변환 실패 (PnP 해 없음)")
        return None

    sel = inl.ravel()
    rv, tv = cv2.solvePnPRefineLM(V[idx][sel], px[idx][sel],
                                  np.asarray(K, float),
                                  np.asarray(dist, float).reshape(-1), rv, tv)
    R, _ = cv2.Rodrigues(rv)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = tv.ravel()

    proj = cv2.projectPoints(V[idx][sel], rv, tv, np.asarray(K, float),
                             np.asarray(dist, float).reshape(-1))[0].reshape(-1, 2)
    err = float(np.median(np.linalg.norm(proj - px[idx][sel], axis=1)))
    ang = float(np.degrees(np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1))))
    if log:
        log(f"스캐너3D→Color  회전 {ang:.2f}°  이동 "
            f"{np.round(tv.ravel(), 1).tolist()}mm  재투영 {err:.2f}px  "
            f"inlier {len(sel)}/{len(idx)}")
    if err > max_reproj_px:
        if log:
            log(f"⚠ 재투영 오차가 크다 ({err:.1f}px) — 결과를 믿지 말 것")
    return T


def apply(T_scan_color: Optional[np.ndarray], vertices_mm) -> np.ndarray:
    """`vertices_mm`(스캐너 프레임) → Color 프레임. `T` 가 None 이면 그대로 돌려준다.

    None 폴백을 조용히 두지 말 것 — 호출부가 경고를 띄워야 한다.
    """
    V = np.asarray(vertices_mm, float)
    if T_scan_color is None:
        return V
    T = np.asarray(T_scan_color, float)
    return (T[:3, :3] @ V.T).T + T[:3, 3]
