"""isaac_debug_viz — 스캔 상태를 **Isaac 뷰포트에 직접** 그린다 (sim 전용 디버그).

왜 필요한가
----------
nbv 가 "무엇을 왜 겨냥하는지"를 로그 숫자만으로 판단하기 어렵다. gap 이 어디에
잡혔는지, NBV 가 어디를 보러 갔는지, 누적 점군이 어디가 비었는지를 **씬 안에서**
보면 진단이 훨씬 빠르다. 시뮬레이터를 쓰는 이점 중 가장 큰 것이다.

그리는 것 (`/World/_debug` 아래, 스캔 대상과 분리)
    · accum   : 누적 점군 (다운샘플)
    · gaps    : gap 대표점 — 크기 L 에 비례한 점
    · target  : 이번에 겨냥한 gap + 카메라 위치(선분)
    · voxel   : 관측/미지 복셀 (옵션)

`MMS_SIM_VIZ=1` 일 때만 동작하며, 실패해도 스캔을 막지 않는다(전부 try 로 감쌈).
"""
from __future__ import annotations

import os

import numpy as np

ROOT = "/World/_debug"
ENABLED = os.environ.get("MMS_SIM_VIZ", "0") == "1"
# 오버레이 불투명도 — 1.0 이면 씬을 가린다. 뒤의 실물 형상이 비쳐 보이도록 낮춘다.
OPACITY = float(os.environ.get("MMS_SIM_VIZ_OPACITY", "0.35"))


def _pts_prim(stage, path, pts, width, color):
    from pxr import UsdGeom, Vt, Gf
    p = stage.GetPrimAtPath(path)
    g = UsdGeom.Points(p) if p.IsValid() else UsdGeom.Points.Define(stage, path)
    pts = np.asarray(pts, float).reshape(-1, 3)
    g.GetPointsAttr().Set(Vt.Vec3fArray([Gf.Vec3f(*map(float, v)) for v in pts]))
    g.GetWidthsAttr().Set(Vt.FloatArray([float(width)] * len(pts)))
    g.GetDisplayColorAttr().Set(Vt.Vec3fArray([Gf.Vec3f(*color)] * max(1, len(pts))))
    g.GetDisplayOpacityAttr().Set(Vt.FloatArray([OPACITY] * max(1, len(pts))))
    return g


def _line_prim(stage, path, a, b, color):
    from pxr import UsdGeom, Vt, Gf
    p = stage.GetPrimAtPath(path)
    c = UsdGeom.BasisCurves(p) if p.IsValid() else UsdGeom.BasisCurves.Define(stage, path)
    c.GetTypeAttr().Set("linear")
    c.GetCurveVertexCountsAttr().Set(Vt.IntArray([2]))
    c.GetPointsAttr().Set(Vt.Vec3fArray([Gf.Vec3f(*map(float, a)), Gf.Vec3f(*map(float, b))]))
    c.GetWidthsAttr().Set(Vt.FloatArray([0.004, 0.004]))
    c.GetDisplayColorAttr().Set(Vt.Vec3fArray([Gf.Vec3f(*color)]))
    c.GetDisplayOpacityAttr().Set(Vt.FloatArray([min(1.0, OPACITY + 0.4)]))
    return c


def show_accum(stage, pts, max_pts=40000, theta=0.0, axis_xy=None):
    """누적 점군 — 흰색. theta/axis_xy 를 주면 현재 턴테이블 각으로 되돌려 그린다."""
    if not ENABLED or pts is None or len(pts) == 0:
        return
    try:
        p = np.asarray(pts, float)
        if axis_xy is not None and abs(theta) > 1e-9:
            ax = np.asarray(axis_xy, float)
            c, s_ = np.cos(theta), np.sin(theta)
            d = p[:, :2] - ax
            p = np.column_stack([ax[0] + c*d[:, 0] - s_*d[:, 1],
                                 ax[1] + s_*d[:, 0] + c*d[:, 1], p[:, 2]])
        if len(p) > max_pts:
            p = p[:: int(np.ceil(len(p) / max_pts))]
        _pts_prim(stage, f"{ROOT}/accum", p, 0.0015, (0.85, 0.85, 0.85))
    except Exception as e:                                   # noqa: BLE001
        print(f"[viz] accum 실패({type(e).__name__})")


def show_gaps(stage, gaps):
    """gap 대표점 — 빨강. 크기는 L 에 비례(큰 결손이 눈에 띄게)."""
    if not ENABLED:
        return
    try:
        if not gaps:
            _pts_prim(stage, f"{ROOT}/gaps", np.zeros((0, 3)), 0.006, (1, 0, 0))
            return
        P = np.array([np.asarray(c.p_O, float) for c in gaps])
        L = np.array([float(c.L) for c in gaps])
        w = float(np.clip(L.mean() * 0.6, 0.004, 0.015))
        _pts_prim(stage, f"{ROOT}/gaps", P, w, (1.0, 0.15, 0.15))
    except Exception as e:                                   # noqa: BLE001
        print(f"[viz] gaps 실패({type(e).__name__})")


def show_target(stage, gap_p, cam_pos):
    """이번에 겨냥한 gap(초록) + 카메라까지 선분 — '왜 거기로 갔나' 가 한눈에."""
    if not ENABLED:
        return
    try:
        _pts_prim(stage, f"{ROOT}/target", np.asarray(gap_p).reshape(1, 3), 0.012,
                  (0.1, 1.0, 0.1))
        _pts_prim(stage, f"{ROOT}/cam", np.asarray(cam_pos).reshape(1, 3), 0.010,
                  (0.2, 0.6, 1.0))
        _line_prim(stage, f"{ROOT}/ray", cam_pos, gap_p, (0.2, 0.8, 0.2))
    except Exception as e:                                   # noqa: BLE001
        print(f"[viz] target 실패({type(e).__name__})")


def show_voxels(stage, pts, center, half, voxel=0.006, theta=0.0, axis_xy=None):
    """관측 복셀(파랑). **미지(주황)는 그리지 않는다** — 너무 많아 다른 게 안 보인다
    (사용자 피드백 2026-08-19). 필요하면 `show_unknown=True` 로 되살릴 것.

    theta/axis_xy 를 주면 canonical 점군을 **현재 턴테이블 각으로 되돌려** 그린다.
    누적 점군은 −θ 로 되돌려 저장되므로(그래야 합쳐진다) 그대로 그리면 물리 물체와
    어긋나 보인다. 겹쳐 보여야 "지금 어디가 채워졌나"를 눈으로 확인할 수 있다.
    """
    if not ENABLED or pts is None or len(pts) == 0:
        return
    try:
        P = np.asarray(pts, float)
        if axis_xy is not None and abs(theta) > 1e-9:
            ax = np.asarray(axis_xy, float)
            c, s_ = np.cos(theta), np.sin(theta)
            d = P[:, :2] - ax
            P = np.column_stack([ax[0] + c*d[:, 0] - s_*d[:, 1],
                                 ax[1] + s_*d[:, 0] + c*d[:, 1], P[:, 2]])
        cen = np.asarray(center, float); h = float(half)
        lo = cen - h
        n = int(np.ceil(2 * h / voxel))
        idx = np.floor((P - lo) / voxel).astype(int)
        idx = idx[np.all((idx >= 0) & (idx < n), axis=1)]
        if not len(idx):
            return
        occ = np.zeros((n, n, n), bool)
        occ[idx[:, 0], idx[:, 1], idx[:, 2]] = True
        gi = np.argwhere(occ)
        _pts_prim(stage, f"{ROOT}/vox_seen", lo + (gi + 0.5) * voxel, voxel * 0.8,
                  (0.25, 0.45, 1.0))
    except Exception as e:                                   # noqa: BLE001
        print(f"[viz] voxel 실패({type(e).__name__})")
