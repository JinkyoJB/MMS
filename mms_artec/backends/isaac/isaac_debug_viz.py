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


#: 디버그 오버레이가 모이는 루트. **캡처 직전에 통째로 숨긴다**(아래 주석).
DBG_ROOT = "/World/_dbg"


# ⚠ 2026-09-17: 오버레이를 `purpose=guide` 로 표시해 카메라에서 숨기려 했는데,
#   **뷰포트에서도 같이 사라진다**(guide 는 기본 표시 대상이 아니다). 보려고 그리는
#   것이라 본말전도다. 그래서 purpose 는 건드리지 않고, 대신 **캡처 순간에만**
#   `set_hidden(stage, True)` 로 숨긴다(`isaac_scanner.capture_points_base`).
#
#   왜 숨겨야 하나 — `get_pointcloud()` 는 씬을 렌더해서 점군을 만든다. 오버레이가
#   보이면 그게 그대로 캡처에 섞인다. 조용히 치명적이다:
#     · `show_accum` → 누적 점군을 그리면 다음 캡처에 **자기 자신이 다시** 들어온다
#     · 장애물 가드 원기둥(r=150mm·높이 300mm) → 크롭을 그대로 통과해서
#       **물체 대신 원기둥을 측량한다.** 짧은 물체도 상단이 계속 늘어나는 것처럼
#       보여 preview 가 매번 최대 높이까지 올라가고, 모든 물체의 모션이 같아진다.
#       (실측으로 확인됨 — 고친 뒤 물체 높이마다 모션이 달라졌다.)


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


def show_obstacle(stage, pts, tag="", color=(1.0, 0.25, 0.0), max_pts=30000):
    """스캔 대상 **충돌 장애물**로 등록된 점군 — 주황.

    `CollisionModel.set_dynamic_obstacle` 에 실제로 넘어간 바로 그 점을 그린다.
    숫자(bbox·점 수)만으로는 **회전체가 물체를 제대로 감쌌는지**, 프레임이 맞는지를
    알 수 없다 — 비대칭 물체에서 장애물이 엉뚱한 방향을 향해도 bbox 는 그럴듯하다.

    입력은 **world** 프레임이다(호출부가 변환해서 준다).
    """
    if not ENABLED or pts is None or len(pts) == 0:
        return
    try:
        p = np.asarray(pts, float).reshape(-1, 3)
        if len(p) > max_pts:                       # 그리기 비용만 줄인다
            p = p[:: max(1, len(p) // max_pts)]
        _pts_prim(stage, f"/World/_dbg/obstacle{('_' + tag) if tag else ''}",
                  p, 0.004, color)
    except Exception:                                      # noqa: BLE001
        pass


def clear_obstacle(stage, tag=""):
    """장애물 오버레이 제거 (다음 등록 때 겹쳐 그리지 않게)."""
    if not ENABLED:
        return
    try:
        stage.RemovePrim(f"/World/_dbg/obstacle{('_' + tag) if tag else ''}")
    except Exception:                                      # noqa: BLE001
        pass


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


# (show_voxels 삭제 2026-09-18 — '관측 복셀' 오버레이는 nbv 가 복셀 맵으로 계획한다는
#  오해를 낳았다. 계획은 메시 경계(frontier) 기반이고 복셀은 수렴 회계에만 쓴다.)

def set_hidden(stage, hidden: bool) -> None:
    """디버그 오버레이 루트를 통째로 숨기거나 되살린다.

    **오버레이가 카메라에 안 섞이게 하는 유일한 수단**이다(위 주석 참조).
    `purpose=guide` 는 뷰포트에서도 가려 버려서 못 쓴다. 캡처 순간에만 껐다 켜므로
    사람이 보는 동안(캡처 사이·`MMS_SIM_PAUSE` 대기)에는 계속 보인다.
    """
    try:
        from pxr import UsdGeom
        p = stage.GetPrimAtPath(DBG_ROOT)
        if not p.IsValid():
            return
        im = UsdGeom.Imageable(p)
        (im.MakeInvisible() if hidden else im.MakeVisible())
    except Exception:                                          # noqa: BLE001
        pass
