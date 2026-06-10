"""
rim_click_offline.py — rim 3점 클릭 + 원피팅 (오프라인, Isaac 불필요).

Isaac python 은 GUI 툴킷이 없어(cv2 headless·tkinter/Qt 없음) 클릭 창을 못 연다.
그래서 캡처(calib_rim_sim.py, Isaac)와 클릭을 분리한다:
  1) Isaac:  ~/isaacsim/python.sh scripts/sim/calib_rim_sim.py
             → scripts/sim/log/rim_capture.npz 저장
  2) 여기:   python scripts/sim/rim_click_offline.py [rim_capture.npz]
             → GUI 있는 일반 python(시스템/conda)에서 클릭+피팅+저장

캡처는 SimulationApp 이 필요하지만, 클릭+피팅은 numpy(+cv2 또는 matplotlib)만 쓴다.
코어는 PhoXi 와 공용: utils.calibration.{rim_picker, turntable_frame}.

조작
----
  cv2(GUI 빌드): 좌클릭=추가 우클릭=취소 Enter=피팅 d=깊이 q=종료
  matplotlib 폴백: 좌클릭=추가 우클릭=취소 가운데클릭/Enter=완료
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np

_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(_ROOT))

from utils.calibration.turntable_frame import fit_circle_3d, build_T_B_F0
from utils.calibration.turntable_axis import axis_error

DEFAULT_NPZ = Path(__file__).resolve().parent / "log" / "rim_capture.npz"


def _cv2_has_gui() -> bool:
    try:
        import cv2
        info = cv2.getBuildInformation()
        return any(k in info for k in ("GTK", "QT:", "QT5", "QT6", "WIN32UI", "Cocoa"))
    except Exception:
        return False


def _to_base(org, T_CB, u, v):
    H, W = org.shape[:2]
    if not (0 <= v < H and 0 <= u < W):
        return None
    p = org[v, u].astype(np.float64)
    if np.all(p == 0) or not np.isfinite(p).all():
        return None
    return T_CB[:3, :3] @ (p / 1000.0) + T_CB[:3, 3]      # organized 가 mm


def pick_cv2(intensity, org, T_CB):
    """기존 cv2 rim_picker UI (GUI 빌드 필요). 반환 pts_B(N,3) 또는 None(취소)."""
    from utils.calibration.rim_picker import RimPicker, run_picker
    state = RimPicker(intensity, org, T_CB)
    action = run_picker(state)
    if action != "fit":
        return None
    return np.array(state.pts_B, dtype=float)


def pick_matplotlib(intensity, org, T_CB):
    """matplotlib 클릭 폴백 (cv2 GUI 없을 때). 반환 pts_B(N,3) 또는 None."""
    import matplotlib.pyplot as plt
    pts_B, markers = [], []
    fig, ax = plt.subplots(figsize=(12, 9))
    ax.imshow(intensity, cmap="gray")
    ax.set_title("rim 클릭: 좌클릭=추가  우클릭=취소  가운데클릭/Enter=완료  (최소 3점)")

    def redraw():
        fig.canvas.draw_idle()

    def onclick(ev):
        if ev.inaxes != ax or ev.xdata is None:
            return
        if ev.button == 1:
            u, v = int(round(ev.xdata)), int(round(ev.ydata))
            b = _to_base(org, T_CB, u, v)
            if b is None:
                print(f"  ({u},{v}) 유효 깊이 없음 — 다른 위치"); return
            pts_B.append(b)
            mk, = ax.plot(u, v, "o", ms=9, mec="white", mfc="lime")
            tx = ax.annotate(str(len(pts_B)), (u + 9, v - 9), color="lime", fontsize=11)
            markers.append((mk, tx))
            print(f"  [{len(pts_B)}] px=({u},{v}) base(mm)="
                  f"({b[0]*1000:.1f},{b[1]*1000:.1f},{b[2]*1000:.1f})")
            redraw()
        elif ev.button == 3 and pts_B:
            pts_B.pop(); mk, tx = markers.pop(); mk.remove(); tx.remove(); redraw()
        elif ev.button == 2:
            plt.close(fig)

    def onkey(ev):
        if ev.key in ("enter", " "):
            plt.close(fig)

    fig.canvas.mpl_connect("button_press_event", onclick)
    fig.canvas.mpl_connect("key_press_event", onkey)
    plt.show()
    return np.array(pts_B, dtype=float) if pts_B else None


def show_result(pts_B, center_B, normal_B, radius):
    """피팅 결과 top-view (matplotlib)."""
    try:
        import matplotlib.pyplot as plt
    except Exception:
        return
    nz = normal_B / np.linalg.norm(normal_B)
    ref = np.array([1.0, 0, 0]) if abs(nz[0]) < 0.9 else np.array([0, 1.0, 0])
    e1 = ref - ref @ nz * nz; e1 /= np.linalg.norm(e1); e2 = np.cross(nz, e1)
    ang = np.linspace(0, 2 * np.pi, 120)
    circ = center_B + radius * (np.outer(np.cos(ang), e1) + np.outer(np.sin(ang), e2))
    fig, ax = plt.subplots(figsize=(6, 6))
    ax.plot(circ[:, 0], circ[:, 1], "-", color="orange", label="fit circle")
    ax.scatter(pts_B[:, 0], pts_B[:, 1], c="lime", edgecolors="k", label="clicks")
    ax.scatter([center_B[0]], [center_B[1]], marker="x", s=160, c="r", label="center(axis)")
    ax.set_aspect("equal"); ax.grid(alpha=0.3); ax.legend()
    ax.set_xlabel("X base (m)"); ax.set_ylabel("Y base (m)")
    ax.set_title("rim 3-point fit (base frame, top view)")
    plt.show()


def main():
    npz = Path(sys.argv[1]) if len(sys.argv) > 1 else DEFAULT_NPZ
    if not npz.exists():
        print(f"[rim-offline] 캡처 없음: {npz}\n  먼저 Isaac 에서 calib_rim_sim.py 실행."); return
    d = np.load(npz)
    intensity = d["intensity"]; org = d["organized_pts"]; T_CB = d["T_CB"]
    print(f"[rim-offline] 로드 {npz}  intensity={intensity.shape} org={org.shape}")

    pts_B = pick_cv2(intensity, org, T_CB) if _cv2_has_gui() else None
    if pts_B is None:
        if _cv2_has_gui():
            print("[rim-offline] 취소됨."); return
        print("[rim-offline] cv2 GUI 없음 → matplotlib 클릭 사용.")
        pts_B = pick_matplotlib(intensity, org, T_CB)
    if pts_B is None or len(pts_B) < 3:
        print("[rim-offline] 3점 미만 — 중단."); return

    center_B, normal_B, radius, residual = fit_circle_3d(pts_B)
    T_B_F0 = build_T_B_F0(center_B, normal_B)
    print("=" * 56)
    print(f"[rim-offline] {len(pts_B)}점  반경={radius*1000:.1f}mm  RMS={residual*1000:.3f}mm")
    print(f"  중심(base mm)=({center_B[0]*1000:.1f},{center_B[1]*1000:.1f},{center_B[2]*1000:.1f})")
    print(f"  법선(base)=({normal_B[0]:.4f},{normal_B[1]:.4f},{normal_B[2]:.4f})")
    out = {"frame": "robot_base", "method": "rim_3point_offline", "n_points": int(len(pts_B)),
           "center_base_m": [float(v) for v in center_B],
           "normal_base": [float(v) for v in normal_B],
           "radius_mm": float(radius * 1000), "rms_mm": float(residual * 1000),
           "surface_height_base_mm": float(center_B[2] * 1000),
           "T_B_F0": [[float(v) for v in row] for row in T_B_F0.tolist()]}
    if "gt_center_base" in d:                              # sim 캡처면 GT 비교
        de, pe = axis_error(center_B, normal_B, d["gt_center_base"], d["gt_normal_base"])
        out["dir_err_deg"] = float(de); out["pos_err_mm"] = float(pe * 1000)
        print(f"  [GT] 축오차={de:.3f}deg  중심XY오차={pe*1000:.2f}mm")
    print("=" * 56)
    res_path = npz.parent / "rim_result.json"
    res_path.write_text(json.dumps(out, indent=2, ensure_ascii=False))
    print(f"[rim-offline] 저장 → {res_path}")
    show_result(pts_B, center_B, normal_B, radius)


if __name__ == "__main__":
    main()
