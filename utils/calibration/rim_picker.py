"""
rim_picker.py — 턴테이블 rim 3점 클릭 UI (OpenCV) + 3D 결과 뷰 (Open3D). sensor-agnostic.

사용자가 조직화된 포인트클라우드(또는 depth→organized) 위에서 rim 점을 클릭 →
base 프레임 3D 점들을 얻는다. 그 점들을 turntable_frame.fit_circle_3d /build_T_B_F0
로 넘기면 T_B_F0 가 나온다.

어느 센서든 다음만 주면 된다:
  intensity      : (H, W) uint8        — 표시용 강도 영상
  organized_pts  : (H, W, 3) float32   — 픽셀별 3D (센서 프레임 C, mm). depth+intrinsic
                   으로부터 만들 수도 있음(Artec/Isaac).
  T_CB           : (4, 4)              — C→B (x_B = T_CB @ x_C, meters)

cv2 필요 (실물/콘다 환경). Isaac sim 경로는 이 모듈을 import 하지 않는다.
"""

from __future__ import annotations

from typing import List, Optional, Tuple

import cv2
import numpy as np

MIN_RIM_PTS = 3
WARN_RIM_PTS = 6
_WIN = ("Turntable Rim Picker"
        "  [LClick=add  RClick=undo  Enter=fit  r=recapture  d=depth  q=quit]")
_PT_COLORS = [(0, 255, 0), (0, 200, 255), (255, 120, 0),
              (180, 0, 255), (255, 255, 0), (0, 180, 255)]


def depth_colormap(organized_pts: np.ndarray) -> np.ndarray:
    """조직화 포인트클라우드 Z(mm) → 깊이 컬러맵 (BGR)."""
    z = organized_pts[:, :, 2].astype(np.float32)
    valid = z > 0.0
    if not valid.any():
        return np.zeros((*z.shape, 3), dtype=np.uint8)
    lo = float(np.percentile(z[valid], 5))
    hi = float(np.percentile(z[valid], 95))
    z_norm = np.clip((z - lo) / max(hi - lo, 1e-6), 0.0, 1.0)
    z_u8 = (z_norm * 255).astype(np.uint8)
    z_u8[~valid] = 0
    return cv2.applyColorMap(z_u8, cv2.COLORMAP_JET)


class RimPicker:
    """OpenCV 마우스 콜백 + 렌더링 상태. pixel→base 3D 변환 내장."""

    def __init__(self, intensity: np.ndarray, organized_pts: np.ndarray,
                 T_CB: np.ndarray, min_pts: int = MIN_RIM_PTS,
                 warn_pts: int = WARN_RIM_PTS) -> None:
        self.intensity = intensity
        self.org_pts = organized_pts
        self.T_CB = T_CB
        self.min_pts = min_pts
        self.warn_pts = warn_pts
        self._depth_vis = depth_colormap(organized_pts)
        self.show_depth = False
        self.pixels: List[Tuple[int, int]] = []
        self.pts_B: List[np.ndarray] = []
        self.status = "rim 위를 클릭하세요"
        self.redraw = True

    def _px_to_3d(self, u: int, v: int) -> Optional[np.ndarray]:
        H, W = self.org_pts.shape[:2]
        if not (0 <= v < H and 0 <= u < W):
            return None
        pt_S_mm = self.org_pts[v, u].astype(np.float64)
        if not np.isfinite(pt_S_mm).all() or np.all(pt_S_mm == 0):
            return None
        pt_S_m = pt_S_mm / 1000.0
        return self.T_CB[:3, :3] @ pt_S_m + self.T_CB[:3, 3]

    def on_mouse(self, event, x, y, flags, param) -> None:
        if event == cv2.EVENT_LBUTTONDOWN:
            pt_B = self._px_to_3d(x, y)
            if pt_B is None:
                self.status = f"[!] ({x},{y}) 유효 깊이 없음 — 다른 위치 클릭"
            else:
                self.pixels.append((x, y)); self.pts_B.append(pt_B)
                mm = pt_B * 1000.0
                self.status = (f"[{len(self.pts_B)}] px=({x},{y})  "
                               f"B=({mm[0]:.1f},{mm[1]:.1f},{mm[2]:.1f}) mm")
            self.redraw = True
        elif event == cv2.EVENT_RBUTTONDOWN:
            if self.pixels:
                self.pixels.pop(); self.pts_B.pop()
                self.status = f"마지막 점 취소 — 남은 점: {len(self.pts_B)}"
                self.redraw = True

    def render(self) -> np.ndarray:
        base = (self._depth_vis.copy() if self.show_depth
                else cv2.cvtColor(self.intensity, cv2.COLOR_GRAY2BGR))
        for i, (u, v) in enumerate(self.pixels):
            color = _PT_COLORS[i % len(_PT_COLORS)]
            cv2.circle(base, (u, v), 7, (255, 255, 255), -1)
            cv2.circle(base, (u, v), 6, color, -1)
            cv2.putText(base, str(i + 1), (u + 9, v - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)
        H, W = base.shape[:2]
        n = len(self.pts_B)
        if n < self.min_pts:
            hint = f"rim 위를 클릭하세요 ({self.min_pts - n}점 더 필요)"
        elif n < self.warn_pts:
            hint = f"Enter=피팅  (현재 {n}점, {self.warn_pts}점 이상 권장)"
        else:
            hint = f"Enter=피팅  ({n}점 수집됨)"
        cv2.rectangle(base, (0, 0), (W, 34), (20, 20, 20), -1)
        cv2.putText(base, hint, (8, 23), cv2.FONT_HERSHEY_SIMPLEX, 0.65,
                    (255, 255, 255), 1, cv2.LINE_AA)
        cv2.rectangle(base, (0, H - 30), (W, H), (20, 20, 20), -1)
        cv2.putText(base, self.status, (8, H - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                    (180, 255, 180), 1, cv2.LINE_AA)
        return base


def run_picker(state: RimPicker) -> str:
    """인터랙티브 피커 실행. 반환: 'fit' | 'recapture' | 'quit'."""
    cv2.namedWindow(_WIN, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(_WIN, 960, 720)
    cv2.setMouseCallback(_WIN, state.on_mouse)
    while True:
        if state.redraw:
            cv2.imshow(_WIN, state.render()); state.redraw = False
        key = cv2.waitKey(30) & 0xFF
        if key in (13, 32):                              # Enter/Space
            if len(state.pts_B) < state.min_pts:
                state.status = f"[!] 최소 {state.min_pts}점 필요 (현재 {len(state.pts_B)}점)"
                state.redraw = True
            else:
                cv2.destroyAllWindows(); return "fit"
        elif key == ord("r"):
            cv2.destroyAllWindows(); return "recapture"
        elif key == ord("d"):
            state.show_depth = not state.show_depth; state.redraw = True
        elif key in (27, ord("q")):
            cv2.destroyAllWindows(); return "quit"


def show_3d_result(org_pts, intensity, T_CB, center_B, normal_B, radius_m,
                   T_B_F0, rim_pts_B) -> None:
    """피팅 결과를 Open3D 로 표시 (포인트클라우드 + 원 + rim점 + 중심 + F/B 축)."""
    try:
        import open3d as o3d
    except ImportError:
        print("[3D View] open3d 없음 — pip install open3d"); return

    H, W = org_pts.shape[:2]
    flat_pts = org_pts.reshape(-1, 3).astype(np.float64)
    flat_gray = intensity.reshape(-1).astype(np.float64) / 255.0
    valid = ~np.all(flat_pts == 0, axis=1) & np.isfinite(flat_pts).all(axis=1)
    pts_S_m = flat_pts[valid] / 1000.0
    grays = flat_gray[valid]
    pts_B_all = (T_CB[:3, :3] @ pts_S_m.T).T + T_CB[:3, 3]

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pts_B_all)
    pcd.colors = o3d.utility.Vector3dVector(np.column_stack([grays, grays, grays]))
    if len(pts_B_all) > 300_000:
        pcd = pcd.voxel_down_sample(voxel_size=0.002)

    n_seg = 120
    ang = np.linspace(0, 2 * np.pi, n_seg, endpoint=False)
    nz = normal_B / np.linalg.norm(normal_B)
    ref = np.array([1.0, 0.0, 0.0]) if abs(nz[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
    e1 = ref - np.dot(ref, nz) * nz; e1 /= np.linalg.norm(e1)
    e2 = np.cross(nz, e1)
    circle_pts = center_B + radius_m * (np.outer(np.cos(ang), e1) + np.outer(np.sin(ang), e2))
    circle_ls = o3d.geometry.LineSet(
        points=o3d.utility.Vector3dVector(circle_pts),
        lines=o3d.utility.Vector2iVector([[i, (i + 1) % n_seg] for i in range(n_seg)]))
    circle_ls.paint_uniform_color([1.0, 0.5, 0.0])

    geoms = [pcd, circle_ls]
    for pt in rim_pts_B:
        s = o3d.geometry.TriangleMesh.create_sphere(radius=0.005)
        s.translate(pt); s.paint_uniform_color([0.1, 1.0, 0.2]); geoms.append(s)
    center_sph = o3d.geometry.TriangleMesh.create_sphere(radius=0.008)
    center_sph.translate(center_B); center_sph.paint_uniform_color([1.0, 0.1, 0.1])
    geoms.append(center_sph)
    T_F_B = np.linalg.inv(T_B_F0)
    f_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=max(0.08, radius_m * 0.6))
    f_frame.transform(T_F_B); geoms.append(f_frame)
    geoms.append(o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.05))

    print("[3D View] 창을 닫으면 저장 여부를 선택합니다. (드래그=회전 휠=줌)")
    o3d.visualization.draw_geometries(
        geoms, window_name="Turntable Frame — 3D 결과 (F축: R=x G=y B=z)",
        width=1280, height=800)
