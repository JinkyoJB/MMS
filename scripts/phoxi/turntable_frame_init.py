#!/usr/bin/env python3
"""
scripts/turntable_frame_init.py

T_B_F0 캘리브레이션: 턴테이블 rim 클릭 → 3D 원 피팅 → ^B T_F(0) 추정.

사전 조건
---------
- hand-eye 캘리브레이션 완료: config/calibration/hand_eye_phoxi.yaml
- 턴테이블 각도 = 0 (theta = 0 상태)
- 로봇이 턴테이블 rim이 잘 보이는 위치

조작법 (OpenCV 창)
------------------
  좌클릭       rim 점 추가 (최소 3개, 6개 이상 권장)
  우클릭       마지막 점 취소
  Enter/Space  원 피팅 실행
  r            재캡처
  d            강도/깊이 뷰 토글
  q / Esc      종료

3D 확인 창 (Open3D)
-------------------
  마우스 왼쪽 드래그  회전
  마우스 오른쪽 드래그 이동
  휠                  줌
  창 닫기             저장 여부 선택으로 이동

출력: config/calibration/turntable_frame.yaml
"""

from __future__ import annotations

import logging
import sys
from pathlib import Path
from typing import List, Optional, Tuple

import cv2
import numpy as np
import yaml
from scipy.spatial.transform import Rotation as ScipyR

_PROJECT_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(_PROJECT_ROOT))

from utils.robot.xarm_interface import XArmInterface
from mms_phoxi.sensor.phoxi_client import PhoxiClient, PhoxiConfig
from utils.transforms import load_transform

# ── CONFIG ─────────────────────────────────────────────────────────────────────

ROBOT_IP       = "192.168.1.210"
HAND_EYE_YAML  = _PROJECT_ROOT / "config" / "calibration" / "hand_eye_phoxi.yaml"
OUTPUT_YAML    = _PROJECT_ROOT / "config" / "calibration" / "turntable_frame.yaml"
MIN_RIM_PTS    = 3      # 피팅에 필요한 최소 점 수
WARN_RIM_PTS   = 6      # 이 수 미만이면 정확도 경고
WARN_RESIDUAL  = 5.0    # mm — 이 이상이면 품질 경고

_WIN = (
    "Turntable Rim Picker"
    "  [LClick=add  RClick=undo  Enter=fit  r=recapture  d=depth  q=quit]"
)

# ── 3D Circle Fitting ───────────────────────────────────────────────────────────

def fit_circle_3d(
    pts: np.ndarray,
) -> Tuple[np.ndarray, np.ndarray, float, float]:
    """
    평면 위 3D 점들에 원을 피팅한다.

    Parameters
    ----------
    pts : (N, 3) float64, 단위 임의 (meters 권장)

    Returns
    -------
    center   : (3,) float64  원의 중심
    normal   : (3,) float64  평면 법선 (단위벡터)
    radius   : float         원 반지름 (입력 단위)
    residual : float         RMS 잔차 (입력 단위)
    """
    if len(pts) < 3:
        raise ValueError(f"최소 3점 필요 (현재 {len(pts)}점)")

    # 1. SVD 평면 피팅
    centroid = pts.mean(axis=0)
    _, _, Vt = np.linalg.svd(pts - centroid)
    normal = Vt[-1]  # 최소 특이값 → 법선

    # 2. 평면 내 정규직교 기저 구성
    ref = np.array([1.0, 0.0, 0.0]) if abs(normal[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
    e1 = ref - np.dot(ref, normal) * normal
    e1 /= np.linalg.norm(e1)
    e2 = np.cross(normal, e1)

    # 3. 평면에 투영 (2D)
    delta = pts - centroid
    u = delta @ e1
    v = delta @ e2

    # 4. 2D 원 피팅: u^2 + v^2 = 2*cx*u + 2*cy*v + d
    A = np.column_stack([2.0 * u, 2.0 * v, np.ones(len(u))])
    b = u**2 + v**2
    x, _, _, _ = np.linalg.lstsq(A, b, rcond=None)
    cx, cy = float(x[0]), float(x[1])
    r2 = float(x[2]) + cx**2 + cy**2
    if r2 <= 0:
        raise RuntimeError("원 피팅 실패: r^2 ≤ 0 — 점이 직선에 가깝거나 너무 적습니다")
    radius = float(np.sqrt(r2))

    center = centroid + cx * e1 + cy * e2

    # 5. RMS 잔차
    pts_c = pts - center
    in_plane = pts_c - np.outer(pts_c @ normal, normal)
    residual = float(np.sqrt(np.mean((np.linalg.norm(in_plane, axis=1) - radius) ** 2)))

    return center, normal, radius, residual


# ── T_B_F0 구성 ────────────────────────────────────────────────────────────────

def build_T_B_F0(center_B: np.ndarray, nz_B: np.ndarray) -> np.ndarray:
    """
    B→F 변환 T_B_F0 (4×4) 구성.

    x_F = T_B_F0 @ x_B

    F 프레임 정의
    -------------
    - 원점 : 턴테이블 rim 원의 중심 (center_B)
    - z축  : rim 평면 법선 (위쪽 = B z 방향)
    - x축  : B x축 [1,0,0]을 F 평면에 투영 (theta=0 기준)
    - y축  : z × x
    """
    nz = nz_B / np.linalg.norm(nz_B)
    if nz[2] < 0:   # z축이 아래를 향하면 뒤집음
        nz = -nz

    # x축: B x축을 평면에 투영
    bx = np.array([1.0, 0.0, 0.0])
    nx = bx - np.dot(bx, nz) * nz
    if np.linalg.norm(nx) < 1e-6:
        by = np.array([0.0, 1.0, 0.0])
        nx = by - np.dot(by, nz) * nz
    nx /= np.linalg.norm(nx)
    ny = np.cross(nz, nx)

    # T_F_B: F축을 열(column)로, F 원점을 translation으로 (F→B 변환)
    T_F_B = np.eye(4)
    T_F_B[:3, :3] = np.column_stack([nx, ny, nz])
    T_F_B[:3, 3]  = center_B

    return np.linalg.inv(T_F_B)   # T_B_F0 = inv(T_F_B)


# ── YAML 저장 ──────────────────────────────────────────────────────────────────

def save_yaml(
    path: Path,
    T_B_F0: np.ndarray,
    n_rim_pts: int,
    radius_mm: float,
    residual_mm: float,
) -> None:
    import datetime

    t = T_B_F0[:3, 3]
    q = ScipyR.from_matrix(T_B_F0[:3, :3]).as_quat()  # [qx, qy, qz, qw]

    data = {
        "date":             datetime.date.today().isoformat(),
        "n_rim_points":     n_rim_pts,
        "rim_radius_mm":    round(float(radius_mm), 2),
        "rim_residual_mm":  round(float(residual_mm), 3),
        "T_B_F0": {
            "translation":   [float(v) for v in t],      # meters
            "rotation_quat": [float(v) for v in q],      # [qx, qy, qz, qw]
            "matrix":        [[float(v) for v in row] for row in T_B_F0.tolist()],
        },
    }

    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        yaml.dump(data, default_flow_style=None, allow_unicode=True),
        encoding="utf-8",
    )
    print(f"[TurntableInit] 저장 완료: {path}")


# ── OpenCV 인터랙티브 피커 ──────────────────────────────────────────────────────

def _depth_colormap(organized_pts: np.ndarray) -> np.ndarray:
    """
    조직화된 포인트 클라우드의 Z값(mm)으로 깊이 컬러맵 생성.
    """
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


class _PickerState:
    """
    OpenCV 마우스 콜백 + 렌더링 상태.

    좌표 단위
    ---------
    organized_pts : (H, W, 3) float32, mm, 센서 프레임 S
    T_CB         : (4, 4) float64, C→B, meters
    pts_B         : List[(3,) float64], meters, Base 프레임
    """

    _PT_COLORS = [(0, 255, 0), (0, 200, 255), (255, 120, 0),
                  (180, 0, 255), (255, 255, 0), (0, 180, 255)]

    def __init__(
        self,
        intensity: np.ndarray,       # (H, W) uint8
        organized_pts: np.ndarray,   # (H, W, 3) float32, mm, sensor frame
        T_CB: np.ndarray,           # (4, 4) C→B, meters
    ) -> None:
        self.intensity    = intensity
        self.org_pts      = organized_pts
        self.T_CB        = T_CB
        self._depth_vis   = _depth_colormap(organized_pts)
        self.show_depth   = False
        self.pixels: List[Tuple[int, int]] = []
        self.pts_B:  List[np.ndarray]      = []
        self.status = "rim 위를 클릭하세요"
        self.redraw = True

    # ── pixel → 3D ─────────────────────────────────────────────────

    def _px_to_3d(self, u: int, v: int) -> Optional[np.ndarray]:
        """
        픽셀 (u, v) → Base 프레임 3D 좌표 (meters).
        유효하지 않으면 None 반환.
        """
        H, W = self.org_pts.shape[:2]
        if not (0 <= v < H and 0 <= u < W):
            return None
        pt_S_mm = self.org_pts[v, u].astype(np.float64)
        if not np.isfinite(pt_S_mm).all() or np.all(pt_S_mm == 0):
            return None
        pt_S_m = pt_S_mm / 1000.0
        return self.T_CB[:3, :3] @ pt_S_m + self.T_CB[:3, 3]

    # ── OpenCV 마우스 콜백 ──────────────────────────────────────────

    def on_mouse(self, event: int, x: int, y: int, flags: int, param) -> None:
        if event == cv2.EVENT_LBUTTONDOWN:
            pt_B = self._px_to_3d(x, y)
            if pt_B is None:
                self.status = f"[!] ({x},{y}) 유효한 깊이 없음 — 다른 위치를 클릭하세요"
            else:
                self.pixels.append((x, y))
                self.pts_B.append(pt_B)
                mm = pt_B * 1000.0
                self.status = (
                    f"[{len(self.pts_B)}] px=({x},{y})  "
                    f"B=({mm[0]:.1f}, {mm[1]:.1f}, {mm[2]:.1f}) mm"
                )
            self.redraw = True

        elif event == cv2.EVENT_RBUTTONDOWN:
            if self.pixels:
                self.pixels.pop()
                self.pts_B.pop()
                self.status = f"마지막 점 취소 — 남은 점: {len(self.pts_B)}"
                self.redraw = True

    # ── 렌더링 ─────────────────────────────────────────────────────

    def render(self) -> np.ndarray:
        base = (
            self._depth_vis.copy()
            if self.show_depth
            else cv2.cvtColor(self.intensity, cv2.COLOR_GRAY2BGR)
        )

        # 클릭 점 표시
        for i, (u, v) in enumerate(self.pixels):
            color = self._PT_COLORS[i % len(self._PT_COLORS)]
            cv2.circle(base, (u, v), 7, (255, 255, 255), -1)
            cv2.circle(base, (u, v), 6, color, -1)
            cv2.putText(base, str(i + 1), (u + 9, v - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)

        H, W = base.shape[:2]
        n = len(self.pts_B)

        # 상단 안내 바
        if n < MIN_RIM_PTS:
            hint = f"rim 위를 클릭하세요 ({MIN_RIM_PTS - n}점 더 필요)"
        elif n < WARN_RIM_PTS:
            hint = f"Enter=피팅  (현재 {n}점, {WARN_RIM_PTS}점 이상 권장)"
        else:
            hint = f"Enter=피팅  ({n}점 수집됨)"

        cv2.rectangle(base, (0, 0), (W, 34), (20, 20, 20), -1)
        cv2.putText(base, hint, (8, 23),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 1, cv2.LINE_AA)

        # 하단 상태 바
        cv2.rectangle(base, (0, H - 30), (W, H), (20, 20, 20), -1)
        cv2.putText(base, self.status, (8, H - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (180, 255, 180), 1, cv2.LINE_AA)

        return base


# ── 피커 루프 ──────────────────────────────────────────────────────────────────

def run_picker(state: _PickerState) -> str:
    """
    인터랙티브 피커 실행.

    Returns
    -------
    'fit'       Enter/Space — 충분한 점이 있을 때
    'recapture' r 키
    'quit'      q / Esc
    """
    cv2.namedWindow(_WIN, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(_WIN, 960, 720)
    cv2.setMouseCallback(_WIN, state.on_mouse)

    while True:
        if state.redraw:
            cv2.imshow(_WIN, state.render())
            state.redraw = False

        key = cv2.waitKey(30) & 0xFF

        if key in (13, 32):  # Enter or Space
            if len(state.pts_B) < MIN_RIM_PTS:
                state.status = (
                    f"[!] 최소 {MIN_RIM_PTS}점 필요 "
                    f"(현재 {len(state.pts_B)}점)"
                )
                state.redraw = True
            else:
                cv2.destroyAllWindows()
                return "fit"

        elif key == ord("r"):
            cv2.destroyAllWindows()
            return "recapture"

        elif key == ord("d"):
            state.show_depth = not state.show_depth
            state.redraw = True

        elif key in (27, ord("q")):
            cv2.destroyAllWindows()
            return "quit"


# ── 3D 결과 시각화 (Open3D) ────────────────────────────────────────────────────

def show_3d_result(
    org_pts: np.ndarray,      # (H, W, 3) float32, mm, 센서 프레임 S
    intensity: np.ndarray,    # (H, W) uint8
    T_CB: np.ndarray,        # (4, 4) C→B, meters
    center_B: np.ndarray,     # (3,) meters, 턴테이블 중심
    normal_B: np.ndarray,     # (3,) 단위벡터, 평면 법선
    radius_m: float,
    T_B_F0: np.ndarray,       # (4, 4) B→F
    rim_pts_B: np.ndarray,    # (N, 3) meters, 클릭된 rim 점들
) -> None:
    """
    피팅 결과를 Open3D 3D 뷰로 표시한다.

    표시 항목
    ---------
    - 텍스처 입힌 포인트 클라우드 (B 프레임)
    - 피팅된 원 (주황색 선)
    - 클릭된 rim 점들 (초록 구)
    - 턴테이블 중심 (빨간 구)
    - F 프레임 좌표축 (R=x, G=y, B=z)
    - B 프레임 원점 좌표축 (참조용, 작게)
    """
    try:
        import open3d as o3d
    except ImportError:
        print("[3D View] open3d가 없습니다 — pip install open3d")
        return

    # ── 1. 텍스처 포인트 클라우드 (B 프레임) ────────────────────────────────
    H, W = org_pts.shape[:2]
    flat_pts  = org_pts.reshape(-1, 3).astype(np.float64)         # (H*W, 3) mm
    flat_gray = intensity.reshape(-1).astype(np.float64) / 255.0  # (H*W,) [0,1]

    valid = ~np.all(flat_pts == 0, axis=1) & np.isfinite(flat_pts).all(axis=1)
    pts_S_m = flat_pts[valid] / 1000.0
    grays   = flat_gray[valid]

    # Sensor → Base 변환 (벡터화)
    pts_B_all = (T_CB[:3, :3] @ pts_S_m.T).T + T_CB[:3, 3]

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pts_B_all)
    pcd.colors = o3d.utility.Vector3dVector(np.column_stack([grays, grays, grays]))

    # 포인트가 너무 많으면 다운샘플 (30만 이상 → voxel 2mm)
    if len(pts_B_all) > 300_000:
        pcd = pcd.voxel_down_sample(voxel_size=0.002)

    # ── 2. 피팅된 원 (주황색 LineSet) ───────────────────────────────────────
    n_seg = 120
    angles = np.linspace(0, 2 * np.pi, n_seg, endpoint=False)
    nz  = normal_B / np.linalg.norm(normal_B)
    ref = np.array([1.0, 0.0, 0.0]) if abs(nz[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
    e1  = ref - np.dot(ref, nz) * nz;  e1 /= np.linalg.norm(e1)
    e2  = np.cross(nz, e1)

    circle_pts = center_B + radius_m * (
        np.outer(np.cos(angles), e1) + np.outer(np.sin(angles), e2)
    )
    circle_ls = o3d.geometry.LineSet(
        points=o3d.utility.Vector3dVector(circle_pts),
        lines =o3d.utility.Vector2iVector([[i, (i + 1) % n_seg] for i in range(n_seg)]),
    )
    circle_ls.paint_uniform_color([1.0, 0.5, 0.0])  # 주황

    # ── 3. 클릭된 rim 점 (초록 구, 5mm) ─────────────────────────────────────
    rim_spheres = []
    for pt in rim_pts_B:
        s = o3d.geometry.TriangleMesh.create_sphere(radius=0.005)
        s.translate(pt)
        s.paint_uniform_color([0.1, 1.0, 0.2])
        rim_spheres.append(s)

    # ── 4. 턴테이블 중심 (빨간 구, 8mm) ─────────────────────────────────────
    center_sph = o3d.geometry.TriangleMesh.create_sphere(radius=0.008)
    center_sph.translate(center_B)
    center_sph.paint_uniform_color([1.0, 0.1, 0.1])

    # ── 5. F 프레임 좌표축 ───────────────────────────────────────────────────
    # T_F_B (F→B): F 프레임을 B 공간에 배치하는 변환
    T_F_B = np.linalg.inv(T_B_F0)
    frame_size = max(0.08, radius_m * 0.6)   # 턴테이블 크기에 비례
    f_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=frame_size)
    f_frame.transform(T_F_B)

    # ── 6. B 프레임 원점 좌표축 (참조용, 50mm) ───────────────────────────────
    b_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.05)

    # ── 표시 ─────────────────────────────────────────────────────────────────
    print("[3D View] 창을 닫으면 저장 여부를 선택합니다.")
    print("          마우스 왼쪽 드래그=회전  오른쪽=이동  휠=줌")
    o3d.visualization.draw_geometries(
        [pcd, circle_ls, center_sph, f_frame, b_frame] + rim_spheres,
        window_name="Turntable Frame Init — 3D 결과 확인  (F축: R=x G=y B=z)",
        width=1280,
        height=800,
    )


# ── Main ───────────────────────────────────────────────────────────────────────

def main() -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )

    print("=" * 60)
    print("  Turntable Frame Init  —  ^B T_F(0)  rim fitting")
    print("=" * 60)
    print(f"  hand-eye YAML : {HAND_EYE_YAML}")
    print(f"  출력 YAML     : {OUTPUT_YAML}")
    print()
    print("사전 확인:")
    print("  1. 턴테이블 각도 = 0  (theta = 0)")
    print("  2. 로봇이 rim 전체가 잘 보이는 위치에 있어야 합니다")
    input("\nEnter 누르면 시작... ")

    # ── hand-eye T_E_C 로드 ────────────────────────────────────────────────────
    if not HAND_EYE_YAML.exists():
        raise FileNotFoundError(f"hand-eye YAML 없음: {HAND_EYE_YAML}")
    T_E_C = load_transform(str(HAND_EYE_YAML), "T_E_C")   # E→C, meters
    T_CE = np.linalg.inv(T_E_C)                           # C→E, meters
    t_mm = T_E_C[:3, 3] * 1000.0
    print(f"\n[TurntableInit] T_E_C 로드  t=({t_mm[0]:.1f},{t_mm[1]:.1f},{t_mm[2]:.1f}) mm")

    # ── 로봇 + 센서 초기화 ─────────────────────────────────────────────────────
    robot  = XArmInterface(ip=ROBOT_IP)
    sensor = PhoxiClient(PhoxiConfig(serial_number="SEA-023", trigger_timeout_s=15.0))
    sensor.initialize()

    try:
        while True:
            # ── T_E_B 취득 (로봇 FK) ─────────────────────────────────────────
            T_E_B = robot.get_ee_pose_mat()          # E→B, meters
            # T_CB: C→B  (x_B = T_CB @ x_C)
            # 체인: C→E→B = T_EB @ T_CE
            T_CB = T_E_B @ T_CE
            ee_mm = T_E_B[:3, 3] * 1000.0
            print(f"\n[TurntableInit] 현재 EE  t=({ee_mm[0]:.1f},{ee_mm[1]:.1f},{ee_mm[2]:.1f}) mm")

            # ── 캡처 ─────────────────────────────────────────────────────────
            print("[TurntableInit] PhoXi 캡처 중...")
            sensor.capture()   # _last_organized_pts, _last_intensity 갱신용

            if sensor._last_organized_pts is None or sensor._last_intensity is None:
                print("[!] 조직화된 포인트 클라우드를 얻지 못했습니다")
                input("Enter 누르면 재시도...")
                continue

            org_pts   = sensor._last_organized_pts   # (H, W, 3) float32, mm
            intensity = sensor._last_intensity        # (H, W) uint8
            print(f"[TurntableInit] 캡처 완료  {org_pts.shape[1]}×{org_pts.shape[0]} px")

            # ── 인터랙티브 피킹 ───────────────────────────────────────────────
            print("\n[조작법]")
            print("  좌클릭       : rim 점 추가")
            print("  우클릭       : 마지막 점 취소")
            print("  Enter/Space  : 원 피팅 실행")
            print("  r            : 재캡처")
            print("  d            : 강도 ↔ 깊이 뷰 토글")
            print("  q / Esc      : 종료")

            state  = _PickerState(intensity, org_pts, T_CB)
            action = run_picker(state)

            if action == "quit":
                print("[TurntableInit] 종료.")
                return

            if action == "recapture":
                print("[TurntableInit] 재캡처...")
                continue

            # action == "fit"
            pts_B = np.array(state.pts_B, dtype=np.float64)   # (N,3) meters

            # ── 원 피팅 ───────────────────────────────────────────────────────
            print(f"\n[TurntableInit] 3D 원 피팅 ({len(pts_B)}점)...")
            try:
                center_B, normal_B, radius_m, residual_m = fit_circle_3d(pts_B)
            except Exception as e:
                print(f"[!] 피팅 실패: {e}")
                print("    점 위치를 다시 선택하세요.")
                input("Enter 누르면 계속...")
                continue

            radius_mm   = radius_m   * 1000.0
            residual_mm = residual_m * 1000.0
            c_mm        = center_B   * 1000.0

            print(f"\n  ─── 원 피팅 결과 ───────────────────────────────")
            print(f"  중심  (B frame) : ({c_mm[0]:.2f}, {c_mm[1]:.2f}, {c_mm[2]:.2f}) mm")
            print(f"  반지름          : {radius_mm:.2f} mm")
            print(f"  법선  (B frame) : ({normal_B[0]:.4f}, {normal_B[1]:.4f}, {normal_B[2]:.4f})")
            print(f"  RMS 잔차        : {residual_mm:.3f} mm", end="")
            if residual_mm > WARN_RESIDUAL:
                print(f"  ← [경고] {WARN_RESIDUAL}mm 초과")
            else:
                print()

            # ── T_B_F0 구성 ──────────────────────────────────────────────────
            T_B_F0 = build_T_B_F0(center_B, normal_B)
            T_F_B  = np.linalg.inv(T_B_F0)

            # 검증: F 원점 in B, z축 tilt
            origin_B_mm  = T_F_B[:3, 3] * 1000.0
            z_axis_in_B  = T_F_B[:3, 2]
            tilt_deg     = float(np.degrees(np.arccos(np.clip(abs(z_axis_in_B[2]), 0.0, 1.0))))
            rpy_deg      = ScipyR.from_matrix(T_B_F0[:3, :3]).as_euler("xyz", degrees=True)

            print(f"\n  ─── T_B_F0 ─────────────────────────────────────")
            print(f"  F 원점 (B 기준) : ({origin_B_mm[0]:.2f}, {origin_B_mm[1]:.2f}, {origin_B_mm[2]:.2f}) mm")
            print(f"  z축 틸트        : {tilt_deg:.2f}°  (0° = 수직)")
            print(f"  RPY (B→F, deg)  : roll={rpy_deg[0]:.2f}  pitch={rpy_deg[1]:.2f}  yaw={rpy_deg[2]:.2f}")

            if tilt_deg > 5.0:
                print(f"  [경고] z축 틸트 {tilt_deg:.1f}° — 턴테이블이 수평인지 확인하세요")

            # ── 3D 결과 시각화 ────────────────────────────────────────────────
            print("\n[TurntableInit] 3D 뷰 표시 중 (창 닫으면 저장 여부 선택)...")
            try:
                show_3d_result(
                    org_pts, intensity, T_CB,
                    center_B, normal_B, radius_m, T_B_F0,
                    pts_B,
                )
            except ImportError:
                print("[3D View] open3d 없음 — 건너뜁니다.")
            except Exception as e:
                print(f"[3D View] 오류: {e}")

            # ── 저장 확인 ─────────────────────────────────────────────────────
            print()
            ans = input("저장할까요? [y=저장 / r=재선택 / n=종료]: ").strip().lower()

            if ans == "y":
                save_yaml(OUTPUT_YAML, T_B_F0, len(pts_B), radius_mm, residual_mm)
                print(f"\n[TurntableInit] 완료. {OUTPUT_YAML}")
                return

            elif ans == "r":
                print("[TurntableInit] 점 재선택...")
                continue

            else:
                print("[TurntableInit] 저장하지 않고 종료.")
                return

    finally:
        sensor.shutdown()
        robot.disconnect()


if __name__ == "__main__":
    main()
