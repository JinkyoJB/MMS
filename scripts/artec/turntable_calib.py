#!/usr/bin/env python3
"""
scripts/artec/turntable_calib.py

T_B_F0 캘리브레이션 — Artec 버전.
턴테이블 rim 점 클릭 → UV→3D 매핑 → B 프레임 변환 → 3D 원 피팅 → T_B_F0 저장.

PhoXi 버전과 차이:
  - Artec 은 organized depth 가 없음. mesh vertices + UV + texture image 사용.
  - texture pixel 클릭 → UV 가 가장 가까운 vertex 의 3D 좌표 (C 프레임, mm)
  - z-back convention 보정 (`ARTEC_TO_OPENCV` z-flip) 후 T_CB 적용

사전 조건
---------
- hand-eye 완료 + `config/sensor_frames.yaml` 의 `T_EC_artec` 갱신
- 턴테이블 θ=0
- 로봇이 turntable rim 이 잘 보이는 위치 (Spider 작동거리 ~250mm)

출력: `config/calibration/turntable_frame.yaml` (T_B_F0)
"""

from __future__ import annotations

import sys
from pathlib import Path
from typing import List, Optional, Tuple

import cv2
import numpy as np
import yaml

_PROJECT_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(_PROJECT_ROOT))

from utils.robot.xarm_interface import XArmInterface
from utils.transforms import load_transform, compute_T_CB
# ★ 원 피팅·T_B_F0 구성·저장은 공유 코어를 쓴다 (PhoXi 판과 동일).
#   과거 이 파일이 fit_circle_3d / T_B_F0 구성 / yaml 저장을 자체 구현해
#   공유 코어와 미묘하게 갈라져 있었다 → 2026-09 통일.
from utils.calibration.turntable_frame import (
    fit_circle_3d, build_T_B_F0, save_turntable_frame_yaml)
from mms_artec.sensor.artec_client import ArtecClient, ArtecConfig

# ── CONFIG ─────────────────────────────────────────────────────────────

ROBOT_IP        = "192.168.1.210"
SENSOR_FRAMES   = _PROJECT_ROOT / "config" / "sensor_frames.yaml"
OUTPUT_YAML     = _PROJECT_ROOT / "config" / "calibration" / "turntable_frame.yaml"
MIN_RIM_PTS     = 3
WARN_RIM_PTS    = 6
WARN_RESIDUAL   = 5.0       # mm

# Artec C frame 은 z-back. z-flip 으로 OpenCV 호환으로 보정.
ARTEC_TO_OPENCV = np.diag([1.0, 1.0, -1.0, 1.0])

_WIN = (
    "Turntable Rim Picker (Artec)"
    "  [LClick=add  RClick=undo  Enter=fit  r=recapture  q=quit]"
)


# ── 3D circle fit (PhoXi 버전과 공용 알고리즘) ──────────────────────────

# ── pixel → 3D (UV nearest vertex) ─────────────────────────────────────

def pixel_to_3d_C(
    pixel: Tuple[float, float],
    vertices_mm: np.ndarray,           # (N, 3) C 프레임 (z-back)
    uv: np.ndarray,                    # (N, 2) [0..1]
    image_wh: Tuple[int, int],
    max_dist_px: float = 8.0,
) -> Optional[np.ndarray]:
    """texture pixel 의 UV-nearest vertex 의 3D 좌표 반환 (mm, OpenCV-호환 C frame)."""
    W, H = image_wh
    pix_uv = uv.astype(np.float64).copy()
    pix_uv[:, 0] *= float(W)
    pix_uv[:, 1] *= float(H)
    u, v = float(pixel[0]), float(pixel[1])
    d2 = (pix_uv[:, 0] - u) ** 2 + (pix_uv[:, 1] - v) ** 2
    j = int(np.argmin(d2))
    if d2[j] > max_dist_px * max_dist_px:
        return None
    p_artec = vertices_mm[j].astype(np.float64)
    # z-flip → OpenCV C
    return p_artec * np.array([1.0, 1.0, -1.0])


# ── interactive picker ────────────────────────────────────────────────

class _RimPicker:
    """Artec 전용 rim 클릭 UI.

    ⚠ 공유 코어 `utils/calibration/rim_picker.RimPicker` 를 못 쓰는 이유 —
      공유 판은 **정렬 점군**(organized_pts, H×W×3)과 `T_CB` 를 받는데, Artec 은
      메시(vertices + uv)를 준다. 픽셀→3D 매핑 방식 자체가 다르다.
      Artec 캡처를 organized_pts 로 변환하는 어댑터를 만들면 공유 판으로 통일 가능.
      (원 피팅·T_B_F0 구성·yaml 저장은 이미 공유 코어를 쓴다)
    """
    def __init__(self, image: np.ndarray, vertices: np.ndarray, uv: np.ndarray):
        self.image = image     # (H, W, 3) RGB
        self.vertices = vertices
        self.uv = uv
        self.points_C: List[np.ndarray] = []     # OpenCV C-frame mm
        self.pixels: List[Tuple[float, float]] = []
        self.W = image.shape[1]; self.H = image.shape[0]

    def add(self, x: int, y: int) -> bool:
        p = pixel_to_3d_C((x, y), self.vertices, self.uv, (self.W, self.H))
        if p is None:
            print(f"  [pick] ({x},{y}) → UV 매칭 실패")
            return False
        self.points_C.append(p)
        self.pixels.append((float(x), float(y)))
        print(f"  [pick] #{len(self.points_C)} ({x},{y}) → ({p[0]:.0f},{p[1]:.0f},{p[2]:.0f}) mm")
        return True

    def undo(self) -> None:
        if self.points_C:
            self.points_C.pop()
            self.pixels.pop()
            print(f"  [pick] undo (남음 {len(self.points_C)})")

    def render(self) -> np.ndarray:
        vis = cv2.cvtColor(self.image, cv2.COLOR_RGB2BGR).copy()
        for i, (u, v) in enumerate(self.pixels):
            cv2.circle(vis, (int(u), int(v)), 6, (0, 255, 0), 2)
            cv2.putText(vis, str(i + 1), (int(u) + 8, int(v) - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.putText(
            vis,
            f"rim pts: {len(self.pixels)}  "
            f"[LClick=add RClick=undo Enter=fit r=recapture q=quit]",
            (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 0), 1,
        )
        return vis


def _capture_frame(client: ArtecClient):
    fmh = client.capture_frame(capture_texture=True)
    if fmh is None or fmh.vertex_count() == 0:
        print("[artec] capture 실패")
        return None
    if not fmh.has_image():
        print("[artec] texture image 없음 — capture_texture 확인")
        return None
    return fmh


# ── main ──────────────────────────────────────────────────────────────

def main():
    if not SENSOR_FRAMES.exists():
        print(f"⚠ {SENSOR_FRAMES} 없음. hand-eye 먼저 완료 후 T_EC_artec 갱신 필요.")
        return 1
    T_EC = load_transform(str(SENSOR_FRAMES), "T_EC_artec")
    print(f"[artec] T_EC_artec loaded  t={T_EC[:3,3]} m")

    robot = XArmInterface(ROBOT_IP)
    cfg = ArtecConfig(serial_number=None, capture_texture=True)
    client = ArtecClient(cfg)
    client.initialize()

    try:
        while True:
            fmh = _capture_frame(client)
            if fmh is None:
                print("재시도? (Enter / q): ", end="")
                if input().strip().lower() == "q":
                    return
                continue

            image = fmh.image()                 # (H, W, 3) RGB
            verts = fmh.vertices()              # (N, 3) mm, z-back
            uv    = fmh.uv()                    # (N, 2) [0..1]
            print(f"[artec] frame  verts={len(verts):,}  image={image.shape[1]}×{image.shape[0]}")

            picker = _RimPicker(image, verts, uv)

            def on_mouse(event, x, y, flags, _):
                if event == cv2.EVENT_LBUTTONDOWN:
                    picker.add(x, y)
                    cv2.imshow(_WIN, picker.render())
                elif event == cv2.EVENT_RBUTTONDOWN:
                    picker.undo()
                    cv2.imshow(_WIN, picker.render())

            cv2.namedWindow(_WIN, cv2.WINDOW_NORMAL)
            cv2.setMouseCallback(_WIN, on_mouse)
            cv2.imshow(_WIN, picker.render())

            action = None
            while True:
                key = cv2.waitKey(20) & 0xFF
                if key in (13, 32):     # Enter / Space
                    action = "fit"; break
                if key == ord('r'):
                    action = "recapture"; break
                if key in (ord('q'), 27):
                    action = "quit"; break
            cv2.destroyWindow(_WIN)

            if action == "quit":
                return
            if action == "recapture":
                continue
            if action != "fit":
                continue
            if len(picker.points_C) < MIN_RIM_PTS:
                print(f"⚠ rim 점 부족 ({len(picker.points_C)} < {MIN_RIM_PTS})")
                continue
            if len(picker.points_C) < WARN_RIM_PTS:
                print(f"⚠ rim 점 적음 ({len(picker.points_C)} < {WARN_RIM_PTS}) — 정확도 ↓ 가능")

            # C → B 변환 (mm 유지)
            T_EB = robot.get_ee_pose_mat()
            T_CB = compute_T_CB(T_EB, T_EC)             # m 단위
            R_CB = T_CB[:3, :3]; t_CB_mm = T_CB[:3, 3] * 1000.0
            pts_C = np.array(picker.points_C)           # (N, 3) mm
            pts_B = pts_C @ R_CB.T + t_CB_mm            # mm

            center_mm, normal, radius_mm, rms_mm = fit_circle_3d(pts_B)
            print(f"\n[fit] center_B (mm) = {center_mm}")
            print(f"      normal         = {normal}")
            print(f"      radius_mm      = {radius_mm:.2f}")
            print(f"      rms residual   = {rms_mm:.3f} mm")
            if rms_mm > WARN_RESIDUAL:
                print(f"⚠ residual 큼 ({rms_mm:.1f} > {WARN_RESIDUAL}mm) — 점 다시 찍기 권장")

            # 공유 코어로 T_B_F0 구성 (축 퇴화 처리 포함). center 는 m 로 넘긴다.
            T_BF0 = build_T_B_F0(center_mm / 1000.0, normal)

            print(f"\n[T_B_F0]\n{T_BF0}")
            ans = input("\n저장할까요? (y/N): ").strip().lower()
            if ans == "y":
                save_turntable_frame_yaml(
                    OUTPUT_YAML, T_BF0, n_points=len(pts_B),
                    radius_mm=radius_mm, residual_mm=rms_mm,
                    extra={"method": "artec_uv_3d_circle_fit"})
                print(f"[artec] saved → {OUTPUT_YAML}")
                return
            else:
                ans2 = input("재시도? (Enter / q): ").strip().lower()
                if ans2 == "q":
                    return

    finally:
        client.shutdown()
        robot.disconnect()


if __name__ == "__main__":
    # ★ 실패를 exit code 로 알린다 (intrinsic_calib.py 주석 참고).
    sys.exit(main() or 0)
