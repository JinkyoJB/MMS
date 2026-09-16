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
import time
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
#: rim 이 실제로 스캔되는 시작 자세. home 은 이 셀에서 테두리가 잘 안 잡힌다.
RIM_POSE_YAML   = _PROJECT_ROOT / "config" / "calibration" / "artec_rim_pose.yaml"
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

def solve_scanner_to_color(vertices_mm, uv, image_wh, K, dist,
                           n_sample: int = 6000, seed: int = 0):
    """**스캐너 3D 프레임 → Color 카메라 프레임** 강체변환을 프레임 데이터에서 직접 푼다.

    ★ 왜 필요한가 — Spider 는 3D 카메라 3대와 Color 카메라 1대가 **따로** 있다.
      `vertices()` 는 3D 카메라들이 만든 프레임이고, `image()`·K·solvePnP 는 Color
      카메라 프레임이다. 둘 사이엔 고정 강체변환이 있다 (2026-09-16 실측,
      SP.10.79103441: 회전 179.94°, 이동 41.8mm, 재투영 0.66px).

      그런데 `T_EC` 는 hand-eye 가 **solvePnP**(2D 픽셀만)로 풀어서 **Color 프레임**
      기준이다. 여기에 스캐너 프레임 점을 그대로 먹이면 180° 회전 + 41.8mm 가
      통째로 오차로 실린다 — rim 원이 실제 테두리에서 23~43mm 어긋난 원인이었다.

      옛 코드는 이걸 `p * [1,1,-1]` 로 때웠다. `diag(1,1,-1)` 은 행렬식 **−1**,
      즉 회전이 아니라 **거울 반사**다. 손잡이를 뒤집고 이동은 아예 빠진다.

    하드코딩하지 않고 매 프레임 푸는 이유: 이 값은 개체 종속이라 스캐너를 바꾸면
    달라진다(`T_EC` 와 같은 성질). 필요한 데이터가 이미 프레임 안에 있으므로
    추가 캘리브가 필요 없다.

    Returns: (4,4) T — x_color = T · x_scanner  (mm). 실패하면 None.
    """
    W, H = image_wh
    V = np.asarray(vertices_mm, float)
    px = np.asarray(uv, float).copy()
    px[:, 0] *= float(W)
    px[:, 1] *= float(H)
    if len(V) < 200:
        return None
    idx = np.random.default_rng(seed).choice(
        len(V), size=min(n_sample, len(V)), replace=False)
    ok, rv, tv, inl = cv2.solvePnPRansac(
        V[idx], px[idx], K, dist, flags=cv2.SOLVEPNP_ITERATIVE,
        reprojectionError=3.0, iterationsCount=300)
    if not ok or inl is None or len(inl) < 100:
        return None
    sel = inl.ravel()
    rv, tv = cv2.solvePnPRefineLM(V[idx][sel], px[idx][sel], K, dist, rv, tv)
    R, _ = cv2.Rodrigues(rv)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = tv.ravel()
    proj = cv2.projectPoints(V[idx][sel], rv, tv, K, dist)[0].reshape(-1, 2)
    err = float(np.median(np.linalg.norm(proj - px[idx][sel], axis=1)))
    ang = float(np.degrees(np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1))))
    print(f"  [frame] 스캐너3D→Color  회전 {ang:.2f}°  이동 "
          f"{np.round(tv.ravel(),1).tolist()}mm  재투영 {err:.2f}px  "
          f"inlier {len(sel)}/{len(idx)}")
    if err > 3.0:
        print(f"    ⚠ 재투영 오차가 크다 ({err:.1f}px) — 결과를 믿지 말 것")
    return T


def uv_match_tolerance(n_vertices: int, image_wh: Tuple[int, int]) -> float:
    """UV 매칭 허용 반경(px)을 **정점 밀도에서** 정한다.

    ★ 예전엔 8px 고정이었다. 그런데 정점이 이미지 위에 퍼진 평균 간격은
      sqrt(W·H/N) 이고, 960×1280 에 정점 1만 개면 **11px** 라 8px 문턱을 원리적으로
      못 넘는다 — 커버리지가 멀쩡한 자리를 찍어도 "UV 매칭 실패" 가 났다.
      같은 리포의 ChArUco 검출기(`artec_charuco_detector._uv_to_3d`)는 이미
      이 적응형 규칙을 쓰고 있었고, rim picker 만 고정값으로 남아 있었다.
    """
    W, H = image_wh
    spacing = float(np.sqrt(W * H / max(int(n_vertices), 1)))
    return float(np.clip(spacing * 3.0, 8.0, 40.0))


def pixel_to_3d_C(
    pixel: Tuple[float, float],
    vertices_mm: np.ndarray,           # (N, 3) C 프레임 (z-back)
    uv: np.ndarray,                    # (N, 2) [0..1]
    image_wh: Tuple[int, int],
    max_dist_px: Optional[float] = None,
    pix_uv: Optional[np.ndarray] = None,
    T_scan_color: Optional[np.ndarray] = None,
) -> Optional[np.ndarray]:
    """texture pixel 의 UV-nearest vertex 의 3D 좌표 반환 (mm, **Color 카메라 프레임**).

    `max_dist_px=None` 이면 정점 밀도에 맞춰 자동으로 정한다.
    `pix_uv` 를 넘기면 UV→픽셀 변환을 다시 하지 않는다(클릭마다 재계산 방지).

    ★ `T_scan_color` 는 `solve_scanner_to_color()` 결과다. **반드시 넘겨야 한다** —
      이게 없으면 옛 동작(z-flip = 거울 반사)으로 떨어지고, `T_EC` 와 프레임이
      어긋나 결과가 통째로 밀린다. 자세한 이유는 그 함수 docstring 참고.
    """
    W, H = image_wh
    if pix_uv is None:
        pix_uv = uv.astype(np.float64).copy()
        pix_uv[:, 0] *= float(W)
        pix_uv[:, 1] *= float(H)
    if max_dist_px is None:
        max_dist_px = uv_match_tolerance(len(vertices_mm), (W, H))
    u, v = float(pixel[0]), float(pixel[1])
    d2 = (pix_uv[:, 0] - u) ** 2 + (pix_uv[:, 1] - v) ** 2
    j = int(np.argmin(d2))
    if d2[j] > max_dist_px * max_dist_px:
        return None
    p_scan = vertices_mm[j].astype(np.float64)
    if T_scan_color is not None:
        return T_scan_color[:3, :3] @ p_scan + T_scan_color[:3, 3]
    # 폴백 — 옛 동작. diag(1,1,-1) 은 회전이 아니라 **거울 반사**(det=−1)이고
    # 41.8mm 이동도 빠진다. T_scan_color 를 못 구했을 때만 쓰이며, 그 경우
    # 결과를 믿으면 안 된다(호출부가 경고를 띄운다).
    return p_scan * np.array([1.0, 1.0, -1.0])


# ── interactive picker ────────────────────────────────────────────────

class _RimPicker:
    """Artec 전용 rim 클릭 UI.

    ⚠ 공유 코어 `utils/calibration/rim_picker.RimPicker` 를 못 쓰는 이유 —
      공유 판은 **정렬 점군**(organized_pts, H×W×3)과 `T_CB` 를 받는데, Artec 은
      메시(vertices + uv)를 준다. 픽셀→3D 매핑 방식 자체가 다르다.
      Artec 캡처를 organized_pts 로 변환하는 어댑터를 만들면 공유 판으로 통일 가능.
      (원 피팅·T_B_F0 구성·yaml 저장은 이미 공유 코어를 쓴다)
    """
    def __init__(self, image: np.ndarray, vertices: np.ndarray, uv: np.ndarray,
                 T_scan_color: Optional[np.ndarray] = None):
        self.image = image     # (H, W, 3) RGB
        self.vertices = vertices
        self.uv = uv
        #: 스캐너3D → Color 프레임. None 이면 옛 z-flip 폴백 (틀린 결과가 나온다).
        self.T_scan_color = T_scan_color
        self.points_C: List[np.ndarray] = []     # OpenCV C-frame mm
        self.pixels: List[Tuple[float, float]] = []
        self.W = image.shape[1]; self.H = image.shape[0]

        # UV→픽셀은 한 번만 계산한다 (클릭마다 N개 재계산하지 않도록).
        self.pix_uv = uv.astype(np.float64).copy()
        self.pix_uv[:, 0] *= float(self.W)
        self.pix_uv[:, 1] *= float(self.H)
        self.tol_px = uv_match_tolerance(len(vertices), (self.W, self.H))

        # ★ **복원된 3D 만** 보여준다 (기본). 텍스처 사진은 메시보다 넓게 찍혀서,
        #   사진에 테두리가 보여도 그 자리에 정점이 없으면 클릭이 무효다 —
        #   둘을 겹쳐 보여주니 오히려 헷갈렸다. 그래서 정점을 **깊이로 색칠해**
        #   그리고, 클릭 가능한 자리만 화면에 존재하게 만든다.
        #   원판 테두리는 깊이 불연속이라 이 화면에서 색 경계로 드러난다.
        self.view_3d = True
        px = np.round(self.pix_uv).astype(int)
        ok = ((px[:, 0] >= 0) & (px[:, 0] < self.W) &
              (px[:, 1] >= 0) & (px[:, 1] < self.H))

        z = np.abs(vertices[:, 2].astype(np.float64))     # 카메라 거리(mm)
        lo, hi = np.percentile(z[ok], [2, 98]) if ok.any() else (0.0, 1.0)
        t = np.clip((z - lo) / max(hi - lo, 1e-6), 0, 1)
        col = cv2.applyColorMap((t * 255).astype(np.uint8), cv2.COLORMAP_TURBO
                                ).reshape(-1, 3)          # 가까움=파랑 … 멂=빨강

        canvas = np.zeros((self.H, self.W, 3), np.uint8)
        canvas[px[ok, 1], px[ok, 0]] = col[ok]
        mask = np.zeros((self.H, self.W), np.uint8)
        mask[px[ok, 1], px[ok, 0]] = 255
        # 정점 간격만큼 부풀려 점들이 면으로 보이게 (클릭 허용 반경과 같은 크기)
        k = max(3, int(round(self.tol_px)) | 1)
        el = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))
        self.render3d = cv2.dilate(canvas, el)
        self.cov = cv2.dilate(mask, el)
        self.z_range = (float(lo), float(hi))
        print(f"  [pick] 정점 {len(vertices):,}개 · UV 허용 {self.tol_px:.0f}px · "
              f"클릭 가능 영역 {100.0*float((self.cov>0).mean()):.0f}% · "
              f"거리 {lo:.0f}~{hi:.0f}mm")

    def add(self, x: int, y: int) -> bool:
        p = pixel_to_3d_C((x, y), self.vertices, self.uv, (self.W, self.H),
                          max_dist_px=self.tol_px, pix_uv=self.pix_uv,
                          T_scan_color=self.T_scan_color)
        if p is None:
            d = float(np.sqrt(((self.pix_uv - [x, y]) ** 2).sum(1).min()))
            print(f"  [pick] ({x},{y}) → 3D 없음 (가장 가까운 정점 {d:.0f}px, "
                  f"허용 {self.tol_px:.0f}px) — 초록 영역 안을 찍을 것")
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
        # 기본은 3D 만. 텍스처가 필요하면 v 로 전환한다.
        vis = (self.render3d.copy() if self.view_3d
               else cv2.cvtColor(self.image, cv2.COLOR_RGB2BGR).copy())
        for i, (u, v) in enumerate(self.pixels):
            cv2.circle(vis, (int(u), int(v)), 6, (0, 255, 0), 2)
            cv2.putText(vis, str(i + 1), (int(u) + 8, int(v) - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        if self.view_3d:
            head = (f"3D MESH  (depth {self.z_range[0]:.0f}-{self.z_range[1]:.0f}mm: "
                    f"blue=near red=far)   click the RIM EDGE")
        else:
            head = "TEXTURE photo (no 3D here)  -  press v for 3D"
        cv2.putText(vis, f"rim pts: {len(self.pixels)}   {head}   v=toggle",
                    (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 1)
        cv2.putText(
            vis, "LClick=add  RClick=undo  Enter=fit  m=manual  r=recapture  q=quit",
            (10, 48), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1,
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

#: 조그 키 → base 프레임 (dx, dy, dz) 단위벡터.
#  rim 은 원판 전체가 화면에 **절대** 안 들어온다 — 원판 지름 ~238mm 인데
#  Spider 화각은 320mm 에서 123×167mm 다. 전체를 담으려면 600mm 넘게 물러나야
#  하는데 작동거리(170~350mm) 밖이다. 그래서 목표는 '전체'가 아니라
#  **가능한 긴 호**이고, 그걸 찾으려면 창을 보면서 조금씩 움직여야 한다.
#  예전엔 인자를 바꿔 스크립트를 **처음부터 다시** 돌려야 했다(홈 복귀 + 스캐너
#  재초기화로 매번 수십 초). 이제 창 안에서 바로 움직이고 다시 캡처한다.
_JOG_KEYS = {
    ord('w'): (+1, 0, 0), ord('s'): (-1, 0, 0),     # base +x / -x
    ord('a'): (0, +1, 0), ord('d'): (0, -1, 0),     # base +y / -y
    ord('e'): (0, 0, +1), ord('c'): (0, 0, -1),     # base +z / -z
}

#: 관절 조그 키 → (관절 인덱스 0-base, 부호).
#
#  ★ 평행이동만으로는 부족하다. 원판 전체가 화각에 안 들어오는 이상 목표는
#    **화면을 가로지르는 긴 호**인데, 그건 카메라를 광축 둘레로 돌려야 만들어진다.
#    J7(마지막 관절)이 바로 그 회전이라, 위치는 거의 그대로 두고 호의 방향만
#    바꿀 수 있다. J6 은 카메라를 기울여 원판의 다른 쪽을 보게 한다.
_JOG_JOINT_KEYS = {
    ord('z'): (6, -1), ord('x'): (6, +1),           # J7 (roll) — 호의 방향
    ord('t'): (5, -1), ord('g'): (5, +1),           # J6 (tilt) — 보는 지점
}


def _print_keys(step_mm: float, step_deg: float) -> None:
    print(f"\n  [키]  m = 수동 조절 — **웹 UI 로 팔을 끌면서** 라이브 화면 보기 (제일 빠르다)")
    print(f"        좌클릭=rim 점 추가  우클릭=취소  Enter/Space=피팅  r=재캡처  q=종료")
    print(f"        v=3D/텍스처 전환   S=지금 자세를 rim 시작자세로 저장   h=home 복귀")
    print(f"     ── 키로 미세하게 맞추고 싶을 때 ──")
    print(f"        이동({step_mm:.0f}mm): w/s=±x  a/d=±y  e/c=±z     [ ]=스텝 절반/두배")
    print(f"        회전({step_deg:.0f}°) : o/p=광축 roll∓(화면만 회전)  "
          f"z/x=J7∓  t/g=J6∓   , .=스텝 절반/두배")


def _save_rim_pose(robot) -> bool:
    """지금 자세를 rim 시작 자세로 기록한다.

    ★ `joints` 를 저장하고 재생한다. `ee_pose` 만 저장하면 재생 시 컨트롤러 IK 로
      다시 풀어야 하는데, 7축이라 같은 TCP 에 해가 무한히 많아 **전혀 다른 자세**가
      나올 수 있다 (`gen_calib_poses.py` 가 joints 를 쓰는 이유와 같다).
    """
    import datetime
    try:
        q = [round(float(v), 4) for v in np.degrees(robot.get_joint_angles(is_radian=True))]
        p = robot.get_pose(is_radian=True)
        ee = [round(float(p[0]), 2), round(float(p[1]), 2), round(float(p[2]), 2),
              *[round(float(np.degrees(v)), 3) for v in p[3:6]]]
    except Exception as e:                                      # noqa: BLE001
        print(f"  ✘ 자세 읽기 실패: {e}")
        return False
    if RIM_POSE_YAML.exists():
        bak = RIM_POSE_YAML.with_suffix(".yaml.bak")
        bak.write_text(RIM_POSE_YAML.read_text(encoding="utf-8"), encoding="utf-8")
    RIM_POSE_YAML.parent.mkdir(parents=True, exist_ok=True)
    RIM_POSE_YAML.write_text(yaml.dump({
        "joints": q, "ee_pose": ee,
        "date": datetime.date.today().isoformat(),
        "note": ("턴테이블 rim 캘리브 시작 자세. 테두리가 실제로 스캔되는 것을 "
                 "확인하고 기록한 값이다. joints 를 그대로 재생한다."),
    }, default_flow_style=None, allow_unicode=True, sort_keys=False), encoding="utf-8")
    print(f"  ✓ rim 시작 자세 저장 → {RIM_POSE_YAML.name}")
    print(f"    J(deg) [{', '.join(f'{v:.1f}' for v in q)}]")
    return True


def _env_safe(q_from, q_to) -> bool:
    """셀 충돌 모델로 **가는 길**까지 본다.

    `precheck` 는 관절한계·self-collision·특이점만 본다 — 턴테이블·프레임 같은
    **환경**은 안 본다. rim 캘리브는 원판 코앞에서 조금씩 움직이는 작업이라
    이 검사가 빠지면 위험하다.
    """
    try:
        from utils.collision import collision_model as cmod
        cm = cmod.get_default()
    except Exception:
        cm = None
    if cm is None:
        return True
    safe, why = cm.is_pose_safe(q_to)
    if not safe:
        print(f"    ✘ 환경 충돌: {why} — 움직이지 않는다")
        return False
    ok, why, _ = cm.is_path_safe(q_from, q_to)
    if not ok:
        print(f"    ✘ 경로 충돌: {why} — 움직이지 않는다")
        return False
    return True


def _jog(robot, delta_mm, *, speed: float) -> bool:
    """base 프레임 상대 이동. `jog.py` 와 같은 안전 경로를 쓴다."""
    sys.path.insert(0, str(_PROJECT_ROOT / "scripts" / "robot"))
    from _common import sdk_ik, verify_target, precheck, move_cartesian   # noqa: E402

    q0 = robot.get_joint_angles(is_radian=True)
    target = robot.get_pose(is_radian=True).copy()
    target[:3] += np.asarray(delta_mm, float)
    print(f"  [jog] d=({delta_mm[0]:+.0f}, {delta_mm[1]:+.0f}, {delta_mm[2]:+.0f})mm "
          f"→ TCP ({target[0]:.0f}, {target[1]:.0f}, {target[2]:.0f})")
    try:
        q = sdk_ik(robot, target, seed=q0)
    except RuntimeError as e:
        print(f"    ✘ IK 실패: {e} — 움직이지 않는다")
        return False
    if not verify_target(robot, q, target) or not precheck(q):
        print("    ✘ 사전검사 실패 — 움직이지 않는다")
        return False
    if not _env_safe(q0, q):
        return False
    move_cartesian(robot, target, speed_mm_s=speed)
    return True


def _live_preview(client) -> None:
    """수동으로 팔을 끄는 **동안** 스캐너 화면을 계속 보여준다.

    ★ 이게 없으면 수동 모드가 의미가 없다 — 뭐가 찍히는지 안 보이면 자리를
      맞출 수가 없다. `live_view.py` 와 같은 방식(capture_frame 루프 + imshow)이다.

    rim 은 원판 전체가 화각에 안 들어오므로, **테두리가 화면을 길게 가로지르도록**
    맞추는 게 목표다. 화면 중앙 십자와 가장자리 여백선을 같이 그려 준다.
    """
    cv2.namedWindow(_WIN, cv2.WINDOW_NORMAL)      # 창이 닫힌 뒤 불릴 수 있다
    n_none = 0
    while True:
        try:
            fmh = client.capture_frame(capture_texture=True)
        except RuntimeError as e:
            print(f"    [warn] capture: {e}")
            time.sleep(0.2)
            fmh = None
        if fmh is None or not fmh.has_image() or not fmh.is_textured():
            n_none += 1
            if n_none in (10, 50) or n_none % 200 == 0:
                print(f"    ⚠ 텍스처 프레임을 {n_none}회 연속 못 받았다 — "
                      f"스캐너 앞이 비었는지 · Artec Studio 가 떠 있지 않은지 확인")
            if cv2.waitKey(30) & 0xFF in (13, 32, ord('q'), 27):
                return
            continue
        n_none = 0

        vis = cv2.cvtColor(fmh.image(), cv2.COLOR_RGB2BGR)
        h, w = vis.shape[:2]
        cv2.drawMarker(vis, (w // 2, h // 2), (0, 255, 255),
                       cv2.MARKER_CROSS, 40, 1)
        cv2.rectangle(vis, (w // 10, h // 10), (w - w // 10, h - h // 10),
                      (0, 180, 180), 1)
        cv2.putText(vis, "MANUAL - drag the arm", (12, 34),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(vis, "Enter/Space = accept   q = cancel", (12, 68),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(vis, "aim: rim arc should cross the frame", (12, h - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 200, 200), 2, cv2.LINE_AA)
        cv2.imshow(_WIN, vis)

        key = cv2.waitKey(30) & 0xFF
        if key in (13, 32):
            print("    ✓ 이 자리로 확정")
            return
        if key in (ord('q'), 27):
            print("    취소 — 그래도 위치 모드로 복귀한다")
            return


def _manual_mode(robot, client, *, open_browser: bool = True) -> bool:
    """**웹 UI 수동 모드 + 라이브 뷰** — `aim.py` 와 같은 방식.

    rim 은 원판 전체를 담을 수 없고 **가능한 긴 호**를 찾는 작업이라, 키 조그로
    더듬는 것보다 손으로 끄는 편이 훨씬 빠르다. 다만 손으로 끄는 수단은
    **웹 UI(xArm Studio)의 Manual Mode** 를 쓴다.

    ★ SDK 로 `set_mode(2)` 를 직접 걸지 않는다.
      2026-09-16 실측에서 그 경로가 컨트롤러 **error 37**
      ("Abnormal movement in Manual Mode — check TCP payload and mounting
      setting")을 띄웠고, `clean_error` 로도 안 지워졌다. 이 리포에는
      `set_tcp_load` 를 설정하는 코드가 **아예 없어서** 중력보상이 스캐너 무게를
      모른다. 반면 웹 UI 경로는 실사용에서 문제없이 동작한다(`aim.py` §조준).

    ⚠ 끝나면 **브라우저 탭을 반드시 닫는다.** 열려 있으면 컨트롤러의 mode/state 를
      웹 쪽이 쥐고 있어 이후 SDK 명령이 무시된다(`docs/robot_control.md` §1).
    """
    import webbrowser

    url = f"http://{ROBOT_IP}:18333"
    print("\n  " + "─" * 62)
    print("  수동 조절 — 웹 UI 로 팔을 끌면서 이 화면을 본다")
    print("  " + "─" * 62)
    print(f"    1. 웹 UI 에서 **Manual Mode** 를 켠다   {url}")
    print("       → 중력보상이 걸려 팔을 손으로 밀 수 있다")
    print("    2. 화면을 보며 **원판 테두리가 길게 걸치도록** 맞춘다")
    print("    3. 자리를 잡으면 손을 뗀다 (툴 무게로 처질 수 있으니 확인)")
    print("    4. **Manual Mode 를 끄고 브라우저 탭을 닫는다**")
    print("  " + "─" * 62)
    if open_browser:
        try:
            webbrowser.open(url)
        except Exception:                                       # noqa: BLE001
            pass

    print("\n  ▶ 라이브 화면을 띄운다. Enter/Space = 이 자리로 확정, q = 취소")
    _live_preview(client)

    print("\n  ⚠ Manual Mode 를 끄고 **브라우저 탭을 닫았는지** 확인하세요.")
    print("     열려 있으면 이후 SDK 명령(조그·home·다음 단계)이 무시된다.")
    try:
        input("  닫았으면 Enter > ")
    except (EOFError, OSError):
        pass

    try:
        err = robot.arm.error_code
        if err:
            print(f"  ⚠ 컨트롤러 error_code={err} — 웹 UI 에서 해소한 뒤 진행할 것")
            return False
        print(f"  ✓ J(deg) "
              f"[{', '.join(f'{v:.1f}' for v in np.degrees(robot.get_joint_angles(is_radian=True)))}]")
    except Exception as e:                                      # noqa: BLE001
        print(f"  ⚠ 상태 확인 실패: {e}")
    return True


def _jog_optical_roll(robot, T_EC, delta_deg: float, *, speed: float) -> bool:
    """**카메라 광축 둘레로만** 돌린다 — 보는 지점은 그대로, 화면만 회전.

    ★ J7 회전과 다르다. 스캐너가 플랜지 축에서 173mm 옆에 달려 있어서
      (`T_EC` 병진) J7 을 돌리면 카메라가 그 반경으로 **선회**한다 —
      실측: J7 +10° 에 카메라가 13.6mm 이동하고 광축이 9.8° 기운다.
      rim 의 호를 화면 대각선으로 길게 눕히려면 **보는 방향은 유지한 채**
      이미지만 돌아야 하므로, 카메라 로컬 z 축 둘레 회전을 따로 만든다.

        T_WC = T_WE · inv(T_EC)        (카메라 world pose)
        T_WE' = T_WC · Rz(θ) · T_EC    (같은 카메라 위치·광축, roll 만 다름)
    """
    sys.path.insert(0, str(_PROJECT_ROOT / "scripts" / "robot"))
    from _common import sdk_ik, verify_target, precheck, move_cartesian   # noqa: E402
    from scipy.spatial.transform import Rotation as _R

    q0 = robot.get_joint_angles(is_radian=True)
    p = robot.get_pose(is_radian=True)
    T_WE = np.eye(4)
    T_WE[:3, :3] = _R.from_euler("xyz", p[3:6]).as_matrix()
    T_WE[:3, 3] = np.asarray(p[:3], float) / 1000.0

    Rz = np.eye(4)
    Rz[:3, :3] = _R.from_euler("z", delta_deg, degrees=True).as_matrix()
    T_WC = T_WE @ np.linalg.inv(T_EC)
    T_new = T_WC @ Rz @ T_EC

    target = np.concatenate([T_new[:3, 3] * 1000.0,
                             _R.from_matrix(T_new[:3, :3]).as_euler("xyz")])
    print(f"  [jog] 광축 roll {delta_deg:+.0f}°")
    try:
        q = sdk_ik(robot, target, seed=q0)
    except RuntimeError as e:
        print(f"    ✘ IK 실패: {e} — 움직이지 않는다")
        return False
    if not verify_target(robot, q, target) or not precheck(q):
        print("    ✘ 사전검사 실패 — 움직이지 않는다")
        return False
    if not _env_safe(q0, q):
        return False
    move_cartesian(robot, target, speed_mm_s=speed)
    return True


def _jog_joint(robot, joint_idx: int, delta_deg: float, *, speed_deg: float) -> bool:
    """관절 하나만 돌린다 (J7 = idx 6).

    ★ 직교 이동이 아니라 **관절 명령**이다. J7 은 광축 둘레 회전에 가까워서
      카메라 위치를 거의 안 바꾸고 화면만 돌린다 — IK 로 풀면 7축이라 다른
      분기가 튀어나올 수 있으므로 관절을 직접 준다.
    """
    sys.path.insert(0, str(_PROJECT_ROOT / "scripts" / "robot"))
    from _common import precheck   # noqa: E402

    q0 = robot.get_joint_angles(is_radian=True)
    q = np.asarray(q0, float).copy()
    q[joint_idx] += np.radians(delta_deg)
    print(f"  [jog] J{joint_idx+1} {delta_deg:+.0f}° "
          f"→ {np.degrees(q[joint_idx]):+.1f}°")
    if not precheck(q):
        print("    ✘ 사전검사 실패 — 움직이지 않는다")
        return False
    if not _env_safe(q0, q):
        return False
    robot.enable_motion()
    code = robot.arm.set_servo_angle(angle=list(np.degrees(q)), speed=speed_deg,
                                     is_radian=False, wait=True)
    if code != 0:
        print(f"    ✘ set_servo_angle code={code}")
        return False
    return True


def _reposition(robot, args) -> bool:
    """rim 캡처 전 로봇을 **보이는 자리로** 옮긴다.

    ★ 왜 필요한가 — `calibrate.py` 로 1→2→3 을 이어 돌리면 이 단계가 시작될 때
      팔이 **2단계(hand-eye)의 마지막 자세**에 그대로 서 있다. 그 자세는 보드를
      가까이 들여다보는 자세라 턴테이블 테두리가 화각에 거의 안 들어온다
      (2026-09-16 실측). 그래서 기본값으로 home 을 먼저 들른다.

    미세조정은 `jog.py` 와 **같은 안전 경로**를 쓴다 — 컨트롤러 IK(`sdk_ik`) →
    `verify_target` → `precheck` → `move_cartesian`.
    `XArmInterface.move_relative()` 는 쓰지 않는다: 해석 IK(sim USD 모델)를 먹여
    실물에서 엉뚱한 곳으로 간다 (`docs/robot_control.md` §함정).
    """
    # ★ 기록해 둔 **rim 자세**가 있으면 그걸 우선한다.
    #   home 은 이 셀에서 테두리가 화각에 잘 안 들어온다 — Artec Studio 로
    #   실제 스캔되는 것을 확인한 자세를 2026-09-16 에 기록해 두었다.
    #   ee_pose 가 아니라 **joints 를 그대로 재생**한다: 7축이라 같은 TCP 에
    #   해가 무한히 많고, 컨트롤러 IK 가 다른 분기를 고르면 전혀 다른 자세가 된다
    #   (`gen_calib_poses.py` 가 joints 를 쓰는 이유와 같다).
    if args.home and RIM_POSE_YAML.exists() and not args.force_home:
        d = yaml.safe_load(RIM_POSE_YAML.read_text(encoding="utf-8")) or {}
        q = d.get("joints")
        if q:
            print(f"[move] 기록된 rim 자세로 이동  ({RIM_POSE_YAML.name}, {d.get('date','?')})")
            print(f"       J(deg) [{', '.join(f'{v:.1f}' for v in q)}]")
            try:
                robot.enable_motion()
                code = robot.arm.set_servo_angle(angle=list(q), speed=args.home_speed,
                                                 is_radian=False, wait=True)
                if code != 0:
                    print(f"  ⚠ set_servo_angle code={code} — home 으로 대체한다")
                    robot.go_home(sensor=args.sensor, speed=args.home_speed, confirm=False)
            except Exception as e:
                print(f"  ⚠ 이동 실패: {e}")
                return False
        else:
            print(f"  ⚠ {RIM_POSE_YAML.name} 에 joints 가 없다 — home 으로 간다")
            robot.go_home(sensor=args.sensor, speed=args.home_speed, confirm=False)
    elif args.home:
        print(f"[move] home 자세로 이동 (sensor={args.sensor}, speed={args.home_speed} deg/s)")
        try:
            robot.go_home(sensor=args.sensor, speed=args.home_speed, confirm=False)
        except Exception as e:
            print(f"  ⚠ go_home 실패: {e}")
            return False

    delta = np.array([args.dx, args.dy, args.dz,
                      *np.radians([args.droll, args.dpitch, args.dyaw])])
    if not delta.any():
        return True

    sys.path.insert(0, str(_PROJECT_ROOT / "scripts" / "robot"))
    from _common import sdk_ik, verify_target, precheck, move_cartesian   # noqa: E402

    target = robot.get_pose(is_radian=True) + delta
    print(f"[move] 미세조정  dx={args.dx:+.1f} dy={args.dy:+.1f} dz={args.dz:+.1f} mm  "
          f"droll={args.droll:+.1f} dpitch={args.dpitch:+.1f} dyaw={args.dyaw:+.1f}°")
    print(f"       목표 TCP  x={target[0]:.1f} y={target[1]:.1f} z={target[2]:.1f} mm")
    try:
        q_target = sdk_ik(robot, target, seed=robot.get_joint_angles(is_radian=True))
    except RuntimeError as e:
        print(f"  ✘ IK 실패: {e}")
        return False
    if not verify_target(robot, q_target, target):
        return False
    if not precheck(q_target):
        print("  ✘ 사전검사 실패 — 이동하지 않는다")
        return False
    if args.dry_run:
        print("  --dry-run : 여기까지. 움직이지 않는다.")
        return False
    move_cartesian(robot, target, speed_mm_s=args.speed)
    return True


def main():
    import argparse

    from scripts.artec._step_guard import warn_if_direct
    warn_if_direct(3)

    ap = argparse.ArgumentParser(description="턴테이블 T_B_F0 캘리브 (rim 클릭)")
    ap.add_argument("--no-home", dest="home", action="store_false",
                    help="이동 없이 지금 자세에서 바로 캡처")
    ap.add_argument("--force-home", action="store_true",
                    help=f"기록된 rim 자세({RIM_POSE_YAML.name}) 대신 home 으로 간다")
    ap.add_argument("--save-pose", action="store_true",
                    help="시작 시 **지금 자세**를 rim 시작 자세로 기록하고 그대로 진행 "
                         "(창에서 S 로도 저장된다)")
    ap.add_argument("--sensor", default="artec", help="home 자세 프리셋")
    ap.add_argument("--home-speed", type=float, default=20.0, help="home 이동 속도 (deg/s)")
    ap.add_argument("--speed", type=float, default=20.0, help="미세조정 속도 (mm/s)")
    ap.add_argument("--jog-step", type=float, default=20.0,
                    help="창 안에서 조그할 때 한 번에 움직이는 거리 (mm). "
                         "창에서 [ ] 로 절반/두배 조절")
    ap.add_argument("--jog-rot-step", type=float, default=10.0,
                    help="창 안에서 관절을 돌릴 때 한 번에 도는 각 (deg). "
                         "창에서 , . 로 절반/두배 조절")
    ap.add_argument("--sensitivity", type=float, default=0.9,
                    help="재구성 민감도 0~1 (SDK 기본 0.5). 턴테이블 상판이 "
                         "**흰 무광/광택 평면**이라 구조광 반사가 약해 기본값으로는 "
                         "정점이 거의 안 나온다 — 2026-09-16 실측 0.5→67개, "
                         "0.9→698개, 1.0→1739개. 다만 올릴수록 노이즈도 섞인다")
    ap.add_argument("--range-mm", nargs=2, type=float, default=None,
                    metavar=("NEAR", "FAR"),
                    help="스캔 깊이 범위 (기본 170~330). 좁히면 배경 노이즈가 준다")
    ap.add_argument("--dry-run", action="store_true",
                    help="미세조정을 검증만 하고 움직이지 않는다")
    for a in ("dx", "dy", "dz"):
        ap.add_argument(f"--{a}", type=float, default=0.0,
                        help="home 이후 상대 이동 (mm, B 프레임)")
    for a in ("droll", "dpitch", "dyaw"):
        ap.add_argument(f"--{a}", type=float, default=0.0,
                        help="home 이후 상대 회전 (deg, B 프레임)")
    args = ap.parse_args()

    if not SENSOR_FRAMES.exists():
        print(f"⚠ {SENSOR_FRAMES} 없음. hand-eye 먼저 완료 후 T_EC_artec 갱신 필요.")
        return 1
    T_EC = load_transform(str(SENSOR_FRAMES), "T_EC_artec")
    print(f"[artec] T_EC_artec loaded  t={T_EC[:3,3]} m")

    # Color 카메라 intrinsic — 정점을 Color 프레임으로 옮길 때 쓴다.
    _intr_path = _PROJECT_ROOT / "config" / "calibration" / "artec_intrinsic.yaml"
    if not _intr_path.exists():
        print(f"⚠ {_intr_path.name} 없음 — intrinsic 먼저 (calibrate.py --only 1)")
        return 1
    _it = yaml.safe_load(_intr_path.read_text(encoding="utf-8"))
    K_color = np.array(_it["K"], float)
    dist_color = np.array(_it.get("dist", [0] * 5), float).reshape(-1)

    robot = XArmInterface(ROBOT_IP)
    if args.save_pose:
        # 지금 자세를 기록하고, 그 자리에서 그대로 진행한다.
        _save_rim_pose(robot)
        args.home = False
    if not _reposition(robot, args):
        robot.disconnect()
        return 1

    cfg = ArtecConfig(serial_number=None, capture_texture=True)
    client = ArtecClient(cfg)
    client.initialize()
    # ★ auto exposure 를 안 켜면 capture_frame 이 계속 None 을 돌려준다(실측 —
    #   live_view.py:70, hand_eye_calib 도 같은 설정으로 시작한다). 라이브
    #   프리뷰가 빈 화면만 도는 원인이 대부분 이것이다.
    try:
        client._scanner.enable_auto_exposure(True)
    except Exception as e:                                      # noqa: BLE001
        print(f"[artec] ⚠ auto exposure 설정 실패: {e}")

    # ★ 재구성 민감도. rim 캘리브의 실질적 병목이다 —
    #   2026-09-16 실측: SDK 기본 0.5 에서 정점이 **67개**밖에 안 나왔다.
    #   흰 상판이 구조광을 거의 안 돌려주기 때문이고, UV 매칭 실패의 진짜 원인도
    #   이것이었다(문턱값이 아니라 데이터가 없었다). 0.9 → 698개, 1.0 → 1739개.
    try:
        print(f"[artec] sensitivity {client._processor.sensitivity():.2f} "
              f"→ {args.sensitivity:.2f}")
        client._processor.set_sensitivity(float(args.sensitivity))
        if args.range_mm:
            client._processor.set_scanning_range(*[float(v) for v in args.range_mm])
            print(f"[artec] scanning range → {args.range_mm[0]:.0f}~{args.range_mm[1]:.0f}mm")
    except Exception as e:                                      # noqa: BLE001
        print(f"[artec] ⚠ 민감도/범위 설정 실패: {e}")

    # 조그 스텝(mm). 루프 밖에 두어 캡처를 반복해도 유지된다. 리스트인 건
    # 창 키 핸들러에서 값을 바꾸기 위함.
    jog_step = [float(args.jog_step)]
    rot_step = [float(args.jog_rot_step)]

    try:
        while True:
            fmh = _capture_frame(client)
            if fmh is None:
                print("재시도? (Enter / q): ", end="")
                if input().strip().lower() == "q":
                    return
                continue

            image = fmh.image()                 # (H, W, 3) RGB
            verts = fmh.vertices()              # (N, 3) mm — **스캐너 3D 프레임**
            uv    = fmh.uv()                    # (N, 2) [0..1]
            print(f"[artec] frame  verts={len(verts):,}  image={image.shape[1]}×{image.shape[0]}")

            # ★ 정점을 Color 카메라 프레임으로 옮길 변환. T_EC 가 Color 기준이라
            #   이게 없으면 180°+41.8mm 가 그대로 오차로 실린다.
            T_scan_color = solve_scanner_to_color(
                verts, uv, (image.shape[1], image.shape[0]), K_color, dist_color)
            if T_scan_color is None:
                print("  ⚠ 스캐너3D→Color 변환을 못 구했다 — 옛 z-flip 으로 진행한다."
                      " 결과가 수십 mm 어긋날 수 있다.")

            picker = _RimPicker(image, verts, uv, T_scan_color)

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

            _print_keys(jog_step[0], rot_step[0])
            action = None
            while True:
                key = cv2.waitKey(20) & 0xFF
                if key in (13, 32):     # Enter / Space
                    action = "fit"; break
                if key == ord('r'):
                    action = "recapture"; break
                if key in (ord('q'), 27):
                    action = "quit"; break
                # ── 조그 — 창을 닫지 않고 팔을 움직이고 다시 캡처한다 ──
                if key in _JOG_KEYS:
                    d = np.array(_JOG_KEYS[key], float) * jog_step[0]
                    action = ("jog", d); break
                if key in _JOG_JOINT_KEYS:
                    idx, sgn = _JOG_JOINT_KEYS[key]
                    action = ("jogj", idx, sgn * rot_step[0]); break
                if key in (ord('o'), ord('p')):
                    action = ("roll", (-1 if key == ord('o') else +1) * rot_step[0])
                    break
                if key in (ord('['), ord(']')):
                    jog_step[0] = float(np.clip(
                        jog_step[0] * (0.5 if key == ord('[') else 2.0), 1.0, 80.0))
                    print(f"  이동 스텝 = {jog_step[0]:.0f} mm")
                if key in (ord(','), ord('.')):
                    rot_step[0] = float(np.clip(
                        rot_step[0] * (0.5 if key == ord(',') else 2.0), 1.0, 90.0))
                    print(f"  회전 스텝 = {rot_step[0]:.0f}°")
                if key == ord('v'):
                    picker.view_3d = not picker.view_3d
                    cv2.imshow(_WIN, picker.render())
                if key == ord('S'):          # 대문자 — 실수로 눌리지 않게
                    _save_rim_pose(robot)
                if key == ord('m'):
                    action = "manual"; break
                if key == ord('h'):
                    action = "home"; break
            cv2.destroyWindow(_WIN)

            if action == "quit":
                return
            if action == "recapture":
                continue
            if action == "manual":
                _manual_mode(robot, client)
                continue
            if action == "home":
                print("[move] home 으로 복귀")
                try:
                    robot.go_home(sensor=args.sensor, speed=args.home_speed, confirm=False)
                except Exception as e:
                    print(f"  ⚠ go_home 실패: {e}")
                continue
            if isinstance(action, tuple) and action[0] == "jog":
                _jog(robot, action[1], speed=args.speed)
                continue
            if isinstance(action, tuple) and action[0] == "jogj":
                _jog_joint(robot, action[1], action[2], speed_deg=args.home_speed)
                continue
            if isinstance(action, tuple) and action[0] == "roll":
                _jog_optical_roll(robot, T_EC, action[1], speed=args.speed)
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

                # ★ 저장 직후 **결과를 화면에 겹쳐** 보여준다.
                #   잔차·반경 같은 숫자는 점들끼리의 일관성만 말해준다 — 그 원이
                #   실제 원판 위에 놓였는지는 알려주지 않는다. 2026-09-16 에
                #   지표가 전부 OK 인데 실제 테두리와 35mm 어긋난 적이 있어서,
                #   같은 자리에서 바로 확인하도록 붙였다.
                try:
                    from scripts.artec.turntable_overlay import build_overlay, show
                    intr = yaml.safe_load(
                        (_PROJECT_ROOT / "config/calibration/artec_intrinsic.yaml")
                        .read_text(encoding="utf-8"))
                    print("\n[verify] 결과를 화면에 겹쳐 확인한다")
                    vis, diag = build_overlay(
                        cv2.cvtColor(image, cv2.COLOR_RGB2BGR), T_EB, T_EC, T_BF0,
                        np.array(intr["K"], float),
                        np.array(intr.get("dist", [0] * 5), float).reshape(-1),
                        radius_mm)
                    show(vis)
                except Exception as e:                          # noqa: BLE001
                    print(f"  [warn] 오버레이 표시 실패: {e}")
                    print("  따로 확인:  python scripts/artec/turntable_overlay.py")
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
