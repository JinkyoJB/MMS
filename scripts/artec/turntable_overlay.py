#!/usr/bin/env python3
"""turntable_overlay.py — 저장된 `T_B_F0` 를 **실제 카메라 화면에 겹쳐서** 검증한다.

왜 필요한가
----------
`check_calibration.py` 는 숫자만 본다 — 점 수·잔차·반경. 그런데 그 숫자가 전부
통과해도 축이 엉뚱한 곳에 있을 수 있다. 점들끼리 깨끗한 원을 이루는 것과, 그 원이
**실제 원판 위에 놓이는 것**은 다른 문제이기 때문이다. 2026-09-16 에 점 15개·잔차
0.18mm·반경 116mm 로 지표가 전부 OK 인데 투영해 보니 실제 테두리와 35mm 어긋난
경우가 있었다. 그걸 눈으로 잡으려고 만들었다.

무엇을 그리나
------------
  · 자홍 — 영상에서 검출한 **실제 원판 경계** (밝은 원판 / 어두운 배경 threshold)
  · 빨강 — 저장된 `T_B_F0` + 저장된 반경으로 투영한 원
  · 초록 — 같은 축, 반경만 **실측값**(기본 119mm)으로 바꾼 원
  · 청록 — 축 원점과 회전축 방향

빨강·초록이 **둘 다** 자홍에서 벗어나면 반경이 아니라 **축의 위치·기울기**가 틀린 것이다.

실행
----
  # 지금 자세 그대로 (로봇 안 움직임)
  python scripts/artec/turntable_overlay.py

  # 기록된 rim 자세로 이동해서 확인  ⚠ 로봇이 움직인다
  python scripts/artec/turntable_overlay.py --pose rim
  python scripts/artec/turntable_overlay.py --pose home

  # 원판 실측 반경이 다르면
  python scripts/artec/turntable_overlay.py --ref-radius 119
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import cv2
import numpy as np
import yaml

_PROJECT_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(_PROJECT_ROOT))

from utils.transforms import load_transform, compute_T_CB

SENSOR_FRAMES = _PROJECT_ROOT / "config" / "sensor_frames.yaml"
TURNTABLE_YAML = _PROJECT_ROOT / "config" / "calibration" / "turntable_frame.yaml"
INTRINSIC_YAML = _PROJECT_ROOT / "config" / "calibration" / "artec_intrinsic.yaml"
RIM_POSE_YAML = _PROJECT_ROOT / "config" / "calibration" / "artec_rim_pose.yaml"
WIN = "turntable T_B_F0 overlay"


def _project(P_C_m, K, dist, W, H):
    """카메라 좌표(m) → 픽셀. 카메라 뒤/화면 밖은 걸러낸다."""
    P = np.asarray(P_C_m, float).reshape(-1, 3)
    front = P[:, 2] > 1e-3
    if not front.any():
        return np.empty((0, 2)), front
    px = cv2.projectPoints(P[front] * 1000.0, np.zeros(3), np.zeros(3),
                           K, dist)[0].reshape(-1, 2)
    inside = ((px[:, 0] >= 0) & (px[:, 0] < W) & (px[:, 1] >= 0) & (px[:, 1] < H))
    return px[inside], front


def _real_edge(img):
    """영상에서 밝은 원판의 외곽선을 뽑는다 (검증 기준선)."""
    g = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    _, m = cv2.threshold(g, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((9, 9), np.uint8))
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((25, 25), np.uint8))
    cnts, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    if not cnts:
        return np.empty((0, 2))
    c = max(cnts, key=cv2.contourArea).reshape(-1, 2).astype(float)
    H, W = g.shape[:2]
    # 이미지 가장자리에 붙은 구간은 원판 경계가 아니다 — 잘린 부분일 뿐.
    keep = (c[:, 0] > 3) & (c[:, 0] < W - 4) & (c[:, 1] > 3) & (c[:, 1] < H - 4)
    return c[keep]


def build_overlay(img_bgr, T_EB, T_EC, T_BF0, K, dist,
                  r_fit_mm: float, r_ref_mm: float = 119.0, verbose: bool = True):
    """오버레이 이미지를 만들고 (이미지, 진단 dict) 를 돌려준다.

    규약:  T_B_F0 는 B→F (x_F = T_B_F0·x_B) 이므로 F→B 는 inv.
           T_CB 는 C→B 이므로 B→C 는 inv.
    """
    vis = img_bgr.copy()
    H, W = vis.shape[:2]
    T_BC = np.linalg.inv(compute_T_CB(T_EB, T_EC))
    T_F0B = np.linalg.inv(T_BF0)

    edge = _real_edge(vis)
    for (x, y) in edge.astype(int)[::3]:
        cv2.circle(vis, (x, y), 1, (255, 0, 255), -1)

    def circle(r_mm, color):
        th = np.linspace(0, 2 * np.pi, 1440)
        P_F = np.stack([r_mm / 1000 * np.cos(th), r_mm / 1000 * np.sin(th),
                        np.zeros_like(th), np.ones_like(th)])
        P_B = (T_F0B @ P_F)[:3].T
        P_C = (T_BC[:3, :3] @ P_B.T).T + T_BC[:3, 3]
        px, _ = _project(P_C, K, dist, W, H)
        for (x, y) in px.astype(int):
            cv2.circle(vis, (x, y), 2, color, -1)
        if len(px) == 0 or len(edge) == 0:
            return None
        d_px = np.sqrt(((px[:, None, :] - edge[None, :, :]) ** 2).sum(-1)).min(1)
        z_mm = float(np.linalg.norm(P_C, axis=1).mean() * 1000.0)
        return dict(n=int(len(px)),
                    med_px=float(np.median(d_px)),
                    med_mm=float(np.median(d_px) / K[0, 0] * z_mm),
                    min_mm=float(d_px.min() / K[0, 0] * z_mm))

    d_fit = circle(r_fit_mm, (0, 0, 255))
    d_ref = circle(r_ref_mm, (0, 255, 0))

    # 축 원점 + 회전축 방향 (F 의 +z, 50mm)
    o_B = T_F0B[:3, 3]
    ax_B = o_B + T_F0B[:3, 2] * 0.05
    for P_B, mark in ((o_B, "o"), (ax_B, "z")):
        P_C = T_BC[:3, :3] @ P_B + T_BC[:3, 3]
        px, _ = _project(P_C[None], K, dist, W, H)
        if len(px):
            p = px[0].astype(int)
            cv2.drawMarker(vis, tuple(p), (255, 255, 0),
                           cv2.MARKER_TILTED_CROSS if mark == "o" else cv2.MARKER_DIAMOND,
                           26, 2)

    cv2.putText(vis, "magenta=REAL edge  red=fit  green=ref  cyan=axis",
                (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (255, 255, 0), 2, cv2.LINE_AA)
    y = 56
    for tag, d in (("fit  r=%.0fmm" % r_fit_mm, d_fit), ("ref  r=%.0fmm" % r_ref_mm, d_ref)):
        if d is None:
            txt = f"{tag}: not visible"
            col = (160, 160, 160)
        else:
            txt = f"{tag}: to real edge  median {d['med_mm']:.1f}mm  min {d['min_mm']:.1f}mm"
            col = (0, 200, 0) if d["med_mm"] < 5 else (0, 80, 255)
        cv2.putText(vis, txt, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.58, col, 2, cv2.LINE_AA)
        y += 26
        if verbose:
            print("  " + txt)

    ok = d_fit is not None and d_fit["med_mm"] < 5.0
    if verbose:
        print("  판정:", "실제 테두리와 일치 ✅" if ok
              else "실제 테두리와 어긋난다 — 축의 위치·기울기 문제 ❌")
    return vis, dict(fit=d_fit, ref=d_ref, ok=ok, n_edge=int(len(edge)))


def show(vis, block: bool = True) -> None:
    """오버레이를 창으로 띄운다. 아무 키나 누르면 닫힌다."""
    # GUI 가 없는 환경(에이전트 셸·SSH)에서는 창이 안 뜬다. 검증용 보조 기능이
    # 본 작업을 죽이면 안 되므로 통째로 감싼다.
    try:
        cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
        h, w = vis.shape[:2]
        cv2.resizeWindow(WIN, min(w, 760), min(h, 1000))
        cv2.imshow(WIN, vis)
        if block:
            print("  (창에서 아무 키나 누르면 닫힌다)")
            cv2.waitKey(0)
            cv2.destroyWindow(WIN)
    except cv2.error as e:
        print(f"  [warn] 창을 띄울 수 없다 ({e.err.splitlines()[0] if e.err else e}) "
              f"— --save 로 파일로 확인할 것")


def main() -> int:
    ap = argparse.ArgumentParser(description="저장된 T_B_F0 를 화면에 겹쳐 검증")
    ap.add_argument("--pose", choices=("here", "rim", "home"), default="here",
                    help="here=안 움직임(기본) · rim=기록된 rim 자세 · home ⚠ 움직인다")
    ap.add_argument("--ref-radius", type=float, default=119.0, help="원판 실측 반경(mm)")
    ap.add_argument("--sensitivity", type=float, default=0.9)
    ap.add_argument("--speed", type=float, default=20.0, help="이동 속도 (deg/s)")
    ap.add_argument("--save", default=None, help="오버레이 PNG 저장 경로")
    args = ap.parse_args()

    for p in (SENSOR_FRAMES, TURNTABLE_YAML, INTRINSIC_YAML):
        if not p.exists():
            print(f"✘ 없음: {p}")
            return 1

    tt = yaml.safe_load(TURNTABLE_YAML.read_text(encoding="utf-8"))
    T_BF0 = np.array(tt["T_B_F0"]["matrix"], float)
    r_fit = float(tt.get("rim_radius_mm", args.ref_radius))
    print(f"[overlay] T_B_F0 {tt.get('date','?')}  점 {tt.get('n_points','?')}개  "
          f"반경 {r_fit:.2f}mm  잔차 {tt.get('rim_residual_mm','?')}mm")

    T_EC = load_transform(str(SENSOR_FRAMES), "T_EC_artec")
    it = yaml.safe_load(INTRINSIC_YAML.read_text(encoding="utf-8"))
    K = np.array(it["K"], float)
    dist = np.array(it.get("dist", [0] * 5), float).reshape(-1)

    from utils.robot.xarm_interface import XArmInterface
    from mms_artec.sensor.artec_client import ArtecClient, ArtecConfig

    robot = XArmInterface("192.168.1.210")
    try:
        if args.pose == "home":
            print("[move] home 으로 이동")
            robot.go_home(sensor="artec", speed=args.speed, confirm=False)
        elif args.pose == "rim":
            q = yaml.safe_load(RIM_POSE_YAML.read_text(encoding="utf-8"))["joints"]
            print(f"[move] 기록된 rim 자세로 이동  {[round(v,1) for v in q]}")
            robot.enable_motion()
            robot.arm.set_servo_angle(angle=list(q), speed=args.speed,
                                      is_radian=False, wait=True)
        T_EB = robot.get_ee_pose_mat()
    finally:
        robot.disconnect()

    sensor = ArtecClient(ArtecConfig(serial_number=None, capture_texture=True))
    sensor.initialize()
    try:
        sensor._scanner.enable_auto_exposure(True)
    except Exception:                                           # noqa: BLE001
        pass
    try:
        sensor._processor.set_sensitivity(args.sensitivity)
    except Exception:                                           # noqa: BLE001
        pass
    try:
        img = None
        for _ in range(8):
            try:
                f = sensor.capture_frame(capture_texture=True)
            except RuntimeError:
                continue
            if f is not None and f.has_image():
                img = cv2.cvtColor(f.image(), cv2.COLOR_RGB2BGR)
                break
        if img is None:
            print("✘ 캡처 실패")
            return 1
    finally:
        sensor.shutdown()

    vis, diag = build_overlay(img, T_EB, T_EC, T_BF0, K, dist,
                              r_fit, args.ref_radius)
    if args.save:
        cv2.imwrite(args.save, vis)
        print(f"  저장 → {args.save}")
    show(vis)
    return 0 if diag["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
