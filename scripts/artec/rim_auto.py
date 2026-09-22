#!/usr/bin/env python
"""rim_auto.py — 턴테이블 rim 을 **자동 검출**해서 `T_B_F0` 를 구한다.

왜 만들었나 (2026-09-21 실측)
-----------------------------
손으로 클릭하는 방식은 **한 자세에서 짧은 호밖에 못 찍는다.** 원판은 화각에 다
안 들어오기 때문이다. 짧은 호에 원을 맞추면 중심뿐 아니라 **반경까지** 무너진다:

    자세 A (호 55°, 18점)   → 반경 117.05mm  잔차 0.222mm
    자세 B (호 58°, 15점)   → 반경 102.23mm  잔차 1.343mm     ← 17mm 틀림
    A+B 합침 (33점)         → 반경 119.34mm  잔차 1.089mm     ← 실측 119mm 와 일치

잔차가 작다고 안심할 수 없다는 점이 핵심이다 — A 는 잔차 0.22mm 인데 반경이
2mm 틀렸고, 중심은 더 틀렸다. **두 자세를 합치자 비로소 맞았다.**

그래서 이 도구는 두 가지를 바꾼다:
  1. **여러 자세**에서 찍어 원주를 넓게 덮는다 (호가 넓어야 원이 정해진다)
  2. 클릭 대신 **영상에서 원판 외곽선을 자동 검출**한다 (점 수가 수백 개가 되고
     사람 손떨림·판단이 빠진다)

    python scripts/artec/rim_auto.py                 # home + 현재 자세
    python scripts/artec/rim_auto.py --no-home       # 현재 자세만
    python scripts/artec/rim_auto.py --dry-run       # 저장 안 함

⚠ 스캐너는 SDK 와 Artec Studio 중 하나만 잡는다 — Studio 를 닫고 실행할 것.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import cv2
import numpy as np
import yaml

_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(_ROOT))

from scripts.artec.turntable_calib import (                    # noqa: E402
    pixel_to_3d_C, solve_scanner_to_color, uv_match_tolerance)
from scripts.artec.turntable_overlay import _real_edge         # noqa: E402
from utils.calibration.turntable_frame import (                # noqa: E402
    fit_circle_3d, build_T_B_F0, save_turntable_frame_yaml)
from utils.transforms import compute_T_CB, load_transform      # noqa: E402

OUT_YAML = _ROOT / "config" / "calibration" / "turntable_frame.yaml"
INTRINSIC = _ROOT / "config" / "calibration" / "artec_intrinsic.yaml"
RIM_POSE = _ROOT / "config" / "calibration" / "artec_rim_pose.yaml"

#: 검출 외곽선을 몇 픽셀 간격으로 쓸지 (전부 쓰면 중복만 많다).
EDGE_STRIDE = 2
#: 호가 이보다 좁으면 원이 안 정해진다 — 실측 근거는 모듈 docstring.
MIN_ARC_DEG = 120.0


def _collect(client, robot, T_EC, tag: str):
    """지금 자세에서 한 장 찍고 **원판 외곽선 3D 점**(base, mm)을 돌려준다."""
    fmh = client.capture_frame(capture_texture=True)
    if fmh is None or not fmh.has_image() or fmh.vertex_count() == 0:
        print(f"  [{tag}] capture 실패 — 건너뜀")
        return np.zeros((0, 3))
    img = fmh.image()
    verts, uv = fmh.vertices(), fmh.uv()
    H, W = img.shape[:2]

    intr = yaml.safe_load(INTRINSIC.read_text(encoding="utf-8"))
    T_sc = solve_scanner_to_color(verts, uv, (W, H),
                                  np.asarray(intr["K"], float),
                                  np.asarray(intr.get("dist", [0] * 5), float))
    if T_sc is None:
        print(f"  [{tag}] ⚠ 스캐너3D→Color 변환 실패 — 이 자세는 버린다")
        return np.zeros((0, 3))

    edge = _real_edge(cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
    if len(edge) == 0:
        print(f"  [{tag}] 원판 외곽선을 못 찾았다 — 건너뜀")
        return np.zeros((0, 3))

    pix_uv = uv.astype(np.float64).copy()
    pix_uv[:, 0] *= float(W); pix_uv[:, 1] *= float(H)
    tol = uv_match_tolerance(len(verts), (W, H))
    pts_C = []
    for (x, y) in edge[::EDGE_STRIDE]:
        p = pixel_to_3d_C((float(x), float(y)), verts, uv, (W, H),
                          max_dist_px=tol, pix_uv=pix_uv, T_scan_color=T_sc)
        if p is not None:
            pts_C.append(p)
    if not pts_C:
        print(f"  [{tag}] 외곽선 {len(edge)}px 중 3D 로 옮긴 점 0개 "
              f"(원판 표면이 재구성 안 됨) — 건너뜀")
        return np.zeros((0, 3))

    T_CB = compute_T_CB(robot.get_ee_pose_mat(), T_EC)
    pts_B = np.asarray(pts_C) @ T_CB[:3, :3].T + T_CB[:3, 3] * 1000.0
    print(f"  [{tag}] 외곽선 {len(edge)}px → 3D {len(pts_B)}점")
    return pts_B


def _sweep_azimuths(client, robot, T_EC, args):
    """축 둘레로 **방위를 벌려가며** 추가 촬영. 반환 = 점군 조각 리스트.

    지금 카메라의 고도각·축거리는 그대로 두고 방위만 바꾼다 — 지금 자세가 rim 을
    잘 보고 있다는 전제에서, 같은 조건으로 원주의 **다른 구간**을 보려는 것이다.
    """
    from utils.robot import view_pose as vp
    from utils.robot import xarm7_kinematics as kin
    from utils.transforms import load_transform, TurntableTransformConfig
    from scripts.robot._common import collision_gate, plan_safe_joint_path

    tt = TurntableTransformConfig(load_transform(
        str(_ROOT / "config" / "calibration" / "turntable_frame.yaml"), "T_B_F0"))
    axis_pt = np.asarray(tt.axis_point_B, float)
    up = -np.asarray(tt.axis_dir_B, float)      # 천장 마운트: '위' 는 축의 반대
    up /= np.linalg.norm(up)

    # 지금 카메라의 (el, az, standoff) 를 축 기준으로 역산한다.
    T_CB = compute_T_CB(robot.get_ee_pose_mat(), T_EC)
    cam = T_CB[:3, 3]
    d = cam - axis_pt
    standoff = float(np.linalg.norm(d))
    el = float(np.degrees(np.arcsin(np.clip(float(d @ up) / standoff, -1, 1))))
    ref = np.array([1.0, 0.0, 0.0]); ref = ref - (ref @ up) * up
    ref /= np.linalg.norm(ref)
    e2 = np.cross(up, ref)
    az0 = float(np.degrees(np.arctan2(d @ e2, d @ ref)))
    print(f"\n  지금 자세 = 축 기준 el {el:.0f}° · 축거리 {standoff*1000:.0f}mm · 방위 {az0:.0f}°")

    cm = collision_gate(required=False)
    q_seed = np.asarray(robot.get_joint_angles(is_radian=True), float)[:7]
    out = []
    for daz in (float(x) for x in args.az_offsets.split(",")):
        if abs(daz) < 1e-6:
            continue
        q, _roll, _eye = vp.solve_view_q(
            kin, axis_pt, el, az0 + daz, standoff, q_seed, T_EC,
            convention=vp.CAM_OPENCV, up=up, az_ref=ref)
        if q is None:
            print(f"  [az {daz:+.0f}°] IK 실패 — 건너뜀")
            continue
        if cm is not None and not cm.is_pose_safe(q)[0]:
            print(f"  [az {daz:+.0f}°] 자세 충돌 — 건너뜀")
            continue
        q0 = np.asarray(robot.get_joint_angles(is_radian=True), float)[:7]
        way = plan_safe_joint_path(q0, q, cm) if cm is not None else [q]
        if way is None:
            print(f"  [az {daz:+.0f}°] 안전 경로 없음 — 건너뜀")
            continue
        robot.enable_motion()
        bad = False
        for w in way:
            # ★ xArm SDK: is_radian=True 이면 speed/mvacc 도 rad/s·rad/s² 로 해석한다 — deg 값을 그대로 넘기면 π rad/s(180°/s)로 클램프돼 설정과 무관하게 최고속이 된다(2026-09-22 발견).
            if robot.arm.set_servo_angle(angle=np.asarray(w).tolist(),
                                         speed=float(np.radians(args.speed)),
                                         mvacc=float(np.radians(args.speed * 4.0)),
                                         is_radian=True, wait=True):
                bad = True
                break
        if bad:
            print(f"  [az {daz:+.0f}°] 이동 실패 — 건너뜀")
            continue
        out.append(_collect(client, robot, T_EC, f"az {daz:+.0f}°"))
    return out


def _arc_span(P, center, normal):
    e1 = np.array([1.0, 0.0, 0.0]); e1 = e1 - np.dot(e1, normal) * normal
    e1 /= np.linalg.norm(e1); e2 = np.cross(normal, e1)
    w = P - center
    a = np.sort(np.degrees(np.arctan2(w @ e2, w @ e1)) % 360.0)
    return 360.0 - float(np.max(np.diff(np.r_[a, a[0] + 360.0])))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--no-home", dest="no_home", action="store_true",
                    help="추가 자세 없이 **지금 자세 한 장만** 찍는다")
    # ★ 실측(2026-09-21, 지금 셀·el 51°·축거리 223mm)에서 도달 확인한 조합.
    #   ±120° 중 +120° 는 IK 가 안 풀렸다 — 한쪽으로 치우쳐도 원주는 덮인다.
    #   도달 못 하는 방위는 건너뛰고 계속하므로 넉넉히 줘도 된다.
    ap.add_argument("--az-offsets", default="-40,40,-90,90,-120",
                    help="지금 방위에서 얼마씩 벌려 더 찍을지 (deg, 쉼표). "
                         "원주를 넓게 덮을수록 원이 잘 정해진다")
    ap.add_argument("--dry-run", action="store_true", help="결과만 보고 저장 안 함")
    ap.add_argument("--speed", type=float, default=20.0)
    a = ap.parse_args()

    from mms_artec.sensor.artec_client import ArtecClient, ArtecConfig
    from utils.robot.xarm_interface import XArmInterface

    T_EC = load_transform(str(_ROOT / "config" / "sensor_frames.yaml"), "T_EC_artec")
    robot = XArmInterface("192.168.1.210")
    client = ArtecClient(ArtecConfig()); client.initialize()
    chunks = []
    try:
        print("=" * 64)
        print("  rim 자동 검출 — 여러 자세의 외곽선을 합쳐 원을 맞춘다")
        print("=" * 64)
        chunks.append(_collect(client, robot, T_EC, "시작 자세"))

        if not a.no_home:
            # ★ **자세를 직접 만든다.** 예전엔 "현재 자세 + home" 이었는데, 실행
            #   시점에 로봇이 이미 home 에 있으면 같은 자리를 두 번 찍는다
            #   (2026-09-21 실측). 원주를 넓게 덮으려면 **축 둘레 방위를 벌려야**
            #   하므로, 지금 카메라의 (el, standoff) 는 유지한 채 az 만 바꾼다.
            chunks += _sweep_azimuths(client, robot, T_EC, a)
    finally:
        client.shutdown()

    P = np.vstack([c for c in chunks if len(c)])
    if len(P) < 50:
        print(f"\n✘ 점이 {len(P)}개뿐 — 원판 표면이 재구성되지 않았을 수 있다.")
        print("   민감도를 올리거나(turntable_calib --sensitivity), 조명을 확인할 것")
        return 1

    center, normal, radius, rms = fit_circle_3d(P)
    span = _arc_span(P, center, normal)
    print()
    print(f"  점 {len(P)}개 · 호 {span:.0f}°")
    print(f"  중심(mm) = {center.round(1)}")
    print(f"  반경     = {radius:.2f} mm   (실측 원판 119mm 근처여야 한다)")
    print(f"  잔차 RMS = {rms:.3f} mm")
    if span < MIN_ARC_DEG:
        print(f"  ⚠ 호가 {span:.0f}° 로 좁다 (<{MIN_ARC_DEG:.0f}°) — 자세를 더 벌려 다시 찍을 것.")
        print(f"     짧은 호는 **반경까지** 틀린다 (실측 102mm vs 참 119mm)")
    if abs(radius - 119.0) > 6.0:
        print(f"  ⚠ 반경이 실측(119mm)과 {abs(radius-119.0):.0f}mm 다르다 — 결과를 믿지 말 것")

    dbg = _ROOT / "output" / "debug"; dbg.mkdir(parents=True, exist_ok=True)
    import datetime as _dt
    f = dbg / f"rim_auto_{_dt.datetime.now():%H%M%S}.npz"
    np.savez_compressed(f, pts_B_mm=P, center_mm=center, normal=normal,
                        radius_mm=radius, rms_mm=rms, arc_span_deg=span)
    print(f"  점 저장 → {f.relative_to(_ROOT)}")

    if a.dry_run:
        print("\n  --dry-run — 저장하지 않는다")
        return 0
    if input("\n  T_B_F0 로 저장할까요? (y/N): ").strip().lower() != "y":
        print("  취소")
        return 0
    save_turntable_frame_yaml(
        OUT_YAML, build_T_B_F0(center / 1000.0, normal), n_points=len(P),
        radius_mm=radius, residual_mm=rms,
        extra={"method": "artec_auto_edge_circle_fit", "arc_span_deg": round(span, 1)})
    print(f"  저장 → {OUT_YAML}")
    print("  검증:  python scripts/artec/check_calibration.py")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
