#!/usr/bin/env python
"""gen_calib_poses.py — 캘리브 자세 목록을 **자동 생성**한다 (teach mode 대체).

sim 이 쓰는 반구 생성기(`handeye_geometry.generate_hemisphere_poses`)를 그대로
real 에 올린 것이다. 기준점만 다르게 잡는다.

  --from-cell  (기본)  **충돌 셀 모델**에서 턴테이블 원판을 찾아 그 위 반구.
                       스캐너·T_EC 불필요 → **로봇 없이 오프라인 생성 가능.**
  --from-view          지금 로봇이 보고 있는 ChArUco 를 검출해 그 보드 위 반구.
                       구 T_EC 를 대략 추정으로 쓴다(빗나갈 때 폴백).

왜 셀 모델을 기준으로 잡나
--------------------------
보드는 턴테이블 원판 위에 놓인다. 그리고 원판이 base 어디 있는지는 **실측 셀 모델이
이미 안다**(`utils/collision/data/cell_env.npz`, `4_collision.md` §6). 이 값은
`T_EC` 와 완전히 독립이라, **지금 구하려는 값에 의존하지 않고** 자세를 만들 수 있다.

생성 후 **해석 IK + 충돌 게이트**로 거른다. 2026-09-15 FK 재교정으로 해석 모델이
실물과 1mm 안에서 맞으므로(`xarm7_dh.yaml`) 이 사전 필터를 믿을 수 있게 됐다.

    python scripts/artec/gen_calib_poses.py                    # 미리보기만
    python scripts/artec/gen_calib_poses.py --write            # yaml 저장
    python scripts/artec/gen_calib_poses.py --center 0.84 -0.02 0.69 --write

출력: `config/calibration/artec_calibration_poses.yaml`
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
import yaml

_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(_ROOT))

from mms_artec.utils.calibration.handeye_geometry import generate_hemisphere_poses
from utils.collision import collision_model as cmod
from utils.robot import view_pose as vp
from utils.robot import xarm7_kinematics as kin
from utils.transforms import load_transform

CELL_NPZ = _ROOT / "utils" / "collision" / "data" / "cell_env.npz"
SENSOR_FRAMES = _ROOT / "config" / "sensor_frames.yaml"
OUT_YAML = _ROOT / "config" / "calibration" / "artec_calibration_poses.yaml"

# sim 실측으로 고른 값 (utils/calibration/handeye_sim.py PoseConfig docstring):
#   (0,15,30,45) x 8방위 → 25 생성 / 10 게이트 제외 / 15 유효, t 0.95mm r 0.10°
POLARS_DEG = (0.0, 15.0, 30.0, 45.0)
AZIS_DEG = (0.0, 45.0, 90.0, 135.0, 180.0, 225.0, 270.0, 315.0)
ROLLS_DEG = (-20.0, 0.0, 20.0)
DIST_JITTER = (-0.02, 0.0, 0.02)
#: 조준 오차에 대한 **여유**로 정한 값이지 스캔 거리가 아니다.
#
#   Spider FOV hfov 30° / vfov 22.62° · 보드 100×60mm · 작동거리 170~350mm
#     standoff   화각(가로×세로)   보드 중심이 벗어나도 되는 여유
#       250mm      134×100mm        17 × 20mm
#       320mm      171×128mm        36 × 34mm     ← 기본값
#       350mm      188×140mm        44 × 40mm
#
# 2026-09-15 실측: 조준 계통 오차가 **32mm** 였다(구 T_EC 회전 8.6°).
# 250mm 의 17mm 여유로는 원리적으로 못 담아 20자세 중 18자세가 잘렸다.
# 거리를 올려도 도달성 손해는 거의 없다(유효 자세 93→85). 코너 정밀도는
# 보드가 프레임의 58% 를 차지하므로 subpixel 검출에 충분하다.
STANDOFF_M = 0.32


def find_disc(npz: Path, hint_xy=None, band=(0.55, 1.05)):
    """셀 점군에서 **턴테이블 원판 상면**을 찾아 (center, normal, radius) 반환.

    base 프레임은 천장 마운트라 **+Z 가 아래**다(`4_collision.md` §6.1). 그래서
    '상면' = 그 덩어리에서 z 가 **가장 작은** 쪽이다. 여기서 부호를 틀리면 반구가
    바닥을 향한다.
    """
    E = np.asarray(np.load(npz)["env"], float)
    m = (E[:, 2] > band[0]) & (E[:, 2] < band[1])
    if hint_xy is not None:
        m &= np.hypot(E[:, 0] - hint_xy[0], E[:, 1] - hint_xy[1]) < 0.25
    P = E[m]
    if len(P) < 500:
        raise RuntimeError(f"원판 후보 점이 부족하다 ({len(P)}). --center 로 직접 줄 것")

    # 얇은 수평층을 훑어 '원판다운' 층을 고른다 — 점이 많고 반경이 원판급(7~17cm).
    #
    # ⚠ 층 전체의 centroid 를 쓰면 안 된다. 그 높이에는 벽·상판 같은 다른 부재도
    #   있어서 중심이 셀 한복판으로 끌려가고 r95 가 1m 급이 된다(실측). 그래서
    #   층마다 **5cm 격자로 가장 조밀한 칸**을 먼저 찾아 그 주변만 본다.
    cands = []
    for z0 in np.arange(P[:, 2].min(), P[:, 2].max(), 0.004):
        S = P[(P[:, 2] >= z0) & (P[:, 2] < z0 + 0.008)]
        if len(S) < 400:
            continue
        gi = np.floor(S[:, :2] / 0.05).astype(int)
        _, inv, cnt = np.unique(gi, axis=0, return_inverse=True, return_counts=True)
        seed_xy = S[inv == int(np.argmax(cnt)), :2].mean(0)
        S = S[np.hypot(S[:, 0] - seed_xy[0], S[:, 1] - seed_xy[1]) < 0.20]
        if len(S) < 400:
            continue
        c = S[:, :2].mean(0)
        r95 = float(np.percentile(np.hypot(S[:, 0] - c[0], S[:, 1] - c[1]), 95))
        if not (0.07 <= r95 <= 0.17):
            continue
        cands.append((len(S), z0, c, r95))

    if not cands:
        raise RuntimeError("원판다운 수평층을 못 찾았다. --center 로 직접 줄 것")

    # xy 로 뭉쳐 대표 하나씩만 남긴다 (같은 부재의 여러 층이 중복으로 잡힌다).
    uniq = []
    for n_pts, z0, c, r95 in sorted(cands, key=lambda t: -t[0]):
        if any(np.hypot(c[0] - u[2][0], c[1] - u[2][1]) < 0.15 for u in uniq):
            continue
        uniq.append((n_pts, z0, c, r95))

    if hint_xy is None:
        # ★ 자동으로 고르지 않는다. 셀 점군은 부재 이름이 전부 'mesh' 라
        #   (4_collision.md §6.1) 무엇이 턴테이블인지 데이터만으론 구별할 수 없다.
        #   실측에서 자동 선택이 **키보드**를 집은 적이 있다.
        print("\n  원판 후보 (base 프레임):")
        for n_pts, z0, c, r95 in uniq[:8]:
            print(f"    --hint-xy {c[0]:.3f} {c[1]:.3f}   "
                  f"상면 z={z0:.3f}  r95={r95*1000:3.0f}mm  점 {n_pts:,}")
        raise SystemExit(
            "\n  위에서 턴테이블을 골라 --hint-xy 로 주거나, --center 로 직접 줄 것.\n"
            "  어느 것인지 모르겠으면:  $ISAAC scripts\\sim\\view_scene.py v2")

    n_pts, z0, c, r95 = min(
        uniq, key=lambda t: float(np.hypot(t[2][0] - hint_xy[0], t[2][1] - hint_xy[1])))
    center = np.array([c[0], c[1], z0], float)
    # 법선은 원판에서 **로봇 쪽**(base 원점 방향)을 향해야 카메라가 위에서 내려다본다.
    normal = np.array([0.0, 0.0, -1.0]) if center[2] > 0 else np.array([0.0, 0.0, 1.0])
    print(f"  [disc] 상면 z={z0:.3f}  중심=({c[0]:+.3f},{c[1]:+.3f})  "
          f"r95={r95*1000:.0f}mm  점 {n_pts:,}")
    return center, normal, r95


def board_from_view(T_EC):
    """지금 로봇이 보고 있는 ChArUco 를 검출해 **보드 중심·법선(base)** 을 돌려준다.

    셀 모델의 원판 중심을 겨누면 보드가 거기 없을 수 있다 — 보드를 원판 한복판에
    정확히 놓지 않았거나, 구 `T_EC` 의 회전 오차(5°면 250mm 에서 22mm)가 얹히면
    Spider 의 좁은 FOV(0.25m 에서 134×100mm) 밖으로 보드가 잘려 나간다.
    2026-09-15 실측: 20자세 중 18자세가 `charuco 부족` 으로 버려졌고, 거리는
    평균 262mm 로 정상이었다 — 즉 **거리가 아니라 조준**의 문제였다.

    ★ 여기서 추정한 중심을 **같은 T_EC 로** 다시 겨누므로, T_EC 의 계통 오차는
      1차적으로 상쇄된다. T_EC 가 낡아도 이 경로가 동작하는 이유다.

    호출 전에 보드가 화면 **중앙에** 오도록 로봇을 맞춰 둘 것.
    """
    import cv2
    from mms_artec.sensor.artec_client import ArtecClient, ArtecConfig
    from mms_artec.utils.calibration.artec_charuco_detector import (
        ArtecCharucoDetector, CharucoBoardSpec)
    from utils.robot.xarm_interface import XArmInterface
    from utils.transforms import compute_T_CB

    intr_path = _ROOT / "config" / "calibration" / "artec_intrinsic.yaml"
    intrinsic = None
    if intr_path.exists():
        d = yaml.safe_load(intr_path.read_text(encoding="utf-8")) or {}
        intrinsic = {"K": d["K"], "dist": d.get("dist", [0] * 5),
                     "image_size": d.get("image_size")}
    spec = CharucoBoardSpec(5, 3, 20.0, 15.0, cv2.aruco.DICT_4X4_50)   # spider 프리셋
    detector = ArtecCharucoDetector(spec, intrinsic=intrinsic)

    robot = XArmInterface(ip="192.168.1.210")
    sensor = ArtecClient(ArtecConfig(serial_number=None, capture_texture=True))
    try:
        sensor.initialize()
        T_EB = robot.get_ee_pose_mat()
        fmh = sensor.capture_frame(capture_texture=True)
        if fmh is None or not fmh.has_image() or not fmh.is_textured():
            raise RuntimeError("텍스처 캡처 실패")
        det = detector.detect(fmh.image(), fmh.vertices(), fmh.uv())
        if det is None:
            raise RuntimeError("ChArUco 미검출 — 보드가 화면 중앙에 오도록 조준할 것")
        print(f"  [from-view] corners={det.n_corners}  "
              f"t_MC=({det.T_MC[0,3]:.0f},{det.T_MC[1,3]:.0f},{det.T_MC[2,3]:.0f})mm")
        T_CB = compute_T_CB(T_EB, T_EC)
        c_B = (T_CB @ np.append(det.T_MC[:3, 3] / 1000.0, 1.0))[:3]
        n_B = T_CB[:3, :3] @ det.T_MC[:3, 2]
        n_B = n_B / np.linalg.norm(n_B)
        # 법선은 로봇 쪽(=base 원점 방향)을 향해야 카메라가 내려다본다.
        if float(n_B @ (np.zeros(3) - c_B)) < 0:
            n_B = -n_B
        return c_B, n_B
    finally:
        try:
            sensor.shutdown()
        except Exception:
            pass
        robot.disconnect()


def main() -> int:
    ap = argparse.ArgumentParser(description="캘리브 자세 자동 생성 (반구)")
    ap.add_argument("--from-view", action="store_true",
                    help="셀 모델 대신 지금 보이는 ChArUco 로 기준을 잡는다")
    ap.add_argument("--center", nargs=3, type=float, metavar=("X", "Y", "Z"),
                    help="기준점을 직접 준다 (base, m)")
    ap.add_argument("--hint-xy", nargs=2, type=float, metavar=("X", "Y"),
                    help="원판 탐색 힌트 (base, m)")
    ap.add_argument("--standoff", type=float, default=STANDOFF_M)
    ap.add_argument("--max-poses", type=int, default=20,
                    help="최대 자세 수 (넘으면 고르게 솎는다)")
    ap.add_argument("--min-joint-margin", type=float, default=5.0,
                    help="관절 한계까지 최소 여유 (deg). 한계에 붙은 해를 버린다")
    ap.add_argument("--write", action="store_true", help="yaml 저장 (없으면 미리보기)")
    ap.add_argument("--out", default=str(OUT_YAML))
    args = ap.parse_args()

    T_EC = load_transform(str(SENSOR_FRAMES), "T_EC_artec")
    print(f"[gen] T_EC |t|={np.linalg.norm(T_EC[:3,3])*1000:.1f}mm  "
          f"(반구를 겨누는 용도 — 정밀할 필요 없다)")

    if args.center:
        center = np.asarray(args.center, float)
        normal = np.array([0.0, 0.0, -1.0]) if center[2] > 0 else np.array([0.0, 0.0, 1.0])
        print(f"[gen] 기준점 직접 지정 {np.round(center,3).tolist()}")
    elif args.from_view:
        center, normal = board_from_view(T_EC)
        print(f"[gen] ChArUco 기준 {np.round(center,3).tolist()}")
    else:
        center, normal, _ = find_disc(CELL_NPZ, args.hint_xy)
        print(f"[gen] 셀 모델 기준 {np.round(center,3).tolist()}")
    print(f"[gen] 법선 {np.round(normal,3).tolist()}   standoff {args.standoff:.3f}m")

    # ★ 규약 변환이 반드시 필요하다. `generate_hemisphere_poses` 는 내부에서
    #   `look_at_camera`(**USD 규약**: 광축 = 카메라 −Z)로 카메라를 세운 뒤 T_EC 를
    #   곱한다. 그런데 `T_EC_artec` 는 hand-eye 와 짝인 **OpenCV 규약**(광축 = +Z)이다.
    #   그대로 넣으면 실제 카메라가 타깃을 **정확히 180° 등지고** 선다(실측 확인).
    #   sim 은 T_EC_gt 가 USD 규약이라 이 문제가 없었다 — real 로 올리며 드러난 것.
    #   두 규약 차이는 카메라 로컬 Y축 180° 회전 하나뿐이다(`4_collision.md` §3).
    T_EC_usd = vp.FLIP_USD_TO_CV @ T_EC
    # `generate_hemisphere_poses` 는 roll·거리를 `k % len` 으로 **순환**시킨다 —
    # (polar, az) 하나당 roll 이 하나뿐이다. 게이트에서 절반 넘게 걸러지면 남는 게
    # 12개 미만이 되므로(실측 25→11), roll·거리마다 따로 불러 후보를 곱한다.
    # roll 다양성은 AX=ZB 가 잘 풀리는 데 그 자체로 도움이 된다.
    poses_T = []
    for roll in ROLLS_DEG:
        for dj in DIST_JITTER:
            poses_T += generate_hemisphere_poses(
                center, normal, T_EC_usd,
                distance_m=args.standoff, polars_deg=list(POLARS_DEG),
                azis_deg=list(AZIS_DEG), rolls_deg=[roll], dist_jitter=[dj])
    print(f"[gen] 후보 {len(poses_T)}개 생성 — IK·충돌 게이트로 거른다")

    cm = cmod.get_default()
    if cm is None:
        print("  ⚠ 충돌 모델 없음 — IK 만으로 거른다")

    HOME = np.radians([0.0, -18.4, 0.0, 70.6, 0.0, 60.0, -45.0])   # artec home
    kept, n_ik, n_col, n_path, n_lim = [], 0, 0, 0, 0
    for i, T in enumerate(poses_T):
        p6 = np.concatenate([T[:3, 3] * 1000.0, kin.R_to_euler_xyz(T[:3, :3])])
        q, ok = kin.ik(p6, seed=HOME)
        if not ok:
            n_ik += 1
            continue
        q = np.asarray(q, float)
        # ★ 관절 한계에 **붙은** 해를 버린다. `kin.ik` 는 해를 JOINT_LOWER/UPPER 로
        #   clip 한 뒤에도 ok=True 를 돌려주므로, 한계에 정확히 얹힌 자세가 통과한다
        #   (실측: 16개 중 4개가 여유 0.00°). 그런 자세는 컨트롤러가 거부하거나
        #   (`set_servo_angle` code 10) 서보가 안착할 여유가 없다.
        margin = float(np.minimum(q - kin.JOINT_LOWER, kin.JOINT_UPPER - q).min())
        if margin < np.radians(args.min_joint_margin):
            n_lim += 1
            continue
        if cm is not None:
            safe, _why = cm.is_pose_safe(q)
            if not safe:
                n_col += 1
                continue
            # ★ 자세만 안전해도 **가는 길**에서 부딪힌다. hand_eye_calib 은 자세마다
            #   home 을 경유하므로(via_home) home→자세 구간을 검사해야 한다.
            #   2026-09-15 실물에서 이 검사가 없어 첫 자세 이동 중 턴테이블과
            #   충돌 직전까지 갔다(비상정지). hemi_02·hemi_05 는 mid(link6) 로 걸린다.
            okp, _whyp, _n = cm.is_path_safe(HOME, q)
            if not okp:
                n_path += 1
                continue
        kept.append((f"hemi_{len(kept):02d}",
                     [round(float(v), 2) for v in
                      np.concatenate([p6[:3], np.degrees(p6[3:6])])],
                     [round(float(v), 4) for v in np.degrees(q)]))

    print(f"[gen] 유효 {len(kept)}개  "
          f"(IK 실패 {n_ik} · 관절한계 {n_lim} · 자세충돌 {n_col} · 경로충돌 {n_path})")
    # 너무 많으면 고르게 솎는다 — 한 방향에 몰리지 않게 순서대로 건너뛴다.
    if len(kept) > args.max_poses:
        step = len(kept) / args.max_poses
        kept = [kept[int(i * step)] for i in range(args.max_poses)]
        kept = [(f"hemi_{i:02d}", ee, q) for i, (_n, ee, q) in enumerate(kept)]
        print(f"[gen] {args.max_poses}개로 솎음")
    if len(kept) < 12:
        print("  ⚠ 12개 미만 — hand-eye 가 잘 안 풀린다. standoff·기준점을 바꿔 볼 것")

    for name, ee, q in kept:
        print(f"    {name}  ee={ee}")

    if not args.write:
        print("\n[gen] 미리보기만 했다. 저장하려면 --write")
        return 0
    if len(kept) < 8:
        print("\n[gen] 8개 미만이라 저장하지 않는다.")
        return 1

    out = Path(args.out)
    if out.exists():
        bak = out.with_suffix(".yaml.bak")
        bak.write_text(out.read_text(encoding="utf-8"), encoding="utf-8")
        print(f"[gen] 기존 파일 백업 → {bak.name}")
    # ★ **joints 를 쓴다.** ee_pose 만 쓰면 hand_eye_calib 이 그걸
    #   `arm.get_inverse_kinematics`(**컨트롤러 IK**)로 다시 풀어서 간다. 7축이라
    #   같은 TCP 에 해가 무한히 많고, 실측에서 해석 IK 와 **118~263° 다른 자세**가
    #   나왔다 — 즉 여기서 검증한 자세와 로봇이 실제로 가는 자세가 달랐다.
    #   joints 키가 있으면 `_run_scripted` 가 그대로 set_servo_angle 한다.
    #   ee_pose 는 참고용으로 같이 남긴다(사람이 읽기 위해).
    out.write_text(yaml.dump(
        {"poses": [{"name": n, "joints": q, "ee_pose": ee} for n, ee, q in kept]},
        default_flow_style=None, allow_unicode=True), encoding="utf-8")
    print(f"[gen] 저장: {out}  ({len(kept)} 자세)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
