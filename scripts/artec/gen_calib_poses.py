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
STANDOFF_M = 0.25


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
    """지금 로봇이 보고 있는 ChArUco 를 검출해 보드 중심·법선(base) 추정. 폴백 경로."""
    from mms_artec.sensor.artec_client import ArtecClient, ArtecConfig
    from utils.calibration.artec_charuco_detector import ArtecCharucoDetector
    from utils.robot.xarm_interface import XArmInterface
    from utils.transforms import compute_T_CB

    robot = XArmInterface("192.168.1.210")
    sensor = ArtecClient(ArtecConfig(serial_number=None, capture_texture=True))
    try:
        sensor.initialize()
        T_EB = robot.get_ee_pose_mat()
        det = ArtecCharucoDetector()
        T_MC = det.detect_board_pose(sensor)          # mm, C frame
        if T_MC is None:
            raise RuntimeError("ChArUco 미검출 — 보드가 보이게 조준할 것")
        T_CB = compute_T_CB(T_EB, T_EC)
        c_B = (T_CB @ np.append(T_MC[:3, 3] / 1000.0, 1.0))[:3]
        n_B = T_CB[:3, :3] @ T_MC[:3, 2]
        return c_B, n_B / np.linalg.norm(n_B)
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

    poses_T = generate_hemisphere_poses(
        center, normal, T_EC,
        distance_m=args.standoff, polars_deg=list(POLARS_DEG),
        azis_deg=list(AZIS_DEG), rolls_deg=list(ROLLS_DEG),
        dist_jitter=list(DIST_JITTER))
    print(f"[gen] 후보 {len(poses_T)}개 생성 — IK·충돌 게이트로 거른다")

    cm = cmod.get_default()
    if cm is None:
        print("  ⚠ 충돌 모델 없음 — IK 만으로 거른다")

    seed = np.radians([0.0, -18.4, 0.0, 70.6, 0.0, 60.0, -45.0])   # artec home
    kept, n_ik, n_col = [], 0, 0
    for i, T in enumerate(poses_T):
        p6 = np.concatenate([T[:3, 3] * 1000.0, kin.R_to_euler_xyz(T[:3, :3])])
        q, ok = kin.ik(p6, seed=seed)
        if not ok:
            n_ik += 1
            continue
        if cm is not None:
            safe, _why = cm.is_pose_safe(np.asarray(q, float))
            if not safe:
                n_col += 1
                continue
        kept.append((f"hemi_{len(kept):02d}",
                     [round(float(v), 2) for v in
                      np.concatenate([p6[:3], np.degrees(p6[3:6])])]))

    print(f"[gen] 유효 {len(kept)}개  (IK 실패 {n_ik} · 충돌 {n_col})")
    if len(kept) < 12:
        print("  ⚠ 12개 미만 — hand-eye 가 잘 안 풀린다. standoff·기준점을 바꿔 볼 것")

    for name, ee in kept:
        print(f"    {name}  {ee}")

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
    out.write_text(yaml.dump(
        {"poses": [{"name": n, "ee_pose": ee} for n, ee in kept]},
        default_flow_style=None, allow_unicode=True), encoding="utf-8")
    print(f"[gen] 저장: {out}  ({len(kept)} 자세)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
