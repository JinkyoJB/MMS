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
이미 안다**(`utils/collision/data/cell_env.npz`, `collision.md` §6). 이 값은
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
#: ★ **넓게** 준다. 좁히면 안 된다.
#
#  2026-09-16 실패 사례 — FOV 여유만 보고 (70,90,110) 로 좁혔다가 유효 자세가
#  81 → **9개**로 무너졌다. 손목이 그 roll 로는 대부분의 반구 자세에 못 간다
#  (IK 실패 196/225). FOV 여유를 벌려도 갈 수 있는 자세가 없으면 의미가 없다.
#
#  넓게 주면 roll 90°(FOV 에 유리)는 **갈 수 있는 자세에서만** 쓰이고 나머지는
#  다른 roll 이 메운다. 실측 비교(기준점 동일, 상한 20자세):
#     (-20,0,20)                    유효 81 → 상대회전 중앙값  92.6°
#     (-45,-20,0,20,45,70,90,110)   유효 ~130 → 중앙값 ~102°
#  roll 다양성은 AX=ZB 가 잘 풀리는 데 그 자체로 도움이 된다.
ROLLS_DEG = (-45.0, -20.0, 0.0, 20.0, 45.0, 70.0, 90.0, 110.0)
DIST_JITTER = (-0.02, 0.0, 0.02)
#: 카메라-보드 거리 (m). **조준 여유와 초점 품질의 맞교환**이다.
#
#     standoff   화각(가로×세로)    보드 84×60mm 중심 이탈 허용   Spider 초점
#       230mm     88 × 117mm            2mm                최적대역 안
#       250mm     96 × 130mm            6mm                최적대역 경계 ← 기본
#       270mm    104 × 138mm           10mm                대역 밖·양호
#       320mm    123 × 167mm           20mm                흐림
#       340mm    131 × 173mm           24mm                **작동거리 초과**
#
#   (화각은 실측 K 기준 가로 21.6° · 세로 28.6°. 2026-09-16 이전 주석은 이
#    가로/세로가 뒤집혀 있었다 — `artec_intrinsic.yaml` 로 다시 계산한 값이다.)
#
# ⚠ 2026-09-21 — **0.32 → 0.28 로 내렸다.**
#   0.32 는 조준 오차 여유를 벌려고 고른 값이었다(2026-09-15 계통 오차 32mm,
#   구 T_EC 회전 8.6°). 그런데 `DIST_JITTER` 가 붙으면 300/320/**340**mm 가
#   되고, Spider 작동거리 상한이 330mm 라 **절반이 범위 밖**이었다. 실측 결과:
#
#       20자세 중 작동거리 초과 10개 · 최적대역(200~250mm) 안 0개
#       → 초점 밖이라 3D 재구성 실패 (`reconstructAndTexturizeMesh 0x80070803`)
#         verts 5~169개, capture 실패 5 · 검출 실패 1 → intrinsic 산출 불가
#
#   여유가 필요 없어진 이유는 `--from-view` 다. 같은 `T_EC` 로 검출하고 같은
#   `T_EC` 로 겨누므로 계통 오차가 1차 상쇄된다 — 생성된 자세의 광축 이탈각이
#   실측 **0.0°** 였다. 조준이 맞으니 거리를 초점에 맞추는 것이 옳다.
#
#   왜 최적대역(200~250mm)까지 안 내리나 — **FOV 게이트가 자세를 걷어낸다.**
#   가까울수록 보드가 화면을 꽉 채워 조금만 비뚤어도 잘린다(실측 유효 자세:
#   0.32→108 · 0.28→42 · 0.25→**19** · 0.23→9). 20자세를 뽑으려면 0.28 이 한계다.
#   같은 로그의 거리별 성적이 0.28 을 뒷받침한다 (지터 포함 260/280/300mm):
#
#       300mm  verts 8,901 / 18,333 / 14,989   → 전부 양호
#       320mm  verts 16 ~ 10,498               → 들쭉날쭉
#       340mm  대부분 실패                      → 작동거리 밖
#
#   더 가까이 가려면 **보드를 줄여야 한다**(`spider_dense` 는 84×60mm).
#   셀 모델 기준(`--from-cell`)은 보드를 원판 한복판에 정확히 놓았을 때만
#   맞으므로 조준 여유가 더 필요하다 — 그때는 `--standoff 0.30` 을 줄 것.
STANDOFF_M = 0.28


def find_disc(npz: Path, hint_xy=None, band=(0.55, 1.05)):
    """셀 점군에서 **턴테이블 원판 상면**을 찾아 (center, normal, radius) 반환.

    base 프레임은 천장 마운트라 **+Z 가 아래**다(`collision.md` §6.1). 그래서
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
        #   (collision.md §6.1) 무엇이 턴테이블인지 데이터만으론 구별할 수 없다.
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
    from mms_artec.utils.calibration.artec_charuco_detector import (
        BOARD_PRESETS, DEFAULT_BOARD_NAME)
    spec = BOARD_PRESETS[DEFAULT_BOARD_NAME]
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


def spec_for_fov():
    """FOV 게이트가 쓸 보드 사양 — 캘리브와 **같은 프리셋**이어야 한다."""
    from mms_artec.utils.calibration.artec_charuco_detector import (
        BOARD_PRESETS, DEFAULT_BOARD_NAME)
    return BOARD_PRESETS[DEFAULT_BOARD_NAME]


def make_fov_gate(T_EC, center_B, normal_B, spec, margin_px: float = 40.0):
    """보드가 **프레임 안에 들어오는가**를 투영으로 판정하는 게이트를 만든다.

    왜 이게 필요한가
    ----------------
    Spider 의 텍스처 이미지는 **Color Camera 한 대**에서 나온다(3D 카메라 3대와 별개).
    그 카메라의 화각은 좁다 — 실측 K 기준 가로 21.8° / 세로 29.2° 로,
    standoff 320mm 에서 **123 × 167mm** 뿐이다. 84×60mm 보드를 넣으면 가로 여유가
    19.5mm 밖에 없어서, 조준이 조금만 밀려도 잘린다(2026-09-16: 15장 전부 2/4 모서리 밖).

    ★ 카메라가 플랜지에서 얼마나 떨어져 있는지는 **`T_EC` 가 이미 담고 있다.**
      `generate_hemisphere_poses` 는 카메라를 반구에 놓고 `T_W_E = T_W_C @ T_EC` 로
      EE 를 역산하므로, 별도로 translation 을 더하면 **이중 계산**이 된다.
      잘림의 원인은 offset 이 빠진 게 아니라 `T_EC` 의 **값이 낡은 것**이다.
      그래서 여기서는 offset 을 더하지 않고, 주어진 `T_EC` 로 **결과를 검사**한다.

    `--from-view` 와 함께 쓰면 낡은 `T_EC` 로도 의미가 있다 — 검출도 겨눔도 검사도
    모두 같은 `T_EC` 를 쓰므로 계통 오차가 1차 상쇄된다.

    intrinsic yaml 이 없으면 `None` 을 돌려준다(게이트 생략).
    """
    intr_path = _ROOT / "config" / "calibration" / "artec_intrinsic.yaml"
    if not intr_path.exists():
        return None
    d = yaml.safe_load(intr_path.read_text(encoding="utf-8")) or {}
    K = np.asarray(d["K"], float)
    dist = np.asarray(d.get("dist", [0] * 5), float).reshape(-1)
    W, H = (d.get("image_size") or [960, 1280])

    # 보드 4모서리를 base 좌표로. 법선에 수직인 두 축을 보드의 가로/세로로 삼는다.
    n = np.asarray(normal_B, float); n /= np.linalg.norm(n)
    u = np.cross(n, [1.0, 0.0, 0.0])
    if np.linalg.norm(u) < 1e-6:
        u = np.cross(n, [0.0, 1.0, 0.0])
    u /= np.linalg.norm(u)
    v = np.cross(n, u)
    bw = spec.squares_x * spec.square_length_mm / 2000.0     # 반폭 (m)
    bh = spec.squares_y * spec.square_length_mm / 2000.0
    c = np.asarray(center_B, float)
    corners_B = np.array([c + su * bw * u + sv * bh * v
                          for su in (-1, 1) for sv in (-1, 1)])

    import cv2

    def fits(T_WE) -> bool:
        """EE world pose 가 주어졌을 때 보드 4모서리가 전부 프레임 안인가."""
        T_WC = T_WE @ np.linalg.inv(T_EC)          # 카메라 world pose
        T_CW = np.linalg.inv(T_WC)
        P = (T_CW[:3, :3] @ corners_B.T).T + T_CW[:3, 3]      # 카메라 좌표 (m)
        if np.any(P[:, 2] <= 1e-3):                # 카메라 뒤 → 탈락
            return False
        px = cv2.projectPoints(P * 1000.0, np.zeros(3), np.zeros(3), K, dist)[0]
        px = px.reshape(-1, 2)
        return bool(np.all((px[:, 0] >= margin_px) & (px[:, 0] < W - margin_px) &
                           (px[:, 1] >= margin_px) & (px[:, 1] < H - margin_px)))

    print(f"  [fov] 게이트 활성 — 화각 "
          f"{2*np.degrees(np.arctan(W/2/K[0,0])):.1f}°×"
          f"{2*np.degrees(np.arctan(H/2/K[1,1])):.1f}°, "
          f"보드 {spec.squares_x*spec.square_length_mm:.0f}×"
          f"{spec.squares_y*spec.square_length_mm:.0f}mm, 여유 {margin_px:.0f}px")
    return fits


def _layout_tag() -> str:
    """경로 검사에 쓴 셀 레이아웃 별칭 — 레이아웃이 바뀌면 검사 결과도 무효다."""
    try:
        return (_ROOT / "utils" / "collision" / "data" / "ACTIVE_LAYOUT.txt"
                ).read_text(encoding="utf-8").strip() or "?"
    except OSError:
        return "?"


def order_poses(kept, cm, home_q):
    """자세를 **이동거리 최소 순서**로 재배열하고, 자세↔자세 경로를 검사한다.

    `via_home` 을 끄면 매 자세마다 home 을 왕복하지 않아 훨씬 빠르지만,
    `home→자세` 만 검사된 상태에서는 **자세i→자세j 경로가 미검증**이다.
    (2026-09-15 에 경로 미검사로 턴테이블과 충돌 직전까지 간 적이 있다.)

    그래서 greedy nearest-neighbour 로 순서를 정하면서 **연속 구간마다
    `is_path_safe` 를 통과하는 것만** 이어 붙인다. 통과 못 한 자세는 뒤로 미루고,
    끝까지 이어지지 않으면 그 자세는 버린다 — 버린 수를 알려준다.
    """
    if not kept:
        return kept, 0
    remaining = list(kept)
    ordered, cur, dropped = [], np.asarray(home_q, float), 0
    while remaining:
        # 현재 자세에서 관절공간 거리가 가까운 순으로 시도
        remaining.sort(key=lambda r: float(np.abs(np.radians(r[2]) - cur).max()))
        for idx, r in enumerate(remaining):
            q = np.radians(r[2])
            if cm is None:
                ok = True
            else:
                ok, _why, _n = cm.is_path_safe(cur, q)
            if ok:
                ordered.append(r)
                cur = q
                remaining.pop(idx)
                break
        else:
            # 남은 어느 자세로도 안전하게 못 간다 — 전부 버린다
            dropped += len(remaining)
            break
    return ordered, dropped


def main() -> int:
    ap = argparse.ArgumentParser(description="캘리브 자세 자동 생성 (반구)")
    ap.add_argument("--from-view", action="store_true",
                    help="셀 모델 대신 지금 보이는 ChArUco 로 기준을 잡는다")
    ap.add_argument("--center", nargs=3, type=float, metavar=("X", "Y", "Z"),
                    help="기준점을 직접 준다 (base, m)")
    ap.add_argument("--hint-xy", nargs=2, type=float, metavar=("X", "Y"),
                    help="원판 탐색 힌트 (base, m)")
    ap.add_argument("--standoff", type=float, default=STANDOFF_M)
    ap.add_argument("--rolls", nargs="+", type=float, default=list(ROLLS_DEG),
                    help="카메라 roll 후보 (deg). 도달성에 크게 영향 — 튜닝용")
    ap.add_argument("--no-fov-gate", action="store_true",
                    help="보드가 프레임에 들어오는지 투영 검사하는 게이트를 끈다")
    ap.add_argument("--fov-margin-px", type=float, default=40.0,
                    help="프레임 가장자리 여유 (px). 크게 주면 보수적")
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
    #   두 규약 차이는 카메라 로컬 Y축 180° 회전 하나뿐이다(`collision.md` §3).
    T_EC_usd = vp.FLIP_USD_TO_CV @ T_EC
    # `generate_hemisphere_poses` 는 roll·거리를 `k % len` 으로 **순환**시킨다 —
    # (polar, az) 하나당 roll 이 하나뿐이다. 게이트에서 절반 넘게 걸러지면 남는 게
    # 12개 미만이 되므로(실측 25→11), roll·거리마다 따로 불러 후보를 곱한다.
    # roll 다양성은 AX=ZB 가 잘 풀리는 데 그 자체로 도움이 된다.
    poses_T = []
    for roll in args.rolls:
        for dj in DIST_JITTER:
            poses_T += generate_hemisphere_poses(
                center, normal, T_EC_usd,
                distance_m=args.standoff, polars_deg=list(POLARS_DEG),
                azis_deg=list(AZIS_DEG), rolls_deg=[roll], dist_jitter=[dj])
    print(f"[gen] 후보 {len(poses_T)}개 생성 — IK·충돌 게이트로 거른다")

    cm = cmod.get_default()
    if cm is None:
        print("  ⚠ 충돌 모델 없음 — IK 만으로 거른다")

    fov_gate = None
    if not args.no_fov_gate:
        fov_gate = make_fov_gate(T_EC, center, normal, spec_for_fov(),
                                 margin_px=args.fov_margin_px)
        if fov_gate is None:
            print("  ⚠ artec_intrinsic.yaml 없음 — FOV 게이트 생략 "
                  "(잘림을 미리 못 거른다)")

    HOME = np.radians([0.0, -18.4, 0.0, 70.6, 0.0, 60.0, -45.0])   # artec home
    kept, n_ik, n_col, n_path, n_lim, n_fov = [], 0, 0, 0, 0, 0
    for i, T in enumerate(poses_T):
        if fov_gate is not None and not fov_gate(T):
            n_fov += 1
            continue
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
          f"(FOV 밖 {n_fov} · IK 실패 {n_ik} · 관절한계 {n_lim} · "
          f"자세충돌 {n_col} · 경로충돌 {n_path})")
    # 너무 많으면 고르게 솎는다 — 한 방향에 몰리지 않게 순서대로 건너뛴다.
    if len(kept) > args.max_poses:
        step = len(kept) / args.max_poses
        kept = [kept[int(i * step)] for i in range(args.max_poses)]
        print(f"[gen] {args.max_poses}개로 솎음")

    # ★ 이동거리 최소 순서로 재배열 + 자세↔자세 경로 검사.
    #   이게 통과해야 home 왕복 없이 이어서 가도 안전하다 (아래 path_checked).
    kept, n_unreach = order_poses(kept, cm, HOME)
    if n_unreach:
        print(f"[gen] 자세간 경로 미확보로 {n_unreach}개 제외")
    kept = [(f"hemi_{i:02d}", ee, q) for i, (_n, ee, q) in enumerate(kept)]
    steps = [float(np.abs(np.radians(kept[i + 1][2]) - np.radians(kept[i][2])).max())
             for i in range(len(kept) - 1)]
    if kept:
        print(f"[gen] 순서 최적화 — 자세간 최대관절이동 "
              f"중앙값 {np.degrees(np.median(steps)) if steps else 0:.0f}°  "
              f"(경로 검사 완료 — home 경유 없이 이어서 간다)")
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
    # ★ **경로 검사를 거쳤다는 사실을 남긴다.** `order_poses` 가 자세↔자세 경로를
    #   충돌 게이트로 확인하고 미확보분을 제외했으므로, 이 목록은 home 을 경유하지
    #   않고 이어서 가도 안전하다. 소비자(intrinsic/hand_eye)가 그걸 알아야
    #   home 경유 생략(기본)을 써도 되는지 스스로 판단할 수 있다. 표식이 없으면
    #   옛 목록(경로 무검사)일 수 있으므로 안전하게 home 을 경유한다.
    import datetime as _dt
    meta = {
        "path_checked": True,
        "generated": _dt.date.today().isoformat(),
        "layout": _layout_tag(),
        "max_step_deg": (round(float(np.degrees(np.median(steps))), 1)
                         if steps else None),
    }
    out.write_text(yaml.dump(
        {**meta,
         "poses": [{"name": n, "joints": q, "ee_pose": ee} for n, ee, q in kept]},
        default_flow_style=None, allow_unicode=True, sort_keys=False),
        encoding="utf-8")
    print(f"[gen] 저장: {out}  ({len(kept)} 자세)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
