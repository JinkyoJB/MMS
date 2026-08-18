"""view_pose.py — 관측자세(el/az/standoff) → 관절각 q. **sim·real 공용**.

왜 공용인가
----------
sim 과 real 이 같은 해석 IK(`xarm7_kinematics.ik`)를 쓰면서도 **부르는 방식이 달라서**
sim 에서 되는 자세가 real 에서 '도달 불가'로 버려지고 있었다:

    항목        sim(_view_q)         real(_axis_view_q)
    roll        6방향 시도            고정 1개
    IK 시드     8개                  1개
    카메라규약   USD(-Z 광축)          OpenCV(+Z 광축)

시뮬레이션으로 실물을 검증한다는 전제가 깨지므로, **자세를 만드는 로직을 여기 하나로**
모은다. 규약 차이는 **데이터(convention 인자)** 로 남긴다 — 실측 결과 두 규약은
카메라 로컬 **Y축 180° 회전** 하나만 다르고(모든 el/az 에서 편차 1.8e-12), 원점은 동일하다.
각 경로의 hand-eye(`T_EC`)가 이미 자기 규약과 짝이 맞으므로 **캘리브레이션은 건드리지 않는다.**

핵심 두 가지 (근거: docs/4_collision.md §1)
-----------------------------------------
1. **roll 은 자유 DOF** — 스캐너를 광축 둘레로 돌려도 같은 면을 본다. `look_at` 의
   up=(0,0,1) 은 그 자유도를 임의로 하나로 묶는다. 실측: 고정 시 el30 이 0/8,
   풀면 5/8. **시드를 12개로 늘려도 고정이면 0/8** → 시드로 대체 불가.
2. **DLS IK 는 국소해** — 시드에 따라 성공/실패가 갈린다. 실측: 수직 파지 0/8 ↔ 8/8.

roll=0·주어진 시드를 **먼저** 쓰므로 기존에 풀리던 자세의 해는 그대로다.
"""
from __future__ import annotations

import math

import numpy as np

CAM_USD = "usd"        # 광축 = 카메라 -Z, up = +Y   (USD/Isaac 카메라)
CAM_OPENCV = "opencv"  # 광축 = 카메라 +Z, y = down  (OpenCV / Artec hand-eye)

# T_WC_opencv = T_WC_usd @ FLIP_USD_TO_CV   (실측 확인, 자기 자신이 역행렬)
FLIP_USD_TO_CV = np.diag([-1.0, 1.0, -1.0, 1.0])

# 크기 순 — 0 을 먼저 쓰고 안 될 때만 최소 각도로 넓힌다.
DEFAULT_ROLLS_DEG = (0.0, -45.0, 45.0, -90.0, 90.0, 180.0)
DEFAULT_N_SEED_ALT = 7


def eye_from_el_az(target_w, el_deg, az_deg, standoff):
    """target 을 (el, az) 방향 standoff 거리에서 보는 카메라 원점."""
    e, a = math.radians(el_deg), math.radians(az_deg)
    return np.asarray(target_w, float) + float(standoff) * np.array(
        [math.cos(e) * math.cos(a), math.cos(e) * math.sin(a), math.sin(e)])


def camera_pose(eye, target, convention=CAM_USD, roll_deg=0.0, world_up=(0.0, 0.0, 1.0)):
    """카메라 → 기준프레임 4x4. `convention` 이 광축 규약을 결정한다."""
    from mms_artec.utils.calibration.handeye_geometry import (
        look_at_camera as _look, make_T as _mk)
    eye = np.asarray(eye, float)
    T = _mk(_look(eye, np.asarray(target, float), world_up), eye)   # USD 규약
    if convention == CAM_OPENCV:
        T = T @ FLIP_USD_TO_CV
    elif convention != CAM_USD:
        raise ValueError(f"알 수 없는 카메라 규약: {convention}")
    if abs(roll_deg) > 1e-9:
        c, s = math.cos(math.radians(roll_deg)), math.sin(math.radians(roll_deg))
        Rz = np.eye(4)
        Rz[:2, :2] = [[c, -s], [s, c]]          # 카메라 로컬 Z(광축) 둘레
        T = T @ Rz
    return T


def ik_seeds(seed, n_alt=DEFAULT_N_SEED_ALT, rng_seed=0):
    """주어진 시드 우선, 실패 시 결정적(재현 가능) 섭동 시드들."""
    s0 = np.asarray(seed, float)
    rng = np.random.default_rng(rng_seed)
    return [s0] + [s0 + rng.uniform(-1.0, 1.0, len(s0)) for _ in range(int(n_alt))]


def solve_view_q(kin, target_w, el_deg, az_deg, standoff, seed, T_EC, *,
                 T_WB=None, convention=CAM_USD, rolls_deg=DEFAULT_ROLLS_DEG,
                 n_seed_alt=DEFAULT_N_SEED_ALT, world_up=(0.0, 0.0, 1.0)):
    """(q, roll_deg, eye) — 못 풀면 (None, None, eye).

    target_w / T_WB 프레임 규칙
      · T_WB 를 주면 target 은 **world**, 내부에서 base 로 변환한다(sim).
      · T_WB=None 이면 target 이 이미 **base** 프레임이다(real).
    T_EC : E→C (카메라→플랜지). 호출자의 `convention` 과 짝이 맞는 값을 넘길 것.
    """
    eye = eye_from_el_az(target_w, el_deg, az_deg, standoff)
    seeds = ik_seeds(seed, n_seed_alt)
    for roll in rolls_deg:
        T_C = camera_pose(eye, target_w, convention, roll, world_up)
        if T_WB is not None:
            T_C = np.linalg.inv(np.asarray(T_WB, float)) @ T_C
        T_EB = T_C @ np.asarray(T_EC, float)
        pose6d = np.concatenate([T_EB[:3, 3] * 1000.0,
                                 kin.R_to_euler_xyz(T_EB[:3, :3])])
        for sd in seeds:
            q, ok = kin.ik(pose6d, seed=sd)
            if ok:
                return q, float(roll), eye
    return None, None, eye


def roll_order_for_gaps(gap_points, gap_weights, eye, target, convention=CAM_USD,
                        rolls_deg=DEFAULT_ROLLS_DEG, world_up=(0.0, 0.0, 1.0)):
    """gap 이 이미지 평면에서 뻗은 방향에 **FOV 넓은 축**을 맞추는 roll 순서.

    Spider FOV 는 비등방(hfov 30° / vfov 22.62°) — 0.25m 에서 발자국 134×100mm.
    긴 축을 부족면이 늘어선 방향에 맞추면 한 패스가 덮는 면이 넓어진다.
    ※ 효과는 A/B 로 확인할 것 — 이 프로젝트의 boundary 지표는 실행마다 흔들린다.
    """
    base = tuple(rolls_deg)
    try:
        P = np.asarray(gap_points, float)
        w = np.maximum(np.asarray(gap_weights, float), 1e-6)
        if len(P) < 2:
            return base
        T = camera_pose(eye, target, convention, 0.0, world_up)
        Pc = (P - np.asarray(eye, float)) @ T[:3, :3]      # → 카메라 로컬
        x = Pc[:, 0] - np.average(Pc[:, 0], weights=w)
        y = Pc[:, 1] - np.average(Pc[:, 1], weights=w)
        phi = math.degrees(0.5 * math.atan2(
            2.0 * float(np.average(x * y, weights=w)),
            float(np.average(x * x, weights=w)) - float(np.average(y * y, weights=w))))
        return tuple(sorted(base, key=lambda r: abs(((r - phi + 90.0) % 180.0) - 90.0)))
    except Exception:
        return base
