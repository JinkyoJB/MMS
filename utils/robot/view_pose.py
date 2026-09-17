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

핵심 두 가지 (근거: docs/collision.md §1)
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

# ── 특이점 회피 ──────────────────────────────────────────────────────────────
# σ_min = 단위정규화 자코비안의 최소 특이값 = **특이점까지의 거리**.
# 여기서 거르지 않으면 IK 가 조건 나쁜 해를 돌려주고, 그 자세에서
#   · 작은 카테시안 오차가 큰 관절 이동으로 증폭되고(떨림·오버슈트)
#   · 경로 보간 중 관절이 급변한다.
# 실측(xArm7, 무작위 1500자세): 최소 0.0004 / 1% 0.0020 / 5% 0.0076 / 중앙 0.069
#   home 0.147,  팔꿈치 신전(q4=0°) 0.087,  q4=140° 0.036(최악권)
# → 0.05 = 하위 ~4% 를 자르는 값. home 대비 3배 여유.
# ※ 7축 여유자유도라 6축 팔의 '손목 특이점(q5=0)' 은 실제로 특이하지 않다(실측 σ_min
#   0.16). 남는 축이 보상하므로, 관절각 규칙이 아니라 **σ_min 으로 판정**해야 한다.
DEFAULT_MIN_SIGMA = 0.05


def eye_from_el_az(target_w, el_deg, az_deg, standoff, up=(0.0, 0.0, 1.0),
                   az_ref=None):
    """target 을 (el, az) 방향 standoff 거리에서 보는 카메라 원점.

    ★ `up` 은 **el 의 기준축**이다 — el=+90° 면 카메라가 이 방향에 놓인다.
      기본 (0,0,1) 은 **world 프레임**(sim)에서 맞다.

      real 은 다르다. `_axis_view_q` 가 base 프레임을 그대로 넘기는데,
      이 셀의 base 는 **천장 마운트라 +Z 가 아래**다(`docs/collision.md` §6.1).
      기본값을 쓰면 el=+30° 가 카메라를 원판 **아래**로 보낸다 — 2026-09-16 실물에서
      eye 가 base 로부터 1.38m(도달반경 0.70m)에 찍혀 3방위 IK 가 전부 실패했고,
      "sim 에서는 되는데 real 에서만 안 되는" 증상으로 나타났다.
      real 은 턴테이블 축이 **로봇을 향하는 방향**을 `up` 으로 넘겨야 한다.

      `gen_calib_poses.find_disc` 주석이 같은 함정을 경고한다 —
      "여기서 부호를 틀리면 반구가 바닥을 향한다".
    """
    e, a = math.radians(el_deg), math.radians(az_deg)
    n = np.asarray(up, float)
    n = n / (np.linalg.norm(n) + 1e-12)
    # n 에 수직인 정규직교 기저 — az=0 의 기준 방향.
    # ★ Gram-Schmidt 로 잡는다. `cross(n, ref)` 를 쓰면 up=(0,0,1) 일 때 u1 이
    #   (0,1,0) 이 되어 **방위각이 90° 돌아간다** — 기존 sim 동작과 달라진다.
    #   아래 식은 up=(0,0,1) 에서 u1=(1,0,0), u2=(0,1,0) 이라 옛 공식과 정확히 같다.
    #
    # `az_ref` 를 주면 그 방향이 az=0 이 된다. real 은 **로봇 쪽**을 넘긴다 —
    # 방위각은 커버리지와 무관하고(회전은 턴테이블 담당) 도달성만 좌우하는데,
    # 타깃이 팔 길이 밖(base 에서 1.09m, 반경 0.70m)이면 카메라를 base 쪽에
    # 놓아야만 닿는다. 2026-09-16 실측: az 0/±30° 는 전부 도달 불가,
    # 105~270° 만 해가 있었고 180°(= 로봇 쪽)가 최적이었다.
    ref = np.asarray(az_ref, float) if az_ref is not None else np.array([1.0, 0.0, 0.0])
    if abs(float(n @ (ref / (np.linalg.norm(ref) + 1e-12)))) > 0.9:
        ref = np.array([0.0, 1.0, 0.0]) if az_ref is None else np.cross(n, [1.0, 0.0, 0.0])
    u1 = ref - float(ref @ n) * n
    u1 /= (np.linalg.norm(u1) + 1e-12)
    u2 = np.cross(n, u1)
    d = math.cos(e) * (math.cos(a) * u1 + math.sin(a) * u2) + math.sin(e) * n
    return np.asarray(target_w, float) + float(standoff) * d


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


def solve_look_at_q(kin, eye, target, seed, T_EC, *,
                    T_WB=None, convention=CAM_USD, rolls_deg=DEFAULT_ROLLS_DEG,
                    n_seed_alt=DEFAULT_N_SEED_ALT, world_up=(0.0, 0.0, 1.0),
                    min_sigma=DEFAULT_MIN_SIGMA, ik_max_iter=None):
    """(q, roll_deg) — **카메라 위치와 겨냥점을 직접** 주는 경로. 못 풀면 (None, None).

    `solve_view_q` 는 '축을 el/az/standoff 에서 본다'는 규칙이라 **오목·내부 면을
    겨냥할 수 없다**(카메라가 늘 물체 바깥 구면에 놓인다). 컵 내벽처럼 법선이 안쪽을
    향하는 면은 표면점 기준으로 카메라를 놓아야 하므로 이 함수를 쓴다.
    """
    eye = np.asarray(eye, float)
    target = np.asarray(target, float)
    # ★ 명백히 도달 불가한 타깃은 IK 를 **부르지 않는다.** 수치 IK 는 실패해도
    #   max_iter 를 다 돌아서, 아래 roll×seed 스윕이면 한 번에 2.8초가 걸린다
    #   (실측 2026-09-16). NBV gap 겨냥은 후보가 수백 개라 14분간 무응답이 된다.
    #   플랜지는 eye 에서 |t_EC| 이내에 있으므로 `|eye| - |t_EC| > 도달상한` 이면
    #   **어떤 roll 로도** 못 풀린다 — 보수적이라 놓치는 해가 없다.
    #   (T_WB 가 있으면 eye 가 base 프레임이 아니므로 이 검사를 건너뛴다.)
    _reach = getattr(kin, "MAX_REACH_M", None)
    if _reach is not None and T_WB is None:
        _d_ec = float(np.linalg.norm(np.asarray(T_EC, float)[:3, 3]))
        if float(np.linalg.norm(eye)) - _d_ec > float(_reach):
            return None, None
    seeds = ik_seeds(seed, n_seed_alt)
    for roll in rolls_deg:
        T_C = camera_pose(eye, target, convention, roll, world_up)
        if T_WB is not None:
            T_C = np.linalg.inv(np.asarray(T_WB, float)) @ T_C
        T_EB = T_C @ np.asarray(T_EC, float)
        pose6d = np.concatenate([T_EB[:3, 3] * 1000.0,
                                 kin.R_to_euler_xyz(T_EB[:3, :3])])
        for sd in seeds:
            # ik_max_iter — 후보 **스크리닝**용 예산. NBV gap 겨냥처럼 실패가
            # 대부분인 대량 평가에서 기본 200 반복을 다 돌면 실패 1건에 수 초씩
            # 든다. 수렴할 해는 초반에 수렴하므로 예산을 줄여도 해를 거의 안
            # 놓친다 (None = kin 기본값, 기존 동작).
            if ik_max_iter is not None:
                q, ok = kin.ik(pose6d, seed=sd, max_iter=int(ik_max_iter))
            else:
                q, ok = kin.ik(pose6d, seed=sd)
            if not ok:
                continue
            if min_sigma > 0.0:
                try:
                    if kin.sigma_min(q) < min_sigma:
                        continue
                except Exception:
                    pass
            return q, float(roll)
    return None, None


def solve_view_q(kin, target_w, el_deg, az_deg, standoff, seed, T_EC, *,
                 T_WB=None, convention=CAM_USD, rolls_deg=DEFAULT_ROLLS_DEG,
                 n_seed_alt=DEFAULT_N_SEED_ALT, world_up=(0.0, 0.0, 1.0),
                 min_sigma=DEFAULT_MIN_SIGMA, up=None, az_ref=None):
    """(q, roll_deg, eye) — target 을 (el, az, standoff) 에서 보는 자세. 실패 시 (None, None, eye).

    카메라가 **물체 바깥 구면**에 놓이는 규칙이라 외부 표면 전용이다. 내부·오목면은
    `solve_look_at_q` 를 쓸 것.

    `up` 은 **el 의 기준축**(기본 = `world_up`). 천장 마운트 base 프레임처럼 +Z 가
    아래인 좌표계에서는 반드시 로봇 쪽 방향을 줘야 한다 — `eye_from_el_az` 참고.
    """
    eye = eye_from_el_az(target_w, el_deg, az_deg, standoff,
                         up=(world_up if up is None else up), az_ref=az_ref)
    q, roll = solve_look_at_q(kin, eye, target_w, seed, T_EC, T_WB=T_WB,
                              convention=convention, rolls_deg=rolls_deg,
                              n_seed_alt=n_seed_alt, world_up=world_up,
                              min_sigma=min_sigma)
    return q, roll, eye


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
