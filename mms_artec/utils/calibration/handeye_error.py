"""handeye_error.py — sim 에서 **hand-eye 캘리브 오차를 의도적으로 주입**한다.

왜 필요한가
----------
sim 은 `T_EC` 를 USD ground truth 로 덮어쓰고, 게다가 `capture_points_base` 가
**T_EC 를 아예 쓰지 않고** Isaac 카메라의 실제 포즈로 점군을 만든다. 그래서 sim 은

    계획(IK)   재구성(점군 변환)
    real  캘리브값(오차有)   캘리브값 → **오차가 점군에 누적**
    sim   GT               카메라 실제 포즈 → **오차 0**

즉 **오차가 가장 아픈 곳(다시점 정합)** 을 한 번도 시험하지 않는다. 실물 hand-eye 는
아무리 잘 잡아도 수 mm·1° 수준의 오차가 남으므로, "sim 에서 됐으니 실물도 된다"는
보장이 성립하지 않는다.

모델
----
파이프라인이 **믿는** 값 `T_EC_belief = T_EC_true ⊕ error` 를 계획·재구성에 쓰게 하고,
실제 카메라는 여전히 `T_EC_true` 에 있다. 실물과 같은 구조다.

재구성 오차는 자세에 따라 달라진다(팔이 움직이면 오차 방향도 회전) —
base 프레임 보정변환은 `D = FK(q)·inv(T_EC_belief) · [FK(q)·inv(T_EC_true)]^-1` 로
q 에 의존하며, 이 q 의존성이 바로 다시점 불일치를 만든다.

사용 (기본값 0 = 무주입, 기존 동작 그대로)
    MMS_SIM_TEC_ERR_MM=5 MMS_SIM_TEC_ERR_DEG=1 python main_artec.py
    MMS_SIM_TEC_SEED=3   # 같은 크기의 다른 방향 오차
"""
from __future__ import annotations

import math
import os

import numpy as np


def _envf(k: str, d: float) -> float:
    try:
        return float(os.environ.get(k, d))
    except ValueError:
        return d


def error_spec():
    """(mm, deg, seed). mm==deg==0 이면 주입 없음."""
    return (_envf("MMS_SIM_TEC_ERR_MM", 0.0),
            _envf("MMS_SIM_TEC_ERR_DEG", 0.0),
            int(_envf("MMS_SIM_TEC_SEED", 0.0)))


def enabled() -> bool:
    mm, deg, _ = error_spec()
    return abs(mm) > 1e-9 or abs(deg) > 1e-9


def perturbation(mm: float, deg: float, seed: int) -> np.ndarray:
    """크기가 정확히 (mm, deg) 인 **결정적** 오차 변환 4x4."""
    rng = np.random.default_rng(seed)
    t = rng.normal(size=3)
    t = t / (np.linalg.norm(t) + 1e-12) * (mm / 1000.0)
    a = rng.normal(size=3)
    a = a / (np.linalg.norm(a) + 1e-12)
    th = math.radians(deg)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    R = np.eye(3) + math.sin(th) * K + (1.0 - math.cos(th)) * (K @ K)  # Rodrigues
    E = np.eye(4)
    E[:3, :3] = R
    E[:3, 3] = t
    return E


def believed_T_EC(T_EC_true, log: bool = True):
    """파이프라인이 **믿을** hand-eye. 주입이 꺼져 있으면 원본을 그대로 돌려준다."""
    if T_EC_true is None or not enabled():
        return T_EC_true
    mm, deg, seed = error_spec()
    T = np.asarray(T_EC_true, float) @ perturbation(mm, deg, seed)
    if log:
        d = float(np.linalg.norm(T[:3, 3] - np.asarray(T_EC_true)[:3, 3])) * 1000.0
        print(f"[handeye_error] ★ sim T_EC 에 캘리브 오차 주입: "
              f"{mm:.1f}mm / {deg:.2f}° (seed={seed}) → 실제 이동 {d:.2f}mm")
        print(f"[handeye_error]   계획·재구성 모두 이 '믿는 값'을 쓴다 "
              f"(실제 카메라는 GT 위치 그대로)")
    return T


def base_correction(q, T_EC_true, T_EC_belief, kin):
    """GT 기준 base 점군 → **믿는 값 기준** base 점군 변환 D (4x4).

    실물은 카메라 프레임 점군을 캘리브값으로 base 에 올린다. sim 은 이미 GT 로 올라온
    점군을 받으므로, 같은 결과를 얻으려면 GT 를 되돌리고 믿는 값으로 다시 올린다.
    q 에 의존하므로 자세마다 오차 방향이 달라진다(= 다시점 불일치의 원인).
    """
    if T_EC_belief is None or T_EC_true is None or T_EC_belief is T_EC_true:
        return np.eye(4)
    FK = kin._fk_frames_m(np.asarray(q, float))[7]          # E(link7) → base
    T_CB_true = FK @ np.linalg.inv(np.asarray(T_EC_true, float))
    T_CB_bel = FK @ np.linalg.inv(np.asarray(T_EC_belief, float))
    return T_CB_bel @ np.linalg.inv(T_CB_true)


def apply_to_points(pts_b, q, T_EC_true, T_EC_belief, kin):
    """base 점군에 위 보정을 적용. 주입이 꺼져 있으면 그대로 반환."""
    pts_b = np.asarray(pts_b, float)
    if len(pts_b) == 0 or T_EC_belief is T_EC_true or T_EC_belief is None:
        return pts_b
    D = base_correction(q, T_EC_true, T_EC_belief, kin)
    return pts_b @ D[:3, :3].T + D[:3, 3]
