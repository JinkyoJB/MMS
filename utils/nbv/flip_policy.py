"""flip_policy.py — Phase 3 flip 각 정책 (sim·real 공용).

물체 종횡비(키/지름)로 flip 각 목록을 정한다. 근거(2026-08-19, sim GT 대조):

  - 키 큰 물체의 윗면(캡)은 측면 카메라에서 grazing 이고, 고앙각 자세는
    자가충돌(tool↔link4)로 도달이 어렵다 → 180° 만으로는 끝면이 남는다.
  - 90° 로 눕히면 양 끝면이 측면을 향해 el 30~70° 로 잡힌다. spray_can(종횡비
    3.0) 실측: completeness 87.2→91.6%, Chamfer 0.80→0.38mm.
  - 90° 눕힘 자세는 실루엣이 비대칭이라 flip 국소정합(ICP)이 회전 축퇴 없이
    성공한다(fitness 1.00) — 회전대칭체에서 정합이 죽는 문제의 우회로이기도 하다.

sim 은 이 목록을 자동 적용하고(isaac_scan_session._flip_angles), real 은 사람이
뒤집으므로 **안내 문구**로 쓴다(artec_multipass_scan_session.next_flip).
"""
from __future__ import annotations

FLIP_ASPECT_DEFAULT = 2.0      # 키/지름 이 값 이상이면 세장형


def flip_angles_for(height_m: float, diameter_m: float,
                    aspect_thresh: float = FLIP_ASPECT_DEFAULT):
    """(각 목록 tuple, 종횡비) 반환. 세장형이면 (90, 180) — 90 이 먼저다.

    90 을 먼저 하는 이유: 눕힌 상태의 정합이 성공하므로, 마지막 180(바닥면)의
    hint 오차가 커도 그 앞 패스들이 master 를 이미 넓혀 놓는다.
    """
    aspect = float(height_m) / max(float(diameter_m), 1e-6)
    if aspect >= float(aspect_thresh):
        return (90.0, 180.0), aspect
    return (180.0,), aspect


def describe_flip(angle_deg: float) -> str:
    """사람 안내 문구 (real Phase 3 프롬프트용)."""
    a = float(angle_deg) % 360.0
    if abs(a - 90.0) < 1e-6 or abs(a - 270.0) < 1e-6:
        return "옆으로 눕혀 주세요 (90° — 긴 축이 수평이 되게)"
    if abs(a - 180.0) < 1e-6:
        return "뒤집어 주세요 (180° — 바닥면이 위로)"
    return f"{a:.0f}° 회전해 주세요"
