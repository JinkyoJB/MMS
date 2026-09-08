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


def dims_from_points(pts, axis_xy=None):
    """스캔 **점군**에서 (키, 지름) 을 잰다 — sim·real 공통 입력.

    ★ 정답(USD AABB) 을 쓰면 안 된다. 실물은 물체 치수를 미리 모르고 **스캔한
      것으로만** 판단해야 하므로, sim 이 GT 를 쓰면 "sim 이 real 을 검증한다"는
      전제가 깨진다(sim 에서만 되는 판단이 real 에서 재현되지 않는다).

    지름은 **축 둘레 최대 반경 ×2** 로 잰다. AABB 폭은 손잡이처럼 한쪽으로
    튀어나온 부분 때문에 실제보다 커져 종횡비를 낮춘다(mug: AABB 126mm 대
    본체 지름 ~96mm). axis_xy 를 주면 그 축 기준, 없으면 점군 중심 기준.
    """
    import numpy as _np
    P = _np.asarray(pts, float)
    if len(P) < 10:
        return None
    h = float(P[:, 2].max() - P[:, 2].min())
    c = _np.asarray(axis_xy, float) if axis_xy is not None else P[:, :2].mean(0)
    r = _np.linalg.norm(P[:, :2] - c, axis=1)
    # 상위 2% 는 손잡이·돌기일 수 있어 분위수로 자른다
    d = float(_np.quantile(r, 0.98) * 2.0)
    return h, d


def flip_angles_for(height_m: float, diameter_m: float,
                    aspect_thresh: float = FLIP_ASPECT_DEFAULT,
                    can_view_bottom=None):
    """(각 목록 tuple, 종횡비) 반환. 90 이 먼저다.

    ★ 판정 기준은 **도달성**이다 — `can_view_bottom()` 이 주어지면 그것만 쓴다.
      "180° 로 뒤집었을 때 바닥면(법선 +ẑ)을 입사각 예산 안에서 볼 수 있는 자세에
      로봇이 도달하는가?" 가 참이면 180° 하나로 충분하고, 거짓이면 90° 를 더한다.

      종횡비는 이 질문의 **대리 지표**일 뿐이라 사각지대가 생긴다(실측 2026-08-20):
      flip 후 물체 꼭대기가 z≈800mm 를 넘으면 고앙각 자세가 도달 불가·자가충돌로
      걸러져 el 30° 로 폴백하고, 바닥면이 grazing 으로 소실된다. 그런데 종횡비가
      2.0 미만이면 90° 보완도 못 받는다 — hand_drill(1.06) 57.7%,
      alarm_clock(1.27) 72.3%, laundry(1.46) 58.5% 가 이 사각지대였다.
      임계 경계 사례(mustard 2.01)의 불안정도 같은 원인이다.

    90 을 먼저 하는 이유: 눕힌 상태의 정합이 성공하므로, 마지막 180(바닥면)의
    hint 오차가 커도 그 앞 패스들이 master 를 이미 넓혀 놓는다.
    """
    aspect = float(height_m) / max(float(diameter_m), 1e-6)
    if can_view_bottom is not None:
        try:
            return ((180.0,) if bool(can_view_bottom()) else (90.0, 180.0)), aspect
        except Exception as e:                  # noqa: BLE001
            # ⚠ 조용히 삼키면 안 된다 — 판정이 죽은 채 종횡비로 폴백해도
            #   로그가 없어 원인을 못 찾는다(실측 2026-08-20 에 실제로 겪음).
            print(f"[flip_policy] ⚠ 도달성 판정 실패({type(e).__name__}: {e}) "
                  f"— 종횡비 {aspect:.2f} 로 폴백")
    if aspect >= float(aspect_thresh):
        return (90.0, 180.0), aspect
    return (180.0,), aspect


def el_needed_for_face(max_incidence_deg: float) -> float:
    """법선 +ẑ 인 면을 입사각 예산 안에서 보려면 필요한 최소 고도각(°).

    카메라가 el 에 있으면 시선과 +ẑ 사이 각이 (90 − el) 이므로 el ≥ 90 − 예산.
    """
    return 90.0 - float(max_incidence_deg)


def describe_flip(angle_deg: float) -> str:
    """사람 안내 문구 (real Phase 3 프롬프트용)."""
    a = float(angle_deg) % 360.0
    if abs(a - 90.0) < 1e-6 or abs(a - 270.0) < 1e-6:
        return "옆으로 눕혀 주세요 (90° — 긴 축이 수평이 되게)"
    if abs(a - 180.0) < 1e-6:
        return "뒤집어 주세요 (180° — 바닥면이 위로)"
    return f"{a:.0f}° 회전해 주세요"
