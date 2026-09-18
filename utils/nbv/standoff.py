"""standoff.py — 카메라를 대상에서 **얼마나 떨어뜨릴지**, 한 곳에서 정한다.

왜 한 곳인가
------------
같은 뜻의 값이 다섯 군데에 서로 다른 이름·다른 숫자로 흩어져 있었다
(2026-09-17 정리 전):

    sim  `WORK_FOCUS = 0.25`                     카메라↔**표면**
    real `adaptive_target_standoff_mm = 225`     카메라↔축
    real `nbv_distance_mm = 225`                 **두 뜻으로 섞여 쓰임**
    preview `d_steps` sim (0.30,0.38) / real (0.24,0.30)   카메라↔축 (고정 격자)

sim 과 real 이 250 vs 225 로 갈라져 있었고 — 오늘 내내 잡아온 그 패턴이다 —
더 나쁜 건 `nbv_distance_mm` 이 **한 파일 안에서 두 의미로** 쓰였다는 것이다.
`_axis_view_q` 에는 **축 거리**로 넘어가는데 `plan_frontier` 에는 **표면 거리**로
넘어갔다. 반경 97mm 짜리 물체면 표면까지 225−97 = **128mm** — Spider 근접한계
(170mm) 안쪽이라 데이터가 아예 안 나온다.

그래서 기준을 **표면 거리 하나**로 못 박는다. 축 거리가 필요한 곳은
`axis_standoff(r)` 로 변환해서 쓴다.

    표면 거리  d_surf  = 카메라 ↔ 물체 표면        ← ★ 이것이 기준
    축   거리  d_axis  = 카메라 ↔ 턴테이블 축 = d_surf + r

왜 표면 거리가 기준인가 — **작동거리 창이 표면까지의 거리로 정의되기** 때문이다
(`SensorModel.dof`, SDK `scanning_range`). 축 거리는 물체 반경에 따라 달라지는
파생값이다.
"""
from __future__ import annotations

import math
import os

#: 카메라 ↔ **표면** 목표 거리 (m). 실험 중에는 여기만 바꾸면 전부 따라온다.
#
#  왜 225mm 인가 — Spider 작동거리 창(기본 가정 200~300mm)의 중앙 부근이고,
#  2026-09-17 sim 스윕 실측에서도 축거리 260mm 에서 반환이 최대였다
#  (r=34mm 물체 → 표면거리 ≈ 226mm). 실물에서 다시 재려면
#  `scripts/artec/range_profile.py`.
#
#  ⚠ 여기는 **물리값이다 — 실측 작동거리 창의 중앙에 두고 그대로 둔다.**
#    "FOV/캡처가 예상보다 좁다" 는 여기서 당겨도 고쳐지지만(가까울수록 밀도↑),
#    `lookaround.BAND_OVERLAP` 으로 밴드를 늘려도 고쳐진다. 둘 다 건드리면
#    이중보정이다. **겹침 쪽만 쓴다** — 이쪽은 근접한계(170mm)·충돌이라는 물리
#    하한이 있고, 당기면 band_h 도 같이 줄어(∝거리) 두 효과가 얽히기 때문이다.
#    이 값을 바꿀 이유는 하나뿐이다: `scripts/artec/range_profile.py` 실측 결과.
#
#  `MMS_WORK_STANDOFF_MM` 로 재실행 없이 덮어쓸 수 있다(실험용).
WORK_STANDOFF_M: float = float(
    os.environ.get("MMS_WORK_STANDOFF_MM", "225.0")) / 1000.0


def axis_standoff(radius_m: float, surf_m: float = None) -> float:
    """턴테이블 **축** 기준 거리 = 표면 거리 + 물체 반경.

    카메라가 축을 겨누는 자세(`_axis_view_q` 류)는 축까지의 거리를 받으므로
    여기서 변환한다. 반경을 빼먹으면 큰 물체에서 근접한계 안으로 들어간다.
    """
    return float(surf_m if surf_m is not None else WORK_STANDOFF_M) + float(radius_m)


def preview_start_distance(dof) -> float:
    """preview 첫 축거리 = 거리격자의 첫 스텝(`preview_grid`).

    반경을 아직 모르므로 "반경 0~한 창폭" 을 덮는 스텝에서 출발한다. 이후는
    `lookaround.next_distance` 가 실측 표면거리로 보정한다.
    """
    return preview_grid(dof)[0]


# ══════════════════════════════════════════════════════════════════════════
#  거리 추종 (el 고정 · 축거리만 히스토그램 중앙으로)
# ══════════════════════════════════════════════════════════════════════════
#
#  왜 필요한가
#  -----------
#  밴드 자세는 `plan_lookaround_viewpoints` 가 **preview 점군으로 추정한 반경**을 써서
#  한 번 정하고 끝이었다. 그런데
#    · preview 반경은 실루엣 추정이라 실제와 다르다(대개 작게 나온다)
#    · 회전하면 반경이 θ 에 따라 변한다 — 원통이 아닌 물체는 표면거리가 출렁인다
#    · 밴드마다 tz 가 달라 그 높이의 반경도 다르다
#  결과적으로 표면거리가 작동거리 창을 벗어난 채 한 바퀴를 다 돌아버린다. 그게
#  실물에서 "가끔 물체와 거리가 멀다" 로 보였던 증상이다.
#
#  그래서 **고도각(el)·조준높이(tz)는 그대로 두고 축거리만** 고친다. el 을
#  건드리면 그 밴드가 덮는 z 대역이 바뀌어 계획 전체가 흔들리지만, 축거리는
#  카메라를 시선 방향으로 밀고 당길 뿐이라 덮는 대역이 거의 그대로다.
#
#  모드 (`MMS_STANDOFF_TRACK`)
#  --------------------------
#    off   기존 동작 — 계획 거리 그대로
#    band  **밴드 시작(θ=0, 턴테이블 정지)에서 1회** 보정하고 회전.
#          로봇이 서 있는 동안만 움직이므로 real 에서도 추가 위험이 없다
#          (이미 밴드 경계에서 로봇이 이동하고 settle 하는 그 자리다).
#    live  ★ 기본(2026-09-18). 회전 중 `every` 프레임마다 보정. 비원통 물체는
#          한 바퀴 안에서 표면거리가 출렁이므로(세제: 반경 45↔97mm) θ=0 의
#          한 값으로 못 박으면 다른 방위에서 창 밖으로 나간다.
#          가림 반영 모사(세제 preview 점군, 240프레임, 2026-09-18):
#              밴드      프레임당 점    합집합 커버   연속프레임 겹침 최소
#              tz=32mm   2142 → 2228    60.7 → 62.3%   82 → 81%
#              tz=91mm   1904 → 2126    54.5 → 59.3%   86 → 62%   (30~45mm 7→42%)
#              tz=149mm   664 → 1042    27.5 → 35.4%   74 → 46%
#          점·커버는 늘고, 대가는 **보정 순간의 연속 프레임 겹침**이 46~62% 로
#          내려가는 것이다. 실물 SLAM 이 그 겹침에서 붙어 있는지가 유일한 미검증
#          항목 — 실기에서 확인할 것. 깨지면 `TRACK_MAX_STEP_M`·`TRACK_EVERY` 를
#          조여서(더 작게·더 자주) 해결하는 것이 순서이고, band 로 되돌리는 것은
#          그 다음이다. 되돌리려면 MMS_STANDOFF_TRACK=band.
#          real 배선: `artec_streaming_scan_session` 밴드 루프가 OK 프레임
#          TRACK_EVERY 개마다 `update()` 를 부른다(2026-09-18). sim 은 `_scan_pass`.
TRACK_MODE: str = os.environ.get("MMS_STANDOFF_TRACK", "live").strip().lower()
#: live 모드에서 몇 프레임마다 볼지. 전회전 144프레임 / 8 = 18회.
TRACK_EVERY: int = max(1, int(os.environ.get("MMS_STANDOFF_TRACK_EVERY", "8")))
#: 한 번에 움직일 수 있는 최대량 (m). 스캔 도중 카메라가 훌쩍 뛰면 프레임 간
#  중첩이 깨져 tracking lost 가 난다 — 보정은 **여러 번 나눠서** 수렴시킨다.
TRACK_MAX_STEP_M: float = float(
    os.environ.get("MMS_STANDOFF_TRACK_STEP_MM", "25.0")) / 1000.0
#: 불감대 (mm) — 표면거리가 창 중앙에서 이만큼 안이면 안 움직인다. 0 = 자동.
#
#  자동값은 **창 폭의 1/6** 이다(200~300mm 창이면 ±17mm). 예전엔 1/4(±25mm)를
#  썼는데 그건 preview 기준이다 — preview 의 목표는 "쓸만한 반환을 빨리 얻기" 라
#  느슨해도 되지만, 밴드 캡처의 목표는 **중앙에 두는 것**이라 다르다.
#  2026-09-17 sim(오차 +70mm 주입): 1/4 이면 표면 275mm 가 경계에서 통과해
#  밴드마다 보정 여부가 갈렸다(밴드1·3·5 보정, 밴드2 통과).
#
#  더 조이면 로봇이 자주 움직여 스캔 시간·SLAM 위험이 는다. 5mm 미만 보정은
#  어차피 무시하므로(`distance_correction`) 떨림으로는 안 간다.
#: 트래커가 표면거리를 재는 **밴드 핵심 높이대** 반폭 (m). 조준높이 ±40mm —
#  세로 FOV 가 240mm 거리에서 ±60mm 이므로 그 안쪽. 양쪽 백엔드가 같은 값.
TRACK_CORE_HALF_M: float = 0.04
#: 같은 것을 **스캐너 프레임**에서 표현한 값 — 광축 기준 세로 각도 반폭 (deg).
#  240mm 거리에서 ±40mm ≈ ±9.5°. 세로 FOV(28.6°) 의 가운데 2/3 다.
TRACK_CORE_HALF_DEG: float = 9.5


def core_mask_camera_frame(pts_cam, half_deg: float = None):
    """스캐너(OpenCV: x 오른쪽·y 아래·z 앞) 프레임 정점 중 **광축 세로 ±half_deg**
    안의 점 마스크 — real 이 `StandoffTracker(core=...)` 에 넘긴다. 프레임 원점이
    카메라이므로 좌표변환 없이 각도만 본다(hand-eye 오차가 안 낀다)."""
    import numpy as np
    P = np.asarray(pts_cam, float)
    h = math.radians(TRACK_CORE_HALF_DEG if half_deg is None else float(half_deg))
    return np.abs(np.arctan2(P[:, 1], np.maximum(P[:, 2], 1e-9))) <= h
TRACK_TOL_M: float = float(os.environ.get("MMS_STANDOFF_TOL_MM", "0.0")) / 1000.0


def window_center(dof) -> float:
    """작동거리 창의 중앙 (m). 표면을 여기에 놓는 것이 목표다."""
    return 0.5 * (float(dof[0]) + float(dof[1]))


def surface_distance(pts, cam_pos) -> "float | None":
    """카메라에서 본 표면거리의 **중앙값** (m). 점이 없으면 None.

    피크가 아니라 중앙값을 쓴다 — 히스토그램 꼬리(바닥·배경 잔재)에 안 흔들린다.
    """
    import numpy as np
    if pts is None or cam_pos is None or len(pts) == 0:
        return None
    from utils.nbv.range_profile import point_ranges   # 거리 계산은 한 곳에서만
    r = point_ranges(pts, cam_pos)
    return float(np.median(r)) if len(r) else None


def distance_correction(d, pts, cam_pos, dof, lo, hi,
                        blind_step: float = 0.04, max_step: float = None,
                        tol: float = None):
    """이번 캡처로 **다음 축거리**를 정한다. (d_next | None, 이유)

    카메라는 축을 거리 `d` 에서 겨누므로, 표면거리 중앙값 `p` 가 나오면 그 방위의
    국소 반경이 `r ≈ d − p` 다. 표면을 창 중앙 `c` 에 놓으려면
    `d_next = c + r = d + (c − p)` — **한 장으로 이동량이 나온다.**

    반환이 없으면 방향을 모른다. 그때는 **가까이** 한 스텝 간다: 실물에서 관측된
    실패는 "너무 멀어 프레임이 빈다" 쪽이었고, 멀어지는 쪽으로 헤매면 계속 빈
    캡처만 쌓인다.

    `d_next` 가 None 이면 "이미 충분, 더 안 움직임" — 호출자는 그대로 진행한다.
    """
    import numpy as np
    c = window_center(dof)
    if tol is None:
        tol = max(0.25 * (float(dof[1]) - float(dof[0])), 0.01)   # 창 폭의 1/4
    p = surface_distance(pts, cam_pos)
    if p is None:
        return (float(np.clip(d - blind_step, lo, hi)), "반환 없음 — 가까이 한 스텝")
    err = c - p
    if abs(err) <= tol:
        return None, f"표면 {p*1000:.0f}mm ≈ 창중앙 {c*1000:.0f}mm — 유지"
    step = err
    capped = ""
    if max_step is not None and abs(step) > max_step:
        step = math.copysign(max_step, err)
        capped = f" (1회 {max_step*1000:.0f}mm 로 제한)"
    d_next = float(np.clip(d + step, lo, hi))
    if abs(d_next - d) < 0.005:
        return None, f"보정 {err*1000:+.0f}mm 이 작다 — 유지"
    return d_next, (f"표면 {p*1000:.0f}mm → 창중앙 {c*1000:.0f}mm 로 "
                    f"{step*1000:+.0f}mm{capped} (d {d*1000:.0f}→{d_next*1000:.0f}mm)")


def blind_probe(d0: float, k: int, lo: float, hi: float,
                step: float = 0.04) -> "float | None":
    """반환이 0일 때 **k 번째 탐침 거리**. 양쪽 다 소진하면 None.

    가까이·멀리를 번갈아 벌려 나간다 (양쪽 브래킷):

        d0−step, d0+step, d0−2·step, d0+2·step, …

    ⚠ 예전에는 **가까이만** 갔다. "실물 실패는 너무 멀어서였다" 는 관찰에 근거한
      것이었는데, 그 관찰은 축거리와 표면거리를 구분하기 **전**의 것이다. `d` 는
      축거리라 반경 100mm 물체면 d=290mm 가 표면 190mm — 근접한계(200mm) 안쪽이고
      반환이 0 이다. 그 상태에서 더 가까이 가면 **탐색이 실패 방향으로 달려간다.**
      반환이 0 이면 방향을 모른다는 뜻이므로, 모르는 채로 한쪽을 믿으면 안 된다.

    가까운 쪽을 먼저 보는 순서는 유지한다 — 멀어서 비는 경우가 여전히 더 흔하고,
    가까운 쪽이 밀도·정밀도 면에서도 원하는 방향이다.

    ★ **한쪽이 막혀도 살아 있는 쪽으로 계속 벌린다.** 범위 밖 탐침은 건너뛰고
      다음 유효 탐침을 센다. 예전에는 k 번째 탐침이 `[lo, hi]` 밖이면 그 자리에서
      `None` 을 돌려줬고, 호출부는 그걸 "다 썼다" 로 읽어 **탐색을 통째로 끝냈다.**
      브래킷은 좌우로 번갈아 벌어지므로 **좁은 쪽이 먼저 소진되면 넓은 쪽에 남은
      거리를 못 써 봤다** — 창 200~300·시작 300·한계 200~480mm 에서:

          260 · 340 · 220 · 380 · (180 = 한계 밖) → 중단
          실효 범위가 220~380mm 로 잘려 상한 480mm 는 이름만 남았다.

      지금은 180 을 건너뛰고 420 · 460 까지 간다. `None` 은 **양쪽 모두** 소진된
      경우에만 나오므로, 호출부의 "탐색범위 소진" 메시지도 그제서야 사실이 된다.
    """
    if k <= 0 or step <= 0:
        return None
    # 양쪽 중 먼 끝까지 덮을 만큼만 돌린다 (무한루프 방지).
    reach = max(float(d0) - float(lo), float(hi) - float(d0))
    n_max = int(math.ceil(max(reach, 0.0) / float(step) - 1e-9)) + 1
    found = 0
    for j in range(1, 2 * n_max + 1):
        n = (j + 1) // 2                      # 1,1,2,2,3,3,…
        sign = -1.0 if (j % 2 == 1) else +1.0  # 가까이 먼저
        d = float(d0) + sign * n * float(step)
        if not ((lo - 1e-9) <= d <= (hi + 1e-9)):
            continue                          # 이쪽은 막혔다 — 반대쪽으로 계속
        found += 1
        if found == k:
            return d
    return None


#: preview 가 상정하는 **최대 물체 반경** (m). 거리격자·탐색한계가 공유한다.
#  이보다 굵은 물체는 격자가 못 덮고, 탐색 상한도 모자란다.
PREVIEW_RADIUS_MAX_M: float = float(
    os.environ.get("MMS_PREVIEW_RADIUS_MAX_MM", "180.0")) / 1000.0


def preview_grid(dof, radius_max_m: float = None):
    """preview 고정 거리격자 — **작동거리 창에서 유도한다.** (축거리 tuple)

    축거리 `d` 에서 창 `dof` 안에 들어오는 표면의 반경은
    `r ∈ [d − far, d − near]` — 한 스텝이 잡는 **반경 폭이 곧 창 폭**이다.
    그래서 반경 0~`radius_max_m` 을 덮으려면 창 폭 간격으로 스텝을 놓으면 된다:

        d_k = far + k·(far − near),   k = 0 … ceil(r_max / 창폭) − 1

    창 (200,300)·r_max 180mm 이면 `(300, 400)mm` — 두 스텝으로 r 0~200mm 를 덮는다.
    (어느 스텝에 잡히는지 자체가 반경 측정이다 — `simulate_planning_captures`.)

    ⚠ 2026-09-17 이전에는 이 값이 **양쪽에 하드코딩돼 갈라져 있었다**:
          sim  (300, 380)mm   ← 창 (200,300) 기준으로는 대체로 맞았다
          real (240, 300)mm   ← 임의값. r≥40mm 면 첫 스텝이 **근접한계 안쪽**이고
                                r=110mm 면 두 스텝 다 안쪽이라 반환이 아예 없다
    창이 바뀌면(스캐너 설정·기종) 둘 다 조용히 틀려진다. 그래서 유도로 바꿨다.
    """
    near, far = float(dof[0]), float(dof[1])
    band = max(far - near, 1e-3)              # 한 스텝이 잡는 반경 폭
    r_max = PREVIEW_RADIUS_MAX_M if radius_max_m is None else float(radius_max_m)
    n = max(1, int(math.ceil(r_max / band - 1e-9)))
    return tuple(far + k * band for k in range(n))


def probe_bounds(dof, radius_max_m: float = None):
    """반환 0 탐색의 축거리 한계 (lo, hi) — **작동거리 창에서 유도한다.**

    축거리 = 표면거리 + 반경이므로, 창이 `dof` 이고 반경이 0~`radius_max_m` 이면
    말이 되는 축거리는 `[dof0, dof1 + radius_max]` 다. 예전에는 preview 의 고정
    거리격자(`d_steps`)에 ±80mm 를 붙여 썼는데, 그 격자는 적응적 탐색이 생기기
    전의 유물이라 창과 아무 관계가 없었다.
    """
    r_max = PREVIEW_RADIUS_MAX_M if radius_max_m is None else float(radius_max_m)
    return (max(float(dof[0]), 0.05), float(dof[1]) + r_max)


class StandoffTracker:
    """밴드 캡처 중 축거리를 창 중앙으로 끌어당기는 소형 컨트롤러.

    sim·real 공용이다 — 두 백엔드가 **같은 판단**을 하도록 로직은 여기에만 둔다.
    백엔드는 "재겨냥(축거리 d 로 다시 IK·이동)" 콜백만 준다.

        tr = StandoffTracker(dof, lo, hi, log=print)
        for i, th in enumerate(thetas):
            ...캡처...
            if tr.due(i):
                d = tr.update(i, d, pts_world, cam_world, retarget)
    """

    def __init__(self, dof, lo, hi, mode: str = None, every: int = None,
                 max_step: float = None, tol: float = None, log=None,
                 core=None):
        """`core` — 표면거리를 **밴드 핵심 높이대**의 점으로만 잰다. 없으면 창 안
        점 전체의 중앙값. 두 형태를 받는다 (백엔드가 넘기는 점군의 프레임이 달라서):
          · (target_z, up_sign, half_m) : base 프레임 점군 — 조준높이 ±half_m (sim)
          · callable(pts) -> bool mask  : 프레임을 아는 쪽이 직접 고른다 (real 은
            스캐너 프레임 정점이라 **광축 기준 세로 각도** ±TRACK_CORE_HALF_DEG)

        ★ 왜 핵심 높이대인가 (2026-09-18 세제 실측·모사). 카메라는 el=30° 로
          내려다보므로, 조준높이보다 **위**의 면은 가깝고 **아래**의 면은 멀다.
          창 안 점 전체의 중앙값은 가까운 윗부분에 끌려 "창 중앙보다 가깝다" 로
          읽히고, 트래커는 카메라를 **물린다**(모사 317→338mm). 물러나면 창의 먼
          끝이 아랫부분을 자른다 — tz=32mm 밴드가 실제로는 50~138mm(중앙 102)만
          잡았다. 이 밴드가 맡은 높이대의 면을 창 중앙에 놓아야 한다.
        """
        self.core = None
        if callable(core):
            self.core = core
        elif core is not None:
            tz, us, half = core
            self.core = (float(tz), (1.0 if float(us) >= 0 else -1.0), float(half))
        self.dof = (float(dof[0]), float(dof[1]))
        # 불감대: 명시값 > env > 자동(창 폭의 1/6)
        self.tol = float(tol) if tol else (
            TRACK_TOL_M or max((self.dof[1] - self.dof[0]) / 6.0, 0.008))
        self.lo, self.hi = float(lo), float(hi)
        self.mode = (mode or TRACK_MODE).strip().lower()
        self.every = int(every or TRACK_EVERY)
        self.max_step = TRACK_MAX_STEP_M if max_step is None else float(max_step)
        self.log = log or (lambda _m: None)
        self.n_moves = 0
        self.n_reject = 0

    @property
    def enabled(self) -> bool:
        return self.mode in ("band", "live")

    def _core_points(self, pts):
        """`core` 가 있으면 조준높이 ±half 의 점만. 30점 미만이면 전체로 폴백."""
        if self.core is None or pts is None or len(pts) == 0:
            return pts
        import numpy as np
        P = np.asarray(pts, float)
        if callable(self.core):
            m = np.asarray(self.core(P), bool)
        else:
            tz, us, half = self.core
            m = np.abs(P[:, 2] * us - tz * us) <= half
        return P[m] if int(m.sum()) >= 30 else pts

    def due(self, i: int) -> bool:
        """이 프레임에서 보정을 시도할 차례인가."""
        if not self.enabled:
            return False
        if self.mode == "band":
            return i == 0                       # 밴드 시작 1회 (턴테이블 정지 중)
        return i % self.every == 0

    def update(self, i: int, d: float, pts, cam_pos, retarget) -> float:
        """보정을 1회 시도하고 **실제로 쓰게 된 축거리**를 돌려준다.

        `retarget(d_new) -> bool` : 그 축거리로 재겨냥(IK+충돌+이동). 실패하면
        False — 그 경우 거리는 원래대로 두고 계속 간다(밴드를 버리지 않는다).
        """
        if not self.enabled:
            return d
        # ★ live 모드에서 **빈 반환은 방향을 모른다** — 붙잡아 둔다.
        #   `distance_correction` 의 "반환 없음 → 가까이 한 스텝" 은 preview·밴드
        #   시작(멀어서 비는 경우가 실측상 대부분)을 위한 규칙이다. 회전 중에는
        #   반대로 **너무 가까워서** 창 앞으로 빠진 직후일 수 있고, 그때 또
        #   가까이 가면 근접한계(lo)까지 달려가 그 밴드가 통째로 빈다(가림 없는
        #   모사에서 실제로 317→200mm 로 폭주했다, 2026-09-18). 한 프레임 비면
        #   다음 측정(8프레임 뒤)까지 자리를 지키는 것이 안전하다.
        if self.mode == "live" and i > 0 and (pts is None or len(pts) == 0):
            self.log(f"  [거리추종] f{i} 반환 없음 — 회전 중이라 유지 (d={d*1000:.0f}mm)")
            return d
        pts = self._core_points(pts)
        d_next, why = distance_correction(d, pts, cam_pos, self.dof,
                                          self.lo, self.hi,
                                          max_step=self.max_step, tol=self.tol)
        if d_next is None:
            if i == 0:                          # 밴드 시작은 한 줄 남긴다
                self.log(f"  [거리추종] f{i} — {why}")
            return d
        if retarget(d_next):
            self.n_moves += 1
            self.log(f"  [거리추종] f{i} {why}")
            return d_next
        self.n_reject += 1
        self.log(f"  [거리추종] f{i} 재겨냥 실패(IK/충돌) — d={d*1000:.0f}mm 유지")
        return d

    def converge(self, d: float, measure, retarget, max_tries: int = 3,
                 i: int = 0) -> float:
        """정지 상태에서 **여러 번 나눠** 창 중앙으로 수렴시킨다. 반환 = 최종 축거리.

        `measure() -> (pts, cam_pos)` : 지금 자세에서 다시 재는 콜백.

        왜 반복인가 — 1회 이동량 제한(`TRACK_MAX_STEP_M`)은 **회전 중** 프레임
        중첩이 깨지는 걸 막으려고 있다. 그래서 크게 어긋나 있으면 한 번에 못
        따라잡는다(80mm 필요한데 25mm 만 간다). 밴드 시작은 턴테이블이 멈춰 있고
        어차피 로봇이 밴드 간 이동을 막 끝낸 자리라, 작은 홉을 몇 번 더 뛰는 데
        드는 비용이 거의 없다 — 제한은 유지한 채 반복해서 수렴시킨다.
        """
        if not self.enabled:
            return d
        for _k in range(max(1, int(max_tries))):
            pts, cam = measure()
            d_new = self.update(i, d, pts, cam, retarget)
            if d_new == d:                      # 유지 판정 또는 재겨냥 실패
                break
            d = d_new
        return d

    def summary(self) -> str:
        if not self.enabled:
            return "거리추종 off"
        return (f"거리추종({self.mode}) 보정 {self.n_moves}회"
                + (f", 거부 {self.n_reject}회" if self.n_reject else ""))
