"""
utils/nbv/scan_stage_controller.py — sim/real 공용 **단계 오케스트레이터**.

시뮬레이션(Isaac)에서 연습한 스캔 흐름을 그대로 실물(Artec)에 적용하기 위해,
preview→lookaround→nbv→flip 의 **순서·게이팅·NBV 수렴 루프**를 한 곳에서 소유한다. 캡처 하드웨어
(sim Isaac 카메라+GT / real Artec streaming+relocalization)만 backend 가 구현.

단계 정의 (사용자 컨셉, 2026-07-01 / 이름 정리 2026-09-17):
  - preview : 물체를 훑어 높이·반경·적정 작업거리를 잰다 (docs/2_preview.md).
              스캔이 아니라 **측량**이다 — 뒤 단계 전부의 입력이 여기서 나온다.
  - lookaround : 물체를 **높이 밴드로 썰어**, 밴드마다 로봇팔을 그 높이의 자세로
              옮기고 턴테이블을 전회전. 한 자세가 다 덮으면 밴드는 1개다.
  - nbv : 로봇팔은 **최소로** 움직이며(관측 elevation 자세) 턴테이블은 자유 회전 →
              부족면(hole) 메꾸기 (NBV 수렴 루프).
  - flip : 물체를 **외부에서 flip**(뒤집기) → 턴테이블만 전회전 → 윗면
              (lookaround·nbv 에서는 바닥이라 못 본 면) 수집.

캡처 세 단계 모두 공통 프리미티브 = "한 자세에서 턴테이블 전회전 캡처"(= capture_rotation).
차이는 **그 프리미티브를 몇 번, 어떤 자세로 부르는가** 다:
  · lookaround : 밴드 수만큼 (높이가 다른 자세들, z 단조 순서, 전체가 한 IScan)
  · nbv        : 수렴할 때까지 (gap 을 겨냥한 자세, 부분 스윕)
  · flip       : 외부 flip 횟수만큼
"로봇 고정" 이 참인 것은 **한 번의 전회전 안에서**다 — 밴드 사이·NBV 반복 사이에는
로봇이 움직이고, 밴드 시작에는 축거리 보정도 들어간다(`standoff.StandoffTracker`).

backend 는 아래 ScanBackend 프리미티브를 구현(덕타이핑). 캡처/구동/mesh 는 backend,
순서/루프/게이팅은 컨트롤러 소유.

Notation: pose 는 backend 가 해석하는 로봇 관절해(q). AT_CURRENT = "현재 로봇 포즈
그대로(구동 없이) 캡처" 센티넬 (real lookaround·flip 처럼 로봇이 이미 제자리인 경우).
"""
from __future__ import annotations

import os
import time as _time
from typing import Any, Optional, Protocol, runtime_checkable


# 로봇을 구동하지 않고 "현재 포즈에서" 캡처하라는 pose 센티넬.
# (real lookaround·flip: 로봇은 home 고정, 물체/턴테이블만 변함 → 구동 불필요)
AT_CURRENT: Any = object()


@runtime_checkable
class ScanBackend(Protocol):
    """공용 단계 컨트롤러가 호출하는 backend 프리미티브 (sim/real 구현)."""

    # **어느 단계까지** 실행할지 (순차 누적). "preview"|"lookaround"|"nbv"|"flip"
    # — 예: "nbv" = preview→lookaround→nbv 까지 하고 flip 은 건너뜀.
    # (2026-09-17 이전에는 정수 1/2/3 이었다. `resolve_stage_until` 이 옛 값도 받는다.)
    stage_until: str
    # nbv 수렴 루프 최대 반복 횟수.
    nbv_k_max: int

    # ── 공통 ────────────────────────────────────────────────────────────
    def confirm_start(self) -> bool:
        """lookaround 시작 전 확인. real=사용자 prompt, sim=즉시 True. False=중단."""
        ...

    def go_home(self) -> None:
        """로봇을 known-good home 자세로 복귀 (nbv→flip 전환 등)."""
        ...

    def capture_rotation(self, pose: Any, label: str, stage: str) -> bool:
        """로봇을 pose 로 고정(AT_CURRENT 면 구동 없음)하고 턴테이블 전회전하며
        캡처·누적. stage="lookaround"|"nbv"|"flip" (캡처 밀도 등 backend 조정용).
        False=중단."""
        ...

    def finalize(self) -> Any:
        """누적 결과를 backend 결과객체로 만들어 반환 (run() 반환값)."""
        ...

    # ── lookaround ─────────────────────────────────────────────────────────
    def pick_lookaround_pose(self) -> Any:
        """lookaround 로봇 포즈. sim·real 모두 공용 플래너
        (`utils/nbv/lookaround.plan_lookaround_viewpoints`)가 고른다.
        보통은 **포즈 리스트**(겹침 z-밴드를 **z 단조** 순서로)이고, 한 자세가
        물체를 다 덮으면 1개짜리 리스트다.

        ⚠ 순서는 **z 단조**다. 예전 주석의 "safe-first(minfill 내림차순)" 는
          밴드가 각자 독립 IScan 이던 시절의 규칙이고, 밴드 전체가 한 IScan 이
          된 뒤로는 tz 가 널뛰어 인접 겹침이 0% 까지 떨어졌다(2026-09-16).
        플래너가 실패하면 AT_CURRENT(현재 자세 고정)로 폴백한다."""
        ...

    # ── nbv (NBV hole-fill) ─────────────────────────────────────────
    def build_coverage_mesh(self) -> Optional[Any]:
        """지금까지 누적된 점군 → 커버리지 평가용 mesh. None=중단(데이터 부족)."""
        ...

    def is_converged(self, mesh: Any) -> bool:
        """커버리지 수렴 판정(True=nbv 종료). 내부에서 coverage 로그도 출력."""
        ...

    def plan_nbv_pose(self, mesh: Any) -> Optional[Any]:
        """mesh 부족면(gap)을 덮는 다음 관측 elevation 자세(q). None=feasible 없음."""
        ...

    # ── flip (external flip → 윗면) ──────────────────────────────────
    def supports_flip(self) -> bool:
        """flip(물체 뒤집기) 지원 여부. real=True, sim=False(손회전 불가)."""
        ...

    def next_flip(self) -> bool:
        """다음 flip pose 로 물체를 뒤집도록 외부(사람)에 안내. 더 없거나 사용자
        종료면 False. real=prompt, sim=USD 회전."""
        ...

    # (선택) flip_extra_poses() -> list[q]
    #   flip 면 패스 뒤 추가 관측 자세(테두리 등). 컨트롤러가 있으면 부르고, 없으면
    #   건너뛴다. 근거·기하: `utils/nbv/flip_policy` 테두리 패스 주석. sim 구현됨, real 보류.


# ── stage_until 해석 (sim·real·main 공용) ────────────────────────────────────
# ⚠ 예전에는 세 곳이 **각자** 해석했고 기본값이 전부 달랐다:
#     main_artec.py = 1,  isaac_scan_session = 2,  ArtecMultiPassScanSessionSettings = 3
#   지금은 MULTIPASS_SETTINGS.stage_until 가 항상 있어 드러나지 않지만, 없어지는 순간
#   **화면에 찍히는 단계와 실제 도는 단계가 갈린다.** 해석은 여기 한 곳에서만 한다.
STAGE_UNTIL_ENV = "MMS_SIM_STAGE_UNTIL"

#: 파이프라인 단계 — **이 순서가 곧 실행 순서**다.
#
#    preview     물체를 훑어 실루엣·크기를 얻는다 (docs/2_preview.md)
#    lookaround  그 실루엣으로 좋은 자세를 골라 전회전 캡처 (docs/3_lookaround.md)
#    nbv         부족한 면을 겨냥해 보강 (docs/4_nbv.md)
#    flip        물체를 뒤집어 바닥면 수집 (docs/5_flip.md)
#
#  ⚠ 2026-09-17 이전에는 `phase_mode` 라는 **정수**(1/2/3)였다. 숫자가 단계
#    이름과 어긋나기 시작해서(preview 를 따로 떼면 lookaround 가 2번인지 1번인지
#    모호하다) 이름으로 바꿨다. `stage_until="nbv"` = "preview 부터 nbv 까지".
STAGES = ("preview", "lookaround", "nbv", "flip")

#: 옛 정수 → 이름. 남아 있는 스크립트·env 가 조용히 다르게 동작하지 않도록
#  받아주되 **경고를 찍는다.** 조용히 무시하면 "왜 nbv 가 안 도나" 로 하루 간다.
_LEGACY_INT = {1: "lookaround", 2: "nbv", 3: "flip"}

STAGE_DESC = {
    "preview": "preview 만 (자세 계획까지, 캡처 안 함)",
    "lookaround": "preview → lookaround (5면)",
    "nbv": "preview → lookaround → nbv (부족면 보강)",
    "flip": "preview → lookaround → nbv → flip (바닥면)",
}


def stage_index(stage) -> int:
    """단계 이름 → 순서 번호. 모르는 이름이면 ValueError."""
    s = str(stage).strip().lower()
    if s not in STAGES:
        raise ValueError(f"모르는 단계 {stage!r} — {', '.join(STAGES)} 중 하나")
    return STAGES.index(s)


def runs_stage(stage_until, stage) -> bool:
    """`stage` 가 `stage_until` 까지의 범위에 드는가 (= 실행하는가)."""
    try:
        return stage_index(stage) <= stage_index(stage_until)
    except ValueError:
        return False


def _coerce_stage(v, where: str):
    """이름/옛 정수 → 단계 이름. 못 읽으면 None."""
    if v is None:
        return None
    if isinstance(v, bool):
        return None
    if isinstance(v, int) or (isinstance(v, str) and v.strip().isdigit()):
        n = int(v)
        name = _LEGACY_INT.get(n)
        if name is None:
            print(f"[stage] ⚠ {where}={v!r} 는 모르는 옛 번호 — 무시")
            return None
        print(f"[stage] ⚠ {where}={v!r} 는 옛 정수 표기다 — "
              f"{name!r} 로 읽는다. 이름으로 바꿀 것({', '.join(STAGES)})")
        return name
    s = str(v).strip().lower()
    if s in STAGES:
        return s
    print(f"[stage] ⚠ {where}={v!r} 를 못 읽음 — 무시 "
          f"(가능한 값: {', '.join(STAGES)})")
    return None


def resolve_stage_until(multipass_settings, *, allow_env: bool = True,
                        default: str = "nbv"):
    """(stage_until, source) — 우선순위: env(sim 전용) → multipass_settings → default.

    allow_env : 환경변수 override 를 허용할지. **real 은 False** 로 부를 것
                (실물 동작이 셸 환경에 좌우되면 안 된다). sim 스윕 스크립트용.
    default   : 설정 자체가 없을 때(개발 중 sim 등). 표시와 실제가 같도록
                호출자들이 **같은 값**을 쓰는 것이 핵심이다.
    """
    if allow_env:
        env = os.environ.get(STAGE_UNTIL_ENV)
        if env is not None:
            name = _coerce_stage(env, f"env {STAGE_UNTIL_ENV}")
            if name is not None:
                return name, f"env {STAGE_UNTIL_ENV}"
    if multipass_settings is not None:
        name = _coerce_stage(getattr(multipass_settings, "stage_until", None),
                             "multipass_settings.stage_until")
        if name is not None:
            return name, "multipass_settings.stage_until"
    return str(default), f"기본값({default})"


def stage_desc(stage_until) -> str:
    return STAGE_DESC.get(str(stage_until), f"stage_until={stage_until}")


_T0 = _time.perf_counter()
_T_LAST = _T0


#: 소요시간 타임라인 — (단계, 항목, 초). 런 끝에 `_print_timeline` 이 표로 찍는다.
#  "어느 단계가 오래 걸리나" 를 로그를 뒤지지 않고 한눈에 보려는 것(2026-09-18).
_TIMELINE: list = []
_CUR_STAGE = "기동"
_STAGE_T0 = _T0          # 현재 단계 시작 시각 — 밴드 루프가 _T_LAST 를 되감아도 총시간은 여기서
_STAGE_ORDER = ("기동", "preview", "lookaround", "nbv", "flip", "finalize")


def _mark(stage: str, item: str, seconds: float) -> None:
    _TIMELINE.append((str(stage), str(item), float(seconds)))


def _step(tag, title: str, detail: str = "") -> None:
    """단계 구분선 + **직전 단계 소요시간**. sim·real 공용."""
    global _T_LAST, _CUR_STAGE, _STAGE_T0
    now = _time.perf_counter()
    dt, total = now - _T_LAST, now - _T0
    _T_LAST = now
    _mark(_CUR_STAGE, "(단계 전체)", now - _STAGE_T0)   # 직전 단계의 총 시간(단계 시작 기준)
    _STAGE_T0 = now
    _CUR_STAGE = {"시작": "기동", 1: "preview", 2: "lookaround", 3: "nbv",
                  4: "flip", "F": "finalize"}.get(tag, str(title))
    print(f"\n[stage] ═══ [{tag}] {title} " + "═" * max(4, 40 - len(title))
          + f"  (직전 {dt:.1f}s · 누적 {total:.1f}s)")
    if detail:
        print(f"[stage]     {detail}")


def _print_timeline() -> None:
    """런 끝 소요시간 요약표. 단계별 합계 + 그 안의 항목(밴드·nbv 반복의 mesh/계획/캡처)."""
    if not _TIMELINE:
        return
    tot, items = {}, {}
    for st, it, sec in _TIMELINE:
        if it == "(단계 전체)":
            tot[st] = tot.get(st, 0.0) + sec
        else:
            items.setdefault(st, []).append((it, sec))
    grand = sum(tot.values())
    print("\n[stage] ───── 소요시간 요약 ─────────────────────────────────────")
    for st in list(_STAGE_ORDER) + [k for k in tot if k not in _STAGE_ORDER]:
        if st not in tot and st not in items:
            continue
        T = tot.get(st, 0.0)
        pad = " " * max(0, 11 - sum(2 if ord(ch) > 127 else 1 for ch in st))   # 한글은 2칸
        line = f"[stage]  {st}{pad}{T:8.1f}s  ({T / max(grand, 1e-9) * 100:3.0f}%)"
        sub = items.get(st)
        if sub:
            line += "   " + " · ".join(f"{it} {sec:.1f}" for it, sec in sub)
        print(line)
    print(f"[stage]  합계       {grand:8.1f}s")
    print("[stage] ────────────────────────────────────────────────────────")


def _flip_extra_passes(backend, n_flip: int) -> None:
    """flip 면 패스 뒤 **추가 관측 자세**(테두리 등) — backend 가 `flip_extra_poses()` 로
    돌려주는 q 마다 전회전 한 번. 없으면 no-op. 근거: `utils/nbv/flip_policy` 테두리 패스 주석."""
    fn = getattr(backend, "flip_extra_poses", None)
    if fn is None:
        return
    try:
        qs = list(fn() or [])
    except Exception as e:                                       # noqa: BLE001
        print(f"[stage]   ⚠ flip 추가 자세 계획 실패({e}) — 건너뜀")
        return
    for j, q in enumerate(qs, 1):
        print(f"[stage]   → flip #{n_flip} 테두리 패스 {j}/{len(qs)}")
        _t0 = _time.perf_counter()
        ok = backend.capture_rotation(q, f"flip #{n_flip} (테두리 {j})", stage="flip")
        _mark("flip", f"#{n_flip} 테두리{j}", _time.perf_counter() - _t0)
        if not ok:
            print("[stage]   ⚠ 테두리 패스 실패 — 건너뜀")


def _finalize_timed(backend):
    _t0 = _time.perf_counter()
    try:
        return backend.finalize()
    finally:
        _mark("finalize", "(단계 전체)", _time.perf_counter() - _t0)
        _print_timeline()


def run_scan_stages(backend: ScanBackend) -> Any:
    """preview→lookaround→nbv→flip 순차 누적 실행 (sim/real 공용).
    반환 = backend.finalize().

    흐름:
      lookaround : pick_lookaround_pose → 밴드마다 capture_rotation (높이별 자세 + 전회전)
      nbv : NBV 수렴 루프 (mesh → 수렴? → NBV 자세 → capture_rotation)
      flip : go_home → (외부 flip → capture_rotation) 반복
    각 단계 실패/중단(False/None)이면 즉시 다음 단계 건너뛰고 finalize.
    """
    # ★ 각 단계를 콘솔에 표시한다. 실물 스캔은 한 번에 수 분이고 중간에 멈추면
    #   "지금 어느 단계인가" 를 알 수 없었다 — 단계 전환이 로그에 안 남아서
    #   프롬프트와 캡처 로그 사이가 통째로 침묵이었다(2026-09-16).
    _step("시작", "시작 확인", "사용자 확인 대기 (prompt_before_first_pass)")
    if not backend.confirm_start():
        return backend.finalize()

    # ── preview — 형상 탐색(높이·반경·적정 거리) ───────────────────────────
    _step(1, "preview", "거리 탐색 → 실루엣 수집 → 관측 자세 산출")
    pose1 = backend.pick_lookaround_pose()
    poses1 = list(pose1) if isinstance(pose1, (list, tuple)) else [pose1]
    print(f"[stage]   → 자세 {len(poses1)}개 "
          f"({'단일' if len(poses1) == 1 else '밴드 계획'})")

    # ★ preview 에서 멈출 수 있다. 거리탐색·실루엣·밴드 산정만 보고 싶을 때
    #   전회전(실물 밴드당 ~30s)을 통째로 안 돌려도 된다 — preview 를 고치는
    #   중에는 이 왕복이 대부분의 시간이었다.
    if not runs_stage(backend.stage_until, "lookaround"):
        print('[stage] lookaround 건너뜀 (stage_until="preview") — 계획만 내고 종료')
        return backend.finalize()

    # ── lookaround — 높이 밴드마다 자세를 옮기며 턴테이블 전회전 ───────────
    # pick_lookaround_pose 는 **pose 리스트(밴드 계획)** 를 준다 — 겹치는 z-대역
    # 자세들을 **z 단조** 순서로. 대역마다 전회전하고, 전체가 한 IScan 이다.
    # (한 자세가 물체를 다 덮으면 리스트 길이가 1 이다 = 옛 "고정" 동작)
    _step(2, "lookaround", f"자세 {len(poses1)}개 × 턴테이블 360°")

    # ★ backend 가 `capture_bands` 를 제공하면 **밴드 전체를 한 scan** 으로 넘긴다.
    #   밴드마다 capture_rotation 을 부르면 밴드 = 별도 IScan 이 되어 SLAM 이 끊기고
    #   밴드끼리 후처리 정합에 의존하게 된다(2026-09-16 실물에서 어긋남).
    #   제공하지 않는 backend(sim)는 아래 기존 루프를 그대로 탄다.
    if len(poses1) > 1 and hasattr(backend, "capture_bands"):
        print(f"[stage]   → 밴드 {len(poses1)}개를 한 scan 으로 연속 캡처")
        _t = _time.perf_counter()
        ok = backend.capture_bands(poses1, stage="lookaround")
        _mark("lookaround", f"bands×{len(poses1)}", _time.perf_counter() - _t)
        globals()["_T_LAST"] = _time.perf_counter()
        print(f"[stage]   {'✓' if ok else '✘'} 밴드 연속 캡처 "
              f"{'완료' if ok else '실패'}  ({_time.perf_counter() - _t:.1f}s)")
        if not ok:
            print("[stage] ✘ lookaround 실패 — 종료")
            return backend.finalize()
        if runs_stage(backend.stage_until, "nbv"):
            _step(3, "nbv", "부족면 NBV 보강 (로봇 최소이동)")
            _run_nbv(backend)
        if runs_stage(backend.stage_until, "flip") and backend.supports_flip():
            _step(4, "flip", "외부 flip → 바닥면 수집")
            backend.go_home()
            n_flip = 0
            while backend.next_flip():
                n_flip += 1
                print(f"[stage]   → flip #{n_flip} 캡처")
                _t0 = _time.perf_counter()
                _fok = backend.capture_rotation(AT_CURRENT, "flip (바닥면)", stage="flip")
                _mark("flip", f"#{n_flip}", _time.perf_counter() - _t0)
                if not _fok:
                    break
                _flip_extra_passes(backend, n_flip)
            print(f"[stage]   → flip {n_flip}회 완료")
        _step("F", "마무리", "모델 확정 (finalize)")
        return _finalize_timed(backend)
    # ★ 밴드 하나가 실패해도 **중단하지 않는다**. 도달 못 한 밴드는 lookaround 의 실패가
    #   아니라 부분적 데이터 결손이고, 그 결손을 메우는 것이 바로 nbv 의 역할이다.
    #   예전에는 밴드 1개 실패 → 즉시 finalize 라 nbv·flip 이 통째로 사라졌다
    #   (실측 2026-08-19: hand_drill 267k점 / spray_can 472k점을 모아놓고 전부 버림.
    #    스캔 결과 윗부분이 잘려 나갔다). 전부 실패했을 때만 포기한다.
    n_ok = 0
    for i, p in enumerate(poses1, 1):
        # 라벨에 band 표기가 이미 있으므로 아래 print 에서 (i/N) 을 또 붙이지 않는다.
        label = ("lookaround (단일 자세)" if len(poses1) == 1
                 else f"lookaround (band {i}/{len(poses1)})")
        # ★ 밴드마다 소요시간을 따로 잰다. 캡처(회전 30s)와 그 **사이 간격**을
        #   구분해야 어디서 시간이 새는지 보인다 — 2026-09-16 "밴드 사이 대기가
        #   너무 길다" 추적용.
        t_gap = _time.perf_counter() - _T_LAST
        print(f"[stage]   → {label} 시작"
              + (f"   [직전 밴드 이후 간격 {t_gap:.1f}s]" if i > 1 else ""))
        _t_cap = _time.perf_counter()
        _ok = backend.capture_rotation(p, label, stage="lookaround")
        _dt = _time.perf_counter() - _t_cap
        _mark("lookaround", f"band{i}", _dt)
        globals()["_T_LAST"] = _time.perf_counter()
        if _ok:
            n_ok += 1
            print(f"[stage]   ✓ {label} 완료  ({_dt:.1f}s)")
        elif len(poses1) > 1:
            print(f"[stage] ⚠ {label} 실패 — 건너뛰고 계속 "
                  f"(남은 결손은 nbv 가 메운다)")
    if n_ok == 0:
        print("[stage] ✘ lookaround 에서 한 패스도 못 얻음 — 종료")
        return backend.finalize()
    if n_ok < len(poses1):
        print(f"[stage] lookaround 부분 성공 {n_ok}/{len(poses1)} 밴드 — nbv 로 진행")

    # ── nbv — 로봇 최소이동 NBV hole-fill ───────────────────────────
    if runs_stage(backend.stage_until, "nbv"):
        _step(3, "nbv", "부족면 NBV 보강 (로봇 최소이동)")
        _run_nbv(backend)
    else:
        print(f"[stage] nbv 건너뜀 (stage_until={backend.stage_until})")

    # ── flip — 외부 flip 후 윗면(바닥면) 수집 ────────────────────────
    if runs_stage(backend.stage_until, "flip") and backend.supports_flip():
        _step(4, "flip", "외부 flip → 바닥면 수집")
        backend.go_home()                       # flip 은 로봇 고정 전제
        n_flip = 0
        while backend.next_flip():               # 외부에서 물체 뒤집기 안내
            n_flip += 1
            print(f"[stage]   → flip #{n_flip} 캡처")
            _t0 = _time.perf_counter()
            _fok = backend.capture_rotation(AT_CURRENT, "flip (바닥면)", stage="flip")
            _mark("flip", f"#{n_flip}", _time.perf_counter() - _t0)
            if not _fok:
                break
            _flip_extra_passes(backend, n_flip)
        print(f"[stage]   → flip {n_flip}회 완료")
    elif runs_stage(backend.stage_until, "flip"):
        print("[stage] flip 건너뜀 (backend 미지원)")
    else:
        print(f"[stage] flip 건너뜀 (stage_until={backend.stage_until})")

    _step("F", "마무리", "모델 확정 (finalize)")
    return _finalize_timed(backend)


def _skip_key_pressed() -> bool:
    """콘솔에 'n' 이 눌려 있으면 True (비차단, Enter 불요). 실패/비콘솔은 False.

    nbv·flip 는 반복이 길어서 사용자가 "이 정도면 됐다" 싶을 때 빠져나갈 손잡이가
    필요하다 (2026-09-16 요청). Windows 콘솔은 msvcrt, POSIX 는 select 로 폴링."""
    try:
        import msvcrt
        hit = False
        while msvcrt.kbhit():
            ch = msvcrt.getwch()
            if ch in ("n", "N"):
                hit = True
        return hit
    except ImportError:
        try:
            import select, sys
            # ★ 비대화형이면 키가 올 수 없다. **여기서 바로 빠진다.**
            #   ⚠ 2026-09-17 발견: stdin 이 EOF(리다이렉트·/dev/null·파이프)면
            #     `select` 는 계속 "읽을 수 있음" 으로 답하고 `readline()` 은
            #     빈 문자열을 준다. 그래서 아래 while 이 **무한루프**였다 —
            #     nbv 첫 반복에 진입도 못 하고 CPU 를 태우며 멎었다.
            #     헤드리스 sim, `nohup`, `tee`, CI 등 **터미널이 아닌 모든 실행**이
            #     여기 걸린다. 실물에서 로그를 파일로 남기면 그대로 멈춘다.
            if not sys.stdin or not sys.stdin.isatty():
                return False
            hit = False
            while select.select([sys.stdin], [], [], 0)[0]:
                line = sys.stdin.readline()
                if line == "":            # EOF — 더 읽을 게 없다(Ctrl-D 포함)
                    break
                if line.strip().lower().startswith("n"):
                    hit = True
            return hit
        except Exception:                                   # noqa: BLE001
            return False
    except Exception:                                       # noqa: BLE001
        return False


def _run_nbv(backend: ScanBackend) -> None:
    """nbv 수렴 루프 (sim `_nbv` / real `_nbv_loop` 통합).

    반복: 누적 mesh 생성 → 수렴이면 종료 → 부족면 관측자세(q) 계획 → 그 자세로
    전회전 캡처. feasible 자세 없거나 캡처 실패면 종료.
    [n] 키로 언제든 남은 반복을 건너뛰고 다음 단계로 넘어간다.
    """
    print("[stage]   (nbv 진행 중 [n] 키 = 남은 보강 건너뛰고 다음 단계)")
    for k in range(backend.nbv_k_max):
        if _skip_key_pressed():
            print(f"[stage]   ⏭ 사용자 [n] — nbv 를 {k}회 보강에서 마친다")
            break
        print(f"[stage]   → NBV 반복 {k + 1}/{backend.nbv_k_max}: 누적 mesh 생성")
        _t0 = _time.perf_counter()
        mesh = backend.build_coverage_mesh()
        _mark("nbv", f"#{k + 1} mesh", _time.perf_counter() - _t0)
        if mesh is None:
            print("[stage]   ✘ mesh 생성 실패 — nbv 종료")
            break
        if backend.is_converged(mesh):
            print(f"[stage]   ✓ 수렴 — nbv 종료 ({k}회 보강)")
            break
        _t0 = _time.perf_counter()
        pose = backend.plan_nbv_pose(mesh)
        _mark("nbv", f"#{k + 1} 계획", _time.perf_counter() - _t0)
        if pose is None:
            print("[stage]   ✘ 도달 가능한 NBV 자세 없음 — nbv 종료")
            break
        _t0 = _time.perf_counter()
        _cap_ok = backend.capture_rotation(pose, f"nbv #{k + 1}", stage="nbv")
        _mark("nbv", f"#{k + 1} 캡처", _time.perf_counter() - _t0)
        if not _cap_ok:
            print(f"[stage]   ✘ NBV #{k + 1} 캡처 실패 — nbv 종료")
            break
