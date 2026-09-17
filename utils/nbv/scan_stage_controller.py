"""
utils/nbv/scan_stage_controller.py — sim/real 공용 Phase 오케스트레이터.

시뮬레이션(Isaac)에서 연습한 스캔 흐름을 그대로 실물(Artec)에 적용하기 위해,
lookaround→2→3 의 **순서·게이팅·NBV 수렴 루프**를 한 곳에서 소유한다. 캡처 하드웨어
(sim Isaac 카메라+GT / real Artec streaming+relocalization)만 backend 가 구현.

단계 정의 (사용자 컨셉, 2026-07-01):
  - lookaround : 데이터가 잘 수집되는 포즈로 로봇팔을 이동·**고정** → 턴테이블만 전회전.
  - nbv : 로봇팔은 **최소로** 움직이며(관측 elevation 자세) 턴테이블은 자유 회전 →
              부족면(hole) 메꾸기 (NBV 수렴 루프).
  - flip : 물체를 **외부에서 flip**(뒤집기) → 턴테이블만 전회전 → 윗면
              (lookaround·nbv 에서는 바닥이라 못 본 면) 수집.

세 phase 모두 공통 패턴 = "로봇 고정 + 턴테이블 전회전 캡처"(= capture_rotation).
nbv 만 관측자세를 옮겨가며 반복하고, flip 는 외부 flip 을 매개로 반복한다.

backend 는 아래 ScanBackend 프리미티브를 구현(덕타이핑). 캡처/구동/mesh 는 backend,
순서/루프/게이팅은 컨트롤러 소유.

Notation: pose 는 backend 가 해석하는 로봇 관절해(q). AT_CURRENT = "현재 로봇 포즈
그대로(구동 없이) 캡처" 센티넬 (real lookaround/3 처럼 로봇이 이미 제자리인 경우).
"""
from __future__ import annotations

import os
import time as _time
from typing import Any, Optional, Protocol, runtime_checkable


# 로봇을 구동하지 않고 "현재 포즈에서" 캡처하라는 pose 센티넬.
# (real lookaround/3: 로봇은 home 고정, 물체/턴테이블만 변함 → 구동 불필요)
AT_CURRENT: Any = object()


@runtime_checkable
class ScanBackend(Protocol):
    """공용 단계 컨트롤러가 호출하는 backend 프리미티브 (sim/real 구현)."""

    # 순차 누적: 1=lookaround, 2=lookaround→2, 3=lookaround→2→3.
    stage_until: int
    # nbv NBV 최대 반복 횟수.
    nbv_k_max: int

    # ── 공통 ────────────────────────────────────────────────────────────
    def confirm_start(self) -> bool:
        """lookaround 시작 전 확인. real=사용자 prompt, sim=즉시 True. False=중단."""
        ...

    def go_home(self) -> None:
        """로봇을 known-good home 자세로 복귀 (nbv→3 전환 등)."""
        ...

    def capture_rotation(self, pose: Any, label: str, phase: int) -> bool:
        """로봇을 pose 로 고정(AT_CURRENT 면 구동 없음)하고 턴테이블 전회전하며
        캡처·누적. phase=1|2|3 (캡처 밀도 등 backend 조정용). False=중단."""
        ...

    def finalize(self) -> Any:
        """누적 결과를 backend 결과객체로 만들어 반환 (run() 반환값)."""
        ...

    # ── lookaround ─────────────────────────────────────────────────────────
    def pick_lookaround_pose(self) -> Any:
        """데이터가 잘 잡히는 lookaround 로봇 포즈. sim·real 모두 공용 플래너
        (`utils/nbv/lookaround.plan_lookaround_viewpoints`)가 고르며, 키 큰
        물체면 **포즈 리스트**(겹침 z-밴드, safe-first 순서)를 돌려준다.
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
        """flip(물체 flip) 지원 여부. real=True, sim=False(손회전 불가)."""
        ...

    def next_flip(self) -> bool:
        """다음 flip pose 로 물체를 뒤집도록 외부(사람)에 안내. 더 없거나 사용자
        종료면 False. real=prompt, sim=미지원(False)."""
        ...


# ── stage_until 해석 (sim·real·main 공용) ────────────────────────────────────
# ⚠ 예전에는 세 곳이 **각자** 해석했고 기본값이 전부 달랐다:
#     main_artec.py = 1,  isaac_scan_session = 2,  ArtecMultiPassScanSessionSettings = 3
#   지금은 MULTIPASS_SETTINGS.stage_until 가 항상 있어 드러나지 않지만, 없어지는 순간
#   **화면에 찍히는 단계와 실제 도는 단계가 갈린다.** 해석은 여기 한 곳에서만 한다.
STAGE_UNTIL_ENV = "MMS_SIM_STAGE_UNTIL"
PHASE_DESC = {
    1: "lookaround (5면)",
    2: "lookaround → 2 (NBV)",
    3: "lookaround → 2 → 3 (바닥면 flip)",
}


def resolve_stage_until(multipass_settings, *, allow_env: bool = True,
                       default: int = 2):
    """(mode, source) — 우선순위: env(sim 전용) → multipass_settings → default.

    allow_env : 환경변수 override 를 허용할지. **real 은 False** 로 부를 것
                (실물 동작이 셸 환경에 좌우되면 안 된다). sim 스윕 스크립트용.
    default   : 설정 자체가 없을 때(개발 중 sim 등). 표시와 실제가 같도록
                호출자들이 **같은 값**을 쓰는 것이 핵심이다.
    """
    if allow_env:
        env = os.environ.get(STAGE_UNTIL_ENV)
        if env is not None:
            try:
                return int(env), f"env {STAGE_UNTIL_ENV}"
            except ValueError:
                print(f"[stage] ⚠ {STAGE_UNTIL_ENV}={env!r} 를 정수로 못 읽음 — 무시")
    if multipass_settings is not None:
        v = getattr(multipass_settings, "stage_until", None)
        if v is not None:
            return int(v), "multipass_settings.stage_until"
    return int(default), f"기본값({default})"


def stage_desc(mode) -> str:
    return PHASE_DESC.get(int(mode), f"stage_until={mode}")


_T0 = _time.perf_counter()
_T_LAST = _T0


def _step(tag, title: str, detail: str = "") -> None:
    """단계 구분선 + **직전 단계 소요시간**. sim·real 공용."""
    global _T_LAST
    now = _time.perf_counter()
    dt, total = now - _T_LAST, now - _T0
    _T_LAST = now
    print(f"\n[stage] ═══ [{tag}] {title} " + "═" * max(4, 40 - len(title))
          + f"  (직전 {dt:.1f}s · 누적 {total:.1f}s)")
    if detail:
        print(f"[stage]     {detail}")


def run_scan_stages(backend: ScanBackend) -> Any:
    """lookaround→2→3 순차 누적 실행 (sim/real 공용). 반환 = backend.finalize().

    흐름:
      lookaround : pick_lookaround_pose → capture_rotation (로봇 고정 + 전회전)
      nbv : NBV 수렴 루프 (mesh → 수렴? → NBV 자세 → capture_rotation)
      flip : go_home → (외부 flip → capture_rotation) 반복
    각 단계 실패/중단(False/None)이면 즉시 다음 단계 건너뛰고 finalize.
    """
    # ★ 각 단계를 콘솔에 표시한다. 실물 스캔은 한 번에 수 분이고 중간에 멈추면
    #   "지금 어느 단계인가" 를 알 수 없었다 — 단계 전환이 로그에 안 남아서
    #   프롬프트와 캡처 로그 사이가 통째로 침묵이었다(2026-09-16).
    _step(1, "lookaround", "좋은 자세 고정 + 턴테이블 전회전")

    # ── lookaround — 좋은 포즈 고정 + 턴테이블 전회전 ──────────────────────
    # pick_lookaround_pose 는 단일 pose 또는 **pose 리스트(밴드 계획)** 반환 가능.
    # 밴드(키큰 물체: 겹침 z-대역 자세들, safe-first 순서)면 대역마다 전회전.
    _step("1.1", "시작 확인", "사용자 확인 대기 (prompt_before_first_pass)")
    if not backend.confirm_start():
        return backend.finalize()

    _step("1.2", "자세 계획", "preview 수집 → 실루엣 → 관측 자세 산출")
    pose1 = backend.pick_lookaround_pose()
    poses1 = list(pose1) if isinstance(pose1, (list, tuple)) else [pose1]
    print(f"[stage]   → 자세 {len(poses1)}개 "
          f"({'단일' if len(poses1) == 1 else '밴드 계획'})")

    _step("1.3", "전회전 캡처", f"자세 {len(poses1)}개 × 턴테이블 360°")

    # ★ backend 가 `capture_bands` 를 제공하면 **밴드 전체를 한 scan** 으로 넘긴다.
    #   밴드마다 capture_rotation 을 부르면 밴드 = 별도 IScan 이 되어 SLAM 이 끊기고
    #   밴드끼리 후처리 정합에 의존하게 된다(2026-09-16 실물에서 어긋남).
    #   제공하지 않는 backend(sim)는 아래 기존 루프를 그대로 탄다.
    if len(poses1) > 1 and hasattr(backend, "capture_bands"):
        print(f"[stage]   → 밴드 {len(poses1)}개를 한 scan 으로 연속 캡처")
        _t = _time.perf_counter()
        ok = backend.capture_bands(poses1, phase=1)
        globals()["_T_LAST"] = _time.perf_counter()
        print(f"[stage]   {'✓' if ok else '✘'} 밴드 연속 캡처 "
              f"{'완료' if ok else '실패'}  ({_time.perf_counter() - _t:.1f}s)")
        if not ok:
            print("[stage] ✘ lookaround 실패 — 종료")
            return backend.finalize()
        if backend.stage_until >= 2:
            _step(2, "nbv", "부족면 NBV 보강 (로봇 최소이동)")
            _run_nbv(backend)
        if backend.stage_until >= 3 and backend.supports_flip():
            _step(3, "flip", "외부 flip → 바닥면 수집")
            backend.go_home()
            n_flip = 0
            while backend.next_flip():
                n_flip += 1
                print(f"[stage]   → flip #{n_flip} 캡처")
                if not backend.capture_rotation(AT_CURRENT,
                                                "flip (flip 윗면)", phase=3):
                    break
            print(f"[stage]   → flip {n_flip}회 완료")
        _step("F", "마무리", "모델 확정 (finalize)")
        return backend.finalize()
    # ★ 밴드 하나가 실패해도 **중단하지 않는다**. 도달 못 한 밴드는 lookaround 의 실패가
    #   아니라 부분적 데이터 결손이고, 그 결손을 메우는 것이 바로 nbv NBV 의 역할이다.
    #   예전에는 밴드 1개 실패 → 즉시 finalize 라 nbv·flip 이 통째로 사라졌다
    #   (실측 2026-08-19: hand_drill 267k점 / spray_can 472k점을 모아놓고 전부 버림.
    #    스캔 결과 윗부분이 잘려 나갔다). 전부 실패했을 때만 포기한다.
    n_ok = 0
    for i, p in enumerate(poses1, 1):
        # 라벨에 band 표기가 이미 있으므로 아래 print 에서 (i/N) 을 또 붙이지 않는다.
        label = ("lookaround (측면 5면)" if len(poses1) == 1
                 else f"lookaround (band {i}/{len(poses1)})")
        # ★ 밴드마다 소요시간을 따로 잰다. 캡처(회전 30s)와 그 **사이 간격**을
        #   구분해야 어디서 시간이 새는지 보인다 — 2026-09-16 "밴드 사이 대기가
        #   너무 길다" 추적용.
        t_gap = _time.perf_counter() - _T_LAST
        print(f"[stage]   → {label} 시작"
              + (f"   [직전 밴드 이후 간격 {t_gap:.1f}s]" if i > 1 else ""))
        _t_cap = _time.perf_counter()
        _ok = backend.capture_rotation(p, label, phase=1)
        _dt = _time.perf_counter() - _t_cap
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
    if backend.stage_until >= 2:
        _step(2, "nbv", "부족면 NBV 보강 (로봇 최소이동)")
        _run_nbv(backend)
    else:
        print(f"[stage] nbv 건너뜀 (stage_until={backend.stage_until})")

    # ── flip — 외부 flip 후 윗면(바닥면) 수집 ────────────────────────
    if backend.stage_until >= 3 and backend.supports_flip():
        _step(3, "flip", "외부 flip → 바닥면 수집")
        backend.go_home()                       # flip 은 로봇 고정 전제
        n_flip = 0
        while backend.next_flip():               # 외부에서 물체 뒤집기 안내
            n_flip += 1
            print(f"[stage]   → flip #{n_flip} 캡처")
            if not backend.capture_rotation(AT_CURRENT, "flip (flip 윗면)", phase=3):
                break
        print(f"[stage]   → flip {n_flip}회 완료")
    elif backend.stage_until >= 3:
        print("[stage] flip 건너뜀 (backend 미지원)")
    else:
        print(f"[stage] flip 건너뜀 (stage_until={backend.stage_until})")

    _step("F", "마무리", "모델 확정 (finalize)")
    return backend.finalize()


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
            hit = False
            while select.select([sys.stdin], [], [], 0)[0]:
                line = sys.stdin.readline()
                if line.strip().lower().startswith("n"):
                    hit = True
            return hit
        except Exception:                                   # noqa: BLE001
            return False
    except Exception:                                       # noqa: BLE001
        return False


def _run_nbv(backend: ScanBackend) -> None:
    """nbv NBV 수렴 루프 (sim `_nbv` / real `_nbv_loop` 통합).

    반복: 누적 mesh 생성 → 수렴이면 종료 → 부족면 관측자세(q) 계획 → 그 자세로
    전회전 캡처. feasible 자세 없거나 캡처 실패면 종료.
    [n] 키로 언제든 남은 반복을 건너뛰고 다음 phase 로 넘어간다.
    """
    print("[stage]   (nbv 진행 중 [n] 키 = 남은 보강 건너뛰고 다음 phase)")
    for k in range(backend.nbv_k_max):
        if _skip_key_pressed():
            print(f"[stage]   ⏭ 사용자 [n] — nbv 를 {k}회 보강에서 마친다")
            break
        print(f"[stage]   → NBV 반복 {k + 1}/{backend.nbv_k_max}: 누적 mesh 생성")
        mesh = backend.build_coverage_mesh()
        if mesh is None:
            print("[stage]   ✘ mesh 생성 실패 — nbv 종료")
            break
        if backend.is_converged(mesh):
            print(f"[stage]   ✓ 수렴 — nbv 종료 ({k}회 보강)")
            break
        pose = backend.plan_nbv_pose(mesh)
        if pose is None:
            print("[stage]   ✘ 도달 가능한 NBV 자세 없음 — nbv 종료")
            break
        if not backend.capture_rotation(pose, f"nbv NBV #{k + 1}", phase=2):
            print(f"[stage]   ✘ NBV #{k + 1} 캡처 실패 — nbv 종료")
            break
