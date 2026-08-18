"""
utils/nbv/scan_phase_controller.py — sim/real 공용 Phase 오케스트레이터.

시뮬레이션(Isaac)에서 연습한 스캔 흐름을 그대로 실물(Artec)에 적용하기 위해,
Phase 1→2→3 의 **순서·게이팅·NBV 수렴 루프**를 한 곳에서 소유한다. 캡처 하드웨어
(sim Isaac 카메라+GT / real Artec streaming+relocalization)만 backend 가 구현.

Phase 정의 (사용자 컨셉, 2026-07-01):
  - Phase 1 : 데이터가 잘 수집되는 포즈로 로봇팔을 이동·**고정** → 턴테이블만 전회전.
  - Phase 2 : 로봇팔은 **최소로** 움직이며(관측 elevation 자세) 턴테이블은 자유 회전 →
              부족면(hole) 메꾸기 (NBV 수렴 루프).
  - Phase 3 : 물체를 **외부에서 flip**(뒤집기) → 턴테이블만 전회전 → 윗면
              (Phase 1·2 에서는 바닥이라 못 본 면) 수집.

세 phase 모두 공통 패턴 = "로봇 고정 + 턴테이블 전회전 캡처"(= capture_rotation).
Phase 2 만 관측자세를 옮겨가며 반복하고, Phase 3 는 외부 flip 을 매개로 반복한다.

backend 는 아래 ScanBackend 프리미티브를 구현(덕타이핑). 캡처/구동/mesh 는 backend,
순서/루프/게이팅은 컨트롤러 소유.

Notation: pose 는 backend 가 해석하는 로봇 관절해(q). AT_CURRENT = "현재 로봇 포즈
그대로(구동 없이) 캡처" 센티넬 (real Phase 1/3 처럼 로봇이 이미 제자리인 경우).
"""
from __future__ import annotations

import os
from typing import Any, Optional, Protocol, runtime_checkable


# 로봇을 구동하지 않고 "현재 포즈에서" 캡처하라는 pose 센티넬.
# (real Phase 1/3: 로봇은 home 고정, 물체/턴테이블만 변함 → 구동 불필요)
AT_CURRENT: Any = object()


@runtime_checkable
class ScanBackend(Protocol):
    """공용 Phase 컨트롤러가 호출하는 backend 프리미티브 (sim/real 구현)."""

    # 순차 누적: 1=Phase1, 2=Phase1→2, 3=Phase1→2→3.
    phase_mode: int
    # Phase 2 NBV 최대 반복 횟수.
    nbv_k_max: int

    # ── 공통 ────────────────────────────────────────────────────────────
    def confirm_start(self) -> bool:
        """Phase 1 시작 전 확인. real=사용자 prompt, sim=즉시 True. False=중단."""
        ...

    def go_home(self) -> None:
        """로봇을 known-good home 자세로 복귀 (Phase 2→3 전환 등)."""
        ...

    def capture_rotation(self, pose: Any, label: str, phase: int) -> bool:
        """로봇을 pose 로 고정(AT_CURRENT 면 구동 없음)하고 턴테이블 전회전하며
        캡처·누적. phase=1|2|3 (캡처 밀도 등 backend 조정용). False=중단."""
        ...

    def finalize(self) -> Any:
        """누적 결과를 backend 결과객체로 만들어 반환 (run() 반환값)."""
        ...

    # ── Phase 1 ─────────────────────────────────────────────────────────
    def pick_phase1_pose(self) -> Any:
        """데이터가 잘 잡히는 Phase 1 로봇 포즈. sim=azimuth sweep 으로 선택,
        real=AT_CURRENT(home 고정). 이후 이 포즈로 capture_rotation."""
        ...

    # ── Phase 2 (NBV hole-fill) ─────────────────────────────────────────
    def build_coverage_mesh(self) -> Optional[Any]:
        """지금까지 누적된 점군 → 커버리지 평가용 mesh. None=중단(데이터 부족)."""
        ...

    def is_converged(self, mesh: Any) -> bool:
        """커버리지 수렴 판정(True=Phase 2 종료). 내부에서 coverage 로그도 출력."""
        ...

    def plan_nbv_pose(self, mesh: Any) -> Optional[Any]:
        """mesh 부족면(gap)을 덮는 다음 관측 elevation 자세(q). None=feasible 없음."""
        ...

    # ── Phase 3 (external flip → 윗면) ──────────────────────────────────
    def supports_phase3(self) -> bool:
        """Phase 3(물체 flip) 지원 여부. real=True, sim=False(손회전 불가)."""
        ...

    def next_flip(self) -> bool:
        """다음 flip pose 로 물체를 뒤집도록 외부(사람)에 안내. 더 없거나 사용자
        종료면 False. real=prompt, sim=미지원(False)."""
        ...


# ── phase_mode 해석 (sim·real·main 공용) ────────────────────────────────────
# ⚠ 예전에는 세 곳이 **각자** 해석했고 기본값이 전부 달랐다:
#     main_artec.py = 1,  isaac_scan_session = 2,  ArtecMultiPassScanSessionSettings = 3
#   지금은 MULTIPASS_SETTINGS.phase_mode 가 항상 있어 드러나지 않지만, 없어지는 순간
#   **화면에 찍히는 단계와 실제 도는 단계가 갈린다.** 해석은 여기 한 곳에서만 한다.
PHASE_MODE_ENV = "MMS_SIM_PHASE_MODE"
PHASE_DESC = {
    1: "Phase 1 (5면)",
    2: "Phase 1 → 2 (NBV)",
    3: "Phase 1 → 2 → 3 (바닥면 flip)",
}


def resolve_phase_mode(multipass_settings, *, allow_env: bool = True,
                       default: int = 2):
    """(mode, source) — 우선순위: env(sim 전용) → multipass_settings → default.

    allow_env : 환경변수 override 를 허용할지. **real 은 False** 로 부를 것
                (실물 동작이 셸 환경에 좌우되면 안 된다). sim 스윕 스크립트용.
    default   : 설정 자체가 없을 때(개발 중 sim 등). 표시와 실제가 같도록
                호출자들이 **같은 값**을 쓰는 것이 핵심이다.
    """
    if allow_env:
        env = os.environ.get(PHASE_MODE_ENV)
        if env is not None:
            try:
                return int(env), f"env {PHASE_MODE_ENV}"
            except ValueError:
                print(f"[phase] ⚠ {PHASE_MODE_ENV}={env!r} 를 정수로 못 읽음 — 무시")
    if multipass_settings is not None:
        v = getattr(multipass_settings, "phase_mode", None)
        if v is not None:
            return int(v), "multipass_settings.phase_mode"
    return int(default), f"기본값({default})"


def phase_desc(mode) -> str:
    return PHASE_DESC.get(int(mode), f"phase_mode={mode}")


def run_scan_phases(backend: ScanBackend) -> Any:
    """Phase 1→2→3 순차 누적 실행 (sim/real 공용). 반환 = backend.finalize().

    흐름:
      Phase 1 : pick_phase1_pose → capture_rotation (로봇 고정 + 전회전)
      Phase 2 : NBV 수렴 루프 (mesh → 수렴? → NBV 자세 → capture_rotation)
      Phase 3 : go_home → (외부 flip → capture_rotation) 반복
    각 단계 실패/중단(False/None)이면 즉시 다음 단계 건너뛰고 finalize.
    """
    # ── Phase 1 — 좋은 포즈 고정 + 턴테이블 전회전 ──────────────────────
    # pick_phase1_pose 는 단일 pose 또는 **pose 리스트(밴드 계획)** 반환 가능.
    # 밴드(키큰 물체: 겹침 z-대역 자세들, safe-first 순서)면 대역마다 전회전.
    if not backend.confirm_start():
        return backend.finalize()
    pose1 = backend.pick_phase1_pose()
    poses1 = list(pose1) if isinstance(pose1, (list, tuple)) else [pose1]
    for i, p in enumerate(poses1, 1):
        label = ("Phase 1 (측면 5면)" if len(poses1) == 1
                 else f"Phase 1 (band {i}/{len(poses1)})")
        if not backend.capture_rotation(p, label, phase=1):
            return backend.finalize()

    # ── Phase 2 — 로봇 최소이동 NBV hole-fill ───────────────────────────
    if backend.phase_mode >= 2:
        _run_phase2_nbv(backend)

    # ── Phase 3 — 외부 flip 후 윗면(바닥면) 수집 ────────────────────────
    if backend.phase_mode >= 3 and backend.supports_phase3():
        backend.go_home()                       # flip 은 로봇 고정 전제
        while backend.next_flip():               # 외부에서 물체 뒤집기 안내
            if not backend.capture_rotation(AT_CURRENT, "Phase 3 (flip 윗면)", phase=3):
                break

    return backend.finalize()


def _run_phase2_nbv(backend: ScanBackend) -> None:
    """Phase 2 NBV 수렴 루프 (sim `_phase2` / real `_phase2_nbv_loop` 통합).

    반복: 누적 mesh 생성 → 수렴이면 종료 → 부족면 관측자세(q) 계획 → 그 자세로
    전회전 캡처. feasible 자세 없거나 캡처 실패면 종료.
    """
    for k in range(backend.nbv_k_max):
        mesh = backend.build_coverage_mesh()
        if mesh is None:
            break
        if backend.is_converged(mesh):
            break
        pose = backend.plan_nbv_pose(mesh)
        if pose is None:
            break
        if not backend.capture_rotation(pose, f"Phase 2 NBV #{k + 1}", phase=2):
            break
