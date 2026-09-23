"""run_paths.py — run 산출물 폴더 규칙 **한 곳** (2026-09-23).

    output/<RUN_TS>/                 run 폴더 (RUN_TS = main_artec.py 가 MMS_RUN_TS 로 심는 실행 일시)
    ├─ README.txt · timeline.csv · final.obj
    ├─ aligned/aligned.sproj         Artec Studio 로 여는 것 (mms_artec/system.py)
    ├─ final/final.sproj
    ├─ events.jsonl · run.log        이벤트 로그 · 콘솔 로그
    ├─ live_cloud.ply                라이브 뷰어 누적 점군 (mms_artec/nbv/live_scan_viewer.py)
    ├─ scan_dumps/                   IScan 별 점군 npz + 적용 변환
    └─ debug/                        디버그 이미지 — 뷰어(scripts/artec/live_range_view.py)가 tail
        ├─ preview/ · lookaround/ · nbv/ · flip/    단계별 거리 이미지 (range_debug_image)
        ├─ cam/<단계>/               카메라 스냅샷 (utils/debug_view.py)
        ├─ nbv_plan/                 nbv 계획 덤프 (utils/nbv/nbv_debug_dump.py)
        ├─ iso/                      probe 격리 PLY
        ├─ preview_points_*.npz      preview 계획 점군
        └─ range_view.log · range.avi

예전(≤2026-09-23 오전)엔 output/debug/<단계>_<RUN>/, output/scan_dumps/<RUN>/, output/events_<RUN>.jsonl,
output/artec_lookaround_<RUN>{,_raw,.obj} 로 흩어져 있었다. `scripts/artec/tidy_output.py` 가 옛 run 을
이 규칙으로 옮긴다.

MMS_RUN_TS 가 없으면(스크립트 단독 실행) output/_norun/ 아래에 쌓인다 — run 폴더를 더럽히지 않게.
"""
from __future__ import annotations

import os
import re
from pathlib import Path

from utils import PROJECT_ROOT

NORUN = "_norun"
_RUN_RE = re.compile(r"^\d{8}_\d{6}$")


def run_ts() -> str | None:
    return os.environ.get("MMS_RUN_TS") or None


def run_dir(ts: str | None = None) -> Path:
    """output/<RUN_TS>/ (절대경로). cwd 가 바뀌어도 리포 밑."""
    return PROJECT_ROOT / "output" / (ts or run_ts() or NORUN)


def debug_dir(stage: str | None = None, ts: str | None = None) -> Path:
    """output/<RUN_TS>/debug[/<stage>]/ — 폴더를 만들어 돌려준다."""
    d = run_dir(ts) / "debug"
    if stage:
        d = d / stage
    d.mkdir(parents=True, exist_ok=True)
    return d


# ── 옛 배치(≤2026-09-23)까지 찾아 주는 조회 함수 — 스크립트용 ──────────────
def list_runs() -> list[str]:
    """output/ 에 있는 run(RUN_TS) 오름차순 — run 폴더 + 옛 배치(events_<RUN>.jsonl)."""
    out = set()
    o = PROJECT_ROOT / "output"
    if not o.is_dir():
        return []
    for p in o.iterdir():
        if p.is_dir() and _RUN_RE.match(p.name):
            out.add(p.name)
    for p in o.glob("events_*.jsonl"):
        out.add(p.stem.replace("events_", ""))
    return sorted(out)


def scan_dumps_dir(ts: str) -> Path:
    d = run_dir(ts) / "scan_dumps"
    legacy = PROJECT_ROOT / "output" / "scan_dumps" / ts
    return d if d.is_dir() or not legacy.is_dir() else legacy


def events_path(ts: str) -> Path:
    p = run_dir(ts) / "events.jsonl"
    legacy = PROJECT_ROOT / "output" / f"events_{ts}.jsonl"
    return p if p.exists() or not legacy.exists() else legacy


def write_run_readme(rdir: Path, hints_applied: bool | None = None, note: str = "") -> None:
    """run 폴더에 후임용 README.txt — 어느 파일을 Artec Studio 로 여는지. 실패는 무시."""
    try:
        rdir.mkdir(parents=True, exist_ok=True)
        fix = {True: " + 보정", False: "", None: "(+ 보정)"}[hints_applied]
        lines = [
            f"MMS run {rdir.name}  (자동 생성 — utils/run_paths.py{' · ' + note if note else ''})",
            "",
            "aligned/aligned.sproj   <- Artec Studio 로 여는 파일. 모든 패스(lookaround·nbv·flip)의 IScan 이",
            f"                          파이프라인 변환(핸드아이 + 턴테이블 각 + flip 힌트{fix})으로 한 좌표계에",
            "                          놓여 있다. SDK 후처리(OutliersRemoval/GR/Fusion) 전이라 Studio 의",
            "                          Autopilot·Global Registration·Fusion 이 그대로 돈다.",
            "final/final.sproj       <- 파이프라인 후처리 결과(융합 메시 포함). 메시가 섞여 있어 Studio 정합",
            "                          도구의 입력으로는 부적합 — 비교용.",
            "final.obj               <- 위 융합 메시 (텍스처 없음. Studio 에서 Texture -> Export).",
            "timeline.csv · run.log · events.jsonl  <- 스캔 타임라인 · 콘솔 로그 · 판단 이벤트(scripts/artec/lost_report.py)",
            "scan_dumps/             <- IScan 별 점군 npz + 적용 변환 (오프라인 정합 재실험: scripts/artec/reg_*.py)",
            "debug/<단계>/           <- 거리 이미지(preview·lookaround·nbv·flip), 뷰어 녹화 range.avi, cam/ 카메라 스냅샷",
            "",
            "수작업 후처리 (정합이 완벽하지 않아 watertight 가 안 된 run):",
            "  1. Studio: File -> Open project -> aligned/aligned.sproj",
            "  2. Workspace 에서 scan 별 Err 확인 (정상 0.2~0.3mm; 1mm 이상이면 그 스캔의 미정합 프레임을 지운다)",
            "  3. Tools -> Global registration (필요하면 Align 으로 flip 스캔을 손으로 맞춘 뒤)",
            "  4. Tools -> Fusion(Sharp/Smooth) -> Fix holes -> Texture -> Export OBJ",
            "  자세한 절차·에러 뜻: docs/6_postprocess.md §4·§5",
            "",
        ]
        (rdir / "README.txt").write_text("\n".join(lines), encoding="utf-8")
    except Exception as e:                                   # noqa: BLE001
        print(f"   README 저장 실패 (무시): {e}")
