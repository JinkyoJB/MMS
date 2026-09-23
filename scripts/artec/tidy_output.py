# scripts/artec/tidy_output.py
#
# 옛 배치(≤2026-09-23 오전)로 흩어진 output/ 산출물을 run 폴더 규칙(utils/run_paths.py)으로 옮긴다.
#
#   output/artec_lookaround_<R>/…​.sproj        → output/<R>/final/final.sproj
#   output/artec_lookaround_<R>_raw/…_raw.sproj → output/<R>/aligned/aligned.sproj
#   output/artec_lookaround_<R>.obj             → output/<R>/final.obj
#   output/artec_lookaround_<R>_timeline.csv    → output/<R>/timeline.csv
#   output/events_<R>.jsonl · run_<R>.log       → output/<R>/events.jsonl · run.log
#   output/scan_dumps/<R>/                      → output/<R>/scan_dumps/
#   output/debug/<stage>_<R>/                   → output/<R>/debug/<stage>/
#   output/debug/range_<R>.avi · range_view_<R>.log → output/<R>/debug/range.avi · range_view.log
#   output/debug/preview/*.png                  → output/<R>/debug/cam/preview/      (R = 수정시각으로 배정)
#   output/debug/preview_points_HHMMSS.npz      → output/<R>/debug/                  (〃)
#   output/live_cloud_<TS>.ply                  → output/<R>/live_cloud.ply          (〃, 이름의 TS 는 쓴 시각)
#   비게 된 output/debug, scan_dumps, iso_debug 는 지운다. registration_test·reg_offline·_live_latest.* 는 건드리지 않는다.
#
#   python scripts/artec/tidy_output.py            # 계획만 출력
#   python scripts/artec/tidy_output.py --apply    # 실제 이동

from __future__ import annotations

import argparse
import re
import shutil
import sys
from datetime import datetime
from pathlib import Path

_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(_ROOT))
for _s in (sys.stdout, sys.stderr):
    try:
        _s.reconfigure(encoding="utf-8", errors="replace")
    except Exception:                                   # noqa: BLE001
        pass
from utils.run_paths import write_run_readme  # noqa: E402

OUT = _ROOT / "output"
RUN = r"(\d{8}_\d{6})"
LEAVE = {"registration_test", "reg_offline", "_norun"}


def _ts(r: str) -> datetime:
    return datetime.strptime(r, "%Y%m%d_%H%M%S")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--apply", action="store_true")
    a = ap.parse_args()
    moves: list[tuple[Path, Path]] = []          # (src, dst)
    renames_sproj: list[tuple[Path, str]] = []   # (sproj, 새 Name)

    def mv(src: Path, dst: Path):
        moves.append((src, dst))

    # ── 1. run 목록 — 파일/폴더 이름에서 RUN_TS 추출 ──────────────────────────
    runs: set[str] = set()
    for p in OUT.iterdir():
        if p.name.startswith("live_cloud_"):      # 라이브 뷰어 점군 — 이름의 시각은 run 시작이 아니다
            continue
        for m in re.finditer(RUN, p.name):
            runs.add(m.group(1))
    if (OUT / "debug").is_dir():
        for p in (OUT / "debug").iterdir():
            for m in re.finditer(RUN, p.name):
                runs.add(m.group(1))
    if (OUT / "scan_dumps").is_dir():
        runs |= {p.name for p in (OUT / "scan_dumps").iterdir() if re.fullmatch(RUN, p.name)}
    order = sorted(runs)

    def run_for_mtime(p: Path) -> str | None:
        """수정시각보다 앞서 시작한 run 중 가장 최근."""
        t = datetime.fromtimestamp(p.stat().st_mtime)
        cands = [r for r in order if _ts(r) <= t]
        return cands[-1] if cands else None

    # ── 2. 이름에 RUN_TS 가 박힌 것들 ────────────────────────────────────────
    for r in order:
        R = OUT / r
        d = OUT / f"artec_lookaround_{r}"
        if d.is_dir():
            mv(d, R / "final")
            sp = d / f"artec_lookaround_{r}.sproj"
            if sp.exists():
                mv(sp, R / "final" / "final.sproj"); renames_sproj.append((R / "final" / "final.sproj", "final"))
        d = OUT / f"artec_lookaround_{r}_raw"
        if d.is_dir():
            mv(d, R / "aligned")
            sp = d / f"artec_lookaround_{r}_raw.sproj"
            if sp.exists():
                mv(sp, R / "aligned" / "aligned.sproj"); renames_sproj.append((R / "aligned" / "aligned.sproj", "aligned"))
        for src, dst in ((OUT / f"artec_lookaround_{r}.obj", R / "final.obj"),
                         (OUT / f"artec_lookaround_{r}_timeline.csv", R / "timeline.csv"),
                         (OUT / f"events_{r}.jsonl", R / "events.jsonl"),
                         (OUT / f"run_{r}.log", R / "run.log"),
                         (OUT / "scan_dumps" / r, R / "scan_dumps"),
                         (OUT / "debug" / f"range_{r}.avi", R / "debug" / "range.avi"),
                         (OUT / "debug" / f"range_{r}.mp4", R / "debug" / "range.mp4"),
                         (OUT / "debug" / f"range_view_{r}.log", R / "debug" / "range_view.log")):
            if src.exists():
                mv(src, dst)
        if (OUT / "debug").is_dir():
            for d in (OUT / "debug").glob(f"*_{r}"):
                if d.is_dir():
                    stage = d.name[: -len(r) - 1]
                    mv(d, R / "debug" / stage)

    # ── 3. 이름에 run 이 없는 것 — 수정시각으로 배정 ─────────────────────────
    unassigned = []
    for p in sorted((OUT / "debug" / "preview").glob("*.png")) if (OUT / "debug" / "preview").is_dir() else []:
        r = run_for_mtime(p)
        (mv(p, OUT / r / "debug" / "cam" / "preview" / p.name) if r else unassigned.append(p))
    for p in sorted((OUT / "debug").glob("preview_points_*.npz")) if (OUT / "debug").is_dir() else []:
        r = run_for_mtime(p)
        (mv(p, OUT / r / "debug" / p.name) if r else unassigned.append(p))
    for p in sorted(OUT.glob("live_cloud_*.ply")):       # 이름의 시각은 RUN_TS 가 아니라 쓴 시각
        r = run_for_mtime(p)
        (mv(p, OUT / r / "live_cloud.ply") if r else unassigned.append(p))

    # ── 계획 출력 ─────────────────────────────────────────────────────────────
    print(f"run {len(order)}개: {', '.join(order)}")
    by_run: dict[str, int] = {}
    for src, dst in moves:
        by_run[dst.relative_to(OUT).parts[0]] = by_run.get(dst.relative_to(OUT).parts[0], 0) + 1
        print(f"  {src.relative_to(OUT)}  →  {dst.relative_to(OUT)}")
    for p in unassigned:
        print(f"  ⚠ 배정 못 함(어느 run 보다 앞섬): {p.relative_to(OUT)} → _norun/")
        mv(p, OUT / "_norun" / p.name)
    print("run 별 이동 수: " + ", ".join(f"{k}:{v}" for k, v in sorted(by_run.items())))
    if not a.apply:
        print("\n(계획만. 실제로 옮기려면 --apply)")
        return 0

    # ── 실행 — 폴더 이동 뒤 그 안의 sproj 이름 바꾸기 순서로 ─────────────────
    #   moves 에는 (폴더 → 새 폴더) 와 (옛 폴더 안의 sproj → 새 폴더 안의 sproj) 가 같이 있다.
    #   폴더를 먼저 옮기면 sproj 의 src 는 사라지므로, 옮긴 뒤 새 위치에서 이름만 바꾼다.
    done_dirs: list[tuple[Path, Path]] = []
    for src, dst in moves:
        if src.is_dir():
            dst.parent.mkdir(parents=True, exist_ok=True)
            if dst.exists():
                print(f"  ✘ 이미 있음, 건너뜀: {dst.relative_to(OUT)}"); continue
            shutil.move(str(src), str(dst)); done_dirs.append((src, dst))
    for src, dst in moves:
        if src.is_dir() or (src, dst) in done_dirs:
            continue
        # 옮겨진 폴더 안의 파일이면 새 위치로 환산
        real = src
        for od, nd in done_dirs:
            try:
                real = nd / src.relative_to(od); break
            except ValueError:
                continue
        if not real.exists():
            print(f"  ✘ 없음: {src.relative_to(OUT)}"); continue
        dst.parent.mkdir(parents=True, exist_ok=True)
        if dst.exists():
            print(f"  ✘ 이미 있음, 건너뜀: {dst.relative_to(OUT)}"); continue
        shutil.move(str(real), str(dst))
    for sp, name in renames_sproj:               # <ArtecProject Name="…"> 표시 이름만 맞춘다
        if sp.exists():
            t = sp.read_text(encoding="utf-8")
            t2 = re.sub(r'(<ArtecProject[^>]*\sName=")[^"]*(")', rf"\g<1>{name}\g<2>", t, count=1)
            if t2 != t:
                sp.write_text(t2, encoding="utf-8")
    for r in order:                              # README
        if (OUT / r).is_dir():
            write_run_readme(OUT / r, hints_applied=None, note="tidy_output.py 로 옛 배치에서 옮겨 온 run")
    for d in (OUT / "debug" / "preview", OUT / "debug" / "nbv", OUT / "debug", OUT / "scan_dumps", OUT / "iso_debug"):
        if d.is_dir() and not any(d.iterdir()):
            d.rmdir(); print(f"  빈 폴더 제거: {d.relative_to(OUT)}")
    left = [p.name for p in OUT.iterdir() if p.name not in LEAVE and not re.fullmatch(RUN, p.name)
            and not p.name.startswith("_live_latest")]
    print("\n완료. run 폴더 밖에 남은 것: " + (", ".join(left) if left else "없음"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
