#!/usr/bin/env python
"""use_layout.py - 충돌 환경 캐시(`cell_env.npz`)를 **별칭으로 갈아끼운다**.

왜 필요한가
-----------
셀 레이아웃은 계속 바뀌는데(`docs/4_collision.md` §6), 충돌 코드는 경로가
`utils/collision/data/cell_env.npz` 하나로 박혀 있다. 그래서 **변형들을 옆에 쌓아두고
활성본만 그 이름으로 복사**한다. 코드는 건드리지 않는다.

    utils/collision/data/
      cell_env.npz                         ← 활성본 (코드가 읽는 것)
      ACTIVE_LAYOUT.txt                    ← 지금 활성 별칭
      layouts/
        cell_env.v3_layout_sim.npz         ← CAD v3_scene 기준 (2026-04 이전 셀)
        cell_env.v2_layout_real.npz        ← v2_real.usd - 2026-09-15 현장 실측
        cell_env.<별칭>.npz                 ← 얼마든지 추가

사용
----
    python scripts/collision/use_layout.py                    # 목록 + 현재 활성
    python scripts/collision/use_layout.py v2_layout_real     # 갈아끼우기
    python scripts/collision/use_layout.py --diff v3_layout_sim v2_layout_real

[!] 심볼릭 링크가 아니라 **복사**다. Windows 에서 링크는 관리자 권한이 필요하고,
  np.load 가 링크를 따라가다 깨지는 경우를 피한다.
"""
from __future__ import annotations

import argparse
import shutil
import sys
from pathlib import Path

import numpy as np

_DATA = Path(__file__).resolve().parents[2] / "utils" / "collision" / "data"
_LAYOUTS = _DATA / "layouts"
_ACTIVE = _DATA / "cell_env.npz"
_MARKER = _DATA / "ACTIVE_LAYOUT.txt"

PREFIX, SUFFIX = "cell_env.", ".npz"


def available() -> dict[str, Path]:
    if not _LAYOUTS.is_dir():
        return {}
    return {p.name[len(PREFIX):-len(SUFFIX)]: p
            for p in sorted(_LAYOUTS.glob(f"{PREFIX}*{SUFFIX}"))}


def describe(path: Path) -> str:
    """점 수와 base 프레임 범위 - 갈아끼우기 전에 눈으로 확인하라고."""
    try:
        d = np.load(path)
        E = d["env"]
        rng = " ".join(f"{a}[{E[:, i].min():+.3f},{E[:, i].max():+.3f}]"
                       for i, a in enumerate("XYZ"))
        return f"{len(E):>9,}pts  {rng}"
    except Exception as e:                                   # noqa: BLE001
        return f"읽기 실패: {type(e).__name__}: {e}"


def current() -> str:
    if _MARKER.exists():
        return _MARKER.read_text(encoding="utf-8").strip()
    return "(알 수 없음 - 이 도구 없이 바뀐 적이 있다)"


def cmd_list() -> None:
    avail = available()
    cur = current()
    print(f"활성: {cur}")
    if _ACTIVE.exists():
        print(f"  {_ACTIVE.name}  {describe(_ACTIVE)}")
    print(f"\n사용 가능 ({_LAYOUTS}):")
    if not avail:
        print("  (없음)")
    for name, p in avail.items():
        print(f"  {'*' if name == cur else ' '} {name:<20} {describe(p)}")
    print("\n갈아끼우기:  python scripts/collision/use_layout.py <별칭>")


def cmd_use(name: str) -> None:
    avail = available()
    if name not in avail:
        print(f"'{name}' 없음. 있는 것: {', '.join(avail) or '(없음)'}", file=sys.stderr)
        sys.exit(1)
    src = avail[name]
    # 활성본이 어느 별칭에도 없는 내용이면 덮어쓰기 전에 살려둔다.
    if _ACTIVE.exists() and current() not in avail:
        rescue = _LAYOUTS / f"{PREFIX}unknown_backup{SUFFIX}"
        if not rescue.exists():
            shutil.copy2(_ACTIVE, rescue)
            print(f"[!] 정체불명 활성본을 살려둠 -> {rescue.name}")
    shutil.copy2(src, _ACTIVE)
    _MARKER.write_text(name + "\n", encoding="utf-8")
    print(f"활성 = {name}\n  {describe(_ACTIVE)}")
    print("\n[!] 실행 중인 프로세스에는 반영되지 않는다 - "
          "`collision_model.get_default` 가 결과를 캐시한다. 다시 띄울 것.")


def cmd_diff(a: str, b: str) -> None:
    avail = available()
    for n in (a, b):
        if n not in avail:
            print(f"'{n}' 없음", file=sys.stderr)
            sys.exit(1)
    Ea, Eb = np.load(avail[a])["env"], np.load(avail[b])["env"]
    print(f"{a:<20} {describe(avail[a])}")
    print(f"{b:<20} {describe(avail[b])}")
    print(f"\n점 수 차이: {len(Eb) - len(Ea):+,}")
    for i, ax in enumerate("XYZ"):
        print(f"  {ax} 범위 이동: "
              f"min {Eb[:, i].min() - Ea[:, i].min():+.3f}m  "
              f"max {Eb[:, i].max() - Ea[:, i].max():+.3f}m")


def main() -> None:
    ap = argparse.ArgumentParser(description="충돌 환경 캐시 별칭 전환")
    ap.add_argument("alias", nargs="?", help="활성으로 만들 별칭 (생략하면 목록)")
    ap.add_argument("--diff", nargs=2, metavar=("A", "B"), help="두 레이아웃 비교")
    args = ap.parse_args()
    if args.diff:
        cmd_diff(*args.diff)
    elif args.alias:
        cmd_use(args.alias)
    else:
        cmd_list()


if __name__ == "__main__":
    main()
