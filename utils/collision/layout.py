"""layout.py — 충돌 환경 캐시(셀 레이아웃) **선택**.

왜 필요한가
-----------
`cell_env.npz` 는 코드가 읽는 활성본 **하나**뿐이고, 변형은
`layouts/cell_env.<별칭>.npz` 에 쌓아 `scripts/collision/use_layout.py` 로
갈아끼운다(`docs/collision.md` §6). 그런데 활성본이 **전역 파일 하나**라
백엔드를 바꿔도 따라오지 않는다.

  2026-09-17 실측 — 이게 sim 을 통째로 죽이고 있었다.
  2026-09-15 에 활성본이 `v2_layout_real`(실측 셀, base [0.406, 0, 1.407])로
  바뀌었는데, sim 은 여전히 v3 씬(base [0.365, 0, 1.500])을 띄운다. base 가
  41mm/93mm 어긋나니 **sim home 자세의 link5/6 이 셀 구조물 안**에 들어간다
  (env 거리 0.0mm / 14.5mm). 그 결과 계획용 preview 16회가 전부
  `이동 거부 — start(link6)` 로 거부되고 → preview 0점 → 플래너 실패 →
  legacy 폴백도 거부 → **lookaround 이 한 점도 못 얻고 끝났다.**
  로그에는 "자세 자체가 충돌" 만 찍혀서 원인이 레이아웃인 줄 알 수가 없었다.

그래서 **백엔드가 레이아웃을 고른다.** sim 은 자기 씬에서 구운 것을, real 은
실측한 것을 쓴다. 활성본(`cell_env.npz`)은 별칭이 없을 때의 폴백으로 남는다.

우선순위:
  1. 명시 인자 `layout=`
  2. 환경변수 `MMS_COLLISION_LAYOUT` (백엔드 팩토리가 setdefault 로 건다)
  3. 활성본 `cell_env.npz`

⚠ `layouts/*.npz` 는 `.gitignore(*.npz)` 대상이라 새 머신에는 없다. 없으면
  말없이 활성본으로 떨어지지 않고 **경고를 찍는다** — 조용한 폴백이 위 사고의
  본질이었다. 생성법은 `layouts/README.md`.
"""
from __future__ import annotations

import os

_HERE = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.join(_HERE, "data")
ACTIVE_NPZ = os.path.join(DATA, "cell_env.npz")
LAYOUTS_DIR = os.path.join(DATA, "layouts")
ENV_VAR = "MMS_COLLISION_LAYOUT"

_warned = set()


def layout_path(name: str) -> str:
    return os.path.join(LAYOUTS_DIR, f"cell_env.{name}.npz")


def active_name() -> str:
    """활성본이 어느 별칭인지 (마커 파일). 없으면 '?'."""
    try:
        with open(os.path.join(DATA, "ACTIVE_LAYOUT.txt"), encoding="utf-8") as f:
            return f.read().strip() or "?"
    except OSError:
        return "?"


def resolve_env_npz(layout: str = None, log=print) -> str:
    """쓸 셀 레이아웃 npz 경로. 위 우선순위대로 고른다."""
    name = layout or os.environ.get(ENV_VAR) or ""
    name = name.strip()
    if not name:
        return ACTIVE_NPZ
    p = layout_path(name)
    if os.path.isfile(p):
        if name not in _warned:
            _warned.add(name)
            log(f"[collision] 셀 레이아웃 '{name}' 사용 "
                f"(활성본 '{active_name()}' 대신)")
        return p
    if name not in _warned:
        _warned.add(name)
        log(f"[collision] ⚠ 레이아웃 '{name}' 캐시 없음 ({p}) — "
            f"활성본 '{active_name()}' 으로 폴백한다. 백엔드와 셀이 다르면 "
            f"자세가 통째로 '충돌'로 거부될 수 있다 "
            f"(생성: utils/collision/data/layouts/README.md)")
    return ACTIVE_NPZ
