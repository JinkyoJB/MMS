"""overlay_env_npz.py — 충돌 점군(npz)을 씬 위에 **겹쳐 본다**.

왜 필요한가
-----------
충돌 캐시(`utils/collision/data/*.npz`)는 로봇 base 프레임 점군이라 숫자로만 보면
어디가 맞고 어디가 틀린지 알 수 없다. 씬에 겹쳐 놓으면 눈으로 바로 갈린다 —
"실측본은 여기 부재가 있는데 우리 씬엔 없다" 같은 것.

⚠ **어느 npz 를 겹치는지가 요점이다.**
  - `cell_env.v2_real_260917.npz` 는 이 씬에서 **구운 것**이라 당연히 일치한다.
    (겹쳐도 "잘 만들어졌나" 확인 이상은 안 된다)
  - `cell_env.npz` (= `v2_layout_real`) 는 **현장 실측 반영본**이다. 이걸 겹쳐야
    우리 씬이 실물과 어디서 갈리는지 보인다. 기본값이 이쪽인 이유다.

  우리 씬은 실측본과 **일부러 다른 곳**이 있다(둘 다 의도한 것):
    · 턴테이블 — 실측본은 v2 placeholder(반경 75mm), 우리 씬은 v3 CAD 조립체(119mm)
    · `xarm7_ceiling_mount` / `tn__HFS84080600_2_jJ9` — 2026-09-17 현장 보정분

쓰는 법 (둘 중 하나)
--------------------
**A. 이미 띄워둔 Isaac Sim 창에 바로** — `--snippet` 으로 코드를 찍어서
   Isaac Sim 의 `Window → Script Editor` 에 붙여넣고 실행한다. 창을 다시 안 띄워도 된다.

    env -u PYTHONPATH ~/miniconda3/envs/mms-env/bin/python \
        scripts/sim/overlay_env_npz.py --snippet

**B. 오버레이 USD 를 만들어서 연다** — 씬을 subLayer 로 깔고 점군만 얹은 파일.

    env -u PYTHONPATH ~/miniconda3/envs/step2usd/bin/python \
        scripts/sim/overlay_env_npz.py --write

좌표 주의
---------
npz 는 **로봇 base 프레임**이다. 씬은 world 라 `/World/xarm7` 의 world 자세로 변환해야
한다. 값을 박아 넣지 않고 **스테이지에서 읽는다** — base 가 옮겨져도 따라온다.
"""
from __future__ import annotations

import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
from mms_paths import asset                                      # noqa: E402

_REPO = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..")
DEFAULT_NPZ = os.path.abspath(
    os.path.join(_REPO, "utils/collision/data/cell_env.npz"))     # 현장 실측본
DEFAULT_SCENE = asset("frame_xarm7_spider_turntable/v2_real_260917.usd")
OVERLAY_PRIM = "/World/_env_overlay"

SNIPPET = '''# ── 충돌 점군 오버레이 (Isaac Sim Script Editor 에 붙여넣기) ──────────────
# npz = 로봇 base 프레임 → /World/xarm7 의 world 자세로 변환해 Points 로 얹는다.
import numpy as np, omni.usd
from pxr import Usd, UsdGeom, Gf, Vt

NPZ      = r"{npz}"
ROBOT    = "/World/xarm7"
PRIM     = "{prim}"
MAX_PTS  = {max_pts}          # 뷰포트 부담 — 이 수로 솎는다
COLOR    = ({r}, {g}, {b})
WIDTH    = {width}            # 점 크기 (m)

stage = omni.usd.get_context().get_stage()
xc = UsdGeom.XformCache()
T = np.array(xc.GetLocalToWorldTransform(stage.GetPrimAtPath(ROBOT))).T   # base→world

E = np.load(NPZ)["env"]
if len(E) > MAX_PTS:
    E = E[:: max(1, len(E) // MAX_PTS)]
W = E @ T[:3, :3].T + T[:3, 3]

if stage.GetPrimAtPath(PRIM):
    stage.RemovePrim(PRIM)                       # 다시 실행하면 갈아끼운다
pts = UsdGeom.Points.Define(stage, PRIM)
pts.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(*map(float, p)) for p in W]))
pts.CreateWidthsAttr(Vt.FloatArray([WIDTH] * len(W)))
pts.CreateDisplayColorAttr(Vt.Vec3fArray([Gf.Vec3f(*COLOR)]))
UsdGeom.Imageable(pts).MakeVisible()
print(f"[overlay] {{len(W):,}}점 표시 → {{PRIM}}  (base→world 적용, 원본 {{len(np.load(NPZ)['env']):,}}점)")
# 지우려면:  stage.RemovePrim("{prim}")
# ─────────────────────────────────────────────────────────────────────────'''


def main() -> None:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--npz", default=DEFAULT_NPZ,
                    help="겹칠 점군 npz (기본 = 현장 실측본 cell_env.npz)")
    ap.add_argument("--scene", default=DEFAULT_SCENE, help="--write 시 바탕 씬")
    ap.add_argument("--out", default=None, help="--write 출력 USD (기본 <scene>_overlay.usd)")
    ap.add_argument("--max-pts", type=int, default=250_000, help="표시 점 수 상한")
    ap.add_argument("--color", type=float, nargs=3, default=(1.0, 0.25, 0.1),
                    metavar=("R", "G", "B"))
    ap.add_argument("--width", type=float, default=0.004, help="점 크기 m")
    ap.add_argument("--snippet", action="store_true",
                    help="이미 띄워둔 Isaac Sim 창에 붙여넣을 코드를 출력한다")
    ap.add_argument("--write", action="store_true", help="오버레이 USD 를 만든다")
    a = ap.parse_args()

    npz = os.path.abspath(a.npz)
    if not os.path.isfile(npz):
        raise SystemExit(f"✘ npz 없음: {npz}")
    n_all = len(np.load(npz)["env"])
    print(f"[overlay] npz {npz}\n[overlay]   {n_all:,}점 "
          f"→ 표시 {min(n_all, a.max_pts):,}점")

    if a.snippet or not a.write:
        print("\n" + SNIPPET.format(
            npz=npz, prim=OVERLAY_PRIM, max_pts=a.max_pts,
            r=a.color[0], g=a.color[1], b=a.color[2], width=a.width) + "\n")
        if not a.write:
            return

    # ── 오버레이 USD ────────────────────────────────────────────────────
    from pxr import Usd, UsdGeom, Gf, Vt
    out = a.out or (os.path.splitext(a.scene)[0] + "_overlay.usd")
    scene_rel = "./" + os.path.relpath(
        os.path.abspath(a.scene), os.path.dirname(os.path.abspath(out)))
    if os.path.exists(out):
        os.remove(out)
    stage = Usd.Stage.CreateNew(out)
    stage.GetRootLayer().subLayerPaths.append(scene_rel)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    stage.SetDefaultPrim(stage.GetPrimAtPath("/World"))

    xc = UsdGeom.XformCache()
    T = np.array(xc.GetLocalToWorldTransform(
        stage.GetPrimAtPath("/World/xarm7"))).T                  # base→world
    E = np.load(npz)["env"]
    if len(E) > a.max_pts:
        E = E[:: max(1, len(E) // a.max_pts)]
    W = E @ T[:3, :3].T + T[:3, 3]

    pts = UsdGeom.Points.Define(stage, OVERLAY_PRIM)
    pts.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(*map(float, p)) for p in W]))
    pts.CreateWidthsAttr(Vt.FloatArray([float(a.width)] * len(W)))
    pts.CreateDisplayColorAttr(Vt.Vec3fArray([Gf.Vec3f(*[float(c) for c in a.color])]))
    stage.GetRootLayer().Save()
    print(f"[overlay] 저장 → {out}\n[overlay]   base(world)={np.round(T[:3, 3], 4)}  "
          f"점 {len(W):,}개")


if __name__ == "__main__":
    main()
