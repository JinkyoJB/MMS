"""overlay_obstacle.py — 등록된 **대상물 장애물**을 씬에 겹쳐 본다 (GUI 없이도).

왜 필요한가
-----------
`set_dynamic_obstacle` 에 넘어간 점군이 물체를 제대로 감쌌는지는 **숫자로는
확인이 안 된다.** bbox·점 수는 회전체가 엉뚱한 방향을 향해도 그럴듯하게 나오고,
프레임(base↔world)을 틀려도 크기는 맞는다. 눈으로 봐야 한다.

sim 실행이 `output/debug/obstacle/<시각>_<태그>.npz` 로 덤프해 둔다:

    raw_b       등록 직전 점군 (base, θ=0 canonical)
    swept_b     축 둘레로 쓸어 실제 등록한 회전체 (base)  ← 이게 충돌 게이트가 보는 것
    axis_b/dir  턴테이블 축 (base)
    T_WB        base→world (씬에 올릴 때 필요)
    margin_m    여유 (실효 keepout = 점군 + 이만큼)

사용:
    # 씬 USD 옆에 오버레이 USD 를 만든다 (Isaac 불필요, pxr 만)
    env -u PYTHONPATH ~/miniconda3/envs/step2usd/bin/python \\
        scripts/sim/overlay_obstacle.py --npz output/debug/obstacle/<파일>.npz

    # 만들어진 USD 를 Isaac/usdview 로 열면 씬 위에 겹쳐 보인다
"""
from __future__ import annotations

import argparse
import glob
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))

from mms_paths import asset                                    # noqa: E402

DEFAULT_SCENE = asset("frame_xarm7_spider_turntable/v2_real_260917.usd")


def _points(stage, path, pts, color, width, opacity):
    from pxr import UsdGeom, Vt, Gf
    g = UsdGeom.Points.Define(stage, path)
    g.GetPointsAttr().Set(Vt.Vec3fArray([Gf.Vec3f(*map(float, v)) for v in pts]))
    g.GetWidthsAttr().Set(Vt.FloatArray([float(width)] * len(pts)))
    g.GetDisplayColorAttr().Set(Vt.Vec3fArray([Gf.Vec3f(*color)] * len(pts)))
    g.GetDisplayOpacityAttr().Set(Vt.FloatArray([float(opacity)] * len(pts)))


def main() -> None:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--npz", default=None, help="덤프 (기본: 가장 최근 것)")
    ap.add_argument("--scene", default=DEFAULT_SCENE)
    ap.add_argument("--out", default=None, help="출력 USD (기본: 덤프 옆)")
    ap.add_argument("--max-pts", type=int, default=40000)
    a = ap.parse_args()

    npz = a.npz
    if npz is None:
        c = sorted(glob.glob(os.path.join(
            os.path.dirname(os.path.abspath(__file__)), "..", "..",
            "output", "debug", "obstacle", "*.npz")))
        if not c:
            raise SystemExit("덤프가 없다 — sim 을 한 번 돌릴 것 "
                             "(MMS_SIM_OBSTACLE_DUMP=1, 기본 켜짐)")
        npz = c[-1]
    d = np.load(npz)
    raw, swept = np.asarray(d["raw_b"], float), np.asarray(d["swept_b"], float)
    T = np.asarray(d["T_WB"], float)
    margin = float(d["margin_m"]) if "margin_m" in d else 0.030

    def to_w(p):
        return p @ T[:3, :3].T + T[:3, 3]

    def thin(p):
        return p if len(p) <= a.max_pts else p[:: max(1, len(p) // a.max_pts)]

    out = a.out or (os.path.splitext(npz)[0] + "_overlay.usd")
    from pxr import Usd, UsdGeom, Sdf
    if os.path.exists(out):
        os.remove(out)
    stage = Usd.Stage.CreateNew(out)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    root = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(root.GetPrim())
    # 씬을 참조로 얹는다 — 원본은 안 건드린다
    rel = os.path.relpath(os.path.abspath(a.scene), os.path.dirname(os.path.abspath(out)))
    stage.OverridePrim("/World/scene").GetReferences().AddReference(
        rel.replace(os.sep, "/"))
    # 등록 전(흰색, 얇게) · 실제 등록한 회전체(주황)
    _points(stage, "/World/_obstacle/raw", thin(to_w(raw)), (1, 1, 1), 0.003, 0.9)
    _points(stage, "/World/_obstacle/swept", thin(to_w(swept)), (1.0, 0.25, 0.0),
            0.004, 0.35)
    stage.GetRootLayer().Save()

    print(f"[overlay] {os.path.basename(npz)}")
    print(f"  등록 전(흰)  {len(raw):,}점")
    print(f"  등록됨(주황) {len(swept):,}점  ← 충돌 게이트가 보는 것")
    print(f"  여유 {margin*1000:.0f}mm  → 실효 keepout = 주황 표면 + {margin*1000:.0f}mm")
    lo, hi = swept.min(0), swept.max(0)
    print(f"  base bbox  x[{lo[0]:.3f},{hi[0]:.3f}] y[{lo[1]:.3f},{hi[1]:.3f}] "
          f"z[{lo[2]:.3f},{hi[2]:.3f}]")
    print(f"\n  → {out}\n    Isaac 또는 usdview 로 열면 씬 위에 겹쳐 보인다.")


if __name__ == "__main__":
    main()
