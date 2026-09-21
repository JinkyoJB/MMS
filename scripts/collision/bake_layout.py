#!/usr/bin/env python
"""bake_layout.py — **하드웨어 레이아웃이 바뀌었을 때** 충돌 캐시를 다시 굽는다.

언제 쓰나
--------
셀을 개조했을 때 — 테이블·벽·툴스탠드를 옮기거나, 로봇 마운트 위치가 바뀌거나,
CAD 를 새로 그렸을 때. 즉 **`cell_env.npz` 의 기준이 되는 형상 자체**가 변한 경우다.

  턴테이블만 조금 틀어진 경우는 이걸 쓸 필요가 없다 — 캘리브(`calibrate.py`)의
  4단계가 알아서 미세조정한다. 이 스크립트는 형상이 바뀐 경우용이다.

무엇을 하나
----------
    USD 씬 ──(표면 샘플, base 프레임)──▶ cell_env.<별칭>.npz ──▶ 활성화

  그리고 **그때의 `T_B_F0` 를 meta 에 적어둔다.** 나중에 턴테이블만 재캘리브했을 때
  "캐시가 어느 턴테이블 자리를 기준으로 구워졌나"를 알아야 미세조정이 가능하다.

    python scripts/collision/bake_layout.py --scene <씬.usd> --alias v2_real_261015
    python scripts/collision/bake_layout.py --list          # 있는 레이아웃 보기

⚠ **Isaac Sim 파이썬**이 필요하다 (USD 를 읽어야 한다). 없으면 자산·Isaac 이 있는
  PC 에서 굽고, 나온 `cell_env.<별칭>.npz` 파일만 현장 PC 로 복사해도 된다
  (`utils/collision/data/layouts/` 에 두고 `use_layout.py <별칭>`).
"""
from __future__ import annotations

import argparse
import datetime as _dt
import os
import subprocess
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO))

DATA = REPO / "utils" / "collision" / "data"
LAYOUTS = DATA / "layouts"
META = DATA / "cell_env.meta.yaml"

#: 셀에 따라 구조물과 턴테이블이 **다른 루트**에 있다. v2 실물 씬 기준 기본값.
DEFAULT_ROOTS = ["/World/Frame/frame_structure", "/World/frame"]

ISAAC_CANDIDATES = [
    os.environ.get("MMS_ISAAC_PY", ""),
    str(Path.home() / "miniconda3" / "envs" / "env_isaacsim" / "python.exe"),
    str(Path.home() / "miniconda3" / "envs" / "env_isaacsim" / "bin" / "python"),
]


def isaac_python() -> Path | None:
    for p in ISAAC_CANDIDATES:
        if p and Path(p).is_file():
            return Path(p)
    return None


def current_T_B_F0():
    """지금 캘리브된 턴테이블 축 — meta 에 적어둘 값. 없으면 None."""
    try:
        from utils.transforms import load_transform, TurntableTransformConfig
        tt = TurntableTransformConfig(load_transform(
            str(REPO / "config" / "calibration" / "turntable_frame.yaml"), "T_B_F0"))
        import numpy as np
        return ([float(x) for x in np.asarray(tt.axis_point_B, float)],
                [float(x) for x in np.asarray(tt.axis_dir_B, float)])
    except Exception as e:                                   # noqa: BLE001
        print(f"  ⚠ T_B_F0 를 못 읽었다({type(e).__name__}) — meta 없이 진행")
        return None


def write_meta(alias: str, scene: str) -> None:
    """★ **캐시가 어느 턴테이블 자리를 기준으로 구워졌는지** 기록한다.

    이게 없으면 나중에 턴테이블만 재캘리브했을 때 "캐시 안의 턴테이블을 어디서
    어디로 옮겨야 하나"를 알 수 없어 미세조정이 불가능하다 (캐시는 그냥 점 뭉치라
    어느 점이 턴테이블인지 스스로 말해주지 않는다).
    """
    import yaml
    axis = current_T_B_F0()
    meta = {"alias": alias, "scene": scene,
            "baked": _dt.date.today().isoformat()}
    if axis is not None:
        meta["T_B_F0_at_bake"] = {"axis_point_B": axis[0], "axis_dir_B": axis[1]}
    META.write_text(yaml.safe_dump(meta, allow_unicode=True, sort_keys=False),
                    encoding="utf-8")
    print(f"  meta 기록 → {META.relative_to(REPO)}")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--scene", help="구울 USD 씬 (기본: 현재 meta 의 씬)")
    ap.add_argument("--alias", help="레이아웃 별칭 (기본: layout_<YYMMDD>)")
    ap.add_argument("--root", nargs="+", default=DEFAULT_ROOTS,
                    help="샘플링할 prim 루트들 — 구조물과 턴테이블이 다른 루트일 수 있다")
    ap.add_argument("--no-activate", action="store_true", help="굽기만 하고 활성화는 안 함")
    ap.add_argument("--list", action="store_true", help="있는 레이아웃 목록만 보기")
    a = ap.parse_args()

    if a.list:
        return subprocess.run(
            [sys.executable, str(REPO / "scripts/collision/use_layout.py"), "--list"],
            cwd=REPO).returncode

    if not a.scene:
        print("✘ --scene 이 필요하다 (구울 USD 씬).")
        print("   셀 형상을 새로 만들려면 먼저: scripts/sim/build_scene_v2_real.py")
        return 2
    alias = a.alias or f"layout_{_dt.date.today():%y%m%d}"

    py = isaac_python()
    print("=" * 62)
    print("  레이아웃 베이크 — USD → 충돌 캐시")
    print("=" * 62)
    print(f"  Isaac 파이썬 : {py or '없음'}")
    print(f"  씬           : {a.scene}")
    print(f"  별칭         : {alias}")
    if py is None:
        print("\n  ✘ Isaac Sim 파이썬이 없다 — USD 를 읽을 수 없다.")
        print("     MMS_ISAAC_PY 로 지정하거나, 자산 있는 PC 에서 굽고")
        print("     나온 npz 를 utils/collision/data/layouts/ 로 복사할 것.")
        return 2
    if not Path(a.scene).exists():
        print(f"\n  ✘ 씬 파일 없음: {a.scene}")
        return 2

    LAYOUTS.mkdir(parents=True, exist_ok=True)
    out = LAYOUTS / f"cell_env.{alias}.npz"
    env = {**os.environ}
    env.pop("PYTHONPATH", None)      # Isaac 확장이 리포 utils 를 가리는 것 방지
    cmd = [str(py), str(REPO / "scripts/sim/export_env_mesh.py"),
           "--scene", a.scene, "--out", str(out), "--root", *a.root]
    print(f"\n  $ {' '.join(cmd)}\n")
    if subprocess.run(cmd, cwd=REPO, env=env).returncode != 0:
        print("\n✘ 베이크 실패"); return 1

    write_meta(alias, a.scene)
    if a.no_activate:
        print(f"\n✓ 구웠다 → {out.relative_to(REPO)}  (활성화는 안 함)")
        print(f"  활성화:  python scripts/collision/use_layout.py {alias}")
        return 0
    if subprocess.run([sys.executable, str(REPO / "scripts/collision/use_layout.py"),
                       alias], cwd=REPO).returncode != 0:
        print("\n✘ 활성화 실패"); return 1
    # 형상을 새로 구웠으면 '낡았다' 표식은 의미가 없다.
    stale = DATA / "STALE_COLLISION.txt"
    if stale.exists():
        stale.unlink()
    print(f"\n✓ 레이아웃 '{alias}' 활성화 완료")
    print("  검산:  python scripts/artec/check_calibration.py")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
