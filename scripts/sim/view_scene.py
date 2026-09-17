#!/usr/bin/env python
"""view_scene.py — sim 씬 USD 를 Isaac Sim GUI 로 열어 **눈으로 본다**.

용도: CAD 기반 씬(v2/v3/v4)이 실물 셀과 얼마나 다른지 확인 (docs/collision.md §6).
로봇을 구동하지도, 물리를 돌리지도 않는다 — stage 를 열고 대기만 한다.

    python scripts/sim/view_scene.py            # v3 (기본)
    python scripts/sim/view_scene.py v2         # 구 씬
    python scripts/sim/view_scene.py v4         # 턴테이블 이설 검토안 (hw_layout §3)
    python scripts/sim/view_scene.py <경로.usd>

⚠ **`env_isaacsim` 에서 돌린다** (`mms-env` 에는 Isaac 이 없다):
    & "$env:USERPROFILE\miniforge3\envs\env_isaacsim\python.exe" scripts/sim/view_scene.py

첫 실행은 익스텐션·셰이더 캐시를 받느라 8분쯤 걸린다. 이후는 캐시를 쓴다.
창을 닫거나 Ctrl+C 로 종료.
"""
from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(_ROOT))

from mms_paths import asset

#: 이름 → 씬 경로. README §sim/real 듀얼 백엔드 참고.
SCENES = {
    "v2": "frame_xarm7_spider_turntable/v2.usd",
    "v3": "frame_xarm7_spider_turntable_v2/v3_scene.usd",
    "v4": "frame_xarm7_spider_turntable_v2/v4_scene.usd",
}


def resolve(name: str) -> str:
    if name in SCENES:
        return asset(SCENES[name])
    p = Path(name)
    return str(p if p.is_absolute() else asset(name))


def main() -> None:
    ap = argparse.ArgumentParser(description="sim 씬 USD 를 GUI 로 열어 본다")
    ap.add_argument("scene", nargs="?", default="v3",
                    help=f"{'|'.join(SCENES)} 또는 .usd 경로 (기본 v3)")
    ap.add_argument("--headless", action="store_true",
                    help="창 없이 열기만 (로드 성공 여부 확인용)")
    args = ap.parse_args()

    path = resolve(args.scene)
    if not os.path.exists(path):
        print(f"[view_scene] 씬이 없다: {path}\n"
              f"  자산 내려받기는 README §환경 '자산(USD) 내려받기' 참고", file=sys.stderr)
        sys.exit(1)
    print(f"[view_scene] {args.scene} → {path}")

    # EULA 는 최초 1회 필요. 대화형 프롬프트를 피하려고 여기서 세운다.
    os.environ.setdefault("OMNI_KIT_ACCEPT_EULA", "YES")

    from isaacsim import SimulationApp
    app = SimulationApp({"headless": bool(args.headless)})

    # ⚠ omni.usd 를 직접 쓴다. `omni.isaac.core.utils.stage` 는 5.x 에서 없어진
    #   구 네임스페이스라(export_env_mesh.py 가 아직 그걸 쓴다) 여기서는 피한다.
    import omni.usd
    ctx = omni.usd.get_context()
    ctx.open_stage(path)
    for _ in range(120):                 # 메시·머티리얼 로드될 때까지 돌린다
        app.update()

    stage = ctx.get_stage()
    if stage is None:
        print("[view_scene] stage 로드 실패", file=sys.stderr)
        app.close()
        sys.exit(1)

    from pxr import Usd, UsdGeom
    pred = Usd.TraverseInstanceProxies(Usd.PrimAllPrimsPredicate)
    meshes = [p for p in Usd.PrimRange(stage.GetPseudoRoot(), pred)
              if p.IsA(UsdGeom.Mesh)]
    print(f"[view_scene] 로드 완료 — 메시 {len(meshes)}개")
    for p in ("/World/xarm7", "/World/frame", "/World/frame/turntable_disc"):
        print(f"    {'OK ' if stage.GetPrimAtPath(p).IsValid() else '없음'} {p}")

    if args.headless:
        app.close()
        return

    print("[view_scene] 창을 닫으면 종료. 마우스 우클릭 드래그=회전, 휠=줌")
    try:
        while app.is_running():
            app.update()
    except KeyboardInterrupt:
        print("\n[view_scene] Ctrl+C")
    finally:
        app.close()


if __name__ == "__main__":
    main()
