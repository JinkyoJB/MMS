"""export_env_mesh.py — 셀 구조물 메시를 **로봇 base 프레임**으로 구워 npz 캐시.

`utils/collision/env_collision.py` 가 이 캐시로 벽·상판·저울·툴스탠드 충돌을 본다.
base 프레임에 저장하므로 sim·real 공용이다(실물도 같은 CAD 를 같은 방식으로 구우면 됨).

    env -u PYTHONPATH ~/miniconda3/envs/env_isaacsim/bin/python \
        scripts/sim/export_env_mesh.py
    #  --exclude-turntable  (턴테이블 그룹 제외 — 기본 포함)
"""
from __future__ import annotations

import argparse
import math
import os
import sys

SCENE = ("/home/keti/workspace/sync/2_Rapid_Digital_Twin/1_MMS/2_3Dassets/"
         "frame_xarm7_spider_turntable_v2/v3_scene.usd")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--scene", default=SCENE)
    ap.add_argument("--out", default="utils/collision/data/cell_env.npz")
    ap.add_argument("--root", default="/World/frame")
    ap.add_argument("--robot", default="/World/xarm7")
    ap.add_argument("--spacing", type=float, default=0.005,
                    help="표면 샘플 간격 m. SDF 복셀(8mm)보다 촘촘해야 한다")
    ap.add_argument("--max-pts", type=int, default=2000000)
    ap.add_argument("--exclude-turntable", action="store_true",
                    help="턴테이블 뭉치 제외(별도 캡슐로 볼 때)")
    args = ap.parse_args()

    sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))
    from isaacsim import SimulationApp
    app = SimulationApp({"headless": True})

    import numpy as np
    from utils.collision.mesh_sampling import sample_surface
    from omni.isaac.core.utils.stage import open_stage
    from omni.usd import get_context
    from pxr import Usd, UsdGeom

    rng = np.random.default_rng(0)          # 결정적 샘플링
    open_stage(args.scene)
    for _ in range(60):
        app.update()
    stage = get_context().get_stage()
    xc = UsdGeom.XformCache()
    pred = Usd.TraverseInstanceProxies(Usd.PrimAllPrimsPredicate)

    def W(p):
        xc.Clear()
        return np.array(xc.GetLocalToWorldTransform(stage.GetPrimAtPath(p))).T

    T_WB = W(args.robot)
    Bi = np.linalg.inv(T_WB)
    TT_AXIS, TT_R, TT_ZR = 0.365, 0.16, (0.50, 0.70)

    P, names = [], []
    for d in Usd.PrimRange(stage.GetPrimAtPath(args.root), pred):
        if not d.IsA(UsdGeom.Mesh):
            continue
        m = UsdGeom.Mesh(d)
        pts = m.GetPointsAttr().Get() or []
        if not len(pts):
            continue
        A = np.array(xc.GetLocalToWorldTransform(d)).T
        # ★ 정점이 아니라 **표면**을 샘플링한다. 압출·판재는 정점이 모서리에만 있어
        #   정점만 담으면 부재 중간이 빈 공간이 된다(실측: 1m 기둥 중간에 점 0개
        #   → 스캐너가 관통). utils/collision/mesh_sampling 참고.
        V = sample_surface(np.asarray(pts, float),
                           m.GetFaceVertexCountsAttr().Get() or [],
                           m.GetFaceVertexIndicesAttr().Get() or [],
                           spacing_m=args.spacing, rng=rng)
        V = V @ A[:3, :3].T + A[:3, 3]                               # world
        c = V.mean(0)
        if (args.exclude_turntable
                and math.hypot(c[0] - TT_AXIS, c[1]) < TT_R
                and TT_ZR[0] < c[2] < TT_ZR[1]):
            continue
        P.append(V @ Bi[:3, :3].T + Bi[:3, 3])                      # → base
        names.append(str(d.GetPath()).rsplit("/", 1)[-1])
    E = np.vstack(P)
    E = E[:: max(1, len(E) // args.max_pts)]

    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    np.savez_compressed(args.out, env=E, names=np.array(sorted(set(names))))
    print(f"저장: {args.out}   {E.shape}  (메시 {len(names)}개)")
    print(f"  로봇 base(world) {np.round(T_WB[:3,3],3).tolist()}")
    print(f"  base 기준 범위 X{np.round([E[:,0].min(),E[:,0].max()],3).tolist()} "
          f"Y{np.round([E[:,1].min(),E[:,1].max()],3).tolist()} "
          f"Z{np.round([E[:,2].min(),E[:,2].max()],3).tolist()}")
    app.close()


if __name__ == "__main__":
    main()
