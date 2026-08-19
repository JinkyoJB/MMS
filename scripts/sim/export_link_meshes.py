"""export_link_meshes.py — 링크/툴 메시를 **링크 로컬 좌표**로 샘플링해 npz 로 캐시.

`utils/collision/mesh_self_collision.py` 가 이 캐시를 읽어 캡슐 근사 없이 자가충돌을
판정한다. 한 번만 뽑아 두면 Isaac 없이도(실물 경로 포함) 쓸 수 있다.

    env -u PYTHONPATH ~/miniconda3/envs/env_isaacsim/bin/python \
        scripts/sim/export_link_meshes.py
    #  --scene <usd> --out utils/collision/data/xarm7_spider_links.npz
"""
from __future__ import annotations

import argparse
import os
import sys

SCENE = ("/home/keti/workspace/sync/2_Rapid_Digital_Twin/1_MMS/2_3Dassets/"
         "frame_xarm7_spider_turntable_v2/v3_scene.usd")
LINK7 = "/World/xarm7/link7"
TOOL = "/World/xarm7/link7/tool"


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--scene", default=SCENE)
    ap.add_argument("--out", default="utils/collision/data/xarm7_spider_links.npz")
    ap.add_argument("--spacing", type=float, default=0.004,
                    help="표면 샘플 간격 m (링크 SDF 복셀 6mm 보다 촘촘하게)")
    ap.add_argument("--link-pts", type=int, default=20000, help="링크당 목표 점 수")
    ap.add_argument("--tool-pts", type=int, default=30000, help="툴 목표 점 수")
    args = ap.parse_args()

    sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))
    from isaacsim import SimulationApp
    app = SimulationApp({"headless": True})

    import numpy as np
    from utils.collision.mesh_sampling import sample_surface
    from omni.isaac.core.utils.stage import open_stage
    from omni.usd import get_context
    from pxr import Usd, UsdGeom

    rng = np.random.default_rng(0)
    open_stage(args.scene)
    for _ in range(60):
        app.update()
    stage = get_context().get_stage()
    xc = UsdGeom.XformCache()
    pred = Usd.TraverseInstanceProxies(Usd.PrimAllPrimsPredicate)

    def W(p):
        xc.Clear()
        return np.array(xc.GetLocalToWorldTransform(stage.GetPrimAtPath(p))).T

    def local_verts(path, ref, skip_tool=False):
        Li = np.linalg.inv(W(ref))
        P = []
        for d in Usd.PrimRange(stage.GetPrimAtPath(path), pred):
            if not d.IsA(UsdGeom.Mesh):
                continue
            if skip_tool and "/tool" in str(d.GetPath()):
                continue
            A = Li @ np.array(xc.GetLocalToWorldTransform(d)).T
            m = UsdGeom.Mesh(d)
            q = m.GetPointsAttr().Get() or []
            if not len(q):
                continue
            # 정점이 아니라 표면 샘플링 — 평평한 부품(툴 플레이트 등)의 면 중앙이
            # 비는 것을 막는다(환경 쪽에서 실제 관통 사고를 냈던 원인).
            V = sample_surface(np.asarray(q, float),
                               m.GetFaceVertexCountsAttr().Get() or [],
                               m.GetFaceVertexIndicesAttr().Get() or [],
                               spacing_m=args.spacing, rng=rng)
            P.append(V @ A[:3, :3].T + A[:3, 3])
        return np.vstack(P) if P else np.zeros((0, 3))

    def thin(V, n):
        return V[:: max(1, len(V) // max(1, n))]

    out = {}
    for i in range(1, 8):
        V = local_verts(f"/World/xarm7/link{i}", f"/World/xarm7/link{i}", skip_tool=True)
        out[f"link{i}"] = thin(V, args.link_pts)
    out["tool"] = thin(local_verts(TOOL, LINK7), args.tool_pts)
    # 손끝 규약 확인용 — 툴 체인은 link7 +Z 로 뻗는다(실측 -4~+265mm).
    out["tool_z_range"] = np.array([out["tool"][:, 2].min(), out["tool"][:, 2].max()])

    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    np.savez_compressed(args.out, **out)
    print("저장:", args.out)
    for k in sorted(out):
        v = out[k]
        print(f"  {k:10s} {v.shape}")
    app.close()


if __name__ == "__main__":
    main()
