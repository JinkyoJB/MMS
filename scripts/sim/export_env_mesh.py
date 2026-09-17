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

import os, sys; sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))
from mms_paths import asset, testset_dir

# ★ SimulationApp 보다 **먼저** 임포트해야 한다. Isaac 확장들이 자기 `utils` 모듈을
# sys.modules 에 선점해버려, 앱을 띄운 뒤에 임포트하면 리포의 utils 패키지가
# 가려져 ModuleNotFoundError: No module named 'utils.collision' 이 난다 (Isaac Sim 5.1 실측).
from utils.collision.mesh_sampling import sample_surface

SCENE = asset("frame_xarm7_spider_turntable_v2/v3_scene.usd")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--scene", default=SCENE)
    ap.add_argument("--out", default="utils/collision/data/cell_env.npz")
    # ★ 여러 개 받는다 — 셀에 따라 구조물과 턴테이블이 **다른 루트**에 있다.
    #   (v2 실물 씬: 구조물 /World/Frame/frame_structure + 턴테이블 /World/frame)
    #   하나만 받던 시절엔 턴테이블이 통째로 빠져 충돌 게이트가 못 봤다.
    ap.add_argument("--root", nargs="+", default=["/World/frame"])
    ap.add_argument("--robot", default="/World/xarm7")
    ap.add_argument("--spacing", type=float, default=0.005,
                    help="표면 샘플 간격 m. SDF 복셀(8mm)보다 촘촘해야 한다")
    ap.add_argument("--max-pts", type=int, default=4000000,
                    help="최종 안전 상한. 복셀 다운샘플 뒤에도 넘으면 그때만 솎는다")
    ap.add_argument("--voxel", type=float, default=0.002,
                    help="다운샘플 복셀 m. 충돌 SDF 가 8mm 이므로 그보다 촘촘하면 충분하다")
    ap.add_argument("--exclude-turntable", action="store_true",
                    help="턴테이블 뭉치 제외(별도 캡슐로 볼 때)")
    args = ap.parse_args()

    sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))
    from isaacsim import SimulationApp
    app = SimulationApp({"headless": True})

    import numpy as np
    # `omni.isaac.core.utils.stage` 는 5.x 에서 없어진 구 네임스페이스다.
    # omni.usd 컨텍스트를 직접 쓴다 (버전 무관).
    from omni.usd import get_context
    from pxr import Usd, UsdGeom

    rng = np.random.default_rng(0)          # 결정적 샘플링
    get_context().open_stage(args.scene)
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
    # ⚠ `--exclude-turntable` 용 상수 — **v3 씬의 축(0.365)에 고정돼 있다.**
    #   다른 셀에서는 맞지 않으니 그 셀에서 이 플래그를 쓰려면 여기부터 고칠 것.
    #   (기본은 턴테이블 포함이라 평소에는 영향 없다.)
    TT_AXIS, TT_R, TT_ZR = 0.365, 0.16, (0.50, 0.70)

    P, names = [], []
    roots = [args.root] if isinstance(args.root, str) else list(args.root)
    prims = []
    for r in roots:
        pr = stage.GetPrimAtPath(r)
        if not pr or not pr.IsValid():
            print(f"  ⚠ 루트 없음(건너뜀): {r}")
            continue
        prims += list(Usd.PrimRange(pr, pred))
    for d in prims:
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
    n_raw = len(E)
    # ★ **복셀 다운샘플**로 줄인다 — 균일 stride 로 솎으면 안 된다.
    #   stride 는 큰 벽이든 작은 부품이든 같은 비율로 버려서, **작은 부재가 굶는다.**
    #   실측 2026-09-17: 원본 30,153,025점을 stride 15 로 2M 까지 줄이자 턴테이블
    #   원판 상면 점이 2,532 → **163개**가 됐다. 8mm SDF 복셀 기준으로 원판 면에
    #   구멍이 뚫리는 수준이라, 충돌 게이트가 원판을 제대로 못 본다.
    #   복셀 다운샘플은 **표면 어디든 복셀당 1점**을 보장하면서 같은 크기로 줄인다
    #   (같은 데이터: 2mm 복셀 → 2,350,544점, 원본의 7.8%).
    if args.voxel > 0:
        key = np.floor(E / args.voxel).astype(np.int64)
        _, idx = np.unique(key, axis=0, return_index=True)
        E = E[np.sort(idx)]
    n_vox = len(E)
    if len(E) > args.max_pts:                     # 안전망 (보통 안 걸린다)
        E = E[:: max(1, len(E) // args.max_pts)]

    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    np.savez_compressed(args.out, env=E, names=np.array(sorted(set(names))))
    print(f"저장: {args.out}   {E.shape}  (메시 {len(names)}개)")
    print(f"  원본 샘플 {n_raw:,} → 복셀 {args.voxel*1000:.0f}mm {n_vox:,} → 최종 {len(E):,}")
    print(f"  로봇 base(world) {np.round(T_WB[:3,3],3).tolist()}")
    print(f"  base 기준 범위 X{np.round([E[:,0].min(),E[:,0].max()],3).tolist()} "
          f"Y{np.round([E[:,1].min(),E[:,1].max()],3).tolist()} "
          f"Z{np.round([E[:,2].min(),E[:,2].max()],3).tolist()}")
    app.close()


if __name__ == "__main__":
    main()
