"""spin_turntable.py — v3_scene 의 턴테이블을 돌려본다 (IsaacTurntable 과 동일한 방식).

회전 방식은 isaac_turntable.py 의 `_co_rotate` 와 같다:
    디스크 중심 (cx, cy) 를 지나는 **world-Z** 축으로 rider prim 들을 kinematic 회전.
    (RevoluteJoint 를 쓰지 않는 이유는 isaac_turntable.py 헤더 참고)

사용:
    # GUI 로 눈으로 보며 한 바퀴
    ~/miniconda3/envs/env_isaacsim/bin/python scripts/sim/spin_turntable.py --gui

    # 헤드리스로 90° 만 돌려 좌표 검증
    ~/miniconda3/envs/env_isaacsim/bin/python scripts/sim/spin_turntable.py \
        --angle 90 --no-loop
"""
from __future__ import annotations

import argparse
import sys

DEFAULT_SCENE = ("/home/keti/workspace/sync/2_Rapid_Digital_Twin/1_MMS/2_3Dassets/"
                 "frame_xarm7_spider_turntable_v2/v3_scene.usd")
DISC_PRIM = "/World/frame/turntable_disc"


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--scene", default=DEFAULT_SCENE)
    ap.add_argument("--disc", default=DISC_PRIM)
    ap.add_argument("--rider", action="append", default=[],
                    help="함께 돌릴 prim 경로 (스캔 대상 등). 여러 번 지정 가능")
    ap.add_argument("--angle", type=float, default=360.0, help="총 회전각 deg")
    ap.add_argument("--step", type=float, default=3.0, help="스텝당 각도 deg")
    ap.add_argument("--gui", action="store_true", help="GUI 로 보며 회전")
    ap.add_argument("--no-loop", action="store_true", help="한 번만 돌고 종료")
    args = ap.parse_args()

    from isaacsim import SimulationApp
    app = SimulationApp({"headless": not args.gui})

    import numpy as np
    from omni.isaac.core.utils.stage import open_stage
    from omni.usd import get_context
    from pxr import Usd, UsdGeom, Gf

    open_stage(args.scene)
    for _ in range(60):
        app.update()
    stage = get_context().get_stage()

    disc = stage.GetPrimAtPath(args.disc)
    if not (disc and disc.IsValid()):
        print(f"✘ disc prim 없음: {args.disc}")
        app.close(); sys.exit(1)

    xc = UsdGeom.XformCache()
    Wd = xc.GetLocalToWorldTransform(disc)
    cx, cy = Wd[3][0], Wd[3][1]          # 회전축 = 디스크 원점 통과 world-Z
    print(f"회전축 (world-Z) 통과점: ({cx:.4f}, {cy:.4f})")

    # rider = 디스크 + 사용자가 지정한 prim 들
    riders = []
    for path in [args.disc] + args.rider:
        p = stage.GetPrimAtPath(path)
        if not (p and p.IsValid()):
            print(f"  ⚠ rider 없음: {path}")
            continue
        xf = UsdGeom.Xformable(p)
        ops = xf.GetOrderedXformOps()
        if not ops:
            print(f"  ⚠ xformOp 없음: {path}")
            continue
        p2w = xc.GetLocalToWorldTransform(p.GetParent())
        riders.append((ops[0], ops[0].Get(), p2w, p2w.GetInverse()))
        print(f"  rider: {path}")

    def rotate_to(deg: float):
        """누적각 deg 로 모든 rider 를 설정 (base 에서 절대 회전)."""
        Rz = Gf.Matrix4d().SetRotate(Gf.Rotation(Gf.Vec3d(0, 0, 1), deg))
        Tp = Gf.Matrix4d().SetTranslate(Gf.Vec3d(cx, cy, 0.0))
        Tn = Gf.Matrix4d().SetTranslate(Gf.Vec3d(-cx, -cy, 0.0))
        S = Tn * Rz * Tp
        for op, base, p2w, p2w_inv in riders:
            op.Set(base * (p2w * S * p2w_inv))

    # 검증용: 디스크 림의 한 점이 어떻게 움직이는지
    def rim_point():
        m = stage.GetPrimAtPath(f"{args.disc}/mesh")
        pts = UsdGeom.Mesh(m).GetPointsAttr().Get()
        M = UsdGeom.XformCache().GetLocalToWorldTransform(m)
        w = M.Transform(Gf.Vec3d(pts[0][0], pts[0][1], pts[0][2]))
        return np.array([w[0], w[1], w[2]])

    p0 = rim_point()
    print(f"림 기준점 시작 {np.round(p0, 4)}  (축까지 거리 "
          f"{np.hypot(p0[0]-cx, p0[1]-cy)*1000:.1f}mm)")

    ang = 0.0
    while True:
        ang += args.step
        rotate_to(ang)
        app.update()
        if ang >= args.angle:
            p1 = rim_point()
            r0 = np.hypot(p0[0] - cx, p0[1] - cy)
            r1 = np.hypot(p1[0] - cx, p1[1] - cy)
            print(f"\n{ang:.0f}° 후 림점 {np.round(p1, 4)}")
            print(f"  축까지 거리 {r0*1000:.2f} → {r1*1000:.2f} mm  (같아야 정상)")
            print(f"  높이 Z {p0[2]:.4f} → {p1[2]:.4f} m  (같아야 정상)")
            if args.no_loop:
                break
            ang = 0.0
            p0 = p1

    app.close()


if __name__ == "__main__":
    main()
