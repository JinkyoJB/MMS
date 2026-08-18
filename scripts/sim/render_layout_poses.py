"""render_layout_poses.py — 레이아웃 검토용 자세를 실제로 세워 스크린샷을 남긴다.

`eval_turntable_layout.py` 가 저장한 관절각(JSON)을 씬에 적용하고 오프스크린 렌더로
PNG 를 뽑는다. 표에 적힌 "el=30 도달 가능" 같은 주장을 **눈으로 확인**하기 위한 것.

    env -u PYTHONPATH ~/miniconda3/envs/env_isaacsim/bin/python \
        scripts/sim/render_layout_poses.py --scene <scene.usd> --key z1.500_x-0.10
    #  --poses scripts/sim/log/layout_poses.json  --out docs/figures/layout
"""
from __future__ import annotations

import argparse
import json
import math
import os


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--scene", required=True, help="렌더할 씬 USD")
    ap.add_argument("--key", required=True, help="poses JSON 의 키 (예: z1.500_x-0.10)")
    ap.add_argument("--poses", default="scripts/sim/log/layout_poses.json")
    ap.add_argument("--out", default="docs/figures/layout")
    ap.add_argument("--res", type=int, nargs=2, default=[1280, 800])
    # 기본은 -Y 쪽 측면 프로파일. 고도각(el)이 그림에서 그대로 읽히도록 거의 수평에서 본다.
    ap.add_argument("--cam-az", type=float, default=-90.0, help="관찰 카메라 방위각")
    ap.add_argument("--cam-el", type=float, default=8.0, help="관찰 카메라 고도각")
    ap.add_argument("--cam-dist", type=float, default=1.9)
    ap.add_argument("--cam-look-z", type=float, default=0.95, help="주시점 높이 m")
    args = ap.parse_args()

    from isaacsim import SimulationApp
    app = SimulationApp({"headless": True, "width": args.res[0], "height": args.res[1]})

    import numpy as np
    from omni.isaac.core import World
    from omni.isaac.core.robots import Robot
    from omni.isaac.core.utils.stage import open_stage
    from omni.usd import get_context
    from pxr import UsdGeom, Gf
    from omni.kit.viewport.utility import get_active_viewport, capture_viewport_to_file

    with open(args.poses) as f:
        poses = json.load(f)
    if args.key not in poses:
        raise SystemExit(f"키 '{args.key}' 없음. 있는 키: {list(poses)[:6]} ...")
    rep = poses[args.key]
    if not rep:
        raise SystemExit(f"'{args.key}' 에 도달 가능한 자세가 하나도 없습니다.")

    open_stage(args.scene)
    world = World(stage_units_in_meters=1.0)
    robot = Robot(prim_path="/World/xarm7", name="xarm7")
    world.scene.add(robot)
    world.reset()

    # 관찰 카메라 — 턴테이블 축을 중심으로 비스듬히
    disc = UsdGeom.Xformable(get_context().get_stage().GetPrimAtPath(
        "/World/frame/turntable_disc"))
    box = disc.ComputeWorldBound(0.0, "default").ComputeAlignedBox()
    ctr = np.array([(box.GetMin()[i] + box.GetMax()[i]) / 2 for i in range(3)])
    ctr[2] = args.cam_look_z
    e, a = math.radians(args.cam_el), math.radians(args.cam_az)
    eye = ctr + args.cam_dist * np.array(
        [math.cos(e) * math.cos(a), math.cos(e) * math.sin(a), math.sin(e)])
    stage = get_context().get_stage()
    cam = UsdGeom.Camera.Define(stage, "/World/ReviewCam")
    cam.CreateFocalLengthAttr(22.0)
    fwd = ctr - eye
    fwd /= np.linalg.norm(fwd)
    right = np.cross(fwd, np.array([0.0, 0.0, 1.0]))
    right /= np.linalg.norm(right)
    up = np.cross(right, fwd)
    M = Gf.Matrix4d(*[float(v) for row in (
        list(right) + [0.0], list(up) + [0.0], list(-fwd) + [0.0], list(eye) + [1.0])
        for v in row])
    x = UsdGeom.Xformable(cam.GetPrim())
    x.ClearXformOpOrder()
    x.AddTransformOp().Set(M)
    vp = get_active_viewport()
    vp.set_active_camera("/World/ReviewCam")

    os.makedirs(args.out, exist_ok=True)
    dof = robot.num_dof
    for name, q in sorted(rep.items()):
        qv = np.zeros(dof)
        qv[:min(7, dof)] = np.asarray(q)[:min(7, dof)]
        robot.set_joint_positions(qv)
        robot.set_joint_velocities(np.zeros(dof))
        for _ in range(6):
            world.step(render=True)
        for _ in range(12):
            app.update()
        path = os.path.join(args.out, f"{args.key}_{name}.png")
        capture_viewport_to_file(vp, path)
        for _ in range(12):
            app.update()
        print(f"  {path}   q(deg)={np.round(np.degrees(q), 1).tolist()}")
    app.close()


if __name__ == "__main__":
    main()
