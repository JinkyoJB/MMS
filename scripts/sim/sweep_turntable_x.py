"""sweep_turntable_x.py — 턴테이블 X 위치를 바꿔가며 로봇이 커버 가능한 관측각 평가.

배경
----
v3(260811) 레이아웃은 턴테이블 축이 로봇 base 바로 아래(둘 다 X=0.365)라 **낮은
고도각(측면 관측)이 전부 도달 불가**다. 팔을 아래로 뻗어 옆으로 꺾어야 하는데 관절범위를
벗어난다. 턴테이블을 X 로 옮기면 측면 접근이 열린다(v2 는 base 가 축에서 0.17m 비켜 있었다).

평가 방법
--------
각 후보 X 에 대해 (elevation × azimuth) 격자로 "턴테이블 축을 standoff 거리에서 바라보는
카메라" 자세를 만들고, **해석 IK + 자가충돌/장애물 충돌**을 통과하는 조합 수를 센다.
낮은 el(측면)일수록 스캔 커버리지 기여가 크므로 가중치를 준다.

사용
    env -u PYTHONPATH ~/miniconda3/envs/env_isaacsim/bin/python \
        scripts/sim/sweep_turntable_x.py
    #  --x-min -0.25 --x-max 0.55 --x-step 0.05  --standoff 0.313  --disc-top 0.665
"""
from __future__ import annotations

import argparse
import math
import os
import sys

import os, sys; sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))
from mms_paths import asset, testset_dir

SCENE = asset("frame_xarm7_spider_turntable_v2/v3_scene.usd")
ROBOT = "/World/xarm7"
LINK7 = "/World/xarm7/link7"
CAMERA = "/World/xarm7/link7/tool/spider/Camera"

# 측면일수록 커버리지 기여가 크다 → 낮은 el 에 가중
EL_WEIGHT = {20: 3.0, 30: 3.0, 40: 2.0, 50: 1.5, 60: 1.0, 70: 0.7, 80: 0.4}
AZIS = (0.0, 30.0, -30.0, 60.0, -60.0, 90.0, -90.0, 180.0)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--x-min", type=float, default=-0.25)
    ap.add_argument("--x-max", type=float, default=0.55)
    ap.add_argument("--x-step", type=float, default=0.05)
    ap.add_argument("--standoff", type=float, default=0.313,
                    help="축→카메라 거리 m (WORK_FOCUS 0.25 + 객체반경 0.063)")
    ap.add_argument("--disc-top", type=float, default=0.665, help="원판 상면 world Z")
    ap.add_argument("--obj-h", type=float, default=0.039,
                    help="겨냥점을 원판 상면에서 얼마나 올릴지(객체 중심 높이) m")
    ap.add_argument("--disc-radius", type=float, default=0.119)
    ap.add_argument("--body-h", type=float, default=0.155)
    args = ap.parse_args()

    from isaacsim import SimulationApp
    app = SimulationApp({"headless": True})

    import numpy as np
    from omni.isaac.core.utils.stage import open_stage
    from omni.usd import get_context
    from pxr import UsdGeom

    sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))
    from utils.robot import xarm7_kinematics as kin
    from utils.collision.robot_collision import (
        CollisionWorld, pose_collision, DEFAULT_LINK_RADII)
    from mms_artec.utils.calibration.handeye_geometry import (
        look_at_camera as _look, make_T as _mk)

    open_stage(SCENE)
    for _ in range(60):
        app.update()
    stage = get_context().get_stage()
    xc = UsdGeom.XformCache()

    def T(p):
        return np.array(xc.GetLocalToWorldTransform(stage.GetPrimAtPath(p))).T

    T_WB = T(ROBOT)
    T_EC = np.linalg.inv(T(CAMERA)) @ T(LINK7)
    print(f"로봇 base(world) {np.round(T_WB[:3,3],3)}   "
          f"standoff {args.standoff:.3f}m  겨냥높이 Z={args.disc_top+args.obj_h:.3f}")

    def w2b(p):
        return (np.asarray(p) - T_WB[:3, 3]) @ T_WB[:3, :3]

    axis_dir_b = T_WB[:3, :3].T @ np.array([0.0, 0.0, 1.0])
    Q_SEED = np.radians(np.array([-7.65, -75.61, -8.95, 78.64, 2.81, 126.71, -69.63]))

    def eval_x(ax_x):
        axis_w = np.array([ax_x, 0.0, args.disc_top])
        tgt = np.array([ax_x, 0.0, args.disc_top + args.obj_h])
        world = CollisionWorld.from_turntable(
            surface_point=w2b(axis_w), axis_dir=axis_dir_b,
            disc_radius=args.disc_radius, body_height=args.body_h, margin=0.01)
        rows, score, n_ok = {}, 0.0, 0
        for el in sorted(EL_WEIGHT):
            oks = []
            for az in AZIS:
                e, a = math.radians(el), math.radians(az)
                eye = tgt + args.standoff * np.array(
                    [math.cos(e) * math.cos(a), math.cos(e) * math.sin(a), math.sin(e)])
                T_WC = _mk(_look(eye, tgt, (0.0, 0.0, 1.0)), eye)
                T_EB = (np.linalg.inv(T_WB) @ T_WC) @ T_EC
                p6 = np.concatenate([T_EB[:3, 3] * 1000.0,
                                     kin.R_to_euler_xyz(T_EB[:3, :3])])
                q, ok = kin.ik(p6, seed=Q_SEED)
                if not ok:
                    continue
                col, _ = pose_collision(world, q, T_EC=T_EC,
                                        link_radii=DEFAULT_LINK_RADII)
                if col:
                    continue
                oks.append(int(az))
            rows[el] = oks
            n_ok += len(oks)
            score += EL_WEIGHT[el] * len(oks)
        return rows, score, n_ok

    xs = np.arange(args.x_min, args.x_max + 1e-9, args.x_step)
    print(f"\n{'X(m)':>7}{'점수':>7}{'통과':>6}   " +
          "".join(f"el{e:<4}" for e in sorted(EL_WEIGHT)))
    results = []
    for x in xs:
        rows, score, n_ok = eval_x(float(x))
        results.append((float(x), score, n_ok, rows))
        cells = "".join(f"{len(rows[e]):<6}" for e in sorted(EL_WEIGHT))
        mark = "  ← 현재" if abs(x - 0.365) < 1e-6 else ""
        print(f"{x:>7.2f}{score:>7.1f}{n_ok:>6}   {cells}{mark}")

    results.sort(key=lambda r: -r[1])
    print("\n=== 상위 5 ===")
    for x, score, n_ok, rows in results[:5]:
        print(f"  X={x:+.2f}  점수 {score:5.1f}  통과 {n_ok:2d}/{len(EL_WEIGHT)*len(AZIS)}")
        for e in sorted(EL_WEIGHT):
            if rows[e]:
                print(f"      el={e:>2}  az={rows[e]}")
    app.close()


if __name__ == "__main__":
    main()
