"""
calib_fixture.py — sim 턴테이블 축 calibration 용 **가상 fixture** 셋업 (Isaac Sim).

실물에선 구 fixture(기둥 위 구)를 턴테이블에 볼트로 고정한다. sim 에선 이 모듈이
같은 역할을 한다 — off-axis 구 prim 을 만들어 `IsaacTurntable.add_rider` 로 디스크에
'부착'(회전 시 함께 돔)하고, 스캐너를 조준하고, base 프레임 z밴드/GT축을 돌려준다.

sim 검증 스크립트와 main_artec(isaac 분기)이 공유한다. 실물 경로엔 불필요.
"""

from __future__ import annotations

import numpy as np

DISC_PRIM = "/World/ScanTarget/turntable_demo/turntable/turntable"


def _world_to_base(world):
    """world→base 4x4 (base = xarm7 root)."""
    bpos, bR = world.base_world_pose()
    T_bw = np.eye(4); T_bw[:3, :3] = bR; T_bw[:3, 3] = bpos
    return np.linalg.inv(T_bw)


def prepare_sim_fixture(world, turntable, sphere_offsets, sphere_radius,
                        standoff=0.25, view_dir=(0.0, -0.6, 0.8)):
    """
    off-axis 구 fixture 생성 + 턴테이블 부착(rider) + 스캐너 조준.

    Parameters
    ----------
    sphere_offsets : [(dx, dy, z_world), ...]  디스크중심 XY 기준 오프셋 + world z
    sphere_radius  : 구 반경 (m)
    standoff       : 카메라 표준오프 (m)
    view_dir       : 카메라가 fixture 를 보는 방향 (위/옆)

    Returns
    -------
    dict: z_bands_base, gt_point_base, gt_dir_base, off_axis_deg, sphere_world
    """
    from pxr import UsdGeom, Gf
    stage = world.stage

    dctr, _ = world.prim_world_pose(DISC_PRIM)
    cx, cy = float(dctr[0]), float(dctr[1])
    Twb = _world_to_base(world)

    # 구는 /World(identity 부모) 바로 아래 생성 → 로컬 translate = world 좌표.
    # (주의: /World/ScanTarget 아래 두면 ScanTarget 의 world 오프셋만큼 구가 밀려
    #  의도한 world 위치를 벗어난다 — AddTranslateOp 은 '로컬' 변환이기 때문.)
    SPHERE_ROOT = "/World"
    pp, pR = world.prim_world_pose(SPHERE_ROOT)
    T_pw = np.eye(4); T_pw[:3, :3] = pR; T_pw[:3, 3] = pp     # 부모(world) → world
    T_wp = np.linalg.inv(T_pw)                                # world → 부모(local)

    sphere_world, z_bands = [], []
    for i, (dx, dy, z) in enumerate(sphere_offsets):
        wp = np.array([cx + dx, cy + dy, z], dtype=float)
        sphere_world.append(wp)
        path = f"{SPHERE_ROOT}/calib_sphere_{i}"
        s = UsdGeom.Sphere.Define(stage, path)
        s.GetRadiusAttr().Set(float(sphere_radius))
        s.GetExtentAttr().Set([Gf.Vec3f(-float(sphere_radius),) * 3,
                               Gf.Vec3f(float(sphere_radius),) * 3])
        local = (T_wp @ np.array([*wp, 1.0]))[:3]             # 부모 변환 보정 → 정확한 world
        UsdGeom.Xformable(s).AddTranslateOp().Set(Gf.Vec3d(*map(float, local)))
        turntable.add_rider(path)                             # 디스크에 부착(co-rotate)
        z_bands.append(float((Twb @ np.array([*wp, 1.0]))[2]))

    fixc = np.mean(sphere_world, axis=0)
    vd = np.asarray(view_dir, float); vd = vd / np.linalg.norm(vd)
    off = world.look_at_camera(target_world=fixc, cam_pos_world=fixc + vd * standoff)

    # GT 턴테이블 축 (base 프레임): 디스크중심 XY, world +Z
    gt_point_base = (Twb @ np.array([cx, cy, 0.0, 1.0]))[:3]
    gt_dir_base = Twb[:3, :3] @ np.array([0.0, 0.0, 1.0])

    return {
        "z_bands_base": z_bands,
        "gt_point_base": gt_point_base,
        "gt_dir_base": gt_dir_base,
        "off_axis_deg": off,
        "sphere_world": sphere_world,
    }
