"""find_home_pose.py — v3_scene 에서 Artec home 관절각을 IK 로 탐색.

기존 HOME_JOINTS_DEG["artec"] 는 v2 레이아웃(로봇 base (0.538,0,1.407),
턴테이블 (0.28,0,0.71)) 기준이라 v3(base (0.365,0,1.5), 턴테이블 (0.365,0,0.665))
에서는 스캐너가 대상을 못 본다.

방법
----
1. USD 에서 **실측**한다 — 로봇 base 자세, link7→Camera 상대자세, 대상 bbox.
   (하드코딩 금지: 툴체인저가 들어가며 link7→Camera 가 v2 와 완전히 달라졌다)
2. 대상 상단을 작동거리 d 에서 내려다보는 카메라 자세를 방위각·고도각 격자로 생성.
3. 카메라 자세 → link7 자세 → base 프레임 → kin.ik.
4. 링크가 상판(Z=0.51) 위에 있는지 등으로 거른 뒤 최적해 출력.

좌표 규약: USD Gf.Matrix4d 는 **행벡터**(p'=p·M), numpy 운동학은 **열벡터**
(x_B = T @ x_E). 그래서 USD → numpy 변환은 항상 **전치**한다.
"""
from __future__ import annotations

import argparse
import os
import sys

import numpy as np
from pxr import Usd, UsdGeom, Gf

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))
from utils.robot import xarm7_kinematics as kin          # noqa: E402

DEFAULT_SCENE = ("/home/keti/workspace/sync/2_Rapid_Digital_Twin/1_MMS/2_3Dassets/"
                 "frame_xarm7_spider_turntable_v2/v3_scene.usd")
ROBOT, LINK7 = "/World/xarm7", "/World/xarm7/link7"
CAMERA = "/World/xarm7/link7/tool/spider/Camera"
OBJECT = "/World/ScanTarget/TestObject"
TABLE_Z = 0.510          # universal_plate 상면 — 링크가 이 아래로 가면 탈락


def usd_pose(xc, stage, path):
    """USD 행벡터 행렬 → numpy 열벡터 pose (해당 prim 의 월드 자세)."""
    return np.array(xc.GetLocalToWorldTransform(stage.GetPrimAtPath(path))).T


def bbox(stage, xc, path):
    pred = Usd.TraverseInstanceProxies(Usd.PrimAllPrimsPredicate)
    P = []
    for d in Usd.PrimRange(stage.GetPrimAtPath(path), pred):
        if not d.IsA(UsdGeom.Mesh):
            continue
        M = xc.GetLocalToWorldTransform(d)
        q = UsdGeom.Mesh(d).GetPointsAttr().Get() or []
        for i in range(0, len(q), 7):
            w = M.Transform(Gf.Vec3d(q[i][0], q[i][1], q[i][2]))
            P.append([w[0], w[1], w[2]])
    P = np.array(P)
    return P.min(0), P.max(0)


def look_at_camera(eye, target, up=(0.0, 0.0, 1.0)):
    """USD 카메라 규약(-Z 를 바라봄, +Y up) 의 월드 자세 4x4 (열벡터)."""
    f = np.asarray(target, float) - np.asarray(eye, float)
    f /= np.linalg.norm(f)
    z = -f
    u = np.asarray(up, float)
    if abs(np.dot(u, z)) > 0.99:
        u = np.array([0.0, 1.0, 0.0])
    x = np.cross(u, z); x /= np.linalg.norm(x)
    y = np.cross(z, x)
    T = np.eye(4)
    T[:3, 0], T[:3, 1], T[:3, 2], T[:3, 3] = x, y, z, eye
    return T


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--scene", default=DEFAULT_SCENE)
    ap.add_argument("--dist", type=float, default=0.25, help="작동거리 m (Spider 0.2~0.3)")
    ap.add_argument("--el", type=float, nargs="+",
                    default=[35, 45, 55, 65], help="고도각 후보 deg (수평=0, 수직내려봄=90)")
    ap.add_argument("--az-step", type=float, default=15.0, help="방위각 격자 deg")
    ap.add_argument("--top", action="store_true",
                    help="대상 '상단' 대신 bbox 중심을 겨냥")
    args = ap.parse_args()

    stage = Usd.Stage.Open(args.scene)
    xc = UsdGeom.XformCache()
    W_base = usd_pose(xc, stage, ROBOT)
    W_l7 = usd_pose(xc, stage, LINK7)
    W_cam = usd_pose(xc, stage, CAMERA)
    M_l7_cam = np.linalg.inv(W_l7) @ W_cam          # link7 기준 카메라 자세 (고정)

    olo, ohi = bbox(stage, xc, OBJECT)
    ctr = (olo + ohi) / 2.0
    target = np.array([ctr[0], ctr[1], ohi[2] if args.top else ctr[2]])
    print(f"로봇 base(world) {np.round(W_base[:3, 3], 4)}")
    print(f"대상 bbox z {olo[2]:.3f}~{ohi[2]:.3f}, 겨냥점 {np.round(target, 4)}")
    print(f"link7→Camera 이동 {np.round(M_l7_cam[:3, 3]*1000, 1)} mm\n")

    # 현재 home 진단
    q0 = np.radians(np.array([38.92, -48.70, -65.29, 21.22, 21.46, 72.70, -96.58]))
    T_b_l7 = kin.fk_T(q0).copy(); T_b_l7[:3, 3] /= 1000.0
    cam_now = (W_base @ T_b_l7 @ M_l7_cam)[:3, 3]
    print(f"[현재 home] 카메라 world {np.round(cam_now, 4)}  "
          f"대상까지 {np.linalg.norm(cam_now-target)*1000:.0f} mm  ← 작동거리 200~300mm 밖")

    best = []
    for el in args.el:
        for az in np.arange(0.0, 360.0, args.az_step):
            a, e = np.radians(az), np.radians(el)
            eye = target + args.dist * np.array(
                [np.cos(e) * np.cos(a), np.cos(e) * np.sin(a), np.sin(e)])
            W_cam_des = look_at_camera(eye, target)
            W_l7_des = W_cam_des @ np.linalg.inv(M_l7_cam)
            T_des = np.linalg.inv(W_base) @ W_l7_des        # base 프레임
            pose = np.concatenate([T_des[:3, 3] * 1000.0,
                                   kin.R_to_euler_xyz(T_des[:3, :3])])
            q, ok = kin.ik(pose, seed=q0)
            if not ok:
                continue
            # 검증: FK 로 카메라를 되돌려 목표와 비교
            T = kin.fk_T(q).copy(); T[:3, 3] /= 1000.0
            cam = (W_base @ T @ M_l7_cam)[:3, 3]
            err = np.linalg.norm(cam - eye)
            if err > 2e-3:
                continue
            org = kin.fk_link_origins(q)                     # base 프레임, m
            z_w = (W_base @ np.c_[org, np.ones(len(org))].T)[2]   # world z
            if z_w.min() < TABLE_Z + 0.02:                   # 상판에 박히는 자세 제외
                continue
            best.append((float(z_w.min()), el, az, q, cam))

    if not best:
        print("\n✘ 해 없음 — --dist/--el 범위를 넓혀 보세요")
        return
    best.sort(key=lambda r: -r[0])          # 링크가 가장 높은(안전한) 해 우선
    print(f"\n유효해 {len(best)}개 — 상위 8")
    print(f"{'el':>4}{'az':>6}  {'최저링크z':>9}  카메라 world           관절각(deg)")
    for zmin, el, az, q, cam in best[:8]:
        print(f"{el:>4.0f}{az:>6.0f}  {zmin:>9.3f}  {np.round(cam,3)}  "
              f"{np.round(np.degrees(q),2).tolist()}")

    zmin, el, az, q, cam = best[0]
    print("\n=== 추천 home ===")
    print(f"  el={el:.0f}° az={az:.0f}°  카메라 {np.round(cam,4)}  "
          f"대상까지 {np.linalg.norm(cam-target)*1000:.0f}mm  최저링크 z={zmin:.3f}")
    print(f'  "artec": {np.round(np.degrees(q), 2).tolist()},')


if __name__ == "__main__":
    main()
