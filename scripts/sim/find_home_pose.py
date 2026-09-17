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
4. **충돌 게이트**(utils/collision/CollisionModel)를 통과하는 것만 남기고,
   여유(slack)가 큰 순으로 출력.

   ★ 2026-09-17 — 예전엔 이 단계가 "링크가 상판(Z=0.51) 위인가" 뿐이었다.
     그래서 손목이 턴테이블 디스크 **27mm 위를 스치는** 자세가 home 으로
     채택됐고(env 여유 21mm < 기준 25mm), `is_path_safe` 의 start 검사에
     걸려 **home 에서 출발하는 모든 이동이 거부**됐다. 계획용 preview 16회가
     전부 `이동 거부 — start(link6)` 로 죽고 lookaround 이 한 점도 못 얻었다.
     자세를 고르는 곳과 자세를 검사하는 곳이 다른 기준을 쓰면 이렇게 된다 —
     **같은 게이트를 쓴다.**

좌표 규약: USD Gf.Matrix4d 는 **행벡터**(p'=p·M), numpy 운동학은 **열벡터**
(x_B = T @ x_E). 그래서 USD → numpy 변환은 항상 **전치**한다.
"""
from __future__ import annotations

import argparse
import os
import sys

import numpy as np
# ★ pxr 는 Isaac 런타임 부트스트랩 뒤에만 임포트된다(standalone python 에는 없다).
#   그래서 **지연 임포트**한다 — `--from-cache` 는 pxr 없이 conda `mms-env` 에서
#   그대로 돌아간다. 충돌 게이트(scipy)는 Isaac 파이썬에 없으므로, 둘을 한 번에
#   쓰려면 캐시 모드가 유일한 길이다.

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))
from utils.robot import xarm7_kinematics as kin          # noqa: E402
from utils.collision import collision_model as _colmodel  # noqa: E402

import os, sys; sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))
from mms_paths import asset, testset_dir

DEFAULT_SCENE = asset("frame_xarm7_spider_turntable_v2/v3_scene.usd")
ROBOT, LINK7 = "/World/xarm7", "/World/xarm7/link7"
CAMERA = "/World/xarm7/link7/tool/spider/Camera"
OBJECT = "/World/ScanTarget/TestObject"
TABLE_Z = 0.510          # universal_plate 상면 — 링크가 이 아래로 가면 탈락


CACHE_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                         "log", "testset_points")


def usd_pose(xc, stage, path):
    """USD 행벡터 행렬 → numpy 열벡터 pose (해당 prim 의 월드 자세)."""
    return np.array(xc.GetLocalToWorldTransform(stage.GetPrimAtPath(path))).T


def scene_from_cache(obj: str = None):
    """`extract_testset_points.py` 가 구운 캐시에서 씬 상수를 읽는다 (pxr 불요).

    `scene.json` 의 `T_EC` 가 곧 link7→Camera 다(E→C). 대상 bbox 는 물체 npz
    점군에서 잰다. 둘 다 같은 v3 씬에서 뽑은 것이라 USD 를 다시 열 이유가 없다.
    """
    import json
    with open(os.path.join(CACHE_DIR, "scene.json"), encoding="utf-8") as f:
        sc = json.load(f)
    W_base = np.array(sc["T_WB"], float)
    M_l7_cam = np.array(sc["T_EC"], float)
    npzs = sorted(f for f in os.listdir(CACHE_DIR) if f.endswith(".npz"))
    if obj:
        npzs = [f for f in npzs if obj in f] or npzs
    P = np.load(os.path.join(CACHE_DIR, npzs[0]))["pts"]
    print(f"[cache] 씬 상수 {CACHE_DIR}/scene.json · 대상 {npzs[0]}")
    return W_base, M_l7_cam, P.min(0), P.max(0)


def bbox(stage, xc, path):
    from pxr import Usd, UsdGeom, Gf                  # 지연 임포트 (위 주석 참고)
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
    ap.add_argument("--self-clear", type=float, default=0.020,
                    help="자가충돌 최소 여유 m (isaac_scan_session SELF_CLEAR_M 과 맞출 것)")
    ap.add_argument("--env-clear", type=float, default=0.025,
                    help="셀 구조물 최소 여유 m (ENV_CLEAR_M 과 맞출 것)")
    ap.add_argument("--no-collision", action="store_true",
                    help="충돌 게이트를 끈다 (옛 동작 — 상판 높이만 검사)")
    ap.add_argument("--from-cache", nargs="?", const="", default=None,
                    metavar="물체",
                    help="USD 대신 extract_testset_points 캐시를 쓴다 (pxr 불요 — "
                         "conda mms-env 에서 실행 가능). 물체 이름 일부를 줄 수 있다")
    args = ap.parse_args()

    # ★ 런타임과 **같은 게이트**. 레이아웃도 백엔드(sim)에 맞춘다 —
    #   활성본은 실측 셀이라 sim base(0.365,0,1.5)와 41/93mm 어긋난다.
    cm = None
    if not args.no_collision:
        os.environ.setdefault("MMS_COLLISION_LAYOUT", "v3_layout_sim")
        cm = _colmodel.get_default(self_margin_m=args.self_clear,
                                   env_margin_m=args.env_clear)
        if cm is None:
            print("⚠ 충돌 캐시 없음 — 게이트 없이 진행(옛 동작). "
                  "scripts/sim/export_env_mesh.py 로 생성할 것")

    if args.from_cache is not None:
        W_base, M_l7_cam, olo, ohi = scene_from_cache(args.from_cache or None)
    else:
        from pxr import Usd, UsdGeom                 # noqa: F811  (지연 임포트)
        stage = Usd.Stage.Open(args.scene)
        xc = UsdGeom.XformCache()
        W_base = usd_pose(xc, stage, ROBOT)
        W_l7 = usd_pose(xc, stage, LINK7)
        W_cam = usd_pose(xc, stage, CAMERA)
        M_l7_cam = np.linalg.inv(W_l7) @ W_cam      # link7 기준 카메라 자세 (고정)
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
            # ★ 런타임 게이트와 동일한 판정. 통과 못 하면 home 이 될 수 없다 —
            #   home 은 모든 경로의 **출발점**이라 여기서 걸리면 아무 데도 못 간다.
            slack = None
            if cm is not None:
                ok_c, why = cm.is_pose_safe(q)
                if not ok_c:
                    continue
                slack = float(cm.slack(q)[0])
            best.append((float(z_w.min()), el, az, q, cam, slack))

    if not best:
        print("\n✘ 해 없음 — --dist/--el 범위를 넓혀 보세요")
        return
    # 충돌 여유가 큰 순 (게이트를 껐으면 예전처럼 링크 높이 순)
    best.sort(key=lambda r: -(r[5] if r[5] is not None else r[0]))
    print(f"\n유효해 {len(best)}개 — 상위 8")
    print(f"{'el':>4}{'az':>6}  {'여유':>8}  {'최저링크z':>9}  카메라 world           관절각(deg)")
    for zmin, el, az, q, cam, slack in best[:8]:
        sl = "  (게이트끔)" if slack is None else f"{slack*1000:>7.1f}mm"
        print(f"{el:>4.0f}{az:>6.0f}  {sl}  {zmin:>9.3f}  {np.round(cam,3)}  "
              f"{np.round(np.degrees(q),2).tolist()}")

    zmin, el, az, q, cam, slack = best[0]
    print("\n=== 추천 home ===")
    print(f"  el={el:.0f}° az={az:.0f}°  카메라 {np.round(cam,4)}  "
          f"대상까지 {np.linalg.norm(cam-target)*1000:.0f}mm  최저링크 z={zmin:.3f}"
          + ("" if slack is None else f"  충돌여유={slack*1000:.1f}mm"))
    print(f"  sigma_min = {kin.sigma_min(q):.4f}")
    print(f'  "artec": {np.round(np.degrees(q), 2).tolist()},')


if __name__ == "__main__":
    main()
