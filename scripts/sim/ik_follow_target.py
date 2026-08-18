"""ik_follow_target.py — GUI 에서 타깃을 끌면 로봇이 IK 로 따라온다 (workspace 확인용).

왜 Lula test widget 을 안 쓰나
------------------------------
Isaac 의 `isaacsim.robot_motion.lula_test_widget` 이 같은 일을 하지만 (a) xArm7 용 Lula
robot description 이 설치본에 없고, (b) 무엇보다 **파이프라인이 쓰는 IK 가 아니다**.
이 프로젝트는 자체 해석 IK(`utils/robot/xarm7_kinematics`)를 쓰므로(docs/main_flow.md
§1.0 '결정(2026-06): xArm SDK IK 미사용'), 도달성 진단은 그 IK 로 해야 의미가 있다.

사용
    ~/miniconda3/envs/env_isaacsim/bin/python scripts/sim/ik_follow_target.py
      → v3_scene_IK.usd 를 만들고(없으면) GUI 로 연다.
      → Stage 에서 /World/IKTarget 을 선택해 이동/회전 기즈모로 끌면 로봇이 따라온다.
      → 마커 색: 초록=도달, 빨강=IK 실패

옵션
    --target camera|flange   타깃이 '카메라 자세'인지 '플랜지 자세'인지 (기본 camera)
    --scene / --out          입출력 USD 경로
"""
from __future__ import annotations

import argparse
import os
import sys

ASSET_DIR = ("/home/keti/workspace/sync/2_Rapid_Digital_Twin/1_MMS/2_3Dassets/"
             "frame_xarm7_spider_turntable_v2")
ROBOT = "/World/xarm7"
LINK7 = "/World/xarm7/link7"
CAMERA = "/World/xarm7/link7/tool/spider/Camera"
OBJECT = "/World/ScanTarget/TestObject"
ADAPTER = "/World/xarm7/link7/tool/spider_Adapter_blue"
SPIDER = "/World/xarm7/link7/tool/spider"
TARGET = "/World/IKTarget"

# IsaacXArm.HOME_JOINTS_DEG["artec"] 와 동일. 타깃 초기 자세를 이 자세의 tip 자세로
# 맞추기 위해 필요하다(아래 make_ik_scene 참고).
Q_HOME_DEG = [-7.65, -75.61, -8.95, 78.64, 2.81, 126.71, -69.63]


def _mesh_center_world(stage, path):
    """prim 하위 메시 정점의 월드 bbox 중심.

    ⚠ prim 원점이 아니라 **메시 중심**을 쓴다 — 어댑터는 prim 원점이 형상 중심에서
      벗어나 있다(실측 34mm). BBoxCache 는 회전 파트에서 부풀려지므로 정점으로 구한다.
    """
    import numpy as np
    from pxr import Usd, UsdGeom, Gf
    xc = UsdGeom.XformCache()
    pred = Usd.TraverseInstanceProxies(Usd.PrimAllPrimsPredicate)
    P = []
    for d in Usd.PrimRange(stage.GetPrimAtPath(path), pred):
        if not d.IsA(UsdGeom.Mesh):
            continue
        M = xc.GetLocalToWorldTransform(d)
        for q in (UsdGeom.Mesh(d).GetPointsAttr().Get() or []):
            w = M.Transform(Gf.Vec3d(q[0], q[1], q[2]))
            P.append([w[0], w[1], w[2]])
    P = np.array(P)
    return (P.min(0) + P.max(0)) / 2.0 if len(P) else None


def make_ik_scene(scene: str, out: str, home_tip=None, hide_spider: bool = False) -> None:
    """v3_scene 을 subLayer 로 깔고 드래그용 타깃 프림만 얹은 USD 생성 (비파괴).

    home_tip : 4x4 numpy(열벡터) — 타깃의 **초기 자세**. home 자세의 tip 자세를 그대로
        쓴다. ⚠ 여기를 identity 로 두면 안 된다 — 타깃 자세가 IK 해의 분기를 결정하는데,
        identity(카메라가 수직 아래)는 이 로봇의 자연스러운 구성과 멀어 손목이 극단적으로
        꺾이고, 그 근처에서 위치를 조금만 옮겨도 해가 다른 분기로 튄다(팔이 반대로 스윙).
        실측: joint1 이 home 자세 기준 −90° 로 갈 것이 identity 기준에선 +89° 로 갔다.
    """
    from pxr import Usd, UsdGeom, Gf, Vt

    stage = Usd.Stage.CreateNew(out)
    stage.GetRootLayer().subLayerPaths.append(f"./{os.path.basename(scene)}")
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    stage.SetDefaultPrim(stage.OverridePrim("/World"))

    # 초기 위치 = 대상 상단에서 작동거리만큼 띄운 곳(스캔 자세와 비슷한 출발점)
    # 시작점 잡는 용도라 BBoxCache 의 회전 과대추정은 무시해도 된다.
    bb = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    o = stage.GetPrimAtPath(OBJECT)
    pos = Gf.Vec3d(0.47, 0.0, 0.93)
    if o and o.IsValid():
        r = bb.ComputeWorldBound(o).ComputeAlignedRange()
        if not r.IsEmpty():
            c = r.GetMidpoint()
            pos = Gf.Vec3d(c[0] + 0.10, c[1], r.GetMax()[2] + 0.22)

    quat = Gf.Quatf(1.0, Gf.Vec3f(0, 0, 0))
    if home_tip is not None:
        pos = Gf.Vec3d(*[float(v) for v in home_tip[:3, 3]])
        _qd = Gf.Matrix4d(*home_tip.T.flatten()).ExtractRotationQuat()
        quat = Gf.Quatf(float(_qd.GetReal()),
                        Gf.Vec3f(*[float(v) for v in _qd.GetImaginary()]))
    # 기즈모가 편집하는 것과 같은 TRS op 로 authoring (xformOp:transform 행렬을 쓰면
    # 기즈모가 translate op 를 따로 추가해 둘이 합성되며 이동 방향이 틀어진다)
    tgt = UsdGeom.Xform.Define(stage, TARGET)
    tgt.AddTranslateOp().Set(pos)
    tgt.AddOrientOp().Set(quat)

    # 마커: 구(도달 여부 색) + XYZ 축 막대(자세 확인)
    s = UsdGeom.Sphere.Define(stage, f"{TARGET}/hit")
    s.CreateRadiusAttr(0.012)
    s.CreateDisplayColorAttr(Vt.Vec3fArray([Gf.Vec3f(0.2, 0.9, 0.2)]))
    for name, axis, col in (("ax_x", (1, 0, 0), (0.9, 0.15, 0.15)),
                            ("ax_y", (0, 1, 0), (0.15, 0.9, 0.15)),
                            ("ax_z", (0, 0, 1), (0.2, 0.4, 1.0))):
        b = UsdGeom.Cylinder.Define(stage, f"{TARGET}/{name}")
        b.CreateRadiusAttr(0.003)
        b.CreateHeightAttr(0.08)
        b.CreateAxisAttr(("X", "Y", "Z")[axis.index(1)])
        b.CreateDisplayColorAttr(Vt.Vec3fArray([Gf.Vec3f(*col)]))
        UsdGeom.Xformable(b.GetPrim()).AddTranslateOp().Set(
            Gf.Vec3d(*[a * 0.04 for a in axis]))
    if hide_spider:
        # 어댑터 기준으로 볼 때 스캐너 메시가 시야를 가리므로 비활성
        stage.OverridePrim(SPIDER).SetActive(False)
    stage.GetRootLayer().Save()
    print(f"[ik] 생성: {out}  (타깃 초기 위치 {tuple(round(v,3) for v in pos)}"
          f"{', spider 비활성' if hide_spider else ''})")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--scene", default=os.path.join(ASSET_DIR, "v3_scene.usd"))
    ap.add_argument("--out", default=None,
                    help="기본: v3_scene_IK.usd (adapter 모드는 v3_scene_IK_adapter.usd)")
    ap.add_argument("--target", default="camera",
                    help="IK 타깃 기준점. camera=스캐너 카메라 / flange=link7 플랜지 / "
                         "adapter=spider_Adapter_blue 메시 중심 / "
                         "'/World/...' 형태의 **임의 prim 경로**(그 메시 중심 기준)")
    ap.add_argument("--hide-spider", action="store_true",
                    help="Spider 를 비활성화 (--target adapter 면 자동 적용)")
    ap.add_argument("--rebuild", action="store_true", help="IK USD 재생성")
    ap.add_argument("--pos-tol", type=float, default=0.005, help="허용 위치오차 m")
    ap.add_argument("--rot-tol", type=float, default=5.0, help="허용 자세오차 deg")
    ap.add_argument("--max-jump", type=float, default=90.0,
                    help="1스텝 관절 급변 허용치 deg(기본 90). 초과하면 거부 — 미도달 시 "
                         "DLS 가 관절한계를 타고 팔 뒤집힌 자세로 수렴하는 것을 막는다")
    ap.add_argument("--selftest", action="store_true",
                    help="GUI 없이 타깃을 격자로 옮기며 도달성만 출력(검증용)")
    args = ap.parse_args()
    # 임의 prim 경로 지원 — 'adapter' 는 그 경로의 별칭
    tip_prim = None
    if args.target == "adapter":
        tip_prim = ADAPTER
    elif args.target.startswith("/"):
        tip_prim = args.target
    if args.out is None:
        if tip_prim:
            _tag = tip_prim.rstrip("/").split("/")[-1][:40]
            args.out = os.path.join(ASSET_DIR, f"v3_scene_IK_{_tag}.usd")
        else:
            args.out = os.path.join(ASSET_DIR, "v3_scene_IK.usd")
    hide_spider = args.hide_spider or args.target == "adapter"

    # ⚠ env_isaacsim 에서 pxr 은 SimulationApp 초기화 **후**에만 import 된다.
    #   USD 생성도 앱을 띄운 뒤에 해야 한다.
    from isaacsim import SimulationApp
    app = SimulationApp({"headless": bool(args.selftest)})

    if args.rebuild or not os.path.isfile(args.out):
        if os.path.isfile(args.out):
            os.remove(args.out)
        import numpy as _np
        from pxr import Usd as _Usd, UsdGeom as _UG
        import omni.usd as _ou
        sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))
        from utils.robot import xarm7_kinematics as _kin
        _src = _Usd.Stage.Open(args.scene)

        def _p(path):
            return _np.array(_ou.get_world_transform_matrix(
                _src.GetPrimAtPath(path))).T
        _Wb = _p(ROBOT)
        if args.target == "flange":
            _M = _np.eye(4)
        elif tip_prim:
            if not _src.GetPrimAtPath(tip_prim).IsValid():
                raise SystemExit(f"[ik] prim 없음: {tip_prim}")
            _T = _p(tip_prim).copy()
            _c = _mesh_center_world(_src, tip_prim)
            if _c is not None:
                _T[:3, 3] = _c          # 자세는 그 prim 것, 원점만 메시 중심으로
            _M = _np.linalg.inv(_p(LINK7)) @ _T
        else:
            _M = _np.linalg.inv(_p(LINK7)) @ _p(CAMERA)
        _Tf = _kin.fk_T(_np.radians(_np.array(Q_HOME_DEG))).copy()
        _Tf[:3, 3] /= 1000.0
        make_ik_scene(args.scene, args.out, home_tip=_Wb @ _Tf @ _M,
                      hide_spider=hide_spider)

    import numpy as np
    import omni.usd as _ousd
    from omni.isaac.core import World
    from omni.isaac.core.robots import Robot
    from omni.isaac.core.utils.stage import open_stage
    from omni.usd import get_context
    from pxr import Usd, UsdGeom, Gf, Vt

    sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))
    from utils.robot import xarm7_kinematics as kin

    open_stage(args.out)
    for _ in range(60):
        app.update()
    stage = get_context().get_stage()

    xc = UsdGeom.XformCache()

    def pose(path):
        """prim 의 월드 자세 → numpy 열벡터.

        USD Gf.Matrix4d 는 행벡터(p'=p·M), 운동학은 열벡터(x_B = T @ x_E) → 전치.
        매 호출 새로 계산하므로 XformCache 의 stale 값 문제가 없다.
        """
        return np.array(_ousd.get_world_transform_matrix(
            stage.GetPrimAtPath(path))).T

    W_base = pose(ROBOT)
    if args.target == "flange":
        M_l7_tip = np.eye(4)
    elif tip_prim:
        _T = pose(tip_prim).copy()
        _c = _mesh_center_world(stage, tip_prim)
        if _c is not None:
            _T[:3, 3] = _c
        M_l7_tip = np.linalg.inv(pose(LINK7)) @ _T
    else:
        M_l7_tip = np.linalg.inv(pose(LINK7)) @ pose(CAMERA)
    print(f"[ik] base(world) {np.round(W_base[:3,3],4)}  "
          f"타깃='{args.target}'  link7→tip {np.round(M_l7_tip[:3,3]*1000,1)}mm")

    world = World(physics_dt=1/60.0, rendering_dt=1/60.0, stage_units_in_meters=1.0)
    robot = world.scene.add(Robot(prim_path=ROBOT, name="xarm7"))
    world.reset()

    # ⚠ 떨림 방지 — IsaacWorld 와 동일하게 세팅한다.
    #   드라이브 게인이 USD 기본값이면 PD 가 물러 진동하고, maxForce 가 낮으면 중력
    #   토크에 포화해 목표에 못 붙는다.
    from isaacsim.core.utils.types import ArticulationAction
    try:
        view = robot._articulation_view
        nd = view.num_dof
        view.set_gains(kps=np.full((1, nd), 2000.0, dtype=np.float32),
                       kds=np.full((1, nd), 200.0, dtype=np.float32))
        view.set_max_efforts(np.full((1, nd), 500.0, dtype=np.float32))
    except Exception as e:
        print(f"[ik][WARN] 드라이브 게인 설정 실패(무시): {e}")

    def solve(W_tip, q_cur):
        """IK + 검증. 반환 (q_new, ok, reason).

        ⚠ 위치만 보면 안 된다 — 미도달 타깃에서 DLS 가 관절한계를 타고 **팔이 뒤집힌**
          자세로 수렴하면 위치는 맞아도 스캐너가 반대를 본다. 자세·관절급변도 본다.
        """
        W_l7 = W_tip @ np.linalg.inv(M_l7_tip)
        T = np.linalg.inv(W_base) @ W_l7
        p6 = np.concatenate([T[:3, 3] * 1000.0, kin.R_to_euler_xyz(T[:3, :3])])
        qn, conv = kin.ik(p6, seed=q_cur)
        if not conv:
            return q_cur, False, "IK 미수렴(도달 불가)"
        Tf = kin.fk_T(qn).copy(); Tf[:3, 3] /= 1000.0
        W_got = W_base @ Tf @ M_l7_tip
        perr = float(np.linalg.norm(W_got[:3, 3] - W_tip[:3, 3]))
        Rr = W_got[:3, :3].T @ W_tip[:3, :3]
        rerr = float(np.degrees(np.arccos(np.clip((np.trace(Rr) - 1) / 2, -1, 1))))
        dj = float(np.degrees(np.abs(qn - q_cur)).max())
        if perr > args.pos_tol:
            return q_cur, False, f"위치오차 {perr*1000:.1f}mm"
        if rerr > args.rot_tol:
            return q_cur, False, f"자세오차 {rerr:.1f}°"
        if dj > args.max_jump:
            return q_cur, False, f"관절 급변 {dj:.0f}° — 팔 뒤집힘 방지(EE 정지)"
        return qn, True, ""

    def drive(qv, teleport=False):
        """드라이브 **위치 타깃만** 갱신 → PD 가 물리적으로 부드럽게 추종한다.

        ⚠ set_joint_positions(텔레포트)를 매 프레임 쓰면 가감속이 전혀 없어 뚝뚝 끊긴다.
          IsaacWorld.drive_to_joints 가 텔레포트하는 건 '목표 자세 정확 도달'이 목적인
          배치용이고, 인터랙티브 추종에는 맞지 않는다.
          단, 드라이브 타깃을 갱신하지 않으면 PD 가 옛 목표로 되끌어 **떨린다** →
          apply_action 은 매 스텝 반드시 재적용할 것.
        teleport=True 는 초기 자세 세팅용(시작 시 한 번).
        """
        act = ArticulationAction(joint_positions=np.asarray(qv, dtype=float))
        try:
            robot.apply_action(act)
            if teleport:
                robot.set_joint_positions(qv)
                robot.set_joint_velocities(np.zeros_like(qv))
        except Exception as e:
            print(f"[ik][WARN] 관절 구동 보류({type(e).__name__})")
        return act

    hit = stage.GetPrimAtPath(f"{TARGET}/hit")
    q = np.radians(np.array(Q_HOME_DEG))
    act = drive(q, teleport=True)        # 시작 자세만 텔레포트로 정확히 맞춘다
    last_ok, n = None, 0
    last_tip, last_why, prev_raw = None, "", None
    print("[ik] Stage 에서 /World/IKTarget 을 선택해 기즈모로 끌어보세요 "
          "(초록=도달, 빨강=실패). Ctrl+C 로 종료.")

    if args.selftest:
        # 대상 중심 주변을 훑어 도달 가능 영역을 표 형태로 출력
        tgt_xf = UsdGeom.Xformable(stage.GetPrimAtPath(TARGET))
        op = tgt_xf.GetOrderedXformOps()[0]
        base_t = np.array(op.Get())
        print(f"\n{'dx':>6}{'dy':>6}{'dz':>6}   도달")
        nok = ntot = 0
        for dz in (-0.10, 0.0, 0.10):
            for dy in (-0.15, 0.0, 0.15):
                for dx in (-0.15, 0.0, 0.15):
                    op.Set(Gf.Vec3d(*(base_t + np.array([dx, dy, dz]))))
                    xc.Clear()
                    _, ok, why = solve(pose(TARGET), q)
                    ntot += 1; nok += bool(ok)
                    print(f"{dx:>6.2f}{dy:>6.2f}{dz:>6.2f}   {'O' if ok else 'X'}"
                          + (f"  {why}" if why else ""))
        print(f"\n도달 {nok}/{ntot}")
        app.close(); return

    while app.is_running():
        robot.apply_action(act)      # 매 스텝 타깃 재적용 — 드라이브가 흘러내리지 않게
        world.step(render=True)
        n += 1
        if n % 3:                                   # 3프레임마다 IK (부하 완화)
            continue
        xc.Clear()
        W_tip = pose(TARGET)
        # 연속 2회 같은 값일 때만 반영 (기즈모 조작 중의 과도값 필터)
        if prev_raw is None or not np.allclose(W_tip, prev_raw, atol=1e-6):
            prev_raw = W_tip.copy()
            continue
        if last_tip is not None and np.allclose(W_tip, last_tip, atol=1e-5):
            continue                       # 정지 상태 — 재계산·재텔레포트 안 함
        last_tip = W_tip.copy()

        qn, ok, why = solve(W_tip, q)
        if ok:
            q = qn
            act = drive(q)          # 도달 O 일 때만 움직인다. 실패면 EE 정지 유지.
        if ok != last_ok or (not ok and why != last_why):
            col = (0.2, 0.9, 0.2) if ok else (0.95, 0.15, 0.15)
            UsdGeom.Sphere(hit).GetDisplayColorAttr().Set(
                Vt.Vec3fArray([Gf.Vec3f(*col)]))
            if ok:
                print(f"[ik] 도달 O  타깃 {np.round(W_tip[:3,3],3)}  "
                      f"q(deg)={np.round(np.degrees(q),1).tolist()}")
            else:
                print(f"[ik] ✘ 도달 불가 — {why}   타깃 {np.round(W_tip[:3,3],3)}  "
                      f"→ EE 정지(현재 자세 유지)")
            last_ok, last_why = ok, why

    app.close()


if __name__ == "__main__":
    main()
