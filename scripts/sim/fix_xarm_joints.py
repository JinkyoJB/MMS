#!/usr/bin/env python
"""fix_xarm_joints.py — USD 의 xArm7 관절 프레임을 **실물 컨트롤러 기준**으로 맞춘다.

왜
--
v2.usd 계열은 팔이 J2 가 꺾인 자세로 저작돼 있었고, 그 자세가 RevoluteJoint 의
localRot0 에 녹아들어 **그게 q=0** 이 돼 있었다. 그래서 같은 관절각에 대해 sim 과
실물이 최대 383mm 어긋났다. 링크 기하·축 방향은 정상이라 **메시는 건드리지 않는다** —
조인트의 고정변환만 교체하면 q=0 의 정의가 실물과 같아진다.

값의 출처는 `utils/robot/xarm7_kinematics.py`(=`config/calibration/xarm7_dh.yaml`).
해석 모델과 USD 가 **같은 숫자**를 쓰게 하는 것이 이 스크립트의 목적이다.

    $ISAAC scripts/sim/fix_xarm_joints.py --scene <in.usd> --out <out.usd>
    $ISAAC scripts/sim/fix_xarm_joints.py --scene <in.usd> --verify-only
"""
from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation as _Rot

_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(_ROOT))
from mms_paths import asset                                   # noqa: E402
# ★ SimulationApp 앞에서 임포트 — Isaac 이 `utils` 를 선점한다(export_env_mesh.py 와 동일)
from utils.robot import xarm7_kinematics as kin               # noqa: E402


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--scene", default="frame_xarm7_spider_turntable/v2_real.usd")
    ap.add_argument("--out", default=None, help="생략하면 <scene>_fixed.usd")
    ap.add_argument("--robot", default="/World/xarm7")
    ap.add_argument("--verify-only", action="store_true")
    args = ap.parse_args()

    scene = args.scene if os.path.isabs(args.scene) else asset(args.scene)
    out = args.out or str(Path(scene).with_name(Path(scene).stem + "_fixed.usd"))
    os.environ.setdefault("OMNI_KIT_ACCEPT_EULA", "YES")

    from isaacsim import SimulationApp
    app = SimulationApp({"headless": True})
    import omni.usd
    from pxr import Gf, Usd, UsdGeom

    ctx = omni.usd.get_context()
    ctx.open_stage(scene)
    for _ in range(90):
        app.update()
    stage = ctx.get_stage()
    mpu = UsdGeom.GetStageMetersPerUnit(stage)
    print(f"[fix] {scene}\n[fix] metersPerUnit={mpu}")

    jroot = stage.GetPrimAtPath(f"{args.robot}/joints")
    if not jroot.IsValid():
        print("[fix] joints 스코프 없음", file=sys.stderr); app.close(); sys.exit(1)
    joints = {j.GetName(): j for j in jroot.GetChildren()}

    # 현재 값 대비 얼마나 바뀌는지 먼저 보여준다 — 눈으로 확인하고 저장.
    print("\n[fix] 관절 고정변환 변화 (localPos0 / localRot0)")
    changes = []
    for i in range(7):
        name = f"joint{i+1}"
        j = joints.get(name)
        if j is None:
            print(f"  {name}: 없음 — 건너뜀"); continue
        newp = kin._LOCALPOS0[i] / mpu
        newr = kin._LOCALROT0[i]                      # [w,x,y,z]
        oldp = np.array(j.GetAttribute("physics:localPos0").Get(), float)
        oq = j.GetAttribute("physics:localRot0").Get()
        oldr = np.array([oq.GetReal(), *oq.GetImaginary()], float)
        dp = np.linalg.norm(newp - oldp) * mpu * 1000.0
        dot = abs(float(np.dot(newr / np.linalg.norm(newr), oldr / np.linalg.norm(oldr))))
        dang = np.degrees(2 * np.arccos(min(1.0, dot)))
        print(f"  {name}:  Δpos {dp:8.3f} mm   Δrot {dang:8.3f} deg")
        changes.append((j, newp, newr))

    if args.verify_only:
        print("\n[fix] --verify-only — 저장하지 않음"); app.close(); return

    for j, newp, newr in changes:
        j.GetAttribute("physics:localPos0").Set(Gf.Vec3f(*[float(v) for v in newp]))
        j.GetAttribute("physics:localRot0").Set(
            Gf.Quatf(float(newr[0]), Gf.Vec3f(float(newr[1]), float(newr[2]), float(newr[3]))))

    # 링크 프림의 rest 변환도 새 FK(q=0) 로 맞춘다. 안 하면 저작된 자세와 관절
    # 구속이 어긋나 PhysX 가 첫 스텝에서 튕긴다(경고 + 눈에 보이는 snap).
    F = kin._fk_frames_m(np.zeros(7))
    print("\n[fix] 링크 rest 변환 → 새 FK(q=0)")
    for i in range(8):
        path = f"{args.robot}/link_base" if i == 0 else f"{args.robot}/link{i}"
        p = stage.GetPrimAtPath(path)
        if not p.IsValid():
            print(f"  {path}: 없음 — 건너뜀"); continue
        T = F[i].copy()
        T[:3, 3] /= mpu
        # ⚠ 기존 op 구성(translate/orient/**scale**)을 유지한채 값만 바꿈다.
        #   ClearXformOpOrder() 후 transform 하나로 덮으면 scale 이 사라져
        #   RigidBody 관성/충돌이 깨지고 PhysX 아티큐레이션이 발산한다(실측).
        from pxr import Gf as _Gf
        quat = _Rot.from_matrix(T[:3, :3]).as_quat()          # xyzw
        x = UsdGeom.Xformable(p)
        names = [o.GetOpName() for o in x.GetOrderedXformOps()]
        if "xformOp:translate" in names and "xformOp:orient" in names:
            p.GetAttribute("xformOp:translate").Set(
                _Gf.Vec3d(*[float(v) for v in T[:3, 3]]))
            oa = p.GetAttribute("xformOp:orient")
            QT = type(oa.Get()) if oa.Get() is not None else _Gf.Quatf
            oa.Set(QT(float(quat[3]), _Gf.Vec3f(float(quat[0]), float(quat[1]), float(quat[2]))
                      if QT is _Gf.Quatf else
                      _Gf.Vec3d(float(quat[0]), float(quat[1]), float(quat[2]))))
        else:
            x.ClearXformOpOrder()
            x.AddTransformOp().Set(Gf.Matrix4d(*T.T.flatten().tolist()))
        print(f"  {path:34s} t={np.round(T[:3,3],4).tolist()}  ops={names}")

    stage.GetRootLayer().Export(out)
    print(f"\n[fix] 저장: {out}")
    app.close()


if __name__ == "__main__":
    main()
