"""
make_hibase_scene.py — 로봇 베이스를 ΔH 올린 오버레이 씬 생성 (원본 비파괴).

베이스 높이 조건부 실험용: 원본 씬(v2 또는 composed testset 씬)을 subLayer 로
깔고 /World/xarm7 의 translate z 만 +ΔH 오버라이드한 새 USD 를 만든다.
real 도 동일 높이로 개조 예정 — sim 선행 검증 (2026-07-08, ΔH 스윕 참조).

실행:
  ~/isaacsim/python.sh scripts/sim/make_hibase_scene.py --dh-cm 10            # v2
  ~/isaacsim/python.sh scripts/sim/make_hibase_scene.py --dh-cm 15 \
      --scene .../composed/0146_mug_on_turntable.usd
출력: <씬폴더>/hibase/<씬이름>_dh<cm>.usd  (stdout 마지막 줄 = 경로)
"""
import os
import argparse

V2 = "/home/keti/workspace/sync/2_Rapid_Digital_Twin/1_MMS/2_3Dassets/frame_xarm7_spider_turntable/v2.usd"
ROBOT = "/World/xarm7"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dh-cm", type=float, required=True, help="베이스 상승량 (cm)")
    ap.add_argument("--scene", default=V2, help="원본 씬 (기본 v2.usd)")
    args = ap.parse_args()

    scene = os.path.abspath(args.scene)
    outdir = os.path.join(os.path.dirname(scene), "hibase")
    os.makedirs(outdir, exist_ok=True)
    stem = os.path.splitext(os.path.basename(scene))[0]
    out = os.path.join(outdir, f"{stem}_dh{args.dh_cm:.0f}.usd")

    from isaacsim import SimulationApp
    app = SimulationApp({"headless": True})
    from pxr import Usd, UsdGeom, Gf

    src = Usd.Stage.Open(scene)
    t0 = Gf.Vec3d(UsdGeom.Xformable(src.GetPrimAtPath(ROBOT))
                  .GetOrderedXformOps()[0].Get())

    stage = Usd.Stage.CreateInMemory()
    stage.GetRootLayer().subLayerPaths.append(scene)
    UsdGeom.SetStageMetersPerUnit(stage, UsdGeom.GetStageMetersPerUnit(src) or 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.GetStageUpAxis(src))
    over = stage.OverridePrim(ROBOT)
    over.CreateAttribute("xformOp:translate",
                         __import__("pxr").Sdf.ValueTypeNames.Double3).Set(
        Gf.Vec3d(t0[0], t0[1], t0[2] + args.dh_cm / 100.0))
    stage.GetRootLayer().Export(out)

    # 검증: 오버레이에서 로봇 z 가 실제로 올라갔나
    chk = Usd.Stage.Open(out)
    z = UsdGeom.XformCache(Usd.TimeCode.Default()).GetLocalToWorldTransform(
        chk.GetPrimAtPath(ROBOT)).ExtractTranslation()[2]
    print(f"[hibase] robot z: {t0[2]:.4f} → {z:.4f} (ΔH={args.dh_cm}cm)")
    print(out)
    app.close()


if __name__ == "__main__":
    main()
