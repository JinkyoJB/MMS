"""
place_testset_object.py — testset USD 를 턴테이블 위에 올려 배치/미리보기.

목적
----
`play/MMS/testset/*.usd` 의 다양한 사물을 베이스 씬(v2.usd: xArm+턴테이블) 의
**턴테이블 디스크 위, 축 중심에 세워** 배치한다. 스캔 실험용 사물 교체 준비.

핵심 처리 (강건성)
------------------
- **비파괴**: v2.usd 를 직접 수정하지 않는다. v2 를 subLayer 로 깔고 오버레이
  레이어에만 (마블 비활성 + TestObject 참조 + 배치 transform) 를 author → 별도
  USD 로 저장. 원본 v2.usd 는 그대로.
- **단위 정규화**: 사물 USD 의 metersPerUnit 이 씬과 다르면 scale 로 보정
  (cm 자산이 m 씬에서 100배로 뜨는 것 방지).
- **up-axis 정규화**: 사물이 Y-up 이면 +90° X 회전으로 Z-up 씬에 맞춰 세움.
- **정렬**: 사물 bbox 를 디스크 축(x,y 중심) + 디스크 상단(z, 바닥접지) 에 정렬.

실행 (Isaac 번들 python — pxr/omni 가 Kit 런타임 필요)
    S=~/workspace/MMS/MMS/scripts/sim/place_testset_object.py
    ~/isaacsim/python.sh $S --list
    ~/isaacsim/python.sh $S --object 0146_mug            # 배치+뷰어
    ~/isaacsim/python.sh $S --object 0146_mug --headless # 저장만
    ~/isaacsim/python.sh $S --all --headless             # 전 사물 각각 저장
    ~/isaacsim/python.sh $S --testset <dir> --all --headless  # 다른 testset 폴더

저장 위치(기본): testset/composed/<name>_on_turntable.usd
그 USD 를 Isaac 에서 열거나 sim 스캔 씬으로 지정해 사용.
"""
import os
import sys
import glob
import math
import argparse

# ── 경로 상수 ───────────────────────────────────────────────────────────────
V2_USD         = asset("frame_xarm7_spider_turntable/v2.usd")
# testset USD 위치 (Isaac standalone 트리). --testset 로 오버라이드 가능.
import os, sys; sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))
from mms_paths import asset, testset_dir

TESTSET_DIR    = testset_dir()
COMPOSED_DIR   = os.path.join(TESTSET_DIR, "composed")

TURNTABLE_MESH = "/World/ScanTarget/turntable_demo/turntable/turntable"
MARBLE_PRIM    = "/World/ScanTarget/Solid_Marble"      # 기존 사물(비활성화 대상)
TEST_PARENT    = "/World/ScanTarget/TestObject"        # 우리가 transform 을 거는 Xform
TEST_ASSET     = TEST_PARENT + "/Asset"                # 사물 USD 참조 지점
DISC_TOP_Z_FALLBACK = 0.713                            # 턴테이블 bbox 실패 시 (USD 검증값)


def list_objects():
    return sorted(glob.glob(os.path.join(TESTSET_DIR, "*.usd")))


def resolve_object(spec: str) -> str:
    """spec = 파일명/스템/인덱스(0086)/절대경로 → 절대 usd 경로."""
    if os.path.isabs(spec) and os.path.isfile(spec):
        return spec
    for p in list_objects():
        stem = os.path.splitext(os.path.basename(p))[0]
        if spec == stem or spec == os.path.basename(p) or stem.startswith(spec):
            return p
    raise FileNotFoundError(f"testset 에서 '{spec}' 를 못 찾음. --list 로 확인.")


# ── USD 합성 (pxr) — SimulationApp 기동 후 import ──────────────────────────────
def compose(obj_usd: str, out_usd: str, extra_scale: float, yaw_deg: float,
            z_offset_mm: float, force_rx90: bool):
    from pxr import Usd, UsdGeom, Gf, Sdf

    v2_abs = os.path.abspath(V2_USD)
    obj_abs = os.path.abspath(obj_usd)

    # 베이스 씬 단위/up-axis 를 먼저 읽는다 (v2 는 미터·Z-up).
    v2_stage = Usd.Stage.Open(v2_abs)
    scene_mpu = UsdGeom.GetStageMetersPerUnit(v2_stage) or 1.0
    up_axis = UsdGeom.GetStageUpAxis(v2_stage)

    # 오버레이 스테이지: v2 를 subLayer 로 (원본 비파괴), 오버레이에만 author.
    # ★ in-memory 루트는 mpu 기본값(0.01=cm)이라 반드시 v2 단위로 맞춰야
    #   unit_scale 계산이 옳다(안 맞추면 미터 자산이 100배로 뜸).
    stage = Usd.Stage.CreateInMemory()
    stage.GetRootLayer().subLayerPaths.append(v2_abs)
    UsdGeom.SetStageMetersPerUnit(stage, scene_mpu)
    UsdGeom.SetStageUpAxis(stage, up_axis)

    # 기존 사물(마블) 비활성 (오버레이 over prim).
    marble = stage.OverridePrim(MARBLE_PRIM)
    marble.SetActive(False)

    # 사물 참조 (defaultPrim). 우리 transform 은 부모 Xform 에만 → 자산 내부 보존.
    UsdGeom.Xform.Define(stage, TEST_PARENT)
    asset = stage.DefinePrim(TEST_ASSET)
    asset.GetReferences().AddReference(obj_abs)

    # ★ 물리 비활성 (오버레이 override, 원본 비파괴) — 에셋에 RigidBodyAPI 가
    # 박혀 있으면 Play 순간 디스크와의 초기 접촉을 임펄스로 해소하며 날아감.
    # 우리 스캔은 물체를 키네마틱(θ 트랜스폼)으로 돌리므로 동역학 불필요.
    try:
        from pxr import UsdPhysics
        for prim in stage.Traverse():
            if not prim.GetPath().pathString.startswith(TEST_PARENT):
                continue
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                UsdPhysics.RigidBodyAPI(prim).GetRigidBodyEnabledAttr().Set(False)
                print(f"  물리 off: {prim.GetPath()}")
    except Exception as e:
        print(f"  ⚠ 물리 비활성 실패({e}) — Play 시 물체가 튈 수 있음")

    # 사물 원본 단위/up-axis 조회 → 정규화 값 산출.
    obj_stage = Usd.Stage.Open(obj_abs)
    obj_mpu = UsdGeom.GetStageMetersPerUnit(obj_stage) or 1.0
    obj_up = UsdGeom.GetStageUpAxis(obj_stage)
    unit_scale = float(obj_mpu) / float(scene_mpu)        # 단위 맞춤
    total_scale = unit_scale * float(extra_scale)

    need_rx90 = force_rx90 or (obj_up == UsdGeom.Tokens.y and up_axis == UsdGeom.Tokens.z)

    # transform 구성: M = T * R * S  (xformOpOrder 나열 순).
    parent = UsdGeom.Xformable(stage.GetPrimAtPath(TEST_PARENT))
    parent.ClearXformOpOrder()
    tOp = parent.AddTranslateOp()
    rOp = parent.AddOrientOp()
    sOp = parent.AddScaleOp()
    sOp.Set(Gf.Vec3f(total_scale, total_scale, total_scale))

    q_yaw = Gf.Quatf(math.cos(math.radians(yaw_deg) / 2.0), 0.0, 0.0,
                     math.sin(math.radians(yaw_deg) / 2.0))
    if need_rx90:
        q_rx = Gf.Quatf(math.cos(math.radians(90) / 2.0),
                        math.sin(math.radians(90) / 2.0), 0.0, 0.0)
        q = q_yaw * q_rx                                   # up-axis 먼저, yaw 나중
    else:
        q = q_yaw
    rOp.Set(q)
    tOp.Set(Gf.Vec3d(0.0, 0.0, 0.0))                       # 우선 원점 → bbox 산출 후 이동

    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                           ['default', 'render'], useExtentsHint=True)

    # 디스크 축 중심 + 상단 z.
    tt = stage.GetPrimAtPath(TURNTABLE_MESH)
    if tt and tt.IsValid():
        r = bc.ComputeWorldBound(tt).ComputeAlignedRange()
        dmn, dmx = r.GetMin(), r.GetMax()
        disc_cx, disc_cy = (dmn[0] + dmx[0]) / 2.0, (dmn[1] + dmx[1]) / 2.0
        disc_top = dmx[2]
    else:
        print("  ⚠ 턴테이블 prim 못 찾음 — DISC_TOP_Z fallback 사용")
        disc_cx = disc_cy = 0.0
        disc_top = DISC_TOP_Z_FALLBACK

    # 사물 현재(scale/rot 적용, T=0) bbox → 중심 xy·바닥 z 정렬.
    rb = bc.ComputeWorldBound(stage.GetPrimAtPath(TEST_PARENT)).ComputeAlignedRange()
    omn, omx = rb.GetMin(), rb.GetMax()
    size = [omx[i] - omn[i] for i in range(3)]
    dx = disc_cx - (omn[0] + omx[0]) / 2.0
    dy = disc_cy - (omn[1] + omx[1]) / 2.0
    dz = disc_top - omn[2] + z_offset_mm / 1000.0
    tOp.Set(Gf.Vec3d(dx, dy, dz))

    os.makedirs(os.path.dirname(out_usd), exist_ok=True)
    stage.GetRootLayer().Export(out_usd)

    print(f"  단위 mpu: obj={obj_mpu} scene={scene_mpu} → unit_scale={unit_scale:.4g} "
          f"(×extra {extra_scale}) up:obj={obj_up}{' →Z보정' if need_rx90 else ''}")
    print(f"  사물 크기(배치후): {size[0]*1000:.0f}×{size[1]*1000:.0f}×{size[2]*1000:.0f} mm")
    print(f"  디스크 중심=({disc_cx:.3f},{disc_cy:.3f}) top_z={disc_top:.3f} → 배치 저장: {out_usd}")
    return out_usd


def view(out_usd: str, app):
    """저장된 합성 USD 를 뷰어로 열어 확인 (창 닫을 때까지 렌더)."""
    from isaacsim.core.utils.stage import open_stage
    open_stage(usd_path=out_usd)
    print("  [view] 뷰어 표시 중 — 창을 닫으면 종료")
    while app.is_running():
        app.update()


def main():
    global TESTSET_DIR, COMPOSED_DIR
    ap = argparse.ArgumentParser()
    ap.add_argument("--object", "-o", help="파일명/스템/인덱스(예: 0146_mug, 0146) 또는 절대경로")
    ap.add_argument("--all", action="store_true", help="testset 전 사물을 각각 저장(headless 권장)")
    ap.add_argument("--list", action="store_true", help="testset 목록 출력 후 종료")
    ap.add_argument("--scale", type=float, default=1.0, help="추가 배율(단위보정 위에 곱)")
    ap.add_argument("--yaw", type=float, default=0.0, help="Z축 초기 회전(도)")
    ap.add_argument("--z-offset", type=float, default=0.0, help="바닥에서 띄울 오프셋(mm)")
    ap.add_argument("--rx90", action="store_true", help="강제 +90° X 회전(up-axis 수동 보정)")
    ap.add_argument("--out", help="출력 USD 경로(단일 사물). 기본=composed/<name>_on_turntable.usd")
    ap.add_argument("--testset", help=f"testset 디렉토리 오버라이드 (기본 {TESTSET_DIR})")
    ap.add_argument("--headless", action="store_true", help="뷰어 없이 배치+저장만")
    args = ap.parse_args()

    if args.testset:
        TESTSET_DIR = os.path.abspath(args.testset)
        COMPOSED_DIR = os.path.join(TESTSET_DIR, "composed")

    if args.list:
        objs = list_objects()
        print(f"[testset] {len(objs)}개 @ {TESTSET_DIR}")
        for p in objs:
            print("  -", os.path.splitext(os.path.basename(p))[0])
        return
    if not args.object and not args.all:
        print("사용법: --object <name> | --all | --list  (자세히는 -h)")
        return

    # SimulationApp 을 가장 먼저 기동해야 pxr/omni 사용 가능.
    from isaacsim import SimulationApp
    headless = args.headless or args.all
    app = SimulationApp({"headless": headless})

    try:
        if args.all:
            for p in list_objects():
                name = os.path.splitext(os.path.basename(p))[0]
                out = os.path.join(COMPOSED_DIR, f"{name}_on_turntable.usd")
                print(f"[place] {name}")
                compose(p, out, args.scale, args.yaw, args.z_offset, args.rx90)
        else:
            obj = resolve_object(args.object)
            name = os.path.splitext(os.path.basename(obj))[0]
            out = args.out or os.path.join(COMPOSED_DIR, f"{name}_on_turntable.usd")
            print(f"[place] {name}")
            compose(obj, out, args.scale, args.yaw, args.z_offset, args.rx90)
            if not headless:
                view(out, app)
    finally:
        app.close()


if __name__ == "__main__":
    main()
