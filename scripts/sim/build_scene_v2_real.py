"""build_scene_v2_real.py — **실물 셀(v2)** 에 맞춘 sim 씬을 v2.usd 위에 얹어 만든다.

왜 필요한가
-----------
셀을 v3 레이아웃으로 바꾸기로 했다가 **실제로는 안 바꿨다** (2026-09-16 현장 확인).
실물은 여전히 v2 배치인데 sim 은 `v3_scene.usd` 를 띄운다. 두 셀은 기하가 근본적으로
다르다 — 턴테이블이 로봇 base 에서 sim 은 수평 0mm(바로 아래), 실물은 799mm 다.
그래서 **자세 선정·도달성·이동량에 대해 sim 이 real 을 전혀 예측하지 못한다**
(같은 az 변화가 실물에서 약 2배 관절이동을 요구한다).

이 스크립트는 `v2.usd` 를 subLayer 로 깔고 **오버라이드만** 얹어(비파괴) 실물 배치를
재현한다. 현장에서 손으로 한 편집(툴체인저 비활성화 + 턴테이블 이동)을 스크립트로
못박아, 누가 다시 돌려도 같은 씬이 나오게 한다.

배치 근거 (전부 실측/역추적, 2026-09-17)
---------------------------------------
1. **로봇 base world = (0.406, 0, 1.4069)**
   `utils/collision/data/cell_env.npz`(= `v2_layout_real`, 현장 실측 반영본)와
   v2.usd 구조물을 base 프레임에서 대조해 역추적했다. X 를 0.386~0.426 으로 훑으면
   0.406 에서 **일치율 99.7%** 로 뾰족한 최대가 나온다(Y=0, Z=1.4069 도 같은 방식).
   v2.usd 원본은 0.5383 이므로 **+132mm** 이동한 것이고, 이는
   `utils/collision/data/layouts/README.md` 의 기록("base X 0.538 → 0.406")과 일치한다.

2. **비활성화 prim 12개** — 같은 대조에서 "v2.usd 에는 있는데 실측 점군에는 없는"
   것을 골라냈다. 일치율이 0~9% 로 뚝 떨어져 경계가 분명하다(나머지는 99~100%).
   툴체인저 툴플레이트 4개 · 그리퍼 · Helios2 · Femto Bolt · 부속 5개.

3. **턴테이블 = 실측 `T_B_F0` 위치**
   디스크 상면 중심은 `TurntableTransformConfig.axis_point_B`(= inv(T_BF0) 의 t)를
   base→world 로 옮긴 점이다. ⚠ `T_BF0[:3,3]` 을 그대로 쓰면 안 된다(그건 변환 성분).
   실물 rim 반경은 121.5mm 인데 v2.usd 의 `turntable_demo` 는 75mm **placeholder** 라
   대신 **v3 CAD 파트**(`parts/turntable_disc.usd`, 반경 119mm)를 쓴다. 실측 점군의
   디스크도 r≈120~130mm 까지 차 있어 CAD 쪽이 맞다.

4. **카메라 = 실측 hand-eye `T_EC_artec`**
   v2/v3 씬의 카메라는 CAD 위치라 실측 hand-eye 와 104mm 어긋나 있었다. 여기서는
   캘리브 값을 그대로 넣어 **sim 의 T_EC = real 의 T_EC** 가 되게 한다.
   USD 카메라는 광축 −Z / up +Y, hand-eye 는 OpenCV(+Z, y-down) 규약이므로
   `T_E_camUSD = T_EC_opencv · Rx(180°)` 로 변환한다.

prim 경로는 **v3 규약 그대로** 만든다(`/World/frame/turntable_disc`,
`/World/xarm7/link7/tool/spider/Camera`, `/World/ScanTarget/TestObject`).
그래야 `isaac_world.py` 상수를 안 고치고 그대로 쓸 수 있다.

실행
----
    env -u PYTHONPATH ~/miniconda3/envs/step2usd/bin/python \
        scripts/sim/build_scene_v2_real.py
    # 대상물까지:  --object "$(...)/testset/0101_spray_can.usd"

⚠ 턴테이블 **기울기**: 실측 `T_B_F0` 의 축은 base Z 에서 **9.0°** 기울어 있다.
  기본값(`--tilt level`)은 이를 **무시하고 수평**으로 놓는다. 두 가지 이유다 —
  (a) `IsaacTurntable` 이 world-Z 축으로 회전시키므로 기울면 회전축이 어긋난다,
  (b) 9° 는 기계적으로 비현실적이라 rim 캘리브의 축 방향 오차가 의심된다
      (원 중심은 잘 잡혀도 17점이 한쪽에 몰리면 평면 법선은 크게 틀어진다).
  실물에서 수평계로 확인할 것. 재현하려면 `--tilt calib`.
"""
from __future__ import annotations

import argparse
import math
import os
import sys

import numpy as np
from pxr import Usd, UsdGeom, Gf, Sdf

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))          # build_scene_v3
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
from mms_paths import asset                                    # noqa: E402

# ── 실측 배치 상수 ──────────────────────────────────────────────────────────
#: 로봇 base world 위치 (§1). v2.usd 원본은 (0.5383, 0, 1.4069).
BASE_POS = (0.4060, 0.0, 1.40690)
#: v2.usd 의 xarm7 루트 회전 = diag(-1, 1, -1) (천장에 매달린 자세). 그대로 둔다.
BASE_ROT = ((-1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, -1.0))

#: 실측 점군에 없는 = 현장에서 떼어낸 prim (§2). `/World/Frame/frame_structure/` 하위.
DEAD_PRIMS = [
    "tn__5K_TCC1_TOOL_PLATE_20240724_",      # 툴체인저 툴측 플레이트 ×4 (툴스탠드 거치분)
    "tn__5K_TCC1_TOOL_PLATE_20240724_1_",
    "tn__5K_TCC1_TOOL_PLATE_20240724_2_",
    "tn__5K_TCC1_TOOL_PLATE_20240724_3_",
    "tn__3F_DeltoGripper_tF",                # 그리퍼
    "Helios2",                               # ToF 카메라
    "tn__Femtobolt_k9",                      # Femto Bolt
    "tn____1_pWZ5vjAufHsTrz8tZC",            # 툴스탠드 부속
    "tn____2_pWZ5vjAufHsTrz8tZC",
    "tn____3_pWZ5vjAufHsTrz8tZC",
    "tn____vkW2Zw5wbBzKWk6yb9",
    "tn____vkW2ya0yl6kv7frLti1",
]
#: v2.usd 의 placeholder 턴테이블(반경 75mm)과 옛 대상물 — v3 CAD 로 대체한다.
DEAD_SCANTARGET = ["turntable_demo", "Solid_Marble"]

#: 현장 확인 후 손으로 바로잡은 부재 (2026-09-17, 사용자 수정분을 스크립트에 흡수).
#  `frame_structure` 는 **mm 단위 + rotateX(90°)** 라 로컬 X 만 옮기면 된다(둘 다 순수
#  평행이동, 회전 동일). 값을 여기 박아두지 않으면 씬을 다시 만들 때마다 손보정이 날아간다.
FIELD_FIX_LOCAL_X = {
    "xarm7_ceiling_mount":    396.39862744,   # 0 → 396.4mm
    "tn__HFS84080600_2_jJ9":   399.47115158,  # 80.69 → 399.47mm (로봇 베이스 빔)
}

#: 턴테이블은 **v3 씬의 조립체 전체**를 통째로 가져온다 — 원판만 옮기면 모터·베어링·
#  엔코더·드라이버가 빠져 충돌 게이트에 **디스크 밑이 빈 공간**으로 남는다
#  (실측 2026-09-17: 디스크 밑 h +5~+50mm 대역 점 수 19,545 → 178).
#  선택 기준은 build_scene_v3.py 와 같다(축에서 반경 안 + Z 밴드) — 이름 나열 금지.
V3_SCENE_REL = "frame_xarm7_spider_turntable_v2/v3_scene.usd"
V3_AXIS_XY = (0.365, 0.0)         # v3 씬의 턴테이블 축 (world)
V3_DISC_TOP_Z = 0.665             # v3 씬의 디스크 상면 z (world)
TT_GROUP_RADIUS = 0.16
TT_GROUP_Z = (0.50, 0.70)     # build_scene_v3 와 동일 — 0.45 로 넓히면 v3 프레임 빔이 섞인다

CAM_ATTRS = dict(                 # v3 씬 카메라에서 가져온 기본값
    clippingRange=Gf.Vec2f(0.01, 0.5), focalLength=10.0,
    horizontalAperture=20.5, verticalAperture=15.2908,
    focusDistance=0.225, fStop=0.0,
)


def set_matrix(prim, M: Gf.Matrix4d) -> None:
    """prim 의 로컬 변환을 단일 xformOp:transform 으로 덮어쓴다."""
    x = UsdGeom.Xformable(prim)
    x.SetXformOpOrder([])
    x.AddTransformOp().Set(M)


def mat4(R, t) -> Gf.Matrix4d:
    """행벡터 USD 행렬 만들기 (numpy 열벡터 (R,t) → USD)."""
    R = np.asarray(R, float)
    return Gf.Matrix4d(
        float(R[0][0]), float(R[1][0]), float(R[2][0]), 0.0,
        float(R[0][1]), float(R[1][1]), float(R[2][1]), 0.0,
        float(R[0][2]), float(R[1][2]), float(R[2][2]), 0.0,
        float(t[0]), float(t[1]), float(t[2]), 1.0)


def turntable_pose_world(tilt: str):
    """실측 `T_B_F0` → 디스크 **상면 중심** world 좌표 + 상면 법선(world).

    반환 (center_w, up_w). `axis_point_B` 를 써야 한다 — `T_BF0[:3,3]` 은 변환 성분
    이라 그걸 위치로 쓰면 로봇 반대편(2.1m 밖)을 가리킨다(2026-09-16 실물 충돌).
    """
    from utils.transforms import load_transform, TurntableTransformConfig
    tt = TurntableTransformConfig(
        load_transform("config/calibration/turntable_frame.yaml", "T_B_F0"))
    R = np.asarray(BASE_ROT, float)
    center_w = R @ tt.axis_point_B + np.asarray(BASE_POS, float)
    # F 의 +z 는 base +Z(= 천장마운트에서 '아래') 쪽으로 정규화돼 있다 → 상면 법선은 그 반대.
    up_w = -(R @ tt.axis_dir_B)
    tilt_deg = math.degrees(math.acos(abs(float(np.clip(up_w[2], -1, 1)))))
    if tilt == "level":
        up_w = np.array([0.0, 0.0, 1.0])
    return center_w, up_w / np.linalg.norm(up_w), tilt_deg


def rot_from_up(up) -> np.ndarray:
    """상면 법선을 +Z 로 갖는 회전(방위는 임의 — 회전대칭이라 무관)."""
    z = np.asarray(up, float); z = z / np.linalg.norm(z)
    a = np.array([1.0, 0.0, 0.0])
    if abs(float(z @ a)) > 0.9:
        a = np.array([0.0, 1.0, 0.0])
    x = np.cross(a, z); x /= np.linalg.norm(x)
    return np.column_stack([x, np.cross(z, x), z])


def v3_turntable_parts(v3_path: str):
    """v3 씬에서 턴테이블 뭉치 파트 이름 + `/World/frame` 의 로컬 변환을 돌려준다."""
    src = Usd.Stage.Open(v3_path)
    xc = UsdGeom.XformCache()
    pred = Usd.TraverseInstanceProxies(Usd.PrimAllPrimsPredicate)
    L_frame = Gf.Matrix4d(xc.GetLocalTransformation(
        src.GetPrimAtPath("/World/frame"))[0])
    names = []
    for c in src.GetPrimAtPath("/World/frame").GetChildren():
        V = []
        for d in Usd.PrimRange(c, pred):
            if not d.IsA(UsdGeom.Mesh):
                continue
            M = np.array(xc.GetLocalToWorldTransform(d)).T
            q = UsdGeom.Mesh(d).GetPointsAttr().Get()
            if q:
                V.append(np.asarray(q, float) @ M[:3, :3].T + M[:3, 3])
        if not V:
            continue
        ctr = np.vstack(V).mean(0)
        if (math.hypot(ctr[0] - V3_AXIS_XY[0], ctr[1] - V3_AXIS_XY[1]) < TT_GROUP_RADIUS
                and TT_GROUP_Z[0] < ctr[2] < TT_GROUP_Z[1]):
            names.append(c.GetName())
    return names, L_frame


def main() -> None:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--src", default=asset(
        "frame_xarm7_spider_turntable/v2.usd"), help="소스 형상 USD")
    ap.add_argument("--out", default=asset(
        "frame_xarm7_spider_turntable/v2_real_260917.usd"), help="출력 씬 USD")
    ap.add_argument("--object", default=None,
                    help="스캔 대상 USD (testset/*.usd). 없으면 대상물 생략")
    ap.add_argument("--tilt", choices=("level", "calib"), default="level",
                    help="턴테이블 기울기 — level(기본, 수평) 또는 calib(실측 9° 재현)")
    a = ap.parse_args()

    src_dir = os.path.dirname(os.path.abspath(a.out))
    src_rel = "./" + os.path.relpath(os.path.abspath(a.src), src_dir).replace(os.sep, "/")
    if os.path.exists(a.out):
        os.remove(a.out)                       # 이전 결과가 남아 있으면 subLayer 가 꼬인다
    stage = Usd.Stage.CreateNew(a.out)
    stage.GetRootLayer().subLayerPaths.append(src_rel)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    stage.SetDefaultPrim(stage.GetPrimAtPath("/World"))
    print(f"[build] src  {a.src}\n[build] out  {a.out}")

    # ── (1) 로봇 base 를 실측 위치로 ────────────────────────────────────
    R = np.asarray(BASE_ROT, float)
    set_matrix(stage.OverridePrim("/World/xarm7"), mat4(R, BASE_POS))
    print(f"  로봇 base → world {tuple(round(v, 4) for v in BASE_POS)} "
          f"(v2.usd 원본 0.5383 → {BASE_POS[0]}, +132mm)")

    # ★ transform 만 바꾸면 **Play 순간 원래 자리로 튕긴다** — root_joint(FixedJoint)의
    #   localPos1/localRot1 이 링크베이스의 월드 자세를 물리적으로 결정하기 때문이다.
    #   새 배치의 역변환으로 갱신한다(build_scene_v3.py 와 같은 처리).
    L1 = mat4(R, BASE_POS).GetInverse()
    rj = stage.OverridePrim("/World/xarm7/joints/root_joint")
    q = L1.ExtractRotationQuat()
    rj.CreateAttribute("physics:localPos1", Sdf.ValueTypeNames.Point3f).Set(
        Gf.Vec3f(L1[3][0], L1[3][1], L1[3][2]))
    rj.CreateAttribute("physics:localRot1", Sdf.ValueTypeNames.Quatf).Set(
        Gf.Quatf(float(q.GetReal()), Gf.Vec3f(*[float(v) for v in q.GetImaginary()])))
    print("  root_joint 앵커 갱신 (Play 시 튕김 방지)")

    # ── (2) 현장에서 떼어낸 것 비활성화 ─────────────────────────────────
    n = 0
    for nm in DEAD_PRIMS:
        p = stage.GetPrimAtPath(f"/World/Frame/frame_structure/{nm}")
        if p and p.IsValid():
            stage.OverridePrim(f"/World/Frame/frame_structure/{nm}").SetActive(False)
            n += 1
        else:
            print(f"  ⚠ 없음(건너뜀): {nm}")
    for nm in DEAD_SCANTARGET:
        p = stage.GetPrimAtPath(f"/World/ScanTarget/{nm}")
        if p and p.IsValid():
            stage.OverridePrim(f"/World/ScanTarget/{nm}").SetActive(False)
            n += 1
    print(f"  비활성화 {n}개 (툴체인저·그리퍼·보조카메라 + v2 placeholder 턴테이블·대상물)")

    # ── 현장 보정 부재 ──────────────────────────────────────────────────
    for nm, x_mm in FIELD_FIX_LOCAL_X.items():
        src_prim = stage.GetPrimAtPath(f"/World/Frame/frame_structure/{nm}")
        if not (src_prim and src_prim.IsValid()):
            print(f"  ⚠ 현장보정 대상 없음(건너뜀): {nm}")
            continue
        M = UsdGeom.Xformable(src_prim).GetLocalTransformation()
        rows = [[M[r][c] for c in range(4)] for r in range(4)]
        old_x = rows[3][0]
        rows[3][0] = float(x_mm)               # USD 행벡터 → 마지막 행이 translation
        set_matrix(stage.OverridePrim(f"/World/Frame/frame_structure/{nm}"),
                   Gf.Matrix4d(*[v for row in rows for v in row]))
        print(f"  현장보정 {nm}: 로컬 X {old_x:.2f} → {x_mm:.2f}mm")

    # ★ `/World/ScanTarget` 은 v2.usd 에서 translate (0.3015, 0, 0) 이 걸려 있다.
    #   그 아래 대상물을 world 좌표로 놓으면 그만큼 밀린다(실측: X 가 302mm 어긋났다).
    #   자식 둘을 모두 껐으니 identity 로 되돌려 world == local 로 만든다.
    set_matrix(stage.OverridePrim("/World/ScanTarget"), Gf.Matrix4d(1.0))

    # ── (3) 턴테이블 = v3 조립체 전체를 실측 T_B_F0 자리로 ──────────────
    center_w, up_w, tilt_deg = turntable_pose_world(a.tilt)
    v3_path = asset(V3_SCENE_REL)
    names, L_frame = v3_turntable_parts(v3_path)
    # Δ = v3 디스크 상면 → 실측 디스크 상면 (기울기 옵션이면 회전까지)
    c0 = np.array([V3_AXIS_XY[0], V3_AXIS_XY[1], V3_DISC_TOP_Z])
    Rd = rot_from_up(up_w) @ rot_from_up((0.0, 0.0, 1.0)).T
    D = np.eye(4); D[:3, :3] = Rd; D[:3, 3] = center_w - Rd @ c0
    # `/World/frame` 에 Δ·(v3 frame 로컬) 을 걸면, 그 아래 참조한 파트들이 각자의
    # v3 로컬 변환만으로 **Δ 만큼 옮겨진 v3 월드 위치**에 정확히 놓인다.
    fr = UsdGeom.Xform.Define(stage, "/World/frame").GetPrim()
    # ⚠ USD Gf.Matrix4d 는 **행벡터**(p' = p·M) 라 합성 순서가 numpy(열벡터)와 반대다.
    #   원하는 것은 열벡터로 Δ·L_frame → 행벡터로는 L_frame * Δ 다.
    set_matrix(fr, L_frame * mat4(D[:3, :3], D[:3, 3]))
    for nm in names:
        p = UsdGeom.Xform.Define(stage, f"/World/frame/{nm}").GetPrim()
        p.GetReferences().AddReference(v3_path, f"/World/frame/{nm}")
    print(f"  턴테이블 = v3 조립체 {len(names)}개 파트를 통째로 이동 "
          f"(원판·모터·베어링·엔코더·드라이버)")
    print(f"    디스크 상면 world = ({center_w[0]:+.4f}, {center_w[1]:+.4f}, "
          f"{center_w[2]:+.4f})  반경 119mm(CAD, 실측 rim 121.5mm)")
    print(f"    실측 축 기울기 {tilt_deg:.1f}° — --tilt={a.tilt} "
          + ("(무시하고 수평)" if a.tilt == "level" else "(재현)"))

    # ── (4) EE 툴 스택 = v3 그대로 ──────────────────────────────────────
    #  v3 의 `tool` 서브트리(툴체인저 브래킷→마스터→툴플레이트→어댑터→스캐너→Camera)를
    #  **손대지 않고** 통째로 참조한다. 실물 EE 구성과 같고, 부재 간 상대자세가 CAD 대로
    #  보존된다(어댑터↔스캐너 간극 0.1mm = 볼트 결합).
    #
    #  ⚠ 한때 여기서 `spider` 를 옮겨 Camera 를 실측 hand-eye(`T_EC_artec`) 자리에
    #    맞췄다가 **스캐너가 어댑터에서 112mm 떠버렸다**(2026-09-17). 광학 프레임만
    #    옮기려던 것인데 스캐너 몸체가 같이 딸려간 것이다.
    #    sim 에서는 USD 가 진실이고 `system.py` 가 T_EC 를 USD 에서 다시 읽으므로
    #    (`_sim_T_EC_gt`), 여기서 캘리브 값을 억지로 넣을 이유가 없다.
    #
    #  ※ 남은 과제 — 실측 hand-eye 와 CAD 툴스택이 104mm 어긋난다(v3 도 동일).
    #    둘 중 무엇이 맞는지는 실물에서 확인할 문제지, 씬에서 카메라를 순간이동시켜
    #    덮을 문제가 아니다.
    tool = UsdGeom.Xform.Define(stage, "/World/xarm7/link7/tool").GetPrim()
    tool.GetReferences().AddReference(v3_path, "/World/xarm7/link7/tool")
    set_matrix(tool, Gf.Matrix4d(1.0))             # link7 직속 (v3 도 identity)
    # v2.usd 가 link7 에 직접 달아둔 옛 스캐너 메시는 끈다 — 스캐너가 둘이 되면
    # 시각적으로도 헷갈리고 자기점 필터에도 걸린다.
    if stage.GetPrimAtPath("/World/xarm7/link7/Artec_Space_Spider_mm").IsValid():
        stage.OverridePrim("/World/xarm7/link7/Artec_Space_Spider_mm").SetActive(False)
    print("  EE 툴 스택 = v3 `tool` 서브트리 그대로 참조 (스캐너 장착자세 CAD 유지)")

    # ── (5) 스캔 대상 ───────────────────────────────────────────────────
    #  배치 규칙(바닥면을 원판 상면에, 축에 정렬, 하위 RigidBody 를 kinematic 화)은
    #  **v3 빌더와 같은 함수**를 쓴다 — 두 벌로 갈라지면 한쪽만 고쳐진다.
    if a.object:
        stage.GetRootLayer().Save()               # 디스크 reference 해석 후 bbox 계산
        from build_scene_v3 import add_scan_target
        add_scan_target(stage, os.path.abspath(a.object))

    stage.GetRootLayer().Save()
    print(f"[build] 저장 완료 → {a.out}")
    print("\n  다음:  MMS_SIM_USD=<out> 로 main_artec.py 실행\n"
          "        충돌 캐시도 이 씬에서 다시 구울 것:\n"
          "          scripts/sim/export_env_mesh.py --scene <out> "
          "--root /World/Frame/frame_structure \\\n"
          "              --out utils/collision/data/layouts/cell_env.v2_real_260917.npz")


if __name__ == "__main__":
    main()
