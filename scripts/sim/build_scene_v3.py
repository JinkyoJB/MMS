"""build_scene_v3.py — v3.usd(형상 src) 위에 sim 씬 오버라이드를 얹어 v3_scene.usd 생성.

**비파괴**: v3.usd 는 건드리지 않고 subLayer 로 깔기만 한다 (place_testset_object.py 와 같은 패턴).
STEP 이 바뀌어 step2usd.py 로 v3.usd 를 다시 만들어도 이 스크립트만 재실행하면 씬이 복원된다.

하는 일
  1. xArm6 링크 7개 비활성화 — CAD 는 납품용 xArm6 이지만 실장비/코드는 xArm7 이다.
  2. xArm7(관절·조인트 포함)을 천장 마운트 위치에 배치. v2.usd 의 /World/xarm7 을
     xarm7.usd 로 추출해 reference (v2.usd 경로 의존을 끊는다).
  3. 저울 본체를 받침판 위로 내려 하부 공간에 매립.

좌표 주의: /World/frame 에 rotateX(+90°) 가 걸려 있다(STEP 이 Y-up). 따라서
**월드 Z 로 내리려면 frame-로컬 Y 를 줄여야 한다** — 로컬 Z 가 아니다.
"""
from __future__ import annotations

import argparse
import numpy as np
import os

from pxr import Usd, UsdGeom, Gf, Sdf

# ── 배치 상수 (실측 기반, 필요하면 여기만 고치면 된다) ──────────────────────
XARM6_PRIMS = [f"p_1305Xarm6_P0{i}" for i in range(2, 8)] + ["p_1305Xarm6_P01_95"]

# xarm7_ceiling_mount 실측: X 0.295~0.435(중심 0.365), Y ±0.070(중심 0), 하면 Z=1.500
XARM7_POS = Gf.Vec3d(0.365, 0.0, 1.500)
# v2.usd 의 xarm7 루트 회전 = diag(-1, 1, -1) (Y축 180°, 천장에 매달린 자세)
XARM7_ROT = ((-1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, -1.0))

# 저울: 본체 하면 0.510 → 받침판(scale_plate) 상면 0.395 로 내림
SCALE_PRIM = "scale_body"
SCALE_DROP = 0.115

# ── EE 툴 스택 ─────────────────────────────────────────────────────────────
# STEP 에서는 이 3개가 xArm6 EE 에 물려 있어, xArm6 를 끄면 허공에 남는다.
# link7 하위로 옮기되 **상호 상대자세는 CAD 그대로 보존**(볼트 결합=강체)하고,
# 그룹 전체만 link7 플랜지에 정렬한다.
# 순서 = 플랜지에서 바깥쪽으로. CAD 최근접거리 실측: 브래킷↔마스터 0.00mm,
# 마스터↔TOOL_PLATE_04 0.00mm, TOOL_PLATE_04↔어댑터 0.10mm → 상호자세 보존만 하면 밀착된다.
# (TOOL_PLATE 는 4개 중 _04 만 EE 에 물려 있고 나머지 3개는 툴 스탠드 거치분이다.)
TOOL_PARTS = [
    "p_5K_TCC1_MOUNTING_FLANGE_BRACKET_20240724_AllCATPart",  # 플랜지 브래킷
    "p_5K_TCC1_MASTER_20240724",                              # 툴체인저 마스터
    "p_5K_TCC1_TOOL_PLATE_20240724_04",                       # 툴체인저 툴측 플레이트
    "spider_Adapter_blue",                                    # 스캐너 어댑터
]
# 앵커 = 브래킷의 로봇쪽 면(bbox 의 +Z 면) 중심. CAD 에서 툴은 여기서 −Z 로 뻗는다.
ANCHOR_PART = TOOL_PARTS[0]
SPIDER_PRIM = "/World/xarm7/link7/Artec_Space_Spider_mm"

# 어댑터부터 앞쪽(스캐너 포함)만 툴축 둘레로 돌린다. 툴체인저는 이산 각도로
# 재장착 가능하므로 물리적으로 유효한 자유도다. 180° 면 스캐너가 하늘 대신 바닥을 본다.
SCANNER_GROUP = ["spider_Adapter_blue"]      # + tool/spider (아래에서 같이 처리)

# Spider 메시의 최종 미세조정 — GUI 에서 손으로 맞춘 값(2026-08-12).
# STEP 에 Spider 본체가 없어 어댑터 장착면을 자동으로 못 찾으므로 이 값이 기준이다.
# 경로는 tool/spider 아래의 **메시 prim**(mm 단위, 부모가 0.001 스케일).
SPIDER_MESH = "Artec_Space_Spider_mm/Space_Spider_Color_2m_mm"
SPIDER_MESH_XFORM = dict(
    translate=(320.50281930124845, 254.82455964522853, 56.92635443937763),
    orient=(0.4905282, 0.038630307, 0.01158598, -0.87049156),   # (w, x, y, z)
    scale=(1.0, 1.0, 1.0),
)

# Camera 위치 — 스캐너 프레임(spider) 로컬, m. GUI 에서 손으로 맞춘 값(2026-08-12).
# 회전(광축)은 하드코딩하지 않고 v2.usd 에서 '메시 기준 상대자세'로 읽어온다
# (실기 캘리브 값이라 그쪽이 근거). 여기서 고정하는 건 **위치뿐**.
CAMERA_LOCAL_T = (0.0005422895257581883,
                  -0.036021616681152446,
                  0.08052594173638061)

# ── 환경 (조명·바닥) ───────────────────────────────────────────────────────
# v3.usd 는 형상만 있어 라이트가 없다 → 렌더가 새까맣게 나온다.
# 바닥 높이는 캐스터 바퀴 하단(Z=-0.071) 에 맞춘다. CAD 의 'floor' 파트는
# 카트 자체 베이스판(Z -0.005~0)이라 방 바닥이 아니다.
GROUND_Z = -0.071
GROUND_HALF = 4.0        # 8m × 8m

# ── 물리 ───────────────────────────────────────────────────────────────────
# 셀은 고정 설비이므로 프레임 파트는 전부 **static collider**(RigidBody 없음).
#
# ⚠ 턴테이블에 RevoluteJoint 를 넣지 말 것. isaac_turntable.py 가 명시하듯 v2.usd 의
#   조인트는 앵커가 disc 의 authored 위치와 어긋나 reset 시 disc 가 ~0.3m 튕겨나갔고,
#   그래서 isaac_world._prepare_turntable() 이 {DISC_PRIM}/RevoluteJoint 를 찾아
#   jointEnabled=False 로 끈다. 회전은 IsaacTurntable 이 disc prim 을 **kinematic 으로
#   직접** 돌린다 → 여기서는 disc 를 kinematic rigid body 로만 만들어 둔다.
#   (disc prim 원점 = (0.365, 0, 0.657) 로 회전축 위 → prim_world_pose 가 그대로 중심)
TURNTABLE_DISC = "turntable_disc"
# 턴테이블 뭉치 선택 기준 — 축에서 이 반경 안 + 이 Z 밴드에 있는 /World/frame 파트.
# (원판·베이스·모터·드라이버·PCB·베어링 등을 이름 나열 없이 한꺼번에 집는다)
TT_GROUP_RADIUS = 0.16
TT_GROUP_Z = (0.50, 0.70)
# 시각용 소품 — 충돌 불필요(keyboard 는 22k면이라 쿠킹만 비싸다)
COLLIDER_SKIP = ("keyboard", "Cordless_Mouse")

# 스캔 대상 기본값 — testset 은 m·Z-up·defaultPrim=root 라 v3_scene 규약과 같다.
TESTSET_DEFAULT = ("/home/keti/isaacsim/standalone_examples/play/MMS/testset/"
                   "0146_mug.usd")


def set_matrix(prim, M: Gf.Matrix4d) -> None:
    """xformOp:transform 값을 오버라이드로 기록.

    ⚠ AddTransformOp() 는 못 쓴다 — subLayer 쪽 prim 이 이미 xformOpOrder 에
    xformOp:transform 을 갖고 있어 'already exists' 로 실패한다. 속성값만 덮어쓴다.

    ⚠ xformOpOrder 는 **항상** 덮어써야 한다. reference 로 들어온 prim(v2.usd 의 xarm7)은
    translate/orient/scale 순서를 이미 갖고 있고 그게 authored 로 잡히므로, 조건부로 두면
    우리가 쓴 xformOp:transform 이 order 에 없어 통째로 무시된다(배치가 원본에 머문다).
    """
    attr = prim.CreateAttribute("xformOp:transform", Sdf.ValueTypeNames.Matrix4d)
    attr.Set(M)
    prim.CreateAttribute("xformOpOrder", Sdf.ValueTypeNames.TokenArray).Set(
        ["xformOp:transform"])


def world_bbox(prim, xc):
    """prim 하위 메시의 월드 tight AABB.

    ⚠ v2.usd 로봇 링크는 instanceable 이라 기본 순회로는 메시가 안 잡힌다 →
       TraverseInstanceProxies 필수.
    """
    lo, hi, n = [1e18] * 3, [-1e18] * 3, 0
    pred = Usd.TraverseInstanceProxies(Usd.PrimAllPrimsPredicate)
    for d in Usd.PrimRange(prim, pred):
        if not d.IsA(UsdGeom.Mesh):
            continue
        M = xc.GetLocalToWorldTransform(d)
        for p in (UsdGeom.Mesh(d).GetPointsAttr().Get() or []):
            w = M.Transform(Gf.Vec3d(p[0], p[1], p[2]))
            for k in range(3):
                lo[k] = min(lo[k], w[k]); hi[k] = max(hi[k], w[k])
            n += 1
    return (lo, hi, n) if n else (None, None, 0)


def mesh_world(prim, xc):
    """prim 하위 메시를 월드 좌표 (points, triangles) 로 모은다."""
    import numpy as np
    pred = Usd.TraverseInstanceProxies(Usd.PrimAllPrimsPredicate)
    P, F = [], []
    for p in Usd.PrimRange(prim, pred):
        if not p.IsA(UsdGeom.Mesh):
            continue
        m = UsdGeom.Mesh(p)
        pts, idx, cnt = (m.GetPointsAttr().Get(), m.GetFaceVertexIndicesAttr().Get(),
                         m.GetFaceVertexCountsAttr().Get())
        if not pts or not idx:
            continue
        M = xc.GetLocalToWorldTransform(p)
        base = len(P)
        for q in pts:
            w = M.Transform(Gf.Vec3d(q[0], q[1], q[2]))
            P.append([w[0], w[1], w[2]])
        o = 0
        for c in cnt:
            for k in range(1, c - 1):
                F.append([base + idx[o], base + idx[o + k], base + idx[o + k + 1]])
            o += c
    return np.array(P), np.array(F, dtype=int)


def mating_frame(prim, away_point, xc):
    """장착면(가장 큰 평면)을 찾아 mating 프레임을 만든다.

    ⚠ bbox 로 추정하면 안 된다 — 브래킷은 77×77×6mm 판인데 CAD 월드에서는 기울어져
      bbox 가 58×76×58 로 나온다. 그래서 실제 삼각형 법선으로 평면을 검출한다.

    반환: (origin, z_axis) — z_axis 는 **툴이 뻗는 방향**(로봇 반대쪽, away_point 쪽).
    """
    import numpy as np
    P, F = mesh_world(prim, xc)
    v0, v1, v2 = P[F[:, 0]], P[F[:, 1]], P[F[:, 2]]
    n = np.cross(v1 - v0, v2 - v0)
    a = np.linalg.norm(n, axis=1)
    k = a > 1e-12
    n, a, v0 = n[k] / a[k, None], a[k] / 2.0, v0[k]
    cen = (P[F[:, 0]][k] + P[F[:, 1]][k] + P[F[:, 2]][k]) / 3.0
    d = np.einsum("ij,ij->i", n, v0)
    best = None
    for i in np.argsort(-a)[:400]:
        m = (n @ n[i] > 0.999) & (np.abs(d - d[i]) < 5e-4)
        A = a[m].sum()
        if best is None or A > best[0]:
            best = (A, n[i], (cen[m] * a[m, None]).sum(0) / A)
    A, nv, c = best
    # 툴이 뻗는 쪽(=다른 툴 파트가 있는 쪽)을 +Z 로 잡는다.
    if np.dot(nv, np.asarray(away_point) - c) < 0:
        nv = -nv
    return c, nv, A


def frame_from_z(origin, z, clock_deg):
    """원점 + Z축 + clocking 으로 직교 프레임(행벡터 규약 Matrix4d) 생성."""
    import numpy as np
    z = np.asarray(z, float); z /= np.linalg.norm(z)
    ref = np.array([1.0, 0.0, 0.0]) if abs(z[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
    x = np.cross(ref, z); x /= np.linalg.norm(x)
    y = np.cross(z, x)
    t = np.radians(clock_deg)
    x2 = x * np.cos(t) + y * np.sin(t)
    y2 = np.cross(z, x2)
    return Gf.Matrix4d(x2[0], x2[1], x2[2], 0.0,
                       y2[0], y2[1], y2[2], 0.0,
                       z[0], z[1], z[2], 0.0,
                       origin[0], origin[1], origin[2], 1.0)


def attach_tool(stage, src_dir: str, src_usd: str, v2_rel: str, clock_deg: float,
                spider_off: float, scanner_clock: float = 180.0) -> None:
    """CAD 툴 스택을 link7 하위로 옮기고 Spider 를 스택 끝으로 밀어낸다."""
    src = Usd.Stage.Open(os.path.join(src_dir, src_usd))
    sxc = UsdGeom.XformCache()

    # 앵커 프레임 F_cad: 원점=브래킷 +Z 면 중심, Z축=툴 진행방향(월드 −Z), X축=월드 +X
    ap = src.GetPrimAtPath(f"/World/frame/{ANCHOR_PART}")
    if not (ap and ap.IsValid()):
        print(f"  ⚠ 앵커 파트 없음: {ANCHOR_PART}"); return
    # 툴이 뻗는 방향 판정용 기준점 = 스택 반대편 파트(어댑터)의 bbox 중심
    tp = src.GetPrimAtPath(f"/World/frame/{TOOL_PARTS[-1]}")
    tlo, thi, _ = world_bbox(tp, sxc)
    away = [(tlo[k] + thi[k]) / 2 for k in range(3)]
    org, zax, area = mating_frame(ap, away, sxc)
    F = frame_from_z(org, zax, clock_deg)
    print(f"  앵커=브래킷 장착면  중심({org[0]:.4f},{org[1]:.4f},{org[2]:.4f})  "
          f"툴축({zax[0]:.3f},{zax[1]:.3f},{zax[2]:.3f})  면적{area*1e6:.0f}mm²")

    # 조정 포인트를 **딱 2개**로 만든다 (GUI 에서 손으로 맞추기 쉽도록):
    #   /World/xarm7/link7/tool         ← 스택 전체 vs EE 플랜지
    #   /World/xarm7/link7/tool/spider  ← 스캐너 vs 어댑터
    # 브래킷·마스터·어댑터는 CAD 상호자세 그대로라 이미 서로 맞물려 있다(건드릴 필요 없음).
    Rz = Gf.Matrix4d()   # 스택 전체 clocking 은 F(앵커 프레임) 안에 이미 반영됨
    # 어댑터 이후만 툴축 둘레로 재장착 (tool 프레임 = link7 프레임 기준)
    Rs = Gf.Matrix4d().SetRotate(Gf.Rotation(Gf.Vec3d(0, 0, 1), scanner_clock))
    Finv = F.GetInverse()
    grp = UsdGeom.Xform.Define(stage, "/World/xarm7/link7/tool")
    set_matrix(grp.GetPrim(), Rz)

    for name in TOOL_PARTS:
        p = src.GetPrimAtPath(f"/World/frame/{name}")
        if not (p and p.IsValid()):
            print(f"  ⚠ 없음: {name}"); continue
        rel = sxc.GetLocalToWorldTransform(p) * Finv      # F_cad 기준 상대자세
        # 원본이 물고 있는 ./parts/<slug>.usd 를 그대로 재사용 (지오메트리 중복 방지).
        # ⚠ prependedItems 는 ListProxy 라 list() 로 감싸야 이어붙일 수 있다.
        slug = None
        for pr in p.GetPrimStack():
            for r in list(pr.referenceList.prependedItems) + \
                     list(pr.referenceList.explicitItems):
                slug = r.assetPath
        dst = UsdGeom.Xform.Define(stage, f"/World/xarm7/link7/tool/{name}")
        if slug:
            dst.GetPrim().GetReferences().AddReference(slug)
        # 어댑터 이후는 툴축 둘레 재장착 각도(scanner_clock)를 반영.
        # 부모(tool) 프레임에서 곱하므로 어댑터·Spider 가 **함께** 강체 회전한다.
        m = rel * Rs if name in SCANNER_GROUP else rel
        set_matrix(dst.GetPrim(), m)            # 그 외는 CAD 상호자세 그대로 (고정)
        stage.OverridePrim(f"/World/frame/{name}").SetActive(False)
        print(f"  {name[:46]:<46} → link7/tool")

    # Spider: v2 의 플랜지 직결본은 끄고, tool 그룹 아래로 새로 건다.
    sp = stage.GetPrimAtPath(SPIDER_PRIM)
    if not (sp and sp.IsValid()):
        print(f"  ⚠ Spider prim 없음: {SPIDER_PRIM}"); return
    M_sp = UsdGeom.Xformable(sp).GetLocalTransformation()   # link7 기준 (0.001 스케일 포함)
    stage.OverridePrim(SPIDER_PRIM).SetActive(False)
    new_sp = UsdGeom.Xform.Define(stage, "/World/xarm7/link7/tool/spider")
    new_sp.GetPrim().GetReferences().AddReference(v2_rel, SPIDER_PRIM)
    # tool 그룹 기준으로 환산 + 스택 길이만큼 밀어냄
    set_matrix(new_sp.GetPrim(),
               M_sp * Rz.GetInverse() * Gf.Matrix4d().SetTranslate(
                   Gf.Vec3d(0, 0, spider_off)) * Rs)
    print(f"  Spider → link7/tool/spider  (Z +{spider_off*1000:.0f}mm, "
          f"툴축 {scanner_clock:.0f}° 재장착)")

    # GUI 에서 손으로 맞춘 Spider 메시 자세를 재현 (재생성해도 유지되도록 고정값)
    mp = stage.OverridePrim(f"/World/xarm7/link7/tool/spider/{SPIDER_MESH}")
    t = SPIDER_MESH_XFORM
    mp.CreateAttribute("xformOp:translate", Sdf.ValueTypeNames.Double3).Set(
        Gf.Vec3d(*t["translate"]))
    mp.CreateAttribute("xformOp:orient", Sdf.ValueTypeNames.Quatf).Set(
        Gf.Quatf(t["orient"][0], Gf.Vec3f(*t["orient"][1:])))
    mp.CreateAttribute("xformOp:scale", Sdf.ValueTypeNames.Float3).Set(
        Gf.Vec3f(*t["scale"]))
    mp.CreateAttribute("xformOpOrder", Sdf.ValueTypeNames.TokenArray).Set(
        ["xformOp:translate", "xformOp:orient", "xformOp:scale"])
    print(f"  Spider 메시 수동조정 반영 t={tuple(round(v,2) for v in t['translate'])}mm")


def consolidate_spider(stage, v2_abs: str, cam_t=CAMERA_LOCAL_T) -> None:
    """spider / Artec_Space_Spider_mm / mesh 프레임을 **메시 기준으로 통일**하고 Camera 재배치.

    원래 구조는 자세가 3단으로 쪼개져 있어(spider 에 0.001 스케일+오프셋, 메시에 수동조정)
    "스캐너 프레임" 이 의미가 없었고, Camera 는 v2 값에 그대로 남아 메시를 안 따라갔다.

    통일 후:
        tool/spider                  강체 프레임(scale 1, m) = 메시의 실제 자세
          ├ Camera                   CAMERA_LOCAL_T (GUI 에서 맞춘 값), 광축은 v2 계승
          └ Artec_Space_Spider_mm    scale 0.001 만 (mm 지오메트리 보정)
              └ Space_Spider_Color_2m_mm   identity
    """
    sp_path = "/World/xarm7/link7/tool/spider"
    xc = UsdGeom.XformCache()
    tool = stage.GetPrimAtPath("/World/xarm7/link7/tool")
    mesh = stage.GetPrimAtPath(f"{sp_path}/{SPIDER_MESH}")
    if not (mesh and mesh.IsValid()):
        print("  ⚠ Spider 메시 없음 — 통일 생략"); return

    M_rel = xc.GetLocalToWorldTransform(mesh) * \
        xc.GetLocalToWorldTransform(tool).GetInverse()
    rows = [np.array([M_rel[r][c] for c in range(3)]) for r in range(3)]
    s = np.array([np.linalg.norm(r) for r in rows])
    R = np.array([rows[i] / s[i] for i in range(3)])
    t = [M_rel[3][k] for k in range(3)]

    # spider = 강체(scale 1)
    set_matrix(stage.GetPrimAtPath(sp_path), Gf.Matrix4d(
        R[0][0], R[0][1], R[0][2], 0.0, R[1][0], R[1][1], R[1][2], 0.0,
        R[2][0], R[2][1], R[2][2], 0.0, t[0], t[1], t[2], 1.0))
    # Artec_Space_Spider_mm = 스케일만,  메시 = identity
    set_matrix(stage.OverridePrim(f"{sp_path}/Artec_Space_Spider_mm"),
               Gf.Matrix4d(s[0], 0, 0, 0, 0, s[1], 0, 0, 0, 0, s[2], 0, 0, 0, 0, 1))
    set_matrix(stage.OverridePrim(f"{sp_path}/{SPIDER_MESH}"), Gf.Matrix4d(1))
    print(f"  Spider 프레임 통일: 원점 {tuple(round(v,5) for v in t)}, "
          f"스케일 {np.round(s, 5).tolist()} 는 mm 노드로 분리")

    # Camera 광축 = v2 에서 메시 프레임 기준으로 읽어온다(실기 캘리브 값).
    v2 = Usd.Stage.Open(v2_abs)
    vxc = UsdGeom.XformCache()
    base = "/World/xarm7/link7/Artec_Space_Spider_mm"
    Mc = vxc.GetLocalToWorldTransform(v2.GetPrimAtPath(f"{base}/Camera"))
    Mm = vxc.GetLocalToWorldTransform(
        v2.GetPrimAtPath(f"{base}/Artec_Space_Spider_mm/Space_Spider_Color_2m_mm"))
    rel = Mc * Mm.GetInverse()
    cr = [np.array([rel[r][c] for c in range(3)]) for r in range(3)]
    cr = [v / np.linalg.norm(v) for v in cr]

    cam = stage.OverridePrim(f"{sp_path}/Camera")
    set_matrix(cam, Gf.Matrix4d(
        cr[0][0], cr[0][1], cr[0][2], 0.0, cr[1][0], cr[1][1], cr[1][2], 0.0,
        cr[2][0], cr[2][1], cr[2][2], 0.0, cam_t[0], cam_t[1], cam_t[2], 1.0))
    # 부모가 이제 m 단위(scale 1) → 클리핑도 m 로 직접 해석된다.
    cam.CreateAttribute("clippingRange", Sdf.ValueTypeNames.Float2).Set(
        Gf.Vec2f(0.01, 0.5))
    d = float(np.linalg.norm(cam_t)) * 1000
    print(f"  Camera: 로컬 {tuple(round(v, 5) for v in cam_t)} m ({d:.2f}mm), "
          f"광축 {np.round(-cr[2], 4).tolist()} (스캐너 −Y), clip (0.01, 0.5)m")


def add_physics(stage, approx: str = "convexHull") -> None:
    """프레임 파트에 static collider, 턴테이블 disc 에 kinematic rigid body."""
    from pxr import UsdPhysics
    frame = stage.GetPrimAtPath("/World/frame")
    n_col = n_skip = 0
    for c in frame.GetChildren():
        name = c.GetName()
        if not c.IsActive():
            continue                      # xArm6·툴 원본 등 비활성분은 제외
        if any(s in name for s in COLLIDER_SKIP):
            n_skip += 1
            continue
        # 실제 지오메트리는 참조된 파트의 'mesh' 자식 → 거기에 collider 를 건다.
        m = stage.GetPrimAtPath(f"/World/frame/{name}/mesh")
        if not (m and m.IsValid()):
            continue
        UsdPhysics.CollisionAPI.Apply(m)
        UsdPhysics.MeshCollisionAPI.Apply(m).CreateApproximationAttr().Set(approx)
        n_col += 1
    print(f"  충돌: static collider {n_col}개 ({approx}), 소품 {n_skip}개 제외")

    disc = stage.GetPrimAtPath(f"/World/frame/{TURNTABLE_DISC}")
    if disc and disc.IsValid():
        rb = UsdPhysics.RigidBodyAPI.Apply(disc)
        rb.CreateKinematicEnabledAttr().Set(True)
        print(f"  턴테이블: {TURNTABLE_DISC} = kinematic rigid body "
              f"(RevoluteJoint 없음 — IsaacTurntable 이 직접 회전)")
    else:
        print(f"  ⚠ {TURNTABLE_DISC} 없음")


def shift_turntable(stage, dx: float) -> None:
    """턴테이블 뭉치를 월드 X 로 dx 만큼 이동 (설계 검토용).

    ⚠ /World/frame 에 rotateX(+90°) 가 걸려 있지만 **X 는 월드와 프레임-로컬이 일치**
      하므로 로컬 X 에 그대로 더하면 된다(Z/Y 는 다르다 — 저울 하강 참고).
    """
    if abs(dx) < 1e-9:
        return
    xc = UsdGeom.XformCache()
    disc = stage.GetPrimAtPath(f"/World/frame/{TURNTABLE_DISC}")
    dlo, dhi, _ = world_bbox(disc, xc)
    cx, cy = (dlo[0] + dhi[0]) / 2, (dlo[1] + dhi[1]) / 2
    moved = []
    for c in stage.GetPrimAtPath("/World/frame").GetChildren():
        if not c.IsActive():
            continue
        lo, hi, n = world_bbox(c, xc)
        if not n:
            continue
        ctr = [(lo[k] + hi[k]) / 2 for k in range(3)]
        if not (TT_GROUP_Z[0] <= ctr[2] <= TT_GROUP_Z[1]):
            continue
        if np.hypot(ctr[0] - cx, ctr[1] - cy) > TT_GROUP_RADIUS:
            continue
        M = UsdGeom.Xformable(c).GetLocalTransformation()
        rows = [[M[r][k] for k in range(4)] for r in range(4)]
        rows[3][0] += dx                       # 프레임-로컬 X == 월드 X
        set_matrix(c, Gf.Matrix4d(*[v for row in rows for v in row]))
        moved.append(c.GetName())
    print(f"  턴테이블 이동: 축 X {cx:.3f} → {cx+dx:.3f} (dx={dx:+.3f}m), "
          f"파트 {len(moved)}개")


def add_scan_target(stage, obj_usd: str) -> None:
    """스캔 대상을 턴테이블 원판 위 중앙에 올린다.

    경로는 v2 규약을 유지 — /World/ScanTarget/TestObject
    (isaac_world.OBJECT_PRIM 기본값, MMS_SIM_OBJECT_PRIM 로 override 가능).
    ⚠ /World/frame 아래가 아니라 **월드 직속**이다. frame 은 rotateX(90) 이 걸려 있어
      그 아래에 두면 배치 계산이 헷갈린다.
    """
    xc = UsdGeom.XformCache()
    disc = stage.GetPrimAtPath(f"/World/frame/{TURNTABLE_DISC}")
    if not (disc and disc.IsValid()):
        print(f"  ⚠ {TURNTABLE_DISC} 없음 — 스캔 대상 생략"); return
    dlo, dhi, _ = world_bbox(disc, xc)
    cx, cy, top = (dlo[0] + dhi[0]) / 2, (dlo[1] + dhi[1]) / 2, dhi[2]

    UsdGeom.Xform.Define(stage, "/World/ScanTarget")
    tgt = UsdGeom.Xform.Define(stage, "/World/ScanTarget/TestObject")
    tgt.GetPrim().GetReferences().AddReference(obj_usd)
    stage.GetRootLayer().Save()          # 참조 해석 후 bbox 를 재계산하기 위해

    o = stage.GetPrimAtPath("/World/ScanTarget/TestObject")
    olo, ohi, n = world_bbox(o, UsdGeom.XformCache())
    if not n:
        print(f"  ⚠ 스캔 대상 메시 없음: {obj_usd}"); return
    t = Gf.Vec3d(cx - (olo[0] + ohi[0]) / 2,
                 cy - (olo[1] + ohi[1]) / 2,
                 top - olo[2])
    set_matrix(o, Gf.Matrix4d().SetTranslate(t))

    # ⚠ testset 객체는 RigidBodyAPI 가 **자식**(예: Smart_Fusion_1)에 붙어 있다.
    #   isaac_world._prepare_turntable() 은 OBJECT_PRIM 자신에만 kinematicEnabled 를
    #   세팅하므로 그대로 두면 자식이 dynamic 으로 남아, IsaacTurntable 의 kinematic
    #   회전과 물리가 싸워 대상이 원판에서 밀려난다(실측: 축까지 3mm→97mm).
    #   → 서브트리에서 RigidBodyAPI 를 가진 prim 을 찾아 모두 kinematic 으로 만든다.
    from pxr import UsdPhysics
    pred = Usd.TraverseInstanceProxies(Usd.PrimAllPrimsPredicate)
    n_kin = 0
    for d in Usd.PrimRange(o, pred):
        if d.HasAPI(UsdPhysics.RigidBodyAPI):
            UsdPhysics.RigidBodyAPI(d).CreateKinematicEnabledAttr().Set(True)
            n_kin += 1
    print(f"  스캔 대상: {os.path.basename(obj_usd)} → /World/ScanTarget/TestObject "
          f"(rigid body {n_kin}개 kinematic 화)")
    print(f"      원판 상면 Z={top:.4f}, 축 ({cx:.3f},{cy:.3f}) 에 정렬 "
          f"(크기 {(ohi[0]-olo[0])*1000:.0f}×{(ohi[1]-olo[1])*1000:.0f}×"
          f"{(ohi[2]-olo[2])*1000:.0f}mm)")


def add_environment(stage, ground: bool, lights: bool) -> None:
    """조명 + 바닥 추가. 둘 다 /World/Environment 아래로 모은다."""
    from pxr import UsdLux, UsdPhysics, Vt
    UsdGeom.Xform.Define(stage, "/World/Environment")

    if lights:
        dome = UsdLux.DomeLight.Define(stage, "/World/Environment/DomeLight")
        dome.CreateIntensityAttr(1000.0)
        dome.CreateColorAttr(Gf.Vec3f(1.0, 1.0, 1.0))
        sun = UsdLux.DistantLight.Define(stage, "/World/Environment/SunLight")
        sun.CreateIntensityAttr(2500.0)
        sun.CreateAngleAttr(0.8)
        sun.CreateColorAttr(Gf.Vec3f(1.0, 0.98, 0.95))
        UsdGeom.Xformable(sun.GetPrim()).AddRotateXYZOp().Set(Gf.Vec3f(-50.0, 0.0, 35.0))
        print(f"  조명: DomeLight(1000) + DistantLight(2500, -50°/35°)")

    if ground:
        h, z = GROUND_HALF, GROUND_Z
        m = UsdGeom.Mesh.Define(stage, "/World/Environment/GroundPlane")
        m.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(-h, -h, z), Gf.Vec3f(h, -h, z),
                                          Gf.Vec3f(h, h, z), Gf.Vec3f(-h, h, z)]))
        m.CreateFaceVertexCountsAttr(Vt.IntArray([4]))
        m.CreateFaceVertexIndicesAttr(Vt.IntArray([0, 1, 2, 3]))
        m.CreateNormalsAttr(Vt.Vec3fArray([Gf.Vec3f(0, 0, 1)] * 4))
        m.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
        m.CreateDisplayColorAttr(Vt.Vec3fArray([Gf.Vec3f(0.35, 0.36, 0.38)]))
        m.CreateExtentAttr(Vt.Vec3fArray([Gf.Vec3f(-h, -h, z), Gf.Vec3f(h, h, z)]))
        # 물리: 정적 콜라이더 (카트 캐스터가 여기 닿는다)
        UsdPhysics.CollisionAPI.Apply(m.GetPrim())
        print(f"  바닥: {2*h:.0f}m×{2*h:.0f}m @ Z={z:.3f} (캐스터 하단), 콜라이더 적용")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir", required=True, help="v3.usd 가 있는 디렉토리")
    ap.add_argument("--src", default="v3.usd")
    ap.add_argument("--out", default="v3_scene.usd")
    ap.add_argument("--v2", default=None,
                    help="xarm7 추출용 v2.usd (기본: ../frame_xarm7_spider_turntable/v2.usd)")
    ap.add_argument("--scale-drop", type=float, default=SCALE_DROP,
                    help="저울 본체 하강량 m (기본 0.115 = 받침판 상면에 안착)")
    ap.add_argument("--turntable-dx", type=float, default=0.0,
                    help="턴테이블 뭉치를 월드 X 로 이동 m (설계 검토용). "
                         "scripts/sim/sweep_turntable_x.py 로 최적값 산출")
    ap.add_argument("--object", default=TESTSET_DEFAULT,
                    help="턴테이블에 올릴 스캔 대상 USD ('none' 이면 생략)")
    ap.add_argument("--cam-t", type=float, nargs=3, default=list(CAMERA_LOCAL_T),
                    metavar=("X", "Y", "Z"),
                    help="Camera 위치(스캐너 프레임 로컬, m). 기본=GUI 에서 맞춘 값")
    ap.add_argument("--no-physics", action="store_true", help="충돌·강체 생략")
    ap.add_argument("--collider-approx", default="convexHull",
                    choices=["convexHull", "boundingCube", "none"],
                    help="프레임 파트 콜라이더 근사 (기본 convexHull)")
    ap.add_argument("--no-lights", action="store_true", help="조명 생략")
    ap.add_argument("--no-ground", action="store_true", help="바닥 생략")
    ap.add_argument("--scanner-clock-deg", type=float, default=180.0,
                    help="어댑터+Spider 만 툴축 둘레로 회전 deg. 180 이면 스캐너가 "
                         "하늘이 아니라 바닥을 본다 (툴체인저 재장착 각도)")
    ap.add_argument("--clock-deg", type=float, default=0.0,
                    help="툴 스택의 툴축(link7 Z) 둘레 회전 deg — 스크린샷 보고 조정")
    ap.add_argument("--spider-offset", type=float, default=0.096,
                    help="Spider 를 link7 Z 로 밀어내는 양 m. 기본 0.096 = 어댑터와 접촉하는 실측 해 "
                         "(STEP 에 Spider 본체가 없어 장착면을 못 찾음 → 접촉거리로 근사)")
    args = ap.parse_args()

    d = os.path.abspath(args.dir)
    out_path = os.path.join(d, args.out)
    v2 = os.path.abspath(args.v2 or os.path.join(
        os.path.dirname(d), "frame_xarm7_spider_turntable", "v2.usd"))
    if not os.path.isfile(v2):
        raise SystemExit(f"v2.usd 없음: {v2}")
    # ⚠ /World/xarm7 서브트리만 CopySpec 으로 떼오면 안 된다 — 메시가 v2.usd **루트**의
    #   /Flattened_Prototype_* 를 internal reference 로 물고 있어 전부 미해결이 된다.
    #   v2.usd 를 통째로 참조하고 prim path 로 xarm7 만 집는다.
    v2_rel = os.path.relpath(v2, d)
    print(f"[1/4] xArm7 소스: {v2_rel}  (prim /World/xarm7)")

    print(f"[2/4] 씬 레이어 생성  {out_path}")
    if os.path.exists(out_path):
        os.remove(out_path)
    stage = Usd.Stage.CreateNew(out_path)
    stage.GetRootLayer().subLayerPaths.append(f"./{args.src}")
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    stage.SetDefaultPrim(stage.OverridePrim("/World"))

    print("[3/4] 오버라이드")
    # (1) xArm6 비활성화
    off = 0
    for name in XARM6_PRIMS:
        p = stage.GetPrimAtPath(f"/World/frame/{name}")
        if not p or not p.IsValid():
            print(f"  ⚠ 없음: {name}")
            continue
        stage.OverridePrim(f"/World/frame/{name}").SetActive(False)
        off += 1
    print(f"  xArm6 링크 비활성화 {off}개")

    # (2) xArm7 배치
    xf = UsdGeom.Xform.Define(stage, "/World/xarm7")
    xf.GetPrim().GetReferences().AddReference(v2_rel, "/World/xarm7")
    R = XARM7_ROT
    set_matrix(xf.GetPrim(), Gf.Matrix4d(
        R[0][0], R[0][1], R[0][2], 0.0,
        R[1][0], R[1][1], R[1][2], 0.0,
        R[2][0], R[2][1], R[2][2], 0.0,
        XARM7_POS[0], XARM7_POS[1], XARM7_POS[2], 1.0))
    print(f"  xArm7 배치 {tuple(round(v,3) for v in XARM7_POS)}")

    # ⚠ transform 만 바꾸면 **Play 순간 v2 위치로 튕긴다**.
    #   root_joint 는 PhysicsFixedJoint 이고 body0=[](월드), body1=link_base 이며
    #   구속은  T_world_link_base · (localPos1,localRot1) = (localPos0,localRot0)=identity.
    #   즉 localPos1/localRot1 이 링크베이스의 월드 자세를 **물리적으로** 결정한다.
    #   → 새 배치의 역변환으로 갱신해야 렌더 자세와 물리 자세가 일치한다.
    M_root = Gf.Matrix4d(
        R[0][0], R[0][1], R[0][2], 0.0,
        R[1][0], R[1][1], R[1][2], 0.0,
        R[2][0], R[2][1], R[2][2], 0.0,
        XARM7_POS[0], XARM7_POS[1], XARM7_POS[2], 1.0)
    L1 = M_root.GetInverse()
    rj = stage.OverridePrim("/World/xarm7/joints/root_joint")
    q = L1.ExtractRotationQuat()
    rj.CreateAttribute("physics:localPos1", Sdf.ValueTypeNames.Point3f).Set(
        Gf.Vec3f(L1[3][0], L1[3][1], L1[3][2]))
    rj.CreateAttribute("physics:localRot1", Sdf.ValueTypeNames.Quatf).Set(
        Gf.Quatf(float(q.GetReal()), Gf.Vec3f(*[float(v) for v in q.GetImaginary()])))
    print(f"  root_joint 앵커 갱신 localPos1="
          f"{tuple(round(L1[3][k],4) for k in range(3))} (Play 시 튕김 방지)")

    # (3) 저울 내리기 — frame 이 rotateX(90) 이므로 월드 Z↓ = 로컬 Y↓
    sp = stage.GetPrimAtPath(f"/World/frame/{SCALE_PRIM}")
    if sp and sp.IsValid():
        M = UsdGeom.Xformable(sp).GetLocalTransformation()
        rows = [[M[r][c] for c in range(4)] for r in range(4)]
        rows[3][1] -= args.scale_drop
        set_matrix(stage.OverridePrim(f"/World/frame/{SCALE_PRIM}"),
                   Gf.Matrix4d(*[v for row in rows for v in row]))
        print(f"  {SCALE_PRIM} 월드 Z −{args.scale_drop*1000:.0f}mm "
              f"(frame-로컬 Y 축으로 이동)")
    else:
        print(f"  ⚠ {SCALE_PRIM} 없음")

    # (4) EE 툴 스택 부착
    attach_tool(stage, d, args.src, v2_rel, args.clock_deg, args.spider_offset,
                args.scanner_clock_deg)

    # (5) Spider 프레임 통일 + Camera 재배치
    consolidate_spider(stage, v2, tuple(args.cam_t))

    # (5.5) 턴테이블 위치 조정 — 스캔 대상 배치보다 **먼저** (대상이 원판 위에 얹히도록)
    shift_turntable(stage, args.turntable_dx)

    # (6) 스캔 대상
    if args.object and args.object.lower() != "none":
        add_scan_target(stage, os.path.abspath(args.object))

    # (7) 환경 — 조명·바닥
    add_environment(stage, ground=not args.no_ground, lights=not args.no_lights)

    # (8) 물리 — 충돌·턴테이블
    if not args.no_physics:
        add_physics(stage, args.collider_approx)

    stage.GetRootLayer().Save()
    print(f"[4/4] 저장 완료 → {out_path}")


if __name__ == "__main__":
    main()
