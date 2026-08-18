"""step2usd.py — STEP 어셈블리를 파트 단위 USD 로 변환.

Onshape OBJ export 는 B-rep face 단위로 메시를 쪼개서(43,423개) 파트 구조가 사라진다.
여기서는 STEP 을 XDE 로 직접 읽어 **파트당 메시 1개**로 병합한다.

출력
    <out>/parts/<slug>.usd   고유 파트 1개당 1파일 (로컬 좌표, defaultPrim=/<slug>)
    <out>/<name>.usd         조립 — 인스턴스마다 reference + xformOp:transform

규약: metersPerUnit=1.0(m), Z-up (v2.usd 와 동일). STEP 은 mm → ×0.001.
같은 파트가 N번 쓰이면 USD 는 1개만 만들고 N번 reference 한다.
"""
from __future__ import annotations

import argparse
import json
import os
import re
import time
import unicodedata

from OCC.Core.STEPCAFControl import STEPCAFControl_Reader
from OCC.Core.TDocStd import TDocStd_Document
from OCC.Core.XCAFDoc import XCAFDoc_DocumentTool
from OCC.Core.TDF import TDF_LabelSequence, TDF_Label, TDF_Tool
from OCC.Core.TCollection import TCollection_AsciiString
from OCC.Core.TopLoc import TopLoc_Location
from OCC.Core.IFSelect import IFSelect_RetDone
from OCC.Core.TopExp import TopExp_Explorer
from OCC.Core.TopAbs import TopAbs_FACE, TopAbs_REVERSED
from OCC.Core.TopoDS import topods
from OCC.Core.BRep import BRep_Tool
from OCC.Core.BRepMesh import BRepMesh_IncrementalMesh
from OCC.Core.Bnd import Bnd_Box
from OCC.Core.BRepBndLib import brepbndlib

from pxr import Usd, UsdGeom, Gf, Vt, Sdf

MM_TO_M = 0.001

# ── 체결류 필터 ────────────────────────────────────────────────────────────
# ⚠ 'bolt' 를 단순 포함검사하면 "Femto bolt"(Orbbec 카메라)가 걸린다 → 장비명 우선 제외.
DEVICE_KEEP = re.compile(r"femto|helios|realsense|kinect|spider|phoxi|motioncam", re.I)
# ⚠ \b 는 못 쓴다: 'screw_ks_KS…' 에서 '_' 가 단어문자라 \bscrew\b 가 실패한다.
#   글자(A-Za-z)만 단어로 보는 lookaround 로 '_'·공백·하이픈 인접을 허용.
FASTENER = re.compile(
    r"(?<![A-Za-z])(screw|washer|nut|bolt|rivet|dowel)(?![A-Za-z])|"
    r"볼트|너트|와셔|와샤|나사|스크류", re.I)

# ── 한글 파트명 → ASCII slug ──────────────────────────────────────────────
KO = {
    "턴테이블": "turntable", "디스크": "disc", "베이스": "base", "모터빔": "motorbeam",
    "툴체인저": "toolchanger", "툴스탠드": "toolstand", "더미": "dummy",
    "저울": "scale", "본체": "body", "벽": "wall", "바닥": "floor",
    "파트": "part", "멀티모달스캔시스템": "mms", "레이아웃": "layout",
    "카메라": "camera", "프레임": "frame", "덮개": "cover", "받침": "support",
}


def slugify(name: str) -> str:
    """USD prim 이름으로 쓸 ASCII slug. 한글은 사전 치환 후 잔여분 제거."""
    s = name.split("^")[0]                      # 'X^서브어셈' → 'X'
    for ko, en in sorted(KO.items(), key=lambda kv: -len(kv[0])):
        s = s.replace(ko, "_" + en + "_")
    s = unicodedata.normalize("NFKD", s)
    s = re.sub(r"[^A-Za-z0-9_]+", "_", s)       # 남은 한글/기호 → _
    s = re.sub(r"_+", "_", s).strip("_")
    if not s or s[0].isdigit():
        s = "p_" + s
    return s


def trsf_to_matrix4d(trsf, scale: float) -> Gf.Matrix4d:
    """gp_Trsf → USD Gf.Matrix4d.

    USD 는 행벡터 규약(p' = p · M)이라 회전부를 **전치**해서 넣고
    이동은 마지막 행에 둔다.
    """
    return Gf.Matrix4d(
        trsf.Value(1, 1), trsf.Value(2, 1), trsf.Value(3, 1), 0.0,
        trsf.Value(1, 2), trsf.Value(2, 2), trsf.Value(3, 2), 0.0,
        trsf.Value(1, 3), trsf.Value(2, 3), trsf.Value(3, 3), 0.0,
        trsf.Value(1, 4) * scale, trsf.Value(2, 4) * scale,
        trsf.Value(3, 4) * scale, 1.0,
    )


def tessellate(shape, lin_defl: float, ang_defl: float):
    """shape 의 모든 face 를 삼각분할해 **하나의** (points, indices) 로 병합."""
    BRepMesh_IncrementalMesh(shape, lin_defl, False, ang_defl, True)
    pts: list[tuple[float, float, float]] = []
    idx: list[int] = []
    exp = TopExp_Explorer(shape, TopAbs_FACE)
    while exp.More():
        face = topods.Face(exp.Current())
        loc = TopLoc_Location()
        tri = BRep_Tool.Triangulation(face, loc)
        if tri is not None:
            t = loc.Transformation()
            base = len(pts)
            for i in range(1, tri.NbNodes() + 1):
                p = tri.Node(i).Transformed(t)
                pts.append((p.X() * MM_TO_M, p.Y() * MM_TO_M, p.Z() * MM_TO_M))
            rev = face.Orientation() == TopAbs_REVERSED
            for i in range(1, tri.NbTriangles() + 1):
                a, b, c = tri.Triangle(i).Get()
                if rev:
                    a, c = c, a                  # 면 방향 뒤집힘 보정
                idx += [base + a - 1, base + b - 1, base + c - 1]
        exp.Next()
    return pts, idx


def write_part_usd(path: str, prim_name: str, pts, idx, display: str) -> None:
    stage = Usd.Stage.CreateNew(path)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    xf = UsdGeom.Xform.Define(stage, f"/{prim_name}")
    stage.SetDefaultPrim(xf.GetPrim())
    xf.GetPrim().SetMetadata("displayName", display)
    mesh = UsdGeom.Mesh.Define(stage, f"/{prim_name}/mesh")
    mesh.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(*p) for p in pts]))
    mesh.CreateFaceVertexCountsAttr(Vt.IntArray([3] * (len(idx) // 3)))
    mesh.CreateFaceVertexIndicesAttr(Vt.IntArray(idx))
    mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    ext = UsdGeom.PointBased(mesh).ComputeExtent(mesh.GetPointsAttr().Get())
    if ext:
        mesh.CreateExtentAttr(ext)
    stage.GetRootLayer().Save()


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--step", required=True)
    ap.add_argument("--out", required=True, help="출력 디렉토리")
    ap.add_argument("--name", default="v3", help="조립 USD 이름 (기본 v3)")
    ap.add_argument("--deflection", type=float, default=0.5, help="선형 편차 mm")
    ap.add_argument("--angular", type=float, default=0.5, help="각 편차 rad")
    ap.add_argument("--keep-fasteners", action="store_true")
    ap.add_argument("--source-up", choices=["y", "z"], default="y",
                    help="STEP 의 up 축. SolidWorks 전역계는 Y-up 이라 기본 y. "
                         "y 면 루트에 rotateX(+90°) 를 걸어 Z-up 으로 세운다.")
    args = ap.parse_args()

    t0 = time.time()
    parts_dir = os.path.join(args.out, "parts")
    os.makedirs(parts_dir, exist_ok=True)

    doc = TDocStd_Document("pythonocc-doc-step-import")
    shape_tool = XCAFDoc_DocumentTool.ShapeTool(doc.Main())
    rdr = STEPCAFControl_Reader()
    rdr.SetColorMode(True); rdr.SetLayerMode(True)
    rdr.SetNameMode(True); rdr.SetMatMode(True)
    print(f"[1/4] STEP 읽는 중 … {args.step}", flush=True)
    assert rdr.ReadFile(args.step) == IFSelect_RetDone, "STEP 읽기 실패"
    rdr.Transfer(doc)
    print(f"      완료 {time.time()-t0:.1f}s", flush=True)

    # ── 트리 순회 → 인스턴스 수집 ─────────────────────────────────────────
    instances = []          # (entry, name, TopoDS_Shape(로컬), gp_Trsf(월드))
    locs: list[TopLoc_Location] = []

    def entry_of(lab):
        s = TCollection_AsciiString()
        TDF_Tool.Entry(lab, s)
        return s.ToCString()

    def walk(lab):
        if shape_tool.IsAssembly(lab):
            comps = TDF_LabelSequence()
            shape_tool.GetComponents(lab, comps)
            for i in range(comps.Length()):
                c = comps.Value(i + 1)
                if shape_tool.IsReference(c):
                    ref = TDF_Label()
                    shape_tool.GetReferredShape(c, ref)
                    locs.append(shape_tool.GetLocation(c))
                    walk(ref)
                    locs.pop()
                else:
                    walk(c)
        elif shape_tool.IsSimpleShape(lab):
            shp = shape_tool.GetShape(lab)
            if shp.IsNull():
                return
            loc = TopLoc_Location()
            for l in locs:
                loc = loc.Multiplied(l)
            instances.append((entry_of(lab), lab.GetLabelName(),
                              shape_tool.GetShape(lab), loc.Transformation()))

    print("[2/4] 어셈블리 트리 순회", flush=True)
    free = TDF_LabelSequence()
    shape_tool.GetFreeShapes(free)
    for i in range(free.Length()):
        walk(free.Value(i + 1))
    print(f"      리프 인스턴스 {len(instances)}개", flush=True)

    # ── 필터 ──────────────────────────────────────────────────────────────
    kept, dropped = [], []
    for inst in instances:
        nm = inst[1]
        is_fast = bool(FASTENER.search(nm)) and not DEVICE_KEEP.search(nm)
        (dropped if (is_fast and not args.keep_fasteners) else kept).append(inst)
    print(f"      체결류 제외 {len(dropped)}개 → 변환 대상 {len(kept)}개", flush=True)

    # ── 고유 파트별 테셀레이션 (인스턴스는 reference 재사용) ───────────────
    uniq: dict[str, dict] = {}
    for entry, name, shp, _ in kept:
        if entry not in uniq:
            uniq[entry] = dict(name=name, shape=shp, slug=None, file=None)
    print(f"[3/4] 고유 파트 {len(uniq)}개 테셀레이션 "
          f"(lin={args.deflection}mm, ang={args.angular}rad)", flush=True)

    used, report = {}, []
    for k, (entry, info) in enumerate(uniq.items()):
        slug = slugify(info["name"])
        if slug in used:                      # 이름 충돌 → 접미사
            used[slug] += 1
            slug = f"{slug}_{used[slug]:02d}"
        else:
            used[slug] = 0
        pts, idx = tessellate(info["shape"], args.deflection, args.angular)
        fn = os.path.join(parts_dir, f"{slug}.usd")
        write_part_usd(fn, slug, pts, idx, info["name"])
        info["slug"], info["file"] = slug, fn
        report.append(dict(entry=entry, name=info["name"], slug=slug,
                           points=len(pts), tris=len(idx) // 3))
        print(f"  [{k+1:>3}/{len(uniq)}] {info['name'][:44]:<44} → {slug:<38}"
              f" {len(pts):>7,}pt {len(idx)//3:>7,}tri", flush=True)

    # ── 조립 USD ─────────────────────────────────────────────────────────
    asm_path = os.path.join(args.out, f"{args.name}.usd")
    print(f"[4/4] 조립 → {asm_path}", flush=True)
    stage = Usd.Stage.CreateNew(asm_path)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    world = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(world.GetPrim())
    root = UsdGeom.Xform.Define(stage, "/World/frame")
    if args.source_up == "y":
        # Y-up(SolidWorks) → Z-up(USD/Isaac): X 축 +90° 회전 (Y→Z, Z→−Y).
        # 파트별 transform 은 원본 그대로 두고 루트에서 한 번만 세운다.
        root.AddRotateXOp().Set(90.0)

    n_used = {}
    for entry, name, _, trsf in kept:
        slug = uniq[entry]["slug"]
        n_used[slug] = n_used.get(slug, 0) + 1
        prim_name = slug if n_used[slug] == 1 else f"{slug}_{n_used[slug]:02d}"
        xf = UsdGeom.Xform.Define(stage, f"/World/frame/{prim_name}")
        xf.GetPrim().GetReferences().AddReference(f"./parts/{slug}.usd")
        xf.AddTransformOp().Set(trsf_to_matrix4d(trsf, MM_TO_M))
    stage.GetRootLayer().Save()

    with open(os.path.join(args.out, f"{args.name}_report.json"), "w") as fp:
        json.dump(dict(unique_parts=report,
                       instances=len(kept), dropped=len(dropped),
                       dropped_names=sorted({d[1] for d in dropped})),
                  fp, ensure_ascii=False, indent=1)

    tot_pt = sum(r["points"] for r in report)
    tot_tri = sum(r["tris"] for r in report)
    print(f"\n완료 {time.time()-t0:.1f}s — 고유파트 {len(uniq)}, 인스턴스 {len(kept)}, "
          f"점 {tot_pt:,}, 삼각형 {tot_tri:,}")
    print(f"  {asm_path}")


if __name__ == "__main__":
    main()
