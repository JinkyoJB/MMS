"""
extract_testset_points.py — testset 10종 점군 + 씬 상수 캐시 (Isaac headless 1회).

composed USD 9종(/World/ScanTarget/TestObject) + v2 marble 에서 물체 표면 점군
(world, 면적비례 샘플 + 2mm voxel)을 npz 로, v2 에서 씬 상수(axis_w, disc_top,
T_WB, T_EC)를 json 으로 저장. 이후 Phase1 viewpoint 검증은 Isaac 없이
오프라인(numpy)으로 빠르게 반복한다.

실행:  ~/isaacsim/python.sh scripts/sim/extract_testset_points.py
출력:  scripts/sim/log/testset_points/{name}.npz + scene.json + summary.txt
(Kit 이 stdout 을 가릴 수 있어 summary.txt 로도 기록)
"""
import os
import json
import glob

OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                       "log", "testset_points")
V2_USD = asset("frame_xarm7_spider_turntable/v2.usd")
import os, sys; sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))
from mms_paths import asset, testset_dir

COMPOSED_GLOB = os.path.join(testset_dir(), "composed", "*_on_turntable.usd")

ROBOT_PRIM = "/World/xarm7"
EE_PRIM = "/World/xarm7/link7"
# prim 경로는 정본(isaac_world)에서 가져온다 — v2/v3 전환 때 낡지 않도록
from mms_artec.backends.isaac import isaac_world as _IW  # noqa: E402

CAM_PRIM = _IW.CAMERA_PRIM            # 정본 참조
TT_MESH = _IW.DISC_PRIM
MARBLE_PRIM = _IW.OBJECT_PRIM
TESTOBJ_PRIM = "/World/ScanTarget/TestObject"

N_SAMPLE = 200_000        # 면적비례 표면 샘플 수 (voxel 전)
VOXEL_M = 0.002


def main():
    from isaacsim import SimulationApp
    app = SimulationApp({"headless": True})
    import numpy as np
    from pxr import Usd, UsdGeom

    os.makedirs(OUT_DIR, exist_ok=True)
    lines = []

    def log(s):
        print(s)
        lines.append(s)

    def world_T(xc, prim):
        return np.array(xc.GetLocalToWorldTransform(prim), float).T   # 열벡터 규약

    def frame_T(xc, prim):
        """프레임(좌표계) 변환용 — prim 에 박힌 scale 제거(직교화).
        (Artec_Space_Spider_mm 등 mm 에셋 하위 prim 은 scale 이 섞여 있어
        그대로 쓰면 T_EC 가 오염됨. 점군 변환은 scale 유지 world_T 사용.)"""
        T = world_T(xc, prim)
        R = T[:3, :3]
        T = T.copy()
        T[:3, :3] = R / np.linalg.norm(R, axis=0, keepdims=True)
        return T

    def subtree_triangles(stage, root_path):
        """root 아래 모든 Mesh 의 world 삼각형 (V0,V1,V2) 수집."""
        xc = UsdGeom.XformCache(Usd.TimeCode.Default())
        root = stage.GetPrimAtPath(root_path)
        tris = []
        for prim in Usd.PrimRange(root):
            if not prim.IsA(UsdGeom.Mesh):
                continue
            mesh = UsdGeom.Mesh(prim)
            pts = mesh.GetPointsAttr().Get()
            fvc = mesh.GetFaceVertexCountsAttr().Get()
            fvi = mesh.GetFaceVertexIndicesAttr().Get()
            if not pts or not fvc or not fvi:
                continue
            P = np.array(pts, float)
            T = world_T(xc, prim)
            P = (np.c_[P, np.ones(len(P))] @ T.T)[:, :3]
            fvc = np.array(fvc, int)
            fvi = np.array(fvi, int)
            # fan triangulation
            offs = np.concatenate([[0], np.cumsum(fvc)])
            for f in range(len(fvc)):
                idx = fvi[offs[f]:offs[f + 1]]
                for k in range(1, len(idx) - 1):
                    tris.append((idx[0], idx[k], idx[k + 1], P))
        if not tris:
            return None
        # (메시별 P 를 참조하므로 배열로 재구성)
        A = np.array([t[3][t[0]] for t in tris])
        B = np.array([t[3][t[1]] for t in tris])
        C = np.array([t[3][t[2]] for t in tris])
        return A, B, C

    def sample_surface(tri, n):
        A, B, C = tri
        area = 0.5 * np.linalg.norm(np.cross(B - A, C - A), axis=1)
        if area.sum() <= 0:
            return np.zeros((0, 3))
        pick = np.random.default_rng(0).choice(
            len(A), size=n, p=area / area.sum())
        r1 = np.sqrt(np.random.default_rng(1).uniform(size=(n, 1)))
        r2 = np.random.default_rng(2).uniform(size=(n, 1))
        return ((1 - r1) * A[pick] + r1 * (1 - r2) * B[pick]
                + r1 * r2 * C[pick])

    def voxel(pts, v):
        keys = np.floor(pts / v).astype(np.int64)
        _, idx = np.unique(keys, axis=0, return_index=True)
        return pts[np.sort(idx)]

    # ── 씬 상수 (v2) ─────────────────────────────────────────────────────
    st = Usd.Stage.Open(V2_USD)
    xc = UsdGeom.XformCache(Usd.TimeCode.Default())
    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"],
                           useExtentsHint=True)
    r = bc.ComputeWorldBound(st.GetPrimAtPath(TT_MESH)).ComputeAlignedRange()
    mn, mx = np.array(r.GetMin()), np.array(r.GetMax())
    axis_w = [(mn[0] + mx[0]) / 2, (mn[1] + mx[1]) / 2, float(mx[2])]
    T_WB = frame_T(xc, st.GetPrimAtPath(ROBOT_PRIM))
    T_W_E = frame_T(xc, st.GetPrimAtPath(EE_PRIM))
    T_W_C = frame_T(xc, st.GetPrimAtPath(CAM_PRIM))
    T_EC = np.linalg.inv(T_W_C) @ T_W_E                    # E→C (하니스와 동일)
    scene = dict(axis_xy=axis_w[:2], disc_top_z=axis_w[2],
                 T_WB=T_WB.tolist(), T_EC=T_EC.tolist())
    json.dump(scene, open(os.path.join(OUT_DIR, "scene.json"), "w"), indent=1)
    log(f"[scene] axis=({axis_w[0]:.4f},{axis_w[1]:.4f}) disc_top={axis_w[2]:.4f}")

    # ── 물체별 점군 ──────────────────────────────────────────────────────
    jobs = [(os.path.basename(p).replace("_on_turntable.usd", ""), p,
             TESTOBJ_PRIM) for p in sorted(glob.glob(COMPOSED_GLOB))]
    jobs.append(("solid_marble", V2_USD, MARBLE_PRIM))

    for name, usd, root in jobs:
        stg = Usd.Stage.Open(usd)
        tri = subtree_triangles(stg, root)
        if tri is None:
            log(f"[{name}] ⚠ mesh 없음 — skip")
            continue
        pts = voxel(sample_surface(tri, N_SAMPLE), VOXEL_M)
        np.savez_compressed(os.path.join(OUT_DIR, f"{name}.npz"),
                            pts=pts.astype(np.float32))
        zmn, zmx = pts[:, 2].min(), pts[:, 2].max()
        rr = np.linalg.norm(pts[:, :2] - np.array(axis_w[:2]), axis=1)
        log(f"[{name}] {len(pts)}pt z=[{zmn:.3f},{zmx:.3f}] "
            f"h={(zmx-zmn)*1000:.0f}mm r_max={rr.max()*1000:.0f}mm")

    open(os.path.join(OUT_DIR, "summary.txt"), "w").write("\n".join(lines))
    app.close()


if __name__ == "__main__":
    main()
