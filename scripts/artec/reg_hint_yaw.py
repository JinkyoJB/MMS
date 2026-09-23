# scripts/artec/reg_hint_yaw.py
#
# flip 패스를 설계 힌트(디스크면 + H/2)로 되돌린 뒤, **사람이 다시 놓을 때 생긴 수평 자유도**
# (축 둘레 yaw + 수평 이동)를 옆면 겹침으로 찾아 두 패스를 한 덩어리로 맞춘다.
#
# 왜 — 뒤집기 힌트는 회전축(base Y)과 반사면(H/2)만 안다. 손으로 뒤집어 놓으면 물체는 축 둘레로
# 임의 각만큼 돌고 몇 mm 옮겨진다. 회전대칭 물체면 못 잡지만, 눕힌 병처럼 footprint 가 길쭉하면
# 옆면 기하로 yaw 가 정해진다(run_102221: yaw 0 가정 시 겹침 8.6mm 잔여).
#
# 절차: H 후보 × yaw(3° 간격) 격자 → 각 후보에서 수평 무게중심 정렬 → 겹침 높이대 최근접 중앙값
#       → 최선에서 게이트 ICP(6→3mm) 로 다듬기 → merged PLY / Poisson OBJ / 옆면 그림 / report.
#
#   python scripts/artec/reg_hint_yaw.py --run 20260923_102221 \
#       --preview-master output/20260923_102221/debug/preview_points_102313.npz --preview-flip output/20260923_102221/debug/preview_points_102800.npz [--show]

from __future__ import annotations

import argparse
import glob
import sys
from pathlib import Path

import numpy as np

_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(_ROOT))
for _s in (sys.stdout, sys.stderr):
    try:
        _s.reconfigure(encoding="utf-8", errors="replace")
    except Exception:                                   # noqa: BLE001
        pass
from utils.transforms import TurntableTransformConfig, load_transform  # noqa: E402
from utils.nbv.lookaround import robust_top_height  # noqa: E402


def rot(axis, ang):
    a = np.asarray(axis, float); a = a / np.linalg.norm(a)
    c, s = np.cos(ang), np.sin(ang)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) * c + np.outer(a, a) * (1 - c) + K * s


def ap(R, t, P):
    return (R @ np.asarray(P, float).T).T + t


def write_ply(path, P, rgb):
    P = np.asarray(P, np.float32); n = len(P)
    rec = np.empty(n, dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("r", "u1"), ("g", "u1"), ("b", "u1")])
    rec["x"], rec["y"], rec["z"] = P[:, 0], P[:, 1], P[:, 2]; rec["r"], rec["g"], rec["b"] = rgb[:, 0], rgb[:, 1], rgb[:, 2]
    with open(path, "wb") as f:
        f.write(("ply\nformat binary_little_endian 1.0\n" f"element vertex {n}\nproperty float x\nproperty float y\nproperty float z\n"
                 "property uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n").encode()); f.write(rec.tobytes())


def main() -> int:
    ap_ = argparse.ArgumentParser()
    ap_.add_argument("--run", default="20260923_102221")
    ap_.add_argument("--preview-master", required=True)
    ap_.add_argument("--preview-flip", required=True)
    ap_.add_argument("--H-list", default="70,75,80,85")
    ap_.add_argument("--yaw-step", type=float, default=3.0)
    ap_.add_argument("--show", action="store_true")
    a = ap_.parse_args()
    from scipy.spatial import cKDTree

    RUN = a.run
    OUT = _ROOT / "output" / "registration_test" / RUN / "yaw_fit"
    OUT.mkdir(parents=True, exist_ok=True)
    rep = []

    def log(m=""):
        print(m, flush=True); rep.append(m)

    tt = TurntableTransformConfig(load_transform(str(_ROOT / "config/calibration/turntable_frame.yaml"), "T_B_F0"))
    AX = np.asarray(tt.axis_point_B, float); AD = np.asarray(tt.axis_dir_B, float); AD /= np.linalg.norm(AD)
    from utils.run_paths import scan_dumps_dir
    D = scan_dumps_dir(RUN)
    d0 = max((np.load(f, allow_pickle=True) for f in glob.glob(str(D / "*lookaround*.npz"))), key=lambda z: len(z["pts_W_mm"]))
    df = max((np.load(f, allow_pickle=True) for f in glob.glob(str(D / "*flip*.npz"))), key=lambda z: len(z["pts_W_mm"]))
    S = np.asarray(d0["T_scan_color_mm"], float).copy(); S[:3, 3] /= 1000
    A_m = np.asarray(d0["master_T_CB"], float) @ S
    A_f = np.linalg.inv(np.asarray(df["T_BC_new"], float)) @ S
    Pm = ap(A_m[:3, :3], A_m[:3, 3], np.asarray(d0["pts_W_mm"], float) / 1000)
    Pf_raw = ap(A_f[:3, :3], A_f[:3, 3], np.asarray(df["pts_W_mm"], float) / 1000)

    def fp(path):
        P = np.asarray(np.load(path, allow_pickle=True)["points_B"], float)
        v = P - AX; h = v @ AD; perp = v - np.outer(h, AD); k = np.linalg.norm(perp, axis=1) < .16
        h, perp = h[k], perp[k]; low = h >= np.percentile(h, 67)
        H, _ = robust_top_height(P, AX, AD); return H, perp[low].mean(0)
    H_up, C_up = fp(a.preview_master); H_fl, C_fl = fp(a.preview_flip)
    log(f"run {RUN}: master {len(Pm):,}pt · flip {len(Pf_raw):,}pt · preview H {H_up*1000:.0f}/{H_fl*1000:.0f}mm")

    # 옆면 겹침 평가용 — master 의 옆면 높이대 안, 축에서 너무 가까운 윗면/뒷면 점은 제외(r>20mm)
    hm = (Pm - AX) @ AD; rm = np.linalg.norm((Pm - AX) - np.outer(hm, AD), axis=1)
    lo, hi = np.percentile(hm, 3), np.percentile(hm, 97)
    Mw = Pm[(rm > 0.02)][::3]
    tree = cKDTree(Mw)
    e1 = np.array([1.0, 0, 0]) - AD * (AD @ [1.0, 0, 0]); e1 /= np.linalg.norm(e1); e2 = np.cross(AD, e1)

    def place(H, yaw_deg, dxy=(0.0, 0.0)):
        R = np.diag([-1.0, 1.0, -1.0])                      # base Y 180° (설정)
        t = (AX + C_up - H / 2 * AD) - R @ (AX + C_fl - H / 2 * AD)
        Q = ap(R, t, Pf_raw)
        # yaw: 물체 자기 footprint 중심을 지나는 수직축 둘레
        c = Q.mean(0); c = c - ((c - AX) @ AD) * AD + ((AX) @ AD) * 0  # 수평 성분만 쓰기 위해 아래에서 축 보정
        Ry = rot(AD, np.radians(yaw_deg))
        piv = Q.mean(0)
        Q = ap(Ry, piv - Ry @ piv, Q)
        Q = Q + dxy[0] * e1 + dxy[1] * e2
        return Q

    def score(Q, sub=6):
        hq = (Q - AX) @ AD; rq = np.linalg.norm((Q - AX) - np.outer(hq, AD), axis=1)
        m = (hq >= lo) & (hq <= hi) & (rq > 0.02)
        if m.sum() < 500:
            return 99.0, 99.0
        d, _ = tree.query(Q[m][::sub], k=1)
        return float(np.median(d) * 1000), float(np.percentile(d, 90) * 1000)

    log("\n== 1. H × yaw 격자 (수평은 무게중심 정렬) ==")
    best = None
    Hs = [float(x) / 1000 for x in a.H_list.split(",")]
    yaws = np.arange(0.0, 360.0, a.yaw_step)
    # master 옆면 수평 중심
    cm = Mw.mean(0)
    for H in Hs:
        row = []
        for yaw in yaws:
            Q = place(H, yaw)
            hq = (Q - AX) @ AD; rq = np.linalg.norm((Q - AX) - np.outer(hq, AD), axis=1)
            m = (hq >= lo) & (hq <= hi) & (rq > 0.02)
            if m.sum() < 500:
                continue
            dc = cm - Q[m].mean(0); dc = dc - (dc @ AD) * AD          # 수평 성분만
            Q2 = Q + dc
            med, p90 = score(Q2)
            row.append((med, p90, yaw, dc))
            if best is None or med < best[0]:
                best = (med, p90, yaw, H, dc)
        r = sorted(row)[:3]
        log(f"  H={H*1000:.0f}mm: 최선 yaw " + ", ".join(f"{y:.0f}°→{m_:.1f}/{p:.1f}mm" for m_, p, y, _ in r))
    med, p90, yaw, H, dc = best
    log(f"\n★ 격자 최선: H={H*1000:.0f}mm yaw={yaw:.0f}° 수평보정 {np.linalg.norm(dc)*1000:.1f}mm → 겹침 중앙 {med:.1f} / p90 {p90:.1f}mm")

    # 미세 yaw
    fine = None
    for y in np.arange(yaw - a.yaw_step, yaw + a.yaw_step + 0.01, 0.5):
        Q = place(H, y); hq = (Q - AX) @ AD; rq = np.linalg.norm((Q - AX) - np.outer(hq, AD), axis=1)
        m = (hq >= lo) & (hq <= hi) & (rq > 0.02); d2 = cm - Q[m].mean(0); d2 = d2 - (d2 @ AD) * AD
        s_ = score(Q + d2)
        if fine is None or s_[0] < fine[0]:
            fine = (s_[0], s_[1], y, d2)
    med, p90, yaw, dc = fine
    Q_best = place(H, yaw) + dc
    log(f"  미세: yaw={yaw:.1f}° → 중앙 {med:.1f} / p90 {p90:.1f}mm")

    # ── 2. 게이트 ICP 로 다듬기 ───────────────────────────────────────────
    log("\n== 2. ICP 다듬기 (corr 6→3mm, 이동 8mm·회전 3° 게이트) ==")
    try:
        import open3d as o3d
        def pcd(P):
            p = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(P)); p = p.voxel_down_sample(0.0015)
            p.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.006, max_nn=30)); return p
        pm, ps = pcd(Pm), pcd(Q_best)
        T = np.eye(4); ok = True
        for corr in (0.006, 0.003):
            res = o3d.pipelines.registration.registration_icp(ps, pm, corr, T, o3d.pipelines.registration.TransformationEstimationPointToPlane(),
                                                              o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=40))
            T = res.transformation
        dt = np.linalg.norm(T[:3, 3]) * 1000; dr = np.degrees(np.arccos(np.clip((np.trace(T[:3, :3]) - 1) / 2, -1, 1)))
        Q_icp = ap(T[:3, :3], T[:3, 3], Q_best); m2, p2 = score(Q_icp)
        log(f"  ICP: Δ이동 {dt:.1f}mm Δ회전 {dr:.2f}° fitness {res.fitness:.2f} → 겹침 중앙 {m2:.1f} / p90 {p2:.1f}mm")
        if dt <= 8.0 and dr <= 3.0 and m2 <= med:
            Q_final, tag = Q_icp, "yaw+icp"
            log("  게이트 통과 — ICP 결과 채택")
        else:
            Q_final, tag = Q_best, "yaw"
            log("  게이트 기각(미끄러짐/악화) — yaw 결과 유지")
    except Exception as e:                                   # noqa: BLE001
        Q_final, tag = Q_best, "yaw"; log(f"  ICP 실패({e}) — yaw 결과 유지")

    # ── 3. 저장 ───────────────────────────────────────────────────────────
    hq = (Q_final - AX) @ AD
    log(f"\n최종({tag}): flip 높이 {np.percentile(hq,1)*1000:+.0f}..{np.percentile(hq,99)*1000:+.0f}mm · master {np.percentile(hm,1)*1000:+.0f}..{np.percentile(hm,99)*1000:+.0f}mm")
    col_m = np.full((len(Pm), 3), (200, 200, 200), np.uint8); col_f = np.full((len(Q_final), 3), (60, 200, 60), np.uint8)
    write_ply(OUT / "merged_yaw.ply", np.vstack([Pm, Q_final]), np.vstack([col_m, col_f]))
    write_ply(OUT / "flip_yaw.ply", Q_final, col_f)
    np.save(OUT / "flip_final_B_m.npy", Q_final[::1].astype(np.float32))
    try:
        import open3d as o3d
        pc = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(np.vstack([Pm, Q_final]))); pc = pc.voxel_down_sample(0.0015)
        pc.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.008, max_nn=30)); pc.orient_normals_consistent_tangent_plane(20)
        mesh, dens = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pc, depth=8)
        dens = np.asarray(dens); mesh.remove_vertices_by_mask(dens < np.quantile(dens, 0.05))
        cl = np.asarray(mesh.cluster_connected_triangles()[0]); big = int(np.argmax(np.bincount(cl)))
        mesh.remove_triangles_by_mask(cl != big); mesh.remove_unreferenced_vertices()
        mesh.scale(1000.0, center=(0, 0, 0)); o3d.io.write_triangle_mesh(str(OUT / "merged_yaw_poisson.obj"), mesh)
        log(f"Poisson 메시(최대 덩어리): {len(mesh.vertices):,} v / {len(mesh.triangles):,} f · watertight={mesh.is_watertight()}")
    except Exception as e:                                   # noqa: BLE001
        log(f"Poisson 실패: {e}")
    try:
        import cv2
        W = 460
        def panel(layers, title):
            img = np.zeros((W, W, 3), np.uint8)
            for P, c in layers:
                v = P - AX; hh = v @ AD; x = (v - np.outer(hh, AD)) @ e1
                u = ((x + .16) / .32 * (W - 1)).astype(int); t = ((hh + .20) / .28 * (W - 1)).astype(int)
                ok = (u >= 0) & (u < W) & (t >= 0) & (t < W); img[t[ok], u[ok]] = c
            vy = int((0 + .20) / .28 * (W - 1)); cv2.line(img, (0, vy), (W - 1, vy), (70, 70, 255), 1)
            cv2.putText(img, title, (6, 17), cv2.FONT_HERSHEY_SIMPLEX, .45, (255, 255, 255), 1); return img
        def top(layers, title):
            img = np.zeros((W, W, 3), np.uint8)
            for P, c in layers:
                v = P - AX; hh = v @ AD; x = (v - np.outer(hh, AD)) @ e1; y = (v - np.outer(hh, AD)) @ e2
                u = ((x + .16) / .32 * (W - 1)).astype(int); t = ((y + .16) / .32 * (W - 1)).astype(int)
                ok = (u >= 0) & (u < W) & (t >= 0) & (t < W); img[t[ok], u[ok]] = c
            cv2.putText(img, title, (6, 17), cv2.FONT_HERSHEY_SIMPLEX, .45, (255, 255, 255), 1); return img
        Q0 = place(H, 0.0)
        out = np.hstack([top([(Pm[::6], (255, 255, 255)), (Q0[::6], (60, 60, 255))], "TOP: yaw=0 (before)"),
                         top([(Pm[::6], (255, 255, 255)), (Q_final[::6], (60, 255, 60))], f"TOP: yaw={yaw:.0f} ({tag})"),
                         panel([(Pm[::6], (255, 255, 255)), (Q_final[::6], (60, 255, 60))], "SIDE: final")])
        cv2.imwrite(str(OUT / "yaw_fit.png"), out)
    except Exception as e:                                   # noqa: BLE001
        log(f"그림 실패: {e}")
    (OUT / "report.txt").write_text("\n".join(rep) + "\n", encoding="utf-8")
    log(f"→ {OUT}")
    if a.show and (OUT / "merged_yaw_poisson.obj").exists():
        import subprocess
        subprocess.Popen([sys.executable, str(_ROOT / "scripts/artec/show_obj.py"), str(OUT / "merged_yaw_poisson.obj"),
                          "--title", f"lookaround + flip (yaw fit {yaw:.0f}deg, H={H*1000:.0f}mm) [{RUN}]"],
                         creationflags=getattr(subprocess, "CREATE_NEW_CONSOLE", 0))
    return 0


if __name__ == "__main__":
    sys.exit(main())
