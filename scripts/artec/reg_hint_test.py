# scripts/artec/reg_hint_test.py
#
# flip 정합 '힌트' 를 **원래 설계대로** 오프라인 계산해 결과를 남긴다.
#
# 설계 (사용자 2026-09-23):
#   · Artec SLAM 의 scan world = 그 IScan 의 **첫 프레임** 카메라(스캐너3D) 프레임.
#   · lookaround 첫 프레임의 hand-eye(T_BC) + 그때 턴테이블 각 → 물체를 base(B) 로.
#   · flip 첫 프레임의 hand-eye + 턴테이블 각 + "물체 최대높이/2 를 피벗으로 180° 뒤집기"
#     → 같은 B 로. 두 개를 B 에서 겹치면 정합.
#
# 지금 코드와의 차이는 **뒤집기 피벗 하나**다:
#   · 현재 코드 `_flip_unflip_B`: 두 스캔의 **무게중심**끼리 맞춘다 → 커버리지가 다르면 틀린다.
#   · 설계:           디스크면(h=0) 과 preview 물체높이 H 로 **h=−H/2** 를 피벗으로.
#
# 산출 (output/registration_test/<RUN>/):
#   master.ply / .obj                     lookaround (기준)
#   flip_run.ply / .obj                   run 이 실제 적용한 배치 (무게중심 피벗)
#   flip_design.ply / .obj                설계대로 (디스크면 + H/2 피벗, 회전축 = base Y)
#   flip_design_axisperp.ply              같은데 회전축을 턴테이블 축에 수직으로 투영
#   merged_run.ply / merged_design.ply    master + flip
#   merged_*_poisson.obj                  Open3D Poisson 메시 (정합이 맞으면 한 덩어리)
#   side_view.png                         옆에서 본 비교 그림
#   report.txt                            수치
#
# 실행: python scripts/artec/reg_hint_test.py [--run 20260923_102221]

from __future__ import annotations

import argparse
import glob
import os
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


# ── 기하 도우미 ───────────────────────────────────────────────────────────
def rot_about_axis(axis, ang):
    a = np.asarray(axis, float)
    a = a / np.linalg.norm(a)
    c, s = np.cos(ang), np.sin(ang)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) * c + np.outer(a, a) * (1 - c) + K * s


def T_from(R, t):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


def apply(T, P):
    P = np.asarray(P, float)
    return (T[:3, :3] @ P.T).T + T[:3, 3]


def mm2m(T_mm):
    T = np.asarray(T_mm, float).copy()
    T[:3, 3] /= 1000.0
    return T


def m2mm(T_m):
    T = np.asarray(T_m, float).copy()
    T[:3, 3] *= 1000.0
    return T


# ── 입출력 ────────────────────────────────────────────────────────────────
def write_ply(path, P_m, rgb=None):
    P = np.asarray(P_m, np.float32)
    n = len(P)
    if rgb is None:
        rgb = np.full((n, 3), 180, np.uint8)
    hdr = ("ply\nformat binary_little_endian 1.0\n"
           f"element vertex {n}\n"
           "property float x\nproperty float y\nproperty float z\n"
           "property uchar red\nproperty uchar green\nproperty uchar blue\n"
           "end_header\n")
    rec = np.empty(n, dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"),
                             ("r", "u1"), ("g", "u1"), ("b", "u1")])
    rec["x"], rec["y"], rec["z"] = P[:, 0], P[:, 1], P[:, 2]
    rec["r"], rec["g"], rec["b"] = rgb[:, 0], rgb[:, 1], rgb[:, 2]
    with open(path, "wb") as f:
        f.write(hdr.encode("ascii"))
        f.write(rec.tobytes())


def write_obj_points(path, P_m):
    with open(path, "w", encoding="ascii") as f:
        f.write("# point cloud (mm)\n")
        for x, y, z in np.asarray(P_m, float) * 1000.0:
            f.write(f"v {x:.2f} {y:.2f} {z:.2f}\n")


def poisson_obj(path, P_m, depth=7):
    try:
        import open3d as o3d
    except Exception:                                   # noqa: BLE001
        return "open3d 없음"
    pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(np.asarray(P_m, float)))
    pcd = pcd.voxel_down_sample(0.0015)
    pcd.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.008, max_nn=30))
    pcd.orient_normals_consistent_tangent_plane(20)
    mesh, dens = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth=depth)
    dens = np.asarray(dens)
    mesh.remove_vertices_by_mask(dens < np.quantile(dens, 0.04))
    mesh.scale(1000.0, center=(0, 0, 0))                # mm 로 저장
    o3d.io.write_triangle_mesh(path, mesh)
    return f"{len(mesh.vertices):,} v / {len(mesh.triangles):,} f"


# ── 메인 ─────────────────────────────────────────────────────────────────
def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--run", default="20260923_102221")
    ap.add_argument("--preview-master", default=None, help="똑바로 선 물체의 preview npz")
    ap.add_argument("--preview-flip", default=None, help="뒤집힌 물체의 preview npz")
    ap.add_argument("--H-mm", type=float, default=None,
                    help="물체 높이를 preview 대신 이 값으로 (민감도 확인용)")
    a = ap.parse_args()

    from utils.run_paths import scan_dumps_dir
    D = scan_dumps_dir(a.run)
    OUT = _ROOT / "output" / "registration_test" / a.run
    OUT.mkdir(parents=True, exist_ok=True)
    rep = []

    def log(msg=""):
        print(msg)
        rep.append(msg)

    # 턴테이블 축 (B, m)
    tt = TurntableTransformConfig(load_transform(str(_ROOT / "config/calibration/turntable_frame.yaml"), "T_B_F0"))
    AX_PT = np.asarray(tt.axis_point_B, float)
    AX_DIR = np.asarray(tt.axis_dir_B, float)
    AX_DIR = AX_DIR / np.linalg.norm(AX_DIR)

    def h_r(P_B):
        v = np.asarray(P_B, float) - AX_PT
        h = v @ AX_DIR
        return h, np.linalg.norm(v - np.outer(h, AX_DIR), axis=1)

    # ── dump 로드 ─────────────────────────────────────────────────────────
    fs = sorted(glob.glob(str(D / "*.npz")))
    look = [f for f in fs if "lookaround" in os.path.basename(f)]
    flip = [f for f in fs if "flip" in os.path.basename(f)]
    if not look or not flip:
        print(f"✘ {D}: lookaround {len(look)} · flip {len(flip)} — 둘 다 있어야 한다")
        return 1
    # master = 가장 큰 lookaround dump (빈 재시도 스캔은 제외)
    d0 = max((np.load(f, allow_pickle=True) for f in look), key=lambda z: len(z["pts_W_mm"]))
    df = max((np.load(f, allow_pickle=True) for f in flip), key=lambda z: len(z["pts_W_mm"]))
    log(f"run {a.run}")
    log(f"  master(lookaround): {len(d0['pts_W_mm']):,} pt   flip: {len(df['pts_W_mm']):,} pt  "
        f"(flip T_BC_used={bool(df['T_BC_is_used']) if 'T_BC_is_used' in df.files else '?'})")

    S = mm2m(d0["T_scan_color_mm"])                     # 스캐너3D → Color (m)
    T_CB_master = np.asarray(d0["master_T_CB"], float)  # C→B (m) — lookaround 첫 프레임 때
    T_BC_master = np.linalg.inv(T_CB_master)
    T_BC_new = np.asarray(df["T_BC_new"], float)        # flip 첫 프레임 때 B→C (m)
    th0 = float(df["theta0"])
    R_obj = np.eye(4)
    if abs(th0) > 1e-6:
        R3 = rot_about_axis(AX_DIR, -th0)
        R_obj = T_from(R3, AX_PT - R3 @ AX_PT)

    # W → B
    A_master = T_CB_master @ S                          # master scan world → B
    A_flip = R_obj @ np.linalg.inv(T_BC_new) @ S        # flip scan world → B (되돌리기 전)
    P_m_W = np.asarray(d0["pts_W_mm"], float) / 1000.0
    P_f_W = np.asarray(df["pts_W_mm"], float) / 1000.0
    # master 는 dump 에 자기 T_pre(항등) 가 이미 적용된 스캔월드 점
    Tm = mm2m(d0["T_pre_mm"])
    P_m_B = apply(A_master, apply(Tm, P_m_W))
    P_f_B_raw = apply(A_flip, P_f_W)                    # 뒤집힌 채 놓인 자리 (B)

    hm, rm = h_r(P_m_B)
    hf, rf = h_r(P_f_B_raw)
    log(f"  B 프레임 확인 — master h {np.percentile(hm,1)*1000:+.0f}..{np.percentile(hm,99)*1000:+.0f}mm (디스크=0, 위=음수)  "
        f"flip(뒤집힌 채) h {np.percentile(hf,1)*1000:+.0f}..{np.percentile(hf,99)*1000:+.0f}mm")
    log(f"  두 스캔 모두 디스크 위(h<0)에 있어야 정상이다.")

    def axis_plane(h, r, r_max=0.03):
        """축 근처(r<r_max) 점의 h 최빈 5mm 구간 — 수평면(어깨·바닥) 높이."""
        near = r < r_max
        hist, edges = np.histogram(h[near], bins=np.arange(-0.20, 0.03, 0.005))
        if not hist.size or hist.max() == 0:
            return None, "-"
        j = int(np.argmax(hist))
        return 0.5 * (edges[j] + edges[j + 1]), f"{edges[j]*1000:+.0f}~{edges[j+1]*1000:+.0f}mm"

    h_plane_m, s_plane_m = axis_plane(hm, rm)
    log(f"  master 축근처 수평면(어깨): {s_plane_m}  ← flip 을 되돌린 뒤 같은 면이 여기 와야 한다")

    # ── preview: 높이 H · footprint 중심 ───────────────────────────────────
    def pick_preview(explicit, before_or_after):
        if explicit:
            return Path(explicit)
        # 그 run 폴더의 preview 점군 (output/<RUN>/debug/preview_points_HHMMSS.npz)
        cands = sorted(glob.glob(str(_ROOT / "output" / a.run / "debug" / "preview_points_*.npz")))
        # run 시각 이후 첫 두 개: [0]=똑바로, [1]=뒤집힘 (보수적으로 이름 시각 기준)
        hh = a.run.split("_")[1][:4]
        same = [c for c in cands if os.path.basename(c)[15:19] >= hh][:2]
        return Path(same[before_or_after]) if len(same) > before_or_after else None

    pv_m = pick_preview(a.preview_master, 0)
    pv_f = pick_preview(a.preview_flip, 1)

    def preview_stats(path):
        P = np.asarray(np.load(path, allow_pickle=True)["points_B"], float)
        h, r = h_r(P)
        keep = r < 0.16
        P, h = P[keep], h[keep]
        # ★ 높이는 밀도 기준 (`robust_top_height`) — p1 은 성긴 잡음 꼬리에 끌려 119mm 가
        #   나왔고(실제 ~80mm), 그 H 로 반사하면 flip 의 뒷면이 master 윗면 높이에 겹친다.
        from utils.nbv.lookaround import robust_top_height
        H, _ = robust_top_height(P, AX_PT, AX_DIR)
        if H is None:
            H = abs(float(np.percentile(h, 1)))
        low = h >= np.percentile(h, 67)                 # 디스크에 가까운 1/3 → footprint
        v = P[low] - AX_PT
        c_perp = (v - np.outer(v @ AX_DIR, AX_DIR)).mean(axis=0)
        return H, c_perp

    H_m, C_m = preview_stats(pv_m)
    H_f, C_f = preview_stats(pv_f)
    H = 0.5 * (H_m + H_f)
    log(f"  preview 높이: 똑바로 {H_m*1000:.0f}mm ({pv_m.name}) · 뒤집힘 {H_f*1000:.0f}mm ({pv_f.name}) → H={H*1000:.0f}mm")
    if a.H_mm:
        H = float(a.H_mm) / 1000.0
        log(f"  ★ --H-mm 지정: H={H*1000:.0f}mm 로 계산")
    log(f"  footprint 중심 축에서: 똑바로 {np.linalg.norm(C_m)*1000:.1f}mm · 뒤집힘 {np.linalg.norm(C_f)*1000:.1f}mm")

    # ── 되돌리기 변환 (B) ─────────────────────────────────────────────────
    R_y180 = rot_about_axis([0, 1.0, 0], np.pi)                        # 설정: base Y 180°
    e_perp = np.array([0, 1.0, 0]) - AX_DIR * (AX_DIR @ [0, 1.0, 0])
    R_p180 = rot_about_axis(e_perp, np.pi)                              # 턴테이블축⊥

    # (run) 무게중심 피벗 — 현재 코드와 같은 식 (master_center 는 로그의 locked 값 ≈ 점군 중앙값)
    c_pass_B = apply(A_flip, np.median(P_f_W, axis=0)[None])[0]
    c_master_B = apply(A_master, np.median(apply(Tm, P_m_W), axis=0)[None])[0]
    T_run = T_from(R_y180, c_master_B - R_y180 @ c_pass_B)

    # (design) 디스크면 + H/2 피벗
    P_f_mid = AX_PT + C_f - (H / 2) * AX_DIR
    P_m_mid = AX_PT + C_m - (H / 2) * AX_DIR
    T_design = T_from(R_y180, P_m_mid - R_y180 @ P_f_mid)
    T_design_p = T_from(R_p180, P_m_mid - R_p180 @ P_f_mid)

    # 실제 run 이 적용한 T_pre 로 놓은 것 (검산용)
    P_f_B_run_applied = apply(A_master, apply(mm2m(df["T_pre_mm"]), P_f_W))

    from scipy.spatial import cKDTree
    tree = cKDTree(P_m_B[::3])

    h_m_lo, h_m_hi = np.percentile(hm, 1), np.percentile(hm, 99)

    def evaluate(tag, P_B):
        h, r = h_r(P_B)
        hp, sp = axis_plane(h, r)
        dz = (f"{(hp - h_plane_m)*1000:+.0f}mm" if (hp is not None and h_plane_m is not None) else "-")
        # 최근접거리는 **master 가 덮은 높이대 안**의 점만 — 겹치지 않는 윗부분이 p90 을 부풀린다
        ovl = (h >= h_m_lo) & (h <= h_m_hi)
        dd, _ = tree.query(P_B[ovl][::5], k=1) if ovl.sum() > 100 else (np.array([np.nan]), None)
        log(f"  [{tag:14s}] h {np.percentile(h,1)*1000:+.0f}..{np.percentile(h,99)*1000:+.0f}mm  "
            f"수평면 {sp} (master 대비 {dz})  겹침대 최근접 중앙 {np.nanmedian(dd)*1000:.1f} / p90 {np.nanpercentile(dd,90)*1000:.1f}mm")
        return h

    log("\n== 결과 (master 기준 B 프레임) ==")
    log("  * 정답 조건: 되돌린 flip 의 원래 바닥면이 디스크면 h≈0 에 오고, 옆면이 master 와 겹쳐야 한다.")
    evaluate("run 실제 적용", P_f_B_run_applied)
    P_run = apply(T_run, P_f_B_raw);        evaluate("무게중심 재계산", P_run)
    P_des = apply(T_design, P_f_B_raw);     evaluate("설계(디스크+H/2)", P_des)
    P_desp = apply(T_design_p, P_f_B_raw);  evaluate("설계+축수직회전", P_desp)

    # ── 저장 ─────────────────────────────────────────────────────────────
    col_m = np.full((len(P_m_B), 3), (200, 200, 200), np.uint8)
    col_r = np.full((len(P_run), 3), (230, 60, 60), np.uint8)
    col_d = np.full((len(P_des), 3), (60, 200, 60), np.uint8)
    col_p = np.full((len(P_desp), 3), (60, 120, 230), np.uint8)
    write_ply(OUT / "master.ply", P_m_B, col_m);              write_obj_points(OUT / "master.obj", P_m_B)
    write_ply(OUT / "flip_run.ply", P_f_B_run_applied, col_r); write_obj_points(OUT / "flip_run.obj", P_f_B_run_applied)
    write_ply(OUT / "flip_design.ply", P_des, col_d);         write_obj_points(OUT / "flip_design.obj", P_des)
    write_ply(OUT / "flip_design_axisperp.ply", P_desp, col_p)
    write_ply(OUT / "merged_run.ply", np.vstack([P_m_B, P_f_B_run_applied]), np.vstack([col_m, col_r]))
    write_ply(OUT / "merged_design.ply", np.vstack([P_m_B, P_des]), np.vstack([col_m, col_d]))
    log("\n== Poisson 메시 (정합이 맞으면 한 덩어리) ==")
    log(f"  merged_run_poisson.obj    : {poisson_obj(str(OUT / 'merged_run_poisson.obj'), np.vstack([P_m_B, P_f_B_run_applied]))}")
    log(f"  merged_design_poisson.obj : {poisson_obj(str(OUT / 'merged_design_poisson.obj'), np.vstack([P_m_B, P_des]))}")

    # 옆면 그림
    try:
        import cv2
        W = 460
        e1 = np.array([1.0, 0, 0]) - AX_DIR * (AX_DIR @ [1.0, 0, 0]); e1 /= np.linalg.norm(e1)

        def panel(layers, title):
            img = np.zeros((W, W, 3), np.uint8)
            for P, c in layers:
                v = P - AX_PT; hh = v @ AX_DIR; x = (v - np.outer(hh, AX_DIR)) @ e1
                u = ((x + .13) / .26 * (W - 1)).astype(int); t = ((hh + .16) / .19 * (W - 1)).astype(int)
                ok = (u >= 0) & (u < W) & (t >= 0) & (t < W); img[t[ok], u[ok]] = c
            vy = int((0 + .16) / .19 * (W - 1)); cv2.line(img, (0, vy), (W - 1, vy), (70, 70, 255), 1)
            cv2.putText(img, "disc h=0", (W - 95, vy - 6), cv2.FONT_HERSHEY_SIMPLEX, .38, (70, 70, 255), 1)
            cv2.putText(img, title, (6, 17), cv2.FONT_HERSHEY_SIMPLEX, .45, (255, 255, 255), 1)
            return img
        out = np.hstack([panel([(P_m_B[::8], (255, 255, 255)), (P_f_B_run_applied[::8], (60, 60, 255))], "RUN (centroid pivot)"),
                         panel([(P_m_B[::8], (255, 255, 255)), (P_des[::8], (60, 255, 60))], "DESIGN (disc + H/2 pivot)")])
        cv2.imwrite(str(OUT / "side_view.png"), out)
    except Exception as e:                              # noqa: BLE001
        log(f"  그림 실패: {e}")

    (OUT / "report.txt").write_text("\n".join(rep) + "\n", encoding="utf-8")
    log(f"\n→ {OUT}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
