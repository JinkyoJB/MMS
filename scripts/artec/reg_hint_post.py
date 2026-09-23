# scripts/artec/reg_hint_post.py
#
# 설계 힌트(디스크면 + H/2 피벗)로 flip 을 배치한 뒤, **Artec SDK 의 후처리를 그대로**
# 돌려 단계마다 결과를 남긴다:  힌트 배치 → OutliersRemoval → GlobalRegistration → PoissonFusion.
#
# "힌트를 제대로 주면 SDK 노이즈제거·전역정합·융합이 되나" 를 run 없이 확인하는 도구.
# raw sproj(정합 전 IScan 원본)를 열어 flip 스캔의 프레임 변환만 run 의 힌트 → 설계 힌트로
# 바꿔 끼운다(다른 것은 손대지 않는다).
#
# 산출 (output/registration_test/<RUN>/post_H<mm>/):
#   00_hint.sproj/.ply     설계 배치 그대로
#   01_outliers.sproj/.ply OutliersRemoval 뒤
#   02_greg.sproj/.ply     GlobalRegistration(GEOMETRY) 뒤 — flip 이 얼마나 움직였는지 report 에
#   03_fusion.obj          PoissonFusion 메시 (SDK) — `--show` 면 창으로 띄운다
#   report.txt             단계별 소요·지표
#
# 실행: python scripts/artec/reg_hint_post.py --run 20260923_102221 \
#         --preview-master output/20260923_102221/debug/preview_points_102313.npz \
#         --preview-flip   output/20260923_102221/debug/preview_points_102800.npz [--H-mm 129] [--show]

from __future__ import annotations

import argparse
import glob
import os
import sys
import time
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


def rot_about_axis(axis, ang):
    a = np.asarray(axis, float); a = a / np.linalg.norm(a)
    c, s = np.cos(ang), np.sin(ang)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) * c + np.outer(a, a) * (1 - c) + K * s


def T_from(R, t):
    T = np.eye(4); T[:3, :3] = R; T[:3, 3] = t; return T


def mm2m(T):
    T = np.asarray(T, float).copy(); T[:3, 3] /= 1000.0; return T


def m2mm(T):
    T = np.asarray(T, float).copy(); T[:3, 3] *= 1000.0; return T


def scan_pts_W_m(scan, stride=3):
    P = []
    for j in range(0, scan.frame_count(), max(1, stride)):
        v = np.asarray(scan.get_frame(j).vertices(), float)
        if v.size == 0:
            continue
        T = np.asarray(scan.get_frame_transformation(j), float)
        P.append(v @ T[:3, :3].T + T[:3, 3])
    return (np.vstack(P) / 1000.0) if P else np.zeros((0, 3))


def write_ply(path, P_m, rgb):
    P = np.asarray(P_m, np.float32); n = len(P)
    rec = np.empty(n, dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("r", "u1"), ("g", "u1"), ("b", "u1")])
    rec["x"], rec["y"], rec["z"] = P[:, 0], P[:, 1], P[:, 2]
    rec["r"], rec["g"], rec["b"] = rgb[:, 0], rgb[:, 1], rgb[:, 2]
    with open(path, "wb") as f:
        f.write(("ply\nformat binary_little_endian 1.0\n"
                 f"element vertex {n}\nproperty float x\nproperty float y\nproperty float z\n"
                 "property uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n").encode())
        f.write(rec.tobytes())


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--run", default="20260923_102221")
    ap.add_argument("--preview-master", required=True)
    ap.add_argument("--preview-flip", required=True)
    ap.add_argument("--H-mm", type=float, default=None, help="물체 높이 고정 (없으면 preview 최댓값)")
    ap.add_argument("--stride", type=int, default=3)
    ap.add_argument("--skip-outliers", action="store_true")
    ap.add_argument("--texture-gr", action="store_true", help="GEOMETRY_AND_TEXTURE 로 GR")
    ap.add_argument("--show", action="store_true", help="fusion 메시를 창으로 띄운다")
    a = ap.parse_args()

    from mms_artec.sensor.artec_client import ArtecClient
    from mms_artec.sensor import artec_base, artec_algorithm as A

    run = a.run
    sproj = _ROOT / "output" / run / "aligned" / "aligned.sproj"            # 2026-09-23 이후 run 폴더
    if not sproj.exists():                                                  # 예전 run: output/artec_lookaround_<RUN>_raw/
        sproj = _ROOT / "output" / f"artec_lookaround_{run}_raw" / f"artec_lookaround_{run}_raw.sproj"
    from utils.run_paths import scan_dumps_dir
    D = scan_dumps_dir(run)
    rep: list[str] = []

    def log(m=""):
        print(m, flush=True); rep.append(m)

    t_all = time.perf_counter()
    # ── 1. raw sproj + dump ─────────────────────────────────────────────────
    entries = ArtecClient.load_project(str(sproj))
    scans = [e.scan for e in entries if getattr(e, "scan", None) is not None and e.scan.frame_count() > 0]
    log(f"run {run}: raw IScan {len(scans)}개 — " + ", ".join(f"{s.frame_count()}fr" for s in scans))
    dumps = {}
    for f in sorted(glob.glob(str(D / "scan*.npz"))):
        z = np.load(f, allow_pickle=True)
        k = int(os.path.basename(f)[4:6])
        if k not in dumps or len(z["pts_W_mm"]) > len(dumps[k]["pts_W_mm"]):
            dumps[k] = z
    i_master = 0
    i_flip = next(k for k in sorted(dumps) if str(dumps[k]["stage"]) == "flip" and k < len(scans))
    d0, df = dumps[i_master], dumps[i_flip]
    log(f"  master = raw scan {i_master} ({scans[i_master].frame_count()}fr) · flip = raw scan {i_flip} "
        f"({scans[i_flip].frame_count()}fr, T_BC_used={bool(df['T_BC_is_used'])})")

    # ── 2. 설계 힌트 (reg_hint_test.py 와 같은 계산) ─────────────────────────
    tt = TurntableTransformConfig(load_transform(str(_ROOT / "config/calibration/turntable_frame.yaml"), "T_B_F0"))
    AX = np.asarray(tt.axis_point_B, float); AD = np.asarray(tt.axis_dir_B, float); AD /= np.linalg.norm(AD)
    S = mm2m(d0["T_scan_color_mm"])
    T_CB_master = np.asarray(d0["master_T_CB"], float)
    T_BC_new = np.asarray(df["T_BC_new"], float)
    th0 = float(df["theta0"])
    R_obj = np.eye(4)
    if abs(th0) > 1e-6:
        R3 = rot_about_axis(AD, -th0); R_obj = T_from(R3, AX - R3 @ AX)
    A_master = T_CB_master @ S
    A_flip = R_obj @ np.linalg.inv(T_BC_new) @ S

    from utils.nbv.lookaround import robust_top_height
    def stats(P):
        v = P - AX; h = v @ AD; perp = v - np.outer(h, AD); k = np.linalg.norm(perp, axis=1) < .16
        h, perp = h[k], perp[k]; low = h >= np.percentile(h, 67)
        H, _ = robust_top_height(P, AX, AD)                  # 밀도 기준 — p1 은 잡음 꼬리에 끌린다
        return (H if H is not None else abs(float(np.percentile(h, 1)))), perp[low].mean(0)
    H_up, C_up = stats(np.asarray(np.load(a.preview_master, allow_pickle=True)["points_B"], float))
    H_fl, C_fl = stats(np.asarray(np.load(a.preview_flip, allow_pickle=True)["points_B"], float))
    H = (a.H_mm / 1000.0) if a.H_mm else max(H_up, H_fl)
    R = np.diag([-1.0, 1.0, -1.0])                                  # base Y 180°
    T_des_B = T_from(R, (AX + C_up - H / 2 * AD) - R @ (AX + C_fl - H / 2 * AD))
    T_pre_design_mm = m2mm(np.linalg.inv(A_master) @ T_des_B @ A_flip)   # flip W → master W
    T_pre_run_mm = np.asarray(df["T_pre_mm"], float)
    log(f"  H={H*1000:.0f}mm (preview 똑바로 {H_up*1000:.0f} · 뒤집힘 {H_fl*1000:.0f}"
        + (f", --H-mm 지정" if a.H_mm else "") + f")  footprint Δ={np.linalg.norm(C_fl-C_up)*1000:.1f}mm")
    dT = T_pre_design_mm @ np.linalg.inv(T_pre_run_mm)
    log(f"  run 힌트 → 설계 힌트 차이: 이동 {np.linalg.norm(dT[:3,3]):.1f}mm · "
        f"회전 {np.degrees(np.arccos(np.clip((np.trace(dT[:3,:3])-1)/2,-1,1))):.2f}°")

    # ── 3. 모델 재구성: flip 프레임 변환만 갈아끼운다 ──────────────────────
    def copy_scan(scan, fix=None):
        ns = artec_base.create_scan()
        for j in range(scan.frame_count()):
            ns.add_frame(scan.get_frame(j))
            T = np.asarray(scan.get_frame_transformation(j), float)
            ns.set_frame_transformation(j, fix @ T if fix is not None else T)
        return ns
    model = artec_base.create_model()
    model.add_scan(copy_scan(scans[i_master]))
    model.add_scan(copy_scan(scans[i_flip], fix=dT))
    OUT = _ROOT / "output" / "registration_test" / run / f"post_H{H*1000:.0f}"
    OUT.mkdir(parents=True, exist_ok=True)

    from scipy.spatial import cKDTree
    A_m_inv = None

    def metrics(tag, mdl, t0=None):
        """flip vs master: 겹침 높이대 최근접 · flip 프레임0 변환(설계 대비 이동/회전)."""
        Pm = scan_pts_W_m(mdl.get_scan(0), a.stride); Pf = scan_pts_W_m(mdl.get_scan(1), a.stride)
        Bm = (A_master[:3, :3] @ Pm.T).T + A_master[:3, 3]; Bf = (A_master[:3, :3] @ Pf.T).T + A_master[:3, 3]
        hm = (Bm - AX) @ AD; hf = (Bf - AX) @ AD
        lo, hi = np.percentile(hm, 1), np.percentile(hm, 99)
        ovl = (hf >= lo) & (hf <= hi)
        dd, _ = cKDTree(Bm[::2]).query(Bf[ovl][::4], k=1)
        T0 = np.asarray(mdl.get_scan(1).get_frame_transformation(0), float)
        T0_des = dT @ np.asarray(scans[i_flip].get_frame_transformation(0), float)
        D = T0 @ np.linalg.inv(T0_des)
        mv = np.linalg.norm(D[:3, 3]); rd = np.degrees(np.arccos(np.clip((np.trace(D[:3, :3]) - 1) / 2, -1, 1)))
        log(f"  [{tag:12s}] master {len(Pm):,}pt · flip {len(Pf):,}pt   겹침대 최근접 중앙 {np.median(dd)*1000:.1f} / p90 "
            f"{np.percentile(dd,90)*1000:.1f}mm   flip 이 설계 배치에서 움직인 양 {mv:.1f}mm / {rd:.2f}°"
            + (f"   ({time.perf_counter()-t0:.0f}s)" if t0 else ""))
        return Pm, Pf

    def save(tag, mdl):
        Pm, Pf = metrics(tag, mdl)
        try:
            ArtecClient.save_project(mdl, str(OUT / f"{tag}.sproj"))
        except Exception as e:                                   # noqa: BLE001
            log(f"    sproj 저장 실패({type(e).__name__}: {e})")
        write_ply(OUT / f"{tag}.ply", np.vstack([Pm, Pf]),
                  np.vstack([np.full((len(Pm), 3), (200, 200, 200), np.uint8),
                             np.full((len(Pf), 3), (60, 200, 60), np.uint8)]))

    log("\n== 00 설계 힌트 배치 ==")
    save("00_hint", model)

    if not a.skip_outliers:
        log("\n== 01 OutliersRemoval ==")
        t0 = time.perf_counter()
        try:
            model = A.Algorithms.outliers_removal(model)
            log(f"  완료 {time.perf_counter()-t0:.0f}s")
            save("01_outliers", model)
        except Exception as e:                                   # noqa: BLE001
            log(f"  ✘ 실패({type(e).__name__}: {e}) — 건너뜀")

    log(f"\n== 02 GlobalRegistration ({'GEOMETRY_AND_TEXTURE' if a.texture_gr else 'GEOMETRY'}) ==")
    t0 = time.perf_counter()
    try:
        gs = A.GlobalRegistrationSettingsDTO.default()
        gs.registration_type = (A.GlobalRegistrationType.GEOMETRY_AND_TEXTURE if a.texture_gr
                                else A.GlobalRegistrationType.GEOMETRY)
        model = A.Algorithms.global_registration(model, gs)
        log(f"  완료 {time.perf_counter()-t0:.0f}s")
        save("02_greg", model)
    except Exception as e:                                       # noqa: BLE001
        log(f"  ✘ 실패({type(e).__name__}: {e}) — 힌트 배치 그대로 fusion 으로")

    log("\n== 03 PoissonFusion ==")
    t0 = time.perf_counter()
    obj = OUT / "03_fusion.obj"
    try:
        fused = A.Algorithms.poisson_fusion(model)
        log(f"  fusion 완료 {time.perf_counter()-t0:.0f}s")
        # SmallObjectsFilter 는 **fusion 뒤**(메시 입력) — 잡음 조각 제거. system.py 와 같은 순서.
        try:
            t1 = time.perf_counter()
            fused = A.Algorithms.small_objects_filter(fused)
            log(f"  SmallObjectsFilter 완료 {time.perf_counter()-t1:.0f}s")
        except Exception as ex:                              # noqa: BLE001
            log(f"  SmallObjectsFilter 실패({type(ex).__name__}: {ex}) — 건너뜀")
        fused.save_obj(str(obj))
        try:
            import open3d as o3d
            _m = o3d.io.read_triangle_mesh(str(obj))
            _cl = np.asarray(_m.cluster_connected_triangles()[0]); _sz = np.sort(np.bincount(_cl))[::-1]
            _T = np.asarray(_m.triangles); _e = np.sort(np.vstack([_T[:, [0, 1]], _T[:, [1, 2]], _T[:, [2, 0]]]), axis=1)
            _, _c = np.unique(_e, axis=0, return_counts=True)
            log(f"  메시 {len(_m.triangles):,} f · 덩어리 {len(_sz)}개 · watertight={_m.is_watertight()} · 열린 경계엣지 {int((_c == 1).sum()):,}")
        except Exception:                                    # noqa: BLE001
            pass
        log(f"  → {obj}")
    except Exception as e:                                       # noqa: BLE001
        log(f"  ✘ 실패({type(e).__name__}: {e})")
        obj = None

    log(f"\n총 {time.perf_counter()-t_all:.0f}s → {OUT}")
    (OUT / "report.txt").write_text("\n".join(rep) + "\n", encoding="utf-8")

    if a.show and obj is not None and obj.exists():
        # `utils.viz.show_textured_obj` 는 텍스처 없는 OBJ 면 창을 안 열고 False 를 돌려준다
        # → 텍스처 없어도 그리는 show_obj.py 를 쓴다 (2026-09-23).
        import subprocess
        subprocess.call([sys.executable, str(_ROOT / "scripts" / "artec" / "show_obj.py"), str(obj),
                         "--title", f"hint(disc+H/2, H={H*1000:.0f}mm) -> Outliers -> GR -> Fusion  [{run}]"])
    return 0


if __name__ == "__main__":
    sys.exit(main())
