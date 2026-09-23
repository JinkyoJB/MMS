# scripts/artec/reg_hint_contour.py
#
# flip 패스의 yaw(축 둘레 각)를 **위에서 본 윤곽(footprint contour)** 으로 정한다.
# 옆면 점 최근접(reg_hint_yaw.py)은 국소 최솟값이 여럿이라 애매했다(72° vs 312°).
# 윤곽은 2D 형상이라 길쭉한 물체면 방위가 훨씬 뚜렷하다 (사용자 제안 2026-09-23).
#
# 방법: 두 점군을 디스크면에 투영 → 각 footprint 중심 기준 방위각 1° 마다 최대 반경 r(φ)
#       (98 분위수, 잡음 제거) → yaw 를 돌리며 |r_m(φ) − r_f(φ+yaw)| 평균이 최소인 각.
#       그 yaw 로 놓고 옆면 최근접도 같이 재서 교차 확인. 결과 PNG(윤곽 겹침)·PLY·report.
#
#   python scripts/artec/reg_hint_contour.py --run 20260923_102221 \
#       --preview-master output/20260923_102221/debug/preview_points_102313.npz --preview-flip output/20260923_102221/debug/preview_points_102800.npz

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


def rot(a, ang):
    a = a / np.linalg.norm(a); c, s = np.cos(ang), np.sin(ang)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) * c + np.outer(a, a) * (1 - c) + K * s


def radial_profile(xy, n_bins=360, q=98.0, h_mask=None):
    """footprint 중심 기준 방위각별 최대 반경 (m). 빈 구간은 이웃으로 보간."""
    c = xy.mean(0)
    d = xy - c
    phi = (np.degrees(np.arctan2(d[:, 1], d[:, 0])) + 360.0) % 360.0
    r = np.linalg.norm(d, axis=1)
    b = np.clip((phi / 360.0 * n_bins).astype(int), 0, n_bins - 1)
    prof = np.full(n_bins, np.nan)
    for i in range(n_bins):
        m = b == i
        if m.sum() >= 5:
            prof[i] = np.percentile(r[m], q)
    if np.isnan(prof).any():
        idx = np.arange(n_bins); ok = ~np.isnan(prof)
        prof = np.interp(idx, idx[ok], prof[ok], period=n_bins)
    return prof, c


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--run", default="20260923_102221")
    ap.add_argument("--preview-master", required=True)
    ap.add_argument("--preview-flip", required=True)
    ap.add_argument("--H-mm", type=float, default=None)
    a = ap.parse_args()
    from scipy.spatial import cKDTree
    import cv2

    RUN = a.run
    OUT = _ROOT / "output" / "registration_test" / RUN / "contour_fit"
    OUT.mkdir(parents=True, exist_ok=True)
    rep = []

    def log(m=""):
        print(m, flush=True); rep.append(m)

    tt = TurntableTransformConfig(load_transform(str(_ROOT / "config/calibration/turntable_frame.yaml"), "T_B_F0"))
    AX = np.asarray(tt.axis_point_B, float); AD = np.asarray(tt.axis_dir_B, float); AD /= np.linalg.norm(AD)
    e1 = np.array([1.0, 0, 0]) - AD * (AD @ [1.0, 0, 0]); e1 /= np.linalg.norm(e1); e2 = np.cross(AD, e1)
    from utils.run_paths import scan_dumps_dir
    D = scan_dumps_dir(RUN)
    d0 = max((np.load(f, allow_pickle=True) for f in glob.glob(str(D / "*lookaround*.npz"))), key=lambda z: len(z["pts_W_mm"]))
    df = max((np.load(f, allow_pickle=True) for f in glob.glob(str(D / "*flip*.npz"))), key=lambda z: len(z["pts_W_mm"]))
    S = np.asarray(d0["T_scan_color_mm"], float).copy(); S[:3, 3] /= 1000
    A_m = np.asarray(d0["master_T_CB"], float) @ S; A_f = np.linalg.inv(np.asarray(df["T_BC_new"], float)) @ S
    apf = lambda A, P: (A[:3, :3] @ P.T).T + A[:3, 3]
    Pm = apf(A_m, np.asarray(d0["pts_W_mm"], float) / 1000)
    Pf_raw = apf(A_f, np.asarray(df["pts_W_mm"], float) / 1000)

    def fp(path):
        P = np.asarray(np.load(path, allow_pickle=True)["points_B"], float)
        v = P - AX; h = v @ AD; perp = v - np.outer(h, AD); k = np.linalg.norm(perp, axis=1) < .16
        h, perp = h[k], perp[k]; low = h >= np.percentile(h, 67)
        H, _ = robust_top_height(P, AX, AD); return H, perp[low].mean(0)
    H_up, C_up = fp(a.preview_master); H_fl, C_fl = fp(a.preview_flip)
    # ★ H 는 **파이프라인과 같은 규칙**이어야 한다(`_flip_pivots_B`): preview(똑바로)·
    #   preview(뒤집힘)·master 세 추정의 **최댓값**. 2026-09-23 이전 이 스크립트는 preview
    #   둘만 써서(75mm vs 파이프라인 95mm) flip 을 20mm 낮게 놓았고, 그 상태의 yaw 최솟값을
    #   "유일" 이라고 보고했다 — 3D 로 열면 두 패스가 세로로 어긋나 있었다.
    H_ms, _ = robust_top_height(Pm, AX, AD)
    H = (a.H_mm / 1000.0) if a.H_mm else max(H_up, H_fl, H_ms or 0.0)
    log(f"  H 추정: preview 똑바로 {H_up*1000:.0f} · 뒤집힘 {H_fl*1000:.0f} · "
        f"master {(H_ms or 0)*1000:.0f} → 사용 {H*1000:.0f}mm")
    R = np.diag([-1.0, 1.0, -1.0]); t = (AX + C_up - H / 2 * AD) - R @ (AX + C_fl - H / 2 * AD)
    Q0 = (R @ Pf_raw.T).T + t                                    # 힌트 배치 (yaw 0)
    log(f"run {RUN}: master {len(Pm):,} · flip {len(Q0):,} · H={H*1000:.0f}mm")

    # ── 1. footprint (디스크면 투영) — 잡음 제거: 축에서 160mm 안, 디스크 위 5mm 이상 ─
    def foot(P):
        v = P - AX; h = v @ AD; perp = v - np.outer(h, AD); r = np.linalg.norm(perp, axis=1)
        m = (r < 0.16) & (h < -0.005)
        return np.stack([perp[m] @ e1, perp[m] @ e2], 1)
    Fm, Ff = foot(Pm), foot(Q0)
    pm, cm = radial_profile(Fm); pf, cf = radial_profile(Ff)
    log(f"  footprint 중심 차(힌트 상태): {np.linalg.norm(cm - cf)*1000:.1f}mm · 윤곽 반경 master {pm.mean()*1000:.0f}(±{pm.std()*1000:.0f}) / flip {pf.mean()*1000:.0f}(±{pf.std()*1000:.0f})mm")
    # 길쭉함 — 윤곽 반경의 편차가 크면 yaw 가 잘 정해진다
    log(f"  master 윤곽 r max/min = {pm.max()*1000:.0f}/{pm.min()*1000:.0f}mm · flip = {pf.max()*1000:.0f}/{pf.min()*1000:.0f}mm")

    # ── 2. yaw 스윕: 프로파일 순환 이동 차이 ────────────────────────────────
    n = len(pm)
    costs = np.array([np.mean(np.abs(pm - np.roll(pf, -k))) for k in range(n)])   # flip 을 +k° 돌린 것
    order = np.argsort(costs)
    mins = [i for i in range(n) if costs[i] < costs[(i - 1) % n] and costs[i] < costs[(i + 1) % n]]
    mins.sort(key=lambda i: costs[i])
    log("\n== 윤곽 프로파일 yaw 스윕 ==")
    log("  국소 최솟값(낮은 순): " + ", ".join(f"{i}°:{costs[i]*1000:.2f}mm" for i in mins[:6]) + f"   (전체 범위 {costs.min()*1000:.2f}~{costs.max()*1000:.2f}mm)")
    yaw_best = int(order[0]); second = costs[mins[1]] if len(mins) > 1 else None
    uniq = (second is None) or (costs[yaw_best] < 0.85 * second)
    log(f"  최선 yaw={yaw_best}° 차이 {costs[yaw_best]*1000:.2f}mm — {'유일' if uniq else '모호(두 번째 골 ' + f'{costs[mins[1]]*1000:.2f}mm)'}")

    # ── 3. 그 yaw 로 놓고 옆면 최근접으로 교차 확인 (yaw 축 = 축방향, 피벗 = flip footprint 중심) ─
    hm = (Pm - AX) @ AD; rm = np.linalg.norm((Pm - AX) - np.outer(hm, AD), axis=1)
    Mw = Pm[rm > 0.02]; tree = cKDTree(Mw[::2])

    def place(yaw_deg):
        Rr = rot(AD, np.radians(yaw_deg))
        piv = AX + cf[0] * e1 + cf[1] * e2
        Q = (Rr @ (Q0 - piv).T).T + piv
        shift = (cm[0] - cf[0]) * e1 + (cm[1] - cf[1]) * e2        # footprint 중심 맞춤
        return Q + shift

    def nn(Q):
        """**양방향** 최근접 중앙값 (mm). 예전엔 flip 점을 master 의 높이대로 걸러서 쟀는데,
        그러면 세로로 어긋난 배치일수록 '맞는 부분만' 골라 재게 돼 점수가 좋아 보인다
        (2026-09-23: 실제 15.7mm 인 배치를 8.8mm 로 보고했다). 걸러내지 않는다."""
        rq = np.linalg.norm((Q - AX) - np.outer((Q - AX) @ AD, AD), axis=1)
        Qw = Q[rq > 0.02]
        if len(Qw) < 500:
            return float("nan"), float("nan")
        d1, _ = tree.query(Qw[::4], k=1)
        d2, _ = cKDTree(Qw[::2]).query(Mw[::4], k=1)
        return float(np.median(d1)) * 1000, float(np.median(d2)) * 1000
    log("\n== 교차 확인: 그 yaw 로 놓았을 때 옆면 최근접 (양방향 중앙값) ==")
    for tag, y in [("yaw 0 (힌트)", 0)] + [(f"윤곽 최솟값 {i}°", i) for i in mins[:3]]:
        m_, p_ = nn(place(y))
        log(f"  {tag:18s}: flip→master {m_:.1f}mm · master→flip {p_:.1f}mm")
    Qb = place(yaw_best)

    # ── 4. 저장: 윤곽 겹침 그림 + 점군 ──────────────────────────────────────
    W = 460
    def top(layers, title):
        img = np.zeros((W, W, 3), np.uint8)
        for P, c in layers:
            v = P - AX; hh = v @ AD; x = (v - np.outer(hh, AD)) @ e1; y = (v - np.outer(hh, AD)) @ e2
            u = ((x + .16) / .32 * (W - 1)).astype(int); t_ = ((y + .16) / .32 * (W - 1)).astype(int)
            ok = (u >= 0) & (u < W) & (t_ >= 0) & (t_ < W); img[t_[ok], u[ok]] = c
        cv2.putText(img, title, (6, 17), cv2.FONT_HERSHEY_SIMPLEX, .45, (255, 255, 255), 1); return img
    def polar(prof, c, col, img):
        pts = np.stack([c[0] + prof * np.cos(np.radians(np.arange(n))), c[1] + prof * np.sin(np.radians(np.arange(n)))], 1)
        u = ((pts[:, 0] + .16) / .32 * (W - 1)).astype(int); t_ = ((pts[:, 1] + .16) / .32 * (W - 1)).astype(int)
        cv2.polylines(img, [np.stack([u, t_], 1).reshape(-1, 1, 2)], True, col, 1)
    img0 = top([(Pm[::6], (255, 255, 255)), (Q0[::6], (60, 60, 255))], "hint (yaw 0)")
    img1 = top([(Pm[::6], (255, 255, 255)), (Qb[::6], (60, 255, 60))], f"contour fit yaw={yaw_best}")
    img2 = np.zeros((W, W, 3), np.uint8); cv2.putText(img2, "contours: master(white) flip@best(green)", (6, 17), cv2.FONT_HERSHEY_SIMPLEX, .42, (255, 255, 255), 1)
    polar(pm, cm, (255, 255, 255), img2); polar(np.roll(pf, -yaw_best), cm, (60, 255, 60), img2)
    cv2.imwrite(str(OUT / "contour_fit.png"), np.hstack([img0, img1, img2]))
    # PLY
    def write_ply(path, P, rgb):
        P = np.asarray(P, np.float32); nn_ = len(P)
        rec = np.empty(nn_, dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("r", "u1"), ("g", "u1"), ("b", "u1")])
        rec["x"], rec["y"], rec["z"] = P[:, 0], P[:, 1], P[:, 2]; rec["r"], rec["g"], rec["b"] = rgb[:, 0], rgb[:, 1], rgb[:, 2]
        with open(path, "wb") as f:
            f.write(("ply\nformat binary_little_endian 1.0\n" f"element vertex {nn_}\nproperty float x\nproperty float y\nproperty float z\n"
                     "property uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n").encode()); f.write(rec.tobytes())
    write_ply(OUT / "merged_contour.ply", np.vstack([Pm, Qb]),
              np.vstack([np.full((len(Pm), 3), (200, 200, 200), np.uint8), np.full((len(Qb), 3), (60, 200, 60), np.uint8)]))
    (OUT / "report.txt").write_text("\n".join(rep) + "\n", encoding="utf-8")
    log(f"\n→ {OUT}  (contour_fit.png, merged_contour.ply)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
