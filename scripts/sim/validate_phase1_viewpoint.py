"""
validate_phase1_viewpoint.py — Phase1 viewpoint selection 오프라인 검증 (Isaac 불필요).

입력: extract_testset_points.py 가 캐시한 10종 점군 + 씬 상수.
실험1: baseline(기존 sphere sampling: el=30, s=0.25+r_max, tz=중심, 첫 IK az)
       vs 제안(maximin 채점 + 밴드분할) — 최악프레임 fill / z-커버 / 밴드 판정
       / IK 도달성. + 2-shot(0°/90° 정적캡처 모사) 채점 랭킹 충실도.
실험2: 전회전 프레임 시뮬에 tracking-lost 프록시(연속 N프레임 fill 미달) 주입
       → baseline(동일자세 재시도) vs 제안(recovery_replan + overlap) 비교.

실행:  (env_isaacsim) python scripts/sim/validate_phase1_viewpoint.py
출력:  표(stdout) + scripts/sim/log/testset_points/validate_results.json
"""
import os
import sys
import json
import math

import numpy as np

_REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, _REPO)

from utils.nbv.phase1_viewpoint import (            # noqa: E402
    SensorModel, crop_object_points, voxel_downsample, estimate_outward_normals,
    make_view_pose, evaluate_viewpoint, plan_phase1_viewpoints, recovery_replan,
    visible_masks, _rot_z, _score, FILL_MIN_CM2)
from utils.robot import xarm7_kinematics as kin     # noqa: E402

DATA = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                    "log", "testset_points")
AZ_SWEEP = [0.0, 30.0, -30.0, 60.0, -60.0, 90.0, -90.0, 180.0]
HOME_Q = np.radians([38.92, -48.70, -65.29, 21.22, 21.46, 72.70, -96.58])
N_THETA_FINAL = 72
TRACK_MIN_CM2 = 6.0        # lost 프록시: 프레임 fill 미달
CONSEC_LOST = 3            # 연속 N 프레임 미달 → lost
BACKOFF_FRAMES = 6         # lost 후 safe-back (≈30°)
MAX_RECOVERY = 3


# ── IK (backend 프리미티브 모사: USD look-at → base → 해석 IK) ────────────────
def ik_feasible_az(scene, el, standoff, tz, axis_xy):
    T_WB = np.array(scene["T_WB"])
    T_EC = np.array(scene["T_EC"])
    for az in AZ_SWEEP:
        p = make_view_pose(axis_xy, tz, el, az, standoff)
        T_WC = np.eye(4)
        T_WC[:3, :3], T_WC[:3, 3] = p.R_wc, p.eye_w
        T_CB = np.linalg.inv(T_WB) @ T_WC
        T_EB = T_CB @ T_EC
        pose6d = np.concatenate([T_EB[:3, 3] * 1000.0,
                                 kin.R_to_euler_xyz(T_EB[:3, :3])])
        q, ok = kin.ik(pose6d, seed=HOME_Q)
        if ok:
            return az
    return None


# ── 계획용 preview 캡처 모사 (real: 거리스텝 + 조준높이 가변, 엄격 DOF) ──────
# (이전 2-shot 은 DOF 0.15~0.7 낙관 가정이었음 → 실기 DOF(0.2~0.3) 그대로의
#  거리스텝 전략(simulate_planning_captures)으로 교체, 2026-07-03)
from utils.nbv.phase1_viewpoint import simulate_planning_captures  # noqa: E402


def synth_two_shot(pts, nrm, axis_xy, sensor, disc_top_z):
    return simulate_planning_captures(pts, nrm, axis_xy, disc_top_z, sensor)


def candidate_grid_scores(pts, nrm, axis_xy, sensor, n_theta=36):
    """플래너와 동일 격자(el×s×tz)의 점수 벡터 — 2-shot 랭킹 충실도용."""
    from utils.nbv.phase1_viewpoint import _standoff_candidates, DEFAULT_ELS
    sos, _ = _standoff_candidates(pts, axis_xy, sensor)
    z = pts[:, 2]
    tzs = [float(np.quantile(z, q)) for q in (0.35, 0.5, 0.65)]
    keys, scores = [], []
    for el in DEFAULT_ELS:
        for s in sos:
            for tz in tzs:
                pose = make_view_pose(axis_xy, tz, el, 0.0, s)
                ev = evaluate_viewpoint(pts, nrm, axis_xy, pose, sensor,
                                        n_theta=n_theta)
                keys.append((el, round(s, 3), round(tz, 3)))
                scores.append(_score(ev))
    return keys, np.array(scores)


def spearman(a, b):
    ra = np.argsort(np.argsort(a)).astype(float)
    rb = np.argsort(np.argsort(b)).astype(float)
    ra -= ra.mean(); rb -= rb.mean()
    d = math.sqrt((ra**2).sum() * (rb**2).sum())
    return float((ra * rb).sum() / d) if d > 0 else 0.0


# ── 실험2: 전회전 + lost 주입 + recovery ─────────────────────────────────────
def run_rotation(pts, nrm, axis_xy, pose, sensor, seen, start_i=0,
                 n=N_THETA_FINAL):
    """start_i 부터 회전 진행. lost 시 (True, lost_i), 완주 시 (False, n)."""
    thetas = np.linspace(0.0, 2 * np.pi, n, endpoint=False)
    run = 0
    for i in range(start_i, n):
        p_th, R2 = _rot_z(pts, axis_xy, thetas[i])
        n_th = nrm.copy(); n_th[:, :2] = nrm[:, :2] @ R2.T
        tr, qu = visible_masks(p_th, n_th, pose, sensor)
        fill = sensor.cm2(int(tr.sum()))
        seen |= qu
        run = run + 1 if fill < TRACK_MIN_CM2 else 0
        if run >= CONSEC_LOST:
            return True, i
    return False, n


def scenario(pts, nrm, axis_xy, poses, sensor, replan: bool):
    """밴드(자세열) 순차 실행 + lost/recovery. replan=False → 동일자세 재시도(기존)."""
    seen = np.zeros(len(pts), dtype=bool)
    lost_events, recoveries, reloc_overlaps = 0, 0, []
    for pose in poses:
        i, tries = 0, 0
        while True:
            lost, i_end = run_rotation(pts, nrm, axis_xy, pose, sensor, seen,
                                       start_i=i)
            if not lost:
                break
            lost_events += 1
            tries += 1
            if tries > MAX_RECOVERY:
                return dict(done=False, lost=lost_events, recov=recoveries,
                            cover=float(seen.mean()), overlaps=reloc_overlaps)
            i = max(0, i_end - BACKOFF_FRAMES)
            if replan:
                if seen.sum() < 100:
                    return dict(done=False, lost=lost_events, recov=recoveries,
                                cover=float(seen.mean()), overlaps=reloc_overlaps)
                th_r = 2 * np.pi * i / N_THETA_FINAL
                new_pose, ov = recovery_replan(pts[seen], nrm[seen], axis_xy,
                                               th_r, sensor)
                if new_pose is None:
                    return dict(done=False, lost=lost_events, recov=recoveries,
                                cover=float(seen.mean()), overlaps=reloc_overlaps)
                pose = new_pose
                recoveries += 1
                reloc_overlaps.append(ov)
            # replan=False: 같은 자세로 재시도 (기존 real fallback 동작)
    return dict(done=True, lost=lost_events, recov=recoveries,
                cover=float(seen.mean()), overlaps=reloc_overlaps)


def main():
    sensor = SensorModel()
    scene = json.load(open(os.path.join(DATA, "scene.json")))
    axis_xy = np.array(scene["axis_xy"])
    disc_top = scene["disc_top_z"]
    results = {}
    names = sorted(f[:-4] for f in os.listdir(DATA) if f.endswith(".npz"))

    print(f"\n{'═'*100}\n실험1 — baseline(기존) vs 제안(maximin+밴드)   "
          f"[fill 단위 cm², 최악프레임 기준]\n{'═'*100}")
    print(f"{'object':24}{'h(mm)':>6} | {'base minfill':>12} {'zcov':>5} "
          f"{'IKaz':>5} | {'prop minfill':>12} {'zcov':>5} {'plan':>10} "
          f"{'ovl':>10} {'IK':>4} | {'2shot':>6}")
    print("─" * 100)

    for name in names:
        pts = np.load(os.path.join(DATA, f"{name}.npz"))["pts"].astype(float)
        pts = crop_object_points(pts, axis_xy, disc_top)
        pts = voxel_downsample(pts, sensor.voxel_m)
        nrm = estimate_outward_normals(pts, axis_xy)
        z = pts[:, 2]
        h_mm = (z.max() - z.min()) * 1000
        r_max = float(np.max(np.linalg.norm(pts[:, :2] - axis_xy, axis=1)))

        # baseline = 현 sim pick_phase1_pose 등가
        b_pose = make_view_pose(axis_xy, float((z.min() + z.max()) / 2), 30.0,
                                0.0, 0.25 + r_max)
        b_ev = evaluate_viewpoint(pts, nrm, axis_xy, b_pose, sensor,
                                  n_theta=N_THETA_FINAL)
        b_az = ik_feasible_az(scene, 30.0, 0.25 + r_max,
                              float((z.min() + z.max()) / 2), axis_xy)

        # 제안
        plan = plan_phase1_viewpoints(pts, nrm, axis_xy, sensor)
        p_evs = [evaluate_viewpoint(pts, nrm, axis_xy, p, sensor,
                                    n_theta=N_THETA_FINAL) for p in plan.poses]
        union = np.zeros(len(pts), dtype=bool)
        for e in p_evs:
            union |= e.seen_mask
        p_zcov = _zcov(z, union)
        p_minfill = min(e.min_fill_cm2 for e in p_evs)
        ik_ok = sum(ik_feasible_az(scene, p.el_deg, p.standoff, p.target_z,
                                   axis_xy) is not None for p in plan.poses)

        # 2-shot 충실도 (채점 랭킹 상관)
        m2 = synth_two_shot(pts, nrm, axis_xy, sensor, disc_top)
        rho = float("nan")
        if m2.sum() > 300:
            nrm2 = estimate_outward_normals(pts[m2], axis_xy)
            _, sc_full = candidate_grid_scores(pts, nrm, axis_xy, sensor)
            _, sc_2s = candidate_grid_scores(pts[m2], nrm2, axis_xy, sensor)
            rho = spearman(sc_full, sc_2s)

        ovs = ",".join(f"{o:.2f}" for o in plan.band_overlap_frac) or "-"
        print(f"{name:24}{h_mm:6.0f} | {b_ev.min_fill_cm2:12.1f} "
              f"{b_ev.z_cover_frac:5.2f} {str(b_az):>5} | {p_minfill:12.1f} "
              f"{p_zcov:5.2f} {plan.note:>10} {ovs:>10} "
              f"{ik_ok}/{len(plan.poses)} | {rho:6.2f}")

        results[name] = dict(
            h_mm=h_mm, baseline=dict(minfill=b_ev.min_fill_cm2,
                                     zcov=b_ev.z_cover_frac, ik_az=b_az),
            proposed=dict(minfill=p_minfill, zcov=p_zcov, banded=plan.banded,
                          n_poses=len(plan.poses),
                          overlaps=plan.band_overlap_frac, ik_ok=ik_ok),
            two_shot_rho=rho,
            _plan=[(p.el_deg, p.standoff, p.target_z) for p in plan.poses],
            _b=(30.0, 0.25 + r_max, float((z.min() + z.max()) / 2)))

    print(f"\n{'═'*100}\n실험2 — lost 주입(fill<{TRACK_MIN_CM2}cm² ×{CONSEC_LOST}"
          f"연속) + recovery   [base=동일자세 재시도 / prop=replan+overlap]\n{'═'*100}")
    print(f"{'object':24} | {'base lost':>9} {'done':>5} {'cover':>6} | "
          f"{'prop lost':>9} {'recov':>5} {'done':>5} {'cover':>6} "
          f"{'reloc-ovl(cm²)':>15}")
    print("─" * 100)
    for name in names:
        pts = np.load(os.path.join(DATA, f"{name}.npz"))["pts"].astype(float)
        pts = crop_object_points(pts, axis_xy, disc_top)
        pts = voxel_downsample(pts, sensor.voxel_m)
        nrm = estimate_outward_normals(pts, axis_xy)
        r = results[name]
        b_pose = make_view_pose(axis_xy, r["_b"][2], r["_b"][0], 0.0, r["_b"][1])
        base = scenario(pts, nrm, axis_xy, [b_pose], sensor, replan=False)
        p_poses = [make_view_pose(axis_xy, tz, el, 0.0, s)
                   for el, s, tz in r["_plan"]]
        prop = scenario(pts, nrm, axis_xy, p_poses, sensor, replan=True)
        ovtxt = ",".join(f"{o:.0f}" for o in prop["overlaps"]) or "-"
        print(f"{name:24} | {base['lost']:9d} {str(base['done']):>5} "
              f"{base['cover']:6.2f} | {prop['lost']:9d} {prop['recov']:5d} "
              f"{str(prop['done']):>5} {prop['cover']:6.2f} {ovtxt:>15}")
        results[name]["exp2"] = dict(base=base, prop={k: v for k, v in
                                                      prop.items()})

    for v in results.values():
        v.pop("_plan", None); v.pop("_b", None)
    out = os.path.join(DATA, "validate_results.json")
    json.dump(results, open(out, "w"), indent=1, default=float)
    print(f"\n[saved] {out}")


def _zcov(z, seen, z_bins=24):
    edges = np.linspace(z.min(), z.max() + 1e-9, z_bins + 1)
    b = np.clip(np.digitize(z, edges) - 1, 0, z_bins - 1)
    tot = np.bincount(b, minlength=z_bins)
    hit = np.bincount(b[seen], minlength=z_bins)
    nz = tot > 0
    return float(np.mean((hit[nz] / np.maximum(tot[nz], 1)) > 0.3))


if __name__ == "__main__":
    main()
