"""make_paper_fig2.py — 논문 Fig. 2 생성.

주장: **관측 조건은 광축 회전 ψ 에 (거의) 불변이지만, 기구학 제약은 ψ 에 민감하다.**

  (a) 전회전 중 최소 관측 면적 vs ψ   — 거의 평탄  (P1)
  (b) 정규화 자코비안의 최소 특이값 vs ψ — 크게 변동 (P2)

순수 numpy + 시뮬 씬 파라미터(scene.json). Isaac 불필요.
라벨은 KSMTE 규정에 맞춰 전부 영문·9pt 이하.
"""
from __future__ import annotations
import os, sys, json, math, argparse
import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))
from utils.nbv.lookaround import (                       # noqa: E402
    SensorModel, ViewPose, crop_object_points, voxel_downsample,
    estimate_outward_normals, make_view_pose, visible_masks, _rot_z)
from utils.robot import xarm7_kinematics as kin                # noqa: E402
from utils.robot import view_pose as vp                        # noqa: E402

DATA = os.path.join(os.path.dirname(__file__), "log", "testset_points")
ELS = (30.0, 40.0, 50.0)
PSI = np.arange(-180.0, 181.0, 15.0)
STANDOFF, SEED_DEG = 0.25, [-7.65, -75.61, -8.95, 78.64, 2.81, 126.71, -69.63]


def roll_pose(pose: ViewPose, psi_deg: float) -> ViewPose:
    """광축(카메라 로컬 z) 둘레로 ψ 회전한 ViewPose."""
    c, s = math.cos(math.radians(psi_deg)), math.sin(math.radians(psi_deg))
    Rz = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    return ViewPose(el_deg=pose.el_deg, standoff=pose.standoff,
                    target_z=pose.target_z, eye_w=pose.eye_w,
                    R_wc=pose.R_wc @ Rz)


def min_fill(pts, nrm, axis_xy, pose, sensor, n_theta=24):
    """전회전 중 최소 관측 면적(cm²)."""
    out = []
    for th in np.linspace(0.0, 2 * np.pi, n_theta, endpoint=False):
        p, R2 = _rot_z(pts, axis_xy, th)
        n = nrm.copy(); n[:, :2] = nrm[:, :2] @ R2.T
        tr, _ = visible_masks(p, n, pose, sensor)
        out.append(sensor.cm2(int(tr.sum())))
    return float(np.min(out))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--obj", default="0001_mustard")
    ap.add_argument("--out", default=os.path.join(
        os.path.dirname(__file__), "..", "..", "..", "8_paper", "2_논문작성",
        "figures", "fig_roll_invariance.png"))
    a = ap.parse_args()

    sensor = SensorModel()
    sc = json.load(open(os.path.join(DATA, "scene.json")))
    axis = np.array(sc["axis_xy"], float)
    T_WB, T_EC = np.array(sc["T_WB"], float), np.array(sc["T_EC"], float)
    pts = np.load(os.path.join(DATA, a.obj + ".npz"))["pts"]
    p = voxel_downsample(crop_object_points(pts, axis, float(sc["disc_top_z"])),
                         sensor.voxel_m)
    n = estimate_outward_normals(p, axis)
    tz = float(np.quantile(p[:, 2], 0.5))
    target_w = np.array([axis[0], axis[1], tz])
    seed = np.radians(SEED_DEG)

    area, sig = {}, {}
    for el in ELS:
        base = make_view_pose(axis, tz, el, 0.0, STANDOFF)
        A, S = [], []
        for psi in PSI:
            A.append(min_fill(p, n, axis, roll_pose(base, psi), sensor))
            q, _, _ = vp.solve_view_q(kin, target_w, el, 0.0, STANDOFF, seed, T_EC,
                                      T_WB=T_WB, convention=vp.CAM_USD,
                                      rolls_deg=(float(psi),), n_seed_alt=7,
                                      min_sigma=0.0)
            S.append(np.nan if q is None else kin.sigma_min(q))
        area[el], sig[el] = np.array(A), np.array(S)
        A_ = area[el]; rel = 100.0 * float(np.ptp(A_)) / max(A_.mean(), 1e-9)
        ok = np.isfinite(sig[el])
        print(f"el={el:>4.0f}  area {A_.min():6.1f}~{A_.max():6.1f} cm² (변동 {rel:4.1f}%)   "
              f"sigma {np.nanmin(sig[el]):.3f}~{np.nanmax(sig[el]):.3f} "
              f"(변동 {100*(np.nanmax(sig[el])-np.nanmin(sig[el]))/np.nanmean(sig[el]):.0f}%)  "
              f"IK 실패 {int((~ok).sum())}/{len(PSI)}")

    import matplotlib; matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 8,
                         "axes.labelsize": 8, "xtick.labelsize": 7.5,
                         "ytick.labelsize": 7.5, "legend.fontsize": 7})
    fig, ax = plt.subplots(2, 1, figsize=(3.35, 2.85), dpi=300, sharex=True)
    mk = {30.0: "o", 40.0: "s", 50.0: "^"}
    for el in ELS:
        ax[0].plot(PSI, area[el] / area[el].mean(), marker=mk[el], ms=2.6, lw=0.9,
                   color="0.25" if el == 30 else ("0.5" if el == 40 else "0.72"),
                   label=f"el = {el:.0f}°")
        ax[1].plot(PSI, sig[el], marker=mk[el], ms=2.6, lw=0.9,
                   color="0.25" if el == 30 else ("0.5" if el == 40 else "0.72"))
    ax[0].axhline(1.0, color="k", lw=0.5, ls=":")
    ax[0].set_ylabel("Normalized observed area")
    ax[0].set_ylim(0.82, 1.18)
    ax[0].legend(frameon=False, ncol=3, loc="upper center")
    ax[0].set_title("(a) observation condition", fontsize=8, pad=3)
    ax[1].axhline(vp.DEFAULT_MIN_SIGMA, color="k", ls="--", lw=1.0)
    ax[1].text(-175, vp.DEFAULT_MIN_SIGMA * 1.12, r"$\sigma_{th}$", fontsize=7.5)
    ax[1].set_ylabel(r"$\sigma_{min}$ of scaled Jacobian")
    ax[1].set_xlabel(r"Optical-axis rotation $\psi$ (deg)")
    ax[1].set_title("(b) kinematic constraint", fontsize=8, pad=3)
    ax[1].set_xticks([-180, -90, 0, 90, 180])
    for x in ax:
        x.spines[["top", "right"]].set_visible(False)
    fig.tight_layout(pad=0.3, h_pad=0.8)
    out = os.path.abspath(a.out); os.makedirs(os.path.dirname(out), exist_ok=True)
    fig.savefig(out, bbox_inches="tight")
    print(f"\n[fig] {out}")


if __name__ == "__main__":
    main()
