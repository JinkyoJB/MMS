"""
calib_3sphere_sim.py — 턴테이블 축 calibration **3구(3-sphere) 방법** 검증 (Isaac Sim).

Auto-calibration 방법 1.1 (3구 자동)을 sim ground-truth 로 검증한다.
  - 반경 아는 구 3개를 턴테이블에 부착(rider) → 회전하며 스캔 → known-R 구중심 피팅 → 축.
  - 실물과 동일 경로(`mms.calibrate_turntable_axis`, base 프레임, Artec SLAM 미사용).

기능
----
  1. **축 시각화**: 보정된 EST 축(마젠타) + GT 축(초록)을 sim 에 그려 겹치는지 본다(GUI).
  2. **강인성 perturb**: env MMS_CALIB_PERTURB(m) 로 turntable_demo 위치를 랜덤 이동.
  3. 로그 → scripts/sim/log/  (PLY, 플롯, result.json …)

실행
----
  ~/isaacsim/python.sh scripts/sim/calib_3sphere_sim.py                  # GUI (축 겹침 확인)
  MMS_ISAAC_HEADLESS=1 ~/isaacsim/python.sh scripts/sim/calib_3sphere_sim.py
  MMS_CALIB_PERTURB=0.04 MMS_CALIB_SEED=3 ~/isaacsim/python.sh scripts/sim/calib_3sphere_sim.py        # 위치 랜덤 이동
"""

import os
import sys
import json
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))   # repo root

from mms_artec.system import ArtecMMS, ArtecMMSConfig
from mms_artec.sensor.artec_config import ArtecConfig
from utils import PROJECT_ROOT
from utils.calibration.turntable_axis import axis_error

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from PIL import Image

LOG_DIR = Path(__file__).resolve().parent / "log"
SPHERE_RGB = [(0.90, 0.23, 0.20), (0.23, 0.67, 0.27), (0.23, 0.43, 0.90)]

DISC_PRIM = "/World/ScanTarget/turntable_demo/turntable/turntable"
# perturb 대상은 ScanTarget(스테이션 루트). 그 아래 turntable_demo(턴테이블)와
# Solid_Marble(대상물)이 형제로 있어 함께 이동한다 = 하드웨어팀이 스테이션 통째 이동.
SCANTARGET_PRIM = "/World/ScanTarget"

# ── 설정 ──────────────────────────────────────────────────────────────────────
SPHERE_RADIUS = 0.012
# (dx, dy, z_world): 디스크중심 XY 오프셋 + world z.
# z 간격은 z밴드 분리(≥2·Z_BAND)용. 단, z 폭이 너무 크면 구들이 스캐너 작동거리
# 창(0.2~0.3m) 밖으로 흩어진다 → 0.80/0.84/0.88 (40mm 간격, centroid 0.84) 로 압축.
SPHERE_OFFSETS = [(0.040, 0.000, 0.80),
                  (-0.020, 0.035, 0.84),
                  (0.000, -0.045, 0.88)]
THETAS_DEG = list(range(0, 360, 30))
STANDOFF = 0.25
VIEW_DIR = (0.0, -0.6, 0.8)
DISC_VIEW = (0.0, -0.3, 0.95)     # disc 표면 조준 방향(거의 위에서 내려봄)
R_DISC = 0.06                     # disc 표면 크롭 반경(축 수직거리, m). rim 곡면 제외
DISC_COLLISION_RADIUS = 0.09      # 충돌검사용 턴테이블(원판+프레임) 캡슐 반경(m)
Z_BAND = 0.02

HEADLESS = os.environ.get("MMS_ISAAC_HEADLESS", "0") == "1"
COLLISIONS = os.environ.get("MMS_ISAAC_COLLISIONS", "0") == "1"   # 로봇 물리충돌 on
PERTURB = float(os.environ.get("MMS_CALIB_PERTURB", "0.0"))   # turntable 위치 랜덤이동 한계(m)
SEED = os.environ.get("MMS_CALIB_SEED")


def save_ply(path, points, colors01):
    pts = np.asarray(points); col = (np.asarray(colors01) * 255).astype(np.uint8)
    with open(path, "w") as f:
        f.write("ply\nformat ascii 1.0\n")
        f.write(f"element vertex {len(pts)}\n")
        f.write("property float x\nproperty float y\nproperty float z\n")
        f.write("property uchar red\nproperty uchar green\nproperty uchar blue\n")
        f.write("end_header\n")
        for p, c in zip(pts, col):
            f.write(f"{p[0]:.6f} {p[1]:.6f} {p[2]:.6f} {c[0]} {c[1]} {c[2]}\n")


def perturb_turntable(W, max_xy):
    """ScanTarget(스테이션 루트)를 랜덤 XY(+소량 Z) 이동 → 턴테이블+대상물 통째 이동
    (하드웨어팀 재배치 시뮬). 강인성 테스트용. 적용량 반환."""
    if max_xy <= 0:
        return np.zeros(3)
    if SEED is not None:
        np.random.seed(int(SEED))
    off = np.array([np.random.uniform(-max_xy, max_xy),
                    np.random.uniform(-max_xy, max_xy),
                    np.random.uniform(-max_xy * 0.3, max_xy * 0.3)])
    prim = W.stage.GetPrimAtPath(SCANTARGET_PRIM)
    a = prim.GetAttribute("xformOp:translate")
    if not a or not a.IsValid():
        print("[perturb] ScanTarget translate op 없음 — perturb skip")
        return np.zeros(3)
    cur = np.array(a.Get(), dtype=float)
    a.Set(type(a.Get())(*(cur + off)))
    for _ in range(20):
        W.world.step(render=False)
    print(f"[perturb] ScanTarget +{np.round(off, 4).tolist()} m (턴테이블+대상물 동반)")
    return off


def draw_axis(W, point_world, dir_world, color, name, length=0.5, radius=0.0035, z_center=0.85):
    """축을 sim 에 원기둥으로 그린다 (point 통과, dir 방향). 시각 비교용."""
    from pxr import UsdGeom, Gf
    d = np.asarray(dir_world, float); d = d / (np.linalg.norm(d) + 1e-12)
    p = np.asarray(point_world, float)
    # z_center 높이의 축 위 점을 중심으로
    if abs(d[2]) > 1e-6:
        t = (z_center - p[2]) / d[2]
        center = p + t * d
    else:
        center = p
    cyl = UsdGeom.Cylinder.Define(W.stage, name)
    cyl.CreateRadiusAttr(float(radius))
    cyl.CreateHeightAttr(float(length))
    cyl.CreateAxisAttr("Z")
    cyl.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    rot = Gf.Rotation(Gf.Vec3d(0, 0, 1), Gf.Vec3d(float(d[0]), float(d[1]), float(d[2])))
    q = rot.GetQuat()
    xf = UsdGeom.Xformable(cyl)
    xf.AddTranslateOp().Set(Gf.Vec3d(float(center[0]), float(center[1]), float(center[2])))
    xf.AddOrientOp().Set(Gf.Quatf(q.GetReal(), Gf.Vec3f(*[float(v) for v in q.GetImaginary()])))


def main():
    cfg = ArtecMMSConfig(
        artec=ArtecConfig(),
        turntable_frame_yaml=str(PROJECT_ROOT / "config/calibration/turntable_frame.yaml"),
        sensor_frames_yaml=str(PROJECT_ROOT / "config/sensor_frames.yaml"),
        T_EC_key="T_EC_artec", backend="isaac", isaac_headless=HEADLESS,
        isaac_robot_collisions=COLLISIONS,
    )
    with ArtecMMS(cfg) as mms:
        W = mms.sensor._world
        from pxr import UsdGeom
        from mms_artec.backends.isaac.calib_fixture import prepare_sim_fixture

        # ── 강인성: turntable 위치 랜덤 이동 (create_hardware 전 — 중심 재인식되게) ──
        applied = perturb_turntable(W, PERTURB)

        robot, tt = mms.create_hardware()
        robot.go_home(sensor="artec", confirm=False)
        UsdGeom.Imageable(
            W.stage.GetPrimAtPath("/World/ScanTarget/Solid_Marble")).MakeInvisible()

        # 가상 구 fixture 부착 + 조준
        fx = prepare_sim_fixture(W, tt, SPHERE_OFFSETS, SPHERE_RADIUS,
                                 standoff=STANDOFF, view_dir=VIEW_DIR)
        print(f"[calib] 스캐너 조준 off-axis {fx['off_axis_deg']:.2f}°")

        LOG_DIR.mkdir(parents=True, exist_ok=True)
        try:
            rgba = W.ensure_camera().get_rgba()
            if rgba is not None and getattr(rgba, "size", 0) > 0:
                Image.fromarray(rgba[:, :, :3].astype(np.uint8)).save(LOG_DIR / "cam_rgb_aim.png")
        except Exception:
            pass

        # ── 실물과 동일 통일 calibration ────────────────────────────────────
        res = mms.calibrate_turntable_axis(
            robot, tt, sphere_radius=SPHERE_RADIUS, sphere_z_bands=fx["z_bands_base"],
            thetas_deg=THETAS_DEG, z_band=Z_BAND, return_clouds=True)
        tracks = res["tracks"]; axis_pt = res["axis_point"]; axis_dir = res["axis_dir"]
        dir_err, pos_err = axis_error(axis_pt, axis_dir, fx["gt_point_base"], fx["gt_dir_base"])

        for i, t in enumerate(tracks):
            print(f"[calib] sphere{i}: {len(t)} 관측")
        print("=" * 56)
        print(f"perturb={np.round(applied, 4).tolist()} m")
        print(f"EST 축(base): point={np.round(axis_pt, 4).tolist()}  dir={np.round(axis_dir, 4).tolist()}")
        print(f"  방향 오차 = {dir_err:.3f} deg")
        print(f"  위치 오차 = {pos_err*1000:.3f} mm   (base 프레임)")
        print("=" * 56)

        # 좌표 준비 (base→world)
        dctr, _ = W.prim_world_pose(DISC_PRIM)
        gt_pt_world = np.array([dctr[0], dctr[1], 0.0]); gt_dir_world = np.array([0, 0, 1.0])
        bpos, bR = W.base_world_pose()
        T_bw = np.eye(4); T_bw[:3, :3] = bR; T_bw[:3, 3] = bpos   # base→world
        est_pt_world = (T_bw @ np.array([*axis_pt, 1.0]))[:3]
        est_dir_world = bR @ np.asarray(axis_dir, float)

        # ── disc 표면 평면 스캔 (충돌회피 + 대상물 기준높이) ──────────────────
        # 축 XY 를 안 뒤 스캐너를 disc 로 내려 조준 → disc 점군 → 평면 피팅 →
        # 표면 높이(F0 원점) + 법선 교차검증. 구는 가려 표면만 잡는다.
        # ★ 축 시각화 원기둥을 그리기 '전에' 캡처(원기둥이 disc 캡처를 오염시킴).
        surf = None
        try:
            for i in range(len(SPHERE_OFFSETS)):                 # 구 숨김(표면만 캡처)
                UsdGeom.Imageable(W.stage.GetPrimAtPath(f"/World/calib_sphere_{i}")).MakeInvisible()
            disc_tgt = np.array([est_pt_world[0], est_pt_world[1], float(dctr[2])])
            dv = np.asarray(DISC_VIEW, float); dv /= np.linalg.norm(dv)
            off_d = W.look_at_camera(target_world=disc_tgt, cam_pos_world=disc_tgt + dv * STANDOFF)
            disc_pts = mms.sensor.capture_points_base(robot, mms._T_EC)
            # 회전축에 수직거리 < R_DISC 만 남겨 floor/배경 제거
            v = disc_pts - np.asarray(axis_pt); ad = np.asarray(axis_dir)
            perp = v - np.outer(v @ ad, ad)
            disc_pts = disc_pts[np.linalg.norm(perp, axis=1) < R_DISC]
            if len(disc_pts) > 8000:                              # 다운샘플(평면피팅 충분)
                disc_pts = disc_pts[np.random.choice(len(disc_pts), 8000, replace=False)]
            print(f"[calib] disc 표면 조준 off-axis {off_d:.2f}°  표면점 {len(disc_pts)}개")
            surf = mms.disc_surface_frame(disc_pts, axis_pt, axis_dir)
            # GT 표면(disc 중심) base z 와 비교
            gt_surf_base = (np.linalg.inv(T_bw) @ np.array([*disc_tgt, 1.0]))[2]
            print("=" * 56)
            print(f"disc 표면높이(base z) = {surf['surface_height']:.4f} m  "
                  f"(GT≈{gt_surf_base:.4f}, Δ={abs(surf['surface_height']-gt_surf_base)*1000:.2f}mm)")
            print(f"  평면법선 vs 구-축 = {surf['normal_axis_angle_deg']:.3f} deg  "
                  f"평면 RMS = {surf['plane_rms']*1000:.3f} mm")
            print("=" * 56)
            # sim 에 표면 disk 그리기(축 위 표면점, 법선=축)
            sp_w = (T_bw @ np.array([*surf["surface_point"], 1.0]))[:3]
            draw_axis(W, sp_w, est_dir_world, (0.2, 0.6, 1.0), "/World/calib_disc_plane",
                      length=0.004, radius=R_DISC, z_center=float(sp_w[2]))
            for i in range(len(SPHERE_OFFSETS)):                 # 구 다시 표시
                UsdGeom.Imageable(W.stage.GetPrimAtPath(f"/World/calib_sphere_{i}")).MakeVisible()
        except Exception as e:
            print(f"[calib][WARN] disc 표면 스캔 실패(무시): {e}")

        # ── sim 에 GT/EST 축 그리기 (disc 스캔 후) ───────────────────────────
        draw_axis(W, gt_pt_world, gt_dir_world, (0.1, 0.9, 0.1), "/World/calib_axis_GT",
                  radius=0.005)
        draw_axis(W, est_pt_world, est_dir_world, (1.0, 0.1, 1.0), "/World/calib_axis_EST",
                  radius=0.0025)
        print("[calib] sim 에 축 그림: GT(초록 굵게) / EST(마젠타 얇게) — 겹치면 일치")

        # ── 자세별 충돌 쿼리 데모 (calibration 결과로 월드 구성) ─────────────
        # utils.collision 공용 — real/sim 동일 API. 충돌 월드는 calibration 의
        # 표면+축(base)으로, 로봇 캡슐은 실제 링크포즈(sim=USD)로.
        col = None
        try:
            if surf is not None:
                world = mms.turntable_collision_world(
                    surf["surface_point"], axis_dir, disc_radius=DISC_COLLISION_RADIUS)
                r_aim = mms.check_pose_collision(robot, world)            # 현재(조준) 자세
                NEW = robot.HOME_JOINTS_DEG["artec"]
                OLD = [0.0, -18.4, 0.0, 70.6, 0.0, 60.0, -45.0]          # 옛(충돌) 홈
                r_new = mms.check_pose_collision(robot, world, q=np.radians(NEW))
                r_old = mms.check_pose_collision(robot, world, q=np.radians(OLD))
                print("=" * 56)
                print(f"[충돌검사] 조준자세 collide={r_aim.collide} "
                      f"여유={r_aim.min_clearance*1000:.0f}mm")
                print(f"[충돌검사] 새 홈    collide={r_new.collide} "
                      f"여유={r_new.min_clearance*1000:.0f}mm  ← 안전")
                print(f"[충돌검사] 옛 홈    collide={r_old.collide} "
                      f"여유={r_old.min_clearance*1000:.0f}mm  ← 충돌 검출")
                print("=" * 56)
                col = {"aim": [r_aim.collide, r_aim.min_clearance * 1000],
                       "new_home": [r_new.collide, r_new.min_clearance * 1000],
                       "old_home": [r_old.collide, r_old.min_clearance * 1000]}
                robot.go_home(sensor="artec", confirm=False)             # GUI 위해 안전자세 복귀
        except Exception as e:
            print(f"[calib][WARN] 충돌검사 실패(무시): {e}")

        # ── 로그 ────────────────────────────────────────────────────────────
        seg_pts, seg_col = [], []
        for _deg, i, band in res["clouds"]:
            b = band if len(band) <= 1500 else band[np.random.choice(len(band), 1500, replace=False)]
            seg_pts.append(b); seg_col.append(np.tile(SPHERE_RGB[i], (len(b), 1)))
        if seg_pts:
            save_ply(LOG_DIR / "recognized_spheres.ply", np.vstack(seg_pts), np.vstack(seg_col))
        rows = ["sphere,cx,cy,cz"]
        for i, t in enumerate(tracks):
            for c in t:
                rows.append(f"{i},{c[0]:.6f},{c[1]:.6f},{c[2]:.6f}")
        (LOG_DIR / "sphere_centers.csv").write_text("\n".join(rows))

        gx, gy = float(fx["gt_point_base"][0]), float(fx["gt_point_base"][1])
        fig, ax = plt.subplots(figsize=(6, 6))
        for i, t in enumerate(tracks):
            if t:
                C = np.array(t)
                ax.scatter(C[:, 0], C[:, 1], s=30, color=SPHERE_RGB[i], label=f"sphere{i} ({len(t)})")
        ax.scatter([gx], [gy], marker="x", s=200, c="k", label="GT axis")
        ax.scatter([axis_pt[0]], [axis_pt[1]], marker="o", s=130,
                   facecolors="none", edgecolors="m", linewidths=2, label="EST axis")
        ax.set_aspect("equal"); ax.grid(True, alpha=0.3)
        ax.set_xlabel("X base (m)"); ax.set_ylabel("Y base (m)")
        ax.set_title(f"3-sphere calib (base frame)\ndir err={dir_err:.3f}deg  pos err={pos_err*1000:.2f}mm")
        ax.legend(fontsize=8)
        fig.tight_layout(); fig.savefig(LOG_DIR / "center_tracks_topview.png", dpi=130)
        plt.close(fig)

        (LOG_DIR / "result.json").write_text(json.dumps({
            "frame": "robot_base", "method": "3sphere",
            "perturb_m": [float(v) for v in applied],
            "gt_axis_point": [float(v) for v in fx["gt_point_base"]],
            "gt_axis_dir": [float(v) for v in fx["gt_dir_base"]],
            "est_axis_point": [float(v) for v in axis_pt],
            "est_axis_dir": [float(v) for v in axis_dir],
            "dir_err_deg": float(dir_err), "pos_err_mm": float(pos_err*1000),
            "n_obs_per_sphere": [len(t) for t in tracks],
            "disc_surface": None if surf is None else {
                "surface_height_base_m": float(surf["surface_height"]),
                "surface_point_base": [float(v) for v in surf["surface_point"]],
                "plane_normal_base": [float(v) for v in surf["plane_normal"]],
                "normal_axis_angle_deg": float(surf["normal_axis_angle_deg"]),
                "plane_rms_mm": float(surf["plane_rms"] * 1000),
            },
            "collision": None if col is None else {
                "aim_collide": bool(col["aim"][0]),       "aim_clear_mm": float(col["aim"][1]),
                "new_home_collide": bool(col["new_home"][0]), "new_home_clear_mm": float(col["new_home"][1]),
                "old_home_collide": bool(col["old_home"][0]), "old_home_clear_mm": float(col["old_home"][1]),
            },
        }, indent=2, ensure_ascii=False))
        print(f"[calib] 로그 저장 → {LOG_DIR}")

        # ── GUI: 축 겹침을 눈으로 확인 (창 닫을 때까지 유지) ────────────────
        if not HEADLESS:
            print("[calib] GUI — GT/EST 축 겹침 확인. 창 닫으면 종료.")
            while W.is_running():
                W.world.step(render=True)

    from mms_artec.backends import shutdown_isaac_world
    shutdown_isaac_world()


if __name__ == "__main__":
    main()
