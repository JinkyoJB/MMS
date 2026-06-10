"""
calib_rim_sim.py — 턴테이블 축/평면 calibration **방법 1.2 (rim 3점 클릭)** 검증·실행 (Isaac Sim).

디스크 rim(원형 윗면 가장자리)을 스캐너로 보고, 사용자가 OpenCV 창에서 3점 이상 클릭 →
3D 원 피팅 → **중심(축 XY)·법선(축 방향)·평면(표면)** 을 한 번에 얻는다(방법 1.1=축만 vs
1.2=축+표면, 상보적). 코어는 PhoXi 와 공용:
  utils.calibration.rim_picker      — 클릭 UI (cv2)
  utils.calibration.turntable_frame — fit_circle_3d / build_T_B_F0
Isaac 어댑터(mms.sensor.capture_organized)가 (intensity, organized_pts[base mm], T_CB=I)를 준다.

실행
----
  # ★ 수동 클릭(2단계): Isaac python 엔 GUI 가 없으므로 캡처와 클릭을 분리한다.
  ~/isaacsim/python.sh scripts/sim/calib_rim_sim.py        # 1) 캡처 → log/rim_capture.npz
  python scripts/sim/rim_click_offline.py                  # 2) 일반 python(GUI)에서 클릭+피팅

  # 자동 검증(클릭 합성, 사람 없이 GT 비교):
  MMS_RIM_AUTO=1 ~/isaacsim/python.sh scripts/sim/calib_rim_sim.py   # GUI sim(축 시각화)
  MMS_ISAAC_HEADLESS=1 MMS_RIM_AUTO=1 ... calib_rim_sim.py           # 헤드리스(수치만)
"""

import os
import sys
import json
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from mms_artec.system import ArtecMMS, ArtecMMSConfig
from mms_artec.sensor.artec_config import ArtecConfig
from utils import PROJECT_ROOT
from utils.calibration.turntable_frame import fit_circle_3d, build_T_B_F0
from utils.calibration.turntable_axis import axis_error

from PIL import Image

LOG_DIR = Path(__file__).resolve().parent / "log"
DISC_PRIM = "/World/ScanTarget/turntable_demo/turntable/turntable"

STANDOFF = 0.25
DISC_VIEW = (0.0, -0.3, 0.95)      # 디스크 위에서 내려다보는 조준
RIM_R_AUTO = 0.05                  # 자동모드 합성 rim 반경(작동거리 FOV 안)
N_AUTO = 12                        # 자동모드 합성 클릭 수

HEADLESS = os.environ.get("MMS_ISAAC_HEADLESS", "0") == "1"
AUTO = HEADLESS or os.environ.get("MMS_RIM_AUTO", "0") == "1"


def draw_axis(W, point_world, dir_world, color, name, length=0.5, radius=0.0035, z_center=0.85):
    from pxr import UsdGeom, Gf
    d = np.asarray(dir_world, float); d = d / (np.linalg.norm(d) + 1e-12)
    p = np.asarray(point_world, float)
    center = p + ((z_center - p[2]) / d[2]) * d if abs(d[2]) > 1e-6 else p
    cyl = UsdGeom.Cylinder.Define(W.stage, name)
    cyl.CreateRadiusAttr(float(radius)); cyl.CreateHeightAttr(float(length))
    cyl.CreateAxisAttr("Z"); cyl.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    q = Gf.Rotation(Gf.Vec3d(0, 0, 1), Gf.Vec3d(*[float(v) for v in d])).GetQuat()
    xf = UsdGeom.Xformable(cyl)
    xf.AddTranslateOp().Set(Gf.Vec3d(*[float(v) for v in center]))
    xf.AddOrientOp().Set(Gf.Quatf(q.GetReal(), Gf.Vec3f(*[float(v) for v in q.GetImaginary()])))


def synth_clicks(W, cam, org, dctr):
    """자동모드: GT rim 원(world)을 픽셀로 투영 → organized 룩업 → base 3D 클릭 합성."""
    ringw = [[dctr[0] + RIM_R_AUTO * np.cos(a), dctr[1] + RIM_R_AUTO * np.sin(a), dctr[2]]
             for a in np.linspace(0, 2 * np.pi, N_AUTO, endpoint=False)]
    uvs = np.asarray(cam.get_image_coords_from_world_points(np.array(ringw)))
    pts = []
    H, Wd = org.shape[:2]
    for uu, vv in uvs:
        iu, iv = int(round(uu)), int(round(vv))
        if 0 <= iv < H and 0 <= iu < Wd and not np.all(org[iv, iu] == 0):
            pts.append(org[iv, iu] / 1000.0)
    return np.array(pts)


def main():
    cfg = ArtecMMSConfig(
        artec=ArtecConfig(),
        turntable_frame_yaml=str(PROJECT_ROOT / "config/calibration/turntable_frame.yaml"),
        sensor_frames_yaml=str(PROJECT_ROOT / "config/sensor_frames.yaml"),
        T_EC_key="T_EC_artec", backend="isaac", isaac_headless=HEADLESS,
    )
    with ArtecMMS(cfg) as mms:
        W = mms.sensor._world
        from pxr import UsdGeom
        robot, tt = mms.create_hardware()
        robot.go_home(sensor="artec", confirm=False)
        UsdGeom.Imageable(
            W.stage.GetPrimAtPath("/World/ScanTarget/Solid_Marble")).MakeInvisible()

        # 디스크 위에서 조준 → 조직화 캡처
        dctr, _ = W.prim_world_pose(DISC_PRIM)
        tgt = np.array([dctr[0], dctr[1], dctr[2]])
        dv = np.asarray(DISC_VIEW, float); dv /= np.linalg.norm(dv)
        off = W.look_at_camera(target_world=tgt, cam_pos_world=tgt + dv * STANDOFF)
        print(f"[rim] 디스크 조준 off-axis {off:.2f}°")
        gray, org, T_CB = mms.sensor.capture_organized()

        # GT (base 프레임) — 오프라인 비교용
        bpos, bR = W.base_world_pose()
        gt_c = bR.T @ (tgt - bpos)
        gt_n = bR.T @ np.array([0.0, 0.0, 1.0])

        # ── 캡처 저장 (오프라인 클릭용) ─────────────────────────────────────
        # ★ Isaac python 엔 GUI 툴킷이 없다(cv2 headless·tkinter/Qt 없음) → 인터랙티브
        #   클릭 불가. 캡처만 저장하고, 클릭+피팅은 GUI 있는 일반 python 에서
        #   scripts/sim/rim_click_offline.py 로 한다(Isaac 불필요).
        LOG_DIR.mkdir(parents=True, exist_ok=True)
        Image.fromarray(gray).save(LOG_DIR / "rim_intensity.png")
        np.savez(LOG_DIR / "rim_capture.npz", intensity=gray, organized_pts=org,
                 T_CB=T_CB, gt_center_base=gt_c, gt_normal_base=gt_n)
        print(f"[rim] 캡처 저장 → {LOG_DIR/'rim_capture.npz'}  (+ rim_intensity.png)")

        if not AUTO:
            print("\n[rim] ※ Isaac python 은 GUI(cv2/tk/Qt) 가 없어 클릭 창을 못 연다.")
            print("      GUI 있는 일반 python 에서 아래를 실행해 직접 클릭하세요:")
            print(f"        python scripts/sim/rim_click_offline.py {LOG_DIR/'rim_capture.npz'}")
            from mms_artec.backends import shutdown_isaac_world; shutdown_isaac_world(); return

        # ── (자동 검증) 클릭 합성 → 피팅 ────────────────────────────────────
        pts_B = synth_clicks(W, W.ensure_camera(), org, dctr)
        print(f"[rim] (자동) rim 점 {len(pts_B)}개 합성")
        if len(pts_B) < 3:
            print("[rim] 3점 미만 — 중단");
            from mms_artec.backends import shutdown_isaac_world; shutdown_isaac_world(); return

        # ── 3D 원 피팅 → T_B_F0 ─────────────────────────────────────────────
        center_B, normal_B, radius, residual = fit_circle_3d(pts_B)
        T_B_F0 = build_T_B_F0(center_B, normal_B)
        dir_err, pos_err = axis_error(center_B, normal_B, gt_c, gt_n)
        print("=" * 56)
        print(f"[rim] 원피팅: 반경={radius*1000:.1f}mm  RMS={residual*1000:.3f}mm  ({len(pts_B)}점)")
        print(f"  축 방향오차 = {dir_err:.3f} deg")
        print(f"  중심 XY오차 = {pos_err*1000:.3f} mm")
        print(f"  표면높이(base z) = {center_B[2]*1000:.1f} mm  (GT≈{gt_c[2]*1000:.1f})")
        print("=" * 56)

        # ── sim 시각화: GT(초록)/EST(마젠타) 축 + 표면 disk + rim 점 ─────────
        T_bw = np.eye(4); T_bw[:3, :3] = bR; T_bw[:3, 3] = bpos
        est_c_w = (T_bw @ np.array([*center_B, 1.0]))[:3]
        est_n_w = bR @ np.asarray(normal_B, float)
        gt_c_w = np.array([dctr[0], dctr[1], dctr[2]])
        draw_axis(W, gt_c_w, np.array([0, 0, 1.0]), (0.1, 0.9, 0.1), "/World/rim_axis_GT", radius=0.005)
        draw_axis(W, est_c_w, est_n_w, (1.0, 0.1, 1.0), "/World/rim_axis_EST", radius=0.0025)
        draw_axis(W, est_c_w, est_n_w, (0.2, 0.6, 1.0), "/World/rim_plane",
                  length=0.004, radius=float(radius), z_center=float(est_c_w[2]))
        for i, p in enumerate(pts_B):
            pw = (T_bw @ np.array([*p, 1.0]))[:3]
            s = UsdGeom.Sphere.Define(W.stage, f"/World/rim_pt_{i}")
            s.GetRadiusAttr().Set(0.004)
            s.GetExtentAttr().Set([(-0.004,)*3, (0.004,)*3])
            UsdGeom.Xformable(s).AddTranslateOp().Set(tuple(float(v) for v in pw))
        print("[rim] sim 에 축 그림: GT(초록)/EST(마젠타) + 표면 disk(파랑) + rim 점")

        (LOG_DIR / "rim_result.json").write_text(json.dumps({
            "frame": "robot_base", "method": "rim_3point", "auto": bool(AUTO),
            "n_points": int(len(pts_B)),
            "center_base_m": [float(v) for v in center_B],
            "normal_base": [float(v) for v in normal_B],
            "radius_mm": float(radius * 1000), "rms_mm": float(residual * 1000),
            "dir_err_deg": float(dir_err), "pos_err_mm": float(pos_err * 1000),
            "surface_height_base_mm": float(center_B[2] * 1000),
            "T_B_F0": [[float(v) for v in row] for row in T_B_F0.tolist()],
        }, indent=2, ensure_ascii=False))
        print(f"[rim] 로그 저장 → {LOG_DIR}")

        if not HEADLESS and AUTO:
            print("[rim] GUI — 축/평면 확인. 창 닫으면 종료.")
            while W.is_running():
                W.world.step(render=True)

    from mms_artec.backends import shutdown_isaac_world
    shutdown_isaac_world()


if __name__ == "__main__":
    main()
