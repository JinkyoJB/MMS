try:
    import msvcrt                       # Windows 전용 (실물 환경)
except ImportError:
    msvcrt = None                       # Linux/Isaac — 키 입력 폴링 비활성
import os
import traceback
from datetime import datetime

import numpy as np

from utils import PROJECT_ROOT

# 모든 output 파일에 같은 타임스탬프(_YYYYMMDD_HHMMSS) 붙여 run 별 구분.
RUN_TS = datetime.now().strftime("%Y%m%d_%H%M%S")
from mms_artec.system import ArtecMMS, ArtecMMSConfig, ArtecProcessSettings
from mms_artec.sensor.artec_config import ArtecConfig   # 바인딩 비의존(경량)

# ── 백엔드 선택 ───────────────────────────────────────────────────────
#   "real"  → 실물 xArm + 턴테이블 + Artec 스캐너 (Windows)
#   "isaac" → Isaac Sim 시뮬레이션 (옆에 실물 없이 개발)
BACKEND = "isaac"
# isaac GUI 표시 여부. 환경변수 MMS_ISAAC_HEADLESS=1 로 헤드리스 강제(서버/CI).
ISAAC_HEADLESS = os.environ.get("MMS_ISAAC_HEADLESS", "0") == "1"

# ── 턴테이블 축 calibration ───────────────────────────────────────────
# True 면 스캔 전에 구 fixture 로 T_B_F0(턴테이블 축)를 재calibration.
# 하드웨어팀이 턴테이블/로봇을 옮겼을 때 사용. (env MMS_RUN_CALIB=1 로도 켜짐)
RUN_CALIBRATION = os.environ.get("MMS_RUN_CALIB", "0") == "1"
CALIB_SPHERE_RADIUS = 0.012                 # 구 반경 (m)
# isaac: 디스크중심 XY 오프셋 + world z (가상 fixture 자동 설치)
CALIB_SPHERE_OFFSETS = [(0.040, 0.000, 0.78),
                        (-0.020, 0.035, 0.84),
                        (0.000, -0.045, 0.90)]
# real: 물리 fixture 의 base-프레임 구 z 높이(설계/실측값). None 이면 real calibration skip.
CALIB_SPHERE_Z_BANDS = None

# 스캔 settings 클래스들은 Artec 바인딩/open3d 에 의존 → real 환경에서만 import 가능.
# isaac(Phase A) 에선 없어도 모션/제어 개발이 가능하도록 try/except 로 보호.
try:
    from mms_artec.nbv.artec_scan_session import ArtecScanSessionSettings
    from mms_artec.nbv.artec_streaming_scan_session import ArtecStreamingScanSessionSettings
    from mms_artec.nbv.artec_multipass_scan_session import (
        ArtecMultiPassScanSessionSettings,
        make_axis_physical_rotations,
    )
    _SCAN_SETTINGS_AVAILABLE = True
except Exception as _e:               # noqa: BLE001
    print(f"[main] ⓘ scan settings import 불가 ({type(_e).__name__}) — "
          f"스캐너 파이프라인 비활성 (Phase A 모션/제어만).")
    _SCAN_SETTINGS_AVAILABLE = False

# ── 하드웨어 ──────────────────────────────────────────────────────────
ROBOT_IP        = "192.168.1.210"
TURNTABLE_IP    = "192.168.0.10"
TURNTABLE_BD_ID = 0

# ── 모션 속도 ──────────────────────────────────────────────────────────
ROBOT_SPEED_DEG_S   = 15.0                  # 로봇 (실제로는 Phase 1 에서 안 움직이지만 settings 보관)
TURNTABLE_VEL_RAD_S = np.radians(10.0)      # 턴테이블

# ── MMS Artec 설정 ────────────────────────────────────────────────────
CFG = ArtecMMSConfig(
    artec=ArtecConfig(
        serial_number=None,                 # None → 첫 번째 스캐너 (Spider SP.10.36181288)
        capture_texture=True,
        target_interval_s=0.0,
    ),
    turntable_frame_yaml=str(PROJECT_ROOT / "config/calibration/turntable_frame.yaml"),
    sensor_frames_yaml=str(PROJECT_ROOT / "config/sensor_frames.yaml"),
    T_EC_key="T_EC_artec",                  # 2026-04-29 hand-eye 결과
    # ── 백엔드 ──────────────────────────────────────────────────────────
    backend=BACKEND,
    robot_ip=ROBOT_IP,
    turntable_ip=TURNTABLE_IP,
    turntable_bd_id=TURNTABLE_BD_ID,
    isaac_headless=ISAAC_HEADLESS,
)

DEV_MODE = True

# 스캔 settings 는 Artec 바인딩/open3d 가 있을 때만 구성한다 (real 환경).
# isaac(Phase A) 에선 None — 모션/제어 개발만, 스캐너는 Phase B.
PROCESS_SETTINGS = None
if _SCAN_SETTINGS_AVAILABLE:
    # ── Streaming Phase 1 (Artec IScanningProcedure 기반, 연속 회전) ──────
    STREAM_SETTINGS = ArtecStreamingScanSessionSettings(
        rotation_duration_s=30.0,         # 30초에 한 바퀴
        rotation_overshoot_deg=5.0,
        target_fps=None,                  # None = scanner.max_fps()
        capture_texture=True,
        ignore_registration_errors=True,
        preview_settle_s=1.5,
        post_record_settle_s=0.5,
        reset_to_zero_first=True,
        timeline_csv_path=str(PROJECT_ROOT / f"output/artec_phase1_{RUN_TS}_timeline.csv"),
    )

    # ── (Legacy) Discrete Phase 1 ────────────────────────────────────────
    SCAN_SETTINGS = ArtecScanSessionSettings(
        phase1_enabled=True,
        phase1_theta_step_deg=15.0,
        phase1_dwell_s=0.40,
        phase1_show_progress=True,
        phase1_wait_window_close=True,
        phase2_enabled=False,
        phase2_K_max=0,
        confirm_each_move=False,
        robot_speed_deg_s=ROBOT_SPEED_DEG_S,
        turntable_vel_rad_s=TURNTABLE_VEL_RAD_S,
    )

    # ── Multi-pass: Phase 1 + Phase 2 통합 (3-pose default) ───────────────
    POSE_ROTATIONS = make_axis_physical_rotations("y", [0.0, 90.0, 180.0])
    MULTIPASS_SETTINGS = ArtecMultiPassScanSessionSettings(
        streaming_settings=STREAM_SETTINGS,
        pose_physical_rotations=POSE_ROTATIONS,
        max_passes=8,
        prompt_before_first_pass=True,
        prompt_between_passes=True,
        prompt_on_tracking_lost=True,
        auto_recovery_enabled=True,
        max_recovery_retries=3,
        safe_back_margin_deg=10.0,
        recovery_robot_speed_deg_s=10.0,
        recovery_turntable_vel_rad_s=float(np.radians(30.0)),
        recovery_elevation_offsets_deg=[-5.0, 0.0, 5.0],
        recovery_elevation_fine_search_enabled=False,
        enable_live_viewer=True,
        probe_debug_dump=True,
    )

    PROCESS_SETTINGS = ArtecProcessSettings(
        dev_mode=DEV_MODE,
        use_streaming_scan=True,
        scan_settings=SCAN_SETTINGS,
        streaming_scan_settings=STREAM_SETTINGS,
        multipass_settings=MULTIPASS_SETTINGS,
        do_serial_registration=False,
        do_global_registration=True,
        fusion="poisson",
        do_outliers_removal=True,
        do_small_objects_filter=True,
        do_simplify=False,
        do_texturize=True,
        export_obj_path=str(PROJECT_ROOT / f"output/artec_phase1_{RUN_TS}.obj"),
        export_sproj_path=str(PROJECT_ROOT / f"output/artec_phase1_{RUN_TS}.sproj"),
    )


def _show_textured_obj(obj_path: str, title: str) -> bool:
    """
    Texturize 된 export OBJ(.obj + .mtl + 텍스처 png)를 텍스처 그대로 표시.

    성공하면 True. 파일 없음 / 텍스처 없음 / 로드 실패면 False (호출측이
    단색 fallback 으로 넘어감).
    """
    import os

    import open3d as o3d

    if not obj_path or not os.path.isfile(obj_path):
        print(f"[main] textured OBJ 없음 ({obj_path}) — 단색 fallback")
        return False
    try:
        mesh = o3d.io.read_triangle_mesh(obj_path, enable_post_processing=True)
    except Exception as e:
        print(f"[main] OBJ 로드 실패 ({type(e).__name__}: {e}) — 단색 fallback")
        return False

    if len(mesh.triangles) == 0:
        print("[main] OBJ 에 삼각형 없음 — 단색 fallback")
        return False

    mesh.compute_vertex_normals()
    has_tex = mesh.has_textures() and mesh.has_triangle_uvs()
    print(f"[main] textured OBJ: verts={len(mesh.vertices):,} "
          f"faces={len(mesh.triangles):,} textured={has_tex}")
    if not has_tex:
        # 텍스처 없으면 단색 fallback 이 더 깔끔
        return False

    axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.05)
    disc = o3d.geometry.TriangleMesh.create_cylinder(
        radius=0.12, height=0.001, resolution=64)
    disc.paint_uniform_color([0.20, 0.35, 0.80])
    disc.compute_vertex_normals()

    print("[main] 텍스처 입은 최종 mesh 표시 — Q/ESC 로 닫기.")
    # draw_geometries(legacy) 는 triangle-uv 텍스처를 렌더함.
    o3d.visualization.draw_geometries(
        [mesh, axis, disc], window_name=f"{title} (textured)",
        width=1280, height=720, mesh_show_back_face=True,
    )
    return True


def _show_composite_mesh(result, mms, title: str = "Artec Phase 1",
                         obj_path: str | None = None) -> None:
    """
    1순위: texturize 된 export OBJ 를 텍스처 그대로 표시.
    2순위: composite mesh (단색).
    3순위: IModel frame transformations 으로 vertices 누적 PointCloud (단색).
    """
    import open3d as o3d

    # ── 1순위: 텍스처 OBJ ─────────────────────────────────────────────
    if obj_path is not None and _show_textured_obj(obj_path, title):
        return

    model = result.model
    ctx = result.ctx                          # streaming 모드면 None
    geoms: list = []

    if model.has_final_mesh():
        verts = model.final_vertices().astype(np.float64) / 1000.0   # mm → m
        faces = model.final_faces().astype(np.int32)
        print(f"\n[main] composite mesh  verts={len(verts):,}  faces={len(faces):,}")
        mesh = o3d.geometry.TriangleMesh()
        mesh.vertices = o3d.utility.Vector3dVector(verts)
        mesh.triangles = o3d.utility.Vector3iVector(faces)
        mesh.compute_vertex_normals()
        mesh.paint_uniform_color([0.85, 0.30, 0.25])
        geoms.append(mesh)
    else:
        # composite 없음 → IModel 안 frame transformations 으로 누적
        print("\n[main] composite mesh 없음 — frame transformations 으로 누적 표시")
        all_pts = []
        n_scans = model.scan_count()
        for s_i in range(n_scans):
            scan = model.get_scan(s_i)
            for f_i in range(scan.frame_count()):
                fmh = scan.get_frame(f_i)
                v_mm = fmh.vertices().astype(np.float64)
                # SDK 가 set 한 frame transformation 사용 (streaming 모드 = SDK SLAM 결과)
                try:
                    T = scan.get_frame_transformation(f_i)
                    v_scan = v_mm @ T[:3, :3].T + T[:3, 3]
                    v_m = v_scan / 1000.0
                except Exception:
                    v_m = v_mm / 1000.0
                all_pts.append(v_m)
        if all_pts:
            all_pts = np.vstack(all_pts)
            print(f"  누적 pts (O 프레임) = {len(all_pts):,}")
            # 너무 많으면 voxel down 으로 가볍게
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(all_pts)
            pcd = pcd.voxel_down_sample(0.001)
            pcd.paint_uniform_color([0.85, 0.30, 0.25])
            geoms.append(pcd)

    if not geoms:
        print("[main] 표시할 데이터 없음 — 시각화 skip")
        return

    # O 프레임 축 + 턴테이블 z=0 디스크
    geoms.append(o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.05))
    disc = o3d.geometry.TriangleMesh.create_cylinder(radius=0.12, height=0.001, resolution=64)
    disc.paint_uniform_color([0.20, 0.35, 0.80])
    disc.compute_vertex_normals()
    geoms.append(disc)

    print("[main] 붉음=mesh  /  파란 원판=turntable z=0 레퍼런스. Q/ESC 로 닫기.")
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name=title, width=1280, height=720)
    for g in geoms:
        vis.add_geometry(g)
    try:
        ctl = vis.get_view_control()
        ctl.set_up([0.0, 0.0, 1.0])
        ctl.set_front([-0.4, -0.4, -0.8])
        ctl.set_lookat([0.0, 0.0, 0.03])
        ctl.set_zoom(0.7)
    except Exception:
        pass
    vis.run()
    vis.destroy_window()


def run_turntable_calibration(mms, robot, turntable):
    """
    구 fixture 로 턴테이블 축(T_B_F0) 재calibration. sim/real 공통 진입점
    (mms.calibrate_turntable_axis). 카메라 점군 → base(T_EC·FK) → 축 피팅이며
    Artec first-frame 추적(SLAM)은 쓰지 않는다.
    """
    print(f"\n[main] === 턴테이블 축 calibration (backend={CFG.backend}) ===")
    if CFG.backend == "isaac":
        # sim: 가상 구 fixture 설치 + 부착 + 조준 (빈 턴테이블)
        from mms_artec.backends.isaac.calib_fixture import prepare_sim_fixture
        from utils.calibration.turntable_axis import axis_error
        from pxr import UsdGeom
        W = mms.sensor._world
        UsdGeom.Imageable(
            W.stage.GetPrimAtPath("/World/ScanTarget/Solid_Marble")).MakeInvisible()
        fx = prepare_sim_fixture(W, turntable, CALIB_SPHERE_OFFSETS, CALIB_SPHERE_RADIUS)
        print(f"  스캐너 조준 off-axis {fx['off_axis_deg']:.2f}°")
        res = mms.calibrate_turntable_axis(
            robot, turntable, sphere_radius=CALIB_SPHERE_RADIUS,
            sphere_z_bands=fx["z_bands_base"])
        de, pe = axis_error(res["axis_point"], res["axis_dir"],
                            fx["gt_point_base"], fx["gt_dir_base"])
        print(f"  [sim] GT 대비 방향오차 {de:.3f}°, 위치오차 {pe*1000:.2f}mm")
    else:
        # real: 물리 fixture 가 부착돼 있고 로봇이 fixture 를 보는 자세라고 가정
        if CALIB_SPHERE_Z_BANDS is None:
            print("  [real] CALIB_SPHERE_Z_BANDS(fixture 설계 z) 미설정 — calibration skip")
            return None
        res = mms.calibrate_turntable_axis(
            robot, turntable, sphere_radius=CALIB_SPHERE_RADIUS,
            sphere_z_bands=CALIB_SPHERE_Z_BANDS)
    ap, ad = res["axis_point"], res["axis_dir"]
    print(f"  ✔ T_B_F0 축(base): point={np.round(ap, 4).tolist()}  "
          f"dir={np.round(ad, 4).tolist()}  관측수={res['n_obs']}")
    return res


def main() -> None:
    confirm_home = (CFG.backend == "real")    # sim 에선 프롬프트 없이 진행

    robot = turntable = None
    try:
        with ArtecMMS(CFG) as mms:
            # robot/turntable 은 backend 에 맞춰 팩토리로 생성
            robot, turntable = mms.create_hardware()

            print(f"\n[main] === Artec MMS (backend={CFG.backend}) ===")
            # Artec scan-start = home (J7 = -45°)
            # robot.go_home(sensor="artec", speed=10, confirm=confirm_home)

            # 턴테이블 축 calibration (옵션) — 스캔 전에 T_B_F0 재설정
            if RUN_CALIBRATION:
                run_turntable_calibration(mms, robot, turntable)

            if PROCESS_SETTINGS is None:
                # Phase A (isaac, 스캐너 미가용): 모션/제어 데모만.
                print("\n[main] 스캐너 파이프라인 비활성 (Phase A). "
                      "모션/제어 sim 데모를 실행합니다.")
                print(f"  현재 TCP(mm,rad) = {np.round(robot.get_pose(), 2).tolist()}")
                print("  턴테이블 한 바퀴 회전 데모 (+360°)...")
                turntable.move_abs(np.radians(360.0), TURNTABLE_VEL_RAD_S)
                print(f"  턴테이블 θ = {np.degrees(turntable.getActualPos()):.1f}°")
                # sim 을 잠시 더 돌려 시각 확인
                if hasattr(mms.sensor, "_world"):
                    mms.sensor._world.step(120)
                print("[main] Phase A 데모 완료.")
                return

            print("\n[main] === Artec Phase 1 (Phase 2 OFF) ===")
            print(f"  T_EC: {CFG.T_EC_key}")
            print(f"  fusion: {PROCESS_SETTINGS.fusion}")
            print(f"  export OBJ:  {PROCESS_SETTINGS.export_obj_path}")
            print(f"  export sproj: {PROCESS_SETTINGS.export_sproj_path}")

            result = None
            try:
                result = mms.artec_process(robot, turntable, settings=PROCESS_SETTINGS)
                # streaming 모드에선 result.ctx=None. model 에서 직접 집계.
                n_frames = sum(result.model.get_scan(i).frame_count()
                               for i in range(result.model.scan_count()))
                print(f"\n[main] Phase 1 완료 — frames={n_frames}  "
                      f"scans={result.model.scan_count()}")
            except KeyboardInterrupt:
                print("\n[main] ⚠ KeyboardInterrupt — 현재 상태까지 보존")
            except Exception as e:
                print(f"\n[main] ✘ {type(e).__name__}: {e}")
                traceback.print_exc()

            if result is not None:
                _show_composite_mesh(
                    result, mms,
                    obj_path=PROCESS_SETTINGS.export_obj_path,
                )

    finally:
        # ── 턴테이블 안전 정지 (CRITICAL) ─────────────────────────────
        # disconnect 만 하면 모터는 계속 돌아감. 반드시 stop → servo OFF → disconnect.
        if turntable is None:
            return
        try:
            turntable.stop()
            print("[main] turntable.stop() OK")
        except Exception as e:
            print(f"[main] ⚠ turntable.stop() 예외: {e}")
            try:
                if hasattr(turntable, "reconnect"):
                    turntable.reconnect()
                turntable.stop()
                print("[main] turntable.stop() 재시도 OK")
            except Exception as e2:
                print(f"[main] ✘ 최종 stop 실패: {e2}  — 물리적 정지/전원차단 필요")
        try:
            turntable.set_servo_on(False)
        except Exception:
            pass
        try:
            turntable.disconnect()
        except Exception:
            pass
        try:
            robot.disconnect()
        except Exception:
            pass
        # isaac 백엔드: SimulationApp 명시적 종료 (atexit crash 방지)
        if CFG.backend == "isaac":
            try:
                from mms_artec.backends import shutdown_isaac_world
                shutdown_isaac_world()
            except Exception:
                pass


if __name__ == "__main__":
    main()
