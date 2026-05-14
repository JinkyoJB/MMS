import msvcrt
import traceback
from datetime import datetime

import numpy as np

from utils import PROJECT_ROOT

# 모든 output 파일에 같은 타임스탬프(_YYYYMMDD_HHMMSS) 붙여 run 별 구분.
RUN_TS = datetime.now().strftime("%Y%m%d_%H%M%S")
from mms_artec.system import ArtecMMS, ArtecMMSConfig, ArtecProcessSettings
from mms_artec.sensor.artec_client import ArtecConfig
from mms_artec.nbv.artec_scan_session import ArtecScanSessionSettings
from mms_artec.nbv.artec_streaming_scan_session import ArtecStreamingScanSessionSettings
from mms_artec.nbv.artec_multipass_scan_session import (
    ArtecMultiPassScanSessionSettings,
    make_axis_physical_rotations,
)
from utils.robot.xarm_interface import XArmInterface
from utils.turntable import Turntable

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
)

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
    # 시계열 로그 — t, theta, scanning_flag, ... → CSV
    timeline_csv_path=str(PROJECT_ROOT / f"output/artec_phase1_{RUN_TS}_timeline.csv"),
)

# ── (Legacy) Discrete Phase 1 — reference 만, use_streaming_scan=False 일 때 사용 ──
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

# ── Multi-pass: Phase 1 + 2 통합 (3-pose default) ─────────────────────
#   Pose 0: 객체 canonical (face1 = top)
#   Pose 1: Y축 +90° 회전 — face5 가 face1 자리로
#   Pose 2: Y축 +180° 회전 — face6 (바닥) 가 위로
#   docs/8_artec_phase2_pose_disambiguation.md §3.1 의 pre-rotation hint 적용.
POSE_ROTATIONS = make_axis_physical_rotations("y", [0.0, 90.0, 180.0])

MULTIPASS_SETTINGS = ArtecMultiPassScanSessionSettings(
    streaming_settings=STREAM_SETTINGS,
    pose_physical_rotations=POSE_ROTATIONS,
    max_passes=8,                       # 3 pose + 재시도 여유
    prompt_before_first_pass=True,
    prompt_between_passes=True,
    prompt_on_tracking_lost=True,
)

# ── ArtecProcess pipeline (Studio §4 의 1, 3-6 단계) ──────────────────
#   1.  Scanning            — Phase 1 만 (위 SCAN_SETTINGS)
#   2.  Cleaning            — skip (Outliers Removal 이 대체)
#   3.  Alignment           — SerialRegistration  (frame-to-frame 정렬 정밀화)
#   4.  Registration        — GlobalRegistration  (scan 1개라 사실상 no-op)
#   5.  Fusion              — PoissonFusion       (watertight mesh)
#   6.  Postprocessing      — Outliers + SmallObjects + (Simplify off) + Texturize
# ★ Dev mode 토글 — True 면 OutliersRemoval / Simplify 자동 skip (5분+ → 1분 이하).
#   Production export 시 False 로.
DEV_MODE = True

PROCESS_SETTINGS = ArtecProcessSettings(
    dev_mode=DEV_MODE,
    use_streaming_scan=True,                # ★ IScanningProcedure (연속 회전)
    scan_settings=SCAN_SETTINGS,
    streaming_scan_settings=STREAM_SETTINGS,
    multipass_settings=MULTIPASS_SETTINGS,
    do_serial_registration=False,           # streaming 이 SDK 안에서 이미 reg 함
    do_global_registration=True,
    fusion="poisson",
    do_outliers_removal=True,               # dev_mode=True 면 자동 False
    do_small_objects_filter=True,
    do_simplify=False,
    do_texturize=True,
    export_obj_path=str(PROJECT_ROOT / f"output/artec_phase1_{RUN_TS}.obj"),
    export_sproj_path=str(PROJECT_ROOT / f"output/artec_phase1_{RUN_TS}.sproj"),
)


def _flush_stdin() -> None:
    """캡처 대기 중 눌린 잔류 Enter 를 stdin 버퍼에서 제거."""
    while msvcrt.kbhit():
        msvcrt.getch()


def connect_turntable() -> Turntable:
    """턴테이블 연결 + 서보 ON. 이전 run 잔여 회전 있으면 즉시 정지."""
    tt = Turntable(bd_id=TURNTABLE_BD_ID, ip=TURNTABLE_IP, pulses_per_rev=50000)
    tt.connect(comm_type=0)
    tt.check_drive_info()
    tt.check_drive_err()
    # 이전 run 이 비정상 종료해서 모터가 계속 돌고 있을 수 있음 — 즉시 정지
    try:
        tt.stop()
    except Exception:
        pass
    tt.set_servo_on(True)
    tt.set_acceleration(np.radians(180), np.radians(180))
    return tt


def _show_composite_mesh(result, mms, title: str = "Artec Phase 1") -> None:
    """
    Composite mesh 가 있으면 그걸로 시각화.
    없으면 IModel 안 frame transformations 으로 vertices 를 누적해 PointCloud.
    """
    import open3d as o3d

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


def main() -> None:
    robot = XArmInterface(ROBOT_IP)
    turntable = connect_turntable()

    try:
        with ArtecMMS(CFG) as mms:
            # Artec scan-start = home (J7 = -45°)
            robot.go_home(sensor="artec", speed=10, confirm=True)

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
                _show_composite_mesh(result, mms)

    finally:
        # ── 턴테이블 안전 정지 (CRITICAL) ─────────────────────────────
        # disconnect 만 하면 모터는 계속 돌아감. 반드시 stop → servo OFF → disconnect.
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


if __name__ == "__main__":
    main()
