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
from mms_artec.nbv.recovery_pose_selector import (
    LocalJitterSelector,
    CentroidVectorSelector,
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

# ── Multi-pass: Phase 1 + Phase 2 통합 (3-pose default) ────────────────
#   docs/7_artec_phase.md 의 Phase 1 (5면) + Phase 2 (바닥면 + 정합) 을
#   한 multi-pass 흐름에서 처리.
#
#   Pose 0 (Phase 1):  객체 canonical (face1 = top, 5면 캡처)
#   Pose 1 (Phase 2a): Y축 +90° 회전 — face5 가 face1 자리로 (overlap 옆면)
#   Pose 2 (Phase 2b): Y축 +180° 회전 — face6 (바닥) 가 위로
#
#   정합: docs/8_artec_phase2_pose_disambiguation.md §5.3 의 centroid-pivot
#   pre-rotation hint 적용. hints_applied=True 이면 post-merge GlobalReg 자동 skip.
POSE_ROTATIONS = make_axis_physical_rotations("y", [0.0, 90.0, 180.0])

# ── Tracking-lost auto-recovery ────────────────────────────────────────
#   tracking lost 발생 시 자동으로:
#     (a) last-good θ + 10° 만큼 turntable 역회전
#     (b) selector 가 새 카메라 pose 결정 → robot 이동
#     (c) 다음 streaming pass 진행 (pose_idx 유지)
#   같은 pose 안에서 연속 3회까지 시도, 초과 시 user prompt 로 fallback.
#
# RECOVERY_STRATEGY:
#   "local_jitter"     — Method A: 현재 pose 주변 ±3cm/±8° N candidate raycast,
#                        master overlap 최대 후보 선택 (exploitation)
#   "centroid_vector"  — Method B: master centroid 의 반대편 stand-off 250mm
#                        에서 centroid 향함 (exploration)
#   None               — 자동 recovery 비활성 (user prompt 만)
RECOVERY_STRATEGY: str | None = "local_jitter"

if RECOVERY_STRATEGY == "local_jitter":
    RECOVERY_SELECTOR = LocalJitterSelector(
        n_candidates=9,       # 9 random + 1 current = 10 후보
        trans_mm=30.0,
        rot_deg=8.0,
        include_current=True,
        seed=None,            # None → 매 호출마다 새 분포
    )
elif RECOVERY_STRATEGY == "centroid_vector":
    RECOVERY_SELECTOR = CentroidVectorSelector(stand_off_mm=250.0)
else:
    RECOVERY_SELECTOR = None

MULTIPASS_SETTINGS = ArtecMultiPassScanSessionSettings(
    streaming_settings=STREAM_SETTINGS,
    pose_physical_rotations=POSE_ROTATIONS,
    max_passes=8,                       # 3 pose + 재시도 여유
    prompt_before_first_pass=True,
    prompt_between_passes=True,
    prompt_on_tracking_lost=True,
    # auto-recovery
    recovery_selector=RECOVERY_SELECTOR,
    max_recovery_retries=3,
    safe_back_margin_deg=10.0,
    recovery_robot_speed_deg_s=10.0,
    recovery_turntable_vel_rad_s=float(np.radians(30.0)),
    # 스캔 도중 누적 컬러 포인트클라우드 실시간 표시 (Phase1 + Phase2 모든
    # pass). 노이즈/정합 멈춤을 눈으로 인지하기 위함. 창을 닫아도 스캔은
    # 계속됨. open3d 없으면 자동 skip.
    enable_live_viewer=True,
    # 회전 전 1회 PREVIEW 로 물체 크기/위치 추정 → 스캐너를 최적 작업거리·
    # 조준으로 이동 (docs/artec_scanning_pipeline.md §3.0). 실패 시 home 유지.
    adaptive_phase1_positioning=True,
    # True: 거리뿐 아니라 물체중심 둘레 zx평면 호 고도각을 적응형
    #       coarse→fine 으로 preview·스코어해 최적 1개 선택 (look_at 재조준
    #       — home preview 경험적 캘리브 광축, mis-aim 회피). 비퇴행 =
    #       home_dist baseline. 범위·오프셋·fine step 은 MULTIPASS_SETTINGS
    #       의 elevation_* 에서 튜닝. False: 기존 거리-only 적응.
    phase1_elevation_search=True,
    # [디버그] 회전차분 probe 결과 색상 PLY 를 output/iso_debug/ 에 덤프
    # (static=회·moving=파·object=초). CloudCompare 로 물체 분리/회전축
    # 검증. 진단 끝나면 False 로. scan/성능 무영향.
    probe_debug_dump=True,
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
    tt.connect(comm_type=1)   # 1=UDP (TCP 는 sustained polling 에서 socket sucked)
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
                _show_composite_mesh(
                    result, mms,
                    obj_path=PROCESS_SETTINGS.export_obj_path,
                )

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
