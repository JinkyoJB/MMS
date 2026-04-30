# mms/nbv/scan_session.py
#
# 상위 레이어(NBV) 오케스트레이터 (docs/2_control_layers.md §8).
# Frame 0 (O 정의) + 루프 (frontier → cost → plan_and_execute → ICP → integrate).
#
# Notation: T_AB  → A → B,  x_B = T_AB @ x_A  (README.md 준수).

from __future__ import annotations

import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Optional, TYPE_CHECKING

import numpy as np
import open3d as o3d

from utils.nbv.frontier import FrontierCandidate, extract_frontier_candidates
from utils.nbv.icp_strategy import IcpResult, icp_with_gates, pick_icp_roll
from utils.nbv.manual_picker import compute_camera_pose_from_normal
from mms_phoxi.nbv.tsdf_volume import TSDFScanVolume
from mms_phoxi.nbv.instant_meshing_volume import InstantMeshingScanVolume
from mms_phoxi.nbv.pcd_accumulate_volume import PcdAccumulateVolume
from utils.nbv._progress_vis import ProgressVisualizer

if TYPE_CHECKING:
    from utils.robot.xarm_interface import XArmInterface
    from mms_phoxi.sensor.phoxi_client import PhoxiClient
    from mms_phoxi.sensor.phoxi_dataset_replay import PhoxiDatasetReplay
    from mms_phoxi.system import MMS
    from utils.turntable.turntable_interface import Turntable


# ─────────────────────────────────────────────────────────────────────────────
# Settings
# ─────────────────────────────────────────────────────────────────────────────

@dataclass
class ScanSessionSettings:
    # 거리
    distance_m: float = 0.414

    # Mesh 누적 (TSDF)
    tsdf_voxel_length: float = 0.002
    tsdf_sdf_trunc: float = 0.006
    tsdf_depth_trunc: float = 2.0

    # Meshing backend
    #   "pcd_accumulate" → 센서 pcd 를 O 프레임에 누적 → Poisson (권장, 디버그 용이)
    #   "open3d"         → Open3D ScalableTSDFVolume (RGBD + intrinsic)
    #   "phoxi_im"       → Photoneo PhoXiInstantMeshing (C API, 마커 필요)
    mesh_backend: str = "pcd_accumulate"

    # Frontier
    min_seg_vertices: int = 10
    min_seg_length: float = 0.01
    max_seg_length: float = 0.08

    # Roll (§5)
    roll_fallback_thresh: float = 0.2

    # NBV cost 가중치 (§7)
    cost_alpha: float = 1.0
    cost_beta: float = 0.5
    cost_gamma: float = 0.0
    cost_delta: float = 0.3

    # ICP gate (§6.3) — 최초 tight / 완화된 loose 값의 중간 1/3 지점
    # (loose 로 갔더니 결과 이상 → 다시 조임)
    icp_max_correspondence_m: float = 0.007    # 5→10 mm 사이 1/3 (7 mm)
    icp_rmse_thresh_m: float = 0.0015          # 1→3 mm 사이 1/3 (1.5 mm)
    icp_fitness_thresh: float = 0.25           # 0.30→0.20 사이 1/3 (0.25)
    icp_drift_trans_m: float = 0.023           # 20→30 mm 사이 1/3 (23 mm)
    icp_drift_rot_deg: float = 12.0            # 10→15° 사이 1/3 (12°)
    icp_target_sample_voxel_m: float = 0.002

    # Planner (docs/1 §4.1)
    theta_range: tuple = (-np.pi, np.pi)
    theta_n_samples: int = 72
    planner_w_tt: float = 0.0

    # 모션 속도 (전체 속도 다운)
    robot_speed_deg_s: float = 15.0
    turntable_vel_rad_s: float = np.radians(10.0)

    # 종료 조건 (§9)
    K_max: int = 20
    plateau_window: int = 3
    plateau_growth_thresh: float = 0.01
    boundary_length_stop: float = 0.01

    # Phase 제어
    phase1_enabled: bool = True                 # rule-based 15° × 360° 회전 스캔
    phase1_theta_step_deg: float = 15.0
    phase1_dwell_s: float = 0.40                # 턴테이블 정지 후 캡처 전 대기 (기계 settling)
    phase1_show_progress: bool = True           # non-blocking TSDF 뷰어
    phase1_wait_window_close: bool = True       # Phase 1 완료 후 창 닫힐 때까지 대기

    # Phase 1 incremental ICP refinement — 2번째 프레임부터
    phase1_icp_refine: bool = True
    phase1_icp_sor_nb: int = 20                 # denoise (statistical outlier) 이웃 수
    phase1_icp_sor_std: float = 2.0             # denoise std ratio

    # Phase 1 종료 후 mesh/pcd 덤프 (Photoneo / CloudCompare 비교용)
    phase1_export_pcd_path: Optional[str] = None        # "output/phase1_merged.ply" 등
    phase1_export_mesh_path: Optional[str] = None       # "output/phase1_mesh.ply"
    phase1_poisson_backend: str = "open3d"              # "open3d" | "photoneo_exe" | "both"

    phase2_enabled: bool = True                 # frontier NBV (Phase 2)

    # 기타
    confirm_each_move: bool = True
    work_in_O: bool = True


# ─────────────────────────────────────────────────────────────────────────────
# Session state
# ─────────────────────────────────────────────────────────────────────────────

@dataclass
class ScanSessionState:
    step: int = 0
    T_CO_list: List[np.ndarray] = field(default_factory=list)   # 프레임별 C→O
    theta_list: List[float] = field(default_factory=list)
    surface_areas: List[float] = field(default_factory=list)
    mesh: Optional[o3d.geometry.TriangleMesh] = None


# ─────────────────────────────────────────────────────────────────────────────
# 유틸
# ─────────────────────────────────────────────────────────────────────────────

def _rot_angle(R_rel: np.ndarray) -> float:
    """3x3 rotation → rotation angle (rad)."""
    c = (np.trace(R_rel) - 1.0) * 0.5
    c = float(np.clip(c, -1.0, 1.0))
    return float(np.arccos(c))


def _mesh_to_target_pcd(
    mesh: o3d.geometry.TriangleMesh,
    voxel_m: float,
) -> o3d.geometry.PointCloud:
    """ICP 타겟용 — 메쉬에서 샘플링 + voxel_down."""
    if len(mesh.triangles) == 0:
        return o3d.geometry.PointCloud()
    pcd = mesh.sample_points_uniformly(
        number_of_points=max(1000, len(mesh.triangles) * 2)
    )
    if voxel_m > 0:
        pcd = pcd.voxel_down_sample(voxel_m)
    return pcd


# ─────────────────────────────────────────────────────────────────────────────
# ScanSession
# ─────────────────────────────────────────────────────────────────────────────

class ScanSession:
    """
    docs/2_control_layers.md §8 오케스트레이터.

    Usage
    -----
    session = ScanSession(mms, robot, turntable, settings=ScanSessionSettings())
    session.run()
    final_mesh = session.state.mesh
    """

    def __init__(
        self,
        mms: "MMS",
        robot: Optional["XArmInterface"] = None,
        turntable: Optional["Turntable"] = None,
        settings: Optional[ScanSessionSettings] = None,
        replay: Optional["PhoxiDatasetReplay"] = None,
    ):
        """
        Live 모드:  mms + robot + turntable 로 하드웨어를 직접 구동.
        Offline 모드: replay 를 넘기면 하드웨어 이동을 건너뛰고
                      저장된 프레임/포즈에서 그대로 재생.
                      (robot / turntable 은 None 허용)
        """
        self.mms = mms
        self.robot = robot
        self.turntable = turntable
        self.settings = settings or ScanSessionSettings()
        self.state = ScanSessionState()
        self._replay = replay

        self._volume = None                              # TSDFScanVolume or InstantMeshingScanVolume
        self._intrinsic: Optional[o3d.camera.PinholeCameraIntrinsic] = None

    @property
    def is_offline(self) -> bool:
        return self._replay is not None

    # ── 유틸 ──────────────────────────────────────────────────────────

    def _capture_frame(
        self,
        i: int,
        theta_target_rad: float,
    ) -> Optional[tuple]:
        """
        한 프레임을 가져와 sensor 버퍼를 갱신하고 (θ_act, T_EB) 를 반환.

        Live  : turntable move + wait + dwell + capture + encoder 읽기 + FK
        Offline: replay.seek(i) + capture(i) + get_pose(i)

        Returns
        -------
        (theta_act_rad, T_EB_44) or None on failure.
        """
        s = self.settings

        if self.is_offline:
            self._replay.seek(i)
            scan = self._replay.capture(frame_id=i, timestamp=0.0)
            if scan is None:
                return None
            theta_act, T_EB = self._replay.get_pose(i)
            return float(theta_act), np.asarray(T_EB, dtype=np.float64)

        # Live
        if i > 0 and self.turntable is not None and self.turntable.is_connected:
            ok = self.turntable.move_abs(
                float(theta_target_rad), float(s.turntable_vel_rad_s),
            )
            if ok is False:
                print("  ⚠ move_abs 명령 거부됨 — skip.")
                return None
            self.turntable.wait_motion_done()
        time.sleep(float(s.phase1_dwell_s))

        theta_act = self._read_theta(fallback=float(theta_target_rad))
        if abs(theta_act - theta_target_rad) > np.radians(2.0):
            print(f"  ⚠ θ 불일치  target={np.degrees(theta_target_rad):+.2f}°  "
                  f"actual={np.degrees(theta_act):+.2f}°  "
                  f"(Δ={np.degrees(theta_act - theta_target_rad):+.2f}°)  "
                  f"→ 턴테이블이 제대로 안 움직였을 가능성.")

        batch = self.mms.capture_frames(
            1, ee_pose_fn=self.robot.get_ee_pose_mat,
        )
        if not batch:
            print("  ⚠ 캡처 실패 — 이 프레임 skip.")
            return None
        frame_k = batch[0]
        return float(theta_act), np.asarray(frame_k.ee_pose_mat_B, dtype=np.float64)

    def _build_current_frame_pcd_C(self) -> o3d.geometry.PointCloud:
        """
        센서 마지막 organized Range → C 프레임 PointCloud (meters).
        ICP source 용.
        """
        sensor = self.mms.sensor
        organized = getattr(sensor, "_last_organized_pts", None)
        if organized is None:
            return o3d.geometry.PointCloud()

        flat_mm = organized.reshape(-1, 3).astype(np.float64)
        z = flat_mm[:, 2]
        # 볼륨과 동일한 depth 범위 사용
        min_mm = float(getattr(self._volume, "min_depth_m", 0.05)) * 1000.0
        max_mm = float(getattr(self._volume, "depth_trunc", 2.0)) * 1000.0
        valid = (
            (z > min_mm) & (z < max_mm) &
            np.isfinite(flat_mm).all(axis=1)
        )
        pts_C_m = flat_mm[valid] / 1000.0
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pts_C_m)
        return pcd

    def _read_theta(self, fallback: float) -> float:
        """
        턴테이블 각도를 안전하게 읽음. 실패(bool/None)시 `fallback` 사용.
        """
        tt = self.turntable
        if tt is None or not getattr(tt, "is_connected", False):
            return float(fallback)
        val = tt.getActualPos()
        if isinstance(val, bool) or val is None:
            print(f"  [turntable] getActualPos 실패 — fallback "
                  f"{np.degrees(fallback):+.2f}° 사용")
            return float(fallback)
        return float(val)

    # ── TSDF 볼륨 lazy init ───────────────────────────────────────────

    def _ensure_volume(self) -> None:
        """첫 캡처 후 선택된 backend 로 볼륨 생성."""
        if self._volume is not None:
            return
        s = self.settings

        backend = (s.mesh_backend or "").lower()

        if backend == "pcd_accumulate":
            print("[ScanSession] backend = PcdAccumulate (O 프레임 누적 → Poisson)")
            self._volume = PcdAccumulateVolume(
                sensor=self.mms.sensor,
                voxel_length=s.tsdf_voxel_length,
                depth_trunc=s.tsdf_depth_trunc,
            )
            return

        if backend == "phoxi_im":
            print("[ScanSession] backend = PhoxiInstantMeshing (C API)")
            self._volume = InstantMeshingScanVolume(
                sensor=self.mms.sensor,
                voxel_length=s.tsdf_voxel_length,
                depth_trunc=s.tsdf_depth_trunc,
            )
            return

        # 기본 = Open3D TSDF — intrinsic 필요
        if not hasattr(self.mms.sensor, "get_intrinsic"):
            raise RuntimeError("sensor.get_intrinsic() 미구현.")
        self._intrinsic = self.mms.sensor.get_intrinsic()
        K = self._intrinsic.intrinsic_matrix
        print(f"[ScanSession] backend = Open3D TSDF  "
              f"fx={K[0,0]:.1f} fy={K[1,1]:.1f} "
              f"cx={K[0,2]:.1f} cy={K[1,2]:.1f} "
              f"({self._intrinsic.width}×{self._intrinsic.height})")
        self._volume = TSDFScanVolume(
            intrinsic=self._intrinsic,
            voxel_length=s.tsdf_voxel_length,
            sdf_trunc=s.tsdf_sdf_trunc,
            depth_trunc=s.tsdf_depth_trunc,
        )

    # ── Entry ──────────────────────────────────────────────────────────

    def run(self) -> None:
        self._prepare()

        # Phase 1 — rule-based 15° × 360° 턴테이블 회전 스캔
        if self.settings.phase1_enabled:
            self._rule_based_scan()
        else:
            self._frame0()

        # Phase 2 — frontier NBV 루프 (offline 에서는 실행 불가 — 로봇/턴테이블 필요)
        if self.settings.phase2_enabled:
            if self.is_offline:
                print("\n[ScanSession] Phase 2 skip — offline 모드에서는 "
                      "실제 로봇/턴테이블 이동이 불가능합니다.")
            else:
                print("\n═══════════════ Phase 2 — Frontier NBV ═══════════════")
                for _ in range(self.settings.K_max):
                    if not self._step():
                        break
                    if self._terminate():
                        print("[ScanSession] 종료 조건 충족 — Phase 2 루프 종료.")
                        break

        print(f"[ScanSession] 완료  총 {len(self.state.T_CO_list)} 프레임 적분됨.")

    # ── 준비 (θ=0 리셋만) ─────────────────────────────────────────────

    def _prepare(self) -> None:
        s = self.settings

        if self.is_offline:
            print("[ScanSession] offline (replay) 모드 — 하드웨어 이동 생략")
            return

        # θ=0 리셋 — Q1: O ≡ F at θ=0 단순화.
        # 이미 0 근처면 move 생략 (servo-on 직후 driver race condition 회피).
        if self.turntable is not None and self.turntable.is_connected:
            cur = self._read_theta(fallback=0.0)
            if abs(cur) < np.radians(0.5):
                print(f"[ScanSession] 턴테이블 이미 θ≈0 ({np.degrees(cur):+.3f}°) — reset 생략")
            else:
                print(f"[ScanSession] 턴테이블 θ=0 리셋 (현재 {np.degrees(cur):+.2f}°)")
                self.turntable.move_abs(0.0, float(s.turntable_vel_rad_s))
                self.turntable.wait_motion_done()

        # Intrinsic / TSDF 볼륨은 Frame 0 의 캡처 직후 생성 (첫 캡처의 organized
        # Range 로부터 intrinsic fit) — 별도 warmup 캡처 불필요.

    # ── Frame 0 ────────────────────────────────────────────────────────

    def _frame0(self) -> None:
        print("\n═══════════════ Frame 0 — O 정의 ═══════════════")
        # scan_start 는 호출자(main.py) 가 이미 해둔 상태라 가정.
        batch = self.mms.capture_frames(1, ee_pose_fn=self.robot.get_ee_pose_mat)
        if not batch:
            raise RuntimeError("[ScanSession] Frame 0 캡처 실패.")
        frame0 = batch[0]

        # 백엔드에 따라 volume 생성 (intrinsic fit 은 open3d 백엔드일 때만)
        self._ensure_volume()

        # T_CO_0 : 현재 EE, θ=0 에서 계산
        T_CO_0 = self.mms.T_CO(theta=0.0, T_EB=frame0.ee_pose_mat_B)

        # TSDF integrate — full frame (§3.2)
        self._volume.integrate_frame(frame0, T_CO_0)

        mesh_0 = self._volume.extract_mesh()
        area = float(mesh_0.get_surface_area()) if len(mesh_0.triangles) else 0.0

        self.state.mesh = mesh_0
        self.state.T_CO_list.append(T_CO_0)
        self.state.theta_list.append(0.0)
        self.state.surface_areas.append(area)
        self.state.step = 0

        print(f"[ScanSession] M_0: verts={len(mesh_0.vertices)} "
              f"tris={len(mesh_0.triangles)}  area={area*1e4:.1f} cm²")

    # ── Phase 1: rule-based 턴테이블 회전 스캔 (15° × 24) ─────────────

    def _rule_based_scan(self) -> None:
        """
        scan_start 자세 고정 + 턴테이블을 `phase1_theta_step_deg` 간격으로 360°
        회전하며 각 각도에서 캡처·TSDF integrate.

        ICP 미사용 — 턴테이블 엔코더 + hand-eye + FK 로 계산한 `T_CO` 를 그대로
        사용 (기계적 회전이므로 θ 가 정확히 알려져 있음).

        `phase1_show_progress=True` 면 매 프레임 non-blocking 뷰어에 누적 mesh 갱신.
        """
        s = self.settings
        step = np.radians(float(s.phase1_theta_step_deg))
        n = int(round(2.0 * np.pi / step))
        # offline 에서는 dataset 의 실제 프레임 수로 cap
        if self.is_offline:
            n_available = int(self._replay.n_frames)
            if n_available < n:
                print(f"[ScanSession] offline: replay n_frames={n_available} "
                      f"< 예상 {n} — 저장된 프레임 수로 제한")
                n = n_available
        thetas = [i * step for i in range(n)]

        print(f"\n═══════════════ Phase 1 — Rule-based {n} frames ═══════════════")
        print(f"  θ step = {s.phase1_theta_step_deg:.1f}°  "
              f"turntable_vel = {np.degrees(s.turntable_vel_rad_s):.1f}°/s  "
              f"dwell = {s.phase1_dwell_s:.2f}s  "
              f"voxel = {s.tsdf_voxel_length*1000:.1f}mm")

        vis: Optional[ProgressVisualizer] = None
        if s.phase1_show_progress:
            vis = ProgressVisualizer(
                window_title=f"Phase 1 — TSDF progress ({n} frames)",
            )

        try:
            for i, theta_tgt in enumerate(thetas):
                print(f"\n─── Phase 1 [{i+1}/{n}] θ_target = "
                      f"{np.degrees(theta_tgt):+.1f}° ───")

                # 프레임 컨텍스트 — live 에서는 turntable+robot+capture, offline 은 replay
                ctx = self._capture_frame(i, theta_tgt)
                if ctx is None:
                    if vis is not None:
                        vis._pump()
                    continue
                theta_act, T_EB = ctx

                # 첫 프레임: 선택된 backend 로 볼륨 생성
                self._ensure_volume()

                # T_CO_nominal — 턴테이블 엔코더 + FK + hand-eye 로 계산
                T_CO_nominal = self.mms.T_CO(theta_act, T_EB)
                T_CO = T_CO_nominal

                # ICP refinement — 2번째 프레임(i>=1)부터,
                #   pcd_accumulate backend 에서만 (누적 pcd 를 target 으로 씀).
                use_icp = (
                    s.phase1_icp_refine and i >= 1 and
                    isinstance(self._volume, PcdAccumulateVolume)
                )
                if use_icp:
                    source_pcd = self._build_current_frame_pcd_C()
                    target_pcd = self._volume.merged_pcd()
                    if len(source_pcd.points) < 200 or len(target_pcd.points) < 200:
                        print(f"  [ICP] skip — source={len(source_pcd.points):,}  "
                              f"target={len(target_pcd.points):,}")
                    else:
                        # denoise — voxel_down + statistical outlier
                        source_pcd = source_pcd.voxel_down_sample(s.tsdf_voxel_length)
                        try:
                            source_pcd, _ = source_pcd.remove_statistical_outlier(
                                nb_neighbors=s.phase1_icp_sor_nb,
                                std_ratio=s.phase1_icp_sor_std,
                            )
                        except Exception:
                            pass
                        try:
                            target_pcd, _ = target_pcd.remove_statistical_outlier(
                                nb_neighbors=s.phase1_icp_sor_nb,
                                std_ratio=s.phase1_icp_sor_std,
                            )
                        except Exception:
                            pass

                        icp_res = icp_with_gates(
                            source_pcd=source_pcd,
                            target_pcd=target_pcd,
                            init_T=T_CO_nominal,
                            max_correspondence_distance=s.icp_max_correspondence_m,
                            rmse_thresh=s.icp_rmse_thresh_m,
                            fitness_thresh=s.icp_fitness_thresh,
                            drift_trans_m=s.icp_drift_trans_m,
                            drift_rot_deg=s.icp_drift_rot_deg,
                        )
                        if icp_res.ok:
                            T_CO = icp_res.T_refined
                            print(f"  [ICP] ok  rmse={icp_res.rmse*1000:.2f}mm  "
                                  f"fit={icp_res.fitness:.2f}  "
                                  f"Δt={icp_res.delta_translation_m*1000:.1f}mm  "
                                  f"Δr={icp_res.delta_rotation_deg:.1f}°")
                        else:
                            print(f"  [ICP] 실패({icp_res.reason}) — nominal 사용")

                # PcdAccumulateVolume 은 frame 인자를 사용 안 함 (sensor._last_* 직접 읽음).
                # Frame 객체가 없는 offline 에서도 동작하도록 None 전달.
                self._volume.integrate_frame(None, T_CO)

                # 진행률 로깅 — backend 에 따라 pcd 또는 mesh
                is_pcd = isinstance(self._volume, PcdAccumulateVolume)

                self.state.T_CO_list.append(T_CO)
                self.state.theta_list.append(theta_act)
                self.state.step = i

                if is_pcd:
                    # 빠른 path — 누적 pcd 만 계산. Poisson 은 Phase 1 끝에 한 번.
                    pcd_merged = self._volume.merged_pcd()
                    n_pts = len(pcd_merged.points)
                    self.state.surface_areas.append(float(n_pts))   # 대용 지표
                    print(f"  θ_act = {np.degrees(theta_act):+.2f}°  "
                          f"accumulated pcd = {n_pts:,} pts")
                    if vis is not None:
                        vis.update_pcd(pcd_merged)
                        vis.update_camera_trajectory(self.state.T_CO_list)
                else:
                    mesh = self._volume.extract_mesh()
                    area = float(mesh.get_surface_area()) \
                        if len(mesh.triangles) else 0.0
                    self.state.mesh = mesh
                    self.state.surface_areas.append(area)
                    print(f"  θ_act = {np.degrees(theta_act):+.2f}°  "
                          f"verts={len(mesh.vertices):,}  "
                          f"area={area*1e4:.1f} cm²")
                    if vis is not None:
                        vis.update_mesh(mesh)
                        vis.update_camera_trajectory(self.state.T_CO_list)

            # Phase 1 loop 종료 — pcd_accumulate backend 면 덤프 + Poisson
            if isinstance(self._volume, PcdAccumulateVolume):
                print(f"\n[Phase 1] {self._volume.n_integrated} 프레임 누적 완료.")

                # PCD 덤프 (비교용)
                if s.phase1_export_pcd_path:
                    self._volume.save_pcd(
                        path=s.phase1_export_pcd_path,
                        with_normals=True,
                    )

                backend = (s.phase1_poisson_backend or "open3d").lower()
                mesh_out = None

                if backend in ("open3d", "both"):
                    print("[Phase 1] Open3D Poisson 생성 중...")
                    mesh_o = self._volume.extract_mesh(verbose=True)
                    print(f"  [open3d] verts={len(mesh_o.vertices):,}  "
                          f"tris={len(mesh_o.triangles):,}  "
                          f"has_colors={mesh_o.has_vertex_colors()}")
                    if s.phase1_export_mesh_path:
                        path = s.phase1_export_mesh_path
                        if backend == "both":
                            path = str(Path(path).with_stem(Path(path).stem + "_open3d"))
                        Path(path).parent.mkdir(parents=True, exist_ok=True)
                        o3d.io.write_triangle_mesh(
                            path, mesh_o,
                            write_vertex_colors=True,
                            write_vertex_normals=True,
                        )
                        print(f"  [open3d] saved → {path}  (binary PLY)")
                        # ASCII PLY 도 같이 — 일부 외부 뷰어에서 호환성 개선
                        ascii_path = str(Path(path).with_stem(Path(path).stem + "_ascii"))
                        try:
                            o3d.io.write_triangle_mesh(
                                ascii_path, mesh_o,
                                write_ascii=True,
                                write_vertex_colors=True,
                                write_vertex_normals=True,
                            )
                            print(f"  [open3d] saved → {ascii_path}  (ASCII PLY)")
                        except Exception as e:
                            print(f"  [open3d] ASCII PLY 저장 실패: {e}")
                    if backend == "open3d" or mesh_out is None:
                        mesh_out = mesh_o

                if backend in ("photoneo_exe", "both"):
                    try:
                        print("[Phase 1] Photoneo PoissonRecon.exe 실행 중...")
                        mesh_p = self._volume.extract_mesh_photoneo(verbose=True)
                        print(f"  [photoneo] verts={len(mesh_p.vertices):,}  "
                              f"tris={len(mesh_p.triangles):,}")
                        if s.phase1_export_mesh_path:
                            path = s.phase1_export_mesh_path
                            if backend == "both":
                                path = str(Path(path).with_stem(Path(path).stem + "_photoneo"))
                            Path(path).parent.mkdir(parents=True, exist_ok=True)
                            o3d.io.write_triangle_mesh(
                                path, mesh_p,
                                write_vertex_colors=True,
                                write_vertex_normals=True,
                            )
                            print(f"  [photoneo] saved → {path}")
                        if backend == "photoneo_exe":
                            mesh_out = mesh_p
                    except Exception as e:
                        print(f"  ⚠ Photoneo PoissonRecon 실패: {e}")
                        if mesh_out is None:
                            mesh_out = self._volume.extract_mesh()

                if mesh_out is None:
                    mesh_out = o3d.geometry.TriangleMesh()
                self.state.mesh = mesh_out
                if vis is not None:
                    vis.update_mesh(mesh_out)
        finally:
            if vis is not None:
                if self.settings.phase1_wait_window_close:
                    print("\n[Phase 1] 완료. 뷰어 창 닫으면 Phase 2 진행.")
                    vis.run_until_closed()
                else:
                    vis.close()

    # ── 스텝 루프 ───────────────────────────────────────────────────────

    def _step(self) -> bool:
        s = self.settings
        k = self.state.step + 1
        print(f"\n═══════════════ Frame {k} ═══════════════")

        # Frontier 추출
        cands = extract_frontier_candidates(
            self.state.mesh,
            min_seg_vertices=s.min_seg_vertices,
            min_seg_length=s.min_seg_length,
            max_seg_length=s.max_seg_length,
        )
        if not cands:
            print("[ScanSession] frontier 후보 없음 — 종료.")
            return False
        print(f"[ScanSession] frontier 후보 {len(cands)}개")

        # 후보 평가 → cost 정렬
        ranked = self._rank_candidates(cands)
        if not ranked:
            print("[ScanSession] feasible 후보 없음 — 종료.")
            return False

        # 후보 순서대로 실행 시도 — ICP 실패 시 다음 후보
        theta_cur = self.state.theta_list[-1]
        for rank, (cand, T_CO_i, plan_i, cost_i) in enumerate(ranked):
            print(f"\n[ScanSession] 후보 #{rank+1}/{len(ranked)}  "
                  f"cost={cost_i:.4f}  θ*={np.degrees(plan_i['theta']):+.2f}°  "
                  f"L={cand.L*1000:.1f}mm")

            # 하드웨어 실행
            result = self.mms.execute_target(
                T_CO_des=T_CO_i, theta=plan_i["theta"],
                robot=self.robot, turntable=self.turntable,
                robot_speed=s.robot_speed_deg_s,
                turntable_vel_rad_s=s.turntable_vel_rad_s,
                move_turntable=True, move_robot=True,
                confirm=s.confirm_each_move,
            )
            if result is None or not result.get("ik_ok", False):
                print("   → 실행 실패, 다음 후보.")
                continue

            # 캡처 + ICP
            captured = self.mms.capture_frames(1, ee_pose_fn=self.robot.get_ee_pose_mat)
            if not captured:
                print("   → 캡처 실패, 다음 후보.")
                continue
            frame_k = captured[0]

            # nominal T_CO (실제 간 θ, EE 로 계산)
            theta_actual = float(self.turntable.getActualPos()) \
                if self.turntable is not None and self.turntable.is_connected \
                else plan_i["theta"]
            T_CO_nominal = self.mms.T_CO(theta_actual, frame_k.ee_pose_mat_B)

            # ICP refinement
            source_pcd = o3d.geometry.PointCloud()
            source_pcd.points = o3d.utility.Vector3dVector(
                frame_k.points.astype(np.float64)
            )
            target_pcd = _mesh_to_target_pcd(
                self.state.mesh, s.icp_target_sample_voxel_m,
            )

            # source points are in Base(B) frame (MMS._scan_to_frame 적용 후).
            # ICP 타겟은 O 프레임. 초기 추정은 T_BO = T_FB(θ) @ T_OF 의 역.
            # 하지만 우리는 "C → O" 를 refine 하려는 것이 아니라
            # "B → O" (frame point 를 O 로 옮기는 transform) 를 refine 해야 한다.
            # T_BO_nominal = inv(T_OF) @ T_BF(θ_actual)
            T_BO_nominal = (
                np.linalg.inv(self.mms._T_OF)
                @ self.mms.turntable_transform.T_BF(theta_actual)
            )

            icp = icp_with_gates(
                source_pcd, target_pcd,
                init_T=T_BO_nominal,
                max_correspondence_distance=s.icp_max_correspondence_m,
                rmse_thresh=s.icp_rmse_thresh_m,
                fitness_thresh=s.icp_fitness_thresh,
                drift_trans_m=s.icp_drift_trans_m,
                drift_rot_deg=s.icp_drift_rot_deg,
            )
            if not icp.ok:
                print(f"   → ICP 실패: {icp.reason}")
                continue

            # refined T_BO → refined T_CO
            # T_CO = T_BO @ T_CB = T_BO @ (T_EB @ inv(T_EC))
            T_CB = self.mms.T_CB(frame_k.ee_pose_mat_B)
            T_CO_refined = icp.T_refined @ T_CB

            # Integrate
            self._volume.integrate_frame(frame_k, T_CO_refined)
            self.state.mesh = self._volume.extract_mesh()
            area = float(self.state.mesh.get_surface_area()) \
                if len(self.state.mesh.triangles) else 0.0

            self.state.T_CO_list.append(T_CO_refined)
            self.state.theta_list.append(theta_actual)
            self.state.surface_areas.append(area)
            self.state.step = k

            prev_area = self.state.surface_areas[-2]
            growth = (area - prev_area) / max(prev_area, 1e-9)
            print(f"[ScanSession] M_{k}: verts={len(self.state.mesh.vertices)}  "
                  f"tris={len(self.state.mesh.triangles)}  "
                  f"area={area*1e4:.1f} cm²  (Δ {growth*100:+.1f}%)  "
                  f"ICP rmse={icp.rmse*1000:.2f}mm fit={icp.fitness:.2f}")
            return True

        print("[ScanSession] 모든 후보 소진 — 스텝 실패, 다음 iter 로.")
        # step 은 증가시키지 않음 (docs §8)
        return True

    # ── 후보 ranking ───────────────────────────────────────────────────

    def _rank_candidates(
        self,
        cands: List[FrontierCandidate],
    ) -> list:
        s = self.settings
        T_CO_cur = self.state.T_CO_list[-1]
        p_cur = T_CO_cur[:3, 3]
        R_cur = T_CO_cur[:3, :3]
        theta_cur = self.state.theta_list[-1]
        p_cam_prev = T_CO_cur[:3, 3]
        R_CO_prev = T_CO_cur[:3, :3]

        # 노말 z<0 강제 (docs/1 §3.1 의 규칙 재사용)
        for c in cands:
            if c.n_O[2] > 0:
                c.n_O = -c.n_O

        # L 정규화
        L_max = max((c.L for c in cands), default=1e-9)
        L_max = L_max if L_max > 1e-9 else 1e-9

        evaluated = []
        for c in cands:
            roll_i = pick_icp_roll(
                p_i_O=c.p_O, n_i_O=c.n_O, p_cam_prev_O=p_cam_prev,
                distance_m=s.distance_m,
                R_CO_prev=R_CO_prev,
                fallback_thresh=s.roll_fallback_thresh,
            )
            T_CO_i = compute_camera_pose_from_normal(
                surface_point=c.p_O, normal=c.n_O,
                distance_m=s.distance_m, roll_rad=roll_i,
            )
            # plan_min_motion_theta 을 직접 호출 (execute 없이 θ만 탐색)
            from utils.control.theta_planner import plan_min_motion_theta
            plan = plan_min_motion_theta(
                T_CO_des=T_CO_i,
                T_OF=self.mms._T_OF, T_EC=self.mms._T_EC,
                tt=self.mms.turntable_transform,
                robot=self.robot,
                theta_current=theta_cur,
                theta_range=s.theta_range, n_samples=s.theta_n_samples,
                w_tt=s.planner_w_tt,
                verbose=False,
            )
            if plan["theta"] is None:
                continue

            # Cost (§7)
            dp = T_CO_i[:3, 3] - p_cur
            dR = R_cur.T @ T_CO_i[:3, :3]
            phi = _rot_angle(dR)
            d_theta = plan["theta"] - theta_cur
            L_hat = c.L / L_max

            cost = (
                s.cost_alpha * float(dp @ dp)
                + s.cost_beta * phi * phi
                + s.cost_gamma * d_theta * d_theta
                - s.cost_delta * L_hat
            )
            evaluated.append((c, T_CO_i, plan, cost))

        evaluated.sort(key=lambda x: x[3])
        return evaluated

    # ── 종료 조건 (§9) ─────────────────────────────────────────────────

    def _terminate(self) -> bool:
        s = self.settings
        if self.state.step >= s.K_max:
            return True

        # Plateau
        if len(self.state.surface_areas) >= s.plateau_window + 1:
            recent = self.state.surface_areas[-(s.plateau_window + 1):]
            growths = [
                (recent[i + 1] - recent[i]) / max(recent[i], 1e-9)
                for i in range(s.plateau_window)
            ]
            if all(g < s.plateau_growth_thresh for g in growths):
                print(f"[ScanSession] plateau — 최근 {s.plateau_window} step 증가율 "
                      f"{[f'{g*100:+.2f}%' for g in growths]}")
                return True

        # Boundary 길이
        from utils.nbv.frontier import extract_boundary_edges
        boundary = extract_boundary_edges(self.state.mesh)
        if len(boundary) == 0:
            return True
        verts = np.asarray(self.state.mesh.vertices)
        d = verts[boundary[:, 0]] - verts[boundary[:, 1]]
        total_len = float(np.linalg.norm(d, axis=1).sum())
        if total_len < s.boundary_length_stop:
            print(f"[ScanSession] boundary 길이 {total_len*1000:.1f}mm < "
                  f"{s.boundary_length_stop*1000:.1f}mm — watertight 근접.")
            return True

        return False
