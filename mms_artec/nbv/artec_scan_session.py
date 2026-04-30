# mms/nbv/artec_scan_session.py
#
# Artec native 자료구조 (FrameMeshHandle / ScanHandle / ModelHandle) 기반의
# Phase 1 + Phase 2 스캔 세션. PhoXi 의 `scan_session.py` 와 별도로 분리.
#
# Studio §4 의 1단계 "Scanning" 만 담당 — Cleaning/Alignment/Registration/Fusion/
# Postprocess 는 `MMS.artec_process()` 가 후속으로 호출.
#
# Notation: T_AB : A → B (x_B = T_AB @ x_A) — README.md / CLAUDE.md 준수.

from __future__ import annotations

import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Optional, TYPE_CHECKING

import numpy as np
import open3d as o3d

from mms_artec.sensor import artec_base
from mms_artec.sensor.artec_client import ArtecClient
from utils.nbv._progress_vis import ProgressVisualizer

if TYPE_CHECKING:
    from mms_artec.system import ArtecMMS as MMS
    from utils.robot.xarm_interface import XArmInterface
    from utils.turntable.turntable_interface import Turntable


# ─── Artec C frame ↔ OpenCV C frame 변환 ────────────────────────────────
# Artec vertices() 는 z-back convention (x, y 는 OpenCV 와 동일).
# hand-eye 는 cv2.solvePnP 로 풀어서 T_EC 가 OpenCV (z-앞) convention.
#
# Multi-frame chamfer test (2026-04-29, θ=0 vs θ=15°):
#   A identity        :  78.4 mm  ❌
#   B z-flip          :   1.82 mm ✅ (best)
#   C y,z flip        :   4.0  mm
#   D x,z flip        :   5.1  mm
#   E x,y flip        :  61.0  mm  ❌
# → Z-flip 만이 정답. 단 reflection (det=-1) 이라 set_frame_transformation
#   에서 Artec 이 받아주는지는 별개 — 거부 시 vertex 변환만 수동 적용 필요.
ARTEC_TO_OPENCV = np.diag([1.0, 1.0, -1.0, 1.0])    # z-flip (reflection, det=-1)


# ─────────────────────────────────────────────────────────────────────────────
# Settings
# ─────────────────────────────────────────────────────────────────────────────

@dataclass
class ArtecScanSessionSettings:
    # ── Phase 1 — rule-based 15°×24 회전 ──────────────────────────────
    phase1_enabled: bool = True
    phase1_theta_step_deg: float = 15.0
    phase1_dwell_s: float = 0.4               # 턴테이블 settling
    phase1_show_progress: bool = True
    phase1_wait_window_close: bool = True

    # ── Phase 2 — frontier NBV 탐색 ───────────────────────────────────
    phase2_enabled: bool = True
    phase2_K_max: int = 10
    distance_m: float = 0.260                 # Spider 작동거리 중간
    roll_fallback_thresh: float = 0.2
    min_seg_vertices: int = 10
    min_seg_length: float = 0.01
    max_seg_length: float = 0.08
    cost_alpha: float = 1.0
    cost_beta: float = 0.5
    cost_gamma: float = 0.0
    cost_delta: float = 0.3
    theta_range: tuple = (-np.pi, np.pi)
    theta_n_samples: int = 72
    planner_w_tt: float = 0.0

    # ── 종료 조건 ─────────────────────────────────────────────────────
    plateau_window: int = 3
    plateau_growth_thresh: float = 0.01
    boundary_length_stop: float = 0.01

    # ── 모션 ──────────────────────────────────────────────────────────
    robot_speed_deg_s: float = 15.0
    turntable_vel_rad_s: float = np.radians(10.0)

    # ── 기타 ──────────────────────────────────────────────────────────
    confirm_each_move: bool = True


# ─────────────────────────────────────────────────────────────────────────────
# Context — Artec native + 외부 메타
# ─────────────────────────────────────────────────────────────────────────────

@dataclass
class ArtecScanContext:
    """
    Artec native 자료구조 + 우리 메타 (T_EB, theta 등) 의 평행 보유.

    Artec FrameMeshHandle 자체에는 EE pose 가 없어서, 자세 i 에서의 T_EB / θ 는
    parallel array 로 외부에서 추적한다. `model` 안 frame 순서와 동일 인덱스.
    """
    model: artec_base.ModelHandle
    scan:  artec_base.ScanHandle              # 현재 누적 중인 IScan (model 안)

    T_EB_list:    List[np.ndarray] = field(default_factory=list)
    theta_list:   List[float]      = field(default_factory=list)
    timestamps:   List[float]      = field(default_factory=list)
    surface_areas: List[float]     = field(default_factory=list)

    @property
    def n_frames(self) -> int:
        return self.scan.frame_count()


# ─────────────────────────────────────────────────────────────────────────────
# Session
# ─────────────────────────────────────────────────────────────────────────────

class ArtecScanSession:
    """
    Artec 전용 스캔 세션. Phase 1 (rule-based 회전) → Phase 2 (frontier NBV).

    PhoXi 의 `ScanSession` 과 동일한 책임이지만 자료구조가 Artec native 라
    별도 클래스로 분리.
    """

    def __init__(
        self,
        mms: "MMS",
        robot: Optional["XArmInterface"] = None,
        turntable: Optional["Turntable"] = None,
        settings: Optional[ArtecScanSessionSettings] = None,
    ):
        self.mms = mms
        self.robot = robot
        self.turntable = turntable
        self.s = settings or ArtecScanSessionSettings()

        # 빈 IModel + IScan 초기화 (model 안에 scan 1개)
        model = artec_base.create_model()
        scan  = artec_base.create_scan()
        model.add_scan(scan)
        self.ctx = ArtecScanContext(model=model, scan=scan)

        # 진행률 viewer (옵션) — Phase 1 매 capture 후 누적 pcd 갱신
        self._vis: Optional[ProgressVisualizer] = None

    # ── Entry ──────────────────────────────────────────────────────────

    def run(self) -> ArtecScanContext:
        self._prepare()
        if self.s.phase1_enabled:
            self._phase1()
        if self.s.phase2_enabled:
            if self.robot is None or self.turntable is None:
                print("[ArtecScanSession] Phase 2 skip — robot/turntable 필요")
            else:
                self._phase2()
        print(f"[ArtecScanSession] 완료 — {self.ctx.n_frames} frames in IScan")
        return self.ctx

    # ── 준비 ──────────────────────────────────────────────────────────

    def _prepare(self) -> None:
        # 턴테이블 θ=0 리셋 (이미 0 근처면 skip).
        # 시작 시 통신 불안정한 경우 종종 있어 — wait 실패하면 reconnect 후 재시도.
        if self.turntable is None or not self.turntable.is_connected:
            return

        # Pre-flight: GetAxisStatus 한 번 — 실패하면 미리 reconnect
        try:
            st = self.turntable.GetAxisStatus()
            if st is False and hasattr(self.turntable, "reconnect"):
                print("[ArtecScanSession] 시작 시 turntable 통신 불안정 — reconnect")
                self.turntable.reconnect()
        except Exception:
            pass

        cur = self._read_theta(fallback=0.0)
        if abs(cur) < np.radians(0.5):
            print(f"[ArtecScanSession] 턴테이블 이미 θ≈0 ({np.degrees(cur):+.3f}°)")
            return

        print(f"[ArtecScanSession] 턴테이블 θ=0 reset (현재 {np.degrees(cur):+.2f}°)")
        for attempt in range(2):
            ok = self.turntable.move_abs(0.0, float(self.s.turntable_vel_rad_s))
            if ok is False:
                if hasattr(self.turntable, "reconnect"):
                    self.turntable.reconnect()
                continue
            done = self.turntable.wait_motion_done(timeout_s=20.0)
            if done is True:
                return
            print(f"[ArtecScanSession] reset wait 실패 (시도 {attempt+1}/2) — reconnect")
            if hasattr(self.turntable, "reconnect"):
                self.turntable.reconnect()
        # 두 번 실패해도 그냥 진행 (Phase 1 첫 iter 가 다시 0 으로 명령)
        print("[ArtecScanSession] reset 미확인 — Phase 1 iter 1 에서 재명령")

    # ── 캡처 1회 ──────────────────────────────────────────────────────

    def _capture_one(
        self,
        i: int,
        theta_target_rad: float,
        move_turntable: bool = True,
    ) -> Optional[tuple]:
        """
        한 자세에서: turntable 이동 → settle → Artec capture → return
        (theta_act, T_EB, fmh).

        턴테이블 통신 실패 시 1회 reconnect 시도 후 재이동.
        """
        s = self.s
        if move_turntable and self.turntable is not None and self.turntable.is_connected:
            ok = self.turntable.move_abs(
                float(theta_target_rad), float(s.turntable_vel_rad_s),
            )
            if ok is False:
                # 통신 / drive 일시 오류일 가능성 → reconnect 1회 재시도
                if hasattr(self.turntable, "reconnect") and self.turntable.reconnect():
                    ok = self.turntable.move_abs(
                        float(theta_target_rad), float(s.turntable_vel_rad_s),
                    )
                if ok is False:
                    print("  ⚠ move_abs 거부 (reconnect 후에도) — skip")
                    return None
            done = self.turntable.wait_motion_done()
            if done is False:
                # axis status 폴링 실패 후 wait_motion_done 가 False — 다음 캡처 전에 reconnect
                print("  [Turntable] wait 실패 — reconnect 후 진행")
                if hasattr(self.turntable, "reconnect"):
                    self.turntable.reconnect()
        time.sleep(float(s.phase1_dwell_s))

        theta_act = self._read_theta(fallback=float(theta_target_rad))
        if abs(theta_act - theta_target_rad) > np.radians(2.0):
            print(f"  ⚠ θ 불일치 target={np.degrees(theta_target_rad):+.2f}° "
                  f"actual={np.degrees(theta_act):+.2f}°")

        try:
            fmh = self.mms.sensor.capture_frame(capture_texture=True)
        except RuntimeError as e:
            print(f"  ⚠ capture 실패: {e} — skip")
            return None
        if fmh is None or fmh.vertex_count() == 0:
            print("  ⚠ 빈 mesh — skip")
            return None

        T_EB = self.robot.get_ee_pose_mat() if self.robot is not None else np.eye(4)
        T_EB = np.asarray(T_EB, dtype=np.float64)
        return float(theta_act), T_EB, fmh

    def _read_theta(self, fallback: float) -> float:
        tt = self.turntable
        if tt is None or not getattr(tt, "is_connected", False):
            return float(fallback)
        v = tt.getActualPos()
        if isinstance(v, bool) or v is None:
            return float(fallback)
        return float(v)

    # ── Phase 1: rule-based 15° × 360° ────────────────────────────────

    def _phase1(self) -> None:
        s = self.s
        step = np.radians(float(s.phase1_theta_step_deg))
        n = int(round(2.0 * np.pi / step))
        thetas = [i * step for i in range(n)]

        print(f"\n═══════════════ Phase 1 — Rule-based {n} frames ═══════════════")
        print(f"  θ step={s.phase1_theta_step_deg:.1f}°  "
              f"turntable_vel={np.degrees(s.turntable_vel_rad_s):.1f}°/s  "
              f"dwell={s.phase1_dwell_s:.2f}s")

        # 진행률 viewer (non-blocking, 매 capture 후 누적 pcd 갱신)
        if s.phase1_show_progress and self._vis is None:
            try:
                self._vis = ProgressVisualizer(
                    window_title=f"Artec Phase 1 — accumulated pcd ({n} frames)",
                )
            except Exception as e:
                print(f"  [warn] ProgressVisualizer 생성 실패: {e}")
                self._vis = None

        try:
            for i, theta_tgt in enumerate(thetas):
                print(f"\n─── Phase 1 [{i+1}/{n}] θ_target={np.degrees(theta_tgt):+.1f}° ───")
                cap = self._capture_one(i, theta_tgt, move_turntable=True)
                if cap is None:
                    if self._vis is not None:
                        self._vis._pump()
                    continue
                theta_act, T_EB, fmh = cap

                # IScan 에 추가 (Artec native)
                self.ctx.scan.add_frame(fmh)
                self.ctx.T_EB_list.append(T_EB)
                self.ctx.theta_list.append(theta_act)
                self.ctx.timestamps.append(time.time())

                # frame transformation set — 우리 T_CO 를 SDK 알고리즘 입력으로
                # (translation m → mm, vertex convention z-flip 보정 포함)
                idx = self.ctx.n_frames - 1
                T_CO = self.mms.T_CO(theta_act, T_EB)
                T_CO_artec = T_CO @ ARTEC_TO_OPENCV          # z-flip 합성
                T_CO_mm = T_CO_artec.copy()
                T_CO_mm[:3, 3] *= 1000.0
                try:
                    self.ctx.scan.set_frame_transformation(idx, T_CO_mm)
                except Exception as e:
                    print(f"  ⚠ set_frame_transformation 실패: {e}")

                print(f"  ✓ frame[{idx}]  verts={fmh.vertex_count():,}  "
                      f"θ_act={np.degrees(theta_act):+.2f}°  "
                      f"T_CO t=[{T_CO_mm[0,3]:.0f},{T_CO_mm[1,3]:.0f},{T_CO_mm[2,3]:.0f}]mm")

                # 누적 pcd 시각화 — 우리 T_CO 로 O 프레임 배치
                if self._vis is not None:
                    merged = self._build_accumulated_pcd_O()
                    n_pts = len(merged.points)
                    print(f"  [vis] 누적 O-frame pcd: {n_pts:,} pts")
                    self._vis.update_pcd(merged)
                    T_CO_list = [
                        self.mms.T_CO(self.ctx.theta_list[k], self.ctx.T_EB_list[k])
                        for k in range(self.ctx.n_frames)
                    ]
                    self._vis.update_camera_trajectory(T_CO_list)
        finally:
            if self._vis is not None and s.phase1_wait_window_close:
                print("\n[Phase 1] 캡처 완료. 뷰어 닫으면 알고리즘 진행.")
                try:
                    self._vis.run_until_closed()
                except Exception:
                    pass
            elif self._vis is not None:
                self._vis.close()
            self._vis = None

    def _build_accumulated_pcd_O(self) -> o3d.geometry.PointCloud:
        """현재까지 누적된 frame 들을 우리 T_CO 로 O 프레임 배치한 PointCloud.

        Artec vertex 는 z-back convention 이므로 z-flip 을 합성한 T_CO 사용.
        """
        all_pts = []
        for i in range(self.ctx.n_frames):
            fmh = self.ctx.scan.get_frame(i)
            v_mm = fmh.vertices().astype(np.float64)
            v_m = v_mm / 1000.0
            T_CO = self.mms.T_CO(self.ctx.theta_list[i], self.ctx.T_EB_list[i])
            T_CO_artec = T_CO @ ARTEC_TO_OPENCV               # z-flip 합성
            R = T_CO_artec[:3, :3]; t = T_CO_artec[:3, 3]
            v_O = v_m @ R.T + t
            all_pts.append(v_O)
        if not all_pts:
            return o3d.geometry.PointCloud()
        pts = np.vstack(all_pts)
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pts)
        pcd = pcd.voxel_down_sample(0.001)
        return pcd

    # ── Phase 2: frontier NBV ─────────────────────────────────────────

    def _phase2(self) -> None:
        """
        매 step:
          1. fast_fusion 으로 임시 composite mesh 생성
          2. frontier 추출 → 후보 카메라 자세
          3. cost 최소 후보 실행 (theta_planner + execute_target)
          4. capture + IScan 에 추가
        """
        from utils.nbv.frontier import extract_frontier_candidates, extract_boundary_edges
        from utils.nbv.icp_strategy import pick_icp_roll
        from utils.nbv.manual_picker import compute_camera_pose_from_normal
        from utils.control.theta_planner import plan_min_motion_theta

        s = self.s
        print(f"\n═══════════════ Phase 2 — Frontier NBV (max {s.phase2_K_max}) ═══════════════")

        for k in range(s.phase2_K_max):
            print(f"\n─── Phase 2 [{k+1}/{s.phase2_K_max}] ───")

            # 1. fast_fusion → 임시 mesh
            try:
                tmp_model = ArtecClient.fast_fusion(self.ctx.model)
                verts_mm = tmp_model.final_vertices()
                faces = tmp_model.final_faces()
            except Exception as e:
                print(f"  fast_fusion 실패: {e} — Phase 2 종료")
                break

            mesh = o3d.geometry.TriangleMesh()
            mesh.vertices = o3d.utility.Vector3dVector(verts_mm.astype(np.float64) / 1000.0)
            mesh.triangles = o3d.utility.Vector3iVector(faces.astype(np.int32))
            mesh.compute_vertex_normals()
            area = float(mesh.get_surface_area()) if len(mesh.triangles) else 0.0
            self.ctx.surface_areas.append(area)

            # 종료 조건 체크
            if self._terminate(mesh):
                print("[Phase 2] 종료 조건 충족 — 루프 종료")
                break

            # 2. frontier 추출
            cands = extract_frontier_candidates(
                mesh,
                min_seg_vertices=s.min_seg_vertices,
                min_seg_length=s.min_seg_length,
                max_seg_length=s.max_seg_length,
            )
            if not cands:
                print("[Phase 2] frontier 후보 없음 — 종료")
                break
            print(f"  frontier 후보: {len(cands)}")

            # n_O[2] < 0 강제 (외부에서 안쪽 보는 방향)
            for c in cands:
                if c.n_O[2] > 0:
                    c.n_O = -c.n_O

            # 3. 후보 ranking (cost 최소)
            theta_cur = self.ctx.theta_list[-1] if self.ctx.theta_list else 0.0
            T_CO_cur = self._last_T_CO()
            R_cur = T_CO_cur[:3, :3]; p_cur = T_CO_cur[:3, 3]
            L_max = max((c.L for c in cands), default=1e-9)

            evaluated = []
            for c in cands:
                roll_i = pick_icp_roll(
                    p_i_O=c.p_O, n_i_O=c.n_O, p_cam_prev_O=p_cur,
                    distance_m=s.distance_m, R_CO_prev=R_cur,
                    fallback_thresh=s.roll_fallback_thresh,
                )
                T_CO_i = compute_camera_pose_from_normal(
                    surface_point=c.p_O, normal=c.n_O,
                    distance_m=s.distance_m, roll_rad=roll_i,
                )
                plan = plan_min_motion_theta(
                    T_CO_des=T_CO_i,
                    T_OF=self.mms._T_OF, T_EC=self.mms._T_EC,
                    tt=self.mms.turntable_transform,
                    robot=self.robot, theta_current=theta_cur,
                    theta_range=s.theta_range, n_samples=s.theta_n_samples,
                    w_tt=s.planner_w_tt, verbose=False,
                )
                if plan["theta"] is None:
                    continue
                dp = T_CO_i[:3, 3] - p_cur
                cost = (
                    s.cost_alpha * float(dp @ dp)
                    + s.cost_beta * 0.0           # 회전 비용 단순화
                    - s.cost_delta * (c.L / L_max)
                )
                evaluated.append((c, T_CO_i, plan, cost))
            if not evaluated:
                print("[Phase 2] feasible 후보 없음 — 종료")
                break
            evaluated.sort(key=lambda x: x[3])

            # 4. 후보 순서대로 실행 시도
            success = False
            for rank, (cand, T_CO_i, plan_i, cost_i) in enumerate(evaluated):
                print(f"  후보 #{rank+1}/{len(evaluated)}  "
                      f"cost={cost_i:.4f}  θ*={np.degrees(plan_i['theta']):+.2f}°")
                result = self.mms.execute_target(
                    T_CO_des=T_CO_i, theta=plan_i["theta"],
                    robot=self.robot, turntable=self.turntable,
                    robot_speed=s.robot_speed_deg_s,
                    turntable_vel_rad_s=s.turntable_vel_rad_s,
                    move_turntable=True, move_robot=True,
                    confirm=s.confirm_each_move,
                )
                if result is None or not result.get("ik_ok", False):
                    print("    실행 실패, 다음 후보")
                    continue

                cap = self._capture_one(self.ctx.n_frames, plan_i["theta"],
                                        move_turntable=False)
                if cap is None:
                    print("    캡처 실패, 다음 후보")
                    continue
                theta_act, T_EB, fmh = cap

                self.ctx.scan.add_frame(fmh)
                self.ctx.T_EB_list.append(T_EB)
                self.ctx.theta_list.append(theta_act)
                self.ctx.timestamps.append(time.time())

                print(f"  ✓ frame[{self.ctx.n_frames-1}]  verts={fmh.vertex_count():,}")
                success = True
                break

            if not success:
                print("[Phase 2] 모든 후보 소진 — 종료")
                break

    # ── 유틸 ──────────────────────────────────────────────────────────

    def _last_T_CO(self) -> np.ndarray:
        """가장 최근 자세의 T_CO."""
        if not self.ctx.theta_list:
            return np.eye(4)
        theta = self.ctx.theta_list[-1]
        T_EB  = self.ctx.T_EB_list[-1]
        return self.mms.T_CO(theta, T_EB)

    def _terminate(self, mesh: o3d.geometry.TriangleMesh) -> bool:
        """plateau / boundary length 기준 종료 판정."""
        s = self.s
        # plateau
        if len(self.ctx.surface_areas) >= s.plateau_window + 1:
            recent = self.ctx.surface_areas[-(s.plateau_window + 1):]
            growths = [
                (recent[i+1] - recent[i]) / max(recent[i], 1e-9)
                for i in range(s.plateau_window)
            ]
            if all(g < s.plateau_growth_thresh for g in growths):
                print(f"[Phase 2] plateau — recent {growths}")
                return True
        # boundary
        from utils.nbv.frontier import extract_boundary_edges
        b = extract_boundary_edges(mesh)
        if len(b) == 0:
            return True
        v = np.asarray(mesh.vertices)
        d = v[b[:, 0]] - v[b[:, 1]]
        total_len = float(np.linalg.norm(d, axis=1).sum())
        if total_len < s.boundary_length_stop:
            print(f"[Phase 2] boundary {total_len*1000:.1f}mm — watertight 근접")
            return True
        return False
