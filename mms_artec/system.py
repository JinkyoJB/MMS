# mms_artec/system.py
#
# Artec 전용 MMS — Artec Spider/3D 스캐너 + xArm7 + 턴테이블 통합.
# IScanningProcedure 기반 Phase 1 + 후처리 알고리즘 파이프라인.
#
# Notation: T_AB : A → B  (x_B = T_AB @ x_A) — README.md / CLAUDE.md 준수.

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Optional, TYPE_CHECKING

import numpy as np

from utils.transforms import (
    TurntableTransformConfig,
    load_transform,
    compute_T_CB,
    compute_T_CO,
    solve_T_EB,
)
from utils.control.hardware_layer import execute_camera_target
from utils.control.theta_planner import plan_min_motion_theta, DEFAULT_JOINT_WEIGHTS

# 백엔드(real/isaac) robot/turntable/sensor 팩토리. 하드웨어/시뮬 의존 모듈은
# 이 안에서 lazy import → Linux(Isaac)/Windows(real) 양쪽에서 system.py import 가능.
from mms_artec.backends import build_hardware, build_sensor

# ArtecConfig 는 경량 모듈(바인딩 비의존). 런타임에는 cfg.artec 로 주입되므로
# 타입 힌트로만 필요 (from __future__ import annotations → 문자열 평가).
if TYPE_CHECKING:
    from mms_artec.sensor.artec_config import ArtecConfig


# ─────────────────────────────────────────────────────────────────────────────
# Config
# ─────────────────────────────────────────────────────────────────────────────

@dataclass
class ArtecMMSConfig:
    """
    Artec MMS 설정.

    Attributes
    ----------
    artec : ArtecConfig
        Artec 스캐너 설정.
    sensor_frames_yaml : str, optional
        T_EC 가 들어있는 yaml. T_EC_key 와 함께 지정.
    T_EC_key : str, optional
        sensor_frames_yaml 안의 T_EC 키 이름 (예: "T_EC_artec").
    turntable_frame_yaml : str, optional
        T_B_F0 yaml. 지정하면 turntable_transform 활성화.
    object_frame_yaml : str, optional
        T_O_F0 yaml. None 이면 identity (O ≡ F at θ=0).
    """
    artec: "ArtecConfig"
    sensor_frames_yaml: Optional[str] = None
    T_EC_key: Optional[str] = None
    turntable_frame_yaml: Optional[str] = None
    object_frame_yaml: Optional[str] = None

    # ── 백엔드 선택 ──────────────────────────────────────────────────────
    # "real"  → 실물 xArm + Ezi-SERVO 턴테이블 + Artec 스캐너 (Windows)
    # "isaac" → Isaac Sim 시뮬레이션 (robot/turntable/scanner 전부)
    backend: str = "real"

    # real 전용 — 하드웨어 주소
    robot_ip: str = "192.168.1.210"
    turntable_ip: str = "192.168.0.10"
    turntable_bd_id: int = 0

    # isaac 전용 — 시뮬 옵션
    isaac_usd_path: Optional[str] = None        # None → 백엔드 기본 USD
    isaac_headless: bool = False
    isaac_robot_collisions: bool = False        # 로봇 링크 물리충돌 on/off
                                                # (키네마틱 로봇 — 닿는 자세선 jitter 주의)

    def __post_init__(self):
        if self.sensor_frames_yaml is not None:
            self.sensor_frames_yaml = str(Path(self.sensor_frames_yaml).resolve())
            if self.T_EC_key is None:
                raise ValueError("sensor_frames_yaml 지정 시 T_EC_key 필요.")
        if self.turntable_frame_yaml is not None:
            self.turntable_frame_yaml = str(Path(self.turntable_frame_yaml).resolve())
        if self.object_frame_yaml is not None:
            self.object_frame_yaml = str(Path(self.object_frame_yaml).resolve())


# ─────────────────────────────────────────────────────────────────────────────
# ArtecMMS — Artec 전용 orchestrator
# ─────────────────────────────────────────────────────────────────────────────

class ArtecMMS:
    """
    Artec 3D 스캐너 + xArm7 + 턴테이블 통합 클래스.

    Pipeline (`docs/6_artec_process.md` §3 SDK General Pipeline):
      0. Scanning  — IScanningProcedure (streaming) 또는 discrete
      1. Alignment — SerialRegistration
      2. Registration — GlobalRegistration
      3. Cleaning — Outliers + SmallObjects
      4. Fusion — Poisson / Fast
      5. Simplify
      6. Texturize

    Frame transforms (좌표계 변환 일람):
      T_FB(theta) — F → B at turntable θ
      T_BF(theta) — B → F at θ
      T_CB(T_EB)  — C → B
      T_CO(theta, T_EB) — C → O (NBV interface)
      solve_T_EB(theta, T_CO_des) — E → B  (NBV → robot target)
    """

    def __init__(self, cfg: ArtecMMSConfig) -> None:
        self.cfg = cfg
        # 백엔드에 맞는 sensor (real=ArtecClient, isaac=IsaacArtecScanner)
        self.sensor = build_sensor(cfg)

        # T_EC: E → C
        if cfg.sensor_frames_yaml is not None:
            self._T_EC: Optional[np.ndarray] = load_transform(
                cfg.sensor_frames_yaml, cfg.T_EC_key
            )
        else:
            self._T_EC = None

        # B ↔ F
        if cfg.turntable_frame_yaml is not None:
            T_BF0 = load_transform(cfg.turntable_frame_yaml, "T_B_F0")
            self.turntable_transform: Optional[TurntableTransformConfig] = (
                TurntableTransformConfig(T_BF0)
            )
        else:
            self.turntable_transform = None

        # O → F
        if cfg.object_frame_yaml is not None:
            self._T_OF: np.ndarray = load_transform(cfg.object_frame_yaml, "T_O_F0")
        else:
            self._T_OF = np.eye(4, dtype=float)

    # ── hardware factory ───────────────────────────────────────────────

    def create_hardware(self):
        """
        cfg.backend 에 맞는 (robot, turntable) 생성.

        real  → XArmInterface + 연결된 Turntable
        isaac → IsaacXArm + IsaacTurntable (공유 IsaacWorld)

        main_artec.py 가 직접 하드웨어를 생성하던 것을 대체한다.
        """
        return build_hardware(self.cfg)

    # ── turntable 축 calibration (T_B_F0) — sim/real 공통 ───────────────

    def calibrate_turntable_axis(
        self, robot, turntable, *, sphere_radius, sphere_z_bands,
        thetas_deg=None, turntable_vel_rad_s=np.pi / 6.0,
        z_band=0.02, z_floor=None, return_clouds=False,
    ) -> dict:
        """
        구 fixture 기반 턴테이블 회전축(=T_B_F0 축) calibration. **base 프레임** 축 반환.

        sim/real 동일 경로:
          θ 마다 → turntable 회전 → sensor.capture_points_base(robot, T_EC) [base 프레임]
                 → z밴드로 구 분리 → known-R 구중심 피팅 → 축 추정.
        ★ Artec first-frame 추적(SLAM) 미사용 — 카메라 위치는 EE FK + T_EC 로만.

        선행조건: fixture(구)가 턴테이블에 부착돼 있고, 로봇이 fixture 를 보는 자세.

        Parameters
        ----------
        sphere_radius : 구 반경 (m)
        sphere_z_bands : 구들의 대략적 base-프레임 z 높이 리스트 (세그먼트용)
        thetas_deg : 캡처 각도들 (기본 0..330 step 30)
        z_band : z밴드 반폭 (m)
        z_floor : 이 z 아래 점 제거 (디스크/바닥). 기본 min(z_bands)-0.05

        Returns
        -------
        {"axis_point","axis_dir","n_obs","tracks"}  — axis_point/dir 은 base 프레임
        """
        from utils.calibration.turntable_axis import fit_sphere_center, estimate_axis
        if self._T_EC is None:
            raise RuntimeError("T_EC 미설정 — sensor_frames_yaml/T_EC_key 필요.")
        if thetas_deg is None:
            thetas_deg = list(range(0, 360, 30))
        if z_floor is None:
            z_floor = min(sphere_z_bands) - 0.05

        tracks = [[] for _ in sphere_z_bands]
        clouds = []                           # (deg, sphere_i, band_points) — return_clouds 시
        for deg in thetas_deg:
            turntable.move_abs(float(np.radians(deg)), float(turntable_vel_rad_s))
            turntable.wait_motion_done()
            pts = self.sensor.capture_points_base(robot, self._T_EC)
            if len(pts) == 0:
                continue
            up = pts[pts[:, 2] > z_floor]
            for i, zt in enumerate(sphere_z_bands):
                band = up[np.abs(up[:, 2] - zt) < z_band]
                if len(band) > 30:
                    tracks[i].append(fit_sphere_center(band, sphere_radius))
                    if return_clouds:
                        clouds.append((int(deg), i, band))

        axis_point, axis_dir = estimate_axis(tracks)
        out = {"axis_point": axis_point, "axis_dir": axis_dir,
               "n_obs": [len(t) for t in tracks], "tracks": tracks}
        if return_clouds:
            out["clouds"] = clouds
        return out

    def disc_surface_frame(self, disc_points_base, axis_point, axis_dir):
        """
        disc 표면 점군(base 프레임) + 3구 회전축 → 완전한 턴테이블 프레임 T_B_F0.

        3구 방법은 회전축(방향+XY)만 준다(궤적 높이=구 높이≠표면). 충돌 회피와
        대상물 기준 높이를 위해 disc **표면**이 필요하다. disc 점군에 평면을 피팅해
          - 표면 높이: 회전축이 평면과 만나는 점 = F0 원점(표면 위 중심)
          - 법선 교차검증: 평면 법선 vs 구-축 방향 (어긋나면 disc 점군/축 의심)
        축 방향은 구 피팅이 더 강건하므로 **Z축은 axis_dir**, 원점만 표면으로 내린다.

        Parameters
        ----------
        disc_points_base : (N,3) disc 표면 점들 (로봇 base 프레임, m). 구/기둥 점은
                           미리 크롭(z<구높이)해서 표면만 줄 것.
        axis_point, axis_dir : calibrate_turntable_axis 결과(base 프레임).

        Returns
        -------
        {"T_B_F0", "surface_point", "plane_normal", "plane_rms",
         "normal_axis_angle_deg", "surface_height"}  — 모두 base 프레임.
        """
        from utils.calibration.turntable_frame import fit_plane, build_T_B_F0
        P = np.asarray(disc_points_base, float)
        plane_pt, plane_n, rms = fit_plane(P)
        d = np.asarray(axis_dir, float); d = d / (np.linalg.norm(d) + 1e-12)
        ap = np.asarray(axis_point, float)
        # 축 법선 교차검증(부호 무시)
        ang = float(np.degrees(np.arccos(np.clip(abs(d @ plane_n), -1.0, 1.0))))
        # 회전축 라인이 평면과 만나는 점 = 표면 위 중심(원점)
        denom = float(d @ plane_n)
        if abs(denom) < 1e-6:
            surface_pt = ap.copy()                    # 축이 평면과 평행(이상) — fallback
        else:
            t = float((plane_pt - ap) @ plane_n) / denom
            surface_pt = ap + t * d
        T_B_F0 = build_T_B_F0(surface_pt, d)           # Z=강건한 구-축, 원점=표면
        return {"T_B_F0": T_B_F0, "surface_point": surface_pt,
                "plane_normal": plane_n, "plane_rms": float(rms),
                "normal_axis_angle_deg": ang, "surface_height": float(surface_pt[2])}

    # ── 자세별 충돌 쿼리 (real/sim 공용) ────────────────────────────────
    def turntable_collision_world(self, surface_point, axis_dir, disc_radius,
                                  body_height=0.20, object_radius=0.0,
                                  object_height=0.0, margin=0.0):
        """
        calibration(표면+축, base 프레임)으로 충돌 월드 생성(turntable[+object] 캡슐).
        utils.collision.CollisionWorld.from_turntable 래퍼 — robot 백엔드 무관.
        """
        from utils.collision import CollisionWorld
        return CollisionWorld.from_turntable(
            surface_point, axis_dir, disc_radius, body_height=body_height,
            object_radius=object_radius, object_height=object_height, margin=margin)

    def check_pose_collision(self, robot, world, q=None,
                             ignore=("link1", "link2"), margin=0.0):
        """
        현재(또는 q) 자세의 충돌 검사 → CollisionResult. real/sim 공통.

        q 미지정: 현재 자세 — robot.collision_capsules()(sim:USD 실제포즈 / real:FK).
        q 지정  : 가상 자세 — 해석 FK 캡슐(capsules_from_joints)로 이동 없이 사전검사.
                  해석 FK 가 USD 와 ~3mm 정합하므로 real/sim 동일하게 동작.
        """
        if q is not None:
            # 가상 자세(pre-move): 해석 FK 캡슐(USD 정합 ~3mm). 이동 불필요, real/sim 동일.
            from utils.collision import capsules_from_joints
            caps = capsules_from_joints(np.asarray(q, float), T_EC=self._T_EC)
        else:
            caps = robot.collision_capsules()           # 현재 자세(sim:USD / real:FK)
        return world.check(caps, margin=margin, ignore=ignore)

    # ── lifecycle ──────────────────────────────────────────────────────

    def initialize(self) -> None:
        self.sensor.initialize()

    def shutdown(self) -> None:
        self.sensor.shutdown()

    def __enter__(self) -> "ArtecMMS":
        self.initialize()
        return self

    def __exit__(self, *_) -> None:
        self.shutdown()

    # ── frame transforms ───────────────────────────────────────────────

    def T_FB(self, theta: float) -> np.ndarray:
        if self.turntable_transform is None:
            raise RuntimeError("turntable_transform 미설정.")
        return self.turntable_transform.T_FB(theta)

    def T_BF(self, theta: float) -> np.ndarray:
        if self.turntable_transform is None:
            raise RuntimeError("turntable_transform 미설정.")
        return self.turntable_transform.T_BF(theta)

    def T_CB(self, T_EB: np.ndarray) -> np.ndarray:
        if self._T_EC is None:
            raise RuntimeError("T_EC 미설정.")
        return compute_T_CB(T_EB, self._T_EC)

    def T_CO(self, theta: float, T_EB: np.ndarray) -> np.ndarray:
        if self.turntable_transform is None or self._T_EC is None:
            raise RuntimeError("turntable_transform / T_EC 미설정.")
        return compute_T_CO(theta, T_EB, self._T_OF, self._T_EC, self.turntable_transform)

    def solve_T_EB(self, theta: float, T_CO_des: np.ndarray) -> np.ndarray:
        if self.turntable_transform is None or self._T_EC is None:
            raise RuntimeError("turntable_transform / T_EC 미설정.")
        return solve_T_EB(theta, T_CO_des, self._T_OF, self._T_EC, self.turntable_transform)

    def set_object_frame(self, T_OF: np.ndarray) -> None:
        assert T_OF.shape == (4, 4)
        self._T_OF = T_OF.astype(float)

    # ── 하위 레이어: planner + executor ────────────────────────────────

    def execute_target(
        self,
        T_CO_des: np.ndarray,
        theta: float,
        robot=None,
        turntable=None,
        robot_speed: float = 30.0,
        turntable_vel_rad_s: float = np.pi / 6.0,
        move_turntable: bool = True,
        move_robot: bool = True,
        confirm: bool = True,
    ) -> dict:
        """T_CO_des + θ 를 실제 로봇/턴테이블 명령으로 실행."""
        if self.turntable_transform is None or self._T_EC is None:
            raise RuntimeError("turntable_transform / T_EC 필요.")
        return execute_camera_target(
            T_CO_des=T_CO_des, theta=theta,
            T_OF=self._T_OF, T_EC=self._T_EC,
            tt=self.turntable_transform,
            robot=robot, turntable=turntable,
            robot_speed=robot_speed, turntable_vel_rad_s=turntable_vel_rad_s,
            move_turntable=move_turntable, move_robot=move_robot, confirm=confirm,
        )

    def plan_and_execute(
        self,
        T_CO_des: np.ndarray,
        robot,
        turntable=None,
        theta_current: Optional[float] = None,
        q_current: Optional[np.ndarray] = None,
        theta_range: tuple = (-np.pi, np.pi),
        n_samples: int = 72,
        joint_weights: Optional[np.ndarray] = None,
        w_tt: float = 0.0,
        robot_speed: float = 30.0,
        turntable_vel_rad_s: float = np.pi / 6.0,
        move_turntable: bool = True,
        move_robot: bool = True,
        confirm: bool = True,
    ) -> dict:
        """θ 최적화 + 실행. Phase 2 NBV 가 사용."""
        if self.turntable_transform is None or self._T_EC is None:
            raise RuntimeError("turntable_transform / T_EC 필요.")
        if robot is None:
            raise ValueError("robot 필요.")

        if theta_current is None:
            if turntable is not None and getattr(turntable, "is_connected", False):
                theta_current = float(turntable.getActualPos())
            else:
                theta_current = 0.0

        plan = plan_min_motion_theta(
            T_CO_des=T_CO_des,
            T_OF=self._T_OF, T_EC=self._T_EC,
            tt=self.turntable_transform,
            robot=robot, theta_current=float(theta_current),
            q_current=q_current,
            theta_range=theta_range, n_samples=n_samples,
            joint_weights=(DEFAULT_JOINT_WEIGHTS if joint_weights is None else joint_weights),
            w_tt=float(w_tt), verbose=True,
        )
        if plan["theta"] is None:
            return {"plan": plan, "exec": None}

        exec_result = self.execute_target(
            T_CO_des=T_CO_des, theta=plan["theta"],
            robot=robot, turntable=turntable,
            robot_speed=robot_speed, turntable_vel_rad_s=turntable_vel_rad_s,
            move_turntable=move_turntable, move_robot=move_robot, confirm=confirm,
        )
        return {"plan": plan, "exec": exec_result}

    # ── Artec process pipeline ─────────────────────────────────────────

    def artec_process(
        self,
        robot,
        turntable,
        settings: Optional["ArtecProcessSettings"] = None,
    ) -> "ArtecProcessResult":
        """
        Artec 풀 파이프라인. `docs/6_artec_process.md` 부록 B SDK General
        Pipeline 순서:
          Scan (streaming/discrete) → SerialReg → GlobalReg → Cleaning
          → Fusion → Simplify → Texturize → Export
        """
        s = settings or ArtecProcessSettings()

        # ── 0. Scanning ─────────────────────────────────────────────────
        ctx = None
        if self.cfg.backend == "isaac":
            # Isaac 백엔드: Artec SLAM(IScanningProcedure) 대신 sim 스캔 경로.
            # 턴테이블 ground-truth θ + 카메라 포즈로 포인트클라우드를 객체 프레임에
            # 직접 누적한다. (Phase B: mms_artec/backends/isaac/isaac_scan_session.py)
            from mms_artec.backends.isaac.isaac_scan_session import IsaacScanSession
            session = IsaacScanSession(self, robot, turntable, s)
            sim_result = session.run()
            model = sim_result.model
            print(f"\n[artec_process] (sim) Scan — {sim_result.n_frames} frames")
        elif s.use_streaming_scan:
            if s.use_multipass_scan:
                # Multi-pass: Phase 1 재진행 (tracking lost recovery) + Phase 2
                # (flip 후 바닥면 스캔). 모든 IScan 이 master IModel 에 누적되고
                # 아래 GlobalRegistration 단계에서 정합됨.
                from mms_artec.nbv.artec_multipass_scan_session import (
                    ArtecMultiPassScanSession,
                )
                session = ArtecMultiPassScanSession(
                    self, robot, turntable, s.multipass_settings,
                )
                multi_result = session.run()
                model = multi_result.model
                print(f"\n[artec_process] Multi-pass Scan — "
                      f"{multi_result.n_passes} passes, "
                      f"{multi_result.n_total_frames} total frames "
                      f"({model.scan_count()} scan(s))")
                # Hints 가 적용됐다면 GlobalReg 가 hint 를 흐트러뜨릴 수 있음.
                # hint 가 authoritative 이므로 post-merge GlobalReg 자동 skip.
                if multi_result.hints_applied and s.do_global_registration:
                    print("[artec_process] ⓘ hints_applied=True → "
                          "post-merge GlobalRegistration 자동 skip "
                          "(hint 가 authoritative)")
                    s.do_global_registration = False
            else:
                from mms_artec.nbv.artec_streaming_scan_session import (
                    ArtecStreamingScanSession,
                )
                session = ArtecStreamingScanSession(
                    self, robot, turntable, s.streaming_scan_settings,
                )
                stream_result = session.run()
                model = stream_result.model
                print(f"\n[artec_process] Streaming Scan — {stream_result.n_frames} frames "
                      f"({stream_result.fps_actual:.1f} fps)")
        else:
            from mms_artec.nbv.artec_scan_session import ArtecScanSession
            session = ArtecScanSession(self, robot, turntable, s.scan_settings)
            ctx = session.run()
            model = ctx.model
            print(f"\n[artec_process] Discrete Scan — {ctx.n_frames} frames")

        def _safe(name: str, fn, current_model):
            print(f"[artec_process] {name} ...")
            try:
                out = fn(current_model)
                try:
                    n_scans = out.scan_count()
                    n_frames = sum(out.get_scan(i).frame_count() for i in range(n_scans))
                    has_mesh = out.has_final_mesh()
                    print(f"   ok  scans={n_scans}  frames={n_frames}  composite={has_mesh}")
                except Exception:
                    pass
                return out
            except RuntimeError as e:
                print(f"   ⚠ {name} 실패: {e}")
                print(f"   → skip")
                return current_model

        def _save_mid(tag: str, current_model):
            if not s.export_sproj_path:
                return
            try:
                from pathlib import Path as _P
                p = _P(s.export_sproj_path)
                p.parent.mkdir(parents=True, exist_ok=True)
                mid = str(p.with_stem(p.stem + f"_{tag}"))
                _P(mid).unlink(missing_ok=True)
                self.sensor.save_project(current_model, mid)
                print(f"   mid-save → {mid}")
            except Exception as _e:
                print(f"   mid-save 실패 (무시): {_e}")

        # 1-2. Reg
        if s.do_serial_registration:
            model = _safe("SerialRegistration", self.sensor.serial_registration, model)
        if s.do_global_registration:
            model = _safe("GlobalRegistration", self.sensor.global_registration, model)
        _save_mid("post_reg", model)

        # 3. Cleaning (Fusion 전)
        if s.do_outliers_removal:
            model = _safe("OutliersRemoval", self.sensor.outliers_removal, model)
        if s.do_small_objects_filter:
            model = _safe("SmallObjectsFilter", self.sensor.small_objects_filter, model)
        _save_mid("pre_fusion", model)

        # 4. Fusion
        fusion = (s.fusion or "none").lower()
        if fusion == "poisson":
            model = _safe("PoissonFusion", self.sensor.poisson_fusion, model)
        elif fusion == "fast":
            model = _safe("FastFusion", self.sensor.fast_fusion, model)

        # 5-6. Simplify + Texturize
        if s.do_simplify:
            model = _safe("MeshSimplify", self.sensor.mesh_simplify, model)
        if s.do_texturize:
            model = _safe("Texturize", self.sensor.texturize, model)

        # Export
        if s.export_obj_path:
            from pathlib import Path as _P
            _P(s.export_obj_path).parent.mkdir(parents=True, exist_ok=True)
            try:
                model.save_obj(s.export_obj_path)
                print(f"[artec_process] saved OBJ → {s.export_obj_path}")
            except Exception as e:
                print(f"[artec_process] OBJ 저장 실패: {e}")
        if s.export_sproj_path:
            from pathlib import Path as _P
            _P(s.export_sproj_path).parent.mkdir(parents=True, exist_ok=True)
            _P(s.export_sproj_path).unlink(missing_ok=True)
            try:
                self.sensor.save_project(model, s.export_sproj_path)
                print(f"[artec_process] saved sproj → {s.export_sproj_path}")
            except RuntimeError as e:
                print(f"[artec_process] sproj 저장 실패: {e}")

        return ArtecProcessResult(model=model, ctx=ctx)


# ─────────────────────────────────────────────────────────────────────────────
# ArtecProcessSettings / Result
# ─────────────────────────────────────────────────────────────────────────────

@dataclass
class ArtecProcessSettings:
    """`ArtecMMS.artec_process()` 통합 설정."""

    # ── 개발/Production 모드 ──────────────────────────────────────────
    # dev_mode=True 면 무거운 후처리 (OutliersRemoval, Simplify) 를 자동 skip.
    # 빠른 iteration 용. 본 export 에선 False 로 되돌릴 것.
    # __post_init__ 에서 do_* 플래그를 강제 override 함 (dev_mode 가 우선).
    dev_mode: bool = False

    use_streaming_scan: bool = True
    # Multi-pass: tracking-lost 재진행 + Phase 2 (flip 후 바닥면 스캔). True 가
    # 새 default — 한 번에 끝내고 싶으면 multipass_settings.prompt_*_=False 로
    # 끄거나 use_multipass_scan=False 로 단일 streaming session 사용.
    use_multipass_scan: bool = True

    scan_settings: Optional["ArtecScanSessionSettings"] = None              # type: ignore[name-defined]
    streaming_scan_settings: Optional["ArtecStreamingScanSessionSettings"] = None    # type: ignore[name-defined]
    multipass_settings: Optional["ArtecMultiPassScanSessionSettings"] = None         # type: ignore[name-defined]

    do_serial_registration: bool = True
    do_global_registration: bool = True

    fusion: str = "poisson"

    do_outliers_removal: bool = True
    do_small_objects_filter: bool = True
    do_simplify: bool = True
    do_texturize: bool = True

    export_obj_path:   Optional[str] = None
    export_sproj_path: Optional[str] = None

    def __post_init__(self):
        if self.scan_settings is None:
            from mms_artec.nbv.artec_scan_session import ArtecScanSessionSettings as _S
            self.scan_settings = _S()
        if self.streaming_scan_settings is None:
            from mms_artec.nbv.artec_streaming_scan_session import (
                ArtecStreamingScanSessionSettings as _SS,
            )
            self.streaming_scan_settings = _SS()
        if self.multipass_settings is None:
            from mms_artec.nbv.artec_multipass_scan_session import (
                ArtecMultiPassScanSessionSettings as _MPS,
            )
            # multipass 안의 streaming_settings 가 위 streaming_scan_settings 와
            # 같은 인스턴스를 공유하도록 — main_artec.py 의 사용자 설정이 그대로 반영.
            self.multipass_settings = _MPS(
                streaming_settings=self.streaming_scan_settings,
            )

        # Dev mode 강제 override — 무거운 단계 skip.
        if self.dev_mode:
            self.do_outliers_removal = False    # frame 별 neighborhood, 5분+
            self.do_simplify = False            # mesh simplification
            # do_small_objects_filter / do_texturize 는 유지 — 비교적 빠르고
            # 결과 검증에 도움. PoissonFusion 도 유지 (mesh 결과 자체).
            print("[ArtecProcessSettings] ⚡ dev_mode ON — "
                  "do_outliers_removal=False, do_simplify=False")


@dataclass
class ArtecProcessResult:
    """artec_process 결과."""
    model: object                                        # ModelHandle (artec_base)
    ctx:   Optional[object] = None                       # ArtecScanContext (discrete only)

    def composite_mesh_o3d(self):
        """Composite mesh 를 Open3D TriangleMesh 로 변환 (mm → m)."""
        import open3d as o3d
        if not self.model.has_final_mesh():
            return o3d.geometry.TriangleMesh()
        verts = self.model.final_vertices().astype(np.float64) / 1000.0
        faces = self.model.final_faces().astype(np.int32)
        m = o3d.geometry.TriangleMesh()
        m.vertices = o3d.utility.Vector3dVector(verts)
        m.triangles = o3d.utility.Vector3iVector(faces)
        m.compute_vertex_normals()
        return m
