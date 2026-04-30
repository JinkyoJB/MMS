# mms_artec/system.py
#
# Artec 전용 MMS — Artec Spider/3D 스캐너 + xArm7 + 턴테이블 통합.
# IScanningProcedure 기반 Phase 1 + 후처리 알고리즘 파이프라인.
#
# Notation: T_AB : A → B  (x_B = T_AB @ x_A) — README.md / CLAUDE.md 준수.

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Optional

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

from mms_artec.sensor.artec_client import ArtecClient, ArtecConfig


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
    artec: ArtecConfig
    sensor_frames_yaml: Optional[str] = None
    T_EC_key: Optional[str] = None
    turntable_frame_yaml: Optional[str] = None
    object_frame_yaml: Optional[str] = None

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
        self.sensor: ArtecClient = ArtecClient(cfg.artec)

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
        if s.use_streaming_scan:
            from mms_artec.nbv.artec_streaming_scan_session import (
                ArtecStreamingScanSession,
            )
            session = ArtecStreamingScanSession(self, robot, turntable, s.streaming_scan_settings)
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
                ArtecClient.save_project(current_model, mid)
                print(f"   mid-save → {mid}")
            except Exception as _e:
                print(f"   mid-save 실패 (무시): {_e}")

        # 1-2. Reg
        if s.do_serial_registration:
            model = _safe("SerialRegistration", ArtecClient.serial_registration, model)
        if s.do_global_registration:
            model = _safe("GlobalRegistration", ArtecClient.global_registration, model)
        _save_mid("post_reg", model)

        # 3. Cleaning (Fusion 전)
        if s.do_outliers_removal:
            model = _safe("OutliersRemoval", ArtecClient.outliers_removal, model)
        if s.do_small_objects_filter:
            model = _safe("SmallObjectsFilter", ArtecClient.small_objects_filter, model)
        _save_mid("pre_fusion", model)

        # 4. Fusion
        fusion = (s.fusion or "none").lower()
        if fusion == "poisson":
            model = _safe("PoissonFusion", ArtecClient.poisson_fusion, model)
        elif fusion == "fast":
            model = _safe("FastFusion", ArtecClient.fast_fusion, model)

        # 5-6. Simplify + Texturize
        if s.do_simplify:
            model = _safe("MeshSimplify", ArtecClient.mesh_simplify, model)
        if s.do_texturize:
            model = _safe("Texturize", ArtecClient.texturize, model)

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
                ArtecClient.save_project(model, s.export_sproj_path)
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

    use_streaming_scan: bool = True

    scan_settings: Optional["ArtecScanSessionSettings"] = None              # type: ignore[name-defined]
    streaming_scan_settings: Optional["ArtecStreamingScanSessionSettings"] = None    # type: ignore[name-defined]

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
