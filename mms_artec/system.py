# mms_artec/system.py
#
# Artec 전용 MMS — Artec Spider/3D 스캐너 + xArm7 + 턴테이블 통합.
# IScanningProcedure 기반 lookaround + 후처리 알고리즘 파이프라인.
#
# Notation: T_AB : A → B  (x_B = T_AB @ x_A) — README.md / CLAUDE.md 준수.

from __future__ import annotations

import traceback
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

    def _sim_T_EC_gt(self) -> Optional[np.ndarray]:
        """sim(USD) 에서 실측한 E→C. isaac 백엔드 전용.

        정의는 isaac_scan_session._T_EC_gt 와 동일: inv(T_WC) @ T_WE.
        prim_world_pose 는 scale 이 제거된 회전을 준다(스캐너 mm 노드 대응).
        """
        try:
            from mms_artec.backends.isaac.isaac_world import (
                CAMERA_PRIM, EE_LINK_PATH)
            w = getattr(self.sensor, "_world", None)
            if w is None:
                return None
            pc, Rc = w.prim_world_pose(CAMERA_PRIM)
            pe, Re = w.prim_world_pose(EE_LINK_PATH)
            T_WC = np.eye(4); T_WC[:3, :3] = Rc; T_WC[:3, 3] = pc
            T_WE = np.eye(4); T_WE[:3, :3] = Re; T_WE[:3, 3] = pe
            return np.linalg.inv(T_WC) @ T_WE
        except Exception as e:                       # noqa: BLE001
            print(f"[MMS][WARN] sim T_EC GT 취득 실패({type(e).__name__}) — "
                  f"config 값 사용")
            return None

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

        # ★ isaac 백엔드는 **USD ground truth** 로 덮어쓴다.
        #   sim 에는 캘리브가 필요 없고 USD 가 진실이다. config/sensor_frames.yaml 은
        #   실물 캘리브(구 장착) 값이라, 툴체인저가 들어간 v3_scene 과 118mm 어긋난다.
        #   그대로 두면 캡처 점군이 엉뚱한 위치로 변환돼 lookaround preview 가 0 이 된다.
        if cfg.backend == "isaac":
            gt = self._sim_T_EC_gt()
            if gt is not None:
                if self._T_EC is not None:
                    d = float(np.linalg.norm(gt[:3, 3] - self._T_EC[:3, 3])) * 1000.0
                    print(f"[MMS] (sim) T_EC ← USD ground truth "
                          f"(config '{cfg.T_EC_key}' 대비 {d:.1f}mm 차이)")
                else:
                    print("[MMS] (sim) T_EC ← USD ground truth")
                self._T_EC = gt
            # 캘리브 오차 주입(기본 0=무주입). 실물의 hand-eye 오차 조건을 sim 에서 재현.
            from mms_artec.utils.calibration import handeye_error as _he
            self._T_EC = _he.believed_T_EC(self._T_EC, log=False)

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

    # ── turntable 표면 프레임 (T_B_F0) ────────────────────────────────
    # 축 추정은 rim 원 피팅으로 통일했다(utils/calibration/turntable_frame.py).
    # 구 fixture 3구 방식은 실물 제작 비용 문제로 폐기 — 2026-09 코드 제거.

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
        axis_point, axis_dir : rim 원 피팅 결과(base 프레임).
                           utils/calibration/turntable_frame.py::fit_circle_3d

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
        """θ 최적화 + 실행. nbv 가 사용."""
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

    # ── artec_process 의 구성요소 ────────────────────────────────────────
    def _run_scan(self, robot, turntable, s):
        """스캔 단계만 담당. (model, ctx, hints_applied) 반환.

        백엔드/설정에 따라 4갈래다. 이걸 artec_process 본문에 두면 '스캔 → 후처리 →
        export' 라는 큰 흐름이 분기 속에 묻힌다.
        """
        ctx = None
        if self.cfg.backend == "isaac":
            # Isaac: Artec SLAM(IScanningProcedure) 대신 sim 스캔 경로.
            # 턴테이블 GT θ + 카메라 포즈로 점군을 직접 누적한다.
            from mms_artec.backends.isaac.isaac_scan_session import IsaacScanSession
            r = IsaacScanSession(self, robot, turntable, s).run()
            print(f"\n[artec_process] (sim) Scan — {r.n_frames} frames")
            return r.model, ctx, False

        # ★ 캡처는 **streaming(IScanningProcedure) 하나**다. 옛 discrete 경로
        #   (`artec_scan_session.ArtecScanSession`, 자세마다 개별 capture())는
        #   2026-09-16 삭제했다 — 오래 안 쓰였고, 세 갈래 분기가 "어느 코드가 도는지"
        #   를 헷갈리게 만들었다. 남은 선택은 **multipass 여부** 하나다.
        if s.use_multipass_scan:
            # lookaround 재진행(tracking lost recovery) + stage_until 분기.
            # 모든 IScan 이 master IModel 에 누적되고 아래 GlobalReg 에서 정합된다.
            from mms_artec.nbv.artec_multipass_scan_session import ArtecMultiPassScanSession
            r = ArtecMultiPassScanSession(self, robot, turntable, s.multipass_settings).run()
            print(f"\n[artec_process] Multi-pass Scan — {r.n_passes} passes, "
                  f"{r.n_total_frames} total frames ({r.model.scan_count()} scan(s))")
            # ★ 스캔 결과를 `ctx` 로 **넘겨준다.** 예전엔 `ctx=None` 이라 세션이
            #   들고 있던 것(밴드 플랜·preview 점군·recovery 회계)이 여기서 전부
            #   사라졌다 — `--until preview` 는 모델이 비어 있으므로 그게 유일한
            #   산출인데 볼 방법이 없었다 (2026-09-21).
            return r.model, r, bool(r.hints_applied)

        from mms_artec.nbv.artec_streaming_scan_session import ArtecStreamingScanSession
        r = ArtecStreamingScanSession(self, robot, turntable, s.streaming_scan_settings).run()
        print(f"\n[artec_process] Streaming Scan — {r.n_frames} frames "
              f"({r.fps_actual:.1f} fps)")
        return r.model, ctx, False

    def _stage(self, name: str, fn, current_model):
        """후처리 한 단계. 실패해도 **이전 모델을 유지**하고 계속한다.

        ★ 예전엔 RuntimeError 만 잡았다. SDK 바인딩이 다른 예외를 던지면 파이프라인이
          죽고 **이미 끝난 스캔이 통째로 버려진다**(실물 스캔은 수 분).
          KeyboardInterrupt/SystemExit 은 Exception 하위가 아니라 그대로 전파된다.
        """
        print(f"[artec_process] {name} ...")
        try:
            out = fn(current_model)
            try:
                n_scans = out.scan_count()
                n_frames = sum(out.get_scan(i).frame_count() for i in range(n_scans))
                print(f"   ok  scans={n_scans}  frames={n_frames}  "
                      f"composite={out.has_final_mesh()}")
            except Exception:
                pass
            return out
        except Exception as e:                       # noqa: BLE001
            print(f"   ⚠ {name} 실패({type(e).__name__}): {e}")
            traceback.print_exc()
            print("   → skip (이전 모델 유지)")
            return current_model

    def _save_intermediate(self, tag: str, model, s) -> None:
        """중간 sproj 저장(디버깅용). 실패는 무시."""
        if not s.export_sproj_path:
            return
        try:
            p = Path(s.export_sproj_path)
            p.parent.mkdir(parents=True, exist_ok=True)
            mid = str(p.with_stem(p.stem + f"_{tag}"))
            Path(mid).unlink(missing_ok=True)
            self.sensor.save_project(model, mid)
            print(f"   mid-save → {mid}")
        except Exception as e:                       # noqa: BLE001
            print(f"   mid-save 실패 (무시): {e}")

    def _export(self, model, s, tag: str = "") -> None:
        """OBJ/sproj 내보내기. **sim·real 공용** — 예전엔 isaac 경로가 별도 블록을
        갖고 있었고 이미 갈라져 있었다(sproj 저장이 sim 에만 없었다).

        저장 실패가 파이프라인 결과를 버리게 두지 않는다 — mkdir/unlink 도 try 안에.
        """
        pre = f"(sim) " if tag else ""
        if s.export_obj_path:
            try:
                Path(s.export_obj_path).parent.mkdir(parents=True, exist_ok=True)
                model.save_obj(s.export_obj_path)
                print(f"[artec_process] {pre}saved OBJ → {s.export_obj_path}")
            except Exception as e:                   # noqa: BLE001
                print(f"[artec_process] {pre}OBJ 저장 실패({type(e).__name__}): {e}")
        # sproj 는 Artec SDK 프로젝트 형식 — sim sensor(IsaacArtecScanner)엔 없다.
        # 능력 확인으로 건너뛴다(설정이 real 것을 쓰고 있어도 조용히 통과).
        if s.export_sproj_path and hasattr(self.sensor, "save_project"):
            try:
                Path(s.export_sproj_path).parent.mkdir(parents=True, exist_ok=True)
                Path(s.export_sproj_path).unlink(missing_ok=True)
                self.sensor.save_project(model, s.export_sproj_path)
                print(f"[artec_process] {pre}saved sproj → {s.export_sproj_path}")
            except Exception as e:                   # noqa: BLE001
                print(f"[artec_process] {pre}sproj 저장 실패({type(e).__name__}): {e}")

    def artec_process(
        self,
        robot,
        turntable,
        settings: Optional["ArtecProcessSettings"] = None,
    ) -> "ArtecProcessResult":
        """
        Artec 풀 파이프라인. `docs/6_artec_process.md` 부록 B SDK General Pipeline 순서:
          Scan → SerialReg → GlobalReg → Cleaning → Fusion → Simplify → Texturize → Export

        구성요소는 `_run_scan` / `_stage` / `_save_intermediate` / `_export` 로 분리돼
        있어, 이 함수는 **흐름만** 보여준다.
        """
        s = settings or ArtecProcessSettings()
        # ★ settings 는 **읽기 전용 입력**으로 다룬다. 예전에는 hints 분기가
        #   `s.do_global_registration = False` 로 직접 껐는데, `s` 는 호출자 객체 그
        #   자체(복사본 아님)라 main_artec.py 의 PROCESS_SETTINGS 가 **영구히 변형**됐다
        #   → 두 번째 호출부터 GlobalRegistration 이 조용히 꺼진 채 돈다.
        do_global_reg = bool(s.do_global_registration)

        # ── 0. Scanning ─────────────────────────────────────────────────
        model, ctx, hints_applied = self._run_scan(robot, turntable, s)

        # hint 가 적용됐다면 GlobalReg 가 hint 를 흐트러뜨릴 수 있다(hint 가 authoritative).
        if hints_applied and do_global_reg:
            print("[artec_process] ⓘ hints_applied=True → post-merge "
                  "GlobalRegistration 자동 skip (hint 가 authoritative)")
            do_global_reg = False

        # ── isaac: Artec SDK 후처리 없음 ────────────────────────────────
        # IsaacScanSession 이 이미 점군/mesh 를 만들었으므로 export 만 한다.
        if self.cfg.backend == "isaac":
            self._export(model, s, tag="sim")
            return ArtecProcessResult(model=model, ctx=ctx)

        # ── 스캔이 없으면 후처리를 돌리지 않는다 ────────────────────────
        # `--until preview` 는 계획만 내고 캡처를 안 한다. 그 빈 모델로 SDK
        # 알고리즘을 부르면 단계마다 ArgumentInvalid(0x80010201) 가 나고
        # **트레이스백이 세 번** 찍혀, 정상 종료인데 실패처럼 보인다
        # (2026-09-21 실물). 돌릴 것이 없으면 조용히 건너뛴다.
        try:
            _n_scan = int(model.scan_count())
        except Exception:                                    # noqa: BLE001
            _n_scan = 0
        if _n_scan == 0:
            print("[artec_process] 스캔 0개 — 후처리·export 건너뜀")
            return ArtecProcessResult(model=model, ctx=ctx)

        # ── 1-2. Registration ───────────────────────────────────────────
        if s.do_serial_registration:
            model = self._stage("SerialRegistration", self.sensor.serial_registration, model)
        if do_global_reg:
            model = self._stage("GlobalRegistration", self.sensor.global_registration, model)
        self._save_intermediate("post_reg", model, s)

        # ── 3. Cleaning (Fusion 전) ─────────────────────────────────────
        if s.do_outliers_removal:
            model = self._stage("OutliersRemoval", self.sensor.outliers_removal, model)
        if s.do_small_objects_filter:
            model = self._stage("SmallObjectsFilter", self.sensor.small_objects_filter, model)
        self._save_intermediate("pre_fusion", model, s)

        # ── 4. Fusion ───────────────────────────────────────────────────
        # 값은 ArtecProcessSettings.__post_init__ 에서 FUSION_CHOICES 로 검증·정규화됨.
        if s.fusion == "poisson":
            model = self._stage("PoissonFusion", self.sensor.poisson_fusion, model)
        elif s.fusion == "fast":
            model = self._stage("FastFusion", self.sensor.fast_fusion, model)

        # ── 5-6. Simplify + Texturize ───────────────────────────────────
        if s.do_simplify:
            model = self._stage("MeshSimplify", self.sensor.mesh_simplify, model)
        if s.do_texturize:
            model = self._stage("Texturize", self.sensor.texturize, model)

        # ── Export ──────────────────────────────────────────────────────
        self._export(model, s)
        return ArtecProcessResult(model=model, ctx=ctx)

# ─────────────────────────────────────────────────────────────────────────────
# ArtecProcessSettings / Result
# ─────────────────────────────────────────────────────────────────────────────

# artec_process 의 Fusion 단계 dispatch 와 **같은 목록**을 쓴다(둘이 갈라지면
# 검증을 통과한 값이 dispatch 에서 무시되는 상황이 생긴다).
FUSION_CHOICES = frozenset({"poisson", "fast", "none"})


@dataclass
class ArtecProcessSettings:
    """`ArtecMMS.artec_process()` 통합 설정."""

    # ── 개발/Production 모드 ──────────────────────────────────────────
    # dev_mode=True 면 무거운 후처리 (OutliersRemoval, Simplify) 를 자동 skip.
    # 빠른 iteration 용. 본 export 에선 False 로 되돌릴 것.
    # __post_init__ 에서 do_* 플래그를 강제 override 함 (dev_mode 가 우선).
    dev_mode: bool = False

    # Multi-pass: tracking-lost 재진행 + nbv(NBV) / flip(flip 바닥면). True 가
    # 새 default — 한 번에 끝내고 싶으면 multipass_settings.prompt_*_=False 로
    # 끄거나 use_multipass_scan=False 로 단일 streaming session 사용.
    #
    # ★ 옛 `use_streaming_scan` 은 2026-09-16 제거했다. discrete 경로가 사라져
    #   streaming 이 유일한 캡처 방식이 됐으므로 켜고 끌 것이 없다.
    use_multipass_scan: bool = True

    streaming_scan_settings: Optional["ArtecStreamingScanSessionSettings"] = None    # type: ignore[name-defined]
    multipass_settings: Optional["ArtecMultiPassScanSessionSettings"] = None         # type: ignore[name-defined]

    do_serial_registration: bool = True
    do_global_registration: bool = True

    # 허용값: "poisson" | "fast" | "none"(또는 None/"").
    # __post_init__ 에서 검증한다 — 오타("poison" 등)면 예전엔 아래 dispatch 가
    # 조용히 아무것도 안 해서 **융합이 빠진 줄 모르고** 결과를 받았다.
    fusion: str = "poisson"

    do_outliers_removal: bool = True
    do_small_objects_filter: bool = True
    do_simplify: bool = True
    do_texturize: bool = True

    export_obj_path:   Optional[str] = None
    export_sproj_path: Optional[str] = None

    def __post_init__(self):
        # ⚠ 아래 scan-settings 클래스들은 Artec SDK(artec_base)에 의존 → isaac 에선 import 실패.
        # isaac 스캔 경로(IsaacScanSession)는 이 세 settings 를 쓰지 않으므로, 실패 시 None 유지.
        try:
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
        except Exception as _e:               # isaac: Artec SDK 없음
            print(f"[ArtecProcessSettings] ⓘ scan-settings import skip "
                  f"({type(_e).__name__}) — isaac 스캔 경로(IsaacScanSession)에선 불필요")

        # Dev mode 강제 override — 무거운 단계 skip.
        if self.dev_mode:
            self.do_outliers_removal = False    # frame 별 neighborhood, 5분+
            self.do_simplify = False            # mesh simplification
            # do_small_objects_filter / do_texturize 는 유지 — 비교적 빠르고
            # 결과 검증에 도움. PoissonFusion 도 유지 (mesh 결과 자체).
            print("[ArtecProcessSettings] ⚡ dev_mode ON — "
                  "do_outliers_removal=False, do_simplify=False")

        # ── fusion 값 검증 (fail-fast) ───────────────────────────────────
        # 스캔을 수 분 돌린 뒤 조용히 융합만 빠지는 것보다, **설정을 만드는 시점에**
        # 즉시 틀렸다고 알리는 편이 낫다. 이 검증은 main 이 뜨자마자 실행된다.
        _f = (self.fusion or "none")
        if not isinstance(_f, str) or _f.lower() not in FUSION_CHOICES:
            raise ValueError(
                f"ArtecProcessSettings.fusion={self.fusion!r} 은 알 수 없는 값입니다. "
                f"허용: {sorted(FUSION_CHOICES)} (대소문자 무관, None='none')")
        self.fusion = _f.lower()


@dataclass
class ArtecProcessResult:
    """artec_process 결과."""
    model: object                                        # ModelHandle (artec_base)
    #: 스캔 세션 결과 (`ArtecMultiPassScanResult`). 모델에 안 담기는 것들이
    #  여기 있다 — preview 점군·밴드 플랜·recovery 회계·hint 기록.
    #  ("discrete only" 였던 옛 주석은 그 경로를 2026-09-16 지우며 무효가 됐다.)
    ctx:   Optional[object] = None

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
