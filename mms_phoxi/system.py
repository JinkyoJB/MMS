# mms_phoxi/system.py

from __future__ import annotations

import time
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Optional, Union

import numpy as np

from mms_phoxi.core.frames import Frame
from mms_phoxi.core.stream import Stream
from utils.scan_result import ScanResult
from utils.transforms import (
    TurntableTransformConfig,
    load_transform,
    compute_T_CB,
    compute_T_CO,
    solve_T_EB,
    transform_points,
    transform_normals,
)
from mms_phoxi.sensor import PhoxiClient, PhoxiConfig
from utils.nbv.manual_picker import pick_camera_target_in_frame
from utils.control.hardware_layer import execute_camera_target, compute_ik_reachability
from utils.control.theta_planner import plan_min_motion_theta, DEFAULT_JOINT_WEIGHTS


@dataclass
class MMSConfig:
    """
    Top-level MMS configuration.

    센서는 orbbec / phoxi / artec 중 하나만 설정한다.

    Attributes
    ----------
    orbbec : OrbbecConfig, optional
        Orbbec Femto Bolt 센서 설정.
    phoxi : PhoxiConfig, optional
        Photoneo PhoXi 3D 센서 설정.
    artec : ArtecConfig, optional
        Artec 3D 스캐너 설정.
    sensor_frames_yaml : str, optional
        Path to config/sensor_frames.yaml.
        Required to load T_EC for camera→base frame conversion.
    T_EC_key : str, optional
        Key in sensor_frames_yaml for the T_EC transform of the active sensor.
        e.g. "T_EC_femto", "T_EC_phoxi", "T_EC_artec".
        Required when sensor_frames_yaml is provided.
    turntable_frame_yaml : str, optional
        Path to turntable frame yaml (key: T_B_F0).
        If provided, MMS.turntable_transform is populated on init.
    object_frame_yaml : str, optional
        Path to object frame yaml (key: T_O_F0).
        Defines T_OF (O→F) for the O–C chain.
        If None, T_OF defaults to identity (O ≡ F at θ=0).
    stream_max_size : int, default=500
        Maximum number of frames retained in MMS.stream (Stream).
        Oldest frames are evicted automatically when the buffer is full.
    """
    phoxi: PhoxiConfig = None  # required
    sensor_frames_yaml: Optional[str] = None
    T_EC_key: Optional[str] = None
    turntable_frame_yaml: Optional[str] = None
    object_frame_yaml: Optional[str] = None
    stream_max_size: int = 500

    def __post_init__(self):
        if self.phoxi is None:
            raise ValueError("MMSConfig: phoxi 설정 필수.")

        if self.sensor_frames_yaml is not None:
            self.sensor_frames_yaml = str(Path(self.sensor_frames_yaml).resolve())
            if self.T_EC_key is None:
                raise ValueError(
                    "MMSConfig: sensor_frames_yaml을 지정하면 T_EC_key도 함께 지정해야 합니다."
                )
        if self.turntable_frame_yaml is not None:
            self.turntable_frame_yaml = str(Path(self.turntable_frame_yaml).resolve())
        if self.object_frame_yaml is not None:
            self.object_frame_yaml = str(Path(self.object_frame_yaml).resolve())


class MMS:
    """
    Top-level MMS (Multi Modal Scanning System) orchestrator.

    Manages sensor lifecycle, frame capture, and the sensor-to-base
    coordinate transform pipeline.

    Notation
    --------
    T_AB maps frame A to frame B:  x_B = T_AB @ x_A
    Frames: B=Base, F=Turntable, O=Internal global, E=End-effector, C=Camera

    Sensor pipeline
    ---------------
    1. sensor.capture() → ScanResult   (C frame, mm)
    2. _scan_to_frame()                (C → B transform, mm → m)
        T_CB = T_EB @ inv(T_EC)
        pts_B = T_CB @ (pts_C / 1000)
    3. Frame                           (B frame, m)

    Attributes
    ----------
    turntable_transform : TurntableTransformConfig or None
        Base <-> Turntable frame transforms. Populated when
        MMSConfig.turntable_frame_yaml is provided.

    Frame transform convenience methods
    ------------------------------------
    T_FB(theta)               F → B at turntable angle theta (rad)
    T_BF(theta)               B → F at turntable angle theta (rad)
    T_CB(T_EB)                C → B  (uses self._T_EC)
    T_CO(theta, T_EB)         C → O  camera pose in object frame (NBV interface)
    solve_T_EB(theta, T_CO)   E → B  robot target from desired camera pose

    Example
    -------
    >>> with MMS(cfg) as mms:
    ...     batch = mms.capture_frames(10)
    ...     mms.preprocess(batch, roi_bbox=(-0.5, 0.5, -0.5, 0.5, 0.1, 1.5))
    ...     x_B = mms.T_FB(theta) @ x_F
    ...     x_F = mms.T_BF(theta) @ x_B
    ...     T = mms.T_CB(robot.get_ee_pose_mat())
    """

    def __init__(self, cfg: MMSConfig) -> None:
        self.cfg = cfg

        # Sensor
        self.sensor: PhoxiClient = PhoxiClient(cfg.phoxi)

        # T_EC: E → C transform (hand-eye calibration result)
        if cfg.sensor_frames_yaml is not None:
            self._T_EC: Optional[np.ndarray] = load_transform(
                cfg.sensor_frames_yaml, cfg.T_EC_key
            )
        else:
            self._T_EC = None

        self.stream: Stream = Stream(max_size=cfg.stream_max_size)

        # Turntable transform (B <-> F)
        if cfg.turntable_frame_yaml is not None:
            T_BF0 = load_transform(cfg.turntable_frame_yaml, "T_B_F0")
            self.turntable_transform: Optional[TurntableTransformConfig] = TurntableTransformConfig(T_BF0)
        else:
            self.turntable_transform = None

        # backwards-compatible alias
        self.world_transform = self.turntable_transform

        # O–F transform: O → F at θ=0 (Internal Global Frame)
        # Defaults to identity (O ≡ F at θ=0) until set_object_frame() is called.
        if cfg.object_frame_yaml is not None:
            self._T_OF: np.ndarray = load_transform(cfg.object_frame_yaml, "T_O_F0")
        else:
            self._T_OF = np.eye(4, dtype=float)

    # ── lifecycle ──────────────────────────────────────────────────────────

    def initialize(self) -> None:
        """Initialize all hardware clients."""
        self.sensor.initialize()

    def shutdown(self) -> None:
        """Shutdown all hardware clients and release resources."""
        self.sensor.shutdown()

    def __enter__(self) -> MMS:
        self.initialize()
        return self

    def __exit__(self, *_) -> None:
        self.shutdown()

    # ── sensor → frame conversion ──────────────────────────────────────────

    def _scan_to_frame(
        self,
        scan: ScanResult,
        ee_pose_mat_B: np.ndarray,
    ) -> Frame:
        """
        Convert a ScanResult (sensor frame, mm) to a Frame (base frame, m).

        Transform chain:
            T_CB = T_EB @ inv(T_EC)
            pts_B = T_CB @ (pts_C_mm / 1000)

        If T_EC is not configured (sensor_frames_yaml not set),
        points are converted mm→m but remain in the camera frame (no rotation).
        A warning is printed in this case.

        Parameters
        ----------
        scan : ScanResult
            Raw capture from sensor.capture() — C frame, mm.
        ee_pose_mat_B : (4,4) np.ndarray
            T_EB — E → B transform at capture time (robot FK).
            Pass np.eye(4) when the robot is not connected.

        Returns
        -------
        Frame
            Points in base (B) frame, meters.
        """
        if self._T_EC is not None:
            T_CB = compute_T_CB(ee_pose_mat_B, self._T_EC)
        else:
            print("[MMS] ⚠ T_EC 미설정 (sensor_frames_yaml 없음). "
                  "포인트가 카메라(C) 프레임 기준으로 유지됩니다.")
            T_CB = ee_pose_mat_B  # identity T_EC assumed

        pts_m = (scan.points / 1000.0).astype(np.float64)
        pts_B = transform_points(T_CB, pts_m).astype(np.float32)

        normals_B: Optional[np.ndarray] = None
        if scan.normals is not None:
            normals_B = transform_normals(
                T_CB, scan.normals.astype(np.float64)
            ).astype(np.float32)

        depth_m = scan.depth / 1000.0 if scan.depth is not None else None

        return Frame(
            sensor_type=scan.sensor_type,
            img=scan.img,
            depth=depth_m,
            points=pts_B,
            normals=normals_B,
            colors=scan.colors,
            frame_id=scan.frame_id,
            timestamp=scan.timestamp,
            ee_pose_mat_B=ee_pose_mat_B.astype(np.float64),
        )

    # ── capture ────────────────────────────────────────────────────────────

    def capture_frames(
        self,
        n: int,
        ee_pose_fn: Optional[Callable[[], np.ndarray]] = None,
    ) -> list[Frame]:
        """
        Capture n frames from the sensor and append them to self.stream.

        Each ScanResult (S frame, mm) is converted to a Frame (B frame, m)
        via _scan_to_frame() using T_E_S loaded from sensor_frames_yaml.

        Frame pacing is controlled by sensor config target_interval_s:
        after each capture, sleeps max(0, target - elapsed) so that
        successive frames are spaced target_interval_s apart.

        Parameters
        ----------
        n : int
            Number of frames to capture.
        ee_pose_fn : () -> (4,4) np.ndarray, optional
            Callable that returns the current T_E_B from the robot.
            If None, identity matrix is used (robot not connected).

        Returns
        -------
        batch : list[Frame]
            Newly captured frames (also appended to self.stream Stream).
        """
        target = self.sensor.cfg.target_interval_s
        batch: list[Frame] = []

        print(f"[MMS] capture_frames: {n} 프레임 수집  "
              f"(target_interval={target:.3f}s)")
        for i in range(n):
            ee_pose_mat_B = (
                ee_pose_fn() if ee_pose_fn is not None
                else np.eye(4, dtype=np.float64)
            )

            t_start = time.perf_counter()
            frame_id = len(self.stream) + len(batch)

            scan: Optional[ScanResult] = self.sensor.capture(
                frame_id=frame_id,
                timestamp=time.time(),
            )

            elapsed = time.perf_counter() - t_start

            if scan is None:
                print(f"  [{i:02d}] 캡처 실패 (None) — 스킵  elapsed={elapsed:.3f}s")
            else:
                frame = self._scan_to_frame(scan, ee_pose_mat_B)
                print(f"  [{i:02d}] frame_id={frame.frame_id}  "
                      f"pts={len(frame.points):>8,}  "
                      f"img={'있음' if frame.img is not None else '없음'}  "
                      f"elapsed={elapsed:.3f}s")
                batch.append(frame)

            if i < n - 1:
                remaining = target - elapsed
                if remaining > 0:
                    time.sleep(remaining)

        self.stream.extend(batch)
        print(f"[MMS] 캡처 완료: {len(batch)}/{n}  "
              f"(누적 {len(self.stream)} 프레임)")
        return batch

    # MMS.preprocess(...) 는 제거됨.
    # lookaround (scan_session) 은 센서 원본(_last_organized_pts)을 직접 사용하며,
    # 모든 cleanup 은 PcdAccumulateVolume 내부에서 옵션으로 처리한다.

    # ── buffer management ──────────────────────────────────────────────────

    def clear_stream(self) -> None:
        """Clear the internal frame stream buffer."""
        self.stream.clear()
        print("[MMS] stream buffer cleared.")

    # ── frame transforms ───────────────────────────────────────────────────

    def T_FB(self, theta: float) -> np.ndarray:
        """
        F → B transform at turntable angle theta (rad).

        x_B = mms.T_FB(theta) @ x_F

        Requires MMSConfig.turntable_frame_yaml to be set.
        """
        if self.turntable_transform is None:
            raise RuntimeError(
                "turntable_transform not configured. Provide turntable_frame_yaml in MMSConfig."
            )
        return self.turntable_transform.T_FB(theta)

    def T_BF(self, theta: float) -> np.ndarray:
        """
        B → F transform at turntable angle theta (rad).

        x_F = mms.T_BF(theta) @ x_B

        Requires MMSConfig.turntable_frame_yaml to be set.
        """
        if self.turntable_transform is None:
            raise RuntimeError(
                "turntable_transform not configured. Provide turntable_frame_yaml in MMSConfig."
            )
        return self.turntable_transform.T_BF(theta)

    # backwards-compatible aliases
    def T_O_B(self, theta: float) -> np.ndarray:
        return self.T_FB(theta)

    def T_B_O(self, theta: float) -> np.ndarray:
        return self.T_BF(theta)

    def T_CB(self, T_EB: np.ndarray) -> np.ndarray:
        """
        C → B transform.

        T_CB = T_EB @ inv(T_EC)
        x_B  = mms.T_CB(T_EB) @ x_C

        Uses self._T_EC loaded from MMSConfig.sensor_frames_yaml.

        Parameters
        ----------
        T_EB : (4,4) np.ndarray
            E → B transform at capture time (from robot FK).
        """
        if self._T_EC is None:
            raise RuntimeError(
                "T_EC not configured. Provide sensor_frames_yaml and T_EC_key in MMSConfig."
            )
        return compute_T_CB(T_EB, self._T_EC)

    # backwards-compatible alias
    def T_S_B(self, T_EB: np.ndarray) -> np.ndarray:
        return self.T_CB(T_EB)

    def T_CO(self, theta: float, T_EB: np.ndarray) -> np.ndarray:
        """
        Camera pose in object frame: T_CO (C → O).

        x_O = T_CO @ x_C
        T_CO[:3, 3] = camera origin in O frame — primary NBV input.

        Chain: C → E → B → F → O
        T_CO = inv(T_OF) @ inv(T_FB(θ)) @ T_EB @ inv(T_EC)

        Requires turntable_frame_yaml and sensor_frames_yaml (T_EC) in MMSConfig.
        Uses self._T_OF (set via set_object_frame() or MMSConfig.object_frame_yaml;
        defaults to identity so O ≡ F at θ=0).

        Parameters
        ----------
        theta : float
            Current turntable angle (rad).
        T_EB : (4,4) np.ndarray
            E → B transform from robot FK.
        """
        if self.turntable_transform is None:
            raise RuntimeError(
                "turntable_transform not configured. Provide turntable_frame_yaml in MMSConfig."
            )
        if self._T_EC is None:
            raise RuntimeError(
                "T_EC not configured. Provide sensor_frames_yaml and T_EC_key in MMSConfig."
            )
        return compute_T_CO(theta, T_EB, self._T_OF, self._T_EC, self.turntable_transform)

    def solve_T_EB(self, theta: float, T_CO_des: np.ndarray) -> np.ndarray:
        """
        Solve for robot target T_EB given a desired camera pose T_CO_des.

        Hardware-layer inverse of T_CO():
        T_EB = T_FB(θ) @ T_OF @ T_CO_des @ T_EC

        Parameters
        ----------
        theta : float
            Turntable angle (rad) at which to evaluate the solution.
        T_CO_des : (4,4) np.ndarray
            Desired camera pose in O frame (C → O), as produced by the NBV layer.

        Returns
        -------
        T_EB : (4,4) np.ndarray  —  E → B  (robot FK target pose)
        """
        if self.turntable_transform is None:
            raise RuntimeError(
                "turntable_transform not configured. Provide turntable_frame_yaml in MMSConfig."
            )
        if self._T_EC is None:
            raise RuntimeError(
                "T_EC not configured. Provide sensor_frames_yaml and T_EC_key in MMSConfig."
            )
        return solve_T_EB(theta, T_CO_des, self._T_OF, self._T_EC, self.turntable_transform)

    # ── 제어 레이어 (docs/1_control_layers.md) ─────────────────────────────

    def nbv_pick_target(
        self,
        frames: Optional[Union[list[Frame], Stream]] = None,
        theta_current: float = 0.0,
        distance_m: float = 0.384,
        knn: int = 30,
        work_in_O: bool = True,
        preview: bool = True,
        default_roll_deg: float = 0.0,
        check_ik: bool = False,
        robot=None,
        theta_target: Optional[float] = None,
        ik_voxel_m: float = 0.025,
        ik_roll_candidates: Optional[tuple] = None,
        ik_keep_only_reachable: bool = False,
    ) -> np.ndarray:
        """
        상위(NBV) 레이어 — 사용자가 Open3D 창에서 표면을 pick 해 카메라 목표 포즈를 만든다.

        상위 레이어는 O–C 관계만 다룬다 (docs/control_layers.md §3.1).
        여기서는 NBV 알고리즘 대신 수동 picker 를 사용한다.

        Frames
        ------
        - self.stream / `frames` 의 `Frame.points` 는 Base(B) 기준 (self._scan_to_frame 결과).
        - work_in_O=True (권장) 이면 points 를 B → O 로 변환 후 picker 실행 →
          반환값은 `T_CO_des` (C → O).
        - work_in_O=False 이면 B 프레임에서 그대로 picker 실행 →
          반환값은 `T_CB_des` (C → B). 디버깅용.

        Parameters
        ----------
        frames : list[Frame] or Stream, optional
            None 이면 self.stream.
        theta_current : float, default=0.0
            현재 턴테이블 각도 (rad). B → O 변환에 사용.
            work_in_O=True 일 때만 의미 있음.
        distance_m : float, default=0.384
            표면 → 카메라 거리 (PhoXi 최적 거리).
        knn : int, default=30
            선택 점 주변 k-NN 으로 노말 refine.
        work_in_O : bool, default=True
            True  → points 를 O 로 변환 후 picker 실행 (반환: T_CO_des)
            False → B 기준으로 picker 실행         (반환: T_CB_des)
        preview : bool, default=True
            picker 후 결과 확인용 창 표시.

        Returns
        -------
        T_CX_des : (4,4) np.ndarray
            work_in_O=True  → T_CO_des  (NBV ↔ 하드웨어 인터페이스)
            work_in_O=False → T_CB_des
        """
        targets = list(frames) if frames is not None else list(self.stream)
        if not targets:
            raise RuntimeError("프레임이 비어 있습니다. 먼저 capture_frames() 를 호출하세요.")

        all_pts_B = np.concatenate([f.points for f in targets], axis=0).astype(np.float64)
        all_nrm_B: Optional[np.ndarray] = None
        if all(f.normals is not None for f in targets):
            all_nrm_B = np.concatenate(
                [f.normals for f in targets], axis=0
            ).astype(np.float64)
        all_cols: Optional[np.ndarray] = None
        if all(f.colors is not None for f in targets):
            all_cols = np.concatenate(
                [f.colors for f in targets], axis=0
            ).astype(np.float64)

        if work_in_O:
            if self.turntable_transform is None:
                raise RuntimeError(
                    "turntable_transform 미설정 — work_in_O=True 에는 "
                    "MMSConfig.turntable_frame_yaml 이 필요합니다. "
                    "(또는 work_in_O=False 로 B 프레임에서 진행)"
                )
            T_BO = np.linalg.inv(self._T_OF) @ self.T_BF(theta_current)
            pts_pick = transform_points(T_BO, all_pts_B)
            nrm_pick = (
                transform_normals(T_BO, all_nrm_B) if all_nrm_B is not None else None
            )
            frame_label = "O"
        else:
            pts_pick = all_pts_B
            nrm_pick = all_nrm_B
            frame_label = "B"

        world_up = np.array([0.0, 0.0, 1.0])
        cols_pick: Optional[np.ndarray] = all_cols
        knn_pick = int(knn)

        # ── IK reachability precompute (옵션) ─────────────────────────────
        if check_ik:
            if robot is None:
                raise ValueError("check_ik=True 에는 robot(XArmInterface) 인자가 필요합니다.")
            if self._T_EC is None:
                raise RuntimeError(
                    "T_EC 미설정 — check_ik=True 에는 hand-eye 캘리브레이션이 필요합니다."
                )
            if self.turntable_transform is None:
                raise RuntimeError(
                    "turntable_transform 미설정 — check_ik=True 에는 "
                    "MMSConfig.turntable_frame_yaml 이 필요합니다."
                )
            theta_ik = theta_current if theta_target is None else float(theta_target)
            # ik_roll_candidates 미지정이면 default_roll_deg 한 샘플만 검사 —
            # 색칠된 reachability 와 사용자가 실제로 쓸 roll 을 일치시켜 경계 케이스 방지
            if ik_roll_candidates is None:
                ik_roll_candidates = (float(np.radians(default_roll_deg)),)

            # Downsample (voxel) + normals 재추정 (빠른 IK 체크용)
            import open3d as o3d
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(np.ascontiguousarray(pts_pick))
            if nrm_pick is not None and len(nrm_pick) == len(pts_pick):
                pcd.normals = o3d.utility.Vector3dVector(np.ascontiguousarray(nrm_pick))
            ds = pcd.voxel_down_sample(float(ik_voxel_m))
            if not ds.has_normals():
                ds.estimate_normals(
                    search_param=o3d.geometry.KDTreeSearchParamHybrid(
                        radius=max(0.015, ik_voxel_m * 1.5), max_nn=30
                    )
                )
                ds.normalize_normals()
            ds_pts = np.asarray(ds.points, dtype=np.float64)
            ds_nrm = np.asarray(ds.normals, dtype=np.float64)

            mask = compute_ik_reachability(
                points=ds_pts,
                normals=ds_nrm,
                distance_m=distance_m,
                theta_target=theta_ik,
                T_OF=self._T_OF,
                T_EC=self._T_EC,
                tt=self.turntable_transform,
                robot=robot,
                roll_candidates=tuple(ik_roll_candidates),
                world_up=world_up,
                frame_is_O=work_in_O,
                verbose=True,
            )

            colors = np.empty((len(ds_pts), 3), dtype=np.float64)
            colors[mask] = [0.10, 0.85, 0.20]    # 초록: reachable
            colors[~mask] = [0.85, 0.20, 0.20]   # 빨강: unreachable

            if ik_keep_only_reachable:
                ds_pts = ds_pts[mask]
                ds_nrm = ds_nrm[mask]
                colors = colors[mask]

            pts_pick = ds_pts
            nrm_pick = ds_nrm
            cols_pick = colors
            # IK 체크가 "각 점의 바로 그 노말" 을 썼으므로 이웃 평균 내지 않음.
            # (knn>1 이면 picker 최종 노말이 색칠 당시 노말과 달라져
            #  초록인데 IK 실패 하는 경계 케이스가 생긴다)
            knn_pick = 1

        T_CX_des = pick_camera_target_in_frame(
            points=pts_pick,
            normals=nrm_pick,
            colors=cols_pick,
            distance_m=distance_m,
            knn=knn_pick,
            world_up=world_up,
            frame_label=frame_label,
            preview=preview,
            default_roll_deg=default_roll_deg,
        )

        return T_CX_des

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
        """
        하위(하드웨어) 레이어 — `T_CO_des` + θ 를 받아 턴테이블/로봇을 실제로 구동.

        체인: T_EB_des = T_FB(θ) · T_OF · T_CO_des · T_EC

        Parameters
        ----------
        T_CO_des : (4,4) np.ndarray   NBV 출력 (C → O)
        theta    : float               턴테이블 절대 목표 각도 (rad)
        robot    : XArmInterface, optional
        turntable: Turntable, optional
        robot_speed       : float, default=30 (deg/s)
        turntable_vel_rad_s: float, default=π/6 (30°/s)
        move_turntable    : bool, default=True
        move_robot        : bool, default=True
        confirm           : bool, default=True

        Returns
        -------
        dict — execute_camera_target() 결과 (키: T_EB_des, pose6d, ik_joints, theta, ...)
        """
        if self.turntable_transform is None:
            raise RuntimeError(
                "turntable_transform 미설정 — MMSConfig.turntable_frame_yaml 이 필요합니다."
            )
        if self._T_EC is None:
            raise RuntimeError(
                "T_EC 미설정 — MMSConfig.sensor_frames_yaml 과 T_EC_key 가 필요합니다."
            )

        return execute_camera_target(
            T_CO_des=T_CO_des,
            theta=theta,
            T_OF=self._T_OF,
            T_EC=self._T_EC,
            tt=self.turntable_transform,
            robot=robot,
            turntable=turntable,
            robot_speed=robot_speed,
            turntable_vel_rad_s=turntable_vel_rad_s,
            move_turntable=move_turntable,
            move_robot=move_robot,
            confirm=confirm,
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
        """
        하위 레이어 계층화 통합 — (1) θ 최적화 planner → (2) 실행.

        Step 1. `plan_min_motion_theta()` 로 T_CO_des 를 만족하는 θ* 탐색
                (cost = weighted joint-L2, 기본 w_tt=0).
        Step 2. 성공하면 `execute_target(..., theta=θ*)` 호출.

        Parameters
        ----------
        T_CO_des : (4,4)
        robot : XArmInterface                  (IK + 이동용 — 필수)
        turntable : Turntable, optional        None 이면 턴테이블 이동 생략
        theta_current : float, optional        None → turntable.getActualPos() (없으면 0)
        q_current : (7,), optional             None → robot.get_joint_angles()
        theta_range, n_samples                 θ 그리드
        joint_weights : (7,)                   None → DEFAULT_JOINT_WEIGHTS
        w_tt : float                           턴테이블 회전 비용 (rad 당)
        robot_speed, turntable_vel_rad_s       실행 속도
        move_turntable, move_robot, confirm    execute_target 옵션

        Returns
        -------
        dict
            "plan" : plan_min_motion_theta 결과
            "exec" : execute_target 결과  — plan 실패 시 None
        """
        if self.turntable_transform is None:
            raise RuntimeError(
                "turntable_transform 미설정 — MMSConfig.turntable_frame_yaml 이 필요합니다."
            )
        if self._T_EC is None:
            raise RuntimeError(
                "T_EC 미설정 — MMSConfig.sensor_frames_yaml 과 T_EC_key 가 필요합니다."
            )
        if robot is None:
            raise ValueError("plan_and_execute 에는 robot(XArmInterface) 가 필요합니다.")

        if theta_current is None:
            if turntable is not None and getattr(turntable, "is_connected", False):
                theta_current = float(turntable.getActualPos())
            else:
                theta_current = 0.0

        plan = plan_min_motion_theta(
            T_CO_des=T_CO_des,
            T_OF=self._T_OF,
            T_EC=self._T_EC,
            tt=self.turntable_transform,
            robot=robot,
            theta_current=float(theta_current),
            q_current=q_current,
            theta_range=theta_range,
            n_samples=n_samples,
            joint_weights=(
                DEFAULT_JOINT_WEIGHTS if joint_weights is None else joint_weights
            ),
            w_tt=float(w_tt),
            verbose=True,
        )

        if plan["theta"] is None:
            print("[plan_and_execute] 계획 실패 — feasible θ 없음. 이동 생략.")
            return {"plan": plan, "exec": None}

        exec_result = self.execute_target(
            T_CO_des=T_CO_des,
            theta=plan["theta"],
            robot=robot,
            turntable=turntable,
            robot_speed=robot_speed,
            turntable_vel_rad_s=turntable_vel_rad_s,
            move_turntable=move_turntable,
            move_robot=move_robot,
            confirm=confirm,
        )
        return {"plan": plan, "exec": exec_result}

    def run_scan_session(
        self,
        robot,
        turntable,
        settings=None,
    ):
        """
        docs/2_control_layers.md §8 — NBV 오케스트레이터 실행.

        호출 전 이미 수행돼야 하는 것:
        - `robot.go_home()` 및 `go_to_scan_start()` (scan_start 포즈)
        - `turntable.connect()` + `set_servo_on(True)`
        - 최소 1회 `capture_frames()` — intrinsic fit 에 필요

        Parameters
        ----------
        robot : XArmInterface
        turntable : Turntable
        settings : ScanSessionSettings, optional

        Returns
        -------
        ScanSession  — 완료 후의 session. `.state.mesh`, `.state.T_CO_list` 접근.
        """
        from mms_phoxi.nbv.scan_session import ScanSession, ScanSessionSettings
        if settings is None:
            settings = ScanSessionSettings()
        session = ScanSession(self, robot, turntable, settings=settings)
        session.run()
        return session

    def set_object_frame(self, T_OF: np.ndarray) -> None:
        """
        Set (or update) the O → F transform used by T_CO() and solve_T_EB().

        Call this once after the first scan to fix the object frame O.
        By default T_OF = identity (O ≡ F at θ = 0).

        Parameters
        ----------
        T_OF : (4,4) np.ndarray
            O → F transform.  x_F = T_OF @ x_O
        """
        assert T_OF.shape == (4, 4), "T_OF must be (4,4)"
        self._T_OF = T_OF.astype(float)

    # ── visualization / diagnostics ────────────────────────────────────────

    def visualize_with_object_frame(
        self,
        frames: Union[list[Frame], "Stream"],
        theta: float = 0.0,
        frame_size: float = 0.1,
        max_pts: int = 500_000,
        title: str = "Frame Transform Verification",
    ) -> None:
        """
        Captured point cloud에 Base(B)·Object(O) 좌표계 축을 겹쳐 시각화한다.

        프레임 변환이 올바른지 눈으로 확인하는 용도:
        - O 축 원점이 턴테이블 중심에 위치하는지
        - O 축 z축이 위쪽을 가리키는지
        - 포인트 클라우드가 올바른 위치에 있는지

        좌표계 표시 색상 (Open3D 기본)
        --------------------------------
        빨간축(X)  초록축(Y)  파란축(Z)

        Parameters
        ----------
        frames : list[Frame] or Stream
            시각화할 프레임. points는 Base(B) 프레임 기준이어야 한다.
        theta : float, default=0.0
            현재 턴테이블 각도 (rad). T_O_B(theta)로 Object 축 위치를 결정한다.
        frame_size : float, default=0.1
            좌표계 축 표시 길이 (m).
        max_pts : int, default=500_000
            렌더링할 최대 포인트 수. 초과 시 랜덤 서브샘플링.
        title : str
            Open3D 창 제목.
        """
        import open3d as o3d
        from scipy.spatial.transform import Rotation as R

        frames_list = list(frames)
        geoms = []

        # ── 로봇 미연결 경고 ──────────────────────────────────────────────
        _identity = np.eye(4)
        no_robot = all(
            np.allclose(f.ee_pose_mat_B, _identity) for f in frames_list
        )
        if no_robot:
            print("[MMS] ⚠ T_EB = identity (로봇 미연결). "
                  "포인트 클라우드가 EE 프레임 기준으로 표시됩니다. "
                  "B·F 축과 일치하지 않을 수 있습니다.")

        # ── Base(B) 좌표계 (원점) ──────────────────────────────────────────
        base_axes = o3d.geometry.TriangleMesh.create_coordinate_frame(
            size=frame_size, origin=[0, 0, 0]
        )
        geoms.append(base_axes)

        # ── Turntable(F) 좌표계 ──────────────────────────────────────────
        if self.turntable_transform is not None:
            T_turntable = self.T_FB(theta)
            tt_axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=frame_size)
            tt_axes.transform(T_turntable)
            geoms.append(tt_axes)

            origin_B = T_turntable[:3, 3]
            rpy_deg = R.from_matrix(T_turntable[:3, :3]).as_euler("xyz", degrees=True)
            print(f"[MMS] Turntable(F) frame  theta={np.degrees(theta):.1f}°")
            print(f"  origin in B : [{origin_B[0]:.4f}, {origin_B[1]:.4f}, {origin_B[2]:.4f}] m")
            print(f"  RPY in B    : [{rpy_deg[0]:.2f}, {rpy_deg[1]:.2f}, {rpy_deg[2]:.2f}] deg")
        else:
            print("[MMS] turntable_transform 미설정 — Turntable 축 표시 생략.")

        # ── Camera(C) 좌표계 (첫 프레임 기준) ────────────────────────────
        if frames_list and self._T_EC is not None:
            T_EB_0 = frames_list[0].ee_pose_mat_B
            T_CB_0 = compute_T_CB(T_EB_0, self._T_EC)
            sensor_axes = o3d.geometry.TriangleMesh.create_coordinate_frame(
                size=frame_size * 0.6
            )
            sensor_axes.transform(T_CB_0)
            geoms.append(sensor_axes)

            c_origin = T_CB_0[:3, 3]
            print(f"[MMS] Camera(C) frame (frame 0 기준)")
            print(f"  origin in B : [{c_origin[0]:.4f}, {c_origin[1]:.4f}, {c_origin[2]:.4f}] m")

        # ── 포인트 클라우드 ────────────────────────────────────────────────
        _COLORS = [
            [0.85, 0.30, 0.25],
            [0.25, 0.65, 0.30],
            [0.25, 0.45, 0.85],
            [0.85, 0.70, 0.15],
            [0.65, 0.25, 0.80],
        ]

        total_pts = sum(len(f.points) for f in frames_list)
        ratio = min(1.0, max_pts / max(total_pts, 1))

        for i, frame in enumerate(frames_list):
            pts = frame.points
            cols = frame.colors

            if ratio < 1.0:
                idx = np.random.choice(len(pts), int(len(pts) * ratio), replace=False)
                pts = pts[idx]
                cols = cols[idx] if cols is not None else None

            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(pts.astype(np.float64))
            if cols is not None:
                pcd.colors = o3d.utility.Vector3dVector(cols.astype(np.float64))
            else:
                pcd.paint_uniform_color(_COLORS[i % len(_COLORS)])
            geoms.append(pcd)

        print(f"[MMS] visualize_with_object_frame: {len(frames_list)} frame(s)  "
              f"total_pts={total_pts:,}  rendered≤{max_pts:,}")
        print("  축 범례: B=큰축(원점)  O=큰축(턴테이블)  S=작은축(센서)  "
              "빨간=X  초록=Y  파란=Z")

        o3d.visualization.draw_geometries(
            geoms,
            window_name=title,
            width=1280,
            height=720,
        )
