"""
IsaacWorld — robot / turntable / scanner 백엔드가 공유하는 Isaac Sim 컨텍스트.

프로세스당 하나의 SimulationApp 만 존재 가능하므로 싱글톤처럼 1회만 생성한다.
standalone 패턴(`python.sh` 로 실행): SimulationApp 생성 → open_stage → World →
reset → drive 게인 보정. (GUI/extension 패턴 아님)

MMS_ext.py 에서 검증된 설정(카메라 광학, 드라이브 게인, joint1 한계 정상화)을 이식.
"""

from __future__ import annotations

import math
import os
from typing import Optional

import numpy as np

# ── 씬/프림 경로 (v3_scene.usd — 260811 신규 레이아웃 기준) ───────────────────
# v3_scene.usd 는 scripts/sim/build_scene_v3.py 가 생성한다(형상 src = v3.usd).
# 구 v2.usd 경로는 git 이력 참고. 씬은 MMS_SIM_USD 로 override 가능.
DEFAULT_USD_PATH = ("/home/keti/workspace/sync/2_Rapid_Digital_Twin/1_MMS/2_3Dassets/"
                    "frame_xarm7_spider_turntable_v2/v3_scene.usd")
ROBOT_PRIM       = "/World/xarm7"
JOINTS_SCOPE     = "/World/xarm7/joints"
EE_LINK_NAME     = "link7"
EE_LINK_PATH     = f"{ROBOT_PRIM}/{EE_LINK_NAME}"
# 스캐너가 툴체인저(브래킷→마스터→툴플레이트→어댑터) 뒤로 옮겨져 경로가 깊어졌다.
CAMERA_PRIM      = "/World/xarm7/link7/tool/spider/Camera"
# 턴테이블 구조:
#   DISC_PRIM   — 도는 원판(kinematic). 객체가 이 위에 놓이고 회전중심을 여기서 읽는다.
#                 ⚠ RevoluteJoint 는 쓰지 않는다 — isaac_turntable.py 헤더 참고.
#   FRAME_PRIM  — 고정 베이스(모터 하우징 쪽).
#   OBJECT_PRIM — 스캔 대상물(원판과 함께 회전).
DISC_PRIM        = "/World/frame/turntable_disc"
FRAME_PRIM       = "/World/frame/turntable_base"
OBJECT_PRIM      = os.environ.get(   # 스캔 대상(rider). build_scene_v3.py --object 가
    "MMS_SIM_OBJECT_PRIM",           # /World/ScanTarget/TestObject 로 올려둔다.
    "/World/ScanTarget/TestObject")

# Artec Space Spider 광학 (MMS_ext.py 와 동일)
SPIDER_HFOV_DEG         = 30.0
SPIDER_WORKING_DISTANCE = (0.2, 0.3)
SPIDER_FOCUS_DISTANCE   = 0.25
SCANNER_RESOLUTION      = (1280, 960)

# 드라이브 게인 (트램블링 방지) / joint1 한계 정상화
DRIVE_STIFFNESS        = 2000.0
DRIVE_DAMPING          = 200.0
WIDEN_JOINT1_LIMIT_DEG = 175.0


class IsaacWorld:
    """Isaac Sim 공유 컨텍스트. robot/turntable/scanner 가 이 인스턴스를 공유한다."""

    def __init__(self, usd_path: Optional[str] = None, headless: bool = False,
                 robot_collisions: bool = False):
        self.usd_path = usd_path or DEFAULT_USD_PATH
        self._robot_collisions = bool(robot_collisions)

        # ── 1. SimulationApp 먼저 (이후 isaac/pxr import 가능) ────────────────
        from isaacsim import SimulationApp
        self._sim_app = SimulationApp({
            "headless": bool(headless),
            "width": 1280, "height": 720,
        })

        # ── 2. 나머지 import ─────────────────────────────────────────────────
        import omni.usd
        from isaacsim.core.api import World
        from isaacsim.core.utils.stage import open_stage
        from isaacsim.core.api.robots import Robot
        from isaacsim.core.prims import RigidPrim
        from isaacsim.sensors.camera import Camera
        from pxr import Usd, UsdGeom, Sdf, Gf

        self._omni_usd = omni.usd
        self._UsdGeom = UsdGeom
        self._Gf = Gf
        self._Usd = Usd
        self._Sdf = Sdf
        self._ArticulationActionImported = False

        # ── 3. stage 열기 + 광학/한계 설정 ───────────────────────────────────
        print(f"[IsaacWorld] Opening USD: {self.usd_path}")
        open_stage(usd_path=self.usd_path)
        self.stage = omni.usd.get_context().get_stage()

        self._widen_joint1_limit()
        self._configure_scanner_camera()
        self._prepare_turntable()
        self._set_robot_collisions(self._robot_collisions)

        # ── 4. World / 로봇 / 카메라 ─────────────────────────────────────────
        self.world = World(physics_dt=1.0 / 60.0, rendering_dt=1.0 / 60.0,
                           stage_units_in_meters=1.0)
        self.robot = self.world.scene.add(Robot(prim_path=ROBOT_PRIM, name="xarm7"))
        self._scanner_cam = Camera(prim_path=CAMERA_PRIM, resolution=SCANNER_RESOLUTION)
        self._rigid_ee = RigidPrim(prim_paths_expr=EE_LINK_PATH)

        # ── 5. 초기화 ────────────────────────────────────────────────────────
        self.world.reset()
        self._rigid_ee.initialize()
        self._configure_joint_drives()
        # 키네마틱 개발용 sim: 로봇 중력 비활성 → 위치 제어가 목표 자세에 정확히
        # 유지된다(중력 sag 제거). 물리 궤적 충실도는 이 용도에서 불필요.
        try:
            self.robot.disable_gravity()
        except Exception as e:
            print(f"[IsaacWorld][WARN] disable_gravity 실패(무시): {e}")

        self.view = self.robot._articulation_view
        self.num_dof = int(self.view.num_dof)
        self.dof_names = list(self.robot.dof_names)

        # XformCache (월드 포즈 조회용)
        self._xc = UsdGeom.XformCache(Usd.TimeCode.Default())

        # 카메라 센서 핸들(Phase B 에서 initialize)
        self._cam_initialized = False

        print(f"[IsaacWorld] ready — dof={self.num_dof} names={self.dof_names} "
              f"headless={headless}")

    # ── 설정 헬퍼 (MMS_ext.py 이식) ─────────────────────────────────────────
    def _widen_joint1_limit(self):
        if WIDEN_JOINT1_LIMIT_DEG is None:
            return
        j1 = self.stage.GetPrimAtPath(f"{JOINTS_SCOPE}/joint1")
        if j1.IsValid():
            j1.GetAttribute("physics:lowerLimit").Set(-float(WIDEN_JOINT1_LIMIT_DEG))
            j1.GetAttribute("physics:upperLimit").Set(float(WIDEN_JOINT1_LIMIT_DEG))

    def _configure_scanner_camera(self):
        Sdf, Gf = self._Sdf, self._Gf
        cam = self.stage.GetPrimAtPath(CAMERA_PRIM)
        if not cam.IsValid():
            print(f"[IsaacWorld][WARN] camera prim not found: {CAMERA_PRIM}")
            return

        def _attr(name, vtype, value):
            a = cam.GetAttribute(name)
            if not a.IsValid():
                a = cam.CreateAttribute(name, vtype)
            a.Set(value)

        h_ap_attr = cam.GetAttribute("horizontalAperture")
        h_aperture = float(h_ap_attr.Get()) if h_ap_attr.IsValid() and h_ap_attr.Get() else 20.5
        focal = h_aperture / (2.0 * math.tan(math.radians(SPIDER_HFOV_DEG) / 2.0))
        w, h = SCANNER_RESOLUTION
        v_aperture = h_aperture * (h / w)

        _attr("focalLength",        Sdf.ValueTypeNames.Float, float(focal))
        _attr("horizontalAperture", Sdf.ValueTypeNames.Float, float(h_aperture))
        _attr("verticalAperture",   Sdf.ValueTypeNames.Float, float(v_aperture))
        _attr("clippingRange",      Sdf.ValueTypeNames.Float2,
              Gf.Vec2f(float(SPIDER_WORKING_DISTANCE[0]), float(SPIDER_WORKING_DISTANCE[1])))
        _attr("focusDistance",      Sdf.ValueTypeNames.Float, float(SPIDER_FOCUS_DISTANCE))

    def _prepare_turntable(self):
        """
        턴테이블 회전 = kinematic 직접 회전 (결정론·올바른 위치).

        원본 v2.usd 의 RevoluteJoint 는 앵커가 disc 의 authored 위치와 불일치해서,
        disc 를 dynamic(조인트 드라이브) 로 두면 reset 시 solver 가 disc 를 앵커로
        스냅해 프레임에서 ~0.3m 떨어뜨린다. → DISC/FRAME/OBJECT 를 모두 kinematic 으로
        고정하고(올바른 USD 위치 유지), IsaacTurntable 이 disc+객체+fixture 를 디스크
        중심 world-Z 로 직접 회전시킨다. 조인트는 비활성화(둘 다 kinematic → static
        bodies 에러 방지).
        """
        from pxr import Sdf
        for path in (DISC_PRIM, FRAME_PRIM, OBJECT_PRIM):
            prim = self.stage.GetPrimAtPath(path)
            if not prim.IsValid():
                print(f"[IsaacWorld][WARN] turntable prim not found: {path}")
                continue
            a = prim.GetAttribute("physics:kinematicEnabled")
            if not a or not a.IsValid():
                a = prim.CreateAttribute("physics:kinematicEnabled", Sdf.ValueTypeNames.Bool)
            a.Set(True)
        rj = self.stage.GetPrimAtPath(f"{DISC_PRIM}/RevoluteJoint")
        if rj.IsValid():
            je = rj.GetAttribute("physics:jointEnabled")
            if not je or not je.IsValid():
                je = rj.CreateAttribute("physics:jointEnabled", Sdf.ValueTypeNames.Bool)
            je.Set(False)

    def _set_robot_collisions(self, enabled: bool):
        """
        로봇 링크 충돌 on/off.

        로봇은 '이상적 키네마틱 로봇'(논리는 명령각 _q 기반, sim 은 시각화 + 카메라
        렌더)이다. 충돌을 **켜면** PhysX 접촉력이 위치 드라이브와 싸워, 실제로 닿는
        자세에서 진동(jitter)·도달오차 → 카메라 포즈 오염 → calibration 오염.
        반대로 닿지 않는 자세(현재 충돌-free 홈/조준 포즈)에선 정착 후 영향 0.
        → 기본은 OFF(렌더 무관, 정확도/안정성 확보). 물리 충돌 검사가 필요하면
          robot_collisions=True 로 켠다(단, 실제 충돌 자세에선 jitter 주의).
        """
        from pxr import Usd, UsdPhysics, Sdf
        n = 0
        for prim in Usd.PrimRange(self.stage.GetPrimAtPath(ROBOT_PRIM)):
            if prim.HasAPI(UsdPhysics.CollisionAPI):
                a = prim.GetAttribute("physics:collisionEnabled")
                if not a or not a.IsValid():
                    a = prim.CreateAttribute("physics:collisionEnabled", Sdf.ValueTypeNames.Bool)
                a.Set(bool(enabled))
                n += 1
        print(f"[IsaacWorld] robot collisions {'ENABLED' if enabled else 'disabled'} "
              f"on {n} prim(s)")

    def _configure_joint_drives(self):
        view = self.robot._articulation_view
        nd = view.num_dof
        kps = np.full((1, nd), DRIVE_STIFFNESS, dtype=np.float32)
        kds = np.full((1, nd), DRIVE_DAMPING, dtype=np.float32)
        view.set_gains(kps=kps, kds=kds)
        # 원본 USD maxForce(20~100)는 일부 자세에서 중력 토크에 포화 → home 자세
        # 도달 오차. 충분한 최대 토크로 올려 위치 드라이브가 목표에 수렴하게 한다.
        try:
            view.set_max_efforts(np.full((1, nd), 500.0, dtype=np.float32))
        except Exception as e:
            print(f"[IsaacWorld][WARN] set_max_efforts 실패(무시): {e}")

    # ── 시뮬레이션 제어 ─────────────────────────────────────────────────────
    def step(self, n: int = 1, render: bool = True):
        for _ in range(int(n)):
            self.world.step(render=render)

    def is_running(self) -> bool:
        return self._sim_app.is_running()

    def close(self):
        try:
            self._sim_app.close()
        except Exception:
            pass

    # ── 관절 상태/구동 ──────────────────────────────────────────────────────
    def get_joint_positions(self) -> np.ndarray:
        return np.asarray(self.robot.get_joint_positions(), dtype=float)

    def _articulation_action(self, q):
        from isaacsim.core.utils.types import ArticulationAction
        return ArticulationAction(joint_positions=np.asarray(q, dtype=float))

    def drive_to_joints(self, q_target: np.ndarray,
                        settle_steps: int = 8, render: bool = True) -> np.ndarray:
        """
        관절공간 목표 q_target(rad, 7) 로 이동.

        sim 개발 목적상 물리 궤적 충실도보다 '목표 자세 정확 도달'이 중요하므로,
        관절 상태를 직접 세팅(텔레포트)하고 드라이브 타깃을 같은 값으로 고정한 뒤
        짧게 settle 한다. (PD 드라이브로 큰 점프를 수렴시키면 중력/스텝수 한계로
        오차가 남음.) 반환: 최종 측정 관절각.
        """
        q_target = np.asarray(q_target, dtype=float)
        # 드라이브 위치 타깃을 목표로 고정 (이후 step 에서도 유지) + 상태 텔레포트
        act = self._articulation_action(q_target)
        self.robot.apply_action(act)
        try:
            self.robot.set_joint_positions(q_target)
            self.robot.set_joint_velocities(np.zeros_like(q_target))
        except Exception:
            pass
        for _ in range(int(settle_steps)):
            self.robot.apply_action(act)
            self.world.step(render=render)
        return self.get_joint_positions()

    # ── 월드 포즈 조회 ──────────────────────────────────────────────────────
    def prim_world_pose(self, prim_path: str):
        """prim 의 (pos(3) m, R(3x3)) world 포즈. scale 제거된 회전."""
        Gf = self._Gf
        self._xc.Clear()
        prim = self.stage.GetPrimAtPath(prim_path)
        m = self._xc.GetLocalToWorldTransform(prim)
        t = m.ExtractTranslation()
        R = np.array(m.RemoveScaleShear().ExtractRotationMatrix()).T  # USD row-vector → 표준 R
        return np.array([t[0], t[1], t[2]], dtype=float), R

    def base_world_pose(self):
        """로봇 베이스(B) world 포즈."""
        return self.prim_world_pose(ROBOT_PRIM)

    def ee_world_pose(self):
        """EE(link7) world 포즈 (PhysX 텐서)."""
        pos, quat = self._rigid_ee.get_world_poses()
        return np.asarray(pos[0], dtype=float), np.asarray(quat[0], dtype=float)

    # ── 카메라 조준 (DLS look-at) ───────────────────────────────────────────
    def look_at_camera(self, target_world, cam_pos_world,
                       up=(0.0, 0.0, 1.0), max_iter: int = 600,
                       pos_tol: float = 2e-3, rot_tol: float = 0.01) -> float:
        """
        스캐너 카메라의 광축(-Z)이 target_world 를 향하도록, 카메라를 cam_pos_world
        에 두는 EE 자세를 DLS IK 로 구동한다. (sim world 프레임 일관)

        Returns: 수렴 후 광축과 (카메라→타깃) 사이 off-axis 각도(도).
        """
        from isaacsim.core.utils.types import ArticulationAction
        target = np.asarray(target_world, float)
        cam_pos = np.asarray(cam_pos_world, float)

        # T_EC (link7 → camera), 현재 포즈에서 계산 (강체 고정이라 불변)
        cp, cR = self.prim_world_pose(CAMERA_PRIM)
        ep, eR = self.prim_world_pose(EE_LINK_PATH)
        T_E = np.eye(4); T_E[:3, :3] = eR; T_E[:3, 3] = ep
        T_C = np.eye(4); T_C[:3, :3] = cR; T_C[:3, 3] = cp
        T_EC = np.linalg.inv(T_E) @ T_C

        # 목표 카메라 자세: +Z = (cam→target 반대) = 카메라에서 멀어지는 광축의 반대
        zc = cam_pos - target
        zc = zc / (np.linalg.norm(zc) + 1e-12)
        xc = np.cross(np.asarray(up, float), zc); xc /= (np.linalg.norm(xc) + 1e-12)
        yc = np.cross(zc, xc)
        T_Cd = np.eye(4); T_Cd[:3, :3] = np.column_stack([xc, yc, zc]); T_Cd[:3, 3] = cam_pos
        T_Ed = T_Cd @ np.linalg.inv(T_EC)
        des_p, des_R = T_Ed[:3, 3], T_Ed[:3, :3]

        view = self.view
        nd = self.num_dof
        bi = view.get_body_index(EE_LINK_NAME)
        lim = np.asarray(view.get_dof_limits())[0]
        lo, hi = lim[:, 0], lim[:, 1]

        def _quatR(q):
            a, b, c, d = q
            return np.array([[1-2*(c*c+d*d), 2*(b*c-a*d), 2*(b*d+a*c)],
                             [2*(b*c+a*d), 1-2*(b*b+d*d), 2*(c*d-a*b)],
                             [2*(b*d-a*c), 2*(c*d+a*b), 1-2*(b*b+c*c)]])

        def _rotvec(R):
            co = np.clip((np.trace(R) - 1) / 2, -1, 1); ang = np.arccos(co)
            if ang < 1e-9:
                return np.zeros(3)
            return ang / (2*np.sin(ang)) * np.array(
                [R[2, 1]-R[1, 2], R[0, 2]-R[2, 0], R[1, 0]-R[0, 1]])

        for _ in range(int(max_iter)):
            p, q = self._rigid_ee.get_world_poses()
            pcur = np.asarray(p[0]); Rcur = _quatR(np.asarray(q[0]))
            e_pos = des_p - pcur
            e_rot = _rotvec(des_R @ Rcur.T)
            if np.linalg.norm(e_pos) < pos_tol and np.linalg.norm(e_rot) < rot_tol:
                break
            e_pos = e_pos * min(1.0, 0.03 / (np.linalg.norm(e_pos) + 1e-9))
            e_rot = e_rot * min(1.0, 0.15 / (np.linalg.norm(e_rot) + 1e-9))
            J = np.asarray(view.get_jacobians())[0][bi][:, -nd:]
            dq = J.T @ np.linalg.solve(J @ J.T + 0.0064 * np.eye(6),
                                       np.concatenate([e_pos, e_rot])) * 0.7
            mx = np.max(np.abs(dq))
            if mx > 0.04:
                dq *= 0.04 / mx
            qn = np.clip(self.get_joint_positions() + dq, lo, hi)
            self.robot.apply_action(ArticulationAction(joint_positions=qn))
            self.world.step(render=True)

        cp2, cR2 = self.prim_world_pose(CAMERA_PRIM)
        fwd = -cR2[:, 2]
        to = target - cp2; to /= (np.linalg.norm(to) + 1e-12)
        return float(np.degrees(np.arccos(np.clip(fwd @ to, -1, 1))))

    # ── 카메라 (Phase B) ────────────────────────────────────────────────────
    def ensure_camera(self):
        if not self._cam_initialized:
            self._scanner_cam.initialize()
            self._scanner_cam.add_distance_to_image_plane_to_frame()
            self._scanner_cam.add_pointcloud_to_frame()
            self._cam_initialized = True
        return self._scanner_cam
