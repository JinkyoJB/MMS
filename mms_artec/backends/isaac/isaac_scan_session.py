"""
isaac_scan_session — Isaac Sim 스캔 세션 (Phase 1 GT 누적 + Phase 2 NBV 보강).

`mms_artec/system.py::artec_process` 의 isaac 분기가 호출하는 **production sim 스캔**.
지금까지 standalone 하니스(MMS_ext_phase2_nbv.py)에만 있던 로직을 여기로 옮겨,
`main_artec.py`(BACKEND="isaac", `~/isaacsim/python.sh main_artec.py`)로 동작하게 한다.

★ standalone(python.sh) 에선 Isaac 확장의 `utils` 패키지 충돌이 없으므로 **공용 lib을
  그대로 import** 한다 → Phase 2 NBV 가 **real 과 동일한 공용 코어**를 쓴다:
    - `utils/nbv/phase2_nbv.py`        : pcd→mesh→gap→커버리지→NBV pose (공용)
    - `utils/collision/robot_collision`: CollisionWorld·swept·keepout (공용)
    - `utils/robot/xarm7_kinematics`   : 해석 IK/FK (공용)
  sim 전용(USD 객체·Isaac 카메라·입사각 스캐너 모델·스캐너 mesh 자가충돌)만 이 파일에 둔다.

흐름: Phase 1(턴테이블 GT θ + 로봇 고정 + 입사각필터 + −θ 누적) → Phase 2(부족면 NBV).
입사각 필터 = 구조광 스캐너가 grazing 면을 못 잡는 모델 → 윗면 gap 생성(=NBV 대상).

설정은 아래 모듈 상수(또는 env)로. 실행 후 결과는 ArtecProcessResult-호환 shim 으로 반환
(후처리 GlobalReg/Fusion 은 sim 에서 sensor stub 가 skip).
"""
from __future__ import annotations

import math
import os
from dataclasses import dataclass, field
from typing import List, Optional

import numpy as np
from pxr import Usd, UsdGeom, Gf

# ── 공용 lib (real 과 동일) ─────────────────────────────────────────────────────
from utils.nbv import phase2_nbv as p2
from utils.collision.robot_collision import (
    CollisionWorld, pose_collision, DEFAULT_LINK_RADII)
from utils.robot import xarm7_kinematics as kin
from utils.control.theta_planner import DEFAULT_JOINT_WEIGHTS
# look-at = **USD 규약(-Z 광축)** — sim 카메라(USD)와 일치(하니스와 동일). 자기완결(numpy).
from mms_artec.utils.calibration.handeye_geometry import (
    look_at_camera as _look_at, make_T as _make_T)


# ── 설정 (env 로 override 가능) ─────────────────────────────────────────────────
def _envf(k, d): return float(os.environ.get(k, d))
def _envs(k, d): return os.environ.get(k, d)

OBJECT_SOURCE   = _envs("MMS_SIM_OBJECT", "usd")     # "usd"(ScanTarget) | "spawn"(테스트 형상)
OBJECT_PRIM     = "/World/ScanTarget/Solid_Marble"   # OBJECT_SOURCE="usd" 일 때 스캔 대상
SPAWN_SHAPE     = _envs("MMS_SIM_SHAPE", "box")      # spawn: box|cylinder|sphere|cone|lshape|stepped
SPAWN_PRIM      = "/World/SimScanObject"

# Spider working distance(0.2~0.3) 중앙. standoff 는 **표면이 이 거리에 오도록** 동적 계산
# (카메라 클립=실 스펙 0.2~0.3 유지 → 표면이 0.25 면 근/원접클립 모두 안전).
WORK_FOCUS      = 0.25
DRIVE_STEPS     = int(_envf("MMS_SIM_DRIVE_STEPS", 30))   # 관절 보간 스텝(클수록 부드럽고 느림)
N_THETA         = 36
N_THETA_P2      = int(_envf("MMS_SIM_NTHETA_P2", 24))   # Phase2 보강 전회전 프레임수(빠르게)
VIEW_EL_DEG     = _envf("MMS_SIM_VIEW_EL", 30.0)     # Phase1 측면 관측 elevation
VIEW_AZIS_DEG   = [0.0, 30.0, -30.0, 60.0, -60.0, 90.0, -90.0, 180.0]
VOXEL_M         = 0.002
MAX_INCIDENCE_DEG = _envf("MMS_SIM_MAXINC", 50.0)    # grazing 임계(입사각>이값 → 미캡처=gap)
APPLY_INCIDENCE = _envs("MMS_SIM_INCIDENCE", "1") == "1"

# Phase 2 NBV
NBV_DISTANCE_M  = 0.225
NBV_APPROACH_ELS  = [55.0, 50.0, 60.0, 65.0]
NBV_APPROACH_AZIS = [0.0, 30.0, -30.0, 60.0, -60.0, 90.0, -90.0, 180.0]
NBV_K_MAX       = 4
NBV_SWEPT_STEPS = 12
GAP_KW = dict(min_seg_vertices=8, min_seg_length=0.006, max_seg_length=0.06)

# 충돌 world (공용 robot_collision)
TT_DISC_RADIUS_M = 0.15
TT_BODY_H_M      = 0.20
COLLISION_MARGIN_M = 0.01
# ⚠ keepout 원기둥(턴테이블 지름)은 **스캔 대상 물체 공간을 감싸** NBV 접근을 막으므로
#   sim 스캔에선 기본 OFF. (물체와 안 겹치는 별도 금지구역이 필요할 때만 MMS_SIM_KEEPOUT=1)
KEEPOUT_ENABLE   = _envs("MMS_SIM_KEEPOUT", "0") == "1"
KEEPOUT_HEIGHT_M = 0.12
# 스캐너 자가충돌은 **공용 pose_collision(scanner_self=True)** 이 처리(스캐너 캡슐 반경=
# DEFAULT_LINK_RADII[6]=0.095, real/sim 공용). sim 전용 bbox 체크는 폐기.
LINK_RADII = DEFAULT_LINK_RADII
SCANNER_PRIM = "/World/xarm7/link7/Artec_Space_Spider_mm"
LINK7_PRIM   = "/World/xarm7/link7"
TURNTABLE_MESH = "/World/ScanTarget/turntable_demo/turntable/turntable"   # sim 턴테이블(축 산출)
FRAME_PRIM     = "/World/ScanTarget/turntable_demo/turntable_frame/turntable_frame"  # 모터 프레임(충돌)
FRAME_ENABLE   = _envs("MMS_SIM_FRAME", "1") == "1"   # 프레임을 충돌 장애물로(이동 중 회피)


# ── 결과 shim (ArtecProcessResult / IModel 호환 최소 구현) ──────────────────────
class _SimFrame:
    def __init__(self, pts_m): self._p = pts_m
    def vertices(self): return self._p * 1000.0      # mm (main 이 /1000)


class _SimScan:
    def __init__(self, pts_m): self._p = pts_m
    def frame_count(self): return 1
    def get_frame(self, i): return _SimFrame(self._p)
    def get_frame_transformation(self, i): return np.eye(4)


class _SimModel:
    """artec_process / main_artec._show_composite_mesh 가 기대하는 최소 인터페이스."""
    def __init__(self, pts_m, mesh=None):
        self._p = np.asarray(pts_m, float)
        self._mesh = mesh
    def scan_count(self): return 1
    def get_scan(self, i): return _SimScan(self._p)
    def has_final_mesh(self):
        return self._mesh is not None and len(self._mesh.triangles) > 0
    def final_vertices(self): return np.asarray(self._mesh.vertices) * 1000.0
    def final_faces(self): return np.asarray(self._mesh.triangles)

    def save_obj(self, path):                        # artec_process export 호환
        import open3d as o3d
        if self.has_final_mesh():
            o3d.io.write_triangle_mesh(path, self._mesh)
        else:
            pc = o3d.geometry.PointCloud()
            pc.points = o3d.utility.Vector3dVector(self._p)
            o3d.io.write_point_cloud(path.rsplit(".", 1)[0] + ".ply", pc)   # 점군은 .obj 미지원


@dataclass
class IsaacScanResult:
    model: _SimModel
    n_frames: int = 0
    ctx = None                                       # streaming 호환(없음)


# ── 세션 ────────────────────────────────────────────────────────────────────────
class IsaacScanSession:
    def __init__(self, mms, robot, turntable, settings=None):
        self.mms = mms
        self.robot = robot
        self.turntable = turntable
        self.s = settings
        self.world = mms.sensor._world              # IsaacWorld
        self.stage = self.world.stage
        self.scanner = mms.sensor                   # IsaacArtecScanner
        # 트랜스폼
        self.T_WB = self._base_world_T()            # base→world
        self.T_EC = self._T_EC_gt()                 # E→C (sim GT, look_at 과 일관)
        # ★ 턴테이블 축 = **sim USD** 에서. crop/누적은 **world** 에서(base 회전·박스높이 무관).
        mn, mx = self._aabb_world(TURNTABLE_MESH)
        self.axis_w = np.array([(mn[0]+mx[0])/2.0, (mn[1]+mx[1])/2.0, mx[2]])  # disc 표면중심(world)
        self.axis_dir_w = np.array([0.0, 0.0, 1.0])                            # 수직(world up)
        self.axis_pt = self._world_to_base(self.axis_w)                       # base(충돌 world 용)
        _ad = self.T_WB[:3, :3].T @ self.axis_dir_w
        self.axis_dir = _ad / (np.linalg.norm(_ad) + 1e-12)
        print(f"[isaac_scan] 턴테이블 축(sim USD) world={np.round(self.axis_w,3).tolist()}")
        # 카메라 클립 = 실제 Spider 스펙(SPIDER_WORKING_DISTANCE=0.2~0.3) 그대로 사용.
        # → standoff 를 **표면이 working-focus(WORK_FOCUS)에 오도록** 동적 계산(검은화면 방지).
        self.accum: List[np.ndarray] = []

    # ── 트랜스폼 헬퍼 ────────────────────────────────────────────────────────
    def _base_world_T(self):
        pos, R = self.world.base_world_pose()
        T = np.eye(4); T[:3, :3] = R; T[:3, 3] = pos
        return T

    def _prim_world_T(self, path):
        xc = UsdGeom.XformCache(Usd.TimeCode.Default())
        m = xc.GetLocalToWorldTransform(self.stage.GetPrimAtPath(path)).RemoveScaleShear()
        T = np.eye(4)
        T[:3, :3] = np.array(m.ExtractRotationMatrix(), float).T
        T[:3, 3] = np.array(m.ExtractTranslation(), float)
        return T

    def _T_EC_gt(self):
        T_WC = self._prim_world_T(SCANNER_PRIM + "/Camera")
        T_WE = self._prim_world_T(LINK7_PRIM)
        return np.linalg.inv(T_WC) @ T_WE           # E→C

    def _world_to_base(self, p_w):
        Twb = self.T_WB
        return (np.asarray(p_w) - Twb[:3, 3]) @ Twb[:3, :3]   # inv(R)=R^T, p_b = R^T(p_w-t)

    def _base_to_world(self, p_b):
        return (np.asarray(p_b) @ self.T_WB[:3, :3].T) + self.T_WB[:3, 3]

    # ── 객체 (USD bbox 또는 spawn) ───────────────────────────────────────────
    def _aabb_world(self, prim_path):
        bbc = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                                [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
        rng = bbc.ComputeWorldBound(self.stage.GetPrimAtPath(prim_path)).ComputeAlignedRange()
        mn = np.array(rng.GetMin(), float); mx = np.array(rng.GetMax(), float)
        return mn, mx

    def _setup_object(self):
        if OBJECT_SOURCE == "spawn":
            self._spawn_object()
            prim = SPAWN_PRIM
        else:
            prim = OBJECT_PRIM
            UsdGeom.Imageable(self.stage.GetPrimAtPath(
                "/World/ScanTarget/Solid_Marble")).MakeVisible()
        mn, mx = self._aabb_world(prim)                       # 객체 world AABB
        self.obj_center_w = (mn + mx) / 2.0                   # 객체 중심(world)
        self.obj_top_w = np.array([self.obj_center_w[0], self.obj_center_w[1], mx[2]])
        self.obj_zlo_w, self.obj_zhi_w = float(mn[2]), float(mx[2])   # **실제 객체 z 범위(world)**
        self.obj_radius = float(max(mx[0] - mn[0], mx[1] - mn[1]) / 2.0)
        self.obj_height = float(mx[2] - mn[2])
        off = float(np.linalg.norm(self.obj_center_w[:2] - self.axis_w[:2]))   # 축 이탈(수평)
        print(f"[isaac_scan] 객체='{prim}' r={self.obj_radius*1000:.0f}mm h={self.obj_height*1000:.0f}mm "
              f"center_w={np.round(self.obj_center_w,3).tolist()} z=[{mn[2]:.3f},{mx[2]:.3f}]")
        print(f"[isaac_scan] ⚠ 객체-턴테이블축 수평이탈={off*1000:.0f}mm "
              f"(클수록 Phase1 고정카메라가 회전 중 객체를 놓침→sector 구멍)")

    def _spawn_object(self):
        # 턴테이블 disc 중심/표면(world)
        mn, mx = self._aabb_world(
            "/World/ScanTarget/turntable_demo/turntable/turntable")
        cx, cy, ztop = (mn[0] + mx[0]) / 2, (mn[1] + mx[1]) / 2, mx[2]
        w, d, h = {"box": (.06, .04, .07), "cylinder": (.06, .06, .07),
                   "sphere": (.06, .06, .06), "cone": (.06, .06, .08),
                   "lshape": (.08, .05, .07), "stepped": (.07, .05, .08)
                   }.get(SPAWN_SHAPE, (.06, .04, .07))
        root = UsdGeom.Xform.Define(self.stage, SPAWN_PRIM)
        root.AddTranslateOp().Set(Gf.Vec3d(cx, cy, ztop + h / 2))
        col = [Gf.Vec3f(0.85, 0.35, 0.2)]

        def cube(p, t, sc):
            g = UsdGeom.Cube.Define(self.stage, p); g.GetSizeAttr().Set(1.0)
            g.AddTranslateOp().Set(Gf.Vec3f(*t)); g.AddScaleOp().Set(Gf.Vec3f(*sc))
            g.CreateDisplayColorAttr(col)
        gp = SPAWN_PRIM + "/geom"
        if SPAWN_SHAPE == "cylinder":
            g = UsdGeom.Cylinder.Define(self.stage, gp); g.GetRadiusAttr().Set(w/2)
            g.GetHeightAttr().Set(h); g.GetAxisAttr().Set("Z"); g.CreateDisplayColorAttr(col)
        elif SPAWN_SHAPE == "sphere":
            g = UsdGeom.Sphere.Define(self.stage, gp); g.GetRadiusAttr().Set(h/2)
            g.CreateDisplayColorAttr(col)
        elif SPAWN_SHAPE == "cone":
            g = UsdGeom.Cone.Define(self.stage, gp); g.GetRadiusAttr().Set(w/2)
            g.GetHeightAttr().Set(h); g.GetAxisAttr().Set("Z"); g.CreateDisplayColorAttr(col)
        elif SPAWN_SHAPE == "lshape":
            cube(gp+"A", (-w/4, 0, 0), (w/2, d, h)); cube(gp+"B", (w/4, 0, -h/4), (w/2, d, h/2))
        elif SPAWN_SHAPE == "stepped":
            cube(gp+"A", (0, 0, -0.2*h), (w, d, 0.6*h)); cube(gp+"B", (0, 0, 0.3*h), (0.5*w, 0.5*d, 0.4*h))
        else:
            cube(gp, (0, 0, 0), (w, d, h))
        # 마블 숨김(테스트 객체만 스캔)
        mb = self.stage.GetPrimAtPath("/World/ScanTarget/Solid_Marble")
        if mb.IsValid():
            UsdGeom.Imageable(mb).MakeInvisible()

    # ── 캡처(입사각 스캐너 모델) — **world 프레임** ──────────────────────────
    def _cam_pos_world(self):
        T_EB = self.robot.get_ee_pose_mat()                   # E→B (m)
        cam_b = (T_EB @ np.linalg.inv(self.T_EC))[:3, 3]      # camera in base
        return self._base_to_world(cam_b)

    def _capture_obj_world(self, log=False):
        """capture_points_base → **world** 변환 → 객체 crop(world bbox) + (옵션)입사각 필터.
        ★ world 에서 crop: base 회전·박스높이 무관. 반환 = world 점군."""
        pc_b = self.scanner.capture_points_base(self.robot, self.mms._T_EC, settle=6)
        if pc_b is None or len(pc_b) == 0:
            if log: print("    [capture] raw 0 — 카메라 점군 없음")
            return np.zeros((0, 3))
        pc = self._base_to_world(pc_b)                        # → world
        c = self.obj_center_w
        r = np.linalg.norm(pc[:, :2] - c[:2], axis=1)
        m = ((r < self.obj_radius + 0.02)
             & (pc[:, 2] > self.obj_zlo_w - 0.005) & (pc[:, 2] < self.obj_zhi_w + 0.02))
        obj = _voxel(pc[m], VOXEL_M)
        n_crop = len(obj)
        if APPLY_INCIDENCE and len(obj) >= 4:
            obj = self._incidence_filter(obj)
        if log:
            print(f"    [capture] raw={len(pc_b)} crop={n_crop} incidence후={len(obj)}")
        return obj

    def _incidence_filter(self, pts_w):
        nrm = _pca_normals(pts_w, c_ref=self.obj_center_w)
        if nrm is None:
            return pts_w
        vd = self._cam_pos_world()[None, :] - pts_w
        vd /= (np.linalg.norm(vd, axis=1, keepdims=True) + 1e-9)
        cos_inc = np.sum(nrm * vd, axis=1)
        return pts_w[cos_inc > math.cos(math.radians(MAX_INCIDENCE_DEG))]

    # ── 충돌 (공용 robot_collision + 스캐너 mesh) ────────────────────────────
    def _build_world(self) -> CollisionWorld:
        w = CollisionWorld.from_turntable(
            surface_point=self.axis_pt, axis_dir=self.axis_dir,
            disc_radius=TT_DISC_RADIUS_M, body_height=TT_BODY_H_M, margin=COLLISION_MARGIN_M)
        if KEEPOUT_ENABLE:
            w.add_cylinder("keepout", self.axis_pt[:2], float(self.axis_pt[2]),
                           float(self.axis_pt[2]) + KEEPOUT_HEIGHT_M,
                           TT_DISC_RADIUS_M, margin=COLLISION_MARGIN_M)
        # 턴테이블 모터 프레임도 장애물(arm 이 부딪히지 않게). world AABB → base box.
        if FRAME_ENABLE:
            try:
                mn, mx = self._aabb_world(FRAME_PRIM)
                cw = np.array([[x, y, z] for x in (mn[0], mx[0])
                               for y in (mn[1], mx[1]) for z in (mn[2], mx[2])])
                cb = self._world_to_base(cw)
                w.add_box("tt_frame", cb.min(0), cb.max(0), margin=COLLISION_MARGIN_M)
                print(f"[isaac_scan] 충돌: 턴테이블 프레임 추가 (size_cm="
                      f"{np.round((mx-mn)*100,1).tolist()})")
            except Exception as e:
                print(f"[isaac_scan] ⚠ 프레임 충돌 추가 실패({e})")
        return w

    def _pose_collision(self, world, q):
        # 공용 pose_collision — 스캐너 자가충돌(scanner_self=True)·턴테이블/프레임 포함.
        col, _ = pose_collision(world, q, T_EC=self.T_EC, link_radii=LINK_RADII)
        return col

    def _pose_collision_reason(self, world, q):
        """''=충돌없음. 아니면 사유(scene/self). self = 스캐너 포함 자가충돌."""
        c1, why = pose_collision(world, q, T_EC=self.T_EC, link_radii=LINK_RADII)
        if c1:
            return ("self" if "self:" in why else "scene")
        return ""

    def _swept_free(self, world, q0, q1):
        """(free, reason). reason = 첫 충돌 샘플의 사유(scene/self) 또는 ''."""
        q0, q1 = np.asarray(q0, float), np.asarray(q1, float)
        for k in range(NBV_SWEPT_STEPS + 1):
            r = self._pose_collision_reason(world, q0 + (k / NBV_SWEPT_STEPS) * (q1 - q0))
            if r:
                return False, r
        return True, ""

    # ── 누적 ─────────────────────────────────────────────────────────────────
    def _accumulate(self, pts_w, theta):
        """world 점군을 턴테이블 축(world) 둘레 −θ 역회전 → canonical(world) 누적."""
        if len(pts_w) == 0:
            return
        canon = _rot_about_axis(pts_w, self.axis_w, self.axis_dir_w, -theta)
        self.accum.append(canon)

    def _merged_pcd(self):
        import open3d as o3d
        pts = np.vstack(self.accum) if self.accum else np.zeros((0, 3))
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pts)
        return pcd, pts

    # ── 자세 (해석 IK + set_servo_angle 구동) ────────────────────────────────
    def _view_q(self, target_w, el_deg, az_deg, standoff, seed):
        """target(**world**) 을 el/az/standoff(up=world+z) 에서 보는 카메라 → base → 해석 IK q.
        (q, eye_w) or (None, eye_w). base z 가 world up 과 달라도 정상."""
        target_w = np.asarray(target_w, float)
        el, az = math.radians(el_deg), math.radians(az_deg)
        eye_w = target_w + standoff * np.array(
            [math.cos(el)*math.cos(az), math.cos(el)*math.sin(az), math.sin(el)])
        # ★ look_at_camera = **USD 규약(-Z 광축, +Y up)** → sim 카메라(USD)와 일치(하니스 동일).
        #   (compute_camera_pose_from_normal 은 OpenCV(+Z) 라 flip 필요했으나 규약혼동·꼬임 유발 → 폐기.)
        T_WC = _make_T(_look_at(eye_w, target_w, (0.0, 0.0, 1.0)), eye_w)
        T_CB = np.linalg.inv(self.T_WB) @ T_WC         # → base
        T_EB = T_CB @ self.T_EC
        pose6d = np.concatenate([T_EB[:3, 3]*1000.0, kin.R_to_euler_xyz(T_EB[:3, :3])])
        q, ok = kin.ik(pose6d, seed=seed)
        return (q, eye_w) if ok else (None, eye_w)

    def _drive(self, q, steps=DRIVE_STEPS):
        """현재→q 를 관절 보간으로 **부드럽게** 이동(스텝마다 렌더). 텔레포트 점프 방지 +
        swept 로 충돌검사한 바로 그 직선 경로를 실제로 traverse(검사=실행 일치)."""
        q1 = np.asarray(q, float)
        q0 = np.asarray(self.robot.get_joint_angles(is_radian=True), float)
        n = max(1, int(steps))
        for k in range(1, n + 1):
            qk = q0 + (k / n) * (q1 - q0)
            self.robot.arm.set_servo_angle(angle=qk.tolist(), is_radian=True, wait=True)
            self.world.step(1, render=True)
        self.world.step(4, render=True)             # 도착 후 정착

    def _scan_pass(self, q, n_theta, label=""):
        """★ 통합 캡처 = 로봇을 q 자세로 두고 **턴테이블 전회전**하며 프레임 캡처·−θ 누적.
        real 캡처(로봇 pose + streaming 전회전 + relocalization)에 1:1 대응. Phase1·2 공용.
        sim 은 GT θ 라 −θ 회전 = relocalization 역할(정확). 입사각 필터로 좋은 프레임만 기여."""
        self._drive(q)
        if label:
            print(f"[isaac_scan] === scan pass: {label} (전회전 {n_theta}프레임) ===")
        before = sum(len(a) for a in self.accum)
        thetas = np.linspace(0.0, 2*np.pi, n_theta, endpoint=False)
        for i, th in enumerate(thetas):
            self.turntable.move_abs(float(th), float(np.radians(30.0)))
            self.turntable.wait_motion_done()
            th_act = float(self.turntable.getActualPos())
            obj = self._capture_obj_world(log=(i % 9 == 0))
            self._accumulate(obj, th_act)
        self.turntable.move_abs(0.0, float(np.radians(30.0)))
        self.turntable.wait_motion_done()
        added = sum(len(a) for a in self.accum) - before
        print(f"[isaac_scan]   pass 완료 (+{added}점, 총 {before+added})")

    def _phase1(self):
        # 작업영역 근처 known-good 자세(artec home)로 먼저 이동 → IK seed 안정.
        # (default 자세는 IK seed 로 부적합 — 측면뷰로 수렴 실패 가능.)
        try:
            self.robot.go_home(sensor="artec", confirm=False)
            self.world.step(6, render=True)
        except Exception as e:
            print(f"[isaac_scan] ⚠ go_home 실패({e}) — 현재자세로 진행")
        seed = np.asarray(self.robot.get_joint_angles(is_radian=True), float)
        self.home_q = seed.copy()                    # retract-approach 경유점(known-good home)
        chosen = None
        # 측면 standoff: 표면(중심에서 obj_radius)이 WORK_FOCUS 에 오도록.
        standoff = WORK_FOCUS + self.obj_radius
        # ★ 타깃 = **턴테이블 축**(객체중심 아님). 객체가 축에서 벗어나도 회전 중 시야·클립 안 유지.
        view_target = np.array([self.axis_w[0], self.axis_w[1], self.obj_center_w[2]])
        for azd in VIEW_AZIS_DEG:                    # 도달 가능한 측면 azimuth 채택
            q, _ = self._view_q(view_target, VIEW_EL_DEG, azd, standoff, seed)
            if q is not None:
                chosen = q
                print(f"[isaac_scan] Phase1 측면자세 az={azd:.0f}° el={VIEW_EL_DEG:.0f}° IK ok")
                break
        if chosen is None:
            print("[isaac_scan] ⚠ Phase1 자세 IK 전부 실패 — home 자세 유지")
            chosen = seed
        self._drive(chosen)
        # ★ 카메라 pointcloud annotator warm-up — replicator 가 첫 몇 프레임은 빈 점군을 줌.
        #   (하니스는 SETTLE 120스텝을 돌려 자연히 프라임됨. production 은 명시적 warm-up 필요.)
        self.world.ensure_camera()
        self.world.step(30, render=True)
        _ = self.scanner.capture_points_base(self.robot, self.mms._T_EC, settle=4)
        # Phase 1 = 측면 자세에서 **턴테이블 전회전 스캔**(real streaming 대응).
        self._scan_pass(chosen, N_THETA, label=f"Phase 1 측면 el={VIEW_EL_DEG:.0f}°")

    # ── Phase 2 (NBV = 추가 관측 elevation 자세, real 전회전 대응) ──────────────
    def _plan_nbv_pose(self, world, q_cur, gaps):
        """**공용 `phase2_nbv.plan_nbv_elevation_pose` 호출**(real 과 동일 알고리즘).
        sim 은 USD look-at·world 프레임만 주입(pose_q_fn). 반환 = q(그 자세에서 _scan_pass 전회전)."""
        def _unit(v):
            v = np.asarray(v, float); return v / (np.linalg.norm(v) + 1e-12)
        n_top = sum(1 for c in gaps if abs(_unit(c.n_O)[2]) > 0.6)
        print(f"[isaac_scan]   gap유형: 윗면(top)={n_top} 측면/뒷면(side)={len(gaps)-n_top}")
        look_target = np.array([self.axis_w[0], self.axis_w[1], self.obj_center_w[2]])
        standoff = WORK_FOCUS + self.obj_radius

        def pose_q(el, az):
            q, _ = self._view_q(look_target, el, az, standoff, q_cur)   # USD 규약(sim 카메라)
            return q

        def swept(q0, q1):
            return self._swept_free(world, q0, q1)[0]

        res = p2.plan_nbv_elevation_pose(
            gaps, q_cur, pose_q, swept, joint_weights=DEFAULT_JOINT_WEIGHTS,
            el_floor_deg=VIEW_EL_DEG, view_azis_deg=tuple(VIEW_AZIS_DEG))
        if res is None:
            print("[isaac_scan] NBV: feasible 관측자세 없음 — 윗면 도달한계(스캐너-link2). z수축 필요.")
            return None
        q, el, az = res
        print(f"[isaac_scan] NBV 관측자세 el={el:.0f}° az={az:.0f}° — 전회전 스캔")
        return q

    def _phase2(self, world):
        print("[isaac_scan] === Phase 2 (부족면 보강 = 추가 elevation 전회전 스캔) ===")
        for k in range(NBV_K_MAX):
            pcd, _ = self._merged_pcd()
            if len(pcd.points) < 200:
                break
            mesh = p2.pcd_to_mesh_poisson(pcd, depth=8, density_quantile=0.04)
            cov = p2.coverage_state(mesh, **GAP_KW)
            print(f"  [{k+1}/{NBV_K_MAX}] boundary={cov.boundary_len_m*1000:.0f}mm "
                  f"cov={cov.angular_cov:.2f} gaps={cov.n_gaps}")
            if p2.is_converged(cov, 0.012, 0.92):
                print("  수렴 — 완료."); break
            gaps = p2.detect_gaps(mesh, **GAP_KW)
            q_cur = np.asarray(self.robot.get_joint_angles(is_radian=True), float)
            q = self._plan_nbv_pose(world, q_cur, gaps)
            if q is None:
                break
            self._scan_pass(q, N_THETA_P2, label=f"Phase 2 보강 #{k+1}")

    # ── run ───────────────────────────────────────────────────────────────────
    def run(self) -> IsaacScanResult:
        # phase_mode 순차 누적 (sim): 1=Phase1만, 2+=Phase1→2(NBV).
        # Phase 3(바닥면 flip)은 사용자 손회전 필요 → real 전용(sim 미지원).
        phase_mode = int(os.environ.get("MMS_SIM_PHASE_MODE", "2"))
        self._setup_object()
        self._phase1()
        if phase_mode >= 2:
            world = self._build_world()
            self._phase2(world)
        else:
            print("[isaac_scan] phase_mode=1 — Phase 1(5면)만 (NBV skip)")
        pcd, pts = self._merged_pcd()
        mesh = None
        try:
            mesh = p2.pcd_to_mesh_poisson(pcd, depth=8, density_quantile=0.04)
        except Exception as e:
            print(f"[isaac_scan] 최종 mesh 실패({e})")
        n = int(len(pts))
        print(f"[isaac_scan] 완료 — 누적 {n}점, mesh={'O' if mesh and len(mesh.triangles) else 'X'}")
        return IsaacScanResult(model=_SimModel(pts, mesh), n_frames=n)


# ── 모듈 함수(자기완결) ─────────────────────────────────────────────────────────
def _voxel(pts, v):
    pts = np.asarray(pts, float)
    if len(pts) == 0:
        return pts
    keys = np.floor(pts / v).astype(np.int64)
    _, idx = np.unique(keys, axis=0, return_index=True)
    return pts[idx]


def _pca_normals(pts, c_ref, k=12):
    N = len(pts)
    if N < 4:
        return None
    k = min(k, N)
    out = np.empty_like(pts)
    for i in range(N):
        d = pts - pts[i]
        idx = np.argpartition(np.einsum('ij,ij->i', d, d), k - 1)[:k]
        nb = pts[idx] - pts[idx].mean(0)
        _, v = np.linalg.eigh(nb.T @ nb)
        n = v[:, 0]
        if n @ (pts[i] - c_ref) < 0:
            n = -n
        out[i] = n
    return out


def _rot_about_axis(pts, axis_pt, axis_dir, ang):
    a = np.asarray(axis_dir, float); a = a / (np.linalg.norm(a) + 1e-12)
    c, s = math.cos(ang), math.sin(ang)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    R = np.eye(3) * c + np.outer(a, a) * (1 - c) + K * s
    return (np.asarray(pts) - axis_pt) @ R.T + axis_pt
