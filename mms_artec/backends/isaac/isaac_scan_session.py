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
from utils.nbv import phase1_viewpoint as p1
from utils.nbv.scan_phase_controller import (
    run_scan_phases, resolve_phase_mode)
from utils.nbv.nbv_planner import NbvPlanner as _NbvPlanner
from utils.collision.robot_collision import (
    CollisionWorld, pose_collision, DEFAULT_LINK_RADII, capsules_from_joints)
from utils.collision import mesh_self_collision as _mesh_sc
from utils.collision import env_collision as _env_col
from utils.robot import view_pose as _vp
from mms_artec.utils.calibration import handeye_error as _he
from utils.robot import xarm7_kinematics as kin
from utils.control.theta_planner import DEFAULT_JOINT_WEIGHTS
# look-at = **USD 규약(-Z 광축)** — sim 카메라(USD)와 일치(하니스와 동일). 자기완결(numpy).
from mms_artec.utils.calibration.handeye_geometry import (
    look_at_camera as _look_at, make_T as _make_T)


# ── 설정 (env 로 override 가능) ─────────────────────────────────────────────────
def _envf(k, d): return float(os.environ.get(k, d))
def _envs(k, d): return os.environ.get(k, d)

OBJECT_SOURCE   = _envs("MMS_SIM_OBJECT", "usd")     # "usd"(ScanTarget) | "spawn"(테스트 형상)
# ⚠ 프림 경로는 isaac_world 가 단일 진실 — 여기서 다시 정의하지 않는다.
#   (v3_scene 전환 때 여기 사본이 v2 경로로 남아 TURNTABLE_MESH 가 null prim 이었다)
from mms_artec.backends.isaac.isaac_world import (
    OBJECT_PRIM as _WORLD_OBJECT_PRIM, DISC_PRIM as _WORLD_DISC_PRIM,
    FRAME_PRIM as _WORLD_FRAME_PRIM, CAMERA_PRIM as _WORLD_CAMERA_PRIM)

OBJECT_PRIM     = _envs("MMS_SIM_OBJECT_PRIM",        # OBJECT_SOURCE="usd" 스캔 대상
                        _WORLD_OBJECT_PRIM)
SPAWN_SHAPE     = _envs("MMS_SIM_SHAPE", "box")      # spawn: box|cylinder|sphere|cone|lshape|stepped
SPAWN_PRIM      = "/World/SimScanObject"

# Spider working distance(0.2~0.3) 중앙. standoff 는 **표면이 이 거리에 오도록** 동적 계산
# (카메라 클립=실 스펙 0.2~0.3 유지 → 표면이 0.25 면 근/원접클립 모두 안전).
WORK_FOCUS      = 0.25
DRIVE_STEPS     = int(_envf("MMS_SIM_DRIVE_STEPS", 30))   # 관절 보간 스텝(클수록 부드럽고 느림)
N_THETA         = int(_envf("MMS_SIM_NTHETA", 36))     # Phase1/3 전회전 프레임수(낮추면 빠름)
N_THETA_P2      = int(_envf("MMS_SIM_NTHETA_P2", 24))   # Phase2 보강 전회전 프레임수(빠르게)
CAPTURE_SETTLE  = int(_envf("MMS_SIM_CAPTURE_SETTLE", 2))  # 캡처 전 정지 렌더 step(정지상태라 2면 충분)
# Phase1 측면 관측 elevation.
# ★ 정정(2026-08-13) — 한때 "v3 레이아웃은 el≤30 도달 불가" 로 판단해 50 으로 올렸으나,
#   **하드웨어 한계가 아니라 `_view_q` 가 광축(roll)을 하나로 고정한 탓**이었다.
#   roll 을 풀면 같은 위치에서 el30 5/8·el40 7/8 az 가 열리고 IK 실패는 0 건이다.
#   (시드만 12개로 늘린 경우는 여전히 0/8 → 원인은 시드가 아니라 roll 이다.)
#   낮은 el 이 없으면 측면·하부가 안 찍혀 gap 이 안 줄므로 원래 값으로 되돌린다.
#   측정: scripts/sim/eval_turntable_layout.py --diag 0.365 --rolls 0   (대조군)
#         scripts/sim/eval_turntable_layout.py --diag 0.365             (roll 자유)
VIEW_EL_DEG     = _envf("MMS_SIM_VIEW_EL", 30.0)
# Phase1 플래너가 고를 수 있는 elevation 후보(p1.DEFAULT_ELS=(20,30,40,50)).
# el 20 은 실측에서도 도달 자세가 없어 제외한다.
P1_ELS = tuple(float(x) for x in
               os.environ.get("MMS_SIM_P1_ELS", "30,40,50,60,70").split(","))
VIEW_AZIS_DEG   = [0.0, 30.0, -30.0, 60.0, -60.0, 90.0, -90.0, 180.0]
# 광축(roll) 둘레 회전 후보 — **크기 순**. 0 을 먼저 쓰므로 기존에 풀리던 자세의 해는
# 그대로고, IK 가 실패할 때만 최소 각도부터 넓힌다. `_view_q` 주석 참고.
#   실측(v3, X=0.365): roll 고정이면 시드를 12개로 늘려도 el30·40 이 0/8.
#                      roll 을 풀면 el30 5/8, el40 7/8, IK 실패 0건.
#   ⚠ 실물은 스캐너 자체 tracking(master relocalization)에 의존한다. 패스 **내부**는
#     roll 이 고정이라 무관하지만, 패스 사이 자세 점프의 한 성분으로 들어간다.
#     실물에서 tracking-lost 가 늘면 MMS_SIM_VIEW_ROLLS=0 으로 되돌릴 것.
VIEW_ROLLS_DEG = tuple(float(x) for x in
                       os.environ.get("MMS_SIM_VIEW_ROLLS",
                                      "0,-45,45,-90,90,180").split(","))
IK_SEED_TRIES   = int(os.environ.get("MMS_SIM_IK_SEEDS", "7"))
# Phase2 에서 roll 을 gap 방향에 맞춰 고를지(1) 기본 순서(roll=0 우선)를 쓸지(0).
# **대조군 스위치** — 효과를 재려면 이것만 끄고 같은 조건으로 비교한다.
NBV_ROLL_ALIGN  = os.environ.get("MMS_SIM_NBV_ROLL_ALIGN", "1") == "1"
# 자가충돌 허용 최소 여유(m) — 실제 메시 최소거리 기준. 캡슐 시절의 암묵 임계보다
# 훨씬 작아 보이지만, 캡슐은 형상을 과대하게 덮어 임계가 부풀려져 있었을 뿐이다.
SELF_CLEAR_M    = _envf("MMS_SIM_SELF_CLEAR", 0.02)
# ── 패스 간 정합 (real 과 같은 구조) ─────────────────────────────────────────
# real 은 새 패스를 master 에 **open3d ICP** 로 붙인다
# (`artec_multipass_scan_session._hint_icp_refine`, colored ICP + point-to-plane fallback,
#  init = 로봇 기구학). sim 은 GT θ 로 직접 누적만 해서 이 단계가 **없었다**.
# → hand-eye 오차를 주입하면 sim 은 흡수 수단이 없어 실물보다 훨씬 심하게 무너진다
#   (실측: 3mm/0.5° 에서 boundary 967→2178mm, gap 18→72). 즉 지금의 sim 은 실물
#   강건성을 검증할 수 없다. 같은 정합 단계를 sim 에도 둔다.
ICP_PASS_ENABLE = _envs("MMS_SIM_ICP_PASS", "1") == "1"
# coarse→fine 다단. 초기 어긋남이 max_corr 보다 크면 ICP 가 대응점을 못 찾으므로
# 처음엔 넉넉하게 잡고 좁혀 간다.
ICP_SCALES_M    = (0.020, 0.008, 0.004)
ICP_MIN_PTS     = 300
# 셀 구조물(벽·상판·저울·툴스탠드) 최소 여유(m). 기존 장애물은 턴테이블·프레임뿐이라
# **칠 수 있는데 안전하다고 판정**하고 있었다(미탐). docs/4_collision.md P1.
ENV_CLEAR_M     = _envf("MMS_SIM_ENV_CLEAR", 0.025)
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
SCANNER_PRIM = _WORLD_CAMERA_PRIM.rsplit("/", 1)[0]      # Camera 의 부모 = 스캐너 프레임
LINK7_PRIM   = "/World/xarm7/link7"
TURNTABLE_MESH = _WORLD_DISC_PRIM      # sim 턴테이블 원판(축 산출)
FRAME_PRIM     = _WORLD_FRAME_PRIM     # 모터 프레임(충돌)
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


# ── 세션 ─────────────────────────────────────────────────────────────────────
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
        # ★ T_EC_true = 실제 카메라 위치(USD GT).  T_EC = 파이프라인이 **믿는** 값.
        #   기본은 둘이 같다. MMS_SIM_TEC_ERR_MM/DEG 로 캘리브 오차를 주입하면 갈라진다
        #   — 실물은 캘리브 오차를 안고 계획·재구성하므로, 그 조건을 sim 에서 재현한다.
        self.T_EC_true = self._T_EC_gt()            # E→C (sim GT)
        self.T_EC = _he.believed_T_EC(self.T_EC_true)
        self._seed_alts = None                      # IK 대안 시드 (_ik_seeds 지연생성)
        # Phase2 자세 선택기(공용). visited 를 세션 동안 들고 있어 같은 자세를
        # 반복 선택하지 않는다 — az 를 '관절이동 최소'로 고르므로 직전 자세의
        # 이동비용이 0 이라 넘기지 않으면 무한 반복한다.
        self._nbv = _NbvPlanner(joint_weights=DEFAULT_JOINT_WEIGHTS,
                                el_floor_deg=VIEW_EL_DEG,
                                view_azis_deg=VIEW_AZIS_DEG,
                                log=lambda m: print(f"[isaac_scan] {m}"))
        self._last_roll = None                      # _view_q 가 채택한 roll(로그용)
        # 자가충돌 = 실제 메시 판정(캐시 없으면 None → 캡슐 폴백)
        self._mesh_self = _mesh_sc.get_default(margin_m=SELF_CLEAR_M)
        if self._mesh_self is not None:
            print(f"[isaac_scan] 자가충돌 = 실제 메시 판정 (여유 {SELF_CLEAR_M*1000:.0f}mm)")
        # 셀 구조물(벽·상판·저울·툴스탠드) — 기존 world 는 턴테이블/프레임만 있었다.
        self._env_mesh = _env_col.get_default(margin_m=ENV_CLEAR_M)
        if self._env_mesh is not None:
            print(f"[isaac_scan] 셀 구조물 충돌 = 실제 메시 {len(self._env_mesh.env)}점 "
                  f"(여유 {ENV_CLEAR_M*1000:.0f}mm)")
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
            _o = self.stage.GetPrimAtPath(prim)
            if _o.IsValid():
                UsdGeom.Imageable(_o).MakeVisible()
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
        mn, mx = self._aabb_world(TURNTABLE_MESH)
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
        # USD 스캔 대상 숨김(spawn 형상만 스캔)
        mb = self.stage.GetPrimAtPath(OBJECT_PRIM)
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
        pc_b = self.scanner.capture_points_base(self.robot, self.mms._T_EC, settle=CAPTURE_SETTLE)
        # sim 카메라는 GT 포즈로 점군을 준다. 실물은 카메라 프레임 점군을 **캘리브값**으로
        # base 에 올리므로 오차가 점군에 실린다 — 주입이 켜져 있으면 그 경로를 재현한다.
        if pc_b is not None and len(pc_b) and self.T_EC is not self.T_EC_true:
            q_now = np.asarray(self.robot.get_joint_angles(is_radian=True), float)
            pc_b = _he.apply_to_points(pc_b, q_now, self.T_EC_true, self.T_EC, kin)
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
        """''=충돌없음. 아니면 사유(scene/self). self = 스캐너 포함 자가충돌.

        ★ 자가충돌은 **실제 메시**로 판정한다(캐시 있을 때). 캡슐 근사는 이 팔에서
          오탐이 심해, NBV 후보 32자세를 IK 로 전부 풀고도 전부 기각시켰다
          (2026-08-13: `IK실패 0, swept충돌 32, self×32`). 근거·수치는
          `utils/collision/mesh_self_collision` 및 docs/hw_layout.md §5.
          장애물(scene) 충돌은 캡슐 그대로 — 오탐 근거가 확인된 쪽은 self 뿐이다.
        """
        c1, why = pose_collision(world, q, T_EC=self.T_EC, link_radii=LINK_RADII)
        is_self = bool(c1) and "self:" in why
        if c1 and not is_self:
            return "scene"
        if self._env_mesh is not None:
            hit, who = self._env_mesh.collides(q)
            if hit:
                return f"scene({who})"
        if self._mesh_self is not None:
            hit, _who = self._mesh_self.collides(q)
            return "self" if hit else ""
        return "self" if is_self else ""

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

    def _merge_pass(self, pass_pts):
        """한 패스의 canonical 점군을 **master 에 ICP 로 붙여** 누적한다.

        real 과 같은 구조다 — real 은 새 IScan 을 master 에 colored ICP 로 붙인다
        (`_hint_icp_refine`). sim 은 이 단계가 없어 hand-eye 오차를 흡수할 수단이
        전혀 없었다(실측: 3mm 오차에서 boundary 2.4배, gap 4배).

        첫 패스는 기준이므로 그대로 넣는다. 이후 패스는 초기값 = identity
        (로봇 기구학이 이미 맞춰 놨으므로) → coarse→fine ICP → 게이트 통과 시에만 적용.
        게이트는 `utils/nbv/icp_strategy.icp_with_gates` (RMSE/fitness/drift 3중).
        """
        if not pass_pts:
            return
        pts = np.vstack(pass_pts)
        if not self.accum or not ICP_PASS_ENABLE or len(pts) < ICP_MIN_PTS:
            self.accum.append(pts)
            return
        master = np.vstack(self.accum)
        if len(master) < ICP_MIN_PTS:
            self.accum.append(pts)
            return
        try:
            import open3d as o3d
            from utils.nbv.icp_strategy import icp_with_gates
            src = o3d.geometry.PointCloud()
            src.points = o3d.utility.Vector3dVector(_voxel(pts, VOXEL_M))
            tgt = o3d.geometry.PointCloud()
            tgt.points = o3d.utility.Vector3dVector(_voxel(master, VOXEL_M))
            T = np.eye(4)
            res = None
            for corr in ICP_SCALES_M:       # coarse→fine
                res = icp_with_gates(src, tgt, T, max_correspondence_distance=corr,
                                     rmse_thresh=corr / 2.0, fitness_thresh=0.20,
                                     drift_trans_m=0.030, drift_rot_deg=10.0)
                T = res.T_refined
            if res is not None and res.ok:
                pts = pts @ T[:3, :3].T + T[:3, 3]
                print(f"[isaac_scan]   ICP 정합 적용: "
                      f"Δt={res.delta_translation_m*1000:.2f}mm "
                      f"Δr={res.delta_rotation_deg:.2f}° "
                      f"fitness={res.fitness:.2f} rmse={res.rmse*1000:.2f}mm")
            else:
                why = res.reason if res is not None else "결과 없음"
                print(f"[isaac_scan]   ⚠ ICP 게이트 실패({why}) — 기구학 그대로 사용")
        except Exception as e:                       # noqa: BLE001
            print(f"[isaac_scan]   ⚠ ICP 예외({type(e).__name__}: {e}) — 기구학 그대로")
        self.accum.append(pts)

    def _merged_pcd(self):
        import open3d as o3d
        pts = np.vstack(self.accum) if self.accum else np.zeros((0, 3))
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pts)
        return pcd, pts

    # ── 자세 (해석 IK + set_servo_angle 구동) ────────────────────────────────
    def _view_q(self, target_w, el_deg, az_deg, standoff, seed, rolls=None):
        """target(**world**) 을 el/az/standoff 에서 보는 카메라 → base → 해석 IK q.
        (q, eye_w) or (None, eye_w).

        ★ 실제 계산은 **sim·real 공용** `utils/robot/view_pose.solve_view_q` 가 한다.
          예전에는 이 함수(sim)만 roll 6방향·시드 8개를 쓰고 real 은 각각 1개였다 —
          같은 solver 를 쓰면서도 sim 에서 되는 자세가 real 에서 버려졌다.
          카메라 규약은 데이터로 넘긴다(sim=USD, real=OpenCV). 근거: docs/4_collision.md §2.
        """
        q, roll, eye_w = _vp.solve_view_q(
            kin, target_w, el_deg, az_deg, standoff, seed, self.T_EC,
            T_WB=self.T_WB, convention=_vp.CAM_USD,
            rolls_deg=(VIEW_ROLLS_DEG if rolls is None else rolls),
            n_seed_alt=IK_SEED_TRIES)
        self._last_roll = roll
        return q, eye_w

    def _gap_roll_order(self, target_w, el_deg, az_deg, standoff, gaps):
        """gap 방향에 FOV 넓은 축을 맞추는 roll 순서 (공용 view_pose 위임).
        `NBV_ROLL_ALIGN=0` 이면 기본 순서(roll=0 우선) — **대조군 스위치**."""
        if not NBV_ROLL_ALIGN or not gaps:
            return tuple(VIEW_ROLLS_DEG)
        eye = _vp.eye_from_el_az(target_w, el_deg, az_deg, standoff)
        return _vp.roll_order_for_gaps(
            [c.p_O for c in gaps], [c.L for c in gaps], eye, target_w,
            convention=_vp.CAM_USD, rolls_deg=VIEW_ROLLS_DEG)

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
        pass_pts = []                       # 이 패스만 따로 모은다(정합 단위)
        for i, th in enumerate(thetas):
            self.turntable.move_abs(float(th), float(np.radians(30.0)))
            self.turntable.wait_motion_done()
            th_act = float(self.turntable.getActualPos())
            obj = self._capture_obj_world(log=(i % 9 == 0))
            if len(obj):
                pass_pts.append(_rot_about_axis(obj, self.axis_w, self.axis_dir_w, -th_act))
        self._merge_pass(pass_pts)
        # 루프 닫기: 350°→360°(≡0°) 앞으로 10° 만 더 회전 후 θ 리셋.
        # (기존엔 350°를 역방향으로 되감으며 렌더 → 느리고 "홱" 끊김. 360°≡0° 라
        #  clearpos 의 _co_rotate(0) 는 시각 점프 없음.)
        self.turntable.move_abs(float(2 * np.pi), float(np.radians(30.0)))
        self.turntable.clearpos()
        added = sum(len(a) for a in self.accum) - before
        print(f"[isaac_scan]   pass 완료 (+{added}점, 총 {before+added})")

    # ── ScanBackend 프리미티브 (공용 utils/nbv/scan_phase_controller) ──────
    # 순서/게이팅/NBV 루프는 공용 컨트롤러 소유. 여기는 sim 캡처 하드웨어만.
    def confirm_start(self) -> bool:
        return True                                   # sim: 사람 확인 불필요

    def go_home(self) -> None:
        try:
            self.robot.go_home(sensor="artec", confirm=False)
            self.world.step(6, render=True)
        except Exception as e:
            print(f"[isaac_scan] ⚠ go_home 실패({e}) — 현재자세로 진행")

    def pick_phase1_pose(self):
        """Phase 1 시점선정 (E2E, 2026-07-03) — **real 과 동일 경로**:
        거리스텝 preview 캡처(실제 카메라) → 기하 크롭 → maximin 플래너.
        반환 = q 또는 [q,...](밴드 계획, 공용 컨트롤러가 대역별 전회전).
        MMS_SIM_P1_MODE=legacy 면 기존 GT-bbox azimuth sweep 사용."""
        self.go_home()
        seed = np.asarray(self.robot.get_joint_angles(is_radian=True), float)
        self.home_q = seed.copy()                    # retract-approach 경유점(known-good home)
        # 카메라 warm-up (replicator 첫 프레임 빈 점군 방지) — preview 전에 필요.
        self.world.ensure_camera()
        self.world.step(30, render=True)
        _ = self.scanner.capture_points_base(self.robot, self.mms._T_EC, settle=4)

        if _envs("MMS_SIM_P1_MODE", "planner") == "legacy":
            return self._pick_phase1_legacy(seed)

        try:
            plan_qs = self._pick_phase1_planner(seed)
            if plan_qs:
                return plan_qs
            print("[isaac_scan] ⚠ P1 플래너 실패 — legacy sweep 으로 fallback")
        except Exception as e:
            print(f"[isaac_scan] ⚠ P1 플래너 예외({type(e).__name__}: {e}) — legacy fallback")
        return self._pick_phase1_legacy(seed)

    # ── Phase 1 시점선정: real 경로 (preview → 크롭 → 플래너) ────────────
    def _pick_phase1_planner(self, seed):
        """계획용 preview 수집(거리스텝×턴테이블 0/90°×조준높이) → 기하 크롭
        (캘리브 축+디스크상단만 사용, GT bbox 미사용) → plan_phase1_viewpoints
        → 자세별 az-sweep IK. real 은 이 함수의 캡처 호출만 Artec preview 로 바뀜."""
        axis_xy = self.axis_w[:2]
        disc_top = float(self.axis_w[2])
        sensor = p1.SensorModel()
        # el_prev — 구값 25.0 은 roll 고정 탓에 IK 전부 실패했다(→ preview 0점). roll 을
        # 풀어 도달성이 확인된 30 으로. 낮을수록 물체 실루엣·높이가 잘 잡힌다.
        d_steps, el_prev = (0.30, 0.38), _envf("MMS_SIM_PREVIEW_EL", 30.0)

        def preview_at(tz, d):
            """조준높이 tz·축거리 d 로 구동 후 preview 캡처 → world 점군(기하 크롭)."""
            for azd in VIEW_AZIS_DEG:                # 도달 azimuth 스윕 (커버리지 무관)
                q, _ = self._view_q(np.array([axis_xy[0], axis_xy[1], tz]),
                                    el_prev, azd, d, seed)
                if q is None:
                    continue
                self._drive(q)
                pc_b = self.scanner.capture_points_base(
                    self.robot, self.mms._T_EC, settle=CAPTURE_SETTLE)
                if pc_b is None or len(pc_b) == 0:
                    return np.zeros((0, 3))
                # ★ 로봇 자기점 제거 (self-filter) — 프레임에 걸린 링크/스캐너
                #   점이 크롭 실린더를 오염해 밴드 폭주시키는 것 방지 (real 동일).
                q_now = np.asarray(self.robot.get_joint_angles(is_radian=True), float)
                pc_b = p1.filter_robot_points(
                    pc_b, capsules_from_joints(q_now, LINK_RADII, T_EC=self.T_EC))
                return p1.crop_object_points(self._base_to_world(pc_b),
                                             axis_xy, disc_top)
            return np.zeros((0, 3))

        # 실루엣 2방향 = 턴테이블 0°/90° (real 동일: 로봇 대신 물체를 돌림).
        # 90° 점군은 encoder 각(-θ)으로 역회전해 물체 프레임 통일.
        acc = []
        for theta in (0.0, math.pi / 2):
            self.turntable.move_abs(float(theta), float(np.radians(30.0)))
            self.turntable.wait_motion_done()
            tz, prev_top = disc_top + 0.05, -np.inf
            for _ in range(4):                       # 조준높이 상승 루프
                for d in d_steps:                    # 거리 2스텝 = 표면반경 0~18cm 커버
                    obj = preview_at(tz, d)
                    if len(obj):
                        acc.append(_rot_about_axis(obj, self.axis_w,
                                                   self.axis_dir_w, -theta))
                top = max((a[:, 2].max() for a in acc), default=tz)
                if top - prev_top < 0.01:            # 상단이 안 늘면 종료 (GT 불요)
                    break
                prev_top, tz = top, top + 0.03
        self.turntable.move_abs(0.0, float(np.radians(30.0)))
        self.turntable.wait_motion_done()

        pts = np.vstack(acc) if acc else np.zeros((0, 3))
        pts = p1.voxel_downsample(pts, sensor.voxel_m)
        if len(pts) < 100:
            print(f"[isaac_scan] P1 preview 점 부족({len(pts)}) — 플래너 불가")
            return None
        nrm = p1.estimate_outward_normals(pts, axis_xy)
        # 후보 elevation 을 명시 — p1.DEFAULT_ELS=(20,30,40,50) 의 20 은 실측에서도
        # 도달 자세가 없다. P1_ELS 는 env 로 조정 가능.
        plan = p1.plan_phase1_viewpoints(pts, nrm, axis_xy, sensor, els=P1_ELS)
        print(f"[isaac_scan] P1 플랜: {plan.note} risk={plan.tracking_risk} "
              f"(preview {len(pts)}pt)")
        qs = []
        for vp, ev in zip(plan.poses, plan.evals):
            q = None
            for azd in VIEW_AZIS_DEG:                # 계획 자세도 az 는 도달성으로
                q, _ = self._view_q(np.array([axis_xy[0], axis_xy[1], vp.target_z]),
                                    vp.el_deg, azd, vp.standoff, seed)
                if q is not None:
                    print(f"[isaac_scan]   자세 el={vp.el_deg:.0f}° s={vp.standoff:.3f} "
                          f"tz={vp.target_z:.3f} az={azd:.0f}° "
                          f"minfill={ev.min_fill_cm2:.0f}cm² IK ok")
                    break
            if q is not None:
                qs.append(q)
        return qs or None

    # ── Phase 1 시점선정: legacy (GT bbox azimuth sweep, fallback) ────────
    def _pick_phase1_legacy(self, seed):
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
        return chosen

    def capture_rotation(self, pose, label: str, phase: int) -> bool:
        # sim: 로봇을 pose 로 구동 + 턴테이블 전회전 캡처. Phase 2 는 프레임 수 축소.
        n_theta = N_THETA_P2 if phase == 2 else N_THETA
        self._scan_pass(pose, n_theta, label=label)
        return True

    # ── Phase 2 (NBV = 추가 관측 elevation 자세, real 전회전 대응) ──────────────
    def _plan_nbv_pose(self, world, q_cur, gaps):
        """Phase 2 관측자세 — 판단은 **sim·real 공용** `utils/nbv/nbv_planner` 가 한다.

        여기서 주입하는 것은 sim 고유의 것 두 가지뿐이다:
          · 자세 생성 = USD 규약(-Z 광축) look-at + world 프레임 타깃
          · 충돌 판정 = 캡슐 world + 실제 메시(자가/셀 구조물)
        gap 분류·아랫면 제외·visited·roll 정렬·실패 진단은 공용 코드가 담당한다.
        """
        look_target = np.array([self.axis_w[0], self.axis_w[1], self.obj_center_w[2]])
        standoff = WORK_FOCUS + self.obj_radius

        def solve_pose(el, az, rolls):
            q, _ = self._view_q(look_target, el, az, standoff, q_cur, rolls=rolls)
            return q, self._last_roll

        def roll_order(el, az, gs):
            return self._gap_roll_order(look_target, el, az, standoff, gs)

        res = self._nbv.plan(gaps, q_cur, solve_pose,
                             lambda a, b: self._swept_free(world, a, b),
                             roll_order_fn=roll_order)
        return None if res is None else res[0]

    # ── Phase 2 (NBV hole-fill) 프리미티브 — 수렴 루프는 공용 컨트롤러 소유 ──
    def build_coverage_mesh(self):
        pcd, _ = self._merged_pcd()
        if len(pcd.points) < 200:
            print("[isaac_scan] Phase2 누적점 부족(<200) — 종료.")
            return None
        return p2.pcd_to_mesh_poisson(pcd, depth=8, density_quantile=0.04)

    def is_converged(self, mesh) -> bool:
        cov = p2.coverage_state(mesh, **GAP_KW)
        print(f"[isaac_scan]   boundary={cov.boundary_len_m*1000:.0f}mm "
              f"cov={cov.angular_cov:.2f} gaps={cov.n_gaps}")
        if p2.is_converged(cov, 0.012, 0.92):
            print("[isaac_scan]   수렴 — 완료.")
            return True
        return False

    def plan_nbv_pose(self, mesh):
        if self._world is None:
            self._world = self._build_world()
        gaps = p2.detect_gaps(mesh, **GAP_KW)
        q_cur = np.asarray(self.robot.get_joint_angles(is_radian=True), float)
        return self._plan_nbv_pose(self._world, q_cur, gaps)

    def supports_phase3(self) -> bool:
        return False                                  # sim: 물체 손회전 불가 → Phase 3 미지원

    def next_flip(self) -> bool:
        return False

    def finalize(self) -> IsaacScanResult:
        pcd, pts = self._merged_pcd()
        mesh = None
        try:
            mesh = p2.pcd_to_mesh_poisson(pcd, depth=8, density_quantile=0.04)
        except Exception as e:
            print(f"[isaac_scan] 최종 mesh 실패({e})")
        n = int(len(pts))
        print(f"[isaac_scan] 완료 — 누적 {n}점, mesh={'O' if mesh and len(mesh.triangles) else 'X'}")
        return IsaacScanResult(model=_SimModel(pts, mesh), n_frames=n)

    # ── run — 공용 Phase 컨트롤러에 위임 ────────────────────────────────────
    def run(self) -> IsaacScanResult:
        # phase_mode 순차 누적 (sim): 1=Phase1만, 2+=Phase1→2(NBV).
        # Phase 3(바닥면 flip)은 사용자 손회전 필요 → supports_phase3=False (sim 미지원).
        #
        # real 과 **같은 설정**(ArtecProcessSettings.multipass_settings.phase_mode)을 따른다.
        # 이전에는 sim 만 환경변수를 봐서, main_artec.py 의 phase_mode 를 바꿔도 sim 은
        # 반응하지 않았다. 환경변수는 스윕 스크립트(scripts/sim/*.sh)용 override 로 남긴다.
        # 해석은 공용 `resolve_phase_mode` 한 곳에서만 — main_artec.py 의 표시와
        # 여기의 실제 동작이 **같은 함수**를 쓰므로 어긋날 수 없다.
        self.phase_mode, src = resolve_phase_mode(
            getattr(self.s, "multipass_settings", None), allow_env=True)
        print(f"[isaac_scan] phase_mode={self.phase_mode} ({src})")
        self.nbv_k_max = NBV_K_MAX
        self._world = None                            # Phase 2 진입 시 lazy build
        self._setup_object()
        return run_scan_phases(self)


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
