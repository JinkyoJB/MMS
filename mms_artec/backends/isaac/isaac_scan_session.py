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
    run_scan_phases, resolve_phase_mode, AT_CURRENT)
from utils.nbv.nbv_planner import NbvPlanner as _NbvPlanner
from utils.collision.robot_collision import (
    CollisionWorld, pose_collision, DEFAULT_LINK_RADII, capsules_from_joints)
from utils.collision import mesh_self_collision as _mesh_sc
from utils.collision import env_collision as _env_col
from utils.collision import collision_model as _colmodel
from utils.control.joint_path_planner import plan_joint_path as _plan_path
from utils.robot import view_pose as _vp
from mms_artec.backends.isaac import isaac_debug_viz as _viz
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
# 관절 보간 스텝 — 클수록 부드럽고 느리다. 스텝마다 world.step(render) 하므로
# **스텝 수가 곧 이동 시간**이다. 30→15 로 속도 2배(2026-08-19 요청).
# 충돌 검사는 스텝 수와 무관하다(보수적 전진이 별도로 경로를 보증).
DRIVE_STEPS     = int(_envf("MMS_SIM_DRIVE_STEPS", 15))
# ★ 프레임 밀도를 **실물과 맞춘다**. 실물 streaming 은 30초/회전 × 스캐너 max_fps(≈8)
#   ≈ 240프레임/회전이다. sim 은 36프레임이라 **20배 성겼고**, 그 결과
#     · Poisson 표면이 조각나 gap 이 폭증하고(실측 20 → 418)
#     · 프레임/패치 간 중첩이 얕아 ICP 정합이 계속 게이트에 걸렸다(RMSE 2.3~2.5mm)
#   각도 구간에 비례해 프레임을 배분하므로 부분 스윕도 같은 밀도를 유지한다.
SCAN_FRAMES_PER_REV = int(_envf("MMS_SIM_FRAMES_PER_REV", 240))   # 실물: 30s × 8fps
# Phase 2 도 **실물 밀도 유지**. 입력을 줄이는 대신 **연산을 가볍게** 해서 소화한다
# (사용자 방침 2026-08-19). 성긴 입력은 메시 조각남·정합 실패의 원인이었다.
NBV_FRAMES_PER_REV = int(_envf("MMS_SIM_NBV_FRAMES_PER_REV", SCAN_FRAMES_PER_REV))
N_THETA         = int(_envf("MMS_SIM_NTHETA", SCAN_FRAMES_PER_REV))
N_THETA_P2      = int(_envf("MMS_SIM_NTHETA_P2", NBV_FRAMES_PER_REV))
# 캡처 전 정지 렌더 step. 턴테이블 move_abs 가 이미 렌더하며 이동하므로 1 이면 충분하다.
# (2 였을 때 프레임당 렌더가 3회 → 240프레임 전회전에서 720회. 실물 밀도로 올린 뒤
#  이게 Phase 2 지연의 큰 몫이었다.)
CAPTURE_SETTLE  = int(_envf("MMS_SIM_CAPTURE_SETTLE", 1))
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
# 로봇이 만들 방위각(az). **턴테이블이 az 를 담당**하므로 로봇은 안전한 좁은 대역만 쓴다.
# 실측(el=35/55 × az 8방향, 물체중심 겨냥):
#     az    관절이동   자가여유   판정
#      0°     58°       52mm     OK      ← 최적
#    -30°    362°       12mm     충돌
#    -60°    409°        3mm     충돌
#    -90°    445°        1mm     충돌
# az 를 벌릴수록 팔이 물체를 크게 돌아 감싸 이동량 8배·여유 1mm 까지 떨어진다
# (16개 조합 중 9개가 충돌 기각). 방위 커버리지는 전회전이 이미 제공하므로
# 로봇이 az 를 만들 이유가 없다 — 턴테이블이 공짜로 해줄 일을 위험하게 하는 셈.
VIEW_AZIS_DEG = [float(x) for x in
                 _envs("MMS_SIM_VIEW_AZIS", "0,30,-30").split(",") if x.strip()]
# 오목 물체(컵 등) 내부를 보려면 높은 고도각이 필요하다. 그런데 **미관측 내벽은 메시에
# 없어 gap 으로 잡히지 않아** el_need 가 올라갈 근거가 없다(닭·달걀). 그래서 gap 과
# 무관하게 이 고도각들을 **최소 한 번** 시도하도록 보장한다.
# 실측: el=55° az=0° standoff 0.25 한 자세 + 전회전으로 컵 내벽·내부바닥 100% 커버.
ENSURE_ELS = tuple(float(x) for x in
                   _envs("MMS_SIM_ENSURE_ELS", "55").split(",") if x.strip())
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
# 셀 구조물(벽·상판·저울·툴스탠드) 최소 여유(m). 기존 장애물은 턴테이블·프레임뿐이라
# **칠 수 있는데 안전하다고 판정**하고 있었다(미탐). docs/4_collision.md P1.
ENV_CLEAR_M     = _envf("MMS_SIM_ENV_CLEAR", 0.025)
VOXEL_M         = 0.002
# 커버리지 메시 재구성용 — 누적이 이보다 크면 다운샘플(판단용이라 원본 해상도 불필요)
MESH_MAX_PTS    = int(_envf("MMS_SIM_MESH_MAX_PTS", 150000))
MESH_VOXEL_M    = _envf("MMS_SIM_MESH_VOXEL", 0.003)
MESH_LOOP_DEPTH = int(_envf("MMS_SIM_MESH_DEPTH", 6))   # 루프용(최종은 8)
# grazing 임계 — 입사각이 이보다 크면 구조광이 못 잡는다(윗면 gap 생성 = NBV 대상)
MAX_INCIDENCE_DEG = _envf("MMS_SIM_MAXINC", 50.0)
APPLY_INCIDENCE = _envs("MMS_SIM_INCIDENCE", "1") == "1"
# 내부·오목면은 **비스듬히 들여다볼 수밖에 없어** 입사각이 필연적으로 크다.
# 외부용 임계(50°)를 그대로 쓰면 어렵게 만든 내부 자세의 점이 전부 걸러진다.
# ⚠ 크게 풀면 안 된다(실측). 70° 로 풀었더니 스치는 각의 얇고 성긴 점이 대량 유입돼
#   Poisson 메시가 파편화됐다: boundary 915 → **9,286mm**, gaps 18 → **251**.
#   sim 은 GT 깊이라 그 점들이 '정확'하지만, 실물 구조광은 그 각도에서 못 잡는다 —
#   즉 완화는 **실물에 없는 데이터를 만들어내는** 셈이라 검증도 무의미해진다.
MAX_INCIDENCE_INNER_DEG = _envf("MMS_SIM_MAXINC_INNER", 60.0)
# ── 패스 간 정합 (real 과 같은 구조) ─────────────────────────────────────────
# real 은 새 패스를 master 에 **open3d ICP** 로 붙인다
# (`artec_multipass_scan_session._hint_icp_refine`, colored ICP + point-to-plane fallback,
#  init = 로봇 기구학). sim 은 GT θ 로 직접 누적만 해서 이 단계가 **없었다**.
# → hand-eye 오차를 주입하면 sim 은 흡수 수단이 없어 실물보다 훨씬 심하게 무너진다
#   (실측: 3mm/0.5° 에서 boundary 967→2178mm, gap 18→72). 즉 지금의 sim 은 실물
#   강건성을 검증할 수 없다. 같은 정합 단계를 sim 에도 둔다.
ICP_PASS_ENABLE = _envs("MMS_SIM_ICP_PASS", "1") == "1"
# coarse→fine 다단. 초기 어긋남이 max_corr 보다 크면 대응점을 못 찾으므로 넓게 시작한다.
ICP_SCALES_M    = (0.020, 0.008, 0.004)
ICP_MIN_PTS     = 300
# 정합 **단위**: "frame" | "pass" | "off".
#   real 의 Artec 은 **프레임 단위**로 master 에 재고정한다. sim 이 패스 단위(강체)로만
#   맞추면, hand-eye 오차가 자세마다 방향이 달라 **한 패스 안에서도 어긋나는 성분**을
#   못 고친다(실측: 3mm 오차에서 boundary 2178→1903, 13% 개선에 그침).
#   → real 과 같은 단위로 내린다. 대조군을 위해 env 로 바꿀 수 있게 둔다.
ICP_LEVEL = _envs("MMS_SIM_ICP_LEVEL", "frame")
ICP_MIN_PTS_FRAME = 150
# 원판 상면에서 이 높이 이내의 점은 **원판**으로 보고 버린다. 물체는 원판 위에 앉아
# 있어 최하단이 원판에 가려 어차피 안 잡히므로 손실이 없다.
DISC_REJECT_M = _envf("MMS_SIM_DISC_REJECT", 0.003)
# ── Phase 3 (바닥면 flip) ────────────────────────────────────────────────────
# real 은 사람이 물체를 손으로 뒤집는다(`make_axis_physical_rotations("y",[0,90,180])`).
# sim 은 물체가 USD prim 이므로 **프로그램으로** 같은 회전을 준다 → 3단계 전체를 sim 에서
# 검증할 수 있다. 남은 gap 의 상당수는 법선이 수평 아래를 향해(실측 중앙값 -13°)
# **어떤 관측 elevation 으로도 못 보므로**, flip 없이는 원리적으로 못 메운다.
FLIP_AXIS = _envs("MMS_SIM_FLIP_AXIS", "y")
# 기본은 **180° 만** — 0° 는 Phase 1·2 가 이미 스캔한 원래 자세이고, 바닥면은
# 180° 뒤집기 하나로 취득된다. 90°(옆으로 눕히기)는 시간이 두 배 들고 이득이 작아
# **요청 시에만** 쓴다:  MMS_SIM_FLIP_ANGLES=90,180
FLIP_ANGLES_DEG = tuple(float(x) for x in
                        _envs("MMS_SIM_FLIP_ANGLES", "180").split(",") if x.strip())

# Phase 2 NBV
NBV_DISTANCE_M  = 0.225
NBV_APPROACH_ELS  = [55.0, 50.0, 60.0, 65.0]
NBV_APPROACH_AZIS = [0.0, 30.0, -30.0, 60.0, -60.0, 90.0, -90.0, 180.0]
# Phase 2 반복 횟수. gap 을 **직접 겨냥**하게 되면서 자세마다 덮는 영역이 국소적이라
# (예전엔 축을 봐서 한 패스가 광역을 덮었다) 더 많은 패스가 필요하다.
NBV_K_MAX       = int(_envf("MMS_SIM_NBV_K", 8))
# ★ Phase 2 부분 스윕 — 목표 gap 이 보이는 **좁은 각도 구간만** 돌고 끝낸다.
#   예전에는 자세마다 무조건 360°(24프레임)를 돌았다. gap 하나 채우려고 한 바퀴를 도는
#   동안 스캐너는 이미 가진 면만 다시 본다 — 순수한 낭비다.
# ⚠ 좁히면 정합이 무너진다(실측). 60°/7프레임에서 프레임 간 중첩이 부족해 ICP 가
#   흔들렸고, 어긋난 점이 이중 표면을 만들어 메시가 조각났다:
#   boundary 963 → 13,927mm, gaps 20 → 418 (구멍이 는 게 아니라 **메시가 부서진 것**).
NBV_PATCH_SPAN_DEG = _envf("MMS_SIM_NBV_SPAN", 90.0)   # 목표 θ 중심 ±span/2
# 보장 고도각 패스의 스윕 폭. 360=전회전(기본). 오목 내부 커버가 전회전에
# 근거하므로 줄이면 내부 취득량이 준다.
ENSURE_SPAN_DEG    = _envf("MMS_SIM_ENSURE_SPAN", 360.0)
# 디버그 오버레이 갱신 주기·점수 — 매 프레임 수만 점을 USD 에 쓰면 캡처보다 비싸다.
DRIFT_TRANS_M   = _envf("MMS_SIM_DRIFT_T", 0.030)
DRIFT_ROT_DEG   = _envf("MMS_SIM_DRIFT_R", 15.0)
ICP_DUMP        = os.environ.get("MMS_SIM_ICP_DUMP", "")
ICP_SCALE_DEBUG = bool(_envf("MMS_SIM_ICP_DEBUG", 0))
PROFILE_EVERY   = int(_envf("MMS_SIM_PROFILE_EVERY", 20))   # N프레임마다 소요시간 내역
VIZ_EVERY       = int(_envf("MMS_SIM_VIZ_EVERY", 12))
VIZ_MAX_PTS     = int(_envf("MMS_SIM_VIZ_MAX_PTS", 12000))
# master 다운샘플 캐시를 다시 만드는 성장 배수. 1.15 = 점이 15% 늘면 갱신.
MASTER_DS_GROW  = _envf("MMS_SIM_MASTER_DS_GROW", 1.15)
# 0 = 각도 구간에 비례해 자동 산정(실물과 같은 프레임 밀도).
NBV_PATCH_FRAMES   = int(_envf("MMS_SIM_NBV_FRAMES", 0))
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
                                ensure_els=ENSURE_ELS,
                                log=lambda m: print(f"[isaac_scan] {m}"))
        self._last_roll = None                      # _view_q 가 채택한 roll(로그용)
        # 자가충돌 = 실제 메시 판정(캐시 없으면 None → 캡슐 폴백)
        self._mesh_self = _mesh_sc.get_default(margin_m=SELF_CLEAR_M)
        if self._mesh_self is not None:
            print(f"[isaac_scan] 자가충돌 = 실제 메시 판정 (여유 {SELF_CLEAR_M*1000:.0f}mm)")
        # 셀 구조물(벽·상판·저울·툴스탠드) — 기존 world 는 턴테이블/프레임만 있었다.
        # ★ Phase 무관 **단일 충돌 게이트**. 로봇=메시, 환경/링크=SDF, 경로=보수적 전진.
        #   예전엔 Phase 2 만 검사하고 Phase 1·3 은 무검사였다(docs/4_collision.md).
        self._cm = _colmodel.get_default(self_margin_m=SELF_CLEAR_M,
                                         env_margin_m=ENV_CLEAR_M)
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
        self._obj_prim = prim                                 # flip 대상(Phase 3)
        mn, mx = self._aabb_world(prim)                       # 객체 world AABB
        self.obj_center_w = (mn + mx) / 2.0                   # 객체 중심(world)
        self.obj_top_w = np.array([self.obj_center_w[0], self.obj_center_w[1], mx[2]])
        self.obj_zlo_w, self.obj_zhi_w = float(mn[2]), float(mx[2])   # **실제 객체 z 범위(world)**
        self.obj_radius = float(max(mx[0] - mn[0], mx[1] - mn[1]) / 2.0)
        self.obj_height = float(mx[2] - mn[2])
        # flip 은 **절대각**(원래 기준)이므로 치수도 항상 **원본**에서 계산해야 한다.
        # 이전에는 1차 flip 이 덮어쓴 값을 2차가 원본으로 착각해 크롭 창이 틀렸다
        # (180° 인데 126mm 창을 써서 원판 점이 섞였다).
        self._obj0 = dict(center=self.obj_center_w.copy(), radius=self.obj_radius,
                          zlo=self.obj_zlo_w, zhi=self.obj_zhi_w)
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
        import time as _t
        _ta = _t.time()
        pc_b = self.scanner.capture_points_base(self.robot, self.mms._T_EC, settle=CAPTURE_SETTLE)
        self._tick("  ├cam", _ta)
        _tb = _t.time()
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
        # ★ 턴테이블 원판 평면을 배제한다. 원판 상면(axis_w[2])에 붙은 점은 물체가
        #   아니라 **원판**이다 — 물체가 그 위에 앉아 있으므로 물체의 진짜 최하단은
        #   원판보다 위에 있고, 어차피 원판에 가려 스캔되지 않는다.
        #   Phase 3 에서 컵을 뒤집으면 **열린 면으로 원판이 그대로 보여** 대량 혼입된다
        #   (실측: 180° 패스가 +68,899점 = 정상 패스의 3.5배 → 메시 파손).
        z_floor = float(self.axis_w[2]) + DISC_REJECT_M
        m = ((r < self.obj_radius + 0.02)
             & (pc[:, 2] > max(self.obj_zlo_w - 0.005, z_floor))
             & (pc[:, 2] < self.obj_zhi_w + 0.02))
        obj = _voxel(pc[m], VOXEL_M)
        self._tick("  ├crop", _tb)
        n_crop = len(obj)
        _td = _t.time()
        if APPLY_INCIDENCE and len(obj) >= 4:
            obj = self._incidence_filter(obj)
        self._tick("  └incidence", _td)
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
        lim = (MAX_INCIDENCE_INNER_DEG if getattr(self, "_gap_pass", False)
               else MAX_INCIDENCE_DEG)
        return pts_w[cos_inc > math.cos(math.radians(lim))]

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
        """''=충돌없음, 아니면 사유. **단일 게이트**(자가+환경 메시/SDF) 우선."""
        if self._cm is not None:
            ok, why = self._cm.is_pose_safe(q)
            return "" if ok else why
        # 폴백: 캐시가 없을 때만 예전 캡슐/메시 경로
        c1, why = pose_collision(world, q, T_EC=self.T_EC, link_radii=LINK_RADII)
        is_self = bool(c1) and "self:" in why
        if c1 and not is_self:
            return "scene"
        if self._env_mesh is not None and self._env_mesh.collides(q)[0]:
            return "scene(env)"
        if self._mesh_self is not None:
            return "self" if self._mesh_self.collides(q)[0] else ""
        return "self" if is_self else ""

    def _swept_free(self, world, q0, q1):
        """(free, reason). 게이트의 **보수적 전진**(터널링 원리적 불가) 사용."""
        if self._cm is not None:
            ok, why, _ = self._cm.is_path_safe(q0, q1)
            return ok, why
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

    def _icp_to_master(self, pts, min_pts, tag):
        """canonical 점군을 누적 master 에 ICP 로 붙여 반환. 게이트 실패 시 원본 그대로.

        real 이 새 스캔을 master 에 붙이는 것(`_hint_icp_refine`)과 같은 역할.
        초기값은 identity — 로봇 기구학이 이미 맞춰 놨기 때문이다.
        """
        if len(pts) < min_pts or not self.accum:
            return pts, None
        master = self._master_ds()
        if master is None or len(master) < ICP_MIN_PTS:
            return pts, None
        try:
            import open3d as o3d
            from utils.nbv.icp_strategy import icp_with_gates
            src = o3d.geometry.PointCloud()
            src.points = o3d.utility.Vector3dVector(_voxel(pts, VOXEL_M))
            tgt = o3d.geometry.PointCloud()
            tgt.points = o3d.utility.Vector3dVector(master)
            if ICP_DUMP and tag == "flip":      # 오프라인 분석용 입력 덤프
                np.savez_compressed(ICP_DUMP, src=np.asarray(src.points),
                                    tgt=np.asarray(tgt.points))
                print(f"[isaac_scan]   (덤프) ICP 입력 → {ICP_DUMP}")
            # flip 패스(tag="flip")는 hint 기준 오차가 프레임 드리프트보다 크므로
            # 이동 허용치를 넓힌다. 그래도 게이트는 유지 — 틀린 정합이 통과하면
            # 메시 전체가 망가진다.
            # ⚠ 게이트를 완화하면 안 된다(2026-08-18 실측). flip 패스에서 RMSE 임계를
            #   2→4mm 로 풀었더니 ICP 가 14.6mm·24.6mm 를 보정했는데 **결과가 나빠졌다**
            #   (bbox Z 79→83mm, 아랫면 정점 25,608→11,033). 뒤집힌 바닥면이 물체 옆면에
            #   미끄러져 붙는 국소최소다. 게이트가 기각한 데는 이유가 있다.
            is_flip = (tag == "flip")
            drift = 0.080 if is_flip else DRIFT_TRANS_M
            drot = 15.0 if is_flip else DRIFT_ROT_DEG
            T, res = np.eye(4), None
            for corr in ICP_SCALES_M:
                self._pump()
                res = icp_with_gates(src, tgt, T, max_correspondence_distance=corr,
                                     rmse_thresh=corr / 2.0, fitness_thresh=0.20,
                                     drift_trans_m=drift, drift_rot_deg=drot)
                if ICP_SCALE_DEBUG:
                    print(f"[isaac_scan]   (icp/{tag}) corr={corr*1000:.0f}mm "
                          f"ok={res.ok} fitness={res.fitness:.3f} "
                          f"rmse={res.rmse*1000:.2f}mm Δt={res.delta_translation_m*1000:.1f}mm "
                          f"Δr={res.delta_rotation_deg:.1f}° [{res.reason}]")
                T = res.T_refined
            if res is not None and res.ok:
                return pts @ T[:3, :3].T + T[:3, 3], res
            return pts, res
        except Exception as e:                       # noqa: BLE001
            print(f"[isaac_scan]   ⚠ ICP 예외({type(e).__name__}: {e}) — 기구학 그대로")
            return pts, None

    def _master_ds(self):
        """누적 master 의 다운샘플 캐시.

        ⚠ 예전 구현은 `n != 이전 n` 으로 무효화했는데, 프레임마다 점이 늘어 **항상**
          다시 만들었다 — 즉 캐시가 아니었다. Phase 2 처럼 누적이 30만점을 넘으면
          프레임마다 vstack+voxel 전체 재계산이라 캡처 간격이 눈에 띄게 벌어졌다.
          → **일정 비율 이상 늘었을 때만** 갱신한다. ICP 타깃은 조금 옛것이어도
            정합 품질에 영향이 거의 없다(어차피 다운샘플된 근사 타깃이다).
        """
        n = sum(len(a) for a in self.accum)
        if n == 0:
            return None
        n0 = getattr(self, "_ds_n", 0)
        if getattr(self, "_ds_cache", None) is None or n > n0 * MASTER_DS_GROW:
            self._ds_cache = _voxel(np.vstack(self.accum), VOXEL_M * 2.0)
            self._ds_n = n
        return self._ds_cache

    def _unflip(self, pts_w):
        """flip 된 물체의 점을 **명목 회전(hint)** 으로 canonical 에 되돌린다.

        실물은 사람이 뒤집으므로 실제 자세를 못 읽는다 → hint 만 쓸 수 있다.
        남는 잔차(사람 손 오차 / sim 물리 정착)는 **ICP 가 메운다**. real 과 같은 구조.
        """
        M = getattr(self, "_flip_hint", None)
        if M is None or len(pts_w) == 0:
            return pts_w
        Mi = np.linalg.inv(M)
        return pts_w @ Mi[:3, :3].T + Mi[:3, 3]

    def _merge_frame(self, pts_canon):
        """프레임 하나를 master 에 정합해 누적 (real 의 프레임 단위 재고정 대응)."""
        if len(pts_canon) == 0:
            return
        if ICP_LEVEL != "frame" or not ICP_PASS_ENABLE:
            self.accum.append(pts_canon)
            return
        out, res = self._icp_to_master(pts_canon, ICP_MIN_PTS_FRAME, "frame")
        self.accum.append(out)
        if res is not None and not res.ok:
            self._icp_fail = getattr(self, "_icp_fail", 0) + 1

    def _merge_pass(self, pass_pts, force_icp=False):
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
        if force_icp and self.accum and len(pts) >= ICP_MIN_PTS:
            # ★ flip 패스는 **전역 정합**으로 붙인다 — 명목 회전(hint)은 사람 손 오차
            #   ±10° 를 안고 들어와 폐기된 경로다(벤치마크에서 '오답' baseline).
            #   전회전 스윕이라 중첩이 크므로(실측 85%) 초기값 없이도 ΔR 0.0° 가 나온다.
            #   게이트는 fitness 가 아니라 **방법 간 합의 + 형상(AABB) 일치** —
            #   회전대칭 표면은 틀린 각도로도 fitness 가 높다(실측 0.98 인데 ΔR 144°).
            from utils.nbv.global_registration import register_consensus
            T, info = register_consensus(_voxel(pts, VOXEL_M), self._master_ds(),
                                         log=lambda m: print(f"[isaac_scan]   {m}"))
            if T is not None:
                pts = pts @ T[:3, :3].T + T[:3, 3]
                print(f"[isaac_scan]   flip 정합 채택({info['method']}) "
                      f"합의ΔR={info['agree_rot_deg']:.1f}° bbox={info['bbox_diff_mm']}mm")
                self.accum.append(pts)
                return
            print(f"[isaac_scan]   ⚠ flip 정합 기각({info.get('reason','')}) "
                  f"— 명목 회전(hint) + master 국소정합으로 폴백")
            # hint 는 **원래 좌표(GT)** 기준이지만 master 는 프레임 ICP 로 이미 수 mm
            # 드리프트해 있다. hint 만 쓰면 표면이 겹쳐 메시가 거칠어진다
            # (실측: bbox Z 79mm vs 실제 77mm, boundary 1453mm).
            # → hint 로 대략 맞춘 뒤 **master 에 국소 ICP** 로 붙인다.
            print(f"[isaac_scan]   (진단) unflip **전** flip패스 z="
                  f"[{pts[:,2].min():.3f},{pts[:,2].max():.3f}] "
                  f"중심={np.round(pts.mean(0),3).tolist()} 점={len(pts)}")
            _h = getattr(self, "_flip_hint", np.eye(4))
            print(f"[isaac_scan]   (진단) hint 예측 물체 z="
                  f"[{self.obj_zlo_w:.3f},{self.obj_zhi_w:.3f}]  "
                  f"hint 이동={np.round(_h[:3,3],3).tolist()}")
            pts = self._unflip(pts)
            _m = self._master_ds()
            if _m is not None and len(pts):
                print(f"[isaac_scan]   (진단) hint 후 flip패스 중심="
                      f"{np.round(pts.mean(0),3).tolist()} bbox="
                      f"{np.round((pts.max(0)-pts.min(0))*1000,0).tolist()}mm / "
                      f"master 중심={np.round(_m.mean(0),3).tolist()} bbox="
                      f"{np.round((_m.max(0)-_m.min(0))*1000,0).tolist()}mm")
            aligned, res = self._icp_to_master(pts, ICP_MIN_PTS, "flip")
            if res is not None and res.ok:
                print(f"[isaac_scan]   flip 국소정합 적용: "
                      f"Δt={res.delta_translation_m*1000:.2f}mm "
                      f"Δr={res.delta_rotation_deg:.2f}° fitness={res.fitness:.2f}")
                pts = aligned
            elif res is not None:
                print(f"[isaac_scan]   ⚠ flip 국소정합 게이트 실패({res.reason}) — hint 그대로")
            self.accum.append(pts)
            return
        if ((ICP_LEVEL != "pass" and not force_icp) or not self.accum
                or not ICP_PASS_ENABLE or len(pts) < ICP_MIN_PTS):
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

    def _drive(self, q, steps=DRIVE_STEPS) -> bool:
        """현재→q 이동. **막히면 우회 경로**를 계획해 따라간다. 반환=이동했는가.

        ★ 예전에는 무조건 직선 보간으로 갔다 — Phase 1·3 은 충돌 검사조차 없었고,
          Phase 2 도 '막히면 그 자세를 버리는' 식이라 **돌아가면 되는 자세를 잃었다**
          (실측: 안전 자세 12개 중 직선이 막힌 쌍이 6개, 전부 우회 성공).
          계획도 실패하면 **움직이지 않고 False** 를 돌려 상위가 그 패스를 건너뛰게 한다.
        """
        q1 = np.asarray(q, float)
        q0 = np.asarray(self.robot.get_joint_angles(is_radian=True), float)
        way = [q0, q1]
        if self._cm is not None:
            ok, why, _ = self._cm.is_path_safe(q0, q1)
            if not ok:
                if why.startswith(("start", "goal")):
                    print(f"[isaac_scan] ✘ 이동 거부 — {why} (자세 자체가 충돌)")
                    return False
                print(f"[isaac_scan]   직선 막힘({why}) — 우회 계획")
                path = _plan_path(q0, q1,
                                  lambda a, b: self._cm.is_path_safe(a, b)[:2],
                                  lower=kin.JOINT_LOWER, upper=kin.JOINT_UPPER,
                                  step=0.3, max_iter=600, shortcut_iters=60,
                                  log=lambda m: print(f"[isaac_scan]   {m}"))
                if path is None:
                    print("[isaac_scan] ✘ 이동 거부 — 우회 경로 없음")
                    return False
                way = path
        n = max(1, int(steps))
        for a, b in zip(way[:-1], way[1:]):
            for k in range(1, n + 1):
                qk = a + (k / n) * (b - a)
                self.robot.arm.set_servo_angle(angle=qk.tolist(), is_radian=True, wait=True)
                self.world.step(1, render=True)
        self.world.step(4, render=True)             # 도착 후 정착
        return True

    def _scan_pass(self, q, n_theta, label="", phase=1):
        """★ 통합 캡처 = 로봇을 q 자세로 두고 **턴테이블 전회전**하며 프레임 캡처·−θ 누적.
        real 캡처(로봇 pose + streaming 전회전 + relocalization)에 1:1 대응. Phase1·2 공용.
        sim 은 GT θ 라 −θ 회전 = relocalization 역할(정확). 입사각 필터로 좋은 프레임만 기여."""
        # AT_CURRENT = '이동 없이 현재 자세에서 캡처'(Phase 3 flip 후). 센티널이므로
        # 관절해로 해석하면 안 된다 — sim 은 Phase 3 가 미지원이라 이 경로가 미검증이었다.
        if q is not AT_CURRENT and q is not None:
            if not self._drive(q):
                print("[isaac_scan]   ⚠ 이동 불가 — 이 패스 건너뜀")
                return False
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
                canon = _rot_about_axis(obj, self.axis_w, self.axis_dir_w, -th_act)
                if phase != 3:
                    canon = self._unflip(canon)      # Phase3 는 아래 전역정합이 담당
                # flip 패스는 프레임 하나의 중첩이 너무 적어 프레임 ICP 가 불안정하다
                # → 패스 전체를 모아 한 번에 정합한다(중첩 확보).
                if ICP_LEVEL == "frame" and phase != 3:
                    self._merge_frame(canon)      # 프레임마다 master 에 재고정
                else:
                    pass_pts.append(canon)
            if _viz.ENABLED and self.accum and (i % VIZ_EVERY == 0):
                _viz.show_accum(self.stage, np.vstack(self.accum),
                                max_pts=VIZ_MAX_PTS,
                                theta=th_act, axis_xy=self.axis_w[:2])
        if ICP_LEVEL != "frame" or phase == 3:
            self._merge_pass(pass_pts, force_icp=(phase == 3))
        elif getattr(self, "_icp_fail", 0):
            print(f"[isaac_scan]   ICP 게이트 실패 프레임 {self._icp_fail}개 "
                  f"(기구학 값 사용)")
            self._icp_fail = 0
        # 루프 닫기: 350°→360°(≡0°) 앞으로 10° 만 더 회전 후 θ 리셋.
        # (기존엔 350°를 역방향으로 되감으며 렌더 → 느리고 "홱" 끊김. 360°≡0° 라
        #  clearpos 의 _co_rotate(0) 는 시각 점프 없음.)
        self.turntable.move_abs(float(2 * np.pi), float(np.radians(30.0)))
        self.turntable.clearpos()
        # ★ 오버레이를 θ=0 으로 되그린다. 캡처 중에는 현재 θ 로 회전시켜 물리 물체에
        #   겹쳐 보여주는데, clearpos() 로 물체가 0 으로 돌아온 뒤에도 마지막 θ 로
        #   남아 있으면 어긋나 보인다(패스 종료 시점의 '이상한' 화면의 원인).
        if _viz.ENABLED and self.accum:
            _viz.show_accum(self.stage, np.vstack(self.accum))
        added = sum(len(a) for a in self.accum) - before
        print(f"[isaac_scan]   pass 완료 (+{added}점, 총 {before+added})")
        return True

    def _scan_patch(self, q, theta_c, label="", span_deg=None, n_frames=None):
        """**부분 스윕** — 목표 θ 주변 좁은 구간만 돌며 캡처한다.

        Phase 2 의 목적은 '특정 결손면 채우기'다. 그 면이 보이는 각도 구간만 돌면 되고,
        한 바퀴를 도는 동안 이미 가진 면을 다시 보는 것은 순수한 낭비다.
        (Phase 1·3 은 전면 커버가 목적이라 전회전 `_scan_pass` 를 그대로 쓴다.)
        """
        if q is not AT_CURRENT and q is not None:
            if not self._drive(q):
                print("[isaac_scan]   ⚠ 이동 불가 — 이 패스 건너뜀")
                return False
        span = math.radians(float(span_deg if span_deg is not None else NBV_PATCH_SPAN_DEG))
        n = int(n_frames if n_frames is not None else NBV_PATCH_FRAMES)
        if n <= 0:      # 각도 비례 — 전회전과 같은 프레임 밀도를 유지한다
            n = int(round(NBV_FRAMES_PER_REV * span / (2 * math.pi)))
        n = max(2, n)
        before = sum(len(a) for a in self.accum)
        patch_pts = []      # ★ 프레임마다 붙이지 않는다 — 아래 참고
        if label:
            print(f"[isaac_scan] === patch: {label} "
                  f"(θ={math.degrees(theta_c):.0f}° ±{math.degrees(span)/2:.0f}°, {n}프레임) ===")
        import time as _t
        for i, th in enumerate(np.linspace(theta_c - span/2, theta_c + span/2, n)):
            _tm = _t.time()
            self.turntable.move_abs(float(th % (2*np.pi)), float(np.radians(30.0)))
            self.turntable.wait_motion_done()
            self._tick("turntable", _tm)
            th_act = float(self.turntable.getActualPos())
            import time as _t
            _tc = _t.time()
            obj = self._capture_obj_world(log=(i == 0))
            self._tick("capture", _tc)
            if len(obj):
                _tm2 = _t.time()
                canon = _rot_about_axis(obj, self.axis_w, self.axis_dir_w, -th_act)
                patch_pts.append(self._unflip(canon))
                self._tick("merge", _tm2)
            # 실시간 오버레이 — 프레임마다 갱신해 Phase 2 가 무엇을 채우는지 바로 보인다
            # (예전엔 계획 시점에만 갱신돼 캡처 중에는 고정된 것처럼 보였다).
            # 오버레이는 **간헐 갱신**. USD 에 수만 점을 매 프레임 쓰면 렌더보다 비싸다.
            if _viz.ENABLED and (i % VIZ_EVERY == 0) and (self.accum or patch_pts):
                _tv = _t.time()
                _viz.show_accum(self.stage, np.vstack(self.accum + patch_pts),
                                max_pts=VIZ_MAX_PTS,
                                theta=th_act, axis_xy=self.axis_w[:2])
                self._tick("viz", _tv)
            if PROFILE_EVERY and (i + 1) % PROFILE_EVERY == 0:
                self.profile_report()          # 병목 추적용 (MMS_SIM_PROFILE_EVERY=0 이면 끔)
        # ★ 정합 단위 = **패치 전체**. 프레임 단위로 붙이면 부분 스윕은 프레임 간
        #   중첩이 부족해 ICP 가 흔들리고 메시가 조각난다(실측: gaps 20→418).
        #   패치를 한 덩어리로 모으면 master 와의 중첩이 충분해 안정적으로 붙는다.
        if patch_pts:
            pts = np.vstack(patch_pts)
            aligned, res = self._icp_to_master(pts, ICP_MIN_PTS, "patch")
            if res is not None and res.ok:
                print(f"[isaac_scan]   패치 정합: Δt={res.delta_translation_m*1000:.2f}mm "
                      f"Δr={res.delta_rotation_deg:.2f}° fitness={res.fitness:.2f}")
                pts = aligned
            elif res is not None:
                print(f"[isaac_scan]   ⚠ 패치 정합 게이트 실패({res.reason}) — 기구학 그대로")
            self.accum.append(pts)
        if _viz.ENABLED and self.accum:
            _viz.show_accum(self.stage, np.vstack(self.accum))   # θ=0 기준으로 되그림
        added = sum(len(a) for a in self.accum) - before
        print(f"[isaac_scan]   patch 완료 (+{added}점, 총 {before+added})")
        return True

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
            if q is not None and self._cm is not None:
                ok, why = self._cm.is_pose_safe(q)
                if not ok:                     # Phase1 도 이제 충돌을 본다(예전 무검사)
                    print(f"[isaac_scan]   az={azd:.0f}° 충돌({why}) — 건너뜀")
                    q = None
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
        """Phase 1·3 = 전회전(전면 커버), **Phase 2 = 부분 스윕**(목표 gap 만).

        Phase 2 에서 계획기가 목표 θ 를 남겨두면(`_next_theta`) 그 주변만 돈다.
        남기지 않았으면(축-고도각 폴백) 기존대로 전회전한다.
        """
        if phase == 2 and getattr(self, "_next_theta", None) is not None:
            th = self._next_theta
            sp = getattr(self, "_next_span", None)
            self._next_theta = None
            self._next_span = None
            return self._scan_patch(pose, th, label=label, span_deg=sp)
        n_theta = N_THETA_P2 if phase == 2 else N_THETA
        return self._scan_pass(pose, n_theta, label=label, phase=phase)

    # ── Phase 2 (NBV = 추가 관측 elevation 자세, real 전회전 대응) ──────────────
    def _plan_nbv_pose(self, world, q_cur, gaps):
        import time as _t
        _tp = _t.time()
        self._pump(2)
        try:
            return self._plan_nbv_pose_inner(world, q_cur, gaps)
        finally:
            self._tick("plan", _tp)
            self._pump(2)

    def _plan_nbv_pose_inner(self, world, q_cur, gaps):
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

        swept = lambda a, b: self._swept_free(world, a, b)
        # ★ 뷰포트 오버레이 — gap 이 어디 잡혔는지, NBV 가 뭘 겨냥하는지 눈으로 본다.
        #   MMS_SIM_VIZ=1 일 때만. 시뮬레이터를 쓰는 최대 이점 중 하나.
        _viz.show_gaps(self.stage, gaps)
        if self.accum:
            _acc = np.vstack(self.accum)
            _viz.show_accum(self.stage, _acc)
            _viz.show_voxels(self.stage, _acc, self.obj_center_w,
                             max(self.obj_radius, self.obj_height / 2) + 0.01)

        def solve_lookat(eye, tgt):
            return _vp.solve_look_at_q(
                kin, eye, tgt, q_cur, self.T_EC, T_WB=self.T_WB,
                convention=_vp.CAM_USD, rolls_deg=VIEW_ROLLS_DEG,
                n_seed_alt=IK_SEED_TRIES)

        # ── ① 보장 고도각 — 오목 내부는 gap 으로 안 잡히므로(닭·달걀) 사전지식 ──
        need = [e for e in ENSURE_ELS
                if not any(abs(float(v[0]) - float(e)) < 1e-6
                           for v in self._nbv.visited)]
        if need:
            res0 = self._nbv.plan(gaps, q_cur, solve_pose, swept,
                                  roll_order_fn=roll_order)
            if res0 is not None:
                # 입사각 완화는 하지 않는다 — 광역 스윕에서 풀면 스치는 점이 대량 유입돼
                # 메시가 붕괴한다(실측: boundary 9,286mm / gaps 251).
                self._gap_pass = False
                # 캡처 경로를 patch 로 통일한다. 기본 span=360°(전회전)라 동작은 그대로고,
                # MMS_SIM_ENSURE_SPAN 으로 줄이면 그만큼 짧아진다(오목 내부는 전회전이
                # 커버의 근거이므로 줄일 때는 내부 취득량을 함께 확인할 것).
                self._next_theta = 0.0
                self._next_span = ENSURE_SPAN_DEG
                return res0[0]

        # ── ② gap 겨냥 (주경로) ─────────────────────────────────────────────
        # ★ 축-고도각 방식(`plan_nbv_elevation_pose`)은 카메라가 **늘 턴테이블 축**을
        #   보므로, gap 이 어디 있든 그쪽을 향하지 않는다. gap 정보도 '법선 고도각의
        #   중앙값' 하나로 압축돼 **위치가 통째로 버려진다**. 그래서 손잡이·내벽 같은
        #   국소 결손을 원리적으로 겨냥할 수 없었다(실측: 손잡이 0점).
        #   → 표면점 p 에서 법선 n 방향 standoff 위치로 **그 gap 을 정면으로** 본다
        #     (`phase2_nbv.nbv_pose_from_candidate` 와 같은 원리).
        #   법선 정면이 막히면(작동거리가 물체보다 큰 오목면 등) 개구부 쪽으로 기울인다.
        fr = self._nbv.plan_frontier(gaps, q_cur, solve_lookat, swept,
                                     standoff_m=WORK_FOCUS,
                                     axis_xy=self.axis_w[:2],
                                     az_pref_deg=tuple(VIEW_AZIS_DEG))
        if fr is not None:
            self._gap_pass = True
            self._next_theta = fr[3]        # 이 θ 주변만 부분 스윕
            try:                            # 겨냥한 gap + 카메라 위치를 씬에 표시
                _q = fr[0]
                _cam = (self.T_WB @ kin._fk_frames_m(_q)[7]
                        @ np.linalg.inv(self.T_EC))[:3, 3]
                _viz.show_target(self.stage, fr[1].p_O, _cam)
            except Exception:
                pass
            return fr[0]

        # ── ③ 폴백: 축-고도각 (턴테이블 회전과 궁합이 좋은 광역 스윕) ──────────
        self._gap_pass = False
        res = self._nbv.plan(gaps, q_cur, solve_pose, swept,
                             roll_order_fn=roll_order)
        return None if res is None else res[0]

    # ── Phase 2 (NBV hole-fill) 프리미티브 — 수렴 루프는 공용 컨트롤러 소유 ──
    def _pump(self, n=1):
        """무거운 파이썬 구간 사이에 **UI 이벤트를 한 번 돌려준다**.

        main_artec 은 파이썬 루프에서 world.step 을 직접 돌리고 Isaac UI 도 **같은
        스레드**라, Poisson·ICP·계획 같은 순수 파이썬 작업 중에는 GUI 가 완전히 멈춘다
        (사용자 불편). 단일 무거운 호출 중에는 어쩔 수 없지만, 호출 **사이사이**에
        펌핑하면 클릭·카메라 조작이 살아난다.
        """
        try:
            app = getattr(self.world, "_sim_app", None)
            if app is not None:
                for _ in range(int(n)):
                    app.update()
        except Exception:
            pass

    def _tick(self, tag, t0):
        import time as _t
        dt = _t.time() - t0
        self._prof = getattr(self, "_prof", {})
        self._prof[tag] = self._prof.get(tag, 0.0) + dt
        return dt

    def profile_report(self):
        pr = getattr(self, "_prof", {})
        if pr:
            tot = sum(pr.values())
            print("[isaac_scan] 소요시간: " + ", ".join(
                f"{k} {v:.1f}s({100*v/tot:.0f}%)"
                for k, v in sorted(pr.items(), key=lambda kv: -kv[1])))

    def build_coverage_mesh(self):
        # ★ Phase 2 는 매 반복 메시를 다시 만든다. 누적 점이 30만을 넘으면 Poisson 이
        #   반복마다 큰 비용이 된다 — **커버리지 판단용**이므로 원본 해상도가 필요 없다.
        #   voxel 로 줄여도 경계(gap) 판단은 거의 동일하다.
        import time as _t
        _t0 = _t.time()
        pcd, _ = self._merged_pcd()
        try:
            import open3d as o3d
            n0 = len(pcd.points)
            if n0 > MESH_MAX_PTS:
                pcd = pcd.voxel_down_sample(MESH_VOXEL_M)
                print(f"[isaac_scan]   메시용 다운샘플 {n0:,} → {len(pcd.points):,}점")
        except Exception:
            pass
        if len(pcd.points) < 200:
            print("[isaac_scan] Phase2 누적점 부족(<200) — 종료.")
            return None
        # ★ 루프용 메시는 **저비용**으로. Artec 도 스캔 중에는 FastFusion(복셀 기반)을
        #   쓰고 Poisson 은 후처리 전용이다. 여기서 필요한 건 '어디가 비었나' 판단이지
        #   최종 품질이 아니므로 depth 를 낮춘다(비용은 depth 에 급격히 증가).
        #   최종 메시는 `finalize()` 에서 depth=8 로 한 번만 만든다.
        self._pump(2)
        m = p2.pcd_to_mesh_poisson(pcd, depth=MESH_LOOP_DEPTH, density_quantile=0.04)
        self._pump(2)
        self._tick("mesh", _t0)
        return m

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
        """sim 도 Phase 3 지원 — 물체를 USD 에서 회전시킨다(real=사람 손회전)."""
        return bool(FLIP_ANGLES_DEG) and self.stage.GetPrimAtPath(self._obj_prim).IsValid()

    def next_flip(self) -> bool:
        """다음 flip 자세로 물체를 회전. 더 없으면 False.

        ★ 누적 좌표계 주의 — 뒤집은 뒤 캡처한 점은 **flip 된 물체 프레임**에 있다.
          canonical(원래 물체 자세)로 되돌리지 않으면 점군이 어긋나 병합된다.
          그래서 회전량을 `self._flip_R` 로 들고 있다가 `_accumulate_canonical` 에서
          역회전한다. real 은 Artec 이 master 에 재고정해 같은 역할을 한다.
        """
        i = getattr(self, "_flip_i", 0)
        if i >= len(FLIP_ANGLES_DEG):
            return False
        ang = float(FLIP_ANGLES_DEG[i])
        self._flip_i = i + 1
        R = _axis_rot(FLIP_AXIS, math.radians(ang))
        o0 = self._obj0                       # ★ 항상 **원본** 치수 기준
        c = o0["center"]
        # ★ 중심 기준 회전만 시키면 물체가 **원판을 파고들거나 뜬다**(실측: 90° 에서
        #   24mm 관통). 실물에서 사람은 물체를 **면에 올려놓는다** — 회전 후 원판
        #   상면에 앉도록 내려준다. 이 보정은 원래 AABB 와 명목 회전만으로 계산되므로
        #   **실물에서도 아는 값**이다(물체가 테이블에 놓인다는 물리 제약).
        lo0 = np.array([c[0] - o0["radius"], c[1] - o0["radius"], o0["zlo"]])
        hi0 = np.array([c[0] + o0["radius"], c[1] + o0["radius"], o0["zhi"]])
        corners = np.array([[x, y, z] for x in (lo0[0], hi0[0])
                            for y in (lo0[1], hi0[1]) for z in (lo0[2], hi0[2])])
        rc = (corners - c) @ R.T + c
        disc_top = float(self.axis_w[2])
        dz = disc_top - float(rc[:, 2].min())        # 원판 위에 앉히기
        M = np.eye(4); M[:3, :3] = R; M[:3, 3] = c - R @ c + np.array([0.0, 0.0, dz])
        print(f"[isaac_scan]   flip 배치: 회전 후 원판 위로 {dz*1000:+.0f}mm 이동")
        # ★ 물체 transform 을 직접 쓰지 않는다 — 턴테이블이 rider base 로 매 회전마다
        #   덮어써서 flip 이 지워진다(실측 실패). 턴테이블 API 로 base 에 합성한다.
        if not self.turntable.set_rider_flip(self._obj_prim, M):
            print(f"[isaac_scan] ⚠ flip 실패 — '{self._obj_prim}' 이 턴테이블 rider 가 아님")
            return False
        self.world.step(8, render=True)
        # ★ 보정에는 **명목 회전(hint)** 만 쓴다 — 실물은 사람이 손으로 뒤집으므로
        #   물체의 실제 자세를 읽을 방법이 없다. USD 에서 world transform 을 읽으면
        #   **sim 만 정확해져 'sim 이 real 을 검증한다'는 전제가 깨진다**.
        #   real 과 같은 구조로 간다: 명목 회전을 hint 로 두고 **ICP 가 잔차를 정제**한다
        #   (real 은 pose_physical_rotations → _hint_icp_refine).
        self._flip_hint = M
        # 크롭 창도 새 자세에 맞춘다 — 원래 자세 기준으로 두면 flip 된 물체의 위아래가
        # 잘리고 원판 점이 섞인다(실측: 90° 에서 상단 5mm 잘림 + 하부 19mm 손실).
        rc2 = rc + np.array([0.0, 0.0, dz])
        self.obj_zlo_w, self.obj_zhi_w = float(rc2[:, 2].min()), float(rc2[:, 2].max())
        self.obj_radius = float(max(rc2[:, 0].max() - rc2[:, 0].min(),
                                    rc2[:, 1].max() - rc2[:, 1].min()) / 2.0)
        self.obj_center_w = np.array([c[0], c[1], (self.obj_zlo_w + self.obj_zhi_w) / 2.0])
        print(f"[isaac_scan]   크롭 갱신: z=[{self.obj_zlo_w:.3f},{self.obj_zhi_w:.3f}] "
              f"r={self.obj_radius*1000:.0f}mm")
        try:    # 진단 전용 — sim 에서만 알 수 있는 실제값. **보정에는 쓰지 않는다.**
            D_true = self._prim_world_T(self._obj_prim) @ np.linalg.inv(self._obj_W0)
            err = float(np.linalg.norm(D_true[:3, 3] - M[:3, 3])) * 1000.0
            print(f"[isaac_scan]   (진단) hint 대비 실제 이동차 {err:.1f}mm "
                  f"— ICP 가 이만큼을 메워야 한다")
        except Exception:
            pass
        print(f"[isaac_scan] === Phase 3: 물체 {FLIP_AXIS}축 {ang:.0f}° flip "
              f"({i+1}/{len(FLIP_ANGLES_DEG)}) ===")
        return True

    def finalize(self) -> IsaacScanResult:
        self.profile_report()
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


def _axis_rot(axis: str, ang: float) -> np.ndarray:
    """base(world Z-up) 기준 단일축 회전 — real `make_axis_physical_rotations` 와 동일 규약."""
    c, s_ = math.cos(ang), math.sin(ang)
    a = (axis or "y").lower()
    if a == "x":
        return np.array([[1, 0, 0], [0, c, -s_], [0, s_, c]], float)
    if a == "z":
        return np.array([[c, -s_, 0], [s_, c, 0], [0, 0, 1]], float)
    return np.array([[c, 0, s_], [0, 1, 0], [-s_, 0, c]], float)      # y


# ── 모듈 함수(자기완결) ─────────────────────────────────────────────────────────
def _voxel(pts, v):
    pts = np.asarray(pts, float)
    if len(pts) == 0:
        return pts
    keys = np.floor(pts / v).astype(np.int64)
    _, idx = np.unique(keys, axis=0, return_index=True)
    return pts[idx]


_PCA_CHUNK = 40000          # (N,k,3) 이웃 버퍼 상한 — 메모리 폭주 방지


def _pca_normals(pts, c_ref, k=12):
    """국소 PCA 법선. KD-tree + 배치 eigh 로 벡터화.

    이전 구현은 점마다 전수 거리(O(N²))를 돌아 8천 점에 2초가 걸렸고, 이것이
    Phase 2 캡처 간 지연의 95% 였다. 결과는 동일하고 속도만 다르다.
    """
    pts = np.ascontiguousarray(pts, dtype=np.float64)
    N = len(pts)
    if N < 4:
        return None
    k = min(k, N)
    c_ref = np.asarray(c_ref, float)

    try:
        from scipy.spatial import cKDTree
        tree = cKDTree(pts)
        out = np.empty_like(pts)
        for s0 in range(0, N, _PCA_CHUNK):          # 청크로 잘라 메모리 상한 유지
            sl = slice(s0, min(s0 + _PCA_CHUNK, N))
            _, idx = tree.query(pts[sl], k=k, workers=-1)
            idx = np.atleast_2d(idx)
            nb = pts[idx]                            # (n,k,3)
            nb -= nb.mean(1, keepdims=True)
            C = np.einsum('nki,nkj->nij', nb, nb)    # (n,3,3) 공분산
            _, v = np.linalg.eigh(C)                 # 배치 고유분해
            out[sl] = v[:, :, 0]                     # 최소 고유값 벡터 = 법선
    except ImportError:                              # scipy 없으면 기존 경로
        out = np.empty_like(pts)
        for i in range(N):
            d = pts - pts[i]
            idx = np.argpartition(np.einsum('ij,ij->i', d, d), k - 1)[:k]
            nb = pts[idx] - pts[idx].mean(0)
            _, v = np.linalg.eigh(nb.T @ nb)
            out[i] = v[:, 0]

    # 바깥쪽(중심 → 점 방향)으로 부호 통일
    flip = np.einsum('ni,ni->n', out, pts - c_ref) < 0
    out[flip] *= -1.0
    return out


def _rot_about_axis(pts, axis_pt, axis_dir, ang):
    a = np.asarray(axis_dir, float); a = a / (np.linalg.norm(a) + 1e-12)
    c, s = math.cos(ang), math.sin(ang)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    R = np.eye(3) * c + np.outer(a, a) * (1 - c) + K * s
    return (np.asarray(pts) - axis_pt) @ R.T + axis_pt
