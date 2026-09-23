"""
isaac_scan_session — Isaac Sim 스캔 세션 (lookaround GT 누적 + nbv 보강).

`mms_artec/system.py::artec_process` 의 isaac 분기가 호출하는 **production sim 스캔**.
지금까지 standalone 하니스(MMS_ext_nbv.py)에만 있던 로직을 여기로 옮겨,
`main_artec.py`(BACKEND="isaac", `~/isaacsim/python.sh main_artec.py`)로 동작하게 한다.

★ standalone(python.sh) 에선 Isaac 확장의 `utils` 패키지 충돌이 없으므로 **공용 lib을
  그대로 import** 한다 → nbv 가 **real 과 동일한 공용 코어**를 쓴다:
    - `utils/nbv/nbv_core.py`        : pcd→mesh→gap→커버리지→NBV pose (공용)
    - `utils/collision/robot_collision`: CollisionWorld·swept·keepout (공용)
    - `utils/robot/xarm7_kinematics`   : 해석 IK/FK (공용)
  sim 전용(USD 객체·Isaac 카메라·입사각 스캐너 모델·스캐너 mesh 자가충돌)만 이 파일에 둔다.

흐름: lookaround(높이 밴드마다 자세 이동 + 턴테이블 GT θ + 입사각필터 + −θ 누적)
      → nbv(부족면 NBV).
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
from utils.nbv import nbv_core as p2
from utils.nbv import lookaround as p1
from utils.nbv.scan_stage_controller import (
    run_scan_stages, resolve_stage_until, AT_CURRENT)
from utils.nbv.nbv_planner import NbvPlanner as _NbvPlanner
from utils.nbv import nbv_planner as _nbvp   # sim·real 공용 수렴·회계·IK 예산 상수
from utils.nbv.nbv_debug_dump import NbvDebugDump as _NbvDebugDump   # 반복마다 겨냥·수집 기록
from utils.collision.robot_collision import (
    CollisionWorld, pose_collision, DEFAULT_LINK_RADII, capsules_from_joints)
from utils.collision import mesh_self_collision as _mesh_sc
from utils.collision import env_collision as _env_col
from utils.collision import collision_model as _colmodel
from utils.control.joint_path_planner import plan_joint_path as _plan_path
from utils.robot import view_pose as _vp
from mms_artec.backends.isaac import isaac_debug_viz as _viz
from mms_artec.backends.isaac.isaac_recorder import Recorder as _Recorder
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
#: 카메라↔**표면** 목표 거리. ★ 공용 상수 하나에서 온다 — sim 만 0.25 를 쓰고
#  real 은 0.225 를 쓰던 갈라짐을 없앴다(`utils/nbv/standoff.py`).
from utils.collision.collision_model import SCAN_OBSTACLE_MARGIN_M as _OBS_MARGIN
from utils.nbv.standoff import (WORK_STANDOFF_M as _WORK_STANDOFF,
                                StandoffTracker as _StandoffTracker,
                                TRACK_CORE_HALF_M as _TRACK_CORE_HALF)
WORK_FOCUS      = _WORK_STANDOFF
# 관절 보간 스텝 — 클수록 부드럽고 느리다. 스텝마다
# `set_servo_angle(wait=True)` + `world.step(render=True)` 를 하므로
# **스텝 수가 곧 이동 시간**이다. 30→15(2026-08-19), 15→8(2026-09-17) 로 각각 2배.
#
# ★ 이 값은 **sim 의 화면 속도일 뿐**이고 실물 속도와 무관하다. real 은 관절속도
#   (`band_move_speed_deg_s` 등 deg/s)로 움직이므로 여기를 바꿔도 real 은 그대로다.
#   따라서 "sim 이 답답하다" 는 여기서 고치는 것이 맞고, 그래도 real 예측은 안 깨진다.
# ★ 충돌 검사는 스텝 수와 무관하다 — 경로 안전은 `is_path_safe`/`_plan_path` 가
#   따로 보증하고, 여기 스텝은 **이미 안전하다고 판정된 경로를 몇 번에 나눠
#   그리느냐** 일 뿐이다. 그래서 줄여도 안전성이 내려가지 않는다.
DRIVE_STEPS     = int(_envf("MMS_SIM_DRIVE_STEPS", 8))
# 녹화 시에는 이동 보간을 촘촘히 해 로봇이 연속으로 움직이는 것처럼 보이게 한다.
# (스캔 결과에는 영향 없음 — 경유 자세만 잘게 나눌 뿐 시작·끝 자세는 동일)
DRIVE_STEPS_REC = int(_envf("MMS_SIM_DRIVE_STEPS_REC", 60))
# ★ 프레임 밀도를 **실물과 맞춘다**. 실물 streaming 은 30초/회전 × 스캐너 max_fps(≈8)
#   ≈ 240프레임/회전이다. sim 은 36프레임이라 **20배 성겼고**, 그 결과
#     · Poisson 표면이 조각나 gap 이 폭증하고(실측 20 → 418)
#     · 프레임/패치 간 중첩이 얕아 ICP 정합이 계속 게이트에 걸렸다(RMSE 2.3~2.5mm)
#   각도 구간에 비례해 프레임을 배분하므로 부분 스윕도 같은 밀도를 유지한다.
SCAN_FRAMES_PER_REV = int(_envf("MMS_SIM_FRAMES_PER_REV", 240))   # 실물: 30s × 8fps
# nbv 도 **실물 밀도 유지**. 입력을 줄이는 대신 **연산을 가볍게** 해서 소화한다
# (사용자 방침 2026-08-19). 성긴 입력은 메시 조각남·정합 실패의 원인이었다.
NBV_FRAMES_PER_REV = int(_envf("MMS_SIM_NBV_FRAMES_PER_REV", SCAN_FRAMES_PER_REV))
N_THETA         = int(_envf("MMS_SIM_NTHETA", SCAN_FRAMES_PER_REV))
N_THETA_NBV     = int(_envf("MMS_SIM_NTHETA_NBV", NBV_FRAMES_PER_REV))
# 캡처 전 정지 렌더 step. 턴테이블 move_abs 가 이미 렌더하며 이동하므로 1 이면 충분하다.
# (2 였을 때 프레임당 렌더가 3회 → 240프레임 전회전에서 720회. 실물 밀도로 올린 뒤
#  이게 nbv 지연의 큰 몫이었다.)
CAPTURE_SETTLE  = int(_envf("MMS_SIM_CAPTURE_SETTLE", 1))
# lookaround 측면 관측 elevation.
# ★ 정정(2026-08-13) — 한때 "v3 레이아웃은 el≤30 도달 불가" 로 판단해 50 으로 올렸으나,
#   **하드웨어 한계가 아니라 `_view_q` 가 광축(roll)을 하나로 고정한 탓**이었다.
#   roll 을 풀면 같은 위치에서 el30 5/8·el40 7/8 az 가 열리고 IK 실패는 0 건이다.
#   (시드만 12개로 늘린 경우는 여전히 0/8 → 원인은 시드가 아니라 roll 이다.)
#   낮은 el 이 없으면 측면·하부가 안 찍혀 gap 이 안 줄므로 원래 값으로 되돌린다.
#   측정: scripts/sim/eval_turntable_layout.py --diag 0.365 --rolls 0   (대조군)
#         scripts/sim/eval_turntable_layout.py --diag 0.365             (roll 자유)
VIEW_EL_DEG     = _envf("MMS_SIM_VIEW_EL", 30.0)
# lookaround 플래너가 고를 수 있는 elevation 후보(p1.DEFAULT_ELS=(20,30,40,50)).
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
# nbv 에서 roll 을 gap 방향에 맞춰 고를지(1) 기본 순서(roll=0 우선)를 쓸지(0).
# **대조군 스위치** — 효과를 재려면 이것만 끄고 같은 조건으로 비교한다.
NBV_ROLL_ALIGN  = os.environ.get("MMS_SIM_NBV_ROLL_ALIGN", "1") == "1"
# 자가충돌 허용 최소 여유(m) — 실제 메시 최소거리 기준. 캡슐 시절의 암묵 임계보다
# 훨씬 작아 보이지만, 캡슐은 형상을 과대하게 덮어 임계가 부풀려져 있었을 뿐이다.
SELF_CLEAR_M    = _envf("MMS_SIM_SELF_CLEAR", 0.02)
# 셀 구조물(벽·상판·저울·툴스탠드) 최소 여유(m). 기존 장애물은 턴테이블·프레임뿐이라
# **칠 수 있는데 안전하다고 판정**하고 있었다(미탐). docs/collision.md P1.
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
#: 입사각 판정용 PCA 법선을 재는 대표점 상한 — `_incidence_filter` 주석.
INCIDENCE_PCA_MAX = int(_envf("MMS_SIM_INCIDENCE_PCA_MAX", 1500))
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
# ── flip (바닥면 flip) ────────────────────────────────────────────────────
# real 은 사람이 물체를 손으로 뒤집는다(`make_axis_physical_rotations("y",[0,90,180])`).
# sim 은 물체가 USD prim 이므로 **프로그램으로** 같은 회전을 준다 → 3단계 전체를 sim 에서
# 검증할 수 있다. 남은 gap 의 상당수는 법선이 수평 아래를 향해(실측 중앙값 -13°)
# **어떤 관측 elevation 으로도 못 보므로**, flip 없이는 원리적으로 못 메운다.
FLIP_AXIS = _envs("MMS_SIM_FLIP_AXIS", "y")
# 기본은 **180° 만** — 0° 는 lookaround·nbv 가 이미 스캔한 원래 자세이고, 바닥면은
# 180° 뒤집기 하나로 취득된다. 90°(옆으로 눕히기)는 시간이 배로 들어 기본은 끄지만,
# **세장형(키/지름 ≥ FLIP_ASPECT)은 자동 추가**된다 — 키 큰 물체의 윗면은 측면에서
# grazing + 고앙각 자세는 자가충돌이라 90° 로 눕혀야 양 끝면이 잡힌다(_flip_angles).
# env 명시(MMS_SIM_FLIP_ANGLES=90,180)가 항상 우선.
FLIP_ANGLES_DEG = tuple(float(x) for x in
                        _envs("MMS_SIM_FLIP_ANGLES", "180").split(",") if x.strip())

# nbv
NBV_DISTANCE_M  = 0.225
NBV_APPROACH_ELS  = [55.0, 50.0, 60.0, 65.0]
NBV_APPROACH_AZIS = [0.0, 30.0, -30.0, 60.0, -60.0, 90.0, -90.0, 180.0]
# nbv 반복 횟수. gap 을 **직접 겨냥**하게 되면서 자세마다 덮는 영역이 국소적이라
# (예전엔 축을 봐서 한 패스가 광역을 덮었다) 더 많은 패스가 필요하다.
NBV_K_MAX       = int(_envf("MMS_SIM_NBV_K", _nbvp.K_MAX))   # 기본은 공용값(12) — real 과 같은 상한
# ★ nbv 부분 스윕 — 목표 gap 이 보이는 **좁은 각도 구간만** 돌고 끝낸다.
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
FLIP_ASPECT     = _envf("MMS_SIM_FLIP_ASPECT", 2.0)      # 키/지름 이 값 이상=세장형
FLIP_EL_MIN     = _envf("MMS_SIM_FLIP_EL_MIN", 30.0)     # flip 관측 고도각 하한
FLIP_EL_MAX     = _envf("MMS_SIM_FLIP_EL_MAX", 70.0)     # 상한(고앙각은 자가충돌)
CONV_NEW_EPS    = _nbvp.CONV_NEW_EPS   # 공용(env MMS_NBV_CONV_NEW_EPS)   # 전역 백스톱 임계 (보수적:
#   GT 검증에서 0.5%/3회는 손실 최대 0.27%p. 주 종료는 gap 단위 dry 회계가 맡는다)
ICP_INFLATION_M = _envf("MMS_SIM_ICP_INFLATION", 0.005)  # 정합이 물체를 부풀리는 한계
#   (실측 9종: 정상 −0.1~3.8mm / 회귀 8.5·10.0mm — 5mm 가 그 사이를 가른다)
DRY_EPS         = _nbvp.DRY_EPS        # 공용(env MMS_NBV_DRY_EPS)        # 패치 생산성 판정 (실측:
#   비생산 패치 0.2~1.2%, 생산 패치 1.7~10.3% — 1.5% 가 그 사이를 가른다)
CONV_VOX_M      = _nbvp.CONV_VOX_M     # 공용       # 커버리지 판정 복셀 크기
CONV_AREA_N     = _nbvp.CONV_STALL_N   # 공용(env MMS_NBV_CONV_STALL_N)   # 연속 정체 횟수
STAGE_DUMP      = os.environ.get("MMS_SIM_STAGE_DUMP", "")
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
        # nbv 자세 선택기(공용). visited 를 세션 동안 들고 있어 같은 자세를
        # 반복 선택하지 않는다 — az 를 '관절이동 최소'로 고르므로 직전 자세의
        # 이동비용이 0 이라 넘기지 않으면 무한 반복한다.
        # ★ `up_sign` 은 축을 잡은 뒤에 넣는다(아래) — 여기서는 아직 모른다.
        self._nbv = _NbvPlanner(joint_weights=DEFAULT_JOINT_WEIGHTS,
                                el_floor_deg=VIEW_EL_DEG,
                                view_azis_deg=VIEW_AZIS_DEG,
                                ensure_els=ENSURE_ELS,
                                log=lambda m: print(f"[isaac_scan] {m}"))
        self._last_roll = None                      # _view_q 가 채택한 roll(로그용)
        # ★ 스캐너 시점 스냅샷 (output/<RUN>/debug/cam/<단계>/). "왜 점이 안 들어왔나" 를 숫자로만
        #   쫓다 하루를 쓴 뒤 넣었다 — 그림 한 장이면 카메라가 딴 데 보는 걸 바로 안다.
        from utils.debug_view import DebugViewSaver
        self._dbg = DebugViewSaver(log=lambda m: print(f"[isaac_scan] {m}"))
        # 자가충돌 = 실제 메시 판정(캐시 없으면 None → 캡슐 폴백)
        self._mesh_self = _mesh_sc.get_default(margin_m=SELF_CLEAR_M)
        if self._mesh_self is not None:
            print(f"[isaac_scan] 자가충돌 = 실제 메시 판정 (여유 {SELF_CLEAR_M*1000:.0f}mm)")
        # 셀 구조물(벽·상판·저울·툴스탠드) — 기존 world 는 턴테이블/프레임만 있었다.
        # ★ 단계 무관 **단일 충돌 게이트**. 로봇=메시, 환경/링크=SDF, 경로=보수적 전진.
        #   예전엔 nbv 만 검사하고 lookaround·flip 은 무검사였다(docs/collision.md).
        self._cm = _colmodel.get_default(self_margin_m=SELF_CLEAR_M,
                                         env_margin_m=ENV_CLEAR_M)
        self._env_mesh = _env_col.get_default(margin_m=ENV_CLEAR_M)
        if self._env_mesh is not None:
            print(f"[isaac_scan] 셀 구조물 충돌 = 실제 메시 {len(self._env_mesh.env)}점 "
                  f"(여유 {ENV_CLEAR_M*1000:.0f}mm)")
        # ★ 턴테이블 축 = **sim USD** 에서 읽되 **곧바로 base 로 옮긴다.**
        #
        #   2026-09-17 이전에는 sim 이 crop·누적·계획을 전부 **world** 에서 했다.
        #   real 은 base 에서 한다. 같은 공용 코드에 서로 다른 프레임을 먹이니
        #   `up_sign` 분기가 생겼고, sim 은 +1 경로만 돌아 **real 전용 경로가 sim
        #   에서 한 번도 검증되지 않았다** — "sim 에서는 되는데 real 에서만" 의
        #   구조적 원인이다(실측: el 기준축을 안 넘겨 real 3방위 IK 전멸).
        #   이제 sim 도 base 에서 돌아 같은 분기를 탄다. USD 는 경계에서만 읽는다.
        mn, mx = self._aabb_world(TURNTABLE_MESH)
        _axis_w = np.array([(mn[0]+mx[0])/2.0, (mn[1]+mx[1])/2.0, mx[2]])  # disc 표면중심(world)
        self.axis_b = self._world_to_base(_axis_w)            # 턴테이블 축점(base)
        _ad = self.T_WB[:3, :3].T @ np.array([0.0, 0.0, 1.0])  # world 수직 → base
        self.axis_dir_b = _ad / (np.linalg.norm(_ad) + 1e-12)
        # ★ base 에서 어느 z 가 '위'인가. 이 셀은 천장 마운트라 −1 이 된다
        #   (`docs/collision.md` §6.1). real 의 `_view_up_B()` 와 같은 뜻이다.
        self.up_sign = -1.0 if float(self.axis_dir_b[2]) < 0 else +1.0
        # ★ `axis_dir_b` 자체가 이미 **물리적 '위'** 다(world +Z 를 base 로 옮긴 것).
        #   여기에 `up_sign` 을 곱하면 안 된다 — 곱하면 항상 +Z 가 되어 천장 마운트
        #   에서 카메라를 **바닥 쪽**으로 보낸다(실측: IK 전멸, preview 0점).
        #   `up_sign` 은 "base 의 +Z 가 위냐 아래냐" 를 알려주는 **부호**일 뿐이고,
        #   `up_vec_b` 는 **방향 벡터**다. 둘은 같은 것이 아니다.
        #   real 의 `_view_up_B()` 도 축 방향을 로봇 쪽으로 맞춘 그 벡터를 준다.
        self.up_vec_b = self.axis_dir_b.copy()                # base 에서 '위' 단위벡터
        print(f"[isaac_scan] 턴테이블 축(sim USD) base={np.round(self.axis_b,3).tolist()} "
              f"up_sign={self.up_sign:+.0f} (base +Z 가 "
              f"{'아래' if self.up_sign < 0 else '위'})")
        # gap 분류(위/아래/측면)·아랫면 제외가 이 부호에 달렸다 — real 과 같은 값.
        self._nbv.up_sign = float(self.up_sign)
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
        self._obj_prim = prim                                 # flip 대상(flip)
        mn, mx = self._aabb_world(prim)                       # 객체 world AABB
        # ★ 8 모서리를 전부 base 로 옮겨 다시 AABB 를 잡는다. z 만 변환하면 base 가
        #   yaw 를 가질 때 x·y 범위가 틀린다.
        _cw = np.array([[x, y, z] for x in (mn[0], mx[0])
                        for y in (mn[1], mx[1]) for z in (mn[2], mx[2])])
        _cb = self._world_to_base(_cw)
        _lo, _hi = _cb.min(0), _cb.max(0)
        self.obj_center_b = (_lo + _hi) / 2.0                 # 객체 중심(base)
        self.obj_zlo_b, self.obj_zhi_b = float(_lo[2]), float(_hi[2])
        self.obj_radius = float(max(_hi[0] - _lo[0], _hi[1] - _lo[1]) / 2.0)
        self.obj_height = float(_hi[2] - _lo[2])
        # 물체 '꼭대기' 는 up_sign 방향 끝이다 (base 가 +Z 아래면 z 최소).
        _top_z = self.obj_zhi_b if self.up_sign > 0 else self.obj_zlo_b
        self.obj_top_b = np.array([self.obj_center_b[0], self.obj_center_b[1], _top_z])
        # flip 은 **절대각**(원래 기준)이므로 치수도 항상 **원본**에서 계산해야 한다.
        # 이전에는 1차 flip 이 덮어쓴 값을 2차가 원본으로 착각해 크롭 창이 틀렸다
        # (180° 인데 126mm 창을 써서 원판 점이 섞였다).
        self._obj0 = dict(center=self.obj_center_b.copy(), radius=self.obj_radius,
                          zlo=self.obj_zlo_b, zhi=self.obj_zhi_b)
        # 자료용 3인칭 녹화 (MMS_SIM_REC=1 일 때만). 턴테이블 회전 스텝에 훅을 건다.
        self._rec = _Recorder(self.stage)
        try:
            from mms_artec.backends.isaac import isaac_turntable as _tt
            _tt.set_recorder(self._rec if self._rec.ok else None)
        except Exception:                                # noqa: BLE001
            pass
        off = float(np.linalg.norm(self.obj_center_b[:2] - self.axis_b[:2]))   # 축 이탈(수평)
        print(f"[isaac_scan] 객체='{prim}' r={self.obj_radius*1000:.0f}mm h={self.obj_height*1000:.0f}mm "
              f"center_b={np.round(self.obj_center_b,3).tolist()} "
              f"z_b=[{self.obj_zlo_b:.3f},{self.obj_zhi_b:.3f}]")
        print(f"[isaac_scan] ⚠ 객체-턴테이블축 수평이탈={off*1000:.0f}mm "
              f"(클수록 lookaround 고정카메라가 회전 중 객체를 놓침→sector 구멍)")

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

    # ── 캡처(입사각 스캐너 모델) — **base 프레임** ───────────────────────────
    def _cam_pos_base(self):
        """카메라 원점의 **base** 좌표 (m). 실패하면 None.

        ★ USD 카메라 prim(GT)이 아니라 **FK + 믿는 T_EC** 로 만든다 — real 이
          `_T_CB` 로 하는 것과 같은 경로다. 예전에는 sim 만 GT prim 을 읽어서
          캘리브 오차 주입(`MMS_SIM_TEC_ERR_MM`)이 거리추종·입사각 판정에
          반영되지 않았다. sim 이 real 보다 유리한 정보를 쓰면 검증이 안 된다.

        None 은 **그 프레임만** 건너뛰게 한다(계약 문제가 아니다 —
        `lookaround._unpack_preview` 주석). 조용히 넘어가지 않도록 한 번은 찍는다.
        """
        try:
            T_EB = self.robot.get_ee_pose_mat()               # E→B (m)
            return (T_EB @ np.linalg.inv(self.T_EC))[:3, 3]   # camera in base
        except Exception as e:                                # noqa: BLE001
            if not getattr(self, "_cam_pos_warned", False):
                self._cam_pos_warned = True
                print(f"[isaac_scan] ⚠ 카메라 위치 계산 실패"
                      f"({type(e).__name__}: {e}) — 이 프레임 거리보정 건너뜀")
            return None

    def _capture_obj_base(self, log=False):
        """capture_points_base → 객체 crop + (옵션)입사각 필터. 반환 = **base** 점군.

        ★ 스캐너가 이미 base 점군을 준다. 예전에는 여기서 world 로 올렸다가
          계획·누적·크롭을 전부 world 에서 했는데, real 은 base 에서 한다 —
          같은 공용 코드에 다른 프레임을 먹이던 자리다. 변환을 없앴다.
        """
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
        pc = pc_b                                             # 이미 base
        c = self.obj_center_b
        r = np.linalg.norm(pc[:, :2] - c[:2], axis=1)
        # ★ 턴테이블 원판 평면을 배제한다. 원판 상면(axis_b[2])에 붙은 점은 물체가
        #   아니라 **원판**이다 — 물체가 그 위에 앉아 있으므로 물체의 진짜 최하단은
        #   원판보다 위에 있고, 어차피 원판에 가려 스캔되지 않는다.
        #   flip 에서 컵을 뒤집으면 **열린 면으로 원판이 그대로 보여** 대량 혼입된다
        #   (실측: 180° 패스가 +68,899점 = 정상 패스의 3.5배 → 메시 파손).
        #   ★ 부호는 `up_sign` 이 정한다 — base 는 +Z 가 아래라, '원판 위' 가
        #     z 증가가 아니다. 'u' 좌표(클수록 위)로 바꿔 프레임과 무관하게 쓴다.
        u = self.up_sign
        zu = pc[:, 2] * u
        z_floor_u = float(self.axis_b[2]) * u + DISC_REJECT_M
        obj_lo_u = min(self.obj_zlo_b * u, self.obj_zhi_b * u)
        obj_hi_u = max(self.obj_zlo_b * u, self.obj_zhi_b * u)
        m = ((r < self.obj_radius + 0.02)
             & (zu > max(obj_lo_u - 0.005, z_floor_u))
             & (zu < obj_hi_u + 0.02))
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

    def _incidence_filter(self, pts_b):
        """입사각 필터 — 법선은 **대표점 부분집합**에서 PCA 로 재고, 판정을 최근접
        대표점에서 전파한다.

        ★ 프레임마다 전 점(2mm 복셀, 수천 점)에 k=12 PCA 를 돌리면 lookaround 프레임
          처리시간의 16%(세제 1,440프레임 = 31s)였다(2026-09-18 프로파일). 법선은
          국소 면 방향이라 4mm 격자 대표점으로 재도 같고, 입사각 판정은 점이 아니라
          면의 성질이므로 최근접 대표점의 판정을 그대로 물려받아도 된다.
          대표점 상한 `INCIDENCE_PCA_MAX`(1,500). 그 이하면 예전과 완전히 같다.
        """
        _cam = self._cam_pos_base()
        if _cam is None:
            return pts_b
        P = np.asarray(pts_b, float)
        if len(P) > INCIDENCE_PCA_MAX:
            rep = _voxel(P, VOXEL_M * 2.0)                 # 4mm 대표점
            if len(rep) > INCIDENCE_PCA_MAX:
                rep = rep[np.random.default_rng(0).choice(len(rep), INCIDENCE_PCA_MAX, replace=False)]
        else:
            rep = P
        nrm = _pca_normals(rep, c_ref=self.obj_center_b)
        if nrm is None:
            return pts_b
        vd = _cam[None, :] - rep
        vd /= (np.linalg.norm(vd, axis=1, keepdims=True) + 1e-9)
        cos_inc = np.sum(nrm * vd, axis=1)
        lim = (MAX_INCIDENCE_INNER_DEG if getattr(self, "_gap_pass", False)
               else MAX_INCIDENCE_DEG)
        ok_rep = cos_inc > math.cos(math.radians(lim))
        if rep is P:
            return P[ok_rep]
        from scipy.spatial import cKDTree
        _, j = cKDTree(rep).query(P, k=1, workers=-1)   # 판정 전파
        return P[ok_rep[j]]

    # ── 충돌 (공용 robot_collision + 스캐너 mesh) ────────────────────────────
    def _build_world(self) -> CollisionWorld:
        w = CollisionWorld.from_turntable(
            surface_point=self.axis_b, axis_dir=self.axis_dir_b,
            disc_radius=TT_DISC_RADIUS_M, body_height=TT_BODY_H_M, margin=COLLISION_MARGIN_M)
        if KEEPOUT_ENABLE:
            # 원판에서 '위'로 KEEPOUT_HEIGHT_M — base 가 +Z 아래면 z 가 줄어든다.
            _k0 = float(self.axis_b[2])
            _k1 = _k0 + self.up_sign * KEEPOUT_HEIGHT_M
            w.add_cylinder("keepout", self.axis_b[:2], min(_k0, _k1), max(_k0, _k1),
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
    def _snap(self, tag: str, note: str = "", stage: str = None) -> None:
        """현재 스캐너 시점을 PNG 로 남긴다 (실패해도 스캔은 계속).

        `stage="preview"` 면 `…/debug/cam/preview/`, 미지정이면 `.../lookaround/`.
        """
        if not getattr(self, "_dbg", None) or not self._dbg.enabled:
            return
        try:
            cam = self.world.ensure_camera()
            self._dbg.save(cam.get_rgba(), tag, note, stage=stage)
        except Exception as e:                                # noqa: BLE001
            print(f"[isaac_scan] [dbgview] ⚠ 스냅 실패({type(e).__name__}: {e})")

    # ── 시각화 경계 — 씬(USD)은 world 라 여기서만 되돌린다 ──────────────
    def _viz_pts(self, pts_b):
        """base 점군 → world (오버레이용). 시각화 외에는 쓰지 말 것."""
        return self._base_to_world(np.asarray(pts_b, float))

    def _viz_axis_xy(self):
        """오버레이가 쓰는 턴테이블 축 xy (world)."""
        return self._base_to_world(self.axis_b)[:2]

    def _accumulate(self, pts_b, theta):
        """base 점군을 턴테이블 축(base) 둘레 −θ 역회전 → canonical(base) 누적."""
        if len(pts_b) == 0:
            return
        canon = _rot_about_axis(pts_b, self.axis_b, self.axis_dir_b, -theta)
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
            from utils.nbv.icp_strategy import refine_to_master
            if ICP_DUMP and tag == "flip":      # 오프라인 분석용 입력 덤프
                np.savez_compressed(ICP_DUMP, src=_voxel(pts, VOXEL_M), tgt=master)
                print(f"[isaac_scan]   (덤프) ICP 입력 → {ICP_DUMP}")
            # ★ 정합은 **sim·real 공용** `refine_to_master` — 다단 point-to-plane
            #   ICP + 3중 게이트 + 팽창 게이트. flip 은 이동 허용치만 넓다(함수 안).
            #   예전엔 이 루프가 sim 에만 있었고 real 은 colored ICP 로 달랐다
            #   (2026-09-18 공용화). 게이트 완화 금지 근거는 함수 주석에 있다.
            res = refine_to_master(
                _voxel(pts, VOXEL_M), master, np.eye(4),
                mode=("flip" if tag == "flip" else "patch"),
                voxel_m=0.0, on_step=self._pump,
                log=(lambda m: print(f"[isaac_scan]   {m}")) if ICP_SCALE_DEBUG else None)
            if res is not None and res.ok:
                T = res.T_refined
                return pts @ T[:3, :3].T + T[:3, 3], res
            return pts, res
        except Exception as e:                       # noqa: BLE001
            print(f"[isaac_scan]   ⚠ ICP 예외({type(e).__name__}: {e}) — 기구학 그대로")
            return pts, None

    def _master_ds(self):
        """누적 master 의 다운샘플 캐시.

        ⚠ 예전 구현은 `n != 이전 n` 으로 무효화했는데, 프레임마다 점이 늘어 **항상**
          다시 만들었다 — 즉 캐시가 아니었다. nbv 처럼 누적이 30만점을 넘으면
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

    def _unflip(self, pts_b):
        """flip 된 물체의 점을 **명목 회전(hint)** 으로 canonical 에 되돌린다.

        실물은 사람이 뒤집으므로 실제 자세를 못 읽는다 → hint 만 쓸 수 있다.
        남는 잔차(사람 손 오차 / sim 물리 정착)는 **ICP 가 메운다**. real 과 같은 구조.
        """
        M = getattr(self, "_flip_hint", None)
        if M is None or len(pts_b) == 0:
            return pts_b
        Mi = np.linalg.inv(M)
        return pts_b @ Mi[:3, :3].T + Mi[:3, 3]

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
                  f"[{self.obj_zlo_b:.3f},{self.obj_zhi_b:.3f}]  "
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
                                     drift_trans_m=0.030, drift_rot_deg=10.0,
                                     max_inflation_m=ICP_INFLATION_M)
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
    def _view_q(self, target_b, el_deg, az_deg, standoff, seed, rolls=None):
        """target(**base**) 을 el/az/standoff 에서 보는 카메라 → 해석 IK q.
        (q, eye_w) or (None, eye_w).

        ★ 실제 계산은 **sim·real 공용** `utils/robot/view_pose.solve_view_q` 가 한다.
          예전에는 이 함수(sim)만 roll 6방향·시드 8개를 쓰고 real 은 각각 1개였다 —
          같은 solver 를 쓰면서도 sim 에서 되는 자세가 real 에서 버려졌다.
          카메라 규약은 데이터로 넘긴다(sim=USD, real=OpenCV). 근거: docs/collision.md §2.
        """
        # ★ 타깃은 **base** 다 → `T_WB=None`. el 의 기준축(`up`)과 az 기준
        #   (`az_ref`)도 real 과 **같은 방식**으로 넘긴다. 예전에는 sim 만 world 로
        #   풀어서(+Z 가 위) 이 두 인자가 필요 없었고, 그래서 real 전용 경로가
        #   sim 에서 한 번도 안 돌았다 — `eye_from_el_az` 주석의 그 버그다.
        q, roll, eye_w = _vp.solve_view_q(
            kin, target_b, el_deg, az_deg, standoff, seed, self.T_EC,
            T_WB=None, convention=_vp.CAM_USD,
            rolls_deg=(VIEW_ROLLS_DEG if rolls is None else rolls),
            n_seed_alt=IK_SEED_TRIES,
            up=self.up_vec_b, world_up=self.up_vec_b,
            az_ref=-np.asarray(target_b, float))
        self._last_roll = roll
        return q, eye_w

    def _gap_roll_order(self, target_b, el_deg, az_deg, standoff, gaps):
        """gap 방향에 FOV 넓은 축을 맞추는 roll 순서 (공용 view_pose 위임).
        `NBV_ROLL_ALIGN=0` 이면 기본 순서(roll=0 우선) — **대조군 스위치**."""
        if not NBV_ROLL_ALIGN or not gaps:
            return tuple(VIEW_ROLLS_DEG)
        eye = _vp.eye_from_el_az(target_b, el_deg, az_deg, standoff,
                                 up=self.up_vec_b)
        return _vp.roll_order_for_gaps(
            [c.p_O for c in gaps], [c.L for c in gaps], eye, target_b,
            convention=_vp.CAM_USD, rolls_deg=VIEW_ROLLS_DEG)

    def _drive(self, q, steps=None) -> bool:
        """현재→q 이동. **막히면 우회 경로**를 계획해 따라간다. 반환=이동했는가.

        ★ 예전에는 무조건 직선 보간으로 갔다 — lookaround·flip 은 충돌 검사조차 없었고,
          nbv 도 '막히면 그 자세를 버리는' 식이라 **돌아가면 되는 자세를 잃었다**
          (실측: 안전 자세 12개 중 직선이 막힌 쌍이 6개, 전부 우회 성공).
          계획도 실패하면 **움직이지 않고 False** 를 돌려 상위가 그 패스를 건너뛰게 한다.
        """
        if steps is None:
            steps = (DRIVE_STEPS_REC if getattr(self, "_rec", None) is not None
                     and self._rec.ok else DRIVE_STEPS)
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
                if getattr(self, "_rec", None) is not None:
                    self._rec.tick()             # 이동 중에도 프레임 적립
        self.world.step(4, render=True)             # 도착 후 정착
        if getattr(self, "_rec", None) is not None:
            self._rec.tick()
        return True

    # ── 거리추종 (el 고정 · 축거리만 창 중앙으로) ────────────────────────
    def _guard_preview_volume(self, el_deg: float) -> None:
        """preview **전에** 축 둘레에 보수적 원기둥을 장애물로 걸어 둔다.

        근거·함정은 공용 `lookaround.guard_cylinder_points` 주석에 있다.
        여기서 정하는 것은 두 가지뿐이다:

          · 반경 = **원판 반경**(물체는 원판 위에 서 있으니 그보다 넓을 수 없다).
            `PREVIEW_RADIUS_MAX_M`(180mm)을 쓰면 실효 keepout 이 탐침 사다리의
            220mm 칸을 잡아먹는다 — 실측: keepout 210mm → 최소 축거리 242mm.
          · 마진 = **0**. 이 원기둥은 이미 "있을 수 있는 최대" 라 그 위에 또
            안전여유를 얹으면 이중보정이다.
        """
        if self._cm is None:
            return
        try:
            _R = min(p1.PREVIEW_RADIUS_MAX_M, TT_DISC_RADIUS_M)
            blocks, d_min = p1.guard_blocks_probe(_R, 0.0, el_deg,
                                                  p1.probe_bounds(self._p1_dof)[0])
            if blocks:
                print(f"[isaac_scan] ⚠ 보호 원기둥(r={_R*1000:.0f}mm)이 탐침을 막는다 "
                      f"— el={el_deg:.0f}° 에서 축거리 {d_min*1000:.0f}mm 아래로 못 간다. "
                      f"그 구간 탐침은 **로봇이 가지도 않고** 빈 캡처로 보인다")
            pts = p1.guard_cylinder_points(self.axis_b, self.up_sign, _R)
            self._cm.set_dynamic_obstacle(pts, margin_m=0.0)
            _viz.show_obstacle(self.stage, self._viz_pts(pts), tag="guard",
                               color=(0.2, 0.5, 1.0))          # 파랑 = 가정치
            print(f"[isaac_scan]   preview 보호 원기둥 — r={_R*1000:.0f}mm 마진 0 "
                  f"({len(pts):,}pt, 최소 축거리 {d_min*1000:.0f}mm)")
        except Exception as e:                                # noqa: BLE001
            print(f"[isaac_scan] ⚠ preview 보호 원기둥 실패({type(e).__name__}: {e})")

    def _pause_to_inspect(self, what: str) -> None:
        """GUI 에서 **다음 단계로 넘어가기 전에 잠깐 멈춰** 눈으로 보게 한다.

        `MMS_SIM_PAUSE` 초(기본 0 = 안 멈춤). 헤드리스면 무시한다 — 볼 창이 없다.
        오버레이를 그려놓고 바로 다음 단계로 넘어가면 화면이 지나가 버린다.
        """
        try:
            sec = float(os.environ.get("MMS_SIM_PAUSE", "0"))
        except ValueError:
            sec = 0.0
        if sec <= 0 or self.world.headless:
            return
        print(f"[isaac_scan]   ⏸ {sec:.0f}초 — {what}")
        import time as _t
        t0 = _t.time()
        while _t.time() - t0 < sec:                # 렌더를 계속 돌려야 창이 산다
            self.world.step(4, render=True)

    def _register_object_obstacle(self, pts_b, tag: str) -> None:
        """**base** 점군을 스캔 대상 장애물로 등록한다 (실패해도 스캔은 계속).

        마진은 `collision_model.SCAN_OBSTACLE_MARGIN_M` 한 곳에서 온다 — real 과
        갈라지지 않게. 더 정확한 점군이 생기면(누적 스캔) 다시 불러 갱신한다.
        """
        if self._cm is None or pts_b is None or len(pts_b) == 0:
            return
        try:
            pts_b = np.asarray(pts_b, float)
            # ★ 등록하는 점군은 **θ=0 canonical** 이다(`_accumulate` 가 −θ 로
            #   되돌려 쌓는다). 그런데 로봇이 움직이는 시점의 턴테이블 각은 0 이
            #   아니다 — nbv 는 부분 스윕이라 특히 그렇다. 비대칭 물체(손잡이 달린
            #   드릴·주전자)면 장애물이 **실제와 다른 방향을 향한 채** 걸린다.
            #   그래서 축 둘레 **회전체 외피**로 등록한다(높이별 최대 반경).
            #   이산 N방향 샘플(`swept_about_axis`)과 달리 각도 틈이 원리적으로
            #   없고, 고정 원기둥(원판 반경)과 달리 물체만큼만 굵어서 고도각 높은
            #   자세(nbv·flip·뚜껑)를 기각하지 않는다 — 근거는 그 함수 주석.
            swept = p1.revolution_envelope(pts_b, self.axis_b, self.axis_dir_b)
            self._cm.set_dynamic_obstacle(swept)
            # ★ **넘어간 바로 그 점**을 그린다. bbox·점 수만으로는 회전체가 물체를
            #   제대로 감쌌는지, 프레임이 맞는지 알 수 없다 — 비대칭 물체에서
            #   장애물이 엉뚱한 방향을 향해도 숫자는 그럴듯하게 나온다.
            _viz.clear_obstacle(self.stage, "guard")       # 가정치는 치운다
            _viz.show_obstacle(self.stage, self._viz_pts(swept), tag=tag)
            # ★ 나중에 오프라인으로 겹쳐 보려고 덤프한다(GUI 를 못 띄울 때).
            #   `scripts/sim/overlay_env_npz.py` 와 같은 방식으로 씬에 올린다.
            if os.environ.get("MMS_SIM_OBSTACLE_DUMP", "1") == "1":
                try:
                    from utils import PROJECT_ROOT
                    d = os.path.join(str(PROJECT_ROOT), "output", "debug", "obstacle")
                    os.makedirs(d, exist_ok=True)
                    f = os.path.join(d, f"{self._dbg.run_ts}_{tag}.npz")
                    np.savez_compressed(f, raw_b=pts_b.astype(np.float32),
                                        swept_b=swept.astype(np.float32),
                                        axis_b=self.axis_b, axis_dir_b=self.axis_dir_b,
                                        T_WB=self.T_WB, margin_m=_OBS_MARGIN)
                    print(f"[isaac_scan]   장애물 덤프 → {os.path.relpath(f, str(PROJECT_ROOT))}")
                except Exception as e:                    # noqa: BLE001
                    print(f"[isaac_scan]   ⚠ 장애물 덤프 실패({e})")
            # ★ **등록된 점군**의 bbox 를 찍는다. 예전엔 원본(pts_b)을 찍었는데,
            #   그러면 회전체가 반영됐는지 로그로는 알 수 없었다 — 원통형 물체는
            #   원본과 회전체의 bbox 가 같아서 특히 구분이 안 된다.
            lo, hi = swept.min(axis=0), swept.max(axis=0)
            _r = float(np.linalg.norm(
                (swept - self.axis_b)[:, :2], axis=1).max())
            print(f"[isaac_scan]   대상물 장애물({tag}) {len(swept):,}pt "
                  f"최대반경 {_r*1000:.0f}mm · base bbox "
                  f"x[{lo[0]:.3f},{hi[0]:.3f}] y[{lo[1]:.3f},{hi[1]:.3f}] "
                  f"z[{lo[2]:.3f},{hi[2]:.3f}]")
        except Exception as e:                                # noqa: BLE001
            print(f"[isaac_scan] ⚠ 대상물 장애물 등록 실패"
                  f"({type(e).__name__}: {e}) — 계속")

    def _standoff_tracker(self, q, label):
        """이 패스용 (tracker, 시작 축거리). 자세 맥락이 없으면 (None, 0).

        맥락 = 이 q 가 어느 밴드(el/az/tz/standoff)였는가. `_pick_lookaround_planner`
        가 남긴 짝에서 **객체 동일성**으로 찾는다. nbv·flip 자세나 AT_CURRENT 는
        맥락이 없으므로 추종하지 않는다 — nbv 는 gap 을 겨냥하는 것이 목적이라
        거리를 흔들면 조준이 틀어진다.
        """
        vp = None
        for _q, _vp in getattr(self, "_band_pairs", ()):
            if _q is q:
                vp = _vp
                break
        if vp is None:
            self._retarget_fn = None
            return None, 0.0
        sensor = getattr(self, "_p1_sensor", None) or p1.SensorModel()
        seed = getattr(self, "_p1_seed", None)
        if seed is None:
            seed = np.asarray(self.robot.get_joint_angles(is_radian=True), float)
        target = np.array([self.axis_b[0], self.axis_b[1], vp.target_z], float)
        # 탐색 한계 — 창 중앙 ±120mm. 계획 거리에서 이만큼 벗어날 일이 있으면
        # 반경 추정이 통째로 틀린 것이라, 더 헤매기보다 그 자리에서 멈추는 게 낫다.
        lo = max(0.05, vp.standoff - 0.12)
        hi = vp.standoff + 0.12

        def retarget(d_new) -> bool:
            qn, _ = self._view_q(target, vp.el_deg, vp.az_deg, float(d_new), seed)
            if qn is None:
                return False
            if self._cm is not None:
                ok, _why = self._cm.is_pose_safe(qn)
                if not ok:
                    return False
            return bool(self._drive(qn))

        self._retarget_fn = retarget
        trk = _StandoffTracker(sensor.dof, lo, hi,
                               log=lambda m: print(f"[isaac_scan]{m}"),
                               # 밴드가 맡은 높이대(조준높이 ±40mm)의 면을 창 중앙에
                               # 놓는다 — 창 안 전체 중앙값은 윗부분에 끌려 물러난다.
                               core=(vp.target_z, self.up_sign, _TRACK_CORE_HALF))
        if trk.enabled:
            print(f"[isaac_scan]   거리추종 {trk.mode} — el={vp.el_deg:.0f}° "
                  f"az={vp.az_deg:.0f}° 고정, 축거리 {vp.standoff*1000:.0f}mm "
                  f"[{lo*1000:.0f}~{hi*1000:.0f}mm] → 창중앙 "
                  f"{(0.5*(sensor.dof[0]+sensor.dof[1]))*1000:.0f}mm")
        return trk, float(vp.standoff)

    def _scan_pass(self, q, n_theta, label="", stage="lookaround"):
        """★ 통합 캡처 = 로봇을 q 자세로 두고 **턴테이블 전회전**하며 프레임 캡처·−θ 누적.
        real 캡처(로봇 pose + streaming 전회전 + relocalization)에 1:1 대응. lookaround·2 공용.
        sim 은 GT θ 라 −θ 회전 = relocalization 역할(정확). 입사각 필터로 좋은 프레임만 기여."""
        # AT_CURRENT = '이동 없이 현재 자세에서 캡처'(flip 으로 물체를 뒤집은 뒤). 센티널이므로
        # 관절해로 해석하면 안 된다 — sim 은 flip 가 미지원이라 이 경로가 미검증이었다.
        if q is not AT_CURRENT and q is not None:
            if not self._drive(q):
                print("[isaac_scan]   ⚠ 이동 불가 — 이 패스 건너뜀")
                return False
        if label:
            print(f"[isaac_scan] === scan pass: {label} (전회전 {n_theta}프레임) ===")
        if getattr(self, "_rec", None) is not None:
            self._rec.begin(label)
        before = sum(len(a) for a in self.accum)
        thetas = np.linspace(0.0, 2*np.pi, n_theta, endpoint=False)
        pass_pts = []                       # 이 패스만 따로 모은다(정합 단위)
        _snap_at = self._dbg.band_frames(len(thetas)) if getattr(self, "_dbg", None) else set()
        _band = (label or "band").replace(" ", "")
        # ★ 거리추종 — el·az·tz 는 그대로, **축거리만** 작동거리 창 중앙으로.
        _trk, _d_cur = self._standoff_tracker(q, label)
        for i, th in enumerate(thetas):
            self.turntable.move_abs(float(th), float(np.radians(30.0)))
            self.turntable.wait_motion_done()
            th_act = float(self.turntable.getActualPos())
            obj = self._capture_obj_base(log=(i % 9 == 0))
            if _trk is not None and _trk.due(i):
                if _trk.mode == "band":
                    # 회전 전이라 정지 상태 — 제한을 유지한 채 여러 번 수렴시킨다.
                    _d_new = _trk.converge(
                        _d_cur,
                        lambda: (self._capture_obj_base(),
                                 self._cam_pos_base()),
                        self._retarget_fn)
                else:
                    _d_new = _trk.update(i, _d_cur, obj,
                                         self._cam_pos_base(),
                                         self._retarget_fn)
                if _d_new != _d_cur:
                    _d_cur = _d_new
                    # 보정 후 같은 θ 에서 다시 찍는다 — 움직인 자세의 프레임을
                    # 누적해야 "가까이 갔다" 가 실제 데이터로 반영된다.
                    obj = self._capture_obj_base()
            if i in _snap_at:              # 밴드마다 처음·중간·끝
                _pos = {0: "start", len(thetas)//2: "mid"}.get(i, "end")
                self._snap(f"{_band}_{_pos}_f{i:03d}",
                           f"{_band} {_pos} f{i} th={np.degrees(th_act):.0f}deg pts={len(obj)}")
            if len(obj):
                canon = _rot_about_axis(obj, self.axis_b, self.axis_dir_b, -th_act)
                if stage != "flip":
                    canon = self._unflip(canon)      # flip 는 아래 전역정합이 담당
                # flip 패스는 프레임 하나의 중첩이 너무 적어 프레임 ICP 가 불안정하다
                # → 패스 전체를 모아 한 번에 정합한다(중첩 확보).
                if ICP_LEVEL == "frame" and stage != "flip":
                    self._merge_frame(canon)      # 프레임마다 master 에 재고정
                else:
                    pass_pts.append(canon)
            if _viz.ENABLED and self.accum and (i % VIZ_EVERY == 0):
                _viz.show_accum(self.stage, self._viz_pts(np.vstack(self.accum)),
                                max_pts=VIZ_MAX_PTS,
                                theta=th_act, axis_xy=self._viz_axis_xy())
        if ICP_LEVEL != "frame" or stage == "flip":
            self._merge_pass(pass_pts, force_icp=(stage == "flip"))
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
            _viz.show_accum(self.stage, self._viz_pts(np.vstack(self.accum)))
        added = sum(len(a) for a in self.accum) - before
        if _trk is not None and _trk.enabled:
            print(f"[isaac_scan]   {_trk.summary()} · 최종 축거리 {_d_cur*1000:.0f}mm")
        # ★ 이 패스가 **실제로 덮은 z 폭**을 찍는다. 계획기의 `band_h`(모델 예측)와
        #   비교하면 `BAND_OVERLAP` 이 흡수하려는 그 비율이 그대로 나온다 —
        #   "모델 149mm vs 실물 87mm (=0.58)" 를 추측이 아니라 매 실행에서 잰다.
        try:
            if added > 0 and self.accum:
                _new = np.vstack(self.accum[-max(1, len(self.accum) - 0):])[-added:]
                _sp = p1.contiguous_z_span(_new[:, 2], np.ones(len(_new), bool))
                print(f"[isaac_scan]   pass 완료 (+{added}점, 총 {before+added}) "
                      f"· 실제 덮은 z폭 {_sp*1000:.0f}mm")
            else:
                print(f"[isaac_scan]   pass 완료 (+{added}점, 총 {before+added})")
        except Exception:                                     # noqa: BLE001
            print(f"[isaac_scan]   pass 완료 (+{added}점, 총 {before+added})")
        return True

    def _scan_patch(self, q, theta_c, label="", span_deg=None, n_frames=None):
        """**부분 스윕** — 목표 θ 주변 좁은 구간만 돌며 캡처한다.

        nbv 의 목적은 '특정 결손면 채우기'다. 그 면이 보이는 각도 구간만 돌면 되고,
        한 바퀴를 도는 동안 이미 가진 면을 다시 보는 것은 순수한 낭비다.
        (lookaround·flip 은 전면 커버가 목적이라 전회전 `_scan_pass` 를 그대로 쓴다.)
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
        if getattr(self, "_rec", None) is not None:
            self._rec.begin(label)
        import time as _t
        for i, th in enumerate(np.linspace(theta_c - span/2, theta_c + span/2, n)):
            _tm = _t.time()
            self.turntable.move_abs(float(th % (2*np.pi)), float(np.radians(30.0)))
            self.turntable.wait_motion_done()
            self._tick("turntable", _tm)
            th_act = float(self.turntable.getActualPos())
            import time as _t
            _tc = _t.time()
            obj = self._capture_obj_base(log=(i == 0))
            self._tick("capture", _tc)
            if len(obj):
                _tm2 = _t.time()
                canon = _rot_about_axis(obj, self.axis_b, self.axis_dir_b, -th_act)
                patch_pts.append(self._unflip(canon))
                self._tick("merge", _tm2)
            # 실시간 오버레이 — 프레임마다 갱신해 nbv 가 무엇을 채우는지 바로 보인다
            # (예전엔 계획 시점에만 갱신돼 캡처 중에는 고정된 것처럼 보였다).
            # 오버레이는 **간헐 갱신**. USD 에 수만 점을 매 프레임 쓰면 렌더보다 비싸다.
            if _viz.ENABLED and (i % VIZ_EVERY == 0) and (self.accum or patch_pts):
                _tv = _t.time()
                _viz.show_accum(self.stage,
                                self._viz_pts(np.vstack(self.accum + patch_pts)),
                                max_pts=VIZ_MAX_PTS,
                                theta=th_act, axis_xy=self._viz_axis_xy())
                self._tick("viz", _tv)
            if PROFILE_EVERY and (i + 1) % PROFILE_EVERY == 0:
                self.profile_report()          # 병목 추적용 (MMS_SIM_PROFILE_EVERY=0 이면 끔)
        # ★ 정합 단위 = **패치 전체**. 프레임 단위로 붙이면 부분 스윕은 프레임 간
        #   중첩이 부족해 ICP 가 흔들리고 메시가 조각난다(실측: gaps 20→418).
        #   패치를 한 덩어리로 모으면 master 와의 중첩이 충분해 안정적으로 붙는다.
        if patch_pts:
            pts = np.vstack(patch_pts)
            raw = pts
            aligned, res = self._icp_to_master(pts, ICP_MIN_PTS, "patch")
            if res is not None and res.ok:
                print(f"[isaac_scan]   패치 정합: Δt={res.delta_translation_m*1000:.2f}mm "
                      f"Δr={res.delta_rotation_deg:.2f}° fitness={res.fitness:.2f}")
                pts = aligned
            elif res is not None:
                print(f"[isaac_scan]   ⚠ 패치 정합 게이트 실패({res.reason}) — 기구학 그대로")
            self.accum.append(pts)
            if getattr(self, "_nbv_dbg", None) is not None:      # 겨냥·수집·정합 기록
                self._nbv_dbg.patch(raw_pts=raw, aligned_pts=pts, result=res)
        elif getattr(self, "_nbv_dbg", None) is not None:
            self._nbv_dbg.patch(raw_pts=np.zeros((0, 3)), aligned_pts=np.zeros((0, 3)), result=None)
        if _viz.ENABLED and self.accum:
            _viz.show_accum(self.stage, self._viz_pts(np.vstack(self.accum)))   # θ=0 기준으로 되그림
        added = sum(len(a) for a in self.accum) - before
        print(f"[isaac_scan]   patch 완료 (+{added}점, 총 {before+added})")
        return True

    # ── ScanBackend 프리미티브 (공용 utils/nbv/scan_stage_controller) ──────
    # 순서/게이팅/NBV 루프는 공용 컨트롤러 소유. 여기는 sim 캡처 하드웨어만.
    def confirm_start(self) -> bool:
        return True                                   # sim: 사람 확인 불필요

    def go_home(self) -> None:
        try:
            self.robot.go_home(sensor="artec", confirm=False)
            self.world.step(6, render=True)
        except Exception as e:
            print(f"[isaac_scan] ⚠ go_home 실패({e}) — 현재자세로 진행")

    def pick_lookaround_pose(self):
        """lookaround 시점선정 (E2E, 2026-07-03) — **real 과 동일 경로**:
        거리스텝 preview 캡처(실제 카메라) → 기하 크롭 → maximin 플래너.
        반환 = q 또는 [q,...](밴드 계획, 공용 컨트롤러가 대역별 전회전).
        플래너가 실패하면 AT_CURRENT(현재 자세에서 캡처) — real 과 같은 폴백이다."""
        self.go_home()
        seed = np.asarray(self.robot.get_joint_angles(is_radian=True), float)
        self.home_q = seed.copy()                    # retract-approach 경유점(known-good home)
        # 카메라 warm-up (replicator 첫 프레임 빈 점군 방지) — preview 전에 필요.
        self.world.ensure_camera()
        self.world.step(30, render=True)
        _ = self.scanner.capture_points_base(self.robot, self.mms._T_EC, settle=4)

        try:
            plan_qs = self._pick_lookaround_planner(seed)
            if plan_qs:
                return plan_qs
            print("[isaac_scan] ⚠ lookaround 플래너 실패 — 현재 자세에서 캡처")
        except Exception as e:
            print(f"[isaac_scan] ⚠ lookaround 플래너 예외"
                  f"({type(e).__name__}: {e}) — 현재 자세에서 캡처")
        return AT_CURRENT

    # ── lookaround 시점선정: real 경로 (preview → 크롭 → 플래너) ────────────
    def _pick_lookaround_planner(self, seed):
        """계획용 preview 수집(거리스텝×턴테이블 0/90°×조준높이) → 기하 크롭
        (캘리브 축+디스크상단만 사용, GT bbox 미사용) → plan_lookaround_viewpoints
        → 자세별 az-sweep IK. real 은 이 함수의 캡처 호출만 Artec preview 로 바뀜."""
        axis_xy = self.axis_b[:2]
        disc_top = float(self.axis_b[2])
        # ★ `dof` 는 **스캐너에게 물어본다** — real 과 같은 경로(`sensor.scanning_range()`).
        #   sim 이 이걸 안 물으면 real 만 실측 창을 쓰고 sim 은 하드코딩 기본값으로 가서
        #   **두 백엔드가 다른 가정으로 밴드를 나눈다.** sim 스캐너가 SDK 와 같은 계약을
        #   모사한다(`IsaacArtecScanner.scanning_range`).
        sensor = p1.SensorModel()
        try:
            _near, _far = self.scanner.scanning_range()
            sensor = p1.sensor_from_scanning_range(
                _near, _far, sensor, log=lambda m: print(f"[isaac_scan]{m}"))
        except Exception as e:                                  # noqa: BLE001
            print(f"[isaac_scan] ⚠ 스캔 범위 조회 실패({type(e).__name__}: {e}) — dof 기본값")
        # el_prev — 구값 25.0 은 roll 고정 탓에 IK 전부 실패했다(→ preview 0점). roll 을
        # 풀어 도달성이 확인된 30 으로. 낮을수록 물체 실루엣·높이가 잘 잡힌다.
        # ★ 거리격자는 **공용 코드가 작동거리 창에서 유도**한다(`preview_grid`).
        #   여기서 하드코딩하면 real 과 갈라진다 — 실제로 갈라져 있었다.
        el_prev = _envf("MMS_SIM_PREVIEW_EL", 30.0)

        def preview_at(tz, d):
            """조준높이 tz·축거리 d 로 구동 후 preview 캡처 → **base** 점군(기하 크롭).

            반환 = (점군, 카메라위치) 둘 다 base. 프레임을 주석에서 틀리면 이
            코드베이스에서 반복해서 사고가 났다 — 바뀌면 여기부터 고칠 것.
            """
            _n_ik = _n_move = 0
            for azd in VIEW_AZIS_DEG:                # 도달 azimuth 스윕 (커버리지 무관)
                q, _ = self._view_q(np.array([axis_xy[0], axis_xy[1], tz]),
                                    el_prev, azd, d, seed)
                if q is None:
                    _n_ik += 1
                    continue
                # ★ 이동이 거부되면 **그 자리에서 찍지 않고** 다음 az 를 시도한다.
                #   예전에는 _drive() 반환값을 버려서, 충돌로 못 간 자세의 프리뷰를
                #   원래 자리에서 찍고 그걸 계획 입력으로 썼다(밴드 계획 오염).
                if not self._drive(q):
                    _n_move += 1
                    continue
                pc_b = self.scanner.capture_points_base(
                    self.robot, self.mms._T_EC, settle=CAPTURE_SETTLE)
                # ★ ① 원시 정점 문턱 — **크롭 전**에 건다. real 은 예전부터
                #   `adaptive_min_preview_verts`(1500)로 이러고 있었는데 sim 만
                #   없었다. 같은 물체·같은 거리에서 raw 1400 짜리 프레임을 real 은
                #   버리고(→ 반환 0 → 탐침) sim 은 써서, **두 백엔드가 다른 거리로
                #   수렴**할 수 있었다. 문턱은 공용 상수 한 곳에서 온다.
                if pc_b is not None and 0 < len(pc_b) < p1.RAW_MIN_VERTS:
                    print(f"[isaac_scan]   tz={tz:.3f} d={d:.2f} az={azd:+.0f}° — "
                          f"원시 정점 부족 {len(pc_b)}<{p1.RAW_MIN_VERTS} — 버림")
                    pc_b = None
                # ★ preview 는 **이동할 때마다** 남긴다. 계획의 입력이 여기서 정해지므로,
                #   밴드가 이상하면 제일 먼저 볼 그림이다.
                _n = 0 if pc_b is None else len(pc_b)
                self._snap(f"preview_tz{tz*1000:.0f}_d{d*1000:.0f}_az{azd:+.0f}",
                           f"preview tz={tz*1000:.0f}mm d={d*1000:.0f}mm "
                           f"az={azd:+.0f}deg raw={_n}", stage="preview")
                if pc_b is None or len(pc_b) == 0:
                    # ★ 빈 캡처여도 **카메라 위치는 같이 준다.** 여기서 맨 배열을
                    #   돌려주면 `collect_planning_points` 가 "콜백이 옛 계약" 으로
                    #   보고 **남은 preview 내내 적응을 끄고** 고정 격자로 떨어진다.
                    #   빈 캡처는 적응이 가장 필요한 순간인데 그때 꺼지던 셈이다.
                    return np.zeros((0, 3)), self._cam_pos_base()
                # ★ 로봇 자기점 제거 (self-filter) — 프레임에 걸린 링크/스캐너
                #   점이 크롭 실린더를 오염해 밴드 폭주시키는 것 방지 (real 동일).
                q_now = np.asarray(self.robot.get_joint_angles(is_radian=True), float)
                pc_b = p1.filter_robot_points(
                    pc_b, capsules_from_joints(q_now, LINK_RADII, T_EC=self.T_EC))
                # ★ (점, 카메라위치) 를 돌려준다 — 적응적 거리조절의 입력이다
                #   (`collect_planning_points` 가 표면거리로 다음 d 를 계산한다).
                return (p1.crop_object_points(pc_b, axis_xy, disc_top,
                                              up_sign=self.up_sign),
                        self._cam_pos_base())
            # ★ **로봇이 가지도 못했다** — 이걸 조용히 빈 배열로만 돌려주면 코어가
            #   "스캐너가 아무것도 못 봤다" 로 읽고 다음 탐침으로 넘어간다. 원인이
            #   전혀 안 보인다(real 은 이동 실패 code 를 찍는데 sim 만 조용했다).
            #   보호 원기둥·충돌·리치 중 무엇에 막혔는지 알려면 이 줄이 필요하다.
            print(f"[isaac_scan]   tz={tz:.3f} d={d:.2f} — 도달 가능한 방위 없음 "
                  f"(IK 실패 {_n_ik} · 이동 실패 {_n_move} / {len(VIEW_AZIS_DEG)} 방위)")
            # 로봇은 안 움직였으므로 **현재** 카메라 위치를 준다. None 을 주면
            # 적응이 통째로 꺼진다(`_unpack_preview` 주석).
            return np.zeros((0, 3)), self._cam_pos_base()

        def move_tt(theta):
            self.turntable.move_abs(float(theta), float(np.radians(30.0)))
            self.turntable.wait_motion_done()

        # ★ preview 자체를 보호한다 — 아직 물체를 모르니 최대 반경 원기둥으로.
        self._p1_dof = sensor.dof
        self._guard_preview_volume(el_prev)
        # 실루엣 수집 루프는 **real 과 같은 공용 코드**(p1.collect_planning_points).
        pts = p1.collect_planning_points(
            preview_at, move_tt, self.axis_b, self.axis_dir_b,
            sensor=sensor, up_sign=self.up_sign,
            log=lambda m: print(f"[isaac_scan] {m}"))
        pts = p1.voxel_downsample(pts, sensor.voxel_m)
        if len(pts) < p1.MIN_PLAN_PTS:
            print(f"[isaac_scan] P1 preview 점 부족({len(pts)}"
                  f"<{p1.MIN_PLAN_PTS}) — 플래너 불가")
            return None
        # ★ 계획 점군을 **충돌 게이트의 동적 장애물**로 등록한다. 셀 CAD(env npz)
        #   에는 스캔 대상이 없다 — "225mm 까지 접근하는 대상" 이라 일부러 뺐는데,
        #   그 결과 자세 사이 이동 경로가 물체를 관통해도 아무도 안 막았다
        #   (2026-09-16 실물: NBV 이동 중 로봇이 대상을 치고 지나감).
        #   등록하면 clearance → is_pose_safe → is_path_safe → 우회계획까지
        #   전부 자동 반영된다.
        #
        #   ⚠ `set_dynamic_obstacle` 은 **base 프레임**을 받는다. sim 의 preview
        #     점군은 world 라 반드시 변환한다 — 안 하면 장애물이 엉뚱한 자리에
        #     생겨 멀쩡한 자세를 기각하거나(더 나쁘게) 진짜 물체를 안 막는다.
        #
        #   real 은 `_pick_lookaround_planner` 에서 같은 일을 한다. 2026-09-17
        #   이전에는 **sim 만 이게 빠져 있었다** — sim 에서 통과한 경로가 real
        #   에서 막히는(또는 그 반대) 갈라짐이었다.
        self._register_object_obstacle(pts, "preview")
        self._pause_to_inspect("대상물 장애물 등록 (주황=등록된 회전체)")
        nrm = p1.estimate_outward_normals(pts, axis_xy)
        # 후보 elevation 을 명시 — p1.DEFAULT_ELS=(20,30,40,50) 의 20 은 실측에서도
        # 도달 자세가 없다. P1_ELS 는 env 로 조정 가능.
        plan = p1.plan_lookaround_viewpoints(pts, nrm, axis_xy, sensor, els=P1_ELS,
                                             up_sign=self.up_sign)
        print(f"[isaac_scan] P1 플랜: {plan.note} risk={plan.tracking_risk} "
              f"(preview {len(pts)}pt)")
        # az 스윕 + 충돌 게이트도 **real 과 같은 공용 코드**(p1.solve_plan_poses).
        qs, vps = p1.solve_plan_poses(
            plan, axis_xy, VIEW_AZIS_DEG,
            solve_q=lambda tgt, el, az, so: self._view_q(tgt, el, az, so, seed)[0],
            is_safe=(self._cm.is_pose_safe if self._cm is not None else None),
            log=lambda m: print(f"[isaac_scan] {m}"), return_poses=True)
        # ★ 캡처 중 거리추종(`_scan_pass`)이 "이 q 가 원래 어떤 el/az/tz 였나" 를
        #   되찾을 수 있게 짝을 남긴다. 컨트롤러는 q 만 들고 다니므로 **객체 동일성**
        #   으로 되찾는다(인덱스로 맞추면 밴드가 하나라도 빠질 때 어긋난다).
        self._band_pairs = list(zip(qs or [], vps))
        self._p1_sensor = sensor
        self._p1_seed = np.asarray(seed, float).copy()
        return qs

    def capture_rotation(self, pose, label: str, stage: str) -> bool:
        """lookaround·flip = 전회전(전면 커버), **nbv = 부분 스윕**(목표 gap 만).

        nbv 에서 계획기가 목표 θ 를 남겨두면(`_next_theta`) 그 주변만 돈다.
        남기지 않았으면(축-고도각 폴백) 기존대로 전회전한다.
        """
        if stage == "nbv" and getattr(self, "_next_theta", None) is not None:
            th = self._next_theta
            sp = getattr(self, "_next_span", None)
            self._next_theta = None
            self._next_span = None
            return self._scan_patch(pose, th, label=label, span_deg=sp)
        n_theta = N_THETA_NBV if stage == "nbv" else N_THETA
        return self._scan_pass(pose, n_theta, label=label, stage=stage)

    # ── nbv (NBV = 추가 관측 elevation 자세, real 전회전 대응) ──────────────
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
        """nbv 관측자세 — 판단은 **sim·real 공용** `utils/nbv/nbv_planner` 가 한다.

        여기서 주입하는 것은 sim 고유의 것 두 가지뿐이다:
          · 자세 생성 = USD 규약(-Z 광축) look-at + world 프레임 타깃
          · 충돌 판정 = 캡슐 world + 실제 메시(자가/셀 구조물)
        gap 분류·아랫면 제외·visited·roll 정렬·실패 진단은 공용 코드가 담당한다.
        """
        look_target = np.array([self.axis_b[0], self.axis_b[1], self.obj_center_b[2]])
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
            _viz.show_accum(self.stage, self._viz_pts(np.vstack(self.accum)))

        def solve_lookat(eye, tgt):
            # 스크리닝 예산은 **real 과 같은 공용값**. sim 만 후하면(7seed/200iter)
            # "sim 에선 풀리고 real 에선 안 풀리는" 후보가 생긴다(2026-09-18 리뷰).
            return _vp.solve_look_at_q(
                kin, eye, tgt, q_cur, self.T_EC, T_WB=None,
                convention=_vp.CAM_USD, rolls_deg=VIEW_ROLLS_DEG,
                n_seed_alt=_nbvp.FRONTIER_IK_SEEDS,
                ik_max_iter=_nbvp.FRONTIER_IK_MAX_ITER)

        # ── ① 보장 고도각 — 오목 내부는 gap 으로 안 잡히므로(닭·달걀) 사전지식 ──
        self._nbv.disc_z = float(self.axis_b[2])       # 바닥 근처 gap 은 모든 경로에서 flip 몫
        need = [e for e in ENSURE_ELS
                if not any(abs(float(v[0]) - float(e)) < 1e-6
                           for v in self._nbv.visited)]
        # ★ 윗면 개구부(경계 고리)가 있을 때만 돈다 — 없는 물체에서 55° 전회전은
        #   30초 낭비다(공용 `needs_ensure`, 2026-09-18). 오목 물체는 입구가 경계로 남는다.
        if need:
            _ens, _uplen = _nbvp.needs_ensure(gaps, self.up_sign)
            if not _ens:
                print(f"[isaac_scan]   보장 고도각 {need} 건너뜀 — 윗면 향한 gap "
                      f"{_uplen*1000:.0f}mm < {_nbvp.ENSURE_MIN_UP_LEN_M*1000:.0f}mm (개구부 없음)")
                for e in need:                       # 다시 묻지 않게 방문 처리
                    self._nbv.visited.append((float(e), 0.0))
                need = []
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
                self._nbv_dbg_plan(gaps, None, "ensure", res0[0], look_target, 0.0, ENSURE_SPAN_DEG,
                                   note=f"ensure el={res0[1]:.0f}° az={res0[2]:.0f}°")
                return res0[0]

        # ── ② gap 겨냥 (주경로) ─────────────────────────────────────────────
        # ★ 축-고도각 방식(`plan_nbv_elevation_pose`)은 카메라가 **늘 턴테이블 축**을
        #   보므로, gap 이 어디 있든 그쪽을 향하지 않는다. gap 정보도 '법선 고도각의
        #   중앙값' 하나로 압축돼 **위치가 통째로 버려진다**. 그래서 손잡이·내벽 같은
        #   국소 결손을 원리적으로 겨냥할 수 없었다(실측: 손잡이 0점).
        #   → 표면점 p 에서 법선 n 방향 standoff 위치로 **그 gap 을 정면으로** 본다
        #     (`nbv_core.nbv_pose_from_candidate` 와 같은 원리).
        #   법선 정면이 막히면(작동거리가 물체보다 큰 오목면 등) 개구부 쪽으로 기울인다.
        # ★ 방위는 **축→로봇 방향 기준**으로 만든다(공용 `robot_side_azimuths`).
        #   VIEW_AZIS_DEG 원시값 (0,±30) 을 그대로 넘기면 base +X = 축 너머
        #   **로봇 반대편**을 가리켜 카메라 eye 가 도달한계 직전으로 가고 IK 가
        #   전멸한다. real 은 2026-09-16 에 고쳤는데 sim 은 그대로였다 — sim 도
        #   v2 셀을 base 프레임으로 돌게 된 뒤로 정확히 180° 어긋나 있었다
        #   (2026-09-18 리뷰). 그래서 gap 겨냥 주경로가 sim 에서 검증된 적이 없다.
        _obj_pts = None
        if self.accum:
            _obj_pts = np.vstack(self.accum)
            if len(_obj_pts) > 3000:
                _obj_pts = _obj_pts[np.random.default_rng(0).choice(
                    len(_obj_pts), 3000, replace=False)]
        fr = self._nbv.plan_frontier(gaps, q_cur, solve_lookat, swept,
                                     standoff_m=WORK_FOCUS,
                                     axis_xy=self.axis_b[:2],
                                     az_pref_deg=_nbvp.robot_side_azimuths(
                                         self.axis_b[:2], VIEW_AZIS_DEG),
                                     obj_pts=_obj_pts,
                                     # 정합 겹침 안전장치 — 누적 원시 점군이 기지 면
                                     known_pts=(np.vstack(self.accum) if self.accum else None))
        if fr is not None:
            self._gap_pass = True
            self._next_theta = fr[3]        # 이 θ 주변만 부분 스윕
            self._nbv_dbg_plan(gaps, fr[1], "frontier", fr[0], np.asarray(fr[1].p_O, float),
                               float(fr[3]), NBV_PATCH_SPAN_DEG,
                               note=f"L={float(fr[1].L)*1000:.0f}mm")
            try:                            # 겨냥한 gap + 카메라 위치를 씬에 표시
                _q = fr[0]
                # 씬 오버레이는 world — gap 점(base)과 카메라를 같이 올린다.
                _cam_b = (kin._fk_frames_m(_q)[7]
                          @ np.linalg.inv(self.T_EC))[:3, 3]
                _viz.show_target(self.stage,
                                 self._base_to_world(np.asarray(fr[1].p_O, float)),
                                 self._base_to_world(_cam_b))
            except Exception:
                pass
            return fr[0]

        # ── ③ 폴백: 축-고도각 — gap 군집을 로봇 앞으로 가져와 **180° 부분 스윕** ──
        #   예전엔 전회전이었다(gap 위치를 버리는 방식). 위치는 후보에 있으니
        #   가장 큰 gap 들이 앞에 오도록 θ 를 정하고 그 주변만 돈다(공용
        #   `gap_cluster_theta`, 2026-09-18). 군집 방위를 못 구하면 전회전.
        self._gap_pass = False
        res = self._nbv.plan(gaps, q_cur, solve_pose, swept,
                             roll_order_fn=roll_order)
        if res is None:
            return None
        _th = _nbvp.gap_cluster_theta(
            gaps, self.axis_b[:2],
            _nbvp.robot_side_azimuths(self.axis_b[:2], VIEW_AZIS_DEG)[0], self.up_sign)
        if _th is not None:
            self._next_theta = float(_th)
            self._next_span = _nbvp.FALLBACK_SPAN_DEG
            print(f"[isaac_scan]   폴백 스윕 θ={math.degrees(_th) % 360:.0f}° 중심 "
                  f"±{_nbvp.FALLBACK_SPAN_DEG/2:.0f}° (gap 군집 방위)")
        self._nbv_dbg_plan(gaps, None, "fallback", res[0], look_target,
                           float(_th) if _th is not None else 0.0,
                           _nbvp.FALLBACK_SPAN_DEG if _th is not None else 360.0,
                           note=f"axis-el el={res[1]:.0f}° az={res[2]:.0f}°")
        return res[0]

    def _nbv_dbg_plan(self, gaps, chosen, mode, q, target_b, theta, span_deg, note=""):
        """nbv 디버그 기록(계획 시점) — `utils/nbv/nbv_debug_dump`. 실패해도 무시."""
        try:
            if getattr(self, "_nbv_dbg", None) is None:
                self._nbv_dbg = _NbvDebugDump()
            eye = (kin._fk_frames_m(np.asarray(q, float))[7]
                   @ np.linalg.inv(self.T_EC))[:3, 3]
            idx = next((i for i, c in enumerate(gaps or []) if c is chosen), -1)
            self._nbv_dbg.plan(gaps=gaps, chosen=idx, mode=mode, eye=eye, target=target_b,
                               theta=theta, span_deg=span_deg,
                               master_pts=(np.vstack(self.accum) if self.accum else None),
                               axis_pt=self.axis_b, up_sign=self.up_sign, note=note)
        except Exception as e:                                   # noqa: BLE001
            print(f"[isaac_scan]   [nbv-dbg] plan 기록 실패({e})")

    # ── nbv (NBV hole-fill) 프리미티브 — 수렴 루프는 공용 컨트롤러 소유 ──
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
        # ★ nbv 는 매 반복 메시를 다시 만든다. 누적 점이 30만을 넘으면 Poisson 이
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
            print("[isaac_scan] nbv 누적점 부족(<200) — 종료.")
            return None
        # ★ 루프용 메시는 **저비용**으로. Artec 도 스캔 중에는 FastFusion(복셀 기반)을
        #   쓰고 Poisson 은 후처리 전용이다. 여기서 필요한 건 '어디가 비었나' 판단이지
        #   최종 품질이 아니므로 depth 를 낮춘다(비용은 depth 에 급격히 증가).
        #   최종 메시는 `finalize()` 에서 depth=8 로 한 번만 만든다.
        self._pump(2)
        m = p2.pcd_to_mesh_poisson(pcd, depth=MESH_LOOP_DEPTH, density_quantile=0.04)
        self._pump(2)
        self._tick("mesh", _t0)
        # ★ 동적 장애물을 **실측 메시로 갱신**한다. preview 점군은 물체를 한쪽
        #   방위에서만 본 것이라 반대쪽이 비어 있다 — nbv 는 그 반대쪽으로 로봇을
        #   보내므로, 안 갱신하면 정작 필요한 순간에 장애물이 없는 셈이 된다.
        #   real 은 같은 자리(`build_coverage_mesh`)에서 이미 이렇게 한다.
        try:
            if m is not None and len(m.vertices):
                self._register_object_obstacle(np.asarray(m.vertices), "mesh")
        except Exception as e:                                # noqa: BLE001
            print(f"[isaac_scan] ⚠ 대상물 장애물 갱신 실패({e}) — 계속")
        return m

    def is_converged(self, mesh) -> bool:
        cov = p2.coverage_state(mesh, **GAP_KW)
        print(f"[isaac_scan]   boundary={cov.boundary_len_m*1000:.0f}mm "
              f"cov={cov.angular_cov:.2f} gaps={cov.n_gaps}")
        # 단계별 메시 덤프 — 지표를 계산하는 **바로 그 메시**를 저장한다(숫자와 그림이
        # 같은 대상을 가리키게). nbv 가 정말 나빠지는지 눈으로 확인하는 용도.
        if STAGE_DUMP:
            try:
                import open3d as _o3d
                i = getattr(self, "_stage_i", 0); self._stage_i = i + 1
                os.makedirs(STAGE_DUMP, exist_ok=True)
                f = os.path.join(STAGE_DUMP,
                                 f"{i:02d}_b{cov.boundary_len_m*1000:.0f}_g{cov.n_gaps}.ply")
                _o3d.io.write_triangle_mesh(f, mesh)
                # 런타임 지표 검증용 — **누적 원시 점군**도 같이 남긴다. 메시가 아니라
                # 이걸로 신규 복셀을 세야 재구성 흔들림에 오염되지 않는다.
                if self.accum:
                    np.savez_compressed(f[:-4] + "_accum.npz",
                                        points=np.vstack(self.accum).astype(np.float32))
                print(f"[isaac_scan]   (덤프) {os.path.basename(f)}")
            except Exception as e:                       # noqa: BLE001
                print(f"[isaac_scan]   ⚠ 단계 덤프 실패({e})")
        # ── 수렴 판정 ────────────────────────────────────────────────────
        # ★ boundary 하나로 판정하면 안 된다. 커버리지가 **넓어지는 동안 boundary 는
        #   늘어난다** — 새로 붙은 표면의 테두리가 그대로 경계로 잡히기 때문이다.
        #   GT 대조 실측(2026-08-19 hand_drill): nbv 가 completeness 를
        #   37.9%→52.7% 로 올리는 동안 boundary 는 519→800mm 로 '악화'했다.
        #   "boundary 가 안 준다 = 나빠졌다" 로 읽으면 정반대 결론이 나온다.
        #
        #   대신 **새 표면이 더 안 붙는지**를 본다(표면적 한계 이득). GT 를 못 쓰는
        #   실물에서도 쓸 수 있고, 위 실측에서 Δ면적% 와 Δcompleteness 가 거의
        #   나란히 움직였다.
        #
        #   ⚠ 기본값 1.5%/3회는 **보수적**이다. GT 대조로 (eps,N) 을 훑어보니 더 공격적인
        #     설정은 머그에서 완전성 2~3%p 를 잃었다(개선이 간헐적이라 두 번 조용했다고
        #     끝난 게 아니다). 1.5%/3 은 드릴에서 1패치를 아끼고 손실 0.13%p, 머그는
        #     손실 0 이었다. **물체 2종으로 맞춘 값이니 더 넓게 검증할 것.**
        #     검증 도구: scripts/sim/eval_vs_gt.py (+ MMS_SIM_STAGE_DUMP)
        #   지표는 **누적 원시 점군의 신규 점유 복셀**로 잰다. 재구성 메시에서 재면
        #   Poisson 표면이 패치마다 미세하게 흔들려, 새로 본 게 없어도 복셀이 바뀐다
        #   (실측: 메시 기반 신규복셀은 머그에서 Δcompleteness 와 r=+0.43 에 그쳤다).
        #   누적 점군은 점이 더해지기만 하므로 신규 복셀 = 진짜 새로 관측한 공간이다.
        new_frac = None
        if self.accum:
            new_frac, self._conv_vox = _nbvp.new_voxel_frac(
                np.vstack(self.accum), getattr(self, "_conv_vox", None), CONV_VOX_M)
        # ── gap 단위 회계 (주 종료 경로) ─────────────────────────────────
        # 직전 패치가 frontier 겨냥이었으면 결과를 planner 에 보고한다. 비생산
        # 패치의 겨냥점 주변은 후보에서 빠져, **계획기가 후보를 소진하면 루프가
        # 자연 종료**된다. 전역 조기종료(아래 백스톱)보다 이것이 주 경로다 —
        # NBV 개선은 간헐적이라(작은 gap 뒤에 큰 gap) 전역 정지는 이득을 잘린다.
        if new_frac is not None and getattr(self, "_nbv", None) is not None:
            self._nbv.report_patch(new_frac >= DRY_EPS)
        area = float(mesh.get_surface_area())
        if new_frac is not None:
            stall = getattr(self, "_stall", None)
            if stall is None:
                stall = self._stall = _nbvp.StallTracker(CONV_NEW_EPS, CONV_AREA_N)
            done = stall.update(new_frac)
            print(f"[isaac_scan]   신규복셀={new_frac*100:.2f}% "
                  f"표면적={area*1e4:.0f}cm² 정체 {stall.flat}/{stall.n}")
            if done:
                print("[isaac_scan]   수렴 — 새로 보이는 곳이 없다. 완료.")
                return True
        if p2.is_converged(cov, _nbvp.CONV_BOUNDARY_M, _nbvp.CONV_COVERAGE_TAU):
            print("[isaac_scan]   수렴 — 완료.")
            return True
        return False

    def plan_nbv_pose(self, mesh):
        if self._world is None:
            self._world = self._build_world()
        gaps = p2.detect_gaps(mesh, **GAP_KW)
        q_cur = np.asarray(self.robot.get_joint_angles(is_radian=True), float)
        return self._plan_nbv_pose(self._world, q_cur, gaps)

    def supports_flip(self) -> bool:
        """sim 도 flip 지원 — 물체를 USD 에서 회전시킨다(real=사람 손회전)."""
        return bool(self._flip_angles()) and self.stage.GetPrimAtPath(self._obj_prim).IsValid()

    @staticmethod
    def _flip_view_els(ang_deg: float):
        """flip 각에 대한 관측 고도각 **후보 사다리** — 예측·실행이 같이 쓴다.

        원래 바닥면 법선 −ẑ 를 flip 회전으로 돌린 방향을 정면으로 보는 el 에서
        시작해 10° 씩 내려간다. 필요 최소치(90 − 입사각예산)까지 내려가고 마지막에
        VIEW_EL_DEG 를 폴백으로 둔다.

        ★ 예측(_can_view_flipped_bottom)과 실행(next_flip)이 **다른 목록**을 쓰면
          "도달 가능하다고 판정해 놓고 실제로는 폴백" 이 된다(실측 2026-08-20:
          예측 [70,60,50,40] 은 60° 성공, 실행은 ±15° 가 전부 70° 로 클램프돼
          [70,30] 만 시도 → 30° 추락 → 바닥 grazing). 한 함수로 묶어 방지한다.
        """
        from utils.nbv.flip_policy import el_needed_for_face
        R = _axis_rot(FLIP_AXIS, math.radians(float(ang_deg)))
        n = R @ np.array([0.0, 0.0, -1.0])
        el_face = math.degrees(math.asin(float(np.clip(n[2], -1.0, 1.0))))
        el_min = (el_needed_for_face(MAX_INCIDENCE_DEG) if el_face > 45.0
                  else FLIP_EL_MIN)
        el_min = float(np.clip(el_min, FLIP_EL_MIN, FLIP_EL_MAX))
        start = float(np.clip(el_face, FLIP_EL_MIN, FLIP_EL_MAX))
        out, e = [], start
        while e >= el_min - 1e-6:
            out.append(round(e, 1))
            e -= 10.0
        if all(abs(VIEW_EL_DEG - x) > 1e-6 for x in out):
            out.append(float(VIEW_EL_DEG))              # 마지막 폴백
        return out, el_face, el_min

    def _predict_flip_geom(self, ang_deg: float):
        """flip 후 물체의 (중심, 반경, z범위) 예측 — next_flip 의 배치 수식과 동일.

        원본 AABB 를 회전시키고 원판 위에 앉힌다. 물체가 테이블에 놓인다는 물리
        제약에서 나오므로 실물에서도 스캔 bbox 로 같은 계산이 가능하다.
        """
        o0 = getattr(self, "_obj0", None)
        if o0 is None:
            return None
        R = _axis_rot(FLIP_AXIS, math.radians(float(ang_deg)))
        c = o0["center"]
        lo = np.array([c[0] - o0["radius"], c[1] - o0["radius"], o0["zlo"]])
        hi = np.array([c[0] + o0["radius"], c[1] + o0["radius"], o0["zhi"]])
        corners = np.array([[x, y, z] for x in (lo[0], hi[0])
                            for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])
        rc = (corners - c) @ R.T + c
        # 원판에 닿는 면은 **아래쪽 끝**이다 — base 가 +Z 아래면 z 최대다.
        _bottom = float(rc[:, 2].min() if self.up_sign > 0 else rc[:, 2].max())
        rc = rc + np.array([0.0, 0.0, float(self.axis_b[2]) - _bottom])
        radius = float(max(rc[:, 0].max() - rc[:, 0].min(),
                           rc[:, 1].max() - rc[:, 1].min()) / 2.0)
        zlo, zhi = float(rc[:, 2].min()), float(rc[:, 2].max())
        return np.array([c[0], c[1], (zlo + zhi) / 2.0]), radius, (zlo, zhi)

    def _can_view_flipped_bottom(self) -> bool:
        """180° flip 후 **바닥면을 입사각 예산 안에서** 볼 자세에 도달하는가.

        이것이 90° flip 추가 여부의 진짜 기준이다(종횡비는 대리 지표일 뿐).
        바닥 법선은 +ẑ 이므로 el ≥ 90 − MAX_INCIDENCE 가 필요하고, 물체가 높으면
        그 고앙각이 자가충돌·도달불가로 걸러져 낮은 el 로 폴백 → grazing 소실.
        """
        g = self._predict_flip_geom(180.0)
        if g is None:
            print("[isaac_scan] flip 판정: 형상 예측 불가(_obj0 없음) — 180° 로 진행")
            return True
        center, radius, _z = g
        els, _elf, el_min = self._flip_view_els(180.0)
        els = [e for e in els if e >= el_min - 1e-6]     # 폴백(저앙각) 제외하고 판정
        seed = np.asarray(self.robot.get_joint_angles(is_radian=True), float)
        standoff = WORK_FOCUS + radius
        tgt = np.array([self.axis_b[0], self.axis_b[1], center[2]])
        for eld in els:
            for azd in VIEW_AZIS_DEG:
                q, _ = self._view_q(tgt, eld, azd, standoff, seed)
                if q is None:
                    continue
                if self._cm is not None and not self._cm.is_pose_safe(q)[0]:
                    continue
                print(f"[isaac_scan] flip 판정: 180° 후 바닥 관측 el={eld:.0f}° "
                      f"az={azd:.0f}° 도달 가능 (필요 el≥{el_min:.0f}°) — 180° 만")
                return True
        print(f"[isaac_scan] flip 판정: 180° 후 바닥 관측 el≥{el_min:.0f}° "
              f"**도달 불가** (물체 상단 z={_z[1]:.3f}m) — 90° 눕히기로 보완")
        return False

    def _flip_angles(self):
        """물체 종횡비로 flip 각을 정한다. env 명시가 항상 우선.

        세장형(키/지름 ≥ FLIP_ASPECT)은 **90° flip 을 추가**한다 — 키 큰 물체의
        윗면(캡)은 측면 카메라에서 grazing 이고 고앙각 자세는 자가충돌로 도달이
        어렵다(실측: spray_can 밴드3 tool↔link4). 90° 로 눕히면 양쪽 끝면이
        측면을 향해 el 30~70° 로 쉽게 잡힌다. 180° 는 항상 마지막(바닥면).
        """
        cached = getattr(self, "_flip_angles_c", None)
        if cached is not None:
            return cached
        if os.environ.get("MMS_SIM_FLIP_ANGLES"):
            self._flip_angles_c = FLIP_ANGLES_DEG          # 명시값 그대로
            return self._flip_angles_c
        # ★ 종횡비는 **스캔한 점군**에서 잰다. USD AABB(정답)를 쓰면 실물이 못 하는
        #   판단을 sim 만 하게 되어 검증 전제가 깨진다. real 은 master 메시 정점으로
        #   같은 함수를 부른다(artec_multipass_scan_session.is_converged).
        from utils.nbv.flip_policy import dims_from_points, flip_angles_for
        hd = dims_from_points(np.vstack(self.accum),
                              axis_xy=self.axis_b[:2]) if self.accum else None
        if hd is None:
            return FLIP_ANGLES_DEG                          # 아직 스캔 전 — 캐시 안 함
        angles, aspect = flip_angles_for(hd[0], hd[1], FLIP_ASPECT,
                                         can_view_bottom=self._can_view_flipped_bottom)
        self._flip_angles_c = angles if len(angles) > 1 else FLIP_ANGLES_DEG
        if len(angles) > 1:
            print(f"[isaac_scan] flip 90° 추가 (90→180 순) — 스캔 측정 "
                  f"h={hd[0]*1000:.0f}mm d={hd[1]*1000:.0f}mm 종횡비 {aspect:.2f}")
        return self._flip_angles_c

    def next_flip(self) -> bool:
        """다음 flip 자세로 물체를 회전. 더 없으면 False.

        ★ 누적 좌표계 주의 — 뒤집은 뒤 캡처한 점은 **flip 된 물체 프레임**에 있다.
          canonical(원래 물체 자세)로 되돌리지 않으면 점군이 어긋나 병합된다.
          그래서 회전량을 `self._flip_R` 로 들고 있다가 `_accumulate_canonical` 에서
          역회전한다. real 은 Artec 이 master 에 재고정해 같은 역할을 한다.
        """
        angles = self._flip_angles()
        i = getattr(self, "_flip_i", 0)
        if i >= len(angles):
            return False
        ang = float(angles[i])
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
        disc_top = float(self.axis_b[2])
        # 원판에 닿는 면 = **아래쪽 끝**. base 가 +Z 아래면 z 최대다.
        _bot = float(rc[:, 2].min() if self.up_sign > 0 else rc[:, 2].max())
        dz = disc_top - _bot                         # 원판 위에 앉히기
        M = np.eye(4); M[:3, :3] = R; M[:3, 3] = c - R @ c + np.array([0.0, 0.0, dz])
        print(f"[isaac_scan]   flip 배치: 회전 후 원판 위로 {dz*1000:+.0f}mm 이동")
        # ★ 물체 transform 을 직접 쓰지 않는다 — 턴테이블이 rider base 로 매 회전마다
        #   덮어써서 flip 이 지워진다(실측 실패). 턴테이블 API 로 base 에 합성한다.
        # ★ USD 경계 — `set_rider_flip` 은 **world** 변환을 받는다. 여기 계산은
        #   전부 base 이므로 닮음변환으로 옮긴다: M_w = T_WB · M_b · T_WB⁻¹.
        _M_w = self.T_WB @ M @ np.linalg.inv(self.T_WB)
        if not self.turntable.set_rider_flip(self._obj_prim, _M_w):
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
        self.obj_zlo_b, self.obj_zhi_b = float(rc2[:, 2].min()), float(rc2[:, 2].max())
        self.obj_radius = float(max(rc2[:, 0].max() - rc2[:, 0].min(),
                                    rc2[:, 1].max() - rc2[:, 1].min()) / 2.0)
        self.obj_center_b = np.array([c[0], c[1], (self.obj_zlo_b + self.obj_zhi_b) / 2.0])
        print(f"[isaac_scan]   크롭 갱신: z=[{self.obj_zlo_b:.3f},{self.obj_zhi_b:.3f}] "
              f"r={self.obj_radius*1000:.0f}mm")
        # ── flip 후 관측자세 계획 — flip = "뒤집힌 물체의 lookaround" ─────────
        # home 자세 캡처는 낮은 물체에서만 우연히 성립한다. 실측(2026-08-19,
        # spray_can 90°→180°): home 카메라 가용 z 대역이 0.67~0.74m 라, 다시 세운
        # 키 207mm 물체는 옆면이 grazing(입사각 필터 전멸) + 꼭대기는 조준 밖
        # → 553점. 뒤집힌 물체의 새 중심/반경으로 측면 자세를 계획해 이동한다.
        # ★ 고도각은 **새로 드러난 면의 법선**에서 정한다. 원래 바닥면 법선 −ẑ 를
        #   flip 회전 R 로 돌린 것이 n_new 이고, 그 방향을 정면으로 보는 el 이
        #   asin(n_new_z) 다. VIEW_EL_DEG(30°) 를 그대로 쓰면 안 된다 —
        #   180° flip 후 바닥은 +ẑ 를 향하므로 el 30° 에서 입사각이 60° 가 되어
        #   MAX_INCIDENCE_DEG(50°) 필터에 **전량 걸린다**. 실측(2026-08-19 mug):
        #   굽 링(법선 수평)은 남고 오목한 바닥 중앙만 100% 소실 → completeness
        #   60.8→53.2%. 반대로 90° flip 은 끝면이 수평을 향하므로 낮은 el 이 맞다.
        try:
            seed = np.asarray(self.robot.get_joint_angles(is_radian=True), float)
            # ★ 조준점 = **새로 드러난 면의 중심**, 축거리 = 그 면까지 **작동거리 창 중앙**.
            #   예전엔 옆면 공식(물체 중심 조준 + WORK_FOCUS+r)을 그대로 썼다. 180° flip
            #   을 el=70° 로 내려다보면 새 윗면은 중심보다 h/2 위라 카메라에서
            #   ≈191mm(세제) — 근접한계 200mm **안쪽**이라 창 필터에 잘리고, 남는 건
            #   가장자리 스침 점뿐이라 입사각 필터가 전멸시켰다. 실측 2026-09-18:
            #   flip 패스 240프레임 전부 `incidence후=0`, 합계 +11점.
            #   새 면 중심 = 원래 바닥면 중심을 flip 회전 R 로 돌려 원판 위로 올린 점.
            _bot0 = np.array([c[0], c[1], (o0["zlo"] if (o0["zlo"] - c[2]) * self.up_sign < 0 else o0["zhi"])])
            face_c = R @ (_bot0 - c) + c + np.array([0.0, 0.0, dz])
            _dof = getattr(self, "_p1_dof", None) or p1.SensorModel().dof
            standoff = 0.5 * (float(_dof[0]) + float(_dof[1]))     # 표면을 창 중앙에
            tgt = face_c
            self._flip_face_c = face_c.copy()          # 테두리 패스(flip_extra_poses)가 쓴다
            print(f"[isaac_scan]   flip 조준: 새 면 중심 z={(face_c[2]*self.up_sign - float(self.axis_b[2])*self.up_sign)*1000:.0f}mm(원판 위) "
                  f"표면거리 {standoff*1000:.0f}mm")
            el_cands, el_face, _elmin = self._flip_view_els(ang)
            moved = False
            for eld in el_cands:                            # 도달·충돌로 걸러 첫 성공
                for azd in VIEW_AZIS_DEG:
                    q3, _ = self._view_q(tgt, eld, azd, standoff, seed)
                    if q3 is not None and self._cm is not None:
                        ok3, _why3 = self._cm.is_pose_safe(q3)
                        if not ok3:
                            q3 = None
                    if q3 is not None and self._drive(q3):
                        print(f"[isaac_scan]   flip 관측자세 이동: az={azd:.0f}° "
                              f"el={eld:.0f}° (새 면 법선 el={el_face:+.0f}°) "
                              f"standoff={standoff*1000:.0f}mm")
                        moved = True
                        break
                if moved:
                    break
            if not moved:
                print("[isaac_scan]   ⚠ flip 관측자세 못 찾음 — 현재 자세로 캡처")
        except Exception as e:                       # noqa: BLE001
            print(f"[isaac_scan]   ⚠ flip 관측자세 계획 실패({e}) — 현재 자세 유지")
        try:    # 진단 전용 — sim 에서만 알 수 있는 실제값. **보정에는 쓰지 않는다.**
            D_true = self._prim_world_T(self._obj_prim) @ np.linalg.inv(self._obj_W0)
            err = float(np.linalg.norm(D_true[:3, 3] - M[:3, 3])) * 1000.0
            print(f"[isaac_scan]   (진단) hint 대비 실제 이동차 {err:.1f}mm "
                  f"— ICP 가 이만큼을 메워야 한다")
        except Exception:
            pass
        print(f"[isaac_scan] === flip: 물체 {FLIP_AXIS}축 {ang:.0f}° flip "
              f"({i+1}/{len(angles)}) ===")
        return True

    def flip_extra_poses(self):
        """flip 면 패스 뒤 **테두리 패스** 자세 — [q] 또는 []. 컨트롤러 `_flip_extra_passes`.

        면 패스(el≈70°)는 새 면은 잡지만 **바닥 모서리(필렛)** 를 못 잡는다 — flip 뒤
        그 면의 법선은 위로 9~25° 라 70° 에서 입사각이 50° 를 넘는다(실측 2026-09-18
        세제: 바닥 둘레 10~20mm 띠가 빔). el≈45° 에서 조준점(새 면 중심)까지의 축거리를
        `flip_policy.rim_standoff` 로 잡으면 테두리 근점이 창 중앙(250mm)에 온다.
        캡처는 면 패스와 같은 flip 경로(hint + 전역/국소 정합)다.
        """
        face_c = getattr(self, "_flip_face_c", None)
        if face_c is None:
            return []
        from utils.nbv.flip_policy import rim_standoff, RIM_EL_CANDS_DEG
        _dof = getattr(self, "_p1_dof", None) or p1.SensorModel().dof
        wc = 0.5 * (float(_dof[0]) + float(_dof[1]))
        seed = np.asarray(self.robot.get_joint_angles(is_radian=True), float)
        r = float(self.obj_radius)
        for eld in RIM_EL_CANDS_DEG:
            s = rim_standoff(r, eld, wc)
            if not np.isfinite(s):
                continue
            for azd in VIEW_AZIS_DEG:
                q, _ = self._view_q(np.asarray(face_c, float), float(eld), float(azd), float(s), seed)
                if q is None:
                    continue
                if self._cm is not None and not self._cm.is_pose_safe(q)[0]:
                    continue
                print(f"[isaac_scan]   flip 테두리 자세: el={eld:.0f}° az={azd:.0f}° "
                      f"축거리 {s*1000:.0f}mm (테두리 r={r*1000:.0f}mm → 창 중앙 {wc*1000:.0f}mm)")
                return [q]
        print("[isaac_scan]   ⚠ flip 테두리 자세 못 찾음 — 테두리 패스 생략")
        return []

    def finalize(self) -> IsaacScanResult:
        if getattr(self, "_rec", None) is not None:
            self._rec.finish()
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

    # ── run — 공용 단계 컨트롤러에 위임 ────────────────────────────────────
    def run(self) -> IsaacScanResult:
        # stage_until 순차 누적 (sim): "lookaround"=밴드 캡처까지,
        # "nbv" 이상=NBV 보강까지. 값은 단계 이름이다(옛 정수도 해석된다).
        # flip(바닥면 flip)은 사용자 손회전 필요 → supports_flip=False (sim 미지원).
        #
        # real 과 **같은 설정**(ArtecProcessSettings.multipass_settings.stage_until)을 따른다.
        # 이전에는 sim 만 환경변수를 봐서, main_artec.py 의 stage_until 를 바꿔도 sim 은
        # 반응하지 않았다. 환경변수는 스윕 스크립트(scripts/sim/*.sh)용 override 로 남긴다.
        # 해석은 공용 `resolve_stage_until` 한 곳에서만 — main_artec.py 의 표시와
        # 여기의 실제 동작이 **같은 함수**를 쓰므로 어긋날 수 없다.
        self.stage_until, src = resolve_stage_until(
            getattr(self.s, "multipass_settings", None), allow_env=True)
        print(f"[isaac_scan] stage_until={self.stage_until} ({src})")
        self.nbv_k_max = NBV_K_MAX
        self._world = None                            # nbv 진입 시 lazy build
        self._setup_object()
        return run_scan_stages(self)


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
    """복셀 다운샘플 — 복셀당 첫 점. **단일 정수 키**로 unique 한다.

    ★ 예전 `np.unique(keys, axis=0)` 는 행 단위 unique 라 느리다(구조체 뷰 정렬).
      프레임마다 불려서 lookaround 프레임 처리시간의 **46~50%**(`crop` 항목,
      세제 240프레임×6밴드 = 87s) 가 여기였다(2026-09-18 프로파일). 축별 정수를
      int64 하나로 묶으면 1-D unique 라 4.6배 빠르고(110k 점: 97→21ms) 결과는 같다.
    """
    pts = np.asarray(pts, float)
    if len(pts) == 0:
        return pts
    keys = np.floor(pts / v).astype(np.int64)
    keys -= keys.min(axis=0)                       # 음수 제거 → 안전한 묶기
    span = keys.max(axis=0) + 1                    # 축별 칸 수
    flat = (keys[:, 0] * span[1] + keys[:, 1]) * span[2] + keys[:, 2]
    _, idx = np.unique(flat, return_index=True)
    return pts[np.sort(idx)]


_PCA_CHUNK = 40000          # (N,k,3) 이웃 버퍼 상한 — 메모리 폭주 방지


def _pca_normals(pts, c_ref, k=12):
    """국소 PCA 법선. KD-tree + 배치 eigh 로 벡터화.

    이전 구현은 점마다 전수 거리(O(N²))를 돌아 8천 점에 2초가 걸렸고, 이것이
    nbv 캡처 간 지연의 95% 였다. 결과는 동일하고 속도만 다르다.
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


# 공용 코어 위임 (real 과 같은 구현을 쓰기 위해 utils/nbv/lookaround 소유).
_rot_about_axis = p1.rot_about_axis
