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
import sys
from typing import Optional

import numpy as np

# ── 씬/프림 경로 (v3_scene.usd — 260811 신규 레이아웃 기준) ───────────────────
# v3_scene.usd 는 scripts/sim/build_scene_v3.py 가 생성한다(형상 src = v3.usd).
# 구 v2.usd 경로는 git 이력 참고. 씬은 MMS_SIM_USD 로 override 가능.
# 자산 루트는 mms_paths 가 리포 위치에 맞춰 해석한다 (MMS_ASSET_ROOT 로 override).
_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__)))))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)
from mms_paths import asset  # noqa: E402

#: 기본 sim 씬 = **실물 셀 배치**(2026-09-17). `MMS_SIM_USD` 로 덮어쓴다.
#  ★ 예전 기본값은 `frame_xarm7_spider_turntable_v2/v3_scene.usd` 였는데, 그건
#    **짓기로 했다가 안 지은 레이아웃**이다(2026-09-16 현장 확인). 실물은 v2 배치이고
#    v3 는 턴테이블이 로봇 base 바로 아래(수평 0mm)라 실물(799mm)과 기하가 달라
#    자세 선정·도달성·이동량을 전혀 예측하지 못했다.
#    생성: `scripts/sim/build_scene_v2_real.py` (근거·검증 수치는 그 docstring)
DEFAULT_USD_PATH = asset("frame_xarm7_spider_turntable/v2_real_260917.usd")
#: 씬별로 맞는 충돌 셀 레이아웃 별칭. 씬과 충돌 점군이 어긋나면 **모든 자세가
#  '충돌'로 거부**된다(2026-09-17: sim home 이 셀 안에 박혀 lookaround 이 0점).
#  경로에 아래 키가 들어 있으면 그 레이아웃을 쓴다. 없으면 활성본을 그대로 쓴다.
SCENE_COLLISION_LAYOUT = {"v3_scene": "v3_layout_sim", "v3_ts_": "v3_layout_sim",
                          "v2_real_260917": "v2_real_260917"}
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
#: 스캐너 FOV — **플래너와 같은 출처**(`SensorModel`)에서 가져온다.
#  ★ 2026-09-17 — 예전엔 여기 30.0° 가 따로 박혀 있었다. 플래너는 실측 K 로 고친
#    21.58°/28.58° 를 쓰는데 **sim 렌더러만 옛 값**이라, 세로 FOV 가 22.73° vs
#    28.58° 로 갈라져 있었다. 세로는 한 자세가 덮는 **높이**를 정하는 축이고 그게
#    곧 밴드 수의 입력이라, sim 으로 밴드 로직을 검증하는 의미가 없어진다.
from utils.nbv.lookaround import SensorModel as _SensorModel
_SPIDER_SENSOR          = _SensorModel()
SPIDER_HFOV_DEG         = _SPIDER_SENSOR.hfov_deg
SPIDER_VFOV_DEG         = _SPIDER_SENSOR.vfov_deg
#: Artec Spider 의 **작동거리 창** — 이 밖의 표면은 실물에서 데이터가 안 나온다.
#  캡처 후 **점 필터**로 적용한다(`isaac_scanner.capture_points_base`).
SPIDER_WORKING_DISTANCE = tuple(
    float(v) for v in os.environ.get("MMS_SIM_WD", "0.2,0.3").split(","))
#: 카메라 near/far 클리핑. **작동거리와 같게 두면 안 된다.**
#  ★ 2026-09-17 — 예전엔 클리핑을 작동거리(0.2~0.3m)로 그대로 박아서, 거리가 조금만
#    벗어나도 depth 가 **통째로 비고 캡처가 0점**이 됐다. 오늘 sim 이 v3·v2 씬 양쪽에서
#    한 점도 못 얻은 원인이 이것이다(클리핑만 넓히자 0 → 4,668점).
#    클리핑은 렌더 한계일 뿐이고 센서 특성은 위 작동거리 필터로 재현한다.
SPIDER_CLIP_RANGE = tuple(
    float(v) for v in os.environ.get("MMS_SIM_CLIP", "0.02,3.0").split(","))
SPIDER_FOCUS_DISTANCE   = 0.25
# ★ 스캐너 해상도 — **점군 밀도만 결정**하지 최종 품질을 좌우하지 않는다.
#   누적은 VOXEL_M(2mm) 로 다운샘플되므로, 0.25m 에서 FOV 134×100mm 를 채우는 데
#   필요한 점은 (134/2)×(100/2) ≈ 3,300 개다. 1280×960 은 프레임당 **73만 점**을 만들어
#   그중 2,300 개만 살아남았다(99.7% 낭비). 실물 밀도(240프레임/회전)로 올리자
#   annotator 자원이 고갈돼 `get_data` 에서 크래시까지 났다.
#   → 필요량의 여유배수만 남긴다. 더 촘촘히 보려면 VOXEL_M 을 먼저 줄일 것.
# GUI 초기 뷰포트 시점 — 턴테이블을 −X/+Y/+Z 쪽에서 가깝게 내려다본다(사용자 요청).
# ⚠ 씬마다 턴테이블 위치가 다르다 — **DISC_PRIM 에서 실측**하고, 못 읽을 때만 이 값을
#   쓴다. v3 씬 기준 상수를 그대로 쓰면 실물 배치 씬(턴테이블 world x≈-0.39)에서
#   GUI 가 빈 공간을 비춘다(2026-09-17).
START_VIEW_TARGET = (0.365, 0.0, 0.70)      # 폴백 — v3 씬의 턴테이블 상면 부근
START_VIEW_DIR    = (-1.0, 1.0, 0.8)        # 카메라가 놓일 방향(타깃 기준)
START_VIEW_DIST   = float(os.environ.get("MMS_SIM_VIEW_DIST", "1.1"))

#: 렌더 해상도 (w, h). **세로형** — 실물 Spider 가 960x1280 세로형이다.
#
#  ★ 세로 FOV 는 **해상도 종횡비가 정한다** — authored `verticalAperture` 가 아니다.
#    Isaac `Camera` 는 정방 화소를 강제한다: `get_vertical_aperture()` 가
#    `horizontalAperture × H/W` 로 다시 계산해 authored 값과 다르면 **덮어쓴다**
#    (isaacsim.sensors.camera/camera.py `_ensure_square_pixels`, 2026-09-18 확인).
#    내부행렬·point cloud 도 그 값을 쓴다. 그래서 (w,h) 가 곧 FOV 손잡이다.
#    예전 주석 "해상도는 표본 밀도만 정한다" 는 틀렸고, 288x384 가 28.51° ≈ 28.58° 로
#    우연히 맞아 아무도 못 알아챘다(GUI 뷰포트가 가로형이면 세로 12° 로 잘려 보인다).
#  → 기본값은 **FOV 에서 h 를 유도**한다(아래 `_scanner_resolution`). env 로 (w,h) 를
#    직접 주면 유도값과 0.3° 넘게 어긋날 때 **기동 시 오류**로 막는다 — 조용히
#    밴드 수가 바뀌는 것보다 낫다. 화소 밀도만 바꾸고 싶으면 w 만 주면 된다.
def _scanner_resolution():
    env = os.environ.get("MMS_SIM_SCAN_RES", "").strip()
    ratio = math.tan(math.radians(SPIDER_VFOV_DEG) / 2.0) / math.tan(math.radians(SPIDER_HFOV_DEG) / 2.0)
    if not env:
        w = 288
        return (w, int(round(w * ratio)))
    parts = [int(v) for v in env.split(",")]
    if len(parts) == 1:                          # w 만 → h 유도
        return (parts[0], int(round(parts[0] * ratio)))
    w, h = parts[:2]
    vfov_eff = 2.0 * math.degrees(math.atan(math.tan(math.radians(SPIDER_HFOV_DEG) / 2.0) * h / w))
    if abs(vfov_eff - SPIDER_VFOV_DEG) > 0.3:
        raise ValueError(
            f"MMS_SIM_SCAN_RES={w},{h} 는 세로 FOV 를 {vfov_eff:.2f}° 로 만든다 "
            f"(센서 모델 {SPIDER_VFOV_DEG:.2f}°). Isaac 은 정방 화소를 강제해 해상도 종횡비가 "
            f"곧 FOV 다. h={int(round(w * ratio))} 로 주거나 w 만 주면 된다.")
    return (w, h)
SCANNER_RESOLUTION      = _scanner_resolution()

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
        self.headless = bool(headless)      # _set_start_view 가 참조

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
        # ★ 파일이 있는지 **먼저** 본다. 없으면 `open_stage` 는 조용히 실패하고
        #   `get_stage()` 가 None 을 돌려주는데, 그 뒤 첫 사용처에서
        #   `AttributeError: 'NoneType' object has no attribute 'GetPrimAtPath'`
        #   가 난다 — 진짜 원인(경로)이 트레이스백 어디에도 안 보인다.
        #   2026-09-17: `MMS_SIM_USD="$ASSET/..."` 에서 $ASSET 이 비어 15초를
        #   기동하고 저 에러로 끝났다. 흔한 실수라 여기서 잡는다.
        if not os.path.isfile(self.usd_path):
            raise FileNotFoundError(
                f"씬 USD 가 없다: {self.usd_path!r}\n"
                f"  · 경로가 비었거나 잘렸으면 MMS_SIM_USD 를 확인할 것 "
                f"(셸 변수가 안 풀렸을 수 있다)\n"
                f"  · 지정 안 하면 기본 씬을 쓴다: {DEFAULT_USD_PATH}\n"
                f"  · 자산 루트: MMS_ASSET_ROOT (지금 {asset('')!r})")
        print(f"[IsaacWorld] Opening USD: {self.usd_path}")
        open_stage(usd_path=self.usd_path)
        self.stage = omni.usd.get_context().get_stage()
        if self.stage is None:
            raise RuntimeError(
                f"씬 USD 를 열지 못했다: {self.usd_path!r}\n"
                f"  파일은 있는데 USD 가 못 읽는다 — 손상됐거나 USD 가 아닐 수 있다.\n"
                f"  `usdview` 로 직접 열어 확인할 것.")

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

        self._set_start_view()

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

    def _set_start_view(self):
        """GUI 뷰포트 카메라를 작업영역이 잘 보이는 초기 시점으로 옮긴다.

        Isaac 은 씬에 저장된 카메라를 자동으로 쓰지 않고 뷰포트 자체 perspective 로
        시작한다. 매번 손으로 돌리지 않도록 시작 시 한 번 설정한다(사용자 요청).
        방향은 **−X / +Y / +Z** 에서 턴테이블을 내려다보는 각.
        """
        if self.headless:
            return
        try:
            from omni.kit.viewport.utility import get_active_viewport
            from pxr import UsdGeom, Gf
            import numpy as _np
            try:                                  # 실제 원판 위치를 우선
                c, _ = self.prim_world_pose(DISC_PRIM)
                tgt = _np.array(c, float)
            except Exception:                     # noqa: BLE001
                tgt = _np.array(START_VIEW_TARGET, float)
            d = _np.array(START_VIEW_DIR, float)
            d = d / (_np.linalg.norm(d) + 1e-12)
            eye = tgt + d * float(START_VIEW_DIST)
            cam_path = "/World/Environment/StartCam"
            cam = UsdGeom.Camera.Define(self.stage, cam_path)
            cam.CreateFocalLengthAttr(24.0)
            f = (tgt - eye); f /= _np.linalg.norm(f)
            r = _np.cross(f, [0, 0, 1.0]); r /= _np.linalg.norm(r)
            u = _np.cross(r, f)
            M = Gf.Matrix4d(*[float(v) for row in (
                list(r) + [0.0], list(u) + [0.0], list(-f) + [0.0], list(eye) + [1.0])
                for v in row])
            x = UsdGeom.Xformable(cam.GetPrim())
            x.ClearXformOpOrder()
            x.AddTransformOp().Set(M)
            get_active_viewport().set_active_camera(cam_path)
            print(f"[IsaacWorld] 시작 시점: eye={_np.round(eye,2).tolist()} → "
                  f"target={tgt.tolist()}")
            # ★ 뷰포트 종횡비 정책 = **fit**. Kit 기본(설정 없음 = 1, match-horizontal)은
            #   가로 조리개만 지키고 세로를 창 종횡비로 잘라, 스캐너 카메라(3:4 세로형)를
            #   16:9 뷰포트로 보면 세로 28.6° 중 12° 만 보인다 — 저장되는 디버그 PNG
            #   (오프스크린 288x385)와 "비율이 다르다" 로 보인 원인(2026-09-18).
            #   fit(=2) 이면 카메라 프레임 전체를 좌우 여백을 두고 보여줘 PNG 와 같다.
            #   값 정의: omni.kit.widget.viewport/api.py (0 vertical·1 horizontal·2 fit·3 crop).
            try:
                import carb
                carb.settings.get_settings().set("/app/hydra/aperture/conform", 2)
                print("[IsaacWorld] 뷰포트 aperture conform = fit (스캐너 카메라를 PNG 와 같은 비율로)")
            except Exception as e:                               # noqa: BLE001
                print(f"[IsaacWorld][WARN] aperture conform 설정 실패(무시): {e}")
        except Exception as e:                                   # noqa: BLE001
            print(f"[IsaacWorld][WARN] 시작 시점 설정 실패(무시): {type(e).__name__}: {e}")

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
        # ★ 세로 조리개는 **해상도 종횡비에서** 잡는다 — Isaac `Camera` 가 정방 화소를
        #   강제해 어차피 이 값으로 덮어쓰기 때문이다(`SCANNER_RESOLUTION` 주석).
        #   vfov 로 직접 잡던 예전 코드는 authored 값만 바꿨고 렌더는 안 따라왔다.
        #   FOV 일치는 `_scanner_resolution()` 이 해상도 쪽에서 보장한다.
        w_px, h_px = SCANNER_RESOLUTION
        v_aperture = h_aperture * float(h_px) / float(w_px)
        vfov_eff = 2.0 * math.degrees(math.atan(v_aperture / (2.0 * focal)))

        _attr("focalLength",        Sdf.ValueTypeNames.Float, float(focal))
        _attr("horizontalAperture", Sdf.ValueTypeNames.Float, float(h_aperture))
        _attr("verticalAperture",   Sdf.ValueTypeNames.Float, float(v_aperture))
        print(f"[IsaacWorld] 스캐너 FOV {SPIDER_HFOV_DEG:.2f}deg(가로) x "
              f"{vfov_eff:.2f}deg(세로, 해상도 {SCANNER_RESOLUTION} 에서 유도; "
              f"센서 모델 {SPIDER_VFOV_DEG:.2f}deg)")
        _attr("clippingRange",      Sdf.ValueTypeNames.Float2,
              Gf.Vec2f(float(SPIDER_CLIP_RANGE[0]), float(SPIDER_CLIP_RANGE[1])))
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
