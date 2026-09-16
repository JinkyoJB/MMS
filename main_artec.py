try:
    import msvcrt                       # Windows 전용 (실물 환경)
except ImportError:
    msvcrt = None                       # Linux/Isaac — 키 입력 폴링 비활성
import os
import traceback
from datetime import datetime

import numpy as np

from utils import PROJECT_ROOT
from utils.viz import show_composite_mesh   # open3d 는 함수 안에서 lazy import
# phase_mode 해석은 sim·real·main 공용 (기본값 불일치 방지)
from utils.nbv.scan_phase_controller import resolve_phase_mode, phase_desc

# excure command:
# env -u PYTHONPATH $MMS_PYTHON main_artec.py   # 기본: ~/miniconda3/envs/env_isaacsim/bin/python

# 모든 output 파일에 같은 타임스탬프(_YYYYMMDD_HHMMSS) 붙여 run 별 구분.
RUN_TS = datetime.now().strftime("%Y%m%d_%H%M%S")


# ── 콘솔 로그 → 파일 tee ──────────────────────────────────────────────
# 실물 스캔은 한 번에 수 분이고 출력이 수백 줄이다. 터미널 스크롤백만 있으면
# 나중에 "그때 뭐라고 찍혔더라" 를 복원할 수 없고, 남에게 보여주려면 붙여넣어야
# 한다. `sim_harness/*` 는 이미 같은 이유로 tee 를 갖고 있었는데(Isaac 콘솔이
# 출력을 가려서) 본 파이프라인엔 없었다.
#   output/run_<TS>.log  — 항상 남는다. MMS_NO_LOGFILE=1 로 끌 수 있다.
import time as _time

_LOG_PATH = None
if os.environ.get("MMS_NO_LOGFILE") != "1":
    class _Tee:
        """stdout/stderr 를 파일에도 쓰고, **줄머리에 경과시간을 붙인다.**

        ★ 시간 표시를 여기 두는 이유 — 파이프라인 전역에 print 가 수백 군데다.
          Tee 한 곳에서 붙이면 **기존 출력 전부**가 자동으로 타임스탬프를 갖는다.
          어느 구간에서 시간이 새는지(대기·sleep·SDK 블로킹) 로그만 보고 잡으려면
          이게 있어야 한다 — 2026-09-16 밴드 사이 지연 추적.
        """

        _t0 = _time.perf_counter()

        def __init__(self, stream, fh):
            self._s, self._f = stream, fh
            self._at_line_start = True

        def _stamp(self, s: str) -> str:
            if not s:
                return s
            out, parts = [], s.split("\n")
            for i, part in enumerate(parts):
                if self._at_line_start and part:
                    out.append(f"[{_time.perf_counter() - _Tee._t0:7.1f}s] {part}")
                    self._at_line_start = False
                else:
                    out.append(part)
                if i < len(parts) - 1:
                    out.append("\n")
                    self._at_line_start = True
            return "".join(out)

        def write(self, s):
            s = self._stamp(s)
            try:
                self._s.write(s)
            except Exception:                                  # noqa: BLE001
                pass
            try:
                self._f.write(s)
                self._f.flush()          # 중단되더라도 마지막 줄까지 남게
            except Exception:                                  # noqa: BLE001
                pass

        def flush(self):
            for t in (self._s, self._f):
                try:
                    t.flush()
                except Exception:                              # noqa: BLE001
                    pass

        def isatty(self):
            return getattr(self._s, "isatty", lambda: False)()

    try:
        import sys as _sys
        _LOG_DIR = PROJECT_ROOT / "output"
        _LOG_DIR.mkdir(parents=True, exist_ok=True)
        _LOG_PATH = _LOG_DIR / f"run_{RUN_TS}.log"
        _fh = open(_LOG_PATH, "w", encoding="utf-8", buffering=1)
        _sys.stdout = _Tee(_sys.stdout, _fh)
        _sys.stderr = _Tee(_sys.stderr, _fh)
        print(f"[main] 콘솔 로그 → {_LOG_PATH}")
    except Exception as _e:                                    # noqa: BLE001
        print(f"[main] ⚠ 로그 파일 생성 실패({_e}) — 콘솔만")
from mms_artec.system import ArtecMMS, ArtecMMSConfig, ArtecProcessSettings
from mms_artec.sensor.artec_config import ArtecConfig   # 바인딩 비의존(경량)

# ── 백엔드 선택 ───────────────────────────────────────────────────────
#   "real"  → 실물 xArm + 턴테이블 + Artec 스캐너 (Windows)
#   "isaac" → Isaac Sim 시뮬레이션 (옆에 실물 없이 개발)
BACKEND = "real"
# ★ 여기(BACKEND)는 **사람이 바꾸는 스위치**일 뿐이다. CFG 를 만든 뒤부터는
#   진실의 출처가 `CFG.backend` 하나다 — 코드에서 백엔드를 분기할 때는
#   반드시 CFG.backend 를 쓸 것(둘을 섞으면 나중에 갈라진다).
# isaac GUI 표시 여부. 환경변수 MMS_ISAAC_HEADLESS=1 로 헤드리스 강제(서버/CI).
ISAAC_HEADLESS = os.environ.get("MMS_ISAAC_HEADLESS", "0") == "1"


try:
    from mms_artec.nbv.artec_streaming_scan_session import ArtecStreamingScanSessionSettings
    from mms_artec.nbv.artec_multipass_scan_session import (
        ArtecMultiPassScanSessionSettings,
        make_axis_physical_rotations,
    )
    _SCAN_SETTINGS_AVAILABLE = True
except Exception as _e:               # noqa: BLE001
    # scan settings 는 Artec SDK 바인딩에 의존 → real SDK 없으면 import 실패.
    # isaac 은 이게 False 여도 아래 `elif CFG.backend=="isaac"` 로 sim 스캔을 빌드한다.
    print(f"[main] ⓘ scan settings import 불가 ({type(_e).__name__}) — "
          f"real Artec 스캔 비활성 (isaac 은 sim 스캔으로 진행).")
    _SCAN_SETTINGS_AVAILABLE = False

# ── 하드웨어 ──────────────────────────────────────────────────────────
ROBOT_IP        = "192.168.1.210"
TURNTABLE_IP    = "192.168.0.10"
TURNTABLE_BD_ID = 0

# ── MMS Artec 설정 ────────────────────────────────────────────────────
CFG = ArtecMMSConfig(
    artec=ArtecConfig(
        serial_number=None,                 # None → 첫 번째 스캐너 (Spider SP.10.79103441)
        capture_texture=True,
        target_interval_s=0.0,
    ),
    turntable_frame_yaml=str(PROJECT_ROOT / "config/calibration/turntable_frame.yaml"),
    sensor_frames_yaml=str(PROJECT_ROOT / "config/sensor_frames.yaml"),
    T_EC_key="T_EC_artec",                  # 2026-04-29 hand-eye 결과
    # ── 백엔드 ──────────────────────────────────────────────────────────
    backend=BACKEND,
    robot_ip=ROBOT_IP,
    turntable_ip=TURNTABLE_IP,
    turntable_bd_id=TURNTABLE_BD_ID,
    isaac_headless=ISAAC_HEADLESS,
    isaac_usd_path=os.environ.get("MMS_SIM_USD") or None,   # testset 합성 씬 지정용
)

DEV_MODE = True

# 스캔 settings 는 Artec 바인딩/open3d 가 있을 때만 구성한다 (real 환경).
# isaac(Phase A) 에선 None — 모션/제어 개발만, 스캐너는 Phase B.
PROCESS_SETTINGS = None
if _SCAN_SETTINGS_AVAILABLE:
    # ── Streaming Phase 1 (Artec IScanningProcedure 기반, 연속 회전) ──────
    STREAM_SETTINGS = ArtecStreamingScanSessionSettings(
        rotation_duration_s=30.0,         # 30초에 한 바퀴
        rotation_overshoot_deg=5.0,
        target_fps=None,                  # None = scanner.max_fps()
        capture_texture=True,
        ignore_registration_errors=True,
        preview_settle_s=1.5,
        post_record_settle_s=0.5,
        reset_to_zero_first=True,
        timeline_csv_path=str(PROJECT_ROOT / f"output/artec_phase1_{RUN_TS}_timeline.csv"),
    )

    # ── Multi-pass: phase_mode = Phase 1 부터 **순차 누적** 실행 ────────────
    #   1 = Phase1(5면) / 2 = Phase1→2(NBV) / 3 = Phase1→2→3(바닥면 flip)
    # ★ 첫 Spider 테스트는 phase_mode=1 로 5면 확인 후 2→3 으로 올릴 것(2·3 미검증).
    # pose_physical_rotations = Phase 3(바닥면 flip) 손회전 설정.
    POSE_ROTATIONS = make_axis_physical_rotations("y", [0.0, 90.0, 180.0])
    MULTIPASS_SETTINGS = ArtecMultiPassScanSessionSettings(
        phase_mode=3,                 # 순차 누적: 1=5면 / 2=+NBV / 3=+바닥면 flip
        streaming_settings=STREAM_SETTINGS,
        pose_physical_rotations=POSE_ROTATIONS,
        max_passes=8,
        prompt_before_first_pass=True,
        prompt_between_passes=True,
        prompt_on_tracking_lost=True,
        auto_recovery_enabled=True,
        max_recovery_retries=3,
        safe_back_margin_deg=10.0,
        recovery_robot_speed_deg_s=10.0,
        recovery_turntable_vel_rad_s=float(np.radians(30.0)),
        recovery_elevation_offsets_deg=[-5.0, 0.0, 5.0],
        recovery_elevation_fine_search_enabled=False,
        enable_live_viewer=True,
        probe_debug_dump=True,
    )

    PROCESS_SETTINGS = ArtecProcessSettings(
        dev_mode=DEV_MODE,
        streaming_scan_settings=STREAM_SETTINGS,
        multipass_settings=MULTIPASS_SETTINGS,
        do_serial_registration=False,
        do_global_registration=True,
        fusion="poisson",
        do_outliers_removal=True,
        do_small_objects_filter=True,
        do_simplify=False,
        do_texturize=True,
        export_obj_path=str(PROJECT_ROOT / f"output/artec_phase1_{RUN_TS}.obj"),
        # ★ sproj 저장은 **기본 끔**. 실측 458초 실행에서 sproj 저장에만 47초가
        #   들었다(중간 저장 포함하면 더). 필요할 때 `--sproj` 로 켠다.
        export_sproj_path=None,
    )
elif CFG.backend == "isaac":
    # isaac: Artec SDK scan-settings 없이 sim 스캔(IsaacScanSession) 실행.
    # IsaacScanSession = Phase1 GT 누적 + Phase2 NBV(공용 phase2_nbv/robot_collision).
    # 후처리(GlobalReg/Fusion/Texturize)는 sim sensor stub 가 skip → 결과=점군/mesh.
    PROCESS_SETTINGS = ArtecProcessSettings(
        dev_mode=DEV_MODE, use_multipass_scan=True,
        do_serial_registration=False, do_global_registration=False, fusion="none",
        do_outliers_removal=False, do_small_objects_filter=False,
        do_simplify=False, do_texturize=False,
        export_obj_path=str(PROJECT_ROOT / f"output/sim_scan_{RUN_TS}.ply"),
    )


def _go_home(robot, confirm: bool, tag: str) -> None:
    """robot 을 Artec home 자세로 이동. 실패해도 흐름 유지(스캔/정리 안 끊기게).

    ┌─ ★ home 관절값을 바꾸려면 (여기 아님 — 백엔드별 robot 인터페이스의 dict) ──────────┐
    │  real : utils/robot/xarm_interface.py        → HOME_JOINTS_DEG["artec"]            │
    │  sim  : mms_artec/backends/isaac/isaac_xarm.py → HOME_JOINTS_DEG["artec"]          │
    │  값 = J1..J7 (deg). sim 은 충돌-free 로 IK 재산출한 별도값(실물과 다름, 의도된 것). │
    │  real 셀을 옮기거나 마운트가 바뀌면 real 쪽 dict 만 수정. sensor="phoxi" 는 다른 키.│
    └────────────────────────────────────────────────────────────────────────────────────┘
    """
    try:
        robot.go_home(sensor="artec", confirm=confirm)
        print(f"[main] ✔ home 자세 ({tag})")
    except Exception as e:
        print(f"[main] ⚠ go_home({tag}) 실패: {type(e).__name__}: {e}")


def main() -> None:
    # home 이동 전 사용자 확인: real=True(실로봇 안전), sim=False(자동).
    confirm_home = (CFG.backend == "real")

    robot = turntable = None
    try:
        with ArtecMMS(CFG) as mms:
            # robot/turntable 은 backend 에 맞춰 팩토리로 생성
            robot, turntable = mms.create_hardware()

            print(f"\n[main] === Artec MMS (backend={CFG.backend}) ===")
            # 턴테이블 축(T_B_F0) = config/calibration/turntable_frame.yaml 에서 자동 로드.
            # 재캘리브가 필요하면 rim-click 스크립트(파일 상단 주석 참고)를 별도 실행할 것.

            if PROCESS_SETTINGS is None:
                # real 백엔드인데 Artec SDK(scan settings) import 실패 → 스캔 불가.
                print("\n[main] ✘ 스캔 불가 — Artec SDK(scan settings) 를 import 하지 못했습니다.\n"
                      "  · real: Artec SDK 바인딩 / open3d 설치를 확인하세요.\n"
                      "  · sim:  BACKEND='isaac' 로 실행하면 sim 스캔이 빌드됩니다.")
                return

            # phase_mode 해석은 **공용 함수 한 곳**에서만 한다 — 예전엔 여기와
            # isaac_scan_session 이 각자 해석했고 기본값도 달라(1 vs 2), 설정이 빠지면
            # 화면에 찍히는 단계와 실제 도는 단계가 갈렸다.
            _pm, _pm_src = resolve_phase_mode(
                MULTIPASS_SETTINGS if _SCAN_SETTINGS_AVAILABLE else None,
                allow_env=(CFG.backend == "isaac"))   # env override 는 sim 만
            _pm_desc = phase_desc(_pm)
            print(f"\n[main] === Artec {_pm_desc}  ({_pm_src}) ===")
            print(f"  T_EC: {CFG.T_EC_key}")
            print(f"  fusion: {PROCESS_SETTINGS.fusion}")
            print(f"  export OBJ:  {PROCESS_SETTINGS.export_obj_path}")
            print(f"  export sproj: "
                  f"{PROCESS_SETTINGS.export_sproj_path or '끔 (--sproj 로 켜기)'}")

            # 흐름: home → Phase 1→2→3 → home  (sim/real 공통)
            _go_home(robot, confirm_home, "시작")

            result = None
            try:
                result = mms.artec_process(robot, turntable, settings=PROCESS_SETTINGS)
                # streaming 모드에선 result.ctx=None. model 에서 직접 집계.
                n_frames = sum(result.model.get_scan(i).frame_count()
                               for i in range(result.model.scan_count()))
                print(f"\n[main] {_pm_desc} 완료 — frames={n_frames}  "
                      f"scans={result.model.scan_count()}")
            except KeyboardInterrupt:
                print("\n[main] ⚠ KeyboardInterrupt — 현재 상태까지 보존")
            except Exception as e:
                print(f"\n[main] ✘ {type(e).__name__}: {e}")
                traceback.print_exc()

            _go_home(robot, confirm_home, "종료")    # Phase 3 후 home 복귀

            if result is not None and os.environ.get("MMS_SIM_NO_VIZ") != "1":
                show_composite_mesh(
                    result, obj_path=PROCESS_SETTINGS.export_obj_path)

    finally:
        # ── 정리 (CRITICAL) ───────────────────────────────────────────
        # ⚠ 이 블록에서 **`return` 하지 말 것.** finally 안의 return 은
        #   (1) 전파 중인 예외를 조용히 삼키고 — create_hardware() 가 던진 에러가
        #       트레이스백 없이 사라져 정상 종료처럼 보인다,
        #   (2) 그 아래 정리를 통째로 건너뛴다 — robot.disconnect() 와
        #       shutdown_isaac_world() 가 실행되지 않는다.
        #   예전에 `if turntable is None: return` 이 있었고, 하필 **하드웨어 생성이
        #   실패한 경로**(turntable=None)에서 Isaac 종료가 빠져 atexit crash 로 이어졌다.
        #   각 단계는 개별 None 가드 + try 로만 보호한다.
        if turntable is not None:
            # 턴테이블: disconnect 만 하면 모터는 계속 돌아감.
            # 반드시 stop → servo OFF → disconnect.
            try:
                turntable.stop()
                print("[main] turntable.stop() OK")
            except Exception as e:
                print(f"[main] ⚠ turntable.stop() 예외: {e}")
                try:
                    if hasattr(turntable, "reconnect"):
                        turntable.reconnect()
                    turntable.stop()
                    print("[main] turntable.stop() 재시도 OK")
                except Exception as e2:
                    print(f"[main] ✘ 최종 stop 실패: {e2}  — 물리적 정지/전원차단 필요")
            try:
                turntable.set_servo_on(False)
            except Exception:
                pass
            try:
                turntable.disconnect()
            except Exception:
                pass
        if robot is not None:
            try:
                robot.disconnect()
            except Exception:
                pass
        # isaac 백엔드: SimulationApp 명시적 종료 (atexit crash 방지).
        # 하드웨어 생성 실패로 turntable/robot 이 None 이어도 **반드시** 실행돼야 한다.
        if CFG.backend == "isaac":
            try:
                from mms_artec.backends import shutdown_isaac_world
                shutdown_isaac_world()
            except Exception:
                pass


def _apply_cli() -> None:
    """실물 파이프라인을 **단계별로** 돌리기 위한 인자.

    ★ 왜 필요한가 — 실물 스캔은 한 번에 수 분이고, 중간에 뭔가 틀리면 어느
      단계가 문제인지 구분이 안 된다. 예전엔 `phase_mode`·`max_passes` 같은
      스위치가 이 파일에 **하드코딩**돼 있어서 매번 소스를 고쳐야 했다.
      기본값은 그대로이므로 인자 없이 실행하면 동작이 바뀌지 않는다.
    """
    import argparse

    # `MULTIPASS_SETTINGS` 는 `if _SCAN_SETTINGS_AVAILABLE:` 안에서 정의된다
    # (바인딩 미빌드 등으로 없을 수 있다).
    m = globals().get("MULTIPASS_SETTINGS")

    ap = argparse.ArgumentParser(
        description="Artec MMS 실물 파이프라인 (단계별 실행 인자)")
    ap.add_argument("--phase", type=int, choices=(1, 2, 3), default=None,
                    help="1=Phase1(5면) · 2=+NBV · 3=+바닥면 flip "
                         + (f"(기본 {m.phase_mode})" if m is not None else ""))
    ap.add_argument("--max-passes", type=int, default=None,
                    help="pass 상한. **1 로 주면 한 자세만** 돌고 끝난다 — "
                         "회전·캡처 한 사이클만 확인할 때")
    ap.add_argument("--no-planner", action="store_true",
                    help="Phase 1 자세 플래너를 끈다 (preview 수집·계획 생략, "
                         "home 고정). 캡처 루프만 떼어 볼 때")
    ap.add_argument("--no-prompt", action="store_true",
                    help="pass 사이 Enter 확인을 생략 (무인 연속 실행)")
    ap.add_argument("--no-viewer", action="store_true",
                    help="라이브 뷰어 스냅샷을 끈다")
    ap.add_argument("--no-recovery", action="store_true",
                    help="tracking lost 자동 복구를 끈다 — 복구 로직을 배제하고 "
                         "원래 스캔이 되는지만 볼 때")
    ap.add_argument("--speed-scale", type=float, default=None, metavar="K",
                    help="로봇 이동 속도를 K 배로 (예: 0.8). 세 곳에 흩어진 "
                         "속도(계획용·NBV·복구)를 한 번에 조절한다")
    ap.add_argument("--sproj", action="store_true",
                    help="Artec .sproj 도 저장한다 (기본 끔 — 실측 47초 소요)")
    ap.add_argument("--test", action="store_true",
                    help="반복 테스트용 — **texturize 생략**. 실측 181초가 빠진다. "
                         "메시 형상만 확인할 때")
    a = ap.parse_args()

    # ── 후처리 스위치 (PROCESS_SETTINGS) — MULTIPASS 유무와 무관 ──────
    ps = globals().get("PROCESS_SETTINGS")
    if ps is not None:
        if a.sproj:
            ps.export_sproj_path = str(
                PROJECT_ROOT / f"output/artec_phase1_{RUN_TS}.sproj")
            print(f"[main] sproj 저장 ON → {ps.export_sproj_path}")
        if a.test:
            ps.do_texturize = False
            print("[main] --test: texturize 생략 (형상만 확인)")

    if m is None:
        if any([a.phase, a.max_passes, a.no_planner, a.no_prompt,
                a.no_viewer, a.no_recovery]):
            print("[main] ⚠ 스캔 설정을 못 불러왔다(바인딩 미빌드?) — 인자 무시")
        return
    if a.phase is not None:
        m.phase_mode = int(a.phase)
    if a.max_passes is not None:
        m.max_passes = int(a.max_passes)
    if a.no_planner:
        m.phase1_planner_enabled = False
    if a.no_prompt:
        m.prompt_before_first_pass = False
        m.prompt_between_passes = False
        m.prompt_on_tracking_lost = False
    if a.no_viewer:
        m.enable_live_viewer = False
    if a.no_recovery:
        m.auto_recovery_enabled = False
    if a.speed_scale is not None:
        k = float(a.speed_scale)
        if not (0.05 <= k <= 3.0):
            print(f"[main] ⚠ --speed-scale {k} 은 범위(0.05~3.0) 밖 — 무시")
        else:
            # 로봇 이동 속도는 목적별로 세 값이다. 배율만 받아 일괄 적용한다 —
            # 하나만 바꾸면 계획용은 느린데 NBV 는 빠른 식으로 어긋난다.
            for name in ("adaptive_robot_speed_deg_s", "nbv_robot_speed_deg_s",
                         "recovery_robot_speed_deg_s"):
                if hasattr(m, name):
                    old = float(getattr(m, name))
                    setattr(m, name, old * k)
                    print(f"[main] {name}: {old:.1f} → {old*k:.1f} deg/s")

    print(f"[main] phase_mode={m.phase_mode}  max_passes={m.max_passes}  "
          f"planner={m.phase1_planner_enabled}  recovery={m.auto_recovery_enabled}  "
          f"prompt={m.prompt_between_passes}  viewer={m.enable_live_viewer}")


if __name__ == "__main__":
    _apply_cli()
    main()
