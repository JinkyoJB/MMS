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
# stage_until 해석은 sim·real·main 공용 (기본값 불일치 방지)
from utils.nbv.scan_stage_controller import (
    resolve_stage_until, stage_desc, runs_stage, STAGES as _STAGES)

# excure command:
# env -u PYTHONPATH $MMS_PYTHON main_artec.py   # 기본: ~/miniconda3/envs/env_isaacsim/bin/python

# 모든 output 파일에 같은 타임스탬프(_YYYYMMDD_HHMMSS) 붙여 run 별 구분.
RUN_TS = datetime.now().strftime("%Y%m%d_%H%M%S")
# ★ run 산출물 폴더 (2026-09-23): output/<RUN_TS>/ — aligned/(Studio 로 열 것)·final/·final.obj·
#   timeline.csv·README.txt. 예전엔 output/artec_lookaround_<RUN>{,_raw,.obj,_timeline.csv} 로
#   흩어져 있었다. 디버그 이미지도 output/<RUN_TS>/debug/<단계>/ (규칙: utils/run_paths.py).
RUN_DIR = PROJECT_ROOT / "output" / RUN_TS
# 세션·스트리밍 모듈이 run 별 산출물(이벤트 로그·원시 스캔 덤프)을 같은 태그로 묶는다.
os.environ.setdefault("MMS_RUN_TS", RUN_TS)


# ── 콘솔 로그 → 파일 tee ──────────────────────────────────────────────
# 실물 스캔은 한 번에 수 분이고 출력이 수백 줄이다. 터미널 스크롤백만 있으면
# 나중에 "그때 뭐라고 찍혔더라" 를 복원할 수 없고, 남에게 보여주려면 붙여넣어야
# 한다. `sim_harness/*` 는 이미 같은 이유로 tee 를 갖고 있었는데(Isaac 콘솔이
# 출력을 가려서) 본 파이프라인엔 없었다.
#   output/<RUN_TS>/run.log  — 항상 남는다. MMS_NO_LOGFILE=1 로 끌 수 있다.
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

        # ★ 아래 셋은 **Isaac 을 띄우기 위해** 필요하다. `SimulationApp.__init__` 이
        #   `faulthandler.enable()` 을 부르는데, 그건 sys.stderr 의 **진짜 fd** 를
        #   요구한다. 없으면 `AttributeError: '_Tee' object has no attribute
        #   'fileno'` 로 Isaac 이 뜨기도 전에 죽는다(2026-09-17).
        #   → tee 를 걸면 sim 이 아예 실행되지 않았다. `MMS_NO_LOGFILE=1` 로
        #     돌리면 통과하는 바람에 한동안 안 드러났다.
        #   원본 스트림의 fd 를 그대로 넘긴다 — faulthandler 는 콘솔로 쓰고,
        #   로그 파일에는 파이썬 레벨 write 만 남는다(네이티브 크래시 추적은
        #   콘솔 쪽에만 남는 것이 정상).
        def fileno(self):
            return self._s.fileno()

        def writable(self):
            return True

        @property
        def encoding(self):
            return getattr(self._s, "encoding", "utf-8")

    try:
        import sys as _sys
        RUN_DIR.mkdir(parents=True, exist_ok=True)   # output/<RUN_TS>/run.log
        _LOG_PATH = RUN_DIR / "run.log"
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
#   환경변수 `MMS_BACKEND` 로 덮어쓸 수 있다 — 소스를 고치지 않고 복붙 한 줄로
#   sim/real 을 바꾸기 위해서다(`docs/sim_commands.md` §0).
BACKEND = os.environ.get("MMS_BACKEND", "isaac").strip().lower()
if BACKEND not in ("real", "isaac"):
    raise SystemExit(f"[main] MMS_BACKEND={BACKEND!r} — 'real' 또는 'isaac' 이어야 한다")
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
        # ── 작동거리 창 (mm) — **실험 중 여기만 바꾼다** ──────────────────
        #   None = SDK 기본값. 값을 주면 ① 단일캡처(FrameProcessor)
        #   ② 스트리밍 스캔(ScanSession) ③ 계획기 `SensorModel.dof` 가 **모두**
        #   이 값을 따른다. 자세한 건 README '작동거리 창을 바꾸려면'.
        #   실측 로그: `[ArtecClient] 작동거리 창 = ???~???mm`
        scan_range_near_mm=None,
        scan_range_far_mm=None,
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
    # ── Streaming lookaround (Artec IScanningProcedure 기반, 연속 회전) ──────
    STREAM_SETTINGS = ArtecStreamingScanSessionSettings(
        rotation_duration_s=30.0,         # 30초에 한 바퀴
        rotation_overshoot_deg=5.0,
        target_fps=None,                  # None = scanner.max_fps()
        capture_texture=True,
        # ★ True 여야 한다 (2026-09-22 실측, run_154059). False 로 바꿔 봤더니 SDK 가
        #   회전 중인 턴테이블 장면에서 8프레임(≈1s, θ≈−10°) 만에 REGISTRATION_FAILED 를
        #   연속으로 내고 registration_error 도 0.000 만 찍혀 밴드 1 시작 직후 lost
        #   (4회 반복, 0/4 밴드). True 인 예전 run 들은 같은 자세·속도에서 fail=0,
        #   regErr≈+0.25 로 밴드 1 을 끝까지 돌았다. 즉 SDK 의 실시간 프레임 정합은
        #   True 모드(실패 프레임도 예측 자세로 넣고 다음 프레임은 그것에 대고 정합)
        #   를 전제로 굴러가고, False 는 마지막 *정합된* 프레임까지의 간격이 벌어져
        #   연쇄 실패한다. lost 판정은 우리 tracker 의 "regErr<0 연속 5" 로 한다.
        #   대가: lost 뒤 꼬리 프레임이 스캔에 섞인다 → 세션 결과의 n_tail_lost 로 잘라낸다.
        ignore_registration_errors=True,
        preview_settle_s=1.5,
        post_record_settle_s=0.5,
        reset_to_zero_first=True,
        timeline_csv_path=str(RUN_DIR / "timeline.csv"),
    )

    # ── Multi-pass: stage_until = **여기까지** 순차 실행 ──────────────────
    #   preview → lookaround → nbv → flip  (앞 단계는 항상 포함된다)
    # ★ 첫 Spider 테스트는 "lookaround" 로 5면 확인 후 nbv→flip 으로 올릴 것.
    # pose_physical_rotations = flip(바닥면 flip) 손회전 설정.
    POSE_ROTATIONS = make_axis_physical_rotations("y", [0.0, 90.0, 180.0])
    MULTIPASS_SETTINGS = ArtecMultiPassScanSessionSettings(
        stage_until="flip",            # preview→lookaround→nbv→flip 전부
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
        # ★ SDK Texturize 는 **CPU 단일코어**라 1900 프레임에 15~20분(2026-09-22 실측, GPU
        #   옵션 없음). 기본 끔 — 최종 sproj(융합 메시 + 텍스처 프레임)를 Artec Studio 에서
        #   열어 Texture(GPU) 하는 것이 수십 초. SDK 로 하려면 `--texturize`.
        do_texturize=False,
        export_obj_path=str(RUN_DIR / "final.obj"),
        # ★ sproj 저장 **기본 켬**(2026-09-22). 두 개가 남는다 — output/<RUN>/aligned/aligned.sproj
        #   (파이프라인 변환이 적용된 IScan 만, SDK 후처리 전 → Artec Studio 로 열어 수작업
        #   후처리하는 파일) 과 final/final.sproj(우리 후처리 결과). 각 ~47s.
        #   빠른 반복 테스트면 `--no-sproj`.
        export_sproj_path=str(RUN_DIR / "final.sproj"),
    )
elif CFG.backend == "isaac":
    # isaac: Artec SDK scan-settings 없이 sim 스캔(IsaacScanSession) 실행.
    # IsaacScanSession = lookaround GT 누적 + nbv(공용 nbv_core/robot_collision).
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

            # stage_until 해석은 **공용 함수 한 곳**에서만 한다 — 예전엔 여기와
            # isaac_scan_session 이 각자 해석했고 기본값도 달라(1 vs 2), 설정이 빠지면
            # 화면에 찍히는 단계와 실제 도는 단계가 갈렸다.
            _pm, _pm_src = resolve_stage_until(
                MULTIPASS_SETTINGS if _SCAN_SETTINGS_AVAILABLE else None,
                allow_env=(CFG.backend == "isaac"))   # env override 는 sim 만
            _pm_desc = stage_desc(_pm)
            print(f"\n[main] === Artec {_pm_desc}  ({_pm_src}) ===")
            print(f"  T_EC: {CFG.T_EC_key}")
            print(f"  fusion: {PROCESS_SETTINGS.fusion}")
            print(f"  run 폴더: {RUN_DIR}")
            print(f"  export OBJ:  {PROCESS_SETTINGS.export_obj_path}")
            _sp = PROCESS_SETTINGS.export_sproj_path
            print(f"  export sproj: {_sp + '  (+ aligned/aligned.sproj — Studio 용)' if _sp else '끔 (--no-sproj)'}")

            # 흐름: home → preview→lookaround→nbv→flip → home  (sim/real 공통)
            #       어디까지 갈지는 --until / stage_until 이 정한다.
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

            _go_home(robot, confirm_home, "종료")    # flip 후 home 복귀

            # ★ 결과 창은 **캡처를 실제로 한 경우에만** 띄운다.
            #   `stage_until="preview"` 는 계획만 내고 끝나므로 모델이 비어 있다.
            #   그때 창이 뜨면 (a) 빈 화면을 보여주고 (b) 제목의 단계 이름 때문에
            #   "lookaround 까지 돌았나?" 로 오해하게 된다 — 실제로 그랬다.
            _n_scan = 0
            if result is not None:
                try:
                    _n_scan = int(result.model.scan_count())
                except Exception:                             # noqa: BLE001
                    _n_scan = 0
            if result is None or os.environ.get("MMS_SIM_NO_VIZ") == "1":
                pass
            elif not runs_stage(_pm, "lookaround"):
                print(f"[main] 캡처 단계를 안 돌았다 (stage_until={_pm!r}) "
                      f"— 결과 창 생략")
            elif _n_scan == 0:
                print("[main] 스캔 결과가 비었다 (scans=0) — 결과 창 생략")
            elif os.environ.get("MMS_ISAAC_HEADLESS", "0") == "1":
                # ★ 헤드리스면 결과 창을 띄우지 않는다. `show_composite_mesh` 는
                #   Open3D 창을 열고 **Q/ESC 를 누를 때까지 블록**한다 — 배치·CI·
                #   nohup 실행은 눌러 줄 사람이 없어 거기서 영원히 멎는다
                #   (2026-09-17 실측: 전체 파이프라인이 정상 종료해놓고 22분 대기).
                #   헤드리스는 "창 없이" 라는 뜻이므로 여기서도 지켜야 한다.
                print(f"[main] 헤드리스 — 결과 창 생략 "
                      f"(메시: {PROCESS_SETTINGS.export_obj_path})")
            else:
                show_composite_mesh(
                    result, obj_path=PROCESS_SETTINGS.export_obj_path,
                    title=f"Artec 스캔 결과 — {_pm} 까지")

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


def _spawn_range_view(record: bool = False) -> None:
    """거리추종·nbv 디버그 이미지 뷰어를 **자식 프로세스**로 띄운다.

    점군 뷰어(`live_scan_view.py`)와 달리 자식으로 띄워도 된다 — 이쪽은
    Filament/Open3D 가 아니라 OpenCV HighGUI 로 이미 저장된 PNG 를 tail 할
    뿐이라, 자식 GUI 가 죽던 그 제약이 없다. 실패해도 스캔에는 영향이 없다.
    """
    import subprocess
    script = PROJECT_ROOT / "scripts" / "artec" / "live_range_view.py"
    if not script.is_file():
        return
    try:
        import sys as _sys
        kw = {}
        if os.name == "nt":
            # 새 콘솔로 떼어 낸다 — 부모가 Ctrl+C 로 죽어도 창이 같이 죽지 않게.
            kw["creationflags"] = (getattr(subprocess, "CREATE_NEW_PROCESS_GROUP", 0)
                                   | getattr(subprocess, "DETACHED_PROCESS", 0))
        argv = [_sys.executable, str(script), "--run", RUN_TS]
        if record:
            argv.append("--record")
        # ★ 자식 출력을 **파일로** 남긴다. DEVNULL 로 버리면 뷰어가 죽어도 흔적이
        #   없어서 "창이 안 뜬다" 를 추적할 수 없다(2026-09-22).
        _vlog = RUN_DIR / "debug" / "range_view.log"
        _vlog.parent.mkdir(parents=True, exist_ok=True)
        _vf = open(_vlog, "w", encoding="utf-8", buffering=1)
        argv.insert(1, "-u")                      # 줄 단위 출력 (죽어도 남게)
        # 자식도 utf-8 로 — 파이프/파일 출력이면 기본이 로케일(cp949)이라 한글·기호에서
        # 죽는다(2026-09-23). 뷰어 안에서도 reconfigure 하지만 양쪽 다 걸어 둔다.
        _env = {**os.environ, "PYTHONIOENCODING": "utf-8"}
        subprocess.Popen(
            argv, cwd=str(PROJECT_ROOT), stdin=subprocess.DEVNULL,
            stdout=_vf, stderr=subprocess.STDOUT, env=_env, **kw)
        print(f"[main] 디버그 이미지 창 실행 — output/{RUN_TS}/debug/<단계>/ 를 tail "
              f"(끄려면 --no-range-view)")
        if record:
            print(f"[main]   녹화 → output/{RUN_TS}/debug/range.avi")
        print(f"[main]   창이 안 뜨면 → output/{RUN_TS}/debug/range_view.log 확인")
    except Exception as e:                                       # noqa: BLE001
        print(f"[main] ⚠ 디버그 이미지 창 실행 실패({type(e).__name__}: {e}) — 수동: "
              f"python scripts/artec/live_range_view.py --run {RUN_TS}")


def _apply_cli() -> None:
    """실물 파이프라인을 **단계별로** 돌리기 위한 인자.

    ★ 왜 필요한가 — 실물 스캔은 한 번에 수 분이고, 중간에 뭔가 틀리면 어느
      단계가 문제인지 구분이 안 된다. 예전엔 `stage_until`·`max_passes` 같은
      스위치가 이 파일에 **하드코딩**돼 있어서 매번 소스를 고쳐야 했다.
      기본값은 그대로이므로 인자 없이 실행하면 동작이 바뀌지 않는다.
    """
    import argparse

    # `MULTIPASS_SETTINGS` 는 `if _SCAN_SETTINGS_AVAILABLE:` 안에서 정의된다
    # (바인딩 미빌드 등으로 없을 수 있다).
    m = globals().get("MULTIPASS_SETTINGS")

    ap = argparse.ArgumentParser(
        description="Artec MMS 실물 파이프라인 (단계별 실행 인자)")
    ap.add_argument("--until", dest="until", choices=_STAGES, default=None,
                    help="여기까지 실행 (preview → lookaround → nbv → flip) "
                         + (f"(기본 {m.stage_until})" if m is not None else ""))
    ap.add_argument("--max-passes", type=int, default=None,
                    help="pass 상한. **1 로 주면 한 자세만** 돌고 끝난다 — "
                         "회전·캡처 한 사이클만 확인할 때")
    ap.add_argument("--no-planner", action="store_true",
                    help="lookaround 자세 플래너를 끈다 (preview 수집·계획 생략, "
                         "home 고정). 캡처 루프만 떼어 볼 때")
    ap.add_argument("--no-prompt", action="store_true",
                    help="pass 사이 Enter 확인을 생략 (무인 연속 실행)")
    ap.add_argument("--no-viewer", action="store_true",
                    help="라이브 뷰어 스냅샷을 끈다")
    ap.add_argument("--no-range-view", action="store_true",
                    help="거리추종·nbv 디버그 이미지 창(자동 실행)을 띄우지 않는다")
    ap.add_argument("--range-video", action="store_true",
                    help="그 창이 본 것을 동영상으로도 남긴다 (output/<RUN>/debug/range.avi). "
                         "기본은 끔 — PNG 는 어차피 남으므로 나중에 "
                         "`live_range_view.py --run <RUN> --make-video` 로도 만들 수 있다")
    ap.add_argument("--no-recovery", action="store_true",
                    help="tracking lost 자동 복구를 끈다 — 복구 로직을 배제하고 "
                         "원래 스캔이 되는지만 볼 때")
    ap.add_argument("--speed-scale", type=float, default=None, metavar="K",
                    help="로봇 이동 속도를 K 배로 (예: 0.8). 세 곳에 흩어진 "
                         "속도(계획용·NBV·복구)를 한 번에 조절한다")
    ap.add_argument("--no-sproj", action="store_true",
                    help="Artec .sproj 저장 생략 (기본은 output/<RUN>/aligned + final 저장, 각 ~47초)")
    ap.add_argument("--texturize", action="store_true",
                    help="SDK Texturize 실행 (기본 끔 — CPU 단일코어 15~20분. 대신 최종 "
                         "sproj 를 Artec Studio 에서 텍스처링, docs/6_postprocess.md §5)")
    ap.add_argument("--test", action="store_true",
                    help="반복 테스트용 — **texturize 생략**. 실측 181초가 빠진다. "
                         "메시 형상만 확인할 때")
    a = ap.parse_args()

    # ── 후처리 스위치 (PROCESS_SETTINGS) — MULTIPASS 유무와 무관 ──────
    ps = globals().get("PROCESS_SETTINGS")
    if ps is not None:
        if a.no_sproj:
            ps.export_sproj_path = None
            print("[main] --no-sproj: sproj 저장 생략")
        if a.texturize:
            ps.do_texturize = True
            print("[main] --texturize: SDK Texturize ON (CPU, 15~20분)")
        if a.test:
            ps.do_texturize = False
            print("[main] --test: texturize 생략 (형상만 확인)")

    if m is None:
        if any([a.until, a.max_passes, a.no_planner, a.no_prompt,
                a.no_viewer, a.no_recovery]):
            print("[main] ⚠ 스캔 설정을 못 불러왔다(바인딩 미빌드?) — 인자 무시")
        return
    if a.until is not None:
        m.stage_until = str(a.until)
    if a.max_passes is not None:
        m.max_passes = int(a.max_passes)
    if a.no_planner:
        m.lookaround_planner_enabled = False
    if a.no_prompt:
        m.prompt_before_first_pass = False
        m.prompt_between_passes = False
        m.prompt_on_tracking_lost = False
    if a.no_viewer:
        m.enable_live_viewer = False
    if not a.no_range_view:
        _spawn_range_view(record=a.range_video)
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

    print(f"[main] stage_until={m.stage_until}  max_passes={m.max_passes}  "
          f"planner={m.lookaround_planner_enabled}  recovery={m.auto_recovery_enabled}  "
          f"prompt={m.prompt_between_passes}  viewer={m.enable_live_viewer}  "
          f"range_view={not a.no_range_view}"
          + ("" if a.no_range_view else f"/rec={a.range_video}"))


if __name__ == "__main__":
    _apply_cli()
    main()
