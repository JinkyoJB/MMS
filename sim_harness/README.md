# sim_harness — Isaac Sim 검증 하니스

알고리즘을 실물에 올리기 전에 **ground-truth 를 아는 가상환경에서 먼저 확인**하는
스크립트들. `docs/1_calibration.md` · `2_phase1.md` · `3_phase2.md` 가 이 파일들을 참조한다.

| 파일 | 검증 대상 | 관련 문서 |
|---|---|---|
| `MMS_ext.py` | 기본 환경·카메라·드라이브 게인 (나머지의 베이스) | — |
| `MMS_ext_calibration.py` | **hand-eye** — ChArUco 렌더 → solvePnP → `AX=ZB` → GT 대조 | `1_calibration.md` |
| `MMS_ext_calibration2.py` | **턴테이블 축** — rim 자동추출 → 원 피팅 → GT 대조 | `1_calibration.md` |
| `MMS_ext_phase1.py` | Phase 1 — GT 누적으로 view-planning 검증 (SLAM 없음) | `2_phase1.md` |
| `MMS_ext_phase1_recovery1.py` | recovery — **빗나감**(대상물을 측면으로 조준) | `2_phase1.md` §6 |
| `MMS_ext_phase1_recovery2.py` | recovery — **윗면 미포착**(너무 낮은 el) | `2_phase1.md` §6 |
| `MMS_ext_phase2_nbv.py` | Phase 2 NBV 루프 | `3_phase2.md` §6.4 |

## 터미널에서 돌리려면 → `scripts/sim/calib_handeye_sim.py`

hand-eye 검증은 **standalone 러너**가 따로 있다. 하니스 로직을 그대로 재사용하면서
SimulationApp 을 직접 띄운다.

```bash
env -u PYTHONPATH ~/miniconda3/envs/env_isaacsim/bin/python -u scripts/sim/calib_handeye_sim.py
```
> 2026-09-09 실측: t_err 1.10mm / r_err 0.04° (PASS)

아래는 **GUI 안에서 직접 돌릴 때**의 이야기다.

## ⚠ 하니스 자체는 standalone 이 아니다 — Isaac GUI 안에서 실행한다

`SimulationApp` 을 스스로 만들지 않는다. **이미 떠 있는 Isaac Sim** 의 app 에 얹히는
구조라, 터미널에서 바로 돌리면 `omni.*` 를 못 찾는다.

```
ModuleNotFoundError: No module named 'omni.usd'   ← standalone 으로 돌렸을 때
```

**실행 방법**

1. Isaac Sim GUI 를 띄운다 (`~/isaacsim/isaac-sim.sh` 등)
2. GUI 를 띄우기 **전** 셸에서 `export MMS_ROOT=/경로/MMS`
3. `Window > Script Editor` 에서:
   ```python
   exec(open("/경로/MMS/sim_harness/MMS_ext_calibration.py").read())
   ```
   VSCode Isaac 확장(코드러너)으로 파일을 열어 실행해도 된다.

> Isaac 트리에 링크해 두면 GUI 파일 브라우저에서 찾기 편하다(필수는 아니다):
> ```bash
> DST=~/isaacsim/standalone_examples/play/MMS && mkdir -p "$DST"
> ln -sf "$MMS_ROOT"/sim_harness/MMS_ext*.py "$DST"/
> ```

> **파이프라인 본체(`main_artec.py`)는 반대로 standalone** 이다:
> `env -u PYTHONPATH ~/miniconda3/envs/env_isaacsim/bin/python main_artec.py`

### 경로는 자동으로 찾는다

절대경로를 소스에 박지 않는다. 필요하면 환경변수로 지정한다.

| 변수 | 무엇 | 기본 |
|---|---|---|
| `MMS_ROOT` | MMS 리포 위치 | 스크립트 상위 → 알려진 경로 순으로 탐색 |
| `MMS_ASSET_ROOT` | 자산(`2_3Dassets`) 루트 | `mms_paths.py` 가 해석 |
| `MMS_HARNESS_OUT` | 캡처·로그 출력 위치 | `~/isaacsim/standalone_examples/play/MMS` |
| `MMS_XARM_SDK` | xArm SDK **소스** 경로 | 미설정(=`mms-env` 의 pip 설치본 사용) |

Isaac 트리로 복사해서 돌리면 `__file__` 이 리포 밖이라 상위 탐색이 실패한다.
그때는 `MMS_ROOT` 를 준다.

```bash
export MMS_ROOT=/경로/MMS
env -u PYTHONPATH ~/isaacsim/python.sh "$DST/MMS_ext_calibration.py"
```

---

## 씬 버전 — v3 기준으로 갱신 완료 (2026-09)

작성 시점은 2026-06(v2 씬)이었으나, 08 월 v3 전환(`docs/v3_sim_migration.md`)에 맞춰
상수를 갱신했다.

| | 기존(v2) | 현재(v3) |
|---|---|---|
| 씬 | `frame_xarm7_spider_turntable/v2.usd` | `frame_xarm7_spider_turntable_v2/v3_scene.usd` |
| 카메라 prim | `.../link7/Artec_Space_Spider_mm/Camera` | `.../link7/tool/spider/Camera` |
| 턴테이블 prim | `/World/ScanTarget/turntable_demo/...` | `/World/frame/turntable_disc` |
| 스캔 대상 | `/World/ScanTarget/Solid_Marble` | `MMS_SIM_OBJECT_PRIM` (기본 `/World/ScanTarget/TestObject`) |
| 보드 낙하 위치 | `(0.330, −0.020)` | `(0.365, 0.0)` — v3 턴테이블 축 |

> prim 경로는 `v3_scene.usd` 를 `pxr` 로 조회해 확인했다. 정본은
> `mms_artec/backends/isaac/isaac_world.py`.
>
> ⚠ **실행 검증은 아직 못 했다** — 상수만 맞춘 상태다. 첫 실행 시 prim 오류가 나면
> `isaac_world.py` 와 대조할 것.

## ★ 설계 원칙 — sim 에 중복 구현하지 않는다

이 하니스는 **USD/Isaac 환경 코드만** 갖는다. 검출·솔버·IK·기하 연산은 MMS 본체
모듈을 **파일경로로 로드해 그대로 쓴다.** sim 과 real 이 다른 코드를 돌면 sim 검증이
의미를 잃기 때문이다.

주의점(자세히는 `docs/1_calibration.md` §6):
- Isaac 런타임에 동명 `utils` 패키지가 있어 `from utils...` 가 깨진다 → **파일경로 로드**
- `@dataclass` 때문에 `sys.modules` 등록이 필수
- 옮길 모듈은 레포 내부 import 가 없는 **자기완결** 형태여야 한다
  (`handeye_geometry` · `ik_provider` · `xarm7_kinematics` 패턴)

## 산출물

실행하면 Isaac 트리 아래 `captures*/` 에 캡처·로그가 쌓인다(리포에는 넣지 않는다).
