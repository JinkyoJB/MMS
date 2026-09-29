# 설치 보충 — Windows 상세

> **기본 절차는 `../README.md` §환경에 있다.** Miniforge 설치, `mms-env` 생성, Artec SDK
> 바인딩 빌드, `env_isaacsim` 구성, 자산 내려받기는 그쪽을 순서대로 따른다.
>
> 본 문서는 README 가 한 줄로 요약한 **사전 요구사항 · 장비 점검 절차 · 문제 해결**을
> 상세히 다룬다.

---

## 1. Artec SDK 바인딩 — 사전 요구사항

Artec SDK 는 C++ 라이브러리이며, pybind11 로 `.pyd` 6개를 생성해야 실물 스캔이 가능하다.
없으면 import 는 조용히 통과하고(래퍼가 lazy-load) **스캐너를 잡는 순간** 실패한다.

| 항목 | 확인 방법 |
|---|---|
| Artec 3D Scanning SDK | `C:\Program Files\Artec\Artec 3D Scanning SDK\include\...\IScanner.h` 존재 |
| VS Build Tools + **C++ 워크로드** | `...\VC\Tools\MSVC\<버전>` 폴더 존재 |
| CMake | VS 번들 사용 (PATH 에 없어도 무방) |
| pybind11 | `pip install pybind11` |

Build Tools 미설치 시
`winget install --id Microsoft.VisualStudio.2022.BuildTools --source winget` 후 설치
관리자에서 **"C++를 사용한 데스크톱 개발"** 워크로드를 반드시 선택한다.

Generator 는 설치된 VS 버전에 맞춘다. 확인:
`& "${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer\vswhere.exe" -property displayName`.
(`docs/artec_SDK/artec0_build_guide.md` 는 VS 18 2026 기준이므로 그대로 사용하면 구성
오류가 발생한다.)

`.pyd` 를 직접 `import artec_sdk_py` 하면 **DLL load failed** 가 발생하는데 이는 정상이다 —
래퍼(`artec_base._load()`)가 `os.add_dll_directory()` 로 SDK `bin-x64` 를 등록한다. 항상
래퍼를 거쳐 사용한다.

---

## 2. 장비 동작 점검 — robot · turntable · scanner

빌드 완료 후 `main_artec.py` 를 실행하기 **전에** 장비 3종을 하나씩 개별 확인한다. 한
번에 모두 켜고 시작하면 어느 장비가 문제인지 구분할 수 없다.

전부 `mms-env` 에서 리포 루트를 기준으로 실행한다.

```powershell
conda activate mms-env
cd C:\dev\MMS
$env:PYTHONIOENCODING="utf-8"
```

| # | 대상 | 커맨드 | 움직임 |
|---|---|---|---|
| 0 | 3종 전체 | `python scripts/check_devices.py` | 읽기 전용 |
| 1 | robot | `python scripts/robot/status.py` | 읽기 전용 |
| 2 | robot | `python scripts/robot/home.py` | ⚠ **움직인다** |
| 3 | turntable | `python scripts/turntable/lookaround_speed_rotation.py` | ⚠ **회전한다** |
| 4 | scanner | `python mms_artec/sensor/binding_test.py` | 캡처만 |

**0 → 1 → 2 → 3 → 4 순서를 지킨다.** 0·1 이 통과하지 못하면 이후는 의미가 없다.

> ⚠ 모션 스크립트(2·3) 실행 전 — 작업 반경에 사람·케이블·공구가 없는지 **육안으로**
> 확인하고 **비상정지 버튼이 손이 닿는 위치**에 있어야 한다. 로봇 웹 UI
> (`http://192.168.1.210:18333`) 탭은 **닫는다** — 열려 있으면 컨트롤러의 mode/state 를
> 웹 쪽이 변경하여 SDK 명령이 무시된다(`robot_control.md` §1).

### 0) 3종 연결 점검 — 읽기 전용

스캐너·xArm7·턴테이블의 연결만 확인한다. 로봇을 움직이지 않고 턴테이블 servo 도 켜지
않는다. 세 줄 모두 `[OK]` 면 통과다.

| 실패 | 원인 |
|---|---|
| `Artec Spider … 바인딩 로드 실패` | SDK 바인딩 빌드 미실행 |
| `Artec Spider … 스캐너를 찾지 못했다` | USB 케이블 / 스캐너 전원 |
| `xArm7 … 연결 안 됨` | IP·전원·랜선 (`ping 192.168.1.210`) |
| `Turntable …` 실패 | Ezi-SERVO 전원, `ping 192.168.0.10`, 벤더 DLL 미설치 |

IP 는 환경변수로 덮어쓸 수 있다 — `MMS_ROBOT_IP`, `MMS_TURNTABLE_IP`, `MMS_TURNTABLE_BD`.

### 1) robot — 상태 읽기

관절각·TCP·에러·관절 여유·특이점 지표를 일괄 확인한다. 절대 움직이지 않는다.
확인할 것은 `error=0` 하나다. `state=4` 는 "motion_enable 전 대기" 로 정상이다. 에러가
있으면 `scripts/robot/recover.py` 를 실행하고, 그래도 남으면 물리적 원인(충돌·한계 초과)
이므로 웹 UI 에서 확인한다.

### 2) robot — home 자세 이동 ⚠ 실제로 움직인다

wrist singularity 를 회피한 중립 자세로 이동한다. **관절 목표를 직접 지정하므로 IK 를
타지 않아** 로봇 모션 중 가장 단순하며 첫 모션 테스트로 적합하다. 실행 시 확인을 묻는데
Enter 가 아니라 **`y` 를 입력**해야 움직인다.

기본값은 Artec 마운트(`--sensor artec`, J7 −45°)다. 센서 교체 시 `--sensor phoxi`,
저속 이동은 `--speed 10`(deg/s, 기본 20). 이후 조그·절대이동은 `robot_control.md` §2
(`jog.py`, `move_pose.py`)를 참조하며, 처음 사용하는 좌표는 반드시 `--dry-run` 을 먼저
수행한다.

### 3) turntable — lookaround 과 동일 조건 360° 회전 ⚠ 실제로 회전한다

connect → servo ON → 30초에 1회전(+5° overshoot) → stop → 0° 복귀 → servo OFF →
disconnect 까지 자동 수행하며 인자는 없다. 회전 속도·가속도가 lookaround 실제 스캔과
동일하므로, 매끄럽게 회전하면 lookaround 의 회전 조건이 검증된 것이다.

함께 확인할 사항 — Artec Studio 의 Recording 을 띄운 채 실행하고, 회전 중 손으로 대상물을
가리거나 제거하여 Studio 가 `tracking lost` 를 검출하는지 확인한다. lookaround watchdog 이
감시하는 것과 동일한 상황이다.

**정지 방법** — `Ctrl+C` 를 누르면 즉시 `stop()` + servo OFF + disconnect 한다. 스크립트가
비정상 종료되어 테이블이 계속 회전하면 `python scripts/turntable/turntable_stop.py`.

### 4) scanner — SDK 바인딩 + 실제 캡처 전체 검증

`enumerate_scanners()` 한 줄보다 훨씬 깊게 확인한다. `.pyd` 6개를 전부 로드하고 스캐너를
잡아 실제로 프레임을 캡처한 뒤 알고리즘·프로젝트 API 까지 차례로 호출한다. Open3D 창이
몇 차례 표시되며, 창을 닫으면 다음 단계로 넘어간다. 마지막에 `결과: 전체 통과` 가
출력되면 성공이다(실패 시 exit code 1 + 실패 목록).

> **스캐너 앞에 물체를 두고 실행한다.** 빈 공간을 보고 있으면 `capture() → None`(스캔
> 데이터 없음)이 반환되며 이는 바인딩 문제가 아니다. 작업거리 **225mm 근처**가 적절하다.
>
> ⚠ **Artec Studio 를 켠 상태에서는 실패한다.** 스캐너는 SDK 와 Studio 중 하나만 점유할
> 수 있다. 목록 조회는 되는데 열기만 `createScanner failed (ErrorCode=0xC0050000)` 로
> 실패하면 이 경우다. 창을 닫아도 프로세스가 남을 수 있다 —
> `Get-Process astudio_pro, artec-ray-server -ErrorAction SilentlyContinue`.

전체 검증이 부담스러우면 연결 확인 한 줄로 대체할 수 있다.

```powershell
python -c "from mms_artec.sensor import artec_base; artec_base._load(); import artec_sdk_py; print(artec_sdk_py.enumerate_scanners())"
```

라이브 점군을 확인하려면 `main_artec.py` 실행 후 별도 터미널에서
`python scripts/artec/live_scan_view.py` 를 실행한다.

### 3종 통과 후

다음은 **캘리브레이션**이다. 절차는 `calibration_runbook.md`, 원리는 `1_calibration.md`,
진입점은 `scripts/artec/calibrate.py` 다. 현재 저장값(2026-09-21)은 `T_B_F0` 가 기준을
충족하나 `T_EC_artec` 은 t_err 8.443mm 로 기준(≤4mm)을 초과하므로 재수행 대상이다.

---

## 3. 나머지 env

| env | 용도 | 명령 |
|---|---|---|
| `step2usd` | STEP(CAD)→USD 변환 | `conda create -y -n step2usd python=3.11` → `conda install -y -c conda-forge pythonocc-core=7.9` → `pip install -r setup/requirements-step2usd.txt` |

⚠ **env 를 섞지 않는다.** `env_isaacsim` 은 numpy **1.26 고정**(isaacsim wheel 의 ABI)이고
나머지는 numpy 2.x 다. `step2usd` 는 STEP 에서 **새 씬을 생성할 때만** 필요하다. 기본 씬
`v2_real_260917.usd` 는 `assets-v2` 릴리스에 포함되어 있으므로 기존 씬으로 실행만 하는
경우에는 불필요하다.

### conda 를 사용할 수 없는 환경

| env | venv 가능 여부 |
|---|---|
| `mms-env` | 가능 — 전부 순수 pip |
| `env_isaacsim` | 가능 — `isaacsim` 은 pip wheel |
| `step2usd` | **불가** — `pythonocc-core` 가 PyPI 에 없다(conda-forge 전용) |

venv 는 인터프리터를 복제할 뿐이므로 Python 3.11 을 먼저 설치해야 한다
(`winget install Python.Python.3.11` → `py -3.11 -m venv .venv-mms`).

---

## 4. 문제 해결

| 증상 | 원인 / 조치 |
|---|---|
| `ModuleNotFoundError: No module named 'numpy'` | 터미널이 Miniforge 설치 전에 열렸다 → 새 터미널 + `conda activate mms-env` |
| `conda` 를 찾을 수 없음 | 위와 동일. 또는 시작 메뉴의 **Miniforge Prompt** 사용. 현재 창에서 훅만 수동 로드하려면 `(& "$env:USERPROFILE\miniforge3\Scripts\conda.exe" shell.powershell hook) \| Out-String \| Invoke-Expression` |
| winget 이 약관 프롬프트에서 중단 | `--source winget` 추가 |
| CMake `Generator … does not match` | `build\` 삭제 후 설치된 VS 버전에 맞는 Generator 로 재구성 |
| `.pyd` 는 있는데 `DLL load failed` | 래퍼를 거치지 않고 직접 import 했다 → `artec_base._load()` 선행 |
| `createScanner failed (0xC0050000)` | Artec Studio 가 스캐너 점유 중 → 완전 종료(§2-4). `enumerate` 는 되는데 `open` 만 실패하면 거의 항상 이 경우다 |
| `import artec_base_py` 실패(`.pyd` 없음) | 바인딩 빌드 미실행, 또는 `-DPython_EXECUTABLE` 누락으로 다른 파이썬용 `.pyd` 생성 |
| `isaacsim` ModuleNotFoundError | `BACKEND` 가 `"isaac"` 인데 `mms-env` 로 실행했다 → 실물이면 `"real"` |
| 빌드 시 `error C2440: 'initializer list'에서 'pybind11::array_t<…>'` | MSVC 가 2-요소 brace shape `{n, 3}` 을 모호하게 해석한다. shape 을 `std::vector<py::ssize_t>{…}` 로 감싼다(현재 소스에 반영됨). pybind11 버전과 무관 |

### Miniforge 를 사용하는 이유

Anaconda 의 기본 채널(`defaults`)은 2024 ToS 개정으로 일정 규모 이상 조직의 상업적 사용
시 유료다. Miniforge 는 `conda-forge` 만 사용하는 BSD 배포판이므로 해당 문제가 없으며
conda CLI 는 동일하다.

---

## 5. 남은 수동 설치

- **턴테이블 Ezi-SERVO** 벤더 DLL (`FAS_EziMOTIONPlusE*`, Windows 전용)
- 정합 벤치마크(`scripts/artec/reg_benchmark/`)는 torch/CUDA 확장이 필요하므로 **별도 env**
  를 권장한다. `mms-env` 에 설치하지 않는다.
