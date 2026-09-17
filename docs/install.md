# 설치 — Windows

> `setup/setup_envs.sh` 는 Linux 전용이다(`~/miniconda3/envs/<env>/bin/python` 경로 가정).
> Windows 에서는 이 문서를 따른다. 아래 순서 그대로 검증된 절차다.

실물 스캔까지 가려면 **4단계**다. 1·2 는 필수, 3 은 Artec 스캐너를 쓸 때만,
4 는 실물 장비 앞에 앉았을 때 한 번씩. 5 는 sim 이 필요할 때만.

| 단계 | 내용 | 소요 |
|---|---|---|
| 1 | Miniforge 설치 | ~3 분 |
| 2 | `mms-env` 생성 + 패키지 | ~5 분 |
| 3 | Artec SDK python 바인딩 빌드 | ~5 분 |
| 4 | 장비 3종 동작 점검 (robot·turntable·scanner) | ~5 분 |
| 5 | (선택) `env_isaacsim` — sim 씬을 눈으로 보려면 | ~1 시간 |

---

## 1. Miniforge 설치

```powershell
winget install --id CondaForge.Miniforge3 --source winget --scope user `
  --accept-package-agreements --accept-source-agreements
```

> **`--source winget` 을 빼지 말 것.** 빼면 msstore 약관 동의 프롬프트에서 멈춘다.
>
> **왜 Anaconda/Miniconda 가 아닌가** — Anaconda 의 기본 채널(`defaults`)은 2024 ToS 개정으로
> 일정 규모 이상 조직의 상업적 사용 시 유료다. Miniforge 는 `conda-forge` 만 쓰는 BSD 배포판이라
> 이 문제가 없다. conda CLI 는 동일하다.

설치되면 `C:\Users\<사용자>\miniforge3` 에 들어가고 PowerShell 프로필에 `conda init` 이 자동으로 박힌다.

### ⚠ 터미널을 새로 열어야 한다

프로필은 셸 시작 시에만 읽힌다. 설치 전에 열어둔 터미널에서는 `conda` 를 못 찾고,
`python` 이 시스템 파이썬을 가리켜 `ModuleNotFoundError: No module named 'numpy'` 가 난다.

VS Code 라면 터미널 휴지통 아이콘 → `Ctrl+Shift+\`` 로 새로 연다. 확인:

```powershell
conda --version
```

새 터미널을 열기 곤란하면 현재 창에서 훅만 수동 로드해도 된다:

```powershell
(& "$env:USERPROFILE\miniforge3\Scripts\conda.exe" shell.powershell hook) | Out-String | Invoke-Expression
```

---

## 2. mms-env (real 백엔드)

```powershell
conda create -y -n mms-env python=3.11
conda activate mms-env
pip install -r requirements.txt
```

검증:

```powershell
python -c "import numpy,cv2,open3d; print(numpy.__version__, cv2.__version__, open3d.__version__)"
```

`2.x  4.x  0.19+` 이면 정상.
**cv2 가 5.x 면 안 된다** — `cv2.calibrateHandEye` 가 없어서 hand-eye 캘리브가 깨진다.
상수는 남아 있어 import 는 통과하므로 발견이 늦다. `requirements.txt` 가 `<5` 로 막고 있다.

여기까지면 캘리브레이션·분석 등 **오프라인 스크립트는 전부 돌아간다.**

---

## 3. Artec SDK python 바인딩 빌드

Artec SDK 는 C++ 라이브러리다. pybind11 로 `.pyd` 6 개를 만들어야 실물 스캔이 된다.
없으면 import 는 조용히 통과하고(래퍼가 lazy-load) **스캐너를 잡는 순간** 터진다.

### 사전 요구사항

| 항목 | 확인 방법 |
|---|---|
| Artec 3D Scanning SDK | `C:\Program Files\Artec\Artec 3D Scanning SDK\include\...\IScanner.h` 존재 |
| VS Build Tools + **C++ 워크로드** | `...\VC\Tools\MSVC\<버전>` 폴더 존재 |
| CMake | VS 번들 사용 (PATH 에 없어도 됨) |
| pybind11 | 아래에서 설치 |

Build Tools 가 없으면: `winget install --id Microsoft.VisualStudio.2022.BuildTools --source winget`
→ 설치 관리자에서 **"C++를 사용한 데스크톱 개발"** 워크로드를 반드시 체크.

### 빌드

```powershell
conda activate mms-env
pip install pybind11

$cmake = "C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe"
$py    = "$env:USERPROFILE\miniforge3\envs\mms-env\python.exe"

& $cmake -S mms_artec\sensor -B mms_artec\sensor\build -G "Visual Studio 17 2022" -A x64 `
         -DPython_EXECUTABLE="$py" -DPython_ROOT_DIR="$env:USERPROFILE\miniforge3\envs\mms-env"
& $cmake --build mms_artec\sensor\build --config Release
```

> **Generator 는 설치된 VS 에 맞춘다.** Build Tools 2022 면 `"Visual Studio 17 2022"`.
> (`docs/artec_SDK/artec0_build_guide.md` 는 VS 18 2026 기준이라 그대로 쓰면 구성 오류가 난다.)
> 확인: `& "${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer\vswhere.exe" -property displayName`
>
> **`-DPython_EXECUTABLE` 를 꼭 준다.** 안 주면 CMake 가 시스템 파이썬을 잡아
> `mms-env` 에서 import 안 되는 `.pyd` 가 나온다.
>
> **재구성 시 `build\` 를 지운다.** `CMakeCache.txt` 에 옛 Generator 가 남아 구성이 실패한다.

성공하면 `mms_artec\sensor\` 에 `.pyd` 6 개가 생긴다:

```powershell
Get-ChildItem mms_artec\sensor\*.pyd | Measure-Object   # Count = 6
```

### 검증 — SDK 가 실제로 호출되는가

```powershell
python -c "from mms_artec.sensor import artec_base; artec_base._load(); import artec_sdk_py; print(artec_sdk_py.enumerate_scanners())"
```

스캐너 목록(미연결이면 `[]`)이 나오면 성공.

> `.pyd` 를 직접 `import artec_sdk_py` 하면 **DLL load failed** 가 난다. 정상이다 —
> 래퍼(`artec_base._load()`)가 `os.add_dll_directory()` 로 SDK `bin-x64` 를 붙여준다.
> 항상 래퍼를 거쳐 쓴다.

---

## 4. 장비 동작 점검 — robot · turntable · scanner

빌드까지 끝났으면, 본 파이프라인(`main_artec.py`)을 돌리기 **전에** 장비 3종을
하나씩 따로 깨워본다. 한 번에 다 켜고 시작하면 어느 장비가 문제인지 구분이 안 된다.

전부 `mms-env` 에서, 리포 루트에서 실행한다.

```powershell
conda activate mms-env
cd \MMS
$env:PYTHONIOENCODING="utf-8"
```

### 요약 — 순서대로 한 줄씩

| # | 대상 | 커맨드 | 움직임 |
|---|---|---|---|
| 0 | 3종 전체 | `python scripts/check_devices.py` | ❌ 읽기 전용 |
| 1 | robot | `python scripts/robot/status.py` | ❌ 읽기 전용 |
| 2 | robot | `python scripts/robot/home.py` | ⚠ **움직인다** |
| 3 | turntable | `python scripts/turntable/lookaround_speed_rotation.py` | ⚠ **회전한다** |
| 4 | scanner | `python mms_artec/sensor/binding_test.py` | ❌ 캡처만 |

**0 → 1 → 2 → 3 → 4 순서를 지킨다.** 0·1 이 통과하지 못하면 2 이후는 의미가 없다.

> ⚠ 모션 스크립트(2·3)를 돌리기 전 — 작업 반경에 사람·케이블·공구가 없는지 **눈으로**
> 확인하고, **비상정지 버튼이 손에 닿는 곳에** 있어야 한다.
> 로봇 웹 UI(`http://192.168.1.210:18333`) 탭은 **닫는다** — 열려 있으면 컨트롤러의
> mode/state 를 웹 쪽이 바꿔서 SDK 명령이 무시된다 (`docs/robot_control.md` §1).

---

### 0) 3종 연결 점검 — 읽기 전용

```powershell
python scripts/check_devices.py
```

스캐너 · xArm7 · 턴테이블의 연결만 확인한다. **로봇을 움직이지 않고, 턴테이블 servo 도 켜지 않는다.**
세 줄 모두 `[OK]` 면 통과.

| 실패 | 원인 |
|---|---|
| `Artec Spider ... 바인딩 로드 실패` | §3 빌드 미실행 |
| `Artec Spider ... 스캐너를 찾지 못했다` | USB 케이블 / 스캐너 전원 |
| `xArm7 ... 연결 안 됨` | IP·전원·랜선 (`ping 192.168.1.210`) |
| `Turntable ...` 실패 | Ezi-SERVO 전원, `ping 192.168.0.10`, 벤더 DLL 미설치 |

IP 는 환경변수로 덮어쓸 수 있다 — `MMS_ROBOT_IP`, `MMS_TURNTABLE_IP`, `MMS_TURNTABLE_BD`.

### 1) robot — 상태 읽기

```powershell
python scripts/robot/status.py
```

관절각·TCP·에러·관절 여유·특이점 지표를 한 번에 본다. **절대 움직이지 않는다.**

봐야 할 건 `error=0` 하나다. `state=4` 는 정상 — "motion_enable 전 대기" 이지 고장이 아니다.
에러가 있으면 `python scripts/robot/recover.py` → 그래도 남으면 물리적 원인(충돌·한계 초과)이므로
웹 UI 에서 확인한다.

### 2) robot — home 자세로 이동 ⚠ 실제로 움직인다

```powershell
python scripts/robot/home.py
```

wrist singularity 를 피한 중립 자세로 간다. **관절 목표를 직접 주므로 IK 를 타지 않는다** —
로봇 모션 경로 중 가장 단순해서, 첫 모션 테스트로 적합하다.
실행하면 확인을 묻는다 — Enter 가 아니라 **`y` 를 입력**해야 움직인다.

기본값은 Artec 마운트(`--sensor artec`, J7 −45°). 센서를 바꿔 달았으면 `--sensor phoxi`.
느리게 가려면 `--speed 10` (deg/s, 기본 20).

여기까지 됐으면 조그·절대이동도 된다 — `docs/robot_control.md` §2 (`jog.py`, `move_pose.py`).
처음 쓰는 좌표는 반드시 `--dry-run` 먼저.

### 3) turntable — lookaround 과 동일 조건 360° 회전 ⚠ 실제로 돈다

```powershell
python scripts/turntable/lookaround_speed_rotation.py
```

connect → servo ON → 30초에 한 바퀴(+5° overshoot) → stop → 0° 복귀 → servo OFF → disconnect
까지 혼자 다 한다. 인자는 없다. 회전 속도·가속도가 lookaround 실제 스캔과 **같은 값**이라,
이게 매끄럽게 돌면 lookaround 의 회전 조건은 검증된 것이다.

같이 해볼 것 — Artec Studio 의 Recording 을 띄운 채로 실행하고, 회전 중 손으로 대상물을
가리거나 빼서 Studio 가 `tracking lost` 를 잡는지 본다. lookaround watchdog 이 보는 것과 같은 상황이다.

**멈추는 법** — `Ctrl+C` 를 누르면 즉시 `stop()` + servo OFF + disconnect 한다.
스크립트가 죽어서 테이블이 계속 돈다면:

```powershell
python scripts/turntable/turntable_stop.py
```

### 4) scanner — SDK 바인딩 + 실제 캡처 전체 검증

```powershell
python mms_artec/sensor/binding_test.py
```

**§3 의 `enumerate_scanners()` 한 줄보다 한참 깊게 본다.** `.pyd` 6 개를 전부 로드하고,
스캐너를 잡아 실제로 프레임을 캡처한 뒤 알고리즘·프로젝트 API 까지 차례로 호출한다.
Open3D 창이 몇 번 뜬다 — 창을 닫으면 다음 단계로 넘어간다.

마지막에 `결과: 전체 통과` 가 나오면 성공 (실패가 있으면 exit code 1 + 실패 목록).

> **스캐너 앞에 물체를 두고 실행한다.** 빈 공간을 보고 있으면 `capture() → None`
> (스캔 데이터 없음) 이 나온다 — 바인딩 문제가 아니다. 작업거리 **225mm 근처**가 좋다.

> ⚠ **Artec Studio 를 켠 채로 돌리면 실패한다.** 스캐너는 SDK 와 Studio 중 하나만
> 잡을 수 있다. 목록 조회는 되는데 열기만
> `createScanner failed (ErrorCode=0xC0050000)` 로 죽으면 이것이다.
> 창을 닫아도 프로세스가 남을 수 있다:
> `Get-Process astudio_pro, artec-ray-server -ErrorAction SilentlyContinue`

전체가 무겁다면 연결 확인만 하는 §3 의 한 줄로 대체할 수 있다:

```powershell
python -c "from mms_artec.sensor import artec_base; artec_base._load(); import artec_sdk_py; print(artec_sdk_py.enumerate_scanners())"
```

라이브 점군을 눈으로 보고 싶으면 `main_artec.py` 를 띄운 뒤 다른 터미널에서
`python scripts/artec/live_scan_view.py` (`docs/3_lookaround.md`).

---

### 3종이 다 통과했다면

다음은 **캘리브레이션**이다. 현재 `sensor_frames.yaml::T_EC_artec`·`turntable_frame.yaml`(`T_B_F0`)은
둘 다 **구 스캐너(`SP.10.36181288`) 기준이라 재캘리브가 필요하다** — README §알려진 한계 0·1.
절차는 `docs/1_calibration.md`, 스크립트는 `scripts/artec/calibrate.py` · `scripts/artec/turntable_calib.py`.

---

## 실행

```powershell
conda activate mms-env
$env:PYTHONIOENCODING="utf-8"      # 콘솔 cp949 이모지 깨짐 방지
python main_artec.py
```

**`main_artec.py` 의 `BACKEND` 를 확인한다.** 현재 기본값은 `"real"` 이다.
`"isaac"` 으로 바꿔 놓으면 `mms-env` 에는 Isaac Sim 이 없어서 import 부터 실패한다.

> Linux 문서의 `env -u PYTHONPATH` 는 ROS python3.10 경로 오염을 막는 장치다. Windows 에선 불필요.

---

## 5. env_isaacsim — Windows 에서 sim 씬 보기

> **필요할 때만.** 실물 스캔(`BACKEND="real"`)에는 Isaac 이 전혀 필요 없다.
> CAD 기반 씬(v2/v3/v4)이 실물 셀과 얼마나 다른지 **눈으로 대조**하거나,
> 충돌 캐시(`cell_env.npz`)를 다시 구울 때 필요하다 (`docs/collision.md` §6).

2026-09-15 Windows 11 / RTX 3080 Laptop 에서 **전 과정 검증**한 절차다.

### 5.1 사양 확인 — 먼저 본다

```powershell
nvidia-smi
```

| 항목 | 최소 | 검증한 머신 |
|---|---|---|
| GPU | RTX (VRAM 8GB) | RTX 3080 Laptop **8GB** ✅ |
| RAM | 16GB+ | 31.8GB |
| 디스크 | **~30GB** 여유 | — |
| 드라이버 | — | **551.95 / CUDA 12.4 에서 정상 동작 확인** |

> 드라이버 551.95(2024-03)는 Isaac Sim 5.1 기준 오래됐지만 **실제로 문제없이 떴다.**
> `NGX DLSS Frame Generation AdapterUnsupported` 경고가 뜨는데 Ada 세대 전용 기능이라
> 3080 에서 안 뜨는 게 정상이고 렌더링과 무관하다. 드라이버를 미리 올릴 필요 없다.

### 5.2 설치

```powershell
conda create -y -n env_isaacsim python=3.11
& "$env:USERPROFILE\miniforge3\envs\env_isaacsim\python.exe" -m pip install `
    -r setup\requirements-isaac.txt --extra-index-url https://pypi.nvidia.com
```

다운로드 ~30GB. `mms-env` 를 **activate 하지 않고** env 의 python.exe 를 직접 부르면
env 오염 걱정이 없다.

> ⚠ **`torch` 가 CPU 빌드로 깔린다.** `requirements-isaac.txt` 주석은 `torch==2.7.0+cu128`
> 이 isaacsim 의존성으로 딸려온다고 적고 있지만, 실제로는 PyPI 의 일반
> `torch==2.7.0`(Windows=CPU 전용)이 잡힌다 — `torch.cuda.is_available() == False`.
> **씬을 보는 데는 무관하다**(RTX 렌더러는 Vulkan 이고 PhysX GPU 도 torch 와 별개).
> torch CUDA 가 필요한 작업을 하려면 따로 깐다.

### 5.3 자산(USD) 내려받기 — **필수**

씬 USD 는 git 에 없다. 이게 없으면 Isaac 을 깔아도 **열 것이 없다.**
(`mms_paths.py` 가 리포 밖을 탐색한다 — README §환경)

```powershell
winget install --id GitHub.cli --source winget --scope user `
  --accept-package-agreements --accept-source-agreements
# ↑ PATH 가 바뀌므로 여기서 터미널을 새로 연다
gh auth login                                    # private 저장소라 인증 필요
gh release download assets-v1 -R JinkyoJB/MMS -p 'mms-assets-v1.tar.zst' -D $env:TEMP
```

검증 — 릴리스에 `.sha256` 이 같이 올라와 있다:

```powershell
(Get-FileHash "$env:TEMP\mms-assets-v1.tar.zst" -Algorithm SHA256).Hash.ToLower()
# 04c84f45590d91871d9f737d5cd6ba85701b8a546e6ade3228246c7b18d84a1c 와 같아야 한다
```

압축 해제 — **`C:\dev` 에 풀면 환경변수가 필요 없다:**

```powershell
tar -xf "$env:TEMP\mms-assets-v1.tar.zst" -C C:\dev --strip-components=1
```

| | |
|---|---|
| Windows `tar.exe` | zstd 를 자동 인식한다 (`-I zstd` 불필요) |
| 결과 | `C:\dev\2_3Dassets` (99MB) + `C:\dev\testset` (638MB) |
| 왜 `C:\dev` 인가 | 리포가 `C:\dev\MMS` 라 `mms_paths.py` 탐색 후보 **#3**(`<repo>/../2_3Dassets`)에 그대로 걸린다. `MMS_ASSET_ROOT` 를 안 잡아도 된다 |

⚠ `2_3Dassets` 와 `testset` 은 **형제 디렉터리**여야 한다 — v3 씬이 `../../testset/...` 로
대상물을 참조한다. 위 명령은 그 배치가 되도록 푼다.

확인:

```powershell
& "$env:USERPROFILE\miniforge3\envs\mms-env\python.exe" -c "import mms_paths,os; p=mms_paths.asset('frame_xarm7_spider_turntable_v2/v3_scene.usd'); print(os.path.exists(p), p)"
```

### 5.4 씬 보기

```powershell
$ISAAC = "$env:USERPROFILE\miniforge3\envs\env_isaacsim\python.exe"
& $ISAAC scripts\sim\view_scene.py            # v3 (기본)
& $ISAAC scripts\sim\view_scene.py v2         # 구 씬
& $ISAAC scripts\sim\view_scene.py v4         # 턴테이블 이설 검토안 (hw_layout §3)
& $ISAAC scripts\sim\view_scene.py v3 --headless   # 창 없이 로드만 확인
```

로봇을 구동하지도 물리를 돌리지도 않는다 — stage 를 열고 대기만 한다.
마우스 우클릭 드래그=회전, 휠=줌. 창을 닫으면 끝난다.

> **첫 실행은 8분쯤 걸린다.** 익스텐션과 셰이더 캐시를 받는다
> (`isaacsim[all]` 전체 세트 — `omni.replicator`, `omni.sensors` …).
> 진행 중 로그가 `Pulling extension:` 만 찍혀도 정상이다. 이후 실행은 캐시를 쓴다.

### 5.5 문제 해결

| 증상 | 원인 / 해결 |
|---|---|
| `씬이 없다: ...v3_scene.usd` | 자산 미설치 → §5.3 |
| 첫 실행이 안 끝나는 것 같다 | 정상. `Simulation App Startup Complete` 까지 ~8분 |
| `NGX DLSS ... AdapterUnsupported` | 무시. Ada 전용 기능이라 3080 에 없는 게 정상 |
| `torch.cuda.is_available() False` | 알려진 사항 — §5.2 주석. 씬 보기에는 무관 |
| `isaacsim` ModuleNotFoundError | `mms-env` 로 실행했다. `env_isaacsim` 의 python.exe 를 직접 부를 것 |
| 씬이 비어 있다 / 텍스처가 없다 | `2_3Dassets` 와 `testset` 이 형제가 아니다 (§5.3) |
| EULA 프롬프트에서 멈춤 | `$env:OMNI_KIT_ACCEPT_EULA="YES"` (`view_scene.py` 는 자동으로 세운다) |

---

## 나머지 env (필요할 때)

| env | 용도 | 명령 |
|---|---|---|
| `env_isaacsim` | Isaac Sim 5.1 시뮬레이션 | **→ §5** (Windows 검증 절차 + 자산 내려받기) |
| `step2usd` | STEP(CAD)→USD 변환 | `conda create -y -n step2usd python=3.11` → `conda install -y -c conda-forge pythonocc-core=7.9` → `pip install -r setup/requirements-step2usd.txt` |

⚠ **env 를 섞지 말 것.** `env_isaacsim` 은 numpy **1.26 고정**(isaacsim wheel 의 ABI), 나머지는 numpy 2.x.
`env_isaacsim` 은 RTX GPU 가 필요하고 다운로드가 수십 GB다.
`step2usd` 는 새 sim 씬을 CAD 에서 만들 때만 필요 — 기존 `v3_scene.usd` 로 돌릴 때는 없어도 된다.

### conda 를 못 쓰는 환경이라면

| env | venv 가능? |
|---|---|
| `mms-env` | ✅ 전부 순수 pip |
| `env_isaacsim` | ✅ `isaacsim` 은 pip wheel |
| `step2usd` | ❌ `pythonocc-core` 가 PyPI 에 없다 — conda-forge 전용 |

venv 는 인터프리터를 복제할 뿐이라 **Python 3.11 을 먼저 설치**해야 한다
(`winget install Python.Python.3.11` → `py -3.11 -m venv .venv-mms`).

---

## 문제 해결

| 증상 | 원인 / 해결 |
|---|---|
| `ModuleNotFoundError: No module named 'numpy'` | 터미널이 Miniforge 설치 전에 열렸다 → 새 터미널 + `conda activate mms-env` |
| `conda` 를 찾을 수 없음 | 위와 동일. 또는 시작 메뉴의 **Miniforge Prompt** 사용 |
| winget 이 약관 프롬프트에서 멈춤 | `--source winget` 추가 |
| CMake `Generator ... does not match` | `build\` 삭제 후 설치된 VS 버전에 맞는 Generator 로 재구성 |
| `.pyd` 는 있는데 `DLL load failed` | 래퍼를 거치지 않고 직접 import 했다 → `artec_base._load()` 먼저 |
| `createScanner failed (ErrorCode=0xC0050000)` | Artec Studio 가 스캐너를 점유 중 → 완전히 종료 (§4-4). `enumerate` 는 되는데 `open` 만 실패하면 거의 항상 이것 |
| `import artec_base_py` 실패 (`.pyd` 없음) | 3단계 빌드 미실행, 또는 `-DPython_EXECUTABLE` 누락으로 다른 파이썬용 `.pyd` 생성 |
| `isaacsim` ModuleNotFoundError | `main_artec.py` 의 `BACKEND` 가 `"isaac"` → 실물이면 `"real"` 로 |
| 빌드 시 `error C2440: 'initializer list'에서 'pybind11::array_t<...>'` | MSVC 가 2-요소 brace shape `{n, 3}` 을 모호하게 본다. shape 을 `std::vector<py::ssize_t>{...}` 로 감싼다 (현재 소스에 반영됨). pybind11 버전과 무관 |

---

## 남은 수동 설치

- **턴테이블 Ezi-SERVO** 벤더 DLL (`FAS_EziMOTIONPlusE*`, Windows 전용)
- 정합 벤치마크(`scripts/artec/reg_benchmark/`)는 torch/CUDA 확장이 필요해 **별도 env 권장**.
  `mms-env` 에 설치하지 않는다.
