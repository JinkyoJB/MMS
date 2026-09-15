# 설치 — Windows

> `setup/setup_envs.sh` 는 Linux 전용이다(`~/miniconda3/envs/<env>/bin/python` 경로 가정).
> Windows 에서는 이 문서를 따른다. 아래 순서 그대로 검증된 절차다.

실물 스캔까지 가려면 **3단계**다. 1·2 는 필수, 3 은 Artec 스캐너를 쓸 때만.

| 단계 | 내용 | 소요 |
|---|---|---|
| 1 | Miniforge 설치 | ~3 분 |
| 2 | `mms-env` 생성 + 패키지 | ~5 분 |
| 3 | Artec SDK python 바인딩 빌드 | ~5 분 |

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

## 실행

```powershell
conda activate mms-env
$env:PYTHONIOENCODING="utf-8"      # 콘솔 cp949 이모지 깨짐 방지
python main_artec.py
```

**`main_artec.py` 의 `BACKEND` 를 확인한다.** 기본값이 `"isaac"` 이라 실물 장비를 쓰려면
`BACKEND = "real"` 로 바꿔야 한다. `mms-env` 에는 Isaac Sim 이 없어서 `"isaac"` 이면 import 부터 실패한다.

> Linux 문서의 `env -u PYTHONPATH` 는 ROS python3.10 경로 오염을 막는 장치다. Windows 에선 불필요.

---

## 나머지 env (필요할 때)

| env | 용도 | 명령 |
|---|---|---|
| `env_isaacsim` | Isaac Sim 5.1 시뮬레이션 | `conda create -y -n env_isaacsim python=3.11` → `pip install -r setup/requirements-isaac.txt --extra-index-url https://pypi.nvidia.com` |
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
| `import artec_base_py` 실패 (`.pyd` 없음) | 3단계 빌드 미실행, 또는 `-DPython_EXECUTABLE` 누락으로 다른 파이썬용 `.pyd` 생성 |
| `isaacsim` ModuleNotFoundError | `main_artec.py` 의 `BACKEND` 가 `"isaac"` → 실물이면 `"real"` 로 |
| 빌드 시 `error C2440: 'initializer list'에서 'pybind11::array_t<...>'` | MSVC 가 2-요소 brace shape `{n, 3}` 을 모호하게 본다. shape 을 `std::vector<py::ssize_t>{...}` 로 감싼다 (현재 소스에 반영됨). pybind11 버전과 무관 |

---

## 남은 수동 설치

- **턴테이블 Ezi-SERVO** 벤더 DLL (`FAS_EziMOTIONPlusE*`, Windows 전용)
- 정합 벤치마크(`scripts/artec/reg_benchmark/`)는 torch/CUDA 확장이 필요해 **별도 env 권장**.
  `mms-env` 에 설치하지 않는다.
