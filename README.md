# MMS — Multi Modal 3D Scanning System

> Artec Spider + xArm7 + 턴테이블 자동 3D 스캐닝 시스템.
> **최종 산출물 = 대상물 전면(full-coverage)의 watertight mesh + texture.**

---

## 구조

```
mms_artec/
  system.py                      ArtecMMS (오케스트레이터 + disc_surface_frame)
  backends/                      real / isaac 백엔드 팩토리
    isaac/{isaac_world,isaac_xarm,isaac_turntable,isaac_scanner}.py
  sensor/artec_client.py         real 스캐너 (+ capture_points_base)
  nbv/artec_streaming_scan_session.py   lookaround streaming SLAM + 4 watchdog
  nbv/artec_multipass_scan_session.py   lookaround+nbv+flip 통합, view-score, recovery
utils/                           ★ sensor-agnostic 공유 코어 (real·sim 공용)
  calibration/{turntable_frame,rim_picker,hand_eye_calibrator,artec_charuco_detector}.py
  collision/{geometry,robot_collision}.py    자세별 충돌 쿼리 (real/sim 공용)
  nbv/{frontier,icp_strategy,manual_picker,nbv_core,flip_policy}.py
  robot/{xarm_interface,xarm7_kinematics}.py     ★ 해석 FK/IK (real·sim 공유)
  turntable/turntable_interface.py    transforms.py    control/theta_planner.py
main_artec.py                    진입점 (BACKEND, RUN_CALIBRATION 토글)
mms_paths.py                     자산(USD) 루트 자동 해석
setup/setup_envs.sh              conda env 3종 생성
sim_harness/MMS_ext_*.py         Isaac 검증 하니스 (실행 시 Isaac 트리로 복사/링크)
```

---

## 환경

**운영체제에 따라 지원 범위가 상이하다.** real 백엔드는 턴테이블 구동에
`utils/turntable/Eziservo_x64` 의 **Windows 전용 DLL**(`FAS_EziMOTIONPlusE`)을 사용하므로
Windows 환경에서만 동작하며, sim 백엔드는 양쪽 모두 지원한다.

| 구분 | Ubuntu | Windows |
|---|---|---|
| sim (`isaac`) | ✅ 주 개발·검증 환경 | ✅ 씬 대조 및 충돌 캐시 재생성 용도 |
| real | ❌ 미지원 (턴테이블 DLL 부재) | ✅ 운영 환경 |
| 환경 구성 | `bash setup/setup_envs.sh` 실행 | **`docs/install.md`** 절차 준수 |
| Artec SDK 바인딩 | 불필요 | **직접 빌드 필요** |
| `env -u PYTHONPATH` | **필수** (ROS python3.10 혼입 방지) | 불필요 |
| 콘솔 인코딩 | 기본 UTF-8 | `$env:PYTHONIOENCODING="utf-8"` 설정 (cp949 문자 깨짐 방지) |

### Ubuntu — sim

순서대로 실행한다. 최초 1회는 1~3 단계를 수행하고, 이후에는 4 단계만 반복한다.

**1) 저장소 복제**
```bash
git clone git@github.com:JinkyoJB/MMS.git ~/workspace/MMS
cd ~/workspace/MMS
```

**2) conda env 3종 생성** — `~/miniconda3/envs/<env>/bin/python` 경로 배치를 전제로 한다.
```bash
bash setup/setup_envs.sh
```
완료 시 env 3종이 생성되며 스크립트가 자체 검증까지 수행한다.

**3) 자산(USD) 배치** — 상세는 아래 「자산(USD) 내려받기」 절 참조.
```bash
export MMS_ASSET_ROOT=~/mms-assets/2_3Dassets     # .bashrc 에 등록 권장
```

**4) 실행** — `env -u PYTHONPATH` 는 ROS python3.10 경로 혼입을 막는 장치로 **생략할 수 없다.**
```bash
env -u PYTHONPATH MMS_BACKEND=isaac \
    ~/miniconda3/envs/env_isaacsim/bin/python -u main_artec.py --no-prompt
```

**5) 검증** — 하드웨어 없이 셀 기하·IK·충돌을 점검한다.
```bash
~/miniconda3/envs/mms-env/bin/python scripts/artec/validate_real_cell.py
```

### Windows — real

`setup_envs.sh` 는 Linux 전용이므로 사용할 수 없다. 아래는 `docs/install.md` 에서 검증된
절차이며, 3 단계의 소요가 가장 크다.

**1) Miniforge 설치**
```powershell
winget install --id CondaForge.Miniforge3 --source winget --scope user `
  --accept-package-agreements --accept-source-agreements
```
⚠ `--source winget` 을 생략하면 msstore 약관 프롬프트에서 중단된다.
⚠ 설치 후 **터미널을 재시작**해야 conda 가 인식된다. 확인: `conda --version`

**2) 저장소 복제 및 `mms-env` 생성**
```powershell
git clone git@github.com:JinkyoJB/MMS.git C:\Users\user\workspace\MMS
cd C:\Users\user\workspace\MMS
conda create -y -n mms-env python=3.11
conda activate mms-env
pip install -r requirements.txt
python -c "import numpy,cv2,open3d; print(numpy.__version__, cv2.__version__, open3d.__version__)"
```
`2.x / 4.x / 0.19+` 이면 정상이다. **cv2 가 5.x 이면 안 된다** —
`cv2.calibrateHandEye` 가 제거되어 hand-eye 캘리브레이션이 동작하지 않는다.

여기까지 완료하면 캘리브레이션·분석 등 **오프라인 스크립트는 모두 동작한다.**

**3) Artec SDK python 바인딩 빌드** — 사전에 Artec 3D Scanning SDK 와
VS Build Tools(**C++ 데스크톱 개발** 워크로드)가 설치되어 있어야 한다.
```powershell
conda activate mms-env
pip install pybind11

$cmake = "C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe"
$py    = "$env:USERPROFILE\miniforge3\envs\mms-env\python.exe"

& $cmake -S mms_artec\sensor -B mms_artec\sensor\build -G "Visual Studio 17 2022" -A x64 `
         -DPython_EXECUTABLE="$py" -DPython_ROOT_DIR="$env:USERPROFILE\miniforge3\envs\mms-env"
& $cmake --build mms_artec\sensor\build --config Release

Get-ChildItem mms_artec\sensor\*.pyd | Measure-Object        # Count = 6 이면 성공
python -c "from mms_artec.sensor import artec_base; artec_base._load(); import artec_sdk_py; print(artec_sdk_py.enumerate_scanners())"
```
⚠ Generator 는 설치된 VS 버전에 맞춘다. ⚠ `-DPython_EXECUTABLE` 을 생략하면 시스템
파이썬을 잡아 `mms-env` 에서 import 되지 않는 `.pyd` 가 생성된다. ⚠ 재구성 시
`build\` 를 삭제한다(`CMakeCache.txt` 에 이전 Generator 가 잔존한다).

**4) 장비 점검** — 위에서 아래로 진행하며, 선행 항목 실패 시 후속 점검은 무의미하다.
```powershell
ping 192.168.1.210 ; ping 192.168.0.10                  # xArm7 / 턴테이블
python scripts/artec/validate_real_cell.py              # 장비 없이 셀 기하·IK·충돌
python scripts/artec/go_home.py                         # 로봇 구동 ⚠ 실제로 움직인다
python scripts/turntable/lookaround_speed_rotation.py   # 턴테이블 ⚠ 실제로 돈다
python scripts/artec/live_scan_view.py                  # 스캐너 + 라이브 점군
```

**5) 실행**
```powershell
conda activate mms-env
$env:MMS_BACKEND = "real"
$env:PYTHONIOENCODING = "utf-8"        # 콘솔 cp949 문자 깨짐 방지
python main_artec.py --until lookaround
```

⚠ `main_artec.py` 의 `BACKEND` 기본값은 `"real"` 이다. 이를 `"isaac"` 으로 변경할 경우
`mms-env` 에 Isaac 이 설치되어 있지 않아 **import 단계에서 실패**한다. 소스 수정 대신
환경변수 `MMS_BACKEND` 로 지정할 것을 권장한다.

### Windows — sim

**필요할 때만 구성한다.** 실물 스캔에는 Isaac 이 불필요하다. CAD 기반 씬(v2/v3/v4)과
실물 셀을 육안 대조하거나, 충돌 캐시(`cell_env.npz`)를 재생성할 때 사용한다
(`docs/collision.md` §6). 2026-09-15 Windows 11 / RTX 3080 Laptop 환경에서 전 과정을
검증하였다.

**1) 사양 확인** — GPU RTX(VRAM 8GB) · RAM 16GB 이상 · 디스크 **약 30GB** 여유.
```powershell
nvidia-smi
```
드라이버 551.95 / CUDA 12.4 에서 정상 동작을 확인하였다. 사전에 드라이버를 갱신할
필요는 없다.

**2) `env_isaacsim` 생성** — 다운로드 약 30GB.
```powershell
conda create -y -n env_isaacsim python=3.11
& "$env:USERPROFILE\miniforge3\envs\env_isaacsim\python.exe" -m pip install `
    -r setup\requirements-isaac.txt --extra-index-url https://pypi.nvidia.com
```
`mms-env` 를 activate 하지 않고 env 의 `python.exe` 를 직접 호출하면 env 오염을 방지할 수 있다.

**3) 자산(USD) 배치** — **필수.** 씬 USD 는 git 에 없으므로 이 단계를 건너뛰면 열 대상이 없다.
```powershell
winget install --id GitHub.cli --source winget --scope user `
  --accept-package-agreements --accept-source-agreements
# ↑ PATH 가 변경되므로 여기서 터미널을 재시작한다

gh auth login                                    # private 저장소이므로 인증이 필요하다
gh release download assets-v2 -R JinkyoJB/MMS -p 'mms-assets-v2.tar.zst' -D $env:TEMP
(Get-FileHash "$env:TEMP\mms-assets-v2.tar.zst" -Algorithm SHA256).Hash.ToLower()

tar -xf "$env:TEMP\mms-assets-v2.tar.zst" -C C:\dev --strip-components=1
```
Windows `tar.exe` 는 zstd 를 자동 인식하므로 `-I zstd` 가 필요 없다. 결과는
`C:\dev\2_3Dassets` 와 `C:\dev\testset` 이다. 리포가 `C:\dev\MMS` 인 경우
`mms_paths.py` 탐색 후보에 그대로 걸리므로 **`MMS_ASSET_ROOT` 설정이 불필요하다.**

⚠ `2_3Dassets` 와 `testset` 은 **형제 디렉터리**여야 한다(v3 씬이 `../../testset/...` 로
대상물을 참조한다). 위 명령은 해당 배치가 되도록 전개한다.

확인:
```powershell
& "$env:USERPROFILE\miniforge3\envs\mms-env\python.exe" -c "import mms_paths,os; p=mms_paths.asset('frame_xarm7_spider_turntable_v2/v3_scene.usd'); print(os.path.exists(p), p)"
```

**4) 씬 열기** — stage 를 열고 대기할 뿐, 로봇 구동이나 물리 연산은 수행하지 않는다.
```powershell
$ISAAC = "$env:USERPROFILE\miniforge3\envs\env_isaacsim\python.exe"
& $ISAAC scripts\sim\view_scene.py                 # v3 (기본)
& $ISAAC scripts\sim\view_scene.py v2              # 구 씬
& $ISAAC scripts\sim\view_scene.py v4              # 턴테이블 이설 검토안
& $ISAAC scripts\sim\view_scene.py v3 --headless   # 창 없이 로드만 확인
```
⚠ **첫 실행은 약 8분이 소요된다.** 익스텐션과 셰이더 캐시를 내려받는 과정이며,
로그에 `Pulling extension:` 만 출력되어도 정상이다. 이후 실행은 캐시를 사용한다.

| 증상 | 원인 및 조치 |
|---|---|
| `씬이 없다: ...v3_scene.usd` | 자산 미설치 → 3) 수행 |
| `씬이 없다: ...v2_real_260917.usd` | 자산이 구버전이다. **`assets-v2`** 로 다시 받는다 |
| `isaacsim` ModuleNotFoundError | `mms-env` 로 실행한 경우다. `env_isaacsim` 의 `python.exe` 를 직접 호출한다 |
| 씬이 비어 있거나 텍스처가 없다 | `2_3Dassets` 와 `testset` 이 형제 관계가 아니다 |
| `torch.cuda.is_available()` 이 `False` | 알려진 사항. `torch==2.7.0` 이 **CPU 전용**으로 설치된다. RTX 렌더러는 Vulkan, PhysX GPU 도 torch 와 별개이므로 씬 확인에는 지장이 없다 |
| `NGX DLSS ... AdapterUnsupported` | 무시한다. Ada 세대 전용 기능이다 |
| EULA 프롬프트에서 중단 | `$env:OMNI_KIT_ACCEPT_EULA="YES"` (`view_scene.py` 는 자동 설정한다) |

### env 3종

| env | 용도 |
|---|---|
| `mms-env` | real 백엔드 + 오프라인 스크립트(캘리브·분석). numpy 2.x |
| `env_isaacsim` | isaac 백엔드 (Isaac Sim 5.1). **numpy 1.x** — 섞으면 ABI 오류 |
| `step2usd` | STEP→USD 전용. Isaac 불필요라 빠름 |

**상세 내용은 `docs/install.md` 참고** (Windows 전 과정 및 장비 점검 명령 수록)

## **자산(USD) 내려받기** — 새 머신에서 최초 1회

씬 USD·텍스처는 GitHub 100 MB 파일 제한을 넘어 git 에 넣지 않는다. 릴리스로 받는다.

> `-R JinkyoJB/MMS`

```bash
gh release download assets-v2 -R JinkyoJB/MMS -p 'mms-assets-v2.tar.zst*'
sha256sum -c mms-assets-v2.tar.zst.sha256

mkdir -p ~/mms-assets
tar -I zstd -xf mms-assets-v2.tar.zst -C ~/mms-assets --strip-components=1
export MMS_ASSET_ROOT=~/mms-assets/2_3Dassets      # .bashrc 에 넣어 두면 편하다
```

⚠ `2_3Dassets` 와 `testset` 은 **형제 디렉터리**여야 한다. v3 씬이 `../../testset/...` 로
대상물을 참조하므로 이 배치가 깨지면 텍스처가 통째로 사라진다.

압축 316 MB / 해제 739 MB. `MMS_ASSET_ROOT` 없이도 `mms_paths.py` 가 알려진 배치를
순서대로 탐색한다(`docs/sim_commands.md`). 자산을 갱신하면 새 태그로 릴리스를 올린다.

**더 자세한건 docs/install.md 참고**

## 장비 연결

**장비** — Artec Spider `SP.10.79103441` (SDK 1.18.4) · xArm7 `192.168.1.210` ·
턴테이블 Ezi-SERVO `192.168.0.10` **UDP**(TCP는 지속 polling 시 socket 막힘)

> SDK 바인딩 변경 시:
> `cmake --build mms_artec/sensor/build --config Release --target <module>`

**장비 테스트** — 위에서 아래로. 앞이 실패하면 뒤는 볼 필요 없다.

```powershell
conda activate mms-env
ping 192.168.1.210 ; ping 192.168.0.10                  # xArm7 / 턴테이블 도달 확인
python -c "import main_artec"                           # 하드웨어 없이 import smoke test
python scripts/artec/validate_real_cell.py              # 장비 없이 셀 기하·IK·충돌 점검
python scripts/artec/go_home.py                         # 로봇 통신·구동 (home 복귀)
python scripts/turntable/lookaround_speed_rotation.py   # 턴테이블 회전 (정지: turntable_stop.py)
python scripts/artec/live_scan_view.py                  # 스캐너 연결 + 라이브 점군 뷰어
```

## 실행 명령

`MMS_BACKEND` 가 `main_artec.py::BACKEND` 를 덮어쓴다 — 소스를 고치지 않고 sim/real 을 바꾼다.

### sim

Linux + `env_isaacsim`. `env -u PYTHONPATH` 는 ROS python3.10 혼입 방지용이라 **필수**.

```bash
cd ~/workspace/MMS

# 기본 (GUI, v2_real 씬)
env -u PYTHONPATH MMS_BACKEND=isaac \
    ~/miniconda3/envs/env_isaacsim/bin/python -u main_artec.py --no-prompt

# 단계 지정(sim 전용 환경변수) + 헤드리스
env -u PYTHONPATH MMS_BACKEND=isaac MMS_SIM_STAGE_UNTIL=lookaround MMS_ISAAC_HEADLESS=1 \
    ~/miniconda3/envs/env_isaacsim/bin/python -u main_artec.py --no-prompt

# 래퍼: [물체] [stage_until]  (⚠ 아직 v3 씬을 쓴다)
./scripts/sim/run_e2e_gui.sh mug nbv
```

**더 자세한건 docs/sim_commands.md 참고**


### real

Windows + `mms-env`. 턴테이블 드라이버가 Windows 전용 DLL 이다.

```powershell
conda activate mms-env
cd C:\Users\user\workspace\MMS
$env:MMS_BACKEND = "real"
$env:PYTHONIOENCODING = "utf-8"       # 콘솔 cp949 이모지 깨짐 방지

python main_artec.py                                            # 기본 (preview→lookaround→nbv)
$env:MMS_BACKEND="real"; python -u main_artec.py --range-video  # 캡쳐 화면 보면서 실행
python main_artec.py --until flip --no-prompt                   # 전 단계 무인 실행
python main_artec.py --until lookaround --max-passes 1 --test   # 한 자세만, texturize 생략
```

real 은 셸 환경변수로 단계를 바꾸지 않는다(`MMS_SIM_STAGE_UNTIL` 무시) — **`--until` 로만** 준다.

| 인자 | 뜻 |
|---|---|
| `--until preview\|lookaround\|nbv\|flip` | 여기까지 실행 (앞 단계는 항상 포함) |
| `--max-passes N` | pass 상한. `1` 이면 한 자세만 돌고 끝 |
| `--no-prompt` | pass 사이 Enter 확인 생략 (무인 연속) |
| `--no-planner` | lookaround 자세 플래너 끔 (home 고정, 캡처 루프만) |
| `--no-viewer` / `--no-range-view` | 라이브 점군 / 거리추종 디버그 창 끔 |
| `--range-video` | 거리 뷰를 동영상으로도 저장 (`output/<RUN_TS>/debug/range.avi`) |
| `--no-recovery` | tracking lost 자동 복구 끔 |
| `--speed-scale K` | 로봇 이동 속도 K 배 (계획·NBV·복구 일괄) |
| `--no-sproj` | `.sproj` 저장 생략 (각 ~47초) |
| `--texturize` / `--test` | SDK Texturize 강제 ON(15~20분) / 강제 OFF(반복 테스트용) |

**더 자세한건 docs/7_real_commands.md 참고**


## 파이프라인

`preview → lookaround → nbv → flip → postprocess` 가 실행 순서이자 `--until` 의 순서다.
아래 커맨드는 전부 위 **real** 블록(`conda activate mms-env`, `MMS_BACKEND=real`) 기준.

### 1. (optional) calibration
설치 후 1회, 이설 시 재수행. 순서가 강제된다 — **intrinsic → hand-eye → 턴테이블 축**
(rim 점을 base 로 옮길 때 `T_EC` 가 쓰이기 때문).

```powershell
python scripts/artec/make_charuco.py --pdf                     # 보드 인쇄 (최초 1회, "실제 크기 100%")
python scripts/artec/gen_calib_poses.py --from-view --write    # hand-eye 자세 목록
python scripts/artec/calibrate.py                              # 전체 (--from 2 / --only 3 로 부분 실행)
```

| 인자 | 하는 일 · 결과 |
|---|---|
| `--only 1` | intrinsic → `config/calibration/artec_intrinsic.yaml` |
| `--only 2` | hand-eye → `config/sensor_frames.yaml::T_EC_artec` (기준 t_err 3.55mm / r_err 1.30°) |
| `--only 3` | 턴테이블 축 → `config/calibration/turntable_frame.yaml` (기준 0.015° / 0.7mm) |
| `--from N` | N 번 단계부터 끝까지 |

⚠ 저장된 `T_B_F0` 는 Artec 장착 **이전** 값이다 — 실물 재개 시 `--only 3` 부터 다시 잡을 것.
**더 자세한건 docs/1_calibration.md · docs/calibration_runbook.md 참고**

### 2. preview
스캔이 아니라 **측량**이다. 물체를 훑어 높이·반경·적정 작업거리를 재고 뒤 단계의 입력을 만든다.

```powershell
python main_artec.py --until preview
python scripts/nbv/verify_preview_contract.py     # 하드웨어 없이 0.3초 계약 검사
```
주요 손잡이: `MMS_WORK_STANDOFF_MM`(225, 카메라↔표면 목표거리) ·
`MMS_PREVIEW_RADIUS_MAX_MM`(180, 상정 최대 반경).
탐침별 거리 이미지 → `output/<RUN_TS>/debug/preview/`, 카메라 스냅샷 → `debug/cam/preview/`.
**더 자세한건 docs/2_preview.md 참고**

### 3. lookaround
물체를 **높이 밴드로 썰어**, 밴드마다 그 높이의 자세로 팔을 옮기고 턴테이블을 전회전.
한 자세가 다 덮으면 밴드는 1개다.

```powershell
python main_artec.py --until lookaround
python scripts/artec/lost_report.py                       # <RUN_TS>/events.jsonl 집계 — 언제 잃고 뭘 했나
python scripts/artec/live_range_view.py --run <RUN_TS>    # 거리추종 뷰 수동 실행
```
주요 손잡이: `MMS_BAND_OVERLAP`(0.40 — **밴드 밀도의 유일한 손잡이**) ·
`MMS_STANDOFF_TRACK`(`live`, 캡처 중 축거리 보정 `off`/`band`/`live`).
거리 이미지 → `output/<RUN_TS>/debug/lookaround/`.
**더 자세한건 docs/3_lookaround.md 참고**

### 4. nbv
로봇팔은 **최소로** 움직이고 턴테이블만 자유 회전하며 부족면(hole)을 메우는 수렴 루프.

```powershell
python main_artec.py --until nbv
python scripts/nbv/nbv_debug_view.py     # 반복별 gap·후보 뷰 ([ ] 로 이동)
```
주요 손잡이: `MMS_NBV_K_MAX`(12, 최대 반복) · `MMS_NBV_DRY_EPS`(0.015, 패치 생산성) ·
`MMS_NBV_CONV_NEW_EPS`/`_STALL_N`(0.005 / 3, 전역 백스톱).
계획 덤프 → `output/<RUN_TS>/debug/nbv_plan/` (`MMS_NBV_DEBUG=0` 로 끔) ·
정지-촬영 프레임 → `debug/nbv/`.
**더 자세한건 docs/4_nbv.md 참고**

### 5. flip
물체를 **외부에서 뒤집고** 턴테이블 전회전 — 앞 단계에서 바닥이라 못 본 면을 수집.
뒤집은 패스를 master 에 붙이는 **정합 힌트**가 이 단계의 핵심이다.

```powershell
python main_artec.py --until flip
python scripts/artec/reg_hint_test.py --run <RUN> --H-mm <높이>   # 힌트만 따로 검증
python scripts/artec/reg_hint_post.py --run <RUN>                # 그 배치로 SDK 후처리까지
```
주요 손잡이: `MMS_SIM_FLIP_AXIS`(`y`) · `MMS_SIM_FLIP_ANGLES`(180) ·
`MMS_SIM_FLIP_ASPECT`(2.0, 넘으면 90° 추가) · `MMS_SIM_FLIP_EL_MIN`/`_MAX`(30 / 70).
⚠ flip pivot 은 **원판면 + H/2**, H 는 밀도 기준 높이여야 한다(무게중심 피벗은 오차).
거리 이미지 → `output/<RUN_TS>/debug/flip/`.
**더 자세한건 docs/5_flip.md 참고**

### 6. postprocess
스캔이 끝나면 `main_artec.py` 가 자동으로 부른다. SDK 호출 순서는
`SerialReg → GlobalReg → **Outliers** → Fusion(poisson) → **SmallObjects** → Simplify → Texturize`
— ⚠ Studio GUI 라벨 순서와 다르다. Outliers 는 Fusion **전**, SmallObjects 는 **후**(composite mesh 입력).

```powershell
python scripts/artec/save_raw_scan.py                               # 스캔 1회 → output/scan_raw/<TS>/
python scripts/artec/merge_compare.py --load output/scan_raw/<TS>   # 스캔 없이 병합 variant 비교
python scripts/artec/reg_offline.py --run <RUN> --methods hint,img,greg
```
설정은 `ArtecProcessSettings`(`mms_artec/system.py`) — `dev_mode` 로 무거운 단계(simplify·texturize)를
건너뛴다. 실물 스캔은 한 번에 수 분이니 **알고리즘을 손볼 때는 반드시 `--load` 경로**를 쓸 것.
결과는 `output/<RUN_TS>/` 한 폴더에 모인다 — 손으로 마무리할 때 Studio 로 여는 건
**`aligned/aligned.sproj`**(SDK 후처리 전), 절차는 그 폴더의 `README.txt`.
**더 자세한건 docs/6_postprocess.md 참고**

### collision
별도 단계가 아니다 — 모든 이동이 지나는 한 곳(`CollisionModel`)에 있어 lookaround·nbv·flip 이
자동으로 덮인다. 호출부에서 따로 할 일은 없다.

```powershell
python scripts/artec/validate_real_cell.py     # 장비 없이 IK·충돌·home 경로 통과율 점검
```
```bash
# 캐시 재생성 — 셀을 개조했다면 필수. Isaac Sim 이 있어야 한다(현장 PC 는 불가)
env -u PYTHONPATH $ISAAC scripts/sim/export_env_mesh.py      # cell_env.npz (base 프레임)
env -u PYTHONPATH $ISAAC scripts/sim/export_link_meshes.py   # xarm7_spider_links.npz
```
⚠ `[collision] 모델 로드 실패` 가 뜨면 검사가 **통째로 skip** 된다 — 그 상태로 로봇을 움직이지 말 것.
**더 자세한건 docs/collision.md 참고**

---

## 산출물 경로

**한 run = 한 폴더**다 — `output/<RUN_TS>/` (`RUN_TS` = 실행 일시 `YYYYMMDD_HHMMSS`).
규칙의 출처는 `utils/run_paths.py` 한 곳이다.

```
output/<RUN_TS>/
├─ README.txt              후임용 안내 — 어느 파일을 Studio 로 여는지 + 수작업 후처리 절차
├─ final.obj               최종 융합 메시 (SDK Texturize 를 안 돌렸으면 텍스처 없음)
├─ aligned/aligned.sproj   ★ Artec Studio 로 여는 것 — 파이프라인 변환만 적용, SDK 후처리 전
├─ final/final.sproj       우리 후처리 결과 (융합 메시 포함). 메시가 섞여 정합 입력으로는 부적합
├─ timeline.csv            θ·EE pose 등 프레임 메타 (streaming 은 result.ctx=None 이라 여기에)
├─ run.log                 콘솔 전체 tee (줄머리에 경과시간). MMS_NO_LOGFILE=1 로 끔
├─ events.jsonl            lost·밴드 전환·복구·병합(method) 이벤트 → scripts/artec/lost_report.py
├─ scan_dumps/             scanNN_<stage>_poseK.npz — IScan 별 점군 + 적용된 T_pre_mm·R_phys·master_T_CB
└─ debug/                  ↓ 아래 표
```

sim 백엔드는 `final.obj` 대신 `output/sim_scan_<RUN_TS>.ply` 로 나간다.
`MMS_RUN_TS` 없이 스크립트를 단독 실행하면 **`output/_norun/`** 밑에 쌓인다 — run 폴더를 더럽히지 않게.

> `--no-sproj` 면 `aligned`·`final` 둘 다 안 남는다(각 ~47초).
> `--test` 면 texturize 를 건너뛰어 OBJ 에 텍스처가 없다.

### 디버깅 이미지 — `output/<RUN_TS>/debug/`

단계마다 **그 순간 스캐너가 실제로 본 그림**을 남긴다. "왜 저기를 봤나" 는 여기부터 연다.
run 중에는 `live_range_view.py` 가 이 폴더를 tail 해서 창으로 띄운다.

| 경로 | 언제 |
|---|---|
| `debug/preview/p{NNN}_tz###_d###_az###.png` | preview 탐침마다 — 거리 이미지. '빈 시야' 판정이 맞나 |
| `debug/lookaround/s{NN}_band{b}_start{k}.png` | 밴드 시작 축거리 보정마다 |
| `debug/lookaround/s{NN}_band{b}_live_f{n}.png` | 회전 중 `live` 거리추종 판정마다 — '정말 멀어졌나' |
| `debug/nbv/nbv{NN}_step{KK}_th{DDD}.png` | nbv 정지-촬영 프레임마다 |
| `debug/flip/s{NN}_band{b}_*.png` | flip 밴드 캡처 (lookaround 와 같은 형식) |
| `debug/cam/<단계>/…png` | 카메라 스냅샷 (거리 이미지가 아니라 사진). preview 이동마다 등 |
| `debug/nbv_plan/nbv_<시각>_<NN>.npz` + `.png` | nbv 계획 덤프 — master·gap 후보·정합 결과. `MMS_NBV_DEBUG=0` 로 끔 |
| `debug/iso/*.ply` | probe static/moving/object 색상 PLY. 기본 **OFF**(`probe_debug_dump=True`), CloudCompare 로 본다 |
| `debug/preview_points_*.npz` | preview 계획 점군 |
| `debug/range.avi` · `range_view.log` | 뷰어 녹화(`--range-video` 일 때만) · 뷰어 로그(창이 안 뜨면 여기부터) |

> 디렉터리는 `MMS_DEBUG_DIR`(카메라 스냅샷 루트) · `MMS_NBV_DEBUG_DIR`(계획 덤프) 로 옮길 수 있다
> — testset 여러 종을 연달아 돌릴 때 물체별로 갈라 담는 용도.

---

**개발 원칙** — 모든 로직은 sim(ground-truth)에서 개발·검증하고 real(Artec SLAM)에
동일 코드를 적용한다. sensor-agnostic 코어(`utils/`)를 real(Artec)과 sim(Isaac)이 공유한다.

> 단계별 구현 상태와 향후 과제는 **인수인계서_A1** §5·§6 참조.
