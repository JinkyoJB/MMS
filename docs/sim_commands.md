# sim 커맨드 모음

> 실물 장비로 돌릴 때는 **`7_real_commands.md`**.
> 씬을 만들거나 셀 배치를 바꾸려면 **`sim_scene.md`**.

---

## 0. 그냥 돌리고 싶다 — 복붙 3줄

### ① 내 환경이 맞는지 (한 번만, 5초)

그대로 붙여넣으면 **뭘 고쳐야 하는지까지** 알려준다.

```bash
cd ~/workspace/sync/2_Rapid_Digital_Twin/1_MMS/MMS   # ← 리포 경로. 다르면 여기만 바꾼다
for p in ~/miniconda3/envs/env_isaacsim/bin/python ~/miniconda3/envs/mms-env/bin/python; do
  [ -x "$p" ] && echo "✓ $p" || echo "✘ 없음: $p   → bash setup/setup_envs.sh"
done
env -u PYTHONPATH ~/miniconda3/envs/env_isaacsim/bin/python -c "import isaacsim" 2>/dev/null \
  && echo "✓ isaacsim import OK" || echo "✘ isaacsim 없음 → env_isaacsim 재설치"
env -u PYTHONPATH ~/miniconda3/envs/mms-env/bin/python -c "
import mms_paths as p, pathlib
r = pathlib.Path(p.asset_root()); s = r/'frame_xarm7_spider_turntable'/'v2_real_260917.usd'
print(('✓ 자산 ' if r.is_dir() else '✘ 자산 없음 ')+str(r))
print(('✓ 기본 씬 ' if s.is_file() else '✘ 기본 씬 없음 ')+str(s))
print('  (자산 ✘ : export MMS_ASSET_ROOT=/경로/2_3Dassets — README 의 gh release download)')
print('  (씬  ✘ : 릴리스에 없는 생성물이다 — sim_scene.md §3 의 2번을 한 번 돌린다)')"
```

### ② 돌린다 (GUI)

```bash
env -u PYTHONPATH MMS_BACKEND=isaac \
    ~/miniconda3/envs/env_isaacsim/bin/python -u main_artec.py --no-prompt
```

### ③ v3 씬으로 돌리고 싶다면

**기본 씬은 실물 배치(`v2_real_260917.usd`)다**(2026-09-17~). v3 는 셀을 바꾸기로
했다가 실제로는 안 바꾼 배치라, 자세 선정·도달성·이동량이 실물과 다르다.
그래도 v3 로 봐야 한다면 씬·레이아웃·home 을 **셋 다** 같이 준다:

```bash
ASSET=$(env -u PYTHONPATH ~/miniconda3/envs/mms-env/bin/python -c \
        "import mms_paths;print(mms_paths.asset_root())")
env -u PYTHONPATH MMS_BACKEND=isaac \
    MMS_SIM_USD="$ASSET/frame_xarm7_spider_turntable_v2/v3_scene.usd" \
    ~/miniconda3/envs/env_isaacsim/bin/python -u main_artec.py --no-prompt
```
충돌 레이아웃은 씬 경로에서 자동으로 `v3_layout_sim` 이 잡힌다. home 은
`MMS_SIM_HOME_DEG` 로 덮어쓴다(`isaac_xarm.py` 주석).

| | v3_scene | **v2_real_260917** (기본) |
|---|---|---|
| 턴테이블 축 (base) | `[0, 0, 0.835]` 수평 **0 mm** | `[0.799, 0.005, 0.688]` 수평 **799 mm** |
| 도달여유 | +255 mm | **+35 mm** |
| sim T_EC vs 실측 hand-eye | 104 mm 차이 | **0.000 mm** |

> 씬을 새로 만들거나 셀 배치가 바뀐 경우는 **`sim_scene.md`**.

### ④ 물체·단계를 바꾸고 싶다

```bash
./scripts/sim/run_e2e_gui.sh spray_can nbv    # 물체 spray_can, nbv 까지 (⚠ v3 씬)
```
인자는 `[물체] [stage_until]`. 물체 이름은 부분만 써도 된다
(`mug` `drill` `detergent` …). `stage_until` 는 단계 이름이고 **앞 단계는 항상 포함**이다:
`preview`(계획만) · `lookaround`(5면) · `nbv`(+보강) · `flip`(+바닥면).
⚠ 이 스크립트는 아직 **v3 씬**을 쓴다 — 실물 배치로 보려면 ② 를 쓸 것.

**헤드리스**(GUI 없이, 원격/CI)로 돌리려면 위 ②에 `MMS_ISAAC_HEADLESS=1` 을 붙인다.

> **왜 `env -u PYTHONPATH` 가 붙나** — 셸에 ROS 의 python3.10 경로가 잡혀 있으면
> 3.11 env 에 섞여 죽는다. 빼먹으면 원인을 알기 힘든 import 오류가 난다.
>
> **왜 `MMS_BACKEND=isaac`** — `main_artec.py::BACKEND` 는 소스 스위치다. 이 환경변수가
> 그걸 덮어써서, 소스를 고치지 않고 sim/real 을 바꿀 수 있다. 실물은 `MMS_BACKEND=real`
> + `mms-env` 파이썬(`7_real_commands.md`).

아래 §1~§4 는 **씬을 새로 만들거나 진단할 때만** 본다.

---

모든 명령은 **리포 루트**에서 실행:
```bash
cd <MMS 리포>          # 예: .../A1_.../1_코드/MMS
```

→ 자산(USD) 경로 해석 규칙과 `MMS_ASSET_ROOT` 등은 **`sim_scene.md` §6**.

## ⚠ 공통 규칙 2가지

**1. `env -u PYTHONPATH` 를 반드시 붙인다** — 셸에 ROS Humble 의 python3.10 경로가
잡혀 있어 3.11 env 에 섞인다.

**2. env 를 용도별로 구분한다**

| env | 용도 | 경로 |
|---|---|---|
| `env_isaacsim` | Isaac Sim 이 필요한 것(씬 렌더·물리·main_artec sim) | `~/miniconda3/envs/env_isaacsim/bin/python` |
| `step2usd` | USD 생성·편집만(pythonocc + usd-core, Isaac 불필요 → 빠름) | `~/miniconda3/envs/step2usd/bin/python` |
| `mms-env` | real 백엔드 + 오프라인 스크립트 | `~/miniconda3/envs/mms-env/bin/python` |

아래에서는 `$ISAAC`, `$U2` 로 줄여 쓴다:
```bash
ISAAC=~/miniconda3/envs/env_isaacsim/bin/python
U2=~/miniconda3/envs/step2usd/bin/python
ASSET=$(source scripts/sim/_paths.sh && mms_asset_root)/frame_xarm7_spider_turntable_v2
```

---

## 1. 씬 생성

→ **`sim_scene.md`** 로 옮겼다. 새 물체 추가는 §2, CAD 에서 새로 만들기는 §4,
셀 배치가 바뀐 경우는 §3.

---

## 2. 확인·진단

### IK 도달성 (드래그 타깃)
GUI 에서 `/World/IKTarget` 을 기즈모로 끌면 로봇이 따라온다. 초록=도달, 빨강=실패.
```bash
env -u PYTHONPATH $ISAAC scripts/sim/ik_follow_target.py                    # 스캐너 카메라 기준
env -u PYTHONPATH $ISAAC scripts/sim/ik_follow_target.py --target flange    # link7 플랜지 기준
env -u PYTHONPATH $ISAAC scripts/sim/ik_follow_target.py --target adapter   # 어댑터 중심(Spider 자동 비활성)
```

**임의 prim 기준** — 경로를 그대로 넘기면 그 prim 의 **메시 중심**이 기준점이 된다:
```bash
env -u PYTHONPATH $ISAAC scripts/sim/ik_follow_target.py --rebuild \
    --target /World/xarm7/link7/tool/p_5K_TCC1_MOUNTING_FLANGE_BRACKET_20240724_AllCATPart
```

| 옵션 | 설명 |
|---|---|
| `--rebuild` | IK 씬 재생성 (`v3_scene.usd` 를 다시 만든 뒤 필수) |
| `--selftest` | GUI 없이 격자 도달성만 출력 |
| `--hide-spider` | Spider 비활성 (adapter 모드는 자동) |
| `--max-jump 90` | 1스텝 관절 급변 허용 deg. 빠르게 끌 때 자꾸 멈추면 올린다 |
| `--pos-tol 0.005` / `--rot-tol 5.0` | 허용 위치·자세 오차 |

씬 파일은 기준점별로 따로 생성된다 (`v3_scene_IK.usd`, `v3_scene_IK_adapter.usd`, …).
방법론·함정은 `$ASSET/v3_scene_IK.md`.

### 캘리브 검증 (standalone)

```bash
env -u PYTHONPATH $ISAAC -u scripts/sim/calib_handeye_sim.py   # hand-eye (~90초)
```
> GT 대비 `t_err`/`r_err` 출력. 기준 5mm/2°. 상세는 `1_calibration.md` §4.

### 턴테이블 축 캘리브 검증 (standalone)

```bash
env -u PYTHONPATH $ISAAC scripts/sim/calib_rim_sim.py     # 캡처 → log/rim_capture.npz
env -u PYTHONPATH python scripts/sim/rim_click_offline.py  # mms-env, 클릭+피팅
MMS_RIM_AUTO=1 env -u PYTHONPATH $ISAAC scripts/sim/calib_rim_sim.py   # 자동
```
> 상세는 `1_calibration.md` §8.

### 턴테이블 회전
```bash
env -u PYTHONPATH $ISAAC scripts/sim/spin_turntable.py --gui            # 계속 회전
env -u PYTHONPATH $ISAAC scripts/sim/spin_turntable.py --angle 90 --no-loop
```
`--step 1`(스텝당 deg) `--rider <prim>`(같이 돌릴 prim)

> GUI 에서 손으로 돌리려면 `/World/frame/turntable_disc` 의 Transform → **Rotate Y**.
> (`/World/frame` 에 rotateX(90°) 가 걸려 있어 부모 Y = 월드 수직축. Z 를 돌리면 눕는다)

### home 관절각 재산출
배치가 바뀌면 home 이 안 맞는다. USD 실측 기반 IK 탐색.
```bash
env -u PYTHONPATH $U2 scripts/sim/find_home_pose.py
env -u PYTHONPATH $U2 scripts/sim/find_home_pose.py --dist 0.28 --el 45 55 65 75
```
결과를 `mms_artec/backends/isaac/isaac_xarm.py::HOME_JOINTS_DEG["artec"]` 에 반영.

---

## 3. 파이프라인 실행

```bash
env -u PYTHONPATH PYTHONUNBUFFERED=1 $ISAAC -u main_artec.py                    # GUI
env -u PYTHONPATH MMS_ISAAC_HEADLESS=1 MMS_SIM_NO_VIZ=1 PYTHONUNBUFFERED=1 \
    $ISAAC -u main_artec.py                                                     # 헤드리스
```

실행 범위는 `main_artec.py::MULTIPASS_SETTINGS.stage_until` 를 따른다
(`preview` / `lookaround` / `nbv` / `flip` — 앞 단계는 항상 포함). 환경변수로 override:
```bash
MMS_SIM_STAGE_UNTIL=preview   # 계획까지만 (전회전 안 함) — 스윕/디버깅용
MMS_SIM_NTHETA=8         # 회전 프레임 수(기본 240 = MMS_SIM_FRAMES_PER_REV) — 빠른 확인용
MMS_SIM_DRIVE_STEPS=6
MMS_SIM_USD=<usd>        # 씬 override
MMS_SIM_OBJECT_PRIM=<prim>
```

### testset 순회 (v3)
```bash
source scripts/sim/_paths.sh
mms_v3_scene            # 기본 씬 (v3_scene.usd)
mms_v3_ts spray_can     # 물체별 씬 (부분이름 매칭)
mms_testset_dir         # 대상물 USD 트리

# 물체별 씬 생성 (1회)는 sim_scene.md §2

# lookaround 만 가볍게 9종 순회
scripts/sim/e2e_sweep.sh
# 9종 전체 lookaround→nbv→flip 스윕 (물체당 ~5분, summary.tsv 생성)
scripts/sim/testset_sweep.sh              # 전체
scripts/sim/testset_sweep.sh 0146_mug     # 특정 물체만
```
> ⚠ 이 스윕은 **v3 씬**을 쓴다 — 실물 배치가 아니다(`sim_scene.md` §1).
> 쓸 수 없는 구 씬과 상대경로 규약도 거기에 정리돼 있다.

### GT 평가·렌더 (스윕 후)
```bash
env -u PYTHONPATH $U2 scripts/sim/extract_gt_mesh.py --all       # 씬→정답 표면 (1회)
python scripts/sim/eval_vs_gt.py --scan <obj> --gt scripts/sim/log/gt/<이름>.npz
python scripts/sim/render_scan_results.py --logs scripts/sim/log/testset_sweep
python scripts/sim/build_results_page.py   # docs/testset_results.md 의 웹판 HTML
```

### 진단·튜닝 env (isaac_scan_session)
| env | 기본 | 용도 |
|---|---|---|
| `MMS_SIM_PROFILE_EVERY` | 20 | N프레임마다 단계별 소요시간 (0=끔) |
| `MMS_SIM_ICP_DEBUG` | 0 | ICP 스케일별 fitness/drift 출력 |
| `MMS_SIM_STAGE_DUMP` | — | nbv 반복마다 메시+누적점군 덤프 dir |
| `MMS_NBV_CONV_NEW_EPS` / `MMS_NBV_CONV_STALL_N` | 0.005 / 3 | 전역 수렴 백스톱 (신규복셀 비율/연속횟수, sim·real 공용) |
| `MMS_NBV_DRY_EPS` | 0.015 | gap 패치 생산성 판정 (dry 회계, sim·real 공용) |
| `MMS_SIM_FLIP_ASPECT` | 2.0 | 세장형(90° flip 추가) 종횡비 임계 |
| `MMS_SIM_FLIP_EL_MIN` / `_MAX` | 30 / 70 | flip 관측 고도각 하한·상한(°) |
| `MMS_SIM_FLIP_ANGLES` | 자동 | flip 각 명시 (설정 시 정책 무시) |
| `MMS_SIM_ICP_INFLATION` | 0.005 | 정합이 물체를 부풀리는 한계 m (평행이동 오류 차단) |
| `MMS_REAL_INFLATION_GATE` | 1 | real 팽창 게이트 (0=끔, 실기 검증 전) |

---

## 4. 자주 겪는 것

| 증상 | 원인·조치 |
|---|---|
| 로봇이 떨린다 / 끊긴다 | 구동 3단·게인 문제 → `$ASSET/v3_scene_IK.md` 함정 1 |
| `MMS_SIM_*` 를 줬는데 안 먹는다 | 실행 중 프로세스엔 반영 안 된다. 다시 띄운다 |

**씬 쪽 증상**(모든 자세 거부 · 캡처 0점 · 새까만 렌더 · 빈 씬 · `turntable prim not
found` · `pxr` import 실패)은 **`sim_scene.md` §8**.

---

## 관련 문서

- `docs/testset_results.md` — 9종 스윕 결과(GT 지표·결함 7건·렌더)
- `$ASSET/v3_scene_IK.md` — 드래그 IK 타깃 방법론(다른 로봇 재사용용)
- `README.md` — 전체 파이프라인 개요
