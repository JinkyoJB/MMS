# sim 커맨드 모음 (v3_scene)

> 실물 장비로 돌릴 때는 **`7_real_commands.md`**.

모든 명령은 **리포 루트**에서 실행:
```bash
cd <MMS 리포>          # 예: .../A1_.../1_코드/MMS
```

## 자산(USD) 경로 — 자동 해석

절대경로를 소스에 박지 않는다. `mms_paths.py`(python) / `scripts/sim/_paths.sh`(셸)이
아래 순서로 **자산 루트(`2_3Dassets`)** 를 찾는다.

| 순위 | 위치 |
|---|---|
| 1 | 환경변수 `MMS_ASSET_ROOT` |
| 2 | `<repo>/../../2_데이터/2_3Dassets` (인수인계 폴더 배치) |
| 3 | `<repo>/../2_3Dassets` (원본 개발 배치) |
| 4 | `<repo>/2_3Dassets` |
| 5 | 구 개발머신 절대경로 (하위호환) |

확인:
```bash
python3 mms_paths.py                                    # 해석 결과 출력
source scripts/sim/_paths.sh && mms_asset_root          # 셸 쪽
```

자산을 다른 곳에 두었다면:
```bash
export MMS_ASSET_ROOT=/경로/2_3Dassets
export MMS_TESTSET_DIR=/경로/testset      # testset USD 트리 (기본 ~/isaacsim/standalone_examples/play/MMS/testset)
export MMS_PYTHON=/경로/python            # Isaac 파이썬 (기본 ~/miniconda3/envs/env_isaacsim/bin/python)
```

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

### STEP → 형상 USD (`v3.usd`)
CAD 가 바뀐 경우에만. ~47초.
```bash
env -u PYTHONPATH $U2 scripts/sim/step2usd.py \
    --step "$(source scripts/sim/_paths.sh && mms_asset_root)/frame_v2/260811_frame.STEP" \
    --out $ASSET --name v3
```
옵션: `--deflection 0.5`(mm) `--keep-fasteners` `--source-up y|z`

> `v3.usd` 와 `parts/*.usd` 는 **읽기 전용(chmod 444)** 이다. 재생성하려면 먼저 풀 것:
> `chmod 644 $ASSET/v3.usd $ASSET/parts/*.usd`

### 형상 → sim 씬 (`v3_scene.usd`)
`v3.usd` 를 subLayer 로 깔고 오버라이드만 얹는다(비파괴). **수동 조정은 여기서 날아간다** —
GUI 에서 손본 게 있으면 먼저 값을 뽑아 스크립트 상수에 박을 것.
```bash
env -u PYTHONPATH $U2 scripts/sim/build_scene_v3.py --dir $ASSET
```
주요 옵션:
```
--object <testset.usd>|none   스캔 대상 (기본 0146_mug)
--scale-drop 0.115            저울 하강량 m
--scanner-clock-deg 180       어댑터+스캐너 툴축 회전
--cam-t X Y Z                 Camera 위치(스캐너 프레임 로컬, m)
--collider-approx convexHull|boundingCube
--no-physics / --no-lights / --no-ground
```

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
> GT 대비 `t_err`/`r_err` 출력. 기준 5mm/2°. 상세는 `1_calibration.md` §7.

### 턴테이블 축 캘리브 검증 (standalone)

```bash
env -u PYTHONPATH $ISAAC scripts/sim/calib_rim_sim.py     # 캡처 → log/rim_capture.npz
env -u PYTHONPATH python scripts/sim/rim_click_offline.py  # mms-env, 클릭+피팅
MMS_RIM_AUTO=1 env -u PYTHONPATH $ISAAC scripts/sim/calib_rim_sim.py   # 자동
```
> 상세는 `1_calibration.md` §13.

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

phase 는 `main_artec.py::MULTIPASS_SETTINGS.phase_mode` 를 따른다(1=Phase1 / 2=+NBV /
3=+바닥면 flip). 환경변수로 override 가능:
```bash
MMS_SIM_PHASE_MODE=1     # 스윕 스크립트용 override
MMS_SIM_NTHETA=8         # 회전 프레임 수(기본 36) — 빠른 확인용
MMS_SIM_DRIVE_STEPS=6
MMS_SIM_USD=<usd>        # 씬 override
MMS_SIM_OBJECT_PRIM=<prim>
```

### testset 순회 (v3)
```bash
# 물체별 v3 씬 생성 (1회) — step2usd 환경
env -u PYTHONPATH $U2 scripts/sim/build_scene_v3.py --dir $ASSET \
    --out v3_ts_<이름>.usd --object ~/isaacsim/standalone_examples/play/MMS/testset/<이름>.usd

# 9종 전체 Phase 1→2→3 스윕 (물체당 ~5분, summary.tsv 생성)
scripts/sim/testset_sweep.sh              # 전체
scripts/sim/testset_sweep.sh 0146_mug     # 특정 물체만
```
> 구 `e2e_sweep.sh`/`composed/*.usd` 는 **v2 레이아웃**이라 턴테이블 prim 을 못 찾고
> 30초 만에 빈 결과로 끝난다(실측 2026-08-19). v3_ts_* 씬을 쓸 것.

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
| `MMS_SIM_STAGE_DUMP` | — | Phase2 반복마다 메시+누적점군 덤프 dir |
| `MMS_SIM_CONV_NEW_EPS` / `_N` | 0.005 / 3 | 전역 수렴 백스톱 (신규복셀 비율/연속횟수) |
| `MMS_SIM_DRY_EPS` | 0.015 | gap 패치 생산성 판정 (dry 회계) |
| `MMS_SIM_FLIP_ASPECT` | 2.0 | 세장형(90° flip 추가) 종횡비 임계 |
| `MMS_SIM_FLIP_EL_MIN` / `_MAX` | 30 / 70 | flip 관측 고도각 하한·상한(°) |
| `MMS_SIM_FLIP_ANGLES` | 자동 | flip 각 명시 (설정 시 정책 무시) |
| `MMS_SIM_ICP_INFLATION` | 0.005 | 정합이 물체를 부풀리는 한계 m (평행이동 오류 차단) |
| `MMS_REAL_INFLATION_GATE` | 1 | real 팽창 게이트 (0=끔, 실기 검증 전) |

---

## 4. 자주 겪는 것

| 증상 | 원인·조치 |
|---|---|
| `ModuleNotFoundError: pxr` | `env_isaacsim` 에서 pxr 은 SimulationApp **초기화 후**에만 import 된다 |
| 렌더가 새까맣다 | 씬에 라이트가 없다. `build_scene_v3.py` 가 넣어주지만, `v3.usd` 만 열면 없다 |
| Play 누르면 로봇이 튄다 | `root_joint` 앵커가 옛 좌표. `build_scene_v3.py` 가 갱신함 |
| 로봇이 떨린다 / 끊긴다 | 구동 3단·게인 문제 → `$ASSET/v3_scene_IK.md` 함정 1 |
| 씬을 열었는데 비어 있다 | `v3.usd` 가 자기 자신을 subLayer 로 물었을 수 있다(Isaac Save 사고). `v3.usd` 를 재생성 |
| `turntable prim not found` | `isaac_world.py` 프림 상수가 씬과 안 맞음 |

---

## 관련 문서

- `docs/v3_sim_migration.md` — v2→v3 이관 현황, 남은 blocker
- `docs/testset_results.md` — 9종 스윕 결과(GT 지표·결함 7건·렌더)
- `$ASSET/v3_scene_IK.md` — 드래그 IK 타깃 방법론(다른 로봇 재사용용)
- `docs/main_flow.md` — 전체 파이프라인
