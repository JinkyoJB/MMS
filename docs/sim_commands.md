# sim 커맨드 모음

> 최초 설치·환경 구성은 **`../README.md` §환경**. 실물 장비는 **`7_real_commands.md`**.
> 씬 생성·셀 배치 변경은 **`sim_scene.md`**.

모든 명령은 **리포 루트**에서 실행한다.

## 공통 규칙 2가지

**1. `env -u PYTHONPATH` 를 반드시 붙인다.** 셸에 ROS Humble 의 python3.10 경로가 잡혀
있어 3.11 env 에 혼입된다. 누락 시 원인 파악이 어려운 import 오류가 발생한다.

**2. env 를 용도별로 구분한다.**

| env | 용도 | 경로 |
|---|---|---|
| `env_isaacsim` | Isaac Sim 이 필요한 작업(씬 렌더·물리·main_artec sim) | `~/miniconda3/envs/env_isaacsim/bin/python` |
| `step2usd` | USD 생성·편집 전용(pythonocc + usd-core, Isaac 불요) | `~/miniconda3/envs/step2usd/bin/python` |
| `mms-env` | real 백엔드 + 오프라인 스크립트 | `~/miniconda3/envs/mms-env/bin/python` |

이하 `$ISAAC`, `$U2` 로 표기한다.

```bash
ISAAC=~/miniconda3/envs/env_isaacsim/bin/python
U2=~/miniconda3/envs/step2usd/bin/python
ASSET=$(source scripts/sim/_paths.sh && mms_asset_root)/frame_xarm7_spider_turntable_v2
```

---

## 1. 파이프라인 실행

```bash
env -u PYTHONPATH MMS_BACKEND=isaac PYTHONUNBUFFERED=1 $ISAAC -u main_artec.py --no-prompt
env -u PYTHONPATH MMS_BACKEND=isaac MMS_ISAAC_HEADLESS=1 MMS_SIM_NO_VIZ=1 \
    PYTHONUNBUFFERED=1 $ISAAC -u main_artec.py --no-prompt        # 헤드리스
```

`MMS_BACKEND` 는 `main_artec.py::BACKEND` 소스 스위치를 덮어쓰므로, 소스 수정 없이
sim/real 을 전환할 수 있다.

실행 범위는 `MULTIPASS_SETTINGS.stage_until` 을 따른다(`preview` / `lookaround` /
`nbv` / `flip`, 앞 단계는 항상 포함).

| 환경변수 | 내용 |
|---|---|
| `MMS_SIM_STAGE_UNTIL` | 실행 단계 override. `preview` 는 계획까지만(전회전 생략) |
| `MMS_SIM_NTHETA` | 회전 프레임 수(기본 240) — 빠른 확인용 |
| `MMS_SIM_DRIVE_STEPS` | 구동 스텝 수 |
| `MMS_SIM_USD` / `MMS_SIM_OBJECT_PRIM` | 씬 / 대상물 prim override |

### 기본 씬과 v3

**기본 씬은 실물 배치 `v2_real_260917.usd` 다**(2026-09-17~). v3 는 셀을 변경하기로
하였다가 실제로는 변경하지 않은 배치이므로, 자세 선정·도달성·이동량이 실물과 다르다.

| | v3_scene | **v2_real_260917** (기본) |
|---|---|---|
| 턴테이블 축 (base) | `[0, 0, 0.835]` — 수평 0 mm | `[0.799, 0.005, 0.688]` — 수평 799 mm |
| 도달여유 | +255 mm | +35 mm |
| sim `T_EC` vs 실측 hand-eye | 104 mm 차이 | 0.000 mm |

v3 로 확인해야 하는 경우 씬·레이아웃·home 을 함께 지정한다. 충돌 레이아웃은 씬 경로에서
`v3_layout_sim` 이 자동 선택되고, home 은 `MMS_SIM_HOME_DEG` 로 덮어쓴다.

```bash
env -u PYTHONPATH MMS_BACKEND=isaac \
    MMS_SIM_USD="$ASSET/v3_scene.usd" $ISAAC -u main_artec.py --no-prompt
```

### 물체·단계 지정 스윕

```bash
./scripts/sim/run_e2e_gui.sh spray_can nbv    # [물체] [stage_until]
```

물체 이름은 부분 일치를 허용한다(`mug` `drill` `detergent` 등).
⚠ 본 스크립트와 아래 스윕은 **v3 씬**을 사용한다.

```bash
source scripts/sim/_paths.sh
mms_v3_scene / mms_v3_ts spray_can / mms_testset_dir   # 경로 헬퍼
scripts/sim/e2e_sweep.sh                  # 9종 lookaround 만
scripts/sim/testset_sweep.sh [0146_mug]   # 9종 전체 (물체당 약 5분, summary.tsv)
```

### GT 평가·렌더

```bash
env -u PYTHONPATH $U2 scripts/sim/extract_gt_mesh.py --all        # 씬→정답 표면 (1회)
python scripts/sim/eval_vs_gt.py --scan <obj> --gt scripts/sim/log/gt/<이름>.npz
python scripts/sim/render_scan_results.py --logs scripts/sim/log/testset_sweep
python scripts/sim/build_results_page.py                          # testset_results.md 웹판
```

---

## 2. 확인·진단

### IK 도달성 (드래그 타깃)

GUI 에서 `/World/IKTarget` 을 기즈모로 이동시키면 로봇이 추종한다(녹색=도달, 적색=실패).

```bash
env -u PYTHONPATH $ISAAC scripts/sim/ik_follow_target.py                  # 스캐너 카메라 기준
env -u PYTHONPATH $ISAAC scripts/sim/ik_follow_target.py --target flange  # link7 플랜지
env -u PYTHONPATH $ISAAC scripts/sim/ik_follow_target.py --target adapter # 어댑터 중심
```

prim 경로를 그대로 넘기면 해당 prim 의 메시 중심이 기준점이 된다.

| 옵션 | 설명 |
|---|---|
| `--rebuild` | IK 씬 재생성 (씬을 다시 만든 뒤 필수) |
| `--selftest` | GUI 없이 격자 도달성만 출력 |
| `--hide-spider` | Spider 비활성 (adapter 모드는 자동) |
| `--max-jump 90` | 1스텝 관절 급변 허용(deg). 드래그가 자주 멈추면 상향 |
| `--pos-tol` / `--rot-tol` | 허용 위치·자세 오차 (기본 0.005 / 5.0) |

씬 파일은 기준점별로 생성된다(`v3_scene_IK.usd`, `v3_scene_IK_adapter.usd` 등).
방법론·주의사항은 `$ASSET/v3_scene_IK.md`.

### 캘리브 검증

```bash
env -u PYTHONPATH $ISAAC -u scripts/sim/calib_handeye_sim.py    # hand-eye, 약 90초
env -u PYTHONPATH $ISAAC scripts/sim/calib_rim_sim.py           # 턴테이블 축 캡처
env -u PYTHONPATH python scripts/sim/rim_click_offline.py       # mms-env, 클릭+피팅
MMS_RIM_AUTO=1 env -u PYTHONPATH $ISAAC scripts/sim/calib_rim_sim.py   # 자동
```

hand-eye 는 GT 대비 `t_err`/`r_err` 를 출력하며 기준은 5mm/2° 다. 상세는
`1_calibration.md` §4·§8.

### 턴테이블 회전

```bash
env -u PYTHONPATH $ISAAC scripts/sim/spin_turntable.py --gui
env -u PYTHONPATH $ISAAC scripts/sim/spin_turntable.py --angle 90 --no-loop
```

`--step 1`(스텝당 deg), `--rider <prim>`(함께 회전시킬 prim).
GUI 에서 수동 회전 시에는 `/World/frame/turntable_disc` 의 **Rotate Y** 를 사용한다
(`/World/frame` 에 rotateX(90°) 가 적용되어 부모 Y 가 월드 수직축이다).

### home 관절각 재산출

배치 변경 시 home 이 맞지 않는다. USD 실측 기반 IK 탐색으로 재산출한 뒤
`mms_artec/backends/isaac/isaac_xarm.py::HOME_JOINTS_DEG["artec"]` 에 반영한다.

```bash
env -u PYTHONPATH $U2 scripts/sim/find_home_pose.py [--dist 0.28] [--el 45 55 65 75]
```

---

## 3. 진단·튜닝 환경변수 (`isaac_scan_session`)

| env | 기본 | 용도 |
|---|---|---|
| `MMS_SIM_PROFILE_EVERY` | 20 | N프레임마다 단계별 소요시간 (0=해제) |
| `MMS_SIM_ICP_DEBUG` | 0 | ICP 스케일별 fitness/drift 출력 |
| `MMS_SIM_STAGE_DUMP` | — | nbv 반복마다 메시+누적점군 덤프 디렉터리 |
| `MMS_NBV_CONV_NEW_EPS` / `_STALL_N` | 0.005 / 3 | 전역 수렴 백스톱 (sim·real 공용) |
| `MMS_NBV_DRY_EPS` | 0.015 | gap 패치 생산성 판정 (sim·real 공용) |
| `MMS_SIM_FLIP_ASPECT` | 2.0 | 세장형(90° flip 추가) 종횡비 임계 |
| `MMS_SIM_FLIP_EL_MIN` / `_MAX` | 30 / 70 | flip 관측 고도각 하한·상한(°) |
| `MMS_SIM_FLIP_ANGLES` | 자동 | flip 각 명시 (설정 시 정책 무시) |
| `MMS_SIM_ICP_INFLATION` | 0.005 | 정합의 물체 팽창 한계(m) — 평행이동 오류 차단 |
| `MMS_REAL_INFLATION_GATE` | 1 | real 팽창 게이트 (0=해제, 실기 검증 전) |

환경변수는 **실행 중인 프로세스에 반영되지 않는다.** 변경 후 재실행한다.

---

## 4. 관련 문서

- `sim_scene.md` — 씬 생성·자산 경로(`MMS_ASSET_ROOT`)·씬 관련 증상 전반
- `testset_results.md` — 9종 스윕 결과(GT 지표·결함·렌더)
- `$ASSET/v3_scene_IK.md` — 드래그 IK 타깃 방법론
- `../README.md` — 설치와 파이프라인 개요
