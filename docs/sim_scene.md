# sim 씬 — 생성 · 선택 · 설정

> Isaac 씬 USD 를 **어떤 걸 쓰고, 어떻게 만들고, 배치가 바뀌면 뭘 고치는가**.
> 씬을 안 건드리고 그냥 돌리기만 할 거면 `sim_commands.md` §0 으로 충분하다.
>
> **핵심 한 줄 — 씬 USD 와 충돌 npz 는 한 몸이다.** 둘이 어긋나면 로봇이 셀 안에
> 박힌 것으로 판정돼 **모든 자세가 거부**된다 (2026-09-17: sim home 여유 0.0mm →
> lookaround 이 한 점도 못 얻음).

---

## 1. 지금 쓰는 씬

기본 씬은 **`frame_xarm7_spider_turntable/v2_real_260917.usd`** 다
(`isaac_world.py::DEFAULT_USD_PATH`, 2026-09-17~). 실물 셀과 같은 배치다.

> ⚠ **"씬은 v3 만 쓴다" 는 더 이상 맞지 않다.** 셀을 v3 로 바꾸기로 했다가 실제로는
> 안 바꿨다(2026-09-16 현장 확인). v3 는 턴테이블이 로봇 base 바로 아래(수평 0mm)인데
> 실물은 799mm 떨어져 있어 **자세 선정·도달성·이동량이 전혀 다르다.**

| 씬 | 언제 쓰나 | 만드는 스크립트 | 충돌 레이아웃 |
|---|---|---|---|
| **`frame_xarm7_spider_turntable/v2_real_260917.usd`** ★기본 | 실물과 같은 배치 = 평소 | `scripts/sim/build_scene_v2_real.py` | `v2_real_260917` |
| `frame_xarm7_spider_turntable_v2/v3_scene.usd` | v3 레이아웃 실험 (안 지어진 계획) | `scripts/sim/build_scene_v3.py` | `v3_layout_sim` |
| `…_v2/v3_ts_<이름>.usd` | 물체별 v3 씬 (testset 스윕) | 〃 `--out` / `--object` | `v3_layout_sim` |
| `…/overlay_ACTIVE.usd` · `overlay_MEASURED.usd` | 씬 위에 충돌 점군을 얹어 눈으로 확인 | `scripts/sim/overlay_env_npz.py` | — |

| | v3_scene | **v2_real_260917** (실물) |
|---|---|---|
| 턴테이블 축 (base) | `[0, 0, 0.835]` 수평 **0 mm** | `[0.799, 0.005, 0.688]` 수평 **799 mm** |
| 도달여유 | +255 mm | **+35 mm** |
| sim T_EC vs 실측 hand-eye | 104 mm 차이 | **0.000 mm** |
| home | `MMS_SIM_HOME_DEG` 로 따로 | **실물과 동일** |

씬은 `MMS_SIM_USD` 로 바꾼다(물체 씬은 `MMS_SIM_OBJECT_PRIM` 도 같이). 충돌 레이아웃은
**씬 경로에서 자동으로** 고른다(`isaac_world.py::SCENE_COLLISION_LAYOUT`) —
`MMS_COLLISION_LAYOUT` 을 직접 주면 그게 우선이다.

> ⚠ **새 머신에서는 `v2_real_260917.usd` 가 없다.** 자산 릴리스에 안 들어 있고
> `v2.usd` 에서 생성하는 파일이다. §3 의 2번을 한 번 돌려야 기본 씬이 열린다.

⚠ 구 `v2.usd` / `testset/composed/*_on_turntable.usd` 는 카메라·턴테이블 prim 경로가
달라 스캔 없이 30초 만에 끝난다(2026-08-19 실측). 그 씬을 만드는
`place_testset_object.py` 도 사용 중단이다.

---

## 2. 새 물체 추가

물체 USD 를 `2_데이터/testset/` 에 넣고 씬을 **한 번** 만든다.

```bash
U2=~/miniconda3/envs/step2usd/bin/python
source scripts/sim/_paths.sh

# 실물 배치 (기본)
env -u PYTHONPATH $U2 scripts/sim/build_scene_v2_real.py \
    --object "$(mms_testset_dir)/<이름>.usd"

# v3 배치 (testset 스윕용)
env -u PYTHONPATH $U2 scripts/sim/build_scene_v3.py --dir "$(mms_v3_dir)" \
    --out v3_ts_<이름>.usd --object "$(mms_testset_dir)/<이름>.usd"
```

> 대상물 참조는 씬 파일 기준 **상대경로**로 들어간다(2026-09-09). 폴더 구조
> (`2_3Dassets/…` ↔ `testset/`)만 유지하면 다른 머신에서도 열린다.

---

## 3. 셀 배치가 바뀌면 — 6단계

로봇 연산(충돌·도달성)이 쓰는 것은 **씬 USD** 와 **충돌 점군 npz** 둘이다. 현장에서
부재를 옮겼거나 캘리브를 다시 했으면 순서대로 돈다. **3번을 건너뛰면 둘이 어긋난다.**

| # | 하는 일 | 건드리는 곳 |
|---|---|---|
| 1 | **실측 재캘리브** — 턴테이블을 옮겼으면 `T_B_F0`, 스캐너를 재장착했으면 hand-eye | `scripts/artec/calibrate.py` → `config/calibration/turntable_frame.yaml` · `config/sensor_frames.yaml` |
| 2 | **씬 USD 재생성** — 부재를 뗐/옮겼으면 스크립트 상수부터 고친다 | `scripts/sim/build_scene_v2_real.py` 의 `BASE_POS` · `DEAD_PRIMS` · `FIELD_FIX_LOCAL_X` |
| 3 | **충돌 npz 재베이크** — 반드시 **2에서 만든 씬**에서 굽는다 | `scripts/sim/export_env_mesh.py` |
| 4 | **활성 레이아웃 교체** | `scripts/collision/use_layout.py <별칭>` |
| 5 | **기본 씬 지정** | `isaac_world.py` 의 `DEFAULT_USD_PATH` · `SCENE_COLLISION_LAYOUT` |
| 6 | **검증** | `scripts/artec/validate_real_cell.py` · `scripts/sim/overlay_env_npz.py` |

```bash
STEP2USD=~/miniconda3/envs/step2usd/bin/python
ISAAC=~/miniconda3/envs/env_isaacsim/bin/python
ALIAS=v2_real_260918                     # 날짜 등으로 새 별칭

# 2) 씬
env -u PYTHONPATH $STEP2USD scripts/sim/build_scene_v2_real.py \
    --out "$(python3 -c 'import mms_paths;print(mms_paths.asset_root())')/frame_xarm7_spider_turntable/$ALIAS.usd" \
    --object "<testset>/0101_spray_can.usd"

# 3) 충돌 npz  (--root 를 **두 개** 준다: 구조물 + 턴테이블)
env -u PYTHONPATH $ISAAC scripts/sim/export_env_mesh.py --scene "<위 .usd>" \
    --root /World/Frame/frame_structure /World/frame --robot /World/xarm7 \
    --out utils/collision/data/layouts/cell_env.$ALIAS.npz

# 4) 활성화 (real·sim 이 같은 활성본을 쓴다)
env -u PYTHONPATH python scripts/collision/use_layout.py $ALIAS

# 6) 검증 — 장비 없이 실물 상수로
env -u PYTHONPATH python scripts/artec/validate_real_cell.py
env -u PYTHONPATH $STEP2USD scripts/sim/overlay_env_npz.py --write
```

### 함정 3가지

1. **npz 는 씬에서 굽는 것이지 측정값이 아니다.** 실물을 실제로 잰 값은 캘리브 yaml
   뿐이다(`T_B_F0` 의 `rim_radius_mm` 등). npz 는 그 위치에 CAD 를 놓고 표면을
   샘플링한 결과다 — "실측본"이라 불려도 출처는 누군가 편집한 USD 다.
2. **활성 npz 는 real 도 쓴다.** `use_layout.py` 로 바꾸면 다음 **실물** 실행의 충돌
   판정이 바뀐다. 그리고 **실행 중인 프로세스엔 반영 안 된다**
   (`collision_model.get_default` 가 캐시한다) — Isaac 창을 다시 띄워야 한다.
3. **굽기는 복셀 다운샘플**(`--voxel`, 기본 2mm)**이다. 균일 stride 로 솎지 말 것.**
   `E[::15]` 식은 큰 벽이든 작은 부재든 같은 비율로 버려서 **작은 부재가 굶는다** —
   실측으로 턴테이블 원판 상면이 2,532 → **163점**이 돼 충돌 게이트가 원판을 거의
   못 봤다(2026-09-17). 2mm 복셀로 바꾸니 비슷한 총점 수에서 1,395점이 됐다.

> 바꾼 뒤에는 **오버레이로 눈으로 한 번 본다.** `overlay_ACTIVE.usd`(초록, 활성 npz)는
> 씬 메시와 완전히 포개져야 정상이고, `overlay_MEASURED.usd`(주황, 현장 실측본)는
> **턴테이블만 어긋나는 게 정상**이다 — 실측본 쪽은 v2 placeholder 를 1.6배 늘린
> 통원기둥이고 우리 씬은 v3 CAD 조립체라 형상 자체가 다른 물건이다(중앙값 22mm).

---

## 4. CAD 에서 씬을 새로 만들 때 (v3 계열)

```bash
U2=~/miniconda3/envs/step2usd/bin/python
ASSET=$(source scripts/sim/_paths.sh && mms_asset_root)/frame_xarm7_spider_turntable_v2
```

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
```bash
env -u PYTHONPATH $U2 scripts/sim/build_scene_v3.py --dir $ASSET
```
```
--object <testset.usd>|none   스캔 대상 (기본 0146_mug)
--scale-drop 0.115            저울 하강량 m
--scanner-clock-deg 180       어댑터+스캐너 툴축 회전
--cam-t X Y Z                 Camera 위치(스캐너 프레임 로컬, m)
--collider-approx convexHull|boundingCube
--no-physics / --no-lights / --no-ground
```

> **두 빌더 모두 원본을 subLayer 로 깔고 오버라이드만 얹는다**(비파괴). 그래서
> **GUI 에서 손으로 고친 것은 재생성 때 날아간다** — 값을 뽑아 스크립트 상수
> (`FIELD_FIX_LOCAL_X` 등)에 박아야 살아남는다.

---

## 5. 카메라 FOV

스캐너 화각은 **런타임이 `isaac_world.py::SPIDER_HFOV_DEG` 로 덮어쓴다.** USD 에
authored 된 값은 GUI 로 씬을 열어볼 때만 보이는데, 그게 실측과 다르면 **눈으로 보는
화각과 플래너가 계산하는 화각이 갈린다.** 맞추려면:

```bash
env -u PYTHONPATH $U2 scripts/sim/set_camera_fov.py            # 모든 씬
env -u PYTHONPATH $U2 scripts/sim/set_camera_fov.py --dry-run  # 확인만
```
실측 K(960×1280 세로형) 기준 가로 **21.58°** / 세로 **28.58°** 다. 세로가 한 자세가
덮는 **높이**를 정하므로 밴드 분할(`3_lookaround.md` §5)의 입력이 된다.

---

## 6. 자산(USD) 경로 — 자동 해석

절대경로를 소스에 박지 않는다. `mms_paths.py`(python) / `scripts/sim/_paths.sh`(셸)이
아래 순서로 **자산 루트(`2_3Dassets`)** 를 찾는다.

| 순위 | 위치 |
|---|---|
| 1 | 환경변수 `MMS_ASSET_ROOT` |
| 2 | `<repo>/../../2_데이터/2_3Dassets` (인수인계 폴더 배치) |
| 3 | `<repo>/../2_3Dassets` (원본 개발 배치) |
| 4 | `<repo>/2_3Dassets` |
| 5 | 구 개발머신 절대경로 (하위호환) |

```bash
python3 mms_paths.py                                    # 해석 결과 출력
source scripts/sim/_paths.sh && mms_asset_root          # 셸 쪽
```

다른 곳에 두었다면:
```bash
export MMS_ASSET_ROOT=/경로/2_3Dassets
export MMS_TESTSET_DIR=/경로/testset      # 대상물 USD 트리
export MMS_PYTHON=/경로/python            # Isaac 파이썬
```

---

## 7. 씬이 바뀌면 같이 봐야 하는 것

| 무엇 | 왜 |
|---|---|
| **home 관절각** | 배치가 바뀌면 home 이 셀 안에 박힐 수 있다. `scripts/sim/find_home_pose.py` (충돌 게이트 포함) 로 재산출 → `isaac_xarm.py::HOME_JOINTS_DEG` |
| **IK 씬** | `scripts/sim/ik_follow_target.py --rebuild` 로 다시 만든다 |
| **GUI 시작 시점** | 축 좌표가 하드코딩이면 빈 공간을 본다 (지금은 디스크에서 유도) |
| **카메라 FOV** | §5 — 새로 만든 씬은 authored 값이 CAD 기본값이다 |

---

## 8. 자주 겪는 것

| 증상 | 원인 · 조치 |
|---|---|
| 모든 자세가 `이동 거부 — start(link6)` | 씬과 충돌 레이아웃이 어긋났다 → §3 의 3~5 |
| IK 는 통과하는데 캡처가 0점 | 카메라가 딴 데를 본다. `prim_world_pose(CAMERA_PRIM)` 를 의도한 eye 와 대조 |
| 렌더가 새까맣다 | 씬에 라이트가 없다. 빌더가 넣어주지만 `v3.usd` 만 열면 없다 |
| Play 누르면 로봇이 튄다 | `root_joint` 앵커가 옛 좌표. 빌더가 갱신한다 |
| 씬을 열었는데 비어 있다 | `v3.usd` 가 자기 자신을 subLayer 로 물었을 수 있다(Isaac Save 사고) → 재생성 |
| `turntable prim not found` | `isaac_world.py` 프림 상수가 씬과 안 맞는다 |
| `ModuleNotFoundError: pxr` | `env_isaacsim` 에서 pxr 은 SimulationApp **초기화 후**에만 import 된다 |
| `씬이 없다: …v2_real_260917.usd` | 릴리스에 없는 생성물이다 → §3 의 2번 |

---

## 관련 문서

- `sim_commands.md` — 그냥 돌리는 커맨드 · 진단 도구 · 스윕
- `collision.md` — 충돌 모델 자체(SDF · 마진 · 현장 패치)
- `3_lookaround.md` · `4_nbv.md` — 스캔 알고리즘
- `install.md` — env 설치 · 자산 내려받기
- `$ASSET/v3_scene_IK.md` — 드래그 IK 타깃 방법론
