# sim 씬 — 생성 · 선택 · 설정

> Isaac 씬 USD 를 **무엇을 쓰고, 어떻게 만들며, 배치 변경 시 무엇을 수정하는가**.
> 씬을 수정하지 않고 실행만 하려면 `sim_commands.md` 로 충분하다.
>
> **원칙 — 씬 USD 와 충돌 npz 는 한 몸이다.** 둘이 어긋나면 로봇이 셀 내부에 박힌
> 것으로 판정되어 **모든 자세가 거부**된다(2026-09-17: sim home 여유 0.0mm 로
> lookaround 가 한 점도 취득하지 못함).

---

## 1. 현재 사용 씬

기본 씬은 **`frame_xarm7_spider_turntable/v2_real_260917.usd`** 이며
(`isaac_world.py::DEFAULT_USD_PATH`, 2026-09-17~) 실물 셀과 동일한 배치다.

> ⚠ 셀을 v3 로 변경하기로 하였으나 실제로는 변경하지 않았다(2026-09-16 현장 확인).
> v3 는 턴테이블이 로봇 base 바로 아래(수평 0mm)이나 실물은 799mm 이격되어 있어
> **자세 선정·도달성·이동량이 전혀 다르다.**

| 씬 | 용도 | 생성 스크립트 | 충돌 레이아웃 |
|---|---|---|---|
| **`…/v2_real_260917.usd`** ★기본 | 실물과 동일 배치 = 평상시 | `build_scene_v2_real.py` | `v2_real_260917` |
| `…_v2/v3_scene.usd` | v3 레이아웃 실험(미시공 계획) | `build_scene_v3.py` | `v3_layout_sim` |
| `…_v2/v3_ts_<이름>.usd` | 물체별 v3 씬(testset 스윕) | 〃 `--out` / `--object` | `v3_layout_sim` |
| `…/overlay_ACTIVE.usd` · `overlay_MEASURED.usd` | 충돌 점군 육안 확인 | `overlay_env_npz.py` | — |

| | v3_scene | **v2_real_260917** (실물) |
|---|---|---|
| 턴테이블 축 (base) | `[0, 0, 0.835]` — 수평 0 mm | `[0.799, 0.005, 0.688]` — 수평 799 mm |
| 도달여유 | +255 mm | +35 mm |
| sim `T_EC` vs 실측 hand-eye | 104 mm 차이 | 0.000 mm |
| home | `MMS_SIM_HOME_DEG` 로 별도 지정 | 실물과 동일 |

씬은 `MMS_SIM_USD` 로 변경한다(물체 씬은 `MMS_SIM_OBJECT_PRIM` 도 함께). 충돌
레이아웃은 씬 경로에서 자동 선택되며(`isaac_world.py::SCENE_COLLISION_LAYOUT`),
`MMS_COLLISION_LAYOUT` 을 지정하면 그것이 우선한다.

⚠ 구 `v2.usd` 및 `testset/composed/*_on_turntable.usd` 는 카메라·턴테이블 prim 경로가
달라 스캔 없이 30초 만에 종료된다. 이를 생성하는 `place_testset_object.py` 도 사용
중단이다.

---

## 2. 새 물체 추가

물체 USD 를 `2_데이터/testset/` 에 넣고 씬을 **1회** 생성한다.

```bash
U2=~/miniconda3/envs/step2usd/bin/python
source scripts/sim/_paths.sh

env -u PYTHONPATH $U2 scripts/sim/build_scene_v2_real.py \
    --object "$(mms_testset_dir)/<이름>.usd"                      # 실물 배치(기본)

env -u PYTHONPATH $U2 scripts/sim/build_scene_v3.py --dir "$(mms_v3_dir)" \
    --out v3_ts_<이름>.usd --object "$(mms_testset_dir)/<이름>.usd"  # v3 배치
```

대상물 참조는 씬 파일 기준 **상대경로**로 기록되므로, 폴더 구조
(`2_3Dassets/…` ↔ `testset/`)만 유지하면 다른 머신에서도 열린다.

---

## 3. 셀 배치가 변경된 경우 — 6단계

로봇 연산(충돌·도달성)이 사용하는 것은 **씬 USD** 와 **충돌 점군 npz** 두 가지다.
현장에서 부재를 이동하였거나 캘리브를 재수행하였으면 순서대로 진행한다.
**3번을 생략하면 둘이 어긋난다.**

| # | 작업 | 대상 |
|---|---|---|
| 1 | **실측 재캘리브** — 턴테이블 이동 시 `T_B_F0`, 스캐너 재장착 시 hand-eye | `scripts/artec/calibrate.py` → `config/calibration/turntable_frame.yaml` · `config/sensor_frames.yaml` |
| 2 | **씬 USD 재생성** — 부재 탈거·이동 시 스크립트 상수부터 수정 | `build_scene_v2_real.py` 의 `BASE_POS` · `DEAD_PRIMS` · `FIELD_FIX_LOCAL_X` |
| 3 | **충돌 npz 재베이크** — 반드시 2에서 생성한 씬에서 굽는다 | `scripts/sim/export_env_mesh.py` |
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

# 3) 충돌 npz  (--root 를 두 개 준다: 구조물 + 턴테이블)
env -u PYTHONPATH $ISAAC scripts/sim/export_env_mesh.py --scene "<위 .usd>" \
    --root /World/Frame/frame_structure /World/frame --robot /World/xarm7 \
    --out utils/collision/data/layouts/cell_env.$ALIAS.npz

# 4) 활성화 (real·sim 이 같은 활성본을 사용한다)
env -u PYTHONPATH python scripts/collision/use_layout.py $ALIAS

# 6) 검증 — 장비 없이 실물 상수로
env -u PYTHONPATH python scripts/artec/validate_real_cell.py
env -u PYTHONPATH $STEP2USD scripts/sim/overlay_env_npz.py --write
```

### 주의사항 3가지

1. **npz 는 씬에서 굽는 것이지 측정값이 아니다.** 실물을 실제로 측정한 값은 캘리브
   yaml 뿐이다(`T_B_F0` 의 `rim_radius_mm` 등). npz 는 그 위치에 CAD 를 놓고 표면을
   샘플링한 결과이므로, "실측본" 으로 불려도 출처는 편집된 USD 다.
2. **활성 npz 는 real 도 사용한다.** `use_layout.py` 로 교체하면 다음 **실물** 실행의
   충돌 판정이 바뀐다. 실행 중인 프로세스에는 반영되지 않으므로
   (`collision_model.get_default` 가 캐시) Isaac 을 재기동해야 한다.
3. **굽기는 복셀 다운샘플**(`--voxel`, 기본 2mm)**이며 균일 stride 로 솎지 않는다.**
   `E[::15]` 방식은 큰 벽과 작은 부재를 같은 비율로 제거하여 **작은 부재가 소실**된다 —
   실측에서 턴테이블 원판 상면이 2,532 → 163점이 되어 충돌 게이트가 원판을 거의 인식하지
   못하였다(2026-09-17). 2mm 복셀 적용 시 유사한 총점 수에서 1,395점이 확보되었다.

변경 후에는 **오버레이로 육안 확인한다.** `overlay_ACTIVE.usd`(녹색, 활성 npz)는 씬
메시와 완전히 일치해야 하고, `overlay_MEASURED.usd`(주황, 현장 실측본)는 **턴테이블만
어긋나는 것이 정상**이다 — 실측본은 v2 placeholder 를 1.6배 확대한 원기둥이고 현 씬은
v3 CAD 조립체로 형상 자체가 다르다(중앙값 22mm).

---

## 4. CAD 에서 씬을 새로 생성할 때 (v3 계열)

```bash
U2=~/miniconda3/envs/step2usd/bin/python
ASSET=$(source scripts/sim/_paths.sh && mms_asset_root)/frame_xarm7_spider_turntable_v2

# STEP → 형상 USD (CAD 변경 시에만, 약 47초)
env -u PYTHONPATH $U2 scripts/sim/step2usd.py \
    --step "$(source scripts/sim/_paths.sh && mms_asset_root)/frame_v2/260811_frame.STEP" \
    --out $ASSET --name v3          # --deflection 0.5 --keep-fasteners --source-up y|z

# 형상 → sim 씬
env -u PYTHONPATH $U2 scripts/sim/build_scene_v3.py --dir $ASSET
```

`build_scene_v3.py` 주요 옵션 — `--object <testset.usd>|none`(기본 0146_mug),
`--scale-drop 0.115`(저울 하강량 m), `--scanner-clock-deg 180`(어댑터+스캐너 툴축 회전),
`--cam-t X Y Z`(스캐너 프레임 로컬 카메라 위치 m),
`--collider-approx convexHull|boundingCube`, `--no-physics` / `--no-lights` / `--no-ground`.

> `v3.usd` 와 `parts/*.usd` 는 읽기 전용(chmod 444)이다. 재생성 전 `chmod 644` 할 것.
>
> **두 빌더 모두 원본을 subLayer 로 깔고 오버라이드만 얹는 비파괴 방식이다.** 따라서
> **GUI 에서 수동 수정한 내용은 재생성 시 소실된다** — 값을 추출하여 스크립트 상수
> (`FIELD_FIX_LOCAL_X` 등)에 반영해야 한다.

---

## 5. 카메라 FOV

스캐너 화각은 **런타임이 `isaac_world.py::SPIDER_HFOV_DEG` 로 덮어쓴다.** USD 에
authored 된 값은 GUI 로 씬을 열 때만 반영되므로, 실측과 다르면 **육안 화각과 플래너
계산 화각이 불일치**한다. 다음으로 맞춘다.

```bash
env -u PYTHONPATH $U2 scripts/sim/set_camera_fov.py [--dry-run]
```

실측 K(960×1280 세로형) 기준 가로 **21.58°** / 세로 **28.58°** 다. 세로 화각이 한 자세가
덮는 높이를 결정하므로 밴드 분할(`3_lookaround.md` §5)의 입력이 된다.

---

## 6. 자산(USD) 경로 해석

절대경로를 소스에 기재하지 않는다. `mms_paths.py`(python) / `scripts/sim/_paths.sh`(셸)이
아래 순서로 **자산 루트(`2_3Dassets`)** 를 탐색한다.

| 순위 | 위치 |
|---|---|
| 1 | 환경변수 `MMS_ASSET_ROOT` |
| 2 | `<repo>/../../2_데이터/2_3Dassets` (인수인계 폴더 배치) |
| 3 | `<repo>/../2_3Dassets` (원본 개발 배치) |
| 4 | `<repo>/2_3Dassets` |
| 5 | 구 개발머신 절대경로 (하위호환) |

```bash
python3 mms_paths.py                              # 해석 결과 출력
source scripts/sim/_paths.sh && mms_asset_root    # 셸

export MMS_ASSET_ROOT=/경로/2_3Dassets
export MMS_TESTSET_DIR=/경로/testset              # 대상물 USD 트리
export MMS_PYTHON=/경로/python                    # Isaac 파이썬
```

---

## 7. 씬 변경 시 함께 점검할 항목

| 항목 | 사유 |
|---|---|
| **home 관절각** | 배치 변경 시 home 이 셀 내부에 박힐 수 있다. `find_home_pose.py`(충돌 게이트 포함)로 재산출 후 `isaac_xarm.py::HOME_JOINTS_DEG` 에 반영 |
| **IK 씬** | `ik_follow_target.py --rebuild` 로 재생성 |
| **카메라 FOV** | §5 — 새 씬은 authored 값이 CAD 기본값이다 |

---

## 8. 문제 해결

| 증상 | 원인 · 조치 |
|---|---|
| 모든 자세가 `이동 거부 — start(link6)` | 씬과 충돌 레이아웃 불일치 → §3 의 3~5 |
| IK 는 통과하나 캡처가 0점 | 카메라가 다른 곳을 본다. `prim_world_pose(CAMERA_PRIM)` 를 의도한 eye 와 대조 |
| 렌더가 검게 나온다 | 씬에 라이트가 없다. 빌더는 추가하나 `v3.usd` 만 열면 없다 |
| Play 시 로봇이 튄다 | `root_joint` 앵커가 구 좌표. 빌더가 갱신한다 |
| 씬이 비어 있다 | `v3.usd` 가 자기 자신을 subLayer 로 참조(Isaac Save 사고) → 재생성 |
| `turntable prim not found` | `isaac_world.py` 프림 상수가 씬과 불일치 |
| `ModuleNotFoundError: pxr` | `env_isaacsim` 에서 pxr 은 SimulationApp **초기화 후**에만 import 된다 |
| `씬이 없다: …v2_real_260917.usd` | 자산이 구버전이다. 릴리스 `assets-v2` 로 다시 받는다(`../README.md`) |

---

## 관련 문서

- `sim_commands.md` — 실행 커맨드 · 진단 도구 · 스윕
- `collision.md` — 충돌 모델(SDF · 마진 · 현장 패치)
- `3_lookaround.md` · `4_nbv.md` — 스캔 알고리즘
- `install.md` — env 설치 · 자산 내려받기
- `$ASSET/v3_scene_IK.md` — 드래그 IK 타깃 방법론
