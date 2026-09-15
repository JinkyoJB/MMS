# MMS — Multi Modal 3D Scanning System

> Artec Spider + xArm7 + 턴테이블 자동 3D 스캐닝 시스템.
> **최종 산출물 = 대상물 전면(full-coverage)의 watertight mesh + texture.**

**이 문서의 역할** — 알고리즘 설계와 그 근거를 다룬다. 각 Phase가 왜 그렇게 판정하는지,
어떤 함정이 있는지가 여기 있다.

| 목적 | 볼 문서 |
|---|---|
| 설치하고 돌려보기 | Windows → **`docs/install.md`** · Linux → `setup/setup_envs.sh` → **인수인계서_A1** §3 |
| 조정 가능한 설정 찾기 | **인수인계서_A1** §4.7 |
| **알고리즘이 왜 이런가** | **이 문서** §1~8 |
| 단계별 상세 | `docs/*.md` (calibration / phase1 / phase2 / collision / hw_layout) |

---

## 규약 — 좌표계와 단위

```
T_AB : 프레임 A → B 변환      x_B = T_AB @ x_A
체인 규칙: T_AC = T_AB @ T_BC      (중간 프레임 B가 약분)
```

> 코드·주석·문서 전부 `T_AB` 형식만 쓴다 (`^A T_B`, `T_A^B` 금지).

| 기호 | 프레임 | 설명 |
|---|---|---|
| **B** | Base | xArm7 로봇 베이스 (≡ 월드) |
| **E** | End-Effector | 로봇 플랜지 / TCP |
| **C** | Camera | 카메라 광학 프레임 |
| **F** | Turntable | 회전축 중심 원점, z축 위쪽 |
| **O** | Object | 첫 스캔 기준 내부 글로벌 프레임 |

**상수** (캘리브레이션으로 결정)

| 변환 | 의미 | 출처 |
|---|---|---|
| `T_EC` | E → C (hand-eye) | `config/sensor_frames.yaml::T_EC_artec` |
| `T_B_F0` | B → F (θ=0) | `config/calibration/turntable_frame.yaml` |

**가변값** (매 스텝 계산)

| 변환 | 계산 |
|---|---|
| `T_FB(θ)` | `T_FB0 @ Rz(θ)` |
| `T_BF(θ)` | `inv(T_FB0 @ Rz(θ))` |
| `T_EB` | 로봇 FK 실시간 (`XArmInterface.get_ee_pose_mat()`) |
| `T_CB` | `T_EB @ inv(T_EC)` |

### ⚠ 단위가 섞인다 — 버그 1순위

| 출처 | translation |
|---|---|
| `get_ee_pose_mat()`, yaml `T_EC` | **m** |
| `xarm.set_position(x,y,z,…)` | **mm** |
| Artec SDK `frame_transformation` / vertices / master pts | **mm** |

→ camera-motion `T_pre`의 translation만 `× 1000` 스케일 (§4 merge hint 블록).

---

## 구조

```
mms_artec/
  system.py                      ArtecMMS (오케스트레이터 + disc_surface_frame)
  backends/                      real / isaac 백엔드 팩토리
    isaac/{isaac_world,isaac_xarm,isaac_turntable,isaac_scanner}.py
  sensor/artec_client.py         real 스캐너 (+ capture_points_base)
  nbv/artec_streaming_scan_session.py   Phase 1 streaming SLAM + 4 watchdog
  nbv/artec_multipass_scan_session.py   Phase 1+2+3 통합, view-score, recovery
utils/                           ★ sensor-agnostic 공유 코어 (real·sim 공용)
  calibration/{turntable_frame,rim_picker,hand_eye_calibrator,artec_charuco_detector}.py
  collision/{geometry,robot_collision}.py    자세별 충돌 쿼리 (real/sim 공용)
  nbv/{frontier,icp_strategy,manual_picker,phase2_nbv,flip_policy}.py
  robot/{xarm_interface,xarm7_kinematics}.py     ★ 해석 FK/IK (real·sim 공유)
  turntable/turntable_interface.py    transforms.py    control/theta_planner.py
main_artec.py                    진입점 (BACKEND, RUN_CALIBRATION 토글)
mms_paths.py                     자산(USD) 루트 자동 해석
setup/setup_envs.sh              conda env 3종 생성
sim_harness/MMS_ext_*.py         Isaac 검증 하니스 (실행 시 Isaac 트리로 복사/링크)
```

---

## 환경

```bash
bash setup/setup_envs.sh          # mms-env / env_isaacsim / step2usd 생성 + 검증
```

| env | 용도 |
|---|---|
| `mms-env` | real 백엔드 + 오프라인 스크립트(캘리브·분석). numpy 2.x |
| `env_isaacsim` | isaac 백엔드 (Isaac Sim 5.1). **numpy 1.x** — 섞으면 ABI 오류 |
| `step2usd` | STEP→USD 전용. Isaac 불필요라 빠름 |

**주의 3가지**

| # | 내용 |
|---|---|
| 1 | 셸의 ROS python3.10 경로가 섞인다 → **모든 실행에 `env -u PYTHONPATH`** |
| 2 | OpenCV 5.x엔 `cv2.calibrateHandEye`가 없다 → **`opencv<5` 고정**. 상수는 남아 있어 import는 통과하므로 발견이 늦다 |
| 3 | 콘솔 cp949 이모지 깨짐 → `PYTHONIOENCODING=utf-8` |

**장비** — Artec Spider `SP.10.36181288` (SDK 1.18.4) · xArm7 `192.168.1.210` ·
턴테이블 Ezi-SERVO `192.168.0.10` **UDP**(TCP는 지속 polling 시 socket 막힘)

**실행 명령**

| 대상 | 문서 | 진입점 |
|---|---|---|
| **sim** (Isaac) | `docs/sim_commands.md` | `./scripts/sim/run_e2e_gui.sh mug` |
| **real** (실물 장비) | `docs/7_real_commands.md` | `python main_artec.py` (`BACKEND="real"`) |

조정 가능한 설정은 인수인계서_A1 §4.7.

> SDK 바인딩 변경 시:
> `cmake --build mms_artec/sensor/build --config Release --target <module>`
>
> raw scan(`output/scan_raw/<TS>/`)은 `master.sproj` + `meta.npz`로 저장된다.
> `merge_compare.py --load`로 **스캔 없이 후처리만 반복 실험**할 수 있다.

---

## 사용 예시

```python
from mms.utils.transforms import (
    load_transform, TurntableTransformConfig,
    compute_T_CB, transform_points,
)

# 1. 턴테이블 ↔ 베이스 변환
T_BF0 = load_transform("config/calibration/turntable_frame.yaml", "T_B_F0")
tt = TurntableTransformConfig(T_BF0)

theta = 1.57  # 90도 (rad)
x_B = tt.T_FB(theta) @ x_F   # F 좌표 → B 좌표
x_F = tt.T_BF(theta) @ x_B   # B 좌표 → F 좌표

# 2. 카메라 → 베이스 변환
T_EC = load_transform("config/calibration/hand_eye_artec.yaml", "T_E_C")
T_EB = robot.get_ee_pose_mat()       # xArm FK 결과 (4x4)
T_CB = compute_T_CB(T_EB, T_EC)
points_B = transform_points(T_CB, points_C)

# 3. MMS를 통한 직접 호출
with MMS(cfg) as mms:
    x_B = mms.T_FB(theta) @ x_F
    x_F = mms.T_BF(theta) @ x_B
    T_cb = mms.T_CB(T_EB)
```

---

### ★ sim / real 듀얼 백엔드 (핵심 전략)

`ArtecMMSConfig.backend = "real" | "isaac"` 하나로 robot/turntable/scanner 를 통째 교체.

| | real | isaac (sim) |
|---|---|---|
| robot | `XArmInterface` (xArm SDK) | `IsaacXArm` (해석적 운동학 + sim) |
| turntable | `Turntable` (Ezi-SERVO) | `IsaacTurntable` (RevoluteJoint 드라이브) |
| scanner | `ArtecClient` (Artec SDK) | `IsaacArtecScanner` (Isaac 카메라) |

**개발 전략**: 로직(calibration·view planning·병합)을 **sim의 ground-truth로 개발·검증**하고,
real에선 **Artec SLAM 위에 그대로 올린다**. (Artec 실시간 SLAM은 real 전용 — 퀄리티 좋음.
sim엔 SLAM이 없으므로 θ·카메라 포즈 ground-truth로 점군을 누적해 같은 로직을 검증.)

- 진입점: `main_artec.py` (`BACKEND` 토글). **백엔드마다 python 이 다르다**:
  - `real` → conda `mms-env` (py3.11). `env -u PYTHONPATH python main_artec.py`
  - `isaac` → Isaac Sim python. 리포 스크립트(`scripts/sim/*.sh`) 기준은 conda `env_isaacsim`:
    `env -u PYTHONPATH ~/miniconda3/envs/env_isaacsim/bin/python -u main_artec.py`
    (NVIDIA 번들 런처 `~/isaacsim/python.sh` 도 동작 — 별도 설치본이라 conda deactivate 필요)
  - ⚠ 셸에 ROS `PYTHONPATH` 가 잡혀 있으면 python3.10 패키지가 섞인다 → `env -u PYTHONPATH` 필수.
- sim 씬(USD): `mms_artec/backends/isaac/isaac_world.py::DEFAULT_USD_PATH`
  (`2_3Dassets/frame_xarm7_spider_turntable_v2/v3_scene.usd` = xArm+턴테이블+대상물).
  물체별 씬은 같은 폴더의 `v3_ts_<이름>.usd`, 생성은 `scripts/sim/build_scene_v3.py`.
  환경변수 `MMS_SIM_USD` 로 override — 물체 씬은 `MMS_SIM_OBJECT_PRIM` 도 같이 준다.
  ⚠ 구 `v2.usd` / `testset/composed/*.usd` 는 카메라·턴테이블 prim 경로가 달라 못 쓴다.
- 백엔드 상세: `mms_artec/backends/README.md`

---





## 알고리즘 — 어디에 무엇이 있나

각 단계의 설계와 근거는 **전용 문서 한 곳**에만 둔다. 아래는 지도다.

### 1. Calibration — `T_E_C`(hand-eye) + `T_B_F0`(턴테이블 축)  ✅

스캐너가 본 것과 로봇이 아는 것을 같은 좌표계로 묶는 두 상수. 기계를 옮기거나
센서를 교체하면 다시 잡는다.

- hand-eye: ChArUco(5×3, 100×60mm) + `solvePnP` → `AX=ZB`. **t 3.55mm / r 1.30°**
- 턴테이블 축: disc rim 점 → 3D 원 피팅. **0.015° / 0.7mm**
- ★ 카메라 위치는 SLAM 이 아니라 **로봇 FK + T_EC** 가 알려준다

> ⚠ `turntable_frame.yaml`(2026-04-23)은 Artec 장착 이전 값 → **재캘리브 1순위**

→ **`docs/1_calibration.md`** (원리·코드 지도·규약·함정·남은 일)

### 2. Phase 1 — 5면 스캐닝 (streaming SLAM + view planning)  ✅🔬

로봇을 한 자세에 고정하고 턴테이블을 360° 돌려 측면·윗면을 얻는다.
Artec 은 frame-to-frame 상대 정합이라 **overlap 유지**가 전부다 — 연속 회전 + max FPS.

- 자세 선정: elevation view-score (최적 작업거리 225mm 근처 · FOV 안, **최악 프레임 기준**)
- 커버 부족(z-커버율 <0.75) 시 밴드 분할, 안전한 밴드부터
- 추적 감시 4종 watchdog + 3회 자동 recovery
- ★ 정합 알고리즘은 **`HYBRID`** — `ICP` 는 빈 턴테이블에도 정합 성공해 lost 를 놓친다

→ **`docs/2_phase1.md`** (아키텍처·watchdog·recovery·라이브 뷰어·view-score·sim 검증)

### 3. Phase 2 — 부족면 NBV 보강  ♻️🔬

누적 점군에서 구멍을 찾아 그 지점만 겨냥해 부분 스윕(±45°). 최대 8회.
종료는 **신규 점유 복셀 비율**로 판정한다(경계 길이는 방향이 반대라 쓰면 안 된다).

→ **`docs/3_phase2.md`** (NBV 루프·수렴 지표·성능 병목·정합 게이트)

### 4. Phase 3 — 바닥면 flip & 병합  ♻️🔬

물체를 뒤집어 바닥면을 얻고 앞 결과와 합친다. 각 IScan 은 자기 첫 프레임을 원점으로
잡으므로 `T_pre` 로 master 좌표에 끌어와야 한다.

→ **`docs/5_phase3_merge.md`** (flip 판정·`T_pre` 3가지 경우·face-merging 문제)

### 5. 충돌 · 특이점  ✅

자세를 실행 **전에** 걸러낸다. 캡슐 근사, real/sim 공용.

→ **`docs/4_collision.md`** · 레이아웃 실측은 **`docs/hw_layout.md`**

### 6. 후처리 & 라이브 시각화  ♻️🔬 / ✅

SDK General Pipeline 으로 최종 메시 생성. **Cleaning 은 반드시 Fusion 앞에.**

→ **`docs/6_postprocess.md`**

---

## 알려진 한계 / 가정

1. **turntable_frame.yaml 미검증** — T_BF0 2026-04-23(Artec pivot 이전), rim 3점·residual 0.0.
   Phase 2 hint·NBV·recovery raycast 가 같은 T_BF0 의존 → 정밀도 의심 시 1순위 재캘리브
   (`scripts/artec/turntable_calib.py`). 라이브 뷰어는 SDK 정합행렬 사용해 이 의존 없음.
2. **tracking-lost ≠ object-presence**: 물체 제거해도 빈 디스크에 정합 성공해 lost 안 뜰 수 있음 → HYBRID + 별도 휴리스틱.
3. **last-good θ 없으면 recovery skip** (시작 직후 lost).
4. **Robot 안전성**: xArm IK/limit/self-collision 의존. 도달 불가 pose 추천 시 `set_position` 실패 → 재시도.
5. **Raycast occlusion 무시** (frustum culling 만; close-range 라 영향 작음).
6. **비대칭·길쭉 객체** centroid hint 가정 취약(§3) — 향후 OBB center.
7. **Console cp949**: 이모지 깨짐 — utf-8/PowerShell 터미널 정상.

---

**개발 원칙** — 모든 로직은 sim(ground-truth)에서 개발·검증하고 real(Artec SLAM)에
동일 코드를 적용한다. sensor-agnostic 코어(`utils/`)를 real(Artec)과 sim(Isaac)이 공유한다.

> 단계별 구현 상태와 향후 과제는 **인수인계서_A1** §5·§6 참조.
