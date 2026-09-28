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
- sim 씬(USD): 기본은 `isaac_world.py::DEFAULT_USD_PATH` = **실물 배치를 재현한
  `frame_xarm7_spider_turntable/v2_real_260917.usd`** 다. `MMS_SIM_USD` 로 바꾼다.

  > ⚠ **v3 는 실물이 아니다.** 셀을 v3 로 바꾸기로 했다가 실제로는 안 바꿨다
  > (2026-09-16 현장 확인). v3 는 턴테이블이 로봇 base 바로 아래(수평 0mm)인데
  > 실물은 799mm 떨어져 있어 **자세 선정·도달성·이동량이 전혀 다르다.**
  > 장비 없이 실물 기하만 점검하려면 `scripts/artec/validate_real_cell.py`.

  → 씬 목록·생성·교체 절차는 **`docs/sim_scene.md`**.
- 백엔드 상세: `mms_artec/backends/README.md`

### ★ 작동거리 창(스캔 range)을 바꾸려면

실물에서 "거리가 멀다/가깝다" 를 조정할 때 **딱 한 곳만** 바꾼다.

```python
# main_artec.py  ::  CFG = ArtecMMSConfig(artec=ArtecConfig(...))
scan_range_near_mm = 210.0     # None = SDK 기본값
scan_range_far_mm  = 265.0
```


### ★ 셀 배치가 바뀌면 — 어디를 고치나

로봇 연산(충돌·도달성)이 쓰는 것은 **씬 USD** 와 **충돌 점군 npz** 둘이고, **둘은 한
몸이다.** 어긋나면 로봇이 셀 안에 박힌 것으로 판정돼 모든 자세가 거부된다.

→ 6단계 절차·명령·함정은 **`docs/sim_scene.md` §3** 에 모아뒀다.

---

## 알고리즘 — 어디에 무엇이 있나

각 단계의 설계와 근거는 **전용 문서 한 곳**에만 둔다. 아래는 지도다.

### 1. Calibration — `T_E_C`(hand-eye) + `T_B_F0`(턴테이블 축)  ✅

스캐너가 본 것과 로봇이 아는 것을 같은 좌표계로 묶는 두 상수. 기계를 옮기거나
센서를 교체하면 다시 잡는다.

- hand-eye: ChArUco(`spider_dense` 7×5, 84×60mm) + `solvePnP` → `AX=ZB`. **t 3.55mm / r 1.30°**
- 턴테이블 축: disc rim 점 → 3D 원 피팅. **0.015° / 0.7mm**
- ★ 카메라 위치는 SLAM 이 아니라 **로봇 FK + T_EC** 가 알려준다

> `sensor_frames.yaml::T_EC_artec` 은 **2026-09-16 재캘리브 완료** (`SP.10.79103441`,
> solvePnP, 19자세). 캘리브 스크립트가 이 파일을 직접 갱신한다 — 별도
> `hand_eye_artec.yaml` 은 폐지했다.
>
> ⚠ `turntable_frame.yaml`(2026-04-23)은 Artec 장착 이전 값 → **재캘리브 1순위**

→ **`docs/1_calibration.md`** (원리·코드 지도·규약·함정·남은 일)

### 2. preview — 형상 탐색  ✅🔬

물체가 어떻게 생겼는지 모르는 채로 시작한다(실물엔 GT 가 없다). 높이·반경·적정
작업거리를 재는 **측량 단계**이고, 이 결과가 뒤 단계 전부의 입력이다.

- 조준높이를 올려 가며 훑는다 — "상단이 더 안 늘면 종료"(최대 4회)
- 축거리는 **들어온 점군의 거리 분포로** 정한다: `d ← d + (창중앙 − 표면거리 중앙값)`
- 반환이 0 이면 방향을 모르므로 **가까이·멀리를 번갈아** 벌린다(±40·±80mm)
- `MMS_SIM_STAGE_UNTIL=preview` 로 여기까지만 돌릴 수 있다 (전회전 생략)

→ **`docs/2_preview.md`** (거리 결정·반환 0 탐색·손잡이·코드 지도)

### 3. lookaround — 5면 스캐닝 (streaming SLAM + view planning)  ✅🔬

물체를 **높이 방향 밴드로 썰어**, 밴드마다 로봇을 그 높이의 자세로 옮기고 턴테이블을
360° 돌린다. 한 자세가 물체를 다 덮으면 밴드는 1개다.
Artec 은 frame-to-frame 상대 정합이라 **overlap 유지**가 전부다 — 연속 회전 + max FPS.
밴드 전체가 **한 IScan** 이라 밴드 사이 이동 중에도 SLAM 이 붙어 있어야 한다.

- 자세 선정: 밴드마다 elevation view-score 재채점 (**최악 프레임 기준**)
- 밴드 수는 겹침을 **측정해서** 정한다. 캡처 순서는 **z 단조**(safe-first 아님)
- 밴드 시작마다 축거리를 창 중앙으로 보정 (`StandoffTracker`)
- 추적 감시 4종 watchdog + 3회 자동 recovery
- ★ 정합 알고리즘은 **`HYBRID`** — `ICP` 는 빈 턴테이블에도 정합 성공해 lost 를 놓친다

→ **`docs/3_lookaround.md`** (아키텍처·watchdog·recovery·라이브 뷰어·view-score·sim 검증)

### 4. nbv — 부족면 NBV 보강  ♻️🔬

누적 점군에서 구멍을 찾아 그 지점만 겨냥해 부분 스윕(±45°). 최대 8회.
종료는 **신규 점유 복셀 비율**로 판정한다(경계 길이는 방향이 반대라 쓰면 안 된다).

→ **`docs/4_nbv.md`** (NBV 루프·수렴 지표·성능 병목·정합 게이트)

### 5. flip — 바닥면 flip & 병합  ♻️🔬

물체를 뒤집어 바닥면을 얻고 앞 결과와 합친다. 각 IScan 은 자기 첫 프레임을 원점으로
잡으므로 `T_pre` 로 master 좌표에 끌어와야 한다.

→ **`docs/5_flip.md`** (flip 판정·`T_pre` 3가지 경우·face-merging 문제)

### 6. 충돌 · 특이점  ✅

자세를 실행 **전에** 걸러낸다. 캡슐 근사, real/sim 공용.

→ **`docs/collision.md`** · 레이아웃 실측은 **`docs/hw_layout.md`**

### 7. 후처리 & 라이브 시각화  ♻️🔬 / ✅

SDK General Pipeline 으로 최종 메시 생성. **Cleaning 은 반드시 Fusion 앞에.**

→ **`docs/6_postprocess.md`**
