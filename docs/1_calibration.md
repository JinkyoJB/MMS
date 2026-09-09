# Calibration — Hand-Eye(`T_EC`) & Turntable(`T_B_F0`)

> MMS 의 두 가지 캘리브를 한 문서에. *무엇을·왜·어떻게* + *어느 함수가 무슨 일을 하는지*.
> 상위 맥락은 `docs/main_flow.md` §1.
> - **Part 1 — Hand-Eye `T_EC`**: 카메라가 로봇 손목(EE)에 어떻게 붙어있나.
> - **Part 2 — Turntable `T_B_F0`**: 턴테이블이 로봇 base 기준 어디서·어느 축으로 도나.
>
> **결과가 이상하면 맨 뒤 〔부록〕 Troubleshooting 부터 본다** — 규약·단위·자세·충돌·
> 알려진 문제를 T1~T7 로 모아두었다.

---

# 〔Part 1〕 Hand-Eye — `T_EC`

## 1. 무엇을 구하나 — `T_EC`

| 프레임 | 의미 |
|---|---|
| **B** | 로봇 base (월드) |
| **E** | EE = 플랜지 = 손목 끝 (스캐너 마운트) |
| **C** | 카메라 광학 프레임 |
| **M** | 마커(보드) 프레임 |

구하려는 값: **`T_EC`** = "E와 C의 고정 관계" (카메라가 손목에 붙은 방식). 한 번 구하면
센서 교체·재설치 전까지 안 변하는 상수 → `config/sensor_frames.yaml::T_EC_artec` 에 저장.

> ★ **규약 (헷갈리면 다 틀어짐):** `T_EC` 는 **E→C**, 즉 `x_C = T_EC · x_E` (= "EE-in-camera").
> 시스템 전체가 이 정의를 쓴다 → `utils/transforms.py::compute_T_CB`: `T_CB = T_EB · inv(T_EC)`.

---

## 2. 전체 흐름

```
                    ┌─────────────── 자세 i = 1..N 반복 ───────────────┐
  [준비]            │                                                   │   [풀이]
  보드 고정    ──►  │  ① 로봇을 다양한 자세로 이동                       │ ──► add_sample 들을
  K(intrinsic) 확보 │  ② 카메라 캡처(이미지)                             │     calibrateHandEye
                    │  ③ ChArUco 검출 + solvePnP → T_MC (보드→카메라)    │     → T_EC
                    │  ④ 로봇 FK → T_BE (손목→base)                      │
                    │  ⑤ add_sample(T_BE, T_MC)                          │
                    └───────────────────────────────────────────────────┘
```

**①~⑤와 풀이는 sim·real 이 같은 MMS 라이브러리 코드를 쓴다.**
다른 건 "자세를 어떻게 만들고 캡처하느냐"의 **껍데기**뿐이다.

| 단계 | 담당 | real | sim |
|---|---|---|---|
| ① 자세 생성 | `generate_hemisphere_poses` | 사람이 teach / yaml 순회 | 반구 자세 생성 |
| ① 자세 이동 | `RobotIK.ik` (자체 해석 IK) | xArm SDK 로 모션 명령 | 관절공간 구동 |
| ② 캡처 | — | Artec 실기 | Isaac 카메라 렌더 |
| ③ 검출 | `ArtecCharucoDetector.detect` → `T_MC`(mm) | 공통 | 공통 |
| ④ FK | — | `XArmInterface.get_ee_pose_mat` | `rigid_ee` |
| ⑤ 누적 | `HandEyeCalibrator.add_sample` | 공통 | 공통 |
| 풀이 | `HandEyeCalibrator.calibrate` → `T_EC` | 공통 | 공통 |

> 보드 물리 사양(5×3, 20/15mm, `DICT_4X4_50`)은 `CharucoBoardSpec` 하나로 정의하고
> **검출기와 텍스처 생성이 공유**한다. 실물 인쇄본과 sim 텍스처가 어긋나면 안 되기 때문.

---

## 3. 코드 지도

**공유 라이브러리 — sim·real 공통 (★ 핵심)**

| 파일 | 역할 |
|---|---|
| `utils/calibration/hand_eye_calibrator.py` | `HandEyeCalibrator` — `add_sample(T_EB, T_MC)` / `calibrate()→T_EC`. 5-method 중 잔차 최소 채택 |
| `utils/calibration/handeye_sim.py` | sim 검증 공통 로직 — 보드 규격·자세 생성·풀이·GT 비교·판정 |
| `utils/calibration/turntable_frame.py` | **턴테이블 공유 코어** — `fit_circle_3d` / `fit_plane` / `build_T_B_F0` / `save_turntable_frame_yaml` / `axis_error` |
| `utils/calibration/rim_picker.py` | rim 클릭 UI (`RimPicker` / `run_picker` / `show_3d_result`) |
| `mms_artec/utils/calibration/artec_charuco_detector.py` | `CharucoBoardSpec` / `ArtecCharucoDetector.detect()→T_MC`(mm, OpenCV cam). K 있으면 solvePnP |
| `mms_artec/utils/calibration/handeye_geometry.py` | SE3 수학 + `look_at_camera` + `generate_hemisphere_poses` (numpy 전용) |
| `utils/robot/ik_provider.py` | `RobotIK(...).ik(pose6d, seed)` — 기본 자체 해석 IK |
| `utils/robot/xarm7_kinematics.py` | 해석 FK/IK (수치 DLS) |
| `utils/collision/collision_model.py` | 충돌·특이점 게이트 (메시 SDF) |
| `utils/transforms.py` | `compute_T_CB(T_EB, T_EC)` — ★ `T_EC` 규약의 출처 |

**실물 파이프라인**

| 파일 | 역할 |
|---|---|
| `scripts/artec/make_charuco.py` | 보드 PNG 생성(인쇄용) |
| `scripts/artec/intrinsic_calib.py` | 카메라 K 측정 (1회) |
| `scripts/artec/hand_eye_calib.py` | 메인 루프 — 자세순회 → detect → add_sample → calibrate → save |
| `scripts/artec/turntable_frame_init.py` | 턴테이블 rim 클릭 (`ARTEC_TO_OPENCV` z-flip + `T_CB`) |
| `scripts/phoxi/turntable_frame_init.py` | 〃 PhoXi 판 (`T_CB = T_EB·T_CE`) |
| `config/sensor_frames.yaml` | 결과 `T_EC_artec` 적용처 |

**sim 검증**

| 파일 | 역할 |
|---|---|
| `scripts/sim/calib_handeye_sim.py` | **standalone 러너** (터미널 실행) |
| `scripts/sim/calib_rim_sim.py` + `rim_click_offline.py` | 턴테이블 축 standalone (캡처 / 클릭 2단계) |
| `sim_harness/MMS_ext_calibration.py` | GUI 드라이버 — USD 보드 생성·렌더·관절구동 (Isaac 전용) |
| `sim_harness/MMS_ext_calibration2.py` | 〃 턴테이블 rim 자동추출 → 피팅 → GT 비교 |

---

## 4. 실행

> 옵션·환경변수의 배경과 주의점은 **부록 T3~T6**.

### sim 검증 (권장: standalone — 터미널에서 바로)

```bash
cd "$MMS_ROOT"                                    # 예: .../A1_.../1_코드/MMS
ISAAC=~/miniconda3/envs/env_isaacsim/bin/python

# ── hand-eye (약 60초) ──
env -u PYTHONPATH $ISAAC -u scripts/sim/calib_handeye_sim.py           # 헤드리스
env -u PYTHONPATH $ISAAC -u scripts/sim/calib_handeye_sim.py --gui     # 화면으로 보며

# ── 턴테이블 축 ──  Isaac python 은 cv2 headless 라 클릭 창을 못 연다 → 2단계
env -u PYTHONPATH $ISAAC scripts/sim/calib_rim_sim.py                  # 1) 캡처
conda activate mms-env
env -u PYTHONPATH python scripts/sim/rim_click_offline.py              # 2) 클릭+피팅

MMS_RIM_AUTO=1 env -u PYTHONPATH $ISAAC scripts/sim/calib_rim_sim.py   # 클릭 없이 자동
```

| 옵션 / 환경변수 | 기본 | 뜻 |
|---|---|---|
| `--gui` | 꺼짐 | Isaac 창 표시 |
| `--max-steps N` | 20000 | update 상한 (정상 완주 ~2400) |
| `--out DIR` | `scripts/sim/log/handeye` | 산출물 위치 |
| `MMS_MOVE_RAMP` | 90 | 이동 램프 — 키우면 천천히 움직인다 |
| `MMS_CALIB_POLARS` / `AZIS` | `0,15,30,45` / 8방위 | 자세 구성 (→ T3) |
| `MMS_CALIB_COLLISION` | 1 | 충돌 게이트 (→ T4) |
| `MMS_DRIVE_KP` / `KD` / `MAXEFF` | 2000 / 200 / 500 | 드라이브 게인·최대토크 |
| `MMS_SETTLE_TOL` | 0.045 rad | 관절 수렴 허용오차 (→ T7) |

**결과** (2026-09-09, v3 씬)

```
[CALIB] 캘리브 자세: 25 (보드중심 [0.365, -0.0, 0.672])
[CALIB]   pose 6: 충돌/특이점 — self(tool↔link3,0mm) → skip     ← 게이트가 거른다
[CALIB] 유효 샘플: 15 | IK: analytic
[CALIB] >>> t_err=0.95 mm  r_err=0.10°
[CALIB]     판정: PASS ✅
```

### sim 검증 (대안: Isaac GUI Script Editor)

화면으로 보드 낙하·로봇 이동을 보며 디버깅할 때. 하니스는 **standalone 으로 못 돈다**(→ T6).

```bash
export MMS_ROOT=/경로/MMS     # GUI 띄우기 전
~/isaacsim/isaac-sim.sh
```
```python
# Window > Script Editor
exec(open("/경로/MMS/sim_harness/MMS_ext_calibration.py").read())      # hand-eye
exec(open("/경로/MMS/sim_harness/MMS_ext_calibration2.py").read())     # 턴테이블 축
```

### real — 실제 캘리브레이션

```bash
conda activate mms-env && cd "$MMS_ROOT"

env -u PYTHONPATH python scripts/artec/make_charuco.py       # 보드 PNG → 실척 인쇄 (최초 1회)
env -u PYTHONPATH python scripts/artec/intrinsic_calib.py    # 카메라 K 측정 (최초 1회)
env -u PYTHONPATH python scripts/artec/hand_eye_calib.py     # 자세 순회 → hand_eye_artec.yaml
env -u PYTHONPATH python scripts/artec/turntable_frame_init.py   # rim 클릭 → turntable_frame.yaml

# 기록된 자세로 재실행
env -u PYTHONPATH python scripts/artec/hand_eye_calib.py \
    --poses config/calibration/artec_calibration_poses.yaml
```

→ hand-eye 결과를 `config/sensor_frames.yaml` 의 `T_EC_artec` 에 반영한다.
자세 15~25개, 자세 간 회전 **≥30°** 확보할 것(→ T3).
실물 기준값: **t_err 3.55mm / r_err 1.30°** (2026-04-29)

---

# 〔Part 2〕 Turntable — `T_B_F0`

> 로봇 base 기준 **턴테이블 회전축·표면**(= `T_B_F0`)을 구한다. 방법은 **rim 점 피팅** 하나로 통일
> (구 어레이 방법은 실물 fixture 비용이 커서 채택 안 함 — rim 으로 대체 가능).

## 5. 무엇을 구하나 — `T_B_F0`

| 프레임 | 의미 |
|---|---|
| **B** | 로봇 base (월드) |
| **F** | 턴테이블 프레임 (원점=회전축이 disc **표면**과 만나는 점, z=회전축, θ=0 기준) |

구하려는 값: **`T_B_F0`** = "턴테이블이 base 기준 어디서·어느 축으로 도나". 한 번 구하면 하드웨어
이동 전까지 상수 → `config/calibration/turntable_frame.yaml` 저장.

> ★ 규약: `T_B_F0` 는 **B→F** (`x_F = T_B_F0·x_B`). F 의 z = 회전축(위쪽), 원점 = 표면 위 축점.

왜 필요 — Phase 2 hint·NBV·recovery·충돌회피가 전부 "턴테이블이 base 기준 어디서 도나"에 의존.
하드웨어팀이 옮기면 무효화 → **버튼 하나로 다시 잡는** 루틴.

## 6. 핵심 원리 — "회전하면 원을 그린다"

> 회전판에 고정된 점은 회전축 둘레로 **원**을 그린다 → 원 법선 = 축방향, 중심 = 축 위 한 점.

disc rim(가장자리)은 그 자체가 축 둘레의 원 → rim 위 점들을 3D 로 모아 원을 피팅하면 축이 나온다.

> ★ **축 ≠ 표면**: rim/궤적 높이 ≠ disc 표면 높이일 수 있음. 충돌회피·대상물 높이를 위해 disc
> **표면 평면**을 따로 잡아 축선과 만나는 점을 F0 원점으로 삼는다(§8).

## 7. Rim 방법 — 흐름 + 함수

```
로봇이 disc rim 을 보는 자세 → 1회 캡처 (organized 포인트클라우드 + T_CB)
        │  ※ 카메라 위치는 로봇이 알려줌: 점 → 센서C → T_CB(=T_EC·FK) → base. Artec SLAM 미사용.
        ▼
   rim 위 점 취득
        │   real: 사용자가 rim 위 3+점 **클릭** (RimPicker)
        │   sim : 알려진 disc 기하로 rim 점 **자동 추출**(방위 binning 최외곽)
        ▼
   pts_B (rim, base) → fit_circle_3d → (center, normal, radius, residual)
        ▼
   (+ disc 표면 평면, §8) → build_T_B_F0(center, normal) → T_B_F0
```

- UI/수학: `utils/calibration/`
  - `rim_picker.py` — `RimPicker(intensity, organized_pts, T_CB)` + `run_picker` (OpenCV 클릭:
    LClick=추가/RClick=취소/Enter=피팅), `show_3d_result` (Open3D). pixel→base 3D 내장.
  - `turntable_frame.py::fit_circle_3d(pts)` → `(center, normal, radius, residual)` (평면 SVD + 2D 대수 원피팅).
- ⚠ Spider 좁은 FOV 탓에 rim 전체가 한 화면에 안 들어올 수 있음 → 보이는 호(arc)에서 취득
  (3점이면 가능하나 호가 짧으면 조건수↓).

## 8. 표면 평면 → `T_B_F0` 빌드

축(방향+XY)만으론 부족 → disc **표면**으로 원점 높이 확정:
- `turntable_frame.py`
  - `fit_plane(pts)` → 표면 평면(점·법선, SVD).
  - `build_T_B_F0(center_B, nz_B)` → F 프레임(원점=center, z=nz, x=base x 투영, y=z×x) → **B→F**.
  - `save_turntable_frame_yaml(...)` → translation(m) + quat 저장.
- (선택) `mms_artec/system.py::ArtecMMS.disc_surface_frame(disc_points_base, axis_point, axis_dir)` —
  표면 평면 ∩ 축선 = F0 원점, z축은 축방향, 평면법선은 교차검증. (구 어레이 경로용이었으나
  rim center/normal 을 바로 `build_T_B_F0` 에 넣어도 됨 — rim 은 표면 근처라 단순.)

# 〔부록〕 Troubleshooting — 규약·함정·주의점

> 캘리브가 이상할 때 여기부터 본다. 대부분 아래 중 하나다.

## T1. 좌표 규약 — 틀리면 "정상인데 틀려 보인다"

| 항목 | 규약 |
|---|---|
| `T_EC` | **E→C** (`x_C = T_EC·x_E`). calibrator 반환값도 이 규약 |
| `T_B_F0` | **B→F** (`x_F = T_B_F0·x_B`) |
| sim GT | `inv(T_W_C) @ T_W_E` — **거꾸로 잡으면 결과가 멀쩡한데도 틀려 보인다** |

**카메라 프레임 USD vs OpenCV** — USD/Isaac 카메라는 광축 **−Z·+Y up**,
solvePnP(OpenCV)는 **+Z·+Y down**. 차이는 `R_FLIP = diag(1,−1,−1)`.

```
T_EC_usd = R_FLIP @ T_EC_ocv      ← T_EC 는 카메라가 출력측이라 왼쪽곱
```
> 실물엔 USD 가 없으니 이 flip 은 **sim 검증 전용**이다.

## T2. 단위 — 섞으면 병진 오차가 폭발한다

| 값 | 단위 |
|---|---|
| `T_BE` translation | **m** |
| `T_MC` translation | **mm** (보드 사양이 mm → solvePnP tvec 도 mm) |
| `organized_pts` | **mm** (`rim_picker` 가 /1000) |
| 저장 yaml | **m** |

`add_sample` 이 내부에서 `T_MC` 를 mm→m 변환한다.

## T3. 자세가 부족하거나 치우쳐 있다

- 병진 정확도는 **자세 간 회전 다양성**에 좌우된다. 거의 수직으로만 내려다보면
  회전축이 비슷해져 병진이 부정확해진다 → polar·roll 범위를 넓힌다.
- 충돌 게이트가 자세를 걸러내므로 **후보를 넉넉히** 만든다.
  기본 `(0,15,30,45)×8` = 25 생성 → 15 유효.

| 구성 | 생성 | 제외 | 유효 | t_err | r_err |
|---|---|---|---|---|---|
| (0,12,22)×5 (구 기본) | 11 | 0 | 11 | 1.10mm | 0.15° |
| (0,15,30,45)×5 | 16 | 5 | 11 | 1.30mm | 0.15° |
| (0,15,30)×8 | 17 | 4 | 13 | 1.13mm | 0.16° |
| **(0,15,30,45)×8** | **25** | 10 | **15** | **0.95mm** | **0.10°** |

조정: `MMS_CALIB_POLARS` · `MMS_CALIB_AZIS` · `MMS_CALIB_ROLLS` · `MMS_CALIB_JITTER` · `MMS_CALIB_DIST`

> sim 은 검출 잡음이 없어 11 자세로도 수렴한다. **실물은 잡음이 있어 회전 다양성이 더 중요**
> 하다(실측 3.55mm). 실기 캘리브 시 자세를 아끼지 말 것.

## T4. 충돌·특이점 게이트

생성한 자세를 **구동 전에** 검사한다. IK 해가 나와도 부딪히면 버린다.

| 검사 | 예시 로그 |
|---|---|
| 자가충돌 (스캐너·툴 ↔ 링크) | `self(tool↔link3,0mm)` |
| 환경충돌 (프레임·툴체인저·턴테이블) | `env(link4,0mm)` |
| 특이점 | `singular(σ=0.020<0.050)` |
| 이동 경로 | `이동경로 충돌 — ...` |

메시 SDF 기반(`utils/collision/collision_model.py`)이며 캐시
`utils/collision/data/{cell_env,xarm7_spider_links}.npz` 를 읽는다.
**캐시가 없으면 게이트가 조용히 꺼지므로 로그의 `충돌 게이트: ON` 을 확인**할 것.
끄려면 `MMS_CALIB_COLLISION=0`.

> `start(...)` 사유는 **출발 자세가 이미 여유 밖**이라는 뜻이다. 하니스는 이때 경고만
> 남기고 이동한다 — 거부하면 이후 전부가 같은 이유로 막히는 연쇄가 생긴다
> (`4_collision.md` §1.4).

## T5. 결과 해석 (sim)

`solve_and_report` 가 GT 대비 **t_err(mm) / r_err(°)** 을 찍고
`captures_calib/handeye_result.npz` 를 남긴다. 디버그 이미지 `ok_NN.png` / `fail_NN.png`.

| 증상 | 의심 |
|---|---|
| `t_err` 큰데 `r_err` 작음 | 자세 회전 다양성 부족 → **T3** |
| 검출 `fail` 많음 | 자세가 너무 비스듬하거나 보드가 시야를 벗어남 |
| 정상인데 큰 오차 | 프레임 flip 누락 → **T1** |

목표 `t < 5mm`, `r < 2°`. 실물은 GT 가 없으니 calibrator 잔차와
point-consistency(고정점을 여러 자세서 base 로 변환 후 산포)로 본다.

## T6. sim 실행 관련

- **하니스는 standalone 이 아니다** — `sim_harness/MMS_ext_*.py` 는 Isaac GUI 안에서만
  돈다(`ModuleNotFoundError: No module named 'omni.usd'`). 터미널에서 돌리려면
  `scripts/sim/calib_handeye_sim.py` 를 쓴다.
- **`utils` 패키지명 충돌** — Isaac 런타임에 동명 `utils` 가 있어 `from utils...` 가 깨진다.
  → MMS 모듈을 **파일경로 로드**(`sys.modules` 등록 필수, `@dataclass` 때문). 옮길 모듈은
  레포 내부 import 없는 **자기완결**이어야 한다(`handeye_geometry`/`ik_provider`/`xarm7_kinematics`).
- **종료 시 빨간 메시지** — `Task was destroyed but it is pending!` 등은 Isaac 위젯 정리
  잡음이다. 결과와 무관하니 무시하고 `===== COMPLETE =====` 블록을 본다.
- **로봇이 너무 빠르면** `MMS_MOVE_RAMP` 를 키운다(기본 90).

## T7. 알려진 문제

- ⚠ **`turntable_frame.yaml` stale 의심** — 2026-04-23(Artec 장착 이전). Phase 2 조준·
  recovery·충돌회피가 전부 여기 의존한다. **정밀도 의심 시 재캘리브 1순위.**
- ⚠ **드라이브 계통 오차** — 완전 정지(속도 0) 후에도 목표 대비 joint1 +1.4° /
  joint2~7 +0.45~0.8° 가 남는다. 게인을 100배 올려도 joint1 은 불변이라 제어 문제가
  아니다. **원인 미규명.** hand-eye 는 실측 EE 자세와 실측 영상을 쌍으로 쓰므로 결과
  영향은 없다(t_err ~1mm PASS). 허용오차 `MMS_SETTLE_TOL` 2.6° 로 헛된 타임아웃만 막아둠.
- ⚠ **코드 중복** — `scripts/artec/turntable_frame_init.py` 가 공유 코어 대신 자체
  `fit_circle_3d`/`_RimPicker` 를 들고 있다(PhoXi 는 공유 코어 사용). 통일 권장.
- ⚠ **IK** — real·sim 모두 `xarm7_kinematics`(수치 DLS). xArm SDK IK 는 컨트롤러 통신이라
  하드웨어 연결이 필요하고 불안정해 **미사용**(`use_sdk=False` 기본). 모션 명령만 SDK.
- 🔬 **hand-eye 검증 스크립트 미작성** (`artec_hand_eye_validate.py`) — 캘리브에 쓰지 않은
  별도 N_test 자세에서 point-consistency 측정. 목표 <0.5mm.
- ⚠ **Spider FOV** 가 좁아 disc rim 전체가 한 화면에 안 들어올 수 있다 → 보이는 호(arc)에서
  클릭. 원 피팅은 3점이면 되나 호가 짧으면 조건수가 나빠진다.
