# MMS 자동 3D 스캐닝 — 전체 흐름 (main_flow)

> Artec Spider + xArm7 + 턴테이블 자동 3D 스캐닝 시스템의 **단일 통합 문서**.
> 큰 그림 + 단계별 핵심 알고리즘 + 공통 인프라 + 좌표/환경 규약을 모두 담는다.
> 상태 표기: ✅완료 · ♻️기존재사용 · 🔬조사/미구현
>
> **최종 산출물 = 아이템 전면(full-coverage)의 water-tight mesh + texture.**
> Phase 1/2/3 · tracking-lost recovery · hand-eye · 라이브 뷰어 등 모든 sub-system 의
> 존재 이유는 결국 이 한 산출물에 귀결.
>
> 좌표 규약: `CLAUDE.md` (`T_AB : A→B`, `x_B = T_AB @ x_A`) — §좌표·단위 규약 참고.

---

## 0. 시스템 개요

```
       ┌─────────── xArm7 (EE에 Artec Spider 장착) ───────────┐
       │   robot.ik/fk, move, look_at  ← 스캐너 위치·자세 제어   │
       └───────────────────────────────────────────────────────┘
                          │  스캐너가 바라봄
       ┌──────────────────▼───────────────────┐
       │  턴테이블 (RevoluteJoint, θ 회전)      │
       │   └ 스캔 대상물(회전) + 보정 fixture    │
       └───────────────────────────────────────┘
```

- **센서**: Artec Spider (구조광, 핸드헬드, 작동거리 0.2~0.3m, 좁은 FOV ~30°, SDK SLAM 추적)
- **로봇**: xArm7 (스캐너를 원하는 distance/방향으로 이동)
- **턴테이블**: 대상물을 회전시켜 측면 전체를 스캐너 앞으로 통과

### ★ sim / real 듀얼 백엔드 (핵심 전략)

`ArtecMMSConfig.backend = "real" | "isaac"` 하나로 robot/turntable/scanner 를 통째 교체.
**같은 인터페이스, 다른 속** — orchestration 코드는 진짜/시뮬을 모른다.

| | real | isaac (sim) |
|---|---|---|
| robot | `XArmInterface` (xArm SDK) | `IsaacXArm` (해석적 운동학 + sim) |
| turntable | `Turntable` (Ezi-SERVO) | `IsaacTurntable` (RevoluteJoint 드라이브) |
| scanner | `ArtecClient` (Artec SDK) | `IsaacArtecScanner` (Isaac 카메라) |

**개발 전략**: 로직(calibration·view planning·병합)을 **sim의 ground-truth로 개발·검증**하고,
real에선 **Artec SLAM 위에 그대로 올린다**. (Artec 실시간 SLAM은 real 전용 — 퀄리티 좋음.
sim엔 SLAM이 없으므로 θ·카메라 포즈 ground-truth로 점군을 누적해 같은 로직을 검증.)

- 진입점: `main_artec.py` (`BACKEND` 토글). 실행: `~/isaacsim/python.sh main_artec.py`.
- 백엔드 상세: `mms_artec/backends/README.md`

---

## 1. Auto-calibration — Hand-eye T_E_C + 턴테이블 축 T_B_F0  ✅

세 상수 트랜스폼을 구해 저장한다. 셋 다 "기계를 옮기거나 센서를 교체하면 다시 잡는다".

| 트랜스폼 | 의미 | 성격 | 저장 |
|---|---|---|---|
| `T_E_C` | EE → 카메라 (hand-eye) | 정적 — 센서 교체·재설치 시 | `config/sensor_frames.yaml` (Artec, 2026-04-29 고정) |
| `T_B_F0` | base → 턴테이블 축(θ=0) | 설치 — 기계 재조립 시 | `config/calibration/turntable_frame.yaml` |
| `T_O_F0` | 내부 글로벌 → 턴테이블 | 런타임 — 매 세션 시작 시 체인 계산 | 세션 메모리만 |

### 1.0 Hand-eye T_E_C — eye-in-hand 정적 보정

카메라가 EE에 rigid 고정(eye-in-hand). 로봇을 N개 자세로 움직이며 고정 타깃을 촬영,
**Robot-World + Hand-eye** (`AX = ZB`) 동시 해. 체인: `T_B_tgt = T_B_E^(i) · T_E_C · T_C_tgt^(i)` (∀i).

> **★ 핸드헬드인데 hand-eye가 성립하는 이유 — 단일캡처 C vs SLAM W (혼동 주의)**
> Artec Spider 는 두 모드가 있다. ① **단일 캡처**(`capture()`→`IFrameMesh`,
> `artec_client.py:112`)는 점군을 **센서 자신의 고정 광학 프레임 C**(geometry camera,
> 하드웨어에 rigid)로 반환 — PhoXi 한 샷과 본질 동일, 시간에 무관하게 불변.
> ② **스트리밍 SLAM**(`ScanSession`)은 **첫 시간-프레임** 카메라 위치를 원점으로 잡은
> 임의 scan-world **W** 에 frame-to-frame 정합을 누적 → 매 프레임 `frame_transformation`
> (C→W)이 다르고 드리프트한다. SDK 문서의 *"첫 프레임을 원점으로"* 는 ②에만 해당.
> **hand-eye 는 ①만 사용**하고 ②의 SLAM 변환은 절대 안 쓴다 (`capture_points_base`,
> `artec_client.py:191`: "first-frame 추적 transform 은 쓰지 않는다" — 카메라 위치는
> SLAM 이 아니라 로봇 FK + T_EC 가 알려준다). → SLAM 의 임의 원점·드리프트가
> calibration 에 미치는 영향 = 0. (cf. §2 streaming 은 ② 위에서 동작, §8 라이브뷰도 ②.)

- 솔버: `AX=ZB` (`cv2.calibrateHandEye` / `calibrateRobotWorldHandEye`).
  N≥3 (실용 15~25), 자세 간 회전 ≥30° 필요. X=`T_E_C`(목표), Z=`T_B_tgt`(부산물).
- 센서별 `T_C_tgt` 취득: PhoXi=Photoneo `RecognizeMarkers`(A4-REV-23A 보드),
  **Artec=ChArUco(texture image) 검출 + `cv2.solvePnP`** (첫 시도 UV→3D nearest-vertex 는
  양자화 ~1mm/자세가 hand-eye 에 증폭돼 잔차 25mm → solvePnP 로 sub-pixel, 3.55mm 로 수렴).
  Spider 좁은 FOV 탓에 **작은 5×3 보드(100×60mm)** 사용(A4 7×5 는 FOV 밖).
- ✅ **Artec hand-eye 완료** (2026-04-29): ChArUco+solvePnP, `cv2.calibrateHandEye` 5-method 중
  PARK 채택 → **t_err 3.55mm / r_err 1.30°**, `config/sensor_frames.yaml::T_EC_artec` 적용.
  구현: `utils/calibration/{hand_eye_calibrator,artec_charuco_detector}.py`,
  `scripts/artec_{intrinsic,hand_eye}_calib.py`.
- 검증: point-consistency error (고정점 P를 여러 자세서 base로 변환 후 산포). 목표 <0.5mm.
- 🧪 **sim 선검증** (Isaac): 실기 적용 전 ground-truth 를 아는 가상환경에서 파이프라인 자체를
  먼저 확인. ChArUco 보드를 **USD 텍스처 평면으로 실제 렌더** → 카메라가 찍고 cv2.aruco 검출 →
  solvePnP → calibrateHandEye → **USD 에서 읽은 GT T_E_C 와 t_err/r_err 비교**.
  스크립트: `standalone_examples/play/MMS/MMS_ext_calibration.py` (Isaac extension 모드, `MMS_ext.py` 기반).
  ChArUco 보드는 얇은 박스 rigid body 로 턴테이블 위에 **중력 낙하·안착**, 마블은 비활성.
  > ★ **모든 연산·로봇제어는 실물과 공유** (sim 중복 구현 금지): sim 하니스는 USD/Isaac 환경
  > 코드만 갖고, 나머지는 MMS 모듈을 **파일경로로 로드해 그대로 사용**:
  > `artec_charuco_detector.py::ArtecCharucoDetector`(검출+solvePnP), `hand_eye_calibrator.py::
  > HandEyeCalibrator`(AX=XB), `handeye_geometry.py`(SE3 수학+look-at+`generate_hemisphere_poses`,
  > numpy 전용 자기완결), `ik_provider.py::RobotIK`(pluggable IK). 주의: Isaac 의 `utils` 패키지명
  > 충돌 → 파일경로 로드, `@dataclass`는 `sys.modules` 등록 필요, 옮길 모듈은 레포 내부 import 없는
  > 자기완결이어야 함(`xarm7_kinematics` 패턴). sim 전용 = USD 셋업·렌더·GT 비교뿐.
  > ★ **T_EC 규약 = E→C = EE-in-camera** (`compute_T_CB`: x_C=T_EC·x_E; HandEyeCalibrator 반환과 동일).
  > sim GT = `inv(T_W_C)@T_W_E`, OpenCV→USD flip 은 **왼쪽곱** `T_EC_usd = FLIP @ T_EC_ocv`
  > (T_EC 는 카메라가 출력측이라 §위 camera-in-EE 와 flip 방향이 반대 — 합성검증 t_err 0.0mm).
  > ⚠ **카메라 프레임 규약 함정**: USD/Isaac 카메라는 광축 **-Z·+Y up**, OpenCV(solvePnP)는
  > **+Z·+Y down**. 둘은 `R_FLIP=diag(1,-1,-1)` 차이. solvePnP 로 푼 T_E_C 는 OpenCV 프레임
  > 이므로 USD GT 와 비교 시 `T_E_C_usd = T_E_C_ocv @ FLIP` 보정 필수(빼먹으면 정상인데도 큰 오차로 보임).
  > ★ **IK 는 pluggable (sim/real 괴리 최소화)**: 실물 xArm IK 는 **컨트롤러 통신**(SDK
  > `get_inverse_kinematics`→`arm_cmd.get_ik`, `@xarm_is_connected`)이라 오프라인 불가 —
  > UFACTORY 가 컨트롤러 IK 의 오프라인 재현본을 제공 안 함(ros MoveIt 은 범용 솔버라 zero-gap 아님).
  > → **로봇 연결되면 xArm SDK IK(zero-gap), 아니면 `utils/robot/xarm7_kinematics.py`(공칭 DH, USD 정합) fallback.**
  > 로봇 켜지면 두 IK 를 N 포즈 비교해 일치하면 offline 신뢰. PhysX 자코비안 크롤은 폐기(또아리 발생) →
  > **artec home(`IsaacXArm.HOME_JOINTS_DEG["artec"]`, 충돌무) seed + 관절공간 구동**.

> ⚠ Artec hand-eye(2026-04-29)는 양호하나 **turntable_frame.yaml(2026-04-23)은 Artec
> pivot 이전 값** → stale 의심(§알려진 한계). 정밀도 의심 시 재캘리브 1순위.

> 상세: **`docs/1_calibration.md`** (hand-eye 로직 흐름 + 코드 지도 + 규약·함정).

### 1.1 턴테이블 축 T_B_F0 — 설치 보정

하드웨어팀이 턴테이블/로봇을 옮기면 T_B_F0(로봇 base 기준 턴테이블 축·평면)가 무효화됨.
**버튼 하나로 다시 잡는** 루틴. 두 가지 방법, **공용 코어**:

원리: *회전판에 고정된 점은 원을 그린다 → 원의 법선=축방향, 중심=축 위 점.*

| 방법 | 설명 | 상태 |
|---|---|---|
| **1.1 3구 자동** | 반경 아는 구 3개를 턴테이블에 부착 → θ 회전하며 스캔 → known-R 구중심 피팅 → 축. **+ disc 표면 평면 스캔**(표면높이=F0 원점, 법선 교차검증) | ✅ |
| **1.2 rim 3점 클릭** | 디스크 rim 을 스캐너로 보고, OpenCV로 사용자가 점 클릭 → 3D 원 피팅 → 축·평면 | ✅ PhoXi+Isaac (검증 0.015°/0.7mm) |

### 핵심 통찰
- **카메라 위치는 로봇이 알려준다**: 점군을 `센서 → T_EC(손-눈)·FK → 로봇 base` 로 변환.
  **Artec first-frame 추적(SLAM)은 calibration에 쓰지 않는다** (임의기준·드리프트). →
  `sensor.capture_points_base(robot, T_EC)`.
- **known-R 구피팅이 결정적**: 앞면(cap)만 봐도 반경 고정 시 중심 유일 (자유R 76° → known-R 0.14°).
- **축 ≠ 표면**: 3구는 회전축(방향+XY)만 준다(궤적높이=구높이). 충돌회피 + 대상물 기준
  높이를 위해 disc **표면**을 따로 잡아야 한다 → 축 XY 안 뒤 disc 조준·평면 피팅
  (`system.disc_surface_frame`). 표면점=F0 원점, 평면법선은 구-축과 교차검증(≈0°).
  rim 곡면은 크롭(R<반경)해서 평평한 윗면만 피팅.

### 파일
- `utils/calibration/turntable_frame.py` — `fit_circle_3d`, `build_T_B_F0`, `save_turntable_frame_yaml` (수학, 센서무관)
- `utils/calibration/turntable_axis.py` — 3구 자동(`fit_sphere_center`, `estimate_axis`, `axis_error`)
- `utils/calibration/rim_picker.py` — OpenCV 클릭 UI + Open3D 뷰 (센서무관)
- `mms_artec/system.py::ArtecMMS.calibrate_turntable_axis(...)` — **sim/real 공통 진입점** (base 프레임 축 반환)
- `scripts/sim/calib_3sphere_sim.py` — 3구 sim 검증 하니스 (GT 비교, 축 시각화, perturb 강인성, 로그→`scripts/sim/log/`)
- `scripts/sim/calib_rim_sim.py` — rim 캡처(Isaac) + 자동검증(클릭 합성). ★ Isaac python 은
  GUI 없음 → 수동 클릭은 `scripts/sim/rim_click_offline.py`(일반 python, cv2/matplotlib)로 분리
- `mms_artec/backends/isaac/calib_fixture.py` — sim 가상 구 fixture 셋업(rider 부착 + 스캐너 조준)
- `mms_artec/backends/isaac/isaac_scanner.py::capture_organized` — rim-클릭용 조직화 캡처
  (intensity + organized_pts[base mm] + T_CB) — Isaac 카메라 어댑터
- `scripts/phoxi/turntable_frame_init.py` — PhoXi rim-클릭 (위 코어 재사용)

### sim 검증 결과
3구 자동, 헤드리스: **축 방향오차 0.03°, 위치오차 0.6mm** (base 프레임). disc 표면 평면:
**법선 vs 구-축 0.03°, 평면 RMS 0mm**(평평), 표면높이로 F0 원점 확정. ScanTarget(스테이션
통째)을 ±0.04m 랜덤 perturb 해도 동일 정밀도 유지(강인). 산출물: `recognized_spheres.ply`,
`center_tracks_topview.png`(궤적+GT/EST축 일치), `cam_rgb_aim.png`, `result.json`(축+표면) 등.

핵심 교훈(sim fixture): 구 prim 은 부모 xform 오프셋이 없는 `/World` 아래 생성해야
의도한 world 좌표에 놓인다. 또 구 z 폭은 스캐너 작동거리 창(0.2~0.3m) 안에 들어오게
압축(centroid 기준 ±0.04m)해야 회전 내내 캡처된다.

### 남은 일
- ✅ Isaac rim-클릭 어댑터 완료(`capture_organized`). real Artec 는 동일 계약(intensity,
  organized_pts, T_CB)만 채우면 `rim_picker` 그대로 재사용.
- ⚠ Spider 의 좁은 FOV(작동거리 0.2~0.3m) 탓에 디스크 rim 전체가 한 화면에 안 들어올 수
  있음 → 보이는 호(arc)에서 점 클릭(원피팅은 3점이면 가능하나 호가 짧으면 조건수↓).
- 🔬 hand-eye 검증 스크립트(`artec_hand_eye_validate.py`) — 별도 N_test 자세서 point-consistency 측정(미작성).

---

## 2. 5면 스캐닝 — Phase 1 streaming SLAM + view planning  ✅🔬

턴테이블이 360° 돌며 측면을 보여줌. **로봇은 (distance, 바라보는 방향)만** 결정하면 됨
(턴테이블이 azimuth 커버 → 로봇은 2D 문제: standoff + elevation).

- 취득 ~4fps 가정. **1fps마다** 현재까지 스캔된 표면 + 스캔 진행방향(θ)을 샘플링 →
  (distance, 방향) 재계산 → 스캐너 이동.
- real: Artec SLAM이 실시간 스캔 누적/추적 → 그 위에 view planning만 얹음.
- sim: ground-truth 누적으로 같은 view-planning 로직 개발·검증.

### Phase 1 — Streaming SLAM 메커니즘 (구현됨)

연속 회전 턴테이블 위에서 Spider 가 `IScanningProcedure` streaming SLAM 으로 한 바퀴 돌며
frame-by-frame 정합. 한 번의 360° 회전으로 **5면**(윗면+옆면4) 관찰; 바닥면은 디스크에 닿아
캡처 불가 → Phase 2(§3)에서.

- **시작 자세 = home 그대로** (2026-05-20 rule). 사용자가 사람 눈으로 물체에 조준해놓은 EE
  자세에서 그대로 360° 회전. **사전 probe/elevation search 안 함** — 사전 적응은 모든 scan 에
  ~1~2분 overhead인데, well-aimed 자세면 대부분 잘 동작. tracking lost 가 나야 비로소 자세가
  나쁘다는 신호 → 그때만 recovery 흐름(§6)이 적응 자세 탐색.
- 회전 중 **robot 고정** (Phase 1 불변식). overlap 은 한 pose 의 넓이가 아니라 **시간**(연속
  회전 + max FPS)에서 나온다. 멀리 빼서 FOV 넓히면 해상도·정확도 급락 + 350mm 초과 시 재구성 불가.

**Why Artec ≠ PhoXi (설계 근본 차이)**

| | PhoXi | Artec |
|---|---|---|
| 좌표 기준 | 절대 (T_CO 직접) | 상대 (이전 frame 기준 ICP) |
| 탈선 시 | 다음 frame 도 복구 가능 | tracking 잃으면 **scan 망가짐** |
| Phase 1 핵심 | 정확한 hand-eye+θ | **OVERLAP 유지** |
| Frame 간격 | 15° step OK | 작은 step(15°도 위험) → **연속 회전** |

→ 천천히 연속 회전 + Spider max FPS 추종. 한 번 끊기면 그 시점까지 frame 만 살아남음.

핵심 설정(`ArtecStreamingScanSessionSettings`): `rotation_duration_s=30`, `target_fps=None`(max),
`registration=HYBRID`(geometry+texture; ICP-only면 물체 제거 후 빈 턴테이블에 정합 성공해 lost 미탐),
turntable 통신 **UDP**(TCP는 sustained polling 시 socket 막힘).

### Elevation view-score — 어느 고도각 φ가 좋은가 (구현됨)

턴테이블이 azimuth를 커버하므로 로봇이 고를 자유도는 **고도각 φ 하나**. 후보 φ들을
preview로 찍어 점수화하고 `argmax_φ`로 best 자세를 선택한다 (`_elevation_search` /
`_phase1_view_score` in `artec_multipass_scan_session.py`).

> **호출 시점**: 정상 scan 시작에는 안 부름 — 사용자가 맞춘 home 자세로 출발.
> **tracking-lost recovery 흐름에서만** 호출 (candidate 축소: 기본 `[-5°, 0°, +5°]`, fine search skip).

> **score(φ)** = 그 자세 preview 중 **물체로 분류된** 점들이 **최적 작업거리(~225mm)
> 근처 · FOV 안** 에 얼마나 모여있나 (개수 가중합).

알고리즘 5단계:
1. **C→B 변환** — preview 점군을 base 프레임으로 (멤버십·z_table이 B 기준).
2. **물체 멤버십** (`_in_object_profile`, 3-AND): cylinder pre-clip → turntable
   hard floor(`z > z_table+8mm`, 디스크 표면 무조건 제외) → probe로 만든 축대칭
   `(r,z)` occupancy lookup(360° 회전대칭화 → 어느 θ candidate든 성립).
3. **광축 기저** — 캘리브된 `fwd_C/up_C`로 정규직교 (hardcoded `+Z_C` 미사용, mis-aim 회피).
4. **FOV 판정** — Spider 30°(H)×21°(V), depth>1mm로 등 뒤 점 제거.
5. **Gaussian 거리 가중** — `w(d)=exp(−((d−225)/25)²)`, band[200,250]=1σ. 같은 점수라도
   225mm에 모인 자세가 이김. → `score = Σ w(depth)·1_FOV`.

**⚠ 한계 (single-θ)**: 현재 score는 θ=0 한 시점 preview만 본다. 비대칭 물체(손잡이/주둥이)는
θ마다 best φ가 달라질 수 있는데 360° 동안 로봇은 고정. 해결안(미구현): probe 객체점을
axis로 회전 보정해 가상 θ N개 합산 / candidate마다 짧게 회전하며 평균 / top-K multi-pass.

### 재사용 부품 (♻️ 대부분 존재)
- `utils/nbv/frontier.py` — 빈 영역(미관측) 후보 추출
- `utils/nbv/manual_picker.py::compute_camera_pose_from_normal` — 표면 법선→카메라 포즈
- `utils/control/theta_planner.py` — θ 최적화 (최소 모션)
- `IsaacWorld.look_at_camera(target, cam_pos)` — 스캐너 조준 프리미티브 (sim)
- `utils/collision` + `ArtecMMS.{turntable_collision_world,check_pose_collision}` — 자세별
  충돌 쿼리(real/sim 공용). 캡슐 근사, calibration 축·표면(base)으로 월드 구성 → view
  planning 후보 자세를 실행 전 거른다. (sim 캡슐=USD 실제 링크포즈, real=공칭 해석 FK)
- `mms_phoxi/nbv/scan_session.py` — frontier→cost→plan→ICP→integrate 루프 (PhoXi 완성형, 패턴 참조)

### 남은 일
- 🔬 **1fps 표면→(distance,방향) 샘플링 오케스트레이션** + Artec SDK SLAM 연동
- 🔬 sim에서 ground-truth 기반 view-planning 검증 (Phase B)

> 구현: `mms_artec/nbv/artec_multipass_scan_session.py` (`_elevation_search`,
> `_phase1_view_score`, `_in_object_profile`, `_build_rz_profile`),
> `mms_artec/nbv/artec_streaming_scan_session.py`

---

## 3. 아랫면 스캐닝 — 180° flip + 재스캔 + 병합  ♻️🔬

2단계 결과물(Artec SLAM 기반, 퀄리티 좋음)에 **바닥면 추가**:
대상물을 180° 뒤집고 턴테이블 360° 재회전 → 새 스캔을 이전 SLAM 데이터와 병합.

`ArtecMultiPassScanSession` 이 Phase 1+2 를 통합 오케스트레이션. 사용자가 객체를 물리적으로
회전시키며 여러 pose 를 새 IScan 으로 캡처, master IModel 에 누적. **시작 자세 = home 그대로**
(Phase 1 과 동일). 사용자가 자세 바꾼 뒤 [Enter] → robot 은 그 home 에서 360° 회전.

- Pose 0: canonical (face1=top) — Phase 1 / Pose 1: Ry(+90°) 옆면 보강 / Pose 2: Ry(+180°) 바닥면.
- `pose_idx`=논리 자세 인덱스(hint index), `n_pass`=실제 IScan 수(retry 포함). tracking-lost
  retry 는 pose_idx 유지(같은 hint), 정상 완료 + 사용자 [Enter] 시만 `pose_idx += 1`.

### Face-merging 문제 (왜 disambiguation 이 필요한가)
새 IScan 은 SDK 가 **자기 첫 frame 기준** 좌표계로 시작 → 첫 IScan 과 무관. GlobalRegistration
이 초기 추정 없이 identity 에서 출발 → 대칭/유사 아이템에서 윗면(face1)과 바닥면(face6)을 동일면
으로 **오인 합병**(local minimum). 대칭↑ → identity cost↓ → 함정↑. (Studio 는 manual alignment
로 시작 transform 을 줘 회피 — 우리 코드엔 없음.) → §4 의 **centroid-pivot pre-rotation hint** 로 해소.

### 기존 설계/구현 (♻️)
- `mms_artec/nbv/artec_multipass_scan_session.py` — `make_axis_physical_rotations("y",[0,90,180])`,
  Phase 2(바닥면+정합), **centroid-pivot pre-rotation hint** 로 두 자세 모호성 해소(상세 §4).

### 남은 일
- 🔬 flip 후 두 SLAM 스캔의 **정합 병합** 검증 (hint 적용 시 GlobalReg skip 규칙 등)
- ⚠ 비대칭·길쭉한 객체: 눕히면 surface-vertex centroid 가 body 기준 이동 →
  "centroid=body 중심" 가정 깨짐. 향후 OBB center 사용 검토.

> 병합 변환(T_pre) 상세는 §4.

---

## 4. 2단계 + 3단계 데이터 병합  ♻️🔬

- Artec SDK `GlobalRegistration` 또는 `utils/nbv/icp_strategy.py::icp_with_gates` / `pick_icp_roll`
- 누적/퓨전: `mms_phoxi/nbv/{tsdf_volume,pcd_accumulate_volume}.integrate_frame / merged_pcd`
- 🔬 Artec 경로로 통합 + 검증
# 4.1. 비유 — 사진 모자이크

각 **IScan** 은 객체 한 면을 360° 돌면서 찍은 사진묶음 (점군).
**master** 는 모든 사진을 한 캔버스에 정렬해서 붙인 모자이크.

문제는, 각 IScan 이 **자기 첫 사진을 (0,0,0) 으로 잡는다**는 점.

```
   IScan_1 의 세계 (W1)             IScan_2 의 세계 (W2)
        ↑z                                ↑z
        │                                 │
        ●─→y  ← 캔 윗면 자세              ●─→y  ← 캔 뒤집은 자세
       /                                 /
      x                                 x
```

W1 과 W2 는 서로 다른 좌표계. 그냥 갖다 붙이면 캔이 두 마리, 90° 어긋난
이상한 모양이 나옴. → **각 IScan 의 점들을 master 좌표(=W1)로 옮길 변환**
이 필요. 그 변환이 곧 `T_pre`.

---

## 4.2. T_pre 란 무엇인가

`T_pre` = "IScan 의 모든 frame 위치를 master 좌표로 옮길 도장".

IScan 안의 각 사진은 SDK 가 이미 자기끼리는 정합해놨음 (`frame_transformation`).
거기에 `T_pre` 하나를 **왼쪽에 곱**하면 IScan 전체가 master 좌표로 평행이동·회전.

```python
for i in range(scan.frame_count()):
    T_old = scan.get_frame_transformation(i)         # IScan 내부 정합 결과
    scan.set_frame_transformation(i, T_pre @ T_old)  # master 좌표로 끌어옴
```

`T_pre` 가 잘못되면 그 IScan 전체가 잘못된 위치/자세로 master 에 들어감 →
모자이크가 어긋남. **mesh 가 직교하거나 둥둥 떠있으면 거의 항상 T_pre 의심.**

---

## 4.3. T_pre 는 어떻게 정해지나 — 3가지 경우

`_attempt_recovery` 결과(`next_T_BC_pending`) > `pose_physical_rotations` > 없음 순.

### 경우 ① 아무것도 없으면 — `T_pre = None`

가장 흔한 케이스. **Phase 1 첫 pass**.

- 사용자가 객체 안 돌림 (`R_phys = I`)
- recovery 발동 안 함 (tracking lost 없음)

→ IScan_1 의 자기 W1 좌표가 곧 master 좌표 (= reference). 추가 변환 0.

```
IScan_1 (W1)  ──── 그대로 ────→ master  (master_center = IScan_1 의 vertex 평균)
```

### 경우 ② 사용자가 객체를 돌렸으면 — R_phys hint

**Phase 2**. 사용자가 캔을 손으로 90° 돌리고 [Enter].

이때 IScan_2 의 W2 는 IScan_1 의 W1 에 비해 객체가 90° 돌아간 상태로
찍힘. master 좌표(W1)로 옮기려면 **반대로 90° 되돌려야** 함.

> 핵심: 객체가 +90° 돌았으면 데이터를 −90° 로 보정.

게다가 그 회전을 **카메라 원점이 아닌 객체 centroid** 를 pivot 으로 해야
함. 카메라 원점 pivot 으로 회전하면 객체 중심이 ~30cm 멀리 튀어버려서
정합 파탄.

```
            (c_pass)          (c_master)
              ●                   ●
             /│\                 /│\
   ┌────────┘ │ └──┐    ──→    ┌─┘ │ └─┐     ← centroid 끼리 일치시키며
   │  IScan_2 │    │           │   │   │       반대 방향 회전
   │ 회전된 자세 │              │ master  │
   └──────────────┘             └─────────┘
```

수식:

```
R_phys      : 사용자가 base 좌표에서 객체에 가한 회전 (예: Ry +90°)
R_W         : 그걸 scan world 좌표로 변환  (= T_BC · R_phys · T_CB)
c_pass      : IScan_2 의 모든 vertex 평균 (mm)
c_master    : Pass 1 (IScan_1) 의 vertex 평균 (lock, mm)

T_pre = Translate(c_master) · inv(R_W) · Translate(−c_pass)
```

직관:
1. IScan_2 의 centroid 를 원점으로 옮긴다 (`Translate(-c_pass)`)
2. 객체 회전을 되돌린다 (`inv(R_W)`)
3. master centroid 자리에 갖다 놓는다 (`Translate(c_master)`)

`hints_applied=True` 가 켜지면 → 그 뒤 단계의 **GlobalRegistration 은
자동 skip** (이미 hint 가 정답에 가깝게 끌어놨는데 다시 흩뜨리지 말라고).

### 경우 ③ Recovery 후 — camera-motion override

tracking lost → recovery 가 robot 을 새 자세로 보냄. **객체는 안 돌았지만
카메라가 움직임**. R_phys 와 무관, 우선순위 최상.

```
T_pre = T_BC_master · inv(T_BC_recovery)     ← translation × 1000 (m→mm)
```

직관: "카메라가 옮긴 만큼만 데이터를 반대로 옮겨주면 객체는 제자리".

이게 적용되면 그 iteration 의 `R_phys` hint 는 무시 (둘 다 적용하면
중복 보정).

---

## 4.4. 흐름 한 눈에

```
사용자가 첫 사진 자세 잡음
       │
       ▼
[Pass 1, pose_idx=0]  ───  T_pre = None   ──→  master ← IScan_1 (그대로)
       │                                        master_center = lock
       │
   사용자가 객체 +90° 돌리고 Enter
       │
       ▼
[Pass 2, pose_idx=1]  ───  T_pre = ② (centroid-pivot, -90°)  ──→  master ← IScan_2
       │                   hints_applied = True
       │
   tracking lost 발생!
       │
       ▼
[Pass 2-retry]  ───────  recovery 가 robot 새 자세로 보냄
                          T_pre = ③ (camera-motion)        ──→  master ← IScan_2_retry
       │
   사용자 [q]
       │
       ▼
artec_process:
   GlobalReg          ──→  hints_applied=True 면 SKIP (위 ② 가 권위자)
   Outliers / Fusion / Texturize
```

---
---

## 5. 부족면/구멍 보충 스캐닝 (NBV 루프)  ♻️🔬

현재 결과물에서 **구멍/미관측 면 검출 → NBV로 그 면 조준 → 추가 스캔 → 정합 병합** 반복.
(고전 NBV — 오래전부터 잘 작동하는 알고리즘. 조사·적용테스트 필요.)

### 재사용 부품 (♻️)
- `utils/nbv/frontier.py` (빈 영역 후보) + `icp_strategy.py` (병합) — PhoXi가 이미 이 루프 구현
- `mms_phoxi/nbv/scan_session.py` 의 Phase 2 frontier NBV 루프 패턴

### 남은 일
- 🔬 mesh hole 검출(open3d/trimesh) → NBV pose → 스캔 → 병합 루프를 Artec로
- 🔬 기존 방법 조사 및 적용 테스트

---

## 6. 공통 인프라 — Tracking watchdog + 자동 recovery  ✅

Phase 1/2 모두 같은 **Spider ↔ Turntable 양방향 피드백** 위에서 동작.

```
              TrackingState (shared)  ──  frames_ok/failed, registration_error,
                                          tracking_lost(bool), stop_event
   Main thread (session.poll_events)        TurntableController (daemon thread)
     FrameEvent 처리, tracking_lost → ←       move_velocity 연속회전, pos polling,
     stop_event.set(), rotation_done→break    stop_event 감시, finally: stop() 항상
```

### TrackingState — 4 watchdog (`artec_streaming_scan_session.py`)
| # | 조건 | 기본 |
|---|---|---|
| 1 | FrameState 연속 정합/재구성 실패 (`consecutive_loss_threshold`) | 8 |
| 2 | callback stall N초 무반응 (`stale_threshold_s`) | 2.0 |
| 3 | `registration_error < 0` 연속 (Studio 'tracking lost' 시그널) | 5 |
| 4 | `registration_error > max` 연속 (정합 품질 급락) | 8 / 1.5 |

- warm-up: scan 직후 `reg_err=-1`은 SDK sentinel. `reg_err≥0` 한 번 본 뒤(`tracking_established`)부터만 (3)(4) 카운트.

### Tracking-lost 자동 recovery (semi-auto)
같은 pose 안에서 최대 **3회** 자동 retry, 초과 시 user-prompt fallback (lost 시그널 ≠ object-presence
라 무한 retry 는 엉뚱 데이터 누적 위험). 정상 pass 나오면 카운터 0 reset.

`_attempt_recovery` 흐름 (multipass):
1. drive-alarm short-circuit (통신 사망/alarm trip → 즉시 종료, 물리 전원 안내).
2. **safe-back**: `safe_back_target_rad = (last_good_rad − final_rad) − sign·margin(10°)`
   = last-good 보다 10° 더 뒤로 turntable 복귀. `last_good_rad==0 & final≠0`(한 번도 안 잡힘) → recovery skip.
3. **`_adaptive_prescan_position(recovery=True)`** — fresh probe(회전 차분으로 물체 envelope+(r,z)
   profile 추정) → 광축 캘리브 → **축소 elevation search** (`[-5°,0°,+5°]`, fine skip) → best φ(§2 view-score)로 robot 이동.
4. `_recapture_T_BC` — 다음 merge 의 camera-motion hint(`T_pre = T_BC_master @ inv(T_BC_recovery)`, R_phys 보다 우선).

| | 첫 시작 (deprecated) | Recovery (현행) |
|---|---|---|
| trigger | scan 직전 1회 | tracking lost 시마다 |
| elevation coarse | [-10,-5,0,+5,+10] | [-5,0,+5] |
| elevation fine | best±step | skip |
| 대략 소요 | ~1분 | ~30초 |

> 옛 selector(`LocalJitterSelector`/`CentroidVectorSelector`)는 폐기. 통합 probe+elevation
> 경로가 exploitation/exploration 둘 다 대체, master point cloud 의존 제거.
> Spider v1 광학 상수(`recovery_pose_selector.py`): FOV 30°×21°, working 170~350mm(optimal 200~250),
> default standoff 250mm, 3D resolution 0.1 / accuracy 0.05mm.

---

## 7. 후처리 파이프라인 — `artec_process`  ♻️🔬

스캔 완료 후 SDK General Pipeline. **순서가 곧 코드 호출 순서** (Studio GUI 라벨과 다름).

| 순서 | 알고리즘 | 단위 | 비고 |
|---|---|---|---|
| 1 | SerialRegistration | frame-to-frame | `do_serial_registration=False` 기본 (streaming 이 이미 정합) |
| 2 | GlobalRegistration | IModel 전체 | `hints_applied=True` 면 **자동 skip** (§4 hint 가 authoritative) |
| 3a | OutliersRemoval | per-frame | **★ Fusion 전**. dev_mode 면 skip |
| 3b | SmallObjectsFilter | per-frame | **★ Fusion 전** |
| 4 | Poisson/FastFusion | clean frames → composite | watertight mesh |
| 5 | MeshSimplify | composite | 옵션. dev_mode 면 skip |
| 6 | Texturization | composite + frame tex | UV/atlas + baking → OBJ/sproj |

> ★ **Cleaning(3a/3b)은 반드시 Fusion 전.** Fusion 후에 두면 outlier 박힌 composite 가
> Texturize 까지 가서 실패(ErrorCode `0x80010203`). Phase 2 frontier 용 임시 mesh 는 `fast_fusion`.

자료구조 계층: `IFrame → IFrameMesh → IScan → IModel → ICompositeMesh`. Artec 은 SDK native
자료구조를 그대로 차용(변환/래핑 없음). streaming 모드에선 `result.ctx=None`, 외부 메타(θ, EE pose)는 timeline CSV.

---

## 8. 라이브 시각화  ✅

스캔 중 노이즈/드리프트/멈춤을 눈으로 확인하는 누적 컬러 포인트클라우드 뷰어. (결과 mesh 만으론 진단 어려움.)

- 누적 좌표 = **SDK 자신의 정합행렬** `FrameEvent.transformation` (sensor→scan-world). θ /
  turntable_frame.yaml / hand-eye 의존 **없음** → 화면 = SDK SLAM 결과 그 자체.
- 노이즈 제거 **A**: per-frame voxel(6mm) 점수<4 고립 점 제거(flying-pixel). **B**(옵션, fail-open):
  probe envelope 수직 실린더 멤버십 게이트(축 대칭이라 물체 회전해도 불변, 단일 `T_CB` 로 전 회전 게이트). B 는 표시 필터일 뿐 누적 변환 불변.
- **아키텍처(검증된 유일 구성)**: 파이프라인측은 컨트롤러만 — OK 프레임 누적 → `output/_live_latest.npy`
  atomic write. 뷰어는 **사용자가 다른 터미널에서 직접 실행** (`scripts/artec/live_scan_view.py`,
  Open3D 신형 **O3DVisualizer(Filament)**). legacy `Visualizer` 는 동적 PointCloud 못 그림, Popen 자식 Filament 창은 즉사 → 별도 터미널 필수.
- 실행: 터미널A `main_artec.py` / 터미널B `live_scan_view.py` (순서 무관, A 종료 시 B 자동 종료, 정합 끊기면 배경 빨강).

---

## 핵심 파일 맵

```
mms_artec/
  system.py                      ArtecMMS (orchestrator + calibrate_turntable_axis)
  backends/                      real/isaac 백엔드 팩토리
    isaac/{isaac_world,isaac_xarm,isaac_turntable,isaac_scanner,calib_fixture}.py
  sensor/artec_client.py         real 스캐너 (+ capture_points_base)
  nbv/artec_*_scan_session.py    Artec 스캔 세션(streaming/multipass)
utils/
  calibration/{turntable_frame,turntable_axis,rim_picker,hand_eye_calibrator}.py
  collision/{geometry,robot_collision}.py          ★ 자세별 충돌 쿼리(real/sim 공용)
  control/{theta_planner,hardware_layer}.py
  nbv/{frontier,icp_strategy,manual_picker}.py     ★ sensor-agnostic NBV 코어
  robot/{xarm_interface,xarm7_kinematics}.py    turntable/turntable_interface.py    transforms.py
mms_phoxi/nbv/{scan_session,tsdf_volume,pcd_accumulate_volume}.py   ★ NBV/병합 참조 구현
main_artec.py                    진입점 (BACKEND, RUN_CALIBRATION 토글)
scripts/sim/calib_3sphere_sim.py    3구 sim calibration 검증(축 시각화·perturb)
```

## 좌표 / 단위 규약

프레임: **B**=xArm base=world, **E**=TCP, **C**=Spider camera(≈ scan-world W; 각 ScanSession 이
첫 frame 카메라 frame 을 W 로 잡음), **F**=turntable, **O**=object. 표기 `T_AB : A→B`, `x_B = T_AB @ x_A`.

| 출처 | translation 단위 |
|---|---|
| `XArmInterface.get_ee_pose_mat()` T_EB, yaml `T_EC` | **m** |
| `xarm.set_position(x,y,z,…)` | **mm** |
| SDK frame_transformation / vertices / master pts | **mm** |

→ camera-motion T_pre 의 translation 만 `× 1000` scale (merge hint 블록).

---

## 알려진 한계 / 가정

1. **turntable_frame.yaml 미검증** — T_BF0 2026-04-23(Artec pivot 이전), rim 3점·residual 0.0.
   Phase 2 hint·NBV·recovery raycast 가 같은 T_BF0 의존 → 정밀도 의심 시 1순위 재캘리브
   (`scripts/phoxi/turntable_frame_init.py`). 라이브 뷰어는 SDK 정합행렬 사용해 이 의존 없음.
2. **tracking-lost ≠ object-presence**: 물체 제거해도 빈 디스크에 정합 성공해 lost 안 뜰 수 있음 → HYBRID + 별도 휴리스틱.
3. **last-good θ 없으면 recovery skip** (시작 직후 lost).
4. **Robot 안전성**: xArm IK/limit/self-collision 의존. 도달 불가 pose 추천 시 `set_position` 실패 → 재시도.
5. **Raycast occlusion 무시** (frustum culling 만; close-range 라 영향 작음).
6. **비대칭·길쭉 객체** centroid hint 가정 취약(§3) — 향후 OBB center.
7. **Console cp949**: 이모지 깨짐 — utf-8/PowerShell 터미널 정상.

---

## 환경 / 재현 체크리스트

- Artec SDK 1.17.3, 스캐너 Spider `SP.10.36181288`. xArm `192.168.1.210`,
  Turntable `192.168.0.10` **UDP**. (개발 환경: Windows 11 PowerShell, conda `mms-env`.)
- 캘리브 파일: `config/sensor_frames.yaml`(`T_EC_artec`), `config/calibration/turntable_frame.yaml`(§알려진 한계 1 주의).
- binding 변경 시: `cmake --build mms_artec/sensor/build --config Release --target <module>`.
- 콘솔 인코딩: `PYTHONIOENCODING=utf-8` (cp949 이모지 깨짐 회피).
- 실행:
  ```
  python main_artec.py                          # 메인 (BACKEND, RUN_CALIBRATION 토글)
  python scripts/artec/live_scan_view.py        # 라이브 뷰어 (별도 터미널)
  python scripts/artec/merge_compare.py [--load output/scan_raw/<TS>]  # 병합 variant 비교
  python -c "import main_artec"                  # import smoke test
  ```
- raw scan 저장(`output/scan_raw/<TS>/`): `master.sproj`(IScan raw frame_transformations) +
  `meta.npz`(`scan_indices/T_pres/T_BC/T_CB/n_scans`). `--load` 로 scan 없이 후처리 반복 실험.

---

## 상태 요약

| 단계 | 상태 | 비고 |
|---|---|---|
| 0. sim/real 백엔드 | ✅ | robot/turntable/scanner 전환, Phase A(모션) 검증 |
| 1. auto-calibration | ✅ | hand-eye(ChArUco+solvePnP, 3.55mm) + 3구 자동(0.03°/0.6mm) + rim 3점클릭(Isaac, 0.015°/0.7mm) |
| 2. 5면 Phase1 streaming | ✅🔬 | streaming SLAM + elevation view-score 구현, sim view-planning 검증 남음 |
| 3. 아랫면(flip 병합) | ♻️🔬 | multipass 설계 존재, 병합 검증 필요 |
| 4. 2+3 병합(T_pre hint) | ♻️🔬 | centroid-pivot hint + ICP/GlobalReg/누적 부품 존재 |
| 5. 구멍 보충 NBV | ♻️🔬 | PhoXi frontier 루프 재사용, Artec 적용+조사 |
| 6. watchdog + recovery | ✅ | 4 watchdog + 3-retry 자동 recovery(probe+축소 elevation) |
| 7. 후처리 artec_process | ♻️🔬 | SDK General Pipeline(cleaning→fusion 순서), Artec 통합 검증 |
| 8. 라이브 시각화 | ✅ | SDK 정합행렬 누적, Filament 뷰어(별도 터미널) |

**개발 원칙**: 모든 로직은 sim(ground-truth)에서 개발·검증 → real(Artec SLAM)에 동일 코드 적용.
sensor-agnostic 코어(`utils/`)를 PhoXi·Artec·Isaac이 공유.
