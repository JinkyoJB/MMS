# MMS 코드 리뷰 — real / sim 2갈래 구조 & real 실행 경로 정리

> 목적: 지금까지 구현된 코드를 **읽기 쉽게** 정리해 ① real/sim 두 갈래가 **공용 lib**을
> 제대로 공유하는지 확인하고 ② **real 경로(`main_artec.py` → calibration → Phase1 → Phase2)**
> 를 Spider 도착 즉시 테스트할 수 있게 한다.
>
> 관련: `docs/main_flow.md`(큰 그림), `docs/1_calibration.md`, `docs/2_phase1.md`, `docs/3_phase2.md`.

---

## 0. 한 장 요약

```
                     ┌───────────────── 공용(real/sim 동일) ─────────────────┐
 main_artec.py  →    │  ArtecMMS(system.py)  =  오케스트레이션 단일 진입       │
 BACKEND 토글        │   ├ create_hardware()      → backend factory 분기       │
 ("real"|"isaac")    │   ├ calibrate_turntable_axis()  (utils.calibration)     │
                     │   └ artec_process()        → scan + 후처리              │
                     └────────────────────────────────────────────────────────┘
                          │ backend 차이는 여기 두 곳에만 격리 ↓
        ┌─────────────────────────────┬──────────────────────────────────┐
   build_hardware/build_sensor       artec_process 안 scan 분기
   (robot/turntable/scanner 교체)     (isaac=sim누적 / real=Artec SLAM)
```

**핵심 설계**: orchestration·연산·플래닝은 전부 **공용**(`utils/`, `mms_artec/system.py`,
`mms_artec/nbv/*`). 하드웨어/시뮬 의존은 **factory + scan 분기 + lazy import** 로만 갈린다.
→ 같은 코드를 sim 으로 개발/검증하고 real 에 그대로 올린다.

---

## 1. real / sim 갈래는 어디서 갈리나 (딱 두 곳)

### (a) 하드웨어 factory — `mms_artec/backends/__init__.py`
`build_hardware(cfg)` / `build_sensor(cfg)` 가 `cfg.backend` 로 분기 (**lazy import**):

| 부품 | real | isaac(sim) | 공용 계약(인터페이스) |
|---|---|---|---|
| robot | `utils.robot.XArmInterface` | `backends/isaac/isaac_xarm.IsaacXArm` | `ik/fk/get_ee_pose_mat/get_joint_angles/enable_motion/get_pose` + `.arm.set_servo_angle/set_position` |
| turntable | `utils.turntable.Turntable` (Ezi-SERVO) | `isaac.IsaacTurntable` | `move_abs/move_velocity/getActualPos/wait_motion_done/stop` |
| scanner | `mms_artec.sensor.ArtecClient` (Artec SDK) | `isaac.IsaacArtecScanner` | `build_sensor` 가 생성, `mms.sensor` 로 접근 |

> ★ **robot 의 IK/FK 는 양쪽 다 `utils/robot/xarm7_kinematics.py`(해석 IK)** 를 쓴다.
> real `XArmInterface.ik` 도 SDK IK 가 아니라 해석 IK 호출(결정 2026-06, [[decision-ik-analytic-not-sdk]]).
> → **충돌검사(스캐너 자가충돌 포함)·IK·NBV 자세선택이 real/sim 동일 결과**. 캡처도 "로봇
> 자세+전회전"으로 통일(2026-06-30). backend 는 카메라규약(USD/OpenCV)·프레임만 주입.

### (b) 스캔 경로 — `mms_artec/system.py::artec_process()`
| backend | scan 방법 | 코드 |
|---|---|---|
| isaac | 턴테이블 GT θ + 카메라로 점군 직접 누적(SLAM 없음) | `backends/isaac/isaac_scan_session.IsaacScanSession` |
| real | Artec **streaming SLAM**(IScanningProcedure) | `nbv/artec_streaming_scan_session` → `nbv/artec_multipass_scan_session` |

스캔 이후 **후처리(GlobalReg/Fusion/Texturize)는 real 전용**(Artec SDK = `mms.sensor.*`).
sim 은 점군까지만.

---

## 2. 공용 라이브러리 지도 (`utils/`) — real/sim 모두 사용

| 모듈 | 역할 | 사용처 |
|---|---|---|
| `utils/transforms.py` | SE3·`solve_T_EB`·`TurntableTransformConfig` | system, planner, NBV |
| `utils/robot/xarm7_kinematics.py` | **해석 FK/IK**(DLS), 링크원점 | XArmInterface·IsaacXArm·충돌·planner |
| `utils/robot/xarm_interface.py` | real xArm(모션=SDK `set_servo_angle`, IK=해석) | real robot |
| `utils/robot/ik_provider.py` | pluggable IK(`use_sdk=False` 기본) | calibration·sim |
| `utils/collision/robot_collision.py` | **CollisionWorld·캡슐·swept·self·keep-out**. `pose_collision(scanner_self=True)` = **스캐너↔베이스링크 자가충돌** 포함, 스캐너 캡슐 반경 `DEFAULT_LINK_RADII[6]=0.095`(스파이더 외형) | **NBV 충돌검사(공용, real=sim)** |
| `utils/collision/geometry.py` | seg-seg/halfspace 거리(numpy) | robot_collision |
| `utils/control/theta_planner.py` | θ 최적화 (SDK판 + **해석판 `_analytic`**) | NBV·execute |
| `utils/control/hardware_layer.py` | `execute_camera_target`·pose 변환 | system |
| `utils/nbv/frontier.py` | 메쉬 경계→구멍 후보 | Phase2 gap 검출 |
| `utils/nbv/manual_picker.py` | 법선→카메라 포즈 | Phase2 NBV 포즈 |
| `utils/nbv/phase2_nbv.py` | **Phase2 코어**(pcd→mesh→gap→커버리지→pose) | Phase2(공용, open3d) |
| `utils/nbv/icp_strategy.py` | ICP gate/roll | 병합 |
| `utils/calibration/{turntable_axis,turntable_frame,hand_eye_calibrator}.py` | 축·프레임·hand-eye 피팅 | calibration(공용) |

> ⚠ **Isaac 하니스(`MMS_ext_*.py`)만 예외**: Isaac 의 `utils` 패키지명 충돌로 `robot_collision`/
> `phase2_nbv` 를 직접 import 못 함 → **자기완결 모듈(kin/geo)만 로드 + 나머지 인라인 미러**.
> 이 인라인은 **sim 검증 전용**이고, real 에 적용되는 본체는 위 공용 lib 이다(혼동 주의).

---

## 3. real 실행 경로 — `main_artec.py` 단계별

`BACKEND = "real"` (line 21). 실행: Windows + xArm/Artec/Ezi-SERVO 연결 후 `python main_artec.py`.

```
main()                                                  # main_artec.py:317
 └ with ArtecMMS(CFG) as mms:                           # system.py:95  (build_sensor=scanner 생성)
     ├ robot, turntable = mms.create_hardware()         # system.py:146 → build_hardware (factory)
     │  (T_B_F0 = turntable_frame.yaml 자동 로드; 재보정은 rim-click 별도 스크립트 — §4.1)
     └ result = mms.artec_process(robot, turntable, PROCESS_SETTINGS)   # system.py:411
          ├ [scan] real+multipass → ArtecMultiPassScanSession.run()     # system.py:436-459
          │         = phase_mode 순차 누적: Phase 1(streaming) → 2(NBV) → 3(flip 바닥면)
          ├ GlobalRegistration (hints_applied 면 skip)                  # system.py:512
          ├ OutliersRemoval / SmallObjectsFilter                        # system.py:517
          ├ PoissonFusion                                               # system.py:525
          ├ Texturize                                                   # system.py:533
          └ Export .obj/.sproj                                          # system.py:537
```

**설정은 `main_artec.py` 상단에서** (CFG, STREAM_SETTINGS, MULTIPASS_SETTINGS, PROCESS_SETTINGS).
- 하드웨어 IP: `ROBOT_IP/TURNTABLE_IP` (line 53-55).
- hand-eye: `T_EC_key="T_EC_artec"`, `sensor_frames.yaml` (line 69-70).
- 턴테이블 축: `config/calibration/turntable_frame.yaml` (line 68).

---

## 4. 단계별 상세 (real 기준)

### 4.1 Calibration  (docs/1_calibration.md)
세 변환을 구해 저장. **연산은 공용 `utils/calibration/*`**, 데이터만 real/sim 다름.

| 변환 | 의미 | 구하는 법 | 저장 |
|---|---|---|---|
| `T_EC` (hand-eye) | EE→카메라 | ChArUco+solvePnP → `hand_eye_calibrator`(AX=ZB). ✅2026-04-29 (3.55mm) | `config/sensor_frames.yaml` |
| `T_B_F0` | base→턴테이블축 | **rim 클릭**(턴테이블 가장자리 점 ≥3개)→`turntable_frame.fit_circle_3d`→`build_T_B_F0` | `config/calibration/turntable_frame.yaml` |
| `T_O_F0` | 내부글로벌→턴테이블 | 런타임 체인 | 세션 메모리 |

- real hand-eye 재보정: `scripts/artec/hand_eye_calib.py`.
- 턴테이블 축 재보정 = **rim 클릭 별도 스크립트**(2026-06 결정, 구 sphere fixture 폐기):
  - real: `python scripts/artec/turntable_frame_init.py` → rim 클릭 → yaml.
  - sim 검증: `sim_harness/MMS_ext_calibration2.py`(rim 자동추출→fit→GT).
  - main_artec 은 **yaml 만 로드**(인라인 sphere 캘리브 제거됨). 코어=`turntable_frame.py`.
- ✅ 폐기 코드 제거 완료(2026-09): `turntable_axis.py`·`calib_3sphere_sim.py`·`calib_fixture.py`·
  `system.py::calibrate_turntable_axis`(sphere). `axis_error` 만 `turntable_frame.py` 로 이관.

### 4.2 Phase 1 — 5면 streaming SLAM  (docs/2_phase1.md)
턴테이블 360° 회전 + **로봇 고정** → 윗면+옆면4. real 은 Artec SLAM, sim 은 GT 누적.

- 본체: `mms_artec/nbv/artec_streaming_scan_session.py` (`ArtecStreamingScanSession.run`).
  연속회전 + max FPS, **HYBRID 정합**, 4-watchdog(tracking-lost 감시), 턴테이블 UDP.
- 오케스트레이션: `artec_multipass_scan_session.py` (Phase1 pass + recovery).
- ✅ 구현 완료. real 핵심값: `STREAM_SETTINGS`(rotation_duration_s=30, HYBRID, reset_to_zero).

### 4.3 Phase 2 — 부족면 NBV 보강  (docs/3_phase2.md)  🛠 신규
**5면 중 부족 영역**을 **로봇이 움직이며** NBV 로 보강. (바닥면은 Phase 2 아님 → §4.4 Phase 3.)
`phase_mode=2` 로 활성.

- 본체: `artec_multipass_scan_session.py` 의 `_phase2_nbv_loop` → `_build_master_mesh_B`
  (pcd→Poisson) → **`_plan_nbv_pose`**(공용 `phase2_nbv.plan_nbv_elevation_pose` — 관측
  elevation 자세, 해석 IK + swept 충돌 + 관절이동 최소) → `_capture_nbv_pose`(**로봇 NBV 자세
  이동 → streaming 전회전 → T_pre/relocalization 병합**). (`_rank_nbv_candidates` per-gap 정면
  방식은 캡처통일로 대체됨.)
- 충돌: `_build_collision_world`(턴테이블 calib) + **keep-out 원기둥**(`add_cylinder`) +
  `swept_pose_collision`(공용 `robot_collision`). ★ **스캐너 자가충돌 자동 포함**(공용
  `pose_collision(scanner_self=True)`, 캡슐반경 0.095) — 벌크 스파이더가 베이스 링크(link2 등)에
  닿는 자세를 reject. real 파일 무수정으로 적용됨(2026-06-30).
- 코어 알고리즘은 `utils/nbv/phase2_nbv.py`(공용, real/sim 동일).
- ★ **캡처 방식 통일(2026-06-30)**: sim `IsaacScanSession` 도 **real 모달리티**를 따름 —
  `_scan_pass`(로봇 한 자세 + **턴테이블 전회전** + 프레임마다 −θ 누적) = real `_capture_nbv_pose`
  (robot pose + streaming 전회전 + relocalization)와 1:1 대응. Phase1·Phase2 **동일 `_scan_pass`**.
  sim 의 **NBV = "다음 관측 elevation 자세" 선택**(`_plan_nbv_pose`: gap 법선 elevation→필요 el,
  도달·충돌-free 중 **최고 el** 채택 → 윗면 보강), 그 자세에서 전회전. (이전의 단일프레임·θ정면화·
  approach-cone 플래너는 폐기.) ⚠ 윗면 풀커버는 el≈90° 필요하나 **스캐너-link2 자가충돌로 도달한계**
  → 크고 높은 객체는 **z 수축** 필요.
- sim 경로: `IsaacScanSession`(production, `main_artec.py` isaac) / `MMS_ext_phase2_nbv.py`(하니스).

### 4.4 Phase 3 — 바닥면 (180° flip)  (기존 flip 흐름, docs/3_phase2.md §8)  ♻️
디스크에 닿아 Phase 1·2 가 못 잡는 **바닥면**을 사용자가 객체를 뒤집어 추가 스캔.
**이전엔 "Phase 2"로 불렸으나 본 설계에서 Phase 3 로 분리**(Phase 2 = NBV 로 재정의).

- `phase_mode=3`(기본 — Phase 3 바닥면): 사용자가 객체 회전 → 추가 streaming pass →
  centroid-pivot `T_pre` hint 로 병합. (`pose_physical_rotations` = 이 손회전 설정)
- main_artec: `POSE_ROTATIONS = make_axis_physical_rotations("y",[0,90,180])` →
  `MULTIPASS_SETTINGS.pose_physical_rotations`. = **Phase 3(바닥면) 설정**이다.
- 토글 = **`phase_mode`**(정수, **순차 누적** = Phase 1..N): **1=Phase1 · 2=Phase1→2(NBV) ·
  3=Phase1→2→3(바닥면 flip)**. 기본 3. Phase 2→3 전환 시 robot `go_home` 후 flip.

### 4.5 후처리 — `artec_process` (real 전용, Artec SDK)
GlobalReg → Cleaning → PoissonFusion → Texturize → Export(.obj/.sproj). `mms.sensor.*`.
(isaac 백엔드에선 SDK 후처리 skip — `IsaacScanSession` 결과 점군/mesh 그대로 반환.)

---

## 5. real/sim 공용성 — 검증 결과

| 기능 | 공용 lib | real 사용 | sim 사용 | 비고 |
|---|---|---|---|---|
| 해석 IK/FK | `xarm7_kinematics` | ✅ | ✅ | 동일 결과 |
| 충돌(world/swept/self/keepout/**스캐너 자가충돌**) | `robot_collision` | ✅ | ✅ production(`IsaacScanSession`) / ⚠ 하니스만 인라인 미러 | **스캐너 자가충돌도 공용**(2026-06-30) → real=sim |
| θ 플래너(해석) | `theta_planner._analytic` | ✅ | ✅ | |
| frontier/NBV pose | `frontier`,`manual_picker`,`phase2_nbv` | ✅ | ⚠ 하니스 인라인 | open3d 필요 |
| **캡처 모달리티** | (개념 공용) | streaming 전회전 | `_scan_pass` 전회전(GT θ=relocalization) | **로봇 pose+전회전으로 통일**(2026-06-30) |
| calibration 피팅 | `utils/calibration/*` | ✅ | ✅ | |
| 오케스트레이션 | `system.py`,`artec_multipass_*` | ✅ | ✅ | scan만 분기 |

**결론**: 충돌(스캐너 포함)·IK·NBV·calibration 로직은 **공용 lib 에 단일 구현** → real/sim 공유 OK.
**캡처도 "로봇 자세 + 턴테이블 전회전"으로 통일** → sim 검증이 real 에 직접 이어짐. **NBV 자세
선택도 공용**(`phase2_nbv.plan_nbv_elevation_pose`, real·sim 둘 다 호출 — backend 는 look-at
규약(USD/OpenCV)·프레임만 주입). 유일한 예외는 **Isaac 하니스(`MMS_ext_*`)의 인라인 미러**
(Isaac import 제약). production sim·real 본체는 공용 lib 단일 구현.

---

## 6. Spider 도착 시 real 테스트 체크리스트

> 순서대로. 각 단계 독립 검증 후 다음으로.

1. **[연결]** `main_artec.py` `BACKEND="real"`, `ROBOT_IP/TURNTABLE_IP` 확인. `python main_artec.py`
   → `create_hardware()` 가 robot/turntable 연결되는지(예외 없이).
2. **[hand-eye]** `config/sensor_frames.yaml::T_EC_artec` 존재 확인(2026-04-29). 센서 교체했으면
   `scripts/artec/hand_eye_calib.py` 재실행.
3. **[턴테이블 축]** `turntable_frame.yaml` stale 의심 시 **rim-click 재보정**:
   `python scripts/artec/turntable_frame_init.py` → 턴테이블 가장자리 점 클릭 → yaml 갱신.
4. **[Phase 1]** `MULTIPASS_SETTINGS` 그대로(또는 `prompt_*`=False 자동화) → 1 pass 5면 스캔.
   live viewer 로 정합 품질 확인. export .obj 확인.
5. **[Phase 3 — flip 바닥면]** 기존 경로(`phase_mode=3`): pose 회전 prompt 따라 바닥면 추가.
6. **[Phase 2 — NBV(신규)]** `MULTIPASS_SETTINGS` 에 **`phase_mode=2`** + 충돌 world 치수
   (`nbv_turntable_radius_mm` 등 실측), keep-out 옵션 설정 → 부족면 자동 보강.
   ⚠ 아래 §7 미해결 항목(스윕/relocalization) 먼저 점검. (스캐너 자가충돌은 공용 해결됨 —
   실측 스파이더 반경으로 `DEFAULT_LINK_RADII[6]` 만 미세조정.)

---

## 7. 알려진 미완성 / real 전환 시 점검 (★중요)

| 항목 | 상태 | 조치 |
|---|---|---|
| `phase_mode=2` | 코드 있음, **main_artec 미설정**(기본 flip) | MULTIPASS_SETTINGS 에 추가해야 NBV 동작 |
| 스캐너 자가충돌 | ✅ **해결**(공용 `pose_collision(scanner_self=True)`, 반경 0.095) | real 실측 스파이더 반경으로 `DEFAULT_LINK_RADII[6]` 미세조정 |
| 캡처 모달리티 통일 | ✅ sim `_scan_pass`=real 전회전 대응 | — (real `_capture_nbv_pose`=streaming 전회전 그대로) |
| NBV 자세선택 공용추출 | ✅ **해결**(공용 `phase2_nbv.plan_nbv_elevation_pose`, real·sim 둘 다 호출. backend 는 look-at 규약(USD/OpenCV)·프레임만 주입) | real 실기에서 elevation 선택 동작 확인 |
| NBV 캡처 = 짧은 스윕 | v1=풀회전 | `streaming_settings` 회전각 override(시간단축용) |
| Artec relocalization R1/R2 | 미배선(R3 fallback) | 실기 검증 후 배선(docs/3 §6.5) |
| `set_servo_angle` speed 단위 | sim 가정 | real 에서 deg/s 확인 |
| 충돌 world 치수 | 기본값 | 셀 실측으로 `nbv_turntable_*`/keep-out 보정 |
| 큰/높은 객체 윗면 | el≈90° 도달불가(스캐너-link2) | **z 수축** 또는 Phase 3 류 별도 처리 |
| open3d (phase2_nbv) | ✅ 양쪽 설치됨 | `mms-env` 0.19 / `env_isaacsim` 0.19 확인(2026-08-12). Phase2 NBV mesh/gap 용 |
| opencv 버전 | ⚠ **`<5` 고정 필요** | OpenCV 5.x 는 `cv2.calibrateHandEye` 가 python 바인딩에 없어 hand-eye 가 깨진다(상수만 남아 import 는 통과 → 발견이 늦다). `requirements.txt` 에 `opencv-python>=4.9,<5` 명시 |
| 후처리 hints/GlobalReg skip | flip 경로용 | NBV 경로의 병합 규칙 점검 |

> 이 표가 곧 "Phase 2 NBV 를 real 에서 켜기 전 할 일" 목록이다. Phase 1·calibration·flip 은
> 기존 검증 경로라 Spider 도착 즉시 테스트 가능.

---

## 8. 파일 빠른 참조

| 보고 싶은 것 | 파일 |
|---|---|
| real 진입/설정 | `main_artec.py` |
| 오케스트레이션(공용) | `mms_artec/system.py` (`artec_process`, `calibrate_turntable_axis`) |
| backend factory | `mms_artec/backends/__init__.py` |
| Phase1 streaming | `mms_artec/nbv/artec_streaming_scan_session.py` |
| Phase1+2 오케스트레이션 | `mms_artec/nbv/artec_multipass_scan_session.py` |
| Phase2 코어(공용) | `utils/nbv/phase2_nbv.py` |
| 충돌(공용) | `utils/collision/robot_collision.py` |
| 해석 IK | `utils/robot/xarm7_kinematics.py` |
| θ 플래너 | `utils/control/theta_planner.py` |
| sim 하니스 | `sim_harness/MMS_ext_{calibration,phase1,phase2_nbv}.py` |
