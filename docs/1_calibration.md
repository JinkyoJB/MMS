# Calibration — Hand-Eye(`T_EC`) & Turntable(`T_B_F0`)

> MMS 의 두 가지 캘리브를 한 문서에. *무엇을·왜·어떻게* + *어느 함수가 무슨 일을 하는지*.
> 상위 맥락은 `docs/main_flow.md` §1.
> - **Part 1 — Hand-Eye `T_EC`**: 카메라가 로봇 손목(EE)에 어떻게 붙어있나.
> - **Part 2 — Turntable `T_B_F0`**: 턴테이블이 로봇 base 기준 어디서·어느 축으로 도나.

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

## 2. 핵심 아이디어 — 왜 풀리나 (AX = ZB)

매 자세 i 에서 **체인**이 성립한다 (보드의 base 위치는 안 변함):

```
   T_B_M  =  T_BE(i) · T_EC · T_MC(i)        (∀ i, 우변이 항상 같은 값)
   └고정┘    └로봇 FK┘ └미지┘  └이미지 검출┘
```

- `T_BE(i)` : 로봇 FK 로 안다 (손목이 base 기준 어디).
- `T_MC(i)` : 이미지에서 보드를 검출해 안다 (보드가 카메라에 어떻게).
- `T_B_M`   : 보드의 base 위치 — 모르지만 **모든 i 에서 동일**.
- `T_EC`    : 미지, **모든 i 에서 동일**.

두 개의 미지(`T_EC`, `T_B_M`)가 여러 자세에 걸쳐 동시에 만족돼야 하므로, 자세를
**충분히·다양하게**(특히 회전 다양성 ≥30°) 모으면 유일하게 풀린다.
이게 OpenCV `cv2.calibrateHandEye` (AX=ZB).

> **그래서 필요한 입력은 자세마다 딱 2개:** `T_BE`(로봇 FK) + `T_MC`(보드 검출). 끝.

---

## 3. 전체 흐름 (한눈에)

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

이 **가운데(①~⑤ + 풀이)는 sim·real 이 똑같은 MMS 라이브러리 코드**를 쓴다.
다른 건 "자세를 어떻게 만들고 캡처하느냐"의 **껍데기**뿐:

| | 자세 만들기 ① | 캡처 ② | FK ④ |
|---|---|---|---|
| **real** | 사람이 teach / yaml 자세 순회 | Artec 실기 캡처 | xArm SDK |
| **sim** | IK 로 반구 자세 생성·구동 | Isaac 카메라 렌더 | sim 아티큘레이션 |

---

## 4. 단계별 — 담당 함수

| 단계 | 하는 일 | 함수 / 위치 |
|---|---|---|
| 보드 정의 | 물리 사양(5×3, 20/15mm, DICT_4X4_50) | `CharucoBoardSpec` — `mms_artec/utils/calibration/artec_charuco_detector.py` |
| ① 자세 생성 | 보드 위 반구 자세 N개 → EE pose | `generate_hemisphere_poses` — `mms_artec/utils/calibration/handeye_geometry.py` |
| ① 자세 이동(IK) | pose6d → 관절각 (**자체 해석**, SDK 미사용) | `RobotIK.ik` — `utils/robot/ik_provider.py` |
| ② 캡처 | 이미지 획득 | (sim) Isaac `camera.get_rgba` / (real) Artec |
| ③ 검출+solvePnP | 이미지 → `T_MC`(보드→카메라, mm) | `ArtecCharucoDetector.detect` — `artec_charuco_detector.py` |
| ④ FK | 손목 pose `T_BE` (m) | (sim) `rigid_ee` / (real) `XArmInterface.get_ee_pose_mat` |
| ⑤ 샘플 누적 | `(T_BE, T_MC)` 저장 | `HandEyeCalibrator.add_sample` — `utils/calibration/hand_eye_calibrator.py` |
| 풀이 | 5-method 중 잔차 최소 → `T_EC` | `HandEyeCalibrator.calibrate` |
| 저장 | yaml | `HandEyeCalibrator.save_yaml` |

---

## 5. 코드 지도 (어디에 뭐가 있나)

### 공유 라이브러리 — sim·real 공통 (★ 핵심)
```
utils/calibration/hand_eye_calibrator.py
    HandEyeCalibrator
      .add_sample(T_EB, T_MC)   # T_EB: E→B(m), T_MC: M→C(mm) — 내부서 mm→m
      .calibrate() → T_EC       # TSAI/PARK/HORAUD/ANDREFF/DANIILIDIS 중 잔차 최소
      .n_samples (property), .save_yaml(...)

mms_artec/utils/calibration/artec_charuco_detector.py
    CharucoBoardSpec(squares_x, squares_y, square_length_mm, marker_length_mm, aruco_dict)
    ArtecCharucoDetector(board_spec, intrinsic={"K","dist"})
      .detect(image_rgb, min_corners, draw_debug) → CharucoDetection
          .T_MC (4×4, mm, 보드→카메라, OpenCV cam)  .n_corners  .debug_image  .procrustes_rmse_mm
      # intrinsic(K) 있으면 solvePnP 경로(권장, sub-pixel), 없으면 UV→3D Procrustes

mms_artec/utils/calibration/handeye_geometry.py   (numpy 전용, 자기완결)
    make_T, inv_T, quat_wxyz_to_R, rot_about_axis, rot_angle_deg, R_to_euler_xyz
    mat_to_pose6d_mm(T)                 # 동차변환 → [x,y,z mm, rpy]
    look_at_camera(eye, center, up, roll)
    generate_hemisphere_poses(center, normal, T_EC, distance, polars, azis, rolls, jitter, up_hint)

utils/robot/ik_provider.py            (자기완결, kin 주입)
    reachable(ip)                      # 컨트롤러 소켓 도달성
    RobotIK(kin, robot_ip, use_sdk=False).ik(pose6d, seed) → (q, ok)
      # 기본 = 자체 해석 kin IK (SDK IK 미사용 — 컨트롤러 통신이라 불안정)

utils/robot/xarm7_kinematics.py        (해석 운동학, numpy 전용)
    fk_T / fk_pose6d / ik(pose6d, seed) / R_to_euler_xyz ...

utils/transforms.py
    compute_T_CB(T_EB, T_EC) = T_EB @ inv(T_EC)     # ★ T_EC 규약의 출처
```

### 실물 파이프라인 (하드웨어)
```
scripts/artec/make_charuco.py       # 보드 PNG 생성(인쇄용)
scripts/artec/intrinsic_calib.py    # 카메라 K 측정 (1회)
scripts/artec/hand_eye_calib.py     # 메인 루프: 자세순회→detect→add_sample→calibrate→save
utils/robot/xarm_interface.py       # XArmInterface (실물 로봇 = xArm SDK)
config/sensor_frames.yaml           # 결과 T_EC_artec 적용처
```

### sim 검증 하니스 (Isaac 전용 — 이 안엔 USD/Isaac 코드만)
```
standalone_examples/play/MMS/MMS_ext_calibration.py
    setup_async()              # USD 열기, 마블 비활성, 보드 rigid 생성, 카메라/로봇/검출기/솔버 셋업
    _on_physics_step()         # 상태머신: BOARD_SETTLE → (자세 i) SETTLE → SOLVE → DONE
    _finalize_board_and_poses()# 보드 안착 후 실제 pose 읽어 자세 생성
    build_calibration_poses()  # generate_hemisphere_poses 에 config 주입(thin wrapper)
    _start_pose(i)             # 목표 EE pose → mat_to_pose6d_mm → RobotIK.ik → drive_joints
    _do_capture(i)             # get_rgba → detector.detect → calibrator.add_sample
    solve_and_report()         # calibrator.calibrate → GT 비교(t_err/r_err) → npz 저장
    drive_joints / get_camera_K / get_prim_world_T / create_charuco_board_rigid ...  # USD/Isaac
```

---

## 6. 꼭 알아야 할 규약·함정

1. **`T_EC` = E→C = EE-in-camera** (`x_C = T_EC·x_E`). calibrator 가 반환하는 것도 이 규약.
   - sim GT(정답) = `inv(T_W_C) @ T_W_E`. ← 이걸 거꾸로 잡으면 결과가 멀쩡한데도 틀려 보임.
2. **카메라 프레임 USD vs OpenCV** — USD/Isaac 카메라는 광축 **-Z·+Y up**, solvePnP(OpenCV)는
   **+Z·+Y down**. 둘은 `R_FLIP=diag(1,-1,-1)`. `T_MC`/`T_EC` 가 OpenCV 프레임이므로 USD GT 와
   비교할 땐 **`T_EC_usd = R_FLIP @ T_EC_ocv`** (T_EC 는 카메라가 출력측 → flip 은 **왼쪽곱**).
   *(실물엔 USD 가 없으니 이 flip 은 sim 검증 전용.)*
3. **단위** — `T_BE`: translation **m** / `T_MC`: translation **mm** (보드 사양이 mm 라 solvePnP
   tvec 도 mm). `add_sample` 이 내부에서 `T_MC` 를 mm→m 변환. 섞으면 병진 오차 폭발.
4. **자세 다양성** — 병진 정확도는 자세 간 **회전 다양성**에 좌우. 거의 수직으로만 내려다보면
   회전축이 비슷해 병진이 부정확. polar/roll 범위를 넓혀야 함.
5. **IK = 자체 해석 운동학** — real·sim 모두 `xarm7_kinematics`(수치 DLS). xArm SDK IK 는
   컨트롤러 통신이라 하드웨어 연결 필요·불안정 → **미사용**(`use_sdk=False` 기본). 모션 명령만 SDK.
6. **(sim) `utils` 패키지명 충돌** — Isaac 런타임에 동명 `utils` 가 있어 `from utils...` 가 깨짐.
   → MMS 모듈을 **파일경로 로드**(`sys.modules` 등록 필수, `@dataclass` 때문). 옮길 모듈은
   레포 내부 import 없는 **자기완결**이어야 함(`handeye_geometry`/`ik_provider`/`xarm7_kinematics`).

---

## 7. 결과 해석 (sim)

`solve_and_report` 가 GT 대비 **t_err(mm) / r_err(°)** 출력 + `captures_calib/handeye_result.npz` 저장.
디버그 이미지 `ok_NN.png`(검출 성공, 코너 표시) / `fail_NN.png`(실패).

- 무왜곡 sim 이면 검출 노이즈·자세 다양성만이 오차원 → **작을수록 좋음**(목표 t<5mm, r<2°).
- `t_err` 큰데 `r_err` 작음 → 자세 회전 다양성 부족(§6-4) 의심.
- 검출 `fail` 많음 → 자세가 너무 비스듬/보드 시야 이탈.

> 실물 검증은 GT 가 없으니 calibrator 내부 **잔차**(_compute_residuals, T_B_M 일관성)와
> point-consistency(고정점을 여러 자세서 base 로 변환 후 산포)로 본다.

---

## 8. 실행

**sim (Isaac):** VSCode Isaac 확장 코드러너 또는 Script Editor 로
`standalone_examples/play/MMS/MMS_ext_calibration.py` 실행. 로그:
`tail -f standalone_examples/play/MMS/captures_calib/calib_log.txt`.

**real:**
```
python scripts/artec/make_charuco.py        # 보드 인쇄
python scripts/artec/intrinsic_calib.py     # K 측정 (1회)
python scripts/artec/hand_eye_calib.py      # 자세 순회 → T_EC → hand_eye_artec.yaml
# → config/sensor_frames.yaml 의 T_EC_artec 에 반영
```

> 실물 결과(2026-04-29): PARK, **t_err 3.55mm / r_err 1.30°** (main_flow §1.0).

---
---

# 〔Part 2〕 Turntable — `T_B_F0`

> 로봇 base 기준 **턴테이블 회전축·표면**(= `T_B_F0`)을 구한다. 방법은 **rim 점 피팅** 하나로 통일
> (구 어레이 방법은 실물 fixture 비용이 커서 채택 안 함 — rim 으로 대체 가능).

## 9. 무엇을 구하나 — `T_B_F0`

| 프레임 | 의미 |
|---|---|
| **B** | 로봇 base (월드) |
| **F** | 턴테이블 프레임 (원점=회전축이 disc **표면**과 만나는 점, z=회전축, θ=0 기준) |

구하려는 값: **`T_B_F0`** = "턴테이블이 base 기준 어디서·어느 축으로 도나". 한 번 구하면 하드웨어
이동 전까지 상수 → `config/calibration/turntable_frame.yaml` 저장.

> ★ 규약: `T_B_F0` 는 **B→F** (`x_F = T_B_F0·x_B`). F 의 z = 회전축(위쪽), 원점 = 표면 위 축점.

왜 필요 — Phase 2 hint·NBV·recovery·충돌회피가 전부 "턴테이블이 base 기준 어디서 도나"에 의존.
하드웨어팀이 옮기면 무효화 → **버튼 하나로 다시 잡는** 루틴.

## 10. 핵심 원리 — "회전하면 원을 그린다"

> 회전판에 고정된 점은 회전축 둘레로 **원**을 그린다 → 원 법선 = 축방향, 중심 = 축 위 한 점.

disc rim(가장자리)은 그 자체가 축 둘레의 원 → rim 위 점들을 3D 로 모아 원을 피팅하면 축이 나온다.

> ★ **축 ≠ 표면**: rim/궤적 높이 ≠ disc 표면 높이일 수 있음. 충돌회피·대상물 높이를 위해 disc
> **표면 평면**을 따로 잡아 축선과 만나는 점을 F0 원점으로 삼는다(§12).

## 11. Rim 방법 — 흐름 + 함수

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
   (+ disc 표면 평면, §12) → build_T_B_F0(center, normal) → T_B_F0
```

- UI/수학: `utils/calibration/`
  - `rim_picker.py` — `RimPicker(intensity, organized_pts, T_CB)` + `run_picker` (OpenCV 클릭:
    LClick=추가/RClick=취소/Enter=피팅), `show_3d_result` (Open3D). pixel→base 3D 내장.
  - `turntable_frame.py::fit_circle_3d(pts)` → `(center, normal, radius, residual)` (평면 SVD + 2D 대수 원피팅).
- ⚠ Spider 좁은 FOV 탓에 rim 전체가 한 화면에 안 들어올 수 있음 → 보이는 호(arc)에서 취득
  (3점이면 가능하나 호가 짧으면 조건수↓).

## 12. 표면 평면 → `T_B_F0` 빌드

축(방향+XY)만으론 부족 → disc **표면**으로 원점 높이 확정:
- `turntable_frame.py`
  - `fit_plane(pts)` → 표면 평면(점·법선, SVD).
  - `build_T_B_F0(center_B, nz_B)` → F 프레임(원점=center, z=nz, x=base x 투영, y=z×x) → **B→F**.
  - `save_turntable_frame_yaml(...)` → translation(m) + quat 저장.
- (선택) `mms_artec/system.py::ArtecMMS.disc_surface_frame(disc_points_base, axis_point, axis_dir)` —
  표면 평면 ∩ 축선 = F0 원점, z축은 축방향, 평면법선은 교차검증. (구 어레이 경로용이었으나
  rim center/normal 을 바로 `build_T_B_F0` 에 넣어도 됨 — rim 은 표면 근처라 단순.)

## 13. 코드 지도 (턴테이블)

```
utils/calibration/turntable_frame.py     (numpy; 저장 시 scipy/yaml) ★ 공유 코어
    fit_circle_3d(pts) → (center, normal, radius, residual)
    fit_plane(pts)     → (point, normal, residual)
    build_T_B_F0(center_B, nz_B) → T_B_F0 (B→F)
    save_turntable_frame_yaml(...)
utils/calibration/rim_picker.py          (cv2) — rim 클릭 UI (RimPicker/run_picker/show_3d_result)

실물 진입:
  scripts/artec/turntable_frame_init.py  # Artec rim 클릭 (ARTEC_TO_OPENCV z-flip + T_CB)
  scripts/phoxi/turntable_frame_init.py  # PhoXi rim 클릭 (T_CB = T_EB·T_CE)

sim 검증:
  standalone_examples/play/MMS/MMS_ext_calibration2.py   # 턴테이블 rim 자동추출 → fit → GT 비교
```

## 14. 규약·함정 + 실행

- **base 프레임 + SLAM 미사용** — `capture_points_base`(센서→T_EC·FK→base). hand-eye 와 동일 철학(§6).
- **단위** — 코어 수학은 **m** 권장. organized_pts 는 **mm**(rim_picker 가 /1000). 저장은 m.
- **`T_B_F0` = B→F** (`x_F = T_B_F0·x_B`).
- ⚠ **코드 중복(정리 필요)** — `scripts/artec/turntable_frame_init.py` 가 공유 코어 대신 자체
  `fit_circle_3d`/`_RimPicker` 를 들고 있음(PhoXi 는 공유 코어 사용). Artec 도 공유 코어로 통일 권장.
- ⚠ **turntable_frame.yaml stale 의심** — 2026-04-23(Artec pivot 이전). 정밀도 의심 시 재캘리브 1순위.

**sim:** `standalone_examples/play/MMS/MMS_ext_calibration2.py` (Isaac 확장/Script Editor).
**real:** `python scripts/artec/turntable_frame_init.py` → rim 클릭 → `config/calibration/turntable_frame.yaml`.

> 상태: rim 방법 ✅ (Isaac+PhoXi 검증 0.015°/0.7mm). 구 어레이 방법은 폐기(rim 으로 대체).
