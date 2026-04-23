# 제어 레이어 구현 (1_control_layers)

`docs/0_control_layers.md` 설계를 코드로 옮긴 현재 스켈레톤. 아직 NBV 알고리즘은
붙어 있지 않고, 상위 레이어는 **사용자가 Open3D 창에서 수동으로 표면을 picking** 하는
플레이스홀더이다. 다만 픽 한번만으로 6DoF 타겟이 결정되도록 "z<0 노말 강제 + roll 자동"
규칙이 들어가 있고, IK 도달 가능 영역은 색칠되어 보인다.

---

## 1. 노테이션

본 문서와 구현은 `README.md`, `CLAUDE.md` 의 규약만을 사용한다.

```
T_AB : 프레임 A → 프레임 B 변환     x_B = T_AB @ x_A
```

체인 — 합성은 오른쪽에서 왼쪽으로 읽는다 (행렬곱의 자연스런 순서).

```
x_C = T_BC @ T_AB @ x_A           →  T_AC = T_BC @ T_AB
x_O = T_FO @ T_BF @ T_EB @ T_CE @ x_C   →  T_CO = T_FO @ T_BF @ T_EB @ T_CE
```

프레임 기호 (대문자 = coordinate frame, `Frame` 은 데이터클래스)

| 기호 | 이름 | 비고 |
|------|------|------|
| B | Base      | xArm7 base (= World) |
| F | Turntable | z-up, 원점 = 회전축 중심 |
| O | Object / Internal Global | 첫 스캔 기준 내부 글로벌 프레임 |
| E | End-Effector | 로봇 flange / TCP |
| C | Camera | 센서별로 `T_EC` 다름 |

---

## 2. 레이어 구조

```
┌────────────────────────────────────────────────────────────────┐
│   상위 — NBV / 재구성 레이어   (mms/nbv/)                      │
│   입력  : 모델 + 과거 T_CO^(k)                                  │
│   출력  : T_CO_des               ← 본 스켈레톤: 사용자 pick      │
│   아는 프레임: O, C 만                                           │
└────────────────────────────────────────────────────────────────┘
              ↕  T_CO_des    (두 레이어 사이 유일한 인터페이스)
┌────────────────────────────────────────────────────────────────┐
│   하위 — 하드웨어 레이어       (mms/control/)                  │
│   입력  : T_CO_des, θ                                            │
│   출력  : T_EB_des  →  xArm set_position                         │
│          θ         →  Turntable.move_abs                         │
│   보조  : compute_ik_reachability() — 픽 전 IK 도달 영역 색칠    │
│   아는 프레임: B, F, O, E, C                                     │
└────────────────────────────────────────────────────────────────┘
              ↕
           [xArm7] + [Turntable]
```

핵심 식:

```
T_CO(θ, T_EB) = T_FO @ T_BF(θ) @ T_EB @ T_CE
T_EB(θ, T_CO_des) = T_FB(θ) @ T_OF @ T_CO_des @ T_EC
```

상수 — `T_EC` (hand-eye), `T_BF0` (설치 시 고정), `T_OF` (첫 스캔에 고정; 기본 I).
가변 — θ (엔코더), `T_EB` (로봇 FK / IK).

---

## 3. 상위 레이어 구현

파일: `mms/nbv/manual_picker.py`

```
pick_camera_target_in_frame(
    points, normals, colors,
    distance_m=0.384,
    knn=30,
    world_up=[0,0,1],
    frame_label="O",
    preview=True,
    default_roll_deg=0.0,
) → T_CX_des   (X = 입력 프레임, 기본 O)
```

### 3.1 동작 순서

1. **시각화** — `VisualizerWithEditing` 창으로 pcd 표시.
   `Shift + 좌클릭` 으로 표면 포인트 1개 이상 선택, `Q` 로 종료.
2. **로컬 노말 추정** — 선택된 점들의 평균 위치 `p_surf` 주변 k-NN 평균 노말.
   (IK precompute 모드에선 `knn=1` 로 자동 축소 — 색칠 당시 노말과 완전 일치시키기 위함)
3. **노말 방향 자동 정규화 (프롬프트 없음)**
   - `n_z > 0` → 부호 반전해 `n_z < 0` 강제
   - `n_z < 0` → 그대로 유지
   - 결과가 결정적이라 사용자는 현재 노말 방향을 신경쓸 필요 없음
4. **카메라 roll 자동 적용 (프롬프트 없음)**
   - `default_roll_deg` 값을 그대로 사용 (예: `90°` = x_cam ↔ y_cam 스왑)
5. **카메라 포즈 생성** — `compute_camera_pose_from_normal()`
   - 카메라 규약: **OpenCV / PhoXi 스타일** (x=right, y=down, z=forward)
   - 위치: `p_cam = p_surf + distance_m * n_out`
   - z_cam = `-n_out`
   - world-up(기본 `[0,0,1]`) 로 x_cam/y_cam 결정 후 z_cam 축 기준 `default_roll_deg` 만큼 회전
6. **미리보기 창 (선택)** — pcd + 선택점(빨간구) + 노말 화살표(초록) + 카메라 축.
7. 반환: `T_CX_des` (4×4)

### 3.2 IK reachability 프리컴퓨트 (색칠)

파일: `mms/control/hardware_layer.py`

```
compute_ik_reachability(
    points, normals,
    distance_m, theta_target,
    T_OF, T_EC, tt, robot,
    roll_candidates=(0.0,),
    world_up=[0,0,1],
    frame_is_O=True,
    check_both_orientations=True,
) → mask: (N,) bool
```

각 점에 대해:
1. `p_cam = p + distance_m * n`, `z_cam = -n` 으로 카메라 포즈 `T_CX` 계산
2. `frame_is_O=True` → `T_EB = T_FB(θ) @ T_OF @ T_CX @ T_EC`
   `frame_is_O=False` → `T_EB = T_CX @ T_EC`
3. `robot.arm.get_inverse_kinematics(pose6d)` 호출 → `code==0` 이면 reachable
4. `check_both_orientations=True` 면 `n` 과 `-n` 모두 검사 → 어느 한쪽이라도 성공하면 True
   (사용자가 자동 플립으로 `-n` 을 고르더라도 안전)
5. `roll_candidates` 여러 개면 하나라도 성공 시 True (early-exit)

MMS 통합 시 pcd 는 먼저 voxel downsample (`ik_voxel_m`, 기본 25 mm → ~300–500 점) 후 노말
재추정하고, 위 함수를 돌린다. 결과 mask 를 초록(reachable) / 빨강(unreachable) 으로
pcd 에 색칠해 picker 창에 그대로 넘긴다.

### 3.3 MMS 통합

`mms/system.py`

```python
T_CO_des = mms.nbv_pick_target(
    frames=None,                 # None → self.stream
    theta_current=0.0,           # B → O 변환에 사용
    distance_m=0.384,
    knn=30,
    work_in_O=True,              # True: O 프레임에서 picking (기본)
    preview=True,
    default_roll_deg=0.0,        # 자동 roll

    # IK reachability 프리컴퓨트
    check_ik=False,              # True 면 아래 옵션 활성
    robot=None,                  # check_ik=True 이면 XArmInterface 필수
    theta_target=None,           # None → theta_current 사용
    ik_voxel_m=0.025,
    ik_roll_candidates=None,     # None → (radians(default_roll_deg),) 한 샘플
    ik_keep_only_reachable=False,
)
```

- `work_in_O=True` — `Frame.points` (B 기준) 를 `T_BO(θ) = inv(T_OF) @ T_BF(θ)` 로 O 프레임에
  옮겨 picker 에 넘긴다. 반환값은 `T_CO_des` (C → O).
- `work_in_O=False` — B 에서 picker 실행. 반환값 `T_CB_des`. 디버깅용.
- `check_ik=True` — 초록/빨강 색칠된 downsampled pcd 로 picker 수행.
  - `knn_pick=1` 로 강제 → 클릭한 점의 바로 그 노말만 사용 (색칠과 완전 일치).
  - `ik_roll_candidates=None` 이면 `default_roll_deg` 한 샘플만 검사해 색칠=사용 roll 일치.

### 3.4 왜 O 프레임에서 pick 하나

`docs/0_control_layers.md §3.1` 에서 "상위 레이어는 O–C 관계만 본다" 는 원칙 때문이다.
θ 가 바뀌어도 O 좌표의 물체는 움직이지 않으므로, 상위 레이어의 어떤 알고리즘
(NBV, frontier 추출, coverage score 등) 도 **항상 O 기준** 으로 식별·누적할 수 있어야
한다. 이 수동 picker 도 같은 규약을 따른다.

---

## 4. 하위 레이어 구현

파일: `mms/control/hardware_layer.py`

```
execute_camera_target(
    T_CO_des, theta, T_OF, T_EC, tt,
    robot=None, turntable=None,
    robot_speed=30 (deg/s),
    turntable_vel_rad_s=π/6 (=30°/s),
    move_turntable=True, move_robot=True,
    confirm=True,
) → dict
```

### 4.1 동작 순서

1. **T_EB 계산**
   ```
   T_EB_des = T_FB(θ) @ T_OF @ T_CO_des @ T_EC
   ```
   `mms.utils.transforms.solve_T_EB()` 호출.

2. **xArm 포맷 변환** — `pose_mat_to_xarm6d()`
   - `[x(mm), y(mm), z(mm), roll(rad), pitch(rad), yaw(rad)]`
   - `scipy` `xyz` 오일러(intrinsic) — 프로젝트 내 `pose_mat_to_6d` / `pose6d_to_mat` 와 동일 규약.

3. **IK 체크** — `robot.ik(pose6d)` 로 도달 가능성 확인.
   - **실패해도 예외를 던지지 않음** → 반환 dict 의 `ik_ok=False` 로 표시하고 하드웨어
     이동을 생략. 호출자가 다시 pick 할 수 있게 한다. (세이프티 루프)

4. **(confirm=True) Enter 대기** — 안전 확인. `q` 입력 시 이동 생략.

5. **턴테이블 먼저 절대 이동** — `turntable.move_abs(theta, vel)` + `wait_motion_done()`.
   로봇이 접근하기 전에 회전이 끝나도록 순서 보장.

6. **로봇 이동** — `robot.enable_motion(); robot.arm.set_position(..., is_radian=True, wait=True)`.

### 4.2 반환 dict

```
{
  "T_EB_des"       : (4,4)  목표 E → B
  "pose6d"         : (6,)   [x(mm), y(mm), z(mm), r, p, y(rad)]
  "ik_ok"          : bool   IK 해가 존재하는지
  "ik_joints"      : (7,)   [rad] or None      — ik_ok=False 면 None
  "theta"          : float                       명령한 턴테이블 각도
  "robot_moved"    : bool
  "turntable_moved": bool
}
```

### 4.3 MMS 통합

```python
result = mms.execute_target(
    T_CO_des=T_CO_des,                 # 상위 레이어 출력
    theta=np.radians(0),               # 턴테이블 절대 목표 각도 (rad)
    robot=robot,
    turntable=turntable,
    robot_speed=20,
    turntable_vel_rad_s=np.radians(30),
    confirm=True,
)
if not result["ik_ok"]:
    # 상위 레이어로 돌아가 다른 타겟 picking
    ...
```

---

## 5. 전체 한 스텝 사이클

`main.py` 의 `control_step()` 이 아래 흐름을 구현한다. IK 실패 시 다시 picker 로 돌아가는
**세이프티 루프** 도 포함.

```python
with MMS(cfg) as mms:
    robot = XArmInterface(ROBOT_IP)
    turntable = connect_turntable()    # connect + servo_on + 가감속 세팅

    # ── 홈 자세 + 스캔 시작 자세 ───────────────────────────────────
    robot.go_home(speed=5, confirm=True)
    # 홈에서는 PhoXi 광축이 턴테이블을 못 보므로 xyz 유지하고 rpy 만 회전
    go_to_scan_start(robot, speed=10, confirm=True)   # rpy=(178.5, -20.5, 0) deg

    theta_target = float(turntable.getActualPos())

    while True:
        # ── ① 캡처 ───────────────────────────────────────────────
        batch = mms.capture_frames(1, ee_pose_fn=robot.get_ee_pose_mat)
        mms.preprocess(batch, roi_bbox=suggest_roi(batch),
                       voxel_size=0.003,
                       enable_voxel=True, enable_denoise=True, enable_normals=True)

        # ── ② 상위 레이어: IK 색칠 + 수동 pick → T_CO_des ───────
        T_CO_des = mms.nbv_pick_target(
            frames=batch,
            theta_current=float(turntable.getActualPos()),
            distance_m=DISTANCE_M,           # 예: 0.414
            default_roll_deg=90.0,           # x↔y 스왑
            check_ik=True,
            robot=robot,
            theta_target=theta_target,
            ik_voxel_m=0.025,
        )

        # ── ③ 하위 레이어: 턴테이블 + 로봇 이동 ──────────────────
        result = mms.execute_target(
            T_CO_des=T_CO_des, theta=theta_target,
            robot=robot, turntable=turntable, confirm=True,
        )

        if result["ik_ok"]:
            break                             # 성공 → 다음 스텝
        if input("IK 실패 — 재선택? (Enter=yes, q=abort): ").strip().lower() == "q":
            break                             # 사용자 중단
        # else: 루프 → 다시 picker
```

---

## 6. 부가 처리

### 6.1 스캔 시작 자세

- 홈 joint (`[0, -30, 0, 60, 0, 90, 0]` deg) 에서는 PhoXi 광축이 턴테이블을 조준하지 않아
  캡처 시 유효 포인트가 거의 0 이다.
- 홈 TCP xyz 를 유지한 채 rpy 만 `(178.5°, -20.5°, 0°)` 로 세팅 → 턴테이블 조준.
  `main.py` 의 `SCAN_START_RPY_DEG`, `go_to_scan_start()` 참조.

### 6.2 전처리 방어 (frames.py)

- `denoise()` — `len(points) < nb_neighbors` 면 조기 return, SOR 이 빈 인덱스를
  반환하면 원본 유지. Small pcd 로 인한 IndexError 방지.

### 6.3 자동 정규화로 없어진 프롬프트

| 단계 | 이전 | 현재 |
|------|------|------|
| 노말 뒤집기 | `y/N` 프롬프트 | `n_z > 0` 이면 자동 반전 (결정적) |
| Roll 입력 | deg 직접 입력 | `default_roll_deg` 자동 적용 |
| IK 실패 시 | RuntimeError | `ik_ok=False` 반환 + 재pick 루프 |

---

## 7. 주요 파일

| 파일 | 역할 |
|------|------|
| `main.py`                              | 한 스텝 사이클 (`control_step`) + 루프 |
| `mms/nbv/manual_picker.py`             | 상위 레이어 picker (자동 플립/roll) |
| `mms/control/hardware_layer.py`        | `execute_camera_target`, `compute_ik_reachability` |
| `mms/system.py`                        | `MMS.nbv_pick_target`, `MMS.execute_target` 통합 |
| `mms/utils/transforms.py`              | `solve_T_EB`, `compute_T_CO`, `TurntableTransformConfig` |
| `mms/robot/xarm_interface.py`          | xArm7 래퍼 (`set_position`, `ik`, `get_ee_pose_mat`) |
| `mms/turntable/turntable_interface.py` | Ezi-SERVO 턴테이블 (`move_abs`, `getActualPos`) |
| `mms/core/frames.py`                   | `Frame` 데이터클래스, `denoise` (소규모 pcd 방어) |

---

## 8. 추후 확장 지점

- **NBV 교체** — `pick_camera_target_in_frame()` 자리에 알고리즘 출력 `T_CO_des` 를
  꽂기만 하면 된다. 하위 레이어와의 인터페이스(`T_CO_des`) 는 그대로.
- **θ 선택 정책** — 지금은 호출자가 θ 를 직접 준다. 추후
  `solve_T_EB(θ_cand, ...)` 의 IK 가능성 / 충돌 검사로 후보 θ 를 iterate 하는
  로직을 `mms/control/` 에 추가 예정.
- **Multi-roll reachability** — `ik_roll_candidates=(0, π/2, π, 3π/2)` 로 확장하면 초록
  영역이 넓어지지만 비용 4×. 필요 시 활성.
- **Reach envelope 프리필터** — `‖p_cam - p_base‖ > 0.88 m` 같은 컷을 IK 호출 전에 적용해
  절반 이상의 IK 콜을 스킵하도록 개선 가능.
- **실측 상태 기록** — 스캔 후 `θ_act`, `T_EB_act` 를 읽어
  `T_CO_act = compute_T_CO(θ_act, T_EB_act, ...)` 를 `Frame` 에 부착하면
  NBV 레이어가 과거 뷰를 누적·학습하는 데 사용할 수 있다.
