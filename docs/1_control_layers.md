# 제어 레이어 구현 (1_control_layers)

`docs/0_control_layers.md` 설계를 코드로 옮긴 현재 스켈레톤. 
그중에서 'hardware layer'에 대한 설계 문서.
아직 NBV 알고리즘은
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

하드웨어 레이어는 **planner (θ 결정)** / **executor (실제 이동)** 두 서브레이어로 분리된다.

```
┌────────────────────────────────────────────────────────────────┐
│   상위 — NBV / 재구성 레이어   (mms/nbv/)                      │
│   입력  : 모델 + 과거 T_CO^(k)                                  │
│   출력  : T_CO_des               ← 본 스켈레톤: 사용자 pick      │
│   아는 프레임: O, C 만                                           │
└────────────────────────────────────────────────────────────────┘
              ↕  T_CO_des    (상위 ↔ 하위 유일한 인터페이스)
┌────────────────────────────────────────────────────────────────┐
│   하위-상 — planner            (mms/control/theta_planner.py)  │
│   입력  : T_CO_des, q_current, θ_current                        │
│   출력  : θ*, q_des*, T_EB*     — 로봇 관절 이동 최소화          │
│   보조  : compute_ik_reachability — 픽 전 IK 도달 영역 색칠     │
│   아는 프레임: B, F, O, E, C                                     │
└────────────────────────────────────────────────────────────────┘
              ↕  θ*, T_EB*
┌────────────────────────────────────────────────────────────────┐
│   하위-하 — executor           (mms/control/hardware_layer.py) │
│   입력  : T_CO_des, θ (결정된 값)                                │
│   출력  : xArm set_position,  Turntable.move_abs                 │
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

### 4.1 Planner — `mms/control/theta_planner.py`

상위가 준 `T_CO_des` 를 만족하는 여러 θ 후보 중, **로봇 관절 이동이 최소** 인 θ* 를 선택.

```
plan_min_motion_theta(
    T_CO_des, T_OF, T_EC, tt, robot,
    theta_current, q_current=None,
    theta_samples=None, theta_range=(-π, π), n_samples=72,
    joint_weights=DEFAULT_JOINT_WEIGHTS,
    w_tt=0.0,
) → dict
```

#### 비용 함수

```
cost(θ) = Σ_i  w_i · (q_des_i(θ) − q_current_i)²   +   w_tt · |θ − θ_current|
```

- `joint_weights` — xArm7 7-DoF 관절별 가중치. 기본
  `[2.0, 2.0, 1.5, 1.0, 1.0, 1.0, 1.0]` (J1·J2 베이스 무겁게, J3 중간, J4~J7 손목 기본).
  베이스가 덜 움직이고 손목을 적극 쓰는 해를 선호하게 된다.
- `w_tt` — 턴테이블 회전 비용 (rad 당). 기본 0 (로봇 이동만 고려).

#### 알고리즘 (그리드 탐색)

1. θ 샘플 그리드 생성: `np.linspace(theta_range, n_samples, endpoint=False)`
   (기본 72 샘플 ≈ 5° 간격, 72 IK 콜 ≈ 0.3–0.5 s)
2. 각 θ 에 대해:
   - `T_EB = T_FB(θ) @ T_OF @ T_CO_des @ T_EC`
   - `pose6d = pose_mat_to_xarm6d(T_EB)`
   - `code, q_des = robot.arm.get_inverse_kinematics(pose6d)`
   - `code != 0` → 제외
   - feasible → cost 계산, 기록
3. `argmin(cost)` 선택 → θ*, q_des*, T_EB* 반환
4. 모두 infeasible 이면 `theta=None` 반환 (상위에서 재pick 하도록 신호)

반환 dict: `theta`, `joints`, `cost`, `T_EB`, `pose6d`, `n_feasible`, `n_samples`,
`all_thetas`, `all_costs`, `all_feasible` — 상위 레이어에서 plot/디버깅용으로 활용 가능.

### 4.2 Executor — `mms/control/hardware_layer.py`

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

#### 동작 순서

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

#### 반환 dict

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

**저수준(executor 단독)** — θ 를 외부에서 주입하는 경우:

```python
result = mms.execute_target(
    T_CO_des=T_CO_des, theta=theta_fixed,
    robot=robot, turntable=turntable,
    robot_speed=20, turntable_vel_rad_s=np.radians(30),
    confirm=True,
)
if not result["ik_ok"]: ...   # 상위 레이어로 돌아가 재pick
```

**고수준(planner + executor)** — θ 를 planner 가 결정:

```python
result = mms.plan_and_execute(
    T_CO_des=T_CO_des, robot=robot, turntable=turntable,
    theta_current=float(turntable.getActualPos()),
    theta_range=(-np.pi, np.pi), n_samples=72,
    w_tt=0.0,                             # 턴테이블 회전 무시
    # joint_weights=None → DEFAULT_JOINT_WEIGHTS
    robot_speed=20, turntable_vel_rad_s=np.radians(30),
    confirm=True,
)
# result = {"plan": {...}, "exec": {...}}
if result["plan"]["theta"] is None:
    # feasible θ 0개 → 재pick
    ...
elif not result["exec"]["ik_ok"]:
    # execute 단계 IK 실패 (경계 케이스) → 재pick
    ...
```

---

## 5. 전체 한 스텝 사이클

`main.py` 의 `control_step()` 이 아래 흐름을 구현한다. planner 실패(feasible θ=0) 또는
execute 단계 IK 실패 시 다시 picker 로 돌아가는 **세이프티 루프** 포함.

```python
with MMS(cfg) as mms:
    robot = XArmInterface(ROBOT_IP)
    turntable = connect_turntable()    # connect + servo_on + 가감속 세팅

    # ── 홈 자세 + 스캔 시작 자세 ───────────────────────────────────
    robot.go_home(speed=5, confirm=True)
    # 홈에서는 PhoXi 광축이 턴테이블을 못 보므로 xyz 유지하고 rpy 만 회전
    go_to_scan_start(robot, speed=10, confirm=True)   # rpy=(178.5, -20.5, 0) deg

    while True:
        # ── ① 캡처 ───────────────────────────────────────────────
        batch = mms.capture_frames(1, ee_pose_fn=robot.get_ee_pose_mat)
        mms.preprocess(batch, roi_bbox=suggest_roi(batch),
                       voxel_size=0.003,
                       enable_voxel=True, enable_denoise=True, enable_normals=True)

        theta_now = float(turntable.getActualPos())

        # ── ② 상위 레이어: IK 색칠 + 수동 pick → T_CO_des ───────
        T_CO_des = mms.nbv_pick_target(
            frames=batch, theta_current=theta_now,
            distance_m=DISTANCE_M,            # 예: 0.414
            default_roll_deg=90.0,            # x↔y 스왑
            check_ik=True, robot=robot,
            theta_target=theta_now,           # 색칠은 현재 θ 기준
            ik_voxel_m=0.025,
        )

        # ── ③ planner + executor ─────────────────────────────────
        result = mms.plan_and_execute(
            T_CO_des=T_CO_des, robot=robot, turntable=turntable,
            theta_current=theta_now,
            theta_range=(-np.pi, np.pi), n_samples=72, w_tt=0.0,
            robot_speed=20, turntable_vel_rad_s=np.radians(30),
            confirm=True,
        )

        plan_ok = result["plan"]["theta"] is not None
        exec_ok = result["exec"] is not None and result["exec"]["ik_ok"]
        if plan_ok and exec_ok:
            break
        if input("재선택? (Enter=yes, q=abort): ").strip().lower() == "q":
            break
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

### 6.4 `main.py` 상수 (현재값)

| 상수 | 현재값 | 역할 |
|------|--------|------|
| `DISTANCE_M` | `0.414` (m) | 표면 → 카메라 거리. PhoXi 초점 영역 내에서 튜닝 대상. |
| `CAMERA_DEFAULT_ROLL_DEG` | `90.0` (°) | 광축(z_cam) 기준 회전. 90° = x_cam ↔ y_cam 스왑. |
| `ROBOT_SPEED_DEG_S` | `20.0` (deg/s) | xArm `set_position` 속도. |
| `TURNTABLE_VEL_RAD_S` | `radians(30)` | 턴테이블 각속도. |
| `PREPROC_VOXEL_M` | `0.003` (m) | 전처리 voxel downsample 크기. |
| `SCAN_START_RPY_DEG` | `(178.5, -20.5, 0.0)` | 스캔 시작 TCP rpy (홈 xyz 유지). |

---

## 7. 주요 파일

| 파일 | 역할 |
|------|------|
| `main.py`                              | 한 스텝 사이클 (`control_step`) + 루프 |
| `mms/nbv/manual_picker.py`             | 상위 레이어 picker (자동 플립/roll) |
| `mms/control/theta_planner.py`         | 하위-상 (planner) — `plan_min_motion_theta` |
| `mms/control/hardware_layer.py`        | 하위-하 (executor) — `execute_camera_target`, `compute_ik_reachability` |
| `mms/system.py`                        | `MMS.nbv_pick_target`, `MMS.execute_target`, `MMS.plan_and_execute` |
| `mms/utils/transforms.py`              | `solve_T_EB`, `compute_T_CO`, `TurntableTransformConfig` |
| `mms/robot/xarm_interface.py`          | xArm7 래퍼 (`set_position`, `ik`, `get_ee_pose_mat`) |
| `mms/turntable/turntable_interface.py` | Ezi-SERVO 턴테이블 (`move_abs`, `getActualPos`) |
| `mms/core/frames.py`                   | `Frame` 데이터클래스, `denoise` (소규모 pcd 방어) |

---

## 8. 추후 확장 지점

- **NBV 교체** — `pick_camera_target_in_frame()` 자리에 알고리즘 출력 `T_CO_des` 를
  꽂기만 하면 된다. 하위 레이어와의 인터페이스(`T_CO_des`) 는 그대로.
- **θ 선택 정책 (구현됨)** — `plan_min_motion_theta` — 현재는 관절 이동 최소화 단일 기준.
  추후 확장: 충돌 검사, 관절 리미트 margin, turntable 리미트, 다목적 비용 (motion + FoV).
- **Multi-roll reachability** — `ik_roll_candidates=(0, π/2, π, 3π/2)` 로 확장하면 초록
  영역이 넓어지지만 비용 4×. 필요 시 활성.
- **Reach envelope 프리필터** — `‖p_cam - p_base‖ > 0.88 m` 같은 컷을 IK 호출 전에 적용해
  절반 이상의 IK 콜을 스킵하도록 개선 가능.
- **Planner 교체** — 그리드 대신 gradient (Jacobian-based) 또는 iLQR/샘플링 최적화로
  교체 가능. `MMS.plan_and_execute` 에서 주입되는 함수만 바꾸면 됨.
- **실측 상태 기록** — 스캔 후 `θ_act`, `T_EB_act` 를 읽어
  `T_CO_act = compute_T_CO(θ_act, T_EB_act, ...)` 를 `Frame` 에 부착하면
  NBV 레이어가 과거 뷰를 누적·학습하는 데 사용할 수 있다.

---

## 9. TODO — 튜닝 파라미터 & IK 체크 로직 개선

앞으로 실기 동작을 보면서 조정해야 하는 파라미터와, 현재 단순화해 둔 IK 확인 로직의
개선 여지를 정리한다.

### 9.1 거리·기하 관련

- **`DISTANCE_M`** (main.py, 현재 `0.414 m`)
  - PhoXi S 의 최소 초점 `0.384 m`. 현재 `+30 mm` 여유를 주고 있음.
  - FoV 부족 / 초점 흐림 / 해상도 trade-off 재측정 필요.
  - TODO: 대표 물체 여러 개에 대해 `distance_m ∈ {0.384, 0.400, 0.414, 0.450, 0.500}` 스윕으로
    가시 영역 / 노이즈 / centroid 일치도 평가.
- **`CAMERA_DEFAULT_ROLL_DEG`** (현재 `90°`)
  - 물체 가로/세로 비, 턴테이블과의 정렬에 따라 `0° / ±90° / 180°` 가 적합할 수 있음.
  - TODO: 여러 roll 에 대한 FoV 커버리지 비교 후 default 재설정 / 또는 per-pick 자동 추천.
- **`knn`** (picker 기본 `30`, check_ik 모드에서 `1` 로 강제)
  - 현재는 "색칠 당시 노말 = 픽 노말" 을 완전히 맞추려고 `knn=1` 강제.
  - 표면이 노이즈로 거친 물체에선 `knn=3~7` 정도의 약한 평균이 더 안정적일 수 있음.
  - TODO: 노이즈 레벨별로 knn 영향 측정 → 적응형 knn (표면 거칠기 기반) 검토.
- **`ik_voxel_m`** (reachability precompute, 현재 `0.025 m`)
  - 작게: 더 촘촘한 초록 영역이지만 IK 콜 수 4× 이상 증가.
  - 크게: 빠르지만 picker 에서 클릭할 점이 너무 드문드문.
  - TODO: 물체 크기와 detail 레벨에 따른 가이드라인 작성.
- **`PREPROC_VOXEL_M`** (전처리, 현재 `0.003 m`)
  - 현재는 `ik_voxel_m` 과 독립. 전처리가 너무 세밀하면 downstream downsample 에서 낭비.
  - TODO: `ik_voxel_m` 의 `k × PREPROC_VOXEL_M` 관계로 묶는 것 검토.

### 9.2 Planner 비용·범위 관련

- **`joint_weights`** (현재 `[2.0, 2.0, 1.5, 1.0, 1.0, 1.0, 1.0]`)
  - 실기에서 J1·J2 가 큰 움직임을 일으키면 weight 상향, 손목이 자꾸 특이점 쪽으로
    움직이면 J5~J7 weight 도 약간 상향.
  - TODO: 실제 스캔 시퀀스 수 회 돌려 평균 joint 이동량 로깅 → 가중치 역추정.
- **`w_tt`** (턴테이블 회전 비용, 현재 `0.0`)
  - 턴테이블 이동이 거슬리거나 케이블/지그에 부담이면 `0.1 ~ 0.3` 으로 시작.
  - TODO: 회전량이 과도하게 크면 관절 이동이 미미해도 회전을 억제하는 logistic 비용 검토.
- **`theta_range` / `n_samples`** (현재 `(-π, π)`, `72`)
  - 물리 리미트 / 케이블 감김 한계 확인 후 범위를 좁힐 예정.
  - 더 조밀한 그리드 (`n_samples=144` = 2.5°) 는 IK 콜 2×. 정확도 vs 속도 관찰.
  - TODO: coarse→fine (2단계) 탐색 — 먼저 8° 그리드로 best 근처 찾고 그 주변만 1° 정밀.

### 9.3 IK 가능 여부 확인 로직

- **(A) Reachability 색칠은 단일 θ 기준**
  - 현재 `compute_ik_reachability` 는 `theta_target` 하나에서만 IK 체크 → "현재 θ 에서
    안 닿음" 만 보여줌. Planner 는 다른 θ 로도 탐색하므로 실제로는 reachable 일 수 있음.
  - TODO: 소수의 θ 샘플 (예: 4~8 개) 에 대해 OR 조건으로 체크해 "어느 θ 에서든 닿음" 을
    나타내는 색칠 옵션 추가.
- **(B) `check_both_orientations=True` 가 현재 절반 낭비**
  - picker 가 "z<0 강제" 로 자동 뒤집기 때문에, 양방향 체크 중 `+n` 쪽 결과는 실제로 안 씀.
  - TODO: `check_both_orientations` 를 기본 `False` 로 내리고, 픽 규칙과 일관된 한
    방향만 체크 → 비용 절반 절감.
- **(C) xArm IK 호출 RTT**
  - SDK `get_inverse_kinematics` 는 약 3–10 ms/call (네트워크 RTT 포함).
  - TODO: Reach envelope 프리필터 (`‖p_cam − p_base‖ > 0.88 m` 컷) 추가 → IK 콜 절반 skip.
  - TODO: 배치 IK 쿼리 API 가 있는지 확인, 없으면 thread pool (SDK thread-safety 전제로).
- **(D) Planner 가 낸 IK 를 executor 가 다시 풀고 있음 (중복)**
  - 현재 planner 가 q_des 를 이미 계산했지만, `execute_camera_target` 은 `robot.ik()` 를
    다시 호출한다.
  - TODO: planner 결과의 `joints` 를 executor 로 넘겨 `robot.arm.set_servo_angle(angle=...)`
    로 바로 이동 → IK 1회 + 해가 일관됨 (재solve 시 다른 branch 로 점프 방지).
- **(E) IK 실패 시 재시도 전략**
  - 현재는 "재pick 루프" 만. 같은 픽 점에서 roll / distance 를 조금 바꿔 자동 재시도
    해보는 fallback 도 유용할 수 있음.
  - TODO: `ik_ok=False` 일 때 `roll ∈ {±5, ±10, ±15}°`, `distance ∈ {0.384, 0.420, 0.450}`
    소규모 그리드로 auto-retry 옵션.
- **(F) 관절 리미트·Singularity margin**
  - xArm IK 가 code==0 이어도 리미트 경계 / 특이점 근처이면 실제 모션 중 fault 발생 가능.
  - TODO: IK 결과 q_des 의 각 관절이 `[limit_min + ε, limit_max − ε]` 안인지, Jacobian
    조건수(`cond(J)`) 가 임계치 이하인지 필터링.
- **(G) 충돌 검사**
  - 현재는 "도달 가능" 만 체크. 턴테이블 / 물체 / 테이블·지그 와의 충돌은 미확인.
  - TODO: 간단한 캡슐/박스 모델 + `trimesh` 또는 `python-fcl` 로 EE·link 충돌 체크를
    planner cost 에 무한대로 주입.
- **(H) Reachability 결과 캐싱**
  - 한 캡처에 대해 IK precompute 를 한 번 한 뒤 사용자가 여러 점을 고민하는 동안 재계산 없이
    재활용 중. θ 바뀌는 시나리오에선 캐시 무효화 필요.
  - TODO: 캐시 키 = `(capture_id, theta_target, distance_m, ik_voxel_m, ik_roll_candidates)`.
