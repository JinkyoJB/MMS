# MMS — Multi Modal 3D Scanning System

로봇(xArm7), 턴테이블, 3D 센서(Orbbec Femto Bolt / PhoXi)를 통합 제어하는 멀티모달 3D 스캐닝 시스템입니다.

---

## 아키텍처

```
MMS  (mms/system.py)
 ├── OrbbecClient   (mms/sensor/orbbec_client.py)   ← Orbbec Femto Bolt
 ├── xArmInterface  (mms/robot/xarm_interface.py)    ← 추후 연동
 └── TurntableInterface (mms/turntable/)             ← 추후 연동
```

`MMS` 클래스가 센서/로봇/턴테이블 전체 생명주기를 관리합니다.
로봇 미연결 시 `ee_pose_fn=None`으로 사용하고, 연결 시 콜백만 주입합니다.

```python
# 로봇 미연결
with MMS(cfg) as mms:
    batch = mms.capture_frames(10)

# 로봇 연결 시
with MMS(cfg) as mms:
    batch = mms.capture_frames(10, ee_pose_fn=robot.get_ee_pose)
```

---

## 좌표계 표기 규칙


```
T_A_B : 프레임 A → 프레임 B 변환   x_B = T_A_B @ x_A
```

| 기호 | 프레임 | 설명 |
|------|--------|------|
| B | Base | xArm7 로봇 베이스 (≡ 월드 프레임 W) |
| O | Object | 턴테이블 오브젝트 (원점: 테이블 중심, z축: 위쪽) |
| E | End-Effector | 로봇 플랜지 / TCP |
| S | Sensor | 카메라 프레임 (S_femto, S_phoxi) |

---

## 좌표계 변환 (Frame Transforms)

### 프레임 관계도

- **B ↔ O** : `config/object_frame.yaml`에 θ=0 기준 고정 초기값(`T_B_O0`)을 저장. 실제 θ별 변환은 `WorldTransformConfig`가 `T_B_O0`에 `Rz(θ)`를 합성하여 계산
- **E ↔ B** : 로봇 FK로 실시간 계산 (`xArmInterface.fk()`)
- **S ↔ E** : 핸드-아이 캘리브레이션 결과, 고정값 (`config/sensor_frames.yaml`)
- **S → B** : 위 세 가지를 합성하여 파생 (`T_E_B @ inv(T_E_S)`)

---

### 변환 일람

| 변환 | 방향 | 출처 | 비고 |
|------|------|------|------|
| `T_B_O0` | Base → Object (θ=0) | `config/object_frame.yaml` | 설치 시 1회 측정 |
| `T_E_S_femto` | EE → Orbbec 센서 | `config/sensor_frames.yaml` | 핸드-아이 캘리브레이션 |
| `T_E_S_phoxi` | EE → PhoXi 센서 | `config/sensor_frames.yaml` | 핸드-아이 캘리브레이션 |
| `T_E_B` | EE → Base | 로봇 FK 실시간 | `xArmInterface.fk()` |

**파생 변환 (코드에서 계산):**

| 변환 | 수식 | 의미 |
|------|------|------|
| `T_O_B(θ)` | `inv(T_B_O0) @ blkdiag(Rz(θ), 1)` | Object → Base (턴테이블 회전 포함) |
| `T_B_O(θ)` | `inv(T_O_B(θ))` | Base → Object |
| `T_S_B` | `T_E_B @ inv(T_E_S)` | Sensor → Base |
| `T_B_E` (NBV용) | `T_B_S @ inv(T_E_S)` | Base → EE target (센서 목표 자세로부터) |

---

### config 파일 스키마

**`config/object_frame.yaml`**
```yaml
T_B_O0:                       # Base → Object at theta = 0
  translation: [x, y, z]      # 미터 단위
  rotation_quat: [qx, qy, qz, qw]
```

**`config/sensor_frames.yaml`**
```yaml
T_E_S_femto:                  # EE → Orbbec Femto Bolt
  translation: [x, y, z]
  rotation_quat: [qx, qy, qz, qw]

T_E_S_phoxi:                  # EE → PhoXi 3D Scanner
  translation: [x, y, z]
  rotation_quat: [qx, qy, qz, qw]
```

---

### 사용 예시

```python
from mms.utils.transforms import (
    load_transform, WorldTransformConfig,
    compute_T_S_B, transform_points, transform_normals,
    sensor_pose_to_ee_pose,
)

# 1. 오브젝트 ↔ 베이스 변환 (턴테이블 각도 포함)
T_B_O0 = load_transform("config/object_frame.yaml", "T_B_O0")
wt = WorldTransformConfig(T_B_O0)

theta = 1.57  # 90도 (rad)
x_B = wt.T_O_B(theta) @ x_O   # 오브젝트 좌표 → 베이스 좌표
x_O = wt.T_B_O(theta) @ x_B   # 베이스 좌표 → 오브젝트 좌표

# 2. 센서 → 베이스 변환
T_E_S = load_transform("config/sensor_frames.yaml", "T_E_S_femto")
T_E_B = robot.fk(joints)       # xArm FK 결과 (4x4)

T_S_B = compute_T_S_B(T_E_B, T_E_S)
points_B = transform_points(T_S_B, points_S)    # 센서 좌표 → 베이스 좌표
normals_B = transform_normals(T_S_B, normals_S) # 법선 벡터 변환

# 3. NBV: 센서 목표 자세 → EE 목표 자세
T_B_E_target = sensor_pose_to_ee_pose(T_B_S_target, T_E_S)

# 4. MMS를 통한 직접 호출 (가장 간편)
with MMS(cfg) as mms:
    x_B  = mms.T_O_B(theta) @ x_O      # Object → Base
    x_O  = mms.T_B_O(theta) @ x_B      # Base → Object
    T_sb = mms.T_S_B(T_E_B)            # Sensor → Base (self.sensor.T_E_S 자동 참조)
    T_sb = mms.T_S_B(T_E_B, T_E_S)     # 센서 T_E_S 직접 지정도 가능
```

---

## 프로젝트 구조

```
MMS/
├── main.py                        # 실행 진입점 / 테스트
├── requirements.txt
├── config/
│   ├── object_frame.yaml          # T_B_O0 (base→object at θ=0)
│   ├── sensor_frames.yaml         # T_E_S_femto, T_E_S_phoxi
│   └── calibration_poses.yaml     # 핸드-아이 캘리브레이션 포즈
└── mms/
    ├── system.py                  # MMS 최상위 오케스트레이터
    ├── core/
    │   ├── frames.py              # Frame 데이터클래스 + 전처리 메서드
    │   ├── stream.py              # Stream — 슬라이딩 윈도우 Frame 버퍼
    │   ├── scan_data.py           # voxel map, TSDF, frontier 추출
    │   └── nbv_planner.py         # NBV 플래너
    ├── sensor/
    │   ├── orbbec_client.py       # OrbbecClient (pyorbbecsdk 래퍼)
    │   └── phoxi_client.py        # PhoxiClient (Harvesters/GenTL 래퍼)
    ├── robot/
    │   ├── xarm_interface.py      # EE pose 읽기 + 모션 제어
    │   └── ee_turntable_planner.py
    ├── turntable/
    │   └── turntable_interface.py # θ 읽기 + θ* 명령
    └── utils/
        ├── transforms.py          # 좌표 변환 유틸 (WorldTransformConfig, compute_T_S_B, ...)
        ├── diagnostics.py         # PCD 통계, ROI 자동 추출
        └── visualization.py       # Open3D 시각화
```

---

## 주요 클래스 & 인터페이스

### `MMS` (`mms/system.py`)

`self.stream`은 `Stream` 인스턴스다. `MMSConfig.stream_max_size`(기본 500)를 초과하면 오래된 프레임이 자동 evict된다.

| 메서드/속성 | 설명 |
|--------|------|
| `initialize()` | 전체 하드웨어 초기화 |
| `shutdown()` | 전체 하드웨어 종료 |
| `capture_frames(n, ee_pose_fn)` | n 프레임 캡처 → `self.stream`에 누적 (pacing은 `OrbbecConfig.target_interval_s`) |
| `preprocess(frames, roi_bbox, ...)` | ROI → voxel → denoise → normals (`list[Frame]` 또는 `Stream` 수용) |
| `clear_stream()` | `self.stream` 버퍼 초기화 |
| `self.stream` | `Stream` — 누적 프레임 버퍼 (`latest()`, `since()`, `between()` 등 조회 가능) |
| `T_O_B(theta)` | Object → Base 변환 (4×4). `object_frame_yaml` 필요 |
| `T_B_O(theta)` | Base → Object 변환 (4×4). `object_frame_yaml` 필요 |
| `T_S_B(T_E_B, T_E_S=None)` | Sensor → Base 변환 (4×4). T_E_S 생략 시 `self.sensor.T_E_S` 사용 |

### `Frame` (`mms/core/frames.py`)

단일 시점 캡처 스냅샷. 내부 데이터는 NumPy 배열로만 보관하며, Open3D 객체는 시각화 시점에만 생성된다.

| 속성 | 타입 | 설명 |
|------|------|------|
| `points` | `np.ndarray (N,3) float32` | base(B) 기준 포인트 클라우드 |
| `normals` | `np.ndarray (N,3) float32 \| None` | 단위 법선 벡터 (estimate_normals() 후 채워짐) |
| `colors` | `np.ndarray (N,3) float32 \| None` | per-point RGB [0,1] |
| `img` | `np.ndarray (H,W,3) uint8 \| None` | RGB 이미지 |
| `depth` | `np.ndarray (H,W) float32 \| None` | depth (미터 단위) |
| `ee_pose_mat_B` | `np.ndarray (4,4)` | T_E^B — 캡처 시점 EE 포즈 |
| `ee_pose_6d_B` | `np.ndarray (6,)` | `[x,y,z,rx,ry,rz]` (property) |

전처리 메서드: `roi_crop()` → `voxel_downsample()` → `denoise()` → `estimate_normals()`
시각화: `frame.to_pcd()` → `o3d.geometry.PointCloud`

---

### `Stream` (`mms/core/stream.py`)

실시간 데이터 흐름을 위한 슬라이딩 윈도우 버퍼. `Frame`이 "단일 시점 스냅샷"이라면 `Stream`은 "시간 순서로 쌓이는 Frame 시퀀스"이다.

```python
stream = Stream(max_size=50)
stream.extend(batch)            # 여러 프레임 추가
stream.append(frame)            # 단일 프레임 추가 (버퍼 초과 시 가장 오래된 것 자동 삭제)

stream.latest(5)                # 가장 최근 5개
stream.since(time.time() - 2.0) # 2초 이내 프레임
stream.between(t0, t1)          # 시간 구간 [t0, t1]
```

| 메서드 | 설명 |
|--------|------|
| `append(frame)` | 프레임 1개 추가 |
| `extend(frames)` | 프레임 리스트 추가 |
| `latest(n)` | 가장 최근 n개 반환 |
| `since(t)` | timestamp ≥ t 인 프레임 반환 |
| `between(t0, t1)` | timestamp ∈ [t0, t1] 인 프레임 반환 |
| `to_list()` | 전체 버퍼를 list로 반환 |
| `clear()` | 버퍼 초기화 |

### `OrbbecClient` (`mms/sensor/orbbec_client.py`)

```python
cfg = OrbbecConfig(
    sensor_frames_yaml="config/sensor_frames.yaml",
    T_E_S_key="T_E_S_femto",
    enable_color=True,
    frame_timeout_ms=2000,
)
client = OrbbecClient(cfg)
client.initialize()
frame = client.capture_frame(ee_pose_mat_B=T_E_B, frame_id=0, timestamp=t)
client.shutdown()
```

---

## 환경 설정

### 1. 레포지토리 클론

```bash
git clone https://github.com/JinkyoJB/MMS
cd MMS
```

### 2. 가상환경 생성 및 패키지 설치

```bash
conda create -n mms-env python=3.11
conda activate mms-env
pip install -r requirements.txt
```

### 3. pyorbbecsdk 설치 (Orbbec 센서 사용 시)

pyorbbecsdk는 pip로 배포되지 않으므로 별도 설치가 필요합니다.
[pyorbbecsdk GitHub](https://github.com/orbbec/pyorbbecsdk) 참고.

---

## 실행

```bash
conda activate mms-env
python main.py
```

`main.py` 실행 순서:
1. `MMS` 초기화 (OrbbecClient warmup 포함)
2. 10 프레임 캡처
3. Raw PCD 좌표 통계 출력 (ROI 범위 파악)
4. Raw PCD Open3D 시각화
5. 자동 ROI 적용 → 전처리 (voxel / denoise / normals)
6. 전처리 결과 Open3D 시각화

---

## 개발자용: requirements.txt 갱신

```bash
conda activate mms-env
pip freeze > requirements.txt
```

> **Note** ROS / pyorbbecsdk 등 pip 외 패키지는 별도 설치가 필요합니다.
