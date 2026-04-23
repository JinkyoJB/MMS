# MMS — Multi Modal 3D Scanning System

로봇(xArm7), 턴테이블, 3D 센서(Orbbec Femto Bolt / PhoXi)를 통합 제어하는 멀티모달 3D 스캐닝 시스템입니다.

---

## 좌표계 표기 규칙 (Notation)

```
T_AB : 프레임 A → 프레임 B 변환    x_B = T_AB @ x_A
```

체인 규칙: `T_AC = T_AB @ T_BC` (중간 프레임 B가 약분)

| 기호 | 프레임 | 설명 |
|------|--------|------|
| B | Base | xArm7 로봇 베이스 (≡ 월드 프레임 W) |
| F | Turntable | 턴테이블 프레임 (원점: 회전축 중심, z축: 위쪽) |
| O | Object (Internal Global) | 첫 스캔 기준 내부 글로벌 프레임 (Artec 스타일) |
| E | End-Effector | 로봇 플랜지 / TCP |
| C | Camera | 카메라 프레임 (C_femto, C_phoxi) |

> **규칙**: 코드 변수명 `T_AB`는 항상 "A에서 B로", `x_B = T_AB @ x_A`.  
> 주석이나 문서에서도 `T_AB` 형식만 사용한다 (`^A T_B`, `T_A^B` 금지).

---

## 아키텍처

```
MMS  (mms/system.py)
 ├── PhoxiClient    (mms/sensor/phoxi/)        ← Photoneo PhoXi 3D
 ├── OrbbecClient   (mms/sensor/orbbec/)       ← Orbbec Femto Bolt
 ├── ArtecClient    (mms/sensor/artec/)        ← Artec 3D Scanner
 ├── XArmInterface  (mms/robot/xarm_interface.py)
 └── TurntableInterface (mms/turntable/)
```

---

## 좌표계 변환 일람

### 상수 (캘리브레이션/설계로 결정)

| 변환 | 의미 | 출처 |
|------|------|------|
| `T_BF0` | B → F (θ=0) | `config/calibration/turntable_frame.yaml` |
| `T_EC` | E → C | `config/calibration/hand_eye_phoxi.yaml` (key: `T_E_C`) |

### 가변값 (매 스텝 계산)

| 변환 | 의미 | 계산 방법 |
|------|------|-----------|
| `T_BF(θ)` | B → F | `inv(T_FB0 @ Rz(θ))` |
| `T_FB(θ)` | F → B | `T_FB0 @ Rz(θ)` |
| `T_EB` | E → B | 로봇 FK 실시간 (`XArmInterface.get_ee_pose_mat()`) |
| `T_CB` | C → B | `T_EB @ inv(T_EC)` |

### config 파일 스키마

**`config/calibration/turntable_frame.yaml`**
```yaml
T_B_F0:
  translation: [x, y, z]      # 미터 단위
  rotation_quat: [qx, qy, qz, qw]
  matrix: [[...], ...]        # 4x4 행렬 (중복 저장)
```

**`config/calibration/hand_eye_phoxi.yaml`**
```yaml
T_E_C:
  translation: [x, y, z]
  rotation_quat: [qx, qy, qz, qw]
  matrix: [[...], ...]
```

**`config/sensor_frames.yaml`** (기타 센서)
```yaml
T_EC_femto:
  translation: [x, y, z]
  rotation_quat: [qx, qy, qz, qw]
T_EC_phoxi:
  translation: [x, y, z]
  rotation_quat: [qx, qy, qz, qw]
```

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
T_EC = load_transform("config/calibration/hand_eye_phoxi.yaml", "T_E_C")
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

## 프로젝트 구조

```
MMS/
├── main.py
├── config/
│   ├── calibration/
│   │   ├── turntable_frame.yaml      # T_B_F0 (B→F at θ=0)
│   │   ├── hand_eye_phoxi.yaml       # T_E_C (E→C, PhoXi)
│   │   └── calibration_poses.yaml
│   └── sensor_frames.yaml            # T_EC_femto, T_EC_phoxi (기타 센서)
└── mms/
    ├── system.py                      # MMS 최상위 오케스트레이터
    ├── core/
    │   ├── frames.py                  # Frame 데이터클래스
    │   └── stream.py                  # Stream — 슬라이딩 윈도우 버퍼
    ├── sensor/
    │   ├── phoxi/                     # PhoxiClient
    │   ├── orbbec/                    # OrbbecClient
    │   └── artec/                     # ArtecClient
    ├── robot/
    │   └── xarm_interface.py
    └── utils/
        ├── transforms.py              # TurntableTransformConfig, compute_T_CB, ...
        ├── calibration/
        │   └── hand_eye_calibrator.py # HandEyeCalibrator (T_EC 추정)
        └── visualization.py
```

---

## 환경 설정

```bash
conda create -n mms-env python=3.11
conda activate mms-env
pip install -r requirements.txt
python main.py
```
