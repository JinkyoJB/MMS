# MMS 캘리브레이션 설계

`control_layers.md` 의 상수 트랜스폼 세 가지를 어떻게 구하고 저장하는지 정리한다.

---

## 1. 대상 트랜스폼 요약

| 트랜스폼 | 의미 | 성격 | 저장 |
|----------|------|------|------|
| `^E T_C` | E(EE) → C(카메라) | **정적 캘리브레이션** — 센서 교체·재설치 시 권장 | `config/calibration/hand_eye_<sensor>.yaml` (재사용 가능) |
| `^B T_F^(0)` | B(베이스) → F(턴테이블) | **설치 캘리브레이션** — 기계 재설치 시 권장 | `config/calibration/turntable.yaml` (재사용 가능) |
| `^O T_F^(0)` | O(내부글로벌) → F(턴테이블) | **런타임 초기화** — 매 스캔 세션 시작 시 | 세션 메모리만, 파일 저장 없음 |

> 캘리브레이션 파일이 이미 존재하면 재사용 가능하다.  
> 센서 교체, 기구부 재조립, 충돌 이벤트 후에는 재캘리브레이션을 권장한다.

이 문서는 `^E T_C` 를 우선 다루고, 나머지는 뒤에 간단히 정리한다.

---

## 2. `^E T_C` — Hand-Eye 캘리브레이션

### 2.1 문제 정의

카메라가 EE에 rigid하게 고정되어 있을 때 (eye-in-hand),  
로봇을 N개의 서로 다른 자세로 이동시키면서 고정된 캘리브레이션 타깃을 촬영한다.

각 자세 i 에서:
- `^B T_E^(i)` — 로봇 FK로 읽은 EE 포즈 (known)
- `^C T_tgt^(i)` — 카메라 기준 타깃 포즈 (센서로 측정)
- `^B T_tgt` — 타깃의 월드 포즈 (unknown, 하지만 모든 i에서 동일)

체인 방정식:

```
^B T_tgt = ^B T_E^(i) · ^E T_C · ^C T_tgt^(i)   ∀ i
```

`^E T_C` 와 `^B T_tgt` 를 동시에 구한다 (**Robot-World + Hand-Eye**, AX = ZB 형식).

OpenCV 함수: `cv2.calibrateRobotWorldHandEye`

---

### 2.2 AX = ZB 공식

```
A_i = ^B T_E^(i)          (로봇 FK)
B_i = ^C T_tgt^(i)        (센서 측정)

→  A_i · X = Z · B_i

X = ^E T_C    (구하려는 값)
Z = ^B T_tgt  (부산물, 타깃 월드 포즈)
```

N ≥ 3 자세, 각 자세 사이에 **충분한 회전** (≥ 30°) 이 필요하다.  
실용적으로는 N = 15–25 자세를 사용한다.

---

### 2.3 센서별 데이터 수집 방법

세 센서는 출력 형식이 달라서 `^C T_tgt^(i)` 를 구하는 방법이 다르다.

#### 2.3.1 Femto Bolt (RGB-D 카메라)

**타깃**: ChArUco 보드 (체커보드보다 부분 가림에 강함)

**절차**:
1. RGB 이미지에서 `cv2.aruco.detectMarkers` + `cv2.aruco.interpolateCornersCharuco`
2. `cv2.solvePnP` 로 `^C T_board` 계산
3. FK에서 `^B T_E` 읽기
4. 쌍 `(^B T_E^(i), ^C T_board^(i))` 수집

```python
# 핵심 코드 스케치
ret, rvec, tvec = cv2.solvePnP(
    board_3d_pts,   # 보드 물리 좌표
    image_2d_pts,   # 이미지 픽셀 좌표
    K, dist_coeffs
)
T_C_tgt = rvec_tvec_to_4x4(rvec, tvec)
```

**장점**: 표준 파이프라인, 서브픽셀 정밀도  
**단점**: 조명·반사에 민감, 깊이카메라 RGB와 Depth 외부 파라미터 별도 보정 필요

---

#### 2.3.2 PhoXi M (구조광 3D 스캐너)

PhoXi는 Photoneo 내장 RecognizeMarkers 기능을 사용한다.  
별도 타깃 없이 **A4-REV-23A 마커 보드** 를 캘리브레이션 타깃으로 사용한다.

**타깃**: Photoneo A4-REV-23A 마커 패턴 보드  
**위치 파일**: `C:\Program Files\Photoneo\PhoXiControl-1.16.5\MarkerPatterns\patterns_with_metadata\A4-REV-23A_positions.txt`

**절차**:
1. `PhoxiClient.detect_marker_transform()` 호출
   - 내부적으로 `RecognizeMarkers=True`, `CoordinateSpace=MarkerSpace` 설정
   - GenTL 청크 `CurrentCameraToCoordinateSpaceTransformation` 읽기 = `T_S_M` (Sensor→Marker)
   - 반환값: `T_M_S = inv(T_S_M)` (Marker→Sensor, translation mm)
2. 반환된 `T_M_S` 를 `HandEyeCalibrator.add_sample(T_E_B, T_M_S)` 에 전달
3. `_last_organized_pts` (H,W,3, mm, 마커 프레임) 로 마커 감지 결과 시각화 가능

```python
# scripts/phoxi_hand_eye_calib.py 핵심 루프
T_M_S = sensor.detect_marker_transform()   # Marker → Sensor (mm)
if T_M_S is not None:
    calibrator.add_sample(T_E_B, T_M_S)
```

**장점**: 별도 지그 불필요, Photoneo SDK 내장 최적화, 조명 무관  
**단점**: A4-REV-23A 보드 필요 (PhoXiControl 설치 시 포함)

---

#### 2.3.3 Artec Spider (핸드헬드형 3D 스캐너)

> **TBD** — 구현 예정. 아래는 개념 설계만 기록한다.

Artec는 EE에 고정되어 있으므로 구체 어레이 방식을 사용한다.  
Artec SDK 스캐닝 파이프라인(`ArtecClient.capture_to_model()`)으로 포인트 클라우드를 획득한다.

**타깃**: 정밀 구체(sphere) 3–4개를 rigid 지그에 고정한 구체 어레이  
**추가 주의사항**:
- Artec 좌표 단위: **mm** → sphere fitting 전에 확인
- `ModelHandle.get_scan(0).get_frame(0).vertices()` 사용
- 단일 프레임 스캔으로 충분 (Registration 불필요)

```python
# 개념 스케치
model = client.capture_to_model(capture_texture=False)
pcd   = model.get_scan(0).get_frame(0).vertices()   # (N,3) float32, mm
center = fit_sphere_ransac(pcd, radius_hint=25.0)   # mm (미구현)
```

---

### 2.4 공통 솔버

센서에 관계없이 `(^B T_E^(i), ^C T_tgt^(i))` 쌍을 모으면 동일한 솔버를 사용한다.

```python
import cv2
import numpy as np

def solve_hand_eye(
    T_B_E_list: list[np.ndarray],   # list of (4,4)
    T_C_tgt_list: list[np.ndarray], # list of (4,4)
    method=cv2.CALIB_ROBOT_WORLD_HAND_EYE_SHAH,
) -> tuple[np.ndarray, np.ndarray]:
    """
    Returns
    -------
    T_E_C : (4,4)  EE → Camera
    T_B_tgt : (4,4)  Base → Target  (부산물)
    """
    assert len(T_B_E_list) == len(T_C_tgt_list)
    assert len(T_B_E_list) >= 3

    R_gripper2base = [T[:3, :3] for T in T_B_E_list]
    t_gripper2base = [T[:3, 3:] for T in T_B_E_list]
    R_target2cam   = [T[:3, :3] for T in T_C_tgt_list]
    t_target2cam   = [T[:3, 3:] for T in T_C_tgt_list]

    R_world2base, t_world2base, R_gripper2cam, t_gripper2cam = \
        cv2.calibrateRobotWorldHandEye(
            R_world2cam  = R_target2cam,   # ^C R_tgt
            t_world2cam  = t_target2cam,
            R_base2gripper = [R.T for R in R_gripper2base],  # ^E R_B
            t_base2gripper = [-R.T @ t for R, t in
                              zip(R_gripper2base, t_gripper2base)],
            method=method,
        )

    T_E_C   = make_4x4(R_gripper2cam,  t_gripper2cam)
    T_B_tgt = make_4x4(np.linalg.inv(R_world2base),
                       -np.linalg.inv(R_world2base) @ t_world2base)
    return T_E_C, T_B_tgt
```

**권장 method**: `CALIB_ROBOT_WORLD_HAND_EYE_SHAH` (rotation + translation 동시 최적화)

---

### 2.5 저장 형식

```yaml
# config/calibration/hand_eye_<sensor>.yaml
sensor: femto_bolt          # femto_bolt | phoxi_m | artec_spider
date: "2026-04-21"
method: charuco_pnp         # charuco_pnp | sphere_array_3d
n_poses: 20
reprojection_error: 0.42    # px (vision) 또는 mm (3D) 단위

T_E_C:
  # EE → Camera  (단위: m, 쿼터니언 xyzw)
  translation: [0.032, -0.015, 0.087]
  rotation_quat: [0.001, 0.707, -0.001, 0.707]
  # 4x4 행렬 (위 translation/quat과 동일한 정보, 편의용)
  matrix:
    - [ 0.0,  0.0,  1.0,  0.032]
    - [-1.0,  0.0,  0.0, -0.015]
    - [ 0.0, -1.0,  0.0,  0.087]
    - [ 0.0,  0.0,  0.0,  1.000]
```

파일 경로 규칙:
```
config/
  calibration/
    hand_eye_femto.yaml
    hand_eye_phoxi.yaml
    hand_eye_artec.yaml
    turntable.yaml
```

---

### 2.6 검증 방법

캘리브레이션 후 아래 두 가지로 정확도를 확인한다.

**① Reprojection error (Femto 전용)**

보류한 N_test개 자세에서 타깃 코너를 재투영해 픽셀 오차 계산.  
목표: < 1.0 px (RMS)

**② Point consistency error (전 센서 공통)**

고정된 구체 또는 표식점 P를 여러 자세에서 측정, 월드 좌표로 변환한 뒤 산포를 측정.

```python
def point_consistency_error(
    T_B_E_test: list[np.ndarray],
    p_C_test:   list[np.ndarray],  # (3,) 각 자세에서 측정한 포인트 (m)
    T_E_C:      np.ndarray,
) -> float:
    """
    p_B^(i) = T_B_E^(i) · T_E_C · p_C^(i) 를 계산하고
    모든 i에서의 표준편차(mm)를 반환.
    목표: < 1.0 mm
    """
    pts_B = []
    for T_B_E, p_C in zip(T_B_E_test, p_C_test):
        p_B = (T_B_E @ T_E_C @ np.append(p_C, 1.0))[:3]
        pts_B.append(p_B)
    pts_B = np.array(pts_B)
    return float(np.linalg.norm(pts_B.std(axis=0)) * 1000)  # mm
```

목표 정확도:

| 센서 | 목표 오차 |
|------|-----------|
| Femto Bolt | < 1.0 mm (point consistency) |
| PhoXi M | < 0.5 mm |
| Artec Spider | < 0.5 mm |

---

## 3. `^B T_F^(0)` — 턴테이블 설치 캘리브레이션

턴테이블 회전축 중심(F)의 B 기준 포즈를 구한다.

### 3.1 방법

**구체 회전 법 (권장)**:

1. 턴테이블 위에 구체를 올린다 (축에서 알려진 반경 r 위치)
2. θ를 M개 등간격으로 회전하며 각 자세에서 로봇 + 센서로 구체 중심 `p_C^(k)` 측정
3. 구체 중심들은 B 프레임에서 **반경 r 인 원** 위에 있다

```
p_B^(k) = T_B_E · T_E_C · p_C^(k)

circle fitting: 원의 중심 → F의 B 기준 위치 (t 성분)
원의 법선 → F의 z축 방향 (R 성분의 z열)
```

**대안**: 레이저 트래커 또는 다이얼 게이지 + 기계 측정

### 3.2 저장

```yaml
# config/calibration/turntable.yaml
date: "2026-04-21"
method: sphere_rotation

T_B_F0:
  # B → F at theta = 0  (단위: m)
  translation: [0.512, 0.001, 0.010]
  rotation_quat: [0.000, 0.000, 0.000, 1.000]
  matrix:
    - [1.0, 0.0, 0.0, 0.512]
    - [0.0, 1.0, 0.0, 0.001]
    - [0.0, 0.0, 1.0, 0.010]
    - [0.0, 0.0, 0.0, 1.000]

residual_mm: 0.3    # circle fitting 잔차
```

---

## 4. `^O T_F^(0)` — 런타임 초기화

`^O T_F^(0)` 는 정적 캘리브레이션이 아니라 **매 스캔 세션 시작 시 계산**된다.

O 프레임은 Artec/PhoXi reconstruction 의 첫 스캔 기준으로 자동 정의되므로,  
θ = 0 시점의 F 포즈를 체인 계산으로 구한다:

```python
# 첫 스캔 시점 (theta = 0)
T_B_F0  = load_turntable_calib()          # ^B T_F^(0)
T_B_E0  = robot.get_T_B_E()              # ^B T_E 현재 FK
T_E_C   = load_hand_eye_calib(sensor)    # ^E T_C

# 첫 프레임: O ≡ C (카메라 프레임 = O 프레임 origin)
# → ^O T_C^(0) = I  (by definition)
# → ^O T_B = inv(T_B_E0 · T_E_C)
T_O_B   = np.linalg.inv(T_B_E0 @ T_E_C)
T_O_F0  = T_O_B @ T_B_F0
```

이 값은 세션 메모리에만 유지되며 파일로 저장하지 않는다.

---

## 5. 구현 로드맵

```
Phase 1 — ^E T_C (hand-eye)
  [x] PhoXi: detect_marker_transform() + HandEyeCalibrator (AX=XB)
  [x] scripts/phoxi_hand_eye_calib.py
  [ ] FemtoCalib  : ChArUco + PnP 데이터 수집 스크립트
  [ ] ArtecCalib  : RANSAC sphere fitting + 포즈 구성 (TBD)
  [ ] Validator   : point_consistency_error()

Phase 2 — ^B T_F^(0) (turntable)
  [ ] 구체 회전 측정 스크립트
  [ ] circle fitting → T_B_F0
  [ ] YAML 저장

Phase 3 — 로딩 & 사용
  [ ] CalibStore 클래스: YAML 읽기 + (4,4) numpy 변환
  [ ] system.py에서 load_hand_eye_calib(sensor) 호출
```

---

## 6. 파일 구조 (예정)

```
mms/
  calibration/
    __init__.py
    calib_store.py          # YAML 로드 → numpy, 유효성 검사
    hand_eye/
      femto_calib.py        # ChArUco + PnP 수집
      sphere_calib.py       # 3D sphere fitting (PhoXi / Artec 공용)
      solver.py             # solve_hand_eye(), point_consistency_error()
    turntable/
      turntable_calib.py    # 구체 회전 측정 + circle fitting

config/
  calibration/
    hand_eye_femto.yaml
    hand_eye_phoxi.yaml
    hand_eye_artec.yaml
    turntable.yaml
```
