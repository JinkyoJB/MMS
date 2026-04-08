# MMS Calibration 현황 및 TODO

> 작성일: 2026-04-08  
> 상태: **🔴 Hand-Eye 캘리브레이션 미완성 — 결과 이상, 디버깅 필요**

---

## 1. 캘리브레이션 종류 및 현황

| 항목 | 파일 | 상태 |
|------|------|------|
| Hand-Eye (T_E_S_phoxi) | `config/sensor_frames.yaml` | 🔴 결과 이상 |
| Object frame (T_B_O0) | `config/object_frame.yaml` | 🟡 미확인 |
| Calibration poses | `config/calibration_poses.yaml` | 🟡 포즈 수 부족 가능성 |

---

## 2. Hand-Eye 캘리브레이션

### 2.1 개요

- **목적**: EE 프레임 → PhoXi 센서 프레임 변환 `T_E_S` 추정
- **방식**: Eye-in-Hand (센서가 로봇 EE에 장착)
- **수식**: AX = XB, X = T_E_S
  - A_i = T_E_B^(i+1) @ inv(T_E_B^i)   (EE 간 상대 운동, Base 기준)
  - B_i = T_M_S^(i+1) @ inv(T_M_S^i)   (마커 간 상대 운동, Sensor 기준)
- **마커**: A4-REV-23A (Photoneo 전용 마커보드)
- **알고리즘**: OpenCV `calibrateHandEye` (TSAI / PARK / HORAUD / ANDREFF / DANIILIDIS 중 잔차 최소)

### 2.2 관련 파일

```
mms/calibration/
  hand_eye_calibrator.py   # HandEyeCalibrator 클래스 + __main__ 실행 스크립트
  marker_detector.py       # A4REV23ADetector: intensity 이미지 → T_M_S 추정
config/
  calibration_poses.yaml   # 캘리브레이션용 로봇 joint 포즈 목록 (degrees)
  sensor_frames.yaml       # 결과 저장 위치 (T_E_S_phoxi 키)
```

### 2.3 실행 방법

```bash
python -m mms.calibration.hand_eye_calibrator
```

로봇이 `calibration_poses.yaml`의 포즈로 자동 이동하며, 각 포즈에서:
1. PhoXi로 intensity 이미지 + 조직화 포인트클라우드 캡처
2. A4-REV-23A 마커 감지 → T_M_S 계산
3. 샘플 수집 후 `calibrateHandEye` 실행
4. 결과를 `config/sensor_frames.yaml`의 `T_E_S_phoxi`에 저장

디버그 이미지(intensity, 블롭, 감지 결과)는 `debug_calib/` 폴더에 저장됨.

### 2.4 현재 저장된 값 (이상 의심)

```yaml
# config/sensor_frames.yaml
T_E_S_phoxi:
  rotation_quat: [0.5691, -0.4216, 0.4109, 0.5740]
  translation: [-0.101, 0.700, 0.186]  # meters
```

translation norm ≈ 730mm. **EE에서 센서까지 730mm는 물리적으로 불가능한 값.**

---

## 3. 검증 결과 (이상 확인됨)

`main.py`의 `check_hand_eye()` 함수로 `visualize_hand_eye_calibration()` 실행:

```
  [ 0]  EE: [+0.311, -0.186, +0.519]  Centroid: [+0.675, +0.348, +0.164]  pts=1,171,373
  [ 1]  EE: [+0.242, -0.052, +0.582]  Centroid: [+0.633, +0.015, +0.041]  pts=  559,796
  [ 2]  EE: [+0.271, +0.170, +0.565]  Centroid: [+0.705, -0.076, +0.063]  pts=1,002,206

  centroid 편차 max : 263.22 mm   ← 정상: <5mm
  판정 : ✘ 재캘리브레이션 권장
```

**EE Y와 PCD centroid Y가 반대 방향으로 움직임** (anti-correlation).  
→ 변환이 반대로 적용되거나 calibration 결과 자체가 크게 잘못됐음을 의미.

---

## 4. 🔴 디버깅 TODO (우선순위 순)

### [A] 단위 불일치 버그 확인 ← 가장 유력한 원인

`hand_eye_calibrator.py`의 `__main__` 스크립트에서:

```python
# T_E_B: EE pose → meters 단위로 변환됨
T_E_B[:3, 3] = pose6[:3] / 1000.0   # mm → m  ✔

# T_M_S: marker_detector.py 내부에서 mm 단위 그대로 반환될 가능성
T_M_S = detector.detect(img_gray, pts_org)
# pts_org의 단위가 mm라면 T_M_S translation도 mm
```

`calibrateHandEye`에 T_E_B(m)와 T_M_S(mm)를 혼용하면 잘못된 T_E_S 출력.

**확인 방법**:
```python
# hand_eye_calibrator.py __main__ 실행 중 로그에서
# T_M_S[:3, 3] 값 확인 — 마커까지의 거리가 mm인지 m인지 점검
# 예: 마커가 센서에서 0.5m 떨어져 있으면 T_M_S[:3,3] ≈ [x, y, 500] mm or [x, y, 0.5] m
```

**수정 방향**: `marker_detector.py`에서 반환 전 `/1000.0` 적용하거나,  
`hand_eye_calibrator.py`에서 T_M_S 받은 후 `T_M_S[:3,3] /= 1000.0`.

### [B] save_yaml의 단위 주석 모순 확인

`hand_eye_calibrator.py` line 261:
```python
t = self._T_E_S[:3, 3]  # meters 단위 (입력 T_M_S가 mm → calibrate가 mm 단위 t 반환)
```

주석이 모순됨 — "meters 단위"라고 쓰고 괄호 안에서 "mm 단위 t 반환"이라고 함.  
**실제로 저장되는 단위가 mm인지 m인지 확인 필요.**  
`load_transform()` (`mms/utils/transforms.py`)은 YAML 값을 meters로 그대로 사용하므로, YAML에 mm로 저장되면 시스템 전체가 잘못된 변환을 사용하게 됨.

### [C] Convention 반전 테스트

위 [A], [B]와 별개로, 빠른 검증:  
`mms/sensor/phoxi_client.py`에서 임시로 T_E_S 방향 반전:

```python
# 현재
T_S_B = compute_T_S_B(T_E_B, T_E_S)  # = T_E_B @ inv(T_E_S)

# 테스트: YAML에 T_S_E로 저장됐을 경우
T_S_B = T_E_B @ T_E_S  # inv 제거
```

`visualize_hand_eye_calibration()` 결과가 개선되면 YAML 저장 convention이 반대인 것.

### [D] 포즈 다양성 확대

`config/calibration_poses.yaml` 현재 5개 포즈, J1 범위 ±23°.  
캘리브레이션 정확도 향상을 위해:
- **8~12개** 포즈 권장
- Joint 1 범위 ±45° 이상
- 센서 tilting (마커를 다양한 각도에서 바라보기) 포함
- 각 포즈에서 마커 전체가 PhoXi 시야 내에 있어야 함

### [E] 마커 감지 품질 확인

캘리브레이션 실행 후 `debug_calib/` 폴더:
- `{pose_name}_intensity.png`: PhoXi intensity 이미지
- `{pose_name}_blobs_bright/dark.png`: 블롭 감지 결과
- `{pose_name}_detected.png`: 최종 마커 감지 결과

마커 감지 실패 포즈가 많으면 샘플 품질이 저하됨.  
→ 포즈별로 이미지 직접 확인 필요.

---

## 5. 캘리브레이션 재실행 절차

1. **[A] 단위 버그 수정** 후 재실행
2. `debug_calib/` 이미지로 마커 감지 확인
3. `config/sensor_frames.yaml`에 저장된 translation norm 확인  
   (물리적으로 타당한 범위: 50mm ~ 400mm)
4. `main.py`의 `check_hand_eye()` 실행 (3개 이상 포즈)  
   → `centroid 편차 max < 5mm` 달성 목표
5. 필요시 포즈 추가 (calibration_poses.yaml)

---

## 6. 참고: 정상적인 T_E_S 예상 범위

PhoXi가 EE에 장착된 Eye-in-Hand 구성 기준:
- `translation` norm: **50mm ~ 400mm** (센서 크기 + 브라켓 고려)
- 현재 값 730mm: **물리적으로 불가 → 캘리브레이션 오류**

---

## 7. 검증 방법

```bash
# 캘리브레이션 실행
python -m mms.calibration.hand_eye_calibrator

# 결과 검증 (main.py)
python main.py  # check_hand_eye() 실행, 3개 pose에서 PCD 겹침 확인
```

`visualize_hand_eye_calibration()` 창에서:
- 3개 색상의 PCD가 B 프레임에서 **겹쳐야** 정상
- 현재: 크게 어긋남 (263mm 편차)
