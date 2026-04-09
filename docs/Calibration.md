# MMS Calibration 현황 및 TODO

> 작성일: 2026-04-09  
> 상태: **🔴 Hand-Eye 캘리브레이션 미완성 — 결과 이상, 디버깅 필요**

---

## 1. 캘리브레이션 종류 및 현황

| 항목 | 파일 | 상태 |
|------|------|------|
| Hand-Eye (T_E_S_phoxi) | `config/sensor_frames.yaml` | 🔴 결과 이상 |
| Object frame (T_B_O0) | `config/object_frame.yaml` | 🟡 미확인 |
| Calibration poses | `config/calibration_poses.yaml` |

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


### 3. blob detection 감지 이상 문제(2026.04.09)
intensity.png




## 3. 검증 방법

```bash
# 캘리브레이션 실행
python -m mms.calibration.hand_eye_calibrator

# 결과 검증 (main.py)
python main.py  # check_hand_eye() 실행, 3개 pose에서 PCD 겹침 확인
```


