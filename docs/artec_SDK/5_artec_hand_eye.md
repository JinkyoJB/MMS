# Artec Hand-Eye 캘리브레이션 — 2026-04-29

> ⚠ **이 문서는 2026-04-29 시점의 작업 기록이다. 실행 절차로 따라가지 말 것.**
>
> 현재 절차는 **`docs/calibration_runbook.md`**, 진입점은
> **`python scripts/artec/calibrate.py`** 하나다.
>
> 이 문서와 지금이 다른 점:
> - 스크립트 경로가 `scripts/artec_*.py` → **`scripts/artec/*.py`** 로 바뀌었다
> - `--interactive`(손으로 끌어 캡처하는 teach mode)는 **제거됐다**
>   (hand-eye 2026-09-15, intrinsic 2026-09-16). 자세 목록은
>   `gen_calib_poses.py` 가 만든다
> - 보드 기본값이 `spider`(5×3/20mm, 코너 8개) → **`spider_dense`**
>   (7×5/12mm, 코너 24개)로 바뀌었다
> - 여기 적힌 `T_EC` 는 **구 스캐너 `SP.10.36181288`** 값이다
>   (현재 장착: `SP.10.79103441`)
>
> 아래 "보드 square 18.5mm 실측(인쇄 기준 20mm)" 은 오차가 아니라
> `make_charuco.py` 의 **여백 버그**였다 — 출력 크기에서 여백을 깎아 칸이
> 공칭보다 작게 인쇄됐다(20mm → 18.67mm). 2026-09-16 수정.

> 센서를 PhoXi → Artec 으로 교체한 뒤 `T_EC_artec` 를 새로 측정한 기록.
> 시도 1회 (UV→3D Procrustes) 가 25mm 잔차 → solvePnP 로 갈아탄 뒤 **3.55mm /
> 1.30°** 로 수렴. 본 문서는 최종 파이프라인 + 시행착오 정리.

---

## 0. 결과 요약

| | 값 |
|---|---|
| 방법 | ChArUco 검출 + (cv2.aruco.calibrateCamera 로 intrinsic 측정) + cv2.solvePnP + cv2.calibrateHandEye 5-method 비교 |
| 보드 | ChArUco 5×3, square 18.5mm 실측 (인쇄 기준 20mm), marker 13.875mm, DICT_4X4_50 |
| 자세 수 | 22 (사전 25 수집, IK 실패 / 검출 실패 자동 제외) |
| Intrinsic reproj rmse | 1.109 px |
| Hand-eye 잔차 (PARK) | **t_err 3.55 mm, r_err 1.30°** |
| `T_EC_artec` translation | (4.3, -177.2, -59.0) mm in EE |
| 저장 | `config/sensor_frames.yaml` `T_EC_artec` |

---

## 1. PhoXi 와의 차이점

| 항목 | PhoXi | Artec |
|------|------|-------|
| 마커 검출 | Photoneo SDK 내장 `RecognizeMarkers` (A4-REV-23A) → 곧바로 `T_M_S` | OpenCV `cv2.aruco` (ChArUco) — 우리가 직접 검출 |
| 캡처 출력 | organized Range (H×W×3) + intensity | IFrameMesh: vertices (N,3) + UV (N,2) + 텍스처 image (H,W,3) |
| 카메라 intrinsics | 불필요 (organized Range 가 이미 3D) | **필요** (texture image 픽셀을 3D 로 보내려면) |
| 광원 제어 | LED 항상 on | flash (구조광) / texture_flash 분리. **texture_flash 는 끌 수 있음** (단 이미지 어두워짐) |

---

## 2. 알고리즘 — solvePnP (최종 채택)

각 로봇 자세 i 에서:

```
1. Artec capture → IFrameMesh
     vertices : (N, 3) mm  C 프레임  (geometry camera)
     uv       : (N, 2) ∈ [0,1]
     image    : (H, W, 3) RGB texture

2. ChArUco corner + ArUco 4-corner 검출 (texture image)
     → 픽셀 (u, v) 와 보드 프레임 좌표 p_M^k (mm, z=0)

3. cv2.solvePnP (intrinsics K, dist 사용)
     → rvec, tvec (M → C)
     → cv2.solvePnPRefineLM 로 LM 정밀화

4. T_M_C = [R(rvec) | tvec] (mm)

5. HandEyeCalibrator.add_sample(T_EB, T_M_C) → 5-method 시도 → 잔차 최소
```

**왜 solvePnP 인가**: 첫 시도였던 **UV→3D nearest-vertex 매칭** 은 양자화 오차가
~1mm/자세, 이게 자세별로 편향돼서 hand-eye 잔차 25mm 로 발산. solvePnP 는
이미지 픽셀을 그대로 사용 → sub-pixel 정밀도, 양자화 없음 → 잔차 7× 감소.

→ **`mms/utils/calibration/artec_charuco_detector.py::detect()`** 가 intrinsic
가 주어지면 자동으로 `_detect_pnp` 경로 사용. 없으면 legacy UV→3D fallback.

---

## 3. ChArUco 보드 (Spider 용)

Spider 의 좁은 FOV (~100×80mm @ 작동거리 200mm) 에 맞춘 작은 보드:

| 파라미터 | 값 |
|---------|-----|
| `squares_x` × `squares_y` | 5 × 3 |
| `square_length_mm` | 20.0 (인쇄 후 실측 18.5mm) |
| `marker_length_mm` | 15.0 (실측 13.875mm — 18.5/20 비율) |
| ArUco 사전 | `DICT_4X4_50` |
| 보드 크기 | 100 × 60 mm |

> A4 보드 (7×5, 30mm/22mm, DICT_5X5_100) 는 Spider FOV 안 들어옴. **`spider`
> 프리셋** 이 default.

생성:
```bash
python scripts/artec_make_charuco.py --board spider
# → debug_calib/charuco_5x3_20_15.png  (1000×600 px @ 10 px/mm)
```

**인쇄**: A4 100% / "실제 크기" 옵션. 기본 사진 인쇄 다이얼로그의 "페이지에
맞춤" 은 보드를 늘려버려서 X. Hagaki (100×148mm) 사이즈 옵션이 우리 5:3 비율과
잘 맞아 100×60mm 출력 가능.

**실측 필수**: 인쇄 후 자/캘리퍼로 한 칸 측정. 보통 ±0.5-2mm 오차. 측정값을
`--square-mm`, `--marker-mm` (= square × 0.75) 으로 넘김.

---

## 4. 구현 파일

```
mms/utils/calibration/
  hand_eye_calibrator.py        — AX=XB 솔버 (PhoXi 와 공용)
  artec_charuco_detector.py     — ChArUco/ArUco 검출 + solvePnP / UV→3D 분기

scripts/
  artec_make_charuco.py         — 보드 PNG 생성
  artec_intrinsic_calib.py      — intrinsic 측정 (1회)
  artec_hand_eye_calib.py       — 메인 캘리브 루프

config/calibration/
  artec_calibration_poses.yaml  — 25 자세 (interactive 로 누적)
  artec_intrinsic.yaml          — K, dist (intrinsic 결과)
  hand_eye_artec.yaml           — T_EC (hand-eye 결과)

config/sensor_frames.yaml       — T_EC_artec (시스템 적용)
debug_calib_artec/              — 자세별 검출 시각화 PNG
debug_intrinsic_artec/          — intrinsic 자세별 검출 시각화 PNG
```

---

## 5. 절차 (재현용)

### Step 0 — 보드 인쇄 + 평면 부착 (1회)

```bash
python scripts/artec_make_charuco.py
# → debug_calib/charuco_5x3_20_15.png
```

A4 100% 인쇄 또는 "100×148mm Hagaki" 옵션. **단단한 평판 (클립보드/아크릴/MDF)
에 양면테이프로 부착** — 종이가 휘면 검출 노이즈 ↑. 자로 한 칸 길이 실측:

| 측정값 | 조치 |
|---|---|
| 19.5-20.5mm | OK, 그대로 |
| 외 | `--square-mm <실측>` 사용, `--marker-mm = 실측 × 0.75` |

### Step 1 — 캘리브 자세 수집 (interactive, 12-25자세)

```bash
python scripts/artec_hand_eye_calib.py --target-n 12 --interactive \
    --square-mm 18.5 --marker-mm 13.875
```

- 로봇이 manual(teach) mode 로 풀림 → **손으로 직접** EE 자세 잡음
- 자세 잡고 **Enter** 한 번 → capture + 검출
- 검출 성공 자세는 `config/calibration/artec_calibration_poses.yaml` 에 자동 추가
- 다양성 가이드:
  - 거리 220-300mm 사이 3-4단계
  - roll/yaw 회전 ±20° 이상
  - pitch tilt ±20°
  - 보드 위치 좌/우/중앙 다양하게
- 첫 캡처가 검출 성공하면 (`✓ corners=N rmse=...`) 좋은 자세

세션을 여러 번 나눠도 됨 — 두번째 실행하면 `manual_NN` 카운터가 yaml 의
다음 번호부터 이어짐.

### Step 2 — Intrinsic 캘리브 (1회, ~7분)

수집된 자세 yaml 을 그대로 재사용 (다양성 충분):

```bash
python scripts/artec_intrinsic_calib.py \
    --square-mm 18.5 --marker-mm 13.875
```

- 로봇이 yaml 의 자세 자동 순회 (via_home 으로 안전 이동)
- 각 자세에서 capture → ChArUco corner 검출 → 누적
- `cv2.aruco.CharucoBoard.matchImagePoints` + `cv2.calibrateCamera` 실행
- `config/calibration/artec_intrinsic.yaml` 에 K, dist 저장

기대: `reproj rmse < 1.5 px`. 첫 실행 1.109 px.

> **주의**: 모은 자세가 적거나 보드가 이미지 가장자리 커버리지 부족이면 dist
> 의 k2/k3 가 발산할 수 있음. `--fix-k3` 가 default ON 으로 강제 0 ←
> overfitting 안전장치.

### Step 3 — Hand-eye 캘리브 (자동 solvePnP)

```bash
python scripts/artec_hand_eye_calib.py \
    --square-mm 18.5 --marker-mm 13.875
```

- yaml 존재 → scripted 모드 자동 진입
- intrinsic 자동 로드 → solvePnP 경로 사용
- 매 자세 자동 이동 (via_home) + capture + PnP → 샘플 누적
- 끝나면 `cv2.calibrateHandEye` 5-method 비교 → 잔차 최소 결과 → 저장

기대 출력:
```
[info] intrinsic 로드: ... reproj_err=1.109px → solvePnP 경로 사용
[scripted] 25 포즈 순회 시작 ...
[1/25] manual_00 ...
  [pnp] pts=24  reproj rmse=0.45px
  [manual_00] ✓ corners=24  rmse=0.45  t_M_C=...

수집된 샘플: 22
calibrate() 실행...
[HandEye] PARK: t_err=3.55mm  r_err=1.30°
[HandEye] 최적: PARK
[HandEye] 저장: config/calibration/hand_eye_artec.yaml
```

### Step 4 — 시스템 적용

`config/sensor_frames.yaml` 의 `T_EC_artec` 에 `hand_eye_artec.yaml` 의
translation / rotation_quat 복사 (이미 적용됨, 2026-04-29):

```yaml
T_EC_artec:
  translation: [0.00433, -0.17716, -0.05900]
  rotation_quat: [0.28410, 0.71129, 0.60273, -0.22377]
```

`MMSConfig(... sensor_frames_yaml="config/sensor_frames.yaml", T_EC_key="T_EC_artec")`
로 자동 로드.

---

## 6. 시행착오 (Lessons learned)

### 6.1 OpenCV API 호환성

- OpenCV 4.7+ 에서 `cv2.aruco.interpolateCornersCharuco`, `cv2.aruco.calibrateCameraCharuco`
  **deprecated/제거**. 대체:
  - 검출: `cv2.aruco.CharucoDetector(board).detectBoard(image)`
  - 캘리브: `CharucoBoard.matchImagePoints` + `cv2.calibrateCamera`
- `cv2.aruco.detectMarkers` 자유함수도 deprecated → `cv2.aruco.ArucoDetector(dict, params).detectMarkers(image)` 사용

### 6.2 Spider FOV 가 작아서 큰 보드 X

- 처음엔 7×5 (210×150mm) A4 보드 사용 → Spider 작동거리 (170-350mm) 안에 안 들어옴
- ArUco 마커 2-5개만 검출, ChArUco 보간 실패
- **5×3 (100×60mm) 작은 보드** 로 교체 → 마커 6-7개 검출, 안정적

### 6.3 인쇄 스케일 문제

- Windows 사진 인쇄 다이얼로그의 "페이지에 맞춤" / "전체 페이지" 옵션은
  보드 크기를 임의로 늘림 (실측 30-35mm/square, 의도 20mm)
- 알고리즘은 **상대 비율** 만 신경쓰니까 실측값 (18.5mm) 을 `--square-mm` 으로
  넘기면 OK. 또는 Word/PDF 로 100×60mm 직접 지정 인쇄.

### 6.4 종이 휨 / 광택

- A4 종이가 약간 우는 경우 ChArUco 가 평면 가정해서 ~mm 단위 오차
- Spider LED projector + 광택지 → 강한 highlight, 마커 검출 약화
- **단단한 평판 부착 + 무광 복사용지** 권장
- Spider 의 `texture_flash` 만 끄면 LED glare 회피 (단 ambient 충분해야 함)

### 6.5 ChArUco 검출 → ArUco 4-corner fallback

- 마커가 검출됐어도 ChArUco corner 보간이 실패할 때 있음 (인접 마커 부족)
- 우리 detector 는 **ArUco 4-corner 도 추가 입력** 으로 사용 → 마커 6개면
  ChArUco 0개여도 24 점 확보
- solvePnP 4점만 있어도 동작하지만 6+ 권장

### 6.6 UV→3D nearest-vertex = bottleneck (1차 시도 실패)

- mesh 정점 ~20k / 이미지 1.2M px → 정점 1개 ≈ 60 px 영역
- 검출 corner pixel → nearest UV vertex 의 3D 좌표 사용 → 양자화 ~1mm
- **자세별로 편향된 양자화 오차** 가 hand-eye 에 amplify → **잔차 25mm**
- solvePnP 는 픽셀을 그대로 사용 → 양자화 없음 → 잔차 3.55mm

### 6.7 Distortion 모델 overfit (k2/k3 발산)

- 첫 intrinsic 결과: `dist = [-0.31, 10.17, -0.011, 0.011, -98.45]`
- k2=10, k3=-98 은 비정상 (보통 |k2|<5, |k3|<50)
- **데이터가 이미지 가장자리 커버리지 부족** 시 다항식이 발산
- 해결: `--fix-k3` flag 로 k3=0 강제 (안전장치, default ON)

### 6.8 충돌 회피

- 자세 간 직선 보간이 바닥/주변 충돌 위험
- **`--via-home` (default ON)**: 매 자세 이동 전 `go_home(sensor="artec")` 경유
- + `MOVE_SPEED_DEG=10` 느리게, settle 1.5s

---

## 7. 미완 / 향후 개선

| 항목 | 우선순위 | 메모 |
|------|---------|------|
| `T_EC_artec` 자동 갱신 | 낮 | 지금은 hand_eye_artec.yaml → sensor_frames.yaml 수동 복사 |
| `scripts/artec_hand_eye_validate.py` | 중 | 별도 N_test 자세에서 t_M_B 일관성 측정 |
| `--fix-k3` off + 더 다양한 자세로 정밀 distortion | 낮 | 1mm 미만 정밀도 필요할 때 |
| 정밀 sphere jig 기반 캘리브 | 낮 | 0.3-1mm 정밀도. 비용/난이도 증가 |
| Texture vs geometry camera 정합 검증 | 중 | UV 매핑이 정확히 vertex 기반이면 영향 X (현재 가정) |
| Artec SDK 자체 SLAM 결과 활용 | 낮 | 비교 baseline 용 |

---

## 8. 요약

PhoXi 의 SDK 마커 검출 ↔ Artec 의 텍스처 카메라 + cv2.aruco 의 차이를 메우는 데
시행착오가 있었지만, 최종 파이프라인은 **표준 (camera + ChArUco + solvePnP)** 와
거의 동일. 결과 잔차 3.55mm 는 Phase 1 rule-based 360° + ICP 에 충분.

다음 작업: `main.py` 의 `CFG` 를 `ArtecConfig + T_EC_key="T_EC_artec"` 로 갈아끼우고
실제 turntable 스캔 1회 시도.
