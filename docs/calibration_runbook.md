# 캘리브레이션 실행 안내

> **절차만** 수록한다. 원리·근거는 `1_calibration.md` 를 참조한다.
> Windows / `mms-env` 기준이며, 처음 수행하는 사람이 단독으로 완료할 수 있도록 작성하였다.

소요 시간은 1~2시간이다(보드 인쇄 제외). 로봇이 실제로 움직이므로 §2 안전 수칙을
먼저 확인한다.

---

## 0. 수행 필요 여부 확인

장비를 조작하지 않는 읽기 전용 검사다.

```powershell
conda activate mms-env
cd C:\dev\MMS
$env:PYTHONIOENCODING="utf-8"
python scripts\artec\check_calibration.py
```

`FAIL` 이 하나라도 있으면 수행한다.

**현재 저장값 (2026-09-21 취득)**

| 값 | 기록 | 기준(§4) | 판정 |
|---|---|---|---|
| `T_EC` | 20자세 · t_err 8.44mm · r_err 1.73° | ≤ 4mm · ≤ 1.5° | ⚠ **기준 초과** |
| `T_B_F0` | 12점 · residual 0.193mm · 반경 118.71mm | ≥6점 · <2mm · 119mm 근처 | OK |

⚠ `T_EC` 는 기록상 기준을 넘는다. 실물 정밀도가 의심되면 hand-eye 재취득이 우선이다.

---

## 1. 순서

```
① intrinsic (K)  →  ② hand-eye (T_EC)  →  ③ turntable (T_B_F0)
   최초 1회만          스캐너 교체 시        기계 이설 시
```

> ⚠ **③ 은 ② 의 결과를 입력으로 사용한다.** `turntable_calib.py` 가 rim 점을
> `T_CB = T_EB · inv(T_EC)` 로 base 에 변환한 뒤 원을 피팅하기 때문이다.
> **순서를 바꾸면 턴테이블 값이 전부 틀어진다.**

①은 렌즈·센서를 교체하지 않았으면 생략한다.

---

## 2. 준비

### 안전

1. 작업 반경 내 사람·케이블·공구 유무를 육안으로 확인
2. **비상정지 버튼을 손이 닿는 위치에** 배치
3. **Artec Studio 를 완전히 종료**
4. 장비 3종 상태 확인: `python scripts\check_devices.py`

> ⚠ **스캐너는 SDK 와 Artec Studio 중 하나만 점유할 수 있다.** Studio 가 실행 중이면
> 목록 조회는 되고 **열기만 실패**한다(`createScanner failed (ErrorCode=0xC0050000)`).
> 창을 닫아도 프로세스가 잔류하는 경우가 있다.
>
> ```powershell
> Get-Process astudio_pro, artec-ray-server -ErrorAction SilentlyContinue
> Stop-Process -Name astudio_pro -Force
> ```

### ChArUco 보드

`mms_artec\sensor\markerboard_img\charuco_7x5_12_9.png`(기본 `spider_dense`,
7×5 · 84×60mm)를 **실척 100%** 로 인쇄한다. 없으면
`python scripts\artec\make_charuco.py` 로 생성한다.

> ⚠ **구 보드 `charuco_5x3_20_15.png` 를 사용하지 않는다.** 내부 코너가 8개뿐이라
> Spider FOV 에서 잘리면 해가 구해지지 않는다.
>
> ⚠ **인쇄 배율이 틀리면 모든 값이 조용히 틀어진다.** 인쇄물의 검은 사각형 한 변을
> 자로 재어 **12.0mm** 인지 확인한다. "페이지에 맞춤" 을 해제한다. 작은 보드이므로
> **20 px/mm(508 DPI)** 인쇄를 권장한다.

평평한 판에 주름·들뜸 없이 부착한다. 휘면 각도 오차가 그대로 반영된다.

### 배치와 조준

보드를 **턴테이블 원판 위에** 올리고, **보드와 원판 가장자리(rim)가 한 화면에**
들어오면서 작동거리가 약 320mm 가 되도록 맞춘다. 좌표를 계산하여 입력하는 것보다
팔을 수동으로 이동시키는 편이 빠르다.

```powershell
python scripts\artec\aim.py
```

로봇 웹 UI 와 스캐너 실시간 뷰어를 함께 표시한다. 조준에는 두 가지가 동시에 필요하다 —
팔을 이동시키는 수단과, 현재 촬영되는 화면이다.

1. xArm Studio 에서 **Manual Mode** 활성화 (중력보상)
2. 보드가 **화면 중심 십자**에 오도록 맞춘다
3. 뷰어 판정을 확인한다

| 표시 | 의미 |
|---|---|
| `OK  markers 7/8  ~250mm` | 조준 완료 |
| `잘림 - 뒤로/중앙으로` | 보드가 프레임 가장자리에서 잘리고 있다 |
| `일부만` / `보드 없음` | 추가 조정 필요 |

4. `OK` 후 손을 뗀다 — **놓기 전에 자세 유지 여부를 확인**한다(툴 체인 265mm 무게로
   처진다)
5. **Manual Mode 해제**
6. 뷰어를 `q` 로 종료 — 스캐너를 해제해야 다음 단계가 점유할 수 있다

> **⚠ 브라우저 탭을 반드시 닫는다.** 웹 UI 가 열려 있으면 컨트롤러의 mode/state 를
> 웹 쪽이 계속 변경하여 SDK 명령이 무시되거나 예상과 다르게 동작한다
> (`robot_control.md` §1).

확인·미세조정은 스크립트로 수행한다.

```powershell
python scripts\robot\status.py                    # 현재 TCP·에러 (이동 없음)
python scripts\robot\jog.py --dz 20 --dry-run     # 확인 후
python scripts\robot\jog.py --dz 20               # 실행
python scripts\robot\home.py                      # 기준 자세
```

> 보드를 원판 위에 두는 이유는 ②와 ③을 동일 조준 자세에서 연속 수행하기 위함이다.

---

## 3. 실행

```powershell
conda activate mms-env
cd C:\dev\MMS
$env:PYTHONIOENCODING="utf-8"

python scripts\artec\calibrate.py                  # 1→2→3→4 전체
python scripts\artec\calibrate.py --from 2         # intrinsic 생략
python scripts\artec\calibrate.py --only 3         # 턴테이블만
```

`calibrate.py` 는 자체 로직 없이 순서대로 호출하고 실패 시 중단한다. 인자를 전달하려면
`--only` 와 함께 `--` 뒤에 둔다.

| 인자 | 기본 | 내용 |
|---|---|---|
| `--from N` / `--only N` | 1 / — | N 단계부터 / N 단계만 |
| `--via-home` | **꺼짐** | 1·2단계에서 자세마다 home 경유 (느리다) |
| `--skip-aim` | 꺼짐 | 시작 시 수동 조준 안내 생략 |

> **home 경유는 기본 해제다.** 경유하면 1·2단계 이동량이 1208° → 4230° 로 증가하여
> 2.0분이 7.1분이 된다. 안전 근거는 **자세 목록이 경로 검사를 거쳤다는 점**이다.
> `gen_calib_poses.py` 가 자세 간 경로를 충돌 게이트로 확인하고 이동거리 최소 순서로
> 재배열한 뒤 yaml 에 `path_checked: true` 를 기록하며, `calibrate.py` 가 이를 보고
> 자동 판단하므로 **통상 인자가 불필요하다.**
>
> ```
> [poses] 경로 검사 완료 (자세간 28.4°) — home 경유 생략
> [poses] 경로 검사 표식 없음 — home 을 경유한다 (느림)     ← 구 목록인 경우
> ```
>
> 검사 시점의 레이아웃이 현재와 다르면(`ACTIVE_LAYOUT.txt`) 검사 결과가 무효이므로
> 역시 경유한다. **빠른 쪽이 기본이되 근거가 없으면 안전한 쪽으로 전환된다.**

### 단계별 인자

> ⚠ **`--via-home` 과 `--no-home` 은 서로 다른 인자다.** `--via-home` 은 1·2단계에서
> 자세 순회 시 매번 home 을 경유하는 것이고, `--no-home` 은 3단계에서 시작 시 아무
> 곳으로도 이동하지 않는 것이다.

**① intrinsic / ② hand-eye**

| 인자 | 기본 | 내용 |
|---|---|---|
| `--poses PATH` | `artec_calibration_poses.yaml` | 자세 목록 |
| `--board NAME` | `spider_dense` | 보드 프리셋 |
| `--square-mm V` | 프리셋값 | 인쇄 실측 보정 |
| `--start-from N` | 0 | N 번째 자세부터 (중단 후 재개) |

**③ 턴테이블 rim**

| 인자 | 기본 | 내용 |
|---|---|---|
| — | 기록된 rim 자세로 이동 | `artec_rim_pose.yaml` 이 있으면 그 자세로 |
| `--force-home` / `--no-home` | — | home 으로 / 현재 자세 그대로 |
| `--save-pose` | — | 현재 자세를 rim 시작 자세로 기록 |
| `--sensitivity V` | 0.9 | 재구성 민감도. 흰 상판은 0.5 로는 정점이 거의 나오지 않는다 |
| `--range-mm N F` | 170 330 | 스캔 깊이 범위. 좁히면 배경 잡음 감소 |
| `--jog-step MM` / `--jog-rot-step DEG` | 20 / 10 | 창 내 조그 단위 |

**rim 창 조작키**

| 키 | 동작 |
|---|---|
| 좌클릭 / 우클릭 | rim 점 추가 / 취소 |
| Enter · Space | 원 피팅 → 저장 확인 → 결과 오버레이 |
| `m` | 웹 UI 수동 모드 + 라이브 화면 |
| `v` | 3D 메시 ↔ 텍스처 전환 |
| `S` | 현재 자세를 rim 시작 자세로 저장 |
| `w s a d e c` | base 프레임 ±x ±y ±z 이동 · `[ ]` 스텝 조절 |
| `o p` | 광축 roll (화면만 회전, 관측점 유지) |
| `z x` / `t g` | J7 / J6 회전 · `, .` 스텝 |
| `r` / `h` / `q` | 재캡처 / home / 종료 |

> rim 은 원판 전체가 화각에 들어오지 않는다(지름 약 238mm, 화각 320mm 에서 123×167mm).
> 목표는 전체가 아니라 **화면을 가로지르는 긴 호**이며, 그 호를 따라 **6~10점을 넓게**
> 취득한다. 3점은 잔차가 항상 0 이라 검증이 성립하지 않는다.

### 결과 육안 검증

숫자(점 수·잔차·반경)가 전부 통과해도 축이 잘못된 위치에 있을 수 있다.

```powershell
python scripts\artec\turntable_overlay.py                 # 현재 자세 (이동 없음)
python scripts\artec\turntable_overlay.py --pose rim      # 기록된 rim 자세 ⚠ 이동함
```

자홍 = 실제 테두리, 빨강 = 피팅 원, 초록 = 실측 반경(119mm), 청록 = 축이다.
**빨강과 초록이 모두 자홍에서 벗어나면 반경이 아니라 축의 위치·기울기가 틀린 것이다.**
`--only 3` 을 저장까지 완료하면 이 창이 자동으로 표시된다.

### ⚠ 자세 목록이 현재 배치와 일치해야 한다

`calibrate.py` 는 yaml 의 자세를 순회하며 보드를 촬영한다. **턴테이블이나 보드 위치가
변경되었으면 해당 자세들은 보드를 관측하지 못한다.** 실제로 23자세 중 17개가 이 사유로
배제되어 intrinsic 이 최소 프레임(8)을 충족하지 못한 사례가 있다.

자세는 수동 조작 없이 자동 생성한다.

```powershell
# ★ 권장 — 현재 보이는 보드 기준 (T_EC 계통 오차가 1차 상쇄된다)
python scripts\artec\gen_calib_poses.py --from-view --write

# 셀 모델 기준 (로봇·스캐너 없이 오프라인)
python scripts\artec\gen_calib_poses.py                        # ① 원판 후보 확인
python scripts\artec\gen_calib_poses.py --hint-xy 0.838 -0.022 --write
```

| 인자 | 기본 | 내용 |
|---|---|---|
| `--from-view` | 꺼짐 | 현재 보이는 ChArUco 를 검출하여 기준점으로 사용 |
| `--center X Y Z` / `--hint-xy X Y` | — | 기준점 직접 지정 / 셀 모델 원판 선택 힌트 |
| `--standoff M` | 0.32 | 조준 여유 기준값 (스캔 거리가 아니다) |
| `--rolls ...` | −45…110 (8개) | **좁히지 않는다** — 90° 로 축소하여 유효 자세가 81→9 로 감소한 사례가 있다 |
| `--max-poses N` | 20 | 상한 |
| `--fov-margin-px V` / `--no-fov-gate` | 40 / 꺼짐 | 프레임 진입 검사 여유 / 해제 |
| `--write` | 꺼짐 | 저장 (없으면 미리보기) |

저장 시 이동거리 최소 순서로 재배열하고 자세 간 경로를 충돌 검사한다. 충돌 셀 모델에서
턴테이블 원판을 찾아 그 위 반구에 자세를 배치하고 해석 IK + 충돌 게이트로 선별하며,
sim 이 사용하는 생성기와 **동일한 코드**다. 로봇도 스캐너도 필요하지 않다.

배제 조건은 IK 실패 · 관절 한계 여유 5° 미만 · 자세 충돌 · **home→자세 경로 충돌**
네 가지다. 마지막 둘이 없으면 충돌 사고가 재현된다.

생성된 자세는 **관절각(`joints`)으로 저장된다.** `ee_pose` 만 저장하면 `hand_eye_calib`
이 컨트롤러 IK 로 다시 풀어서 이동하는데, 7축이라 동일 TCP 에 해가 무한히 많아
**검증한 자세와 실제 이동 자세가 달라진다**(실측 Δq 140~251°) — `collision.md` T7.

①의 후보 목록은 **자동 선택하지 않는다.** 셀 점군은 부재 이름이 전부 `mesh` 라 무엇이
턴테이블인지 데이터만으로 구별할 수 없으며, 자동 선택이 키보드를 선택한 사례가 있다.

```
  원판 후보 (base 프레임):
    --hint-xy -0.257 0.006   상면 z=0.778  r95=143mm  점 125,013
    --hint-xy  0.838 -0.022  상면 z=0.694  r95= 75mm  점 10,444   ← 턴테이블
```

판별이 어려우면 씬을 육안으로 확인한다: `$ISAAC scripts\sim\view_scene.py v2`

#### `charuco 부족` 이 대부분인 경우

조준 기준점이 실제 보드와 다른 것이다. 셀 모델의 원판 중심을 조준하는데 보드가 그
위치에 없으면 Spider 의 좁은 FOV 에서 보드가 잘린다. 실측 사례에서 20자세 중 18자세가
`charuco 부족` 이었고, 마커 픽셀 크기로 역산한 거리는 262mm(의도 250mm)로 **정상이었다** —
거리가 아니라 조준의 문제였다.

→ `aim.py` 로 조준한 뒤 `gen_calib_poses.py --from-view --write` 로 재생성한다.
여기서 추정한 중심을 동일한 `T_EC` 로 다시 조준하므로 `T_EC` 의 계통 오차가 1차
상쇄된다.

조준을 맞췄는데도 잘리면 FOV 여유가 부족한 것이므로 거리를 올린다.

| standoff | 화각(가로×세로) | 보드 이탈 허용 여유 | 유효 자세 |
|---|---|---|---|
| 250mm | 134×100mm | 17 × 20mm | 93 |
| **320mm** (기본) | 171×128mm | **36 × 34mm** | 85 |
| 350mm | 188×140mm | 44 × 40mm | 73 |

거리를 올려도 도달성 손실은 거의 없다(필요한 자세는 20개). 작동거리 상한이 350mm 이므로
그 이상은 사용하지 않는다. 기존 yaml 은 `.yaml.bak` 으로 백업된다.

### hand-eye 수행 시 유의

- 자세 **15~25개**
- 자세 간 회전 **30° 이상** — 회전이 작으면 `AX=ZB` 가 풀리지 않는다
- 보드가 화면을 벗어난 자세는 배제된다

---

## 4. 합격 기준

| 값 | 기준 | 비고 |
|---|---|---|
| hand-eye `t_err` | **≤ 4 mm** | 참고 기준값 3.55mm (2026-04-29) |
| hand-eye `r_err` | **≤ 1.5°** | 참고 기준값 1.30° |
| 턴테이블 rim 점 수 | **≥ 6** | 3점은 미결정 |
| 턴테이블 residual | **< 2 mm** | 5mm 초과 시 재작업 |
| 턴테이블 rim 반경 | **119 mm 근처** | 실측 원판 반경 |

산출물

```
config\calibration\artec_intrinsic.yaml      ← ①
config\sensor_frames.yaml::T_EC_artec        ← ②
config\calibration\turntable_frame.yaml      ← ③
```

---

## 5. 완료 후 검산

```powershell
python scripts\artec\check_calibration.py       # 수치 검사
python scripts\artec\turntable_overlay.py       # 육안 검사
```

**두 가지를 모두 수행한다.** `check_calibration.py` 는 점 수·잔차·반경만 확인한다.
점들이 일관된 원을 이루는지는 알려주지만 그 원이 **실제 원판 위에 있는지**는 판단하지
못한다. 실제로 점 15개·잔차 0.18mm·반경 116mm 로 지표가 전부 통과했으나 투영 결과
실제 테두리와 43mm 어긋난 사례가 있다(원인은 `T_EC` 회전 오차 5~7°).

`check_calibration.py` 는 **두 독립 출처를 교차검증**한다 — 캘리브 결과(`T_B_F0`)가
가리키는 위치에 충돌 캐시(실측 셀 모델)의 구조물이 실제로 존재하는지 확인한다.

```
교차검증 — 활성 레이아웃 'v2_real_260917' 기준 게이트 사각 0.3% · p99 20mm
[OK]   충돌 캐시가 캘리브된 턴테이블을 덮고 있다
```

두 값이 독립이므로 **일치하면 서로를 보증하고**, 불일치하면 둘 중 하나가 틀린 것이다.
셀 모델 쪽이 의심되면 `collision.md` §6 을 참조한다.

통과 후 실제 스캔으로 확인한다.

```powershell
python main_artec.py          # BACKEND = "real" 확인
```

---

## 6. 문제 해결

| 증상 | 원인 / 조치 |
|---|---|
| `createScanner failed (ErrorCode=0xC0050000)` | **Artec Studio 가 스캐너를 점유 중**이다(§2). `enumerate` 는 되는데 `open` 만 실패하면 거의 항상 이 경우다 |
| `charuco 부족 … — skip` 이 다수 | 자세 목록이 현재 배치와 불일치 → `gen_calib_poses.py --from-view --write`. **거리부터 의심하지 않는다** |
| `⚠ 최소 8 frame 필요. 종료.` | 위와 동일 원인. exit 1 로 중단된다 |
| hand-eye 가 intrinsic 날짜를 확인 요청 | 90일 초과 intrinsic 이다. 스캐너를 교체했으면 `N` 으로 중단하고 intrinsic 부터 수행 |
| 뷰어가 빈 화면만 표시 | `텍스처 프레임을 N회 연속 못 받았다` 경고 확인 — Artec Studio 실행 중이거나 스캐너 앞이 비어 있다 |
| 캡처 이미지 확인 | `debug_intrinsic_artec/<pose>.png` (intrinsic) · `debug_calib_artec/<pose>_raw.png`, `_aruco.png` (hand-eye) |
| 보드 미검출 | 인쇄 배율(§2) · 조명 반사 · 작동거리 이탈 |
| hand-eye 오차 과다 | 자세 간 회전 30° 이상 확보, 자세 15개 이상 |
| 특정 자세만 반복 실패 | IK 시드 문제일 수 있다 — `1_calibration.md` T1 |
| residual 이 0.0 | rim 점이 3개다. **정상이 아니라 미결정** — 6점 이상으로 재수행 |
| 로봇 "도달 불가" | `--dry-run` 으로 확인 후 조준 자세 재설정 |
| `cv2.calibrateHandEye` 없음 | OpenCV 5.x 다 → `pip install "opencv-python<5"` |
| 콘솔 문자 깨짐 | `$env:PYTHONIOENCODING="utf-8"` |
| 로봇이 명령을 무시 | **웹 UI 탭이 열려 있다** — 조준 완료 후 닫는다 (§2) |
| 수동 모드에서 팔이 움직이지 않음 | 수동 모드 미활성 또는 `error_code != 0` → `python scripts\robot\recover.py` |

---

## 7. 재수행 시점

| 사건 | 재취득 대상 |
|---|---|
| 스캐너 교체 | **T_EC** (개체마다 광학 프레임이 다르다) → T_B_F0 |
| 스캐너 탈착 후 재장착 | T_EC |
| 턴테이블 이설·재조립 | T_B_F0 |
| 프레임·로봇 마운트 이동 | T_B_F0 + **충돌 캐시** (`collision.md` §6) |
| 렌즈·센서 교체 | intrinsic 부터 전부 |
| 로봇 교체 | 위 전부 + **`xarm7_dh.yaml` 재교정** (개체 종속) |

---

## 관련 문서

- 원리·근거·함정 — `1_calibration.md`
- 로봇 수동 조작 — `robot_control.md`
- 셀 모델이 실물과 다를 때 — `collision.md` §6
