# 캘리브레이션 실행 안내

> **손으로 따라 하는 절차만** 담는다. 원리·근거는 `1_calibration.md`.
> Windows / `mms-env` 기준. 처음 하는 사람이 혼자 끝낼 수 있게 쓴 문서다.

소요 **1~2시간** (보드 인쇄 제외). 로봇이 움직인다 — §2 안전수칙 먼저.

---

## 0. 지금 해야 하나?

먼저 검산부터. **장비를 건드리지 않는 읽기 전용**이다.

```powershell
conda activate mms-env
cd C:\dev\MMS
$env:PYTHONIOENCODING="utf-8"
python scripts\artec\check_calibration.py
```

`FAIL` 이 하나라도 있으면 해야 한다. 전부 `OK` 면 안 해도 된다.

2026-09-15 현재 상태 — **둘 다 해야 한다:**

| 값 | 문제 |
|---|---|
| `T_EC` (hand-eye) | 구 스캐너 `SP.10.36181288` 기준. 현재는 `SP.10.79103441` — **카메라 광학 프레임은 개체마다 다르다** |
| `T_B_F0` (턴테이블 축) | rim **3점**·residual 0.0 = 미결정 피팅(3점은 원이 항상 정확히 지나간다). 게다가 좌표가 실측 셀 모델과 안 맞는다(§5) |

---

## 1. 순서 — 이것만 지키면 된다

```
① intrinsic (K)  →  ② hand-eye (T_EC)  →  ③ turntable (T_B_F0)
   최초 1회만          스캐너 바꾸면         기계 옮기면
```

> ⚠ **③ 은 ② 의 결과를 입력으로 받는다.** `turntable_calib.py` 가 rim 점을
> `T_CB = T_EB · inv(T_EC)` 로 base 에 옮긴 뒤 원을 피팅하기 때문이다.
> **순서를 바꾸면 턴테이블 값이 통째로 틀린다.**

①은 렌즈·센서를 바꾸지 않았으면 건너뛴다.

---

## 2. 준비

### 안전 — 로봇이 실제로 움직인다

1. 작업 반경에 사람·케이블·공구 없는지 **눈으로**
2. **비상정지 버튼이 손에 닿는 곳에**
3. **Artec Studio 를 완전히 종료한다** — 아래 ⚠
4. 장비 3종 살아 있는지: `python scripts\check_devices.py`

> ⚠ **스캐너는 SDK 와 Artec Studio 중 하나만 잡을 수 있다.** Studio 가 떠 있으면
> 목록 조회는 되는데 **열기만 실패**한다:
>
> ```
> RuntimeError: [ArtecSDK] createScanner failed (ErrorCode=0xC0050000)
> ```
>
> 창을 닫아도 프로세스가 남는 경우가 있다. 확인하고 죽인다:
>
> ```powershell
> Get-Process astudio_pro, artec-ray-server -ErrorAction SilentlyContinue
> Stop-Process -Name astudio_pro -Force
> ```

> 웹 UI 는 **조준에 쓰고 나서 닫는다** — 순서가 있다. 아래 〈배치와 조준〉 참고.

### ChArUco 보드

`mms_artec\sensor\markerboard_img\charuco_5x3_20_15.png` 를 **실척(100%)** 으로 인쇄한다.
없으면 `python scripts\artec\make_charuco.py` 로 다시 만든다.

> ⚠ **인쇄 배율이 틀리면 모든 값이 조용히 틀린다.** 인쇄물의 검은 사각형 한 변을
> 자로 재서 **20.0mm** 인지 확인한다. "페이지에 맞춤" 옵션을 끌 것.

평평한 판에 **주름·들뜸 없이** 붙인다. 휘면 각도 오차가 그대로 들어간다.

### 배치와 조준 — 웹 UI 수동 모드가 가장 빠르다

보드를 **턴테이블 원판 위에** 올린 뒤, **보드와 원판 가장자리(rim)가 한 화면에**
들어오고 작동거리가 **약 250mm**(Spider 스윗스팟 200~300mm)가 되게 맞춘다.

좌표를 계산해 넣는 것보다 **팔을 손으로 끌어다 맞추는 쪽이 빠르다.**

#### ① 웹 UI 수동 모드로 조준

```
http://192.168.1.210:18333
```

UFACTORY 가 컨트롤러에 내장한 웹 앱이다. 브라우저만 있으면 되고 설치가 필요 없다.

1. **Manual Mode(수동 모드)** 를 켠다 — 중력보상이 걸려 팔을 손으로 밀 수 있다
2. 스캐너를 잡고 **보드 + rim 이 같이 보이는 자리**로 가져간다
3. 거리는 눈대중 25cm 로 충분하다 — 캘리브가 자세를 다시 순회한다
4. 맞으면 **수동 모드를 끈다**

> ⚠ 수동 모드에서는 팔이 자유롭게 움직인다. 손을 떼기 전에 자세가 유지되는지
> 확인할 것 — 툴 무게(스캐너 체인 265mm)로 처질 수 있다.

#### ② ★ 브라우저 탭을 닫는다

**빼먹으면 이후 스크립트가 통째로 안 먹는다.** 웹 UI 가 열려 있으면 컨트롤러의
mode/state 를 웹 쪽이 계속 바꿔서, SDK 로 보낸 명령이 무시되거나 예상 밖으로 동작한다
(`robot_control.md` §1). 캘리브·`main_artec.py` 를 돌릴 때는 **탭을 닫아 둔다.**

#### ③ 확인·미세조정은 스크립트로

```powershell
python scripts\robot\status.py                    # 현재 TCP·에러 (안 움직임)
python scripts\robot\jog.py --dz 20 --dry-run     # 먼저 확인
python scripts\robot\jog.py --dz 20               # 실행
```

수동 모드를 못 쓰는 상황이면 `home.py` 로 기준 자세를 잡고 `jog.py` 로 옮긴다.

```powershell
python scripts\robot\home.py
```

> 보드를 원판 위에 두는 이유 — ②와 ③을 **같은 조준 자세에서 이어서** 할 수 있어
> 조준을 한 번만 하면 된다.

---

## 3. 실행

```powershell
conda activate mms-env
cd C:\dev\MMS
$env:PYTHONIOENCODING="utf-8"
```

### 한 번에 (권장)

```powershell
python scripts\artec\calibrate.py            # 1→2→3 순서대로
python scripts\artec\calibrate.py --from 2   # intrinsic 건너뛰고
python scripts\artec\calibrate.py --only 3   # 턴테이블만
```

`calibrate.py` 는 **자체 로직이 없다.** 순서대로 부르고, 실패하면 거기서 멈춘다.
개별로 불러도 결과는 같다.

### 개별로

```powershell
python scripts\artec\intrinsic_calib.py      # ① K (최초 1회)
python scripts\artec\hand_eye_calib.py       # ② T_EC
python scripts\artec\turntable_calib.py      # ③ T_B_F0 (rim 클릭)
```

### ⚠ 자세 목록(`artec_calibration_poses.yaml`)이 지금 배치와 맞아야 한다

`calibrate.py` 는 yaml 에 기록된 자세를 순회하며 보드를 찍는다. **턴테이블이나 보드
위치가 바뀌었으면 그 자세들은 보드를 못 본다.** 2026-09-15 현장에서 23자세 중 17개가
이렇게 버려져 intrinsic 이 최소 프레임(8)을 못 채웠다:

```
[manual_01] charuco 부족 (aruco=0, charuco=0) — skip
[manual_06] charuco 부족 (aruco=0, charuco=0) — skip
...
수집 frames: 6
⚠ 최소 8 frame 필요. 종료.
```

**자세를 다시 만든다** — 손으로 잡을 필요 없이 자동 생성된다.

```powershell
python scripts\artec\gen_calib_poses.py                        # ① 원판 후보 확인
python scripts\artec\gen_calib_poses.py --hint-xy 0.838 -0.022 # ② 미리보기
python scripts\artec\gen_calib_poses.py --hint-xy 0.838 -0.022 --write   # ③ 저장
```

**충돌 셀 모델**(`4_collision.md` §6)에서 턴테이블 원판을 찾아 그 위 반구에 자세를
깔고, 해석 IK + 충돌 게이트로 거른다. sim 이 쓰는 생성기와 **같은 코드**다.

- 로봇도 스캐너도 필요 없다 — **오프라인**에서 만든다
- `T_EC` 에 의존하지 않는다(겨누는 용도로만 쓴다) → 지금 구하려는 값과 순환이 없다
- 반구: polar 0/15/30/45 × 8방위 = 25후보 → 게이트 통과 **14~15개**

①에서 후보 목록이 나온다. **자동으로 고르지 않는다** — 셀 점군은 부재 이름이 전부
`mesh` 라(§6.1) 무엇이 턴테이블인지 데이터만으로 구별할 수 없다. 실제로 자동 선택이
**키보드**를 집은 적이 있다.

```
  원판 후보 (base 프레임):
    --hint-xy -0.257 0.006   상면 z=0.778  r95=143mm  점 125,013
    --hint-xy  0.838 -0.022  상면 z=0.694  r95= 75mm  점 10,444   ← 턴테이블
```

어느 것인지 모르겠으면 씬을 눈으로 본다: `$ISAAC scripts\sim\view_scene.py v2`

> 기존 yaml 은 `.yaml.bak` 으로 백업된다.

### ② hand-eye 를 돌릴 때

로봇이 여러 자세를 순회하며 보드를 찍는다.

- 자세 **15~25개**
- 자세 사이 회전 **30° 이상** — 회전이 작으면 `AX=ZB` 가 안 풀린다
- 보드가 화면 밖으로 나가면 그 자세는 버려진다

기록된 자세로 다시 돌리려면:

```powershell
python scripts\artec\hand_eye_calib.py --poses config\calibration\artec_calibration_poses.yaml
```

### ③ 턴테이블 rim 클릭

창이 뜨면 **원판 가장자리**를 클릭한다.

| 조작 | |
|---|---|
| 좌클릭 | 점 추가 |
| 우클릭 | 되돌리기 |
| `Enter` | 피팅 |
| `r` | 다시 캡처 |
| `q` | 종료 |

> ⚠ **최소 3점이지만 3점으로 끝내지 말 것.** 3점을 지나는 원은 **항상 정확히 하나**라
> residual 이 0 으로 나오고, 그건 "정확하다"가 아니라 **"검증이 불가능하다"** 는 뜻이다.
> 지금 저장돼 있는 값이 바로 그 상태다.
>
> **6점 이상**, 가능한 한 **원주에 고르게** 흩어서 찍는다. rim 이 한 화면에 다 안
> 들어오면 보이는 호(arc)에서 찍되, 호가 짧으면 조건수가 나빠진다 — 로봇을 옮겨
> 반대쪽 호도 찍는 게 낫다.

---

## 4. 합격 기준

| 값 | 기준 | 비고 |
|---|---|---|
| hand-eye `t_err` | **≤ 4 mm** | 실물 기준값 3.55mm (2026-04-29) |
| hand-eye `r_err` | **≤ 1.5°** | 실물 기준값 1.30° |
| 턴테이블 rim 점 수 | **≥ 6** | 3점은 미결정 |
| 턴테이블 residual | **< 2 mm** | 5mm 넘으면 재작업 |
| 턴테이블 rim 반경 | **119 mm 근처** | 실측 원판 반경 |

산출물:

```
config\calibration\artec_intrinsic.yaml      ← ①
config\sensor_frames.yaml::T_EC_artec        ← ②
config\calibration\turntable_frame.yaml      ← ③
```

---

## 5. 끝나고 반드시 검산

```powershell
python scripts\artec\check_calibration.py
```

이 도구는 **두 독립 출처를 교차검증**한다 — 캘리브 결과(`T_B_F0`)가 가리키는 자리에
충돌 캐시(실측 셀 모델)의 구조물이 실제로 있는지 본다. 엉뚱한 곳을 가리키면 여기서 걸린다.

```
교차검증 — 활성 레이아웃 'v2_layout_real' 에서 이 좌표 15cm 안의 셀 점: 0개
[FAIL] 그 자리에 아무것도 없다
```

↑ **지금 저장된 값이 내는 결과다.** 턴테이블 축이 셀 모델과 전혀 다른 곳을 가리킨다
(`T_B_F0` 원점 `[-0.766, -0.013, -0.686]`, base 기준 1.03m). 재캘리브가 필요한 이유가
숫자로 드러난 것이다.

> 셀 모델 쪽이 의심되면 `docs/4_collision.md` §6 을 본다. 두 값이 독립이라
> **둘이 맞으면 서로를 보증**하고, 안 맞으면 둘 중 하나가 틀린 것이다.

통과하면 실제 스캔으로 확인한다:

```powershell
python main_artec.py          # BACKEND = "real" 확인
```

---

## 6. 잘 안 될 때

| 증상 | 원인 / 조치 |
|---|---|
| `createScanner failed (ErrorCode=0xC0050000)` | **Artec Studio 가 스캐너를 점유 중**이다. 완전히 종료할 것 (§2). `enumerate` 는 되는데 `open` 만 실패하면 거의 항상 이것이다 |
| `charuco 부족 (aruco=0…) — skip` 이 많다 | 자세 목록이 지금 배치와 안 맞는다 → `--interactive` 로 다시 딴다 (§3) |
| `⚠ 최소 8 frame 필요. 종료.` | 위와 같은 원인. **이 경우 exit 1 로 멈춘다** — 예전엔 조용히 통과해 다음 단계가 낡은 값을 썼다 |
| hand-eye 가 intrinsic 날짜를 묻는다 | 90일 넘은 intrinsic 이다. 스캐너를 바꿨으면 **`N` 을 눌러 중단**하고 intrinsic 부터 |
| 보드를 못 찾는다 | 인쇄 배율(§2) · 조명 반사 · 작동거리 250mm 벗어남 |
| hand-eye 오차가 크다 | 자세 간 회전이 작다 → **30° 이상** 확보. 자세 수를 15개 이상으로 |
| 특정 자세만 계속 실패 | IK 시드 문제일 수 있다 — `1_calibration.md` T1 |
| residual 이 0.0 인데 이상하다 | rim 점이 3개다. **정상이 아니라 미결정** — 6점 이상 다시 |
| 로봇이 "도달 불가" | `--dry-run` 으로 확인, 조준 자세를 다시 잡는다 |
| `cv2.calibrateHandEye` 없음 | OpenCV 5.x 다 → `pip install "opencv-python<5"` (`install.md` §2) |
| 콘솔 글자 깨짐 | `$env:PYTHONIOENCODING="utf-8"` |
| 로봇이 명령을 무시한다 | **웹 UI 탭이 열려 있다** — 조준 끝났으면 닫는다 (§2 ②) |
| 수동 모드에서 팔이 안 밀린다 | 수동 모드가 안 켜졌거나 `error_code != 0` → `python scripts\robot\recover.py` |

---

## 7. 언제 다시 해야 하나

| 사건 | 다시 잡을 것 |
|---|---|
| 스캐너 교체 | **T_EC** (개체마다 광학 프레임이 다르다) · 그 다음 T_B_F0 |
| 스캐너 탈착 후 재장착 | T_EC |
| 턴테이블 이설·재조립 | T_B_F0 |
| 프레임·로봇 마운트 이동 | T_B_F0 + **충돌 캐시** (`4_collision.md` §6) |
| 렌즈·센서 교체 | intrinsic 부터 전부 |
| 로봇 교체 | 위 전부 + **`xarm7_dh.yaml` 재교정** (개체 종속) |

---

## 관련 문서

- 원리·근거·함정 — `1_calibration.md`
- 좌표 규약 · 단위 — `README.md` §규약
- 로봇 수동 조작 — `robot_control.md`
- 셀 모델이 실물과 다를 때 — `4_collision.md` §6
- 설치 — `install.md`
