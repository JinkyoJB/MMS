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

`mms_artec\sensor\markerboard_img\charuco_7x5_12_9.png`(기본 `spider_dense`,
7×5 · 84×60mm)를 **실척(100%)** 으로 인쇄한다. 없으면
`python scripts\artec\make_charuco.py` 로 다시 만든다.

> ⚠ **구 보드 `charuco_5x3_20_15.png` 를 쓰지 말 것.** 내부 코너가 8개뿐이라
> Spider FOV 에서 잘리면 캘리브가 풀리지 않는다 (`1_calibration.md` §2).
>
> ⚠ **인쇄 배율이 틀리면 모든 값이 조용히 틀린다.** 인쇄물의 검은 사각형 한 변을
> 자로 재서 **12.0mm** 인지 확인한다. "페이지에 맞춤" 옵션을 끌 것.
> 작은 보드라 **20 px/mm(508 DPI)** 로 인쇄하는 편이 안전하다.

평평한 판에 **주름·들뜸 없이** 붙인다. 휘면 각도 오차가 그대로 들어간다.

### 배치와 조준 — 웹 UI 수동 모드가 가장 빠르다

보드를 **턴테이블 원판 위에** 올린 뒤, **보드와 원판 가장자리(rim)가 한 화면에**
들어오고 작동거리가 **약 320mm** 가 되게 맞춘다 — 거리를 이렇게 잡는 이유는 §3 거리표.

좌표를 계산해 넣는 것보다 **팔을 손으로 끌어다 맞추는 쪽이 빠르다.**

#### ① 한 방에 — `aim.py`

```powershell
python scripts\artec\aim.py
```

로봇 웹 UI(브라우저)와 **스캐너 실시간 뷰어**를 같이 띄운다. 조준은 이 둘을
동시에 봐야 한다 — 팔을 끄는 수단과, 지금 뭐가 찍히는지.

1. 브라우저의 xArm Studio 에서 **Manual Mode** 를 켠다 — 중력보상이 걸려 손으로 민다
2. 스캐너를 잡고 보드가 **화면 중심 십자(자홍색)** 에 오도록 맞춘다
3. 뷰어가 그 자리에서 판정한다:

| 표시 | 뜻 |
|---|---|
| `OK  markers 7/8  ~250mm` | 조준 완료 |
| `잘림 - 뒤로/중앙으로` | 노란 박스가 프레임 가장자리에 닿았다 — 보드가 잘리고 있다 |
| `일부만` / `보드 없음` | 더 맞춰야 한다 |

4. `OK` 가 뜨면 손을 뗀다 — **놓기 전에 자세가 유지되는지 확인**(툴 체인 265mm 무게로 처진다)
5. **Manual Mode 를 끈다**
6. 뷰어를 `q` 로 닫는다 — **스캐너를 놔줘야 다음 단계가 잡는다**

> 뷰어만 따로 쓰려면 `python scripts\artec\live_view.py`
> (`s` 키로 스냅샷 → `output\live_view\`).
>
> ⚠ **Artec Studio 를 닫아야 한다** — 스캐너는 SDK 와 Studio 중 하나만 잡는다.
> `aim.py` 가 실행 전에 확인하고 막아준다.

**왜 이게 필요한가** — Artec Studio 는 스캔 모드 미리보기만 주고, 리포의 다른
뷰어(`live_scan_view.py` 등)는 전부 **점군**이다. "보드가 화면 안에 다 들어왔나"는
텍스처 이미지라야 보인다. 2026-09-15 에 이게 없어서 20자세를 다 돌린 **뒤에야**
18자세가 보드 잘림으로 버려진 걸 알았다.

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
cd <리포 경로>          # 이 PC 는 C:\Users\user\workspace\MMS
$env:PYTHONIOENCODING="utf-8"
```

### 한 번에 (권장)

```powershell
python scripts\artec\calibrate.py                  # 1→2→3→4 순서대로
python scripts\artec\calibrate.py --from 2         # intrinsic 건너뛰고
python scripts\artec\calibrate.py --only 3         # 턴테이블만
```

`calibrate.py` 는 **자체 로직이 없다.** 순서대로 부르고, 실패하면 거기서 멈춘다.
개별로 불러도 결과는 같다.

| 인자 | 기본 | 뜻 |
|---|---|---|
| `--from N` | 1 | N 단계부터 |
| `--only N` | — | N 단계만 |
| `--via-home` | **꺼짐** | 1·2단계에서 자세마다 home 을 경유한다 (느리다) |
| `--skip-aim` | 꺼짐 | 시작 시 수동 조준 안내를 건너뛴다 |

> **home 경유는 기본이 꺼져 있다** (2026-09-21 뒤집음). 경유하면 1·2단계 이동량이
> 1208° → 4230° 로 늘어 **2.0분 → 7.1분** 이 된다(20자세, 10 deg/s 기준).
>
> 안전 근거는 **자세 목록이 경로 검사를 거쳤다는 것**이다. `gen_calib_poses.py` 가
> 자세↔자세 경로를 충돌 게이트로 확인하고 이동거리 최소 순서로 재배열하며,
> 그 사실을 yaml 에 `path_checked: true` 로 남긴다. `calibrate.py` 는 그 표식을
> 보고 자동으로 정하므로 **보통 인자를 줄 필요가 없다**:
>
> ```
> [poses] 경로 검사 완료 (2026-09-21, 자세간 28.4°) — home 경유 생략
> [poses] 경로 검사 표식 없음 — home 을 경유한다 (느림)          ← 옛 목록이면
> ```
>
> 검사 당시 레이아웃이 지금과 다르면(`ACTIVE_LAYOUT.txt`) 검사 결과가 무효이므로
> 역시 경유한다. 즉 **빠른 쪽이 기본이되, 근거가 없으면 안전한 쪽으로 떨어진다.**

### 한 단계만

```powershell
python scripts\artec\calibrate.py --only 1   # ① K (최초 1회)
python scripts\artec\calibrate.py --only 2   # ② T_EC
python scripts\artec\calibrate.py --only 3   # ③ T_B_F0 (rim 클릭)
```

인자를 넘기려면 `--only` 와 함께 `--` 뒤에 둔다:

```powershell
python scripts\artec\calibrate.py --only 2 -- --poses config\calibration\artec_calibration_poses.yaml
```

> 단계 스크립트(`intrinsic_calib.py` 등)를 직접 불러도 동작은 같다. 다만 순서
> (K → T_EC → T_B_F0)를 건너뛰면 낡은 값이 조용히 들어가므로, 직접 실행하면
> 안내가 한 번 뜬다.

### 자주 쓰는 인자 — 단계별

> ⚠ **`--via-home` 과 `--no-home` 은 다른 인자다.** 이름이 비슷해서 헷갈린다.
>
> | 인자 | 있는 곳 | 뜻 |
> |---|---|---|
> | `--via-home` | 1·2단계 (`calibrate.py`) | 자세를 순회할 때 **매번 home 을 경유한다** (기본 꺼짐) |
> | `--no-home` | 3단계 (`turntable_calib.py`) | 시작할 때 **아무 데도 안 간다** |
>
> 3단계는 자세 순회가 없어서 `--via-home` 이 없고,
> 1·2단계는 시작 위치 개념이 없어서 `--no-home` 이 없다.

**① intrinsic / ② hand-eye** (둘 다 자세 목록을 순회한다)

| 인자 | 기본 | 뜻 |
|---|---|---|
| `--via-home` | 꺼짐 | home 왕복 **추가** (기본은 생략 — 위 설명 참고) |
| `--poses PATH` | `artec_calibration_poses.yaml` | 자세 목록 |
| `--board NAME` | `spider_dense` | 보드 프리셋 |
| `--square-mm V` | 프리셋값 | 인쇄 실측으로 보정 |
| `--start-from N` | 0 | N 번째 자세부터 (중간에 끊겼을 때) |

**③ 턴테이블 rim** — 시작 자세와 조절 수단이 핵심이다.

| 인자 | 기본 | 뜻 |
|---|---|---|
| — | **기록된 rim 자세로 이동** | `artec_rim_pose.yaml` 이 있으면 그 자세로 간다 |
| `--force-home` | — | 기록된 자세 대신 **home** 으로 |
| `--no-home` | — | **아무 데도 안 간다** — 지금 자세 그대로 캡처 (`--via-home` 과 다름) |
| `--save-pose` | — | 지금 자세를 rim 시작 자세로 기록하고 진행 |
| `--sensitivity V` | 0.9 | 재구성 민감도. 흰 상판은 기본 0.5 로는 정점이 거의 안 나온다 |
| `--range-mm N F` | 170 330 | 스캔 깊이 범위. 좁히면 배경 노이즈가 준다 |
| `--jog-step MM` | 20 | 창 안 조그 한 번의 이동량 |
| `--jog-rot-step DEG` | 10 | 창 안 조그 한 번의 회전량 |

```powershell
python scripts\artec\calibrate.py --only 3 -- --no-home
python scripts\artec\calibrate.py --only 3 -- --sensitivity 1.0 --range-mm 250 350
python scripts\artec\calibrate.py --only 3 -- --save-pose
```

**rim 창 안에서 쓰는 키**

| 키 | 동작 |
|---|---|
| **좌클릭 / 우클릭** | rim 점 추가 / 취소 |
| **Enter · Space** | 원 피팅 → 저장 여부 확인 → **결과 오버레이 창** |
| **m** | 웹 UI 수동 모드 + 라이브 화면 (손으로 팔을 끌며 조준) |
| **v** | 3D 메시 ↔ 텍스처 사진 전환 (기본 3D) |
| **S** | 지금 자세를 rim 시작 자세로 저장 |
| `w s a d e c` | base 프레임 ±x ±y ±z 이동 · `[ ]` 스텝 절반/두배 |
| `o p` | **광축 roll** — 화면만 회전, 보는 지점 유지 (호를 눕힐 때) |
| `z x` / `t g` | J7 / J6 회전 (카메라가 선회한다) · `, .` 스텝 |
| **r** / **h** / **q** | 재캡처 / home 복귀 / 종료 |

> rim 은 원판 **전체가 화각에 안 들어온다** — 지름 ~238mm 인데 Spider 화각은
> 320mm 에서 123×167mm 다. 목표는 전체가 아니라 **화면을 가로지르는 긴 호**이고,
> 그 호를 따라 **6~10점을 넓게** 찍는다. 3점이면 잔차가 **항상 0** 이라 검증이
> 성립하지 않는다.

### ④ 결과를 눈으로 검증 — `turntable_overlay.py`

숫자(점 수·잔차·반경)가 전부 통과해도 축이 엉뚱한 곳에 있을 수 있다. 저장된
`T_B_F0` 를 실제 화면에 투영해 **실제 테두리와 겹치는지** 본다.

```powershell
python scripts\artec\turntable_overlay.py                 # 지금 자세 (안 움직임)
python scripts\artec\turntable_overlay.py --pose rim      # 기록된 rim 자세로 ⚠ 움직임
python scripts\artec\turntable_overlay.py --save out.png
```

자홍=실제 테두리, 빨강=피팅 원, 초록=실측 반경(119mm) 원, 청록=축.
**빨강·초록이 둘 다 자홍에서 벗어나면 반경이 아니라 축의 위치·기울기가 틀린 것이다.**
`--only 3` 을 저장까지 마치면 이 창이 자동으로 뜬다.

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
# ★ 권장 — 지금 보이는 보드를 기준으로 (T_EC 계통 오차가 1차 상쇄된다)
python scripts\artec\gen_calib_poses.py --from-view            # 미리보기
python scripts\artec\gen_calib_poses.py --from-view --write    # 저장

# 셀 모델 기준 (로봇·스캐너 없이 오프라인)
python scripts\artec\gen_calib_poses.py                        # ① 원판 후보 확인
python scripts\artec\gen_calib_poses.py --hint-xy 0.838 -0.022 # ② 미리보기
python scripts\artec\gen_calib_poses.py --hint-xy 0.838 -0.022 --write   # ③ 저장
```

| 인자 | 기본 | 뜻 |
|---|---|---|
| `--from-view` | 꺼짐 | 지금 보이는 ChArUco 를 검출해 기준점으로 |
| `--center X Y Z` | — | 기준점을 직접 (base, m) |
| `--hint-xy X Y` | — | 셀 모델에서 원판을 고를 힌트 |
| `--standoff M` | 0.32 | 조준 여유로 정한 값 (스캔 거리가 아니다) |
| `--rolls ...` | −45…110 (8개) | **좁히지 말 것** — 90°로 좁혔다가 유효 자세가 81→9 로 무너진 적이 있다 |
| `--max-poses N` | 20 | 상한 |
| `--fov-margin-px V` | 40 | 보드가 프레임에 들어오는지 검사할 때의 가장자리 여유 |
| `--no-fov-gate` | 꺼짐 | 그 검사를 끈다 |
| `--write` | 꺼짐 | 저장 (없으면 미리보기만) |

저장 시 자세를 **이동거리 최소 순서로 재배열**하고 자세↔자세 경로를 충돌 검사한다
— 그래서 home 경유 생략이 안전하고, 그게 기본이다.

**충돌 셀 모델**(`collision.md` §6)에서 턴테이블 원판을 찾아 그 위 반구에 자세를
깔고, 해석 IK + 충돌 게이트로 거른다. sim 이 쓰는 생성기와 **같은 코드**다.

- 로봇도 스캐너도 필요 없다 — **오프라인**에서 만든다
- `T_EC` 에 의존하지 않는다(겨누는 용도로만 쓴다) → 지금 구하려는 값과 순환이 없다
- 반구: polar 0/15/30/45 × 8방위, roll·거리 조합 → 225후보 → 게이트 통과 후 **20개**로 솎음
- standoff 기본 **320mm** — 스캔 거리가 아니라 **조준 오차에 대한 여유**로 정한 값이다

생성된 자세는 **관절각(`joints`)으로 저장**된다. `ee_pose` 만 저장하면
`hand_eye_calib` 이 그걸 컨트롤러 IK 로 다시 풀어서 가는데, 7축이라 같은 TCP 에
해가 무한히 많아 **검증한 자세와 실제로 가는 자세가 달라진다**(실측 Δq 140~251°).
2026-09-15 에 이것 때문에 첫 자세 이동 중 턴테이블과 충돌 직전까지 갔다
— `collision.md` T7.

걸러지는 조건 넷: IK 실패 · **관절 한계 여유 5° 미만** · 자세 충돌 ·
**home→자세 경로 충돌**. 마지막 둘이 없으면 위 사고가 재현된다.

①에서 후보 목록이 나온다. **자동으로 고르지 않는다** — 셀 점군은 부재 이름이 전부
`mesh` 라(§6.1) 무엇이 턴테이블인지 데이터만으로 구별할 수 없다. 실제로 자동 선택이
**키보드**를 집은 적이 있다.

```
  원판 후보 (base 프레임):
    --hint-xy -0.257 0.006   상면 z=0.778  r95=143mm  점 125,013
    --hint-xy  0.838 -0.022  상면 z=0.694  r95= 75mm  점 10,444   ← 턴테이블
```

어느 것인지 모르겠으면 씬을 눈으로 본다: `$ISAAC scripts\sim\view_scene.py v2`

#### ⚠ `charuco 부족` 이 대부분이면 — 조준 기준점이 틀렸다

셀 모델의 **원판 중심**을 겨누는데 보드가 거기 없으면, Spider 의 좁은 FOV
(0.25m 에서 134×100mm)에서 보드가 잘려 나간다. 보드를 원판 한복판에 정확히
놓지 않았거나, 구 `T_EC` 의 회전 오차(5°면 250mm 에서 22mm)가 얹히면 그렇다.

2026-09-15 실측 — 20자세 중 **18자세가 `charuco 부족`**:

```
[hemi_00] charuco 부족 (aruco=2, charuco=0) — skip
[hemi_12] aruco=7  charuco corners=8            ← 중앙에 온 것만 성공
```

**거리는 정상이었다.** `debug_intrinsic_artec/*.png` 의 마커 픽셀 크기로 역산하니
평균 **262mm**(의도 250mm) — 즉 거리가 아니라 **조준**의 문제다.

→ **실제 보드를 기준으로 다시 만든다:**

```powershell
# 1) aim.py 로 'OK' 가 뜨도록 조준 (§2). 끝나면 뷰어 q, Manual Mode off, 탭 닫기
# 2) 그 자세에서 검출해 보드 중심을 직접 잡는다
python scripts\artec\gen_calib_poses.py --from-view --write
```

여기서 추정한 중심을 **같은 `T_EC` 로** 다시 겨누므로 `T_EC` 의 계통 오차가
1차적으로 상쇄된다 — `T_EC` 가 낡아도 이 경로가 동작하는 이유다.

보드 위치를 자로 재서 알고 있으면 `--center X Y Z` 로 직접 줘도 된다.

#### 그래도 잘리면 — 거리를 더 올린다

조준을 맞췄는데도 디버그 이미지가 조각으로 잘리면 **FOV 여유가 모자란 것**이다.

| standoff | 화각(가로×세로) | 보드가 벗어나도 되는 여유 | 유효 자세 |
|---|---|---|---|
| 250mm | 134×100mm | 17 × 20mm | 93 |
| **320mm** (기본) | 171×128mm | **36 × 34mm** | 85 |
| 350mm | 188×140mm | 44 × 40mm | 73 |

```powershell
python scripts\artec\gen_calib_poses.py --from-view --standoff 0.35 --write
```

거리를 올려도 **도달성 손해는 거의 없다**(유효 93→73, 필요한 건 20개).
작동거리 상한이 350mm 이므로 그 이상은 쓰지 않는다.

> 기존 yaml 은 `.yaml.bak` 으로 백업된다.

### ② hand-eye 를 돌릴 때

로봇이 여러 자세를 순회하며 보드를 찍는다.

- 자세 **15~25개**
- 자세 사이 회전 **30° 이상** — 회전이 작으면 `AX=ZB` 가 안 풀린다
- 보드가 화면 밖으로 나가면 그 자세는 버려진다

기록된 자세로 다시 돌리려면:

```powershell
python scripts\artec\calibrate.py --only 2 -- --poses config\calibration\artec_calibration_poses.yaml
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
python scripts\artec\check_calibration.py       # 숫자 검사
python scripts\artec\turntable_overlay.py       # 눈으로 검사 (T_B_F0 를 화면에 투영)
```

**둘 다 봐야 한다.** `check_calibration.py` 는 점 수·잔차·반경만 본다 — 점들끼리
일관된 원인지는 알려주지만, 그 원이 **실제 원판 위에 놓였는지**는 모른다.
2026-09-16 에 점 15개·잔차 0.18mm·반경 116mm 로 지표가 전부 OK 인데
투영해 보니 실제 테두리와 **43mm** 어긋난 적이 있다(원인은 `T_EC` 회전 오차 5~7°).

이 도구는 **두 독립 출처를 교차검증**한다 — 캘리브 결과(`T_B_F0`)가 가리키는 자리에
충돌 캐시(실측 셀 모델)의 구조물이 실제로 있는지 본다. 엉뚱한 곳을 가리키면 여기서 걸린다.

```
교차검증 — 활성 레이아웃 'v2_layout_real' 에서 이 좌표 15cm 안의 셀 점: 0개
[FAIL] 그 자리에 아무것도 없다
```

↑ **2026-09-16 현재 저장된 값이 내는 결과다.** 턴테이블 축이 셀 모델과 다른 곳을
가리킨다(`T_B_F0` 원점 `[-0.626, -0.177, -0.810]`, base 기준 1.04m).

> 셀 모델 쪽이 의심되면 `docs/collision.md` §6 을 본다. 두 값이 독립이라
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
| `charuco 부족 (aruco=0…) — skip` 이 많다 | 자세 목록이 지금 배치와 안 맞는다 → `gen_calib_poses.py --from-view --write` 로 다시 만든다 (§3). *(`--interactive`(teach mode)는 2026-09-16 제거했다)* |
| `⚠ 최소 8 frame 필요. 종료.` | 위와 같은 원인. **이 경우 exit 1 로 멈춘다** — 예전엔 조용히 통과해 다음 단계가 낡은 값을 썼다 |
| hand-eye 가 intrinsic 날짜를 묻는다 | 90일 넘은 intrinsic 이다. 스캐너를 바꿨으면 **`N` 을 눌러 중단**하고 intrinsic 부터 |
| `charuco 부족` 이 대부분 | **조준 기준점이 실제 보드와 다르다** → `--from-view` 로 다시 생성 (§3). 거리부터 의심하지 말 것 — `debug_intrinsic_artec/*.png` 로 확인된다 |
| 조준이 맞는지 미리 보고 싶다 | `python scripts\artec\aim.py` — 웹 UI + 실시간 뷰어. 그 자리에서 OK/잘림 판정 |
| 뷰어가 빈 화면만 돈다 | `텍스처 프레임을 N회 연속 못 받았다` 경고를 볼 것 — Artec Studio 가 떠 있거나 스캐너 앞이 비었다 |
| 캡처 이미지를 보고 싶다 | `debug_intrinsic_artec/<pose>.png` (intrinsic) · `debug_calib_artec/<pose>_raw.png`, `_aruco.png` (hand-eye). 자세마다 저장된다 |
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
| 프레임·로봇 마운트 이동 | T_B_F0 + **충돌 캐시** (`collision.md` §6) |
| 렌즈·센서 교체 | intrinsic 부터 전부 |
| 로봇 교체 | 위 전부 + **`xarm7_dh.yaml` 재교정** (개체 종속) |

---

## 관련 문서

- 원리·근거·함정 — `1_calibration.md`
- 좌표 규약 · 단위 — `README.md` §규약
- 로봇 수동 조작 — `robot_control.md`
- 셀 모델이 실물과 다를 때 — `collision.md` §6
- 설치 — `install.md`
