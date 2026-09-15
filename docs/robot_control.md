# 로봇 제어 — xArm7

> 실물 xArm7 을 손으로 다루는 방법. 웹 UI 와 `scripts/robot/*` 두 가지.
> Phase 1~3 자동 스캔은 `main_artec.py` 가 하고, 이 문서는 **그 전후에 사람이 하는 조작**을 다룬다.

| 하고 싶은 것 | 방법 |
|---|---|
| 지금 상태 보기 | `python scripts/robot/status.py` |
| 에러 지우기 | `python scripts/robot/recover.py` |
| 기준 자세로 | `python scripts/robot/home.py` |
| 조금씩 움직이기 | `python scripts/robot/jog.py --dz 20` |
| 특정 좌표로 | `python scripts/robot/move_pose.py --xyz 400 0 350` |
| 손으로 끌기 · 에러 원인 보기 | 웹 UI (아래) |

**IP** `192.168.1.210` · 펌웨어 `v2.5.1` · 모든 스크립트는 `--ip` 로 덮어쓸 수 있다.

---

## 1. 웹 UI — xArm Studio

```
http://192.168.1.210:18333
```

UFACTORY 가 컨트롤러에 내장한 웹 앱이다. 브라우저만 있으면 되고 설치가 필요 없다.

**이럴 때 쓴다**

- `error_code` 가 0 이 아닐 때 — 무슨 에러인지 이름으로 보여준다
- 조그(jog)로 로봇을 눈으로 보며 안전한 자세로 옮기기
- 관절 한계·충돌 감지 민감도 설정 확인
- 펌웨어 버전, TCP 오프셋, 페이로드 확인

**⚠ Python SDK 와 동시에 쓰지 말 것.** 웹 UI 가 열려 있으면 컨트롤러의 mode/state 를
웹 쪽이 바꿔버려서, SDK 로 보낸 명령이 무시되거나 예상 밖으로 동작한다.
`main_artec.py` 나 캘리브레이션을 돌릴 때는 **브라우저 탭을 닫는다.**

> 포트 18334 도 열려 있지만 내부 서비스용이다. 브라우저로 접속할 곳은 18333.

---

## 2. 스크립트

전부 `mms-env` 에서 돌린다.

```powershell
conda activate mms-env
cd C:\dev\MMS
$env:PYTHONIOENCODING="utf-8"
```

### 공통 안전 장치

`scripts/robot/_common.py` 가 모든 모션 스크립트에 아래를 강제한다.

| 단계 | 내용 |
|---|---|
| 연결 | `error_code != 0` 이면 **거부** — 먼저 `recover.py` |
| IK | **컨트롤러 IK**(`get_inverse_kinematics`). 해석 IK 안 씀 (§4) |
| 검산 | 컨트롤러 FK 로 되짚어 목표와 1mm 이내인지 확인 |
| 사전검사 | 관절 한계 · self-collision · 특이점 근접 |
| 확인 | Enter 가 아니라 **`y` 입력** — Enter 는 실수로 눌린다 |

`--dry-run` 을 붙이면 **검증까지만 하고 움직이지 않는다.** 처음 쓰는 좌표는 항상 이걸로 먼저 본다.

### status.py — 읽기 전용

```powershell
python scripts/robot/status.py
```

관절각·TCP·에러·관절 여유·특이점 지표·self-collision 을 한 번에 보여준다.
**절대 움직이지 않으므로** 아무 때나 안전하게 돌릴 수 있다.

`state=4` 는 정상이다 — "motion_enable 전 대기". 고장이 아니다. 봐야 할 건 `error`.

### recover.py — 에러 클리어

```powershell
python scripts/robot/recover.py            # 에러/경고만 클리어
python scripts/robot/recover.py --enable   # motion_enable 까지 (움직일 수 있다)
```

`--enable` 은 `set_state(0)` 을 부르는데, 이게 컨트롤러를 **이전 명령 위치로 resume**
시켜 로봇이 곧바로 움직일 수 있다. 그래서 확인을 받는다.

클리어해도 에러가 남으면 물리적 원인(충돌·한계 초과)이다 → 웹 UI 에서 확인.

### home.py — 기준 자세

```powershell
python scripts/robot/home.py                  # artec (기본)
python scripts/robot/home.py --sensor phoxi
```

wrist singularity 를 피한 중립 자세로, IK seed 로도 쓴다.
**센서마다 J7 이 다르다** — Artec 은 J7 기준 −45° 회전 마운트라 home 값이 따로 있다
(`XArmInterface.HOME_JOINTS_DEG`). 센서를 바꿔 달았으면 `--sensor` 를 맞춰야 한다.

관절 목표를 직접 주는 방식이라 IK 를 타지 않는다.

### jog.py — 상대 이동

```powershell
python scripts/robot/jog.py --dz 20 --dry-run   # 먼저 확인
python scripts/robot/jog.py --dz 20             # 실행
python scripts/robot/jog.py --dx -10 --dy 5
python scripts/robot/jog.py --dyaw 15           # deg
```

현재 TCP 기준 상대 이동(B 프레임). **회전은 deg 로 받는다**
(`XArmInterface.move_relative` 는 rad 를 받으므로 헷갈리지 말 것).
`--speed` 는 **mm/s**, 기본 20.

### move_pose.py — 절대 좌표

```powershell
python scripts/robot/move_pose.py --xyz 400 0 350 --dry-run
python scripts/robot/move_pose.py --xyz 400 0 350 --rpy 180 0 0
```

`--rpy` 생략 시 현재 자세를 유지한 채 위치만 바꾼다. 직선거리 100mm 초과면 경고한다.

---

## 3. 좌표·단위 — 버그 1순위

README §규약 과 같다. 여기서 자주 틀린다:

| 출처 | 단위 |
|---|---|
| `get_ee_pose_mat()`, yaml `T_EC` | **m** |
| `arm.set_position(x,y,z,…)`, `get_pose()` | **mm** |
| Artec SDK vertices / `frame_transformation` | **mm** |
| 스크립트 `--dx/--dy/--dz`, `--xyz` | **mm** |
| 스크립트 `--droll/--dpitch/--dyaw`, `--rpy` | **deg** |
| `XArmInterface.move_relative(d_roll=…)` | **rad** |

---

## 4. ⚠ 함정 — 해석 IK 를 실물 모션에 쓰면 안 된다

`utils/robot/xarm7_kinematics.py` 의 FK/IK 는 **sim USD**(`xarm7_spider/v2.usd`)에서
추출한 모델이다. 파일 첫 주석도 "이 모델은 **자기일관적이면 충분하다**" 고 적고 있다 —
sim 전용 전제다.

**이 실물과 어긋난다.** 2026-09-15 측정:

```
관절각 동일, 컨트롤러 FK vs 해석 FK
  컨트롤러 (= get_position, TCP offset 0) : (560.76, -37.00, 381.23) mm
  해석 FK                                  : (544.66, -36.72,  90.12) mm
  차이                                     : ( -16.1,  +0.28, -291.11) mm
```

그래서 **`XArmInterface.move_relative()` 는 실물에서 쓰면 안 된다.** 그 함수는
`get_pose()`(컨트롤러 프레임)에 `self.ik()`(해석 IK, USD 프레임)를 먹인다. 두 모델이
다르므로 결과가 엉뚱하다. 실측(로봇은 움직이지 않고 컨트롤러 FK 로 확인):

```
의도    : dz = +20 mm
실제    : (-139.6, -3.2, +276.7) mm       ← 300mm 짜리 예상 밖 동작
```

→ `scripts/robot/*` 는 전부 **컨트롤러 IK**(`arm.get_inverse_kinematics`)를 쓴다.
같은 목표에 대해 오차 0.001mm 로 일치한다.

**영향 범위**

| 대상 | 영향 |
|---|---|
| hand-eye 캘리브레이션 | **없음** — `get_ee_pose_mat()` 는 컨트롤러 `get_pose()` 를 쓴다 |
| sim(isaac) 백엔드 | **없음** — sim 은 같은 USD 모델로 일관 |
| `move_relative()` 실물 호출 | **깨짐** |
| `collision_capsules()` · `precheck` 의 self-collision/특이점 | 같은 모델 기반이라 **실물 기준으로는 참고용** |

> 미해결. 해석 모델을 실물에 맞추려면 DH/USD 재추출이 필요하다.
> `v2.usd` 는 README 에서 이미 deprecated 로 표시된 씬이다 — 재추출 시 v3 기준으로 볼 것.

---

## 5. 움직이기 전 체크리스트

1. `python scripts/robot/check_devices.py` — 아니면 `status.py` 로 `error=0` 확인
2. 작업 반경에 사람·케이블·공구 없는지 **눈으로**
3. **비상정지 버튼이 손에 닿는 곳에**
4. 처음 쓰는 좌표는 `--dry-run` 먼저
5. `--speed` 를 낮게 (기본값이 이미 느리다)
6. 웹 UI 탭은 닫기

**멈추는 법** — 비상정지 버튼이 1순위. 스크립트는 `Ctrl+C` 로 끊을 수 있지만
`wait=True` 로 대기 중이면 컨트롤러는 이미 받은 명령을 끝까지 수행한다.
턴테이블이 도는 중이라면 `python scripts/turntable/turntable_stop.py`.

---

## 관련 문서

- 장비 3종 연결 점검 — `scripts/check_devices.py`
- 좌표계 규약 · 장비 IP — `README.md`
- hand-eye / 턴테이블 축 캘리브 — `docs/1_calibration.md`
- 충돌 검사 — `docs/4_collision.md`
- 실물 실행 명령 — `docs/7_real_commands.md`
