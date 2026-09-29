# 로봇 수동 조작 — xArm7

> 자동 스캔은 `main_artec.py` 가 수행하며, 본 문서는 **그 전후에 사람이 직접 하는 조작**
> 을 다룬다. 웹 UI 와 `scripts/robot/*` 두 가지 수단이 있다.

| 목적 | 방법 |
|---|---|
| 현재 상태 확인 | `python scripts/robot/status.py` |
| 에러 클리어 | `python scripts/robot/recover.py` |
| 기준 자세로 복귀 | `python scripts/robot/home.py` |
| 미세 이동 | `python scripts/robot/jog.py --dz 20` |
| 지정 좌표 이동 | `python scripts/robot/move_pose.py --xyz 400 0 350` |
| 수동 견인 · 에러 원인 확인 | 웹 UI (§1) |

**IP** `192.168.1.210` · 펌웨어 `v2.5.1`. 모든 스크립트는 `--ip` 로 덮어쓸 수 있다.

---

## 1. 웹 UI — xArm Studio

```
http://192.168.1.210:18333
```

UFACTORY 가 컨트롤러에 내장한 웹 앱으로 브라우저만 있으면 되며 별도 설치가 불필요하다.
`error_code` 가 0 이 아닐 때 에러명 확인, 조그로 안전 자세 이동, 관절 한계·충돌 감지
민감도 확인, 펌웨어·TCP 오프셋·페이로드 확인에 사용한다.

> ⚠ **Python SDK 와 동시에 사용하지 않는다.** 웹 UI 가 열려 있으면 컨트롤러의
> mode/state 를 웹 쪽이 변경하여 SDK 명령이 무시되거나 예상 외로 동작한다.
> `main_artec.py` 나 캘리브레이션 실행 시에는 브라우저 탭을 닫는다.
> (포트 18334 는 내부 서비스용이다.)

---

## 2. 스크립트

전부 `mms-env` 에서 실행한다.

```powershell
conda activate mms-env
cd C:\dev\MMS
$env:PYTHONIOENCODING="utf-8"
```

### 공통 안전 장치

`scripts/robot/_common.py` 가 모든 모션 스크립트에 아래를 강제한다.

| 단계 | 내용 |
|---|---|
| 연결 | `error_code != 0` 이면 거부 — `recover.py` 선행 |
| IK | **컨트롤러 IK**(`get_inverse_kinematics`). 해석 IK 미사용(§4) |
| 검산 | 컨트롤러 FK 로 되짚어 목표와 1mm 이내인지 확인 |
| 사전검사 | 관절 한계 · self-collision · 특이점 근접 |
| 확인 | Enter 가 아니라 **`y` 입력** — Enter 는 오조작 위험 |

`--dry-run` 을 붙이면 검증까지만 수행하고 움직이지 않는다. 처음 사용하는 좌표는 항상
이것으로 먼저 확인한다.

### 스크립트별 요지

**`status.py`** — 읽기 전용. 관절각·TCP·에러·관절 여유·특이점 지표·self-collision 을
일괄 출력하며 로봇을 움직이지 않으므로 언제든 안전하다. `state=4` 는
"motion_enable 전 대기" 로 정상이며, 확인할 것은 `error` 다.

**`recover.py`** — 에러/경고 클리어. `--enable` 은 `set_state(0)` 을 호출하여 컨트롤러를
**이전 명령 위치로 resume** 시키므로 로봇이 즉시 움직일 수 있다(그래서 확인을 받는다).
클리어해도 에러가 남으면 물리적 원인(충돌·한계 초과)이므로 웹 UI 에서 확인한다.

**`home.py`** — wrist singularity 를 회피한 중립 자세이며 IK seed 로도 사용한다.
관절 목표를 직접 지정하므로 IK 를 타지 않는다. **센서마다 J7 이 다르다** — Artec 은
J7 기준 −45° 회전 마운트이므로 home 값이 별도다(`XArmInterface.HOME_JOINTS_DEG`).
센서 교체 시 `--sensor` 를 맞춘다(`--sensor phoxi` 등).

**`jog.py`** — 현재 TCP 기준 상대 이동(B 프레임). `--dx/--dy/--dz` 는 mm,
`--droll/--dpitch/--dyaw` 는 **deg** 다(`XArmInterface.move_relative` 는 rad 를 받으므로
혼동하지 않는다). `--speed` 는 mm/s, 기본 20.

**`move_pose.py`** — 절대 좌표 이동. `--rpy` 생략 시 현재 자세를 유지한 채 위치만
변경하며, 직선거리 100mm 초과 시 경고한다.

---

## 3. 단위 — 버그 1순위

| 출처 | 단위 |
|---|---|
| `get_ee_pose_mat()`, yaml `T_EC` | **m** |
| `arm.set_position(x,y,z,…)`, `get_pose()` | **mm** |
| Artec SDK vertices / `frame_transformation` | **mm** |
| 스크립트 `--dx/--dy/--dz`, `--xyz` | **mm** |
| 스크립트 `--droll/--dpitch/--dyaw`, `--rpy` | **deg** |
| `XArmInterface.move_relative(d_roll=…)` | **rad** |

---

## 4. ⚠ 해석 IK 를 실물 모션에 사용하지 않는다

`utils/robot/xarm7_kinematics.py` 의 FK/IK 는 **sim USD**(`xarm7_spider/v2.usd`)에서
추출한 모델이며, 파일 첫 주석도 "이 모델은 자기일관적이면 충분하다" 는 sim 전용 전제를
명시하고 있다.

**이 모델은 실물과 어긋난다**(2026-09-15 측정, 동일 관절각 기준).

```
컨트롤러 FK (= get_position, TCP offset 0) : (560.76, -37.00, 381.23) mm
해석 FK                                    : (544.66, -36.72,  90.12) mm
차이                                       : ( -16.1,  +0.28, -291.11) mm
```

따라서 **`XArmInterface.move_relative()` 는 실물에서 사용하지 않는다.** 해당 함수는
`get_pose()`(컨트롤러 프레임)에 `self.ik()`(해석 IK, USD 프레임)를 적용하므로 두 모델이
달라 결과가 어긋난다. 실측에서 `dz = +20mm` 의도가 `(-139.6, -3.2, +276.7) mm` 로
나타났다(로봇은 움직이지 않고 컨트롤러 FK 로 확인).

`scripts/robot/*` 는 전부 **컨트롤러 IK**(`arm.get_inverse_kinematics`)를 사용하며, 동일
목표에 대해 오차 0.001mm 로 일치한다.

| 대상 | 영향 |
|---|---|
| hand-eye 캘리브레이션 | 없음 — `get_ee_pose_mat()` 는 컨트롤러 `get_pose()` 를 사용 |
| sim(isaac) 백엔드 | 없음 — 동일 USD 모델로 일관 |
| `move_relative()` 실물 호출 | **오동작** |
| `collision_capsules()`·`precheck` 의 self-collision/특이점 | 동일 모델 기반이므로 실물 기준으로는 참고용 |

> **미해결.** 해석 모델을 실물에 맞추려면 DH/USD 재추출이 필요하다. `v2.usd` 는 이미
> deprecated 씬이므로 재추출 시 v3 기준으로 검토한다.

---

## 5. 기동 전 확인사항

1. `python scripts/check_devices.py` 또는 `status.py` 로 `error=0` 확인
2. 작업 반경 내 사람·케이블·공구 유무를 **육안으로** 확인
3. 비상정지 버튼을 손이 닿는 위치에 둔다
4. 처음 사용하는 좌표는 `--dry-run` 선행
5. `--speed` 를 낮게 유지한다
6. 웹 UI 탭을 닫는다

**정지 방법** — 비상정지 버튼이 1순위다. 스크립트는 `Ctrl+C` 로 중단할 수 있으나
`wait=True` 로 대기 중이면 컨트롤러는 이미 수신한 명령을 끝까지 수행한다. 턴테이블
회전 중에는 `python scripts/turntable/turntable_stop.py` 를 사용한다.

---

## 관련 문서

- 장비 3종 연결 점검 — `scripts/check_devices.py`
- 좌표계 규약 · 백엔드 구조 — `troubleshooting.md`
- hand-eye / 턴테이블 축 캘리브 — `1_calibration.md` · `calibration_runbook.md`
- 충돌 검사 — `collision.md`
- 실물 실행 명령 — `7_real_commands.md`
