# real 커맨드 모음 — 실물 장비로 동작 테스트

> sim 쪽은 **`sim_commands.md`**. 이 문서는 **실물 하드웨어**로 돌릴 때다.
>
> 모든 명령은 리포 루트에서, **`mms-env`** 로 실행한다.
> ```bash
> cd <MMS 리포>
> conda activate mms-env
> ```

## ⚠ 시작 전 3가지

**1. env 를 맞춘다** — 실물 스크립트는 `mms-env` 에서만 돈다.
`xarm` SDK 가 `env_isaacsim` 에는 없다(sim 은 xArm SDK 가 필요 없다).
```
ModuleNotFoundError: No module named 'xarm'   ← env 를 잘못 고른 것
```

**2. `env -u PYTHONPATH` 를 붙인다** — 셸에 ROS python3.10 경로가 잡혀 있으면 섞인다.

**3. 장비가 네트워크에 붙어 있는지 먼저 확인한다.**
```bash
ping -c2 192.168.1.210      # xArm7
ping -c2 192.168.0.10       # 턴테이블 (Ezi-SERVO)
```
> `Exception: connect socket failed` 는 대부분 **로봇에 도달할 수 없다**는 뜻이다.
> `ip -4 addr` 로 `192.168.1.x` / `192.168.0.x` 대역 인터페이스가 있는지 본다.
> 로봇망은 보통 DHCP 가 없어 고정 IP 를 직접 줘야 한다:
> ```bash
> sudo ip addr add 192.168.1.100/24 dev <iface>
> sudo ip link set <iface> up
> ```
> 턴테이블 Ezi-SERVO 드라이버는 **Windows 전용 DLL** 이다 — 리눅스에서는
> 로봇+스캐너만 쓰는 작업(캘리브 등)까지만 가능하다.

---

## 0. 연결 확인 (제일 먼저)

```bash
# 로봇만 홈 자세로 — 통신·구동 최소 확인
env -u PYTHONPATH python scripts/artec/go_home.py

# import smoke test (하드웨어 없이 코드만 확인)
env -u PYTHONPATH python -c "import main_artec"
```

---

## 1. 캘리브레이션 (설치 후 1회, 이설 시 재수행)

**단일 진입점이 순서를 강제한다.**

```bash
env -u PYTHONPATH python scripts/artec/calibrate.py          # 전체
env -u PYTHONPATH python scripts/artec/calibrate.py --from 2 # 2번 단계부터
env -u PYTHONPATH python scripts/artec/calibrate.py --only 3 # 한 단계만
```

순서는 **intrinsic → hand-eye → 턴테이블 축**이다. `turntable_calib` 이 rim 점을 base 로
변환한 뒤 원을 피팅하는데 그 변환에 `T_EC` 가 들어가기 때문이다. 원리와 순서를 뒤집는
방법은 `1_calibration.md` §9·T10 을 본다.

시작 전에 **사람이 수동으로 조준**한다 — 보드와 턴테이블이 카메라에 들어와야 한다.
보드를 원판 위에 올려두면 hand-eye 와 턴테이블을 같은 조준 자세에서 이어서 할 수 있다.

아래는 단계별로 따로 돌릴 때다.

### 1.1 ChArUco 보드 준비 (최초 1회)

```bash
env -u PYTHONPATH python scripts/artec/make_charuco.py
```
출력 PNG 를 **실척으로 인쇄**해 평평한 판에 붙인다. 기본 5×3 / sq 20mm / mk 15mm.
> Spider 는 FOV 가 좁아 A4 7×5 보드는 화면 밖으로 나간다. 작은 보드를 쓴다.

### 1.2 카메라 내부 파라미터

```bash
env -u PYTHONPATH python scripts/artec/intrinsic_calib.py
# → config/calibration/artec_intrinsic.yaml
```

### 1.3 Hand-eye `T_E_C`

```bash
# 첫 실행 — 수동 모드: 로봇을 직접 움직이며 Enter 로 캡처 (성공 자세는 yaml 에 기록)
env -u PYTHONPATH python scripts/artec/hand_eye_calib.py

# 이후 — 기록된 자세로 자동 순회
env -u PYTHONPATH python scripts/artec/hand_eye_calib.py \
    --poses config/calibration/artec_calibration_poses.yaml
# → config/calibration/hand_eye_artec.yaml
```
- **자세 15~25개**, 자세 간 **회전 ≥30°** 확보할 것 (적으면 안 풀린다)
- 기준값: t_err **3.55mm** / r_err **1.30°** (2026-04-29)
- 결과를 `config/sensor_frames.yaml::T_EC_artec` 에 반영한다

### 1.4 턴테이블 축 `T_B_F0`

```bash
env -u PYTHONPATH python scripts/artec/turntable_calib.py
# → 원판 rim 위 점을 클릭 → config/calibration/turntable_frame.yaml
```
- rim 이 한 화면에 다 안 들어오면 보이는 호(arc)에서 클릭. 3점이면 되지만 호가 짧으면 정밀도 저하
- 기준값: **0.015° / 0.7mm**

> ⚠ 현재 저장된 값은 **2026-04-23**, Artec 장착 이전이다. Phase 2 조준·recovery·충돌
> 회피가 전부 여기 의존하므로 **실물 재개 시 이것부터 다시 잡을 것.**

---

## 2. 스캔 실행

### 2.1 메인 파이프라인

`main_artec.py` 상단 `BACKEND = "real"` 확인 후:

```bash
env -u PYTHONPATH python main_artec.py
```

### 2.2 라이브 뷰어 (권장 — 별도 터미널)

스캔 중 누적 점군을 실시간으로 본다. 노이즈·드리프트·멈춤을 눈으로 잡는 용도.

```bash
# 터미널 B (순서 무관, A 종료 시 자동 종료)
env -u PYTHONPATH python scripts/artec/live_scan_view.py
```
- 정합이 끊기면 배경이 **빨강**으로 바뀐다
- ★ **반드시 별도 터미널**. 파이프라인이 자식 프로세스로 띄우면 Filament 창이 즉사한다

### 2.3 턴테이블 단독 조작

```bash
env -u PYTHONPATH python scripts/turntable/phase1_speed_rotation.py   # 회전 테스트
env -u PYTHONPATH python scripts/turntable/turntable_stop.py          # 비상 정지
```

---

## 3. 후처리 반복 실험 (스캔 없이)

스캔을 다시 돌리지 않고 **병합·후처리만** 바꿔가며 비교할 수 있다.

```bash
# 1) 스캔 1회만 돌려 raw 저장
env -u PYTHONPATH python scripts/artec/save_raw_scan.py
# → output/scan_raw/<TS>/  (master.sproj + meta.npz)

# 2) 저장분으로 4가지 병합 variant 비교
env -u PYTHONPATH python scripts/artec/merge_compare.py --load output/scan_raw/<TS>
```
> 실물 스캔은 한 번에 수 분이 걸린다. 알고리즘을 손볼 때는 반드시 이 경로를 쓸 것.

---

## 4. 시연·보고용

```bash
env -u PYTHONPATH python scripts/artec/main_artec_demo.py
```
평소 알고리즘은 tracking lost 가 없으면 로봇이 거의 안 움직이는 게 정상이라
영상으로는 심심하다. 이 스크립트는 **보고용으로 동선을 일부러 넣는다.**

---

## 5. 자주 겪는 것

| 증상 | 원인 · 대응 |
|---|---|
| `No module named 'xarm'` | env 오선택 → `conda activate mms-env` |
| `No module named 'cv2'` | **base 파이썬**으로 실행한 것 (`miniconda3/bin/python`). `conda activate mms-env` 후 `python` |
| `No module named 'omni.usd'` | `sim_harness/` 하니스를 standalone 으로 실행한 것 → Isaac GUI Script Editor 에서 실행 (`1_calibration.md` T6) |
| `connect socket failed` | 로봇 미도달 → §0 네트워크 확인 |
| `scan settings import 불가` | Artec SDK python 바인딩 미빌드 → `artec_SDK/artec0_build_guide.md`. sim 은 정상 동작 |
| 스캔이 자꾸 끊긴다 | 시작 자세가 나쁘다. 물체를 조준한 상태로 시작하고, 그래도 반복되면 recovery 로그의 elevation 을 확인 (`2_phase1.md` §7) |
| 물체를 치웠는데 계속 스캔됨 | 정합이 `ICP` 로 되어 있을 수 있다 → `HYBRID` 확인 (`2_phase1.md`) |
| 결과 메시가 어긋남·떠 있음 | 거의 항상 `T_pre` 문제 → `5_phase3_merge.md` |
| 콘솔 이모지 깨짐 | `PYTHONIOENCODING=utf-8` |

> SDK 바인딩 변경 시 재빌드:
> `cmake --build mms_artec/sensor/build --config Release --target <module>`

---

## 6. 조정 가능한 설정은 어디 있나

값을 바꾸려면 코드를 뒤지지 말고 해당 문서의 설정 표를 본다.

| 무엇 | 어디 |
|---|---|
| 캘리브 보드 프리셋·자세 수 | `1_calibration.md` §2·§4 |
| Phase 1 — 자세 후보, 캡처 밀도, 회전 시간, watchdog 임계 | `2_phase1.md` §1 |
| Phase 2 — NBV 반복, 거리, 충돌 world 치수, 수렴 임계 | `3_phase2.md` §1 |
| 충돌 — 안전 여유, 특이점 임계, SDF 격자 | `4_collision.md` §1 |
| Phase 3 — flip 축·각, pass 상한 | `5_phase3_merge.md` §1 |
| 후처리 — 단계별 on/off, fusion 방식 | `6_postprocess.md` §1 |

하드웨어 IP·홈 자세 같은 상수는 `main_artec.py` 상단에 모여 있다.

---

## 관련 문서

| 무엇 | 문서 |
|---|---|
| sim 실행·씬 생성 | `sim_commands.md` |
| 캘리브 원리·함정 | `1_calibration.md` |
| Phase 1·2·3 동작 | `2_phase1.md` · `3_phase2.md` · `5_phase3_merge.md` |
| 셀 배치·치수 | `hw_layout.md` |
