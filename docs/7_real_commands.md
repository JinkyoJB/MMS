# real 커맨드 모음 — 실물 장비 운용

> sim 은 `sim_commands.md`. 본 문서는 **실물 하드웨어** 운용을 다룬다.
> 모든 명령은 리포 루트에서 **`mms-env`** 로 실행한다(`conda activate mms-env`).

## 시작 전 점검 3가지

**1. env 를 맞춘다.** 실물 스크립트는 `mms-env` 에서만 동작한다. `xarm` SDK 는
`env_isaacsim` 에 없다(sim 은 xArm SDK 를 사용하지 않는다).
`ModuleNotFoundError: No module named 'xarm'` 은 env 오선택이다.

**2. `env -u PYTHONPATH` 를 붙인다.** 셸에 ROS python3.10 경로가 잡혀 있으면 혼입된다.

**3. 장비의 네트워크 연결을 먼저 확인한다.**

```bash
ping -c2 192.168.1.210      # xArm7
ping -c2 192.168.0.10       # 턴테이블 (Ezi-SERVO)
```

`Exception: connect socket failed` 는 대부분 로봇에 도달할 수 없다는 의미다.
`ip -4 addr` 로 `192.168.1.x` / `192.168.0.x` 대역 인터페이스를 확인한다. 로봇망에는
DHCP 가 없으므로 고정 IP 를 직접 부여한다.

```bash
sudo ip addr add 192.168.1.100/24 dev <iface>
sudo ip link set <iface> up
```

> 턴테이블 Ezi-SERVO 드라이버는 **Windows 전용 DLL** 이다. 리눅스에서는 로봇+스캐너만
> 사용하는 작업(캘리브 등)까지만 가능하다.

---

## 1. 장비 없이 선행하는 셀 기하 점검 (권장)

```bash
env -u PYTHONPATH python scripts/artec/validate_real_cell.py
```

캘리브 yaml(`T_B_F0`·`T_EC_artec`)과 충돌 캐시만으로 **계획 로직을 실물 상수로** 검증한다.
로봇·스캐너·턴테이블이 없어도 된다. 출력은 네 가지다.

1. 셀 기하 — 턴테이블 축 위치, 도달한계 여유, '위' 방향
2. home 의 충돌 게이트 통과 여부 — 여기서 막히면 `is_path_safe` 의 start 검사에 걸려
   **모든 이동이 거부**된다. 실물 투입 전에 반드시 확인한다
3. lookaround 계획 자세(el × az)의 IK · 충돌 · home 기준 경로 통과율
4. nbv 의 반복 횟수와 이동량

### sim 으로는 본 점검을 대체할 수 없다

두 셀의 기하가 다르다(2026-09-17 실측).

| | 턴테이블 축 (base) | 수평 | 전체 | 도달여유 |
|---|---|---|---|---|
| v3 씬 (기본 아님) | `[0, 0, 0.835]` | 0 mm | 835 mm | +255 mm |
| real (실측 `T_B_F0`) | `[0.799, 0.005, 0.688]` | 799 mm | 1055 mm | **+35 mm** |

real 은 팔이 거의 펴진 자세(도달여유 +35mm)로 야코비안이 불리하다. 동일 코드·동일 후보
격자에서 az 를 0°→+30° 변경할 때의 최대 관절이동이 약 2배다.

```
sim  : el45 23.3°  el55 27.7°  el65 23.9°
real : el45 51.3°  el55 55.5°  el65 47.7°
```

따라서 **"sim 에서 검증하고 real 에 올린다" 는 이동량에 대해서는 성립하지 않는다.**
스캔 품질·SLAM·병합은 sim 에서 확인하되, **도달성·이동량은 본 스크립트로** 확인한다.

> ⚠ 비교 시 **IK seed 를 일치시킬 것.** 해석 IK 는 국소해이므로 seed 에 따라 결과가 크게
> 달라진다. 구 home 을 seed 로 쓰면 real 이 6/12 만 풀리고 el65 가 103° 로 나오나, 실물
> home 으로 재면 12/12 전부 풀린다. 기하의 한계가 아니라 seed 문제다.

---

## 2. 연결 확인

```bash
env -u PYTHONPATH python scripts/artec/go_home.py   # 로봇만 홈 자세 — 통신·구동 확인
env -u PYTHONPATH python -c "import main_artec"     # 하드웨어 없이 import 확인
```

---

## 3. 캘리브레이션 (설치 후 1회, 이설 시 재수행)

**단일 진입점이 순서를 강제한다.**

```bash
env -u PYTHONPATH python scripts/artec/calibrate.py            # 전체
env -u PYTHONPATH python scripts/artec/calibrate.py --from 2   # 2단계부터
env -u PYTHONPATH python scripts/artec/calibrate.py --only 3   # 한 단계만
```

순서는 **intrinsic → hand-eye → 턴테이블 축**이다. `turntable_calib` 이 rim 점을 base 로
변환한 뒤 원을 피팅하는데, 그 변환에 `T_EC` 가 포함되기 때문이다. 원리·순서는
`1_calibration.md` §9, 절차 상세는 `calibration_runbook.md`.

시작 전 **사람이 수동으로 조준**한다 — 보드와 턴테이블이 카메라에 들어와야 한다. 보드를
원판 위에 올려두면 hand-eye 와 턴테이블을 같은 조준 자세에서 연속 수행할 수 있다.

### 3.1 ChArUco 보드 준비 (최초 1회)

```bash
env -u PYTHONPATH python scripts/artec/make_charuco.py --pdf
```

기본 프리셋은 `spider_dense` — **7×5 / sq 12mm / mk 9mm (84×60mm, 내부 코너 24개)**.
출력 A4 PDF 는 **"실제 크기 / 100%"** 로 인쇄한다(`자동 맞춤` 해제). 페이지의 100mm 검증
자를 재어 다르면 그 비율로 `--square-mm` 을 보정한다. 인쇄물은 평판에 주름 없이 붙인다.

> Spider 는 FOV 가 좁다 — 320mm 에서 가로 123mm × 세로 167mm 다. `a4` 프리셋
> (7×5 / 30mm = 210×150mm)은 화면을 벗어난다. 구 `spider` 프리셋(5×3 / 20mm)은 들어가나
> 내부 코너가 8개뿐이라 조금만 잘려도 캘리브가 풀리지 않는다.

### 3.2 카메라 내부 파라미터

```bash
env -u PYTHONPATH python scripts/artec/calibrate.py --only 1
# → config/calibration/artec_intrinsic.yaml
```

### 3.3 Hand-eye `T_EC`

```bash
env -u PYTHONPATH python scripts/artec/gen_calib_poses.py --from-view --write   # 자세 목록 생성
env -u PYTHONPATH python scripts/artec/calibrate.py --only 2
# → config/sensor_frames.yaml 의 T_EC_artec 직접 갱신 (이전 값은 .bak)
```

- 자세 **15~25개**, 자세 간 **회전 ≥30°** 를 확보한다(부족하면 풀리지 않는다)
- 허용 기준은 t_err ≤4mm / r_err ≤1.5° 다
- ⚠ 현재 저장값(2026-09-21, 20자세)은 **t_err 8.443mm / r_err 1.732°** 로 기준을 초과한다.
  실물 재개 시 재수행 대상이다

> 수동(teach) 모드는 제거되었다(hand-eye 2026-09-15, intrinsic 2026-09-16).
> `set_mode(2)` 가 브레이크를 해제하므로, 스캐너를 장착한 상태에서 이를 모르고 시작하면
> 팔이 하강한다. 현재는 자세 목록 순회만 수행한다.

### 3.4 턴테이블 축 `T_B_F0`

```bash
env -u PYTHONPATH python scripts/artec/calibrate.py --only 3
# → 원판 rim 위 점 클릭 → config/calibration/turntable_frame.yaml
```

- rim 이 한 화면에 들어오지 않으면 보이는 호에서 클릭한다. 3점이면 되나 호가 짧으면
  정밀도가 저하된다
- 허용 기준은 잔차 ≤1mm 다. 현재 저장값(2026-09-21, 12점)은 잔차 **0.193mm**,
  rim 반경 118.71mm 로 기준을 충족한다

---

## 4. 스캔 실행

`main_artec.py` 상단 `BACKEND = "real"` 을 확인한 뒤 실행한다.

```bash
env -u PYTHONPATH python main_artec.py                          # 터미널 A
env -u PYTHONPATH python scripts/artec/live_scan_view.py        # 터미널 B (라이브 뷰어)
```

라이브 뷰어는 누적 점군을 실시간 표시하여 노이즈·드리프트·정지를 확인한다. 정합이
끊기면 배경이 적색으로 바뀐다. **반드시 별도 터미널에서 실행한다** — 파이프라인이
자식 프로세스로 띄우면 Filament 창이 즉시 종료된다(`6_postprocess.md` T2).

턴테이블 단독 조작:

```bash
env -u PYTHONPATH python scripts/turntable/lookaround_speed_rotation.py   # 회전 테스트
env -u PYTHONPATH python scripts/turntable/turntable_stop.py              # 비상 정지
```

---

## 5. 후처리 반복 실험 (스캔 없이)

실물 스캔은 1회에 수 분이 소요된다. 알고리즘 수정 시에는 반드시 아래 경로를 사용한다.

```bash
env -u PYTHONPATH python scripts/artec/save_raw_scan.py                    # raw 저장
env -u PYTHONPATH python scripts/artec/merge_compare.py --load output/scan_raw/<TS>
```

---

## 6. 시연·보고용

```bash
env -u PYTHONPATH python scripts/artec/main_artec_demo.py
```

평상시 알고리즘은 tracking lost 가 없으면 로봇이 거의 움직이지 않는 것이 정상이므로
영상 소재로는 부적합하다. 본 스크립트는 보고용으로 동선을 의도적으로 추가한다.

---

## 7. 문제 해결

| 증상 | 원인 · 조치 |
|---|---|
| `No module named 'xarm'` | env 오선택 → `conda activate mms-env` |
| `No module named 'cv2'` | base 파이썬으로 실행한 것 → `conda activate mms-env` 후 `python` |
| `No module named 'omni.usd'` | `sim_harness/` 를 standalone 으로 실행한 것 → Isaac GUI Script Editor 에서 실행 |
| `connect socket failed` | 로봇 미도달 → 상단 네트워크 점검 |
| `scan settings import 불가` | Artec SDK python 바인딩 미빌드 → `artec_SDK/artec0_build_guide.md`. sim 은 정상 동작 |
| 스캔이 반복 중단된다 | 시작 자세 불량. 물체를 조준한 상태로 시작하고, 반복되면 recovery 로그의 elevation 확인(`3_lookaround.md`) |
| 물체를 치웠는데 계속 스캔된다 | 정합이 `ICP` 로 설정되어 있을 수 있다 → `HYBRID` 확인 |
| 결과 메시가 어긋나거나 부유한다 | 거의 항상 `T_pre` 문제 → `5_flip.md` |
| 콘솔 이모지 깨짐 | `PYTHONIOENCODING=utf-8` |

SDK 바인딩 변경 시 재빌드:
`cmake --build mms_artec/sensor/build --config Release --target <module>`

---

## 8. 설정 위치

값을 변경할 때는 코드를 탐색하지 말고 해당 문서의 설정 표를 참조한다.

| 항목 | 위치 |
|---|---|
| 캘리브 보드 프리셋·자세 수 | `1_calibration.md` §2·§4 |
| lookaround — 자세 후보, 캡처 밀도, 회전 시간, watchdog | `3_lookaround.md` §1 |
| nbv — 반복, 거리, 충돌 world 치수, 수렴 임계 | `4_nbv.md` §1 |
| 충돌 — 안전 여유, 특이점 임계, SDF 격자 | `collision.md` §1 |
| flip — 축·각, pass 상한 | `5_flip.md` §1 |
| 후처리 — 단계별 on/off, fusion 방식 | `6_postprocess.md` §1 |

하드웨어 IP·홈 자세 등 상수는 `main_artec.py` 상단에 모여 있다. 셀 배치·치수는
`hw_layout.md` 를 참조한다.
