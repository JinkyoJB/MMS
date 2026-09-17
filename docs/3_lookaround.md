# lookaround — 5면 스캐닝

> 턴테이블이 한 바퀴 도는 동안 로봇은 **한 자세에 고정**된 채 연속 스캔해 윗면과 옆면
> 4개, 합쳐서 5면을 얻는다. 바닥면은 원판에 닿아 있어 flip(뒤집기)의 몫이다.
>
> 앞 단계 `1_calibration.md` · 부족면 보강 `4_nbv.md`
>
> **쓰기만 하면 §1(실행)·§2(sim·real 차이)로 충분하다.** §3 이후는 왜 그렇게 도는지다.
> 문제가 생기면 맨 뒤 부록(T1~T9)을 본다.

---

## 1. 실행

### sim

```bash
./scripts/sim/run_e2e_gui.sh                     # 기본 씬, lookaround
./scripts/sim/run_e2e_gui.sh spray_can           # 물체별 씬 (부분이름, 3밴드 사례)
./scripts/sim/run_e2e_gui.sh mug 2 planner 10    # lookaround→2, 베이스 +10cm
```

인자는 `[물체] [stage_until] [planner|legacy] [ΔH cm]` 다. `stage_until` 는 누적이라
`1`=5면, `2`=+NBV, `3`=+바닥면이다.

![sim lookaround 실행 화면](figures/lookaround/sim_run_spray_can.png)

오른쪽은 종료 후 뜨는 결과 뷰어(`utils/viz.py::show_composite_mesh`)다. 파란 원판은
턴테이블 상면 기준 도형이고, 물체가 **단색 빨강인 것은 `paint_uniform_color` 탓**이지
스캔 색이 아니다.

| 환경변수 | 뜻 | 기본 |
|---|---|---|
| `MMS_SIM_STAGE_UNTIL` | 1 / 2 / 3 (누적) | 1 |
| `MMS_SIM_P1_MODE` | `planner`(§4 채점기) 또는 `legacy` | planner |
| `MMS_SIM_P1_ELS` | 채점할 고도각 후보(°) | `30,40,50,60,70` |
| `MMS_SIM_NTHETA` | 한 바퀴 캡처 프레임 수 | 24 |
| `MMS_SIM_USD` / `_OBJECT_PRIM` | 씬·대상물 prim | 스크립트가 계산 |
| `MMS_ISAAC_HEADLESS` | 1이면 GUI 없이 | 0 |

**씬은 v3 만 쓴다**(`2_3Dassets/frame_xarm7_spider_turntable_v2/v3_scene.usd`,
물체별은 `v3_ts_<이름>.usd`). 새 물체는 USD 를 `2_데이터/testset/` 에 넣고 한 번 만든다.

```bash
env -u PYTHONPATH $MMS_PYTHON scripts/sim/build_scene_v3.py \
    --out v3_ts_<이름>.usd --object "$(mms_testset_dir)/<이름>.usd"
```

### 실물

`main_artec.py` 의 `BACKEND = "real"` 로 바꾸고 실행한다. 회전 시간 등 lookaround 설정은
같은 파일 `STREAM_SETTINGS` 에 있고 기본은 `rotation_duration_s = 30.0` 이다.

```bash
env -u PYTHONPATH $MMS_PYTHON main_artec.py
```

시작 자세는 §4 플래너가 고른다. 끄려면 `lookaround_planner_enabled=False`.

> **첫 실물 테스트는 `stage_until = 1` 로 5면부터.** nbv·flip 도, 자세 선정도 실물
> 검증 전이다(T7).

---

## 2. sim 과 real — 무엇이 같고 무엇이 다른가

판단 로직(자세 선정 §4, 밴드 분할 §5)은 **같은 코드다.** `collect_planning_points` →
`plan_lookaround_viewpoints` → `solve_plan_poses` 를 두 백엔드가 그대로 부르고, 주입하는
것은 콜백 세 개(preview 캡처 / 자세 IK / 충돌 게이트)뿐이다. 작업 프레임만 sim=world,
real=base 로 다르다.

**다른 것은 스캔 쪽이다.**

| | sim | real |
|---|---|---|
| 프레임 정합 | 없음. GT θ 로 누적(§8) | Artec SDK streaming SLAM |
| 추적 상실 | 개념 없음 | watchdog 4종 + 자동 복구(§7) |
| 라이브 뷰어 | `MMS_SIM_VIZ=1` 뷰포트 오버레이 | SDK 정합행렬 미러 |
| 턴테이블 | kinematic 직접 회전(명령각=실측각) | Ezi-SERVO, **UDP**(T2) |

즉 **자세를 어디로 보낼지는 sim 이 담보하지만, 그 자세에서 스캔이 붙어 있느냐는 실물
에서만 확인된다.** 실물 실패는 대부분 아래쪽 행에서 나오므로 T1·T2·T7 을 먼저 읽는 게
빠르다.

---

## 3. 왜 이렇게 도는가

Artec Spider 는 프레임 좌표를 **직전 프레임에 정합해서** 얻는다(streaming SLAM). 기준이
상대적이라 한 프레임을 놓치면 그 뒤를 이어붙일 근거가 사라진다 — **추적을 잃는 순간
그때까지 쌓인 것만 남는다.**

그래서 성패를 가르는 것은 hand-eye 나 θ 의 정확도가 아니라 **프레임 간 겹침**이다.
각도를 띄엄띄엄 옮기면 겹침이 끊기므로 턴테이블을 **천천히 연속 회전시키면서 최대 FPS
로** 찍는다. 회전을 멈추고 자세를 옮기는 방식은 쓰지 않는다.

이 성질에서 나머지가 따라온다. 방위각은 턴테이블이 다 커버하므로 로봇이 고를 자유도는
**고도각 φ 하나**(§4)고, 한 자세로 높이를 못 덮으면 z 방향 밴드로 나눈다(§5). 그리고
추적을 감시할 watchdog 과 끊겼을 때 되돌아갈 지점이 반드시 필요하다(§7).

---

## 4. 자세 선정 — 전회전 maximin 채점

`utils/nbv/lookaround.py::plan_lookaround_viewpoints` (sim·real 공용).

물체가 도는 동안 로봇은 고정이므로, 자세를 고르는 일은 **360° 전체를 그 자세 하나로
감당할 수 있는가**를 묻는 일이다. 후보마다 θ 를 36등분해 한 바퀴를 미리 시뮬레이션하고
**평균이 아니라 최악 프레임으로** 점수를 매긴다(maximin). 납작한 물체가 edge-on 으로
지나가는 한순간에 끊기면 그 뒤가 전부 날아가기 때문이다.

```
score = 2.0 · min(min_fill / 12cm², 1)   ← 최악 프레임 가시면적 (지배항)
      + 1.0 · z_cover_frac                ← 높이 방향 커버율
      + 0.5 · covered_frac                ← 품질-가시 비율
```

탐색 격자는 고도각(`DEFAULT_ELS = 20/30/40/50°`, sim 은 `30~70`) × standoff 3종
(`near+20mm+r_max`, `mid+r_max`, `mid+r_max+30mm`) × 타깃높이 3종(z 의 35/50/65 분위수)다.
**방위각은 점수에 안 들어간다** — 회전 대칭이라 관측 조건을 못 바꾸고 도달성과 충돌만
좌우하므로, 백엔드가 스윕해 통과하는 값을 고른다(T4).

가시성 판정(`SensorModel`)은 frustum ∩ 작동거리(0.20~0.30m) ∩ 입사각인데, **입사각
한계가 둘**인 것이 요점이다. 누적·품질에는 50°까지만 쳐주고 추적 기여는 75°까지
인정한다 — 비스듬히 스치는 면은 모델엔 보탬이 안 돼도 추적은 붙잡아 준다.

물체 점은 클러스터링 없이 **캘리브 기하로 자른다**(`crop_object_points`: 원판 위 +
축 실린더 안). "턴테이블을 물체로 착각해 조준하는" 실패가 구조적으로 불가능해진다.

**점군은 GT 없이 실제 캡처로 모은다**(`collect_planning_points`). 턴테이블을 0°/90° 로
돌려 실루엣 두 방향을 얻고(90°는 −θ 로 되돌림), 조준높이는 디스크 위 5cm 에서 시작해
"새 캡처가 상단을 더 못 늘리면 종료"로 최대 4회 올린다. 각 높이에서 축거리 0.30·0.38m
두 스텝을 찍으면 작동거리 창(0.20~0.30m) 덕에 표면 반경 0~18cm 가 전부 덮인다.
캡처한 점은 로봇 자기 점을 먼저 지운다(`filter_robot_points`) — 링크가 프레임에 걸리면
크롭 실린더를 오염시켜 밴드 수가 폭주한다(2026-07-08 세제 13밴드).

최악 프레임이 `FILL_MIN_CM2 = 6cm²` 미만이면 계획은 그대로 두되 `tracking_risk` 로 표시만
한다. 복구 전용 채점기는 이것과 **다른 것**이다(§7).

---

## 5. 밴드 분할

한 자세로 높이를 다 못 덮으면 z 방향으로 겹치는 밴드를 나눠 여러 번 돈다.

![밴드 분할 계획](figures/lookaround/fig4_bands.png)

**기준은 높이가 아니라 §4 에서 고른 자세의 z-커버율**이다(`< ZCOVER_MIN 0.75` 이면 분할).
standoff 가 물체 반경에 비례하므로 **가는 물체일수록** 카메라가 가까워 시야가 높이를 못
덮는다 — 293mm 세제(지름 194)가 2밴드인데 207mm 스프레이캔(지름 68)은 3밴드다.
`[lookaround] z_cover=` 로그로 확인한다.

밴드 높이는 자세가 실제로 덮은 z 폭으로 잡고 `BAND_OVERLAP = 0.35` 만큼 겹친다. 이 겹침이
곧 relocalization 성립 조건이다. 순서는 **안전한 것부터**(min_fill 내림차순) — 위험한
밴드를 나중에 돌면 그때까지 쌓인 모델이 복구의 기준점으로 남는다.

밴드 하나가 실패해도 중단하지 않고 건너뛴다. 도달 못 한 대역은 nbv 가 메운다(T3).

---

## 6. 실물 아키텍처

`mms_artec/nbv/artec_streaming_scan_session.py::ArtecStreamingScanSession.run()` 이
스캐너와 턴테이블을 두 thread 로 물려 돌린다. 둘은 `TrackingState` 하나를 공유하고,
추적을 잃으면 즉시 `stop_event` 로 턴테이블을 세운다.

```
   Main thread                        TurntableController (thread)
   poll_events() ─ SDK 큐 비움         move_velocity 연속 회전
   θ 기록 · 라이브 뷰어 공급           getActualPos polling 10Hz
   watchdog 4종 검사                   stop_event → 즉시 정지
   last-good θ 갱신                    drive 통신사망 watchdog
            └────── TrackingState (공유) ──────┘
```

시작 시 턴테이블 logical 0 을 reset 하고, preview 를 띄워 settle·drain 한 뒤(preview
프레임이 통계에 섞이면 안 되므로 카운터를 0 으로) `start_record` 와 동시에 회전을 건다.
매 tick 마다 `poll_events()` 를 부르는데 **이걸 거르면 SDK 가 얼어붙는다.**

결과 `ArtecStreamingScanResult` 는 `model`, `n_frames`, `rotation_actual_deg`,
`fps_actual`, `tracking_lost`, `loss_reason`, 그리고 복구가 되돌아갈 기준인
**`last_good_theta_rad`**(= `reg_err ≥ 0` 인 마지막 θ)를 담는다.

---

## 7. 추적 감시와 자동 복구

| # | 조건 | 설정키 | 기본 |
|---|---|---|---|
| 1 | 정합/재구성 실패 연속 | `consecutive_loss_threshold` | 8 |
| 2 | callback 무반응 | `stale_threshold_s` | 2.0 |
| 3 | `reg_err < 0` 연속 (Studio 의 tracking lost) | `consecutive_reg_err_threshold` | 5 |
| 4 | `reg_err > max` 연속 | `consecutive_high_err_threshold` / `max_acceptable_reg_error` | 8 / 1.5 |

`reg_err = -1` 은 SDK sentinel 이라, `reg_err ≥ 0` 을 한 번 본 뒤부터만 (3)(4)를 센다
(`tracking_established`). 이 플래그를 건드리면 warm-up 이 곧바로 lost 로 잡힌다.

**복구**(`ArtecMultiPassScanSession`)는 같은 자세에서 최대 3회 retry 하고,
`last_good_theta_rad` 보다 10° 더 뒤로 턴테이블을 되돌린 뒤(safe-back)
`_adaptive_prescan_position(recovery=True)` 로 자세를 다시 고른다.

이때 쓰는 채점기는 §4 의 maximin 과 **다르다.** 복구는 빨라야 하므로 한 바퀴를 돌리지
않고 preview 한 장만 본다 — 물체로 분류된 점이 **최적거리 225mm 근처 FOV 안에** 얼마나
모였는지를 `w(d) = exp(−((d−225)/25)²)` 로 가중해 합산한다. 후보는 `[-5°, 0°, +5°]` 로
줄여 30초 안에 끝낸다. 구현은 `artec_multipass_scan_session.py::_elevation_search`,
`_lookaround_view_score`. 한계는 T6.

sim 검증은 `sim_harness/MMS_ext_lookaround_recovery1.py`(빗나가게 조준 → 점 0 → lost),
`recovery2.py`(el 을 낮춰 윗면 grazing → lost) 두 개다. 둘 다 safe-back → 자세 재탐색 →
재개로 5면을 완성한다.

---

## 8. 라이브 뷰어와 sim 검증

**라이브 뷰어**(real 전용)는 누적 좌표로 **SDK 정합행렬 `FrameEvent.transformation`만**
쓴다. θ·yaml·hand-eye 가 개입하지 않으므로 화면이 곧 SLAM 결과 그 자체다. 필터도 SDK 가
IScan 에 넣는 기준(`reg_err ≥ 0`)과 똑같이만 건다 — 자체 기준으로 더 거르면 거울이
아니게 된다. 표시는 별도 터미널의 Filament 뷰어(`mms_artec/nbv/live_scan_viewer.py`).

**sim 검증**에는 SLAM 이 없다. 대신 GT θ 와 회전축으로 누적한다.

```
p_obj = Rz(−θ) · (p_world − axis_point) + axis_point
```

θ 가 정확하면(sim 은 GT) 모든 옆면이 정확히 겹쳐 쌓인다. 실물의 SLAM 은 이렇게 검증된
누적 위에 그대로 올라간다. 하니스는 `sim_harness/MMS_ext_lookaround.py`.

Isaac 없이 밴드 계획만 확인하려면:

```bash
~/isaacsim/python.sh scripts/sim/extract_testset_points.py   # 점군 캐시(1회)
$MMS_PYTHON scripts/sim/validate_lookaround.py
```

---

## 9. 코드 지도

```
utils/nbv/lookaround.py       ★ 자세 선정 전부 (sim·real 공용)
    collect_planning_points()           # §4 preview 수집 루프
    plan_lookaround_viewpoints()            # §4 maximin 채점 + §5 밴드 분할
    solve_plan_poses()                  # az 스윕 IK + 충돌 게이트
    crop_object_points / filter_robot_points
utils/nbv/scan_stage_controller.py  # lookaround→2→3 순서 (밴드 실패 허용)

mms_artec/nbv/artec_streaming_scan_session.py   ★ 실물 streaming SLAM
    TrackingState(watchdog 4종) · TurntableController · run()
mms_artec/nbv/artec_multipass_scan_session.py   # 오케스트레이션 + 복구 + view-score
mms_artec/nbv/live_scan_viewer.py               # SDK 정합행렬 미러
mms_artec/nbv/recovery_pose_selector.py         # 복구용 Spider 광학 상수
mms_artec/backends/isaac/isaac_turntable.py     # sim 턴테이블 (kinematic 직접 회전)
utils/turntable/turntable_interface.py          # 실물 턴테이블 (UDP)

scripts/sim/run_e2e_gui.sh · validate_lookaround.py
sim_harness/MMS_ext_lookaround{,_recovery1,_recovery2}.py
```

---

# 〔부록〕 Troubleshooting

### T1. 물체를 치웠는데 tracking lost 가 안 뜬다
`registration` 이 ICP-only 다. 물체가 없어도 **빈 원판에 정합이 성공**해 버린다.
`set_registration_type(HYBRID)`(geometry+texture)를 반드시 쓴다.

### T2. 스캔 도중 턴테이블 통신이 멎는다
턴테이블은 **UDP** 로 붙여야 한다. TCP 는 `getActualPos` 를 10Hz 로 계속 두드리는
구간에서 socket 이 막힌다.

### T3. 밴드/자세 관련 회귀 3건 (모두 수정됨)
- 밴드 하나가 도달 불가하면 즉시 finalize 해서 **nbv·flip 이 통째로 사라졌다.** 지금은
  건너뛰고 계속, 전부 실패했을 때만 포기한다.
- **충돌 게이트가 밴드 경로에만 없었다.** IK 해가 나오면 충돌을 안 보고 확정해서, 충돌
  없는 다른 az 를 시도조차 못 했다(실측: az 0°는 tool↔link4 4mm 자가충돌, az 30°는 통과).
- 계획용 preview 가 `_drive()` 반환값을 버려 **이동이 거부돼도 그 자리에서 찍었다.**
  고친 뒤 유효 캡처 2→4장, standoff 283→313mm — 계획이 실제로 오염되고 있었다.

> 셋 다 같은 교훈이다. **같은 판단을 하는 코드가 여러 곳이면 게이트가 전부에 들어갔는지
> grep 으로 확인하고, 실패를 돌려주는 함수의 반환값을 버리지 말 것.**

### T4. 충돌 없는 az 가 있는데 "도달 불가"가 뜬다
az 는 도달성·충돌만 좌우하므로 `solve_plan_poses` 가 스윕한다. 로그에
`band tz=… az=…° 충돌(...)` 이 안 보이면 게이트가 안 걸린 것이다(T3 두 번째 건).

### T5. 스캔 시작 직후 lost 로 오판된다
`reg_err = -1` 은 실패가 아니라 SDK sentinel 이다. §7 의 `tracking_established` 확인.

### T6. 〔한계〕 복구용 view-score 는 θ 한 시점만 본다
θ=0 preview 하나로 고르므로, 손잡이·주둥이가 있는 비대칭 물체는 다른 구간에서 나쁠 수
있다. 미구현 해결안: ① 물체점을 축 기준 회전 보정해 가상 θ N개 합산 ② 후보마다 짧게
돌려 평균 ③ top-K multi-pass. §4 의 maximin 은 36 프레임을 다 보므로 이 한계가 없다 —
복구가 그걸 안 쓰는 이유는 시간뿐이다.

### T7. 〔미검증〕 real 자세 선정을 실물에서 안 돌려봤다
코드는 붙어 있고(2026-09-09, sim 과 같은 함수) sim 실행만 검증됐다. 첫 실물 때:
1. **`fill_target 12cm²` / `fill_min 6cm²` 은 sim 기준값이다.** 실물 SLAM 이 버티는 최소
   가시면적으로 다시 잡아야 `tracking_risk` 경고가 맞는다.
2. 계획용 preview 가 **최대 16회 이동**한다(거리 2 × 높이 4 × 턴테이블 2). 게이트를
   통과하지만 첫 시도는 비상정지 옆에서 볼 것.
3. `T_BF0` 가 없으면 플래너가 포기하고 home 고정으로 간다 — 캘리브부터.

막히면 `lookaround_planner_enabled=False` 로 예전 동작(home 고정)으로 돌아갈 수 있다.

### T8. sim 물체가 회색이거나, 30초 만에 빈 결과로 끝난다
- **회색** — 텍스처 참조 결손. `UsdPreviewSurface` 는 텍스처가 안 열리면 말없이 회색으로
  렌더한다. 9종 중 8종은 복구했고 `0263_protein_drink` 만 남았다(`2_데이터/README.md`).
  lookaround~3 은 형상만 쓰므로 결과에는 영향이 없다.
- **빈 결과** — 구 v2 씬(`v2.usd`, `composed/*_on_turntable.usd`)을 쓴 것이다. 카메라·
  턴테이블 prim 경로가 달라 스캔 없이 끝난다. v3 씬만 쓴다(§1).

### T9. 지켜야 할 규약
- **연속 회전 + 최대 FPS.** 겹침이 생명이라 각도를 띄엄띄엄 옮기지 않는다.
- **자세 선정은 sim·real 이 같은 코드.** 한쪽만 고치면 갈라진다 — 백엔드가 아니라
  `utils/nbv/lookaround.py` 를 고친다.
- **라이브 뷰어 누적은 SDK 정합행렬만.** θ·hand-eye 를 섞으면 디버깅 가치가 사라진다.
- **sim 은 SLAM 이 없다.** sim 이 잘 된다고 실물 정합이 잘 된다는 뜻이 아니다.
- **last-good θ** 는 복구 safe-back 의 유일한 기준이다.
