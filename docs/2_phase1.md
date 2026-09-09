# Phase 1 — 5면 스캐닝 (streaming SLAM)

> 턴테이블이 한 바퀴 도는 동안 Spider 가 연속 스캔으로 윗면과 옆면 4개, 합쳐서 5면을
> 취득한다. 바닥면은 원판에 닿아 있어 얻을 수 없으므로 Phase 3(뒤집기)의 몫이다.
>
> 앞 단계는 `1_calibration.md`, 부족면 보강은 `3_phase2.md` 를 본다.
>
> **결과가 이상하면 맨 뒤 〔부록〕 Troubleshooting 을 먼저 본다.** 규약·함정·알려진
> 미해결 문제를 T1~T8 로 모아두었다.

---

## 1. 무엇을 하나

턴테이블 위의 대상물이 도는 동안 **로봇은 한 자세에 고정된다.** 방위각은 턴테이블이
전부 커버하므로 로봇이 물체 둘레를 돌 이유가 없다.

```
   [고정] 스캐너 ──본다──▶ ◐ 대상물 (턴테이블 위, θ 회전)
                              │
                         360° 도는 동안 옆면이 차례로 스캐너 앞을 통과
```

로봇이 고르는 자유도는 사실상 **고도각 φ 하나**이고, 물체가 키가 커서 한 자세로 높이를
다 못 덮을 때만 z 방향으로 **밴드**를 나눠 여러 바퀴를 돈다. 전자는 §3, 후자는 §4 에서
다룬다.

---

## 2. 적용된 로직 — 연속 회전 스트리밍 스캔

Artec Spider 는 한 frame 의 좌표를 **직전 frame 에 정합해서** 얻는다. 절대 좌표를 프레임마다
따로 주는 센서와 달리 기준이 상대적이므로, 한 frame 을 놓치면 그 다음부터 좌표를 이어붙일
근거가 사라진다. **추적을 잃는 순간 그 시점까지 쌓인 frame 만 살아남는다.**

그래서 Phase 1 의 성패를 가르는 것은 hand-eye 나 θ 의 정확도가 아니라 **frame 간
겹침(overlap)** 이다. 각도를 띄엄띄엄 옮기면 겹침이 끊기므로, 턴테이블을 **천천히 연속
회전시키면서 최대 FPS 로 찍어** 매 frame 이 직전 frame 과 겹치게 한다. 회전을 멈추고
자세를 옮기는 방식은 쓰지 않는다.

이 성질 때문에 Phase 1 에는 추적 상태를 실시간으로 감시하는 watchdog 과, 끊겼을 때
되돌아갈 지점(last-good θ), 그리고 자동 복구 절차가 함께 붙는다(§6).

시작 자세는 **사용자가 눈으로 조준해 둔 home 자세를 그대로 쓴다**(2026-05-20 결정).
사전 probe 나 elevation 탐색을 하지 않는데, 그 탐색이 모든 스캔에 1~2분을 더하는 데
비해 잘 조준된 자세면 대개 그냥 성공하기 때문이다. 추적이 끊겨야 비로소 자세가 나쁘다는
신호이고, 그때만 **recovery 로직**(추적을 잃었을 때 자동으로 되돌아가 자세를 다시
고르는 복구 절차, §6)이 개입한다.

---

## 3. 자세 선정 — elevation view-score

로봇이 고를 자유도는 고도각 φ 하나이므로, 후보 φ 들을 preview 로 찍어 점수화하고
가장 높은 것을 고른다.

> **score(φ)** = 그 자세의 preview 점 중 **물체로 분류된** 점들이
> **최적 작업거리(≈225mm) 근처의 FOV 안** 에 얼마나 모여 있는가(개수 가중합).

| 단계 | 하는 일 |
|---|---|
| 1 | preview 점군을 카메라 프레임에서 base 프레임으로 옮긴다. 멤버십 판정과 `z_table` 이 모두 base 기준이기 때문이다. |
| 2 | 물체에 속하는 점만 세 조건의 AND 로 고른다. 실린더로 미리 자르고, 턴테이블 상판을 hard floor(`z > z_table+8mm`)로 쳐내고, probe 로 만든 축대칭 `(r,z)` occupancy 를 조회한다. occupancy 는 360° 회전대칭화되어 있어 어느 θ 에서도 성립한다. |
| 3 | 광축 기저를 캘리브레이션된 `fwd_C`·`up_C` 로부터 정규직교화해 만든다. 하드코딩된 `+Z_C` 를 쓰지 않는 이유는 스캐너가 조금 삐뚤어 장착돼도 판정이 틀리지 않게 하기 위해서다. |
| 4 | Spider 의 FOV 30°(수평)×21°(수직) 안인지 본다. `depth > 1mm` 조건으로 카메라 등 뒤의 점을 제거한다. |
| 5 | 거리에 Gaussian 가중을 준다. `w(d) = exp(−((d−225)/25)²)` 이므로 작동 대역 [200,250]mm 가 1σ 에 해당한다. |

최종 점수는 `score = Σ w(depth) · 1_FOV` 다. 물체 점 개수가 같더라도 **225mm 부근에
모인 자세가 이긴다.**

**언제 호출되나.** 정상 시작 때는 호출하지 않는다(사용자가 맞춘 home 자세로 출발).
**recovery 로직(§6)이 돌 때만** 호출하며, 그때는 후보를 `[-5°, 0°, +5°]` 로 줄이고
fine search 를 건너뛰어 30초 안에 끝낸다.

구현은 `artec_multipass_scan_session.py` 의 `_elevation_search`, `_phase1_view_score`,
`_in_object_profile`, `_build_rz_profile` 이다. 판정에 쓰는 Spider v1 광학 상수는
`recovery_pose_selector.py` 에 있다 — FOV 30°×21°, 작동거리 170~350mm(최적 200~250),
기본 standoff 250mm, 3D 해상도 0.1mm, 정확도 0.05mm.

view-score 의 구조적 한계는 부록 T6 을 본다.

---

## 4. 밴드 분할

한 자세로 대상물의 높이를 다 덮지 못하면 z 방향으로 서로 겹치는 **밴드**를 나눠 여러 번
돈다. 계획은 `phase1_viewpoint.plan_phase1_viewpoints` 가 세운다.

![밴드 분할 계획](figures/phase1_viewpoint/fig4_bands.png)

**분할 여부는 물체의 높이가 아니라 단일 자세의 z-커버율로 정해진다.** standoff 가 물체
반경에 비례해 결정되므로 **가는 물체일수록** 카메라가 가까이 붙어 시야가 높이를 못 덮는다.
그래서 293mm 세제(지름 194mm)가 2밴드인데 그보다 낮은 207mm 스프레이캔(지름 68mm)은
3밴드로 나뉜다. 어떤 밴드로 갈렸는지는 `[phase1] z_cover=` 로그로 확인한다.

밴드는 **안전한 것부터** 돈다. 위 그림에서 minfill 이 큰 band1 → band2 → … 순으로
스캔하는 이유는, 위험한 밴드를 나중에 돌면 그때까지 쌓인 모델이 추적 복구(relocalization)의
기준점으로 남아 있기 때문이다.

밴드 경로에서 과거에 터졌던 두 결함은 부록 T3·T4 에, 아직 남아 있는 문제는 T5 에 있다.

---

## 5. 실물 아키텍처 — Spider ↔ 턴테이블 양방향 피드백

`mms_artec/nbv/artec_streaming_scan_session.py` 의 `ArtecStreamingScanSession.run()` 이
스캐너와 턴테이블을 두 개의 thread 로 물려 돌린다. 둘은 `TrackingState` 하나를 공유하고,
스캐너 쪽이 추적을 잃으면 그 즉시 `stop_event` 로 턴테이블을 세운다.

```
              ┌──────────── TrackingState (공유) ─────────────┐
              │  frames_ok/failed, consecutive_lost,           │
              │  last_reg_error, tracking_lost, stop_event     │
              └──────▲───────────────────────────▲─────────────┘
   frame_callback 갱신│                           │ stop_event 감시
   ┌─────────────────┴────────┐      ┌────────────┴───────────────┐
   │ Main thread (run)        │      │ TurntableController(thread)│
   │  session.poll_events()   │      │  move_velocity 연속 회전    │
   │  4 watchdog 검사          │      │  getActualPos polling(10Hz)│
   │  live viewer 공급         │      │  stop_event → 즉시 stop     │
   │  last-good θ 기록         │      │  drive 통신사망 watchdog     │
   └──────────────────────────┘      └────────────────────────────┘
```

### `run()` 의 흐름

1. FPS 를 목표값과 스캐너 최대값 중 작은 쪽으로 정하고, registration 을 HYBRID
   (geometry + texture)로 놓는다.
2. ScanSession 을 만든다. `set_registration_type(HYBRID)`, `set_pipeline(...)`,
   `initial_state=PREVIEW`, `capture_texture=ALWAYS` 를 설정하고
   `frame_callback` 에 `TrackingState.on_frame` 을 건다.
3. 턴테이블의 logical 0 을 reset 한다(clearpos). 드라이브 alarm 등으로 실패하면 빈 결과를
   즉시 반환하고 끝낸다.
4. Preview 를 띄우고 settle 시킨 뒤 큐를 비운다(drain). Preview frame 은 통계에
   포함되면 안 되므로 여기서 카운터를 0 으로 되돌린다.
5. `start_record` 와 동시에 `TurntableController` thread 를 띄운다. 각속도는
   `2π / rotation_duration_s`(기본 30초에 한 바퀴), 목표각은 `360° + overshoot` 이다.
6. Main loop 는 매 tick 마다 다음을 한다.
   - `session.poll_events()` 로 SDK 큐를 비운다. 이걸 거르면 SDK 가 얼어붙는다.
   - `tt_ctrl.actual_pos_rad` 로 현재 θ 를 읽어 timeline 에 기록한다.
   - 라이브 뷰어에 OK frame 의 `frame_mesh` 와 **SDK 정합행렬 `ev.transformation`** 을
     넘겨 scan-world 좌표로 누적시킨다(§7).
   - `reg_err ≥ 0` 인 마지막 θ 를 **last-good θ** 로 갱신한다. recovery 로직이
     되돌아갈 기준점이다.
   - watchdog 4개(§6)를 검사하고, 하나라도 걸리면 턴테이블을 즉시 정지시킨다.
   - `tt_ctrl.completed` 면 정상 완료, `aborted` 면 abort 로 빠져나온다.

### 결과 (`ArtecStreamingScanResult`)

`model`(IModel), `n_frames`, `rotation_actual_deg`, `duration_s`, `fps_actual`,
`tracking_lost`, `loss_reason`, `frames_ok`/`frames_failed`, 그리고 recovery 로직의
rollback 기준이 되는 **`last_good_theta_rad`** 를 담는다.

---

## 6. 추적 감시와 자동 복구(recovery) 로직

### 4가지 watchdog (`TrackingState`)

| # | 조건 | 설정키 | 기본값 |
|---|---|---|---|
| 1 | FrameState 의 정합/재구성 실패가 연속으로 이어진다 | `consecutive_loss_threshold` | 8 |
| 2 | callback 이 N초 동안 아무 반응이 없다 | `stale_threshold_s` | 2.0 |
| 3 | `registration_error < 0` 이 연속된다 (Studio 의 'tracking lost' 시그널) | `consecutive_reg_err_threshold` | 5 |
| 4 | `registration_error > max` 가 연속된다 (정합 품질 급락) | `consecutive_high_err_threshold` / `max_acceptable_reg_error` | 8 / 1.5 |

스캔 직후의 `reg_err = -1` 은 SDK 가 쓰는 sentinel 값이다. 그래서 `reg_err ≥ 0` 을 한 번
본 뒤(`tracking_established`)부터만 (3)(4)를 센다. warm-up 구간을 lost 로 오인하지 않기
위한 장치다.

### 자동 복구(recovery) 로직

추적을 잃었다고 판정되면 스캔을 포기하지 않고 자동으로 복구를 시도한다. Phase 1 과 2 를
묶어 지휘하는 `ArtecMultiPassScanSession` 이 lost 를 받아 다음 순서로 처리한다.

1. 같은 자세에서 최대 **3회**까지 자동 retry 한다. lost 는 "물체가 없다"와 다르므로
   무한 retry 는 금지다.
2. **safe-back** — `last_good_theta_rad` 보다 10° 더 뒤로 턴테이블을 되돌린다.
3. `_adaptive_prescan_position(recovery=True)` 로 probe 를 새로 찍고 축소된 elevation
   search(§3)를 돌려 로봇 자세를 다시 고른다.

sim 에는 SLAM 이 없으므로 캡처된 점 개수를 프록시로 삼아, 일부러 나쁜 자세를 줘서 lost 를
유발하는 방식으로 이 흐름을 검증했다.

- `sim_harness/MMS_ext_phase1_recovery1.py` — **빗나감.** 대상물을 측면으로 빗나가게
  조준해 FOV 밖으로 내보내면 점이 거의 0 이 되어 lost 가 뜬다. recovery 로직이 물체 점이
  가장 많은 elevation 을 골라 중심을 다시 조준하고 스캔을 재개한다.
- `sim_harness/MMS_ext_phase1_recovery2.py` — **윗면 미포착.** el 을 너무 낮춰 옆에서
  보면 윗면이 grazing 되어 잡히지 않는다. recovery 로직이 윗면 비율이 최대가 되는 쪽으로
  elevation 을 올려 윗면을 잡고 재개한다.

둘 다 θ safe-back → 자세 재탐색 → 재개를 거쳐 5면을 완성한다. 형상이 까다로울 필요는
없다 — 자세가 나쁘면 lost 가 난다.

---

## 7. 라이브 뷰어 — SDK SLAM 을 그대로 비추는 거울

누적 좌표로 **SDK 가 준 정합행렬 `FrameEvent.transformation`**(sensor→scan-world)만 쓴다.
θ 도, yaml 도, hand-eye 도 개입하지 않는다. 즉 화면에 보이는 것이 곧 SDK SLAM 의 결과
그 자체이고, 뷰어에 이상하게 보이면 실제 스캔이 이상한 것이다.

필터도 SDK 가 IScan 에 frame 을 넣는 기준(`reg_err ≥ 0`)과 **똑같이만** 건다. 뷰어가
자체 기준으로 더 걸러내면 거울이 아니게 되기 때문이다.

표시는 별도 터미널의 Filament 뷰어로 한다(`mms_artec/nbv/live_scan_viewer.py`).
Isaac 확장 GUI 에서 창을 띄울 수 없는 것과 같은 제약이다.

---

## 8. sim 검증 — ground-truth 누적

sim 에는 Artec SLAM 이 없다. 대신 **턴테이블 θ(ground-truth)와 회전축(calibration)** 으로
점군을 누적해 같은 결과(5면 재구성)를 얻고, 이걸로 view-coverage 와 누적 로직을 검증한다.

```
대상물이 θ 회전 (ground-truth)
   │  매 θ: 카메라 포인트클라우드 캡처(world)
   │        대상물 점만 분리 (디스크 위 + 영역 크롭)
   │        축 둘레로 −θ 역회전 → 대상물 기준 프레임(θ=0)
   ▼
누적 → 5면(윗면 + 옆면 4) 완성 점군
```

핵심 식은 `p_obj = Rz(−θ)·(p_world − axis_point) + axis_point` 이고, axis 는 턴테이블
회전축(`1_calibration.md`)이다. θ 가 정확하면(sim 에서는 ground-truth 다) 모든 옆면이
정확히 겹쳐 쌓인다. 이것이 "SLAM 대신 GT 누적"이다.

스크립트는 `sim_harness/MMS_ext_phase1.py`(Isaac 확장)이다. 대상물(box)을 known θ 로
키네마틱 회전시켜 캡처하고 −θ 로 되돌려 누적한 뒤 재구성한다. 실물의 streaming/SLAM 은
이렇게 검증된 누적 로직 위에 그대로 올라간다.

---

## 9. 실행

### sim — Phase 1 E2E (GUI)

```bash
cd <MMS repo>
./scripts/sim/run_e2e_gui.sh                        # marble, Phase 1, 플래너
./scripts/sim/run_e2e_gui.sh spray_can              # testset 물체 (부분이름으로 매칭)
./scripts/sim/run_e2e_gui.sh detergent 1 planner    # 밴드 분할이 걸리는 사례
./scripts/sim/run_e2e_gui.sh marble 1 legacy        # 플래너 없이 A/B 비교
./scripts/sim/run_e2e_gui.sh marble 2 planner 10    # 베이스 +10cm, Phase 1→2
```

인자는 `[물체] [phase_mode] [planner|legacy] [ΔH cm]` 순이다. `phase_mode` 는 누적
실행이라 `1` = 5면, `2` = +NBV 보강, `3` = +바닥면 flip 을 뜻한다. `ΔH > 0` 이면
`hibase/*_dh<cm>.usd` 오버레이 씬을 한 번 생성해 캐시하며, 원본 `v2.usd` 는 건드리지 않는다.

주요 환경변수는 다음과 같다.

| 변수 | 뜻 | 스크립트 기본 |
|---|---|---|
| `MMS_SIM_PHASE_MODE` | 1 / 2 / 3 (누적) | 1 |
| `MMS_SIM_P1_MODE` | `planner`(밴드 플래너) 또는 `legacy` | planner |
| `MMS_SIM_NTHETA` | 한 바퀴를 몇 개 θ 로 나눠 캡처할지 | 24 |
| `MMS_SIM_DRIVE_STEPS` | 자세 이동 보간 step 수 | 20 |
| `MMS_SIM_USD` | 씬 USD 경로 | 스크립트가 계산 |
| `MMS_SIM_OBJECT_PRIM` | 대상물 prim 경로 (testset 씬) | 스크립트가 계산 |
| `MMS_ISAAC_HEADLESS` | `1` 이면 GUI 없이 실행 (서버/CI) | 0 |

직접 부를 때는 아래와 같다. `-u` 는 `app.close()` 로 버퍼가 날아가는 것을 막는다.

```bash
env -u PYTHONPATH \
  MMS_SIM_PHASE_MODE=1 MMS_SIM_P1_MODE=planner MMS_SIM_NTHETA=24 \
  ~/miniconda3/envs/env_isaacsim/bin/python -u main_artec.py
```

### sim — 밴드 계획만 오프라인 검증 (Isaac 불필요)

```bash
# 1) testset 점군 캐시 (최초 1회)
~/isaacsim/python.sh scripts/sim/extract_testset_points.py
# 2) 자세 선정·밴드 분할 검증
$MMS_PYTHON scripts/sim/validate_phase1_viewpoint.py
```

씬을 띄우지 않고 캐시된 점군만으로 기존 sphere sampling 대비 maximin 채점 + 밴드
분할의 최악프레임 fill·z-커버·IK 도달성을 비교하고, tracking-lost 프록시를 주입해
recovery 로직의 재계획도 함께 본다. 결과는 stdout 표와
`scripts/sim/log/testset_points/validate_results.json` 이다.

### 실물

`main_artec.py` 위쪽의 `BACKEND = "real"` 로 바꾸고 실행한다. Phase 1 관련 설정은 같은
파일의 `STREAM_SETTINGS`(`ArtecStreamingScanSessionSettings`)에 모여 있고, 한 바퀴에
걸리는 시간은 `rotation_duration_s = 30.0` 이 기본이다.

```bash
env -u PYTHONPATH $MMS_PYTHON main_artec.py
```

> **첫 Spider 실물 테스트는 반드시 `phase_mode = 1` 로 5면부터 확인한다.** Phase 2·3 은
> 실물에서 아직 검증되지 않았다.

---

## 10. 코드 지도

```
mms_artec/nbv/artec_streaming_scan_session.py   ★ 실물 streaming SLAM
    ArtecStreamingScanSessionSettings   # rotation_duration_s, registration=HYBRID, watchdog 임계 …
    TrackingState                       # .on_frame, .should_stop, watchdog 4종
    TurntableController                 # 별도 thread, move_velocity, getActualPos polling, stop_event
    ArtecStreamingScanSession.run()     # → ArtecStreamingScanResult
mms_artec/nbv/artec_multipass_scan_session.py   # Phase 1+2 오케스트레이션 + recovery 로직 + view-score
mms_artec/nbv/live_scan_viewer.py               # SDK 정합행렬 누적 뷰어
utils/nbv/phase1_viewpoint.py                   # 밴드 분할·순서 계획
mms_artec/nbv/recovery_pose_selector.py         # Spider 광학 상수, 자세 후보
utils/turntable/turntable_interface.py          # 실물 턴테이블 (move_velocity/getActualPos, UDP)
mms_artec/backends/isaac/isaac_turntable.py     # sim 턴테이블 (RevoluteJoint)

sim_harness/MMS_ext_phase1.py                   # sim 검증 (GT 누적)
sim_harness/MMS_ext_phase1_recovery1.py         # sim recovery 로직 검증 — 빗나감
sim_harness/MMS_ext_phase1_recovery2.py         # sim recovery 로직 검증 — 윗면 미포착
scripts/sim/run_e2e_gui.sh                      # sim E2E 실행 진입점
scripts/sim/validate_phase1_viewpoint.py        # 밴드 계획 단독 검증
```

---

# 〔부록〕 Troubleshooting

### T1. 물체를 치웠는데도 tracking lost 가 안 뜬다

`registration` 이 ICP-only 로 설정된 경우다. 물체가 없어도 **빈 턴테이블 원판에 정합이
성공해 버려서** lost 판정이 나지 않는다. `set_registration_type(HYBRID)`(geometry+texture)를
반드시 쓴다. 관련 기록은 `project_artec_tracking_lost_limitation`.

### T2. 스캔 도중 턴테이블 통신이 멎는다

턴테이블은 **UDP** 로 통신해야 한다. TCP 로 붙이면 `getActualPos` 를 10Hz 로 계속 두드리는
sustained polling 구간에서 socket 이 막힌다. `utils/turntable/turntable_interface.py` 참조.

### T3. 밴드 하나가 실패했는데 스캔 전체가 끝나버린다 (수정됨)

`scan_phase_controller.py` 가 밴드 한 대역이 도달 불가면 즉시 finalize 해서 Phase 2·3 이
통째로 사라졌다. hand drill 267k점, spray can 472k점을 모아놓고 버린 사례가 있다(스캔
윗부분이 잘려 나감). 도달하지 못한 대역은 부분 결손일 뿐이고 그걸 메우는 것이 Phase 2 의
역할이므로, 지금은 **실패한 밴드는 건너뛰고 계속 진행하며 전부 실패했을 때만 포기**한다.

### T4. 충돌 없는 다른 방위각이 있는데도 "도달 불가" 가 뜬다 (수정됨)

자세 선정 경로가 밴드와 legacy 둘인데 **충돌 검사가 legacy 에만 있었다**
(`isaac_scan_session.py`). 밴드 경로는 IK 해가 나오면 충돌을 보지 않고 확정해 버려서,
충돌 없는 다른 az 를 시도조차 못 하고 이동 단계에서 거부됐다. 실측으로 az 0° 는
tool↔link4 가 **4mm** 차로 자가충돌이고 az 30° 는 통과였다. 수정 후 spray can 의
Phase 1 경계가 408mm → 227mm 로 줄었다.

> 이것은 "도달 불가 판정이 나오면 평가기를 먼저 의심하라"의 네 번째 사례다.
> **같은 판단을 하는 코드가 여러 곳에 있으면 게이트가 전부에 들어갔는지 grep 으로 확인한다.**

### T5. 〔미해결〕 이동이 거부돼도 그 자리에서 프리뷰를 찍는다

`isaac_scan_session.py` 의 `preview_at`(:988)과 `_pick_phase1_legacy`(:1080)가 `_drive()`
반환값을 무시한다. 이동이 거부돼도 원래 자리에서 캡처하므로 프리뷰가 오염될 수 있고,
그러면 밴드 계획 자체가 틀어진다. z_cover 경계 사례(0.67, 0.71)가 이 탓일 가능성이 있으나
검증되지 않았다.

### T6. 〔한계〕 view-score 는 θ 한 시점만 본다

현재 score 는 θ=0 의 preview 하나만 평가한다. 손잡이나 주둥이가 달린 비대칭 물체는 θ 마다
최적 φ 가 다를 수 있는데 360° 동안 로봇은 고정이므로, 한 시점 기준으로 고른 φ 가 다른
구간에서 나쁠 수 있다. 미구현 해결안은 세 가지다 — ① probe 로 얻은 물체점을 축 기준으로
회전 보정해 가상의 θ N개를 합산, ② 후보마다 짧게 회전시켜 평균, ③ top-K 를 뽑아 multi-pass.

### T7. 스캔 시작 직후 lost 로 오판된다

`reg_err = -1` 은 SDK 의 sentinel 이지 실패가 아니다. `reg_err ≥ 0` 을 한 번 본 뒤부터만
watchdog (3)(4)를 세도록 `tracking_established` 플래그가 걸려 있다. 이 플래그를 건드리면
warm-up 구간이 곧바로 lost 로 잡힌다.

### T8. 지켜야 할 규약 몇 가지

- **연속 회전 + 최대 FPS.** 겹침이 생명이므로 각도를 띄엄띄엄 옮기지 않는다.
- **시작 자세는 home 그대로.** 사전 probe 를 넣지 않는다. lost 가 났을 때만 recovery 로직이
  자세를 탐색한다.
- **라이브 뷰어 누적은 SDK 정합행렬만 쓴다.** θ·yaml·hand-eye 를 섞으면 뷰어가 SLAM 의
  거울이 아니게 되어 디버깅 가치가 사라진다.
- **sim 에는 SLAM 이 없다.** GT θ 와 축으로 누적할 뿐이고, 실물은 그 위에 SLAM 만 얹는다.
  sim 이 잘 된다고 실물 정합이 잘 된다는 뜻은 아니다.
- **last-good θ** 는 `reg_err ≥ 0` 인 마지막 θ 이며 recovery 로직 safe-back 의 유일한 기준이다.
- **턴테이블 정지는 stop_event 로 즉시** 이뤄져야 한다. 별도 thread 가 이를 감시한다.
