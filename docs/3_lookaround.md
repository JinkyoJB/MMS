# lookaround — 5면 스캐닝

> 턴테이블이 한 바퀴 도는 동안 로봇은 **한 자세에 고정**된 채 연속 스캔한다. 물체가
> 높으면 z 방향 띠(밴드)로 나눠 여러 번 돈다. 그렇게 윗면과 옆면 4개, 합쳐서 5면을
> 얻는다. 바닥면은 원판에 닿아 있어 flip(뒤집기)의 몫이다.
>
> 앞 단계 `2_preview.md`(형상 탐색) · 부족면 보강 `4_nbv.md`
>
> **§1(실행)·§2(구현 컨셉)면 쓰기에 충분하다.** §3 이후는 왜 그렇게 도는지다.
> 문제가 생기면 맨 뒤 부록(T1~T9)을 본다.

---

## 1. 실행

### sim

```bash
cd ~/workspace/sync/2_Rapid_Digital_Twin/1_MMS/MMS
env -u PYTHONPATH MMS_BACKEND=isaac MMS_SIM_STAGE_UNTIL=lookaround \
    ~/miniconda3/envs/env_isaacsim/bin/python -u main_artec.py --no-prompt
```

기본 씬이 `v2_real_260917.usd`(실물 셀 재현)라 인자가 없어도 실물 배치로 돈다.
v3 씬으로 A/B 하려면 `scripts/sim/run_e2e_gui.sh [물체] [stage_until]` — 그 러너는
**v3 전용**이라 실물 배치와 다르다.

### real

```bash
env -u PYTHONPATH MMS_BACKEND=real \
    ~/miniconda3/envs/mms-env/bin/python -u main_artec.py --until lookaround
```

`--until` 은 `preview | lookaround | nbv | flip` 이고 **앞 단계는 항상 포함**이다.
real 은 환경변수 override 를 안 받는다 — 실물 동작이 셸 환경에 좌우되면 안 된다.

### 손잡이

밴드·캡처에 관한 것만이다. 거리·실루엣 쪽 손잡이는 **`2_preview.md` §5**.

| 환경변수 | 뜻 | 기본 |
|---|---|---|
| `MMS_SIM_STAGE_UNTIL` | `preview`/`lookaround`/`nbv`/`flip` (sim 전용) | `nbv` |
| `MMS_BAND_OVERLAP` | 인접 밴드 겹침 — **밴드 밀도의 유일한 손잡이**(§5) | 0.61 |
| `MMS_STANDOFF_TRACK` | 밴드 캡처 중 거리추종 `off`/`band`/`live` (§2) | `band` |
| `MMS_SIM_P1_ELS` | 채점할 고도각 후보(°) | `30,40,50,60,70` |
| `MMS_SIM_NTHETA` | 한 바퀴 캡처 프레임 수 (실물 240 = 30s × 8fps) | 240 |
| `MMS_ISAAC_HEADLESS` | 1이면 GUI 없이 (결과 창도 안 뜬다) | 0 |
| `MMS_SIM_USD` / `_OBJECT_PRIM` | 씬·대상물 prim (→ `sim_scene.md`) | v2_real 씬 |

---

## 2. 구현 컨셉

### 한 문장

**"한 자세가 실제로 덮은 높이"를 재고, 그만큼씩 겹치게 물체를 가로로 썬다.**

### 자유도를 누가 갖는가

턴테이블이 도니까 **방위는 로봇이 고를 일이 아니다.** 로봇에게 남는 관측 자유도는
**고도각 `el` 하나**고, 거기에 조준높이 `tz` 와 축거리 `standoff` 가 붙는다.

| 축 | 누가 정하나 |
|---|---|
| `tz` | **밴드가 정한다.** 밴드 센터가 곧 조준높이. 캡처는 **z 단조** 순(§5) |
| `el` | **밴드마다 다시 고른다** — 그 밴드 안의 점으로 maximin 재채점(§4) |
| `standoff` | `el` 과 같이 고르고, 캡처 직전에 **실측으로 다시 잡는다**(아래) |
| `az` | **관측과 무관** — 도달성·충돌만 보고 고른다 |

> **`az` 를 헷갈리지 말 것.** az 를 바꿔도 **보는 면은 안 바뀐다** — 턴테이블이 한 바퀴
> 도니 같은 `el`·`tz` 면 결과가 같다. az 스윕은 "IK 가 풀리고 충돌 안 나는 방위를 찾는"
> 용도다(`solve_plan_poses`). 이걸 관측 자유도로 착각해 az 만 바꿔 전회전을 반복하던
> 버그가 nbv 에 있었다(`4_nbv.md`).

### 밴드 결정 — ②는 시작점, ④가 판정

```
① 단일 자세로 되나?
   후보(el × standoff × tz)를 전회전 maximin 으로 채점 → 최선 1개
   그 자세가 실제로 덮은 연속 z 구간 = seen_span
   seen_span/높이 ≥ 90%  그리고  z_cover ≥ 75%   →  단일로 끝

② 안 되면 — 몇 개로 썰까 (시작점)
   band_h   = seen_span                     ← 실측 그대로. 안전계수 안 곱한다
   step_max = band_h × (1 − BAND_OVERLAP)   ← 센터 간격 상한
   m_min    = 1 + ceil(센터span / step_max)

③ 각 밴드를 다시 푼다
   센터 m 개를 linspace → 밴드마다 el·standoff 재채점
   윗면(뚜껑) 보강 자세를 넣고, 캡처 순서를 z 단조로 정렬

④ 겹침을 **재서** 모자라면 늘린다               ← 진짜 판정
   인접 자세의 seen 교집합 < BAND_ADJ_OVERLAP_MIN(25%) → 밴드 +1 하고 ③ (최대 +3)

⑤ 이득이 없으면 되돌린다
   밴드 합집합 커버 − 단일+윗면 커버 < 3.5%p  →  단일 자세로 복귀
```

**공칭 기하 겹침은 "창이 얼마나 포개지나"지 "같은 면을 보나"가 아니다.** 곡률·입사각·
가림 때문에 실제 겹침은 늘 더 작게 나온다(실측: 공칭 38% → 측정 20%). 그래서 ②는
시작점일 뿐이고 ④가 실제 판정이다.

### 거리추종 — el 은 고정, 축거리만 창 중앙으로

계획이 정한 standoff 는 **preview 가 추정한 반경**에서 나온다. 그 추정은 대개 작고,
회전하면 반경이 θ 에 따라 변하며, 밴드마다 tz 가 달라 그 높이의 반경도 다르다. 그대로
두면 표면거리가 작동거리 창을 벗어난 채 한 바퀴를 다 돈다 — 실물에서 "가끔 물체와
거리가 멀다" 로 보이던 증상이다.

그래서 **밴드 시작(턴테이블 정지 중)에 표면거리를 재서 축거리만 고친다.**

```
표면거리 중앙값 p  →  국소 반경 r ≈ d − p  →  d_next = 창중앙 c + r = d + (c − p)
```

`el`·`az`·`tz` 는 건드리지 않는다 — el 을 바꾸면 그 밴드가 덮는 z 대역이 바뀌어 계획
전체가 흔들리지만, 축거리는 카메라를 시선 방향으로 밀고 당길 뿐이다. 1회 이동은
25mm 로 제한하고 정지 중에는 최대 3회 나눠 수렴시킨다(급하게 뛰면 프레임 중첩이
깨진다). 판단은 `utils/nbv/standoff.py::StandoffTracker` 한 곳, sim·real 공용.

실측(sim, 오차 +70mm 주입): 5밴드 전부 250mm 부근(251~260mm)으로 수렴.

### 대상물은 계획 전에 장애물로 등록된다

셀 CAD 에는 스캔 대상이 없다 — "225mm 까지 접근하는 대상" 이라 일부러 뺐는데, 그 결과
자세 사이 이동 경로가 물체를 관통해도 아무도 안 막았다(2026-09-16 실물: nbv 이동 중
로봇이 대상을 치고 지나감). 지금은 세 겹이다:

1. **preview 전** — 축 둘레에 원판 반경(150mm) 보수적 원기둥 (아직 물체를 모르므로)
2. **preview 후** — 실루엣 점군
3. **nbv 메시 후** — 실측 메시 정점 (더 정확)

2·3 은 **축 둘레로 쓸어 회전체**로 등록한다(`swept_about_axis`). 점군은 θ=0 canonical
인데 로봇이 움직이는 시점의 턴테이블 각은 0 이 아니라서, 비대칭 물체면 장애물이 실제와
다른 방향을 향하기 때문이다. 등록은 `solve_plan_poses` 의 충돌 게이트보다 **앞**이라
밴드 자세 선정부터 반영된다.

### sim 과 real

**판단 로직은 같은 코드다.** `collect_planning_points` → `plan_lookaround_viewpoints` →
`solve_plan_poses` 를 두 백엔드가 그대로 부르고, 주입하는 것은 콜백 세 개(preview 캡처 /
자세 IK / 충돌 게이트)뿐이다. **작업 프레임도 2026-09-17 부터 둘 다 base 다** — 예전엔
sim 만 world 라 `up_sign=+1` 경로만 돌았고, real 전용 분기(천장 마운트라 +Z 가 아래)가
sim 에서 한 번도 검증되지 않았다. 그게 "sim 에서는 되는데 real 에서만 안 되는" 의
구조적 원인이었다. sim 은 USD 를 읽고 씬에 그릴 때만 world 로 나간다.

다른 것은 **스캔 쪽**이다 — real 은 Artec SDK 의 streaming SLAM 으로 프레임을 정합하고
추적 상실 감시·복구가 붙는다(§7). sim 은 GT θ 로 누적한다(§8). 즉 **자세를 어디로 보낼지는
sim 이 담보하지만, 그 자세에서 스캔이 붙어 있느냐는 실물에서만 확인된다.**

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

탐색 격자는 고도각(`DEFAULT_ELS = 30/40/50/60/70°`) × standoff 3종
(`near+20mm+r_max`, `mid+r_max`, `mid+r_max+30mm`) × 타깃높이 3종(z 의 35/50/65 분위수)다.
`el=20°` 는 도달 자세가 없어 2026-09-17 에 기본값에서 뺐다 — 두 백엔드가 이미 안 쓰고
있었는데 기본값만 남아서 **검증 스크립트가 실제로 안 쓰는 el 로 검증**하고 있었다.
**방위각은 점수에 안 들어간다** — 회전 대칭이라 관측 조건을 못 바꾸고 도달성과 충돌만
좌우하므로, 백엔드가 스윕해 통과하는 값을 고른다(T4).

가시성 판정(`SensorModel`)은 frustum ∩ 작동거리(0.20~0.30m) ∩ 입사각인데, **입사각
한계가 둘**인 것이 요점이다. 누적·품질에는 50°까지만 쳐주고 추적 기여는 75°까지
인정한다 — 비스듬히 스치는 면은 모델엔 보탬이 안 돼도 추적은 붙잡아 준다.

물체 점은 클러스터링 없이 **캘리브 기하로 자른다**(`crop_object_points`: 원판 위 +
축 실린더 안). "턴테이블을 물체로 착각해 조준하는" 실패가 구조적으로 불가능해진다.

**점군은 GT 없이 실제 캡처로 모은다** — 그 단계가 preview 이고 내용은 **`2_preview.md`**
에 있다. 여기서 알아야 할 것은 하나다: **이 점군이 아래 밴드 산정의 입력 전부**라,
반경을 작게 잡으면 standoff 가 어긋나고 높이를 놓치면 밴드 수가 통째로 틀린다.

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

밴드 높이 `band_h` 는 자세가 실제로 덮은 **연속 z 구간 그대로**다 — 안전계수를 곱하지
않는다. 겹침은 `BAND_OVERLAP`(기본 0.61) 한 곳에서만 흡수한다. 밴드 수는
`m = 1 + ceil(센터span / (band_h·(1−overlap)))` — **ceil** 이어야 실제 센터 간격이
의도를 넘지 않아 겹침이 기준 아래로 안 떨어진다.

> ⚠ **0.61 은 측정해서 정한 값이 아니다.** 2026-09-17 정리 전에는 같은 일을 하는 계수가
> 둘이었다(`BAND_H_SAFETY 0.60` × `(1−0.35)` = 0.39 → 실효 겹침 61%). 코드에는 35% 라고
> 적혀 있는데 실제로는 61% 로 돌고 있었다. 둘을 곱해 하나로 합쳤을 뿐이라 **0.61 은
> 합병의 산물**이다.
>
> 요구 기준(`BAND_ADJ_OVERLAP_MIN`)은 25% 인데 공칭이 그 2.5배라, **④의 되먹임 루프가
> 발동할 일이 없다**(모자라면 늘리지만 남아돌아도 줄이지는 않는다). 낮춰야 할 근거가
> 있다 — testset 9종 GT 오프라인 A/B 에서 0.35 도 밴드간 최소겹침 최악 51%(요구의 2배)
> 였고 자세가 31→24개(15.5→12.0분)로 줄었다. 다만 상수 주석의 **실제 sim 실행** A/B 는
> 0.35 에서 세제가 단일 자세로 무너져 44,360점(vs 98,808점)이라 서로 충돌한다.
> **실제 sim 실행 A/B 가 있어야 판단할 수 있다 — 아직 안 했다.**

### 캡처 순서 = z 단조 (2026-09-17)

순서는 **z 오름/내림 한 방향**이다. 방향만 안전한 끝(min_fill 큰 쪽)에서 시작한다.

> ⚠ 예전에는 **safe-first**(min_fill 내림차순)였다. 그건 **밴드 = 독립 IScan** 이던
> 시절의 규칙이다 — 밴드가 끊겨도 각자 정합되니 위험한 밴드를 뒤로 미루는 게 이득이었다.
> 2026-09-16 에 밴드 전체가 **한 IScan** 이 되면서 전제가 사라졌다: 로봇이 다음 밴드로
> 옮겨가는 동안 SLAM 이 이어져야 하고, 그러려면 **캡처 순서상 인접한 두 자세가 같은 면을
> 봐야 한다.**
>
> 실측 2026-09-17 (testset 9종 실제 점군, 인접 자세 seen 겹침 최악값):
>
> | 물체 | safe-first | z 단조 |
> |---|---|---|
> | mustard | **0%** | 64% |
> | spray_can | **0%** | 58% |
> | laundry_detergent | **0%** | 20%* |
> | alarm_clock | 12% | 57% |
>
> \* 뚜껑 보강 자세와의 이음매. 밴드끼리는 60% 이상이다.
>
> 9종 중 4종이 0~12% 였다 — 실물에서 본 "band1/band2 가 정합되지 않음 · 메시 2덩어리"
> 가 이 순서 문제다.

겹침은 **공칭 기하값이 아니라 재서** 확인한다(`BAND_ADJ_OVERLAP_MIN = 0.25`). 곡률·입사각
때문에 실제 겹침은 늘 공칭보다 작다(실측: 공칭 38% → 측정 20%). 미달이면 밴드를 하나씩
늘려 다시 짠다(최대 `BAND_MAX_EXTRA = 3`).

반대로 **밴드가 이득이 없으면 단일 자세로 되돌린다**(`BAND_GAIN_MIN_COV = 0.035`).
z-span 미달의 원인이 높이가 아니라 **윗면**일 수 있는데(납작·넓은 물체), 그건 z 로 쪼개도
해결되지 않고 전회전 횟수만 늘어난다 — `_augment_top_face` 의 몫이다. 이득은 z-span 이
아니라 **덮은 점 비율**로 잰다(실측 alarm_clock: Δz-span 0mm 인데 Δ커버 +4.8%p).

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
    plan_lookaround_viewpoints()        # §4 maximin 채점 + §5 밴드 분할
    solve_plan_poses()                  # az 스윕 IK + 충돌 게이트 (채택 az 를 되돌려줌)
    make_view_pose(up_sign)             # 채점용 자세 — 실행(eye_from_el_az)과 같은 up
    collect_planning_points()           # preview 수집 루프 (→ 2_preview.md)
    crop_object_points / filter_robot_points
    swept_about_axis / guard_cylinder_points   # 대상물 장애물 (§2)
utils/nbv/standoff.py               ★ 거리 판단 전부
    StandoffTracker                 # §2 거리추종 (밴드 시작 보정)
    그 밖(거리격자·탐침·보정식)      # → 2_preview.md
utils/nbv/scan_stage_controller.py  # preview→lookaround→nbv→flip 순서 (밴드 실패 허용)
    STAGES · runs_stage · resolve_stage_until

mms_artec/nbv/artec_streaming_scan_session.py   ★ 실물 streaming SLAM
    TrackingState(watchdog 4종) · TurntableController · run()
mms_artec/nbv/artec_multipass_scan_session.py   # 오케스트레이션 + 복구 + view-score
mms_artec/nbv/live_scan_viewer.py               # SDK 정합행렬 미러
mms_artec/nbv/recovery_pose_selector.py         # 복구용 Spider 광학 상수
mms_artec/backends/isaac/isaac_turntable.py     # sim 턴테이블 (kinematic 직접 회전)
utils/turntable/turntable_interface.py          # 실물 턴테이블 (UDP)

scripts/sim/run_e2e_gui.sh · validate_lookaround.py   # v3 씬 A/B
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
2. 계획용 preview 가 **최대 48회 이동**한다(턴테이블 2 × 높이 4 × 거리 시도 6).
   보통은 8~9회에서 끝나지만(sim 실측 9회, 거리가 1~2회에 수렴), 반환이 계속 0인
   물체(검은색·투명·반사)는 상한까지 간다. 2026-09-17 에 **24→48** 로 늘었다 —
   반환 0 탐색이 한쪽이 아니라 양쪽으로 벌어지게 바뀌면서 시도 상한을 3→6 으로
   올렸기 때문이다(`2_preview.md` §3). 게이트는 통과하지만 첫 시도는 비상정지
   옆에서 볼 것. 급하면 `max_dist_tries` 를 낮춘다.
3. `T_BF0` 가 없으면 플래너가 포기하고 home 고정으로 간다 — 캘리브부터.

막히면 `lookaround_planner_enabled=False` 로 예전 동작(home 고정)으로 돌아갈 수 있다.

### T8. sim 물체가 회색이거나, 30초 만에 빈 결과로 끝난다
- **회색** — 텍스처 참조 결손. `UsdPreviewSurface` 는 텍스처가 안 열리면 말없이 회색으로
  렌더한다. 9종 중 8종은 복구했고 `0263_protein_drink` 만 남았다(`2_데이터/README.md`).
  lookaround~3 은 형상만 쓰므로 결과에는 영향이 없다.
- **빈 결과** — 구 씬(`v2.usd`, `composed/*_on_turntable.usd`)을 쓴 것이다. 카메라·
  턴테이블 prim 경로가 달라 스캔 없이 끝난다. 쓸 수 있는 씬 목록은 `sim_scene.md` §1.
- **모든 자세가 "이동 거부"** — 씬과 충돌 점군이 어긋났다. `sim_scene.md` §3.

### T9. 지켜야 할 규약
- **연속 회전 + 최대 FPS.** 겹침이 생명이라 각도를 띄엄띄엄 옮기지 않는다.
- **자세 선정은 sim·real 이 같은 코드.** 한쪽만 고치면 갈라진다 — 백엔드가 아니라
  `utils/nbv/lookaround.py` 를 고친다.
- **라이브 뷰어 누적은 SDK 정합행렬만.** θ·hand-eye 를 섞으면 디버깅 가치가 사라진다.
- **sim 은 SLAM 이 없다.** sim 이 잘 된다고 실물 정합이 잘 된다는 뜻이 아니다.
- **last-good θ** 는 복구 safe-back 의 유일한 기준이다.
- **밴드 캡처 순서는 z 단조.** 밴드가 한 IScan 이므로 인접 자세가 겹쳐야 SLAM 이 이어진다
  (§5). minfill 순서로 되돌리지 말 것.
- **자세를 고르는 곳과 검사하는 곳은 같은 게이트를 쓴다.** `find_home_pose.py` 가 충돌
  게이트 없이 home 을 고르는 바람에 손목이 디스크 27mm 위를 스치는 home 이 채택됐고,
  `is_path_safe` 의 start 검사에 걸려 **모든 이동이 거부**됐다(2026-09-17).
