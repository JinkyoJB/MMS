# lookaround — 5면 스캐닝

> 물체를 **높이 방향 띠(밴드)로 분할**하여, 밴드마다 로봇을 해당 높이의 자세로 이동시키고
> 턴테이블을 1회전시킨다. 밴드 전체가 **하나의 연속 스캔(IScan)** 이므로 로봇이 밴드 사이를
> 이동하는 동안에도 SLAM 이 유지되어야 한다. 그래서 밴드는 z 단조 순서로, 서로 겹치게 돈다.
>
> 이 과정으로 윗면과 옆면을 합한 5면을 취득한다. 바닥면은 원판에 접해 있어 flip 의 몫이다.
> 한 자세가 물체 높이를 충분히 덮는 경우(납작하거나 작은 물체)에는 밴드가 1개로 끝난다.
>
> 선행 `2_preview.md`(형상 탐색) · 후속 `4_nbv.md`(부족면 보강)
>
> **§0·§1 만으로 운용이 가능하다.** §2 이후는 설계 근거이며, 문제 발생 시 부록을 참조한다.
>
> **검증 상태 (2026-09-22)** — sim 에서 testset 9종 계획·스캔 확인. 실물은 밴드 1 회전이
> 안정적으로 유지되나(fail=0, regErr≈+0.25), **밴드 이음매에서는 추적 상실이 잔존**한다(§7).
> `ignore_registration_errors` 는 **True 를 유지**해야 한다(T4).

---

## 0. 결정 요약

### 목적함수

후보(고도각 × standoff × 조준높이)마다 턴테이블 1회전을 36등분하여 사전 평가하고,
점수가 가장 높은 하나를 채택한다.

```
score = 2.0 · min(min_fill / 12cm², 1)   ← 최악 프레임 가시면적 (지배항)
      + 1.0 · z_cover_frac                ← 높이 방향 커버율
      + 0.5 · covered_frac                ← 품질 기준 가시 비율
      + 0.5 · side_cover_frac             ← 옆면(법선 수평±30°) 커버율
      − 0.8 · (standoff 초과분 / 100mm)   ← 근접 후보 선호

min_fill = min over θ ( 해당 시점 가시면적 )   ← 평균이 아니라 최솟값
```

**최솟값(maximin)을 사용하는 것이 핵심이다.** Artec 은 직전 프레임에 정합하므로 한 프레임만
놓쳐도 이후가 전부 유실된다. 평균 가시성은 의미가 없다.

### 판정 기준

| 판정 대상 | 기준 | 임계 | 설정키 |
|---|---|---|---|
| 단일 자세 종료 | **두 조건 모두** 충족 시에만 단일 | `z_cover ≥ 0.75` AND `zspan/h ≥ 0.90` | `ZCOVER_MIN` / `ZSPAN_MIN_FRAC` |
| 초기 밴드 수 | `m = 1 + ⌈센터span / (band_h·(1−ov))⌉`. `band_h` 는 해당 자세가 **실측으로 덮은** 연속 z 폭 | `ov = 0.40` | `BAND_OVERLAP` |
| 밴드 추가 분할 | 공칭이 아닌 **측정된** 인접 겹침 | 최소 겹침 < 25% → +1 (최대 +3) | `BAND_ADJ_OVERLAP_MIN` / `BAND_MAX_EXTRA` |
| 단일로 복귀 | 분할로 **추가 확보한 점 비율** | 이득 < 3.5%p → 복귀 | `BAND_GAIN_MIN_COV` |
| 옆면 예외 | 면적 이득이 미달이어도 옆면이 부족하면 밴드 유지 | 옆면 < 75% AND 이득 ≥ 4%p | `SIDE_COV_MIN` / `SIDE_GAIN_MIN` |
| 윗면 보강 | 윗면 커버율 부족, 또는 위쪽 법선 점이 다수 | 커버 < 50% · el ≥ 50° · 강제 5% | `CAP_COVER_MIN` / `CAP_EL_MIN_DEG` / `CAP_FORCE_FRAC` |
| 캡처 순서 | **z 단조** 한 방향. 시작 방향만 안전한 끝(min_fill 큰 쪽) | — | `_band_capture_order` |
| 자세 실행 여부 | `min_fill` 이 배제선 미만이면 가시면이 없어 추적 상실 | ≤ 1cm² 제외 · < 6cm² 경고 | `FILL_HARD_MIN_CM2` / `FILL_MIN_CM2` |
| 방위각(az) | **최적화 대상이 아니다.** 회전은 턴테이블이 담당하므로 IK·충돌을 처음 통과하는 값을 사용 | 후보 `0°, +30°, −30°` | `solve_plan_poses` |
| 캡처 중 거리 보정 | 밴드 핵심 높이대의 표면거리가 작동거리 창 중앙에서 벗어나면 **축거리만** 조정 | 1회 ≤ 25mm · 밴드 시작 ≤3회 + OK 프레임 8개마다 | `StandoffTracker` |

### 원칙 두 가지

1. **공칭은 시작점이고 측정이 판정이다.** 밴드 수는 기하식으로 시작하되 실제 겹침을
   측정하여 증감한다.
2. **로봇이 선택하는 관측 자유도는 고도각 하나다.** 방위는 턴테이블이 담당하므로 점수에
   반영되지 않으며, 축거리는 작동거리 창이 결정한다.

---

## 1. 실행

```bash
# sim
env -u PYTHONPATH MMS_BACKEND=isaac MMS_SIM_STAGE_UNTIL=lookaround \
    ~/miniconda3/envs/env_isaacsim/bin/python -u main_artec.py --no-prompt

# real
env -u PYTHONPATH MMS_BACKEND=real \
    ~/miniconda3/envs/mms-env/bin/python -u main_artec.py --until lookaround
```

`--until` 은 `preview | lookaround | nbv | flip` 이며 **선행 단계는 항상 포함**된다.

| 플래그 | 용도 |
|---|---|
| `--no-recovery` | 추적 상실 자동복구 해제. 원 스캔의 정합 여부만 확인할 때 |
| `--no-planner` | 자세 플래너 해제(home 고정). 캡처 루프만 분리 확인할 때 |
| `--max-passes 1` | 한 자세만 실행 |
| `--no-prompt` / `--no-viewer` | pass 간 확인 생략 / 라이브 뷰어 해제 |
| `--speed-scale K` | 로봇 속도 배율(계획·nbv·복구 일괄) |
| `--no-sproj` / `--texturize` | sproj 저장 생략(각 ~47초) / SDK 텍스처링(15~20분) |

### 산출물

한 run 의 결과는 `output/<RUN_TS>/` 한 폴더에 모인다(규칙: `utils/run_paths.py`).

| 경로 | 내용 |
|---|---|
| `run.log` · `events.jsonl` | 콘솔 전체 · 판단 이벤트(§7) |
| `scan_dumps/scanNN_<stage>_poseK.npz` | IScan 별 원본 점군 (오프라인 정합용) |
| `aligned/aligned.sproj` | SDK 후처리 전 IScan — **Artec Studio 로 여는 파일** |
| `debug/lookaround/s{NN}_band{b}_*.png` | 거리추종 판정 시점의 거리 이미지 + 텍스처 |

거리 이미지는 3D 점을 광축 기준 각도좌표로 전개하고 카메라 거리로 착색한 것으로,
preview·lookaround·nbv·flip 네 단계가 동일 형식이다. run 중에는
`live_range_view.py` 가 자동 실행되어 창으로 표시한다(`--no-range-view` 로 해제).
`SPACE` 정지, `←`/`→` 로 전후 비교, `q` 종료.

```bash
python scripts/artec/lost_report.py                      # 추적 상실·복구 이벤트 집계
python scripts/artec/live_range_view.py --run <RUN_TS>   # 수동 실행
python main_artec.py --range-video                       # 녹화 → debug/range.avi
```

> ⚠ 뷰어 창은 run 마다 생성되며 자동으로 닫히지 않는다. 종료된 run 의 창을 현재 run 으로
> 오인하기 쉬우므로 **창 제목의 RUN_TS** 를 확인한다. 새 이미지가 45초 이상 없으면 배너가
> `FINISHED` 로 전환된다.

### 설정 항목

밴드·캡처 관련 항목이다. 거리·실루엣 관련은 `2_preview.md` §5 를 참조한다.

| 환경변수 | 내용 | 기본값 |
|---|---|---|
| `MMS_SIM_STAGE_UNTIL` | 실행 종료 단계 (sim 전용) | `nbv` |
| `MMS_BAND_OVERLAP` | 인접 밴드 겹침 — **밴드 밀도의 유일한 조정 항목** | 0.40 |
| `MMS_STANDOFF_TRACK` | 캡처 중 거리추종 `off`/`band`/`live` | `live` |
| `MMS_SIM_P1_ELS` | 평가 대상 고도각 후보(°) | `20,25,30,40,50,60,70` |
| `MMS_SIM_NTHETA` | 1회전 캡처 프레임 수 (실물 240 = 30s × 8fps) | 240 |
| `MMS_ISAAC_HEADLESS` | 1이면 GUI 없이 실행 | 0 |

---

## 2. 구현 개요

**한 자세가 실제로 덮은 높이를 측정하고, 그만큼씩 겹치도록 물체를 수평 분할한다.**

### 관측 자유도의 배분

턴테이블이 회전하므로 방위는 로봇의 선택 대상이 아니다. 로봇에 남는 관측 자유도는
고도각 `el` 하나이며, 여기에 조준높이 `tz` 와 축거리 `standoff` 가 부속된다.

![관측 자세 파라미터 — tz · el · standoff · az](figures/lookaround/fig0_pose_params.svg)

네 값 모두 **턴테이블 축**을 기준으로 정의한다(물체 중심이 아니다). 물체가 축에서 벗어나
놓여도 회전 중 시야와 작동거리가 유지되도록 하기 위함이다.

| 축 | 결정 주체 |
|---|---|
| `tz` | 밴드가 결정한다. 밴드 센터가 곧 조준높이이며, 캡처는 z 단조 순이다 |
| `el` | 밴드마다 재선정한다. 해당 밴드의 점으로 maximin 재평가(§4) |
| `standoff` | `el` 과 함께 선정하고, 캡처 직전 실측으로 재조정한다 |
| `az` | 관측과 무관하다. 도달성·충돌만 고려하여 선정한다 |

> **az 를 관측 자유도로 오인하지 않는다.** 턴테이블이 1회전하므로 동일한 `el`·`tz` 에서는
> az 를 바꿔도 결과가 같다. az 스윕은 IK 가 풀리고 충돌하지 않는 방위를 탐색하는 용도다.

### 밴드 결정 절차

```
① 단일 자세 가능 여부
   후보(el × standoff × tz)를 전회전 maximin 으로 평가 → 최선 1개
   zspan/높이 ≥ 90%  AND  z_cover ≥ 75%  →  단일 종료

② 미충족 시 초기 분할 수 산정
   band_h   = 실측 seen_span            ← 안전계수를 곱하지 않는다
   step_max = band_h × (1 − BAND_OVERLAP)
   m_min    = 1 + ceil(센터span / step_max)

③ 밴드별 재계산
   센터를 등간격 배치 → 밴드마다 el·standoff 재평가
   윗면 보강 자세 추가, 캡처 순서를 z 단조로 정렬

④ 겹침 측정 후 증설                      ← 실질 판정
   인접 자세 seen 교집합 < 25% → 밴드 +1 후 ③ 반복 (최대 +3)

⑤ 이득 부족 시 복귀
   밴드 합집합 커버 − 단일+윗면 커버 < 3.5%p  →  단일 자세로 복귀
```

공칭 기하 겹침은 "시야가 포개지는 정도"이지 "동일 면을 관측하는가"가 아니다. 곡률·입사각·
가림으로 인해 실제 겹침은 항상 더 작다(실측: 공칭 38% → 측정 20%). ②는 시작점이며 ④가
실질 판정이다.

### 거리추종 — 축거리만 조정한다

계획 단계의 standoff 는 preview 추정 반경에 기반하므로 실제보다 작게 나오는 경향이 있고,
회전에 따라 반경이 변하며 밴드마다 해당 높이의 반경도 다르다. 보정하지 않으면 표면거리가
작동거리 창을 벗어난 상태로 1회전을 수행하게 된다.

```
표면거리 중앙값 p  →  국소 반경 r ≈ d − p  →  d_next = 창중앙 c + r = d + (c − p)
```

측정에는 **밴드 핵심 높이대**(조준 ±40mm / 광축 세로 ±9.5°)의 점만 사용한다. 전체 중앙값은
상부에 편향되어 카메라가 후퇴한다. `el`·`az`·`tz` 는 조정하지 않는다 — el 변경은 해당 밴드의
z 대역을 바꾸어 계획 전체에 영향을 주지만, 축거리는 시선 방향 전후 이동에 그친다.
1회 이동은 25mm 로 제한한다. 판단은 `utils/nbv/standoff.py::StandoffTracker` 단일 지점에서
수행하며 sim·real 공용이다.

보호 조건은 다섯 가지다.

| 조건 | 내용 |
|---|---|
| 프레임 신뢰 조건 | `tracking_lost` 가 아니고 연속 reg<0 가 5 미만. 정합 성공 여부를 조건으로 걸면 SDK 워밍업 구간에서 추종이 비활성화된다 |
| 이동-측정 일관성 | 15mm 이상 이동했는데 측정이 60% 미만으로 따라오면 해당 밴드 추종 중단(`standoff_skip`) |
| 무측정 시 정지 | 측정이 없으면 이동하지 않는다 |
| 희소 프레임 배제 | `TRACK_MIN_PTS` 300 미만은 측정으로 인정하지 않는다. 정상 프레임은 수천 점이다 |
| 포화 가드 | 측정이 창 가장자리 45mm 이내이면 일관성 판정을 보류하고 접근을 계속한다 |

> ⚠ 스캐너 프레임의 광축은 **−z** 다. 핵심 높이대 마스크에 `max(z, 0)` 을 사용하면 실물에서
> 항상 빈 집합이 된다. `|z|` 를 사용한다.

> ⚠ **단일 자세 계획도 밴드 경로를 경유한다.** 자세가 1개일 때 별도 경로로 분기하면
> 거리추종·되돌아가기·IScan 이어붙이기가 누락된다. 정상 동작 시
> `[stage] → 밴드 1개를 한 scan 으로 …` 가 출력된다.

### 대상물의 장애물 등록

셀 CAD 에는 스캔 대상이 포함되지 않으므로, 별도 등록이 없으면 자세 간 이동 경로가 물체를
관통해도 차단되지 않는다. 등록은 3단계로 이루어진다.

1. **preview 전** — 축 둘레에 원판 반경(150mm) 원기둥. 물체 정보가 없는 시점이므로
   "물체는 원판보다 넓을 수 없다"를 전제한다. **마진은 0** 이다. 원기둥 자체가 최대
   가정이므로 마진을 추가하면 이중 보정이 되고, 실효 keepout 이 커지면 preview 의 근거리
   탐침 구간을 잠식한다. `guard_blocks_probe` 가 차단 구간을 사전 계산하여 경고한다.
2. **preview 후** — 실루엣 점군
3. **nbv 메시 후** — 실측 메시 정점

2·3 은 축 둘레로 쓸어 회전체로 등록한다(`swept_about_axis`). 점군은 θ=0 기준인 반면 로봇
이동 시점의 턴테이블 각은 0 이 아니므로, 비대칭 물체에서 장애물 방향이 어긋나기 때문이다.
등록은 `solve_plan_poses` 의 충돌 게이트보다 선행하므로 자세 선정부터 반영된다.

### sim 과 real

**판단 로직은 동일 코드다.** `collect_planning_points` → `plan_lookaround_viewpoints` →
`solve_plan_poses` 를 양 백엔드가 공유하며, 주입되는 것은 콜백 3종(preview 캡처 / IK /
충돌 게이트)뿐이다. **작업 프레임도 양쪽 모두 base 다.**

차이는 스캔 방식이다. real 은 Artec SDK 의 streaming SLAM 으로 프레임을 정합하며 추적
상실 감시·복구가 부가된다(§7). sim 은 GT θ 로 누적한다(§8). 즉 **자세 선정의 타당성은
sim 이 담보하나, 해당 자세에서 스캔이 유지되는지는 실물에서만 확인된다.**

---

## 3. 설계 근거

Artec Spider 는 프레임 좌표를 직전 프레임에 정합하여 산출한다(streaming SLAM). 기준이
상대적이므로 한 프레임을 놓치면 이후를 연결할 근거가 소실되며, **추적 상실 시점까지
누적된 결과만 남는다.**

따라서 성패를 좌우하는 것은 hand-eye 나 θ 의 정확도가 아니라 **프레임 간 겹침**이다.
각도를 이산적으로 이동시키면 겹침이 끊기므로, 턴테이블을 저속 연속 회전시키면서 최대
FPS 로 촬영한다. 회전을 정지하고 자세를 옮기는 방식은 이 단계에서 사용하지 않는다
(nbv 의 정지-촬영은 IScan 을 별도로 생성하므로 해당하지 않는다).

이 성질로부터 나머지가 귀결된다. 방위는 턴테이블이 담당하므로 로봇의 자유도는 고도각
하나이며(§4), 한 자세로 높이를 덮지 못하면 z 방향 밴드로 분할하고(§5), 추적 감시와
복구 지점이 필수가 된다(§7).

---

## 4. 자세 선정 — 전회전 maximin 평가

`utils/nbv/lookaround.py::plan_lookaround_viewpoints` (sim·real 공용).

한 밴드 안에서 로봇은 고정되어 있으므로, 자세 선정은 **해당 밴드의 360° 전체를 자세
하나로 감당할 수 있는가**를 판정하는 작업이다. 후보마다 θ 를 36등분하여 1회전을
사전 시뮬레이션하고 **평균이 아닌 최악 프레임으로** 평가한다. 납작한 물체가 edge-on 으로
지나가는 순간에 추적이 끊기면 이후가 전부 유실되기 때문이다.

탐색 격자는 고도각(`DEFAULT_ELS = 20/25/30/40/50/60/70°`) × standoff 3종
(`near+20mm+r_max`, `mid+r_max`, `mid+r_max+30mm`) × 타깃높이 3종(z 의 35/50/65 분위수)이다.
도달 불가능한 el 은 `solve_plan_poses` 의 IK·충돌 검사가 배제하고 로그에 기록한다.

가시성 판정(`SensorModel`)은 frustum ∩ 작동거리 창 ∩ 입사각이다. **작동거리 창은 스캐너에
조회한다**(`scanning_range()`, 양 백엔드 공통 계약). 이 창이 밴드 수와 standoff 를 동시에
좌우하므로 `2_preview.md` §3 과 동일한 값을 참조한다.

**입사각 한계는 두 가지다.** 누적·품질에는 50° 까지, 추적 기여는 75° 까지 인정한다.
비스듬히 스치는 면은 모델에는 기여하지 않아도 추적은 유지시킨다. 또한 플래너의 품질 판정은
캡처 필터보다 **5° 엄격한 45°** 를 적용한다 — 동일 값을 쓰면 모델이 "간신히 덮는다"로
계산한 면이 실제 캡처에서 경계에 걸려 전부 배제된다.

물체 점은 클러스터링 없이 **캘리브 기하로 절단한다**(`crop_object_points`: 원판 위 + 축
실린더 내부). 턴테이블을 물체로 오인하는 실패가 구조적으로 배제된다. 캡처한 점에서 로봇
자체의 점을 먼저 제거한다(`filter_robot_points`) — 링크가 프레임에 포함되면 크롭 실린더가
오염되어 밴드 수가 과다 산출된다.

**계획 점군은 preview 가 실제 캡처로 수집한다**(`2_preview.md`). 이 점군이 밴드 산정의
입력 전부이므로, 반경을 과소 추정하면 standoff 가 어긋나고 높이를 놓치면 밴드 수가
전부 틀어진다.

### 옆면 커버리지

고도각 후보가 30° 이상뿐이면 모두 내려다보는 자세가 되어 눕힌 물체의 **옆면**(법선이
수평)이 구조적으로 취득되지 않는다. 이 상태에서는 lookaround 와 flip 이 공유하는 면이
없어 **flip 정합 자체가 성립하지 않는다**(후처리 문제가 아니라 취득 문제다).

세 가지로 대응한다.

1. `DEFAULT_ELS` 에 **20·25°** 를 추가한다. 도달 불가 여부는 IK·충돌 검사가 판단한다.
2. 자세 평가에 **옆면 커버 항**(`SIDE_SCORE_W=0.5`)을 추가한다. 전체 커버만으로는
   15~30° 가 동점이 되어 추적 여유가 큰 30° 가 선택된다.
3. 밴드 유지 판정에 **옆면 조건**을 추가한다(§0 판정표).

로그에 `[lookaround] 옆면(법선 수평±30°) 커버 — 밴드 …% vs 단일+윗면 …%` 가 출력된다.

⚠ 낮은 el 의 **실물 도달성은 미검증**이다. el 20·25° 채택 여부와 `도달 자세 없음` 발생을
확인할 것.

### standoff 근접 선호

standoff 후보 중 원거리 안이 선택되면 표면이 작동거리 창의 far 끝에 놓여 3D 점이 급감하고,
측정이 포화되어 거리추종이 중단된다. 따라서 `STANDOFF_NEAR_W=0.8` 로 100mm 멀어질 때마다
벌점을 부여한다(옆면 항 최대 0.5 를 상쇄). 위험이 비대칭이기 때문이다 — 가까우면 추종이
밀어낼 수 있으나, 멀면 점이 소실되어 되돌릴 근거가 없다.

⚠ 오프라인 재생으로는 검증되지 않았다. run 로그의 `자세 el=… s=…` 에서 s 가 후보 중
최솟값인지 확인할 것.

---

## 5. 밴드 분할

![밴드 분할 계획](figures/lookaround/fig4_bands.png)

> 그림은 계획기를 실제로 실행하여 생성한다. 코드 변경 시 재생성할 것:
> `env -u PYTHONPATH $MMS_PYTHON scripts/sim/plot_band_plan.py --obj laundry_detergent`
>
> 번호는 캡처 순서, `tz` 는 조준높이, 삼각형은 카메라 위치다.

판정 기준은 물체 높이가 아니라 §4 에서 선정된 자세가 **실제로 덮은 양**이며, 두 조건을
모두 충족해야 단일 자세로 종료한다.

```
z_cover  ≥ ZCOVER_MIN     0.75     ← 도달한 z-bin 의 비율
zspan/h  ≥ ZSPAN_MIN_FRAC 0.90     ← 실제로 덮은 연속 z 구간 / 물체 높이
```

둘째 조건이 없으면 상하만 걸치고 중앙이 비는 자세가 통과한다. `0101_spray_can` 이 해당
사례로, `z_cover=0.75` 로 첫 조건은 통과하나 `zspan=153/202=0.76` 으로 둘째에서 분할로
전환된다. 첫 조건만 적용하면 이 물체를 단일 자세로 종료하여 절반만 스캔하게 된다.

판정 결과는 `[lookaround] z_cover=… zspan=…` 한 줄에 함께 기록된다.

밴드 수를 좌우하는 것은 `band_h`(자세가 덮은 연속 z 구간)와 물체 높이의 비다. `band_h` 는
standoff 에 비례하고 standoff 는 물체 반경에 비례하므로 가는 물체일수록 한 자세가 덮는
높이가 감소하나, 실측상 그 효과는 크지 않고 **밴드 수는 사실상 높이가 결정한다.**

| 물체 | 높이 | 지름 | standoff | `band_h` | 결과 |
|---|---|---|---|---|---|
| `0154_laundry_detergent` | 286mm | 194mm | 317mm | 156mm | 5밴드 + 윗면 1 |
| `0101_spray_can` | 202mm | 69mm | 285mm | 153mm | 4밴드 |
| `0001_mustard` | 186mm | 98mm | 299mm | 145mm | 4밴드 |

`band_h` 에는 안전계수를 곱하지 않는다. 겹침은 `BAND_OVERLAP`(0.40) 한 곳에서만 흡수한다.
밴드 수 산정에 **ceil** 을 사용해야 실제 센터 간격이 의도를 초과하지 않는다.

### 물체 높이 산정

preview 점군의 분위수·최댓값으로 상단을 정하면 성긴 잡음에 편향된다(실측 40mm 과대).
이는 밴드 계획(허공을 도는 상단 밴드)·flip 피벗·nbv 부족분을 동시에 왜곡한다.

현재는 `robust_top_height` 단일 함수로 산정한다 — 디스크면부터 5mm 구간 히스토그램을
상단에서 내려오며 **처음으로 전체의 2% 이상을 포함하는 구간의 상단 경계**. 플래너는 그
위의 점을 계획 점군에서 제외하며, 윗면 자세·flip 피벗·nbv 부족분이 같은 값을 참조한다.
얇은 돌출부(노즐)는 밴드 계획에서 제외될 수 있으며 nbv 의 몫이다.

### 윗면 보강 자세

상단부의 위쪽 법선 점 수가 문턱 미만이면 윗면이 없는 물체(구·원뿔)로 판단하여 보강 자세를
생략한다. 문턱은 점군 밀도에 비례시킨다(`max(15, 0.2% of N)`) — 절대값을 쓰면 한두 점
차이로 보강이 누락된다.

선정 시 입사각을 감안하여 점수가 근소하게 낮아도 가파른 el 을 택한다(`CAP_SCORE_MARGIN`
0.15). 평가에 사용되는 cap 점은 preview 가 측면에서 관측한 테두리뿐이고 윗면 중앙은
점이 없어 평가에 반영되지 않기 때문이다. 수평면의 입사각은 90°−el 이므로 el 차이에 따른
품질 격차가 크다.

넓은 수평 윗면(위쪽 법선 점이 전체의 5% 초과)이면 커버율과 무관하게 높은 el 자세를
강제 추가한다(`CAP_FORCE_FRAC`).

⚠ **preview 는 el=30° 측면에서만 관측**하므로 윗면 커버율은 항상 테두리 기준의
과대평가다. 근본 해결은 preview 탐침에 높은 el 을 추가하는 것이다.

### 캡처 순서 — z 단조

순서는 z 오름 또는 내림 한 방향이며, 시작 방향만 안전한 끝(min_fill 큰 쪽)에서 정한다.

밴드가 **독립 IScan** 이던 시기에는 safe-first(min_fill 내림차순)가 유리했으나, 밴드 전체가
**하나의 IScan** 이 되면서 전제가 변경되었다. 로봇이 다음 밴드로 이동하는 동안 SLAM 이
유지되려면 **캡처 순서상 인접한 두 자세가 동일 면을 관측해야 한다.** 실측에서 safe-first 는
testset 9종 중 4종의 인접 겹침이 0~12% 였다(z 단조는 20~64%).

겹침은 공칭이 아니라 측정하여 확인하며(`BAND_ADJ_OVERLAP_MIN = 0.25`), 미달 시 밴드를
증설한다(최대 `BAND_MAX_EXTRA = 3`). 반대로 이득이 없으면 단일 자세로 복귀한다
(`BAND_GAIN_MIN_COV = 0.035`). z-span 미달의 원인이 높이가 아니라 윗면인 경우
(납작하고 넓은 물체) z 분할로는 해결되지 않고 전회전 횟수만 증가하기 때문이다. 이득은
z-span 이 아니라 **덮은 점 비율**로 측정한다.

밴드 하나가 실패해도 중단하지 않고 건너뛴다. 미도달 대역은 nbv 가 보완한다.

---

## 6. 실물 아키텍처

`mms_artec/nbv/artec_streaming_scan_session.py::ArtecStreamingScanSession.run()` 이
스캐너와 턴테이블을 두 thread 로 구동한다. 양자는 `TrackingState` 를 공유하며, 추적
상실 시 `stop_event` 로 턴테이블을 즉시 정지시킨다.

```
   Main thread                        TurntableController (thread)
   poll_events() ─ SDK 큐 처리         move_velocity 연속 회전
   θ 기록 · 라이브 뷰어 공급           getActualPos polling 10Hz
   watchdog 4종 검사                   stop_event → 즉시 정지
   last-good θ 갱신                    통신 두절 watchdog
            └────── TrackingState (공유) ──────┘
```

시작 시 턴테이블 logical 0 을 초기화하고, preview 를 구동하여 settle·drain 한 뒤
`start_record` 와 동시에 회전을 시작한다. 매 tick 마다 `poll_events()` 를 호출해야 하며,
**이를 누락하면 SDK 가 정지한다.**

결과 `ArtecStreamingScanResult` 는 `model`, `n_frames`, `rotation_actual_deg`,
`fps_actual`, `tracking_lost`, 복구 기준 `last_good_theta_rad`, 밴드 진행
`n_bands`/`n_bands_done`/`band_reasons`, 추적 상실 시 절단할 `n_tail_lost` 를 포함한다.

---

## 7. 추적 감시와 자동 복구

| # | 조건 | 설정키 | 기본값 |
|---|---|---|---|
| 1 | 정합/재구성 실패 연속 | `consecutive_loss_threshold` | 8 |
| 2 | callback 무응답 | `stale_threshold_s` | 2.0 |
| 3 | `reg_err < 0` 연속 (Studio 의 tracking lost) | `consecutive_reg_err_threshold` | 5 |
| 4 | `reg_err > max` 연속 | `consecutive_high_err_threshold` / `max_acceptable_reg_error` | 8 / 1.5 |

`reg_err = -1` 은 SDK sentinel 이므로 `reg_err ≥ 0` 을 1회 관측한 후부터 (3)(4)를 계수한다
(`tracking_established`). 이 플래그를 변경하면 워밍업 구간이 즉시 상실로 판정된다.
`ignore_registration_errors=True` 에서는 (1)이 사실상 발생하지 않으며 실제 상실은 (3)이
검출한다.

### 복구 경로

**1) 밴드 계획이 있는 경우(기본).** probe 기반 복구를 사용하지 않는다. 계획된 밴드 자세는
preview 로 검증된 자세이므로, 상실 시 **해당 자세로 복귀하여 새 IScan 으로 잔여 밴드를
이어서 촬영한다**(`capture_bands`). 완주한 밴드는 보존하고, 잔여 밴드의 첫 자세로 이동하여
`T_BC` 재캡처 → 기구학 보정으로 master 프레임에 초기 배치 → 인접 밴드 겹침으로 ICP
정밀화. 동일 자세에서 2회 연속 0 밴드이면 잔여는 nbv 의 몫으로 넘긴다(`band_partial`).

probe 복구를 생략하는 이유는 probe 가 물체 크기를 오판하여 빈 시야 자세로 이동시키는
사례가 확인되었기 때문이다.

**2) 계획이 없는 경우(legacy, `--no-planner`).** 동일 자세에서 최대 3회 재시도하고,
`last_good_theta_rad` 보다 10° 되돌린 뒤 자세를 재선정한다. 이때의 평가기는 §4 의 maximin
과 다르다 — 속도를 위해 1회전 대신 preview 한 장만 사용한다(한계는 T5).

### 밴드 전환

밴드 전환 직후의 추적 상실이 반복 관측되었다. SDK 스트리밍은 한번 상실하면 마지막
키프레임과 겹치는 시야로 복귀하기 전에는 재정합하지 않는다.

현재는 `(el, az, tz, 축거리)` 를 보간하여 **5mm 스텝**마다 IK 를 재계산하고(`_band_path`),
스텝마다 `regErr` 를 기록한다(`_band_transition`). 스텝 후 연속 reg<0 ≥ 3 이면 정지하고,
복구될 때까지 **경유 스텝을 역행**한 뒤 스텝을 절반으로 줄여 재전진한다(최대 2회).
재전진이 60스텝을 초과하거나 출발점까지 역행해야 하면 해당 IScan 을 포기하고 잔여 밴드는
새 IScan 으로 처리한다(`BAND_RELOC_MAX_STEPS` / `BAND_RELOC_MIN_ALPHA`).

**전환의 출발점은 계획 자세가 아니라 거리추종 후 실제 자세다.** 계획 자세에서 출발하도록
구성하면 첫 스텝이 수십 mm 도약이 되어 즉시 상실된다. 출발 축거리는 직전 밴드의 추종값,
목표는 다음 밴드 계획값에 동일 보정을 적용한 값(±120mm 클램프)이며, 다음 밴드의 트래커는
그 보정값에서 시작한다(`d0_override`).

완주한 밴드마다 (축거리 d, 표면거리 p) 를 기록하여(`band_standoff_m`), `r_eff = d − p` 의
중앙값을 nbv 축거리에 사용한다. preview 의 p95 반경은 과대 추정되는 경향이 있다.

### 이벤트 로그

판단 지점마다 `output/<RUN_TS>/events.jsonl` 에 기록한다(`utils/nbv/event_log.py`):
`lost`, `band_move`, `band_partial`, `recovery`, `rotation_ok`, `merge`.
집계는 `scripts/artec/lost_report.py`.

---

## 8. 라이브 뷰어와 sim 검증

**라이브 뷰어**(real 전용)는 누적에 **SDK 정합행렬만** 사용한다. θ·yaml·hand-eye 가
개입하지 않으므로 화면이 곧 SLAM 결과다. 필터도 SDK 기준(`reg_err ≥ 0`)과 동일하게만
적용한다. 자체 기준을 추가하면 검증 도구로서의 의미가 소멸한다.

터미널을 둘로 나누어 사용한다. 이 PC 에서 라이브 점군이 정상 렌더된 유일한 구성이
사용자가 직접 실행한 Filament 프로세스였기 때문이다.

```powershell
python main_artec.py --until lookaround        # 터미널 A — 스캔
python scripts\artec\live_scan_view.py         # 터미널 B — 뷰어
```

> ⚠ `mms_artec/nbv/live_scan_viewer.py` 는 뷰어가 아니라 스냅샷을 기록하는 파이프라인
> 모듈이다. 실행 대상은 `scripts/artec/live_scan_view.py` 다.

**sim 에는 SLAM 이 없다.** GT θ 와 회전축으로 누적한다.

```
p_obj = R(axis_dir, −θ) · (p_base − axis_point) + axis_point
```

`Rz` 가 아니라 축 방향 `axis_dir` 둘레의 일반 회전이다. 실측 `T_B_F0` 의 축이 base z 에서
기울어 있을 수 있어 `Rz` 로 고정하면 그만큼 누적이 어긋난다.

Isaac 없이 밴드 계획만 확인하려면:

```bash
~/isaacsim/python.sh scripts/sim/extract_testset_points.py   # 점군 캐시(1회)
$MMS_PYTHON scripts/sim/validate_lookaround.py
```

---

## 9. 코드 지도

```
utils/nbv/lookaround.py       ★ 자세 선정 전부 (sim·real 공용)
    plan_lookaround_viewpoints()        # §4 maximin 평가 + §5 밴드 분할
    solve_plan_poses()                  # az 스윕 IK + 충돌 게이트
    make_view_pose(up_sign)             # 평가용 자세 (실행과 동일한 up)
    collect_planning_points()           # preview 수집 루프 (→ 2_preview.md)
    crop_object_points / filter_robot_points
    swept_about_axis / guard_cylinder_points   # 대상물 장애물 (§2)
utils/nbv/standoff.py               ★ 거리 판단 전부
    StandoffTracker                 # §2 거리추종
utils/nbv/scan_stage_controller.py  # 단계 순서 (밴드 실패 허용)
utils/nbv/event_log.py              # events.jsonl (§7)
scripts/artec/lost_report.py        # 이벤트 집계

mms_artec/nbv/artec_streaming_scan_session.py   ★ 실물 streaming SLAM
    TrackingState(watchdog 4종) · TurntableController · run()
    _band_transition · _track_standoff
mms_artec/nbv/artec_multipass_scan_session.py   # 오케스트레이션 + 복구
    capture_bands · _do_one_rotation · _dump_scan_raw
mms_artec/nbv/live_scan_viewer.py               # SDK 정합행렬 미러
mms_artec/backends/isaac/isaac_turntable.py     # sim 턴테이블
utils/turntable/turntable_interface.py          # 실물 턴테이블 (UDP)

scripts/sim/plot_band_plan.py       # §5 그림 생성기
scripts/sim/run_e2e_gui.sh          # v3 씬 전용 — 회귀 확인용이며 실물 배치가 아니다
sim_harness/MMS_ext_lookaround*.py
    # ⚠ Isaac GUI Script Editor 용 독립 하니스다. 로봇을 한 자세에 고정하는 구 방식이라
    #   밴드·z 단조·거리추종이 적용되지 않는다 — 파이프라인 검증에 사용하지 않는다.
```

---

# 〔부록〕 문제 해결

### T1. 물체를 제거했는데 추적 상실이 발생하지 않는다
`registration` 이 ICP-only 로 설정되어 빈 원판에 정합이 성공한 경우다.
`set_registration_type(HYBRID)`(geometry+texture)를 사용한다.

### T2. 스캔 중 턴테이블 통신이 두절된다
턴테이블은 **UDP** 로 연결한다. TCP 는 `getActualPos` 를 10Hz 로 지속 조회하는 구간에서
소켓이 차단된다.

### T3. 충돌하지 않는 az 가 있는데 도달 불가로 판정된다
az 는 도달성·충돌만 좌우하므로 `solve_plan_poses` 가 스윕한다. 로그에
`band tz=… az=…° 충돌(...)` 이 없으면 게이트가 적용되지 않은 것이다.

### T4. 밴드 1 이 시작 1~2초 만에 `REGISTRATION_FAILED` 로 종료된다
`ignore_registration_errors` 가 False 인 경우다. **True 를 사용한다.**
SDK 문서상으로는 False 가 타당해 보이나, SDK 의 실시간 정합은 True 모드(실패 프레임도
예측 자세로 삽입하고 다음 프레임을 그에 정합)를 전제로 동작하며, False 에서는 마지막
정합 프레임과의 간격이 벌어져 연쇄 실패한다. 상실 판정은 우리 tracker 가 수행하고,
True 의 대가인 미정합 꼬리는 `n_tail_lost` 로 절단한다(`_trim_lost_tail`).
정상 동작은 `fail=0 regErr≈+0.25 last=OK` 다.

### T5. 〔한계〕 복구용 평가기는 θ 한 시점만 관측한다
θ=0 preview 하나로 선정하므로 손잡이·주둥이가 있는 비대칭 물체는 다른 구간에서 불리할
수 있다. §4 의 maximin 은 36 프레임 전부를 평가하므로 이 한계가 없다. 복구가 이를
사용하지 않는 이유는 소요 시간뿐이다.

### T6. 〔미검증〕 real 자세 선정을 실물에서 확인하지 않았다
코드는 연결되어 있으나(sim 과 동일 함수) sim 실행만 검증되었다. 첫 실물 운용 시:
1. `fill_target 12cm²` / `fill_min 6cm²` 은 sim 기준값이다. 실물 SLAM 이 유지되는 최소
   가시면적으로 재설정해야 `tracking_risk` 경고가 유효하다.
2. 계획용 preview 가 **최대 48회 이동**한다. 통상 8~9회에서 종료되나, 반환이 계속 0인
   물체(검은색·투명·반사)는 상한까지 도달한다. 첫 시도는 비상정지 옆에서 관찰할 것.
3. `T_BF0` 가 없으면 플래너가 중단되고 home 고정으로 전환된다 — 캘리브레이션이 선행되어야 한다.

`lookaround_planner_enabled=False` 로 구 동작(home 고정)으로 복귀할 수 있다.

### T7. sim 물체가 회색이거나 빈 결과로 종료된다
- **회색** — 텍스처 참조 결손. `UsdPreviewSurface` 는 텍스처를 열지 못하면 회색으로
  렌더한다. lookaround 는 형상만 사용하므로 결과에는 영향이 없다.
- **빈 결과** — 구 씬을 사용한 경우다. 카메라·턴테이블 prim 경로가 달라 스캔 없이
  종료된다. 사용 가능한 씬은 `sim_scene.md` §1.
- **모든 자세가 이동 거부** — 씬과 충돌 점군이 불일치한다. `sim_scene.md` §3.

### T8. 준수 사항
- **연속 회전 + 최대 FPS.** 겹침이 핵심이므로 각도를 이산적으로 이동시키지 않는다.
- **자세 선정은 sim·real 동일 코드.** 백엔드가 아니라 `utils/nbv/lookaround.py` 를 수정한다.
- **라이브 뷰어 누적은 SDK 정합행렬만 사용한다.**
- **sim 의 성공이 실물 정합의 성공을 의미하지 않는다.**
- **밴드 캡처 순서는 z 단조를 유지한다**(§5). min_fill 순서로 되돌리지 않는다.
- **자세 선정과 검사는 동일 게이트를 사용한다.** 한쪽에 게이트가 누락되면 다른 쪽 검사에서
  모든 이동이 거부되는 상태가 발생한다.
- **`ignore_registration_errors=True`** 를 유지한다(T4).
- **밴드 계획이 있으면 probe 복구를 사용하지 않는다**(§7).
- **동일 판단을 수행하는 코드가 복수이면** 게이트가 전부에 적용되었는지 확인하고,
  실패를 반환하는 함수의 반환값을 버리지 않는다.
