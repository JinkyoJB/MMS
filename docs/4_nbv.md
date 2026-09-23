# nbv — 부족면 NBV 보강

> lookaround 이 만든 5면에는 반드시 구멍이 남는다 — 윗면·옆면 경계의 grazing 영역, 옆면
> 사이 occlusion 띠, 손잡이나 오목부. nbv 는 그 구멍을 자동으로 찾아(frontier)
> 로봇이 그쪽을 조준해(NBV) 추가로 찍고 붙인다.
>
> 앞 단계 `3_lookaround.md` · 바닥면 `5_flip.md`
>
> **쓰기만 하면 §1(실행)·§2(sim·real 차이)로 충분하다.** 문제가 생기면 부록(T1~T8)을 본다.
>
> ⚠ **실물에서 검증된 적이 없다.** 로봇이 물체 옆을 지나가므로 충돌 설정부터 확인할 것(T2).

---

## 1. 실행

nbv 만 따로 돌릴 수는 없다 — 메울 구멍이 있어야 하므로 `stage_until` 는 누적이다.

```bash
./scripts/sim/run_e2e_gui.sh spray_can 2 planner    # lookaround → 2
```

| 환경변수 | 뜻 | 기본 |
|---|---|---|
| `MMS_NBV_K_MAX` | NBV 최대 반복 (sim·real 공용) | 12 |
| `MMS_SIM_NBV_SPAN` | 목표 θ 중심 스윕 폭(°) | 90 (±45°) |
| `MMS_NBV_DRY_EPS` | 패치 생산성 판정(신규 복셀 비율, sim·real 공용) | 0.015 |
| `MMS_NBV_CONV_NEW_EPS` / `MMS_NBV_CONV_STALL_N` | 전역 백스톱(비율 / 연속 횟수, sim·real 공용) | 0.005 / 3 |
| `MMS_SIM_ENSURE_ELS` | 반드시 한 번은 방문할 고도각(°) | 55 |
| `MMS_SIM_STAGE_DUMP` | 반복마다 메시·누적점군 덤프 디렉터리 | (끔) |
| `MMS_SIM_PROFILE_EVERY` | N 프레임마다 단계별 소요시간 (0=끔) | 20 |

**실물**은 `BACKEND = "real"` + `MULTIPASS_SETTINGS.stage_until = 2` 로 두고 실행한다.
주요 설정(`ArtecMultiPassScanSessionSettings`):

| 키 | 기본 | 뜻 |
|---|---|---|
| `nbv_K_max` | 12 | 최대 반복 |
| `nbv_distance_mm` | 225 | 턴테이블 축에서 카메라까지 standoff |
| `nbv_el_floor_deg` | 30 | 관측 고도각 하한 (lookaround 측면각 위에서만 보강). `el_need` clip 에도 쓰인다 |
| `nbv_swept_steps` | 12 | 궤적 충돌검사 보간 지점 수 |
| `nbv_theta_assist` | False | True 면 턴테이블을 보조 자유도로 |
| `nbv_frontier_enabled` | True | False = 축-고도각만 (2026-09-10 이전 동작) |
| `nbv_frontier_az_pref_deg` | `(0, 30, -30)` | gap 을 가져다 놓을 로봇 편한 방위 |
| `nbv_ensure_els_deg` | `(55,)` | gap 과 무관하게 한 번은 갈 고도각 (오목 내부 대비) |
| `nbv_view_azis_deg` | `(0, 30, -30)` | 축-고도각 방위 후보 |
| `nbv_turntable_radius_mm` / `_body_height_mm` / `_collision_margin_mm` | 150 / 200 / 10 | 충돌 world 치수 — **실제 셀 측정치로 보정할 것**(T2) |
| `MMS_REAL_DRY_GAP` / `MMS_NBV_DRY_EPS` | 1 / 0.015 | gap 회계 on-off(real), 생산성 임계(sim·real 공용) |

---

## 2. sim 과 real — 무엇이 같고 무엇이 다른가

판단 로직(`utils/nbv/` 의 frontier 검출·자세 생성·후보 순위·수렴 회계)은 **같은 코드다.**
다른 것은 캡처와 병합이다.

| | sim | real |
|---|---|---|
| 자세 선정 | ① ensure_el → ② gap 직접 겨냥 → ③ 축-고도각 | **같은 3단**(2026-09-10 배선) |
| 캡처 범위 | ② 목표 θ 중심 **부분 스윕**(±45°) · ③ 폴백은 gap 군집 중심 **±90°**(2026-09-18) · ① ensure_el 은 전회전이지만 **윗면 개구부가 있을 때만** 돈다(`needs_ensure`) | **같음** |
| 병합 | 기구학(identity) 초기값 → **공용 `refine_to_master`** (다단 point-to-plane ICP + RMSE/fitness/drift(단계·누적)/팽창 게이트) | camera-motion `T_pre` 초기값 → **같은 함수**(2026-09-18 배선. 그 전엔 nbv 는 정합 없이 T_pre 만, flip 은 colored ICP) |
| SDK relocalization | 개념 없음 | **미배선**(T5). `nbv_use_relocalization=True` 여도 실제로는 위 경로로 간다 |
| 수렴 회계 기준 | 누적 **원시 점군**의 신규 복셀 | master **메시 정점**의 신규 복셀 |

### 디버그 뷰어 — 겨냥·수집·정합을 그림으로 (sim·real 공용, 2026-09-18)

반복마다 `output/<RUN>/debug/nbv_plan/nbv_<런>_<NN>.npz` + `.png` 가 남는다(`utils/nbv/nbv_debug_dump.py`,
끄기 `MMS_NBV_DEBUG=0`). 담기는 것: master 점군(회색) · gap 후보 전부(주황, 크기=L) ·
고른 gap(빨강) · 카메라 프러스텀과 광축(파랑) · 스윕 구간(초록) · 찍힌 점(노랑) ·
정합 후 점(연두) · 정합 결과(ok/fitness/Δt). PNG 는 창 없이도 생기고(위·옆 두 시점),
창으로 보려면 **다른 터미널에서**:

```bash
conda activate mms-env && python scripts/nbv/nbv_debug_view.py     # [ ] 로 반복 이동
```

확인 순서: 파란 프러스텀이 빨간 gap 을 담고 있나(겨냥) → 노란 점이 그 gap 자리에 있나(수집)
→ 연두가 노랑에서 크게 안 벗어나고 회색과 이어지나(정합).

이 뷰어로 처음 잡은 결함(2026-09-18, 세제): 바닥 테두리 gap 을 겨냥한 패치가 **정합에서
18.6mm 밀려 붙었는데 게이트를 전부 통과**했다 — 패치 대부분이 새 면이라 ICP 가 기존 면으로
미끄러진 것. 그래서 ① 정합은 초기 자세에서 master 와 8mm 안에 겹치는 점만 쓰고(`REFINE_OVERLAP_R_M`),
② 패치 보정 한계를 8mm/2°(초기값 오차 크기)로 내렸고(`REFINE_DRIFT`), ③ 원판 위 25mm 안의
gap 은 frontier 에서 뺐다(`FRONTIER_MIN_HEIGHT_M` — flip 몫).

**실물 통합에서 가장 불확실한 지점은 병합이다.** 자세를 어디로 보낼지는 sim 이
담보하지만, 로봇이 이동한 뒤 새 스캔이 기존 master 에 제대로 붙느냐는 실물에서만
확인된다.

---

## 3. 한 바퀴에 무슨 일이 일어나나

lookaround 은 로봇을 고정하고 턴테이블을 돌렸다. nbv 는 반대로 **물체가 서 있고 로봇이
움직인다.** 그래서 자세 하나가 아니라 **가는 경로 전체**를 검사해야 한다(§4).

```
   master 점군 ─▶ 메시 ─▶ 경계(구멍) 검출 ─▶ 후보 순위
        ▲                                        │
        └── 병합 ◀── 캡처 ◀── 로봇 이동 ◀────────┘   (수렴할 때까지)
```

턴테이블은 기본적으로 고정이다(`nbv_theta_assist = False`). 보조 자유도로 쓰는 경로가
있지만 기본은 로봇만 움직인다.

**구멍 검출** — master 점군을 base 프레임으로 뽑아 Poisson 메시를 만들고(depth 6, 저밀도
정점은 4% 분위수로 trim) **삼각형 하나에만 속한 edge** 를 모은다. 그것이 곧 표면 경계,
즉 구멍이다(`extract_frontier_candidates`). 각 후보는 대표점 `p`, 바깥 법선 `n`, 길이 `L`
을 갖는다. 6mm 미만은 잡음이라 버리고 60mm 초과는 잘라 나눈다. depth 를 낮게 잡은 것은
이 메시가 **판단용**이기 때문이다 — 최종 산출물은 후처리에서 따로 만든다.

---

## 4. 자세 선정

자세를 고르는 방식이 **둘**이고, 호출 순서는 sim·real 이 같다.

```
① ensure_el 미방문이면 축-고도각으로 그 고도각을 먼저 확보 (오목 내부 대비)
② gap 직접 겨냥 (주경로)
③ ②가 실패하면 축-고도각으로 폴백
```

### 축-고도각 (`plan_nbv_elevation_pose`)

카메라를 물체 바깥 구면에 놓고 **늘 턴테이블 축을 겨눈다.** 고를 것은 고도각과 방위각뿐
이다. 캡처가 "로봇 한 자세 + 턴테이블 전회전"으로 통일돼 있으니(2026-06-30), NBV 는
결국 **어느 고도각으로 한 바퀴 더 돌 것인가**를 정하는 문제가 된다.

1. gap 들의 바깥 법선 고도각을 모아 **중앙값**을 `el_need` 로 삼는다
   (`el_floor+5` ~ 88° 로 clip).
2. 후보 고도각은 `{el_need, −10, −20, −30}` 에 `el_extra(65, 55, 45)` 를 더해
   **`el_need` 에 가까운 순**으로 정렬한다.
3. `ensure_els`(기본 55°)를 아직 안 가봤으면 맨 앞에 놓는다. 오목 물체 내부는 미관측이라
   메시에 없고, 없으면 경계로도 안 잡혀 `el_need` 가 올라갈 근거가 없다(닭·달걀). 실측으로
   el 55° 한 자세 + 전회전이 컵 내벽과 내부 바닥을 100% 덮었다.
4. 각 고도각에서 방위각을 스윕해(real `nbv_view_azis_deg = (0, ±30)`) IK 와 궤적 충돌을
   통과한 것 중 관절이동이 가장 작은 것을 고른다.

```
cost = Σ_i w_i·(q_i − q_cur,i)²        (w = [2, 2, 1.5, 1, 1, 1, 1])
```

base 쪽 큰 관절이 비싸므로 같은 목표면 **손목을 먼저 쓴다.** `visited` 를 넘기는 것이
필수인데, 안 넘기면 직전 자세의 이동비용이 0 이라 **같은 자세를 무한 반복**한다(실측:
el 65 / az −30 을 4회 연속 선택, gap 18→18→20→19).

### 재방문 판정은 **el 로만** 한다 (2026-09-17)

`visited` 는 `(el, az)` 쌍으로 기록하지만, 건너뛸지는 **el 만** 본다. 이 방식의 전제가
"az 는 관측 조건을 바꾸지 않는다 — 회전은 턴테이블이 담당한다" 이기 때문이다. 같은 el 을
az 만 바꿔 다시 **전회전**하는 것은 **새 정보가 0 인데 로봇만 크게 움직이는** 일이다.

`(el, az)` 로 판정하면 el 하나당 az 후보 수만큼 재방문이 허용돼 정확히 그 낭비가 생긴다.
실측 2026-09-17 — **실물 셀 상수 + 실물 home + 충돌 게이트**, 가짜 gap 3개로 8회 반복
(`scripts/artec/validate_real_cell.py` 와 같은 조건):

| 판정 | 전회전 패스 | 총 관절이동 | 방문 az | 최악 한 걸음 |
|---|---|---|---|---|
| `(el, az)` | 8 | 404.6° | 0, ±30 전부 | 129° |
| **`el` 만** | **4** | **139.2°** | **0 하나** | 44° |

**전회전 8회 → 4회**(30s/회 = 2분 절약), 관절이동 **66% 감소**, 최악 한 걸음 129°→44°,
그리고 로봇이 **한 방위에 머문다.** 줄어든 4회는 전부 같은 el 을 az 만 바꿔 다시 돌던
중복이라 커버리지 손실이 없다.

아랫면 gap(`n_z < −0.6`)은 후보에서 제외한다 — 관측으로는 못 메우고 flip flip 이
유일한 해법이다(`hw_layout.md` T4).

### gap 직접 겨냥 (`plan_frontier`) — 주경로

축-고도각은 gap 정보를 **"법선 고도각의 중앙값" 하나로 압축**하므로 gap 이 어디 있는지를
통째로 버린다. 손잡이나 내벽 같은 국소 결손은 원리적으로 겨냥할 수 없다(실측: 손잡이 0점).

그래서 표면점 `p` 를 정면으로 보는 자세를 만든다. **턴테이블이 gap 을 로봇 앞으로
가져온다** — gap 의 방위각을 로봇이 편한 방위(`0, ±30°`)로 만드는 θ 를 구하고 그 θ 에서
겨냥하므로, 로봇은 고도각과 거리만 담당해 팔이 크게 움직이지 않는다. 법선 정면이 막히면
(작동거리보다 물체가 큰 오목면 등) 개구부 쪽으로 기울인다.

```
d(θ_t) = normalize(n̂·cos θ_t + ẑ·sin θ_t)      θ_t ∈ {0, 30, 45, 60, 75°}
```

겨냥한 지점은 `visited_frontier` 에 쌓여 20mm 안쪽은 다시 고르지 않고, 비생산으로
판정된 지점은 40mm 반경이 후보에서 빠진다(§6 gap 회계).

> **캡처는 목표 θ 를 중심으로 한 부분 스윕이다**(`nbv_patch_span_deg = 90`, sim 과 동일).
> 턴테이블을 `θ + span/2` 로 보낸 뒤 span 만큼만 돌린다 — 이미 가진 면을 다시 도는
> 시간이 그만큼 빠진다. 2026-09-16 배선, 실기 검증 전(T6).

### 실행 가능성 — 궤적 전체를 검사한다

두 방식 모두 자세를 **해석 IK**(`xarm7_kinematics.ik`)로 풀고, `q_cur → q_des` 궤적을
`nbv_swept_steps`(12) 지점으로 나눠 전부 검사한다(`swept_pose_collision`). 한 곳이라도
걸리면 그 후보를 버린다. 끝점이 포함되므로 단일 자세 검사를 대체한다.

lookaround 도 밴드마다 자세를 옮기므로 같은 검사가 필요하지만, 거기서는 이동이 z 방향
이웃 밴드로 짧다. nbv 는 gap 을 겨냥해 **반대편으로도 크게 돌므로 가는 길이 문제**가 된다.
SDK IK 는 쓰지 않는다(2026-06 결정, `collision.md` §3).

---

## 5. 캡처와 병합

1등 자세로 이동해(`_move_robot_to_q`, 여기서도 충돌 게이트를 통과) 캡처한다. sim 은 목표
θ 주변만 부분 스윕한다 — 구멍이 있는 방향만 보면 되므로 한 바퀴를 다 돌 이유가 없다.

real 도 gap 겨냥이면 같은 부분 스윕이고(①③ 은 전회전), 새 IScan 을 다음 변환으로
master 좌표에 올린다.

```
T_pre = S⁻¹ · T_BC_master · R_B(axis, −θ₀) · T_CB_new · S
```

`T_BC_master` 는 **첫 회전 시작 시점의** 카메라-base 변환(= master 좌표의 정의 — 매 이동
마다 갱신되는 `self._T_CB` 를 쓰면 master ≈ new 가 돼 보정이 무효가 된다), `T_CB_new` 는
새 자세의 FK + hand-eye, `θ₀` 는 **캡처 시작 시점의 실제 턴테이블 각**이다. gap 겨냥은
물체가 θ₀ 만큼 돌아간 채로 스캔하므로 카메라 이동만 보정하면 그 스캔이 회전된 채 합쳐진다.
`S` 는 스캐너3D→Color 변환 — scan-world 는 스캐너 프레임인데 hand-eye 는 Color 기준이다.
이 `T_pre` 를 초기값으로 ICP 를 한 번 더 돌려 다듬고(`hint_icp_refine_static`) master 에
합친다. (2026-09-16: 셋 다 빠져 있어 메시가 물체에서 ~0.5m 뜨고 유령 geometry 가 생겼다.)

설계상으로는 SDK relocalization 으로 이 변환 자체를 생략할 수 있지만 아직 배선되지
않았다(T5).

---

## 6. 종료 판정

### boundary 로 판정하면 안 된다

경계 길이는 커버리지가 좋아질 때도 **늘어난다.** 새로 붙은 표면의 테두리가 그대로 경계로
잡히기 때문이다. GT 대조 실측(2026-08-19):

| | boundary | completeness@1mm |
|---|---|---|
| hand_drill | 519 → 800mm ("악화") | **37.9 → 52.7%** (개선) |
| mug | 533 → 586mm ("악화") | **33.1 → 37.7%** (개선) |

상관은 r=+0.96 으로 높지만 **부호가 반대**다. 이걸 거꾸로 읽어 "nbv 가 머그를
악화시킨다"는 결론을 한동안 유지했었다(T4).

### 대신 "새 표면이 더 안 붙는가"를 본다

누적 점군을 4mm 복셀로 나눠 **이번 패치가 새로 점유한 복셀 비율**을 잰다. 정보이득을 직접
재는 것이라 GT 없는 실물에서도 쓴다. 메시가 아니라 **원시 점군**에서 재야 한다 — 메시는
Poisson 표면이 패치마다 흔들려 새로 본 게 없어도 복셀이 바뀐다(r=+0.43 vs 원시 +0.93).

### 전역 정지가 아니라 gap 단위 회계

NBV 의 개선은 **간헐적**이다. 작은 구멍이 두어 번 조용하다가 큰 구멍에서 한 번에 +3.3%p
가 나온다. 그래서 "연속 N 회 조용하면 종료"류 전역 규칙은 손실 없는 설정이면 절약도 0
이었다.

지금은 비생산 패치의 **겨냥점 주변 40mm 만 후보에서 제외**한다(`report_patch`). 계획기가
후보를 소진하면 루프가 자연히 끝난다. 판정이 틀려도 잃는 건 그 영역 하나뿐이고 스캔이
통째로 일찍 끝나지 않는다. 생산성 임계는 신규 복셀 1.5%(실측 분포 비생산 0.2~1.2% /
생산 1.7~10.3% 의 사이값)인데 **물체 2종으로 맞춘 값**이다(T3).

---

## 7. sim 검증 — 두 층으로 나눈다

**GT 검증층**(sim 전용, 런타임 불개입) — 씬에서 정답 표면을 뽑아 표준 지표
(completeness / accuracy / F-score / Chamfer)를 계산한다. **런타임 지표가 맞는지 검증하는
층**이며, 이게 없어서 위 boundary 오독을 오래 몰랐다. 9종 결과는 `docs/testset_results.md`.

**런타임 판단층**(GT 없음, 실물과 동일) — 신규 점유 복셀로만 판단한다.

```bash
env -u PYTHONPATH ~/miniconda3/envs/step2usd/bin/python scripts/sim/extract_gt_mesh.py --all
python scripts/sim/eval_vs_gt.py --scan <obj> --gt scripts/sim/log/gt/<name>.npz
python scripts/sim/validate_convergence.py --dirs scripts/sim/log/conv_val/*
```

Isaac 하니스 `sim_harness/MMS_ext_nbv.py` 는 lookaround 을 낮은 고도각으로 돌려
**윗면에 일부러 구멍을 내고** nbv 가 그걸 검출·보강하는지 본다.

---

## 8. 코드 지도

```
utils/nbv/frontier.py              # 경계 edge → 구멍 후보
utils/nbv/nbv_core.py            # pcd_to_mesh_poisson / detect_gaps / coverage_state
                                   #   nbv_pose_from_candidate / joint_motion_cost
utils/nbv/nbv_planner.py           # 후보 순위 · dry gap 회계(report_patch)
utils/nbv/manual_picker.py         # compute_camera_pose_from_normal
utils/nbv/icp_strategy.py          # icp_with_gates / pick_icp_roll
utils/collision/robot_collision.py # swept_pose_collision / collision_free_ik
utils/control/theta_planner.py     # DEFAULT_JOINT_WEIGHTS, θ assist 최소이동

mms_artec/nbv/artec_multipass_scan_session.py   ★ real nbv
    _build_master_mesh_B · _nbv_feasible_q
    _capture_nbv_pose · _merge_into_master · is_converged · _build_collision_world
mms_artec/backends/isaac/isaac_scan_session.py  # sim (_scan_patch 부분 스윕)

scripts/sim/{extract_gt_mesh,eval_vs_gt,validate_convergence}.py
sim_harness/MMS_ext_nbv.py
```

---

# 〔부록〕 Troubleshooting

### T1. 로봇이 이동을 거부한다
궤적 충돌검사가 `q_cur → q_des` 사이에서 걸렸다. 자세는 안전해도 **가는 길**이 막히면
거부된다. 로그가 `start(...)`/`goal(...)` 이면 자세 자체가 불가한 경우고, 그 외에는 우회
경로 계획까지 실패한 것이다. 진짜 원인은 대개 T2 다.

### T2. 충돌 world 가 실제 셀과 다르다
`_build_collision_world` 는 턴테이블 캘리브(`T_BF0`)를 기준으로 원판과 몸체를 원기둥으로
근사한다. 치수가 설정값이므로 **실제 셀을 재서 넣어야 한다.** 프레임 구조물이 더 있으면
`add_box` 로 추가한다.

**`T_BF0` 가 없으면 world 가 구성되지 않고 충돌검사가 통째로 skip 된다.**
`[nbv] ⚠ turntable_transform/T_BF0 없음` 이 뜨면 그 상태로 로봇을 움직이지 말 것.

### T3. 〔한계〕 생산성 임계 1.5% 는 물체 2종으로 맞춘 값이다
드릴에서 1패치를 아끼며 손실 0.13%p, 머그는 손실 0 이었다. 더 공격적인 설정은 머그에서
완전성 2~3%p 를 잃었다 — 개선이 간헐적이라 두 번 조용했다고 끝난 게 아니었다.
같은 조건에서도 completeness 는 실행 간 ~5%p 편차가 있으니 **그보다 작은 차이로 임계를
다투지 말 것.** 넓히려면 `validate_convergence.py`.

### T4. 〔교훈〕 지표의 부호를 GT 로 한 번은 확인할 것
boundary 를 "작을수록 좋다"로 읽어 nbv 가 물체를 악화시킨다고 한동안 결론 냈었다.
단계별 메시를 덤프해 보니 반대였다 — 떠 있던 파편이 본체에 연결되며 경계가 늘어난
것이었다. **런타임 지표는 그 자체로 옳다는 보장이 없다.** sim 에서만이라도 정답과 대조해
부호와 상관을 확인한 뒤 판단에 쓴다.

### T5. 〔미배선〕 SDK relocalization 을 아직 안 쓴다
설계상 1차 병합 경로는 SDK relocalization 이다. 로봇이 이동하면 일시적 tracking-lost 가
나고, 기존 표면을 다시 비추면 SDK 가 재고정해 프레임이 **곧바로 master 좌표로** 들어온다
(`T_pre = I`). 노출 경로 후보는 둘이다.

- **R1** — lookaround 의 `IScanningProcedure` 를 stop 하지 않고 유지. 이동 중 PREVIEW,
  재고정 후 RECORD 토글. `ArtecStreamingScanSession` 확장 필요.
- **R2** — `ScanningState.CONTINUE_RECORD` 로 master 를 입력 삼아 record 재개. SDK enum 은
  있으나 문서상 비권장이라 동작 검증 필요.

둘 다 실기 검증 전이라 현재는 §5 의 camera-motion `T_pre` 경로로 간다. 실기에서 볼 것은
relocalize 성공률·재고정 시간·필요 overlap 이다. relocalize 는 기존 표면과 **overlap 이
있어야** 성립하는데 NBV 는 구멍 가장자리를 보므로 대개 충족된다. 부족하면 스윕 각을
키우거나 standoff 를 조정한다. 정합은 **HYBRID 필수**(`3_lookaround.md` T1).

### T6. real 부분 스윕 — 배선됨(2026-09-16), 실기 검증 전

gap 직접 겨냥(②)의 캡처는 이제 **목표 θ 중심 부분 스윕**이다
(`nbv_patch_span_deg = 90`, sim 과 같은 값). `_capture_nbv_pose` 가
`streaming_settings.sweep_rad` 로 넘기고, 스트리밍 세션이 그만큼만 돌며 타임아웃도
목표각에 비례한다.

**①(ensure_el)·③(축-고도각) 은 설계상 전회전이 맞다** — 카메라가 턴테이블 축을 보므로
한 바퀴가 곧 커버리지의 근거다. "gap 을 겨냥했는데 360° 를 돈다" 가 보이면 ② 가 실패해
③ 으로 폴백한 것이다. `[nbv] gap 겨냥 실패 —` 로그의 내역(IK/충돌/dry)을 볼 것.

⚠ 실기 검증 전이다.

참고로 `nbv_pose_from_candidate`, `NbvPlanner.split_inward` 은 **정의만 있고 호출부가
없다**(per-gap 정면 방식의 잔재, 2026-06-30 캡처 통일 때 대체). 같은 잔재였던
`_rank_nbv_candidates`·`geometric_cost` 는 2026-09-18 에 지웠다. 코드를 읽을 때
현행으로 오해하지 말 것.

### T7. 〔死조건〕 `is_converged` 의 boundary 임계는 발동한 적이 없다
경계 총길이 12mm 미만을 요구하는데 실측은 130~900mm 다. **한 번도 참이 된 적이 없다**
(real 동일). 실제 종료는 gap 회계와 `nbv_K_max` 가 낸다. 살리려면 임계를 실측 분포에
맞춰 다시 잡아야 한다.

### T8. nbv 가 느리다 / 같은 자리를 계속 겨냥한다
- **느림** — 점군에 O(N²) 파이썬 루프를 넣지 말 것. 입사각 필터의 `_pca_normals` 가 점마다
  전수 거리 계산 + 3×3 고유분해를 돌려 프레임당 2초를 먹었고 한 패치가 4분을 넘겼다.
  KD-tree + 배치 `eigh` 로 37배 빨라졌고 판정 결과는 동일했다. 추측으로 두 번 헛짚었으니
  **최적화 전에 `MMS_SIM_PROFILE_EVERY` 로 계측**하고(누적 타이머라 리포트 간 **증분**을
  볼 것), 고친 뒤엔 최종 불리언 마스크까지 옛 구현과 대조할 것.
- **같은 자리** — gap 회계가 도는지 본다. `[nbv] 신규복셀=…%` 로그가 없으면 꺼져 있거나
  예외로 건너뛰는 중이다(real `MMS_REAL_DRY_GAP=1`).

## real nbv 재설계 — 2026-09-22 (run_132655 진단)

**무엇이 실패했나.** nbv 12회 중 유효 데이터는 #1(윗면 보장 전회전) 하나. gap 겨냥 6회는
카메라가 뚜껑에서 161~218mm(Spider 근접한계 170mm)에 놓여 빈 프레임(5~4,800점)이었고,
폴백 5회는 윗면을 내려다보는 자세라 SDK 추적이 5프레임 만에 끊겼다. 빈 캡처가 그대로
master 에 병합돼 다음 반복의 메시·gap 을 오염시켰다(허공 gap → 또 빈 캡처).
원인 셋: preview 가 뚜껑을 버려 물체 높이를 36mm 낮게 봄(2_preview 원시 문턱 수정),
근접 가드 `EYE_CLEAR_M=150mm` 가 근접한계 아래, 병합에 품질 게이트 없음.

**바뀐 것** (`artec_multipass_scan_session.py`)
| 항목 | 지금 |
|---|---|
| 근접 가드 | 스캐너 near + 40mm. gap 겨냥·폴백·윗면 보장 자세 전부. 가드 점군 = 메시 ∪ preview ∪ master |
| 병합 게이트 | 4mm 복셀 5,000점·OK 10프레임 미만이면 병합 안 함 + dry 회계 (`nbv_min_patch_*`) |
| 캡처 | **정지-촬영** (`nbv_capture_mode="step"`, 30° 간격): 턴테이블을 세우고 프레임 하나씩 찍어 알고 있는 θ 로 배치 — SDK 추적 없음. `"sweep"` 이면 예전 스트리밍 |
| 정리 | 패치도 SerialReg (+옵션 Outlier) |

정지-촬영 프레임 변환 `T_k = S⁻¹·T_BC·R_B(axis,−Δθ_k)·T_CB·S` 는 합성 검증 0.0000mm.
ICP 게이트(RMSE = corr/2, drift 8mm/2°)는 실측 RMSE 2.4~2.5mm 로 전부 기각됐지만, 완화는
`icp_strategy.py` 의 경고(풀면 미끄러짐)대로 raw sproj 오프라인 실험으로 정하고 건드리지 않았다.

### 디버그 이미지 (2026-09-22)

> ⚠ **축거리(standoff)는 물체 반경 추정에 비례한다** — 축을 겨누는 자세(보장 고도각 등)에만.
> 표면거리 = 축거리 − **그 방위의 국소 반경**이고, 물체가 원통이 아니면 방위마다 크게 다르다
> (run_125718: 46~84mm). 반경을 잘못 보면 어떤 방위가 작동창 밖으로 나간다:
>
> | 반경 추정 | 축거리 | 방위별 표면거리 | 창(170~330) |
> |---|---|---|---|
> | 거리추종 `d−p` 12mm (한 밴드 −15mm) | 237mm | 154~191mm | **5/12 구간이 근접한계 안** → 빈 캡처 |
> | master 점군 p95 69mm | 294mm | 211~248mm | 전부 정상, 최적대(200~250) |
> | 잡음 과대 122mm | 347mm | 264~301mm | 창 안이나 최적대 밖 |
>
> 그래서 `_object_radius_m` 은 master 점군의 기하 실측(p95)을 기준으로 삼고 추종값은 그
> 0.5~1.5배 안일 때만 쓴다. 그리고 `_check_standoff_window` 가 **매 nbv 진입마다** 방위별
> 표면거리를 예측해 창을 벗어나면 경고한다(`⚠ 창 밖 N구간`).
>
> ⚠ gap 겨냥(`plan_frontier`)은 **표면 기준** `nbv_distance_mm` 를 그대로 받으므로 이 반경과
> 무관하다. run_125718 의 `물체근접 42` 기각은 **원인 미규명**이다(todo).

정지-촬영 프레임마다 `output/<RUN_TS>/debug/nbv/nbvNN_stepKK_thDDD.png` 를 남긴다 —
lookaround 와 같은 거리 이미지(3D 점을 각도좌표로 펼치고 카메라 거리로 색칠, 흰 선 = 핵심
높이대) + 텍스처 프레임. 제목에 점 수·거리 중앙값·축거리. "빈 캡처" 판정이 났을 때 물체가
시야 어디에 있었는지, 점이 정말 없었는지를 바로 본다. 끄려면 `MMS_LOOKAROUND_DEBUG_IMG=0`. run 중에는 `live_range_view.py` 창에 lookaround
이미지와 함께 시간순으로 뜬다(`3_lookaround.md` §1).
nbv 축거리의 반경은 lookaround 거리추종 실측(`r_eff = d − p`)을 1순위로 쓴다(`3_lookaround.md` §7).
