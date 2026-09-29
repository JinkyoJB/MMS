# nbv — 부족면 NBV 보강

> lookaround 가 취득한 5면에는 구멍이 남는다 — 윗면·옆면 경계의 grazing 영역, 옆면 사이
> occlusion, 손잡이나 오목부. nbv 는 그 구멍을 자동 검출하여(frontier) 로봇이 조준하고
> 추가 촬영하여 병합한다.
>
> 선행 `3_lookaround.md` · 바닥면 `5_flip.md`
>
> **운용에는 §1·§2 로 충분하다.** 문제 발생 시 부록(T1~T8)을 참조한다.
>
> ⚠ **실물에서 검증되지 않았다.** 로봇이 물체 옆을 통과하므로 충돌 설정을 먼저
> 확인한다(T2).

---

## 1. 실행

nbv 만 단독 실행할 수 없다. 보완할 구멍이 있어야 하므로 단계는 누적이다.

```bash
python main_artec.py --until nbv
python scripts/nbv/nbv_debug_view.py     # 반복별 gap·후보 확인
```

| 환경변수 | 내용 | 기본 |
|---|---|---|
| `MMS_NBV_K_MAX` | NBV 최대 반복 | 12 |
| `MMS_SIM_NBV_SPAN` | 목표 θ 중심 스윕 폭(°) | 90 (±45°) |
| `MMS_NBV_DRY_EPS` | 패치 생산성 판정(신규 복셀 비율) | 0.015 |
| `MMS_NBV_CONV_NEW_EPS` / `_STALL_N` | 전역 백스톱(비율 / 연속 횟수) | 0.005 / 3 |
| `MMS_SIM_ENSURE_ELS` | 반드시 1회 방문할 고도각(°) | 55 |
| `MMS_NBV_DEBUG` | 계획 덤프 (0=해제) | 1 |

실물 설정(`ArtecMultiPassScanSessionSettings`):

| 키 | 기본 | 내용 |
|---|---|---|
| `nbv_K_max` | 12 | 최대 반복 |
| `nbv_distance_mm` | 225 | **표면 기준** 카메라 거리 |
| `nbv_el_floor_deg` | 30 | 관측 고도각 하한. `el_need` clip 에도 사용 |
| `nbv_swept_steps` | 12 | 궤적 충돌검사 보간 지점 수 |
| `nbv_theta_assist` | False | True 면 턴테이블을 보조 자유도로 사용 |
| `nbv_frontier_enabled` | True | False = 축-고도각만 사용 |
| `nbv_frontier_az_pref_deg` | `(0, 30, -30)` | gap 을 배치할 로봇 선호 방위 |
| `nbv_ensure_els_deg` | `(55,)` | gap 과 무관하게 1회 방문 (오목 내부 대비) |
| `nbv_capture_mode` | `"step"` | 정지-촬영(30° 간격). `"sweep"` 은 스트리밍 |
| `nbv_min_patch_*` | 5,000점 / 3프레임 | 병합 게이트 (step 모드 기준) |
| `nbv_turntable_radius_mm` / `_body_height_mm` / `_collision_margin_mm` | 150 / 200 / 10 | 충돌 world 치수 — **실제 셀 측정치로 보정할 것**(T2) |

---

## 2. sim 과 real

판단 로직(frontier 검출·자세 생성·후보 순위·수렴 회계)은 **동일 코드다.** 차이는 캡처와
병합이다.

| | sim | real |
|---|---|---|
| 자세 선정 | ① ensure_el → ② gap 직접 겨냥 → ③ 축-고도각 | **동일 3단계** |
| 캡처 범위 | ② 목표 θ 중심 부분 스윕(±45°) · ③ 폴백은 gap 군집 중심 ±90° · ① 은 전회전이되 윗면 개구부가 있을 때만 | **동일** |
| 병합 | 기구학 초기값 → **공용 `refine_to_master`** (다단 point-to-plane ICP + RMSE/fitness/drift/팽창 게이트) | camera-motion `T_pre` 초기값 → **동일 함수** |
| SDK relocalization | 개념 없음 | **미배선**(T5) |
| 수렴 회계 기준 | 누적 **원시 점군**의 신규 복셀 | master **메시 정점**의 신규 복셀 |

**실물 통합에서 가장 불확실한 지점은 병합이다.** 자세 선정은 sim 이 담보하나, 로봇 이동
후 새 스캔이 기존 master 에 정상 정합되는지는 실물에서만 확인된다.

### 디버그 뷰어

반복마다 `output/<RUN>/debug/nbv_plan/nbv_<런>_<NN>.npz` + `.png` 가 생성된다. 내용은
master 점군(회색) · gap 후보 전부(주황, 크기=L) · 선택된 gap(빨강) · 카메라 프러스텀과
광축(파랑) · 스윕 구간(초록) · 촬영된 점(노랑) · 정합 후 점(연두) · 정합 결과다.

확인 순서는 **파란 프러스텀이 빨간 gap 을 포함하는가(겨냥) → 노란 점이 그 gap 위치에
있는가(수집) → 연두가 노랑에서 크게 벗어나지 않고 회색과 연결되는가(정합)** 다.

이 뷰어로 검출한 결함의 대응으로 세 가지가 반영되어 있다. ① 정합은 초기 자세에서
master 와 8mm 내에 겹치는 점만 사용하고(`REFINE_OVERLAP_R_M`), ② 패치 보정 한계를
8mm/2° 로 제한하며(`REFINE_DRIFT`), ③ 원판 위 25mm 이내 gap 은 frontier 에서 제외한다
(`FRONTIER_MIN_HEIGHT_M` — flip 의 몫).

---

## 3. 동작 개요

lookaround 는 로봇을 고정하고 턴테이블을 회전시켰다. nbv 는 반대로 **물체가 정지하고
로봇이 이동한다.** 따라서 자세 하나가 아니라 **경로 전체**를 검사해야 한다(§4).

```
   master 점군 ─▶ 메시 ─▶ 경계(구멍) 검출 ─▶ 후보 순위
        ▲                                        │
        └── 병합 ◀── 캡처 ◀── 로봇 이동 ◀────────┘   (수렴까지)
```

**구멍 검출** — master 점군을 base 프레임으로 추출하여 Poisson 메시를 생성하고(depth 6,
저밀도 정점은 4% 분위수로 trim) **삼각형 하나에만 속한 edge** 를 수집한다. 그것이 표면
경계, 즉 구멍이다(`extract_frontier_candidates`). 각 후보는 대표점 `p`, 바깥 법선 `n`,
길이 `L` 을 가진다. 6mm 미만은 잡음으로 제외하고 60mm 초과는 분할한다. depth 를 낮게
설정한 것은 이 메시가 **판단용**이기 때문이며, 최종 산출물은 후처리에서 별도 생성한다.

---

## 4. 자세 선정

호출 순서는 sim·real 이 동일하다.

```
① ensure_el 미방문이면 축-고도각으로 해당 고도각을 먼저 확보 (오목 내부 대비)
② gap 직접 겨냥 (주경로)
③ ② 실패 시 축-고도각으로 폴백
```

### 축-고도각 (`plan_nbv_elevation_pose`)

카메라를 물체 바깥 구면에 배치하고 **항상 턴테이블 축을 조준한다.** 선택 대상은 고도각과
방위각뿐이다. 캡처가 "로봇 한 자세 + 턴테이블 전회전"으로 통일되어 있으므로, NBV 는
결국 **어느 고도각으로 한 바퀴 더 돌 것인가**를 결정하는 문제가 된다.

1. gap 들의 바깥 법선 고도각의 **중앙값**을 `el_need` 로 삼는다(`el_floor+5` ~ 88° clip).
2. 후보 고도각은 `{el_need, −10, −20, −30}` 에 `el_extra(65, 55, 45)` 를 더해
   `el_need` 에 가까운 순으로 정렬한다.
3. `ensure_els`(기본 55°)를 미방문이면 최우선에 배치한다. 오목 물체 내부는 미관측이라
   메시에 없고, 없으면 경계로도 검출되지 않아 `el_need` 가 상승할 근거가 없다.
   실측으로 el 55° 한 자세 + 전회전이 컵 내벽과 내부 바닥을 100% 덮었다.
4. 각 고도각에서 방위각을 스윕하여 IK 와 궤적 충돌을 통과한 것 중 관절이동이 최소인
   것을 선택한다.

```
cost = Σ_i w_i·(q_i − q_cur,i)²        (w = [2, 2, 1.5, 1, 1, 1, 1])
```

base 측 큰 관절의 비용이 높으므로 동일 목표라면 **손목을 우선 사용한다.** `visited` 전달이
필수이며, 누락하면 직전 자세의 이동비용이 0 이 되어 **동일 자세를 무한 반복**한다.

**재방문 판정은 el 로만 수행한다.** `visited` 는 `(el, az)` 쌍으로 기록하되 건너뛸지는
el 만 확인한다. "az 는 관측 조건을 바꾸지 않는다 — 회전은 턴테이블이 담당한다"가 전제이기
때문이다. 동일 el 을 az 만 바꿔 다시 전회전하는 것은 **새 정보가 없는데 로봇만 크게
이동하는** 낭비다.

| 판정 | 전회전 패스 | 총 관절이동 | 방문 az | 최악 한 걸음 |
|---|---|---|---|---|
| `(el, az)` | 8 | 404.6° | 0, ±30 전부 | 129° |
| **`el` 만** | **4** | **139.2°** | **0 하나** | 44° |

전회전 8→4회(회당 30s), 관절이동 66% 감소, 커버리지 손실은 없다.

아랫면 gap(`n_z < −0.6`)은 후보에서 제외한다 — 관측으로는 보완할 수 없고 flip 이 유일한
해법이다.

### gap 직접 겨냥 (`plan_frontier`) — 주경로

축-고도각은 gap 정보를 "법선 고도각의 중앙값" 하나로 압축하므로 gap 의 위치 정보를
상실한다. 손잡이나 내벽 같은 국소 결손은 원리적으로 조준할 수 없다(실측: 손잡이 0점).

따라서 표면점 `p` 를 정면으로 관측하는 자세를 생성한다. **턴테이블이 gap 을 로봇 앞으로
이동시킨다** — gap 의 방위각을 로봇 선호 방위(`0, ±30°`)로 만드는 θ 를 구하고 그 θ 에서
조준하므로, 로봇은 고도각과 거리만 담당하여 팔의 이동이 작다. 법선 정면이 차단되면
(작동거리보다 물체가 큰 오목면 등) 개구부 쪽으로 기울인다.

```
d(θ_t) = normalize(n̂·cos θ_t + ẑ·sin θ_t)      θ_t ∈ {0, 30, 45, 60, 75°}
```

조준한 지점은 `visited_frontier` 에 누적되어 20mm 이내는 재선택하지 않으며, 비생산으로
판정된 지점은 40mm 반경이 후보에서 제외된다(§6).

### 실행 가능성 — 궤적 전체 검사

두 방식 모두 자세를 해석 IK 로 풀고, `q_cur → q_des` 궤적을 `nbv_swept_steps`(12) 지점으로
분할하여 전부 검사한다(`swept_pose_collision`). 한 곳이라도 걸리면 해당 후보를 배제한다.
끝점이 포함되므로 단일 자세 검사를 대체한다.

nbv 는 gap 조준을 위해 반대편까지 크게 이동하므로 **경로가 문제가 된다.**

---

## 5. 캡처와 병합

선순위 자세로 이동하여(`_move_robot_to_q`, 여기서도 충돌 게이트 통과) 캡처한다.

기본 캡처는 **정지-촬영**(`nbv_capture_mode="step"`, 30° 간격)이다. 턴테이블을 정지시키고
프레임을 하나씩 촬영하여 알려진 θ 로 배치하므로 **SDK 추적에 의존하지 않는다.**
프레임 변환 `T_k = S⁻¹·T_BC·R_B(axis,−Δθ_k)·T_CB·S` 는 합성 검증에서 0.0000mm 다.

스트리밍 모드(`"sweep"`)에서는 새 IScan 을 다음 변환으로 master 좌표에 배치한다.

```
T_pre = S⁻¹ · T_BC_master · R_B(axis, −θ₀) · T_CB_new · S
```

`T_BC_master` 는 **첫 회전 시작 시점의** 카메라-base 변환이다(master 좌표의 정의. 매
이동마다 갱신되는 `self._T_CB` 를 사용하면 master ≈ new 가 되어 보정이 무효화된다).
`T_CB_new` 는 새 자세의 FK + hand-eye, `θ₀` 는 **캡처 시작 시점의 실제 턴테이블 각**이다.
`S` 는 스캐너3D→Color 변환이다(scan-world 는 스캐너 프레임, hand-eye 는 Color 기준).

이 `T_pre` 를 초기값으로 ICP 를 추가 수행하여 master 에 병합한다.

**병합 게이트** — 4mm 복셀 5,000점 미만이거나 OK 프레임이 문턱 미만이면 병합하지 않고
dry 로 회계한다. step 모드는 90° 스윕에 7프레임이 설계값이므로 프레임 문턱이 3이다.

---

## 6. 종료 판정

### boundary 로 판정해서는 안 된다

경계 길이는 커버리지가 개선될 때도 **증가한다.** 새로 부착된 표면의 테두리가 경계로
검출되기 때문이다.

| | boundary | completeness@1mm |
|---|---|---|
| hand_drill | 519 → 800mm ("악화") | **37.9 → 52.7%** (개선) |
| mug | 533 → 586mm ("악화") | **33.1 → 37.7%** (개선) |

상관은 r=+0.96 으로 높으나 **부호가 반대다.**

### 신규 표면 점유율로 판정한다

누적 점군을 4mm 복셀로 분할하여 **이번 패치가 새로 점유한 복셀 비율**을 측정한다.
정보이득을 직접 측정하므로 GT 가 없는 실물에서도 사용 가능하다. 메시가 아니라 **원시
점군**에서 측정해야 한다 — 메시는 Poisson 표면이 패치마다 변동하여 신규 관측이 없어도
복셀이 변한다(r=+0.43 vs 원시 +0.93).

### 전역 정지가 아니라 gap 단위 회계

NBV 의 개선은 **간헐적**이다. 작은 구멍이 몇 차례 조용하다가 큰 구멍에서 한 번에 +3.3%p
가 발생한다. 따라서 "연속 N회 조용하면 종료" 방식의 전역 규칙은 손실이 없도록 설정하면
절약 효과도 없었다.

현재는 비생산 패치의 **조준점 주변 40mm 만 후보에서 제외한다**(`report_patch`). 계획기가
후보를 소진하면 루프가 자연 종료된다. 판정이 틀려도 손실은 해당 영역 하나뿐이며 스캔이
조기 종료되지 않는다. 생산성 임계는 신규 복셀 1.5% 이나 **물체 2종으로 조정한 값이다**(T3).

---

## 7. sim 검증 — 두 층

**GT 검증층**(sim 전용, 런타임 불개입) — 씬에서 정답 표면을 추출하여 표준 지표
(completeness / accuracy / F-score / Chamfer)를 계산한다. **런타임 지표의 타당성을
검증하는 층**이며, 이것이 없어 위 boundary 오독을 장기간 인지하지 못했다.
9종 결과는 `testset_results.md`.

**런타임 판단층**(GT 없음, 실물과 동일) — 신규 점유 복셀로만 판단한다.

```bash
env -u PYTHONPATH ~/miniconda3/envs/step2usd/bin/python scripts/sim/extract_gt_mesh.py --all
python scripts/sim/eval_vs_gt.py --scan <obj> --gt scripts/sim/log/gt/<name>.npz
python scripts/sim/validate_convergence.py --dirs scripts/sim/log/conv_val/*
```

---

## 8. 코드 지도

```
utils/nbv/frontier.py              # 경계 edge → 구멍 후보
utils/nbv/nbv_core.py              # pcd_to_mesh_poisson / detect_gaps / coverage_state
utils/nbv/nbv_planner.py           # 후보 순위 · dry gap 회계(report_patch)
utils/nbv/icp_strategy.py          # icp_with_gates / pick_icp_roll
utils/nbv/nbv_debug_dump.py        # 계획 덤프 (§2)
utils/collision/robot_collision.py # swept_pose_collision / collision_free_ik
utils/control/theta_planner.py     # DEFAULT_JOINT_WEIGHTS, θ assist 최소이동

mms_artec/nbv/artec_multipass_scan_session.py   ★ real nbv
    _build_master_mesh_B · _nbv_feasible_q · _capture_nbv_pose
    _merge_into_master · is_converged · _build_collision_world
mms_artec/backends/isaac/isaac_scan_session.py  # sim (_scan_patch 부분 스윕)

scripts/nbv/nbv_debug_view.py
scripts/sim/{extract_gt_mesh,eval_vs_gt,validate_convergence}.py
```

---

# 〔부록〕 문제 해결

### T1. 로봇이 이동을 거부한다
궤적 충돌검사가 `q_cur → q_des` 구간에서 차단한 경우다. 자세가 안전해도 **경로**가
막히면 거부된다. 로그가 `start(...)`/`goal(...)` 이면 자세 자체가 불가능한 경우이고,
그 외에는 우회 경로 계획까지 실패한 것이다. 실제 원인은 대개 T2 다.

### T2. 충돌 world 가 실제 셀과 다르다
`_build_collision_world` 는 턴테이블 캘리브(`T_BF0`)를 기준으로 원판과 몸체를 원기둥으로
근사한다. 치수가 설정값이므로 **실제 셀을 측정하여 입력해야 한다.** 프레임 구조물이
추가로 있으면 `add_box` 로 등록한다.

**`T_BF0` 가 없으면 world 가 구성되지 않고 충돌검사가 통째로 비활성화된다.**
`[nbv] ⚠ turntable_transform/T_BF0 없음` 이 출력되면 그 상태로 로봇을 구동하지 않는다.

### T3. 〔한계〕 생산성 임계 1.5% 는 물체 2종으로 조정한 값이다
드릴에서 1패치를 절약하며 손실 0.13%p, 머그는 손실 0 이었다. 더 공격적인 설정은 머그에서
완전성 2~3%p 를 상실했다. 동일 조건에서도 completeness 는 실행 간 약 5%p 편차가 있으므로
**그보다 작은 차이로 임계를 조정하지 않는다.** 확장하려면
`scripts/sim/validate_convergence.py` 를 사용한다.

### T4. 〔교훈〕 지표의 부호를 GT 로 한 번은 확인한다
boundary 를 "작을수록 좋다"로 해석하여 nbv 가 결과를 악화시킨다고 장기간 결론지었다.
단계별 메시를 덤프한 결과 반대였다 — 분리되어 있던 파편이 본체에 연결되며 경계가 증가한
것이었다. **런타임 지표는 그 자체로 타당하다는 보장이 없다.** sim 에서만이라도 정답과
대조하여 부호와 상관을 확인한 뒤 판단에 사용한다.

### T5. 〔미배선〕 SDK relocalization
설계상 1차 병합 경로는 SDK relocalization 이다. 로봇이 이동하면 일시적 추적 상실이
발생하고, 기존 표면을 다시 관측하면 SDK 가 재고정하여 프레임이 곧바로 master 좌표로
입력된다(`T_pre = I`). 노출 경로 후보는 두 가지다.

- **R1** — lookaround 의 `IScanningProcedure` 를 유지한 채 이동 중 PREVIEW, 재고정 후
  RECORD 토글. `ArtecStreamingScanSession` 확장이 필요하다.
- **R2** — `ScanningState.CONTINUE_RECORD` 로 master 를 입력 삼아 record 재개. SDK enum 은
  존재하나 문서상 비권장이므로 동작 검증이 필요하다.

둘 다 실기 검증 전이므로 현재는 §5 의 camera-motion 경로를 사용한다. 실기에서 확인할
항목은 relocalize 성공률·재고정 시간·필요 overlap 이다. relocalize 는 기존 표면과 overlap
이 있어야 성립하는데 NBV 는 구멍 가장자리를 관측하므로 대개 충족된다. 정합은
**HYBRID 필수**(`3_lookaround.md` T1).

### T6. 전회전이 발생하면 폴백을 의심한다
**①(ensure_el)·③(축-고도각)은 설계상 전회전이 맞다** — 카메라가 턴테이블 축을 조준하므로
한 바퀴가 곧 커버리지의 근거다. "gap 을 조준했는데 360° 를 도는" 상황이 관찰되면 ② 가
실패하여 ③ 으로 폴백한 것이다. `[nbv] gap 겨냥 실패 —` 로그의 내역(IK/충돌/dry)을 확인한다.

참고로 `nbv_pose_from_candidate`, `NbvPlanner.split_inward` 는 **정의만 있고 호출부가
없다**(per-gap 정면 방식의 잔재). 코드를 읽을 때 현행으로 오인하지 않는다.

### T7. 〔死조건〕 `is_converged` 의 boundary 임계는 발동한 적이 없다
경계 총길이 12mm 미만을 요구하는데 실측은 130~900mm 다. **한 번도 참이 된 적이 없다.**
실제 종료는 gap 회계와 `nbv_K_max` 가 결정한다. 활용하려면 임계를 실측 분포에 맞춰
재설정해야 한다.

### T8. ⚠ 축거리는 물체 반경 추정에 비례한다

**축을 조준하는 자세(ensure_el·폴백)에만 해당한다.** 표면거리 = 축거리 − **해당 방위의
국소 반경**이며, 물체가 원통이 아니면 방위마다 크게 다르다(실측 46~84mm). 반경을 잘못
추정하면 일부 방위가 작동창을 벗어난다.

| 반경 추정 | 축거리 | 방위별 표면거리 | 창(170~330) |
|---|---|---|---|
| 거리추종 `d−p` 12mm | 237mm | 154~191mm | **5/12 구간이 근접한계 안** → 빈 캡처 |
| master 점군 p95 69mm | 294mm | 211~248mm | 전부 정상, 최적대(200~250) |
| 잡음 과대 122mm | 347mm | 264~301mm | 창 안이나 최적대 밖 |

따라서 `_object_radius_m` 은 master 점군의 기하 실측(p95)을 기준으로 하며, 거리추종값은
그 0.5~1.5배 범위일 때만 사용한다. `_check_standoff_window` 가 매 nbv 진입마다 방위별
표면거리를 예측하여 창을 벗어나면 경고한다(`⚠ 창 밖 N구간`).

**gap 직접 겨냥은 표면 기준 `nbv_distance_mm` 를 그대로 사용하므로 이 반경과 무관하다.**

근접 가드는 스캐너 near + 40mm 이며 gap 겨냥·폴백·ensure_el 자세 전부에 적용된다.
가드 점군은 메시 ∪ preview ∪ master 다.

### T9. nbv 가 느리거나 동일 지점을 반복 조준한다
- **속도** — 점군에 O(N²) 파이썬 루프를 사용하지 않는다. 입사각 필터의 `_pca_normals` 가
  점마다 전수 거리 계산과 3×3 고유분해를 수행하여 프레임당 2초를 소요한 사례가 있다.
  KD-tree + 배치 `eigh` 로 37배 개선되었고 판정 결과는 동일했다. **최적화 전에
  `MMS_SIM_PROFILE_EVERY` 로 계측**하고(누적 타이머이므로 리포트 간 증분을 확인),
  수정 후에는 최종 불리언 마스크까지 이전 구현과 대조한다.
- **반복 조준** — gap 회계 동작 여부를 확인한다. `[nbv] 신규복셀=…%` 로그가 없으면
  비활성 상태이거나 예외로 건너뛰는 중이다(real `MMS_REAL_DRY_GAP=1`).

### T10. 디버그 이미지
정지-촬영 프레임마다 `output/<RUN_TS>/debug/nbv/nbvNN_stepKK_thDDD.png` 를 남긴다.
lookaround 와 동일한 거리 이미지(3D 점을 각도좌표로 전개, 카메라 거리로 착색, 흰 선 =
핵심 높이대) + 텍스처 프레임이며, 제목에 점 수·거리 중앙값·축거리가 표시된다.
"빈 캡처" 판정 시 물체가 시야 어디에 있었는지 즉시 확인할 수 있다.
해제는 `MMS_LOOKAROUND_DEBUG_IMG=0`.
