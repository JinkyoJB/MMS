# Phase 2 — 부족면 NBV 보강

> Phase 1 이 만든 5면 점군에는 반드시 구멍이 남는다. 윗면-옆면 경계의 grazing 영역,
> 옆면 사이 occlusion 띠, 손잡이나 오목부가 그렇다. Phase 2 는 그 구멍을 자동으로
> 찾아(frontier) 로봇이 그쪽을 조준해(NBV) 추가로 찍고 붙인다.
>
> 앞 단계는 `2_phase1.md`, 바닥면은 `5_phase3_merge.md` 를 본다.
>
> **통합만 필요하다면 §3(sim·real 차이)과 §4(실행)까지만 읽어도 된다.** §5 이후는
> 검출·자세 선정·병합·종료 판정의 원리와 코드 지도다.
>
> **결과가 이상하면 맨 뒤 〔부록〕 Troubleshooting 을 먼저 본다.** 규약·함정·알려진
> 미해결 문제를 T1~T8 로 모아두었다.

---

## 1. 무엇을 하나

Phase 1 은 로봇을 고정하고 턴테이블만 돌렸다. Phase 2 는 반대로 **턴테이블이 아니라
로봇이 움직인다.** 물체는 제자리에 있고 카메라가 구멍 쪽으로 옮겨간다.

```
   master 점군 ──▶ 메시 ──▶ 경계(구멍) 검출 ──▶ 후보 자세 순위
        ▲                                              │
        └────── 병합 ◀── 캡처 ◀── 로봇 이동 ◀──────────┘
                                (수렴할 때까지 반복)
```

Phase 1 과 달리 **로봇이 물체 가까이에서 움직이므로 충돌이 실제 위험**이다. 그래서
자세 하나하나가 아니라 **가는 경로 전체**를 검사한다(§6).

---

## 2. 적용된 로직

한 바퀴는 다음 여섯 단계다.

| 단계 | 하는 일 | 상세 |
|---|---|---|
| 1 | master 점군을 base 프레임 메시로 만든다 | §5 |
| 2 | 메시 경계에서 구멍 후보를 뽑는다 (frontier) | §5 |
| 3 | 후보마다 그 면을 정면으로 보는 카메라 자세를 만들고, 해석 IK·궤적 충돌검사를 통과한 것만 남겨 **관절이동이 가장 작은 순서**로 정렬한다 | §6 |
| 4 | 1등 자세로 이동해 캡처한다 | §7 |
| 5 | master 에 병합하고 메시를 다시 만든다 | §7 |
| 6 | 수렴했는지 본다. 아니면 1 로 돌아간다 | §8 |

방위각을 로봇이 만들지 않던 Phase 1 과 달리, 여기서는 로봇이 목표 지점을 직접 조준한다.
그래도 **턴테이블은 기본적으로 고정**이다(`nbv_theta_assist = False`). 턴테이블을 보조
자유도로 쓰는 경로가 있지만 기본값은 로봇만 움직이는 쪽이다.

---

## 3. sim 과 real — 무엇이 같고 무엇이 다른가

판단 로직은 **sim·real 이 같은 코드**다. `utils/nbv/` 아래의 frontier 검출, 자세 생성,
후보 순위, 수렴 회계를 두 백엔드가 그대로 부른다. 다른 것은 캡처 수단과, 아래 표의
네 가지다.

| | sim | real |
|---|---|---|
| 캡처 범위 | 목표 θ 중심 **부분 스윕**(`MMS_SIM_NBV_SPAN`, 기본 ±45°) | **전회전** — 짧은 스윕은 아직 배선 안 됨 |
| 병합 | GT θ 로 누적하므로 정합 불필요 | camera-motion `T_pre` + ICP (R3 경로) |
| SDK relocalization | 개념 없음 | **미검증·미배선**(T7). `nbv_use_relocalization=True` 여도 실제로는 R3 fallback 으로 간다 |
| 수렴 회계 기준 | 누적 **원시 점군**의 신규 점유 복셀 | master **메시 정점**의 신규 점유 복셀 (원시 점군이 iteration 사이에 남지 않음) |

**실물 통합에서 가장 불확실한 지점은 병합이다.** 자세를 어디로 보낼지는 sim 에서
검증됐지만, 로봇이 이동한 뒤 새 스캔이 기존 master 에 제대로 붙느냐는 실물에서만
확인된다. 지금 real 경로는 relocalization 없이 **camera-motion `T_pre` 로 초기값을 주고
ICP 로 다듬는** 방식 하나로만 동작한다(§7, T7).

---

## 4. 실행

### sim

```bash
./scripts/sim/run_e2e_gui.sh spray_can 2 planner    # Phase 1 → 2 (누적)
./scripts/sim/run_e2e_gui.sh detergent 2 planner
```

`phase_mode` 는 누적이라 `2` 는 "Phase 1 을 돌고 이어서 Phase 2"를 뜻한다. Phase 2 만
따로 돌릴 수는 없다 — 메울 구멍이 있어야 하기 때문이다.

| 변수 | 뜻 | 기본 |
|---|---|---|
| `MMS_SIM_NBV_K` | NBV 최대 반복 횟수 | 8 |
| `MMS_SIM_NBV_SPAN` | 목표 θ 중심 스윕 폭(°) | 90 |
| `MMS_SIM_NTHETA_P2` | Phase 2 캡처 프레임 수 | Phase 1 과 동일 |
| `MMS_SIM_DRY_EPS` | 패치 생산성 판정 임계(신규 복셀 비율) | 0.015 |
| `MMS_SIM_CONV_NEW_EPS` / `_AREA_N` | 전역 백스톱(신규 복셀 비율 / 연속 횟수) | 0.005 / 3 |
| `MMS_SIM_CONV_VOX` | 커버리지 판정 복셀 크기(m) | 0.004 |
| `MMS_SIM_ENSURE_ELS` | 반드시 한 번은 방문할 고도각(°) | 55 |
| `MMS_SIM_STAGE_DUMP` | 반복마다 메시·누적점군을 덤프할 디렉터리 | (끔) |
| `MMS_SIM_PROFILE_EVERY` | N 프레임마다 단계별 소요시간 출력 (0=끔) | 20 |

### 실물

`main_artec.py` 의 `BACKEND = "real"`, `MULTIPASS_SETTINGS.phase_mode = 2` 로 두고
실행한다.

```bash
env -u PYTHONPATH $MMS_PYTHON main_artec.py
```

> **첫 실물 시도는 `phase_mode = 1` 로 Phase 1 만 확인한 뒤 올린다.** Phase 2 는 실물에서
> 검증된 적이 없고, 로봇이 물체 옆에서 움직이므로 충돌 world 설정이 실제 셀과 맞는지부터
> 확인해야 한다(T2).

주요 real 설정(`ArtecMultiPassScanSessionSettings`)은 다음과 같다.

| 키 | 기본 | 뜻 |
|---|---|---|
| `nbv_K_max` | 12 | 최대 반복 |
| `nbv_distance_mm` | 225 | 목표 표면까지 카메라 거리(Artec 최적대역) |
| `nbv_el_floor_deg` | 30 | 관측 고도각 하한 (Phase 1 측면각 위에서만 보강) |
| `nbv_swept_steps` | 12 | 궤적 충돌검사 보간 지점 수 |
| `nbv_robot_speed_deg_s` | 24 | 이동 속도 |
| `nbv_sweep_deg` | 15 | 캡처 시 턴테이블 ±스윕(현재 미배선) |
| `nbv_theta_assist` | False | True 면 턴테이블을 보조 자유도로 사용 |
| `nbv_turntable_radius_mm` / `_body_height_mm` / `_collision_margin_mm` | 150 / 200 / 10 | 충돌 world 치수 — **실제 셀 측정치로 보정할 것** |
| `MMS_REAL_DRY_GAP` / `MMS_REAL_DRY_EPS` | 1 / 0.015 | gap 회계 on-off 와 생산성 임계 |

---

## 5. 부족면 검출 (frontier)

master IModel 을 base 프레임 점군으로 뽑고(`_master_to_pcd_B`), 법선을 추정해 Poisson
으로 메시를 만든다(`_build_master_mesh_B`, depth 6, 저밀도 정점은 4% 분위수로 잘라낸다).

그 메시에서 **삼각형 하나에만 속한 edge** 를 모으면 그것이 곧 표면의 경계, 즉 구멍이다
(`extract_frontier_candidates`). 각 후보는 대표점 `p`, 바깥 법선 `n`, 경계 길이 `L` 을
가진다. 너무 짧은 조각은 잡음이므로 버리고(`nbv_min_seg_length_mm = 6`), 너무 긴 것은
잘라 나눈다(`nbv_max_seg_length_mm = 60`).

Poisson depth 를 6 으로 낮게 잡은 것은 이 메시가 **판단용**이기 때문이다. 최종 산출물
메시는 후처리에서 따로 더 높은 depth 로 만든다(`6_postprocess.md`).

---

## 6. 후보 자세 선정과 순위

### 6.1 목표 자세 만들기

후보의 바깥 법선을 뒤집어 광축으로 삼으면 그 면을 정면으로 보는 카메라 자세가 된다
(`compute_camera_pose_from_normal`). 거리는 Artec 최적 작업거리 225mm 다.

`nbv_el_floor_deg = 30` 아래로 내려가는 자세는 만들지 않는다. 그 아래는 Phase 1 이 이미
훑고 지나간 측면각이라 새로 얻을 것이 없기 때문이다.

### 6.2 실행 가능성 — 궤적 전체를 검사한다

목표 카메라 자세를 EE 자세로 바꾸고 **해석 IK**(`xarm7_kinematics.ik`)로 푼다. 현재
관절각을 seed 로 주므로 가장 가까운 해가 나온다. SDK IK 는 쓰지 않는다(결정 2026-06 —
하드웨어 통신에 의존해 불안정하고 sim 과 공유할 수 없다).

**충돌 검사는 자세 하나가 아니라 `q_cur → q_des` 궤적 전체에 건다.** 관절공간을 선형
보간해 `nbv_swept_steps`(12) 지점마다 검사하고, 한 지점이라도 걸리면 그 후보를 버린다
(`swept_pose_collision`). 끝점도 포함되므로 단일 자세 검사를 대체한다.

Phase 1 은 로봇이 한 자세에 고정돼 있어 이런 검사가 필요 없었다. Phase 2 는 물체 옆을
지나가므로 **가는 길이 문제**다.

### 6.3 순위 — 관절이동이 가장 작은 것부터

```
cost = Σ_i w_i · (q_i − q_cur,i)²  −  δ · L̂        (δ = nbv_cost_delta = 0.3)
```

가중은 `DEFAULT_JOINT_WEIGHTS = [2, 2, 1.5, 1, 1, 1, 1]`(`utils/control/theta_planner.py`)
이다. base 쪽 큰 관절(J1·J2)이 비싸므로 같은 목표라면 **손목을 먼저 쓴다**. `L̂` 은 구멍
길이를 최대값으로 정규화한 것이라, 큰 구멍이 우선된다.

카메라 위치 이동량(`‖Δp_cam‖`)이나 턴테이블 회전량은 cost 에 넣지 않는다. 실제로 시간과
위험을 결정하는 것은 관절이 얼마나 도느냐이고, 그건 관절공간에서 바로 잴 수 있다.

---

## 7. 캡처와 병합

1등 자세로 로봇을 옮기고(`_move_robot_to_q`, 여기서도 충돌 게이트를 통과한다) 캡처한다.

**sim** 은 목표 θ 주변만 부분 스윕한다. 구멍이 있는 방향만 보면 되므로 한 바퀴를 다 돌
이유가 없다. 병합은 GT θ 로 누적하므로 정합 과정이 없다.

**real** 은 현재 전회전으로 찍고, 새 IScan 을 다음 변환으로 master 좌표에 올린다.

```
T_pre = T_BC_master · inv(T_BC_nbv)
```

`T_BC_master` 는 Phase 1 시점의 카메라-base 변환(= master 좌표의 정의)이고 `T_BC_nbv` 는
새 자세의 FK + hand-eye 다. 물체가 돌지 않았으므로 두 시점의 차이는 **카메라 이동뿐**
이다. 이 `T_pre` 를 초기값으로 ICP 를 한 번 더 돌려 다듬고(`hint_icp_refine_static`)
master 에 합친다(`_merge_into_master`).

설계상으로는 SDK relocalization 으로 이 변환 자체를 생략할 수 있지만(새 프레임이 곧바로
master 좌표로 들어온다) **아직 배선되지 않았다**(T7).

---

## 8. 종료 판정

### 8.1 boundary 로 판정하면 안 된다

경계 길이는 커버리지가 좋아질 때도 **늘어난다.** 새로 붙은 표면의 테두리가 그대로 경계로
잡히기 때문이다. GT 대조 실측(2026-08-19)이 이를 확인해 준다.

| | boundary | completeness@1mm |
|---|---|---|
| hand_drill | 519 → 800mm ("악화") | **37.9 → 52.7%** (개선) |
| mug | 533 → 586mm ("악화") | **33.1 → 37.7%** (개선) |

경계와 완전성의 상관은 r = +0.96 으로 높지만 **부호가 반대**다. 이걸 거꾸로 읽어
"Phase 2 가 머그를 악화시킨다"는 결론을 한동안 유지했었다(T5).

### 8.2 대신 "새 표면이 더 안 붙는가"를 본다

누적 점군을 4mm 복셀로 나눠 **이번 패치가 새로 점유한 복셀의 비율**을 잰다. 정보이득을
직접 재는 것이고 GT 가 없는 실물에서도 그대로 쓸 수 있다.

메시가 아니라 **원시 점군**에서 재야 한다. 메시에서 재면 Poisson 표면이 패치마다 미세하게
흔들려, 새로 본 것이 없어도 복셀이 바뀐다(실측 상관 r = +0.43 vs 원시 점군 +0.93).

### 8.3 전역 정지가 아니라 gap 단위 회계

GT 대조에서 NBV 의 개선은 **간헐적**이었다. 작은 구멍 패치가 두어 번 조용하다가 큰 구멍
패치가 한 번에 +3.3%p 를 올린다. 그래서 "연속 N 회 조용하면 종료"류의 전역 규칙은
손실 없는 설정이면 절약도 0 이었다.

지금은 비생산 패치의 **겨냥점 주변 40mm 만 후보에서 제외**한다
(`nbv_planner.report_patch`). 계획기가 후보를 소진하면 루프가 자연히 끝난다. 판정이
틀려도 잃는 것은 그 영역 하나뿐이고, 스캔이 통째로 일찍 끝나지 않는다.

생산성 임계는 신규 복셀 1.5% 다. 실측 분포(비생산 0.2~1.2% / 생산 1.7~10.3%)의 사이값인데
**물체 두 종으로 맞춘 값**이므로 T6 을 읽고 넓혀 검증할 것.

전역 백스톱(0.5% 미만이 3회 연속)은 병리 상황 대비로만 남겼다. 실측 최대 손실은 0.27%p 다.

---

## 9. sim 검증 — 두 층으로 나눠 본다

**GT 검증층** (sim 전용, 런타임에 개입하지 않는다) — 씬에서 정답 표면을 뽑아
(`extract_gt_mesh.py`) 표준 지표(completeness / accuracy / F-score / Chamfer)를 계산한다
(`eval_vs_gt.py`). **런타임 지표가 맞는지 검증하는 층**이며, 이 층이 없어서 §8.1 의
boundary 오독을 오래 몰랐다. 9종 결과 총람은 `docs/testset_results.md` 에 있다.

**런타임 판단층** (GT 없음, 실물과 동일) — §8.2 의 신규 점유 복셀로만 판단한다.

```bash
# GT 추출 (9종 일괄) — step2usd 환경
env -u PYTHONPATH ~/miniconda3/envs/step2usd/bin/python scripts/sim/extract_gt_mesh.py --all
# 단일 / 단계별 평가
python scripts/sim/eval_vs_gt.py --scan <obj> --gt scripts/sim/log/gt/<name>.npz
python scripts/sim/eval_vs_gt.py --stages <덤프dir> --gt ...
# 수렴 임계 검증 (MMS_SIM_STAGE_DUMP 산출물 필요)
python scripts/sim/validate_convergence.py --dirs scripts/sim/log/conv_val/*
```

Isaac 하니스 `sim_harness/MMS_ext_phase2_nbv.py` 는 Phase 1 을 낮은 고도각으로 돌려
**윗면에 일부러 구멍을 내고**(입사각 필터) Phase 2 가 그것을 검출·보강하는지 본다.

---

## 10. 코드 지도

```
utils/nbv/frontier.py                  # 경계 edge → 구멍 후보 (sim·real 공용)
utils/nbv/phase2_nbv.py                # pcd_to_mesh_poisson / detect_gaps /
                                       #   coverage_state / angular_coverage /
                                       #   nbv_pose_from_candidate / joint_motion_cost
utils/nbv/nbv_planner.py               # 후보 순위·dry gap 회계 (report_patch)
utils/nbv/manual_picker.py             # compute_camera_pose_from_normal
utils/nbv/icp_strategy.py              # icp_with_gates / pick_icp_roll
utils/collision/robot_collision.py     # swept_pose_collision / collision_free_ik
utils/robot/xarm7_kinematics.py        # 해석 IK/FK
utils/control/theta_planner.py         # θ assist 시 최소이동 θ (해석 IK 주입형)

mms_artec/nbv/artec_multipass_scan_session.py   ★ real Phase 2
    _build_master_mesh_B / _nbv_feasible_q / _rank_nbv_candidates
    _plan_nbv_pose / _capture_nbv_pose / _merge_into_master
    is_converged / _build_collision_world
mms_artec/backends/isaac/isaac_scan_session.py  # sim Phase 2 (_scan_patch 부분 스윕)
utils/nbv/scan_phase_controller.py              # Phase 1→2→3 순서·NBV 수렴 루프

scripts/sim/extract_gt_mesh.py / eval_vs_gt.py / validate_convergence.py
sim_harness/MMS_ext_phase2_nbv.py               # sim 검증 하니스
```

---

# 〔부록〕 Troubleshooting

### T1. 로봇이 이동을 거부한다 — "우회 경로 없음"

궤적 충돌검사(`swept_pose_collision`)가 `q_cur → q_des` 사이에서 걸린 것이다. 자세 자체는
안전해도 **가는 길**이 막히면 거부된다. 로그가 `start(...)` / `goal(...)` 로 시작하면 자세
자체가 불가한 경우이고, 그 외에는 우회 경로 계획까지 실패한 것이다.

`nbv_swept_steps` 를 키우면 더 촘촘히 보지만 느려진다. 실제 원인은 대개 충돌 world 치수가
실제 셀과 다른 것이다(T2).

### T2. 충돌 world 가 실제 셀과 다르다

`_build_collision_world` 는 턴테이블 캘리브(`T_BF0`)를 기준으로 원판과 몸체를 원기둥으로
근사한다. 치수는 설정값(`nbv_turntable_radius_mm = 150`, `_body_height_mm = 200`,
`_collision_margin_mm = 10`)이므로 **실제 셀을 재서 넣어야 한다.** 프레임 구조물이 더 있으면
`add_box` 로 추가한다.

**`T_BF0` 가 없으면 world 가 아예 구성되지 않고 충돌검사가 통째로 skip 된다.**
`[nbv] ⚠ turntable_transform/T_BF0 없음` 이 뜨면 그 상태로 로봇을 움직이지 말 것 —
캘리브레이션(`1_calibration.md`)부터 한다.

### T3. NBV 가 같은 자리를 계속 겨냥한다

도달 불가하거나 이미 촘촘한 영역이다. gap 회계(§8.3)가 비생산 패치의 겨냥점 주변 40mm 를
후보에서 빼도록 되어 있으니, 그게 도는지부터 본다 — real 은 `MMS_REAL_DRY_GAP=1`,
sim 은 `MMS_SIM_DRY_EPS` 가 관여한다. `[nbv] 신규복셀=…%` 로그가 안 보이면 회계가 꺼져
있거나 예외로 건너뛰고 있는 것이다.

### T4. 〔死조건〕 `is_converged` 의 boundary 임계는 발동한 적이 없다

`_p2.is_converged(cov, nbv_boundary_stop_mm/1000, nbv_coverage_tau)` 는 경계 총길이가
12mm 미만일 것을 요구하는데 실측값은 130~900mm 다. **한 번도 참이 된 적이 없다**(real 도
동일). 실제 종료는 §8.3 의 gap 회계와 `nbv_K_max` 가 낸다. 이 조건을 살리려면 임계를
실측 분포에 맞춰 다시 잡아야 한다.

### T5. 〔교훈〕 지표의 부호를 GT 로 한 번은 확인할 것

boundary 를 "작을수록 좋다"로 읽어 Phase 2 가 물체를 악화시킨다고 한동안 결론 냈었다.
단계별 메시를 덤프해 눈으로 보니 반대였다 — 떠 있던 파편이 본체에 연결되면서 경계가
늘어난 것이었다. GT 대조층(§9)이 없었으면 계속 몰랐을 것이다.

**런타임 지표는 그 자체로 옳다는 보장이 없다.** sim 에서만이라도 정답과 대조해 부호와
상관을 확인한 뒤 판단에 쓴다.

### T6. 〔한계〕 생산성 임계 1.5% 는 물체 2종으로 맞춘 값이다

실측 분포(비생산 0.2~1.2% / 생산 1.7~10.3%)의 사이값으로 정했고, 드릴에서 1패치를
아끼며 손실 0.13%p, 머그는 손실 0 이었다. 더 공격적인 설정은 머그에서 완전성 2~3%p 를
잃었다 — 개선이 간헐적이라 두 번 조용했다고 끝난 게 아니었기 때문이다.

같은 조건에서도 completeness 는 실행 간 ~5%p 편차가 있다. **그보다 작은 차이로 임계를
다투지 말 것.** 대상을 넓히려면 `validate_convergence.py` 를 쓴다.

### T7. 〔미배선〕 SDK relocalization 은 아직 안 쓰고 있다

설계상 Phase 2 의 1차 병합 경로는 Artec SDK 의 relocalization 이다. 로봇이 이동하면 일시적
tracking-lost 가 나고, 기존 스캔 표면을 다시 비추면 SDK 가 재고정해 프레임이 **곧바로
master 좌표로** 들어온다(`T_pre = I`, 병합 불필요). 노출 경로 후보는 둘이다.

- **R1** — Phase 1 의 `IScanningProcedure` 를 stop 하지 않고 유지한다. 이동 중에는 PREVIEW,
  재고정 후 RECORD 로 토글한다. `ArtecStreamingScanSession` 이 pass 끝에 stop 하지 않도록
  확장해야 한다.
- **R2** — `ScanningState.CONTINUE_RECORD` 로 master 를 입력 삼아 record 를 재개한다.
  SDK enum 은 있으나 문서상 비권장이라 동작 검증이 필요하다.

둘 다 **실기 검증 전**이라 현재 코드는 `nbv_use_relocalization = True` 여도 R3
(camera-motion `T_pre` + ICP, §7)로 간다. 실기에서 확인할 것은 relocalize 성공률, 재고정
소요시간, 필요한 overlap 임계다. relocalize 는 기존 표면과의 **overlap 이 있어야** 성립하는데,
NBV 는 구멍 가장자리를 보므로 대개 자연히 충족된다. 부족하면 스윕 각을 키우거나 standoff 를
조정한다.

정합 방식은 **HYBRID 여야 한다.** ICP-only 면 빈 디스크에 잘못 정합된다(`2_phase1.md` T1).

### T8. 지켜야 할 규약 몇 가지

- **로봇이 움직인다.** Phase 1 의 "로봇 고정" 불변식이 여기서 깨진다. 이동 전 궤적 전체
  충돌검사가 반드시 걸려야 하고, 충돌 world 가 없으면 검사가 skip 된다는 점을 기억할 것.
- **해석 IK 전용.** SDK IK 는 쓰지 않는다(2026-06 결정). θ planner 도 해석 IK 를 주입받는다.
- **물체는 돌지 않는다.** 병합은 카메라 이동만 반영하는 case ③ 이고 `R_phys = I` 다.
  Phase 3 의 flip hint(case ②)와 섞지 말 것.
- **점군에 O(N²) 파이썬 루프를 쓰지 않는다.** 입사각 필터의 `_pca_normals` 가 점마다 전수
  거리 계산과 3×3 고유분해를 돌려 프레임당 2초를 먹었고, Phase 2 한 패치가 4분을 넘겼다.
  KD-tree + 배치 `eigh` 로 바꿔 37배 빨라졌다. 판정 결과는 완전히 동일했다(입사각 50°
  통과 집합 불일치 0개).
- **최적화 전에 계측한다.** 위 건은 추측으로 두 번 헛짚었다(스캐너 해상도, 캐시 버그).
  `_tick`/`profile_report` 는 그대로 남겨 뒀으니 `MMS_SIM_PROFILE_EVERY` 로 켜서 쓴다.
  누적 타이머이므로 **리포트 간 증분**을 봐야 프레임당 비용이 나온다.
- **최적화했으면 동치 검증을 쓴다.** 결과값뿐 아니라 *실제 사용처의 출력*(최종 불리언
  마스크)까지 옛 구현과 비교한다.
