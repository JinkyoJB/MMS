# Phase 2 — 부족면 NBV 보강

> Phase 1 이 만든 5면에는 반드시 구멍이 남는다 — 윗면·옆면 경계의 grazing 영역, 옆면
> 사이 occlusion 띠, 손잡이나 오목부. Phase 2 는 그 구멍을 자동으로 찾아(frontier)
> 로봇이 그쪽을 조준해(NBV) 추가로 찍고 붙인다.
>
> 앞 단계 `2_phase1.md` · 바닥면 `5_phase3_merge.md`
>
> **쓰기만 하면 §1(실행)·§2(sim·real 차이)로 충분하다.** 문제가 생기면 부록(T1~T8)을 본다.
>
> ⚠ **실물에서 검증된 적이 없다.** 로봇이 물체 옆을 지나가므로 충돌 설정부터 확인할 것(T2).

---

## 1. 실행

Phase 2 만 따로 돌릴 수는 없다 — 메울 구멍이 있어야 하므로 `phase_mode` 는 누적이다.

```bash
./scripts/sim/run_e2e_gui.sh spray_can 2 planner    # Phase 1 → 2
```

| 환경변수 | 뜻 | 기본 |
|---|---|---|
| `MMS_SIM_NBV_K` | NBV 최대 반복 | 8 |
| `MMS_SIM_NBV_SPAN` | 목표 θ 중심 스윕 폭(°) | 90 (±45°) |
| `MMS_SIM_DRY_EPS` | 패치 생산성 판정(신규 복셀 비율) | 0.015 |
| `MMS_SIM_CONV_NEW_EPS` / `_AREA_N` | 전역 백스톱(비율 / 연속 횟수) | 0.005 / 3 |
| `MMS_SIM_ENSURE_ELS` | 반드시 한 번은 방문할 고도각(°) | 55 |
| `MMS_SIM_STAGE_DUMP` | 반복마다 메시·누적점군 덤프 디렉터리 | (끔) |
| `MMS_SIM_PROFILE_EVERY` | N 프레임마다 단계별 소요시간 (0=끔) | 20 |

**실물**은 `BACKEND = "real"` + `MULTIPASS_SETTINGS.phase_mode = 2` 로 두고 실행한다.
주요 설정(`ArtecMultiPassScanSessionSettings`):

| 키 | 기본 | 뜻 |
|---|---|---|
| `nbv_K_max` | 12 | 최대 반복 |
| `nbv_distance_mm` | 225 | 턴테이블 축에서 카메라까지 standoff |
| `nbv_el_floor_deg` | 30 | 관측 고도각 하한 (Phase 1 측면각 위에서만 보강). `el_need` clip 에도 쓰인다 |
| `nbv_swept_steps` | 12 | 궤적 충돌검사 보간 지점 수 |
| `nbv_theta_assist` | False | True 면 턴테이블을 보조 자유도로 |
| `nbv_turntable_radius_mm` / `_body_height_mm` / `_collision_margin_mm` | 150 / 200 / 10 | 충돌 world 치수 — **실제 셀 측정치로 보정할 것**(T2) |
| `MMS_REAL_DRY_GAP` / `MMS_REAL_DRY_EPS` | 1 / 0.015 | gap 회계 on-off, 생산성 임계 |

---

## 2. sim 과 real — 무엇이 같고 무엇이 다른가

판단 로직(`utils/nbv/` 의 frontier 검출·자세 생성·후보 순위·수렴 회계)은 **같은 코드다.**
다른 것은 캡처와 병합이다.

| | sim | real |
|---|---|---|
| 자세 선정 | gap 직접 겨냥 + 축-고도각 폴백 | **축-고도각만**(T6) |
| 캡처 범위 | 목표 θ 중심 **부분 스윕**(±45°) | **전회전** — 짧은 스윕 미배선 |
| 병합 | GT θ 누적이라 정합 불필요 | camera-motion `T_pre` + ICP (§5) |
| SDK relocalization | 개념 없음 | **미배선**(T5). `nbv_use_relocalization=True` 여도 실제로는 위 경로로 간다 |
| 수렴 회계 기준 | 누적 **원시 점군**의 신규 복셀 | master **메시 정점**의 신규 복셀 |

**실물 통합에서 가장 불확실한 지점은 병합이다.** 자세를 어디로 보낼지는 sim 이
담보하지만, 로봇이 이동한 뒤 새 스캔이 기존 master 에 제대로 붙느냐는 실물에서만
확인된다.

---

## 3. 한 바퀴에 무슨 일이 일어나나

Phase 1 은 로봇을 고정하고 턴테이블을 돌렸다. Phase 2 는 반대로 **물체가 서 있고 로봇이
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

방식이 **둘**이고 sim 은 둘 다, real 은 하나만 쓴다.

### 축-고도각 (`plan_nbv_elevation_pose`) — real 의 유일한 경로

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
4. 각 고도각에서 방위각을 `(0, ±30, ±60, ±90, 180)` 로 스윕하며 **이미 스캔한 (el, az)는
   건너뛰고**, IK 와 궤적 충돌을 통과한 것 중 관절이동이 가장 작은 것을 고른다.

```
cost = Σ_i w_i·(q_i − q_cur,i)²        (w = [2, 2, 1.5, 1, 1, 1, 1])
```

base 쪽 큰 관절이 비싸므로 같은 목표면 **손목을 먼저 쓴다.** `visited` 를 넘기는 것이
필수인데, 안 넘기면 직전 자세의 이동비용이 0 이라 **같은 자세를 무한 반복**한다(실측:
el 65 / az −30 을 4회 연속 선택, gap 18→18→20→19).

아랫면 gap(`n_z < −0.6`)은 후보에서 제외한다 — 관측으로는 못 메우고 Phase 3 flip 이
유일한 해법이다(`hw_layout.md` T4).

### gap 직접 겨냥 (`plan_frontier`) — sim 의 주경로

축-고도각은 gap 정보를 **"법선 고도각의 중앙값" 하나로 압축**하므로 gap 이 어디 있는지를
통째로 버린다. 손잡이나 내벽 같은 국소 결손은 원리적으로 겨냥할 수 없다(실측: 손잡이 0점).

그래서 sim 은 표면점 `p` 를 정면으로 보는 자세를 만든다. **턴테이블이 gap 을 로봇 앞으로
가져온다** — gap 의 방위각을 로봇이 편한 방위(`0, ±30°`)로 만드는 θ 를 구하고 그 θ 에서
겨냥하므로, 로봇은 고도각과 거리만 담당해 팔이 크게 움직이지 않는다. 법선 정면이 막히면
(작동거리보다 물체가 큰 오목면 등) 개구부 쪽으로 기울인다.

```
d(θ_t) = normalize(n̂·cos θ_t + ẑ·sin θ_t)      θ_t ∈ {0, 30, 45, 60, 75°}
```

sim 의 호출 순서는 ① `ensure_els` 미방문이면 축-고도각 → ② **gap 겨냥(주경로)** →
③ 폴백으로 축-고도각이다. **real 은 ②가 없다**(T6).

### 실행 가능성 — 궤적 전체를 검사한다

두 방식 모두 자세를 **해석 IK**(`xarm7_kinematics.ik`)로 풀고, `q_cur → q_des` 궤적을
`nbv_swept_steps`(12) 지점으로 나눠 전부 검사한다(`swept_pose_collision`). 한 곳이라도
걸리면 그 후보를 버린다. 끝점이 포함되므로 단일 자세 검사를 대체한다.

Phase 1 은 로봇이 고정이라 필요 없던 검사다 — 여기서는 **가는 길이 문제**다. SDK IK 는
쓰지 않는다(2026-06 결정, `4_collision.md` §3).

---

## 5. 캡처와 병합

1등 자세로 이동해(`_move_robot_to_q`, 여기서도 충돌 게이트를 통과) 캡처한다. sim 은 목표
θ 주변만 부분 스윕한다 — 구멍이 있는 방향만 보면 되므로 한 바퀴를 다 돌 이유가 없다.

real 은 현재 전회전으로 찍고 새 IScan 을 다음 변환으로 master 좌표에 올린다.

```
T_pre = T_BC_master · inv(T_BC_nbv)
```

`T_BC_master` 는 Phase 1 시점의 카메라-base 변환(= master 좌표의 정의), `T_BC_nbv` 는 새
자세의 FK + hand-eye 다. **물체가 돌지 않았으므로 두 시점의 차이는 카메라 이동뿐**이다
(`R_phys = I`. Phase 3 의 flip hint 와 섞지 말 것). 이 `T_pre` 를 초기값으로 ICP 를 한 번
더 돌려 다듬고(`hint_icp_refine_static`) master 에 합친다.

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

상관은 r=+0.96 으로 높지만 **부호가 반대**다. 이걸 거꾸로 읽어 "Phase 2 가 머그를
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

Isaac 하니스 `sim_harness/MMS_ext_phase2_nbv.py` 는 Phase 1 을 낮은 고도각으로 돌려
**윗면에 일부러 구멍을 내고** Phase 2 가 그걸 검출·보강하는지 본다.

---

## 8. 코드 지도

```
utils/nbv/frontier.py              # 경계 edge → 구멍 후보
utils/nbv/phase2_nbv.py            # pcd_to_mesh_poisson / detect_gaps / coverage_state
                                   #   nbv_pose_from_candidate / joint_motion_cost
utils/nbv/nbv_planner.py           # 후보 순위 · dry gap 회계(report_patch)
utils/nbv/manual_picker.py         # compute_camera_pose_from_normal
utils/nbv/icp_strategy.py          # icp_with_gates / pick_icp_roll
utils/collision/robot_collision.py # swept_pose_collision / collision_free_ik
utils/control/theta_planner.py     # DEFAULT_JOINT_WEIGHTS, θ assist 최소이동

mms_artec/nbv/artec_multipass_scan_session.py   ★ real Phase 2
    _build_master_mesh_B · _nbv_feasible_q · _rank_nbv_candidates
    _capture_nbv_pose · _merge_into_master · is_converged · _build_collision_world
mms_artec/backends/isaac/isaac_scan_session.py  # sim (_scan_patch 부분 스윕)

scripts/sim/{extract_gt_mesh,eval_vs_gt,validate_convergence}.py
sim_harness/MMS_ext_phase2_nbv.py
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
boundary 를 "작을수록 좋다"로 읽어 Phase 2 가 물체를 악화시킨다고 한동안 결론 냈었다.
단계별 메시를 덤프해 보니 반대였다 — 떠 있던 파편이 본체에 연결되며 경계가 늘어난
것이었다. **런타임 지표는 그 자체로 옳다는 보장이 없다.** sim 에서만이라도 정답과 대조해
부호와 상관을 확인한 뒤 판단에 쓴다.

### T5. 〔미배선〕 SDK relocalization 을 아직 안 쓴다
설계상 1차 병합 경로는 SDK relocalization 이다. 로봇이 이동하면 일시적 tracking-lost 가
나고, 기존 표면을 다시 비추면 SDK 가 재고정해 프레임이 **곧바로 master 좌표로** 들어온다
(`T_pre = I`). 노출 경로 후보는 둘이다.

- **R1** — Phase 1 의 `IScanningProcedure` 를 stop 하지 않고 유지. 이동 중 PREVIEW,
  재고정 후 RECORD 토글. `ArtecStreamingScanSession` 확장 필요.
- **R2** — `ScanningState.CONTINUE_RECORD` 로 master 를 입력 삼아 record 재개. SDK enum 은
  있으나 문서상 비권장이라 동작 검증 필요.

둘 다 실기 검증 전이라 현재는 §5 의 camera-motion `T_pre` 경로로 간다. 실기에서 볼 것은
relocalize 성공률·재고정 시간·필요 overlap 이다. relocalize 는 기존 표면과 **overlap 이
있어야** 성립하는데 NBV 는 구멍 가장자리를 보므로 대개 충족된다. 부족하면 스윕 각을
키우거나 standoff 를 조정한다. 정합은 **HYBRID 필수**(`2_phase1.md` T1).

### T6. 〔미배선〕 real 은 gap 을 직접 겨냥하지 못한다

`NbvPlanner.plan_frontier` 는 sim 에서만 불린다. real 의 `_plan_nbv_pose` 는 축-고도각
`plan()` 하나만 부르므로, **손잡이나 컵 내벽 같은 국소 결손을 원리적으로 겨냥할 수 없다**
(gap 위치가 "법선 고도각 중앙값" 하나로 압축된다). 실물에서 그런 부위가 안 메워지면 이것이
원인이다.

붙이는 일 자체는 크지 않다 — `plan_frontier` 는 공용 코드이고 real 이 주입해야 할 것은
look-at 자세를 푸는 콜백 하나(`solve_look_at_q` OpenCV 규약)와 목표 θ 를 캡처에 넘기는
경로다. 다만 real 캡처가 **아직 전회전**이라(§2) 목표 θ 주변만 도는 부분 스윕도 함께
배선해야 실익이 난다.

참고로 `_rank_nbv_candidates` 와 `nbv_pose_from_candidate` 도 per-gap 정면 방식의 잔재로
남아 있으나 **어디서도 호출되지 않는다**(2026-06-30 캡처 통일 때 대체됨). 코드를 읽을 때
그쪽을 현행으로 오해하지 말 것.

### T7. 〔死조건〕 `is_converged` 의 boundary 임계는 발동한 적이 없다
경계 총길이 12mm 미만을 요구하는데 실측은 130~900mm 다. **한 번도 참이 된 적이 없다**
(real 동일). 실제 종료는 gap 회계와 `nbv_K_max` 가 낸다. 살리려면 임계를 실측 분포에
맞춰 다시 잡아야 한다.

### T8. Phase 2 가 느리다 / 같은 자리를 계속 겨냥한다
- **느림** — 점군에 O(N²) 파이썬 루프를 넣지 말 것. 입사각 필터의 `_pca_normals` 가 점마다
  전수 거리 계산 + 3×3 고유분해를 돌려 프레임당 2초를 먹었고 한 패치가 4분을 넘겼다.
  KD-tree + 배치 `eigh` 로 37배 빨라졌고 판정 결과는 동일했다. 추측으로 두 번 헛짚었으니
  **최적화 전에 `MMS_SIM_PROFILE_EVERY` 로 계측**하고(누적 타이머라 리포트 간 **증분**을
  볼 것), 고친 뒤엔 최종 불리언 마스크까지 옛 구현과 대조할 것.
- **같은 자리** — gap 회계가 도는지 본다. `[nbv] 신규복셀=…%` 로그가 없으면 꺼져 있거나
  예외로 건너뛰는 중이다(real `MMS_REAL_DRY_GAP=1`).
