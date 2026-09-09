# Phase 2 — 부족면 NBV 보강 (5면 완벽 수집) · 설계안

> **상태: 🛠 구현 진행 중.** 코어(Step 1~3·6·7) 코드 완료·컴파일 통과, Step 1 단위검증 ✅.
> open3d 검증·전체 루프·relocalization(Step 4·5·8)은 MMS 런타임/Isaac/실기 필요 (§9).
> 상위 맥락 `docs/main_flow.md` §2(Phase 1)·§5(NBV 루프). Phase 1 흐름은 `docs/2_phase1.md`.
> IK 규약: 해석 운동학 전용(`docs/main_flow.md` §1, 결정 2026-06 — SDK IK 미사용).

---

## 0. 한 줄 요약

Phase 1(턴테이블 1회전·EE 고정 → 5면)이 만든 **부분 점군에서 부족/미관측 영역을
자동 검출(frontier)** 하고, 그 영역을 **로봇이 능동적으로 조준(NBV)** 해 추가 캡처·병합하는
루프를 돌려 **5면 데이터를 완벽(watertight 근접)** 하게 만든다.

- **Phase 1 = 고정** (변경 없음): 로봇 home 고정, 턴테이블 360° streaming SLAM → 5면.
- **Phase 2 = 본 설계로 재정의**: 로봇이 **움직이며** 부족면을 NBV 로 메운다.
- 바닥면(180° flip)은 본 Phase 2 의 범위가 아님 → **Phase 3 로 분리**(§8).

---

## 1. 무엇이 바뀌나 (현재 → 새 설계)

| | 현재 §3 "Phase 2" | 새 Phase 2 (본 문서) |
|---|---|---|
| 목적 | 바닥면 추가 (180° flip) | **5면 중 부족 영역 보강** |
| 트리거 | 사용자가 손으로 물체 회전 + [Enter] | **자동** (frontier 검출 → NBV) |
| 로봇 | 고정 (회전은 사용자) | **로봇이 부족면을 조준해 이동** |
| 턴테이블 | 360° 재회전 | v1 고정(robot-only). 옵션 θ-assist 시에도 **선택기준=관절이동 최소** |
| 병합 | centroid-pivot R_phys hint (case ②) | **SDK relocalization (T_pre=I)**, 실패시 camera-motion T_pre+ICP |
| 종료 | 사용자 [q] / max_passes | **커버리지 수렴**(plateau·boundary→0·각도 커버리지) |

> 현재 §3 의 flip 흐름(`pose_physical_rotations`, case ② hint)은 **그대로 보존**하고
> Phase 3(바닥면)로 라벨만 옮긴다. 본 Phase 2 는 **새 모드**로 추가 (`phase_mode=2`).

---

## 2. 핵심 통찰 — PhoXi NBV 루프가 이미 완성형

`mms_phoxi/nbv/scan_session.py::ScanSession` 의 **Phase 2** 가 정확히 우리가 원하는 루프다:

```
frontier 추출 → 후보 rank(cost) → execute(로봇+턴테이블 이동) → 캡처 → ICP → integrate → terminate
```

본 설계 = 이 검증된 루프를 **Artec 백엔드로 포팅**. 알고리즘은 동일, **3가지만 Artec 에 맞게 교체**:

1. **누적 표현**: PhoXi=TSDF/`PcdAccumulateVolume` → Artec=master IModel(`_master_to_pcd_B`).
2. **IK**: PhoXi=SDK IK(`robot.arm.get_inverse_kinematics`) → **해석 IK + 충돌검사**(결정 2026-06).
3. **캡처/병합**: PhoXi=절대 `T_CO` 한 프레임 정합 → Artec=**SDK relocalization 으로 master 좌표 직접 정합**
   (T_pre=I, §3.4.1), 실패 시에만 camera-motion T_pre+ICP fallback.

---

## 3. 알고리즘 (NBV 루프)

### 3.0 입력
Phase 1 master IModel (5면, 구멍 있음). `_master_to_pcd_B(master, T_CB, voxel)` → **B 프레임 컬러 pcd**.

### 3.1 부족면 검출 (frontier)
1. master pcd → normal 추정 → **Poisson / ball-pivoting** 으로 mesh (`_build_master_mesh_B`, 신규).
2. `extract_frontier_candidates(mesh, min_seg_vertices, min_seg_length, max_seg_length)`
   → `[FrontierCandidate(p_O, n_O, L, vids)]`. 각 후보 = 한 구멍/경계 세그먼트의
   대표점 `p`, 바깥 법선 `n`, 길이 `L`. (`utils/nbv/frontier.py` 그대로 재사용.)

> frontier = "삼각형 하나에만 속한 edge"(=메쉬 경계). 큰 구멍·미관측 면이 긴 경계로 잡힌다.
> ★ Phase 1 은 위·옆을 잘 덮으므로 frontier 는 보통 **윗면-옆면 경계의 grazing 영역,
> 옆면 사이 occlusion 띠, 손잡이/오목부** 에 집중된다.

### 3.2 후보 → NBV 목표 포즈
각 후보에 대해 (`compute_camera_pose_from_normal`, `utils/nbv/manual_picker.py` 재사용):

```
T_CB_des = compute_camera_pose_from_normal(p_B, n_B, distance=225mm, roll)
```
- 광축 `z_cam = -n` → 카메라가 표면을 정면으로 봄. 거리 = Artec 최적대역 **225mm**(Phase 1 view-score 와 동일 `d*`).
- roll 은 `pick_icp_roll`(직전 포즈와 자세 연속성) 또는 0.

### 3.3 실행 가능성 + 관절이동-최소 rank (★ 해석 IK)
**목표 = 로봇 관절 이동 최소 + 충돌-free.** (턴테이블 θ 를 최소화하는 게 아니다.)
후보마다:
1. **충돌-free 해석 IK**: 후보 목표 카메라 포즈 `T_CB_des` → `T_EB` → **해석 IK**
   (`xarm7_kinematics.ik`, 현재 q 로 warm-start → 가장 가까운 해 branch) → **충돌검사**
   (`collision_free_ik`/`pose_collision`) 통과 q. 실패 후보는 제외.
   - **v1 = 턴테이블 고정**(robot-only): 목표 포즈에서 바로 IK+충돌. θ 탐색 없음.
   - (옵션) `nbv_theta_assist`: 턴테이블을 **보조 DOF** 로 θ 그리드 탐색 — 단 선택기준은
     **여전히 관절이동 최소**(θ 자체를 줄이는 게 아님). `plan_min_motion_theta_analytic` (§6.2).
2. **관절이동 cost** = `Σ_i w_i·(q_i − q_cur,i)²  − δ·L̂`. base 관절(J1·J2) 가중↑
   → 같은 목표면 **손목 우선**(큰 관절 적게). 큰 구멍(L↑) 우선. Cartesian `‖Δp_cam‖`·턴테이블
   `Δθ` 항은 **쓰지 않음**(관절공간이 직접 목표). q 는 1 의 IK 결과를 재사용(추가비용 0).
3. feasible(IK+충돌 OK) 후보를 cost 오름차순 → **최소 관절이동 후보부터** 방문.

> ✅ **충돌-free 범위 = 궤적 전체(swept-path)** (사용자 결정). `q_cur → q_des` 를 관절공간
> 선형 보간(`set_servo_angle`=MoveJ 근사)해 `nbv_swept_steps`(기본 12) 지점마다
> `pose_collision` 검사 — 한 지점이라도 충돌하면 그 후보 reject (`swept_pose_collision`,
> `utils/collision/robot_collision.py`). endpoint 도 포함하므로 단일 자세검사를 대체.

### 3.4 캡처 (Artec) — **확정: (B) streaming + SDK relocalization**
NBV pose 로 이동 후 **로봇 고정 + 턴테이블 소각도 스윕**으로 짧은 스트리밍. 핵심은
**Artec SDK relocalization** 으로 새 시점을 기존 master 표면에 자동 재정합(§3.4.1)하는 것.

- **(B) streaming IScan [확정]**: 로봇을 T_CB_des 로 이동(`_move_robot_to_T_CB`) →
  로봇 고정 + 턴테이블 ±Δθ(예 ±15°) 짧은 record. 새 시점이 **이미 스캔된 표면과 overlap**
  하므로 SDK 가 relocalize → 프레임이 **master 좌표로 직접** 들어옴.
  - 장점: 기존 IModel·texture·watchdog/recovery 파이프라인 그대로. relocalize 성공 시 **별도 병합/T_pre 불필요**.
  - 주의: relocalize 는 기존 표면과의 **overlap 필수**. NBV 는 구멍 가장자리(=기존면 인접)를 보므로 자연 충족.
    overlap 부족·실패 시 §3.4.1 R3 fallback.
- **(A) 단일 patch [대안]**: relocalize 불가한 깊은 오목부에서 FK+hand-eye 단일캡처 + ICP.

#### 3.4.1 Relocalization 원리 & 모드 (★ 본 요청 반영)
Artec real-time 정합(HYBRID)은 들어오는 frame 을 **누적 reconstruction 에 맞춰 등록**한다.
로봇이 NBV 로 이동하면 일시적 tracking-lost → 다시 **기존 스캔 표면을 비추면 relocalize**(재고정)
→ 같은 좌표계로 frame 누적 재개. 즉 §6 의 tracking-lost→recovery 경로가 여기선 **정상 동작(설계상 기대됨)**.

| 모드 | 방법 | 비고 |
|---|---|---|
| **R1 단일 연속 procedure [권장]** | Phase 1 의 `IScanningProcedure` 를 **stop 하지 않고** 유지. NBV 이동 = 의도된 transient loss → relocalize → resume. | T_pre=I, 병합 없음. `ArtecStreamingScanSession` 이 pass 끝에 stop 안 하도록 확장 필요. |
| **R2 CONTINUE_RECORD** | Phase 1 stop(master 회수) 후, master 를 입력으로 record 재개(relocalize). | `ScanningState.CONTINUE_RECORD`(SDK enum 존재, 문서상 비권장) — 래퍼·동작 검증 필요(§6.5). |
| **R3 fallback** | relocalize 실패(overlap 부족) 시 §3.5 의 camera-motion `T_pre`(case ③)+ICP 로 수동 병합. | 안전망 — relocalize 없이도 동작 보장. |

> ⚠ SDK relocalization 의 정확한 노출 경로(R1 stop-억제 vs R2 CONTINUE_RECORD)는 구현 전
> **실기 검증 필요**(§6.5 / §9). 어느 쪽이든 실패 시 R3 가 받쳐 루프는 항상 완결.

### 3.5 병합
- **relocalize 성공(R1/R2)**: 프레임이 이미 master 좌표 → **병합 불필요**(T_pre=I). re-mesh 만.
- **relocalize 실패(R3 fallback)**: 새 IScan ← `T_pre = T_BC_master·inv(T_BC_nbv)`(case ③) 좌측곱
  → (옵션) `hint_icp_refine_static`(init=T_pre) → `_merge_into_master`.
- 캡처 후 master→pcd→mesh 재생성 → §3.1 로 루프.

### 3.6 종료 (커버리지 수렴) — PhoXi `_terminate` 재사용 + 각도 커버리지
1. `K_max` 도달, 또는
2. **plateau**: 최근 N 스텝 표면적/점수 증가율 < thresh, 또는
3. **boundary 길이 → 0**: `extract_boundary_edges` 총길이 < thresh (watertight 근접), 또는
4. **(추가) 각도 커버리지**: Fibonacci 구 위 view-direction 집합 중, 관측 법선이 그 방향과
   거의 평행한 비율 ≥ τ (예 0.95) → "5면 모든 방향 충분히 봤다". = "완벽" 정량 기준.

---

## 4. 좌표·병합 규약 (relocalization 1차 / case ③ fallback)

Phase 2 는 **객체는 안 돌고 카메라(로봇)만 움직임**.

- **1차 = SDK relocalization**: 새 frame 이 기존 reconstruction 에 재고정 → **SDK 가 좌표정합 수행**,
  우리 쪽 변환 불필요(T_pre=I). (§3.4.1)
- **fallback = main_flow §4.3 경우 ③** (relocalize 실패 시):

```
fallback:  T_pre = T_BC_master · inv(T_BC_nbv)     # IScan frame 좌측곱 (case ③)
(A) 대안:  p_B = T_BC_nbv · p_C  (단일캡처)  →  ICP(target = master pcd)
```
- `T_BC_master` = Phase 1 기준 카메라-base (master 좌표 정의). `T_BC_nbv` = NBV pose 의 FK+hand-eye.
- case ② (R_phys) hint 와 **충돌 없음**: Phase 2 에선 `R_phys=I`.

---

## 5. 재사용 부품 (대부분 존재 ♻️)

| 부품 | 위치 | 역할 |
|---|---|---|
| `extract_frontier_candidates` / `extract_boundary_edges` | `utils/nbv/frontier.py` | 구멍·경계 검출 ♻️ |
| `compute_camera_pose_from_normal` | `utils/nbv/manual_picker.py` | 법선→카메라 목표포즈 ♻️ |
| `icp_with_gates` / `pick_icp_roll` | `utils/nbv/icp_strategy.py` | patch ICP 정합 + roll ♻️ |
| `_master_to_pcd_B` / `_iscan_to_pcd_B` | multipass session | master→B프레임 pcd ♻️ |
| `_move_robot_to_T_CB` / `_recapture_T_BC` | multipass session | 로봇 이동 + FK 재캡처 ♻️ |
| `hint_icp_refine_static` / `_merge_into_master` | multipass session | R3 fallback 병합 ♻️ |
| `ArtecStreamingScanSession` / 4-watchdog / `_attempt_recovery` | mms_artec/nbv | **relocalization 경로**(전환-loss→재고정) 재사용 ♻️ |
| `ScanSession`(PREVIEW/RECORD/STOP, `CONTINUE_RECORD`) | `mms_artec/sensor/artec_scanning.py` | relocalization 노출(확장 대상, §6.5) |
| `collision_free_ik` / `pose_collision` | `utils/collision/robot_collision.py` | **해석 IK 충돌-free 자세** ♻️ |
| `ik` / `fk_pose6d` | `utils/robot/xarm7_kinematics.py` | **해석 IK/FK** ♻️ |
| `ScanSession._rank_candidates` / `_step` / `_terminate` | `mms_phoxi/nbv/scan_session.py` | **포팅 원본(패턴)** ♻️ |
| `ProgressVisualizer` | `utils/nbv/_progress_vis.py` | 진행 뷰어 ♻️ |

신규로 작성할 부분은 **오케스트레이션 + Poisson mesh + 해석-IK θ planner** 뿐.

---

## 6. 코드 수정 지점 (정확히)

### 6.1 `mms_artec/nbv/artec_multipass_scan_session.py` (주 변경)
- **`ArtecMultiPassScanSessionSettings`**: Phase-2 NBV 파라미터 추가
  `phase_mode: int = 3`(**순차 누적**: 1=Phase1 / 2=1→2(NBV) / 3=1→2→3(flip); 기본 3), `nbv_distance_mm=225`,
  frontier 3종(`min_seg_vertices/min_seg_length/max_seg_length`), cost 가중(`α,β,γ,δ`),
  `nbv_K_max`, `nbv_plateau_window/thresh`, `nbv_boundary_stop_mm`, `nbv_coverage_tau`,
  `nbv_capture_mode: "stream"|"patch" = "stream"`, `nbv_sweep_deg=15`,
  `nbv_robot_speed_deg_s`, `nbv_theta_n_samples`.
- **`run()`** (정상완료 분기, 순차 누적): `phase_mode>=2` 면 Phase 1 pose 0 완료 후
  사용자-flip advance 대신 **`self._phase2_nbv_loop(master_model)`** 호출.
- **신규 메서드**:
  - `_phase2_nbv_loop(master)` — §3 루프 본체 (PhoXi `ScanSession.run` Phase 2 포팅).
  - `_build_master_mesh_B(master)` — `_master_to_pcd_B` → normal → Poisson → mesh.
  - `_rank_frontier_candidates(cands, mesh)` — §3.3 (**해석-IK θ planner** 사용).
  - `_capture_nbv_stream(T_CB_des)` — §3.4(B): 이동→(relocalize 대기: 프레임 OK·reg_err≥0 재확인)
    →턴테이블 ±Δθ 스윕 record. **relocalize 성공 시 master 좌표 직접 누적(병합 없음)**,
    실패 시 `_recapture_T_BC`→camera-motion T_pre + `hint_icp_refine_static`→`_merge_into_master`(R3).
  - `_phase2_terminate(state)` — §3.6.
  - `_coverage_metric(mesh)` — §3.6-4 각도 커버리지.
- **재사용**: `_master_to_pcd_B`, `_move_robot_to_T_CB`, `_recapture_T_BC`, `_merge_into_master`.

### 6.2 `utils/control/theta_planner.py` (IK 교체 — ★중요)
현재 `plan_min_motion_theta` 가 **SDK IK** `robot.arm.get_inverse_kinematics`(line 126) 하드코딩.
- **수정**: `ik_fn` 파라미터 주입(또는 `plan_min_motion_theta_analytic`) → 내부에서
  `xarm7_kinematics.ik(pose6d, seed)` + `pose_collision` 검사. 기본은 해석 IK.
- 사유: 결정 2026-06 — SDK IK 는 하드웨어 통신 의존 → 불안정. real·sim 공용 해석 IK.

### 6.3 `utils/nbv/frontier.py` (소폭, 선택)
- 그대로 사용. (선택) 구멍 **면적** 가중 후보 점수 보강 — 필요 시.

### 6.4 sim 검증 하니스 (신규) — `sim_harness/MMS_ext_phase2_nbv.py`
- Phase 1 GT 누적(`MMS_ext_phase1.py` 패턴)으로 5면 pcd 생성하되 **일부 영역 의도적 구멍**
  (특정 θ skip / occluder) → frontier 검출 → NBV 목표포즈 → **해석 IK+충돌** →
  Isaac 로봇 이동(`look_at_camera`) → 그 자세 GT 캡처 → patch 병합 → **구멍 메워짐 검증**.
- ★ 연산·로봇제어는 real 과 공유(sim 중복구현 금지) — sim 은 USD/렌더/GT 비교만.

### 6.5 Artec relocalization 노출/검증 (★ 본 요청)
- `mms_artec/sensor/artec_scanning.py` (`ScanSession`/`ScanSessionSettings`): 현재 PREVIEW/RECORD/STOP
  + HYBRID + `ignore_registration_errors` 만 노출. relocalization 적용 위해 확인·확장:
  - **R1**: `ArtecStreamingScanSession` 가 pass 종료 시 `stop()` 하지 않고 **procedure 를 살려두는**
    모드 추가(Phase 1→Phase 2 동안 동일 `IScanningProcedure` 유지). 로봇 이동 중엔 PREVIEW,
    재고정 후 RECORD 토글 고려.
  - **R2**: `ScanningState.CONTINUE_RECORD` 경로가 **기존 master 에 relocalize** 하는지 실기 검증
    (문서상 비권장 — 동작·안정성 확인 후 채택).
  - HYBRID 정합 필수(ICP-only 면 빈 디스크 오정합, §2_phase1). overlap 충분하도록 NBV pose 가
    구멍 가장자리를 보게 유지.
- **검증 산출물**: relocalize 성공률 / 재고정 소요시간 / overlap 임계. 실패율 높으면 R3(fallback) 비중↑.

### 6.6 문서
- `docs/main_flow.md` §3 → "Phase 3(바닥면 flip)" 로 라벨 이동, §5 NBV 루프 → 본 Phase 2 로 흡수.
- 본 `docs/3_phase2.md` = 단계 상세 (확정본).

---

## 7. 규약·함정

1. **로봇이 움직인다** (Phase 1 불변식 깨짐) — 매 이동 전 **궤적 전체 충돌검사**
   (`swept_pose_collision`: q_cur→q_des 관절보간 N지점). 자세필터(endpoint)는 그 부분집합.
   천천히 이동(`wait=True`). ⚠ 단, 충돌 world(`_collision_world`)가 구성돼 있어야 작동
   (turntable 표면+프레임 box; 미구성 시 검사 skip → §9 미해결).
2. **해석 IK 전용** — SDK IK 금지(결정 2026-06). θ planner 도 해석 IK 로.
3. **camera-motion case ③** — 객체 안 돎(`R_phys=I`). hint case ② 와 혼용 금지.
4. **relocalization = 의도된 transient loss** — 로봇 이동 시 lost 는 오류가 아니라 설계 동작.
   기존 표면과 **overlap** 이 있어야 재고정됨(NBV 가 구멍 가장자리를 봐서 충족). 실패 시 R3 fallback.
   ★ HYBRID 정합 필수, overlap 부족하면 Δθ↑/standoff 조정.
5. **종료 = 커버리지 수렴** — boundary→0 + 각도 커버리지 τ. "면적 plateau" 만으론 구멍 잔존 가능.
6. **순차 누적** — `phase_mode=N` → Phase 1..N 순서대로. 기본 3(=1→2→3 전 파이프라인).
   Phase 2→3 전환 시 NBV 로 움직인 robot 을 `go_home` 후 flip. 첫 real 은 1→2→3 단계적 테스트.

---

## 8. 범위 / Phase 재구성 제안

```
Phase 1  턴테이블 360° · EE 고정          → 5면 (변경 없음)         ✅
Phase 2  부족면 NBV 보강 (로봇 능동 이동)  → 5면 완벽               🔬 (본 문서)
Phase 3  180° flip · 바닥면 (기존 §3)      → 6면 / full-coverage    ♻️ (라벨만 이동)
```
바닥면을 본 Phase 2 NBV 로 같이 처리할지는 **분리 권장**(디스크 접촉면은 물리 flip 필요 →
frontier 로 안 풀림). 단, flip 후의 잔여 구멍은 Phase 2 NBV 를 **재실행**해 메울 수 있음(재사용).

---

## 9. 구현 단계 (진행 상황)

1. [x] **`theta_planner.plan_min_motion_theta_analytic`** (`ik_fn`+`collision_fn` 주입, 해석 IK).
   검증 ✅ `scripts/phase2/verify_theta_analytic.py` (라운드트립·min-motion·충돌필터).
   ★ 부수: **kin euler 규약 ≠ scipy `as_euler('xyz')`** 발견 → `pose_from_T` 기본을 kin 규약으로.
2. [x] **`utils/nbv/phase2_nbv.py`** — `pcd_to_mesh_poisson`/`detect_gaps`/`coverage_state`/
   `angular_coverage`/`nbv_pose_from_candidate`/`joint_motion_cost`. 컴파일 ✅.
   검증(open3d) ⏳ `scripts/phase2/verify_phase2_core.py` (런타임에서 실행).
3. [x] **`_rank_nbv_candidates`** (관절이동-최소 cost `Σwᵢ(qᵢ−q_cur)²`) + **`_nbv_feasible_q`**
   (해석 IK + **궤적 전체 swept-path 충돌**) + **`_build_collision_world`**(turntable calib).
   검증 ✅ `scripts/phase2/verify_swept_collision.py` (경로 중간 관통 검출).
4. [ ] **Artec relocalization 실기 검증**(§6.5: R1 stop-억제 vs R2 CONTINUE_RECORD) — 성공률·재고정시간.
5. [~] **`_capture_nbv_pose`** — v1: 해석IK→`set_servo_angle` 관절구동 + 기존 streaming(풀회전)
   + camera-motion T_pre(R3) 병합. ⏳ 짧은 스윕·relocalize(R1/R2)는 4 이후 배선.
6. [x] **`_phase2_nbv_loop`** + `_build_master_mesh_B` + 수렴판정(`is_converged`). 컴파일 ✅.
7. [x] **`run()` 분기** (`phase_mode` 순차 누적 1→2→3, Phase2→3 go_home). 컴파일 ✅.
8. [x] **`MMS_ext_phase2_nbv.py`** sim 하니스 (Isaac, `standalone_examples/play/MMS/`).
   Phase1→Phase2 연속. gap = **입사각 필터**(저앙각 측면뷰→윗면 grazing 미스캔). Phase2 =
   윗면 gap 검출→NBV 고앙각 포즈(해석 IK+swept 충돌+관절이동 최소)→이동→재캡처→메움 판정.
   문법 ✅, gap 메커니즘 단위검증 ✅(저앙각 윗면 keep 0.00 / 고앙각 1.00). 전체 실행 = Isaac 런타임.
   ★ Isaac `utils` 충돌로 자기완결 모듈(kin/geo)만 로드, 입사각·gap·NBV·swept는 인라인 미러.
9. [ ] real 1회 검증 → `docs/main_flow.md` 반영(§3→Phase3, §5 흡수).

> **현재**: 1·2·3·6·7 코드 완료(컴파일 통과). 단위검증 ✅ Step 1(theta)·swept-path 충돌.
> 2 의 open3d 검증·전체 루프는 MMS 런타임(open3d)·Isaac·실기 필요. 5 는 v1(R3) 골격,
> relocalization(4)·짧은스윕은 후속.
> ⚠ 실기 확인: `set_servo_angle` speed 단위 · 짧은스윕 streaming 회전각 override ·
> **충돌 world 치수**(`nbv_turntable_radius_mm`/`body_height`/`margin`은 실제 셀 측정치로 보정,
> 필요시 프레임 box `add_box` 추가). world 미구성 시 swept 검사 skip 되니 calib 필수.
```

---

## 10. 성능 — 캡처 간 지연 추적 (2026-08)

실물 밀도(240 프레임/rev)로 올리자 Phase 2 한 패치에 **4분 이상**이 걸렸다.
120 으로 낮춰도 그대로였다. 원인 규명 과정을 남긴다 — 결론보다 **방법**이 중요하다.

### 10.1 추측으로 두 번 헛짚음

| 시도 | 근거 | 결과 |
|---|---|---|
| 스캐너 해상도 1280×960 → 384×288 | "렌더가 비쌀 것" | 효과 미미 (`cam` 은 2%) |
| `_master_ds()` 캐시 수정 | "매 프레임 전체 vstack" | 실제 버그였지만 병목 아님 |

두 번 다 사용자가 수 분짜리 재실행을 했고, 안 고쳐지자 "수정이 반영은 됐나"라는
잘못된 갈래로 빠졌다. → 메모리 `profile-before-optimizing`.

### 10.2 계측 — 캡처 루프 단계 분해

`_scan_patch` 안에 단계별 `self._tick()` 을 넣고 `MMS_SIM_PROFILE_EVERY`(기본 20)
프레임마다 `profile_report()` 를 찍었다. **누적 타이머이므로 리포트 간 증분**을 봐야
프레임당 비용이 나온다. 또 `capture` 는 부모 타이머라 `cam/crop/incidence` 를
중복 계상한다 — 합계로 착각하면 안 된다.

```
소요시간: └incidence 115.7s(65%), capture  36.8s(21%), ...   ← 프레임 20
소요시간: └incidence 155.6s(59%), capture  78.3s(30%), ...   ← 프레임 40
소요시간: └incidence 197.7s(56%), capture 122.1s(35%), ...   ← 프레임 60
```

20프레임당 증분 → **프레임당**: `incidence` 2.0s · `crop` 0.06s · `cam` 0.03s ·
`turntable` 0.05s. **입사각 필터 하나가 95%.**

### 10.3 원인 — `_pca_normals` 가 O(N²) 파이썬 루프

```python
for i in range(N):              # N=8067
    d = pts - pts[i]            # 매 반복 8067×3 배열
    idx = np.argpartition(...)  # 매 반복 전수 정렬
    _, v = np.linalg.eigh(...)  # 점당 eigh 1회
```

6500만 회 거리 계산 + 8067회 3×3 고유분해. Phase 1 보다 Phase 2 가 느린 이유도
같다 — 커버리지가 넓어질수록 크롭 점 N 이 늘고 비용은 N² 로 는다.

### 10.4 수정 — KD-tree + 배치 eigh

`scipy.spatial.cKDTree.query(..., workers=-1)` 로 전 점의 k-이웃을 한 번에 얻고,
`np.linalg.eigh` 를 (N,3,3) 스택에 배치 적용. 청크(`_PCA_CHUNK=40000`)로 끊어
이웃 버퍼 상한을 유지한다(메모리 폭주 방지).

**동치 검증** — 실제 파일에서 `ast` 로 함수를 뽑아 옛 구현과 같은 입력에 돌렸다.
결과뿐 아니라 *실제 사용처의 출력*(입사각 50° 통과 집합)까지 비교:

| N | 구 | 신 | 가속 | 법선 일치 | 필터 통과 불일치 |
|---|---|---|---|---|---|
| 8067 | 1.092s | 0.029s | **37배** | 100% | **0개** |

### 10.5 규약

- 점군 O(N²) 파이썬 루프는 금지. KD-tree(scipy/open3d) + 배치 선형대수로.
  `phase1_viewpoint.estimate_outward_normals` 는 open3d C++ KD-tree 라 문제없다.
- 최적화 전 **동치 검증**을 먼저 쓴다. 필터/판정 함수는 최종 불리언 마스크까지 비교.
- 병목 추적이 끝나도 `_tick` 은 남겨 둔다. `MMS_SIM_PROFILE_EVERY=0` 으로 끈다.

---

## 11. 수렴 지표 — boundary 를 반대로 읽고 있었다 (2026-08-19)

### 11.1 발견

Phase 2 가 hand_drill 경계를 519→800mm 로 "악화"시키는 듯했다. 단계별 메시를
덤프해 눈으로 보니 반대였다 — 분리돼 떠 있던 파편 3개가 배터리부로 **연결·완성**
되고 있었다. 씬의 정답(GT) 표면과 대조하니 확정:

| | boundary | completeness@1mm |
|---|---|---|
| hand_drill | 519 → 800 ("악화") | **37.9 → 52.7%** (개선) |
| mug | 533 → 586 ("악화") | **33.1 → 37.7%** (개선) |

**boundary 는 커버리지가 넓어질 때도 오른다** — 새로 붙은 표면의 테두리가 그대로
경계로 잡힌다. completeness 와 상관은 r=+0.96 으로 높지만 **부호 해석이 반대**였다.
"머그에서 Phase 2 가 악화시킨다"는 이전 결론은 전부 이 오독이었다.

### 11.2 두 층 구조

**GT 검증층** (sim 전용, 런타임 불개입): `scripts/sim/extract_gt_mesh.py` 로 씬에서
정답 표면을 뽑고 `eval_vs_gt.py` 로 표준 지표(completeness/accuracy/F-score/Chamfer)
계산. **런타임 지표가 맞는지 검증·보정하는 용도** — 이 층이 없어서 boundary 오독을
지금까지 몰랐다. 결과 총람: `docs/testset_results.md`.

**런타임 판단층** (GT 없음, 실물 동일): 수렴은 **누적 원시 점군의 신규 점유 복셀**
(정보이득, NBV 문헌 표준)로 판정한다. 메시에서 재면 Poisson 흔들림에 오염된다
(실측 r=+0.43 vs 원시점군 r=+0.93).

### 11.3 종료 구조 — 전역 정지가 아니라 gap 회계

GT 검증에서 NBV 개선은 **간헐적**임이 드러났다(작은 gap 패치가 조용한 뒤 큰 gap
패치가 +3.3%p). "연속 N 회 조용하면 종료"류 전역 규칙은 손실 없는 설정이 절약도
0 이었다 — 전역 규칙으로는 벌 게 없다.

→ **gap 단위 dry 회계** (`nbv_planner.report_patch`): 비생산 패치의 겨냥점 주변
(40mm)만 후보에서 제외 → 계획기가 후보를 소진하면 **자연 종료**. 판단이 틀려도
영역 하나를 잃을 뿐 스캔이 일찍 끝나지 않는다. 전역 백스톱(0.5%/3회, 실측 최대손실
0.27%p)은 병리 상황 대비로만 남겼다. 실효: spray_can 은 P1 이 충분하자 8패치 대신
2패치 만에 종료.

패치 생산성 임계 `DRY_EPS=1.5%` 는 실측 분포(비생산 0.2~1.2% / 생산 1.7~10.3%)의
사이값. **물체 2종으로 맞춘 값** — `validate_convergence.py` 로 대상을 넓혀 재검할 것.
같은 실행 조건의 completeness 실행 간 편차가 ~5%p 있으므로 그보다 작은 차이로
임계를 다투지 말 것.

기존 `p2.is_converged(cov, 0.012, 0.92)` 는 경계 12mm 미만을 요구하는데 실측값이
130~900mm 라 **한 번도 발동한 적이 없다**(死조건, real 도 동일).

### 11.4 도구

```bash
# GT 추출 (9종 일괄) — step2usd 환경
env -u PYTHONPATH ~/miniconda3/envs/step2usd/bin/python scripts/sim/extract_gt_mesh.py --all
# 단일/단계별 평가
python scripts/sim/eval_vs_gt.py --scan <obj> --gt scripts/sim/log/gt/<name>.npz
python scripts/sim/eval_vs_gt.py --stages <덤프dir> --gt ...
# 수렴 임계 검증 (MMS_SIM_STAGE_DUMP 실행 산출물 필요)
python scripts/sim/validate_convergence.py --dirs scripts/sim/log/conv_val/*
```
