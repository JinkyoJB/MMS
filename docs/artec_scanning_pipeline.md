# Artec 스캐닝 파이프라인 — Phase 1/2/3 · Recovery · Process · Live Viz

> 좌표 규약은 `CLAUDE.md` (`T_AB : A→B`, `x_B = T_AB @ x_A`) 준수.

---

## 0. 최종 목표

**MMS 파이프라인의 산출물 = 아이템 전면(full-coverage)의 water-tight mesh +
texture.** Phase 1/2/3, tracking-lost recovery, hand-eye, 라이브 뷰어 등 모든
sub-system 의 존재 이유는 결국 이 한 산출물에 귀결.

---

## 1. 전체 흐름 / 코드 맵

```
main_artec.py
 └─ ArtecMMS.artec_process()                         (mms_artec/system.py)
     ├─ 0. Scanning
     │   └─ ArtecMultiPassScanSession.run()          (nbv/artec_multipass_scan_session.py)
     │        └─ (pass 반복) ArtecStreamingScanSession.run()  (nbv/artec_streaming_scan_session.py)
     │             ├─ TurntableController (별도 thread)
     │             ├─ TrackingState (4 watchdog)
     │             ├─ tracking lost → _attempt_recovery() + RecoveryPoseSelector
     │             │                                  (nbv/recovery_pose_selector.py)
     │             └─ FrameEvent → LiveScanViewer 스냅샷  (nbv/live_scan_viewer.py)
     ├─ 1. SerialRegistration   (do_serial_registration=False 기본 — streaming 이 이미 정합)
     ├─ 2. GlobalRegistration   (hints_applied=True 면 자동 skip)
     ├─ 3. OutliersRemoval + SmallObjectsFilter   (★ Fusion 전, dev_mode 면 Outliers skip)
     ├─ 4. Poisson/FastFusion   → composite mesh
     ├─ 5. MeshSimplify         (옵션, dev_mode 면 skip)
     └─ 6. Texturization        → OBJ/sproj export
```

`main_artec.py` 기본값 (현재): `use_streaming_scan=True`,
`use_multipass_scan=True`(system.py 기본), `POSE_ROTATIONS =
make_axis_physical_rotations("y",[0,90,180])`, `max_passes=8`,
`RECOVERY_STRATEGY="local_jitter"`, `enable_live_viewer=True`,
`dev_mode=True`(Outliers/Simplify skip), `fusion="poisson"`.

> PhoXi 경로와 평행 분리. Artec 은 SDK native 자료구조를 그대로 차용
> (변환/래핑 없음). streaming 모드에선 `result.ctx=None` (legacy discrete
> 경로의 `ArtecScanContext` 미사용); 외부 메타(θ, EE pose)는 timeline CSV 로만.

---

## 2. 공통 아키텍처 — Spider ↔ Turntable 양방향 피드백

Phase 1/2 모두 같은 피드백 위에서 동작.

```
              TrackingState (shared)
              - frames_ok / failed,  consecutive_lost
              - registration_error,  tracking_lost(bool)
              - stop_event(Event)
                 ▲                       ▲
   callback 갱신 │                       │ stop_event 감시
   ┌─────────────┴───────┐     ┌─────────┴──────────────┐
   │ Main thread          │     │ TurntableController     │
   │  session.poll_events │     │  (별도 daemon thread)   │
   │   (50ms drain)       │     │  move_velocity 연속회전 │
   │  FrameEvent 처리     │     │  while not stop:        │
   │  if tracking_lost:   │     │    pos polling          │
   │     stop_event.set() │     │    도달/stop → 정지     │
   │  rotation_done→break │     │  finally: stop() 항상   │
   └──────────────────────┘     └─────────────────────────┘
```

### TrackingState — 4 watchdog (`artec_streaming_scan_session.py`)

| # | 조건 | 설정 키 | 기본 |
|---|---|---|---|
| 1 | FrameState 연속 정합/재구성 실패 | `consecutive_loss_threshold` | 8 |
| 2 | callback stall (N초 무반응) | `stale_threshold_s` | 2.0 |
| 3 | `registration_error < 0` 연속 (Studio 'tracking lost' 시그널) | `consecutive_reg_err_threshold` | 5 |
| 4 | `registration_error > max` 연속 (정합 품질 급락) | `consecutive_high_err_threshold` / `max_acceptable_reg_error` | 8 / 1.5 |

- warm-up: scan 시작 직후 `reg_err=-1` 은 SDK sentinel. `reg_err≥0` 한 번
  본 뒤(`tracking_established`)부터만 (3)(4) 카운트.
- `registration=HYBRID`(geometry+texture). ICP 만 쓰면 물체 제거 후 빈
  턴테이블에 정합 성공시켜 lost 가 안 뜸 (memory:
  `project_artec_tracking_lost_limitation`).
- Turntable 통신은 **UDP** (TCP 는 sustained polling 에서 socket 막힘).

---

## 3. Phase 1 — Streaming SLAM (연속 회전)

연속 회전 턴테이블 위에서 Spider 가 `IScanningProcedure` streaming SLAM 으로
한 바퀴 돌며 frame-by-frame 정합. 한 번의 360° 회전으로 **5면**(윗면+옆면4)
관찰; 바닥면은 디스크에 닿아 캡처 불가 → Phase 2 에서.

### 3.0 물체-적응 사전 포지셔닝 (object-adaptive pre-positioning)

회전 시작 **전 1회**, 물체 크기/위치를 보고 스캐너(EE) 를 그 물체에 맞는
거리·방향으로 이동. 회전 중에는 robot 고정(Phase 1 불변식 유지) — pre-position
만 신규. `ArtecMultiPassScanSession._adaptive_prescan_position()` (run() 시작,
pass 루프 전; `adaptive_phase1_positioning=True` 기본).

**목표 함수 (중요)** — "FOV 최대(넓게 보기)" 가 **아님**. Spider 는
close-range 라 멀리 빼면 FOV 는 넓어져도 해상도·정확도 급락 + 350mm 초과 시
재구성 불가. Phase 1 overlap 은 한 pose 의 넓이가 아니라 **시간(연속 회전 +
max FPS)** 에서 나온다. 정합을 망치는 실제 요인은 (a) 작업거리 이탈,
(b) grazing 입사, (c) feature 빈약. → 적응 목표 =
**물체 측면 띠를 최적 작업거리(~225mm, [200,250] band)에서, 좋은 입사각으로,
한 바퀴 내내 frustum 안에 유지.** coverage 는 목적이 아니라 그 대역 안의 제약.

거리는 stand-off backoff 로 이미 적응. **추가된 축 = 고도각(elevation)**:
물체중심 둘레 zx평면(= 현재 시선을 품은 수직면) 호를 따라 여러 고도각
후보를 **적응형 coarse→fine** 으로 preview·스코어해 최적 1개 선택
(탐색범위 `elevation_range_deg`=[−10°,+10°]). 재조준은 `look_at` 의 hardcoded
`+Z_C` 가정 대신 **경험적 캘리브된 광축**(`look_at_axes`)을 사용 →
mis-aim bug 회피. `phase1_elevation_search=True`(기본).
`False` 면 거리-only(`_distance_only_position`).

#### 선행 — 왜 단일뷰 isolation 을 버렸나 (2026-05-19)

구 `_isolate_object`(작업거리 게이트 + RANSAC 수평평면 제거 + DBSCAN)는
"카메라처럼 장면 전체를 보고 영역 분리" 전제였다. Spider 는 close-range·
협FOV(225mm 에서 footprint≈120×83mm) → preview 한 장은 **장면의 작은
부분 패치**라 "큰 평면 + 위 클러스터" 전역 구조가 안 들어오고, RANSAC
평면제거가 pose 의존적으로 되다 안 되다 함(실측: 같은 물체인데 후보별
plane=0 ↔ 10085, object=2.4k ↔ 16k → 탐색이 턴테이블 보는 자세 선택).
**단일 부분뷰 기하만으로 object↔turntable 분리는 원리적으로 불가**
(memory `project_spider_partial_view_no_scene_segmentation`). → 폐기.

#### 선행 1 — 턴테이블 회전 차분 물체 probe (`_motion_probe_object`)

물체를 구별짓는 **불변 물리 신호**: 물체는 턴테이블 수직축 둘레로
**돈다**. 회전대칭 디스크·정적 배경은 축 회전에 점집합이 **불변**(자기
자신에 매핑). 이를 segmentation 메커니즘으로 사용.

- 스캔 전 1회, **robot=home 고정**(사용자가 물체를 조준한 검증 자세).
  턴테이블을 `probe_total_deg`(기본 150°) 를 `probe_steps`(기본 6) 로
  나눠 회전하며 각 step `θ_k` 에서 preview 캡처 → `T_CB_home` (전 step
  동일) 으로 base B 변환 → 점집합 `P_k`. probe 후 턴테이블 시작각 복귀.
- **static/moving 분리**: 모든 `P_k` 합집합을 voxel(`probe_vox_mm`,
  기본 4mm) 화. voxel 의 frame-support = 점유한 step 수.
  support ≥ `probe_static_support_frac`·K → **static**(디스크+배경, 제거).
  나머지 점유 voxel = **moving** = 물체 swept set. moving 점 DBSCAN →
  최대 클러스터(일회성 이동 배경·손 제거) = **물체 swept points**.
- **회전축·envelope 추정**: 알고있는 명령각 `Δθ_k` 로 `P_0` 를 수직축
  둘레 회전시켜 `P_k` 와 best-match 되는 수평 축점 `(c_x,c_y)` 를 2-파라
  최소화로 추정(= **턴테이블 회전축**, stale yaml 불필요·부산물 자동
  산출, `axis_fit_resid_mm` 보고). envelope = 수직 실린더:
  `r_obj`=축까지 수평거리 95pct, `z_bot/z_top`=swept z 5/95pct,
  `z_table`≈`z_bot`. **피벗 = (c_x,c_y,z_mid)**.
- **fallback**: moving 점 < `probe_min_moving_pts` 또는 axis resid 과대
  → `_distance_only_position`(center = moving centroid 있으면 그것,
  없으면 home preview median; 경고). look_at 스윙 금지.
- 진단: `probe_debug_dump=True` 면 step별 static/moving + envelope 를
  색상 PLY 로 `iso_debug_dir` 에 덤프(CloudCompare). 기본 OFF, scan 무영향.

#### 선행 2 — C-프레임 광축 경험적 캘리브 (`calibrate_camera_axes_from_preview`)

`T_EC_artec` C 프레임 `+Z` 는 실제 광축과 어긋남(2026-05-19 실측). SDK
intrinsic·yaml 신뢰 안 함. **probe 의 물체점(θ0 frame, C)** 으로:
- `fwd_C` = `unit(median(obj_v_C))` (frustum 축, 이제 신뢰가능한 물체점).
- 교차검증 `R_BC_home @ unit(center_B−cam_home)` 사잇각 `crosscheck_deg`
  (>20° 경고). `up_C` = `R_BC_home @ world_Z` 의 `fwd_C`⟂ 성분 →
  home 재현 `closure_rot_deg`(>15° 경고). 캘리브 `(fwd_C,up_C)` 는
  `CentroidVectorSelector` 에도 주입(latent bug 동시 해소).

**알고리즘** (`_adaptive_prescan_position` → `_elevation_search`)

1. **회전 차분 probe** (선행 1) → 물체 envelope(축 `(c_x,c_y)`,
   `r_obj`,`z_bot/z_top`,`z_table`) + 물체점. 실패 → 거리-only fallback.
2. **C-프레임 캘리브** (선행 2) — probe 물체점으로. 실패 → 거리-only.
3. **호(arc) 정의** — 피벗 = envelope 축 `center_B=(c_x,c_y,z_mid)`.
   `a_B = unit(world_Z × radial_horizontal)`,
   `radial_horizontal=(cam_home − center_B)` 수평성분. 카메라가 물체
   바로 위면 호 정의 불가 → 거리-only fallback.
4. **Stand-off backoff** (`_standoff_distance`) — `d`:
   `h_obj+2·margin ≤ 2·d·tan(HALF_FOV_V)`(h_obj=`z_top−z_bot`) 만족하는
   ~225mm, 안 되면 필요한 d 로 증가하되 `[SPIDER_NEAR+r_obj,
   SPIDER_FAR]` clamp. 한계 초과 → 부분 커버 수용 + 경고.
5. **φ 후보 — 적응형 coarse→fine** (탐색범위
   `elevation_range_deg`=**[−10°,+10°]**). 각 φ:
   `eye_B = center_B + R(a_B,φ)·unit(radial)·d`, orientation =
   `look_at_axes(eye_B, center_B, fwd_C, up_C, world_Z)`.
   - **coarse**: `home_dist`(거리-only pose, 비퇴행 baseline) +
     `elevation_coarse_offsets_deg`(기본 `[−10,−5,0,+5,+10]`).
   - **fine**: coarse best φ*(≠home_dist) 주변 `φ*±elevation_fine_step_deg`
     (기본 ±2.5°), range clamp, ε중복 skip. best=home_dist 면 생략.
6. **후보 평가** (coarse·fine 공통) — robot 이동(`_move_robot_to_T_CB`,
   code≠0 → skip) → preview → **envelope 멤버십**으로 물체점 판정
   (`_in_object_envelope`: 축까지 수평거리 ≤ `r_obj+env_r_margin` ∧
   z∈[`z_bot−env_z_margin`,`z_top+env_z_margin`]) — **pose 무관·견고,
   매 preview RANSAC 안 함**. **v1 스코어** = 그 물체점 ∩
   최적거리밴드[200,250]mm ∩ 카메라 FOV(fwd_C/up_C 기저) 개수. 로그에
   `물체점 N` 병기. (입사각 항 v2.)
7. **선택·확정** — coarse ∪ fine ∪ `home_dist` argmax. 유효 후보 없음
   → **home 복귀**(비퇴행). best 이동 성공 → `self._T_BC/_T_CB`
   **재캡처**(recovery hint 일관성).

> 비용: 스캔 전 1회 = probe(`probe_steps`≈6 preview + 소회전 + 복귀,
> robot 고정) + 고도각 ~8 move·preview. 수십초~1분대.
> 회전 중 robot 고정이라 streaming/recovery 기존 로직 불변.
> pose 1/2(flip)은 같은 robot pose 유지. 구 `_isolate_object`/`iso_*`
> (단일뷰 RANSAC)는 폐기 — envelope 멤버십이 대체.

### Why Artec ≠ PhoXi (설계 근본 차이)

| | PhoXi | Artec |
|---|---|---|
| 좌표 기준 | 절대 (T_CO 직접) | 상대 (이전 frame 기준 ICP) |
| 탈선 시 | 다음 frame 도 복구 가능 | tracking 잃으면 **scan 망가짐** |
| Phase 1 핵심 | 정확한 hand-eye+θ | **OVERLAP 유지** |
| Frame 간격 | 15° step OK | 작은 step (15°도 위험) → **연속 회전** |

→ 천천히 연속 회전 + Spider max FPS 추종. 한 번 끊기면 그 시점까지 frame 만
살아남음.

### 핵심 설정 (`ArtecStreamingScanSessionSettings`)

- `rotation_duration_s=30`, `target_fps=None`(=max_fps), `registration=HYBRID`
- pipeline = `REGISTER_FRAME | FIND_GEOMETRY_KEYFRAME | MAP_TEXTURE |
  CONVERT_TEXTURES`, `capture_texture=ALWAYS`, `ignore_registration_errors=True`
- `reset_to_zero_first=True` — scan 전 turntable clearpos 로 logical 0°.
  단 recovery 직후엔 multipass 가 임시 False (직접 move_abs 로 위치 잡음).
- 종료 시 post-scan clearpos → 현재 물리 위치를 logical 0° 재정의
  (recovery safe-back 계산의 기준).

### 결과 (`ArtecStreamingScanResult`)

`model, n_frames, rotation_actual_deg, duration_s, fps_actual,
tracking_lost, loss_reason, frames_ok, frames_failed,
last_good_theta_rad`(reg_err≥0 였던 마지막 timeline θ — recovery rollback 기준).

---

## 4. Phase 2 — 바닥면 캡처 + Pose Disambiguation

`ArtecMultiPassScanSession` 이 Phase 1+2 를 통합 오케스트레이션. 사용자가
객체를 물리적으로 회전시키며 여러 pose 를 새 IScan 으로 캡처, master IModel 에
누적.

- Pose 0: canonical (face1=top)         — Phase 1
- Pose 1: Ry(+90°) — face5 가 위로       — 옆면 보강
- Pose 2: Ry(+180°) — face6(바닥) 위로   — Phase 2
- `pose_idx` = 자세(논리 인덱스, hint 의 index). `n_pass` = 실제 IScan 수
  (retry 포함). tracking-lost retry 는 pose_idx 유지(같은 hint 재사용),
  정상 완료 후 사용자 [Enter] 시만 `pose_idx += 1`.

### 4.1 Face-merging 문제

새 IScan 은 SDK 가 **자기 첫 frame 기준** 좌표계로 시작 → 첫 IScan 좌표계와
무관. GlobalRegistration 이 초기 추정 없이 identity 에서 출발 → 대칭/유사
아이템에서 윗면(face1)과 바닥면(face6)을 동일면으로 **오인 합병**(local
minimum). 대칭↑ → identity cost↓ → 함정↑. (Studio 는 manual alignment 로
시작 transform 을 줘서 회피 — 우리 코드엔 없음.)

### 4.2 해결 — centroid-pivot pre-rotation hint (구현됨)

사용자가 약속한 물리 회전 `R_phys` 를 IScan 의 모든 frame_transformation 에
좌측 곱해 GlobalReg 가 옳은 basin 에서 출발하게 함. 회전 pivot 은 **객체
centroid** (scan-world 원점 = 카메라 위치가 아니라 — 그러면 90°에 중심이
~42cm 이동해 정합 파탄).

```
T_pre[:3,:3] = inv(R_phys)[:3,:3]
T_pre[:3, 3] = c_master - inv(R_phys)[:3,:3] @ c_pass
for i in scan.frame_count():            # scan.set_frame_transformation
    T_new = T_pre @ scan.get_frame_transformation(i)
```

- `c_master` = Pass 1 vertex centroid (lock), `c_pass` = 현재 pass centroid.
  `_compute_model_centroid` (scan 당 50 frame subsample).
- inverse 이유: 사용자가 Ry(+90°) 했으면 데이터가 +90° 회전돼 있으니
  Ry(-90°) 로 되돌림. 작은 오차는 GlobalReg 가 refine.
- `R_phys` 는 base frame B 정의 → scan-world 적용 시
  `R_W = T_BC @ R_B @ T_CB` 변환 (Spider 가 EE 마운트라 base Y ≠ scan Y).
- **hints_applied=True → artec_process 의 post-merge GlobalReg 자동 skip**
  (hint 가 authoritative; GlobalReg 가 다시 흩뜨리는 것 방지).
- API: `pose_physical_rotations: List[Optional[np.ndarray]]`,
  `make_axis_physical_rotations(axis, angles_deg)`.

### 4.3 Failure modes / 대안

- 회전축/부호 틀림 → wrong basin. 진단: 로그의 `c_master/c_pass` translation
  이상.
- **비대칭·길쭉한 객체**: 눕히면 surface-vertex centroid 가 body 기준 이동 →
  "centroid=body 중심" 가정 깨짐. 향후: OBB center 사용.
- 직교 대안(코드 0): 비대칭 마커 부착(SDK hybrid 가 symmetry 깸) — 마커
  흔적 후처리 필요. 장기: gripper 로 임의 자세 자동화(Phase 3 와 함께).

---

## 5. Phase 3 — Hole-fill (미구현)

Phase 1+2 결과 mesh 의 boundary edge → frontier 영역 검출 → 해당 영역을
보는 EE pose 계산 → 로봇 천천히 이동(overlap 유지 필수). 우선순위는
Phase 1+2 mesh 품질 검증 후. gripper 기반 임의자세(코드8 §3.5)와 함께 설계.

---

## 6. Tracking-lost 자동 Recovery

**semi-auto**: 같은 pose 안에서 최대 **3회** 자동 retry, 초과 시 user-prompt
fallback (lost 시그널이 object-presence 와 1:1 이 아니라 무한 retry 는 엉뚱
데이터 누적 위험). 정상 pass 나오면 retry 카운터 0 reset.

### 6.1 _attempt_recovery 흐름 (multipass)

```
tracking_lost
 ├─ drive-alarm short-circuit (통신 사망 / alarm trip → 즉시 종료, 물리 전원안내)
 └─ selector 있고 retry<3:
     1. safe_back_target_rad 계산 (§6.2)
     2. turntable: stop → check_drive_err → set_servo_on → move_abs → wait
     3. master_pts_B = master_points_in_base_frame(model, self._T_CB, 30k)
        (비어있으면 turntable safe-back 만으로 같은 자리 재시도)
     4. T_CB_now = T_EC @ inv(T_EB_now)
     5. decision = selector.select(T_CB_now, master_pts_B)
        (None 이면 같은 자리 재시도)
     6. Δ<2mm 면 robot 이동 skip; 아니면 xArm.set_position(T_CB_target @ T_EC)
     7. T_BC_recovery 캡처 → next_T_BC_pending 로 다음 merge 의
        camera-motion hint override
```

다음 iteration: `next_skip_clearpos=True` (이미 safe-back 위치) +
merge 단계에서 `T_pre = T_BC_master @ inv(T_BC_recovery)` (translation ×1000,
R_phys hint 보다 우선).

### 6.2 Safe-back 각도

post-scan clearpos 로 recovery 시점 turntable logical = 0° 가정.

```
final_rad      = scan 종료 θ (rotation_actual_deg)
last_good_rad  = reg_err≥0 였던 마지막 θ
scan_dir_sign  = sign(final_rad)
margin_rad     = radians(safe_back_margin_deg=10)
safe_back_target_rad = (last_good_rad - final_rad) - scan_dir_sign*margin_rad
```

예: last_good -156.7°, final -174.5° → target = +17.8 -(-1)*10 = **+27.8°**
(= last-good 보다 10° 더 뒤). `last_good_rad==0 & final≠0` (tracking 한 번도
안 잡힘) → recovery skip, user prompt.

### 6.3 RecoveryPoseSelector (`recovery_pose_selector.py`)

`main_artec.py` 의 `RECOVERY_STRATEGY` 토글: `"local_jitter"` /
`"centroid_vector"` / `None`. swap 가능 Protocol:
`select(T_CB_current, master_pts_B_mm) -> Optional[RecoveryPoseDecision]`.

- **Method A — LocalJitterSelector** (exploitation): 현재 pose 주변
  ±30mm/±8° 로 N개(=9) candidate, master point cloud 를 frustum raycast
  (occlusion 무시, 속도 우선) → 가시점 최대 선택. master 와 overlap 보장.
- **Method B — CentroidVectorSelector** (exploration): master centroid 기준
  마지막 실패 방향 반대편 stand-off 250mm 에서 centroid 응시 (`look_at`).

Spider v1 광학 상수 (`recovery_pose_selector.py` 상단, 코드 기준):

| 상수 | 값 |
|---|---|
| FOV (H×V) | 30° × 21° → half (15°, 10.5°) |
| `SPIDER_NEAR_MM` / `SPIDER_FAR_MM` | **170 / 350** (full working range) |
| 최적 작업거리 | ~200–250 mm |
| `SPIDER_DEFAULT_STANDOFF_MM` | 250 |
| 3D resolution / accuracy | 0.1 / 0.05 mm |

> `master_points_in_base_frame` 는 **C→B = `T_CB`(=inv(T_BC))** 사용
> (과거 T_BC 순방향 오용 버그 수정됨). depth gate 도 100mm→180mm 폭으로
> 완화(170–350)되어 selector 가 후보 0 으로 굶지 않음.

---

## 7. 후속 알고리즘 파이프라인 (`artec_process`)

**SDK General Pipeline 순서**(= 코드 실제 호출 순서). Studio GUI 라벨
("Scanning→Cleaning→Alignment→Registration→Fusion→Postprocessing") 은 사용자
워크플로우 라벨이지 알고리즘 순서가 아님.

| 순서 | 알고리즘 | 단위 | 비고 |
|---|---|---|---|
| 1 | SerialRegistration | frame-to-frame | `do_serial_registration=False` 기본 (streaming 이 이미 정합) |
| 2 | GlobalRegistration | IModel 전체 | `hints_applied=True` 면 자동 skip |
| 3a | OutliersRemoval | per-frame | **Fusion 전**. dev_mode 면 skip |
| 3b | SmallObjectsFilter | per-frame | **Fusion 전** |
| 4 | Poisson/FastFusion | clean frames → composite | watertight mesh |
| 5 | MeshSimplify | composite | 옵션. dev_mode 면 skip |
| 6 | Texturization | composite + frame tex | UV/atlas + baking → OBJ/sproj |

> ★ **Cleaning(3a/3b)은 반드시 Fusion 전.** Fusion 후에 두면 outlier 박힌
> composite 가 Texturize 까지 가서 실패 (관측: ErrorCode `0x80010203`).
> Phase 2 frontier 용 임시 mesh 는 `fast_fusion` 사용.

자료구조 계층 / 래퍼:

```
IFrame → IFrameMesh → IScan → IModel → ICompositeMesh
         FrameMeshHandle  ScanHandle  ModelHandle   (ModelHandle.final_vertices/faces)
```

`FrameMeshHandle`: `vertices`(N,3 mm, sensor) `faces`(M,3) `uv`(N,2)
`image`(H,W,3). 다중 frame 누적용 binding: `create_scan / scan_add_frame /
model_add_scan` (`artec_base_binding.cpp`). composite 는 fusion 후
`ModelHandle.has_final_mesh()/final_vertices()/final_faces()`.

---

## 8. 라이브 시각화 (2026-05-19 도입)

스캔 중 노이즈/드리프트/멈춤을 눈으로 확인하기 위한 누적 컬러
포인트클라우드 뷰어. **결과만으로 판단하기 어렵던 문제 해결.**

- 누적 좌표 = **SDK 자신의 정합행렬** `FrameEvent.transformation`
  (sensor→scan-world; `artec_scanning_binding.cpp` 에 노출 추가).
  `x_world_mm = v_mm @ T[:3,:3].T + T[:3,3]`, /1000 → m. θ /
  turntable_frame.yaml / hand-eye 의존 **없음** → 화면 = SDK SLAM 결과 그 자체.
- **아키텍처 (검증된 유일 구성)**:
  - 파이프라인측 `LiveScanViewer` = 컨트롤러만. Open3D/Popen 안 함. 매 OK
    프레임 누적 → `output/_live_latest.npy` atomic write
    (tmp+os.replace) + `.flag`/`.stop` 보조.
  - 뷰어 = 사용자가 **다른 터미널에서 직접 실행**:
    `python scripts/artec/live_scan_view.py`. Open3D **신형
    O3DVisualizer(Filament)** — 메인스레드 `app.run()`, watcher
    백그라운드 스레드가 snapshot mtime 폴링 → `post_to_main_thread`.
- 실행: 터미널A `python main_artec.py` / 터미널B `python
  scripts/artec/live_scan_view.py` (순서 무관, A 종료 시 B 자동 종료,
  정합 끊기면 배경 빨강). 튜닝: `LiveScanViewer(voxel_mm=…)`,
  `live_scan_view.py` 상단 `GAMMA`/`POINT_SIZE`.
- **금지 (확정 실패 경로)**: legacy `o3d.visualization.Visualizer` 는 이
  환경에서 동적 PointCloud 못 그림(메쉬는 됨). Popen 으로 띄운 자식
  Filament 창은 즉시 죽음. → 뷰어는 사용자가 별도 터미널 실행.
  검증 스크립트: `scripts/artec/live_scan_viz_test.py` (스캐너 직결판).
- 점 전용 PLY (`output/live_cloud_*.ply`) 는 Windows 3D뷰어 불가 →
  CloudCompare/MeshLab.

---

## 9. 좌표 / 단위 규약

프레임: **B**=xArm base=world, **E**=TCP, **C**=Spider camera (≈ scan-world
W; 각 ScanSession 이 첫 frame 카메라 frame 을 W 로 잡는다는 가정), **F**=
turntable, **O**=object. `T_EC_artec` = hand-eye (config/sensor_frames.yaml,
2026-04-29, 고정).

| 출처 | translation 단위 |
|---|---|
| `XArmInterface.get_ee_pose_mat()` T_EB, yaml `T_EC` | **m** |
| `xarm.set_position(x,y,z,…)` | **mm** |
| SDK frame_transformation / vertices / master pts | **mm** |

→ camera-motion T_pre 의 translation 만 `× 1000` scale (merge hint 블록).
W 가정이 어긋나도 raycast scoring 은 candidate **상대** 비교라 영향 작고
GlobalReg 가 후처리 보정.

---

## 10. 알려진 한계 / 가정

1. **turntable_frame.yaml 미검증 가능성** — T_BF0 2026-04-23 (Artec pivot
   이전), rim 3점·residual 0.0. θ 기반 배치가 원호로 번진 정황. Phase 2
   hint·NBV·recovery raycast 가 같은 T_BF0 의존 → 정밀도 의심 시 1순위.
   재캘리브: `scripts/artec/turntable_frame_init.py`.
   (라이브 뷰어는 SDK 정합행렬로 전환해 이 의존 없음.)
2. **tracking-lost ≠ object-presence**: 물체 제거해도 빈 디스크에 정합
   성공시켜 lost 안 뜰 수 있음 → HYBRID + 별도 휴리스틱.
3. **last-good θ 없으면 recovery skip** (시작 직후 lost).
4. **Robot 안전성**: xArm IK/limit/self-collision 에 의존. 도달 불가 pose
   추천 시 `set_position` 실패 → 같은 자리 재시도.
5. **Raycast occlusion 무시** (frustum culling 만; close-range 라 영향 작음).
6. **Console cp949**: ⚡ 등 이모지 깨짐 — utf-8/PowerShell 터미널 정상.

---

## 11. 환경 / 재현 체크리스트

- Artec SDK 1.17.3, 스캐너 = Spider `SP.10.36181288`.
- xArm `192.168.1.210`, Turntable `192.168.0.10` **UDP**.
- `config/sensor_frames.yaml` `T_EC_artec`,
  `config/calibration/turntable_frame.yaml` 확인 (§10.1 주의).
- binding 변경 시: `cmake --build mms_artec/sensor/build --config Release
  --target <module>` (예: `artec_scanning_py` — transformation 노출분).
- `python -c "import main_artec"` import chain 클린 → 터미널A/B 로 실행.
- A/B recovery 비교: `RECOVERY_STRATEGY` 토글, 동일 물체 3회씩,
  `ArtecMultiPassScanResult.n_recovery_attempts/_succeeded`,
  `pass_results[i].n_frames/tracking_lost` 비교.
