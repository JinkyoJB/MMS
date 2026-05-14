# Artec Phase 1 — Tracking-Lost Auto Recovery (2026-05-14)

> **목적**: Phase 1 streaming scan 도중 tracking lost 발생 시, 사용자 개입 없이
> turntable 을 last-good 각도로 역회전 + 스캐너를 새 viewpoint 로 이동 → 자동
> 재시도 하는 시스템 설계 / 구현 기록. 다른 컴퓨터에서 이어 개발하기 위한
> 컨텍스트 문서.

본 문서는 docs/7_artec_phase.md / docs/8_artec_phase2_pose_disambiguation.md
의 후속이며 multi-pass orchestrator 의 tracking-lost 분기 확장에 해당.

---

## 0. 배경 / 동기

### 0.1. 직전 상황 (2026-05-14 기준)

- Turntable 제어 문제가 **UDP** (TCP 대신) 로 통신해서 해결됨.
- Phase 1 streaming scan 의 tracking-lost 검출은 4가지 워치독으로 동작:
  1. FrameState 기반 연속 정합 실패 (`consecutive_loss_threshold`)
  2. callback stall (`stale_threshold_s`)
  3. `registration_error < 0` 연속 (`consecutive_reg_err_threshold`)
  4. `registration_error > max_acceptable` 연속 (`consecutive_high_err_threshold`)
- 검출 시 동작: turntable 즉시 정지 + sub-result 반환 + multipass 가
  user prompt 로 retry 여부 확인 (기존 동작).

### 0.2. 문제

- Tracking lost 후 **같은 자리에서** 같은 turntable 시작 위치로 재시도하면
  같은 지점에서 또 lost 날 확률이 높음 (실패 지점의 geometry / 텍스처 그대로).
- User 개입 (Enter) 이 필요해 자동화가 끊김.

### 0.3. 목표 (사용자 명세)

> "tracking lost 됐을 때 멈추는 단계까지는 OK. 이제 tracking lost 가 뜬 상태로
> 다시 turntable 을 돌리고 3d 스캐너 위치를 조정하는 알고리즘을 넣어보자."

→ tracking lost 시 **자동** 으로:
1. turntable 을 "마지막으로 정상이었던 각도" 보다 약간 더 뒤로 역회전
2. 스캐너 위치 재조정
3. 재시작 (같은 pose 안에서 N회까지)

---

## 1. 사용 센서 / 광학 상수 (Spider v1)

reference: <https://www.artec3d.com/portable-3d-scanners/old/spider>  
(URL 은 Space Spider 페이지로 redirect; 실질적으로 동일 광학)

| 항목 | 값 |
|---|---|
| Working distance | **200–300 mm** |
| Angular FOV | **30° × 21°** (H × V) |
| FOV @ 200mm | 90 × 70 mm |
| FOV @ 300mm | 180 × 140 mm |
| Frame rate | up to 7.5 FPS |
| 3D resolution | 0.1 mm |
| 3D accuracy | 0.05 mm |
| Tracking | hybrid geometry + color |

→ Recovery viewpoint 설계 시:
- Stand-off default = **250mm** (working range 중간)
- Raycast frustum half-angles = (15°, 10.5°)
- Depth range filter = [200, 300] mm

상수는 `mms_artec/nbv/recovery_pose_selector.py` 상단에 정의:
`SPIDER_HALF_FOV_H`, `SPIDER_HALF_FOV_V`, `SPIDER_NEAR_MM`, `SPIDER_FAR_MM`,
`SPIDER_DEFAULT_STANDOFF_MM`.

---

## 2. 설계 결정 (Q&A)

### 2.1. 자동 vs semi-auto

**결정**: semi-auto — 같은 pose 안에서 **최대 3회** 자동 retry. 초과 시 user
prompt 로 fallback.

**이유**: tracking-lost 시그널이 object-presence 와 1:1 매칭이 아니어서
(memory: `project_artec_tracking_lost_limitation`) 무한 자동 retry 는 엉뚱한
데이터 누적 위험.

### 2.2. Safe-back 마진

**결정**: last-good θ 보다 **10°** 더 뒤. config 화 (`safe_back_margin_deg`).

**이유**: 같은 각도에서 시작하면 같은 지점에서 또 lost. 작은 마진으로
fresh overlap 확보.

### 2.3. Scanner 재배치 알고리즘 — 두 방법 A/B 테스트

#### Method A — Local jitter + raycast scoring (exploitation)

- 현재 camera pose 주변 ±3cm translation, ±8° rotation 으로 N 개 candidate 샘플
- 각 candidate 에서 master point cloud 를 카메라 frustum 에 raycast (occlusion
  무시 — 속도 우선)
- 가시 점 개수 최대인 후보 선택
- candidate 평가는 **raycast 시뮬레이션** (실 robot 이동 X) → 빠름

선택 이유: 마스터와의 overlap 보장 (registration 잘 되도록).

#### Method B — Centroid-vector (exploration)

- Master point cloud 의 centroid 를 base frame 으로 변환
- 마지막 실패 방향 (= 현재 camera → centroid 벡터) 의 **반대편** 에 stand-off
  250mm 떨어진 위치
- 그 위치에서 centroid 를 바라보는 카메라 pose (`look_at`)

선택 이유: 단순 / 결정적 / 보지 못한 면 우선. 단순 (a) "반대편" 으로 시작 — 
NBV-lite (히스토그램 기반) 는 Phase 2 NBV 와 거의 같아져서 phase 1 범위 초과.

### 2.4. 두 방법 swap 가능한 인터페이스

```python
class RecoveryPoseSelector(Protocol):
    name: str
    def select(
        self,
        T_CB_current: np.ndarray,
        master_pts_B_mm: np.ndarray,
    ) -> Optional[RecoveryPoseDecision]: ...
```

main_artec.py 의 `RECOVERY_STRATEGY` 토글로 swap → 같은 object 로 3회씩
측정해서 (성공률, 회복 후 coverage, 총 recovery 시간) 비교.

### 2.5. 사용자가 답한 design choice (체크리스트)

- [x] Method A scoring: **raycast** (실 robot 이동 X)
- [x] Method B 방향: **(a) 마지막 실패 방향의 반대편** (NBV-lite 보류)
- [x] safe-back 마진: **10° 고정** (config 화는 했음)
- [x] 최대 재시도: **3회**
- [x] Spider 광학 상수: 공식 spec 페이지 참조

---

## 3. 아키텍처

### 3.1. Component 다이어그램

```
┌────────────────────────────────────────────────────────────────┐
│  main_artec.py                                                 │
│   RECOVERY_STRATEGY = "local_jitter" | "centroid_vector"|None  │
│         │                                                      │
│         ▼                                                      │
│   LocalJitterSelector / CentroidVectorSelector                 │
└──────────────────────────────┬─────────────────────────────────┘
                               │ recovery_selector
                               ▼
┌────────────────────────────────────────────────────────────────┐
│  ArtecMultiPassScanSession.run()                               │
│   while n_pass < max_passes:                                   │
│      streaming = ArtecStreamingScanSession.run()  ← 한 회전    │
│      merge into master (with T_pre hint)                       │
│      if tracking_lost:                                         │
│          ━ drive-alarm short-circuit ━                         │
│          ┌─→ _attempt_recovery():                              │
│          │    1. compute safe_back_target_rad                  │
│          │    2. turntable.move_abs(safe_back, vel)            │
│          │    3. master_pts_B = points_in_base_frame(...)      │
│          │    4. T_CB_current = T_EC @ T_BE(now)               │
│          │    5. decision = selector.select(...)               │
│          │    6. xArm.set_position(T_EB_target)                │
│          │    7. T_BC_recovery = T_EC @ T_BE(after)            │
│          │    return (ok, T_BC_recovery)                       │
│          ▼                                                     │
│      next_T_BC_pending = T_BC_recovery                         │
│      next_skip_clearpos = True                                 │
│      continue  ← 다음 iter 의 merge 단계가                     │
│                  camera-motion T_pre 로 override               │
│      (else: 정상 → retry 카운터 reset)                         │
└────────────────────────────────────────────────────────────────┘
```

### 3.2. State machine — recovery 카운터

```
   ┌─────────┐
   │ pass 1  │── 성공 ──→ retry_count = 0 (reset)
   │  start  │
   └────┬────┘
        │ lost
        ▼
  ┌─────────────┐       (max=3)
  │ recovery #1 │── ok ──→ pass 2 (within same pose_idx)
  └─────────────┘                │
                                 │ lost
                                 ▼
                         ┌─────────────┐
                         │ recovery #2 │── ok ──→ pass 3
                         └─────────────┘                │
                                                        │ lost
                                                        ▼
                                                ┌─────────────┐
                                                │ recovery #3 │── ok ──→ pass 4
                                                └─────────────┘                │
                                                                              │ lost
                                                                              ▼
                                                              retry_count ≥ max
                                                              → user prompt fallback
```

정상 완료된 pass 가 나오면 retry_count 0 으로 reset. 동일 pose 안에서
연속 lost 만 카운트.

---

## 4. 좌표 프레임 / 단위 주의사항

CLAUDE.md 의 `T_AB : A → B  (x_B = T_AB @ x_A)` 컨벤션 준수.

### 4.1. 프레임

- **B**: xArm 베이스 = world.
- **E**: TCP / flange.
- **C**: Spider camera frame (= scan world W 와 근사 동일 — §4.3 참조).
- **T_EC**: hand-eye calibration 결과 (config/sensor_frames.yaml 의
  `T_EC_artec`, 2026-04-29 측정). 고정.

### 4.2. 단위

| 출처 | translation |
|---|---|
| `XArmInterface.get_ee_pose_mat()` 반환 T_EB | **m** |
| `xarm.set_position(x, y, z, ...)` | **mm** |
| `T_EC` (yaml 에서 로드) | **m** |
| SDK frame_transformation, vertices | **mm** |
| master point cloud 내부 처리 | **mm** |

이 mix 때문에 camera-motion T_pre 의 translation 만 `× 1000` 으로 scale.
구현 위치: `artec_multipass_scan_session.py` 의 merge hint 블록.

### 4.3. SDK W frame 가정

각 `ScanSession` 은 fresh 생성되며, SDK 가 첫 frame 의 카메라 frame 을 world
W 로 잡는다는 **가정**:

- W_master = camera frame at start of first pass ≈ C
- 따라서 master vertices (W frame) ≈ C frame 좌표.
- W → B 변환: `T_BC_master` (multipass `__init__` 에서 캡처한 `self._T_BC`).

⚠ 이 가정이 어긋나면 raycast scoring 의 절대 절차는 비뚤어지지만 candidate
간 **상대** 비교는 유지되므로 selector 결정 자체는 영향이 작음. 정합 단계의
GlobalRegistration 이 후처리 보정.

### 4.4. Camera-motion T_pre

Recovery pass 의 IScan 을 master 에 merge 할 때, robot 이 움직였으므로
new W (W_new ≈ C_at_recovery) 와 master W 가 다름.

```
T_pre = T_BC_master @ inv(T_BC_recovery)        # W_new → W_master
T_pre[:3, 3] *= 1000                            # m → mm (SDK 단위)
```

이 T_pre 가 `_merge_into_master` 의 left-multiply hint 로 들어가
모든 frame_transformation 에 적용됨. 기존 `R_phys` 기반 hint 와는 mutually
exclusive — recovery override 가 우선.

---

## 5. Safe-back 각도 계산

### 5.1. 가정

- failed scan 종료 직후 streaming session 이 post-scan clearpos 를 실행해
  현재 physical 위치를 logical 0° 로 재정의.
- 따라서 multipass 가 recovery 진입 시점에 turntable logical = 0°.

### 5.2. 공식

failed scan 좌표계에서:
- `final_rad` = scan 종료 시 turntable θ (= `rotation_actual_deg` radians)
- `last_good_rad` = `reg_err ≥ 0` 였던 마지막 timeline sample 의 θ
- `scan_dir_sign` = `sign(final_rad)`  (예: negative direction 회전 → -1)
- `margin_rad` = `radians(safe_back_margin_deg)`

post-clearpos 좌표계 (= 현재 logical) 에서 target:
```
delta_to_lastgood   = last_good_rad - final_rad
safe_back_target_rad = delta_to_lastgood - scan_dir_sign * margin_rad
```

### 5.3. 예시 (실제 로그 기준)

```
failed: last_good θ=-156.7°,  final θ=-174.5°  (negative direction)
delta_to_lastgood   = -156.7 - (-174.5) = +17.8°
scan_dir_sign       = -1
safe_back_target    = +17.8 - (-1) * 10.0 = +27.8°
```

→ post-clearpos 좌표계의 +27.8° 위치 = failed scan 의 -146.7° 위치 (= last-good
보다 10° 더 뒤).

### 5.4. Edge case — last_good 없음

`last_good_rad == 0.0` 이고 `final_rad ≠ 0` 이면 tracking 이 한 번도 안 잡혔다는
의미. safe-back 의미 없음 → recovery skip, user prompt 로 fallback.

---

## 6. 파일별 변경 요약

### 6.1. 신규 — `mms_artec/nbv/recovery_pose_selector.py`

```
Spider v1 광학 상수
  SPIDER_HALF_FOV_H = radians(15)
  SPIDER_HALF_FOV_V = radians(10.5)
  SPIDER_NEAR_MM = 200,  SPIDER_FAR_MM = 300
  SPIDER_DEFAULT_STANDOFF_MM = 250

Helpers
  master_points_in_base_frame(model, T_BC_master, max_points=30_000)
      → (M,3) np.ndarray, base frame mm
      W (≈ C of first pass) → B 변환, subsample.
  visible_point_count(pts_B_mm, T_CB_candidate, near, far, half_h, half_v)
      → int  (frustum culling, no occlusion)
  look_at(eye_B, target_B, up_hint=[0,0,1])
      → T_CB (4x4, translation m). +Z_C forward to target.

Protocol
  RecoveryPoseSelector
    name: str
    select(T_CB_current, master_pts_B_mm) -> Optional[RecoveryPoseDecision]

@dataclass LocalJitterSelector  (Method A)
  n_candidates=9, trans_mm=30, rot_deg=8, include_current=True, seed=None

@dataclass CentroidVectorSelector  (Method B)
  stand_off_mm=250, up_hint_B=[0,0,1]
```

### 6.2. 수정 — `mms_artec/nbv/artec_streaming_scan_session.py`

- `ArtecStreamingScanResult.last_good_theta_rad: float = 0.0` 필드 추가.
- 메인 루프 timeline 기록 직후, `tracking.tracking_established and
  last_reg_error >= 0 and consecutive_reg_err == 0` 일 때 현재 θ 를 캐시.
- return 시 결과 객체에 전달.

### 6.3. 수정 — `mms_artec/nbv/artec_multipass_scan_session.py`

- imports: `time`, `RecoveryPoseSelector`, `master_points_in_base_frame`,
  `pose_mat_to_6d`.
- `ArtecMultiPassScanSessionSettings` 신규 필드:
  - `recovery_selector: Optional[RecoveryPoseSelector] = None`
  - `max_recovery_retries: int = 3`
  - `safe_back_margin_deg: float = 10.0`
  - `recovery_robot_speed_deg_s: float = 10.0`
  - `recovery_turntable_vel_rad_s: float = radians(30.0)`
- `__init__` 에 `self._T_EC` (E→C copy) 저장 — recovery 시 재사용.
- `run()` 루프 추가 변수:
  - `recovery_retry_count`, `n_recovery_attempts`, `n_recovery_succeeded`
  - `next_T_BC_pending: Optional[np.ndarray]`
  - `next_skip_clearpos: bool`
- streaming session 실행 직전, `next_skip_clearpos` 가 True 면
  `s.streaming_settings.reset_to_zero_first` 를 임시 False 로 set (finally 에서
  원복) — multipass 가 manual move_abs 로 위치 정해놨기 때문.
- merge 단계 hint 계산:
  - `next_T_BC_pending` 있으면 camera-motion correction
    `T_pre = T_BC_master @ inv(T_BC_recovery)`, translation × 1000 으로 override
    (기존 R_phys hint 보다 우선).
- tracking-lost 분기:
  - drive-alarm short-circuit 후 `_attempt_recovery` 호출.
  - 성공 시 `recovery_retry_count++`, `n_recovery_attempts++`, `continue`.
  - 한계 초과 / drive_dead / selector None 이면 기존 user-prompt fallback.
  - User 가 수동 retry 한 경우 recovery_retry_count = 0 reset.
- 정상 pass 후 retry_count > 0 이면 `n_recovery_succeeded++`, count reset.
- `ArtecMultiPassScanResult.n_recovery_attempts`, `n_recovery_succeeded` 추가.
- 신규 메서드 `_attempt_recovery(sub_result, master_model, retry_idx)`:
  1. safe-back angle 계산 (§5)
  2. turntable.stop → check_drive_err → set_servo_on → move_abs →
     wait_motion_done
  3. master_pts_B = master_points_in_base_frame(...)
     (비어있으면 return True, None — turntable safe-back 만으로 재시도)
  4. T_CB_now = T_EC @ inv(T_EB_now)
  5. decision = selector.select(...)
     (decision None 이면 return True, None — 같은 자리 재시도)
  6. T_CB_target ≈ T_CB_now (Δ<2mm) 면 robot 이동 skip
  7. xArm.set_position(T_EB_target = T_CB_target @ T_EC), wait=True
  8. T_BC_recovery 캡처, return (True, T_BC_recovery)

### 6.4. 수정 — `main_artec.py`

```python
RECOVERY_STRATEGY: str | None = "local_jitter"   # 또는 "centroid_vector" / None

if RECOVERY_STRATEGY == "local_jitter":
    RECOVERY_SELECTOR = LocalJitterSelector(
        n_candidates=9, trans_mm=30.0, rot_deg=8.0, include_current=True,
        seed=None,
    )
elif RECOVERY_STRATEGY == "centroid_vector":
    RECOVERY_SELECTOR = CentroidVectorSelector(stand_off_mm=250.0)
else:
    RECOVERY_SELECTOR = None

MULTIPASS_SETTINGS = ArtecMultiPassScanSessionSettings(
    ...,
    recovery_selector=RECOVERY_SELECTOR,
    max_recovery_retries=3,
    safe_back_margin_deg=10.0,
    recovery_robot_speed_deg_s=10.0,
    recovery_turntable_vel_rad_s=float(np.radians(30.0)),
)
```

---

## 7. 실행 / 테스트

### 7.1. 예상 콘솔 출력 (tracking-lost 시)

```
⚠ tracking lost: 연속 5 프레임 registration_error<0 — turntable 즉시 정지
session.stop() ...

[StreamingScan 결과]
  종료 사유      : tracking lost: ...
  rotation       : -174.5°
  last_good θ    : -156.7°

[Turntable] post-scan 0° 복귀 ...
  [Turntable] clearpos: -174.5° → logical 0°

━━━━━━━━━━ Recovery #1/3 ━━━━━━━━━━
  failed scan: last-good θ = -156.7°  final θ = -174.5°
  safe-back target (new coords): +27.8° (margin=10.0°)
  turntable @ +27.7° (target +27.8°)
  selector       : local_jitter
  decision       : local_jitter top=j4 visible=1842 (of 10 cand)
  score          : 1842
  → robot move   : x=...  y=...  z=...mm  rpy=(...)°
  T_BC re-captured: trans = (...) mm
  → recovery #1/3 진입 — 자동 재시도

╔══════════════════════════════════════════════╗
║   Multi-pass scan — Pass 2 / max 8   (pose 0)    ║
╚══════════════════════════════════════════════╝
...
[recovery hint] camera-motion correction applied (Δtrans = (...) mm)
  [pass 2 / pose 0] N scan(s) → master (total scans=2)
```

### 7.2. A/B 비교 메트릭

`ArtecMultiPassScanResult` 의:
- `n_recovery_attempts`
- `n_recovery_succeeded`
- `pass_results[i].n_frames`
- `pass_results[i].tracking_lost`

같은 object 로 3회씩 (A/B 각각) 측정해서 비교.

---

## 8. 알려진 가정 / 한계

1. **SDK W frame 가정** (§4.3): "W ≈ camera frame at scan start". 어긋나면 T_pre
   camera-motion correction 이 비뚤어질 수 있음 — GlobalRegistration 후처리가
   보정.
2. **last-good θ 없으면 recovery skip**: tracking 이 시작 직후 안 잡힌 채 lost
   되면 safe-back 기준점이 없음.
3. **Robot 안전성**: xArm 의 IK / joint limit / 자체 collision 검사에 의존.
   selector 가 도달 불가능 pose 추천하면 `set_position` 이 code≠0 으로 실패 →
   같은 자리 재시도.
4. **Raycast occlusion 무시**: Method A 의 visible-point count 는 frustum culling
   만. 물체 뒤쪽 점도 visible 로 카운트. Spider 가 close-range 라 큰 문제 안 될
   가능성 높음.
5. **Centroid-vector 의 "반대편"**: 단순 (a) 구현. NBV-lite (히스토그램 기반)
   확장은 Phase 2 와 중복돼서 보류.
6. **Console encoding (cp949)**: `system.py` 의 ⚡ 이모지가 cp949 콘솔에서 깨짐.
   기존 이슈이며 recovery 와 무관. PowerShell / utf-8 터미널에서는 정상.

---

## 9. 다른 컴퓨터에서 이어서 할 때 체크리스트

### 9.1. 환경 동기화

- [ ] git pull (해당 브랜치)
- [ ] Artec SDK 1.17.3 설치 (`docs/artec0_build_guide.md`)
- [ ] xArm IP 192.168.1.210, Turntable IP 192.168.0.10 / UDP 통신 (TCP 아님)
- [ ] `config/sensor_frames.yaml` 의 `T_EC_artec` 확인 (2026-04-29 hand-eye 결과)
- [ ] `config/calibration/turntable_frame.yaml` 확인

### 9.2. 첫 실행 시 점검

- [ ] `python -c "import main_artec"` 로 import chain 클린
- [ ] `RECOVERY_STRATEGY` 토글 동작 확인 ("local_jitter" / "centroid_vector" / None)
- [ ] Spider 가 사용 중인 스캐너 맞는지 (SP.10.36181288)
- [ ] Spider 광학 상수 (`recovery_pose_selector.py` 상단) 가 spec 페이지와 일치

### 9.3. Recovery 동작 검증 시나리오

1. Phase 1 시작 → 회전 중 손으로 객체 일부 가려서 의도적으로 tracking lost 유도.
2. 자동 recovery 진입 (콘솔에 `━━ Recovery #1/3 ━━` 출력) 확인.
3. turntable 이 last-good 보다 10° 더 뒤로 역회전 했는지 (예상 각도 콘솔 확인).
4. xArm 이 selector 가 결정한 pose 로 이동 (collision 없이) 확인.
5. 재시작된 streaming session 의 timeline 정상 (reg_err ≥ 0) 확인.
6. master 의 `scan_count` 가 증가 (recovery scan 도 누적) 확인.

### 9.4. A/B 비교 실험 절차

같은 물체로:
- (A) `RECOVERY_STRATEGY = "local_jitter"` 로 3회 실행
- (B) `RECOVERY_STRATEGY = "centroid_vector"` 로 3회 실행

비교:
- 성공률 = `n_recovery_succeeded / n_recovery_attempts`
- 회복 후 추가 coverage = master scan_count 차이, final mesh vertex 수 차이
- 총 recovery 시간 = `Recovery #N/3 ...` 블록 wallclock

---

## 10. 미해결 / 향후 작업

- [ ] Method B "방향 선정 (b)" — master mesh normal 히스토그램 기반 NBV-lite
      (Phase 2 NBV 와 통합 시 고려)
- [ ] Recovery 실패 (max 도달 후 user prompt) 까지 갔을 때, user 가 수동으로
      "다음 pose 로 advance" 선택할 수 있는 옵션 (현재는 같은 pose 재시도만)
- [ ] Recovery 동안 캡처한 IScan 의 partial 데이터가 master 와 충돌 시
      자동 reject 옵션 (현재는 항상 merge)
- [ ] xArm reachability pre-check — selector 가 후보를 IK 시도 후 도달 가능한
      것만 score 비교
- [ ] Raycast 에 voxel-based occlusion (open3d HiddenPointRemoval 등) 적용
      — 정확도 ↑, 속도 ↓
- [ ] Phase 2 (자세 변경 + recovery) 결합 — R_phys hint 와 camera-motion
      correction 의 조합 hint

---

## 11. 참고

- `docs/7_artec_phase.md` — Phase 1/2/3 큰 그림
- `docs/8_artec_phase2_pose_disambiguation.md` — pose-rotation hint
- `docs/5_artec_hand_eye.md` — T_EC_artec 결과 / 측정 절차
- `CLAUDE.md` — coordinate frame notation (T_AB 컨벤션)
- memory `project_artec_spider_v1.md` — 센서 spec 메모
- memory `project_artec_tracking_lost_limitation.md` — lost flag 신뢰성 한계
