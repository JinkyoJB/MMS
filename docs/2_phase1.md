# Phase 1 — Streaming SLAM (5면 스캐닝) · 로직 흐름 & 코드 지도

> 턴테이블이 360° 도는 동안 Spider 가 **연속 스캔**으로 윗면+옆면4(=5면)을 한 바퀴에 취득.
> 상위 맥락 `docs/main_flow.md` §2. 캘리브는 `docs/1_calibration.md`.

---

## 1. 무엇을 하나

턴테이블 위 대상물이 도는 동안, **로봇(스캐너)은 고정**, 턴테이블이 azimuth 를 커버.
한 바퀴(360°)로 윗면 + 옆면 4개 = **5면**. 바닥면은 디스크에 닿아 캡처 불가 → Phase 2(flip).

```
   [고정] 스캐너 ──본다──▶ ◐ 대상물 (턴테이블 위, θ 회전)
                              │
                         360° 도는 동안 옆면이 차례로 스캐너 앞을 통과
```

---

## 2. 왜 "스트리밍 SLAM" 인가 (Artec ≠ PhoXi)

| | PhoXi | Artec Spider |
|---|---|---|
| 좌표 기준 | 절대 (T_CO 직접) | 상대 (이전 frame 기준 ICP) |
| 한 frame 놓치면 | 다음 frame 복구 가능 | **tracking 잃으면 scan 망가짐** |
| Phase 1 핵심 | 정확한 hand-eye+θ | **OVERLAP 유지** |
| frame 간격 | 15° step OK | 작은 step(15°도 위험) → **연속 회전** |

→ Artec 은 frame-to-frame 정합이라 **겹침(overlap)** 이 생명. 그래서 띄엄띄엄이 아니라
**천천히 연속 회전 + max FPS** 로 매 frame 이 직전과 겹치게 한다. 한 번 끊기면 그 시점까지만 살아남음.

> ★ **시작 자세 = home 그대로** (2026-05-20 rule). 사용자가 사람 눈으로 조준한 EE 자세에서
> 그대로 360° 회전. 사전 probe/elevation 탐색 안 함(overhead 0). tracking lost 가 나야 비로소
> recovery 흐름(§6)이 적응 자세를 탐색.

---

## 3. 실물 아키텍처 — Spider ↔ Turntable 양방향 피드백

`mms_artec/nbv/artec_streaming_scan_session.py` (`ArtecStreamingScanSession.run()`).

```
              ┌──────────── TrackingState (공유) ────────────┐
              │  frames_ok/failed, consecutive_lost,          │
              │  last_reg_error, tracking_lost, stop_event     │
              └──────▲───────────────────────────▲────────────┘
   frame_callback 갱신│                            │ stop_event 감시
   ┌─────────────────┴───────┐      ┌─────────────┴──────────────┐
   │ Main thread (run)        │      │ TurntableController(thread) │
   │  session.poll_events()   │      │  move_velocity 연속 회전     │
   │  4 watchdog 검사          │      │  getActualPos polling(10Hz) │
   │  live viewer 공급         │      │  stop_event→즉시 stop        │
   │  last-good θ 기록         │      │  drive 통신死 watchdog        │
   └──────────────────────────┘      └────────────────────────────┘
```

### 흐름 (`run()`)
1. **FPS** = min(target, max). HYBRID registration(geometry+texture).
2. **ScanSession** 생성: `set_registration_type(HYBRID)`, `set_pipeline(...)`, `initial_state=PREVIEW`,
   `capture_texture=ALWAYS`. `frame_callback = TrackingState.on_frame`.
3. **턴테이블 logical 0 reset**(clearpos). 실패(drive alarm) → 빈 결과 즉시 반환.
4. **Preview → settle → drain** (preview frame 은 카운트 안 함) → 카운터 0.
5. **start_record + TurntableController 시작**(별도 thread). `vel = 2π/rotation_duration_s`,
   `target = 360°+overshoot`.
6. **Main loop**: 매 tick
   - `session.poll_events()` (SDK 큐 비움 — freeze 방지)
   - θ = `tt_ctrl.actual_pos_rad`, timeline 기록
   - **라이브 뷰어 공급**: OK frame 의 `frame_mesh` + **SDK 정합행렬 `ev.transformation`** 을
     scan-world 로 누적 (θ/hand-eye/yaml **무의존** — 화면 = SDK SLAM 그 자체)
   - **last-good θ** 갱신 (reg_err≥0 인 마지막 θ — recovery rollback 기준)
   - **4 watchdog** 검사(§4) → lost 면 turntable 즉시 정지
   - `tt_ctrl.completed` → 정상 완료 / `aborted` → abort

### 결과 (`ArtecStreamingScanResult`)
`model`(IModel), `n_frames`, `rotation_actual_deg`, `duration_s`, `fps_actual`,
`tracking_lost`, `loss_reason`, `frames_ok/failed`, **`last_good_theta_rad`**(recovery rollback 기준).

---

## 4. TrackingState — 4 watchdog

| # | 조건 | 설정키 | 기본 |
|---|---|---|---|
| 1 | FrameState 연속 정합/재구성 실패 | `consecutive_loss_threshold` | 8 |
| 2 | callback stall (N초 무반응) | `stale_threshold_s` | 2.0 |
| 3 | `registration_error < 0` 연속 (Studio 'tracking lost' 시그널) | `consecutive_reg_err_threshold` | 5 |
| 4 | `registration_error > max` 연속 (정합 품질 급락) | `consecutive_high_err_threshold`/`max_acceptable_reg_error` | 8 / 1.5 |

- warm-up: scan 직후 `reg_err=-1` 은 SDK sentinel → `reg_err≥0` 한 번 본 뒤(`tracking_established`)부터만 (3)(4) 카운트.
- `registration=HYBRID` 필수: ICP-only 면 물체 치워도 빈 디스크에 정합 성공해 lost 가 안 뜸
  (`project_artec_tracking_lost_limitation`). 턴테이블 통신은 **UDP** (TCP 는 sustained polling 시 socket 막힘).

---

## 5. 라이브 뷰어 (Spider 전용 — SDK SLAM 거울)

- 누적 좌표 = **SDK 정합행렬** `FrameEvent.transformation`(sensor→scan-world). θ/yaml/hand-eye **무의존**.
- viewer 는 IScan 을 그대로 비추는 거울 — SDK 가 IScan 에 넣는 기준(`reg_err≥0`)과 **동일하게만** 필터.
- 별도 터미널 Filament 뷰어로 표시 (Isaac 확장 GUI 제약과 동일 — `mms_artec/nbv/live_scan_viewer.py`).

---

## 6. Tracking-lost 자동 recovery (hook)

`ArtecMultiPassScanSession` 이 Phase 1+2 통합 오케스트레이션. lost 시:
- 같은 pose 안에서 최대 **3회** 자동 retry (lost ≠ object-presence 라 무한 retry 금지).
- **safe-back**: `last_good_theta_rad` 보다 10° 더 뒤로 turntable 복귀.
- **`_adaptive_prescan_position(recovery=True)`**: fresh probe + 축소 elevation search → 새 robot 자세.
- 상세는 main_flow §6, view-score 는 main_flow §2.

> **sim 검증** (SLAM 없으므로 캡처 점을 프록시로, 나쁜 자세로 trigger → recovery 가 자세 재선정):
> - `MMS_ext_phase1_recovery1.py` **[빗나감]**: 대상물을 측면으로 빗나가게 조준(FOV 밖)→점≈0→lost.
>   recovery 가 대상물 중심 재조준(elevation 후보 중 **대상물 점 최대**) → 재개.
> - `MMS_ext_phase1_recovery2.py` **[윗면 미포착]**: 너무 낮은 el(옆에서)→윗면 grazing 미포착(**윗면
>   비율** 낮음)→lost. recovery 가 **elevation 올려(윗면비율 최대)** → 윗면 포착 → 재개.
> 둘 다 θ safe-back + 자세 재탐색 + 재개 → 5면 완성. (형상이 어려울 필요 없음 — 자세가 나쁘면 lost.)

---

## 7. sim 검증 — ground-truth 누적 (SLAM 없음)

sim 엔 Artec SLAM 이 없다. 대신 **턴테이블 θ(ground-truth) + 회전축(calibration)** 으로 점군을
누적해 같은 결과(5면 재구성)를 얻고, view-coverage·누적 로직을 검증한다.

```
대상물이 θ 회전 (ground-truth)
   │  매 θ: 카메라 포인트클라우드 캡처(world)
   │        대상물 점만 분리 (디스크 위 + 영역 크롭)
   │        축 둘레로 −θ 역회전 → 대상물 기준 프레임(θ=0)
   ▼
누적 → 5면(윗면+옆면4) 완성 점군
```

- 핵심: `p_obj = Rz(−θ)·(p_world − axis_point) + axis_point` (axis = 턴테이블 회전축, §1_calibration).
  θ 가 정확(sim ground-truth)하면 모든 옆면이 정확히 겹쳐 쌓인다 = "SLAM 대신 GT 누적".
- 스크립트: **`standalone_examples/play/MMS/MMS_ext_phase1.py`** (Isaac 확장).
  대상물(box)을 known θ 로 회전(키네마틱) → 캡처 → −θ 누적 → 재구성. real 의 streaming/SLAM 은
  이 검증된 누적 위에 그대로 올린다.

---

## 8. 코드 지도

```
mms_artec/nbv/artec_streaming_scan_session.py   ★ 실물 streaming SLAM
    ArtecStreamingScanSessionSettings   # rotation_duration_s, registration=HYBRID, 4 watchdog 임계 ...
    TrackingState (.on_frame, .should_stop, 4 watchdog)
    TurntableController (별도 thread, move_velocity, getActualPos polling, stop_event)
    ArtecStreamingScanSession.run() → ArtecStreamingScanResult
mms_artec/nbv/artec_multipass_scan_session.py   # Phase 1+2 오케스트레이션 + recovery
mms_artec/nbv/live_scan_viewer.py               # SDK 정합행렬 누적 뷰어
utils/turntable/turntable_interface.py          # 실물 턴테이블 (move_velocity/getActualPos, UDP)
mms_artec/backends/isaac/isaac_turntable.py     # sim 턴테이블 (RevoluteJoint)

standalone_examples/play/MMS/MMS_ext_phase1.py  # sim 검증(GT 누적)
```

---

## 9. 규약·함정

1. **연속 회전 + max FPS** — overlap 이 생명. 띄엄띄엄 step 금지.
2. **HYBRID registration 필수** — ICP-only 면 빈 디스크에 정합돼 lost 미탐.
3. **시작 = home 그대로** — 사전 probe 안 함. lost 시에만 recovery 가 적응 자세 탐색.
4. **live 누적은 SDK 정합행렬** — θ/yaml/hand-eye 무의존 (화면 = SLAM 결과 그 자체).
5. **sim 은 SLAM 없음** — GT θ + 축으로 누적(−θ 역회전). real 은 그 위에 SLAM 만 얹음.
6. **last-good θ** = reg_err≥0 마지막 θ — recovery safe-back rollback 기준.
7. **턴테이블 UDP** (TCP socket 막힘), 별도 thread + stop_event 로 즉시 정지.
