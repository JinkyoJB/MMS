# Artec Phase 1/2/3 설계 — 2026-04-29 (concept), 2026-05-14 (구현 진행)

> **목적**: `main_artec.py` 의 Phase 1/2/3 컨셉과 동작 기록. Artec 의 상대-좌표
> tracking 특성 + Spider/Turntable 양방향 피드백 설계.

---

## 0. 최종 목표 (잊지 말 것)

> **MMS 파이프라인의 최종 산출물 = 아이템 전면 (full-coverage) 스캔의
> water-tight mesh + texture.**

Phase 1/2/3, tracking-lost watchdog, hand-eye calibration, SDK IO export 등 모든
sub-system 의 존재 이유는 결국 이 한 가지 산출물에 귀결.

---

## 1. 공통 Architecture — Spider ↔ Turntable 양방향 피드백

Phase 1/2/3 모두 같은 양방향 피드백 위에서 동작:

```
                  ┌──────────────────────────┐
                  │  TrackingState (shared)  │
                  │  - frames_ok / failed    │
                  │  - consecutive_lost      │
                  │  - tracking_lost (bool)  │
                  │  - stop_event (Event)    │
                  └──────────────────────────┘
                       ▲                ▲
        callback 갱신  │                │ stop_event 감시
                       │                │
       ┌───────────────┴────┐   ┌───────┴─────────────┐
       │  Main Thread       │   │  Turntable Thread   │
       │                    │   │                     │
       │  ScanSession.      │   │  move_velocity()    │
       │    poll_events()   │   │                     │
       │    (50ms 간격)     │   │  while not stop:    │
       │       ↓ 큐 drain   │   │    pos = poll       │
       │       ↓ callback   │   │    if pos>=target:  │
       │                    │   │      complete; brk  │
       │  if tracking_lost: │   │    sleep(50ms)      │
       │    stop_event.set()│   │                     │
       │  if rotation_done: │   │  finally:           │
       │    break           │   │    stop()  ←── 항상  │
       └────────────────────┘   └─────────────────────┘
```

---

## 2. Phase 1 — 5면 캡처 (구현됨)

연속 회전하는 턴테이블 위에서 Artec Spider 가 **streaming SLAM**
(`IScanningProcedure`) 으로 한 바퀴 돌며 frame-by-frame 정합. **프레임 콜백** 이
tracking 손실을 감지해 **즉시 턴테이블 정지** → overlap 잃기 전에 차단.

물체 공간을 6면체로 추상화하면 Phase 1 한 번의 회전으로 **5면 (윗면 + 옆면
4개)** 이 관찰됨. 바닥면은 turntable 디스크에 닿아 있어 캡처 불가.

Scanning + Alignment 교차 — 연속 회전 360° + frame_callback 실시간 정합.

### 2.1 Why Artec ≠ PhoXi (Phase 1 설계 근본 차이)

| 항목 | PhoXi | Artec |
|------|-------|-------|
| 좌표계 기준 | 절대 (T_CO 직접 계산) | 상대 (이전 frame 기준 ICP) |
| 탈선 시 | 다음 frame 도 T_CO 로 정렬 가능 | tracking 잃으면 **scan 전체 망가짐** |
| Phase 1 핵심 | 정확한 hand-eye + θ | **OVERLAP 유지** |
| Frame 간격 | 15° step OK (큰 jump 가능) | 작은 step (15° 도 위험) → **연속 회전** |

**결론**: Artec 은 turntable 을 천천히 연속 회전 + Spider 가 max FPS 로 따라가야
함. 회전 도중 한 번이라도 tracking 끊기면 그 시점까지의 frame 들만 살아남음
(이후는 정합 X).

### 2.2 결과 데이터 (`ArtecStreamingScanResult`)

```python
@dataclass
class ArtecStreamingScanResult:
    model: ModelHandle               # Artec native (registered frames)
    n_frames: int                    # 총 frame 수 (model 안)
    rotation_actual_deg: float       # 실제 회전한 각도
    duration_s: float                # record + 회전 총 시간
    fps_actual: float                # n_frames / duration
    tracking_lost: bool              # 한 번이라도 lost?
    loss_reason: str                 # "연속 8 프레임 정합 실패 (...)"
    frames_ok: int                   # OK 상태로 추가된 frame
    frames_failed: int               # 실패 카운트
```

---

## 3. Phase 2 — 바닥면 캡처 + Phase 1 정합 (구현 중)

Phase 1 에서 못 본 바닥면의 데이터 취득 후, **현재까지 누적된 데이터를 최대한
이용해서 Phase 1 결과와 정합**하여 mesh 생성하는 단계.

핵심 아이디어:
- 객체를 뒤집어 (또는 옆으로 눕혀) 바닥면이 Spider 시야에 노출되게 함.
- 옆면이 일부 overlap 되도록 자세 선택 (Phase 1 데이터와 정합 가능).
- 새 IScan 으로 캡처하면 SDK 의 scan world 가 Phase 1 과 분리됨 → 정합 hint 필요.

### 3.1 구현: multi-pose multi-IScan 누적 (현재)

`mms_artec/nbv/artec_multipass_scan_session.py` 가 Phase 1+2 를 통합 오케스트레이션.

- Pose 0: canonical (face1 up) — Phase 1
- Pose 1: Ry(+90°) — face5 가 위로 (옆면 보강)
- Pose 2: Ry(+180°) — face6 가 위로 (바닥면)
- 사용자가 매 pose 사이에 객체를 물리적으로 회전, [Enter] 로 다음 pass.
- Tracking lost 발생 시 같은 pose 로 retry — 모든 IScan 이 master IModel 에 누적.

### 3.2 정합 — Pre-rotation hint with centroid pivot

Phase 1 데이터와 Phase 2 데이터를 SDK GlobalReg 만으로 정합하면 대칭 객체에서
local minimum 함정 (face1 ↔ face6 합병). docs/8 참조.

해결: 알려진 물리 회전 R_phys 를 hint 로 사용. 회전 pivot 은 **객체의 centroid**
(scan world 원점이 아닌). 자세한 수식은 docs/8_artec_phase2_pose_disambiguation.md
참조.

```python
T_pre = Translate(c_master) @ inv(R_phys) @ Translate(-c_pass)
# c_master  = Pass 1 의 vertex centroid (master 기준점)
# c_pass    = 현재 pass 의 vertex centroid
# R_phys    = 객체에 가한 물리 회전
# IScan 의 모든 frame transformation 에 좌측 곱
```

Hint 가 authoritative 이므로 post-merge GlobalRegistration 은 자동 skip (hint 를
다시 흩뜨리는 것 방지).

---

## 4. Phase 3 — Hole-fill (미구현)

Phase 1+2 결과 mesh 의 hole / frontier 영역을 탐색하고 추가 회전/움직임으로 보강.

- Fusion 후 mesh 의 boundary edge 검출 → frontier 영역 식별
- 해당 영역을 보는 EE pose 계산 → 로봇 천천히 이동
- Tracking overlap 유지 필수 (빠른 이동 = lost)

구현 우선순위는 Phase 1+2 결과 mesh 품질 검증 후 결정.

---

## 5. 후속 알고리즘 단계 (`ArtecMMS.artec_process`)

`docs/6_artec_process.md` 부록 B 의 §3 SDK General Pipeline:

```
0. Scanning            — Phase 1 + (Phase 2) 의 multi-pass
   └─ ArtecMultiPassScanSession.run() → IModel (N IScans 누적)
1. Alignment           — SerialRegistration       (do_serial_registration=False 기본)
2. Registration        — GlobalRegistration       (hints_applied=True 면 자동 skip)
3. Cleaning            — OutliersRemoval + SmallObjectsFilter   (Fusion 전!)
4. Fusion              — PoissonFusion / FastFusion
5. Simplify            — MeshSimplify (옵션)
6. Texturize           — Texturization
```

`do_serial_registration=False` (default) — streaming 단계에서 SDK 가 이미
frame-to-frame 정합 했으니 외부 SerialReg 중복 안 함.

**Dev mode** (`ArtecProcessSettings.dev_mode=True`) — 빠른 iteration 위해
OutliersRemoval / Simplify 자동 skip. Production export 시 False.

---

## 6. 관련 문서

- `docs/6_artec_process.md` — SDK 알고리즘 파이프라인 전체 순서
- `docs/8_artec_phase2_pose_disambiguation.md` — Phase 2 의 face merging 문제와
  pre-rotation hint 설계
