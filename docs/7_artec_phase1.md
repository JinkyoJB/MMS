# Artec Phase 1 설계 및 구현 — 2026-04-29

> **목적**: 현재 `main_artec.py` 의 Phase 1 동작을 기록. Artec 의 상대-좌표
> tracking 특성 + Spider/Turntable 양방향 피드백 설계.

---

## 0. 최종 목표 (잊지 말 것)

> **MMS 파이프라인의 최종 산출물 = 아이템 전면 (full-coverage) 스캔의
> water-tight mesh + texture.**

Phase 1 (turntable 360°), tracking-lost watchdog, hand-eye calibration,
SDK IO export 등 모든 sub-system 의 존재 이유는 결국 이 한 가지 산출물에 귀결.

설계/구현 결정 시 항상 최종 산출물 기준 trade-off:
- 누락된 표면(top, bottom 등)은 어떻게 커버할지 → Phase 2 가 필요한 이유.
- Texture 가 빠지거나 깨진 export 는 incomplete (2026-05-07 SDK
  `Io::saveObjCompositeToFile` 로 OBJ+MTL+PNG export 패치 완료).
- Tracking-lost watchdog 의 false negative (물체 제거 인지 못함) 는 결과 mesh 품질을
  더럽힐 수 있는 보안 결함이며 향후 보완 대상 (§15 참조).

---

## 0.1 한 줄 요약

연속 회전하는 턴테이블 위에서 Artec Spider 가 **streaming SLAM** (`IScanningProcedure`)
으로 한 바퀴 돌며 frame-by-frame 정합. **프레임 콜백** 이 tracking 손실을 감지해
**즉시 턴테이블 정지** → overlap 잃기 전에 차단.

---

## 1. Why Artec ≠ PhoXi (Phase 1 설계 근본 차이)

| 항목 | PhoXi | Artec |
|------|-------|-------|
| 좌표계 기준 | 절대 (T_CO 직접 계산) | 상대 (이전 frame 기준 ICP) |
| 탈선 시 | 다음 frame 도 T_CO 로 정렬 가능 | tracking 잃으면 **scan 전체 망가짐** |
| Phase 1 핵심 | 정확한 hand-eye + θ | **OVERLAP 유지** |
| Frame 간격 | 15° step OK (큰 jump 가능) | 작은 step (15° 도 위험) → **연속 회전** |

**결론**: Artec 은 turntable 을 천천히 연속 회전 + Spider 가 max FPS 로 따라가야 함.
회전 도중 한 번이라도 tracking 끊기면 그 시점까지의 frame 들만 살아남음 (이후는 정합 X).

---

## 2. 사용자 요구 워크플로우 (구현 + 향후)

```
Phase 1 — Scanning + Alignment 교차 (구현됨)
  연속 회전 360° + frame_callback 실시간 정합

Phase 2 — Gap-fill Scanning + Alignment (TODO)
  Phase 1 mesh 의 frontier 영역 탐색
  추가 회전/움직임으로 보강
  단, tracking overlap 유지 필수

Registration / Cleaning / Fusion / Texturize (구현됨)
  ArtecMMS.artec_process 가 SDK General Pipeline §3 순서로 호출
  (docs/6_artec_process.md 부록 B 참조)
```

---

## 3. 이전 구현의 두 문제점

### 3.1 Spider 가 ~20초 후 freeze

증상: 흰색 LED 가 20s 까지만 깜박이다 멈춤. mesh 누적도 거기서 정지.

**원인**: SDK 의 `onFrameScanned` 이벤트 큐를 비우지 않으면 내부 back-pressure 로
인해 capture pipeline 이 멈춘다. 우리는 `session.poll_events()` 를 호출하지 않고
있었음.

### 3.2 Turntable 이 35초 후 계속 돔

증상: Spider 가 멈춰도, main 함수가 종료돼도 turntable 은 계속 회전.

**원인 1**: `move_velocity` 가 명시적 `stop()` 없이는 멈추지 않음.
**원인 2**: 이전 polling loop 가 단순 timeout 기반 — sensor 상태와 무관.
**원인 3**: 예외 발생 시 `turntable.stop()` 도달 못 함 (try/finally 누락).

---

## 4. 새 설계 — Spider ↔ Turntable 양방향 피드백

### 4.1 Architecture

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

### 4.2 TrackingState

`mms_artec/nbv/artec_streaming_scan_session.py::TrackingState`:

```python
@dataclass
class TrackingState:
    frames_ok: int = 0                        # FrameState.OK 카운트
    frames_failed: int = 0                    # 그 외 모두
    consecutive_lost: int = 0                 # 연속 정합 실패
    last_state: Optional[FrameState] = None
    tracking_lost: bool = False               # 한 번이라도 임계 초과
    last_loss_reason: str = ""

    _lock: threading.Lock
    _stop_event: threading.Event              # ★ Turntable thread 가 감시

    def on_frame(self, event: FrameEvent):    # ← Spider frame_callback
        with self._lock:
            self.last_state = event.frame_state
            if event.frame_state == FrameState.OK:
                self.frames_ok += 1
                self.consecutive_lost = 0     # 회복
            else:
                self.frames_failed += 1
                if event.frame_state in (REGISTRATION_FAILED,
                                         RECONSTRUCTION_FAILED,
                                         ADD_TO_SCAN_FAILED):
                    self.consecutive_lost += 1

    def should_stop(self, threshold: int) -> bool:
        # main thread 가 50ms 마다 호출
        with self._lock:
            if self.tracking_lost:
                return True
            if self.consecutive_lost >= threshold:
                self.tracking_lost = True
                self._stop_event.set()        # → Turntable thread 즉시 종료
                return True
        return False
```

**핵심**: 단일 frame 실패는 무시 (일시적 점멸 가능). **연속 N 회 (default 8)** 실패해야만 tracking_lost. OK frame 한 번만 들어오면 카운터 리셋 — 회복 가능.

### 4.3 TurntableController

별도 thread 로 회전:

```python
class TurntableController:
    def _run(self):
        try:
            ok = self.turntable.move_velocity(vel_rad_s, direction=0)
            if not ok: return
            while not self.stop_event.is_set():
                pos = self.turntable.getActualPos()
                if abs(pos) >= self.target_rad:
                    self.completed = True
                    break
                time.sleep(0.05)
        finally:
            self.turntable.stop()             # ★ 무조건 정지
```

- `stop_event` 감지하면 즉시 break → finally 에서 stop
- 360° 도달도 같은 경로로 종료
- 통신 일시 실패는 무시하고 polling 계속

### 4.4 Main Loop

```python
session.set_frame_callback(tracking.on_frame)
session.start_preview(); time.sleep(1.5); session.poll_events()

session.start_record()
tt_ctrl = TurntableController(...); tt_ctrl.start()

try:
    while True:
        if elapsed > timeout:
            tracking.request_stop("timeout"); break

        session.poll_events()                  # ★ SDK 큐 drain (freeze 방지)

        if tracking.should_stop(threshold):
            print("⚠ tracking lost — turntable 즉시 정지")
            break

        if tt_ctrl.completed:
            print("✓ rotation 정상 완료"); break

        time.sleep(0.05)
finally:
    tracking.stop_event.set()                  # 안전 차원 한 번 더
    tt_ctrl.join(timeout=3.0)
    self.turntable.stop()
    session.poll_events()                      # 마지막 큐 drain
    model = session.stop()
```

---

## 5. 구성 옵션 (`ArtecStreamingScanSessionSettings`)

| 필드 | 기본 | 설명 |
|---|---|---|
| `rotation_duration_s` | 30.0 | 360° 한 바퀴 시간 (s) |
| `rotation_overshoot_deg` | 5.0 | 마지막 frame 보장 위해 360° + 여유 |
| `target_fps` | None | None = scanner.max_fps() |
| `capture_texture` | True | 텍스처 매핑 (Phase 6 Texturize 에 필요) |
| `registration_type` | ICP | Hybrid / Texture 가능 |
| `pipeline_flags` | REGISTER + KEYFRAME + MAP_TEXTURE + CONVERT_TEXTURES | |
| `ignore_registration_errors` | True | 부분 실패해도 SDK 가 계속 |
| **`consecutive_loss_threshold`** | **8** | **이 값 이상 연속 실패 → 즉시 정지** |
| `preview_settle_s` | 1.5 | preview 안정화 대기 |
| `post_record_settle_s` | 0.5 | record 종료 후 마지막 frame 캡처 여유 |
| `poll_interval_s` | 0.05 | main loop 의 event drain 주기 |

---

## 6. 결과 데이터 (`ArtecStreamingScanResult`)

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

## 7. 콘솔 출력 예시

```
═══════════════ Artec Streaming Phase 1 ═══════════════
  fps                : 7.0 / max 7.0
  rotation_duration_s: 30.0
  loss_threshold     : 8 consecutive failed frames
  expected frames    : ~210

  reset turntable θ=0 (현재 +5.32°)
  Move Done. (stable=5×10ms)

  start_preview (settle 1.5s)
  start_record + 턴테이블 thread 시작
  [ 1.0s] θ= 12.0°  frames ok=14   fail=0  consec_lost=0
  [ 2.0s] θ= 24.0°  frames ok=28   fail=0  consec_lost=0
  ...
  [29.0s] θ=348.0°  frames ok=199  fail=4  consec_lost=0
  [30.5s] θ=365.7°  frames ok=210  fail=4  consec_lost=0
  ✓ rotation 정상 완료
  session.stop() ...

[StreamingScan 결과]
  종료 사유      : rotation 정상 완료
  duration       : 30.7 s
  rotation       : 365.7°
  scans          : 1
  total frames   : 214  (ok=210  fail=4)
  effective fps  : 7.0
  tracking_lost  : False
```

또는 tracking 잃은 경우:

```
  [14.7s] θ=178.6°  frames ok=98   fail=12  consec_lost=8
  ⚠ tracking lost: 연속 8 프레임 정합 실패 (last=REGISTRATION_FAILED) — turntable 즉시 정지
  session.stop() ...

[StreamingScan 결과]
  종료 사유      : tracking lost: 연속 8 프레임 정합 실패 ...
  duration       : 15.2 s
  rotation       : 179.1°
  total frames   : 98
  tracking_lost  : True
```

---

## 8. 후속 알고리즘 단계 (`ArtecMMS.artec_process`)

`docs/6_artec_process.md` 부록 B 의 §3 SDK General Pipeline:

```
0. Scanning (this doc)
   └─ ArtecStreamingScanSession.run() → IModel
1. Alignment       — SerialRegistration
2. Registration    — GlobalRegistration
3. Cleaning        — OutliersRemoval + SmallObjectsFilter   (Fusion 전!)
4. Fusion          — PoissonFusion / FastFusion
5. Simplify        — MeshSimplify (옵션)
6. Texturize       — Texturization
```

`do_serial_registration=False` (default) — streaming 단계에서 SDK 가 이미 frame-to-frame
정합 했으니 외부 SerialReg 중복 안 함.

---

## 9. 미완 / TODO

### 9.1 Phase 2 — Gap-fill Scanning (다음 사이클)

Phase 1 mesh 가 어느 정도 만들어진 후, **frontier (boundary edge)** 영역을 추가
스캔. Artec 의 상대 tracking 특성상:

**Option A — 같은 IScan 연장**: Phase 1 끝나도 session 안 끊고 robot/turntable 을
**천천히** 움직이며 추가 frame. tracking 유지 필수 → 빠르게 움직이면 lost.

**Option B — 새 IScan + 후속 GlobalReg**: Phase 1 끝나면 session.stop() 후 새 scan
시작. 이후 model 안에 IScan 이 여러 개 → GlobalRegistration 으로 정합. Artec
Studio 의 표준 multi-scan 워크플로우.

→ 일단 Phase 1 양방향 피드백 검증 후 Option B 가 안전해 보임.

### 9.2 Tracking 회복 시도

현재는 lost 즉시 종료. 향후:
- lost 감지 → turntable 살짝 역회전 → Spider 가 마지막 OK frame 영역 다시 보게
- consecutive_lost 가 다시 0 으로 떨어지면 record 재개
- 실패하면 그 시점 stop

복잡도 ↑. 사용자가 "여기까지의 mesh 만 살리자" 가 더 실용적.

### 9.3 진행률 시각화

현재 콘솔 로그만. Open3D 로 누적 mesh 실시간 표시는 streaming SDK 의 특성상
까다로움 (model 이 process 도중엔 read-only). Phase 2 부터 고려.

### 9.4 회전 속도 자동 튜닝

`rotation_duration_s` 가 너무 빠르면 tracking 못 따라감. 자동:
- 첫 30° 시도 → 성공률 보고 속도 조정 → 본 회전.

---

## 10. 파일 위치

```
mms_artec/nbv/
  artec_streaming_scan_session.py   ← 본 문서의 구현
    ├── ArtecStreamingScanSessionSettings  (config dataclass)
    ├── TrackingState                       (frame_callback 이 갱신)
    ├── TurntableController                 (별도 thread)
    ├── ArtecStreamingScanResult            (반환 dataclass)
    └── ArtecStreamingScanSession           (orchestrator)

mms_artec/nbv/
  artec_scan_session.py             ← discrete (15°×24) — 비교/legacy

mms_artec/system.py
  ArtecMMS.artec_process()          ← Phase 1 + 후속 알고리즘 통합 호출

main_artec.py                       ← entry point
```

---

## 11. 한 줄 요약 (재)

**Spider 의 frame_callback 과 Turntable 의 thread 가 `stop_event` 로 동기화.**
**poll_events 가 SDK 큐를 정기 drain 해서 freeze 방지.**
**tracking 잃으면 turntable 즉시 stop() — overlap 깨지기 전에 차단.**

---

## 12. 라이브 진단 — 2026-04-29 첫 시도

### 12.1 관측된 현상

```
fps                : 15.0 / max 15.0
rotation_duration_s: 30.0
expected frames    : ~450

[ 1.0s] θ= -11.0°  ok=6   ...
[ 2.0s] θ= -23.3°  ok=11  ...
...
[15.0s] θ=-180.0°  ok=88  fail=0  consec_lost=0
[16.0s] θ=-191.3°  ok=92  fail=0  consec_lost=0
[17.0s] θ=-203.3°  ok=92  fail=0  consec_lost=0   ← FROZEN
[18.0s] θ=-215.6°  ok=92  fail=0  consec_lost=0
...
[30.0s] θ=-359.6°  ok=92  fail=0  consec_lost=0
✓ rotation 정상 완료
total frames: 90  effective fps: 2.9  tracking_lost: False
```

**Spider 의 흰 LED 가 ~16-20s 에 깜박임 멈춤.** 이후 turntable 만 계속 회전,
Spider 는 idle. **`fail` 카운터도 0 유지** — frame_callback 자체가 안 들어옴.

### 12.2 원인 — `max_frame_count = 100` (SDK 기본값)

```python
>>> from mms_artec.sensor.artec_scanning import ScanSessionSettings
>>> s = ScanSessionSettings.default()
>>> print(s.max_frame_count)
100
```

`IScanningProcedure` 의 `max_frame_count` 가 **0 (unlimited)** 이 아니라 **100**.
이 값에 도달하면 SDK 가 **silent auto-stop** — capture pipeline 정지, frame_callback
도 더 이상 안 fire.

92 OK frame + preview 단계의 dropped/recorded 몇 개 = 내부 카운터 100 도달 → 정확히
관측된 시점에 멈춤.

### 12.3 왜 우리 callback 으로는 못 잡았나

내부 카운터에는 **OK 가 아닌 frame** 도 포함됐을 가능성. SDK 가 stop 결정한 이후에는
**onFrameScanned 자체를 fire 안 함** → 우리는 "정합 실패" 로 인식 못 함. SDK 의
관점에선 정상 종료 (요청한 max_frame_count 달성). 외부에서는 freeze 처럼 보임.

### 12.4 부수 관측

- **`fps_actual = 2.9`** (expected 15.0). Spider Gen3 는 **`MAP_TEXTURE +
  CONVERT_TEXTURES` 파이프라인 활성화 시 실효 5-7 FPS** 가 한계 (texture 변환이 가장
  느린 단계). 15 FPS 설정해도 처리 못 따라감.
- **Turntable 회전이 음수** (`-359°`). `move_velocity(direction=0)` 의 결과가 우리가
  의도한 방향과 반대. 현재 setup 에서는 `direction=1` 이 +z 회전일 가능성.
- **모터 0° 복귀 시 `move_abs 실패 (return=133)`** — drive 가 -360° 위치에서 다른
  명령 받기 전 settling 필요한 상태일 수 있음. 또는 음수 위치라 절대 명령이 거부됨.

### 12.5 수정

**A. `max_frame_count = 0` (unlimited) 강제**

`ArtecStreamingScanSession.run()` 의 settings 구성에 추가:

```python
settings = ScanSessionSettings.default()
settings.set_max_frame_count(0)        # ★ unlimited
```

→ SDK 가 외부 stop (우리 `session.stop()`) 까지 capture 계속.

**B. (선택) 무거운 텍스처 파이프라인 옵션화**

기본값을 `MAP_TEXTURE | CONVERT_TEXTURES` 빼고 더 가볍게:

```python
pipeline_flags: int = (
    REGISTER_FRAME |
    FIND_GEOMETRY_KEYFRAME |
    # MAP_TEXTURE / CONVERT_TEXTURES 는 옵션
)
```

`capture_texture=ALWAYS` 만 두고 fusion 후 별도 Texturize 단계에서 처리. fps 가
~2.9 → ~7-12 FPS 로 회복 기대.

**C. Turntable 회전 방향 — 자동 부호 결정**

`getActualPos()` 가 음수로 가면 `direction` 을 자동으로 뒤집기. 또는 단순히
`direction=1` 으로 하드코딩 후 부호 검증.

**D. 0° 복귀 절차 보강**

리턴 133 (Ezi-SERVO `FMM_NotMoveStateBitOff` 일 가능성): drive 가 servo on 상태
유지하지만 motion bit 가 안 풀린 상태. 호출 전 `clean_error()` 또는 짧은 sleep.

### 12.6 우선순위

| Fix | 영향 | 효과 |
|---|---|---|
| **A** (`max_frame_count=0`) | 1 줄 추가 | **freeze 즉시 해결** ★ 필수 |
| B (텍스처 분리) | 2-3 줄, fps 개선 | 회전 30s 동안 더 많은 frame |
| C (회전 방향) | direction 부호 | 양수 회전으로 가독성 ↑ |
| D (복귀 보강) | settle/clean | 부수 에러 정리 |

A 만 해도 본 증상 해결. B 는 frame 수 확보를 위해 추후. C/D 는 cosmetic.

### 12.7 90 frame / 360° 의 의미

Phase 1 결과 90 frame × 360° = 4° 당 1 frame. Spider 의 small FOV (~150mm) 와
turntable rotation 으로 4° 마다 약 95% overlap → SDK 의 ICP 가 정합 충분히 가능.
실제 `consec_lost=0` 으로 모든 frame OK. **tracking quality 자체는 좋음**.

→ max_frame_count 만 풀면 동일 회전에서 ~210 frame (FPS 7×30s) 또는 텍스처 분리
시 ~360 frame (FPS 12×30s) 기대.

---

## 13. 사용자가 짚어준 설계 누락 — Frame stall watchdog

### 13.1 문제

§12 의 라이브 시도에서 사용자가 정확히 짚은 의문:
> *"frame 92에서 멈췄으면 tracking_flag 가 false 가 됐어야 하는데 왜 turntable 은 계속 돌아?"*

내 원래 `TrackingState.should_stop()` 은 **"FAILED frame 이 N회 연속"** 만 검사.
하지만 SDK silent auto-stop 의 경우 **callback 자체가 안 fire** — OK 도 FAIL 도
없음. `consecutive_lost` 0 그대로 → tracking_lost 미설정 → turntable 계속 회전.

내 설계의 가정이 틀렸음:
> "tracking 잃으면 FAILED callback 들이 들어온다"

실제론 tracking 이 아니라 **capture pipeline 자체가 silent 하게 중단**된 케이스가
존재함. 정합 문제와 다른 차원.

### 13.2 보강 — Frame Stall Watchdog

**핵심**: 마지막 callback 시각을 기록하고, **N초 무반응이면 lost 판정**.

```python
@dataclass
class TrackingState:
    last_frame_time: float = 0.0      # 매 callback 갱신

    def on_frame(self, event):
        self.last_frame_time = time.time()    # state 무관 갱신
        ...

    def should_stop(self, threshold, stale_s):
        # (1) 연속 정합 실패
        if self.consecutive_lost >= threshold: ...
        # (2) callback stall ★
        if time.time() - self.last_frame_time > stale_s:
            self.tracking_lost = True
            self.last_loss_reason = f"frame stall {silence:.1f}s..."
            self._stop_event.set()
            return True
```

기본 `stale_threshold_s = 2.0`. FPS 3-15 환경에서 매 frame 0.07-0.33s 간격이면,
2초 무반응 = 6-30 frame 누락 = 명백한 freeze.

### 13.3 두 검출 매커니즘 비교

| 검출 | 트리거 조건 | 잡는 케이스 |
|------|-----------|-----------|
| 연속 정합 실패 | FAILED callback N회 연속 | OVERLAP 잃은 tracking lost (callback 살아있음) |
| **Frame stall** | 마지막 callback 후 N초 침묵 | **SDK silent stop / USB freeze / capture pipeline 정지** |

§12 의 freeze (max_frame_count=100 도달) 는 **검출 (2) 가 잡았어야 할 케이스**.
이전 코드엔 (2) 가 없어서 못 잡았음.

### 13.4 사용자 질문 답

> *"지금 코드 바꿨으니까 내가 의도한대로 움직일까?"*

두 가지 fix 가 함께 적용된 상태:

1. **`max_frame_count = 0`** — 100 한계 제거 → 정상 케이스에선 freeze 안 일어남
2. **stall watchdog** — 그래도 어떤 이유로든 callback 끊기면 2초 안에 감지 →
   `tracking_lost = True` → `stop_event.set()` → turntable thread 즉시 종료

따라서 **이제 의도대로 동작**:
- 정상: 360° 풀 회전 + 모든 frame OK
- 이상: 어디서든 멈추면 2초 안에 turntable 도 같이 멈춤
- max_frame_count 같은 SDK 트랩에 또 걸려도 stall watchdog 가 잡음

### 13.5 동작 검증

```python
# 단위 테스트 결과
ts.mark_started()
ts.on_frame(Ev(OK))
ts.should_stop(8, 2.0)    # 직후: False
time.sleep(0.5)
ts.should_stop(8, 2.0)    # 0.5s: False (2s 임계 미달)
time.sleep(2.0)
ts.should_stop(8, 2.0)    # 2.5s: True
# reason: "frame stall 2.5s (callback 무반응) — SDK auto-stop 또는 capture freeze 가능"
# stop_event set: True
```

OK frame 들어오면 시각이 갱신되니까 정상 동작 중엔 절대 trigger 안 됨.

---

## 14. 다음 사이클

1. fix A (max_frame_count=0) + stall watchdog 적용 후 재실행
2. 정상 케이스: 360° 풀 회전 + frames > 200 + tracking_lost=False
3. 만약 또 어디서 멈추면: 이번엔 stall watchdog 가 잡고 turntable 도 같이 정지 →
   로그에 `frame stall X.Xs ...` 출력 → 어디서 freeze 됐는지 시간 정확히 알 수 있음
4. fps 가 여전히 2-3 이면 fix B (텍스처 분리) 시도
5. 그 후 SerialReg/GlobalReg/Fusion/Texturize 정상 통과 확인
6. Phase 2 (gap-fill) 설계 이동

---

## 15. Studio 비교 검증 — 2026-05-07: tracking-lost 시그널의 한계

### 15.1 검증 동기

Phase 1 streaming 회전 중 watchdog 들이 잘 동작하는지 검증하기 위해, 회전 중에
물리적으로 **(a) 손으로 가림** / **(b) 아이템 제거** 두 케이스를 만들어 봤다.
우리 코드와 Artec Studio 양쪽에서 같은 setup 으로 비교.

병행 실행을 위해 Phase 1 과 동일한 회전 (`vel = 2π/30 rad/s`, 365°) 만 도는
standalone 스크립트 추가:

```
scripts/turntable/phase1_speed_rotation.py
```

이걸 돌려놓고 Studio 의 Recording 으로 동시 캡처해서 Studio 의 'tracking lost'
표시가 언제 뜨는지 관측.

### 15.2 watchdog 시도 변천 (이날 한 일)

| 단계 | 검출 방법 | 결과 |
|---|---|---|
| (a) FrameState 기반 | `REGISTRATION_FAILED` 등 N회 연속 | 아이템 빼도 OK 만 들어옴 — 못 잡음 |
| (b) Vertex count 휴리스틱 | OK 라도 vertex < 1500 N회 → lost | 빈 디스크도 vertex 가 충분히 나옴 — 신뢰도 낮음 |
| (c) Native `registrationError` < 0 | SDK Studio 와 동일 시그널 | warm-up 의 -1.0 sentinel 처리 후, healthy 정착 시 작동. 그러나… |
| (d) `registrationError > max_acceptable` (양수 임계) | regErr 가 baseline 대비 outlier | hybrid 라도 빈 배경 정합 시 reg_err 가 baseline 범위 안에 머물러 또 못 잡음 |

(c) 에서 SDK binding 에 `RegistrationInfo.registrationError`, `geometryKeyFrame`,
`textureKeyFrame` 노출 (`artec_scanning_binding.cpp`). RegistrationType 도 `ICP` →
`HYBRID` (Studio 기본) 로 전환.

### 15.3 Studio 실험 결과 (결정적)

회전 중 두 시나리오:

- **손으로 가림**: Studio 가 'tracking lost' 정상 표시. 손 치우면 자동 복귀.
- **아이템 제거**: 'tracking lost' **거의 안 뜸**. 아이템을 집을 때 한 순간만
  깜빡할 뿐, 제거 후엔 Studio 가 빈 턴테이블/배경에 정합 성공시켜 계속 정상 표시.

### 15.4 결론 — `tracking lost` ≠ 올바른 대상 추적

**SDK 의 `RegistrationInfo.registrationError < 0` / `FrameState_RegistrationFailed`
는 "어떤 정합 솔루션도 못 찾음" 시그널이지 "올바른 대상 추적 중" 시그널이 아니다.**

물리적으로 설명:
- **손으로 가림**: 손 빠른 움직임으로 텍스처+geometry baseline 모두 깨짐 → 정합 자체
  실패 → lost 정상 표시.
- **물체 제거**: scanner 시야엔 여전히 "뭔가" (턴테이블 디스크, 배경) 가 있음.
  Hybrid (geometry+texture) 가 그 배경에 정합 성공 → reg_err 작은 양수 → SDK 입장에선
  tracking 정상. **잘못된 대상을 잘 추적하는 상태**.

Studio 도 이 한계를 그대로 가짐. 즉 callback 레벨 native 시그널만으로는 **물체-제거
검출 불가능**.

### 15.5 함의

§0 의 최종 목표 (water-tight mesh + texture) 관점에서, 이 false negative 는 결과 mesh
품질을 직접 더럽히는 결함:
- 사용자가 모르고 아이템을 건드리거나 빼면, scan 은 계속 진행되어 잘못된 frame 들이
  scan 에 추가됨.
- Phase 1 끝나고 fusion 하면 의미 없는 mesh 가 만들어질 수 있음.

향후 보완 후보 (CLAUDE.md 의 "Phase 1 에 crop 금지" feedback 은 그대로 존중):
- **Frame mesh centroid jump**: 직전 프레임 대비 centroid 가 갑자기 점프하면 다른
  표면 추적 시작 신호. False positive 가능성 (occluded 부위 바뀔 때 출렁임).
- **Geometry keyframe rate**: SDK 가 한참 동안 새 geometry keyframe 안 만들면 의미
  있는 새 geometry 가 안 들어오는 신호. (방금 binding 에 노출한 `geometry_keyframe`
  플래그 활용.)
- **Vertex 개수 baseline drop**: healthy 초반 N프레임 mean 잡고 25-30% 미만으로
  떨어지면 lost. (기각된 bare-threshold 휴리스틱 보다 robust.)

### 15.6 현 코드 상태

`mms_artec/nbv/artec_streaming_scan_session.py` 의 watchdog 들은 모두 살아있음:
- (1) FrameState 기반 `consecutive_lost`
- (2) Frame stall (`stale_threshold_s`)
- (3) `registration_error<0` 연속 (`consecutive_reg_err_threshold`)
- (4) `registration_error > max_acceptable` 연속 (`consecutive_high_err_threshold`)

이 watchdog 들은 **손-가림 / 너무 빠른 움직임 / SDK silent freeze** 를 잡는다.
**물체 제거는 못 잡는다** — 알고 운영. 별도 검출 메커니즘은 별도 사이클에서 도입.

### 15.7 부수 패치 (이날 함께 처리)

- `mms_artec/sensor/artec_base_binding.cpp::model_save_obj` — 수기 OBJ 라이터 →
  SDK `Io::saveObjCompositeToFile` 로 교체. **OBJ + MTL + PNG 동반 export**.
  이전 export 는 `vt`/`mtllib`/`usemtl` 모두 누락해 텍스처 손실.
- `main_artec.py` — output 4종 (obj/mtl/png/sproj/timeline.csv) 에 실행 시점
  `_YYYYMMDD_HHMMSS` 박힘 (run 별 구분).
- `main_artec.py:207` — streaming 모드의 `result.ctx=None` 참조 AttributeError 수정.
- `scripts/turntable/phase1_speed_rotation.py` — Phase 1 동일 속도 standalone
  회전 스크립트 (Studio 비교용). 거리 기반 종료 + drive `clearpos()` fallback.
