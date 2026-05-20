# IScan 병합 — 쉽게 이해하기

> Phase 1/2 에서 얻은 여러 IScan 을 master 모델 하나로 합치는 흐름을
> 비유 위주로 정리. 상세 좌표식은 `artec_scanning_pipeline.md` §4.2 참고.

---

## 1. 비유 — 사진 모자이크

각 **IScan** 은 객체 한 면을 360° 돌면서 찍은 사진묶음 (점군).
**master** 는 모든 사진을 한 캔버스에 정렬해서 붙인 모자이크.

문제는, 각 IScan 이 **자기 첫 사진을 (0,0,0) 으로 잡는다**는 점.

```
   IScan_1 의 세계 (W1)             IScan_2 의 세계 (W2)
        ↑z                                ↑z
        │                                 │
        ●─→y  ← 캔 윗면 자세              ●─→y  ← 캔 뒤집은 자세
       /                                 /
      x                                 x
```

W1 과 W2 는 서로 다른 좌표계. 그냥 갖다 붙이면 캔이 두 마리, 90° 어긋난
이상한 모양이 나옴. → **각 IScan 의 점들을 master 좌표(=W1)로 옮길 변환**
이 필요. 그 변환이 곧 `T_pre`.

---

## 2. T_pre 란 무엇인가

`T_pre` = "IScan 의 모든 frame 위치를 master 좌표로 옮길 도장".

IScan 안의 각 사진은 SDK 가 이미 자기끼리는 정합해놨음 (`frame_transformation`).
거기에 `T_pre` 하나를 **왼쪽에 곱**하면 IScan 전체가 master 좌표로 평행이동·회전.

```python
for i in range(scan.frame_count()):
    T_old = scan.get_frame_transformation(i)         # IScan 내부 정합 결과
    scan.set_frame_transformation(i, T_pre @ T_old)  # master 좌표로 끌어옴
```

`T_pre` 가 잘못되면 그 IScan 전체가 잘못된 위치/자세로 master 에 들어감 →
모자이크가 어긋남. **mesh 가 직교하거나 둥둥 떠있으면 거의 항상 T_pre 의심.**

---

## 3. T_pre 는 어떻게 정해지나 — 3가지 경우

`_attempt_recovery` 결과(`next_T_BC_pending`) > `pose_physical_rotations` > 없음 순.

### 경우 ① 아무것도 없으면 — `T_pre = None`

가장 흔한 케이스. **Phase 1 첫 pass**.

- 사용자가 객체 안 돌림 (`R_phys = I`)
- recovery 발동 안 함 (tracking lost 없음)

→ IScan_1 의 자기 W1 좌표가 곧 master 좌표 (= reference). 추가 변환 0.

```
IScan_1 (W1)  ──── 그대로 ────→ master  (master_center = IScan_1 의 vertex 평균)
```

### 경우 ② 사용자가 객체를 돌렸으면 — R_phys hint

**Phase 2**. 사용자가 캔을 손으로 90° 돌리고 [Enter].

이때 IScan_2 의 W2 는 IScan_1 의 W1 에 비해 객체가 90° 돌아간 상태로
찍힘. master 좌표(W1)로 옮기려면 **반대로 90° 되돌려야** 함.

> 핵심: 객체가 +90° 돌았으면 데이터를 −90° 로 보정.

게다가 그 회전을 **카메라 원점이 아닌 객체 centroid** 를 pivot 으로 해야
함. 카메라 원점 pivot 으로 회전하면 객체 중심이 ~30cm 멀리 튀어버려서
정합 파탄.

```
            (c_pass)          (c_master)
              ●                   ●
             /│\                 /│\
   ┌────────┘ │ └──┐    ──→    ┌─┘ │ └─┐     ← centroid 끼리 일치시키며
   │  IScan_2 │    │           │   │   │       반대 방향 회전
   │ 회전된 자세 │              │ master  │
   └──────────────┘             └─────────┘
```

수식:

```
R_phys      : 사용자가 base 좌표에서 객체에 가한 회전 (예: Ry +90°)
R_W         : 그걸 scan world 좌표로 변환  (= T_BC · R_phys · T_CB)
c_pass      : IScan_2 의 모든 vertex 평균 (mm)
c_master    : Pass 1 (IScan_1) 의 vertex 평균 (lock, mm)

T_pre = Translate(c_master) · inv(R_W) · Translate(−c_pass)
```

직관:
1. IScan_2 의 centroid 를 원점으로 옮긴다 (`Translate(-c_pass)`)
2. 객체 회전을 되돌린다 (`inv(R_W)`)
3. master centroid 자리에 갖다 놓는다 (`Translate(c_master)`)

`hints_applied=True` 가 켜지면 → 그 뒤 단계의 **GlobalRegistration 은
자동 skip** (이미 hint 가 정답에 가깝게 끌어놨는데 다시 흩뜨리지 말라고).

### 경우 ③ Recovery 후 — camera-motion override

tracking lost → recovery 가 robot 을 새 자세로 보냄. **객체는 안 돌았지만
카메라가 움직임**. R_phys 와 무관, 우선순위 최상.

```
T_pre = T_BC_master · inv(T_BC_recovery)     ← translation × 1000 (m→mm)
```

직관: "카메라가 옮긴 만큼만 데이터를 반대로 옮겨주면 객체는 제자리".

이게 적용되면 그 iteration 의 `R_phys` hint 는 무시 (둘 다 적용하면
중복 보정).

---

## 4. 흐름 한 눈에

```
사용자가 첫 사진 자세 잡음
       │
       ▼
[Pass 1, pose_idx=0]  ───  T_pre = None   ──→  master ← IScan_1 (그대로)
       │                                        master_center = lock
       │
   사용자가 객체 +90° 돌리고 Enter
       │
       ▼
[Pass 2, pose_idx=1]  ───  T_pre = ② (centroid-pivot, -90°)  ──→  master ← IScan_2
       │                   hints_applied = True
       │
   tracking lost 발생!
       │
       ▼
[Pass 2-retry]  ───────  recovery 가 robot 새 자세로 보냄
                          T_pre = ③ (camera-motion)        ──→  master ← IScan_2_retry
       │
   사용자 [q]
       │
       ▼
artec_process:
   GlobalReg          ──→  hints_applied=True 면 SKIP (위 ② 가 권위자)
   Outliers / Fusion / Texturize
```

---

## 5. 잘못되면 어떻게 보이나 — 진단표

| 증상 | 의심 위치 | 어떻게 확인 |
|---|---|---|
| **mesh 가 두 조각으로 직교** | ② R_phys 의 축/부호 틀림 | 로그의 `[hint pose N] frame=base/scan_world` + `translation = (...)` 확인. `make_axis_physical_rotations("y", ...)` 의 축이 실제 사용자가 돌린 축과 같은지 |
| mesh 일부가 30cm 정도 튀어 나감 | ② centroid pivot 인데 c_pass / c_master 가 객체 중심과 동떨어짐 | 비대칭 객체 → vertex centroid ≠ body 중심. 로그의 `c_pass` / `c_master` 값 비교 |
| Phase 2 적용 후 오히려 더 나빠짐 | ② 가 frame 좌표계 변환을 안 함 | 로그에 `frame=scan_world (fallback)` 보이면 `T_BC` 캡처 실패 — base→scan world 변환 안 일어남 |
| recovery 후 mesh 가 평행이동 어긋남 | ③ T_BC_recovery 갱신 누락 | recovery 로그에 `T_BC re-captured: trans = (...)` 가 찍히는지 |
| **큰 평면 mesh 가 객체와 직교해 붙어있음** | hint 무관, **턴테이블 디스크가 IScan 에 통째로 들어감** | live viewer 의 object_gate 와 별개로 실제 IScan vertices 에 디스크가 들어있는지. tracking-lost-limitation memory 참고 |
| mesh 가 한 면만 살아있고 나머진 노이즈 | Phase 2 가 아예 안 돌고 Phase 1 1-pass 결과만 fusion | `pass_results` 길이 확인. 1이면 Phase 2 진행 안 됨 |

---

## 6. 빠른 체크 — 이번 결과(직교 mesh) 의심 순위

스크린샷이 "Artec Phase 1 (textured)" 라벨 (Phase 2 가 아니라 Phase 1
결과). 두 가지 가능성이 가장 큼:

### (A) Phase 1 1-pass 만 돌고 끝난 경우 → hint 무관

- `pass_results` 길이 = 1
- `hints_applied = False`
- 결과 mesh = IScan_1 한 개의 fusion. T_pre 은 identity.
- 직교 mesh 의 원인은 **그 한 IScan 안의 frame drift** 또는 **턴테이블 등이 같이 fusion 됨**.

확인: 로그에서 `Multi-pass 종료` 블록의 `master scan_count` 와
`master total frames` 값 / `hints_applied` 줄.

### (B) 턴테이블 디스크가 IScan vertices 에 포함됨 → fusion 이 디스크+객체 통합

- live viewer 의 object_gate 는 **표시 필터일 뿐** — 실제 IScan 데이터는
  그대로 (memory `feedback_live_viewer_must_mirror_scan`).
- Spider HYBRID 정합이 빈 디스크에도 success 시켜 lost 안 뜨면 디스크가
  계속 누적 (memory `project_artec_tracking_lost_limitation`).
- Poisson fusion 은 watertight 만들려고 객체+디스크 점군 외피를 덮어 큰
  평면이 객체에 직교해 붙은 모양 생성.

확인: master IModel 의 vertices 를 PLY 로 덤프해서 CloudCompare 로 보기.
디스크 평면이 있으면 (B). 디스크는 보이고 객체 frame 별 drift 가 보이면 (A) + (B).

### (C) Hint 가 잘못 적용된 경우 — Phase 2 가 실제로 돌았다면

- 로그에 `[hint pose 1] translation = (...)` 같은 줄 있어야 함.
- 없다면 R_phys hint 가 발동 안 한 것 (= Phase 2 안 들어감).
- 있는데 translation 이 비현실적(>50mm)이면 frame 변환 또는 centroid 계산
  버그 의심.

---

## 7. 관련 파일 / 함수

- `mms_artec/nbv/artec_multipass_scan_session.py`
  - `_merge_into_master` — 실제 병합 호출
  - `_apply_pre_rotation` — `T_pre @ T_old` 곱셈
  - `_compute_model_centroid` — c_pass / c_master 계산 (50 frame subsample)
  - `_attempt_recovery` → `next_T_BC_pending` 세팅 (경우 ③)
  - `run()` 의 "Pose hint 계산" 블록 — 우선순위 결정
- `mms_artec/system.py` — `artec_process` 안 `hints_applied=True` 시
  GlobalReg skip
- 사용자가 정의: `main_artec.py` 의 `POSE_ROTATIONS =
  make_axis_physical_rotations("y", [0, 90, 180])`
