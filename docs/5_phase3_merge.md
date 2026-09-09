# Phase 3 — 바닥면 flip & 병합 (`T_pre`)

> 대상물을 뒤집어 바닥면을 얻고, 앞서 얻은 스캔과 하나로 합치는 단계.
> **결과 메시가 어긋나거나 둥둥 떠 있으면 거의 항상 `T_pre` 문제다.**
>
> 관련: Phase 1 = `2_phase1.md` · Phase 2 = `3_phase2.md` ·
> 정합 게이트(bbox 팽창) = `3_phase2.md` §4

---

2단계 결과물(Artec SLAM 기반, 퀄리티 좋음)에 **바닥면 추가**:
대상물을 180° 뒤집고 턴테이블 360° 재회전 → 새 스캔을 이전 SLAM 데이터와 병합.

`ArtecMultiPassScanSession` 이 Phase 1+2 를 통합 오케스트레이션. 사용자가 객체를 물리적으로
회전시키며 여러 pose 를 새 IScan 으로 캡처, master IModel 에 누적. **시작 자세 = home 그대로**
(Phase 1 과 동일). 사용자가 자세 바꾼 뒤 [Enter] → robot 은 그 home 에서 360° 회전.

- Pose 0: canonical (face1=top) — Phase 1 / Pose 1: Ry(+90°) 옆면 보강 / Pose 2: Ry(+180°) 바닥면.
- `pose_idx`=논리 자세 인덱스(hint index), `n_pass`=실제 IScan 수(retry 포함). tracking-lost
  retry 는 pose_idx 유지(같은 hint), 정상 완료 + 사용자 [Enter] 시만 `pose_idx += 1`.

### Face-merging 문제 (왜 disambiguation 이 필요한가)
새 IScan 은 SDK 가 **자기 첫 frame 기준** 좌표계로 시작 → 첫 IScan 과 무관. GlobalRegistration
이 초기 추정 없이 identity 에서 출발 → 대칭/유사 아이템에서 윗면(face1)과 바닥면(face6)을 동일면
으로 **오인 합병**(local minimum). 대칭↑ → identity cost↓ → 함정↑. (Studio 는 manual alignment
로 시작 transform 을 줘 회피 — 우리 코드엔 없음.) → §4 의 **centroid-pivot pre-rotation hint** 로 해소.

### 기존 설계/구현 (♻️)
- `mms_artec/nbv/artec_multipass_scan_session.py` — `make_axis_physical_rotations("y",[0,90,180])`,
  Phase 2(바닥면+정합), **centroid-pivot pre-rotation hint** 로 두 자세 모호성 해소(상세 §4).

### 남은 일
- 🔬 flip 후 두 SLAM 스캔의 **정합 병합** 검증 (hint 적용 시 GlobalReg skip 규칙 등)
- ⚠ 비대칭·길쭉한 객체: 눕히면 surface-vertex centroid 가 body 기준 이동 →
  "centroid=body 중심" 가정 깨짐. 향후 OBB center 사용 검토.

> 병합 변환(T_pre) 상세는 §4.

---

## 2. 병합 개요

- Artec SDK `GlobalRegistration` 또는 `utils/nbv/icp_strategy.py::icp_with_gates` / `pick_icp_roll`
- 누적/퓨전: `mms_phoxi/nbv/{tsdf_volume,pcd_accumulate_volume}.integrate_frame / merged_pcd`
- 🔬 Artec 경로로 통합 + 검증
## 3. `T_pre` 란 무엇인가

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

## 4. `T_pre` 는 어떻게 정해지나 — 3가지 경우

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

## 5. 흐름 한 눈에

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
---

