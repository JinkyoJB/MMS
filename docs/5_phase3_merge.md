# Phase 3 — 바닥면 flip & 병합 (`T_pre`)

> 대상물을 손으로 뒤집어 원판에 닿아 있던 바닥면을 얻고, 앞서 얻은 스캔과 하나로 합친다.
> **결과 메시가 어긋나거나 둥둥 떠 있으면 거의 항상 `T_pre` 문제다.**
>
> 앞 단계 `2_phase1.md` · `3_phase2.md` · 후처리 `6_postprocess.md`

---

## 1. 실행과 설정

`phase_mode = 3` 으로 두면 Phase 1 → 2 → 3 이 이어서 돈다. Phase 3 은 **사람이 물체를
뒤집는 단계**라 자동으로 넘어가지 않고, 회전 안내를 출력한 뒤 `[Enter]` 를 기다린다.

```bash
./scripts/sim/run_e2e_gui.sh spray_can 3 planner    # sim
env -u PYTHONPATH $MMS_PYTHON main_artec.py         # real (BACKEND="real", phase_mode=3)
```

로봇은 Phase 2 에서 움직인 자세를 `go_home` 으로 되돌린 뒤 **home 에 고정**된다. 회전은
사람이 하고 로봇은 그 자리에서 턴테이블 한 바퀴를 찍는다.

### 어떤 자세로 뒤집나

`main_artec.py` 의 한 줄이 전부다.

```python
POSE_ROTATIONS = make_axis_physical_rotations("y", [0.0, 90.0, 180.0])
```

base 프레임 Y 축 기준 회전이고 순서대로 pose 0 = canonical(Phase 1·2 가 쓴 자세),
pose 1 = 옆으로 눕힘, pose 2 = 바닥면이다. 축이나 각을 바꾸려면 이 줄을 고친다.

| 설정 | 뜻 | 기본 |
|---|---|---|
| `pose_physical_rotations` | 자세별 사용자 물리 회전 (base 프레임) | `[I, Ry90, Ry180]` |
| `max_passes` | 전체 IScan 수 상한 (retry 포함) | 6 |
| `prompt_before_first_pass` | 첫 pass 전 사용자 확인 | True |

sim 은 사람이 없으므로 환경변수로 흉내 낸다.

| 환경변수 | 뜻 | 기본 |
|---|---|---|
| `MMS_SIM_FLIP_AXIS` | 뒤집는 축 | `y` |
| `MMS_SIM_FLIP_ANGLES` | 뒤집는 각(°) | `180` |
| `MMS_SIM_FLIP_ASPECT` | 세장형 판정 종횡비(키/지름). 넘으면 90° 추가 | 2.0 |
| `MMS_SIM_FLIP_EL_MIN` / `_MAX` | flip 관측 고도각 하한·상한 | 30 / 70 |

`pose_idx` 는 논리 자세 번호(hint index), `n_pass` 는 실제 IScan 수다. tracking-lost
retry 는 `pose_idx` 를 유지하고(같은 hint), 정상 완료 후 사용자가 `[Enter]` 를 눌렀을
때만 `pose_idx += 1` 이 된다.

---

## 2. `T_pre` 란 무엇인가

**IScan 하나를 통째로 master 좌표로 옮기는 도장**이다.

IScan 안의 프레임들은 SDK 가 이미 자기끼리 정합해 두었다(`frame_transformation`).
거기에 `T_pre` 를 왼쪽에 한 번 곱하면 IScan 전체가 master 좌표로 이동한다.

```python
for i in range(scan.frame_count()):
    T_old = scan.get_frame_transformation(i)
    scan.set_frame_transformation(i, T_pre @ T_old)
```

`T_pre` 가 틀리면 그 IScan 전체가 잘못된 위치·자세로 들어가 모자이크가 어긋난다.

---

## 3. `T_pre` 는 어떻게 정해지나 — 세 경우

우선순위는 `_attempt_recovery` 결과(`next_T_BC_pending`) > `pose_physical_rotations` >
없음 순이다.

### ① 아무것도 없으면 — `T_pre = None`

Phase 1 첫 pass 다. 사용자가 물체를 안 돌렸고 recovery 도 안 났다. IScan_1 의 좌표가 곧
master 좌표가 되고 `master_center` 는 그 vertex 평균으로 고정(lock)된다.

### ② 사용자가 물체를 돌렸으면 — R_phys hint

물체가 +90° 돌았으면 데이터를 **−90° 로 되돌린다.** 그런데 그 회전을 **카메라 원점이
아니라 물체 centroid 를 pivot 으로** 해야 한다. 카메라 원점으로 돌리면 물체 중심이
30cm 쯤 튀어 정합이 파탄난다.

```
R_phys   : 사용자가 base 좌표에서 물체에 가한 회전 (예: Ry +90°)
R_W      : 그것을 scan world 로 변환 (= T_BC · R_phys · T_CB)
c_pass   : 이번 IScan 의 vertex 평균 (mm)
c_master : 첫 IScan 의 vertex 평균 (lock, mm)

T_pre = Translate(c_master) · inv(R_W) · Translate(−c_pass)
```

읽는 순서는 오른쪽부터다. centroid 를 원점으로 옮기고, 회전을 되돌리고, master centroid
자리에 놓는다.

이게 적용되면 `hints_applied = True` 가 켜지고 **후처리의 GlobalRegistration 이 자동으로
skip 된다** — hint 가 이미 정답 가까이 끌어놨는데 다시 흩뜨리지 않기 위해서다.

### ③ Recovery 후 — camera-motion override

tracking-lost 로 recovery 가 로봇을 새 자세로 보낸 경우다. **물체는 안 돌았고 카메라만
움직였다.**

```
T_pre = T_BC_master · inv(T_BC_recovery)      (translation 은 m→mm 로 ×1000)
```

카메라가 옮긴 만큼만 데이터를 반대로 옮기면 물체는 제자리다. 이게 적용되면 그 iteration
의 R_phys hint 는 무시한다 — 둘 다 적용하면 중복 보정이다.

---

## 4. 흐름

```
[pass 1, pose 0]  T_pre = ①      → master ← IScan_1 (그대로), master_center lock
      ↓  사용자가 물체 +90° 돌리고 [Enter]
[pass 2, pose 1]  T_pre = ②      → master ← IScan_2,  hints_applied = True
      ↓  tracking lost
[pass 2 retry ]  T_pre = ③      → master ← IScan_2_retry
      ↓  사용자 [q]
후처리: GlobalReg (hints_applied 면 skip) → Outliers → Fusion → Texturize
```

---

## 5. 코드 지도

```
mms_artec/nbv/artec_multipass_scan_session.py
    make_axis_physical_rotations()   # 자세별 물리 회전 생성
    _get_pre_rotation_for_pose()     # ② hint 계산 (centroid-pivot)
    _compute_model_centroid()        # c_pass / c_master
    _apply_pre_rotation()            # 프레임 변환에 T_pre 좌측곱
    _merge_into_master()             # master 누적
    next_flip() / _print_pose_hint_for_user()
utils/nbv/flip_policy.py             # 세장형 판정, flip 각 정책
utils/nbv/icp_strategy.py            # 정합 게이트 (bbox 팽창·drift)
```

---

# 〔부록〕 Troubleshooting

### T1. 윗면과 바닥면이 같은 면으로 합쳐진다 (face-merging)

새 IScan 은 SDK 가 **자기 첫 프레임 기준** 좌표계로 시작하므로 앞선 IScan 과 아무 관계가
없다. GlobalRegistration 이 초기 추정 없이 identity 에서 출발하면, 대칭이거나 비슷한
물체에서 윗면과 바닥면을 같은 면으로 **오인 합병**한다(local minimum). 대칭이 강할수록
identity cost 가 낮아 함정이 깊다.

Artec Studio 는 사용자가 manual alignment 로 시작 transform 을 주어 피하는데, 우리 코드엔
그게 없다. 그 자리를 §3 ② 의 centroid-pivot hint 가 대신한다. **`hints_applied` 가
켜졌는데도 어긋나면 hint 자체(R_phys, centroid)를 먼저 의심한다.**

### T2. 〔한계〕 비대칭·길쭉한 물체는 centroid 가정이 깨진다

②는 "표면 vertex centroid ≈ 물체 중심"을 가정한다. 길쭉한 물체를 눕히면 보이는 면이
바뀌면서 surface centroid 가 body 중심에서 벗어나 hint 가 틀어진다. 향후 OBB center 로
바꾸는 것을 검토한다.

### T3. 〔미검증〕 flip 후 두 SLAM 스캔의 정합 병합

sim 에서는 GT 로 검증했지만 **실물에서 flip 뒤 병합이 확인된 적이 없다.** hint 를 적용했을
때 GlobalRegistration 을 정말 건너뛰어도 되는지(§3 ②)도 실기에서 봐야 한다. Phase 3 은
`phase_mode = 3` 으로만 켜지므로, 첫 실물은 1 → 2 → 3 순서로 단계적으로 올린다.

### T4. 규약

- `T_pre` 는 **좌측곱**이다. 프레임 변환에 오른쪽으로 곱하면 프레임 로컬 회전이 된다.
- ②와 ③은 **동시에 적용하지 않는다.** ③이 우선이고 그때 R_phys hint 는 버린다.
- `pose_physical_rotations` 는 **base 프레임**(Z up) 기준이다. scan world 로 옮기려면
  `T_BC · R · T_CB` 를 거쳐야 한다. 이걸 빼먹으면 메시가 바람개비처럼 돈다.
