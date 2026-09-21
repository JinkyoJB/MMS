# flip — 바닥면 flip & 병합 (`T_pre`)

> 대상물을 손으로 뒤집어 원판에 닿아 있던 바닥면을 얻고, 앞서 얻은 스캔과 하나로 합친다.
> **결과 메시가 어긋나거나 둥둥 떠 있으면 거의 항상 `T_pre` 문제다.**
>
> 앞 단계 `3_lookaround.md` · `4_nbv.md` · 후처리 `6_postprocess.md`

---

## 1. 실행과 설정

`stage_until = 3` 으로 두면 lookaround → 2 → 3 이 이어서 돈다. flip 은 **사람이 물체를
뒤집는 단계**라 자동으로 넘어가지 않고, 회전 안내를 출력한 뒤 `[Enter]` 를 기다린다.

```bash
./scripts/sim/run_e2e_gui.sh spray_can 3 planner    # sim
env -u PYTHONPATH $MMS_PYTHON main_artec.py         # real (BACKEND="real", stage_until=3)
```

뒤집기마다 로봇은 먼저 `go_home` 으로 물러난다(사람이 물체를 만지는 동안). 프롬프트는
**뒤집기 하나씩** 묻는다 — `[Enter]` 뒤집었다, `[n]` 이 뒤집기는 건너뛰고 다음 각을
묻는다, `[q]` 종료 (키 하나, Enter 불필요). 90° 를 건너뛰고 180° 만 하려면 첫 물음에
`n`, 두 번째에 `Enter`.

뒤집힌 물체는 **새 형상**이므로 real 은 lookaround 와 같은 preview → 밴드 계획을 다시
돌려(`plan_flip_poses`) 밴드 자세로 캡처한다 — 거리·높이가 뒤집힌 물체에 맞춰진다
(2026-09-21 이전에는 home 고정 한 자세였다). 계획이 실패하면 home 고정으로 폴백. sim 은
아직 home 고정이다.

### 어떤 자세로 뒤집나

`main_artec.py` 의 한 줄이 전부다.

```python
POSE_ROTATIONS = make_axis_physical_rotations("y", [0.0, 90.0, 180.0])
```

base 프레임 Y 축 기준 회전이고 순서대로 pose 0 = canonical(lookaround·nbv 가 쓴 자세),
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

lookaround 첫 pass 다. 사용자가 물체를 안 돌렸고 recovery 도 안 났다. IScan_1 의 좌표가 곧
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

카메라가 옮긴 만큼만 데이터를 반대로 옮기면 물체는 제자리다. recovery 처럼 **물체가 안
돈** 경우엔 그 iteration 의 R_phys hint 를 무시한다 — 둘 다 적용하면 중복 보정이다.

### ④ flip 밴드 캡처 — ② 와 ③ 의 합성 (2026-09-21)

flip 도 preview → 밴드 계획으로 로봇이 움직이므로 **카메라 이동과 뒤집기가 동시에**
있다. 둘을 base 프레임에서 한 식으로 합친다:

```
T_pre = S⁻¹ · T_BC_master · T_unflip_B · R_B(axis, −θ0) · T_CB_new · S
T_unflip_B : 회전 R_phys⁻¹, 평행이동 = c_master_B − R_phys⁻¹·c_pass_B  (무게중심 피벗)
```

`_flip_unflip_B` + `_compose_T_pre_W_mm(T_extra_B=…)`. 합성 검증(합성 점군, θ0=0/37°):
오차 0.000mm, 카메라 보정만 하면 145mm 어긋남.

**힌트는 초기값일 뿐이다.** 사람 손회전 오차와 부분 스캔의 무게중심 가정 때문에 힌트가
틀리면 후처리 GlobalReg 도 건너뛰어(`hints_applied`) 그대로 굳는다(2026-09-21
run_162322: 180° flip 정합 실패). 그래서 `_flip_global_refine` 이 순서대로 시도한다:

1. **텍스처 특징점 매칭** (`utils/nbv/image_match.py`) — 프레임 원본 사진(`frame.image()`
   + `uv()`)에서 RootSIFT → 3D 대응 → RANSAC rigid. **후처리 Texturize 와 무관**하다
   (스캔 중 `capture_texture=ALWAYS` 로 저장된 사진을 쓴다). 회전대칭 물체의 앞뒤·뒤집힘을
   유일하게 구분한 방법(벤치마크 2026-06-11). 스캔월드→master월드를 직접 준다.
   게이트 inlier ≥ 40. 합성 검증: 170° 뒤집힘+이동 → 0.08°/0.3mm.
2. **기하 전역 정합 합의** (`global_registration.register_consensus`, FGR·FPFH + ICP) —
   base 프레임에서 `T_unflip_B` 를 교체.
3. 둘 다 실패면 힌트.

콘솔 `[flip] …` 과 이벤트 로그 `merge.method`(img|greg|hint|camera)에 어느 쪽을 썼는지 남는다.

### 원시 스캔 덤프 — 정합을 오프라인에서 다시 돌리려면

병합마다 `output/scans/<RUN_TS>/scanNN_<stage>_poseK.npz` 에 그 IScan 의 점(스캔 월드
mm, 색)·적용 `T_pre_mm`·`master_T_CB`·`T_BC_new`·`T_scan_color_mm`·`R_phys` 를 남긴다.
master 월드 = scan00(T_pre 항등). SDK 없이 numpy/open3d 만으로 정합을 재현할 수 있다.
SDK 프로젝트째 남기려면 `--sproj`(IScan 별로 보존, 저장 ~47s).

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
때 GlobalRegistration 을 정말 건너뛰어도 되는지(§3 ②)도 실기에서 봐야 한다. flip 은
`stage_until = 3` 으로만 켜지므로, 첫 실물은 1 → 2 → 3 순서로 단계적으로 올린다.

### T4. 규약

- `T_pre` 는 **좌측곱**이다. 프레임 변환에 오른쪽으로 곱하면 프레임 로컬 회전이 된다.
- ②와 ③은 **동시에 적용하지 않는다.** ③이 우선이고 그때 R_phys hint 는 버린다.
- `pose_physical_rotations` 는 **base 프레임**(Z up) 기준이다. scan world 로 옮기려면
  `T_BC · R · T_CB` 를 거쳐야 한다. 이걸 빼먹으면 메시가 바람개비처럼 돈다.


## 테두리(rim) 패스 — 바닥 모서리 (2026-09-18)

flip 면 패스(el≈70°)만으로는 **바닥 모서리(필렛)** 가 남는다. 세제 실측: 원판 위 5~30mm
옆면·필렛의 법선이 아래로 9~25° 라, 180° flip 뒤 위로 9~25° 를 향한다 — el 70° 에서
입사각 45~61° 로 50° 필터에 잘리고, lookaround 는 애초에 위에서 못 본다. 결과 메시의
바닥 둘레에 10~20mm 띠가 빈다.

그래서 flip = "뒤집힌 물체의 lookaround": 면 패스 뒤 **테두리 패스**를 el≈35°(후보
35/30/40/45, 도달·충돌로 첫 성공 — 측정 필렛 법선 +12° 기준 30~35° 에서 100% 통과, 45° 는 57%)로 한 번 더 돈다. 축거리는 `flip_policy.rim_standoff`
— 테두리의 카메라 쪽 점이 작동거리 창 중앙(250mm)에 오는 닫힌식(세제 r=97mm, el 45°
→ 309mm). 컨트롤러 `_flip_extra_passes` 가 `backend.flip_extra_poses()` 를 불러
캡처한다. **sim 구현·real 보류**(real 은 flip hint 합성이 필요 — 스텁 주석 참조).
시간: flip 당 전회전 +1회.
