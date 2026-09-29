# flip — 바닥면 취득과 병합 (`T_pre`)

> 대상물을 사람이 뒤집어 원판에 접해 있던 바닥면을 취득하고, 앞서 얻은 스캔과 병합한다.
> **결과 메시가 어긋나거나 부유하면 거의 항상 `T_pre` 문제다.**
>
> 선행 `3_lookaround.md` · `4_nbv.md` · 후처리 `6_postprocess.md`

---

## 1. 실행과 설정

`--until flip` 으로 지정하면 preview → lookaround → nbv → flip 이 연속 수행된다.
flip 은 **사람이 물체를 뒤집는 단계**이므로 자동으로 진행되지 않고, 회전 안내를 출력한
뒤 입력을 대기한다.

```bash
env -u PYTHONPATH $MMS_PYTHON main_artec.py --until flip
```

뒤집기마다 로봇은 먼저 `go_home` 으로 후퇴한다(사람이 물체를 조작하는 동안). 프롬프트는
뒤집기 단위로 확인한다 — `[Enter]` 완료, `[n]` 이번 뒤집기 생략, `[q]` 종료.

뒤집힌 물체는 **새로운 형상**이므로 real 은 lookaround 와 동일하게 preview → 밴드 계획을
다시 수행하여(`plan_flip_poses`) 밴드 자세로 캡처한다. 거리·높이가 뒤집힌 물체에 맞춰진다.
계획이 실패하면 home 고정으로 폴백한다. sim 은 home 고정이다.

### 뒤집기 자세

```python
POSE_ROTATIONS = make_axis_physical_rotations("y", [0.0, 90.0, 180.0])
```

base 프레임 Y 축 기준 회전이며 pose 0 = canonical(lookaround·nbv 가 사용한 자세),
pose 1 = 측면, pose 2 = 바닥면이다. 축이나 각도 변경은 이 한 줄로 한다.

| 설정 | 내용 | 기본 |
|---|---|---|
| `pose_physical_rotations` | 자세별 물리 회전 (base 프레임) | `[I, Ry90, Ry180]` |
| `max_passes` | 전체 IScan 수 상한 (retry 포함) | 6 |
| `prompt_before_first_pass` | 첫 pass 전 확인 | True |

sim 은 사람이 없으므로 환경변수로 모사한다.

| 환경변수 | 내용 | 기본 |
|---|---|---|
| `MMS_SIM_FLIP_AXIS` / `_ANGLES` | 뒤집는 축 / 각(°) | `y` / `180` |
| `MMS_SIM_FLIP_ASPECT` | 세장형 판정 종횡비. 초과 시 90° 추가 | 2.0 |
| `MMS_SIM_FLIP_EL_MIN` / `_MAX` | flip 관측 고도각 하한·상한 | 30 / 70 |

`pose_idx` 는 논리 자세 번호(hint index)이고 `n_pass` 는 실제 IScan 수다. 추적 상실
retry 는 `pose_idx` 를 유지하며, 정상 완료 후 사용자가 진행을 승인한 경우에만 증가한다.

---

## 2. `T_pre` 의 정의

**IScan 하나를 통째로 master 좌표로 이동시키는 변환이다.**

IScan 내부 프레임들은 SDK 가 이미 상호 정합해 두었으므로(`frame_transformation`),
`T_pre` 를 왼쪽에 곱하면 IScan 전체가 master 좌표로 이동한다.

```python
for i in range(scan.frame_count()):
    T_old = scan.get_frame_transformation(i)
    scan.set_frame_transformation(i, T_pre @ T_old)
```

`T_pre` 가 틀리면 해당 IScan 전체가 잘못된 위치·자세로 병합된다.

---

## 3. `T_pre` 산출 — 네 경우

우선순위는 `_attempt_recovery` 결과 > `pose_physical_rotations` > 없음 순이다.

### ① 초기 — `T_pre = None`

lookaround 첫 pass 다. IScan_1 의 좌표가 곧 master 좌표가 되고 `master_center` 는 그
vertex 평균으로 고정된다.

### ② 사용자가 물체를 회전시킨 경우 — R_phys 힌트

물체가 +90° 회전했으면 데이터를 −90° 로 되돌린다. 단, 그 회전은 **카메라 원점이 아니라
물체를 pivot 으로** 수행해야 한다. 카메라 원점 기준으로 회전하면 물체 중심이 30cm 가량
이동하여 정합이 실패한다.

```
R_phys   : 사용자가 base 좌표에서 물체에 가한 회전
R_W      : 이를 scan world 로 변환 (= T_BC · R_phys · T_CB)
T_pre    = Translate(c_master) · inv(R_W) · Translate(−c_pass)
```

오른쪽부터 읽는다 — pivot 을 원점으로 이동, 회전 되돌리기, master 자리에 배치.

이것이 적용되면 `hints_applied = True` 가 설정되고 **후처리의 GlobalRegistration 이
자동으로 생략된다** — 힌트가 이미 정답 근처로 배치했는데 다시 흩뜨리지 않기 위함이다.

### ③ 복구 후 — camera-motion override

추적 상실로 복구가 로봇을 새 자세로 이동시킨 경우다. **물체는 회전하지 않았고 카메라만
이동했다.**

```
T_pre = T_BC_master · inv(T_BC_recovery)      (translation 은 m→mm 변환)
```

카메라 이동량만큼 데이터를 반대로 이동시키면 물체는 제자리다. 이 경우 해당 반복의
R_phys 힌트는 무시한다 — 둘을 함께 적용하면 이중 보정이다.

### ④ flip 밴드 캡처 — ②와 ③의 합성

flip 도 preview → 밴드 계획으로 로봇이 이동하므로 **카메라 이동과 뒤집기가 동시에**
발생한다. base 프레임에서 하나의 식으로 합성한다.

```
T_pre = S⁻¹ · T_BC_master · T_unflip_B · R_B(axis, −θ0) · T_CB_new · S
T_unflip_B : 회전 R_phys⁻¹, 평행이동 = P_m_mid − R_phys⁻¹·P_f_mid
             P_*_mid = 축점 + footprint중심 − (H/2)·축방향
```

합성 점군 검증에서 오차 0.000mm 이며, 카메라 보정만 적용하면 145mm 어긋난다.

**피벗은 무게중심이 아니라 디스크면 + 물체높이/2 다.** 물체는 두 자세 모두 디스크 위에
서므로, 높이 H 인 물체를 뒤집으면 바닥면(h=0)과 뚜껑(h=−H)이 교환된다. 즉 축 위
h=−H/2 를 지나는 수평축 180° 회전이 곧 되돌리기이며, 수평 위치만 두 자세의 footprint
중심 차로 맞춘다. 무게중심 피벗은 두 스캔이 서로 다른 부위를 덮으면 그 차이가 그대로
수직 오차가 된다(실측 +20mm).

H 는 `robust_top_height`(밀도 기준)로 세 추정의 최댓값을 사용한다 — preview(정립)·
preview(뒤집힘)·master 메시(`_flip_pivots_B`).

> ⚠ **미해결.** 올바른 두께로 반사하면 flip 이 물리적으로 맞는 위치에 놓이지만 겹침대
> 최근접 거리가 약 10mm 로 남고, 축·방위를 바꾸거나 ICP 를 수행해도 감소하지 않는다.
> **두 패스가 같은 강체의 같은 면을 담고 있다는 가정 밖의 오차**가 존재한다(안착 시
> 기울어짐, 또는 정확히 180° 가 아닌 뒤집기).

### 수평 자유도 보정

사람이 다시 놓을 때의 **yaw + xy** 는 힌트가 원리적으로 알 수 없다. 따라서
`_flip_global_refine` 이 텍스처 매칭 다음, 전역정합 앞에 **footprint yaw 맞춤**
(`_flip_yaw_fit`)을 수행한다. 힌트 배치에서 축 둘레 yaw 를 3° 간격으로 탐색하고 수평은
옆면 무게중심으로 맞춰 겹침 중앙값이 최소인 각을 선택한다.

채택 조건은 세 가지다 — 8mm 미만, yaw 0 대비 20% 이상 개선, **최솟값이 유일**(두 번째
극소값보다 15% 이상 낮음). 두 극소값이 유사하면 미채택이 올바른 동작이다. 기하만으로는
결정할 수 없기 때문이다.

윤곽(contour) 매칭도 시험하였으나(`scripts/artec/reg_hint_contour.py`) 동일하게 두
극소값이 나온다. 원인은 알고리즘이 아니라 **flip 패스의 데이터 분포**다 — 점의 71% 가
위를 향한 뒷면에 있고 옆면은 6% 에 불과하여(master 13%) 윤곽이 거의 2회 대칭이 된다.
해법은 정합이 아니라 flip 패스도 낮은 고도각 밴드로 옆면을 촬영하는 것이며, 현재
계획기가 그렇게 구성한다.

### 전역 보정 순서

**힌트는 초기값일 뿐이다.** 사람의 회전 오차 때문에 힌트가 틀리면 후처리 GlobalReg 도
생략되어(`hints_applied`) 그대로 고정된다. 따라서 `_flip_global_refine` 이 순서대로
시도한다.

1. **텍스처 특징점 매칭**(`utils/nbv/image_match.py`) — 프레임 원본 사진에서 RootSIFT →
   3D 대응 → RANSAC rigid. **후처리 Texturize 와 무관하다**(스캔 중
   `capture_texture=ALWAYS` 로 저장된 사진을 사용). 회전대칭 물체의 앞뒤·뒤집힘을
   유일하게 구분한 방법이다. 게이트는 inlier ≥ 40 이며, 합성 검증에서 0.08°/0.3mm 다.
2. **기하 전역 정합 합의**(`global_registration.register_consensus`, FGR·FPFH + ICP) —
   base 프레임에서 `T_unflip_B` 를 대체한다.
3. 둘 다 실패하면 힌트를 사용한다.

콘솔 `[flip] …` 과 이벤트 로그 `merge.method`(img|greg|hint|camera)에 사용된 경로가
기록된다.

### 원시 스캔 덤프

병합마다 `output/<RUN_TS>/scan_dumps/scanNN_<stage>_poseK.npz` 에 해당 IScan 의 점
(스캔 월드 mm, 색)·적용된 `T_pre_mm`·`master_T_CB`·`T_BC_new`·`R_phys` 를 남긴다.
master 월드는 scan00(T_pre 항등)이며, SDK 없이 numpy/open3d 만으로 정합을 재현할 수 있다.

검증 스크립트는 `scripts/artec/reg_hint_test.py`(`--H-mm` 로 민감도 확인),
SDK 후처리까지는 `reg_hint_post.py` 다.

---

## 4. 흐름

```
[pass 1, pose 0]  T_pre = ①   → master ← IScan_1 (그대로), master_center 고정
      ↓  사용자가 물체 회전 후 진행
[pass 2, pose 1]  T_pre = ②   → master ← IScan_2,  hints_applied = True
      ↓  추적 상실
[pass 2 retry ]   T_pre = ③   → master ← IScan_2_retry
      ↓  종료
후처리: GlobalReg (hints_applied 면 생략) → Outliers → Fusion → Texturize
```

---

## 5. 테두리 패스

flip 면 패스(el≈70°)만으로는 **바닥 모서리(필렛)** 가 남는다. 실측에서 원판 위 5~30mm
옆면·필렛의 법선이 아래로 9~25° 이므로, 180° 뒤집으면 위로 9~25° 를 향한다. el 70°
에서는 입사각 45~61° 로 50° 필터에 배제되고, lookaround 는 위에서 관측할 수 없다.
결과 메시의 바닥 둘레에 10~20mm 띠가 비게 된다.

따라서 flip 은 "뒤집힌 물체의 lookaround" 로 구성한다. 면 패스 후 **테두리 패스**를
el≈35°(후보 35/30/40/45 중 도달·충돌로 첫 성공)로 한 번 더 수행한다. 측정된 필렛 법선
+12° 기준으로 30~35° 에서 100% 통과하며 45° 는 57% 다. 축거리는
`flip_policy.rim_standoff` 로, 테두리의 카메라 쪽 점이 작동거리 창 중앙에 오도록 하는
닫힌식이다.

**sim 구현 완료, real 보류**(real 은 flip 힌트 합성이 필요하다). 소요는 flip 당
전회전 1회 추가다.

---

## 6. 코드 지도

```
mms_artec/nbv/artec_multipass_scan_session.py
    make_axis_physical_rotations()   # 자세별 물리 회전 생성
    _get_pre_rotation_for_pose()     # ② 힌트 계산
    _flip_pivots_B / _flip_unflip_B  # ④ 피벗·되돌리기
    _flip_global_refine / _flip_yaw_fit
    _apply_pre_rotation()            # 프레임 변환에 T_pre 좌측곱
    _merge_into_master()             # master 누적
utils/nbv/flip_policy.py             # 세장형 판정, flip 각 정책, rim_standoff
utils/nbv/image_match.py             # 텍스처 특징점 매칭
utils/nbv/icp_strategy.py            # 정합 게이트 (bbox 팽창·drift)
scripts/artec/reg_hint_test.py · reg_hint_post.py · reg_hint_yaw.py · reg_hint_contour.py
```

---

# 〔부록〕 문제 해결

### T1. 윗면과 바닥면이 같은 면으로 병합된다

새 IScan 은 SDK 가 자기 첫 프레임 기준 좌표계로 시작하므로 선행 IScan 과 무관하다.
GlobalRegistration 이 초기 추정 없이 identity 에서 출발하면 대칭이거나 유사한 물체에서
윗면과 바닥면을 같은 면으로 **오인 병합**한다(local minimum). 대칭이 강할수록 identity
cost 가 낮아 이 함정이 깊다.

Artec Studio 는 사용자가 manual alignment 로 시작 변환을 제공하여 회피하며, 본 코드에서는
§3 ②의 힌트가 그 역할을 한다. **`hints_applied` 가 설정되었는데도 어긋나면 힌트 자체
(R_phys, 피벗)를 먼저 의심한다.**

### T2. 〔미검증〕 실물에서 flip 후 병합
sim 에서는 GT 로 검증했으나 **실물에서 flip 후 병합이 확인된 적이 없다.** 힌트를 적용했을
때 GlobalRegistration 을 생략해도 되는지도 실기에서 확인해야 한다. flip 은 `--until flip`
으로만 활성화되므로, 첫 실물 운용은 preview → lookaround → nbv → flip 순으로 단계적으로
진행한다.

### T3. 규약
- `T_pre` 는 **좌측곱**이다. 프레임 변환에 오른쪽으로 곱하면 프레임 로컬 회전이 된다.
- ②와 ③은 **동시에 적용하지 않는다.** ③이 우선이며 그때 R_phys 힌트는 폐기한다.
- `pose_physical_rotations` 는 **base 프레임**(Z up) 기준이다. scan world 로 옮기려면
  `T_BC · R · T_CB` 를 거쳐야 한다. 이를 누락하면 메시가 회전한 채로 병합된다.

### T4. 디버그 이미지
flip 도 밴드 캡처이므로 lookaround 와 **동일한 거리 이미지**가
`output/<RUN_TS>/debug/flip/` 에 남는다. 확인 방법은 `3_lookaround.md` §1.
