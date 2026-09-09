# v3_scene 이관 — sim 파이프라인 현황과 남은 blocker

v2.usd(구 레이아웃) → v3_scene.usd(260811 신규 하드웨어)로 sim 씬을 바꾼 뒤의 상태.
목표: **sim 에서 파이프라인을 완성해 실물에 그대로 올려도 문제없게** 만든다.

관련: 씬 생성 `scripts/sim/build_scene_v3.py`, IK 도달성 확인 `scripts/sim/ik_follow_target.py`
(방법론은 `2_3Dassets/frame_xarm7_spider_turntable_v2/v3_scene_IK.md`)

---

## 완료

| | 내용 |
|---|---|
| 씬 | STEP → `v3.usd`(형상) → `v3_scene.usd`(씬 오버라이드). 조명·바닥·충돌·턴테이블 kinematic |
| 프림 경로 | `isaac_world.py` 상수 교체(`DEFAULT_USD_PATH`/`DISC_PRIM`/`FRAME_PRIM`/`CAMERA_PRIM`/`OBJECT_PRIM`) |
| 경로 중복 제거 | `isaac_scan_session.py` 가 자체 사본을 갖고 있어 `TURNTABLE_MESH` 가 null prim → `isaac_world` 에서 import 하도록 통일 |
| home 자세 | v2 기준 값이라 카메라가 대상에서 **499mm** 떨어져 preview 0 → `scripts/sim/find_home_pose.py` 로 재산출(작동거리 250mm) |
| phase_mode | sim 만 환경변수를 봐서 `main_artec.py` 설정이 무시되던 것 → `multipass_settings.phase_mode` 를 따르고 env 는 override 로 |

---

## ★ Blocker 1 — sim 인데 **실물 hand-eye 파일**을 쓰고 있다

`T_EC`(EE→카메라)가 **두 경로에서 서로 다른 값**을 쓴다.

| 용도 | 값 | 상태 |
|---|---|---|
| 뷰포인트 IK (`_view_q`) | `self.T_EC = _T_EC_gt()` — USD 실측 | ✅ 새 씬 자동 반영 |
| **점군 캡처** (`capture_points_base`) | `self.mms._T_EC` — `config/sensor_frames.yaml::T_EC_artec` | ❌ **구 장착 기준** |

```
sim GT (USD 실측)   translation(mm)  [ -16.34, -163.87,  +56.86 ]
config 실물 캘리브   translation(mm)  [  +4.33, -177.16,  -59.00 ]
                                      위치 차이 118.4 mm (회전도 부호 반전)
```

**영향**: 캡처한 점군을 118mm 어긋난 hand-eye 로 base 프레임에 변환 → 크롭 영역에
아무것도 안 들어와 `P1 preview 점 부족(0) — 플래너 불가` → legacy sweep 으로 fallback.
이후 스캔 점(14,629)도 엉뚱한 위치에 쌓여 결과 mesh 가 이상해진다.

**원인**: 툴체인저(브래킷→마스터→툴플레이트→어댑터)가 들어가며 link7→카메라 관계가
완전히 바뀌었는데, `config/sensor_frames.yaml` 은 2026-04-29 구 장착 캘리브 값이다.

**조치**
- sim: 캘리브가 필요 없다. USD 가 ground truth 이므로 **캡처도 `_T_EC_gt()` 를 쓰게** 한다.
  (`mms._T_EC` 를 isaac 백엔드에서 GT 로 덮어쓰기)
- real: 툴 장착이 바뀌었으므로 **hand-eye 재캘리브 필수** (`scripts/artec/hand_eye_calib.py`).
  `1_calibration.md` — 센서 교체·재설치 시 다시 잡는다.

---

## Blocker 2 — Phase 1 뷰포인트 IK 전부 실패

```
[isaac_scan] ⚠ Phase1 자세 IK 전부 실패 — home 자세 유지
```

`_view_q` 가 `VIEW_AZIS_DEG`(8방위) × `VIEW_EL_DEG`(30°) 를 훑는데 모두 실패.
⚠ Blocker 1 때문에 지금은 legacy 경로로 빠져 **원인이 가려져 있다** — 1을 먼저 고치고
플래너가 실제 뷰포인트를 낼 때 다시 진단해야 한다.

참고: 순수 IK 는 정상이다. `ik_follow_target.py --selftest` 로 대상 주변 ±150mm 격자에서
24/27 도달. 즉 솔버가 아니라 **뷰포인트 조건(el/standoff/자세)** 이 새 배치와 안 맞는 쪽.

---

## Blocker 3 — Phase 2 NBV 관측자세 없음

```
[isaac_scan] gap유형: 윗면(top)=6 측면/뒷면(side)=15
[isaac_scan] NBV: feasible 관측자세 없음 — 윗면 도달한계(스캐너-link2). z수축 필요.
```

gap 은 찾는데 관측자세가 안 나온다. 충돌 world 치수가 아직 v2 기준일 가능성이 높다
(`utils/collision/robot_collision.py` 의 `nbv_turntable_*`, keep-out).

v3 실측값:
```
상판(universal_plate) 상면 Z=0.510    천장 Z=1.500
벽 X ±0.575 / Y ±0.375               턴테이블 축 (0.365, 0), 원판 반경 119mm, 상면 Z=0.665
로봇 base (0.365, 0, 1.500)          스캔 대상 상면 Z=0.742
```

---

## 순서

1. **Blocker 1** (sim 캡처를 GT T_EC 로) — 이걸 고쳐야 preview 가 살아나고 나머지 진단이 유효해진다
2. Blocker 2 — 플래너가 낸 실제 뷰포인트로 IK 실패 원인 특정
3. Blocker 3 — 충돌 world 치수를 v3 실측으로 교체

## 실물 이관 시 체크

- [ ] hand-eye 재캘리브 (`T_EC_artec`) — 툴체인저 장착 반영
- [ ] `turntable_frame.yaml` (T_B_F0) 재캘리브 — 턴테이블 위치가 바뀜
- [ ] `HOME_JOINTS_DEG["artec"]` — 실물에서도 충돌 없는지 확인 (sim 값은 v3 배치 기준)
- [ ] 충돌 world 치수 실측 반영
