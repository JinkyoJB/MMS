# 규약과 구조 — 좌표계 · 듀얼 백엔드 · 문서 지도

> 코드를 읽기 전에 먼저 보는 문서다. 단계별 설계·근거는 각 전용 문서에만 둔다.

---

## 1. 좌표계 규약

```
T_AB : 프레임 A → B 변환      x_B = T_AB @ x_A
체인 규칙 : T_AC = T_AB @ T_BC      (중간 프레임 B 가 약분)
```

코드·주석·문서 전부 `T_AB` 형식만 사용한다(`^A T_B`, `T_A^B` 금지).

| 기호 | 프레임 | 설명 |
|---|---|---|
| **B** | Base | xArm7 로봇 베이스 (≡ 월드) |
| **E** | End-Effector | 로봇 플랜지 / TCP |
| **C** | Camera | 카메라 광학 프레임 |
| **F** | Turntable | 회전축 중심 원점, z축 위쪽 |
| **O** | Object | 첫 스캔 기준 내부 글로벌 프레임 |

**상수** (캘리브레이션으로 결정)

| 변환 | 의미 | 출처 |
|---|---|---|
| `T_EC` | E → C (hand-eye) | `config/sensor_frames.yaml::T_EC_artec` |
| `T_B_F0` | B → F (θ=0) | `config/calibration/turntable_frame.yaml` |

**가변값** (매 스텝 계산) — `T_FB(θ) = T_FB0 @ Rz(θ)` · `T_BF(θ) = inv(T_FB(θ))` ·
`T_EB` 는 로봇 FK 실시간(`XArmInterface.get_ee_pose_mat()`) · `T_CB = T_EB @ inv(T_EC)`.

### ⚠ 단위 혼용 — 버그 1순위

| 출처 | translation |
|---|---|
| `get_ee_pose_mat()`, yaml `T_EC` | **m** |
| `xarm.set_position(x,y,z,…)` | **mm** |
| Artec SDK `frame_transformation` / vertices / master pts | **mm** |

따라서 camera-motion `T_pre` 의 translation 만 `× 1000` 스케일한다(`5_flip.md` §3③).

---

## 2. sim / real 듀얼 백엔드

`ArtecMMSConfig.backend = "real" | "isaac"` 하나로 robot/turntable/scanner 를 일괄
교체한다.

| | real | isaac (sim) |
|---|---|---|
| robot | `XArmInterface` (xArm SDK) | `IsaacXArm` (해석적 운동학 + sim) |
| turntable | `Turntable` (Ezi-SERVO) | `IsaacTurntable` (RevoluteJoint 드라이브) |
| scanner | `ArtecClient` (Artec SDK) | `IsaacArtecScanner` (Isaac 카메라) |

**개발 전략** — 로직(캘리브·자세 계획·병합)을 sim 의 ground-truth 로 개발·검증하고,
real 에서는 Artec SLAM 위에 그대로 올린다. Artec 실시간 SLAM 은 real 전용이며 품질이
우수하다. sim 에는 SLAM 이 없으므로 θ·카메라 자세 ground-truth 로 점군을 누적하여
동일 로직을 검증한다.

진입점은 `main_artec.py` 이고 `BACKEND` 또는 `MMS_BACKEND` 로 전환한다.
**백엔드마다 파이썬이 다르다.**

| 백엔드 | 파이썬 |
|---|---|
| `real` | conda `mms-env` (py3.11) |
| `isaac` | conda `env_isaacsim` (NVIDIA 번들 `~/isaacsim/python.sh` 도 동작하나 별도 설치본이므로 conda deactivate 필요) |

⚠ 셸에 ROS `PYTHONPATH` 가 잡혀 있으면 python3.10 패키지가 혼입되므로
`env -u PYTHONPATH` 가 필수다.

sim 씬 기본값은 `isaac_world.py::DEFAULT_USD_PATH` = 실물 배치를 재현한
`frame_xarm7_spider_turntable/v2_real_260917.usd` 이며 `MMS_SIM_USD` 로 변경한다.
**v3 는 실물 배치가 아니다**(`sim_scene.md` §1). 장비 없이 실물 기하만 점검하려면
`scripts/artec/validate_real_cell.py` 를 사용한다.

백엔드 상세는 `mms_artec/backends/README.md`.

---

## 3. 자주 바꾸는 값

**작동거리 창(스캔 range)** — 실물에서 거리가 멀거나 가까울 때는 한 곳만 수정한다.

```python
# main_artec.py :: CFG = ArtecMMSConfig(artec=ArtecConfig(...))
scan_range_near_mm = 210.0     # None = SDK 기본값
scan_range_far_mm  = 265.0
```

**셀 배치 변경** — 로봇 연산(충돌·도달성)이 사용하는 것은 씬 USD 와 충돌 점군 npz 이며
**둘은 한 몸이다.** 어긋나면 로봇이 셀 내부에 박힌 것으로 판정되어 모든 자세가 거부된다.
6단계 절차는 `sim_scene.md` §3.

---

## 4. 알고리즘 문서 지도

| 단계 | 요지 | 문서 |
|---|---|---|
| **calibration** | 스캐너가 본 것과 로봇이 아는 것을 같은 좌표계로 묶는 두 상수(`T_EC`·`T_B_F0`). 카메라 위치는 SLAM 이 아니라 **로봇 FK + `T_EC`** 가 알려준다 | `1_calibration.md`, 절차는 `calibration_runbook.md` |
| **preview** | 물체 형상을 모르는 상태에서 높이·반경·적정 작업거리를 재는 측량 단계. 이후 전 단계의 입력이 된다 | `2_preview.md` |
| **lookaround** | 물체를 높이 방향 밴드로 분할하여 밴드마다 자세를 옮기고 턴테이블을 360° 회전. 밴드 전체가 한 IScan 이므로 밴드 간 이동 중에도 SLAM 이 유지되어야 한다 | `3_lookaround.md` |
| **nbv** | 누적 점군의 구멍을 찾아 해당 지점만 겨냥한 부분 스윕(±45°, 최대 8회). 종료는 신규 점유 복셀 비율로 판정 | `4_nbv.md` |
| **flip** | 물체를 뒤집어 바닥면을 취득하고 병합. 각 IScan 은 자기 첫 프레임이 원점이므로 `T_pre` 로 master 좌표로 끌어온다 | `5_flip.md` |
| **collision** | 자세를 실행 **전에** 필터링. SDF 기반 단일 게이트, real/sim 공용 | `collision.md`, 실측은 `hw_layout.md` |
| **postprocess** | SDK General Pipeline 으로 최종 메시 생성. **OutliersRemoval 은 Fusion 앞** | `6_postprocess.md` |

실행 명령은 `sim_commands.md`(sim) · `7_real_commands.md`(real), 설치는 `install.md`,
로봇 수동 조작은 `robot_control.md` 다. 남은 과제는 `../todo.md`.

---

## 5. 캘리브 현황 (2026-09-21 기준)

| 상수 | 값 | 판정 |
|---|---|---|
| `T_EC_artec` | 20자세, solvePnP, 7×5_sq12 보드. t_err **8.443mm** / r_err **1.732°** | ⚠ 기준(≤4mm / ≤1.5°) 초과 — 실물 재개 시 재수행 |
| `T_B_F0` | 12점 원 피팅. rim 반경 118.71mm, 잔차 **0.193mm** | 기준(≤1mm) 충족 |

캘리브 스크립트가 `sensor_frames.yaml` 을 직접 갱신한다(구 `hand_eye_artec.yaml` 폐지).
