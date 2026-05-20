# MMS — Artec 파이프라인 세션 핸드오프 (2026-05-20)

> 이 문서는 다른 Claude Code 터미널이 이어받기 위한 컨텍스트 스냅샷이야.
> 영구 정보는 `C:\Users\user\.claude\projects\C--Users-user-workspace-MMS\memory\MEMORY.md`
> 와 `docs/` 에 있어. 이 문서는 **이번 세션에서 일어난 일과 다음 액션**.

---

## 0. 빠른 컨텍스트

- 프로젝트: **MMS** = 턴테이블 + xArm7 + Artec **Spider v1** 스캐너로 객체
  full-coverage water-tight mesh + texture 생성.
- 현재 단계: Phase 1 (canonical 자세 360° 회전) + Phase 2 (객체 자세 바꿔
  바닥면 캡처) 통합 multipass scan 디버깅. 결과 mesh 가 watertight 안 나옴
  (직교하는 흰 평면이 캔과 붙어있음). 원인 진단 중.
- 좌표 규약: `CLAUDE.md` 의 `T_AB : A→B`, `x_B = T_AB @ x_A`. **무조건 준수**.
- 환경: Windows 11, PowerShell, conda env `mms-env`. UTF-8 출력은
  `PYTHONIOENCODING=utf-8` 필요 (cp949 콘솔이 이모지 깨뜨림).

---

## 1. 이번 세션 변경 요약 (2026-05-20)

### A. Rule 재설계 — Phase 1 home 시작, 적응 자세는 recovery 에서만

**이전**: scan 시작 직전 `_adaptive_prescan_position` 호출 (probe + elevation
search, ~1분 overhead).

**현재**: scan 첫 시작 = robot=home 그대로. tracking lost 발생 시에만
`_attempt_recovery` 가 turntable safe-back + `_adaptive_prescan_position(
recovery=True)` 호출 (fresh probe + **축소 elevation** `[-5°, 0°, +5°]`,
fine skip).

- 옛 `RecoveryPoseSelector`/`LocalJitterSelector`/`CentroidVectorSelector` 전부
  **제거**. helper (`look_at_axes`, `calibrate_camera_axes_from_preview`,
  `master_points_in_base_frame`, `visible_point_count`) + SPIDER 상수만 보존.
- Settings 신설: `auto_recovery_enabled=True`, `recovery_elevation_offsets_deg
  =[-5,0,+5]`, `recovery_elevation_fine_search_enabled=False`.
- Settings 제거: `recovery_selector`, `adaptive_phase1_positioning`,
  `phase1_elevation_search`, `phase1_motion_probe`, `elevation_coarse_offsets_deg`.

### B. View-score (elevation candidate 평가) v1.5

물체/턴테이블 분리 정확도 향상:
- **턴테이블 평면 hard floor** — `z > z_table + table_clear_mm` (기본 8mm)
- **(r,z) 축대칭 occupancy 프로파일** — moving voxel 들의 실제 단면 (cylinder
  대체)
- **Gaussian 거리 가중** — d* = 225mm, σ_d = (band_hi - band_lo) / 2
- 후보별 디버그 PLY (통과 녹/턴테이블 빨강/profile 밖 회) → `output/iso_debug/`

상세: `docs/artec_phase1_view_score.md`.

### C. 병합 비교 인프라 (새로 만든 부분, 아직 실행 안 함)

같은 raw scan 데이터로 3 variant 비교 가능:
- **noHint** — hint 적용 0, `GlobalReg(GEOMETRY)`
- **textureBased** — hint 적용 0, `GlobalReg(GEOMETRY_AND_TEXTURE)` = SDK 텍스처 정합
- **hintRefine** — recorded_hints 적용 → `GlobalReg(GEOMETRY)` refine

기반:
- Settings 추가: `apply_hints_to_frame_transformations: bool = True`. False 면
  `_merge_into_master` 가 IScan frame 에 T_pre 안 박고 `result.recorded_hints`
  list 에 `(scan_idx, T_pre)` 기록만.
- 새 스크립트: `scripts/artec/merge_compare.py`. main_artec 의 CFG/
  MULTIPASS_SETTINGS 재활용해서 scan 1회 → 3 variant 후처리 → `output/
  merge_compare/{variant}_{TS}.{obj,sproj}` 저장.

---

## 2. 알려진 이슈 (진단 진행 중)

### 직교 mesh 문제

`output/artec_phase1_20260520_123738.obj` 에서 캔(ZERO SUGAR 250ml) 옆에 큰
흰 평면 mesh 가 캔과 **직교**해 붙은 양상. 사용자 스크린샷으로 확인됨.

로그 진단 결과:
- 3 pass 모두 정상 (tracking lost=False, frames=222/221/222).
- `hints_applied=True` → post-merge GlobalReg 자동 skip (지난 동작).
- `[hint pose 1] translation = (+9.2, -264.9, -258.6) mm` 가 보이며 hint 적용
  됐음.

**의심 순위**:
1. **사용자 손회전 각도 오차** — 사람이 90°/180° 정확히 못 돌림 (±10° 예사).
   `hints_applied=True` 라 GlobalReg refine skip → 오차 그대로 fusion.
2. **base Y 축 vs 사용자 직관 불일치** — `make_axis_physical_rotations("y", ...)`
   의 Y 가 xArm base Y. 사용자가 직관한 회전 축과 다를 수 있음.
3. **턴테이블 디스크 포함** — live viewer object_gate 는 표시 필터일 뿐.
   실제 IScan vertices 에 디스크 포함 가능. memory
   `project_artec_tracking_lost_limitation` 참고.
4. **R_phys 가 canonical(절대) 기준인데 사용자가 누적으로 돌림** — pose 2 의
   180° 가 canonical 기준이지만 사용자가 pose 1(90°) 에서 +90° 추가한 걸로
   착각 가능.

### 다음 액션 (사용자 합의됨)

**즉시 할 일**: `python scripts/artec/merge_compare.py` 실행. 한 번 scan
(3 pass) → 3 variant OBJ 생성 → CloudCompare/MeshLab/Artec Studio 로 시각
비교. textureBased 가 watertight 잘 만들면 SDK 텍스처 정합이 우리 hint 보다
강한 것 → hints_applied 폐기 후 SDK 에 맡기는 방향. hintRefine 이 noHint
보다 좋으면 hint 가 좋은 initial 이라는 검증 → 추후 hint refine 모드 채택.

---

## 3. 파일 맵 (이번 세션 신규/변경)

```
docs/
 ├─ artec_scanning_pipeline.md          ← Phase 1/2/Recovery 전체 흐름 (대폭 갱신)
 ├─ artec_phase1_view_score.md          ← NEW: elevation candidate score 알고리즘
 └─ artec_iscan_merge_explained.md      ← NEW: IScan 병합 (T_pre/hint) 비유 + 진단표

mms_artec/nbv/
 ├─ artec_multipass_scan_session.py     ← 대폭 변경
 │   - Settings: apply_hints_to_frame_transformations (신규)
 │              auto_recovery_enabled, recovery_elevation_offsets_deg (신규)
 │              recovery_selector, adaptive_phase1_positioning,
 │              phase1_elevation_search, phase1_motion_probe,
 │              elevation_coarse_offsets_deg (제거)
 │   - Result: recorded_hints (신규)
 │   - _attempt_recovery: selector 호출 제거, _adaptive_prescan_position(recovery=True) 호출
 │   - _adaptive_prescan_position(recovery: bool): recovery 모드 추가
 │   - _elevation_search(offsets_deg, fine_search_enabled): 파라미터화
 │   - _in_object_profile: (r,z) profile + table floor (cylinder envelope 대체)
 │   - _build_rz_profile: probe 단계 (r,z) 점유 맵 빌드 (신규)
 │   - _phase1_view_score: Gaussian 거리 가중 v1.5
 │   - _dump_candidate_ply: 후보별 통과/탈락 색칠 PLY (신규)
 │   - run(): _adaptive_prescan_position 호출 제거 (첫 시작 = home)
 │   - merge 블록: apply_hints=False 시 recorded_hints 에 기록만
 │
 └─ recovery_pose_selector.py           ← 대폭 축소
     - RecoveryPoseDecision/Protocol/LocalJitterSelector/CentroidVectorSelector 제거
     - helper (look_at, look_at_axes, calibrate_camera_axes_from_preview,
       master_points_in_base_frame, visible_point_count) + SPIDER 상수 보존

main_artec.py                            ← 정리
 - LocalJitterSelector/CentroidVectorSelector import 제거
 - RECOVERY_STRATEGY 토글 + RECOVERY_SELECTOR 변수 제거
 - MULTIPASS_SETTINGS: auto_recovery_enabled, recovery_elevation_* 노출

scripts/artec/
 └─ merge_compare.py                    ← NEW: 3-variant 병합 비교
```

---

## 4. Memory 포인터 (영구)

`MEMORY.md` 인덱스의 핵심 항목:
- `feedback_no_crop_in_phase1` — Phase 1 에 crop 필터 다시 넣지 말 것
- `feedback_artec_pipeline_order` — Outliers/SmallObjects 는 Fusion 전 (SDK §3)
- `feedback_live_viewer_must_mirror_scan` — viewer cleanup 은 IScan source 에서, viewer 변경은 annotation 까지만
- `project_artec_spider_v1` — Spider v1, 170–350mm (optimal 200–250mm), FOV 30°×21°
- `project_artec_tracking_lost_limitation` — tracking_lost flag ≠ object presence
- `project_spider_partial_view_no_scene_segmentation` — 단일 preview 로 장면분할 불가
- `project_artec_camera_optical_axis_convention` — `look_at` 의 +Z 가정 금지, `look_at_axes` 사용
- `project_open3d_must_use_filament_o3dvisualizer` — legacy Visualizer 는 동적 PointCloud 못 그림
- `project_turntable_frame_yaml_stale_for_artec` — T_BF0 stale, 신뢰 안 함

---

## 5. 실행/검증 명령

```powershell
# 메인 파이프라인
PYTHONIOENCODING=utf-8 python main_artec.py

# 병합 비교 — scan + 자동 raw 저장 + 4 variant 후처리 (기본)
PYTHONIOENCODING=utf-8 python scripts/artec/merge_compare.py

# 병합 비교 — 저장된 raw 로 4 variant 만 (scan 스킵, 하드웨어 불필요)
PYTHONIOENCODING=utf-8 python scripts/artec/merge_compare.py --load output/scan_raw/<TS>

# raw scan 저장만 (variants 스킵)
PYTHONIOENCODING=utf-8 python scripts/artec/save_raw_scan.py
# 또는: python scripts/artec/merge_compare.py --save-only

# 라이브 뷰어 (다른 터미널)
python scripts/artec/live_scan_view.py

# import smoke test
PYTHONIOENCODING=utf-8 python -c "import main_artec; print('OK')"
```

cp949 issue 회피 위해 항상 `PYTHONIOENCODING=utf-8`.

**raw 저장 형식** (`output/scan_raw/<TS>/`):
- `master.sproj` — IScan 들의 raw frame_transformations
- `meta.npz` — `scan_indices`, `T_pres`, `T_BC`, `T_CB`, `n_scans`

병합 알고리즘 실험을 반복할 때 매번 scan 안 돌리고 `--load` 로 같은 raw
데이터에 다양한 후처리 시도. ICP 파라미터 튜닝·새 variant 추가 등에 유용.

---

## 6. 사용자와의 상호작용 톤

- 한국어로 답해. 짧고 구조적으로 (헤더, 표, 코드 블록).
- 사용자는 시각·실용 중심 — 길고 추상적인 설명보다 "정확히 이 줄 보면 됨"
  / "이 명령 돌리면 됨" 식.
- 사용자가 "어려워" / "이해 못하겠어" 라고 하면 비유 + 도식으로 다시 설명.
  (예: IScan 병합을 "사진 모자이크" 로 비유 — `docs/artec_iscan_merge_explained.md`)
- 큰 설계 변경은 옵션 2~3개로 정리해 `AskUserQuestion` 으로 결정 받고 진행.
- task 도구(`TaskCreate`/`TaskUpdate`) 로 multi-step 작업 진행 가시화.
- 영구 정보(사용자 선호, 프로젝트 결정) 는 memory 에 저장. 일시적 진행 상태는 task 로.

---

## 7. 다음 세션이 바로 할 액션

**4-variant 비교 ready** (1차 3-variant 비교 결과: noHint=textureBased 둘 다
앞-뒤 합쳐서 5면, hintRefine 직교 mesh). 캔 같은 회전대칭 객체의 face-
merging 해결 시도 — Open3D **colored ICP** 로 사용자 손회전을 측정.

1. **`python scripts/artec/merge_compare.py` 실행** — 1회 scan (3 pose) →
   같은 raw 데이터로 **4 variant** 결과:
   - `noHint`, `textureBased`, `hintRefine` (1차에서 봤던 것)
   - `hintIcpRefine` (신규): hint 를 init 으로 Open3D colored ICP →
     측정된 T_pre 박음. progressive master (1+...+i-1 누적) 를 ICP target
     으로 사용. 사용자 손회전 ±10° 오차 흡수 + ambiguity init 으로 깸.

2. **결과 비교**: `output/merge_compare/hintIcpRefine_*.obj` 가 다른 셋과
   다른 모양이면 ICP 접근이 face-merging 깬 것. ICP 로그의 `fitness`,
   `inlier_rmse`, `Δtrans` 가 분기 진단:
   - `fitness > 0.5` & `Δtrans 10~30mm` → 사용자 hint 가 약간 빗나갔고 ICP 가 잘 보정
   - `fitness < 0.2` → master 와 새 IScan overlap 부족, ICP 부적합
   - `Δtrans < 2mm` → hint 가 거의 정확했고 refine 효과 미미

3. **분기**:
   - hintIcpRefine 성공 → main_artec.py 에 `hint_icp_refine_mode=True` 적용
   - 실패 → 옵션 A (비대칭 마커) 또는 옵션 D (Artec Studio manual align) 로
   - 모든 variant 가 5면 합쳐 그대로 → 객체 자체 한계 (캔 회전대칭이 너무
     강함) → 비대칭 객체로 검증 먼저

---

## 8. 절대 하지 말 것

- T_AB 외 다른 표기법 (`T_A^B`, `^A T_B`) 사용 금지.
- legacy `look_at` (+Z 가정) 신규 코드에 사용 금지. `look_at_axes` 만.
- Phase 1 에 crop 필터 (cylindrical/Z) 다시 도입 금지.
- 사용자 명시 요청 없이 destructive git 명령 (`reset --hard`, `push --force`)
  실행 금지.
- 사용자 명시 요청 없이 새 `.md` 파일 만들지 말 것. 단 사용자가 "md 로
  정리해줘" 라고 한 건 명시 요청이라 OK.
- main 브랜치에 직접 commit 금지. 사용자가 별도로 PR 흐름 안내함.

---

이 문서를 읽었다는 신호로 다음 세션은 첫 응답에서 "ONBOARDING 읽음, 다음
액션은 merge_compare 실행 결과 받기" 정도로 컨펌해줘.
