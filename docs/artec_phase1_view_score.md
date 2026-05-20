# Phase 1 View Score — Elevation Candidate 평가 알고리즘

> Phase 1 사전 포지셔닝(`_adaptive_prescan_position` → `_elevation_search`)
> 에서 **어느 고도각 φ 가 가장 좋은가** 를 정량화하는 함수의 동작 설명.
> 코드: `mms_artec/nbv/artec_multipass_scan_session.py`
>   - `_phase1_view_score` (메인 점수 함수)
>   - `_in_object_profile` (물체 멤버십 판정)
>   - `_build_rz_profile` (probe 단계, 점유 맵 빌드)
> 좌표 규약: `CLAUDE.md` (`T_AB : A→B`, `x_B = T_AB @ x_A`).

---

## 0. 무엇을 결정하는 함수인가

`_elevation_search` 는 물체 둘레 zx평면 호를 따라 여러 고도각 후보
`φ` 를 평가해 best 1개를 골라 robot 을 그 자세로 이동시킨다. 그 다음
turntable 이 360° 도는 동안 robot 은 고정이라, 이 한 번의 선택이 다음
회전의 시야를 결정한다.

> **호출 컨텍스트** (2026-05-20 rule 변경): 이 함수는 **Phase 1/2 의
> 첫 시작에서는 호출되지 않는다**. scan 은 사용자가 맞춰놓은 home 자세
> 에서 그대로 출발하고, **tracking lost 가 발생한 recovery 흐름**
> (`docs/artec_scanning_pipeline.md` §6) 에서만 호출된다. recovery 시간
> 제약을 위해 candidate 수가 축소됨 (`recovery_elevation_offsets_deg`
> 기본 `[-5°, 0°, +5°]`, fine search skip).

따라서 score 는 "그 자세로 360° 돌렸을 때 정합 품질이 얼마나 좋겠는가" 의
**프록시 1개로 1개 자세를 비교**할 수 있는 스칼라여야 한다.

현재 정의:

> **score(φ)** = "그 자세에서 캡처한 preview 중 **물체로 분류된** 점들이
> **최적 작업거리(~225mm) 근처** 에 얼마나 많이 모여있느냐"

낮은 score = 물체 살짝 비껴봄/너무 멀거나 가깝게 봄/턴테이블만 잘 보임.

---

## 1. 입력 / 출력

### 입력
| 이름 | 타입 | 의미 |
|---|---|---|
| `verts_C_mm` | (N, 3) float | candidate 자세에서 찍은 preview 점군. **C 프레임, mm**. |
| `T_CB_cand` | (4, 4) float | 그 candidate 의 C → B 변환. preview 캡처 직후의 robot pose 로부터 도출. |
| `fwd_C`, `up_C` | (3,) float | probe 물체점으로 경험적 캘리브된 광축 / 상방 (C 프레임). hardcoded `+Z_C` 가정 안 씀 — `calibrate_camera_axes_from_preview` 결과. |
| `env` | dict | probe 결과 envelope. 핵심 키: `axis_xy`, `r_obj`, `z_lo/z_hi`, `z_table`, `rz_occ`(2D bool), `bin_mm`, `r_origin_m`, `z_origin_m`. |

### 출력
`(score: float, n_obj: int, n_excl_table: int)`

- `score` — argmax 비교에 사용되는 스칼라. (개수 가중합)
- `n_obj` — 멤버십 통과 물체점 수 (진단).
- `n_excl_table` — cylinder 안인데 turntable floor 에 잘린 점 수 (진단).

---

## 2. 단계별 알고리즘

```
verts_C_mm  ──①── xB (m, B 프레임)
                 │
                 ├──② _in_object_profile ─────→ mask_obj  (객체점)
                 │     ├ cylinder pre-clip               mask_excl_table (턴테이블 floor 에 잘림)
                 │     ├ z > z_table + 8mm  (hard floor)
                 │     └ (r,z) profile lookup
                 │
verts_C_mm[mask_obj]
                 │
                 ├──③ 광축 정규직교 기저 (e1=fwd, e2=up⟂, e3=right)
                 │
                 ├──④ depth / lateral / vertical 분해
                 │
                 ├──⑤ FOV 안 판정 (Spider 30°×21°)
                 │
                 ├──⑥ Gaussian 거리 가중치 w(depth)
                 │
                 └──⑦ score = Σ w · 1_FOV
```

### ① C → B 변환

candidate 자세 T_CB_cand 로 preview 점을 base 프레임으로 옮긴다.

```
xB = (verts_C_mm / 1000) @ R_CB.T + t_CB        # m, B 프레임
```

물체 멤버십 판정이 B 프레임에서 정의돼있기 때문 (envelope·z_table 모두 B).

### ② 물체 멤버십 판정 — `_in_object_profile(xB, env)`

세 단계 AND. cylinder 는 빠른 pre-clip, hard floor 는 turntable 차단,
(r,z) profile lookup 이 실제 단면 검증.

#### (i) cylinder pre-clip (느슨하게)

```
r       = ‖xB[:, :2] − axis_xy‖        # 축까지 수평거리
in_cyl  = (r ≤ r_obj + 8mm)
        ∧ (z_lo − 6mm ≤ z ≤ z_hi + 6mm)
```
- envelope cylinder + 안전 여유. 빠르고 보수적.
- **단독 사용 금지** — z_lo ≈ z_table 이라 6mm 여유가 디스크 표면을
  cylinder 안으로 끌어들임 (회귀 원인, §3 한계 참고).

#### (ii) turntable hard floor (필수)

```
z_floor          = z_table + table_clear_mm    # 기본 8mm
above            = z > z_floor
mask_excl_table  = in_cyl ∧ ¬above             # 진단용
```
- z 가 turntable 평면 위 8mm 이하인 모든 점을 **무조건 제외**.
- 평평한 물체 바닥 일부가 잘리는 비용은 수용 (Phase 2 가 바닥 면 별도 캡처).

#### (iii) (r,z) profile lookup (정밀)

probe 단계에서 만든 축대칭 occupancy 맵으로 실제 단면 검증.

```
bin_m   = env['bin_mm'] / 1000
ri      = floor((r - r_origin) / bin_m)
zi      = floor((z - z_origin) / bin_m)
in_b    = (0 ≤ ri < nr) ∧ (0 ≤ zi < nz)
mask_obj[p] = in_cyl[p] ∧ above[p] ∧ in_b[p] ∧ rz_occ[ri[p], zi[p]]
```
- `rz_occ` 는 probe 의 moving voxel 들을 `(r, z)` 평면으로 reduce 한 2D bool.
  축 둘레 360° **회전대칭화** 되어있어 어느 θ 의 candidate 에서나 멤버십 성립.
- profile 빌드 시 `profile_r_margin_mm`/`profile_z_margin_mm` (기본 6mm) 만큼
  binary dilation → 측정 잡음/여유 흡수.
- profile 없으면 cylinder + floor 만으로 fallback.

### ③ 광축 정규직교 기저

캘리브된 `fwd_C`, `up_C` 로 정규직교 기저:

```
e1 = fwd_C / ‖fwd_C‖                       # 카메라 깊이 축
e2 = up_C − (up_C · e1) e1
e2 /= ‖e2‖                                 # 상 (e1 ⟂ 성분)
e3 = e1 × e2                               # 우
```

`+Z_C` hardcoded 가정의 legacy `look_at` 은 미사용 — mis-aim bug 회피.

### ④ depth / lateral / vertical 분해

객체점 `v ∈ verts_C_mm[mask_obj]` 의 카메라 좌표:

```
depth = v · e1     # 카메라 축 방향 거리 (전방 +)
lat   = v · e3     # 좌우 변위
vert  = v · e2     # 상하 변위
```

### ⑤ FOV 안 판정

Spider v1 FOV 30°(H) × 21°(V) → half-angle (15°, 10.5°).

```
in_fov = (depth > 1mm)
       ∧ (|lat|  ≤ depth · tan(15°))
       ∧ (|vert| ≤ depth · tan(10.5°))
```

depth>1mm 은 카메라 등 뒤 점(잘못된 회전) 거름. 광축 비대칭이라
`H/V` 따로 처리.

### ⑥ Gaussian 거리 가중치

작업거리 밴드 `[band_lo, band_hi] = [200, 250]mm` 기반:

```
d*    = adaptive_target_standoff_mm        # 225mm
σ_d   = (band_hi − band_lo) / 2            # 25mm
w(d)  = exp(−((d − d*) / σ_d)²)
```

#### 가중치 곡선

| depth d (mm) | w(d)   | 의미                       |
|--------------|--------|----------------------------|
| 225          | 1.00   | 최적 거리, 가장 무거움     |
| 200, 250     | 0.37   | band 끝 = 1σ = e⁻¹         |
| 175, 275     | 0.018  | 2σ, 거의 무시               |
| 170, 280     | ~0.01  | Spider near 한계 근처       |
| 350          | ≈ 0    | Spider far 한계, 사실상 0   |

밴드 안에서도 225mm 에 가까운 점이 더 무겁다. 같은 객체점 개수라도
"d* 에 모인 자세" 가 이긴다.

### ⑦ 합산

```
score = Σ_p ∈ mask_obj  w(depth_p) · 1_in_fov(p)
```

float 스칼라. `_elevation_search` 의 `argmax_φ score(φ)` 비교 대상.

---

## 3. best 선택

```python
best = (sc, tag, T_cand)   # _eval 안에서 score 가 갱신될 때마다 교체
if best is None or sc > best[0]:
    best = (sc, tag, T_cand)
```

- coarse phase: home_dist + 호출자가 넘긴 `offsets_deg` (recovery 시 기본
  `[-5, 0, +5]` = `recovery_elevation_offsets_deg`)
- fine phase: `fine_search_enabled=True` 일 때만 coarse best φ* 주변
  `φ* ± elevation_fine_step_deg` 추가 (recovery 기본 False)
- 최종 best 가 home_dist 면 robot 이동 0 (비퇴행 baseline).
- 유효 candidate 없거나 best score ≤ 0 → home 복귀.

---

## 4. 진단 로그 해석

각 candidate 마다 한 줄:

```
[elev] c-5.0: score=23.4 (obj 412, table_excl 17)
       └tag  └float    └객체점수 └floor 차단수
```

해석 가이드:
- `obj` 큰데 `score` 작음 → 객체점이 많지만 d* 에서 멀다 (band 밖).
- `table_excl` 큼 → cylinder 안에 turntable 표면이 많이 들어왔다는 뜻.
  자세가 너무 내려다보고 있다는 신호. profile 이 fail-safe 로 잘랐음.
- `obj`=0, `score`=0 → 멤버십 통과한 점 없음. 자세가 envelope 밖.

PLY 디버그 (`probe_debug_dump=True`):
- `output/iso_debug/cand_<seq>_<tag>.ply`
- 색: **녹**=mask_obj, **빨강**=mask_excl_table, **회**=나머지
- CloudCompare 로 열어서 후보별 시야 검증.

---

## 5. 한계 — 단일 θ 평가

**현재 score 는 한 시점(probe 복귀 직후 turntable θ=0) preview 만 본다.**
elevation φ 만 바꿔서 비교하지만 θ 는 고정.

```
score(φ) = score(φ, θ=0)             # 실제 평가
의도       score(φ) = avg_θ score(φ, θ)   # 회전 평균이 더 맞음
또는       score(φ) = min_θ score(φ, θ)   # worst-case
```

비대칭 물체 (손잡이/주둥이) 는 θ=0 에서 보이는 단면이 다른 θ 의 단면과
크게 달라 best φ 가 θ 마다 달라질 수 있다. 360° 한 바퀴 동안 robot 은
고정이라 한 φ 가 모든 θ 에서 좋아야 안전.

해결 방향 (미구현, 별도 결정 대기):
1. probe 가 이미 본 step별 객체점을 turntable axis 로 회전 보정 →
   객체 frame O 의 통합 표면 모델 → candidate 평가 시 가상 θ N개 합산.
2. candidate 마다 실제 turntable 짧게 돌리며 N preview 평균.
3. (1)+(2) 결합 — coarse 는 (1), top-K 만 (2).
4. single best 폐기, top-K elevation 각각 별도 pass 로 multi-scan.

회전 대칭에 가까운 물체는 (1) 만으로도 충분, 강한 비대칭이면 (2) 또는 (4)
필요. trade-off 는 비용(시간) vs 정확도(자세 일반성).

---

## 6. 관련 설정 키 (`ArtecMultiPassScanSessionSettings`)

| 키 | 기본 | 의미 |
|---|---|---|
| `adaptive_target_standoff_mm` | 225.0 | Gaussian 중심 d* |
| `elevation_optimal_band_mm` | (200, 250) | σ_d 산출 (band/2) |
| `env_r_margin_mm` | 8.0 | cylinder 반경 여유 |
| `env_z_margin_mm` | 6.0 | cylinder z 여유 |
| `table_clear_mm` | 8.0 | turntable hard floor 높이 |
| `profile_bin_mm` | 0.0 (= probe_vox_mm) | (r,z) 점유 bin |
| `profile_r_margin_mm` | 6.0 | (r,z) profile dilation r |
| `profile_z_margin_mm` | 6.0 | (r,z) profile dilation z |
| `probe_debug_dump` | False | candidate PLY 덤프 ON/OFF |
| `iso_debug_dir` | `output/iso_debug` | 덤프 위치 |

---

## 7. 관련 파일

- `mms_artec/nbv/artec_multipass_scan_session.py`
  - `_phase1_view_score` — 이 문서의 메인 알고리즘
  - `_in_object_profile` — ② 멤버십
  - `_build_rz_profile` — probe 단계 (r,z) 맵 빌드
  - `_dump_candidate_ply` — PLY 덤프
  - `_elevation_search` — argmax 호출자
- `mms_artec/nbv/recovery_pose_selector.py`
  - `calibrate_camera_axes_from_preview` — fwd_C / up_C 캘리브
  - `look_at_axes` — candidate pose 생성
  - `SPIDER_HALF_FOV_H/V`, `SPIDER_NEAR_MM/FAR_MM` — Spider 광학 상수
- `docs/artec_scanning_pipeline.md` §3.0 — 상위 파이프라인 맥락
