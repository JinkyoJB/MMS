# Phase 1 — Rule-based 360° 턴테이블 스캔 구현 기록

`docs/2_control_layers.md` 설계의 **Phase 1 (first-pass mesh 생성)** 을 실구현한 내용
정리. Phase 2 (frontier NBV) 는 아직 off.

> 초점: PhoXi + xArm7 + Ezi-SERVO 턴테이블 환경에서 **T_CO 만 믿고** 360° 회전으로
> 초기 mesh 를 만드는 파이프라인. 중간 시행착오(TSDF/InstantMeshing 실패, 턴테이블
> 동기화 이슈, 회색 mesh 문제 등) 과정과 최종 동작 상태를 기록.

---

## 1. 한 줄 요약

```
θ=0° reset → scan_init joints 이동
  → 24 프레임 ( θ=0, 15, 30, … 345° )
      각 스텝: 턴테이블 이동 → settling → PhoXi capture
              → T_CO = T_FO · T_BF(θ) · T_EB · T_CE  (docs/1 체인룰)
              → (i≥1) ICP refinement vs 누적 pcd
              → volume.integrate (pcd 누적)
  → 최종: 턴테이블 평면 제거 + SOR denoise + Poisson → mesh.ply
```

---

## 2. 실행 흐름 (`main.py`)

```
CFG (PhoXi SEA-023, turntable_frame.yaml, hand_eye_phoxi.yaml)
SCAN_SETTINGS (ScanSessionSettings)
  ├─ mesh_backend = "pcd_accumulate"
  ├─ phase1_enabled = True,  phase2_enabled = False
  ├─ phase1_theta_step_deg = 15.0            → 24 프레임
  ├─ phase1_dwell_s        = 0.40            → settling 여유
  ├─ phase1_icp_refine     = True            → i≥1 ICP
  ├─ phase1_poisson_backend = "both"         → Open3D + PhotoneoRecon
  └─ phase1_export_pcd_path / mesh_path      → output/*.ply

main()
  ├─ go_to_scan_init(SCAN_INIT_JOINTS_DEG)   ← 이전 home+rpy 2단계 → 1단계로 통합
  ├─ _run_scan_session(mms, robot, turntable)
  │    └─ session.run()  ← ScanSession
  └─ finally: _show_session_mesh(session)   (디버그 뷰, Q 로 종료)
```

`SCAN_INIT_JOINTS_DEG = [-0.5, -43.5, 0.2, 46.2, -0.2, 69.2, 1.0]` — PhoXi 광축이
턴테이블 중심을 내려다보도록 튜닝된 joint 구성.

---

## 3. 레이어 구조

```
┌─────────────────────────────────────────────────────────────┐
│ ScanSession._rule_based_scan()          (mms/nbv/scan_session.py)
│   ├─ θ=0 reset                                              │
│   ├─ for i in 0..23:                                        │
│   │    turntable.move_abs(θ_i) + wait_motion_done()         │
│   │    dwell (0.40s)                                        │
│   │    capture → Frame                                      │
│   │    T_CO_nominal = mms.T_CO(θ_act, T_EB)                 │
│   │    if i≥1: ICP refine → T_CO                            │
│   │    volume.integrate_frame(frame, T_CO)                  │
│   │    vis.update_pcd(merged_pcd_cropped)                   │
│   ├─ save_pcd → output/phase1_merged.ply                    │
│   └─ Poisson (open3d + photoneo_exe) → phase1_mesh_*.ply    │
└─────────────────────────────────────────────────────────────┘
               ↑                                 ↑
        PcdAccumulateVolume                ProgressVisualizer
        (mms/nbv/pcd_accumulate_volume.py) (mms/nbv/_progress_vis.py)
        - integrate_frame(C → O)           - non-blocking 뷰어
        - merged_pcd_cropped                 (update_pcd / update_mesh)
        - _remove_turntable_plane
        - _denoise_sor (SOR)
        - extract_cropped_mesh (Poisson)
```

---

## 4. 좌표 변환 (docs/1 체인룰 재사용)

```
T_CO(θ, T_EB) = T_FO · T_BF(θ) · T_EB · T_CE      (C → O)

  T_FO   = I           (T_OF = I 가정, O ≡ F at θ=0)
  T_BF(θ) = inv(T_FB(θ)) = inv(T_FB0 · Rz(θ))
  T_EB   = 로봇 FK 결과 (meters)
  T_CE   = inv(T_EC)   (hand-eye 결과)
```

`PcdAccumulateVolume.integrate_frame`:
```python
# sensor organized (H,W,3), mm, C 프레임
pts_C_mm = self.sensor._last_organized_pts.reshape(-1, 3)
# invalid(z=0) + depth 범위 필터
# mm → m
pts_C_m = pts_C_mm[valid] / 1000.0
# C → O
pts_O = pts_C_m @ T_CO[:3,:3].T + T_CO[:3, 3]
self._pts_O.append(pts_O)
```

---

## 5. Mesh backend 선택 이유

| Backend | 상태 | 이유 |
|---------|------|------|
| Open3D `ScalableTSDFVolume` | 테스트됨, **부적합** | RGBD+intrinsic 규약 맞추기 복잡, 초기 실험에서 턴테이블 표면만 잡히고 물체 소실 |
| Photoneo `PhoXiInstantMeshing` | 테스트됨, **사용 불가** | 마커보드 기반 tracking 가정 — 외부 FK pose 가 정상으로 들어가도 "Scene is empty" 반환. `AddScan transformation` 을 매 프레임 다른 값으로 넣어도 개선 안 됨. |
| **`PcdAccumulateVolume` (자체)** | **선택** | pose 검증 용이. C→O 변환만 맞으면 mesh 나옴. Poisson 은 외부 라이브러리에 위임 (Open3D / PhotoneoRecon). |

---

## 6. 핵심 버그 & 해결 기록

### 6.1 턴테이블이 처음 2~3번만 움직이고 멈춤 → "부채꼴" mesh

**원인**: `wait_motion_done()` 이 `move_abs(θ)` 직후 드라이버 반영 전 첫
`GetAxisStatus` 에서 **이전 target 의 `INPOSITION=1`** 을 보고 즉시 리턴.

**해결** (`mms/turntable/turntable_interface.py`):
- `wait_motion_done` 을 **2단계** 로 변경:
  - **Stage 1**: 모션 시작 (`MOTIONING=1` 또는 `INPOSITION=0`) 확인 (`start_timeout_s=1.0`)
  - **Stage 2**: 정지 (`MOTIONING=0 AND INPOSITION=1`) 가 **연속 N회** (`stable_reads=5, stable_poll_s=0.010`) 관측돼야 통과
- `move_abs` 가 `FAS_MoveSingleAxisAbsPosEx` 반환값 체크 + alarm auto-reset 추가.
- ScanSession 에서 `|θ_act − θ_tgt| > 2°` 시 경고.

### 6.2 Phase 1 pose 누적 오차 → incremental ICP refinement

Encoder + FK + hand-eye 만 믿으면 mm 단위 오차가 누적. **2번째 프레임부터 ICP** 로
refine:

```python
for i in 0..23:
    T_CO_nominal = mms.T_CO(θ_act, T_EB)
    T_CO = T_CO_nominal

    if i >= 1:
        source = _build_current_frame_pcd_C()           # 이번 프레임, C 프레임
        target = volume.merged_pcd_cropped(...)         # 누적 (0..i-1), O 프레임
        source = voxel_down + SOR denoise
        target = SOR denoise
        icp_res = icp_with_gates(
            source, target, init_T=T_CO_nominal,
            max_corr=5mm, rmse<1mm, fit>0.3, drift<20mm/10°,
        )
        if icp_res.ok: T_CO = icp_res.T_refined
        else:          T_CO = T_CO_nominal  # fallback

    volume.integrate_frame(frame, T_CO)
```

ICP 는 docs/1 `icp_strategy.py::icp_with_gates` 재사용 (point-to-plane + triple gate).

### 6.3 회색 mesh + 노이즈 / 턴테이블 평면 혼입

**대응 3종** (`PcdAccumulateVolume`):

1. **Intensity auto-stretch** — `_last_intensity` 가 Mono12 원본에서 raw 값 범위가 좁아
   RGB 가 `[0.02, 0.44]` 로 어두움. **2–98 percentile 로 [0,1] 확장**:
   ```python
   lo = np.percentile(vi, 2); hi = np.percentile(vi, 98)
   g  = clip((vi − lo) / (hi − lo), 0, 1)
   ```
2. **RANSAC 턴테이블 평면 제거** (`_remove_turntable_plane`):
   - `z_O < plane_z_search_max (=0.03 m)` 점들만 후보로
   - `segment_plane(distance=3 mm, iters=1000)`
   - 전체 pcd 에서 평면 근방 3 mm 점 제거
   - 물체 상면은 z 가 높아 후보 밖이므로 안전.
3. **Statistical outlier removal** (`_denoise_sor`): `nb=30, std=2.0`.

Vertex color 유실 버그도 수정 — 이전엔 한 프레임의 intensity 가 None 이면 전체
merged pcd 에서 colors 가 사라졌음. 이제 gray(0.5) fallback 으로 항상 유지.

### 6.4 출력 PLY 가 외부 뷰어에서 회색 (Windows 3D 뷰어, Artec Studio)

PLY 에 **vertex color 는 정상 저장** 됨 (헤더 `property uchar red/green/blue` 확인).
문제는 **viewer 측 렌더링 기본값**:

| Viewer | PLY vertex color |
|--------|------------------|
| Open3D (우리 GUI) | ✅ 자동 |
| CloudCompare | ✅ Properties → Colors → RGB |
| MeshLab | ✅ Render Mode → Per Vertex Color |
| Windows 3D 뷰어 | ✗ PLY vertex color 렌더링 제한적 |
| Artec Studio | ✗ UV texture 만 기대, vertex color 비권장 |

**제공 방법**:
- Binary PLY (`phase1_mesh_open3d.ply`) + ASCII PLY (`*_ascii.ply`) 둘 다 저장.
- GLB export 는 Open3D 의 glTF writer 가 깨진 파일을 만들어 skip (trimesh 설치 시
  추가 가능).
- 진짜 UV texture 는 **texture baking (UV unwrap + image projection)** 이 필요 —
  현재 out of scope.

---

## 7. 현재 파라미터 (`main.py::SCAN_SETTINGS`)

### 공통
```
distance_m          = 0.414 m
CAMERA_ROLL_DEG     = 90 (Phase 2 용)
ROBOT_SPEED_DEG_S   = 15 (완속)
TURNTABLE_VEL_RAD_S = radians(10)  (10°/s, 완속)
```

### Crop / TSDF
```
crop_z_min          = −0.05 m       (디버깅용 느슨; 안정되면 +0.005)
crop_R              = 0.20 m        (디버깅용 느슨; 안정되면 0.12)
tsdf_voxel_length   = 0.002 m       (2 mm voxel)
```

### Phase 1
```
phase1_theta_step_deg     = 15.0    → 24 frames
phase1_dwell_s            = 0.40 s  (settling 여유)
phase1_show_progress      = True
phase1_wait_window_close  = True

phase1_icp_refine         = True
phase1_icp_sor_nb         = 20
phase1_icp_sor_std        = 2.0

phase1_poisson_backend    = "both"  (open3d + photoneo_exe)
phase1_export_pcd_path    = output/phase1_merged.ply
phase1_export_mesh_path   = output/phase1_mesh.ply
```

### ICP gate
```
icp_max_correspondence_m  = 0.005 m
icp_rmse_thresh_m         = 0.001 m
icp_fitness_thresh        = 0.3
icp_drift_trans_m         = 0.020 m
icp_drift_rot_deg         = 10.0
```

### PcdAccumulateVolume 내부 (기본값)
```
poisson_depth              = 9
poisson_density_quantile   = 0.02     (하위 2% density vertex 제거)
orient_normals_k           = 15
intensity_auto_stretch     = True
intensity_percentile       = (2.0, 98.0)
sor_nb_neighbors           = 30
sor_std_ratio              = 2.0
remove_turntable_plane     = True
plane_z_search_max         = 0.030 m  (z<3cm 영역에서 plane fit)
plane_distance_thresh      = 0.003 m  (3mm 이내 제거)
plane_ransac_iters         = 1000
```

---

## 8. 출력 파일 (`output/`)

| 파일 | 타입 | vertex color |
|------|------|--------------|
| `phase1_merged.ply` | PointCloud (binary) | ✅ (auto-stretched grayscale) |
| `phase1_mesh_open3d.ply` | TriangleMesh (binary) | ✅ |
| `phase1_mesh_open3d_ascii.ply` | TriangleMesh (ASCII) | ✅ |
| `phase1_mesh_photoneo.ply` | TriangleMesh (binary) | ✗ (PoissonRecon.exe 가 geometry-only) |

권장 viewer: **CloudCompare** + RGB 표시 모드.

---

## 9. 기록된 시행착오 (짧게)

| 증상 | 원인 | 해결 |
|------|------|------|
| Open3D TSDF 로 턴테이블 표면만 mesh | extrinsic/intrinsic 처리 차이 | Pcd 누적 방식으로 backend 교체 |
| Photoneo InstantMeshing "Scene is empty" | 마커보드 tracking 전제 — 외부 FK 로 unverified | 포기, pcd_accumulate 채택 |
| 부채꼴(fan) mesh | `wait_motion_done` race → 턴테이블 stall | 2-stage wait + stability 5-poll |
| 회색 mesh | intensity Mono12 raw 값이 작아 RGB 어두움 | percentile auto-stretch |
| 턴테이블 면이 mesh 로 올라옴 | crop z_min 만으론 캘리브 오차 통과 | RANSAC plane segmentation (z<3cm 후보) |
| 일부 프레임 intensity 없으면 색 전체 소실 | merged_pcd 가 "all frames have color" 조건 | 누락 프레임 gray(0.5) fallback |
| PhoXi lock 후 다음 실행이 Helios 로 붙음 | `serial_number=None` 이면 첫 장치 선택 | `serial_number="SEA-023"` 고정 |

---

## 10. 다음 단계 (Phase 2)

아직 off 상태 (`phase2_enabled=False`). 활성화 시:
- `mms/nbv/frontier.py` — boundary edge + connected segment 추출
- `mms/nbv/icp_strategy.py::pick_icp_roll` — 전략 (C)
- `mms/nbv/scan_session.py::_step` — NBV cost 기반 frontier 후보 선택 +
  `plan_and_execute` (docs/1 하위 레이어) + ICP refinement + 추가 프레임 통합
- 종료 조건: K_max(=20) / plateau / boundary 길이

Phase 1 기준 mesh 품질이 충분히 안정돼야 Phase 2 의 frontier 추출이 의미 있으므로,
Phase 1 파라미터 튜닝 (voxel / Poisson depth / SOR / plane 제거) 이 선결.

---

## 11. 관련 파일

| 파일 | 역할 |
|------|------|
| `main.py` | 엔트리, `SCAN_SETTINGS`, `go_to_scan_init`, `_run_scan_session`, `_show_session_mesh` |
| `mms/nbv/scan_session.py` | `ScanSession`, `_rule_based_scan`, `_build_current_frame_pcd_C`, `_read_theta` |
| `mms/nbv/pcd_accumulate_volume.py` | **주 backend** — 누적/plane제거/SOR/Poisson/저장 |
| `mms/nbv/_progress_vis.py` | non-blocking Open3D 뷰어 (`update_pcd` / `update_mesh`) |
| `mms/nbv/icp_strategy.py` | `icp_with_gates` (point-to-plane + triple gate) |
| `mms/nbv/tsdf_volume.py` | Open3D TSDF 백엔드 (미사용, 유지) |
| `mms/nbv/instant_meshing_volume.py` | Photoneo InstantMeshing 백엔드 (미사용, 유지) |
| `mms/turntable/turntable_interface.py` | `move_abs` (반환값 체크), `wait_motion_done` (2-stage) |
| `mms/sensor/phoxi/phoxi_client.py` | `get_intrinsic` (organized Range → pinhole fit), `_last_organized_pts/_intensity/_depth_mm` |
| `mms/sensor/phoxi/phoxi_meshing.py` | `reconstruct_photoneo_exe` (PoissonRecon.exe 래퍼) |

---

## 12. 메모

- Phase 1 pose 를 encoder+FK+hand-eye 만 믿는 것으로 시작해도 ICP refinement
  덕분에 누적 pcd 는 수 mm 이내 정렬.
- Poisson 은 **입력 pcd 의 normal orientation** 에 민감. `orient_normals_consistent_tangent_plane(k=15)` 로
  안정화했으나, 대상 표면이 얇거나 노이즈 많으면 여전히 fluffy 해질 수 있음 →
  `poisson_depth` / `density_quantile` 튜닝 또는 Photoneo PoissonRecon.exe 사용.
- Artec Studio 수준의 texture(UV + image) 를 원하면 별도 baking 모듈 필요.
- `docs/1_control_layers.md` 의 하위 레이어(`plan_and_execute`) 는 Phase 2 에서만 호출됨.
  Phase 1 은 턴테이블만 움직이고 로봇은 scan_init 자세 고정.

---

## 13. RGB 텍스처 입력 추가 (2026-04-24 업데이트)

`docs/phoxi_raw_dataset.md` 작성 과정에서 **PhoXi Gen3 의 내장 2D RGB 카메라** 지원이 확인됐다.
매뉴얼(PhoXiControl 1.16 User Manual p.10/40/48) 발췌:

> "RGB camera — Refers to the 2D RGB camera unit inside the MotionCam-3D Color **and PhoXi 3D Scanner Gen3**."
> "ColorCameraImage — outputs an image from the RGB camera unit in the resolution specified in the Color Settings."

이전 §6.3 에서 "PhoXi 는 monochrome" 이라 한 것은 **Gen2 이하 해당**. 우리 SEA-023 (Gen3) 은 RGB 지원.

### 13.1 두 가지 텍스처 획득 경로

| 경로 | 설정 | Intensity 결과 | 별도 ColorCamera 출력 |
|------|------|----------------|---------------------|
| **(A) ColorCamera 컴포넌트** | `ComponentSelector=ColorCamera` enable | grayscale (기존) | **별도 2D RGB 이미지** `(H_c, W_c, 3) uint8` — primary 와 독립 해상도, 내부 calib 으로 3D 점에 project 가능하지만 수학 추가 |
| **(B) TextureSource=Color** | `TextureSource.value = "Color"` | **RGB per 3D point** `(H, W, 3) uint8` — primary 해상도 유지, projection 불필요 | 없음 |
| **(A + B) 동시** | 둘 다 활성 | RGB per-point | 추가 2D RGB 이미지 |

우리 파이프라인에서 **Colored ICP** / Poisson vertex color 로 직접 쓰기 좋은 건 **(B)**.

### 13.2 실측된 GenICam 노드 (scripts/phoxi_probe_color.py 결과)

```
ComponentSelector.symbolics = ['Intensity','Range','Confidence',
                               'CoordinateMapA','CoordinateMapB','Normal','ColorCamera']
CameraSpace.symbolics        = ['PrimaryCamera','ColorCamera','CustomCamera','MarkerOrthoCamera']
TextureSource.symbolics      = ['Laser','LaserEnhanced','LED','Color','Computed','ComputedEnhanced','Focus']
ColorSettings_Resolution.symbolics =
    ['Resolution_3864x2192','Resolution_2576x1460','Resolution_1932x1096','Resolution_1288x730']
Scan3dFocalLength       = 2827.093505   (mm 단위)  ← native intrinsic, get_intrinsic fit 대체 가능
Scan3dPrincipalPointU/V = 1189.58 / 1017.15
```

### 13.3 코드 변경

**`mms/sensor/phoxi/phoxi_client.py`**
- `PhoxiConfig` 에 세 필드 추가:
  - `enable_color_camera: bool = False`
  - `texture_source: str = "LED"`
  - `color_resolution: Optional[str] = None`
- `initialize()` 가 `ColorCamera` 컴포넌트 enable 및 `TextureSource/ColorSettings_Resolution` 세팅
- `capture()` 에서 새 필드 채움:
  - `_last_color_image: (H_c, W_c, 3) uint8` — ColorCamera 컴포넌트 decoded
  - `_last_intensity_is_rgb: bool` — TextureSource=Color 일 때 True
  - `_last_intensity: (H, W)` 또는 `(H, W, 3)` 분기

**`scripts/phoxi_dataset_capture.py`**
- CLI: `--color` (ColorCamera enable), `--texture-source {LED,Laser,Color,...}`, `--color-resolution`
- 프레임 저장 확장: `color_image.npy` / `color_image.png` 추가, `meta.intensity_is_rgb`, `meta.color_image_shape`
- `session_meta.json` 에 `texture_source`, `color_camera_enabled`, `color_resolution` 기록
- `--out` 지정 시 **자동으로 `_YYYYMMDD_HHMMSS` suffix** 붙여 덮어쓰기 방지 (resume 시에만 정확 이름)

**`scripts/inspect_dataset_frame.py`**
- `color_image.npy` 자동 로드
- Intensity 가 `(H, W, 3)` 이면 **auto_stretch 없이 RGB 직접** pcd color 로 사용
- **matplotlib 2D 창** 에 Intensity + ColorCamera 이미지 side-by-side 표시 (`--texture-2d` 기본 on)

### 13.4 새 dataset 파일 스키마 (phoxi_raw_dataset.md §5 보완)

```
datasets/phoxi_color_ab_20260424_141529/
├── session_meta.json         + texture_source, color_camera_enabled, color_resolution
├── calibration/              (기존)
└── frames/frame_XXX/
    ├── range.npy             (2064, 2472, 3) float32 mm     ← 변함없음
    ├── intensity.npy         (2064, 2472) uint8 grayscale
    │                          또는 (2064, 2472, 3) uint8 RGB  ← TextureSource=Color 일 때
    ├── intensity.png                                         ← 동일 데이터 PNG
    ├── normals.npy           (2064, 2472, 3) float32        ← 변함없음
    ├── color_image.npy       (H_c, W_c, 3) uint8  ★ 신규     ← --color 일 때만
    ├── color_image.png                                        ← 신규
    └── meta.json             + intensity_is_rgb, color_image_shape
```

### 13.5 Phase 1 파이프라인 반영 TODO

아래 파일들이 **아직 grayscale 전제**. 적용 순서:

| 파일 | 필요 변경 |
|------|----------|
| `mms/nbv/pcd_accumulate_volume.py::integrate_frame` | `_last_intensity` shape 감지 — `(H, W, 3)` 면 auto_stretch 없이 직접, `(H, W)` 면 기존 percentile |
| `mms/sensor/phoxi/phoxi_dataset_replay.py::capture` | `color_image.npy` 로드해서 `_last_color_image` 에, intensity shape 자동 감지 |
| `mms/nbv/scan_session.py::_rule_based_scan` (progress vis) | ProgressVisualizer 에 RGB pcd 가 자연스럽게 흘러들도록 (이미 `update_pcd(pcd)` 가 `pcd.colors` 를 유지하므로 대부분 자동) |
| `main.py::SCAN_SETTINGS` (live capture 시) | `PhoxiConfig` 에 `texture_source="Color"`, `enable_color_camera=True` 를 옵션으로 노출 |

주의: **플래닝/측정값 변화 없음** — T_CO 체인, SOR, plane 제거, Poisson 까지 모두 pose/geometry 기반이라 RGB 여부와 무관.

---

## 14. Texture 기반 ICP (Colored ICP) 가능성

### 14.1 왜 고려하는가

Geometry-only ICP 의 약점 — 기하가 평평하거나 대칭인 영역에서 정합 슬라이딩(sliding degeneracy).
예: 플라스틱 상자의 평평한 측면, 같은 형상의 반복 구조. PhoXi S 해상도 높아서 대부분 geometry 로
붙지만, 얇고 평평한 물체에선 Colored ICP 가 효과적.

Phase 1 의 조건은 Colored ICP 에 **특히 유리**:
- 카메라 자체는 scan_init 자세 **고정** → exposure/WB 변화 없음
- θ 회전만 — 동일 물체 BRDF 가 유지돼 **RGB per-point 값이 일관**
- Auto-stretch 전 raw intensity 사용하면 광도 절대값도 프레임 간 비교 가능

### 14.2 이론 (Park, Zhou, Koltun 2017)

Colored ICP 목적함수:
```
E(T) = (1 − λ) · E_C(T) + λ · E_G(T)
```
- `E_G` = point-to-plane geometric residual (기존 point-to-plane ICP 와 동일)
- `E_C` = photometric residual — source 점의 색과 target surface 에서의 색 사이 차이
- `λ = lambda_geometric` (Open3D 기본 0.968 → geometry 비중 큼, 적당한 조절)

Multi-resolution 로 voxel 0.02 → 0.01 → 0.005 점진적 refinement 가 표준.

### 14.3 Open3D API

```python
o3d.pipelines.registration.registration_colored_icp(
    source, target,
    max_correspondence_distance,
    init,
    o3d.pipelines.registration.TransformationEstimationForColoredICP(
        lambda_geometric=0.968,
    ),
    o3d.pipelines.registration.ICPConvergenceCriteria(
        relative_fitness=1e-6, relative_rmse=1e-6, max_iteration=K,
    ),
)
```

요구사항:
- `source.colors`, `target.colors` 모두 `Vector3dVector [0,1]` 로 세팅
- `source.normals`, `target.normals` 모두 세팅 (point-to-plane 항에 필요)

### 14.4 우리 코드에 얹는 법 (제안)

**`mms/nbv/icp_strategy.py`** 에 `icp_colored_with_gates` 추가:

```python
def icp_colored_with_gates(
    source_pcd, target_pcd, init_T,
    voxel_schedule=(0.020, 0.010, 0.005),
    iter_schedule=(50, 30, 14),
    lambda_geometric=0.968,
    rmse_thresh=0.001,
    fitness_thresh=0.3,
    drift_trans_m=0.020,
    drift_rot_deg=10.0,
) -> IcpResult:
    T = np.asarray(init_T, dtype=float).copy()
    for voxel_m, max_iter in zip(voxel_schedule, iter_schedule):
        s = source_pcd.voxel_down_sample(voxel_m)
        t = target_pcd.voxel_down_sample(voxel_m)
        s.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(
            radius=voxel_m * 2.0, max_nn=30))
        t.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(
            radius=voxel_m * 2.0, max_nn=30))
        # 두 pcd 에 colors 있어야 함 — 없으면 geometry ICP 로 fallback
        if not (s.has_colors() and t.has_colors()):
            # ... fallback to regular ICP ...
            break
        result = o3d.pipelines.registration.registration_colored_icp(
            s, t, voxel_m * 1.4, T,
            o3d.pipelines.registration.TransformationEstimationForColoredICP(
                lambda_geometric=lambda_geometric),
            o3d.pipelines.registration.ICPConvergenceCriteria(
                relative_fitness=1e-6, relative_rmse=1e-6, max_iteration=max_iter),
        )
        T = result.transformation

    # Triple gate (기존과 동일) — rmse, fitness, drift
    ...
    return IcpResult(...)
```

**ScanSession** 는 `phase1_icp_method: Literal["geometric","colored","auto"] = "auto"` 옵션 추가:
- `"auto"`: 두 pcd 모두 colors 있으면 colored, 아니면 geometric
- `"geometric"`: 기존
- `"colored"`: 강제 (colors 없으면 에러)

`_rule_based_scan` 에서 ICP 호출부:
```python
if s.phase1_icp_method == "colored" or (
    s.phase1_icp_method == "auto"
    and source_pcd.has_colors() and target_pcd.has_colors()
):
    icp_res = icp_colored_with_gates(source_pcd, target_pcd, init_T=T_CO_nominal, ...)
else:
    icp_res = icp_with_gates(source_pcd, target_pcd, init_T=T_CO_nominal, ...)
```

### 14.5 필요한 선결 조건 (§13.5 와 중첩)

- **PcdAccumulateVolume.merged_pcd_cropped** 가 반환하는 target pcd 에 RGB 가 있어야 함 → 현재 intensity_auto_stretch 경로는 grayscale 을 3채널 복제, Colored ICP 작동하긴 하지만 color gradient 가 약함. 실제 RGB 값 쓰려면 `integrate_frame` 에서 `(H, W, 3)` intensity 를 직접 저장.
- **source pcd (새 프레임)** 도 RGB 보유 필요. `_build_current_frame_pcd_C` 확장 필요 — 현재는 position 만 넣음. `_last_intensity` shape 보고 RGB 컬럼 추가.

### 14.6 성능 / 기대 효과

| 항목 | Geometry ICP (현재) | Colored ICP (예상) |
|------|-------------------|--------------------|
| 속도 / 프레임 | ~0.3 s | ~0.8 s (multires 3 stage) |
| 평평한 면 정합 | sliding 위험 | 텍스처로 고정 |
| 대칭 물체 | 180° 모호 | 텍스처로 해결 |
| 순수 geometry 물체 (흰 플라스틱) | 동일 | 개선 없음 |
| 조명/WB 변화 민감도 | 없음 | 있음 (우리 조건은 고정이라 무관) |
| Open3D 구현 | ✅ | ✅ |

Phase 1 평균 프레임당 전체 처리 시간 중 ICP 는 10% 정도 → 0.5 s 증가는 수용 가능.

### 14.7 실험 우선순위

1. **§13.5 TODO 먼저 완료** — RGB per-point 를 누적 pcd 까지 보존
2. Colored ICP 옵션 추가 (§14.4) — `phase1_icp_method="auto"` default
3. 동일 데이터셋으로 `icp_method={"geometric", "colored"}` A/B 비교:
   - Mesh edge alignment 눈으로 비교
   - Colored ICP 의 Δt / Δr 가 geometric 대비 얼마나 변하는지
   - RMSE/fitness 수치 (geometry 기준, gate 는 그대로)

### 14.8 주의점 / 한계

- **PhoXi LED 조명** 은 백색광이지만 **구조광 프로젝터 영향** 으로 특정 파장 비중이 높을 수 있음 (실측 필요).
- **TextureSource=Color** 에서 실제 RGB 가 얼마나 유효한지 — 매뉴얼은 primary camera 가 RGB 로 interpolation / projection 된 결과임을 시사 (p.72 의 CameraSpace 규약). 실제 색 품질은 ColorCamera 컴포넌트(A) 가 더 좋을 가능성.
- **매우 어두운 표면**: percentile stretch 를 끄면 color gradient 가 약해 Colored ICP 기여 낮음. λ 를 낮춰 geometry 비중 높이거나 stretch 적용.

---

## 15. 업데이트 관련 파일 재정리

| 파일 | Phase 1 RGB 관련 역할 |
|------|----------------------|
| `mms/sensor/phoxi/phoxi_client.py` | PhoxiConfig(enable_color_camera, texture_source, color_resolution), capture() 가 _last_color_image / _last_intensity_is_rgb 채움 |
| `scripts/phoxi_dataset_capture.py` | `--color`, `--texture-source`, `--color-resolution` CLI, 프레임에 color_image.npy/png 저장, session_meta 에 옵션 기록, `--out` 자동 timestamp suffix |
| `scripts/phoxi_probe_color.py` | 센서 반납 전 1회 실행해 GenICam 노드 enumerate (Gen3 RGB 존재 확인) |
| `scripts/inspect_dataset_frame.py` | color_image 자동 로드, RGB intensity 를 pcd color 로 직접 사용, matplotlib 2D 텍스처 창 |
| `mms/nbv/pcd_accumulate_volume.py` | **TODO** — intensity 가 (H,W,3) 일 때 분기 |
| `mms/sensor/phoxi/phoxi_dataset_replay.py` | **TODO** — color_image 로드, intensity RGB shape 인식 |
| `mms/nbv/icp_strategy.py` | **TODO** — `icp_colored_with_gates` 추가 |
| `mms/nbv/scan_session.py` | **TODO** — `phase1_icp_method="auto"` 플래그, `_build_current_frame_pcd_C` 에 RGB 컬럼 |
| `docs/phoxi_raw_dataset.md` | RGB 데이터 흐름 상세 (§2.2, §4.1, §13) |
| `docs/3_phase1.md` | §13, §14, §15 (이 파일) — RGB 입력 반영 + Colored ICP 도입 계획 |

---

## 16. 결론 / 권장 진행

1. 센서 반납 전 **`--texture-source Color --color`** 로 한 세션 이상 재캡처 확보 (완료 ✓).
2. 오프라인에서 §13.5 / §15 의 TODO 완료 → `phase1_from_dataset.py` 가 RGB 를 써서 mesh 생성하도록.
3. A/B: Geometry-only vs Colored ICP 로 동일 데이터셋 mesh 생성 후 육안 / 수치 비교.
4. 결과에 따라 `phase1_icp_method` 기본값 확정 → Phase 2 활성화 검토.


 Phase 1 데이터 경로

  PhoXi sensor (GenTL Range/Normal/Intensity/ColorCamera)
    ↓  capture() — raw 그대로 sensor._last_* 에 저장
    ↓  (Frame 객체는 ee_pose 용으로만 유지, 데이터 변형 없음)
  PcdAccumulateVolume.integrate_frame(frame, T_CO)
    ↓  sensor._last_organized_pts 직접 읽기 (mm → m)
    ↓  depth 필터 (50mm < z < 2000mm) — 이것만 유일한 필터
    ↓  C → O 변환 (T_CO)
    ↓  누적
  extract_cropped_mesh (루프 종료 시 1회)
    ↓  voxel_down_sample (누적 pcd 합치기용, 품질 손상 거의 없음)
    ↓  [SOR skip]
    ↓  [plane-remove skip]
    ↓  estimate_normals + orient_consistent
    ↓  Poisson
    ↓  [density 필터 skip]
    ↓  원통 crop (z_min, R)  ← main.py SCAN_SETTINGS 에서 제어
    → mesh