# PhoXi Raw Dataset — 수집 / 저장 / 재사용 파이프라인 상세

PhoXi 3D Scanner Gen3 S 를 3시간 후 반납해야 하는 상황에서, **센서가 주는 raw 데이터
그대로** 를 디스크에 떠두고 나중에 동일 파이프라인을 오프라인으로 돌릴 수 있게 한
기록. C++/GenTL 레벨의 데이터 흐름부터 디스크 스키마, 재생 방법까지 정리.

---

## 1. 전체 데이터 흐름 (Hardware → Disk)

```
[PhoXi 3D Scanner Gen3 (S)]               (센서 H/W)
   │ 물리 — GigE Vision + structured-light 프로젝터
   ▼
[PhoXi Control 펌웨어]                     (SEA-023 내부, vendor)
   │ GenICam-compliant GigE Vision 인터페이스
   ▼
[mvGenTLProducer.cti]                      (DLL; MATRIX VISION 제공)
   │ GenTL Producer — C ABI, pixel-format PFNC
   ▼
[genicam Python binding (_gentl, genicam.gentl, genicam.genapi)]
   │ ctypes bridge: GenTL capsule → Python
   ▼
[Harvesters]                               (Python — ImageAcquirer, Buffer, Component2DImage)
   │ buffer.payload.components[i].data  →  numpy view (GenTL 버퍼 메모리)
   ▼
[PhoxiClient]                              (mms/sensor/phoxi/phoxi_client.py)
   │ reshape + copy + valid-filter + intensity decode
   │   → self._last_organized_pts        (H, W, 3) float32 mm
   │   → self._last_organized_normals    (H, W, 3) float32
   │   → self._last_intensity            (H, W)    uint8
   │   → self._last_depth_mm             (H, W)    float32 mm
   │   → ScanResult (flat valid pts 만 포함)
   ▼
[scripts/phoxi_dataset_capture.py]
   │ np.save / PIL-like PNG / JSON
   ▼
[Disk]  datasets/phoxi_YYYYMMDD_HHMMSS/frames/frame_XXX/
```

---

## 2. PhoXi 출력 설정 (CalibratedABC_Grid)

`PhoxiClient.initialize()` 에서:

```python
features.Scan3dOutputMode.value = "CalibratedABC_Grid"
features.TriggerMode.value      = "On"
features.TriggerSource.value    = "Software"
features.TextureSource.value    = "LED"            # 구조광 대신 LED → texture grayscale
_enable_components(features, ["Intensity", "Range", "Normal"])
```

### 2.1 "CalibratedABC_Grid" 의 의미

- **ABC** = X, Y, Z (3D 좌표 세 성분).
- **Calibrated** = 제조사 내부 캘리브로 이미 rectify/undistort 된 3D 좌표.
  수치 단위는 **mm**, 기준은 **센서(C) 프레임**.
- **Grid** = 2D pixel grid 로 organized — 각 픽셀이 `(X, Y, Z)` 1조 를 담음.
  (Unorganized 모드는 유효 포인트만 flat list 로 주므로 pixel 대응 안 됨.)

### 2.2 활성 컴포넌트 3종

| 컴포넌트 | GenICam 이름 (`data_format`) | PFNC 예시 | 의미 | Fetched numpy |
|----------|------------------------------|-----------|------|---------------|
| Range    | `Coord3D_ABC32f`             | `0x02180063` | 픽셀당 `float32 x3` — (X,Y,Z) mm, C 프레임 | `(H·W·3,)` float32, flat |
| Normal   | `Coord3D_ABC32f`             | `0x02180063` | 픽셀당 `float32 x3` — surface normal | `(H·W·3,)` float32, flat |
| Intensity | `Mono12` or `Mono10` (장치에 따라) | `0x01100105` (Mono12) | grayscale 조도 (LED 조명) | `(H·W,)` uint16 (packed) |

해상도 (PhoXi S Gen3, 실측):
- `width × height = 2472 × 2064` (`Component2DImage.width`, `.height` 으로 읽음)

---

## 3. C++ / GenTL 레벨에서 실제로 넘어오는 형태

### 3.1 GenTL Producer 쪽 (C ABI)

GenTL 은 C API 로 정의된 vendor-neutral 카메라 인터페이스. mvGenTLProducer 가
PhoXi 의 GigE Vision 프로토콜을 GenTL 로 번역한다.

중요한 구조체 (개념적, 실제 헤더는 `GenTL.h`):

```c
// 각 fetched "buffer" 에 대해 다음과 같은 속성이 노출됨 (DS_ → "Data Stream")
typedef enum {
    BUFFER_INFO_BASE,            // void*     — pixel memory pointer
    BUFFER_INFO_SIZE,            // size_t    — total bytes
    BUFFER_INFO_WIDTH,           // uint64_t
    BUFFER_INFO_HEIGHT,          // uint64_t
    BUFFER_INFO_PIXELFORMAT,     // uint64_t  — PFNC code
    BUFFER_INFO_PAYLOADTYPE,     // int64_t   — IMAGE, MULTI_PART, CHUNK_DATA ...
    // ...
} BUFFER_INFO_CMD;

// Multi-part payload 일 때:
DS_GetBufferPartInfo(handle, part_idx, PART_INFO_DATA_PTR, ...);
```

PhoXi 의 한 fetch 는 **MULTI_PART payload** — Range, Normal, Intensity 세 파트를
한 번에 묶어 보냄. 각 파트는 개별 `pixel_format` / `width` / `height` 를 가짐.

### 3.2 PFNC (Pixel Format Naming Convention)

| PFNC 코드 (hex) | 이름 | pixel 당 비트 | numpy dtype |
|-----------------|------|--------------|--------------|
| `0x02180063` | `Coord3D_ABC32f` | 96 (= 3 × float32) | float32, 3채널 |
| `0x01100105` | `Mono12` | 12 (packed) / 16 (unpacked) | uint16 |
| `0x01080105` | `Mono10` | 10 / 16 | uint16 |
| `0x01080001` | `Mono8` | 8 | uint8 |
| `0x02180002` | `RGB8` | 24 | uint8, 3채널 |

Harvesters 가 PFNC 를 보고 numpy dtype 과 shape 을 결정.

### 3.3 Harvesters 의 Python 표현

`buffer.payload.components` → `List[Component2DImage]`. 각 객체:

```python
class Component2DImage:
    data: np.ndarray         # GenTL 버퍼에 대한 numpy view — context 안에서만 유효
    width: int
    height: int
    data_format: str         # "Coord3D_ABC32f" 등 PFNC 이름
    num_components_per_pixel: int
    # ...
```

**중요 — zero-copy view**: `Component2DImage.data` 는 GenTL 버퍼 메모리를 **그대로**
numpy 로 감싼 view 다. fetch context 를 벗어나면 해당 GenTL 버퍼가 재사용
(re-queue) 될 수 있어서 데이터가 오염된다. 그래서 **반드시 `.copy()`** 해서 Python
소유 메모리로 옮겨야 한다.

---

## 4. PhoxiClient 에서의 실제 처리

`mms/sensor/phoxi/phoxi_client.py::capture()` 핵심:

```python
# 1) software trigger
self._features.TriggerSoftware.execute()

# 2) fetch (context manager; timeout 안에 대기)
with self._ia.fetch(timeout=15.0) as buffer:
    # 파트 이름 순서로 묶음 (initialize 에서 enable 한 순서)
    components: Dict[str, Component2DImage] = dict(
        zip(self._comp_names, buffer.payload.components)
    )

    # === Range ===
    range_comp = components["Range"]
    h_img = range_comp.height          # 2064
    w_img = range_comp.width           # 2472
    # .data 는 (H*W*3,) float32 flat view — reshape 후 반드시 copy
    points_flat = range_comp.data.reshape(-1, 3).copy().astype(np.float32)

    # === Normal ===  (PFNC 동일)
    normal_flat = components["Normal"].data.reshape(-1, 3).copy().astype(np.float32)

    # === Intensity ===  (Mono12 등, raw 16-bit)
    tex_raw = components["Intensity"].data.copy()
    tex_fmt = components["Intensity"].data_format
```

Context 빠져나오면 GenTL 버퍼 해제, 우리는 copy 로 소유.

### 4.1 Organized reshape

```python
organized_pts = points_flat.reshape(h_img, w_img, 3)   # (H, W, 3) float32 mm
self._last_organized_pts = organized_pts
self._last_organized_normals = normal_flat.reshape(h_img, w_img, 3).copy()

depth_mm = organized_pts[:, :, 2].astype(np.float32)   # Z 성분 (mm)
depth_mm[depth_mm < 0] = 0.0
self._last_depth_mm = depth_mm
```

### 4.2 Intensity decode (raw → uint8)

```python
@staticmethod
def _decode_intensity(raw, fmt, h, w):
    if fmt == "RGB8":
        return raw.astype(np.uint8).reshape(h, w, 3).mean(axis=2).astype(np.uint8)
    elif fmt == "Mono10":
        return (raw.astype(np.float32) / 1024.0 * 255).clip(0,255).astype(np.uint8).reshape(h, w)
    elif fmt == "Mono12":
        return (raw.astype(np.float32) / 4096.0 * 255).clip(0,255).astype(np.uint8).reshape(h, w)
    else:
        _max = float(raw.max()) or 1.0
        return (raw.astype(np.float32) / _max * 255).clip(0,255).astype(np.uint8).reshape(h, w)
```

우리는 `tex_fmt` 값에 따라 12/10-bit 풀 레인지를 [0, 255] 로 linear 매핑.

### 4.3 Invalid pixel 필터

PhoXi 는 측정 실패 픽셀을 `(0, 0, 0)` 으로 채움.

```python
valid = ~np.all(points_flat == 0.0, axis=1)       # (H*W,) bool
points_valid  = points_flat[valid]                # (N_valid, 3)
normals_valid = normal_flat[valid]                # (N_valid, 3)
```

`ScanResult` 에는 `points_valid` / `normals_valid` 만 flat 으로 담고, **organized
텐서는 `self._last_*` 에 보관** — 데이터셋 덤프는 이쪽을 쓴다.

### 4.4 세션 버퍼 요약

| 필드 | shape | dtype | 단위 | 프레임 |
|------|-------|-------|------|--------|
| `_last_organized_pts` | `(H, W, 3)` | float32 | mm | C (센서) |
| `_last_organized_normals` | `(H, W, 3)` | float32 | unit vec | C |
| `_last_intensity` | `(H, W)` | uint8 | — | — |
| `_last_depth_mm` | `(H, W)` | float32 | mm | Z 성분 |

모두 row-major (C-order). invalid 픽셀은 `(0, 0, 0)` / 0 으로 유지 — 나중 단계에서
필터.

---

## 5. Dataset 디스크 스키마

### 5.1 폴더 구조

```
datasets/phoxi_20260424_124213/
├── session_meta.json           ← 세션 메타 + intrinsic + 캘리브 포인터
├── calibration/
│   ├── hand_eye_phoxi.yaml     ← T_E_C (E→C) 원본 복사
│   └── turntable_frame.yaml    ← T_B_F0 (B→F at θ=0) 원본 복사
└── frames/
    ├── frame_000/
    │   ├── range.npy           (H, W, 3) float32 mm  ← PhoXi Range organized
    │   ├── intensity.npy       (H, W)    uint8      ← Mono12 decoded grayscale
    │   ├── intensity.png                             ← 동일 데이터 PNG
    │   ├── normals.npy         (H, W, 3) float32    ← PhoXi Normal organized
    │   └── meta.json           ← θ_target/actual, T_EB, timestamp
    ├── frame_001/
    ...
    └── frame_023/
```

### 5.2 `session_meta.json` 예시

```json
{
  "session": "20260424_124213",
  "n_frames": 24,
  "theta_step_deg": 15.0,
  "turntable_vel_rad_s": 0.17453,
  "dwell_s": 0.4,
  "scan_init_joints_deg": [-0.5, -43.5, 0.2, 46.2, -0.2, 69.2, 1.0],
  "robot_ip": "192.168.1.210",
  "turntable_ip": "192.168.0.10",
  "phoxi_serial": "SEA-023",
  "intrinsic": {
    "fx": 2791.5, "fy": 2768.9,
    "cx": 1189.4, "cy": 1024.2,
    "width": 2472, "height": 2064
  },
  "calibration": {
    "hand_eye_yaml": "hand_eye_phoxi.yaml",
    "turntable_frame_yaml": "turntable_frame.yaml",
    "T_OF": "identity (O ≡ F at θ=0)"
  },
  "frames_dir": "frames",
  "created_at": "2026-04-24T12:42:13"
}
```

**intrinsic** 은 `PhoxiClient.get_intrinsic()` 의 least-squares fit 결과
(organized Range 의 `(X, Y, Z)` 와 pixel 좌표 `(u, v)` 로 pinhole 모델 `u = fx·X/Z + cx`,
`v = fy·Y/Z + cy` 을 풂).

**calibration/** 하위에 `hand_eye_phoxi.yaml` (T_E_C) 와 `turntable_frame.yaml`
(T_B_F0) 를 **복사해 둠** — 세션이 self-contained.

### 5.3 `frames/frame_XXX/meta.json`

```json
{
  "idx": 5,
  "theta_target_deg": 75.0,
  "theta_target_rad": 1.30899693899,
  "theta_actual_deg": 75.0,
  "theta_actual_rad": 1.30899617,
  "T_EB": [
    [ ..., ..., ..., ...],
    [ ..., ..., ..., ...],
    [ ..., ..., ..., ...],
    [0.0, 0.0, 0.0, 1.0]
  ],
  "timestamp": 1714060000.123,
  "capture_elapsed_s": 2.56,
  "resolution": [2064, 2472],
  "n_valid_pixels": 712463
}
```

**T_EB**: 캡처 시점의 로봇 FK 결과 (E → B), translation 단위 **meters** (`pose6d_to_mat`
가 mm/1000 을 이미 수행).

### 5.4 용량 어림 (PhoXi S, 2064×2472)

| 파일 | 바이트 |
|------|------|
| `range.npy` | 2064·2472·3·4 B ≈ **58 MB** |
| `normals.npy` | 동일 ≈ **58 MB** |
| `intensity.npy` | 2064·2472 B ≈ **5 MB** |
| `intensity.png` | 보통 0.5–2 MB (압축) |
| `meta.json` | < 1 KB |

**프레임당 ≈ 120 MB**, 24 프레임 → **약 3 GB**. 두 세션이면 ~6 GB.

---

## 6. 저장 스크립트 (`scripts/phoxi_dataset_capture.py`)

### 6.1 실행 흐름

1. 출력 폴더 생성 (`datasets/phoxi_YYYYMMDD_HHMMSS/`)
2. 캘리브레이션 YAML 2종 복사 → `calibration/`
3. 로봇/턴테이블/PhoXi 순차 연결
4. 로봇 `set_servo_angle(SCAN_INIT_JOINTS_DEG)` 로 스캔 시작 자세 이동
5. 턴테이블 `_reset_turntable_to_zero` (내부적으로 `_move_with_recovery` 사용)
6. **Warmup capture 1회** — `get_intrinsic()` fit 용
7. `session_meta.json` 기록
8. θ 스케줄 (`step_rad × N = 2π`) 루프:
   - `_move_with_recovery(tt, θ_i, vel, n_retries=2)` — alarm reset + servo 재기동
     포함 2회 재시도
   - `dwell (0.40s)` — 기계 settling
   - `sensor.capture()` → PhoXi 트리거 + fetch
   - `save_frame(...)` — `range.npy`, `intensity.npy/png`, `normals.npy`,
     `meta.json`
9. 이동 실패 / 캡처 실패 시 해당 프레임 skip (다음 프레임 진행)

### 6.2 CLI 옵션

| 옵션 | 기본 | 설명 |
|------|------|------|
| `--step <deg>` | 15.0 | θ 간격 (deg). 15° 면 24 프레임, 10° 면 36 프레임. |
| `--out <name>` | `phoxi_<stamp>` | 세션 폴더명 |
| `--resume` | off | 기존 세션에서 `frame_XXX/range.npy` 없는 인덱스만 재캡처 (**멱등**) |
| `--skip-scan-init` | off | 로봇 이동 생략 (이미 좋은 자세일 때) |
| `--confirm-each` | off | 프레임마다 Enter 대기 (디버그) |

### 6.3 턴테이블 복구 로직

Ezi-SERVO 드라이버는 부하 누적이나 누적 명령 queue 오류로 가끔 fault 상태에
빠진다. 로그에 나타나는 신호:

```
Function(FAS_GetAxisStatus) was failed.    ← 반복
[Turntable] wait_motion_done: axis status 실패 (20회) — 포기
[Turntable] move_abs 실패 (return=1)
```

복구 시퀀스 (`_recover_turntable`):

```python
tt.check_drive_err()      # alarm reset
sleep(0.3)
tt.set_servo_on(False)
sleep(0.3)
tt.set_servo_on(True)
sleep(0.5)
```

`_move_with_recovery(tt, θ, vel, n_retries=2)` 가 `move_abs` 거부 또는
`wait_motion_done` 실패 시 자동으로 이 복구를 호출 후 재시도. **2회 재시도 후에도
실패하면** 물리 전원 재투입이 필요 (드라이버 하드-락).

### 6.4 `--resume` 의미

`frame_XXX/range.npy` 가 존재하는 인덱스는 skip. 빠진 인덱스로만 이동/캡처.
동일 명령을 여러 번 돌려도 멱등 — 중간에 끊어져도 다시 실행하면 빠진 부분만
채움.

단, **`frame_000` 이 θ=0 에 제대로 찍혔는지는 별도 확인** — `θ=0` reset 이 timeout
되면 첫 프레임이 엉뚱한 각도에서 찍힌 채 저장된다. 그 경우 수동으로 폴더 삭제
후 `--resume`.

---

## 7. 오프라인 재생 — `PhoxiDatasetReplay`

`mms/sensor/phoxi/phoxi_dataset_replay.py`

센서 없이 디스크의 데이터셋을 live sensor 처럼 쓰기 위한 어댑터. `PhoxiClient`
와 동일한 **인터페이스** 를 제공:

| API | PhoxiClient (live) | PhoxiDatasetReplay (offline) |
|-----|-------------------|------------------------------|
| `initialize()` | GenTL producer 로드 + Harvesters create | 디렉토리 인덱싱 |
| `shutdown()` | ia.stop / harvester.reset | no-op |
| `capture(frame_id, ts) → ScanResult` | 소프트웨어 트리거 + fetch | 다음 `frame_XXX` 로드 |
| `get_intrinsic(force_refit=False)` | organized Range → least-squares fit | `session_meta.intrinsic` 우선, 없으면 fit |
| `_last_organized_pts` / `_last_intensity` / `_last_depth_mm` / `_last_organized_normals` | capture 시 갱신 | capture 시 갱신 |

추가 API (오프라인 전용):

| API | 설명 |
|-----|------|
| `n_frames` | 총 프레임 수 |
| `seek(idx)` | 다음 재생 프레임 인덱스 지정 |
| `get_pose(idx) → (θ_rad, T_EB)` | 저장된 캡처 시점의 실제 θ 및 로봇 FK |
| `session_meta` | 세션 메타 dict 복사본 |

---

## 8. Phase 1 에 데이터셋을 적용하는 방법

### 8.1 원리

Phase 1 mesh 는 `T_CO = T_FO · T_BF(θ) · T_EB · T_CE` 체인과 센서 organized
range 만 있으면 **live/오프라인 구분 없이** 동일하게 재구성된다 (docs/0_nbv_phase.md §4).

Dataset 에 이미:
- **organized range (C frame mm)** — `frame_XXX/range.npy`
- **θ_actual** — `frame_XXX/meta.json`
- **T_EB** — 동일 meta
- **T_EC / T_B_F0** — `calibration/*.yaml`
- **intrinsic** — `session_meta.json`

전부 있으므로 T_CO 재계산이 가능.

### 8.2 옵션 A — `MMS.sensor` 에 replay 주입 (최소 변경)

```python
from mms.system import MMS, MMSConfig
from mms.sensor.phoxi.phoxi_client import PhoxiConfig
from mms.sensor.phoxi.phoxi_dataset_replay import PhoxiDatasetReplay, PhoxiDatasetConfig

DATASET = "datasets/phoxi_20260424_124213"

cfg = MMSConfig(
    phoxi=PhoxiConfig(serial_number=None, target_interval_s=0.0),   # placeholder
    turntable_frame_yaml=f"{DATASET}/calibration/turntable_frame.yaml",
    sensor_frames_yaml  =f"{DATASET}/calibration/hand_eye_phoxi.yaml",
    T_EC_key            ="T_E_C",
)
mms = MMS(cfg)
# live sensor 대신 replay 로 교체
mms.sensor = PhoxiDatasetReplay(PhoxiDatasetConfig(dataset_dir=DATASET))
mms.sensor.initialize()
```

이 상태에서 `mms.capture_frames(1)` 을 부를 때마다 디스크의 다음 프레임이 읽힌다.
단, `ScanSession._rule_based_scan` 은 여전히 **real turntable/robot** 을 요구하므로
오프라인 재생에는 그대로 쓸 수 없다 → 옵션 B.

### 8.3 옵션 B (권장) — 오프라인 Phase 1 전용 스크립트

`scripts/phase1_from_dataset.py` 를 하나 두는 게 깔끔.

```python
# 의사코드
replay = PhoxiDatasetReplay(PhoxiDatasetConfig(dataset_dir=DATASET))
replay.initialize()

T_EC  = load_transform(f"{DATASET}/calibration/hand_eye_phoxi.yaml", "T_E_C")
T_BF0 = load_transform(f"{DATASET}/calibration/turntable_frame.yaml", "T_B_F0")
tt    = TurntableTransformConfig(T_BF0)
T_OF  = np.eye(4)                                # O ≡ F at θ=0
T_CE  = np.linalg.inv(T_EC)

volume = PcdAccumulateVolume(sensor=replay, voxel_length=0.002, depth_trunc=2.0)

for i in range(replay.n_frames):
    replay.seek(i)
    scan = replay.capture(frame_id=i)            # organized_* 필드 자동 갱신
    theta_act, T_EB = replay.get_pose(i)
    T_CO = np.linalg.inv(T_OF) @ np.linalg.inv(tt.T_FB(theta_act)) @ T_EB @ T_CE
    volume.integrate_frame(frame=None, T_CO=T_CO)
    # (i >= 1) incremental ICP refinement 도 여기서 가능

# 마무리
volume.save_pcd(f"{DATASET}/out/phase1_merged.ply", cropped=True, z_min=0.005, R=0.12)
mesh = volume.extract_cropped_mesh(z_min=0.005, R=0.12, verbose=True)
o3d.io.write_triangle_mesh(
    f"{DATASET}/out/phase1_mesh.ply", mesh,
    write_vertex_colors=True, write_vertex_normals=True,
)
```

ICP refinement 는 `mms.nbv.icp_strategy.icp_with_gates` 를 live 파이프라인과 동일하게 재사용.

### 8.4 MMS 래퍼 (T_CO 재계산을 Helper 에 맡기기)

더 깔끔하게 하려면 MMS 의 `T_CO(θ, T_EB)` 메서드를 그대로 쓸 수 있다 — MMS 가
`self._T_OF`, `self._T_EC`, `self.turntable_transform` 을 이미 가지고 있음.

```python
mms = MMS(cfg_with_dataset_yaml)
mms.sensor = replay
mms.initialize()                   # replay init (no-op)

for i in range(replay.n_frames):
    replay.seek(i)
    scan = replay.capture(frame_id=i)
    θ, T_EB = replay.get_pose(i)
    T_CO = mms.T_CO(theta=θ, T_EB=T_EB)
    volume.integrate_frame(frame=None, T_CO=T_CO)
```

### 8.5 검증용 sanity test

Live 파이프라인과 동일한 mesh 가 나오는지:
1. 같은 θ_actual, T_EB, organized range 로 T_CO 계산
2. PcdAccumulateVolume 의 Poisson / plane-remove / SOR 파라미터 동일
3. 결과 `phase1_mesh_*.ply` 를 vertex-wise 비교 (CloudCompare C2C distance <
   0.5 mm)

---

## 9. 시행착오 & 함정

### 9.1 Ezi-SERVO 드라이버 fault
- `FAS_GetAxisStatus` 연속 실패 → 드라이버 fault.
- 스크립트 자동 복구 2회. 실패 시 **전원 재투입**.

### 9.2 frame_000 오염
- scan 직전 `θ=0 reset` 이 timeout 되면 frame_000 이 엉뚱한 각도에서 찍힘.
- 데이터셋 품질 확인: `frames/frame_000/meta.json` 의 `theta_actual_deg`가 ~0
  근처인지 꼭 체크.
- 틀리면 폴더 삭제 후 `--resume`.

### 9.3 PhoXi 장치 lock
- `AccessDeniedException` 발생 시 이전 프로세스가 정상 종료 안 됨.
- **회피**: `PhoxiConfig(serial_number="SEA-023")` 로 시리얼 고정
  (None 이면 GigE 네트워크의 다른 장치 — 예: Helios — 가 잡힐 수 있음).
- 필요 시 PhoXiControl GUI 에서 disconnect 후 재시도.

### 9.4 Mono12 intensity 어두움
- 12-bit raw 값이 실제로는 낮은 부분만 씀 → decoded uint8 에서 0~50 수준.
- `PcdAccumulateVolume.intensity_auto_stretch=True` 가 per-frame percentile 2–98
  으로 확장 — 이는 **통합 시** 적용되며, 디스크 파일의 `intensity.npy` 는 raw decoded
  값을 유지 (postprocess 에서 원본 그대로 쓸 수 있게).

### 9.5 Harvesters zero-copy view
- 절대 `.data` 를 fetch context 밖으로 가지고 나가지 말 것 (GenTL 버퍼 재사용됨).
- 항상 `.copy()` 로 Python 소유 메모리로 복사.

---

## 10. 재현성 체크리스트

새 세션 / 오프라인 재생에서 mesh 가 live 와 동일한지 확인하려면:

- [ ] `session_meta.intrinsic` 가 있고 `fx, fy, cx, cy` 값이 정상 (fx/fy ≈ 2770, cx ≈ w/2)
- [ ] `calibration/hand_eye_phoxi.yaml` 의 `T_E_C` 가 원본과 동일
- [ ] `calibration/turntable_frame.yaml` 의 `T_B_F0` 가 원본과 동일
- [ ] `frames/*/meta.json` 의 `theta_actual_deg` 가 예상 값 (0, 15, 30, ...) 과 2° 이내
- [ ] `range.npy` 의 dtype=float32, shape=(2064, 2472, 3)
- [ ] `normals.npy` 의 dtype=float32, shape=(2064, 2472, 3)
- [ ] `intensity.npy` 의 dtype=uint8, shape=(2064, 2472)
- [ ] invalid pixel 마커 `(0,0,0)` 이 이미지 가장자리 등에 존재

---

## 11. 관련 파일

| 파일 | 역할 |
|------|------|
| `mms/sensor/phoxi/phoxi_client.py` | **live** PhoXi 드라이버 — Harvesters/GenTL 래퍼. `_last_organized_*`, `get_intrinsic`, `capture → ScanResult`. |
| `mms/sensor/phoxi/phoxi_dataset_replay.py` | **offline** 어댑터 — PhoxiClient API 와 호환되는 디스크 replay. |
| `scripts/phoxi_dataset_capture.py` | **데이터셋 캡처 실행 스크립트** — `_move_with_recovery`, `--resume`, `session_meta` 작성. |
| `mms/sensor/scan_result.py` | `ScanResult` dataclass — live/offline 공통 출력. |
| `mms/core/frames.py` | `Frame` dataclass — `MMS._scan_to_frame` 에서 ScanResult → B frame 변환. 본 데이터셋은 ScanResult 단계까지만 저장 (B frame 변환은 pose 기반 재계산 가능하므로). |
| `mms/utils/transforms.py` | `TurntableTransformConfig`, `load_transform`, `compute_T_CO` 등 — 오프라인에서도 동일하게 재사용. |
| `mms/nbv/pcd_accumulate_volume.py` | O 프레임 누적 + Poisson mesh — live/offline 공통. |

---

## 12. 다음 단계

- `scripts/phase1_from_dataset.py` 작성 — §8.3 옵션 B 구체화.
- 여러 세션을 combined dataset 으로 묶어 동일한 Phase 1 파이프라인에 투입.
- Intrinsic + 캘리브레이션이 같은 한 **로봇 없이 dataset 만으로 Phase 2 (frontier NBV) 시뮬**도 가능 (실제 이동은 못 하지만 T_CO_des → 가장 가까운 저장된 프레임 선택).
- vertex color 가 제대로 보일 GLB/OBJ+texture baking 루트도 검토.
