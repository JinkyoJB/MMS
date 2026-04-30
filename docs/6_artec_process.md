# Artec Process Pipeline — 2026-04-29

> Artec 자료구조 (FrameMeshHandle / ScanHandle / ModelHandle) 기반의 풀 워크플로우.
> Artec Studio §4 의 6 단계 (Scanning → Cleaning → Alignment → Registration → Fusion
> → Postprocessing) 를 코드로 옮긴 것. PhoXi 의 `Frame` / `Stream` 흐름과 분리.

---

## 1. 설계 원칙

- **PhoXi 와 평행 분리**. PhoXi 는 기존 `mms/core/frames.py` (`Frame`/`Stream`) +
  `mms/nbv/scan_session.py` 그대로 유지. Artec 은 SDK native 자료구조 + 새
  세션/오케스트레이터.
- **Artec 자료구조를 그대로 차용**. 변환/래핑하지 않음. T_EB·θ 같은 외부 메타만
  parallel array 로 보관.
- **Studio §4 워크플로우 1:1 매핑**. 우리는 1단계 (Scanning) 의 Phase 1+2 를
  자동화하고, 2~6단계는 SDK 알고리즘에 위임.

---

## 2. 자료구조 흐름

```
ArtecClient.capture_frame()
  ↓
FrameMeshHandle  ─ vertices(N,3 mm) + uv(N,2) + image(H,W,3) + faces(M,3)
                  (Artec C 프레임)
  ↓ ScanHandle.add_frame(fmh)
ScanHandle  ─ 한 연속 스캔 세션의 frame 시퀀스
  ↓ ModelHandle.add_scan(scan)
ModelHandle  ─ scan 묶음 (우리 케이스에선 보통 IScan 1개)
  ↓ Algorithms (SerialReg → GlobalReg → Fusion → Postprocess)
ModelHandle (with composite mesh)
  ↓ final_vertices() / final_faces()
o3d.TriangleMesh
```

외부 메타 (Artec native 가 안 가짐):

```python
@dataclass
class ArtecScanContext:
    model: ModelHandle              # SDK native
    scan:  ScanHandle               # 누적 중인 IScan (model 안)
    T_EB_list:    list[np.ndarray]  # 각 frame i 의 EE pose (E→B, m)
    theta_list:   list[float]       # 각 frame i 의 turntable θ (rad)
    timestamps:   list[float]
    surface_areas: list[float]      # Phase 2 plateau 검출용
```

---

## 3. C++ Binding 확장 (2026-04-29)

기존 binding 은 IModel/IScan READ 만 노출. 다중 frame 누적을 위해 추가:

```cpp
// mms/sensor/artec/artec_base_binding.cpp
m.def("create_scan",       &create_scan);
m.def("scan_add_frame",    &scan_add_frame);
m.def("model_add_scan",    &model_add_scan);
```

대응 Python:

```python
# mms/sensor/artec/artec_base.py
def create_scan() -> ScanHandle: ...

class ScanHandle:
    def add_frame(self, frame: FrameMeshHandle) -> None: ...

class ModelHandle:
    def add_scan(self, scan: ScanHandle) -> None: ...
```

> 빌드: `cmake --build mms/sensor/artec/build --config Release`

---

## 4. 단계 매핑 — §3 SDK General Pipeline (실제 호출 순서)

`MMS.artec_process()` 는 **§3 SDK General Pipeline** 순서를 따른다.
(Studio §4 는 GUI 사용자 워크플로우 라벨이지 알고리즘 호출 순서 X — 참고용.)

| 단계 | 알고리즘 | 입력 | 메모 |
|---|---|---|---|
| **0. Scanning** | `ArtecScanSession.run()` | (live capture) | Phase 1 (15°×24) + Phase 2 (frontier NBV) — 우리 contribution |
| **1. Alignment** | `serial_registration` | IModel (raw frames) | Frame-to-frame ICP 정렬 |
| **2. Registration** | `global_registration` | IModel | 전체 scan 정합 (scan 1개면 거의 no-op) |
| **3a. Cleaning #1** | `outliers_removal` | IModel (per-frame mesh) | **Fusion 전** — frame 단계 노이즈 제거 |
| **3b. Cleaning #2** | `small_objects_filter` | IModel | Fusion 전 — 작은 floating cluster 제거 |
| **(중간 저장)** | `save_project()` | IModel | Fusion 실패 복구용 .sproj |
| **4. Fusion** | `poisson_fusion` (또는 `fast_fusion`) | IModel (clean frames) | composite mesh 생성 |
| **5. Simplify** | `mesh_simplify` | composite mesh | polycount 감소 (옵션) |
| **6. Texturize** | `texturize` | composite mesh + frame textures | UV/atlas + texture baking |

> **중요**: Cleaning(3a/3b) 은 Fusion **전**에 와야 한다. Fusion 후에 두면 composite
> mesh 에 outlier 가 박힌 채 Texturize 까지 가서 실패함 (관측: ErrorCode 0x80010203).
> Studio §4 의 "6. Postprocessing" 은 사용자 GUI 흐름이고, 알고리즘 순서는 §3.

각 알고리즘은 `ArtecClient` 의 staticmethod 로 wrap 됨 (`mms/sensor/artec/artec_client.py`).
참고로 `Algorithms.run_pipeline()` 도 같은 §3 순서로 구현돼 있음.

---

## 5. 핵심 결정사항 (2026-04-29)

### 5.1 ICP — SDK SerialRegistration 으로 위임

Phase 1 누적 시 우리 ICP **사용 안 함**. T_CO_nominal (T_FO·T_BF(θ)·T_EB·T_CE)
로 frame 그대로 push. Fusion 전 SerialRegistration 이 frame-to-frame 정렬 정밀화.

이유:
- Artec SDK 가 mesh 기반 ICP 를 자체 구현 (geometry + optional texture 기준)
- 코드 중복 회피
- Hand-eye 잔차 3.55mm 가 SerialRegistration 의 수렴 영역 안

### 5.2 Phase 2 frontier — fast_fusion 임시 mesh 사용

매 NBV step 에서 `ArtecClient.fast_fusion(model)` 호출 → 임시 composite mesh
→ frontier 추출. PoissonFusion 보다 빠르고 boundary 검출에는 충분.

### 5.3 Cleaning 단계 — skip

Studio 의 Eraser (수동 GUI 도구) 는 자동화 파이프라인에 부적합. Outliers Removal
+ Small Objects Filter 가 자동 cleaning 을 대체.

### 5.4 CompositeMesh 접근 — final_vertices/final_faces

별도 CompositeMeshHandle 미노출. `ModelHandle.final_vertices()` /
`.final_faces()` / `.has_final_mesh()` 로 접근. fusion 후만 유효.

---

## 6. 사용 예 (예정)

```python
from mms.system import MMS, MMSConfig, ArtecProcessSettings
from mms.sensor.artec.artec_client import ArtecConfig
from mms.nbv.artec_scan_session import ArtecScanSessionSettings
from mms.robot.xarm_interface import XArmInterface
from mms.turntable import Turntable

cfg = MMSConfig(
    artec=ArtecConfig(serial_number=None, capture_texture=True),
    sensor_frames_yaml="config/sensor_frames.yaml",
    T_EC_key="T_EC_artec",
    turntable_frame_yaml="config/calibration/turntable_frame.yaml",
)

scan_s = ArtecScanSessionSettings(
    phase1_enabled=True,
    phase1_theta_step_deg=15.0,
    phase2_enabled=True,
    phase2_K_max=10,
    distance_m=0.260,
    confirm_each_move=False,
)
process_s = ArtecProcessSettings(
    scan_settings=scan_s,
    do_serial_registration=True,
    do_global_registration=True,
    fusion="poisson",
    do_outliers_removal=True,
    do_small_objects_filter=True,
    do_simplify=True,
    do_texturize=True,
    export_obj_path="output/artec_scan.obj",
    export_sproj_path="output/artec_scan.sproj",
)

robot = XArmInterface("192.168.1.210")
robot.go_home(sensor="artec", confirm=True)
turntable = Turntable(...)  # 연결/서보온

with MMS(cfg) as mms:
    result = mms.artec_process(robot, turntable, settings=process_s)
    composite_o3d = result.composite_mesh_o3d()    # Open3D TriangleMesh (m 단위)
    # 시각화 / 추가 처리 ...

robot.disconnect()
turntable.disconnect()
```

---

## 7. 신규 / 수정 파일 (2026-04-29)

```
NEW   mms/nbv/artec_scan_session.py       — ArtecScanSession + ArtecScanContext
EDIT  mms/system.py                       — MMS.artec_process()
                                            + ArtecProcessSettings / ArtecProcessResult
EDIT  mms/sensor/artec/artec_base_binding.cpp
       (+ create_scan, scan_add_frame, model_add_scan)
EDIT  mms/sensor/artec/artec_base.py
       (+ create_scan(), ScanHandle.add_frame(), ModelHandle.add_scan())

KEEP  mms/core/frames.py, stream.py        — PhoXi 전용 그대로
KEEP  mms/nbv/scan_session.py              — PhoXi 전용 그대로
KEEP  mms/sensor/phoxi/                    — reference / dataset replay 그대로
```

---

## 8. 미완 / 향후

| 항목 | 우선순위 | 메모 |
|---|---|---|
| `main.py` 의 ArtecConfig 분기 + artec_process 호출 예시 | 높 | 실제 라이브 검증 |
| Phase 2 ICP refinement 옵션 | 중 | 현재는 SDK SerialReg 만. 보강 필요시 추가 |
| `ArtecScanContext` save/load (json + .sproj) | 중 | 디버그 / replay 용 |
| `ArtecScanSession` interactive mode | 낮 | hand-eye 처럼 manual move + Enter 캡처 |
| Phase 2 cost 계수 튜닝 | 중 | 실제 데이터로 a/b/d 조정 |
| ScanHandle vs 다중 ScanHandle | 낮 | 현재는 1개. 여러 세션 합치고 싶으면 GlobalReg 가 처리 |

---

## 9. 한 줄 요약

PhoXi 의 `Frame`/`Stream` 은 PhoXi 전용으로 보존, **Artec 은 SDK 네이티브 자료구조**
(IScan/IModel) 기반 별도 파이프라인. `MMS.artec_process()` 가 SDK General Pipeline §3
순서 (SerialReg → GlobalReg → Cleaning → Fusion → Simplify → Texturize) 로 실행 →
composite mesh 출력.

---

## 부록 A — Artec SDK 데이터 계층 (참고)

Artec SDK 2.0 의 3-tier 계층:

```
IFrame              — 스캐너 단일 캡처 (raw 2D depth + texture)
   ↓ IFrameProcessor.reconstructTexturizedMesh()
IFrameMesh          — 단일 시점 3D mesh patch (vertices + faces + uv + texture)
   ↓ IScan.add(IFrameMesh)
IScan               — 한 연속 스캔의 frame 시퀀스
   ↓ IModel.add(IScan)
IModel              — 전체 프로젝트 (여러 IScan 묶음)
   ↓ Algorithms (SerialReg → ... → Fusion)
ICompositeMesh      — 융합된 최종 mesh (IModel.getCompositeContainer() 안)
```

본 프로젝트의 `mms/sensor/artec/artec_base.py` 가 이 4 인터페이스를 각각
`FrameMeshHandle`, `ScanHandle`, `ModelHandle` (CompositeMesh 는
`ModelHandle.final_vertices/faces`) 로 wrap.

---

## 부록 B — Artec SDK General Pipeline (참고, 표준 순서)

SDK Algorithm API 의 권장 호출 순서. `MMS.artec_process()` 는 이 순서를 그대로 따름.
`mms/sensor/artec/artec_algorithm.py::Algorithms.run_pipeline()` 도 동일 순서로 wrap.

| 순서 | 알고리즘 | 작용 단위 | 목적 |
|---|---|---|---|
| 1 | SerialRegistration | IFrameMesh (frame-to-frame) | 인접 프레임 ICP 정렬 |
| 2 | GlobalRegistration | IScan / IModel 전체 | 전역 좌표계 최적화 |
| 3a | OutliersRemoval | IFrameMesh per-frame | 통계적 outlier 제거 |
| 3b | SmallObjectsFilter | IFrameMesh per-frame | 작은 cluster 제거 |
| 4 | Fast/PoissonFusion | 정리된 frames 묶음 → ICompositeMesh | 표면 재구성 |
| 5 | MeshSimplification | ICompositeMesh | polycount 감소 |
| 6 | Texturization | ICompositeMesh + frame textures | UV/atlas + texture baking |

> **주의**: Studio GUI 의 사용자 워크플로우 ("Scanning → Cleaning → Alignment →
> Registration → Fusion → Postprocessing") 는 **GUI 라벨 순서지 알고리즘 호출
> 순서가 아니다.** Studio "Cleaning" = 수동 Eraser, "Postprocessing" = fusion 후
> 사용자에게 보여주는 단계 묶음. 코드에서는 위의 §B 순서를 따라야 함.
