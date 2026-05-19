| **래퍼** | **메서드** | **반환 타입 / shape** | **내부 SDK 타입(숨김)** | **1차/2차** | **이유** |
| --- | --- | --- | --- | --- | --- |
| `Algorithms` | `is_available()` | `bool` | `checkAlgorithmsPermission()` | 1차 | 현재 머신에서 알고리즘 사용 가능 여부 확인. |
|  | `serial_registration(model, settings)` | `ModelHandle` | `createSerialRegistrationAlgorithm()` + `IAlgorithm` + `AlgorithmWorkset` | 1차 | 가장 기본적인 후처리 시작점. |
|  | `global_registration(model, settings)` | `ModelHandle` | `createGlobalRegistrationAlgorithm()` + `IAlgorithm` + `AlgorithmWorkset` | 1차 | SDK 2.0 기준 핵심 정합 단계. |
|  | `outliers_removal(model, settings)` | `ModelHandle` | `createOutliersRemovalAlgorithm()` + `AlgorithmWorkset` | 1차 | 노이즈 제거. |
|  | `small_objects_filter(model, settings)` | `ModelHandle` | `createSmallObjectsFilterAlgorithm()` + `AlgorithmWorkset` | 1차 | 작은 객체 제거. |
|  | `fast_fusion(model, settings)` | `ModelHandle` | `createFastFusionAlgorithm()` + `AlgorithmWorkset` | 1차 | 빠른 메시 생성. |
|  | `poisson_fusion(model, settings)` | `ModelHandle` | `createPoissonFusionAlgorithm()` + `AlgorithmWorkset` | 1차 | Sharp/Smooth fusion. |
|  | `mesh_simplify(model, settings)` | `ModelHandle` | `createMeshSimplificationAlgorithm()` + `AlgorithmWorkset` | 1차 | 고품질 단순화. |
|  | `fast_mesh_simplify(model, settings)` | `ModelHandle` | `createFastMeshSimplificationAlgorithm()` + `AlgorithmWorkset` | 2차 | 빠른 단순화. |
|  | `texturize(model, settings)` | `ModelHandle` | `createTexturizationAlgorithm()` + `AlgorithmWorkset` | 1차 | 최종 텍스처 생성. |
|  | `auto_align(model, settings)` | `ModelHandle` | `createAutoalignAlgorithm()` + `AlgorithmWorkset` | 2차 | 다중 scan 조립 자동화. |
|  | `loop_closure(model, settings)` | `ModelHandle` | `createLoopClosureAlgorithm()` + `AlgorithmWorkset` | 2차 | SDK 2.0에선 보통 생략 가능. |
|  | `run_pipeline(model, pipeline_settings)` | `ModelHandle` | 여러 `IAlgorithm` + `AlgorithmWorkset` 연쇄 | 1차 | 자주 쓰는 순서를 Python에서 한 번에 실행. |
| `SerialRegistrationSettingsDTO` | `registration_type` | `SerialRegistrationType` enum | `SerialRegistrationSettings` | 1차 | Rough/Fine + textured 여부 지정. |
|  | `scanner_type` | `ScannerType` | `SerialRegistrationSettings.scannerType` | 1차 | scanner preset 반영. |
| `GlobalRegistrationSettingsDTO` | `registration_type` | `GlobalRegistrationType` enum | `GlobalRegistrationSettings` | 1차 | geometry-only 또는 geometry+texture 선택. |
|  | `scanner_type` | `ScannerType` | `GlobalRegistrationSettings.scannerType` | 1차 | scanner preset 반영. |
| `OutliersRemovalSettingsDTO` | 각 파라미터 | 숫자/불리언 | `OutliersRemovalSettings` | 2차 | 노이즈 제거 세부 조정. |
| `SmallObjectsFilterSettingsDTO` | `filter_type`, `filter_threshold` | enum, int | `SmallObjectsFilterSettings` | 1차 | 가장 큰 객체만 남기거나 polygon 수 threshold 적용. |
| `FastFusionSettingsDTO` | 주요 파라미터 | 숫자/불리언 | `FastFusionSettings` | 1차 | 빠른 메시 생성용 설정. |
| `PoissonFusionSettingsDTO` | `fusion_type`, `fill_holes_type`, `max_hole_radius`, 기타 | enum, float, bool | `PoissonFusionSettings` | 1차 | watertight/precise fusion 조정. |
| `MeshSimplificationSettingsDTO` | `simplify_type`, `metric`, `target`, `error`, `angle_threshold` 등 | enum, int, float | `MeshSimplificationSettings` | 1차 | triangle 수/정확도 중심 단순화. |
| `FastMeshSimplificationSettingsDTO` | 주요 파라미터 | 숫자/불리언 | `FastMeshSimplificationSettings` | 2차 | 빠른 triangle reduction. |
| `TexturizationSettingsDTO` | `texturize_type`, `resolution`, `input_filter` 등 | enum | `TexturizationSettings` | 1차 | atlas/advanced, 해상도, texture frame 선택. |
| `PipelineSettingsDTO` | `do_serial_registration`, `do_global_registration`, `do_outliers_removal`, `do_fusion`, `do_simplify`, `do_texturize`, 각 하위 설정 | bool + DTO 묶음 | 여러 settings DTO 조합 | 1차 | Python에서 자주 쓰는 전체 후처리 파이프라인 정의. |

## **Python enum 매핑 표**

| **Python enum** | **SDK enum 출처** | **값** |
| --- | --- | --- |
| `SerialRegistrationType` | `SerialRegistrationType` | `ROUGH`, `ROUGH_TEXTURED`, `FINE`, `FINE_TEXTURED` |
| `GlobalRegistrationType` | `GlobalRegistrationType` | `GEOMETRY`, `GEOMETRY_AND_TEXTURE` |
| `PoissonFusionType` | `PoissonFusionType` | `SHARP`, `SMOOTH` |
| `FillHolesType` | `FillHolesType` | `ALL`, `BY_RADIUS` |
| `SmallObjectsFilterType` | `SmallObjectsFilterType` | `LEAVE_BIGGEST_OBJECT`, `FILTER_BY_THRESHOLD` |
| `SimplifyType` | `SimplifyType` | `TRIANGLE_QUANTITY`, `ACCURACY`, `REMESH`, `TRIANGLE_QUANTITY_FAST` |
| `SimplifyMetric` | `SimplifyMetric` | `EDGE_LENGTH`, `EDGE_LENGTH_AND_ANGLE`, `DISTANCE_TO_SURFACE`, `DISTANCE_TO_SURFACE_ITERATIVE` |
| `TexturizeType` | `TexturizeType` | `ADVANCED`, `ATLAS`, `KEEP_ATLAS`, `VERTEX_COLOR_TO_ATLAS` |
| `TexturizeResolution` | `TexturizeResolution` | `R512`, `R1024`, `R2048`, `R4096`, `R8192`, `R16384` |
| `InputFilter` | `InputFilter` | `USE_TEXTURE_KEYFRAMES`, `USE_ALL_TEXTURES` |

## **shape 관련 메모**

Algorithm API 자체는 보통 **numpy 배열을 직접 반환하지 않습니다.** 대부분 입력도 `ModelHandle`, 출력도 `ModelHandle`로 보는 것이 맞습니다. 알고리즘은 `AlgorithmWorkset` 안의 `IModel` 구조를 변환하는 역할이기 때문입니다. geometry array shape `(N,3)`나 face shape `(M,3)`는 **알고리즘 결과로 나온 `ModelHandle` 또는 `FrameMeshHandle`의 Base API 메서드**인 `final_vertices()`, `final_faces()`, `vertices()`, `faces()`에서 꺼내는 구조가 자연스럽습니다.

즉 이 레이어의 shape 표기는 이렇게 이해하면 됩니다.

| **대상** | **반환 shape** |
| --- | --- |
| `Algorithms.*(...)` | 직접 shape 없음, `ModelHandle` 반환 |
| `result_model.final_vertices()` | `(N, 3)`, `float32` |
| `result_model.final_faces()` | `(M, 3)`, `int32` |