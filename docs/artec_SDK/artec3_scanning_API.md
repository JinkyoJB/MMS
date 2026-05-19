### 3-2. **Python 래퍼 확정안 (내부 SDK + shape 포함)**

**1) Scanning API Python 래퍼 메서드 정의**

| **래퍼** | **메서드** | **반환 타입 / shape** | **내부 SDK 타입(숨김)** | **1차/2차** | **이유** |
| --- | --- | --- | --- | --- | --- |
| `ScanSessionSettings` | `default()` | `ScanSessionSettings` | `ScanningProcedureSettings` + `initializeScanningProcedureSettings()` | 1차 | 기본 설정 생성. |
|  | `set_registration_type(v)` | `None` | `ScanningProcedureSettings.registrationType` | 1차 | ICP/Hybrid/Texture 선택. |
|  | `set_pipeline(flags)` | `None` | `ScanningProcedureSettings.pipelineConfiguration` | 1차 | 스캔 파이프라인 구성. |
|  | `set_initial_state(state)` | `None` | `ScanningProcedureSettings.initialState` | 1차 | Preview/Record 초기 상태 지정. |
|  | `set_max_frame_count(n)` | `None` | `ScanningProcedureSettings.maxFrameCount` | 1차 | 프레임 수 제한. |
|  | `set_capture_texture(method, frequency=None)` | `None` | `ScanningProcedureSettings.captureTexture`, `captureTextureFrequency` | 1차 | 텍스처 캡처 정책 지정. |
|  | `set_ignore_registration_errors(v)` | `None` | `ScanningProcedureSettings.ignoreRegistrationErrors` | 1차 | 등록 실패 프레임 처리 정책. |
|  | `set_save_empty_surfaces(v)` | `None` | `ScanningProcedureSettings.saveEmptySurfaces` | 2차 | 빈 surface 유지 여부. |
| `ScanSession` | `create(scanner, settings=None)` | `ScanSession` | `createScanningProcedure()` + `TRef<IScanningProcedure>` | 1차 | 단일 스캐너 스캔 세션 생성. |
|  | `start_preview()` | `None` | `IScanningProcedure::setState(Preview)` + `IJob` 실행 | 1차 | scan에는 넣지 않고 현재 입력만 보기. |
|  | `start_record()` | `None` | `IScanningProcedure::setState(Record)` + `IJob` 실행 | 1차 | 실제 스캔 기록 시작. |
|  | `stop()` | `ModelHandle` 또는 `None` | `IScanningProcedure::setState(Stop)` + 결과 `IModel` | 1차 | 스캔 종료, 결과 모델 회수. |
|  | `state()` | `ScanningState` | `IScanningProcedure::getState()` | 1차 | 현재 세션 상태 조회. |
|  | `set_state(state)` | `None` | `IScanningProcedure::setState()` | 1차 | Preview/Record/Stop 전환. |
|  | `set_sensitivity(v)` | `None` | `IScanningProcedure::setSensitivity()` | 1차 | mesh reconstruction sensitivity 조정. |
|  | `sensitivity()` | `float` | `IScanningProcedure::getSensitivity()` | 1차 | 현재 감도 조회. |
|  | `set_scanning_range(near, far)` | `None` | `IScanningProcedure::setScanningRange()` | 1차 | 근/원거리 범위 제한. |
|  | `scanning_range()` | `tuple[float, float]` | `IScanningProcedure::getScanningRange()` | 1차 | 현재 스캔 range 조회. |
|  | `set_roi(x, y, w, h)` | `None` | `IScanningProcedure::setROI(RectF*)` | 2차 | 화면 ROI 지정. |
|  | `clear_roi()` | `None` | `IScanningProcedure::setROI(NULL)` | 2차 | ROI 비활성화. |
|  | `roi()` | `tuple[float, float, float, float] \| None` | `IScanningProcedure::getROI(RectF*)` | 2차 | ROI 조회. |
|  | `set_frame_callback(fn)` | `None` | `IScanningProcedureObserver` / `ScanningProcedureSettings.scanningCallback` | 1차 | 실시간 프레임 이벤트 연결. |
|  | `set_state_callback(fn)` | `None` | `IScanningProcedureObserver` | 2차 | 상태 변화 이벤트 연결. |
| `FrameEvent` | `frame_state` | `FrameState` enum | `RegistrationInfo.frameState` | 1차 | 프레임 처리 결과. |
|  | `frame_mesh` | `FrameMeshHandle \| None` | `RegistrationInfo.frame` → `IFrameMesh` | 1차 | 성공 시 생성된 frame mesh. |
|  | `scan_index` | `int` 또는 `None` | observer 전달 정보 기반 | 2차 | 어느 scan에 속했는지 추적. |
|  | `frame_index` | `int` 또는 `None` | observer 전달 정보 기반 | 2차 | 프레임 번호 추적. |
| `BundleSession` | `create(scanners, settings=None)` | `BundleSession` | `createScanningProcedureBundle()` + `IScanningProcedureBundle` | 2차 | 멀티 스캐너 bundle 세션. |

**2) Python enum/DTO 정의**

| **Python 타입** | **값 / 필드** | **내부 SDK 출처** |
| --- | --- | --- |
| `ScanningState` | `PREVIEW`, `RECORD`, `STOP` (`CONTINUE_RECORD`는 비권장) | `ScanningState_*` |
| `RegistrationType` | `ICP`, `HYBRID`, `TEXTURE` | `RegistrationAlgorithmType_*` |
| `CaptureTextureMethod` | `NONE`, `EVERY_N`, `ON_TEXTURE_KEYFRAME`, `ALWAYS` | `CaptureTextureMethod_*` |
| `FrameState` | `OK`, `TRIGGER_CAPTURE_FAILED`, `CAPTURE_FAILED`, `RECONSTRUCTION_FAILED`, `REGISTRATION_FAILED`, `TEXTURE_MAPPING_FAILED`, `ADD_TO_SCAN_FAILED` | `FrameState_*` |
| `ScanningPipelineFlags` | `CAPTURE_ONLY`, `CONVERT_TEXTURES`, `CALCULATE_NORMALS`, `MAP_TEXTURE`, `REGISTER_FRAME`, `FIND_GEOMETRY_KEYFRAME`, `FAST_CAPTURE` | `ScanningPipeline_*` |

### 3-3. 숨길 것

- raw `IScanningProcedure`, `ScanningProcedureSettings`, `IScanningProcedureObserver`는 Python에 직접 노출하지 않음.
- C++에서만 `TRef<IScanningProcedure>`로 소유하고, Python에는 `ScanSession`만 노출.