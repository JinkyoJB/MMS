### 2-2. **Python 래퍼 확정안 (내부 SDK + shape 포함)**

**1) Capturing API Python 래퍼 메서드 정의**

| 래퍼 | 메서드 | 반환 타입 / numpy shape | 내부 SDK 타입(숨김) | 1차/2차 | 이유 |
| --- | --- | --- | --- | --- | --- |
| `ScannerManager` | `list_scanners()` | `list[ScannerIdInfo]` | `enumerateScanners()` + `IArrayScannerId` [docs.artec3d](https://docs.artec3d.com/sdk/2.0/) | 1차 | 연결된 스캐너 목록 조회. |
|  | `open(scanner_id)` | `ScannerHandle` | `createScanner()` + `IScanner` docs.artec3d+1 | 1차 | 선택한 장치 열기. |
| `ScannerHandle` | `id()` | `ScannerIdInfo` | `IScanner::getId()` + `ScannerId` [docs.artec3d](https://docs.artec3d.com/sdk/2.0/classartec_1_1sdk_1_1capturing_1_1_i_scanner.html) | 1차 | 장치 식별 정보. |
|  | `info()` | `ScannerInfoDTO` | `IScanner::getInfo()` + `ScannerInfo` [docs.artec3d](https://docs.artec3d.com/sdk/2.0/classartec_1_1sdk_1_1capturing_1_1_i_scanner.html) | 1차 | 장치 정보 조회. |
|  | `frame_number()` | `int` | `IScanner::getFrameNumber()` [docs.artec3d](https://docs.artec3d.com/sdk/2.0/classartec_1_1sdk_1_1capturing_1_1_i_scanner.html) | 1차 | 현재 프레임 번호 확인. |
|  | `capture(capture_texture=False)` | `CapturedFrameHandle` | `IScanner::capture()` + `IFrame` [docs.artec3d](https://docs.artec3d.com/sdk/2.0/classartec_1_1sdk_1_1capturing_1_1_i_scanner.html) | 1차 | 가장 기본적인 단일 프레임 취득. |
|  | `capture_texture()` | `np.ndarray`, shape `(H, W, C)`, `uint8` 또는 `(H, W)` raw | `IScanner::captureTexture()` + `IImage` [docs.artec3d](https://docs.artec3d.com/sdk/2.0/classartec_1_1sdk_1_1capturing_1_1_i_scanner.html) | 2차 | texture-only 취득. |
|  | `create_frame_processor(settings=None)` | `FrameProcessorHandle` | `IScanner::createFrameProcessor()` + `IFrameProcessor` + `FrameProcessorDesc` docs.artec3d+1 | 1차 | raw frame를 mesh로 변환하기 위한 processor 생성. |
|  | `fps()` | `float` | `IScanner::getFPS()` [docs.artec3d](https://docs.artec3d.com/sdk/2.0/classartec_1_1sdk_1_1capturing_1_1_i_scanner.html) | 1차 | 현재 FPS 조회. |
|  | `set_fps(fps)` | `None` | `IScanner::setFPS()` [docs.artec3d](https://docs.artec3d.com/sdk/2.0/classartec_1_1sdk_1_1capturing_1_1_i_scanner.html) | 1차 | 프레임 속도 조절. |
|  | `max_fps()` | `float` | `IScanner::getMaximumFPS()` [docs.artec3d](https://docs.artec3d.com/sdk/2.0/classartec_1_1sdk_1_1capturing_1_1_i_scanner.html) | 1차 | 장치 최대 FPS 확인. |
|  | `flash_enabled()` | `bool` | `IScanner::isFlashEnabled()` [docs.artec3d](https://docs.artec3d.com/sdk/2.0/classartec_1_1sdk_1_1capturing_1_1_i_scanner.html) | 2차 | flash 상태 조회. |
|  | `enable_flash(enable)` | `None` | `IScanner::enableFlash()` [docs.artec3d](https://docs.artec3d.com/sdk/2.0/classartec_1_1sdk_1_1capturing_1_1_i_scanner.html) | 2차 | geometry flash 제어. |
|  | `texture_flash_enabled()` | `bool` | `IScanner::isTextureFlashEnabled()` [docs.artec3d](https://docs.artec3d.com/sdk/2.0/classartec_1_1sdk_1_1capturing_1_1_i_scanner.html) | 2차 | texture flash 상태 조회. |
|  | `enable_texture_flash(enable)` | `None` | `IScanner::enableTextureFlash()` [docs.artec3d](https://docs.artec3d.com/sdk/2.0/classartec_1_1sdk_1_1capturing_1_1_i_scanner.html) | 2차 | texture flash 제어. |
|  | `texture_gain()` | `float` | `IScanner::getTextureGain()` [docs.artec3d](https://docs.artec3d.com/sdk/2.0/classartec_1_1sdk_1_1capturing_1_1_i_scanner.html) | 2차 | texture gain 조회. |
|  | `set_texture_gain(v)` | `None` | `IScanner::setTextureGain()` [docs.artec3d](https://docs.artec3d.com/sdk/2.0/classartec_1_1sdk_1_1capturing_1_1_i_scanner.html) | 2차 | texture gain 설정. |
|  | `texture_shutter_speed()` | `float` | `IScanner::getTextureShutterSpeed()` [docs.artec3d](https://docs.artec3d.com/sdk/2.0/classartec_1_1sdk_1_1capturing_1_1_i_scanner.html) | 2차 | texture exposure 조회. |
|  | `set_texture_shutter_speed(v)` | `None` | `IScanner::setTextureShutterSpeed()` [docs.artec3d](https://docs.artec3d.com/sdk/2.0/classartec_1_1sdk_1_1capturing_1_1_i_scanner.html) | 2차 | texture exposure 설정. |
|  | `auto_exposure_enabled()` | `bool` | `IScanner::isAutoExposureEnabled()` [docs.artec3d](https://docs.artec3d.com/sdk/2.0/classartec_1_1sdk_1_1capturing_1_1_i_scanner.html) | 2차 | 자동 노출 상태. |
|  | `enable_auto_exposure(enable)` | `None` | `IScanner::enableAutoExposure()` [docs.artec3d](https://docs.artec3d.com/sdk/2.0/classartec_1_1sdk_1_1capturing_1_1_i_scanner.html) | 2차 | 자동 노출 설정. |
|  | `auto_white_balance_enabled()` | `bool` | `IScanner::isAutoWhiteBalanceEnabled()` [docs.artec3d](https://docs.artec3d.com/sdk/2.0/classartec_1_1sdk_1_1capturing_1_1_i_scanner.html) | 2차 | 자동 화이트밸런스 상태. |
|  | `enable_auto_white_balance(enable)` | `None` | `IScanner::enableAutoWhiteBalance()` [docs.artec3d](https://docs.artec3d.com/sdk/2.0/classartec_1_1sdk_1_1capturing_1_1_i_scanner.html) | 2차 | 자동 화이트밸런스 설정. |
|  | `set_hw_trigger(enable)` | `None` | `IScanner::setUseHwTrigger()` [docs.artec3d](https://docs.artec3d.com/sdk/2.0/classartec_1_1sdk_1_1capturing_1_1_i_scanner.html) | 2차 | 외부 동기화 트리거 설정. |
|  | `hw_trigger_enabled()` | `bool` | `IScanner::getUseHwTrigger()` [docs.artec3d](https://docs.artec3d.com/sdk/2.0/classartec_1_1sdk_1_1capturing_1_1_i_scanner.html) | 2차 | 외부 트리거 상태 확인. |
|  | `fire_trigger(capture_texture=False)` | `None` | `IScanner::fireTrigger()` [docs.artec3d](https://docs.artec3d.com/sdk/2.0/classartec_1_1sdk_1_1capturing_1_1_i_scanner.html) | 2차 | 외부 트리거 기반 캡처 시작. |
|  | `retrieve_frame(capture_texture=False)` | `CapturedFrameHandle` | `IScanner::retrieveFrame()` + `IFrame` [docs.artec3d](https://docs.artec3d.com/sdk/2.0/classartec_1_1sdk_1_1capturing_1_1_i_scanner.html) | 2차 | fireTrigger 후 결과 수신. |
| `CapturedFrameHandle` | `frame_number()` | `int` | `IFrame::getFrameNumber()` docs.artec3d+1 | 1차 | frame 식별. |
|  | `has_texture()` | `bool` | `IFrame` 내부 texture/image 존재 여부 docs.artec3d+1 | 1차 | texture 캡처 여부 확인. |
|  | `texture()` | `np.ndarray | None`, shape `(H, W, C)` 또는 raw `(H, W)` | `IFrame` + `IImage` docs.artec3d+1 | 2차 | raw 또는 포함 texture 확인. |
|  | `summary()` | `CapturedFrameSummary` | `IFrame` 메타 기반 | 1차 | raw frame 요약. |
| `FrameProcessorHandle` | `reconstruct(frame)` | `FrameMeshHandle` | `IFrameProcessor::reconstructMesh()` + `IFrameMesh` artec3d+1 | 1차 | raw frame에서 geometry mesh 생성. |
|  | `reconstruct_textured(frame)` | `FrameMeshHandle` | `IFrameProcessor::reconstructAndTexturizeMesh()` + `IFrameMesh` artec3d+1 | 2차 | texturized frame mesh 생성. |
|  | `settings()` | `FrameProcessorSettings` | `FrameProcessorDesc` docs.artec3d+1 | 2차 | processor 설정 확인. |
| `ScannerObserverHandle` | `set_button_callback(fn)` | `None` | `IScannerObserver` / `ScannerObserverBase` docs.artec3d+2 | 2차 | 스캐너 버튼 이벤트 수신. |

**2) DTO 정의**

| DTO | 필드 | 타입 / 의미 | 내부 SDK 출처 |
| --- | --- | --- | --- |
| `ScannerIdInfo` | `serial`, `model`, `name` 등 | 스캐너 식별용 최소 정보 | `ScannerId` [docs.artec3d](https://docs.artec3d.com/sdk/2.0/) |
| `ScannerInfoDTO` | `scanner_type`, `max_fps` 등 | 장치 파라미터 요약 | `ScannerInfo` + `IScanner::getInfo()` [docs.artec3d](https://docs.artec3d.com/sdk/2.0/classartec_1_1sdk_1_1capturing_1_1_i_scanner.html) |
| `CapturedFrameSummary` | `frame_number`, `has_texture` | raw frame 요약 | `IFrame` docs.artec3d+1 |
| `FrameProcessorSettings` | `sensitivity`, `range` 등 | processor 재구성 설정 | `FrameProcessorDesc` docs.artec3d+1 |

### 2-3. 숨길 것

- raw `IScanner`, `IScannerObserver`, `IFrameProcessor` 및 관련 descriptor는 C++ 내부에서만 사용.artec3d+2