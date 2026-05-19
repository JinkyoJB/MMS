# Artec Sensor Module

## 구조

```
mms/sensor/artec/
├── CMakeLists.txt        # pybind11 빌드 설정
├── artec_binding.cpp     # C++ → Python 바인딩 (pybind11)
├── artec_client.py       # Python 클라이언트 (ArtecClient, ArtecConfig)
└── artec_sdk_py.pyd      # 빌드 산출물 (git 미포함, 빌드 후 생성됨)
```

---

## 왜 빌드가 필요한가

Artec 3D Scanning SDK는 C++ 라이브러리다. Python에서 직접 호출할 수 없기 때문에
**pybind11**을 사용해 C++ 코드를 Python 확장 모듈(`.pyd`)로 컴파일한다.

```
Artec SDK (C++ DLL)
    ↕  C++ 헤더 + .lib
artec_binding.cpp  ── pybind11 ──→  artec_sdk_py.pyd
                                         ↕  import
                                    artec_client.py
```

`.pyd` 파일은 Windows DLL과 동일한 포맷의 Python 확장 모듈이다.
런타임에 `import artec_sdk_py`로 로드되며, Artec SDK의 DLL들
(`artec-sdk-base.dll`, `artec-sdk-capturing.dll`)을 동적으로 참조한다.

---

## 빌드 방법

### 사전 요구사항

| 항목 | 내용 |
|------|------|
| Artec 3D Scanning SDK | `C:\Program Files\Artec\Artec 3D Scanning SDK\` |
| Visual Studio 18 2026 | `C:\Program Files\Microsoft Visual Studio\18\Professional\` |
| CMake | VS 번들 CMake 사용 (`...\CMake\bin\cmake.exe`) |
| pybind11 | `pip install pybind11` |

> **CMake 경로 주의:** Visual Studio 18 (2026)에 번들된 CMake를 사용해야 한다.
> 시스템 PATH에 없으면 절대 경로로 직접 호출한다:
> ```
> $cmake = "C:\Program Files\Microsoft Visual Studio\18\Professional\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe"
> ```

### 빌드 명령 (PowerShell)

```powershell
cd mms\sensor\artec

# CMake 경로 (VS 18 번들)
$cmake = "C:\Program Files\Microsoft Visual Studio\18\Professional\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe"

# 구성 (기존 빌드 캐시가 있으면 rm -rf build 후 재실행)
& $cmake -B build -G "Visual Studio 18 2026" -A x64

# 빌드
& $cmake --build build --config Release
```

빌드 성공 시 `mms/sensor/artec/artec_sdk_py.pyd`가 생성된다.
(`CMakeLists.txt`의 `LIBRARY_OUTPUT_DIRECTORY`가 `artec/`로 직접 설정되어 있어
`build/Release/` 서브폴더 없이 바로 배치된다.)

### CMakeLists.txt 핵심 설정

```cmake
# Python 인터프리터 + Development 헤더 자동 탐색
find_package(Python 3.8 REQUIRED COMPONENTS Interpreter Development.Module)

# pip로 설치된 pybind11의 CMake 디렉터리를 동적으로 찾음
execute_process(
    COMMAND "${Python_EXECUTABLE}" -c "import pybind11; print(pybind11.get_cmake_dir())"
    OUTPUT_VARIABLE _pybind11_cmake_dir ...)
find_package(pybind11 REQUIRED)

# Artec SDK 헤더 + import library 연결
target_include_directories(artec_sdk_py PRIVATE "${ARTEC_INCLUDE}")
target_link_libraries(artec_sdk_py PRIVATE
    "${ARTEC_LIB_DIR}/artec-sdk-base.lib"
    "${ARTEC_LIB_DIR}/artec-sdk-capturing.lib"
)

# MSVC: 소스 파일을 UTF-8로 해석 (한국어 주석 포함 시 필수)
target_compile_options(artec_sdk_py PRIVATE /utf-8)

# 출력을 artec/ 폴더에 직접 배치 (Release/ 서브폴더 없음)
set_target_properties(artec_sdk_py PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY_RELEASE "${CMAKE_CURRENT_SOURCE_DIR}"
    RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CMAKE_CURRENT_SOURCE_DIR}")
```

> **재빌드 주의:** CMake 구성을 변경하면 `build/` 디렉터리를 삭제하고 다시 구성해야 한다.
> 기존 `CMakeCache.txt`에 잘못된 Generator 정보가 남아 있으면 구성 오류가 발생한다.

---

## 현재 래핑된 Artec SDK API

### 모듈 함수

| Python | C++ SDK | 설명 |
|--------|---------|------|
| `enumerate_scanners()` | `cap::enumerateScanners()` | 연결된 스캐너 목록 반환. 각 항목: `{index, serial, name, license, has_texture_camera}` |

### `CaptureResult` 클래스

`ArtecScanner.capture()`의 반환값. `IFrameMesh`에서 변환된 numpy 배열들.

| 필드 | 타입 | 내용 |
|------|------|------|
| `points` | `(N, 3) float32` | 버텍스 좌표, mm, 센서(S) 프레임 |
| `triangles` | `(M, 3) int32` | 삼각형 면 인덱스 |
| `normals` | `(N, 3) float32` | 스무스 법선. 없으면 shape `(0, 3)` |
| `texture_image` | `(H, W, 3) uint8` | RGB 텍스처 이미지. 없으면 shape `(0, 0, 3)` |
| `tex_width` | `int` | 텍스처 폭 (px) |
| `tex_height` | `int` | 텍스처 높이 (px) |

### `ArtecScanner` 클래스

| Python 메서드 | C++ SDK | 설명 |
|--------------|---------|------|
| `__init__(serial="")` | — | 시리얼 번호 지정. `""` → 첫 번째 발견 스캐너 |
| `initialize()` | `enumerateScanners` + `createScanner` + `createFrameProcessor` | 스캐너 연결 및 FrameProcessor 초기화 |
| `shutdown()` | `TRef::release()` | 스캐너 연결 해제 및 리소스 해방 |
| `capture(capture_texture=True)` | `IScanner::capture` + `reconstructMesh` / `reconstructAndTexturizeMesh` | 1회 스캔. GIL 해제 후 실행. `CaptureResult` 반환 |
| `get_fps()` | `IScanner::getFPS()` | 현재 FPS 조회 |
| `set_fps(fps)` | `IScanner::setFPS()` | FPS 설정 |
| `get_max_fps()` | `IScanner::getMaximumFPS()` | 최대 FPS 조회 |
| `get_texture_gain()` | `IScanner::getTextureGain()` | 텍스처 게인 조회 |
| `set_texture_gain(gain)` | `IScanner::setTextureGain()` | 텍스처 게인 설정 |
| `get_serial()` | `IScanner::getId()->serial` | 시리얼 번호 문자열 |
| `get_name()` | `IScanner::getId()->name` | 스캐너 이름 문자열 |
| `is_initialized()` | — | 초기화 여부 |

---

## 내부 구현 세부사항

### 데이터 복사 방식

**포인트 / 법선** (`IArrayPoint3F`):
`Point3F` 구조체는 `float x, y, z`가 연속 배치되어 있으므로
`memcpy`로 numpy 버퍼에 직접 복사한다.

```cpp
base::IArrayPoint3F* arr = mesh->getPoints();
int n = arr->getSize();
auto out = py::array_t<float>({(py::ssize_t)n, (py::ssize_t)3});
std::memcpy(out.mutable_data(), arr->getPointer(), n * sizeof(base::Point3F));
```

**삼각형 인덱스** (`IArrayIndexTriplet`):
`IndexTriplet`의 멤버 타입이 `int`(플랫폼 의존)이므로
`int32_t`로 명시적 캐스트하며 원소 단위로 복사한다.

```cpp
const base::IndexTriplet* src = arr->getPointer();
int32_t* dst = out.mutable_data();
for (int i = 0; i < n; ++i) {
    dst[i*3+0] = static_cast<int32_t>(src[i].x);
    dst[i*3+1] = static_cast<int32_t>(src[i].y);
    dst[i*3+2] = static_cast<int32_t>(src[i].z);
}
```

**텍스처 이미지** (`IImage`):
픽셀 포맷(`BGR` / `RGB` / `Mono`)을 확인하고 항상 RGB로 변환한다.
`pitch` (행 바이트 길이)를 고려해 행 단위로 복사한다.

**GIL (Global Interpreter Lock)**:
`capture()` 내부의 블로킹 SDK 호출(`IScanner::capture`, `reconstructMesh`)은
`py::gil_scoped_release`로 GIL을 해제한 상태에서 실행된다.
이를 통해 캡처 중에도 다른 Python 스레드가 실행 가능하다.

### wchar_t 문자열 처리

Artec SDK의 스캐너 이름·시리얼은 `wchar_t*` 타입이다.
Windows API `WideCharToMultiByte(CP_UTF8, ...)`로 UTF-8 `std::string`으로 변환한다.

---

## 새로운 SDK 함수 래핑 방법

다른 Artec SDK 함수(또는 다른 C++ SDK)를 추가로 래핑해야 할 때의 절차:

### 1단계: C++ SDK 헤더 확인

Artec SDK 헤더 위치: `C:\Program Files\Artec\Artec 3D Scanning SDK\include\`

```
artec/sdk/
├── base/
│   ├── IFrameMesh.h      # 메쉬 인터페이스
│   ├── IImage.h          # 이미지 인터페이스
│   ├── IArray.h          # 배열 인터페이스 (IArrayPoint3F 등)
│   ├── Types.h           # 기본 타입 (Point3F, IndexTriplet 등)
│   └── Errors.h          # ErrorCode 열거형
└── capturing/
    ├── IScanner.h        # 스캐너 인터페이스
    ├── IFrame.h          # 캡처 프레임 인터페이스
    ├── IFrameProcessor.h # 메쉬 재구성 인터페이스
    └── IArrayScannerId.h # 스캐너 목록 인터페이스
```

새 헤더가 필요하면 `artec_binding.cpp` 상단에 `#include`를 추가한다.

### 2단계: `artec_binding.cpp`에 C++ 코드 추가

**패턴 A: 단순 값 조회/설정 메서드**

```cpp
// ArtecScanner 클래스 내부에 추가
float get_exposure() const {
    require_init();
    return scanner_->getExposure();  // SDK 메서드 직접 호출
}
void set_exposure(float v) {
    require_init();
    scanner_->setExposure(v);
}
```

**패턴 B: 새로운 데이터 타입 반환 (numpy 변환)**

```cpp
// 예: 깊이 맵을 (H, W) float32 numpy 배열로 반환
py::array_t<float> get_depth_map(base::IDepthMap* dmap) {
    int w = dmap->getWidth();
    int h = dmap->getHeight();
    auto out = py::array_t<float>({(py::ssize_t)h, (py::ssize_t)w});
    const float* src = dmap->getPointer();
    std::memcpy(out.mutable_data(), src, h * w * sizeof(float));
    return out;
}
```

**패턴 C: SDK 에러 코드 처리**

모든 SDK 함수는 `base::ErrorCode`를 반환한다.
기존 `check_ec()` 헬퍼를 사용한다:

```cpp
base::TRef<SomeType> result;
check_ec(scanner_->someFunction(&result, args), "someFunction");
```

**패턴 D: 블로킹 작업 (GIL 해제)**

캡처·재구성처럼 시간이 걸리는 작업은 GIL을 해제한다:

```cpp
{
    py::gil_scoped_release release_gil;
    check_ec(scanner_->longRunningOperation(&out, args), "longRunningOperation");
}
// GIL이 복구된 후 Python 객체 생성
return convert_to_numpy(out);
```

### 3단계: pybind11 모듈 등록

`PYBIND11_MODULE` 블록 안에 새 메서드·클래스를 등록한다:

```cpp
PYBIND11_MODULE(artec_sdk_py, m)
{
    // 기존 코드 ...

    // ArtecScanner에 메서드 추가
    py::class_<ArtecScanner>(m, "ArtecScanner")
        // 기존 def들 ...
        .def("get_exposure", &ArtecScanner::get_exposure)
        .def("set_exposure", &ArtecScanner::set_exposure, py::arg("value"));

    // 새 데이터 클래스 추가
    py::class_<DepthMapResult>(m, "DepthMapResult")
        .def_readwrite("data", &DepthMapResult::data, "(H,W) float32, mm");
}
```

### 4단계: 새 라이브러리가 필요한 경우 CMakeLists.txt 수정

Artec SDK는 기능별로 라이브러리가 분리되어 있다:

```cmake
target_link_libraries(artec_sdk_py PRIVATE
    "${ARTEC_LIB_DIR}/artec-sdk-base.lib"
    "${ARTEC_LIB_DIR}/artec-sdk-capturing.lib"
    # 새 기능이 다른 lib를 필요로 하면 여기에 추가:
    # "${ARTEC_LIB_DIR}/artec-sdk-algorithms.lib"
    # "${ARTEC_LIB_DIR}/artec-sdk-project.lib"
)
```

SDK 설치 경로 `bin-x64/` 아래의 `.lib` 파일 목록을 확인한다.

### 5단계: 재빌드

```powershell
$cmake = "C:\Program Files\Microsoft Visual Studio\18\Professional\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe"
& $cmake --build build --config Release
```

소스 파일(`.cpp`)만 수정했다면 `build/` 삭제 없이 재빌드해도 된다.
CMakeLists.txt를 수정했다면 `rm -rf build` 후 재구성한다.

### 6단계: Python 클라이언트(`artec_client.py`) 업데이트

새 기능을 `ArtecClient`의 메서드로 노출한다:

```python
def get_exposure(self) -> float:
    return self._scanner.get_exposure()

def set_exposure(self, value: float) -> None:
    self._scanner.set_exposure(value)
```

---

## 다른 C++ SDK를 새로 래핑하는 경우

완전히 새로운 C++ SDK(예: 다른 3D 스캐너, 로봇 SDK 등)를 래핑할 때의 체크리스트:

1. **CMakeLists.txt 복사**: `artec/CMakeLists.txt`를 새 폴더에 복사하고
   SDK 경로·라이브러리 이름·모듈 이름을 수정한다.

2. **바인딩 파일 작성**: `artec_binding.cpp` 구조를 참고한다:
   - `check_ec()` 패턴으로 에러 처리
   - `TRef<T>` 대신 SDK의 스마트 포인터 사용
   - `py::array_t<T>` + `memcpy`로 데이터 배열 변환
   - `py::gil_scoped_release`로 블로킹 구간 GIL 해제

3. **DLL 로드**: Python에서 import 전에 SDK DLL 경로를 PATH에 추가해야 한다.
   `artec_client.py`의 `_load_artec_sdk_py()` 함수를 참고:
   ```python
   os.add_dll_directory(str(SDK_BIN_PATH))
   ```

4. **인코딩**: `.cpp` 파일에 한국어 등 비ASCII 문자를 쓰려면
   `target_compile_options(... PRIVATE /utf-8)`이 필수다.
   MSVC 기본 인코딩(CP949)으로는 소스 파일을 잘못 해석해
   예상치 못한 컴파일 오류가 발생한다.
