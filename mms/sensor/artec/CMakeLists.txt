# mms/sensor/artec/CMakeLists.txt
#
# pybind11 빌드: Artec 3D Scanning SDK → 6개 .pyd
#
# 모듈 구분
# ---------
#   artec_sdk_py       : High-level Capturing API (ArtecScanner, CaptureResult)
#                        소스: artec_binding.cpp
#   artec_base_py      : Base API (ModelHandle, ScanHandle, FrameMeshHandle, DTOs)
#                        + 캡슐 기반 캡처 헬퍼 (capture_frame_handle, capture_to_model)
#                        소스: artec_base_binding.cpp
#   artec_capturing_py : Fine-grained Capturing API (ScannerManager, ScannerHandle,
#                        CapturedFrameHandle, FrameProcessorHandle, DTOs)
#                        소스: artec_capturing_binding.cpp
#   artec_scanning_py  : Scanning API (ScanSession, ScanSessionSettings, FrameEvent, DTOs)
#                        소스: artec_scanning_binding.cpp
#   artec_algorithm_py : Algorithm API (Registration, Fusion, Simplification, Texturization)
#                        소스: artec_algorithm_binding.cpp
#   artec_project_py   : Project API (ProjectManager, ProjectHandle, Load/Save, Entries)
#                        소스: artec_project_binding.cpp
#
# 공통 유틸리티: artec_common.h (두 모듈이 헤더로 공유)
#
# 사용법 (PowerShell)
# -------------------
#   cd mms/sensor/artec
#   $cmake = "...\CMake\bin\cmake.exe"   # Visual Studio 18 번들 CMake
#   & $cmake -B build -G "Visual Studio 18 2026" -A x64
#   & $cmake --build build --config Release
#
# 출력 위치: 이 CMakeLists.txt 가 있는 디렉터리 (artec/) 아래에 직접 배치

cmake_minimum_required(VERSION 3.18)
project(artec_bindings LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ── Python ─────────────────────────────────────────────────────────────
find_package(Python 3.8 REQUIRED COMPONENTS Interpreter Development.Module)

# ── pybind11 ───────────────────────────────────────────────────────────
execute_process(
    COMMAND "${Python_EXECUTABLE}" -c "import pybind11; print(pybind11.get_cmake_dir())"
    OUTPUT_VARIABLE _pybind11_cmake_dir
    OUTPUT_STRIP_TRAILING_WHITESPACE
    RESULT_VARIABLE _pybind11_result
)
if(NOT _pybind11_result EQUAL 0)
    message(FATAL_ERROR
        "pybind11를 찾을 수 없습니다.\n"
        "  pip install pybind11  후 다시 시도하세요.")
endif()
list(APPEND CMAKE_PREFIX_PATH "${_pybind11_cmake_dir}")
find_package(pybind11 REQUIRED)

# ── Artec SDK 경로 ─────────────────────────────────────────────────────
set(ARTEC_SDK_ROOT "C:/Program Files/Artec/Artec 3D Scanning SDK"
    CACHE PATH "Artec 3D Scanning SDK 설치 경로")

set(ARTEC_INCLUDE "${ARTEC_SDK_ROOT}/include")
set(ARTEC_LIB_DIR "${ARTEC_SDK_ROOT}/bin-x64")

if(NOT EXISTS "${ARTEC_INCLUDE}/artec/sdk/capturing/IScanner.h")
    message(FATAL_ERROR
        "Artec SDK 헤더를 찾을 수 없습니다: ${ARTEC_INCLUDE}\n"
        "ARTEC_SDK_ROOT를 올바른 경로로 설정하세요.")
endif()

# ── 공통 설정 헬퍼 (두 타겟에 동일하게 적용) ──────────────────────────
function(artec_target_setup target_name)
    target_include_directories(${target_name} PRIVATE
        "${ARTEC_INCLUDE}"
        "${CMAKE_CURRENT_SOURCE_DIR}"   # artec_common.h 위치
    )
    target_link_libraries(${target_name} PRIVATE
        "${ARTEC_LIB_DIR}/artec-sdk-base.lib"
        "${ARTEC_LIB_DIR}/artec-sdk-capturing.lib"
    )
    if(MSVC)
        target_compile_options(${target_name} PRIVATE /utf-8)
    endif()
    if(WIN32)
        set_target_properties(${target_name} PROPERTIES
            VS_DEBUGGER_ENVIRONMENT "PATH=${ARTEC_LIB_DIR};$ENV{PATH}"
        )
    endif()
    # 출력을 artec/ 폴더에 직접 배치 (Release/ 서브폴더 없음)
    set_target_properties(${target_name} PROPERTIES
        LIBRARY_OUTPUT_DIRECTORY         "${CMAKE_CURRENT_SOURCE_DIR}"
        LIBRARY_OUTPUT_DIRECTORY_RELEASE "${CMAKE_CURRENT_SOURCE_DIR}"
        LIBRARY_OUTPUT_DIRECTORY_DEBUG   "${CMAKE_CURRENT_SOURCE_DIR}"
        RUNTIME_OUTPUT_DIRECTORY         "${CMAKE_CURRENT_SOURCE_DIR}"
        RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CMAKE_CURRENT_SOURCE_DIR}"
        RUNTIME_OUTPUT_DIRECTORY_DEBUG   "${CMAKE_CURRENT_SOURCE_DIR}"
    )
endfunction()

# ── artec_sdk_py: Capturing API ────────────────────────────────────────
pybind11_add_module(artec_sdk_py MODULE artec_binding.cpp)
artec_target_setup(artec_sdk_py)

# ── artec_base_py: Base API ────────────────────────────────────────────
pybind11_add_module(artec_base_py MODULE artec_base_binding.cpp)
artec_target_setup(artec_base_py)

# ── artec_capturing_py: Fine-grained Capturing API ─────────────────────
pybind11_add_module(artec_capturing_py MODULE artec_capturing_binding.cpp)
artec_target_setup(artec_capturing_py)

# ── artec_scanning_py: Scanning API ────────────────────────────────────
pybind11_add_module(artec_scanning_py MODULE artec_scanning_binding.cpp)
artec_target_setup(artec_scanning_py)
target_link_libraries(artec_scanning_py PRIVATE
    "${ARTEC_LIB_DIR}/artec-sdk-scanning.lib"
)

# ── artec_algorithm_py: Algorithm API ──────────────────────────────────
pybind11_add_module(artec_algorithm_py MODULE artec_algorithm_binding.cpp)
artec_target_setup(artec_algorithm_py)
target_link_libraries(artec_algorithm_py PRIVATE
    "${ARTEC_LIB_DIR}/artec-sdk-algorithms.lib"
)

# ── artec_project_py: Project API ──────────────────────────────────────
pybind11_add_module(artec_project_py MODULE artec_project_binding.cpp)
artec_target_setup(artec_project_py)
target_link_libraries(artec_project_py PRIVATE
    "${ARTEC_LIB_DIR}/artec-sdk-project.lib"
)
