# mms/sensor/artec/CMakeLists.txt
#
# pybind11 빌드: Artec 3D Scanning SDK → artec_sdk_py.pyd
#
# 사용법 (PowerShell)
# -------------------
#   cd mms/sensor/artec
#   cmake -B build -G "Visual Studio 18 2026" -A x64
#   cmake --build build --config Release
#
# 출력: build/Release/artec_sdk_py.pyd  (Python 확장 모듈)
# 이후: artec_sdk_py.pyd을 이 디렉터리(artec/)에 복사하거나
#       OUTPUT_DIRECTORY를 직접 이 디렉터리로 지정.

cmake_minimum_required(VERSION 3.18)
project(artec_sdk_py LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ── Python 경로 ────────────────────────────────────────────────────────
find_package(Python 3.8 REQUIRED COMPONENTS Interpreter Development.Module)

# ── pybind11 (pip install pybind11 으로 설치된 경우) ────────────────────
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

# ── pybind11 모듈 ──────────────────────────────────────────────────────
pybind11_add_module(artec_sdk_py MODULE artec_binding.cpp)

target_include_directories(artec_sdk_py PRIVATE "${ARTEC_INCLUDE}")

target_link_libraries(artec_sdk_py PRIVATE
    "${ARTEC_LIB_DIR}/artec-sdk-base.lib"
    "${ARTEC_LIB_DIR}/artec-sdk-capturing.lib"
)

# MSVC: treat source files as UTF-8
if(MSVC)
    target_compile_options(artec_sdk_py PRIVATE /utf-8)
endif()

# Windows: DLL 검색을 위해 ARTEC_LIB_DIR 을 빌드 시 경로에 추가
if(WIN32)
    set_target_properties(artec_sdk_py PROPERTIES
        VS_DEBUGGER_ENVIRONMENT "PATH=${ARTEC_LIB_DIR};$ENV{PATH}"
    )
endif()

# ── 출력 위치: 이 CMakeLists.txt 가 있는 디렉터리 ──────────────────────
# Release / Debug 서브 폴더 없이 직접 artec/ 아래에 배치
set_target_properties(artec_sdk_py PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY         "${CMAKE_CURRENT_SOURCE_DIR}"
    LIBRARY_OUTPUT_DIRECTORY_RELEASE "${CMAKE_CURRENT_SOURCE_DIR}"
    LIBRARY_OUTPUT_DIRECTORY_DEBUG   "${CMAKE_CURRENT_SOURCE_DIR}"
    RUNTIME_OUTPUT_DIRECTORY         "${CMAKE_CURRENT_SOURCE_DIR}"
    RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CMAKE_CURRENT_SOURCE_DIR}"
    RUNTIME_OUTPUT_DIRECTORY_DEBUG   "${CMAKE_CURRENT_SOURCE_DIR}"
)
