# TeachingMesh — 티칭포인트 기반 전체 메쉬 획득

> 작성일: 2026-04-08

---

## 1. 목표

로봇을 사전에 정의된 **티칭포인트** 경로로 이동시키며 PhoXi로 여러 시점의 스캔을 획득하고,
Photoneo 3DMeshing 기능을 활용하여 **고품질 통합 메쉬**를 생성한다.
중간 단계마다 현재까지 획득된 메쉬를 시각화하여 커버리지를 실시간으로 확인한다.

---

## 2. 왜 이 방식이 필요한가

### 2.1 GUI 사용 불가 이유

Photoneo **3DMeshingApp.exe** GUI를 사용하면:
- GUI가 PhoXi 디바이스를 독점 연결 → Python 코드에서 PhoXi 제어 불가
- 로봇 이동 명령(xArm Python API)과 PhoXi 트리거를 Python 코드로 동기화할 수 없음
- 자동화(티칭포인트 순환 → 캡처 → 메쉬) 불가능

### 2.2 Python 재구현이 필요한 이유

```
필요한 것: Python이 로봇 + PhoXi + 메쉬 생성 전체를 제어

GUI 방식:
  3DMeshingApp.exe → PhoXi 독점 → Python 제어 불가 ✗

Python 방식:
  Python → PhoXi (harvesters GenTL)  → capture
  Python → xArm  (xarm-python-sdk)   → 이동
  Python → 3DMeshing API (pybind11)  → 메쉬 생성 ✓
```

---

## 3. Photoneo 3DMeshing API — DLL 구조 상세

설치 경로: `C:\Program Files\Photoneo\3DMeshing\2.3.0\`

전체 디렉터리에는 수십 개의 DLL이 있지만 MMS에서 실질적으로 관계있는 것을 역할별로 분류한다.

---

### 3.1 핵심 API DLL (직접 wrapping 대상)

#### `PhoXi3DMeshing_API_msvc143.dll`
**가장 중요한 DLL.** Python에서 호출하려는 모든 메쉬 처리 기능의 진입점.

링크 lib: `API/lib/Release/PhoXi3DMeshing_API_msvc143.lib`  
헤더: `API/include/PhoXi3DMeshing_API/PhoXi3DMeshingApi.h`

| 기능 그룹 | 함수 | 설명 |
|-----------|------|------|
| **스캔 입력** | `AddScan(PFrame)` | PhoXi Python API의 PFrame 객체를 직접 전달 |
| | `ImportScan(path)` | .ply / .cogs / .praw 파일에서 스캔 로드 |
| | `RemoveScan(id)`, `RemoveAllScans()` | 스캔 삭제 |
| **스캔 처리** | `FilterScans(settings)` | 크로스체크 필터: 여러 시점에서 보이지 않는 점 제거 |
| | `AlignScans(settings)` | Skeletex ICP 정렬: 스캔 간 오차 보정 |
| | `AlignScanSets(base_ids, align_ids)` | PCL registration으로 두 스캔 세트 정렬 |
| | `CropScansWithBoundingBox(Aabb)` | ROI 외부 점 제거 |
| | `TransformScan(mat4, id)` | 수동 변환 적용 |
| **Union PCD** | `GetUnionPointCloud(settings)` | 모든 스캔 → 단일 비정렬 PCD 반환 |
| | `GetColoredUnionPointCloud(settings)` | 색상 포함 버전 |
| | `ExportUnionPointCloud(path)` | .cogs / .ply 파일로 저장 |
| **메쉬 생성** | `CreateMesh(poisson_path, settings)` | PoissonRecon.exe 호출로 표면 재구성 |
| | `TrimMesh(trim_distance)` | Poisson 아티팩트(필요없는 표면) 제거 |
| | `GetMesh()` | `Mesh{positions, normals, faces}` 반환 |
| | `GetColoredMesh()` | 색상 포함 버전 |
| | `ExportMesh(path)` | .cogs / .stl / .fbx / .ply 저장 |
| **디바이스** | `ConnectDevice(device_id)` | PhoXi Control 통해 디바이스 연결 |
| | `SetPrimaryDevice(id)` | 주 스캐너 지정 |
| | `CalibratePrimaryDevice(table)` | 턴테이블 기준 보정 (MMS에서는 불필요) |
| | `LoadDevicesSetup(path)`, `SaveDevicesSetup(path)` | 캘리브레이션 파일 저장/로드 |

주요 데이터 구조:
```cpp
struct PointCloud { positions, normals, intensities };   // (N,3) float
struct ColoredPointCloud { positions, normals, colors };
struct Mesh   { PointCloud points; faces };              // faces: (M,3) uint32
struct Aabb   { glm::vec3 min, max };                   // 단위: mm
```

---

#### `TurnTables_msvc143.dll`
**물리적 턴테이블 제어 모듈.** MMS에서는 로봇이 턴테이블 역할을 대신하므로 직접 사용하지 않는다.  
그러나 `PhoXi3DMeshing_API_msvc143.lib`에 정적 링크되어 있어 런타임 의존성으로 반드시 존재해야 한다.

링크 lib: `API/lib/Release/TurnTables_msvc143.lib`  
헤더: `API/include/TurnTables/`

| 클래스 | 역할 |
|--------|------|
| `Table` | 턴테이블 추상 기반 클래스. `Rotate(angle_deg)` 인터페이스 정의 |
| `TableDriver` | `Rotate()`, `StartRotation()`, `WaitToFinish()` — 비동기 회전 제어 |
| `TableConnectionBuilder` | `Connect(BuiltInTableParams)` — XIMCUSB 포트 연결, `Connect(PluginTableParams)` — 외부 플러그인 연결 |
| `BuiltInTableParams` | `usb_port`, `rotation_speed`, `microsteps_per_rev` (Standa 턴테이블 전용) |
| `RotationParams` | 회전 각도 목록(`rotation_angles`), 딜레이, 타임아웃 |

MMS 활용 가능성: Python 쪽에서 Robot을 `Table`처럼 추상화하여 `CaptureScansAuto()`에 넘길 수 있으나 구현 복잡도가 높아 **현재 미사용**.  
대신 Python에서 로봇 이동 → `AddScan()` 직접 호출 방식 채택.

---

#### `Utils_msvc143.dll`
**내부 유틸리티 모음.** API 전반에서 사용하는 공통 기반 라이브러리.

링크 lib: `API/lib/Release/Utils_msvc143.lib`  
헤더: `API/include/Utils/` (60개 이상의 헤더)

주요 포함 내용:
- `GeometryStructures.h` — `geom::Aabb`, `geom::Axis` 등 기하 구조
- `EigenConversions.h` — Eigen ↔ glm 변환
- `Filesystem.h` — 경로/파일 유틸 (`std_ext::ToStringWithPrecision` 포함)
- `StrongType.h`, `StrongID.h` — 타입 안전 ID (`ConnectedDeviceId` 등)
- `Ulog.h` — 로깅 카테고리 (`ulog::Category`)
- `SerializableJson.h` — JSON 직렬화 (ProcessingSettings.Serialize/Deserialize)
- `Threading.h`, `AtomicQueue.h` — 비동기 처리
- `CameraParams.h`, `IntrinsicParams.h` — 카메라 파라미터

Python wrapping 시 `Ulog` 기반 로그 스트림(`std::ostream`)을 Python `sys.stdout`에 연결하면 로그를 Python 쪽에서 수신 가능.

---

### 3.2 필수 런타임 의존 DLL

3DMeshing_API를 wrapping한 .pyd를 로드할 때 아래 DLL들이 PATH에 있어야 한다.

| DLL | 역할 | MMS 관련성 |
|-----|------|-----------|
| `PhoXi_API_dynamic.dll` | PhoXi 디바이스 연결, `pho::api::PFrame` 타입 정의 | `AddScan(PFrame)` 인터페이스 |
| `PhoXi_API.dll` | PhoXi API 정적 링크 버전 | 같이 필요할 수 있음 |
| `COGS_msvc143.dll` | Photoneo 독자 파일 포맷 `.cogs` 파서 | `ImportScan`, `ExportMesh` |
| `COGS_P_msvc143.dll` | COGS 처리 확장 | 동상 |
| `GEOM_msvc143.dll` | Skeletex 기하 연산: ICP, PCD 처리 | `AlignScans`, `FilterScans` |
| `PhoXiCalib_msvc143.dll` | 카메라 캘리브레이션 알고리즘 | `CalibratePrimaryDevice` |
| `PhoTools_msvc143.dll` | PhoXi 내부 도구 | 스캔 처리 전반 |
| `phoInterprocess2_dynamic_Release.dll` | PhoXiControl ↔ API 프로세스 간 통신 | `ConnectDevice` |
| `PhoLibrary_dynamic_Release.dll` | Photoneo 공통 라이브러리 기반 | 전반 |
| `phoLogger_dynamic_Release.dll` | 로그 시스템 | 전반 |
| `opencv_core3420.dll` 외 7개 | 이미지/PCD 처리 (OpenCV 3.4.20) | 스캔 처리 |
| `cudart64_12.dll` 외 CUDA dlls | GPU 가속 처리 | GPU 사용 시 |

---

### 3.3 GUI 전용 DLL (wrapping 불필요)

아래 DLL들은 **3DMeshingApp.exe GUI 전용**이며 API wrapping에 필요 없다.

```
HIRO_msvc143.dll, HIRO_DRAW_msvc143.dll   ← 3D 렌더링 엔진
GUIP_msvc143.dll                           ← GUI 패널
GLW_msvc143.dll                            ← OpenGL 래퍼
CEGUIBase-0.dll, CEGUIOpenGLRenderer-0.dll 외 ← CEGUI 프레임워크
Qt5Core.dll, Qt5Widgets.dll 외             ← Qt5 GUI
glbinding.dll, glfw3.dll                   ← OpenGL 바인딩
vulkan-1.dll                               ← Vulkan 렌더링
```

---

### 3.4 MMS에서 실제 필요한 DLL 요약

MMS는 로봇이 PhoXi를 들고 이동하는 Eye-in-Hand 구성이므로 턴테이블, GUI 관련 DLL은 불필요.  
pybind11 래퍼 빌드 후 Python이 `import p3dm_py` 할 때 PATH에 있어야 하는 DLL:

```
필수 (직접 기능):
  PhoXi3DMeshing_API_msvc143.dll
  TurnTables_msvc143.dll          ← API에 링크됨, 사용 안 해도 필요
  Utils_msvc143.dll
  COGS_msvc143.dll
  COGS_P_msvc143.dll
  GEOM_msvc143.dll
  PhoTools_msvc143.dll
  PhoXiCalib_msvc143.dll
  PhoXi_API_dynamic.dll
  phoInterprocess2_dynamic_Release.dll
  PhoLibrary_dynamic_Release.dll
  phoLogger_dynamic_Release.dll

필수 (런타임):
  msvcp140.dll, vcruntime140.dll  ← MSVC 2019 재배포 패키지
  concrt140.dll

선택 (GPU 가속):
  cudart64_12.dll 외 CUDA 계열
```

---

### 3.5 `PoissonRecon.exe`의 역할

`CreateMesh(poisson_recon_path, settings)`는 내부에서 `PoissonRecon.exe`를 **subprocess**로 호출한다.  
경로: `C:\Program Files\Photoneo\3DMeshing\2.3.0\PoissonRecon.exe`

```cpp
// Reconstruction 설정
struct Reconstruction {
    uint32_t depth = 9;         // Octree depth (8~11, 높을수록 세밀/느림)
    uint32_t point_weight = 1;  // 0=순수 Poisson, 높을수록 sharp edge
    std::string intermediate_dir = "intermediate";  // 중간 데이터 교환 디렉터리
    UnionCreation union_creation;  // PCD 통합 전처리 옵션
};
```

Python wrapping 후 사용:
```python
POISSON_EXE = r"C:\Program Files\Photoneo\3DMeshing\2.3.0\PoissonRecon.exe"
api.create_mesh(POISSON_EXE)   # 내부적으로 subprocess 호출
```

---

## 4. 처리 파이프라인

```
AddScan() × N  (각 티칭포인트에서 캡처)
  │
  ├─ FilterScans()          크로스체크: 여러 시점에서 보이지 않는 노이즈 제거
  │     settings: consensus_level, cell_size, use_cluster_filtering
  │
  ├─ AlignScans()           Skeletex ICP 정밀 정렬
  │     settings: pa_max_iter_global, pa_ratio_of_samples, ScanRelating 전략
  │
  ├─ CropScansWithBoundingBox(Aabb)   (선택) 물체 ROI 외부 제거
  │
  ├─ CreateMesh(poisson_exe)  PoissonRecon.exe subprocess 호출
  │     settings: depth(9), point_weight(1)
  │
  ├─ TrimMesh()             Poisson 아티팩트 제거
  │
  └─ GetMesh() / ExportMesh(.ply)
```

---

## 5. TeachingMesh 파이프라인 설계

### 5.1 전체 흐름

로봇은 **top, front** 두 포즈에 고정되고, 각 포즈에서 **턴테이블을 5°씩 72회** 회전하며 캡처한다.  
턴테이블 1회전(360°) × 2 로봇 포즈 = 총 144회 스캔.

```
┌─────────────────────────────────────────────────────────┐
│  teaching_mesh.py (메인 제어)                            │
│                                                          │
│  for robot_pose in [top, front]:                         │
│    robot.move_to(robot_pose)      ← xArm 이동            │
│                                                          │
│    for theta in range(0, 360, 5): ← 턴테이블 5°씩 72스텝 │
│      turntable.rotate_to(theta)   ← 턴테이블 이동         │
│      frame = phoxi.capture()      ← PhoXi 캡처           │
│      api.add_scan(frame)          ← 3DMeshing에 추가      │
│      visualize_union_pcd(api)     ← 중간 시각화 (선택)    │
│                                                          │
│  api.filter_scans()                                      │
│  api.align_scans()                                       │
│  api.create_mesh(POISSON_EXE)                            │
│  api.trim_mesh()                                         │
│  mesh = api.get_mesh_as_numpy()                          │
│  visualize_mesh(mesh)                                    │
└─────────────────────────────────────────────────────────┘
```

### 5.2 티칭포인트 정의

`config/teaching_poses.yaml`:
```yaml
# 로봇 티칭 포즈 (degrees)
# 턴테이블이 1회전하는 동안 PhoXi가 바라보는 시점 방향
# top  : 물체 위에서 내려다봄 → 윗면 + 측면 상부 커버리지
# front: 물체 정면 → 측면 전체 + 밑면 일부 커버리지
poses:
  - name: "top"
    joints: [0.0, -30.0, 0.0, 60.0, 0.0, 90.0, 0.0]

  - name: "front"
    joints: [0.0, 10.0, 0.0, 80.0, 0.0, 60.0, 0.0]

# 턴테이블 설정
turntable:
  step_deg: 5          # 1스텝당 회전량 (°)
  total_deg: 360       # 1포즈당 회전 범위 (°)
  settle_ms: 300       # 회전 후 진동 안정화 대기 (ms)
  # → 포즈당 캡처 수: 360 / 5 = 72회
  # → 총 캡처 수: 72 × 2 = 144회
```

**커버리지 전략:**

| 로봇 포즈 | 턴테이블 | 커버 영역 |
|-----------|----------|-----------|
| `top` (위) | 0° → 355° (5°씩 72회) | 윗면, 측면 상부 |
| `front` (정면) | 0° → 355° (5°씩 72회) | 측면 전체, 밑면 일부 |

> 밑면 커버리지가 부족할 경우 `bottom_tilt` 포즈 추가로 보완 가능.

### 5.3 구현 위치

```
mms/
  teaching/
    __init__.py
    teaching_mesh.py          ← 메인 파이프라인 (미구현)
    teaching_poses.py         ← 포즈 로드/검증 (미구현)
  sensor/
    phoxi_meshing.py          ← 단일 프레임 Poisson (구현됨)
    phoxi_instant_meshing.py  ← TSDF 실시간 퓨전, ctypes (구현됨)
    p3dm_wrapper.py           ← 3DMeshing API Python 래퍼 (미구현)
config/
  teaching_poses.yaml         ← 신규
```

### 5.4 단계별 구현 계획

| 단계 | 내용 | 선행 조건 |
|------|------|-----------|
| **1** | `teaching_poses.yaml` 작성 + 로봇으로 검증 | xArm 연결 |
| **2** | pybind11 래퍼 빌드 (아래 섹션 6 참조) | VS2022 + CMake |
| **3** | `p3dm_wrapper.py` — Python에서 API 호출 | 단계 2 |
| **4** | `teaching_mesh.py` — 전체 파이프라인 연동 | 단계 1, 3 |
| **5** | 중간 시각화 (각 포즈 후 Union PCD 표시) | Open3D |
| **6** | Hand-Eye calibration 완료 | Calibration.md 참조 |

> ⚠️ **단계 6 선행 필요**: T_E_S가 정확해야 각 시점의 PCD가 B 프레임에서 올바른 위치에 배치된다.  
> 현재 Hand-Eye calibration 미완성 → `docs/Calibration.md` 참조.

---

## 6. Python 통합: pybind11 바인딩

3DMeshing API는 **C++ 클래스** (이름 맹글링)로 구현되어 있어 ctypes로 직접 호출 불가.  
pybind11로 `.pyd` (Windows Python extension module)를 컴파일하여 `import p3dm_py` 형태로 사용한다.

### 6.1 필요 환경

```
Visual Studio 2022 (MSVC 143)   ← .lib와 동일 컴파일러 필수
CMake 3.20+
pybind11                        pip install pybind11
Python 3.11 (conda 환경)
```

### 6.2 바인딩 코드 (`mms/bindings/p3dm_wrapper.cpp`)

```cpp
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>
#include <sstream>
#include <PhoXi3DMeshing_API/PhoXi3DMeshingApi.h>

namespace py = pybind11;
using namespace p3dm_api;

// Mesh → (verts_np, faces_np) 변환 헬퍼
py::tuple mesh_to_numpy(const Mesh &mesh) {
    size_t nv = mesh.points.positions.size();
    size_t nf = mesh.faces.size();
    auto verts = py::array_t<float>({nv, size_t(3)});
    auto faces = py::array_t<uint32_t>({nf, size_t(3)});
    auto v = verts.mutable_unchecked<2>();
    auto f = faces.mutable_unchecked<2>();
    for (size_t i = 0; i < nv; ++i) {
        v(i,0)=mesh.points.positions[i].x;
        v(i,1)=mesh.points.positions[i].y;
        v(i,2)=mesh.points.positions[i].z;
    }
    for (size_t i = 0; i < nf; ++i) {
        f(i,0)=mesh.faces[i].x;
        f(i,1)=mesh.faces[i].y;
        f(i,2)=mesh.faces[i].z;
    }
    return py::make_tuple(verts, faces);
}

// PointCloud → (positions_np, normals_np) 변환 헬퍼
py::tuple pcd_to_numpy(const PointCloud &pcd) {
    size_t n = pcd.positions.size();
    auto pos = py::array_t<float>({n, size_t(3)});
    auto nrm = py::array_t<float>({n, size_t(3)});
    auto p = pos.mutable_unchecked<2>();
    auto nr = nrm.mutable_unchecked<2>();
    for (size_t i = 0; i < n; ++i) {
        p(i,0)=pcd.positions[i].x; p(i,1)=pcd.positions[i].y; p(i,2)=pcd.positions[i].z;
    }
    bool has_normals = !pcd.normals.empty();
    if (has_normals)
        for (size_t i = 0; i < n; ++i) {
            nr(i,0)=pcd.normals[i].x; nr(i,1)=pcd.normals[i].y; nr(i,2)=pcd.normals[i].z;
        }
    return py::make_tuple(pos, has_normals ? nrm : py::array_t<float>());
}

PYBIND11_MODULE(p3dm_py, m) {
    m.doc() = "PhoXi3DMeshing API Python bindings";

    // ── ProcessingSettings 노출 (선택 파라미터) ──────────────────────
    py::class_<settings::Filtering>(m, "Filtering")
        .def(py::init<>())
        .def_readwrite("consensus_level",        &settings::Filtering::pf_consensus_level)
        .def_readwrite("auto_cell_size",         &settings::Filtering::pf_auto_cell_size)
        .def_readwrite("cell_size",              &settings::Filtering::pf_cell_size)
        .def_readwrite("use_cluster_filtering",  &settings::Filtering::use_cluster_filtering);

    py::class_<settings::Alignment>(m, "Alignment")
        .def(py::init<>())
        .def_readwrite("max_iter_global",        &settings::Alignment::pa_max_iter_global)
        .def_readwrite("ratio_of_samples",       &settings::Alignment::pa_ratio_of_samples);

    py::class_<settings::Reconstruction>(m, "Reconstruction")
        .def(py::init<>())
        .def_readwrite("depth",                  &settings::Reconstruction::depth)
        .def_readwrite("point_weight",           &settings::Reconstruction::point_weight)
        .def_readwrite("intermediate_dir",       &settings::Reconstruction::intermediate_dir);

    py::class_<Aabb>(m, "Aabb")
        .def(py::init<>())
        .def_readwrite("min", &Aabb::min)
        .def_readwrite("max", &Aabb::max);

    // ── PhoXi3DMeshingApi ─────────────────────────────────────────────
    py::class_<PhoXi3DMeshingApi>(m, "PhoXi3DMeshingApi")
        .def(py::init([](){ 
            // Python stdout으로 로그 리다이렉트
            static std::ostringstream oss;
            return std::make_unique<PhoXi3DMeshingApi>(std::cout);
        }))
        // 스캔 입력
        .def("import_scan",       &PhoXi3DMeshingApi::ImportScan,
             "path (.ply/.cogs/.praw) → scan id")
        .def("remove_scan",       &PhoXi3DMeshingApi::RemoveScan)
        .def("remove_all_scans",  &PhoXi3DMeshingApi::RemoveAllScans)
        .def("get_all_scan_ids",  &PhoXi3DMeshingApi::GetAllScanIds)
        // 스캔 처리
        .def("filter_scans",      [](PhoXi3DMeshingApi &api, settings::Filtering s){
            return api.FilterScans(s); }, py::arg("settings") = settings::Filtering{})
        .def("align_scans",       [](PhoXi3DMeshingApi &api, settings::Alignment s){
            return api.AlignScans(s); }, py::arg("settings") = settings::Alignment{})
        .def("crop_scans",        [](PhoXi3DMeshingApi &api, 
            std::array<float,3> mn, std::array<float,3> mx){
            Aabb aabb;
            aabb.min = {mn[0], mn[1], mn[2]};
            aabb.max = {mx[0], mx[1], mx[2]};
            api.CropScansWithBoundingBox(aabb);
        }, "min_xyz, max_xyz (mm)")
        // Union PCD
        .def("get_union_pcd_numpy", [](PhoXi3DMeshingApi &api){
            auto pcd = api.GetUnionPointCloud();
            return pcd_to_numpy(*pcd);
        }, "→ (positions_np (N,3), normals_np (N,3))")
        .def("export_union_pcd",  [](PhoXi3DMeshingApi &api, const std::string &path){
            return api.ExportUnionPointCloud(path);
        })
        // 메쉬 생성
        .def("create_mesh",       [](PhoXi3DMeshingApi &api,
            const std::string &poisson_path, settings::Reconstruction s){
            return api.CreateMesh(poisson_path, s);
        }, py::arg("poisson_exe_path"), py::arg("settings") = settings::Reconstruction{})
        .def("trim_mesh",         [](PhoXi3DMeshingApi &api){
            return api.TrimMesh();
        })
        .def("get_mesh_numpy",    [](PhoXi3DMeshingApi &api){
            return mesh_to_numpy(api.GetMesh());
        }, "→ (verts (N,3) float32, faces (M,3) uint32)")
        .def("export_mesh",       [](PhoXi3DMeshingApi &api, const std::string &path){
            return api.ExportMesh(path);
        }, "path (.ply/.stl/.fbx/.cogs)")
        .def("clear",             &PhoXi3DMeshingApi::Clear);
}
```

### 6.3 CMakeLists.txt (`mms/bindings/CMakeLists.txt`)

```cmake
cmake_minimum_required(VERSION 3.20)
project(p3dm_py)

# Python 환경 (conda)
find_package(Python 3.11 REQUIRED COMPONENTS Interpreter Development)
find_package(pybind11 REQUIRED CONFIG)

set(P3DM_DIR "C:/Program Files/Photoneo/3DMeshing/2.3.0/API")

pybind11_add_module(p3dm_py p3dm_wrapper.cpp)

target_include_directories(p3dm_py PRIVATE
    ${P3DM_DIR}/include
    ${P3DM_DIR}/ThirdParty/include
)

target_link_libraries(p3dm_py PRIVATE
    ${P3DM_DIR}/lib/Release/PhoXi3DMeshing_API_msvc143.lib
    ${P3DM_DIR}/lib/Release/TurnTables_msvc143.lib
    ${P3DM_DIR}/lib/Release/Utils_msvc143.lib
    ${P3DM_DIR}/ThirdParty/lib/Release/PhoXi_API.lib
)

# DLL 경로를 PATH에 추가하도록 빌드 후 스크립트에 명시
message(STATUS "빌드 후 PATH에 추가 필요: C:/Program Files/Photoneo/3DMeshing/2.3.0")
```

### 6.4 빌드 명령

```bat
:: Developer Command Prompt for VS 2022 에서 실행
cd mms\bindings
cmake -B build -DCMAKE_BUILD_TYPE=Release -A x64
cmake --build build --config Release
:: 결과: build\Release\p3dm_py.pyd
```

### 6.5 Python에서 사용

```python
# mms/sensor/p3dm_wrapper.py
import os, sys
from pathlib import Path

_P3DM_DLL_DIR = Path(r"C:\Program Files\Photoneo\3DMeshing\2.3.0")
os.add_dll_directory(str(_P3DM_DLL_DIR))          # 의존 DLL 검색 경로
sys.path.insert(0, str(Path(__file__).parent.parent / "bindings/build/Release"))

import p3dm_py
import numpy as np
import open3d as o3d

POISSON_EXE = str(_P3DM_DLL_DIR / "PoissonRecon.exe")

class P3DMWrapper:
    """PhoXi3DMeshing API Python 래퍼."""

    def __init__(self):
        self._api = p3dm_py.PhoXi3DMeshingApi()

    def add_scan_from_file(self, ply_path: str) -> int:
        return self._api.import_scan(ply_path)

    def process(
        self,
        crop_min_mm=None, crop_max_mm=None,
        filter_settings=None, align_settings=None,
        poisson_depth: int = 9,
    ) -> o3d.geometry.TriangleMesh:
        if crop_min_mm and crop_max_mm:
            self._api.crop_scans(crop_min_mm, crop_max_mm)
        self._api.filter_scans(filter_settings or p3dm_py.Filtering())
        self._api.align_scans(align_settings or p3dm_py.Alignment())
        recon = p3dm_py.Reconstruction()
        recon.depth = poisson_depth
        self._api.create_mesh(POISSON_EXE, recon)
        self._api.trim_mesh()
        return self._get_o3d_mesh()

    def _get_o3d_mesh(self) -> o3d.geometry.TriangleMesh:
        verts, faces = self._api.get_mesh_numpy()
        mesh = o3d.geometry.TriangleMesh()
        mesh.vertices  = o3d.utility.Vector3dVector(verts.astype(np.float64))
        mesh.triangles = o3d.utility.Vector3iVector(faces.astype(np.int32))
        mesh.compute_vertex_normals()
        return mesh

    def clear(self):
        self._api.clear()
```

### 6.6 `AddScan(PFrame)` 직접 연동 (추후)

현재 래퍼에서 `ImportScan(ply_path)` 경유 방식을 사용하는 이유:  
`AddScan`의 입력 타입 `pho::api::PFrame`은 Photoneo Python SDK가 반환하는 객체와 C++ ABI 레벨에서 일치해야 한다.
PhoXi Python SDK가 pybind11 기반으로 구현되어 있다면 Python 레벨에서의 변환이 가능하지만, SDK 내부 바인딩 확인이 필요하다.  
확인 전까지는 **캡처 → PLY 저장 → ImportScan** 방식이 안전하다.

---

## 7. 참고

- 3DMeshing API 예제: `C:\Program Files\Photoneo\3DMeshing\2.3.0\API\examples\`
  - `ManualMode/main.cpp` — 수동 캡처 + 정렬 + 메쉬 전체 파이프라인
  - `AutomaticMode/main.cpp` — 턴테이블 자동 캡처
- PoissonRecon.exe: `C:\Program Files\Photoneo\3DMeshing\2.3.0\PoissonRecon.exe`
- Hand-Eye calibration 상태: `docs/Calibration.md`
