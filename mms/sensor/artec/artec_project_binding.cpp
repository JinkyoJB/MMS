/**
 * mms/sensor/artec/artec_project_binding.cpp
 *
 * pybind11 binding: Artec Project SDK -> artec_project_py
 *
 * 역할: SDK C++ 타입을 capsule 기반 free function으로만 노출.
 *       클래스 구조(ProjectManager, ProjectHandle, *SettingsDTO 등)는
 *       artec_project.py 에서 구현.
 *
 * ============================================================
 * 실행 패턴
 * ============================================================
 *   - IProject* : capsule "IProject*"  (addRef on create/open)
 *   - IModel*   : capsule "IModel*"    (로드 결과 / 저장 소스)
 *   - IScan*    : capsule "IScan*"     (모델에서 꺼낸 스캔)
 *   - ICompositeMesh* : capsule "ICompositeMesh*"  (합성 메시)
 *   - 모든 load/save job → executeJob (GIL 해제)
 *
 * ============================================================
 * 노출 함수 목록
 * ============================================================
 *
 * [프로젝트 관리]
 *   project_create(path_str)         -> IProject* capsule
 *   project_open(path_str)           -> IProject* capsule
 *   project_max_version()            -> int
 *
 * [IProject 속성]
 *   project_version(proj_cap)        -> int
 *   project_entry_count(proj_cap)    -> int
 *   project_get_entry(proj_cap, i)   -> dict {uuid, entry_type, size_bytes, name}
 *
 * [로드 / 저장]
 *   project_load_all(proj_cap)                                -> IModel* capsule
 *   project_save(proj_cap, model_cap, path_str, compr_level)  -> None
 *
 * [IScan 확장 (UUID 매핑용)]
 *   scan_uuid(scan_cap)  -> str
 *   scan_name(scan_cap)  -> str
 *
 * [ICompositeMesh]
 *   model_composite_count(model_cap)        -> int
 *   model_get_composite(model_cap, i)       -> ICompositeMesh* capsule
 *   composite_mesh_uuid(cmesh_cap)          -> str
 *   composite_mesh_name(cmesh_cap)          -> str
 *   composite_mesh_vertex_count(cmesh_cap)  -> int
 *   composite_mesh_face_count(cmesh_cap)    -> int
 *   composite_mesh_vertices(cmesh_cap)      -> (N,3) float32
 *   composite_mesh_faces(cmesh_cap)         -> (M,3) int32
 *   composite_mesh_is_textured(cmesh_cap)   -> bool
 */

#include "artec_common.h"

#include <artec/sdk/base/TRef.h>
#include <artec/sdk/base/IRef.h>
#include <artec/sdk/base/IModel.h>
#include <artec/sdk/base/IScan.h>
#include <artec/sdk/base/ICompositeMesh.h>
#include <artec/sdk/base/ICompositeContainer.h>
#include <artec/sdk/base/IString.h>
#include <artec/sdk/base/Uuid.h>
#include <artec/sdk/base/IJob.h>
#include <artec/sdk/base/AlgorithmWorkset.h>
#include <artec/sdk/base/IArray.h>

#include <artec/sdk/project/IProject.h>
#include <artec/sdk/project/ProjectSettings.h>
#include <artec/sdk/project/ProjectSaverSettings.h>
#include <artec/sdk/project/ProjectLoaderSettings.h>
#include <artec/sdk/project/EntryInfo.h>
#include <artec/sdk/project/EntryType.h>

#include <cstring>
#include <stdexcept>
#include <string>

namespace proj = artec::sdk::project;

// ============================================================
// 내부 헬퍼
// ============================================================

static proj::IProject* get_project(py::capsule c)
{
    auto* p = static_cast<proj::IProject*>(static_cast<void*>(c));
    if (!p) throw std::runtime_error("[artec_project] Null IProject* capsule.");
    return p;
}

static base::IModel* get_model(py::capsule c)
{
    auto* p = static_cast<base::IModel*>(static_cast<void*>(c));
    if (!p) throw std::runtime_error("[artec_project] Null IModel* capsule.");
    return p;
}

static base::IScan* get_scan(py::capsule c)
{
    auto* p = static_cast<base::IScan*>(static_cast<void*>(c));
    if (!p) throw std::runtime_error("[artec_project] Null IScan* capsule.");
    return p;
}

static base::ICompositeMesh* get_composite_mesh(py::capsule c)
{
    auto* p = static_cast<base::ICompositeMesh*>(static_cast<void*>(c));
    if (!p) throw std::runtime_error("[artec_project] Null ICompositeMesh* capsule.");
    return p;
}

static py::capsule make_project_cap(proj::IProject* raw)
{
    return py::capsule(raw, "IProject*",
        [](void* p){ static_cast<proj::IProject*>(p)->release(); });
}

static py::capsule make_model_cap(base::IModel* raw)
{
    return py::capsule(raw, "IModel*",
        [](void* p){ static_cast<base::IModel*>(p)->release(); });
}

static py::capsule make_scan_cap(base::IScan* raw)
{
    return py::capsule(raw, "IScan*",
        [](void* p){ static_cast<base::IScan*>(p)->release(); });
}

static py::capsule make_composite_mesh_cap(base::ICompositeMesh* raw)
{
    return py::capsule(raw, "ICompositeMesh*",
        [](void* p){ static_cast<base::ICompositeMesh*>(p)->release(); });
}

// UUID -> UTF-8 문자열 변환
static std::string uuid_to_str(const base::Uuid& uuid)
{
    base::IString* istr = nullptr;
    base::ErrorCode ec = base::convertUuidtoString(&istr, &uuid);
    if (ec != base::ErrorCode_OK || !istr)
        return "";
    base::TRef<base::IString> ref;
    ref.attach(istr);
    return wcs_to_utf8(ref->getPointer());
}

// ============================================================
// 프로젝트 관리
// ============================================================

static py::capsule project_create(const std::string& path_str)
{
    std::wstring path_w = utf8_to_wcs(path_str);
    proj::ProjectSettings settings{};
    settings.path = path_w.c_str();

    proj::IProject* raw = nullptr;
    check_ec(proj::createNewProject(&raw, &settings), "createNewProject");
    return make_project_cap(raw);
}

static py::capsule project_open(const std::string& path_str)
{
    std::wstring path_w = utf8_to_wcs(path_str);
    proj::IProject* raw = nullptr;
    check_ec(proj::openProject(&raw, path_w.c_str()), "openProject");
    return make_project_cap(raw);
}

static int project_max_version()
{
    return proj::getMaximumSupportedProjectVersion();
}

// ============================================================
// IProject 속성
// ============================================================

static int project_version(py::capsule proj_cap)
{
    return get_project(proj_cap)->getProjectVersion();
}

static int project_entry_count(py::capsule proj_cap)
{
    return get_project(proj_cap)->getEntryCount();
}

// 단일 엔트리 → dict {uuid, entry_type, size_bytes, name}
static py::dict project_get_entry(py::capsule proj_cap, int index)
{
    auto* project = get_project(proj_cap);
    int   n       = project->getEntryCount();
    if (index < 0 || index >= n)
        throw std::out_of_range("[artec_project] entry index out of range.");

    proj::EntryInfo      info{};
    base::IString*       name_raw = nullptr;
    check_ec(project->getEntry(index, &info, &name_raw), "getEntry");

    base::TRef<base::IString> name_ref;
    name_ref.attach(name_raw);   // takes ownership (SDK addRef'd it)

    std::string uuid_s = uuid_to_str(info.uuid);
    std::string name_s = (name_ref && name_ref->getPointer())
                         ? wcs_to_utf8(name_ref->getPointer())
                         : "";

    py::dict d;
    d["uuid"]       = uuid_s;
    d["entry_type"] = static_cast<int>(info.type);  // Unknown = -1 (0xFFFFFFFF as int)
    d["size_bytes"] = static_cast<long long>(info.size);
    d["name"]       = name_s;
    return d;
}

// ============================================================
// 로드
// ============================================================

// 프로젝트의 모든 엔트리를 새 IModel로 로드 → IModel* capsule
static py::capsule project_load_all(py::capsule proj_cap)
{
    auto* project = get_project(proj_cap);

    // 빈 출력 모델 생성
    base::TRef<base::IModel> out_model;
    check_ec(base::createModel(&out_model), "createModel (for load)");

    // loader 설정: entryList = nullptr → 모든 엔트리 로드
    proj::ProjectLoaderSettings settings{};
    settings.entryList = nullptr;

    base::IJob* loader_raw = nullptr;
    check_ec(project->createLoader(&loader_raw, &settings), "createLoader");
    base::TRef<base::IJob> loader;
    loader.attach(loader_raw);

    base::IModel*          mp = static_cast<base::IModel*>(out_model);
    base::AlgorithmWorkset ws{};
    ws.in           = mp;
    ws.out          = mp;
    ws.progress     = nullptr;
    ws.cancellation = nullptr;
    ws.threadsCount = 0;

    {
        py::gil_scoped_release rel;
        check_ec(base::executeJob(loader_raw, &ws), "executeJob (load)");
    }

    // capsule 용 addRef (out_model TRef가 함수 끝에서 release → net refcount 유지)
    mp->addRef();
    return make_model_cap(mp);
}

// ============================================================
// 저장
// ============================================================

static void project_save(
    py::capsule       proj_cap,
    py::capsule       model_cap,
    const std::string& path_str,
    int                compression_level)
{
    auto* project = get_project(proj_cap);
    auto* model   = get_model(model_cap);

    std::wstring path_w = utf8_to_wcs(path_str);
    base::Uuid   empty_uuid{};  // 16바이트 0으로 초기화

    proj::ProjectSaverSettings settings{};
    settings.path             = path_w.c_str();
    settings.projectId        = empty_uuid;
    settings.compressionLevel = compression_level;

    base::IJob* saver_raw = nullptr;
    check_ec(project->createSaver(&saver_raw, &settings), "createSaver");
    base::TRef<base::IJob> saver;
    saver.attach(saver_raw);

    base::AlgorithmWorkset ws{};
    ws.in           = model;
    ws.out          = model;
    ws.progress     = nullptr;
    ws.cancellation = nullptr;
    ws.threadsCount = 0;

    {
        py::gil_scoped_release rel;
        check_ec(base::executeJob(saver_raw, &ws), "executeJob (save)");
    }
}

// ============================================================
// IScan 확장 (UUID 매핑용)
// ============================================================

static std::string scan_uuid(py::capsule cap)
{
    auto* scan = get_scan(cap);
    return uuid_to_str(scan->getUuid());
}

static std::string scan_name(py::capsule cap)
{
    auto* scan = get_scan(cap);
    const wchar_t* name = scan->getName();
    return name ? wcs_to_utf8(name) : "";
}

// ============================================================
// ICompositeMesh (IModel CompositeContainer)
// ============================================================

static int model_composite_count(py::capsule cap)
{
    auto* model = get_model(cap);
    auto* cc    = model->getCompositeContainer();
    return cc ? cc->getSize() : 0;
}

static py::capsule model_get_composite(py::capsule cap, int i)
{
    auto* model = get_model(cap);
    auto* cc    = model->getCompositeContainer();
    if (!cc)
        throw std::runtime_error("[artec_project] Model has no CompositeContainer.");
    int n = cc->getSize();
    if (i < 0 || i >= n)
        throw std::out_of_range("[artec_project] composite mesh index out of range.");
    auto* cmesh = cc->getElement(i);
    if (!cmesh)
        throw std::runtime_error("[artec_project] getElement returned null ICompositeMesh.");
    cmesh->addRef();
    return make_composite_mesh_cap(cmesh);
}

static std::string composite_mesh_uuid(py::capsule cap)
{
    return uuid_to_str(get_composite_mesh(cap)->getUuid());
}

static std::string composite_mesh_name(py::capsule cap)
{
    const wchar_t* name = get_composite_mesh(cap)->getName();
    return name ? wcs_to_utf8(name) : "";
}

static int composite_mesh_vertex_count(py::capsule cap)
{
    auto* pts = get_composite_mesh(cap)->getPoints();
    return pts ? pts->getSize() : 0;
}

static int composite_mesh_face_count(py::capsule cap)
{
    auto* tris = get_composite_mesh(cap)->getTriangles();
    return tris ? tris->getSize() : 0;
}

static py::array_t<float> composite_mesh_vertices(py::capsule cap)
{
    auto* pts = get_composite_mesh(cap)->getPoints();
    if (!pts || pts->getSize() == 0)
        return py::array_t<float>({(py::ssize_t)0, (py::ssize_t)3});

    int n = pts->getSize();
    auto arr = py::array_t<float>({(py::ssize_t)n, (py::ssize_t)3});
    // Point3F 는 {float x, y, z} — tightly packed
    std::memcpy(arr.mutable_data(), pts->getPointer(),
                static_cast<size_t>(n) * 3 * sizeof(float));
    return arr;
}

static py::array_t<int32_t> composite_mesh_faces(py::capsule cap)
{
    auto* tris = get_composite_mesh(cap)->getTriangles();
    if (!tris || tris->getSize() == 0)
        return py::array_t<int32_t>({(py::ssize_t)0, (py::ssize_t)3});

    int m = tris->getSize();
    auto arr = py::array_t<int32_t>({(py::ssize_t)m, (py::ssize_t)3});
    // IndexTriplet 은 {int data[3]} — tightly packed
    std::memcpy(arr.mutable_data(), tris->getPointer(),
                static_cast<size_t>(m) * 3 * sizeof(int32_t));
    return arr;
}

static bool composite_mesh_is_textured(py::capsule cap)
{
    return get_composite_mesh(cap)->isTextured();
}

// ============================================================
// pybind11 module: artec_project_py
// ============================================================

PYBIND11_MODULE(artec_project_py, m)
{
    m.doc() =
        "Artec Project SDK raw bindings (artec_project_py).\n"
        "All functions are free functions operating on capsule handles.\n"
        "Python classes (ProjectManager, ProjectHandle, CompositeMeshHandle,\n"
        "DTOs, enums) live in artec_project.py.\n"
        "\n"
        "Load pattern  : project_load_all(proj_cap) -> IModel* capsule\n"
        "Save pattern  : project_save(proj_cap, model_cap, path, level)\n"
        "Entry UUID match: scan_uuid / composite_mesh_uuid vs project_get_entry dict";

    // 프로젝트 관리
    m.def("project_create",      &project_create,
          py::arg("path"),
          "Create a new Artec project at given path. Returns IProject* capsule.");
    m.def("project_open",        &project_open,
          py::arg("path"),
          "Open an existing Artec project. Returns IProject* capsule.");
    m.def("project_max_version", &project_max_version,
          "Return maximum supported project version.");

    // IProject 속성
    m.def("project_version",     &project_version,     py::arg("proj_cap"));
    m.def("project_entry_count", &project_entry_count, py::arg("proj_cap"));
    m.def("project_get_entry",   &project_get_entry,
          py::arg("proj_cap"), py::arg("index"),
          "Return dict {uuid, entry_type, size_bytes, name} for entry at index.");

    // 로드 / 저장
    m.def("project_load_all", &project_load_all,
          py::arg("proj_cap"),
          "Load all project entries into a new IModel. Returns IModel* capsule.");
    m.def("project_save", &project_save,
          py::arg("proj_cap"), py::arg("model_cap"),
          py::arg("path"), py::arg("compression_level") = 0,
          "Save model data to project file at path.");

    // IScan 확장
    m.def("scan_uuid", &scan_uuid, py::arg("scan_cap"),
          "Return UUID string of IScan*.");
    m.def("scan_name", &scan_name, py::arg("scan_cap"),
          "Return name string of IScan*.");

    // ICompositeMesh
    m.def("model_composite_count",   &model_composite_count,   py::arg("model_cap"));
    m.def("model_get_composite",     &model_get_composite,
          py::arg("model_cap"), py::arg("index"),
          "Return ICompositeMesh* capsule at given index.");
    m.def("composite_mesh_uuid",          &composite_mesh_uuid,          py::arg("cmesh_cap"));
    m.def("composite_mesh_name",          &composite_mesh_name,          py::arg("cmesh_cap"));
    m.def("composite_mesh_vertex_count",  &composite_mesh_vertex_count,  py::arg("cmesh_cap"));
    m.def("composite_mesh_face_count",    &composite_mesh_face_count,    py::arg("cmesh_cap"));
    m.def("composite_mesh_vertices",      &composite_mesh_vertices,      py::arg("cmesh_cap"),
          "Return (N,3) float32 vertex positions in mm.");
    m.def("composite_mesh_faces",         &composite_mesh_faces,         py::arg("cmesh_cap"),
          "Return (M,3) int32 triangle face indices.");
    m.def("composite_mesh_is_textured",   &composite_mesh_is_textured,   py::arg("cmesh_cap"));
}
