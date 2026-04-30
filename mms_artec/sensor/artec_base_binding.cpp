/**
 * mms/sensor/artec/artec_base_binding.cpp
 *
 * pybind11 binding: Artec SDK Base API -> Python module (artec_base_py)
 *
 * 역할: SDK C++ 타입을 capsule 기반 free function으로만 노출.
 *       클래스 구조(FrameMeshHandle, ScanHandle, ModelHandle, DTOs)는
 *       artec_base.py 에서 Python으로 구현.
 *
 * ============================================================
 * 노출 함수 목록
 * ============================================================
 *
 * [IFrameMesh* 조작]
 *   frame_mesh_vertices(cap)      -> (N,3) float32 ndarray
 *   frame_mesh_faces(cap)         -> (M,3) int32   ndarray
 *   frame_mesh_uv(cap)            -> (N,2) float32 ndarray | None
 *   frame_mesh_image(cap)         -> (H,W,3) uint8 ndarray | None
 *   frame_mesh_is_textured(cap)   -> bool
 *   frame_mesh_has_image(cap)     -> bool
 *   frame_mesh_vertex_count(cap)  -> int
 *   frame_mesh_face_count(cap)    -> int
 *   frame_mesh_image_dims(cap)    -> tuple(width, height)
 *
 * [IScan* 조작]
 *   scan_frame_count(cap)         -> int
 *   scan_get_frame(cap, i)        -> IFrameMesh* capsule
 *   scan_is_empty(cap)            -> bool
 *
 * [IModel* 조작]
 *   model_scan_count(cap)         -> int
 *   model_get_scan(cap, i)        -> IScan* capsule
 *   model_has_final_mesh(cap)     -> bool
 *   model_final_vertices(cap)     -> (N,3) float32 ndarray
 *   model_final_faces(cap)        -> (M,3) int32   ndarray
 *   model_save_obj(cap, path)     -> None
 *   model_composite_textured(cap) -> bool
 *
 * [모듈 함수]
 *   create_model()                             -> IModel*    capsule
 *   capture_frame_mesh(sc, proc, with_texture) -> IFrameMesh* capsule
 *   capture_to_model(sc, proc, with_texture)   -> IModel*    capsule
 *
 *   sc   : IScanner*       capsule from artec_sdk_py / artec_capturing_py
 *   proc : IFrameProcessor* capsule from artec_sdk_py / artec_capturing_py
 */

#include "artec_common.h"

#include <fstream>
#include <vector>

#include <artec/sdk/base/TRef.h>
#include <artec/sdk/base/IRef.h>
#include <artec/sdk/base/IArray.h>
#include <artec/sdk/base/IFrameMesh.h>
#include <artec/sdk/base/Point.h>
#include <artec/sdk/base/IModel.h>
#include <artec/sdk/base/IScan.h>
#include <artec/sdk/base/ICompositeMesh.h>
#include <artec/sdk/base/ICompositeContainer.h>
#include <artec/sdk/base/IMesh.h>
#include <artec/sdk/base/Matrix.h>

#include <artec/sdk/capturing/IScanner.h>
#include <artec/sdk/capturing/IFrame.h>
#include <artec/sdk/capturing/IFrameProcessor.h>

namespace cap = artec::sdk::capturing;

// ============================================================
// 내부 헬퍼: capsule → raw pointer
// ============================================================

static base::IFrameMesh* get_frame_mesh(py::capsule c)
{
    auto* p = static_cast<base::IFrameMesh*>(static_cast<void*>(c));
    if (!p) throw std::runtime_error("[artec_base] Null IFrameMesh capsule.");
    return p;
}

static base::IScan* get_scan(py::capsule c)
{
    auto* p = static_cast<base::IScan*>(static_cast<void*>(c));
    if (!p) throw std::runtime_error("[artec_base] Null IScan capsule.");
    return p;
}

static base::IModel* get_model(py::capsule c)
{
    auto* p = static_cast<base::IModel*>(static_cast<void*>(c));
    if (!p) throw std::runtime_error("[artec_base] Null IModel capsule.");
    return p;
}

// ============================================================
// 내부 헬퍼: raw pointer → capsule (release deleter)
// ============================================================

static py::capsule make_frame_mesh_cap(base::IFrameMesh* raw)
{
    return py::capsule(raw, "IFrameMesh*",
        [](void* p){ static_cast<base::IFrameMesh*>(p)->release(); });
}

static py::capsule make_scan_cap(base::IScan* raw)
{
    return py::capsule(raw, "IScan*",
        [](void* p){ static_cast<base::IScan*>(p)->release(); });
}

static py::capsule make_model_cap(base::IModel* raw)
{
    return py::capsule(raw, "IModel*",
        [](void* p){ static_cast<base::IModel*>(p)->release(); });
}

// ============================================================
// IFrameMesh* 조작 함수
// ============================================================

static py::array_t<float> frame_mesh_vertices(py::capsule cap)
{
    base::IArrayPoint3F* arr = get_frame_mesh(cap)->getPoints();
    int n = arr ? arr->getSize() : 0;
    auto out = py::array_t<float>({(py::ssize_t)n, (py::ssize_t)3});
    if (n > 0)
        std::memcpy(out.mutable_data(), arr->getPointer(), n * sizeof(base::Point3F));
    return out;
}

static py::array_t<int32_t> frame_mesh_faces(py::capsule cap)
{
    base::IArrayIndexTriplet* arr = get_frame_mesh(cap)->getTriangles();
    int n = arr ? arr->getSize() : 0;
    auto out = py::array_t<int32_t>({(py::ssize_t)n, (py::ssize_t)3});
    if (n > 0)
    {
        const base::IndexTriplet* src = arr->getPointer();
        int32_t* dst = out.mutable_data();
        for (int i = 0; i < n; ++i)
        {
            dst[i*3+0] = static_cast<int32_t>(src[i].x);
            dst[i*3+1] = static_cast<int32_t>(src[i].y);
            dst[i*3+2] = static_cast<int32_t>(src[i].z);
        }
    }
    return out;
}

static py::object frame_mesh_uv(py::capsule cap)
{
    base::IArrayUVCoordinates* arr = get_frame_mesh(cap)->getUVCoordinates();
    if (!arr || arr->getSize() == 0) return py::none();
    int n = arr->getSize();
    auto out = py::array_t<float>({(py::ssize_t)n, (py::ssize_t)2});
    const base::UVCoordinates* src = arr->getPointer();
    float* dst = out.mutable_data();
    for (int i = 0; i < n; ++i)
    {
        dst[i*2+0] = src[i].u;
        dst[i*2+1] = src[i].v;
    }
    return out;
}

static py::object frame_mesh_image(py::capsule cap)
{
    auto* mesh = get_frame_mesh(cap);
    if (!mesh->isTextured()) return py::none();
    const base::IImage* img = mesh->getImage();
    if (!img || img->getWidth() == 0 || img->getHeight() == 0) return py::none();
    return image_to_rgb(img);
}

static bool frame_mesh_is_textured(py::capsule cap)
{
    return get_frame_mesh(cap)->isTextured();
}

static bool frame_mesh_has_image(py::capsule cap)
{
    auto* mesh = get_frame_mesh(cap);
    if (!mesh->isTextured()) return false;
    const base::IImage* img = mesh->getImage();
    return img && img->getWidth() > 0 && img->getHeight() > 0;
}

static int frame_mesh_vertex_count(py::capsule cap)
{
    base::IArrayPoint3F* arr = get_frame_mesh(cap)->getPoints();
    return arr ? arr->getSize() : 0;
}

static int frame_mesh_face_count(py::capsule cap)
{
    base::IArrayIndexTriplet* arr = get_frame_mesh(cap)->getTriangles();
    return arr ? arr->getSize() : 0;
}

static py::tuple frame_mesh_image_dims(py::capsule cap)
{
    auto* mesh = get_frame_mesh(cap);
    if (!mesh->isTextured()) return py::make_tuple(0, 0);
    const base::IImage* img = mesh->getImage();
    if (!img) return py::make_tuple(0, 0);
    return py::make_tuple(img->getWidth(), img->getHeight());
}

// ============================================================
// IScan* 조작 함수
// ============================================================

static int scan_frame_count(py::capsule cap)
{
    return get_scan(cap)->getSize();
}

static py::capsule scan_get_frame(py::capsule cap, int i)
{
    auto* scan = get_scan(cap);
    int n = scan->getSize();
    if (i < 0 || i >= n)
        throw std::out_of_range(
            "frame index " + std::to_string(i)
            + " out of range [0, " + std::to_string(n) + ")");
    base::TRef<base::IFrameMesh> f;
    f = scan->getElement(i);   // TRef::operator= calls addRef
    if (!f) throw std::runtime_error("getElement(" + std::to_string(i) + ") returned null");
    return make_frame_mesh_cap(f.detach()); // detach transfers ownership to capsule
}

static bool scan_is_empty(py::capsule cap)
{
    return get_scan(cap)->getSize() == 0;
}

// scan_add_frame(scan_cap, frame_cap) — IScan->add(IFrameMesh*).
// SDK 가 내부에서 addRef 하므로, 호출자(Python capsule) 의 ref 는 유지된다.
static void scan_add_frame(py::capsule scan_cap, py::capsule frame_cap)
{
    auto* scan  = get_scan(scan_cap);
    auto* frame = get_frame_mesh(frame_cap);
    check_ec(scan->add(frame), "scan::add");
}

// ── numpy 4x4 ↔ base::Matrix4x4D ─────────────────────────────────
static base::Matrix4x4D np_to_mat4d(py::array_t<double, py::array::c_style | py::array::forcecast> arr)
{
    if (arr.ndim() != 2 || arr.shape(0) != 4 || arr.shape(1) != 4)
        throw std::runtime_error("[artec_base] Matrix must be (4, 4) float64.");
    base::Matrix4x4D m;
    auto buf = arr.unchecked<2>();
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            m.m[i][j] = buf(i, j);
    return m;
}

static py::array_t<double> mat4d_to_np(const base::Matrix4x4D& m)
{
    auto out = py::array_t<double>({(py::ssize_t)4, (py::ssize_t)4});
    auto buf = out.mutable_unchecked<2>();
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            buf(i, j) = m.m[i][j];
    return out;
}

// scan_set_frame_transformation(scan_cap, idx, mat4) — IScan->setTransformation(idx, m).
// 등록 알고리즘이 사용할 frame 의 초기 / 정합 transformation 을 외부에서 set.
static void scan_set_frame_transformation(
    py::capsule scan_cap, int idx, py::array_t<double> mat)
{
    auto* scan = get_scan(scan_cap);
    base::Matrix4x4D m = np_to_mat4d(mat);
    check_ec(scan->setTransformation(idx, m), "scan::setTransformation");
}

static py::array_t<double> scan_get_frame_transformation(
    py::capsule scan_cap, int idx)
{
    auto* scan = get_scan(scan_cap);
    return mat4d_to_np(scan->getTransformation(idx));
}

// ============================================================
// IModel* 조작 함수
// ============================================================

static int model_scan_count(py::capsule cap)
{
    return get_model(cap)->getSize();
}

static py::capsule model_get_scan(py::capsule cap, int i)
{
    auto* model = get_model(cap);
    int n = model->getSize();
    if (i < 0 || i >= n)
        throw std::out_of_range(
            "scan index " + std::to_string(i)
            + " out of range [0, " + std::to_string(n) + ")");
    base::TRef<base::IScan> s;
    s = model->getElement(i);  // addRef
    if (!s) throw std::runtime_error("getElement(" + std::to_string(i) + ") returned null");
    return make_scan_cap(s.detach());
}

static bool model_has_final_mesh(py::capsule cap)
{
    base::ICompositeContainer* cont = get_model(cap)->getCompositeContainer();
    return cont && cont->getSize() > 0;
}

static base::ICompositeMesh* get_first_composite(py::capsule cap)
{
    base::ICompositeContainer* cont = get_model(cap)->getCompositeContainer();
    if (!cont || cont->getSize() == 0) return nullptr;
    return cont->getElement(0);
}

static py::array_t<float> model_final_vertices(py::capsule cap)
{
    base::ICompositeMesh* cm = get_first_composite(cap);
    if (!cm) return py::array_t<float>({(py::ssize_t)0, (py::ssize_t)3});
    base::IArrayPoint3F* arr = cm->getPoints();
    int n = arr ? arr->getSize() : 0;
    auto out = py::array_t<float>({(py::ssize_t)n, (py::ssize_t)3});
    if (n > 0)
        std::memcpy(out.mutable_data(), arr->getPointer(), n * sizeof(base::Point3F));
    return out;
}

static py::array_t<int32_t> model_final_faces(py::capsule cap)
{
    base::ICompositeMesh* cm = get_first_composite(cap);
    if (!cm) return py::array_t<int32_t>({(py::ssize_t)0, (py::ssize_t)3});
    base::IArrayIndexTriplet* arr = cm->getTriangles();
    int n = arr ? arr->getSize() : 0;
    auto out = py::array_t<int32_t>({(py::ssize_t)n, (py::ssize_t)3});
    if (n > 0)
    {
        const base::IndexTriplet* src = arr->getPointer();
        int32_t* dst = out.mutable_data();
        for (int i = 0; i < n; ++i)
        {
            dst[i*3+0] = static_cast<int32_t>(src[i].x);
            dst[i*3+1] = static_cast<int32_t>(src[i].y);
            dst[i*3+2] = static_cast<int32_t>(src[i].z);
        }
    }
    return out;
}

static bool model_composite_textured(py::capsule cap)
{
    base::ICompositeMesh* cm = get_first_composite(cap);
    return cm && cm->getTexturesCount() > 0;
}

// model_add_scan(model_cap, scan_cap) — IModel->add(IScan*).
// SDK 가 내부에서 addRef 하므로, 호출자(Python capsule) 의 ref 는 유지된다.
static void model_add_scan(py::capsule model_cap, py::capsule scan_cap)
{
    auto* model = get_model(model_cap);
    auto* scan  = get_scan(scan_cap);
    check_ec(model->add(scan), "model::add");
}

static void model_save_obj(py::capsule cap, const std::string& path)
{
    base::ICompositeMesh* cm = get_first_composite(cap);
    if (!cm)
        throw std::runtime_error(
            "[artec_base] No composite mesh. Run algorithms before saving.");

    base::IArrayPoint3F*      pts  = cm->getPoints();
    base::IArrayIndexTriplet* tris = cm->getTriangles();
    int nv = pts  ? pts->getSize()  : 0;
    int nf = tris ? tris->getSize() : 0;

    std::ofstream ofs(path);
    if (!ofs)
        throw std::runtime_error("[artec_base] Cannot open file: " + path);

    ofs << "# Artec SDK export — artec_base\n";
    ofs << "# vertices: " << nv << "  faces: " << nf << "\n\n";

    const base::Point3F*      vp = pts  ? pts->getPointer()  : nullptr;
    const base::IndexTriplet* tp = tris ? tris->getPointer() : nullptr;

    for (int i = 0; i < nv; ++i)
        ofs << "v " << vp[i].x << " " << vp[i].y << " " << vp[i].z << "\n";
    for (int i = 0; i < nf; ++i)
        ofs << "f " << (tp[i].x + 1) << " "
                    << (tp[i].y + 1) << " "
                    << (tp[i].z + 1) << "\n";
}

// ============================================================
// 모듈 함수
// ============================================================

static py::capsule create_model()
{
    base::TRef<base::IModel> model;
    check_ec(base::createModel(&model), "createModel");
    return make_model_cap(model.detach());
}

static py::capsule create_scan()
{
    base::TRef<base::IScan> scan;
    check_ec(base::createScan(&scan), "createScan");
    return make_scan_cap(scan.detach());
}

// Internal: capture + reconstruct → IFrameMesh TRef
static base::TRef<base::IFrameMesh> do_capture(
    py::capsule sc_cap, py::capsule proc_cap, bool with_texture)
{
    auto* scanner = static_cast<cap::IScanner*>(static_cast<void*>(sc_cap));
    auto* proc    = static_cast<cap::IFrameProcessor*>(static_cast<void*>(proc_cap));
    if (!scanner) throw std::runtime_error("[artec_base] Invalid scanner capsule.");
    if (!proc)    throw std::runtime_error("[artec_base] Invalid processor capsule.");

    base::TRef<cap::IFrame>      frame;
    base::TRef<base::IFrameMesh> mesh;
    {
        py::gil_scoped_release release_gil;
        auto ec = scanner->capture(&frame, with_texture);
        if (ec != base::ErrorCode_OK)
        {
            char buf[128];
            std::snprintf(buf, sizeof(buf),
                "[artec_base] capture failed (ErrorCode=0x%08X)",
                static_cast<unsigned>(ec));
            throw std::runtime_error(buf);
        }
        if (with_texture)
            check_ec(proc->reconstructAndTexturizeMesh(&mesh, frame),
                     "reconstructAndTexturizeMesh");
        else
            check_ec(proc->reconstructMesh(&mesh, frame), "reconstructMesh");
    }
    return mesh;
}

// capture_frame_mesh(sc_cap, proc_cap, with_texture) -> IFrameMesh* capsule
static py::capsule capture_frame_mesh(
    py::capsule sc_cap, py::capsule proc_cap, bool with_texture = true)
{
    base::TRef<base::IFrameMesh> mesh = do_capture(sc_cap, proc_cap, with_texture);
    return make_frame_mesh_cap(mesh.detach());
}

// capture_to_model(sc_cap, proc_cap, with_texture) -> IModel* capsule
static py::capsule capture_to_model(
    py::capsule sc_cap, py::capsule proc_cap, bool with_texture = true)
{
    base::TRef<base::IFrameMesh> mesh = do_capture(sc_cap, proc_cap, with_texture);

    base::TRef<base::IModel> model;
    check_ec(base::createModel(&model), "createModel");

    base::TRef<base::IScan> scan;
    check_ec(base::createScan(&scan), "createScan");
    check_ec(scan->add(static_cast<base::IFrameMesh*>(mesh)), "scan::add");
    check_ec(model->add(static_cast<base::IScan*>(scan)),     "model::add");

    return make_model_cap(model.detach());
}

// ============================================================
// pybind11 module: artec_base_py
// ============================================================

PYBIND11_MODULE(artec_base_py, m)
{
    m.doc() =
        "Artec SDK Base API raw bindings (artec_base_py).\n"
        "All functions are free functions operating on capsule handles.\n"
        "Python classes (FrameMeshHandle, ScanHandle, ModelHandle, DTOs)\n"
        "live in artec_base.py.";

    // ── IFrameMesh* 조작 ─────────────────────────────────────

    m.def("frame_mesh_vertices",
          &frame_mesh_vertices, py::arg("cap"),
          "Return (N,3) float32 vertex positions in mm, sensor frame.");

    m.def("frame_mesh_faces",
          &frame_mesh_faces, py::arg("cap"),
          "Return (M,3) int32 triangle face indices.");

    m.def("frame_mesh_uv",
          &frame_mesh_uv, py::arg("cap"),
          "Return (N,2) float32 UV coordinates, or None.");

    m.def("frame_mesh_image",
          &frame_mesh_image, py::arg("cap"),
          "Return (H,W,3) uint8 RGB texture image, or None.");

    m.def("frame_mesh_is_textured",
          &frame_mesh_is_textured, py::arg("cap"));

    m.def("frame_mesh_has_image",
          &frame_mesh_has_image, py::arg("cap"));

    m.def("frame_mesh_vertex_count",
          &frame_mesh_vertex_count, py::arg("cap"));

    m.def("frame_mesh_face_count",
          &frame_mesh_face_count, py::arg("cap"));

    m.def("frame_mesh_image_dims",
          &frame_mesh_image_dims, py::arg("cap"),
          "Return (width, height) of texture image. (0,0) if no image.");

    // ── IScan* 조작 ──────────────────────────────────────────

    m.def("scan_frame_count",
          &scan_frame_count, py::arg("cap"));

    m.def("scan_get_frame",
          &scan_get_frame, py::arg("cap"), py::arg("i"),
          "Return IFrameMesh* capsule for frame i.");

    m.def("scan_is_empty",
          &scan_is_empty, py::arg("cap"));

    m.def("scan_add_frame",
          &scan_add_frame, py::arg("scan_cap"), py::arg("frame_cap"),
          "Append IFrameMesh* into IScan*.");

    m.def("scan_set_frame_transformation",
          &scan_set_frame_transformation,
          py::arg("scan_cap"), py::arg("idx"), py::arg("matrix"),
          "Set 4x4 rigid transformation for frame[idx] (numpy float64).");

    m.def("scan_get_frame_transformation",
          &scan_get_frame_transformation,
          py::arg("scan_cap"), py::arg("idx"),
          "Get 4x4 rigid transformation for frame[idx] (numpy float64).");

    // ── IModel* 조작 ─────────────────────────────────────────

    m.def("model_scan_count",
          &model_scan_count, py::arg("cap"));

    m.def("model_get_scan",
          &model_get_scan, py::arg("cap"), py::arg("i"),
          "Return IScan* capsule for scan i.");

    m.def("model_has_final_mesh",
          &model_has_final_mesh, py::arg("cap"));

    m.def("model_final_vertices",
          &model_final_vertices, py::arg("cap"),
          "Return (N,3) float32 vertices of the final composite mesh.");

    m.def("model_final_faces",
          &model_final_faces, py::arg("cap"),
          "Return (M,3) int32 faces of the final composite mesh.");

    m.def("model_composite_textured",
          &model_composite_textured, py::arg("cap"),
          "Return True if the final composite mesh has textures.");

    m.def("model_add_scan",
          &model_add_scan, py::arg("model_cap"), py::arg("scan_cap"),
          "Append IScan* into IModel*.");

    m.def("model_save_obj",
          &model_save_obj, py::arg("cap"), py::arg("path"),
          "Save final composite mesh as OBJ file.");

    // ── 모듈 함수 ────────────────────────────────────────────

    m.def("create_scan",
          &create_scan,
          "Create empty IScan, return capsule.");

    m.def("create_model",
          &create_model,
          "Create an empty IModel. Returns IModel* capsule.");

    m.def("capture_frame_mesh",
          &capture_frame_mesh,
          py::arg("scanner_capsule"),
          py::arg("processor_capsule"),
          py::arg("with_texture") = true,
          "Capture one frame. Returns IFrameMesh* capsule. GIL released during capture.");

    m.def("capture_to_model",
          &capture_to_model,
          py::arg("scanner_capsule"),
          py::arg("processor_capsule"),
          py::arg("with_texture") = true,
          "Capture one frame, wrap in IModel > IScan > IFrameMesh. "
          "Returns IModel* capsule. GIL released during capture.");
}
