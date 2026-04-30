/**
 * mms/sensor/artec/artec_binding.cpp
 *
 * pybind11 binding: Artec Capturing SDK -> Python module (artec_sdk_py)
 *
 * Handles scanner connection, raw frame capture, and mesh reconstruction.
 * Base API (ModelHandle, ScanHandle, FrameMeshHandle) lives in artec_base_py.
 *
 * ============================================================
 * Python API
 * ============================================================
 *
 *   enumerate_scanners()      -> list[{index, serial, name, license, has_texture_camera}]
 *
 *   CaptureResult
 *     .points          (N,3) float32, mm, sensor frame
 *     .triangles       (M,3) int32
 *     .normals         (N,3) float32, (0,3) if unavailable
 *     .texture_image   (H,W,3) uint8, (0,0,3) if unavailable
 *     .tex_width / .tex_height
 *
 *   ArtecScanner(serial="")
 *     .initialize()
 *     .shutdown()
 *     .capture(capture_texture=True)    -> CaptureResult
 *     .scanner_capsule()                -> py::capsule  [for artec_base_py]
 *     .processor_capsule()              -> py::capsule  [for artec_base_py]
 *     .get_fps / set_fps / get_max_fps
 *     .get_texture_gain / set_texture_gain
 *     .get_serial / get_name / is_initialized
 *
 * Coordinate unit: Artec SDK uses mm.
 */

#include "artec_common.h"

#include <artec/sdk/base/TRef.h>
#include <artec/sdk/base/IRef.h>
#include <artec/sdk/base/IArray.h>
#include <artec/sdk/base/IFrameMesh.h>
#include <artec/sdk/base/Point.h>

#include <artec/sdk/capturing/IScanner.h>
#include <artec/sdk/capturing/IFrame.h>
#include <artec/sdk/capturing/IFrameProcessor.h>
#include <artec/sdk/capturing/IArrayScannerId.h>
#include <artec/sdk/capturing/ScannerInfo.h>

namespace cap = artec::sdk::capturing;

// ============================================================
// CaptureResult  (raw numpy output from capture())
// ============================================================

struct CaptureResult
{
    py::array_t<float>   points;        // (N,3) float32, mm, sensor frame
    py::array_t<int32_t> triangles;     // (M,3) int32
    py::array_t<float>   normals;       // (N,3) float32, (0,3) if unavailable
    py::array_t<uint8_t> texture_image; // (H,W,3) uint8, (0,0,3) if unavailable
    int tex_width  = 0;
    int tex_height = 0;
};

// IFrameMesh -> CaptureResult
static CaptureResult mesh_to_capture_result(base::IFrameMesh* mesh)
{
    CaptureResult r;

    // Vertices
    {
        base::IArrayPoint3F* arr = mesh->getPoints();
        int n = arr ? arr->getSize() : 0;
        auto out = py::array_t<float>({(py::ssize_t)n, (py::ssize_t)3});
        if (n > 0)
            std::memcpy(out.mutable_data(), arr->getPointer(), n * sizeof(base::Point3F));
        r.points = std::move(out);
    }

    // Triangles
    {
        base::IArrayIndexTriplet* arr = mesh->getTriangles();
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
        r.triangles = std::move(out);
    }

    // Normals
    {
        base::IArrayPoint3F* arr = mesh->getPointsNormals();
        int n = arr ? arr->getSize() : 0;
        auto out = py::array_t<float>({(py::ssize_t)n, (py::ssize_t)3});
        if (n > 0)
            std::memcpy(out.mutable_data(), arr->getPointer(), n * sizeof(base::Point3F));
        r.normals = std::move(out);
    }

    // Texture image
    {
        py::array_t<uint8_t> empty({(py::ssize_t)0, (py::ssize_t)0, (py::ssize_t)3});
        if (mesh->isTextured())
        {
            const base::IImage* img = mesh->getImage();
            if (img && img->getWidth() > 0 && img->getHeight() > 0)
            {
                r.tex_width  = img->getWidth();
                r.tex_height = img->getHeight();
                r.texture_image = image_to_rgb(img);
            }
            else
            {
                r.texture_image = std::move(empty);
            }
        }
        else
        {
            r.texture_image = std::move(empty);
        }
    }

    return r;
}

// ============================================================
// ArtecScanner
// ============================================================

class ArtecScanner
{
public:
    explicit ArtecScanner(const std::string& serial = "")
        : serial_(serial)
    {}

    ~ArtecScanner() { shutdown(); }

    void initialize()
    {
        if (scanner_) return;

        base::TRef<cap::IArrayScannerId> list;
        check_ec(cap::enumerateScanners(&list), "enumerateScanners");

        int n = list->getSize();
        if (n == 0)
            throw std::runtime_error("[ArtecScanner] No scanners found.");

        const cap::ScannerId* ids = list->getPointer();
        int idx = -1;
        if (serial_.empty())
        {
            idx = 0;
        }
        else
        {
            std::wstring target(serial_.begin(), serial_.end());
            for (int i = 0; i < n; ++i)
            {
                if (target == ids[i].serial) { idx = i; break; }
            }
            if (idx < 0)
                throw std::runtime_error("[ArtecScanner] Serial not found: " + serial_);
        }

        check_ec(cap::createScanner(&scanner_, &ids[idx]), "createScanner");
        check_ec(scanner_->createFrameProcessor(&processor_, nullptr),
                 "createFrameProcessor");

        const cap::ScannerId* id = scanner_->getId();
        std::printf("[ArtecScanner] Connected: %s  serial=%s\n",
                    wcs_to_utf8(id->name).c_str(),
                    wcs_to_utf8(id->serial).c_str());
    }

    void shutdown()
    {
        processor_.release();
        scanner_.release();
    }

    // ----------------------------------------------------------
    // capture() -> CaptureResult  (raw numpy)
    // ----------------------------------------------------------
    CaptureResult capture(bool capture_texture = true)
    {
        base::TRef<base::IFrameMesh> mesh = do_capture(capture_texture);
        return mesh_to_capture_result(static_cast<base::IFrameMesh*>(mesh));
    }

    // ----------------------------------------------------------
    // Capsule accessors for artec_base_py
    //
    // Each capsule holds a raw pointer with its own AddRef/release pair.
    // The TRef members in this class keep their own references, so the
    // scanner stays alive independently of the capsule lifetime.
    // ----------------------------------------------------------
    py::capsule scanner_capsule() const
    {
        require_init();
        cap::IScanner* raw = static_cast<cap::IScanner*>(scanner_);
        raw->addRef();
        return py::capsule(raw, "cap::IScanner*", [](void* ptr) {
            static_cast<cap::IScanner*>(ptr)->release();
        });
    }

    py::capsule processor_capsule() const
    {
        require_init();
        cap::IFrameProcessor* raw = static_cast<cap::IFrameProcessor*>(processor_);
        raw->addRef();
        return py::capsule(raw, "cap::IFrameProcessor*", [](void* ptr) {
            static_cast<cap::IFrameProcessor*>(ptr)->release();
        });
    }

    // ----------------------------------------------------------
    // Scanner property accessors
    // ----------------------------------------------------------
    float       get_fps()          const { require_init(); return scanner_->getFPS(); }
    void        set_fps(float fps)       { require_init(); scanner_->setFPS(fps); }
    float       get_max_fps()      const { require_init(); return scanner_->getMaximumFPS(); }
    float       get_texture_gain() const { require_init(); return scanner_->getTextureGain(); }
    void        set_texture_gain(float g){ require_init(); scanner_->setTextureGain(g); }
    bool        is_initialized()   const { return static_cast<bool>(scanner_); }

    std::string get_serial() const
    {
        if (!scanner_) return serial_;
        return wcs_to_utf8(scanner_->getId()->serial);
    }
    std::string get_name() const
    {
        if (!scanner_) return {};
        return wcs_to_utf8(scanner_->getId()->name);
    }

private:
    void require_init() const
    {
        if (!scanner_)
            throw std::runtime_error("[ArtecScanner] Not initialized.");
    }

    // Shared capture logic: scanner -> IFrameMesh TRef
    base::TRef<base::IFrameMesh> do_capture(bool with_texture)
    {
        require_init();
        base::TRef<cap::IFrame>      frame;
        base::TRef<base::IFrameMesh> mesh;
        {
            py::gil_scoped_release release_gil;

            auto ec = scanner_->capture(&frame, with_texture);
            if (ec != base::ErrorCode_OK)
            {
                char buf[128];
                std::snprintf(buf, sizeof(buf),
                    "capture failed (ErrorCode=0x%08X)",
                    static_cast<unsigned>(ec));
                throw std::runtime_error(buf);
            }
            if (with_texture)
                check_ec(processor_->reconstructAndTexturizeMesh(&mesh, frame),
                         "reconstructAndTexturizeMesh");
            else
                check_ec(processor_->reconstructMesh(&mesh, frame),
                         "reconstructMesh");
        }
        return mesh;
    }

    std::string                       serial_;
    base::TRef<cap::IScanner>         scanner_;
    base::TRef<cap::IFrameProcessor>  processor_;
};

// ============================================================
// Module-level: enumerate_scanners
// ============================================================

static py::list enumerate_scanners()
{
    base::TRef<cap::IArrayScannerId> list;
    check_ec(cap::enumerateScanners(&list), "enumerateScanners");

    py::list result;
    int n = list->getSize();
    const cap::ScannerId* ids = list->getPointer();
    for (int i = 0; i < n; ++i)
    {
        py::dict d;
        d["index"]              = i;
        d["serial"]             = wcs_to_utf8(ids[i].serial);
        d["name"]               = wcs_to_utf8(ids[i].name);
        d["license"]            = wcs_to_utf8(ids[i].license);
        d["has_texture_camera"] = ids[i].isTextureCameraAvailable;
        result.append(d);
    }
    return result;
}

// ============================================================
// pybind11 module: artec_sdk_py
// ============================================================

PYBIND11_MODULE(artec_sdk_py, m)
{
    m.doc() = "Artec 3D Scanning SDK Python binding — Capturing API (artec_sdk_py)";

    py::class_<CaptureResult>(m, "CaptureResult")
        .def_readwrite("points",
            &CaptureResult::points,
            "(N,3) float32, vertex positions in mm, sensor frame")
        .def_readwrite("triangles",
            &CaptureResult::triangles,
            "(M,3) int32, triangle vertex indices")
        .def_readwrite("normals",
            &CaptureResult::normals,
            "(N,3) float32, smooth per-vertex normals. Shape (0,3) if unavailable.")
        .def_readwrite("texture_image",
            &CaptureResult::texture_image,
            "(H,W,3) uint8, RGB texture. Shape (0,0,3) if unavailable.")
        .def_readwrite("tex_width",  &CaptureResult::tex_width)
        .def_readwrite("tex_height", &CaptureResult::tex_height)
        .def("__repr__", [](const CaptureResult& r) {
            char buf[128];
            std::snprintf(buf, sizeof(buf),
                "CaptureResult(pts=%d, tris=%d, tex=%dx%d)",
                (int)r.points.shape(0), (int)r.triangles.shape(0),
                r.tex_width, r.tex_height);
            return std::string(buf);
        });

    py::class_<ArtecScanner>(m, "ArtecScanner")
        .def(py::init<const std::string&>(),
             py::arg("serial") = "",
             "serial: scanner serial number. Empty string selects first available scanner.")
        .def("initialize",
             &ArtecScanner::initialize,
             "Connect to scanner and initialize FrameProcessor.")
        .def("shutdown",
             &ArtecScanner::shutdown,
             "Disconnect and release resources.")
        .def("capture",
             &ArtecScanner::capture,
             py::arg("capture_texture") = true,
             "Capture one frame. Returns CaptureResult (raw numpy). GIL released during capture.")
        .def("scanner_capsule",
             &ArtecScanner::scanner_capsule,
             "Return py::capsule holding IScanner* for use with artec_base_py.")
        .def("processor_capsule",
             &ArtecScanner::processor_capsule,
             "Return py::capsule holding IFrameProcessor* for use with artec_base_py.")
        .def("get_fps",          &ArtecScanner::get_fps)
        .def("set_fps",          &ArtecScanner::set_fps,          py::arg("fps"))
        .def("get_max_fps",      &ArtecScanner::get_max_fps)
        .def("get_texture_gain", &ArtecScanner::get_texture_gain)
        .def("set_texture_gain", &ArtecScanner::set_texture_gain, py::arg("gain"))
        .def("get_serial",       &ArtecScanner::get_serial)
        .def("get_name",         &ArtecScanner::get_name)
        .def("is_initialized",   &ArtecScanner::is_initialized)
        .def("__repr__", [](const ArtecScanner& s) {
            return "<ArtecScanner serial='" + s.get_serial()
                 + "' name='" + s.get_name() + "'>";
        });

    m.def("enumerate_scanners",
          &enumerate_scanners,
          "Return list of connected Artec scanners."
          " Each entry: {index, serial, name, license, has_texture_camera}");
}
