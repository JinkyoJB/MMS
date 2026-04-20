/**
 * mms/sensor/artec/artec_binding.cpp
 *
 * pybind11 binding: Artec 3D Scanning SDK -> Python module (artec_sdk_py)
 *
 * Main workflow:
 *   enumerate_scanners()         -> list of connected scanners
 *   ArtecScanner(serial)
 *     .initialize()              -> enumerateScanners + createScanner + createFrameProcessor
 *     .capture(capture_texture)  -> IFrame + reconstructMesh -> CaptureResult (numpy arrays)
 *     .shutdown()                -> release resources
 *
 * Coordinate unit: Artec SDK uses mm. artec_client.py converts to m.
 */

#define WIN32_LEAN_AND_MEAN
#include <windows.h>

#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <stdexcept>
#include <string>

// Artec SDK headers
#include <artec/sdk/base/TRef.h>
#include <artec/sdk/base/IRef.h>
#include <artec/sdk/base/IArray.h>
#include <artec/sdk/base/IFrameMesh.h>
#include <artec/sdk/base/IImage.h>
#include <artec/sdk/base/Types.h>
#include <artec/sdk/base/Point.h>
#include <artec/sdk/base/Errors.h>
#include <artec/sdk/capturing/IScanner.h>
#include <artec/sdk/capturing/IFrame.h>
#include <artec/sdk/capturing/IFrameProcessor.h>
#include <artec/sdk/capturing/IArrayScannerId.h>
#include <artec/sdk/capturing/ScannerInfo.h>

namespace py   = pybind11;
namespace base = artec::sdk::base;
namespace cap  = artec::sdk::capturing;

// ----------------------------------------------------------------------------
// Utilities
// ----------------------------------------------------------------------------

// wchar_t* -> UTF-8 std::string (Windows API)
static std::string wcs_to_utf8(const wchar_t* wstr)
{
    if (!wstr || wstr[0] == L'\0') return {};
    int len = WideCharToMultiByte(CP_UTF8, 0, wstr, -1, nullptr, 0, nullptr, nullptr);
    if (len <= 1) return {};
    std::string out(len - 1, '\0');
    WideCharToMultiByte(CP_UTF8, 0, wstr, -1, &out[0], len, nullptr, nullptr);
    return out;
}

// ErrorCode check -> throw std::runtime_error on failure
static void check_ec(base::ErrorCode ec, const char* context)
{
    if (ec != base::ErrorCode_OK)
    {
        char buf[256];
        std::snprintf(buf, sizeof(buf),
            "[ArtecSDK] %s failed (ErrorCode=0x%08X)",
            context, static_cast<unsigned>(ec));
        throw std::runtime_error(buf);
    }
}

// ----------------------------------------------------------------------------
// CaptureResult
// ----------------------------------------------------------------------------

struct CaptureResult
{
    // (N, 3) float32 -- vertex positions, mm, sensor (S) frame
    py::array_t<float>   points;
    // (M, 3) int32 -- triangle indices
    py::array_t<int32_t> triangles;
    // (N, 3) float32 -- smooth per-vertex normals. Shape (0,3) if unavailable.
    py::array_t<float>   normals;
    // (H, W, 3) uint8 -- RGB texture image. Shape (0,0,3) if unavailable.
    py::array_t<uint8_t> texture_image;
    int tex_width  = 0;
    int tex_height = 0;
};

// ----------------------------------------------------------------------------
// IImage -> (H, W, 3) uint8 RGB helper
// ----------------------------------------------------------------------------

static py::array_t<uint8_t> image_to_rgb(const base::IImage* img)
{
    int w     = img->getWidth();
    int h     = img->getHeight();
    int pitch = img->getPitch();
    base::PixelFormat fmt = img->getPixelFormat();
    const uint8_t* src = static_cast<const uint8_t*>(img->getPointer());

    auto arr = py::array_t<uint8_t>({(py::ssize_t)h, (py::ssize_t)w, (py::ssize_t)3});
    uint8_t* dst = arr.mutable_data();

    if (fmt == base::PixelFormat_BGR)
    {
        for (int r = 0; r < h; ++r)
        {
            const uint8_t* rs = src + r * pitch;
            uint8_t*       rd = dst + r * w * 3;
            for (int c = 0; c < w; ++c)
            {
                rd[c*3+0] = rs[c*3+2]; // R <- B
                rd[c*3+1] = rs[c*3+1]; // G
                rd[c*3+2] = rs[c*3+0]; // B <- R
            }
        }
        return arr;
    }
    if (fmt == base::PixelFormat_RGB)
    {
        for (int r = 0; r < h; ++r)
            std::memcpy(dst + r * w * 3, src + r * pitch, w * 3);
        return arr;
    }
    if (fmt == base::PixelFormat_Mono)
    {
        for (int r = 0; r < h; ++r)
        {
            const uint8_t* rs = src + r * pitch;
            uint8_t*       rd = dst + r * w * 3;
            for (int c = 0; c < w; ++c)
                rd[c*3+0] = rd[c*3+1] = rd[c*3+2] = rs[c];
        }
        return arr;
    }

    // Unsupported pixel format -> return empty
    return py::array_t<uint8_t>({(py::ssize_t)0, (py::ssize_t)0, (py::ssize_t)3});
}

// ----------------------------------------------------------------------------
// IFrameMesh -> CaptureResult
// ----------------------------------------------------------------------------

static CaptureResult mesh_to_result(base::IFrameMesh* mesh)
{
    CaptureResult r;

    // Vertex positions
    {
        base::IArrayPoint3F* arr = mesh->getPoints();
        int n = arr ? arr->getSize() : 0;
        auto out = py::array_t<float>({(py::ssize_t)n, (py::ssize_t)3});
        if (n > 0)
        {
            // Point3F memory layout: float x, y, z (contiguous)
            std::memcpy(out.mutable_data(),
                        arr->getPointer(),
                        n * sizeof(base::Point3F));
        }
        r.points = std::move(out);
    }

    // Triangle indices
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

    // Smooth per-vertex normals
    {
        base::IArrayPoint3F* arr = mesh->getPointsNormals();
        int n = arr ? arr->getSize() : 0;
        auto out = py::array_t<float>({(py::ssize_t)n, (py::ssize_t)3});
        if (n > 0)
        {
            std::memcpy(out.mutable_data(),
                        arr->getPointer(),
                        n * sizeof(base::Point3F));
        }
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

// ----------------------------------------------------------------------------
// ArtecScanner
// ----------------------------------------------------------------------------

class ArtecScanner
{
public:
    // serial: scanner serial number. Empty string selects first available.
    explicit ArtecScanner(const std::string& serial = "")
        : serial_(serial)
    {}

    ~ArtecScanner() { shutdown(); }

    void initialize()
    {
        if (scanner_) return; // already initialized

        // 1. Enumerate scanners
        base::TRef<cap::IArrayScannerId> list;
        check_ec(cap::enumerateScanners(&list), "enumerateScanners");

        int n = list->getSize();
        if (n == 0)
            throw std::runtime_error("[ArtecScanner] No scanners found.");

        const cap::ScannerId* ids = list->getPointer();

        // 2. Select target scanner
        int idx = -1;
        if (serial_.empty())
        {
            idx = 0;
        }
        else
        {
            // ASCII serial -> wstring comparison
            std::wstring target(serial_.begin(), serial_.end());
            for (int i = 0; i < n; ++i)
            {
                if (target == ids[i].serial)
                {
                    idx = i;
                    break;
                }
            }
            if (idx < 0)
                throw std::runtime_error("[ArtecScanner] Serial not found: " + serial_);
        }

        // 3. Create IScanner
        check_ec(cap::createScanner(&scanner_, &ids[idx]), "createScanner");

        // 4. Create IFrameProcessor (default settings)
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

    // Capture one frame and return reconstructed mesh as numpy arrays.
    // GIL is released during the blocking capture call.
    CaptureResult capture(bool capture_texture = true)
    {
        if (!scanner_)
            throw std::runtime_error("[ArtecScanner] Not initialized.");

        base::TRef<cap::IFrame>      frame;
        base::TRef<base::IFrameMesh> mesh;

        {
            py::gil_scoped_release release_gil;

            auto ec = scanner_->capture(&frame, capture_texture);
            if (ec != base::ErrorCode_OK)
            {
                char buf[128];
                std::snprintf(buf, sizeof(buf),
                    "capture failed (ErrorCode=0x%08X)",
                    static_cast<unsigned>(ec));
                throw std::runtime_error(buf);
            }

            if (capture_texture)
                check_ec(processor_->reconstructAndTexturizeMesh(&mesh, frame),
                         "reconstructAndTexturizeMesh");
            else
                check_ec(processor_->reconstructMesh(&mesh, frame),
                         "reconstructMesh");
        }

        return mesh_to_result(mesh);
    }

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

    std::string                       serial_;
    base::TRef<cap::IScanner>         scanner_;
    base::TRef<cap::IFrameProcessor>  processor_;
};

// ----------------------------------------------------------------------------
// Module-level: enumerate_scanners
// ----------------------------------------------------------------------------

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

// ----------------------------------------------------------------------------
// pybind11 module
// ----------------------------------------------------------------------------

PYBIND11_MODULE(artec_sdk_py, m)
{
    m.doc() = "Artec 3D Scanning SDK Python binding (pybind11)";

    m.def("enumerate_scanners", &enumerate_scanners,
          "Return list of connected Artec scanners."
          " Each entry: {index, serial, name, license, has_texture_camera}");

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
                (int)r.points.shape(0),
                (int)r.triangles.shape(0),
                r.tex_width, r.tex_height);
            return std::string(buf);
        });

    py::class_<ArtecScanner>(m, "ArtecScanner")
        .def(py::init<const std::string&>(),
             py::arg("serial") = "",
             "serial: scanner serial number. Empty string selects first available scanner.")
        .def("initialize",       &ArtecScanner::initialize,
             "Connect to scanner and initialize FrameProcessor.")
        .def("shutdown",         &ArtecScanner::shutdown,
             "Disconnect and release resources.")
        .def("capture",          &ArtecScanner::capture,
             py::arg("capture_texture") = true,
             "Capture one frame. Returns CaptureResult. GIL released during capture.")
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
}
