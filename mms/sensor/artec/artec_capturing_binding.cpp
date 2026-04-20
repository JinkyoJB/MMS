/**
 * mms/sensor/artec/artec_capturing_binding.cpp
 *
 * pybind11 binding: Artec Capturing SDK -> artec_capturing_py
 *
 * 역할: SDK C++ 타입을 capsule 기반 free function으로만 노출.
 *       클래스 구조(ScannerManager, ScannerHandle 등)는 artec_capturing.py 에서 구현.
 *
 * ============================================================
 * 노출 함수 목록
 * ============================================================
 *
 * [스캐너 열거 / 생성]
 *   enumerate_scanners()                    -> list[dict]
 *   create_scanner(index)                   -> capsule  (IScanner*)
 *   create_scanner_by_serial(serial)        -> capsule
 *
 * [스캐너 속성]
 *   scanner_id(sc_cap)                      -> dict
 *   scanner_info(sc_cap)                    -> dict
 *   scanner_frame_number(sc_cap)            -> int
 *   scanner_fps(sc_cap)                     -> float
 *   scanner_set_fps(sc_cap, fps)
 *   scanner_max_fps(sc_cap)                 -> float
 *   scanner_flash_enabled(sc_cap)           -> bool
 *   scanner_enable_flash(sc_cap, enable)
 *   scanner_texture_flash_enabled(sc_cap)   -> bool
 *   scanner_enable_texture_flash(sc_cap, v)
 *   scanner_texture_gain(sc_cap)            -> float
 *   scanner_set_texture_gain(sc_cap, v)
 *   scanner_texture_shutter_speed(sc_cap)   -> float
 *   scanner_set_texture_shutter_speed(sc_cap, v)
 *   scanner_auto_exposure_enabled(sc_cap)   -> bool
 *   scanner_enable_auto_exposure(sc_cap, v)
 *   scanner_auto_white_balance_enabled(sc_cap) -> bool
 *   scanner_enable_auto_white_balance(sc_cap, v)
 *   scanner_hw_trigger_enabled(sc_cap)      -> bool
 *   scanner_set_hw_trigger(sc_cap, v)
 *
 * [스캐너 캡처 / 트리거]
 *   scanner_capture(sc_cap, capture_texture)         -> capsule (IFrame*)
 *   scanner_capture_texture(sc_cap)                  -> tuple(ndarray|None, frame_number)
 *   scanner_fire_trigger(sc_cap, capture_texture)
 *   scanner_retrieve_frame(sc_cap, capture_texture)  -> capsule (IFrame*)
 *
 * [FrameProcessor 생성]
 *   scanner_create_processor(sc_cap, settings=None)  -> capsule (IFrameProcessor*)
 *   scanner_init_processor_desc(sc_cap)              -> dict  (default FrameProcessorDesc)
 *
 * [raw frame 조회]
 *   frame_number(frame_cap)   -> int
 *   frame_has_texture(frame_cap) -> bool
 *   frame_texture(frame_cap)  -> ndarray | None
 *
 * [mesh 재구성]
 *   processor_reconstruct(proc_cap, frame_cap)          -> artec_base_py.FrameMeshHandle
 *   processor_reconstruct_textured(proc_cap, frame_cap) -> artec_base_py.FrameMeshHandle
 *   processor_sensitivity(proc_cap)                -> float
 *   processor_set_sensitivity(proc_cap, v)
 *   processor_scanning_range(proc_cap)             -> tuple(near_mm, far_mm)
 *   processor_set_scanning_range(proc_cap, n, f)
 */

#include "artec_common.h"

#include <artec/sdk/base/TRef.h>
#include <artec/sdk/base/IRef.h>
#include <artec/sdk/base/IFrameMesh.h>
#include <artec/sdk/base/ScannerType.h>

#include <artec/sdk/capturing/IScanner.h>
#include <artec/sdk/capturing/IFrame.h>
#include <artec/sdk/capturing/IFrameProcessor.h>
#include <artec/sdk/capturing/IArrayScannerId.h>
#include <artec/sdk/capturing/ScannerInfo.h>

namespace cap = artec::sdk::capturing;

// ============================================================
// 내부 헬퍼
// ============================================================

static cap::IScanner* get_scanner(py::capsule cap)
{
    auto* p = static_cast<cap::IScanner*>(static_cast<void*>(cap));
    if (!p) throw std::runtime_error("[artec_capturing] Null scanner capsule.");
    return p;
}

static cap::IFrame* get_frame(py::capsule cap)
{
    auto* p = static_cast<cap::IFrame*>(static_cast<void*>(cap));
    if (!p) throw std::runtime_error("[artec_capturing] Null frame capsule.");
    return p;
}

static cap::IFrameProcessor* get_processor(py::capsule cap)
{
    auto* p = static_cast<cap::IFrameProcessor*>(static_cast<void*>(cap));
    if (!p) throw std::runtime_error("[artec_capturing] Null processor capsule.");
    return p;
}

// capsule 생성 (SDK가 addRef한 raw pointer를 캡슐로 wrapping)
static py::capsule make_scanner_cap(cap::IScanner* raw)
{
    return py::capsule(raw, "IScanner*",
        [](void* p){ static_cast<cap::IScanner*>(p)->release(); });
}

static py::capsule make_frame_cap(cap::IFrame* raw)
{
    return py::capsule(raw, "IFrame*",
        [](void* p){ static_cast<cap::IFrame*>(p)->release(); });
}

static py::capsule make_processor_cap(cap::IFrameProcessor* raw)
{
    return py::capsule(raw, "IFrameProcessor*",
        [](void* p){ static_cast<cap::IFrameProcessor*>(p)->release(); });
}

static std::string scanner_type_str(base::ScannerType t)
{
    switch (t)
    {
        case base::ScannerType_Small:  return "Artec S";
        case base::ScannerType_WallyL: return "Artec M/MH/MHT";
        case base::ScannerType_WallyP: return "Artec M/MH/MHT";
        case base::ScannerType_Eva:    return "Artec Eva";
        case base::ScannerType_LargeP: return "Artec L";
        case base::ScannerType_LargeL: return "Artec L";
        case base::ScannerType_Spider: return "Artec Spider";
        case base::ScannerType_LargeG: return "Artec L2";
        default:                       return "Unknown";
    }
}

static py::dict scanner_id_to_dict(const cap::ScannerId& id)
{
    py::dict d;
    d["calibration_id"]     = id.calibrationId;
    d["serial"]             = wcs_to_utf8(id.serial);
    d["name"]               = wcs_to_utf8(id.name);
    d["license"]            = wcs_to_utf8(id.license);
    d["main_camera_serial"] = wcs_to_utf8(id.mainCameraSerial);
    d["has_texture_camera"] = id.isTextureCameraAvailable;
    d["scanner_type_str"]   = scanner_type_str(id.type);
    return d;
}

// ============================================================
// 스캐너 열거 / 생성
// ============================================================

static py::list enumerate_scanners()
{
    base::TRef<cap::IArrayScannerId> list;
    check_ec(cap::enumerateScanners(&list), "enumerateScanners");
    py::list result;
    int n = list->getSize();
    const cap::ScannerId* ids = list->getPointer();
    for (int i = 0; i < n; ++i)
        result.append(scanner_id_to_dict(ids[i]));
    return result;
}

static py::capsule create_scanner(int index = 0)
{
    base::TRef<cap::IArrayScannerId> list;
    check_ec(cap::enumerateScanners(&list), "enumerateScanners");
    int n = list->getSize();
    if (index < 0 || index >= n)
        throw std::out_of_range(
            "[artec_capturing] Scanner index " + std::to_string(index)
            + " out of range (found " + std::to_string(n) + ").");
    cap::IScanner* raw = nullptr;
    check_ec(cap::createScanner(&raw, &list->getPointer()[index]), "createScanner");
    return make_scanner_cap(raw);
}

static py::capsule create_scanner_by_serial(const std::string& serial)
{
    base::TRef<cap::IArrayScannerId> list;
    check_ec(cap::enumerateScanners(&list), "enumerateScanners");
    int n = list->getSize();
    const cap::ScannerId* ids = list->getPointer();
    std::wstring target(serial.begin(), serial.end());
    for (int i = 0; i < n; ++i)
    {
        if (target == ids[i].serial)
        {
            cap::IScanner* raw = nullptr;
            check_ec(cap::createScanner(&raw, &ids[i]), "createScanner");
            return make_scanner_cap(raw);
        }
    }
    throw std::runtime_error("[artec_capturing] Serial not found: " + serial);
}

// ============================================================
// 스캐너 속성 조회
// ============================================================

static py::dict scanner_id(py::capsule sc_cap)
{
    return scanner_id_to_dict(*get_scanner(sc_cap)->getId());
}

static py::dict scanner_info(py::capsule sc_cap)
{
    const cap::ScannerInfo* si = get_scanner(sc_cap)->getInfo();
    py::dict d;
    if (si)
    {
        d["depth_map_size_x"]        = si->depthMapSizeX;
        d["depth_map_size_y"]        = si->depthMapSizeY;
        d["has_texture_camera"]      = si->isTextureCameraAvailable;
        d["texture_size_x"]          = si->textureSizeX;
        d["texture_size_y"]          = si->textureSizeY;
        d["external_sync_supported"] = si->isExternalSynchronizationSupported;
        d["gain_available"]          = si->isGainAvailable;
        d["scanner_buttons_mask"]    = si->scannerButtonsMask;
    }
    return d;
}

static int   scanner_frame_number(py::capsule sc_cap)  { return get_scanner(sc_cap)->getFrameNumber(); }
static float scanner_fps(py::capsule sc_cap)            { return get_scanner(sc_cap)->getFPS(); }
static void  scanner_set_fps(py::capsule sc_cap, float v){ get_scanner(sc_cap)->setFPS(v); }
static float scanner_max_fps(py::capsule sc_cap)         { return get_scanner(sc_cap)->getMaximumFPS(); }

static bool scanner_flash_enabled(py::capsule sc_cap)   { return get_scanner(sc_cap)->isFlashEnabled(); }
static void scanner_enable_flash(py::capsule sc_cap, bool v) { get_scanner(sc_cap)->enableFlash(v); }
static bool scanner_texture_flash_enabled(py::capsule sc_cap) { return get_scanner(sc_cap)->isTextureFlashEnabled(); }
static void scanner_enable_texture_flash(py::capsule sc_cap, bool v) { get_scanner(sc_cap)->enableTextureFlash(v); }

static float scanner_texture_gain(py::capsule sc_cap)   { return get_scanner(sc_cap)->getTextureGain(); }
static void  scanner_set_texture_gain(py::capsule sc_cap, float v) { get_scanner(sc_cap)->setTextureGain(v); }
static float scanner_texture_shutter_speed(py::capsule sc_cap) { return get_scanner(sc_cap)->getTextureShutterSpeed(); }
static void  scanner_set_texture_shutter_speed(py::capsule sc_cap, float v) { get_scanner(sc_cap)->setTextureShutterSpeed(v); }

static bool scanner_auto_exposure_enabled(py::capsule sc_cap) { return get_scanner(sc_cap)->isAutoExposureEnabled(); }
static void scanner_enable_auto_exposure(py::capsule sc_cap, bool v) { get_scanner(sc_cap)->enableAutoExposure(v); }
static bool scanner_auto_white_balance_enabled(py::capsule sc_cap) { return get_scanner(sc_cap)->isAutoWhiteBalanceEnabled(); }
static void scanner_enable_auto_white_balance(py::capsule sc_cap, bool v) { get_scanner(sc_cap)->enableAutoWhiteBalance(v); }

static bool scanner_hw_trigger_enabled(py::capsule sc_cap) { return get_scanner(sc_cap)->getUseHwTrigger(); }
static void scanner_set_hw_trigger(py::capsule sc_cap, bool v) { get_scanner(sc_cap)->setUseHwTrigger(v); }

// ============================================================
// 스캐너 캡처 / 트리거
// ============================================================

static py::capsule scanner_capture(py::capsule sc_cap, bool capture_texture = false)
{
    cap::IFrame* raw = nullptr;
    {
        py::gil_scoped_release release_gil;
        check_ec(get_scanner(sc_cap)->capture(&raw, capture_texture), "capture");
    }
    return make_frame_cap(raw);
}

static py::object scanner_capture_texture(py::capsule sc_cap)
{
    base::TRef<base::IImage> img;
    int frame_num = 0;
    {
        py::gil_scoped_release release_gil;
        check_ec(get_scanner(sc_cap)->captureTexture(&img, &frame_num), "captureTexture");
    }
    base::IImage* raw = static_cast<base::IImage*>(img);
    if (!raw || raw->getWidth() == 0 || raw->getHeight() == 0)
        return py::make_tuple(py::none(), frame_num);
    return py::make_tuple(image_to_rgb(raw), frame_num);
}

static void scanner_fire_trigger(py::capsule sc_cap, bool capture_texture = false)
{
    check_ec(get_scanner(sc_cap)->fireTrigger(capture_texture), "fireTrigger");
}

static py::capsule scanner_retrieve_frame(py::capsule sc_cap, bool capture_texture = false)
{
    cap::IFrame* raw = nullptr;
    {
        py::gil_scoped_release release_gil;
        check_ec(get_scanner(sc_cap)->retrieveFrame(&raw, capture_texture), "retrieveFrame");
    }
    return make_frame_cap(raw);
}

// ============================================================
// FrameProcessor 생성
// ============================================================

// settings: None 또는 dict (FrameProcessorSettings 필드와 동일한 키)
static py::capsule scanner_create_processor(
    py::capsule sc_cap,
    py::object  settings = py::none())
{
    cap::IScanner*         scanner  = get_scanner(sc_cap);
    cap::IFrameProcessor*  proc_raw = nullptr;

    if (settings.is_none())
    {
        check_ec(scanner->createFrameProcessor(&proc_raw, nullptr),
                 "createFrameProcessor");
        return make_processor_cap(proc_raw);
    }

    py::dict d   = settings.cast<py::dict>();
    auto geti    = [&](const char* k, int   def) -> int   { return d.contains(k) ? d[k].cast<int>()   : def; };
    auto getd    = [&](const char* k, double def) -> double{ return d.contains(k) ? d[k].cast<double>(): def; };
    auto getb    = [&](const char* k, bool  def) -> bool  { return d.contains(k) ? d[k].cast<bool>()  : def; };
    auto getf    = [&](const char* k, float def) -> float { return d.contains(k) ? d[k].cast<float>() : def; };

    cap::FrameProcessorDesc desc{};
    check_ec(scanner->initFrameProcessorDesc(&desc), "initFrameProcessorDesc");

    int   mos = geti("minimum_object_size",     0);
    int   ts  = geti("triangles_step",          0);
    double el = getd("edge_length_threshold",   0.0);
    bool  ip  = getb("interpolate",             false);
    double ml = getd("max_interpolated_length", 0.0);
    double ma = getd("max_angle",               0.0);

    if (mos > 0) desc.minimumObjectSize    = mos;
    if (ts  > 0) desc.trianglesStep        = ts;
    if (el  > 0) desc.edgeLengthThreshold  = el;
    desc.interpolate = ip;
    if (ml  > 0) desc.maxInterpolatedLength = ml;
    if (ma  > 0) desc.maxAngle             = ma;

    check_ec(scanner->createFrameProcessor(&proc_raw, &desc), "createFrameProcessor");

    float sens     = getf("sensitivity",   0.0f);
    float near_mm  = getf("range_near_mm", 0.0f);
    float far_mm   = getf("range_far_mm",  0.0f);
    if (sens   > 0.0f) check_ec(proc_raw->setSensitivity(sens), "setSensitivity");
    if (far_mm > 0.0f) check_ec(proc_raw->setScanningRange(near_mm, far_mm), "setScanningRange");

    return make_processor_cap(proc_raw);
}

static py::dict scanner_init_processor_desc(py::capsule sc_cap)
{
    cap::FrameProcessorDesc desc{};
    check_ec(get_scanner(sc_cap)->initFrameProcessorDesc(&desc), "initFrameProcessorDesc");
    py::dict d;
    d["minimum_object_size"]     = desc.minimumObjectSize;
    d["triangles_step"]          = desc.trianglesStep;
    d["edge_length_threshold"]   = desc.edgeLengthThreshold;
    d["interpolate"]             = desc.interpolate;
    d["max_interpolated_length"] = desc.maxInterpolatedLength;
    d["max_angle"]               = desc.maxAngle;
    return d;
}

// ============================================================
// raw frame 조회
// ============================================================

static int  frame_number(py::capsule frame_cap)
{
    return get_frame(frame_cap)->getFrameNumber();
}

static bool frame_has_texture(py::capsule frame_cap)
{
    const base::IImage* t = get_frame(frame_cap)->getTexture();
    return t && t->getWidth() > 0 && t->getHeight() > 0;
}

static py::object frame_texture(py::capsule frame_cap)
{
    const base::IImage* t = get_frame(frame_cap)->getTexture();
    if (!t || t->getWidth() == 0 || t->getHeight() == 0)
        return py::none();
    return image_to_rgb(t);
}

// ============================================================
// mesh 재구성 (→ IFrameMesh* capsule)
// ============================================================

// SDK가 refcount=1로 돌려준 raw pointer를 capsule로 wrapping.
// Python 쪽(artec_capturing.py)에서 artec_base.FrameMeshHandle(cap)으로 래핑.
static py::capsule make_frame_mesh_cap(base::IFrameMesh* raw)
{
    return py::capsule(raw, "IFrameMesh*", [](void* p) {
        static_cast<base::IFrameMesh*>(p)->release();
    });
}

static py::capsule processor_reconstruct(py::capsule proc_cap, py::capsule frame_cap)
{
    base::IFrameMesh* mesh_raw = nullptr;
    {
        py::gil_scoped_release release_gil;
        check_ec(
            get_processor(proc_cap)->reconstructMesh(&mesh_raw, get_frame(frame_cap)),
            "reconstructMesh");
    }
    return make_frame_mesh_cap(mesh_raw);
}

static py::capsule processor_reconstruct_textured(py::capsule proc_cap, py::capsule frame_cap)
{
    base::IFrameMesh* mesh_raw = nullptr;
    {
        py::gil_scoped_release release_gil;
        check_ec(
            get_processor(proc_cap)->reconstructAndTexturizeMesh(
                &mesh_raw, get_frame(frame_cap)),
            "reconstructAndTexturizeMesh");
    }
    return make_frame_mesh_cap(mesh_raw);
}

static float processor_sensitivity(py::capsule proc_cap)
{
    return get_processor(proc_cap)->getSensitivity();
}

static void processor_set_sensitivity(py::capsule proc_cap, float v)
{
    check_ec(get_processor(proc_cap)->setSensitivity(v), "setSensitivity");
}

static py::tuple processor_scanning_range(py::capsule proc_cap)
{
    float near_mm = 0.0f, far_mm = 0.0f;
    get_processor(proc_cap)->getScanningRange(&near_mm, &far_mm);
    return py::make_tuple(near_mm, far_mm);
}

static void processor_set_scanning_range(py::capsule proc_cap, float near_mm, float far_mm)
{
    check_ec(get_processor(proc_cap)->setScanningRange(near_mm, far_mm), "setScanningRange");
}

// ============================================================
// pybind11 module: artec_capturing_py
// ============================================================

PYBIND11_MODULE(artec_capturing_py, m)
{
    m.doc() =
        "Artec Capturing SDK raw bindings (artec_capturing_py).\n"
        "All functions are free functions operating on capsule handles.\n"
        "Python classes (ScannerHandle, etc.) live in artec_capturing.py.";

    // 스캐너 열거 / 생성
    m.def("enumerate_scanners",       &enumerate_scanners,
          "list[dict] — connected scanners.");
    m.def("create_scanner",           &create_scanner,
          py::arg("index") = 0,
          "Open scanner by index. Returns IScanner* capsule.");
    m.def("create_scanner_by_serial", &create_scanner_by_serial,
          py::arg("serial"),
          "Open scanner by serial string. Returns IScanner* capsule.");

    // 스캐너 속성
    m.def("scanner_id",               &scanner_id,           py::arg("sc_cap"));
    m.def("scanner_info",             &scanner_info,         py::arg("sc_cap"));
    m.def("scanner_frame_number",     &scanner_frame_number, py::arg("sc_cap"));
    m.def("scanner_fps",              &scanner_fps,          py::arg("sc_cap"));
    m.def("scanner_set_fps",          &scanner_set_fps,      py::arg("sc_cap"), py::arg("fps"));
    m.def("scanner_max_fps",          &scanner_max_fps,      py::arg("sc_cap"));

    m.def("scanner_flash_enabled",          &scanner_flash_enabled,          py::arg("sc_cap"));
    m.def("scanner_enable_flash",           &scanner_enable_flash,           py::arg("sc_cap"), py::arg("enable"));
    m.def("scanner_texture_flash_enabled",  &scanner_texture_flash_enabled,  py::arg("sc_cap"));
    m.def("scanner_enable_texture_flash",   &scanner_enable_texture_flash,   py::arg("sc_cap"), py::arg("enable"));

    m.def("scanner_texture_gain",             &scanner_texture_gain,             py::arg("sc_cap"));
    m.def("scanner_set_texture_gain",         &scanner_set_texture_gain,         py::arg("sc_cap"), py::arg("gain"));
    m.def("scanner_texture_shutter_speed",    &scanner_texture_shutter_speed,    py::arg("sc_cap"));
    m.def("scanner_set_texture_shutter_speed",&scanner_set_texture_shutter_speed,py::arg("sc_cap"), py::arg("ms"));

    m.def("scanner_auto_exposure_enabled",      &scanner_auto_exposure_enabled,      py::arg("sc_cap"));
    m.def("scanner_enable_auto_exposure",       &scanner_enable_auto_exposure,       py::arg("sc_cap"), py::arg("enable"));
    m.def("scanner_auto_white_balance_enabled", &scanner_auto_white_balance_enabled, py::arg("sc_cap"));
    m.def("scanner_enable_auto_white_balance",  &scanner_enable_auto_white_balance,  py::arg("sc_cap"), py::arg("enable"));

    m.def("scanner_hw_trigger_enabled", &scanner_hw_trigger_enabled, py::arg("sc_cap"));
    m.def("scanner_set_hw_trigger",     &scanner_set_hw_trigger,     py::arg("sc_cap"), py::arg("enable"));

    // 캡처 / 트리거
    m.def("scanner_capture",
          &scanner_capture,
          py::arg("sc_cap"), py::arg("capture_texture") = false,
          "Capture one raw frame. Returns IFrame* capsule. GIL released.");
    m.def("scanner_capture_texture",
          &scanner_capture_texture,
          py::arg("sc_cap"),
          "Capture texture only. Returns (ndarray|None, frame_number). GIL released.");
    m.def("scanner_fire_trigger",
          &scanner_fire_trigger,
          py::arg("sc_cap"), py::arg("capture_texture") = false);
    m.def("scanner_retrieve_frame",
          &scanner_retrieve_frame,
          py::arg("sc_cap"), py::arg("capture_texture") = false,
          "Retrieve frame after fire_trigger. Returns IFrame* capsule. GIL released.");

    // FrameProcessor 생성
    m.def("scanner_create_processor",
          &scanner_create_processor,
          py::arg("sc_cap"), py::arg("settings") = py::none(),
          "Create IFrameProcessor. settings: dict or None. Returns IFrameProcessor* capsule.");
    m.def("scanner_init_processor_desc",
          &scanner_init_processor_desc,
          py::arg("sc_cap"),
          "Return dict of default FrameProcessorDesc values for this scanner.");

    // raw frame 조회
    m.def("frame_number",     &frame_number,     py::arg("frame_cap"));
    m.def("frame_has_texture",&frame_has_texture,py::arg("frame_cap"));
    m.def("frame_texture",    &frame_texture,    py::arg("frame_cap"),
          "Raw texture (H,W,3) uint8 or None.");

    // mesh 재구성
    m.def("processor_reconstruct",
          &processor_reconstruct,
          py::arg("proc_cap"), py::arg("frame_cap"),
          "Reconstruct geometry mesh. Returns IFrameMesh* capsule. GIL released.");
    m.def("processor_reconstruct_textured",
          &processor_reconstruct_textured,
          py::arg("proc_cap"), py::arg("frame_cap"),
          "Reconstruct textured mesh. Returns IFrameMesh* capsule. GIL released.");
    m.def("processor_sensitivity",        &processor_sensitivity,        py::arg("proc_cap"));
    m.def("processor_set_sensitivity",    &processor_set_sensitivity,    py::arg("proc_cap"), py::arg("value"));
    m.def("processor_scanning_range",     &processor_scanning_range,     py::arg("proc_cap"),
          "Return (near_mm, far_mm) tuple.");
    m.def("processor_set_scanning_range", &processor_set_scanning_range,
          py::arg("proc_cap"), py::arg("near_mm"), py::arg("far_mm"));
}
