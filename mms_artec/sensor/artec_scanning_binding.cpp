/**
 * mms/sensor/artec/artec_scanning_binding.cpp
 *
 * pybind11 binding: Artec Scanning SDK -> artec_scanning_py
 *
 * 역할: SDK C++ 타입을 capsule 기반 free function으로만 노출.
 *       클래스 구조(ScanSession, ScanSessionSettings 등)는 artec_scanning.py 에서 구현.
 *
 * ============================================================
 * 노출 함수 목록
 * ============================================================
 *
 * [설정 초기화]
 *   init_scan_settings()                             -> dict
 *
 * [세션 생성 / 실행]
 *   create_session(sc_cap, settings_dict)            -> capsule (ScanProcHandle*)
 *   session_launch(sess_cap)                         -> None   (executeJob in background thread)
 *   session_join(sess_cap)                           -> None   (wait for background thread)
 *   session_is_running(sess_cap)                     -> bool
 *
 * [상태 제어]
 *   session_get_state(sess_cap)                      -> int    (ScanningState enum value)
 *   session_set_state(sess_cap, state_int)           -> None
 *
 * [감도 / 스캔 범위]
 *   session_sensitivity(sess_cap)                    -> float
 *   session_set_sensitivity(sess_cap, v)             -> None
 *   session_scanning_range(sess_cap)                 -> tuple(near, far)
 *   session_set_scanning_range(sess_cap, near, far)  -> None
 *
 * [ROI]
 *   session_set_roi(sess_cap, x, y, w, h)            -> None
 *   session_clear_roi(sess_cap)                      -> None
 *   session_get_roi(sess_cap)                        -> tuple(x,y,w,h) | None
 *
 * [결과 / 이벤트]
 *   session_result_model(sess_cap)                   -> artec_base_py.ModelHandle
 *   session_poll_events(sess_cap)                    -> list[dict]
 *       dict keys: frame_state(int), scanner_index(int), frame_mesh(FrameMeshHandle|None)
 */

#include "artec_common.h"

#include <artec/sdk/base/TRef.h>
#include <artec/sdk/base/IRef.h>
#include <artec/sdk/base/RefBase.h>
#include <artec/sdk/base/IModel.h>
#include <artec/sdk/base/IJob.h>
#include <artec/sdk/base/AlgorithmWorkset.h>
#include <artec/sdk/base/IFrameMesh.h>
#include <artec/sdk/base/Rect.h>

#include <artec/sdk/capturing/IScanner.h>

#include <artec/sdk/scanning/IScanningProcedure.h>
#include <artec/sdk/scanning/IScanningProcedureObserver.h>

#include <atomic>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>

namespace scanning = artec::sdk::scanning;
namespace cap      = artec::sdk::capturing;

// ============================================================
// FrameEventData — 큐에 저장되는 이벤트 데이터
// ============================================================

struct FrameEventData
{
    int              frame_state        = 0;      // scanning::FrameState value
    int              scanner_index      = 0;
    base::IFrameMesh* frame_mesh        = nullptr; // addRef'd; NULL이면 mesh 없음
    double           registration_error = 0.0;    // SDK native: <0 = registration failed
    bool             geometry_keyframe  = false;
    bool             texture_keyframe   = false;
};

// ============================================================
// ScanObserver — onFrameScanned 이벤트를 thread-safe 큐에 누적
// ============================================================

class ScanObserver : public scanning::ScanningProcedureObserverBase
{
public:
    // onFrameCaptured: lightweight (mesh 아직 없음, 메타데이터만)
    void onFrameCaptured(const scanning::RegistrationInfo* /*info*/) override
    {
        // 메타데이터만 필요한 경우 여기서 큐잉 가능 — 현재는 생략
    }

    // onFrameScanned: 처리 완료된 프레임 이벤트 큐잉
    void onFrameScanned(const scanning::RegistrationInfo* info) override
    {
        if (!info) return;

        FrameEventData ev;
        ev.frame_state        = static_cast<int>(info->frameState);
        ev.scanner_index      = info->scannerIndex;
        ev.registration_error = info->registrationError;
        ev.geometry_keyframe  = info->geometryKeyFrame;
        ev.texture_keyframe   = info->textureKeyFrame;

        // frame 은 콜백 반환 전까지만 유효 → addRef로 수명 연장
        if (info->frame && info->frameState == scanning::FrameState_Ok)
        {
            const_cast<base::IFrameMesh*>(info->frame)->addRef();
            ev.frame_mesh = const_cast<base::IFrameMesh*>(info->frame);
        }

        std::lock_guard<std::mutex> lk(mtx_);
        events_.push(ev);
        ++total_scanned_;
    }

    void onScanningFinished(int /*scannerIndex*/) override
    {
        finished_.store(true);
    }

    // 큐에서 이벤트 전부 꺼내기 (Python 스레드에서 호출)
    std::vector<FrameEventData> drain()
    {
        std::vector<FrameEventData> out;
        std::lock_guard<std::mutex> lk(mtx_);
        while (!events_.empty())
        {
            out.push_back(events_.front());
            events_.pop();
        }
        return out;
    }

    bool is_finished()   const { return finished_.load(); }
    int  total_scanned() const { return total_scanned_.load(); }

private:
    std::mutex                    mtx_;
    std::queue<FrameEventData>    events_;
    std::atomic<bool>             finished_{false};
    std::atomic<int>              total_scanned_{0};
};

// ============================================================
// ScanProcHandle — capsule 페이로드
// ============================================================

struct ScanProcHandle
{
    base::TRef<scanning::IScanningProcedure> proc;
    base::TRef<base::IModel>                 in_model;
    base::TRef<base::IModel>                 out_model;
    base::AlgorithmWorkset                   workset{};
    base::TRef<ScanObserver>                 observer;

    std::thread       run_thread;
    std::atomic<bool> running{false};
};

// ============================================================
// 내부 헬퍼
// ============================================================

static cap::IScanner* get_scanner(py::capsule sc_cap)
{
    auto* p = static_cast<cap::IScanner*>(static_cast<void*>(sc_cap));
    if (!p) throw std::runtime_error("[artec_scanning] Null scanner capsule.");
    return p;
}

static ScanProcHandle* get_session(py::capsule sess_cap)
{
    auto* p = static_cast<ScanProcHandle*>(static_cast<void*>(sess_cap));
    if (!p) throw std::runtime_error("[artec_scanning] Null session capsule.");
    return p;
}

// capsule 생성 — 삭제 시 스캔 중지 + 스레드 join + delete
static py::capsule make_session_cap(ScanProcHandle* h)
{
    return py::capsule(h, "ScanProcHandle*", [](void* p) {
        auto* handle = static_cast<ScanProcHandle*>(p);
        // 스캔 중이면 중지 요청
        if (handle->proc && handle->running.load())
        {
            try { handle->proc->setState(scanning::ScanningState_Stop); }
            catch (...) {}
        }
        // 스레드 종료 대기
        if (handle->run_thread.joinable())
            handle->run_thread.join();
        delete handle;
    });
}

// IFrameMesh* (addRef'd) → IFrameMesh* capsule
// Python 쪽(artec_scanning.py)에서 artec_base.FrameMeshHandle(cap)으로 래핑.
static py::capsule make_frame_mesh_cap(base::IFrameMesh* raw)
{
    return py::capsule(raw, "IFrameMesh*", [](void* p) {
        static_cast<base::IFrameMesh*>(p)->release();
    });
}

// IModel* (addRef'd) → IModel* capsule
// Python 쪽(artec_scanning.py)에서 artec_base.ModelHandle(cap)으로 래핑.
static py::capsule make_model_cap(base::IModel* raw)
{
    return py::capsule(raw, "IModel*", [](void* p) {
        static_cast<base::IModel*>(p)->release();
    });
}

// ============================================================
// 설정 초기화
// ============================================================

static py::dict init_scan_settings()
{
    scanning::ScanningProcedureSettings s{};
    check_ec(scanning::initializeScanningProcedureSettings(&s),
             "initializeScanningProcedureSettings");
    py::dict d;
    d["max_frame_count"]             = s.maxFrameCount;
    d["registration_type"]           = static_cast<int>(s.registrationType);
    d["pipeline"]                    = s.pipelineConfiguration;
    d["initial_state"]               = static_cast<int>(s.initialState);
    d["capture_texture"]             = static_cast<int>(s.captureTexture);
    d["capture_texture_frequency"]   = s.captureTextureFrequency;
    d["ignore_registration_errors"]  = s.ignoreRegistrationErrors;
    d["save_empty_surfaces"]         = s.saveEmptySurfaces;
    return d;
}

// ============================================================
// 세션 생성
// ============================================================

static py::capsule create_session(py::capsule sc_cap, py::object settings_obj)
{
    cap::IScanner* scanner = get_scanner(sc_cap);

    auto* h = new ScanProcHandle();

    try
    {
        // in / out 모델 생성
        check_ec(base::createModel(&h->in_model),  "createModel(in)");
        check_ec(base::createModel(&h->out_model), "createModel(out)");

        // workset 설정
        h->workset.in          = static_cast<base::IModel*>(h->in_model);
        h->workset.out         = static_cast<base::IModel*>(h->out_model);
        h->workset.progress    = nullptr;
        h->workset.cancellation = nullptr;
        h->workset.threadsCount = 0;

        // observer 생성 (refcount=1, attach로 소유권 이전)
        auto* obs = new ScanObserver();
        h->observer.attach(obs);

        // 설정 초기화 (SDK 기본값)
        scanning::ScanningProcedureSettings settings{};
        check_ec(scanning::initializeScanningProcedureSettings(&settings),
                 "initializeScanningProcedureSettings");

        // Python dict → settings 덮어쓰기
        if (!settings_obj.is_none())
        {
            py::dict d = settings_obj.cast<py::dict>();

            auto geti = [&](const char* k, int def) -> int {
                return d.contains(k) ? d[k].cast<int>() : def;
            };
            auto getb = [&](const char* k, bool def) -> bool {
                return d.contains(k) ? d[k].cast<bool>() : def;
            };

            int max_fc = geti("max_frame_count", -1);
            if (max_fc >= 0)
                settings.maxFrameCount = max_fc;

            int reg = geti("registration_type", -1);
            if (reg >= 0)
                settings.registrationType =
                    static_cast<scanning::RegistrationAlgorithmType>(reg);

            int pipe = geti("pipeline", -1);
            if (pipe >= 0)
                settings.pipelineConfiguration = pipe;

            int istate = geti("initial_state", -1);
            if (istate >= 0)
                settings.initialState = static_cast<scanning::ScanningState>(istate);

            int ctex = geti("capture_texture", -1);
            if (ctex >= 0)
                settings.captureTexture = static_cast<scanning::CaptureTextureMethod>(ctex);

            int ctex_freq = geti("capture_texture_frequency", -1);
            if (ctex_freq >= 0)
                settings.captureTextureFrequency = ctex_freq;

            settings.ignoreRegistrationErrors =
                getb("ignore_registration_errors", settings.ignoreRegistrationErrors);
            settings.saveEmptySurfaces =
                getb("save_empty_surfaces", settings.saveEmptySurfaces);
        }

        // 콜백 연결 (ScanProcHandle.observer 가 lifetime 보장)
        settings.scanningCallback = obs;

        // IScanningProcedure 생성 (refcount=1, attach로 소유권 이전)
        scanning::IScanningProcedure* proc_raw = nullptr;
        check_ec(scanning::createScanningProcedure(&proc_raw, scanner, &settings),
                 "createScanningProcedure");
        h->proc.attach(proc_raw);
    }
    catch (...)
    {
        delete h;
        throw;
    }

    return make_session_cap(h);
}

// ============================================================
// 세션 실행 / 대기
// ============================================================

static void session_launch(py::capsule sess_cap)
{
    auto* h = get_session(sess_cap);
    if (h->running.load()) return; // 이미 실행 중
    if (h->run_thread.joinable()) h->run_thread.join(); // 이전 스레드 정리

    h->running.store(true);

    // executeJob 은 blocking → 별도 스레드에서 실행 (GIL 없음)
    h->run_thread = std::thread([h]() {
        scanning::IScanningProcedure* proc = h->proc;
        base::executeJob(proc, &h->workset);
        h->running.store(false);
    });
}

static void session_join(py::capsule sess_cap)
{
    auto* h = get_session(sess_cap);
    // Python GIL을 해제하고 대기 (다른 Python 스레드가 진행 가능)
    {
        py::gil_scoped_release rel;
        if (h->run_thread.joinable())
            h->run_thread.join();
    }
}

static bool session_is_running(py::capsule sess_cap)
{
    return get_session(sess_cap)->running.load();
}

// ============================================================
// 상태 제어
// ============================================================

static int session_get_state(py::capsule sess_cap)
{
    return static_cast<int>(get_session(sess_cap)->proc->getState());
}

static void session_set_state(py::capsule sess_cap, int state_int)
{
    check_ec(
        get_session(sess_cap)->proc->setState(
            static_cast<scanning::ScanningState>(state_int)),
        "setState");
}

// ============================================================
// 감도 / 스캔 범위
// ============================================================

static float session_sensitivity(py::capsule sess_cap)
{
    return get_session(sess_cap)->proc->getSensitivity();
}

static void session_set_sensitivity(py::capsule sess_cap, float v)
{
    check_ec(get_session(sess_cap)->proc->setSensitivity(v), "setSensitivity");
}

static py::tuple session_scanning_range(py::capsule sess_cap)
{
    float near_v = 0.0f, far_v = 0.0f;
    check_ec(get_session(sess_cap)->proc->getScanningRange(&near_v, &far_v),
             "getScanningRange");
    return py::make_tuple(near_v, far_v);
}

static void session_set_scanning_range(py::capsule sess_cap, float near_v, float far_v)
{
    check_ec(get_session(sess_cap)->proc->setScanningRange(near_v, far_v),
             "setScanningRange");
}

// ============================================================
// ROI
// ============================================================

static void session_set_roi(
    py::capsule sess_cap, float x, float y, float w, float h)
{
    base::RectF rect(x, y, x + w, y + h);
    check_ec(get_session(sess_cap)->proc->setROI(&rect), "setROI");
}

static void session_clear_roi(py::capsule sess_cap)
{
    check_ec(get_session(sess_cap)->proc->setROI(nullptr), "setROI(null)");
}

static py::object session_get_roi(py::capsule sess_cap)
{
    base::RectF rect;
    auto ec = get_session(sess_cap)->proc->getROI(&rect);
    if (ec != base::ErrorCode_OK)
        return py::none();
    float w = rect.right  - rect.left;
    float h = rect.bottom - rect.top;
    if (w <= 0.0f || h <= 0.0f)
        return py::none();
    return py::make_tuple(rect.left, rect.top, w, h);
}

// ============================================================
// 결과 모델
// ============================================================

static py::object session_result_model(py::capsule sess_cap)
{
    auto* h   = get_session(sess_cap);
    auto* raw = static_cast<base::IModel*>(h->out_model);
    if (!raw) return py::none();
    raw->addRef(); // capsule 이 소유할 ref
    return make_model_cap(raw);
}

// ============================================================
// 이벤트 폴링
// ============================================================

static py::list session_poll_events(py::capsule sess_cap)
{
    auto* h      = get_session(sess_cap);
    auto* obs    = static_cast<ScanObserver*>(h->observer);
    auto  events = obs->drain();

    py::list result;
    for (auto& ev : events)
    {
        py::dict d;
        d["frame_state"]        = ev.frame_state;
        d["scanner_index"]      = ev.scanner_index;
        d["registration_error"] = ev.registration_error;
        d["geometry_keyframe"]  = ev.geometry_keyframe;
        d["texture_keyframe"]   = ev.texture_keyframe;
        // frame_mesh: IFrameMesh* capsule or None
        // Python 쪽(artec_scanning.py)에서 artec_base.FrameMeshHandle(cap)으로 래핑.
        if (ev.frame_mesh)
            d["frame_mesh"] = make_frame_mesh_cap(ev.frame_mesh);
        else
            d["frame_mesh"] = py::none();
        result.append(d);
    }
    return result;
}

// ============================================================
// pybind11 module: artec_scanning_py
// ============================================================

PYBIND11_MODULE(artec_scanning_py, m)
{
    m.doc() =
        "Artec Scanning SDK raw bindings (artec_scanning_py).\n"
        "All functions are free functions operating on capsule handles.\n"
        "Python classes (ScanSession, ScanSessionSettings, etc.) live in artec_scanning.py.";

    m.def("init_scan_settings",
          &init_scan_settings,
          "Return dict of default ScanningProcedureSettings values.");

    m.def("create_session",
          &create_session,
          py::arg("scanner_capsule"),
          py::arg("settings") = py::none(),
          "Create scanning session. scanner_capsule: IScanner* from artec_capturing_py.\n"
          "settings: dict or None. Returns ScanProcHandle* capsule.");

    m.def("session_launch",
          &session_launch,
          py::arg("sess_cap"),
          "Launch executeJob in background thread. No-op if already running.");

    m.def("session_join",
          &session_join,
          py::arg("sess_cap"),
          "Wait for background thread to finish. Releases GIL while waiting.");

    m.def("session_is_running",
          &session_is_running,
          py::arg("sess_cap"),
          "Return True if executeJob is still running in background.");

    m.def("session_get_state",
          &session_get_state,
          py::arg("sess_cap"),
          "Return current ScanningState as int.");

    m.def("session_set_state",
          &session_set_state,
          py::arg("sess_cap"), py::arg("state_int"),
          "Set ScanningState. Can be called while scanning is in progress.");

    m.def("session_sensitivity",
          &session_sensitivity,
          py::arg("sess_cap"),
          "Return current sensitivity (0.0–1.0).");

    m.def("session_set_sensitivity",
          &session_set_sensitivity,
          py::arg("sess_cap"), py::arg("value"),
          "Set sensitivity (0.0–1.0).");

    m.def("session_scanning_range",
          &session_scanning_range,
          py::arg("sess_cap"),
          "Return (near_mm, far_mm) tuple.");

    m.def("session_set_scanning_range",
          &session_set_scanning_range,
          py::arg("sess_cap"), py::arg("near_mm"), py::arg("far_mm"),
          "Set scanning range in mm.");

    m.def("session_set_roi",
          &session_set_roi,
          py::arg("sess_cap"),
          py::arg("x"), py::arg("y"), py::arg("w"), py::arg("h"),
          "Set ROI as (x, y, width, height).");

    m.def("session_clear_roi",
          &session_clear_roi,
          py::arg("sess_cap"),
          "Disable ROI (setROI(NULL)).");

    m.def("session_get_roi",
          &session_get_roi,
          py::arg("sess_cap"),
          "Return (x, y, w, h) or None if ROI is not set.");

    m.def("session_result_model",
          &session_result_model,
          py::arg("sess_cap"),
          "Return artec_base_py.ModelHandle wrapping the output model (out_model).");

    m.def("session_poll_events",
          &session_poll_events,
          py::arg("sess_cap"),
          "Drain and return list of onFrameScanned events.\n"
          "Each dict: {frame_state, scanner_index, frame_mesh}.");
}
