/**
 * mms/sensor/artec/artec_algorithm_binding.cpp
 *
 * pybind11 binding: Artec Algorithm SDK -> artec_algorithm_py
 *
 * 역할: SDK C++ 타입을 capsule 기반 free function으로만 노출.
 *       클래스 구조(Algorithms, *SettingsDTO 등)는 artec_algorithm.py 에서 구현.
 *
 * ============================================================
 * 실행 패턴
 * ============================================================
 *   - 모든 알고리즘은 in == out (동일 IModel*) 로 executeJob 실행 → in-place 수정.
 *   - 반환값: 동일 IModel* 에 addRef 한 새 capsule.
 *   - GIL: executeJob 전에 gil_scoped_release.
 *
 * ============================================================
 * 노출 함수 목록
 * ============================================================
 *
 * [권한 / 유틸]
 *   check_permission()                          -> bool
 *   get_scanner_type(sc_cap)                    -> int   (ScannerType enum value)
 *
 * [알고리즘]
 *   serial_registration(model_cap, settings)    -> IModel* capsule
 *   global_registration(model_cap, settings)    -> IModel* capsule
 *   outliers_removal(model_cap, settings)       -> IModel* capsule
 *   small_objects_filter(model_cap, settings)   -> IModel* capsule
 *   fast_fusion(model_cap, settings)            -> IModel* capsule
 *   poisson_fusion(model_cap, settings)         -> IModel* capsule
 *   mesh_simplify(model_cap, settings)          -> IModel* capsule
 *   fast_mesh_simplify(model_cap, settings)     -> IModel* capsule
 *   texturize(model_cap, settings)              -> IModel* capsule
 *   auto_align(model_cap, settings)             -> IModel* capsule
 *   loop_closure(model_cap, settings)           -> IModel* capsule
 *
 * [설정 초기화 (SDK 스캐너 기본값 반환)]
 *   init_fast_fusion_settings(scanner_type_int)                        -> dict
 *   init_poisson_fusion_settings(scanner_type_int)                     -> dict
 *   init_texturization_settings(scanner_type_int)                      -> dict
 *   init_small_objects_filter_settings(scanner_type_int)               -> dict
 *   init_mesh_simplification_settings(scanner_type_int, simplify_type) -> dict
 *   init_fast_mesh_simplification_settings(scanner_type_int)           -> dict
 *   init_outliers_removal_settings(scanner_type_int)                   -> dict
 *
 *   settings 파라미터: None 또는 dict (아래 각 함수 주석 참조)
 */

#include "artec_common.h"

#include <artec/sdk/base/TRef.h>
#include <artec/sdk/base/IRef.h>
#include <artec/sdk/base/IModel.h>
#include <artec/sdk/base/IJob.h>
#include <artec/sdk/base/AlgorithmWorkset.h>
#include <artec/sdk/base/ScannerType.h>

#include <artec/sdk/capturing/IScanner.h>
#include <artec/sdk/capturing/IArrayScannerId.h>

#include <artec/sdk/algorithms/Algorithms.h>
#include <artec/sdk/algorithms/IAlgorithm.h>

namespace algo = artec::sdk::algorithms;
namespace cap  = artec::sdk::capturing;

// ============================================================
// 내부 헬퍼
// ============================================================

static base::IModel* get_model(py::capsule c)
{
    auto* p = static_cast<base::IModel*>(static_cast<void*>(c));
    if (!p) throw std::runtime_error("[artec_algorithm] Null IModel capsule.");
    return p;
}

static py::capsule make_model_cap(base::IModel* raw)
{
    return py::capsule(raw, "IModel*",
        [](void* p){ static_cast<base::IModel*>(p)->release(); });
}

// Core: 알고리즘 실행 (in != out, GIL 해제)
// algo_raw: createXxx 가 반환한 IAlgorithm* (refcount=1 추정)
// 반환: 알고리즘 결과가 담긴 새 out IModel* capsule
//
// AlgorithmWorkset 주의사항:
//   "Output model is empty in the most cases."
//   ws.in == ws.out 로 in-place 실행 시 ErrorCode_ArgumentInvalid 발생.
//   ws.out 은 항상 별도의 빈 모델이어야 한다.
static py::capsule run_algo(algo::IAlgorithm* algo_raw, py::capsule model_cap)
{
    base::TRef<algo::IAlgorithm> algo_ref;
    algo_ref.attach(algo_raw);

    auto* model_in = get_model(model_cap);

    // 별도의 빈 출력 모델 생성
    base::TRef<base::IModel> out_model;
    check_ec(base::createModel(&out_model), "createModel(out)");

    base::AlgorithmWorkset ws{};
    ws.in           = model_in;
    ws.out          = static_cast<base::IModel*>(out_model);
    ws.progress     = nullptr;
    ws.cancellation = nullptr;
    ws.threadsCount = 0;

    {
        py::gil_scoped_release rel;
        check_ec(base::executeJob(algo_raw, &ws), "executeJob");
    }

    base::IModel* out_raw = static_cast<base::IModel*>(out_model);
    out_raw->addRef();
    return make_model_cap(out_raw);
}

// ============================================================
// 권한 / 유틸
// ============================================================

static bool check_permission()
{
    return algo::checkAlgorithmsPermission();
}

// IScanner* capsule 에서 ScannerType 정수 추출
static int get_scanner_type(py::capsule sc_cap)
{
    auto* scanner = static_cast<cap::IScanner*>(static_cast<void*>(sc_cap));
    if (!scanner)
        throw std::runtime_error("[artec_algorithm] Null scanner capsule.");
    return static_cast<int>(scanner->getId()->type);
}

// ============================================================
// settings dict 파싱 헬퍼 매크로
// ============================================================

#define GET_INT(d, key, dst)   if ((d).contains(key)) (dst) = static_cast<std::remove_reference<decltype(dst)>::type>((d)[key].cast<int>())
#define GET_FLOAT(d, key, dst) if ((d).contains(key)) (dst) = (d)[key].cast<float>()
#define GET_BOOL(d, key, dst)  if ((d).contains(key)) (dst) = (d)[key].cast<bool>()

// ============================================================
// Serial Registration
// ============================================================
// settings keys: scanner_type(int), registration_type(int)

static py::capsule serial_registration(py::capsule model_cap, py::object s_obj)
{
    algo::SerialRegistrationSettings s{};
    s.scannerType      = base::ScannerType_Unknown;
    s.registrationType = algo::SerialRegistrationType_Fine;

    if (!s_obj.is_none())
    {
        py::dict d = s_obj.cast<py::dict>();
        GET_INT(d, "scanner_type",      s.scannerType);
        GET_INT(d, "registration_type", s.registrationType);
    }

    algo::IAlgorithm* ar = nullptr;
    check_ec(algo::createSerialRegistrationAlgorithm(&ar, &s),
             "createSerialRegistrationAlgorithm");
    return run_algo(ar, model_cap);
}

// ============================================================
// Global Registration
// ============================================================
// settings keys: scanner_type(int), registration_type(int)

static py::capsule global_registration(py::capsule model_cap, py::object s_obj)
{
    algo::GlobalRegistrationSettings s{};
    s.scannerType      = base::ScannerType_Unknown;
    s.registrationType = algo::GlobalRegistrationType_Geometry;

    if (!s_obj.is_none())
    {
        py::dict d = s_obj.cast<py::dict>();
        GET_INT(d, "scanner_type",      s.scannerType);
        GET_INT(d, "registration_type", s.registrationType);
    }

    algo::IAlgorithm* ar = nullptr;
    check_ec(algo::createGlobalRegistrationAlgorithm(&ar, &s),
             "createGlobalRegistrationAlgorithm");
    return run_algo(ar, model_cap);
}

// ============================================================
// Outliers Removal
// ============================================================
// settings keys: scanner_type(int), standard_deviation_multiplier(float), resolution(float)

static py::capsule outliers_removal(py::capsule model_cap, py::object s_obj)
{
    base::ScannerType st = base::ScannerType_Unknown;
    if (!s_obj.is_none())
    {
        py::dict d = s_obj.cast<py::dict>();
        GET_INT(d, "scanner_type", st);
    }

    algo::OutliersRemovalSettings s{};
    check_ec(algo::initializeOutliersRemovalSettings(&s, st),
             "initializeOutliersRemovalSettings");

    if (!s_obj.is_none())
    {
        py::dict d = s_obj.cast<py::dict>();
        GET_FLOAT(d, "standard_deviation_multiplier", s.standardDeviationMultiplier);
        GET_FLOAT(d, "resolution",                   s.resolution);
    }

    algo::IAlgorithm* ar = nullptr;
    check_ec(algo::createOutliersRemovalAlgorithm(&ar, &s),
             "createOutliersRemovalAlgorithm");
    return run_algo(ar, model_cap);
}

// ============================================================
// Small Objects Filter
// ============================================================
// settings keys: scanner_type(int), filter_type(int), filter_threshold(int)

static py::capsule small_objects_filter(py::capsule model_cap, py::object s_obj)
{
    base::ScannerType st = base::ScannerType_Unknown;
    if (!s_obj.is_none())
    {
        py::dict d = s_obj.cast<py::dict>();
        GET_INT(d, "scanner_type", st);
    }

    algo::SmallObjectsFilterSettings s{};
    check_ec(algo::initializeSmallObjectsFilterSettings(&s, st),
             "initializeSmallObjectsFilterSettings");

    if (!s_obj.is_none())
    {
        py::dict d = s_obj.cast<py::dict>();
        GET_INT(d, "filter_type",      s.filterType);
        GET_INT(d, "filter_threshold", s.filterThreshold);
    }

    algo::IAlgorithm* ar = nullptr;
    check_ec(algo::createSmallObjectsFilterAlgorithm(&ar, &s),
             "createSmallObjectsFilterAlgorithm");
    return run_algo(ar, model_cap);
}

// ============================================================
// Fast Fusion
// ============================================================
// settings keys: scanner_type(int), resolution(float), radius(int), generate_normals(bool)

static py::capsule fast_fusion(py::capsule model_cap, py::object s_obj)
{
    base::ScannerType st = base::ScannerType_Unknown;
    if (!s_obj.is_none())
    {
        py::dict d = s_obj.cast<py::dict>();
        GET_INT(d, "scanner_type", st);
    }

    algo::FastFusionSettings s{};
    check_ec(algo::initializeFastFusionSettings(&s, st),
             "initializeFastFusionSettings");

    if (!s_obj.is_none())
    {
        py::dict d = s_obj.cast<py::dict>();
        GET_FLOAT(d, "resolution",       s.resolution);
        GET_INT  (d, "radius",           s.radius);
        GET_BOOL (d, "generate_normals", s.generateNormals);
    }

    algo::IAlgorithm* ar = nullptr;
    check_ec(algo::createFastFusionAlgorithm(&ar, &s),
             "createFastFusionAlgorithm");
    return run_algo(ar, model_cap);
}

// ============================================================
// Poisson Fusion
// ============================================================
// settings keys: scanner_type, fusion_type, fill_type, resolution, max_hole_radius,
//                remove_targets, target_inner_size, target_outer_size,
//                generate_normals, input_filter_type

static py::capsule poisson_fusion(py::capsule model_cap, py::object s_obj)
{
    base::ScannerType st = base::ScannerType_Unknown;
    if (!s_obj.is_none())
    {
        py::dict d = s_obj.cast<py::dict>();
        GET_INT(d, "scanner_type", st);
    }

    algo::PoissonFusionSettings s{};
    check_ec(algo::initializePoissonFusionSettings(&s, st),
             "initializePoissonFusionSettings");

    if (!s_obj.is_none())
    {
        py::dict d = s_obj.cast<py::dict>();
        GET_INT  (d, "fusion_type",        s.fusionType);
        GET_INT  (d, "fill_type",          s.fillType);
        GET_FLOAT(d, "resolution",         s.resolution);
        GET_FLOAT(d, "max_hole_radius",    s.maxHoleRadius);
        GET_BOOL (d, "remove_targets",     s.removeTargets);
        GET_FLOAT(d, "target_inner_size",  s.targetInnerSize);
        GET_FLOAT(d, "target_outer_size",  s.targetOuterSize);
        GET_BOOL (d, "generate_normals",   s.generateNormals);
        GET_INT  (d, "input_filter_type",  s.inputFilterType);
    }

    algo::IAlgorithm* ar = nullptr;
    check_ec(algo::createPoissonFusionAlgorithm(&ar, &s),
             "createPoissonFusionAlgorithm");
    return run_algo(ar, model_cap);
}

// ============================================================
// Mesh Simplification
// ============================================================
// settings keys: scanner_type, simplify_type, simplify_metrics, triangle_number,
//                keep_boundary, angle_threshold, remesh_edge_threshold, error

static py::capsule mesh_simplify(py::capsule model_cap, py::object s_obj)
{
    base::ScannerType st    = base::ScannerType_Unknown;
    algo::SimplifyType stype = algo::SimplifyType_TriangleQuantity;

    if (!s_obj.is_none())
    {
        py::dict d = s_obj.cast<py::dict>();
        GET_INT(d, "scanner_type",  st);
        GET_INT(d, "simplify_type", stype);
    }

    algo::MeshSimplificationSettings s{};
    check_ec(algo::initializeMeshSimplificationSettings(&s, st, stype),
             "initializeMeshSimplificationSettings");

    if (!s_obj.is_none())
    {
        py::dict d = s_obj.cast<py::dict>();
        GET_INT  (d, "simplify_metrics",     s.simplifyMetrics);
        GET_INT  (d, "triangle_number",      s.triangleNumber);
        GET_BOOL (d, "keep_boundary",        s.keepBoundary);
        GET_FLOAT(d, "angle_threshold",      s.angleThreshold);
        GET_FLOAT(d, "remesh_edge_threshold",s.remeshEdgeThreshold);
        GET_FLOAT(d, "error",                s.error);
    }

    algo::IAlgorithm* ar = nullptr;
    check_ec(algo::createMeshSimplificationAlgorithm(&ar, &s),
             "createMeshSimplificationAlgorithm");
    return run_algo(ar, model_cap);
}

// ============================================================
// Fast Mesh Simplification
// ============================================================
// settings keys: scanner_type, triangle_number, keep_boundary,
//                enable_additional_criteria, enable_distance_threshold,
//                distance_threshold, enable_angle_threshold, angle_threshold,
//                enable_aspect_ratio_threshold, aspect_ratio_threshold

static py::capsule fast_mesh_simplify(py::capsule model_cap, py::object s_obj)
{
    base::ScannerType st = base::ScannerType_Unknown;
    if (!s_obj.is_none())
    {
        py::dict d = s_obj.cast<py::dict>();
        GET_INT(d, "scanner_type", st);
    }

    algo::FastMeshSimplificationSettings s{};
    check_ec(algo::initializeFastMeshSimplificationSettings(&s, st),
             "initializeFastMeshSimplificationSettings");

    if (!s_obj.is_none())
    {
        py::dict d = s_obj.cast<py::dict>();
        GET_INT  (d, "triangle_number",              s.triangleNumber);
        GET_BOOL (d, "keep_boundary",                s.keepBoundary);
        GET_BOOL (d, "enable_additional_criteria",   s.enableAdditionalCriteria);
        GET_BOOL (d, "enable_distance_threshold",    s.enableDistanceThreshold);
        GET_FLOAT(d, "distance_threshold",           s.distanceThreshold);
        GET_BOOL (d, "enable_angle_threshold",       s.enableAngleThreshold);
        GET_FLOAT(d, "angle_threshold",              s.angleThreshold);
        GET_BOOL (d, "enable_aspect_ratio_threshold",s.enableAspectRatioThreshold);
        GET_FLOAT(d, "aspect_ratio_threshold",       s.aspectRatioThreshold);
    }

    algo::IAlgorithm* ar = nullptr;
    check_ec(algo::createFastMeshSimplificationAlgorithm(&ar, &s),
             "createFastMeshSimplificationAlgorithm");
    return run_algo(ar, model_cap);
}

// ============================================================
// Texturization
// ============================================================
// settings keys: scanner_type, texturize_type, texturize_resolution,
//                enable_background_segmentation, enable_ambient_lighting_compensation,
//                atlas_unfolding_polygon_limit, enable_texture_inpainting,
//                use_texture_normalization, input_filter_type

static py::capsule texturize(py::capsule model_cap, py::object s_obj)
{
    base::ScannerType st = base::ScannerType_Unknown;
    if (!s_obj.is_none())
    {
        py::dict d = s_obj.cast<py::dict>();
        GET_INT(d, "scanner_type", st);
    }

    algo::TexturizationSettings s{};
    check_ec(algo::initializeTexturizationSettings(&s, st),
             "initializeTexturizationSettings");

    if (!s_obj.is_none())
    {
        py::dict d = s_obj.cast<py::dict>();
        GET_INT  (d, "texturize_type",                      s.texturizeType);
        GET_INT  (d, "texturize_resolution",                s.texturizeResolution);
        GET_BOOL (d, "enable_background_segmentation",      s.enableBackgroundSegmentation);
        GET_BOOL (d, "enable_ambient_lighting_compensation",s.enableAmbientLightingCompensation);
        GET_INT  (d, "atlas_unfolding_polygon_limit",       s.atlasUnfoldingPolygonLimit);
        GET_BOOL (d, "enable_texture_inpainting",           s.enableTextureInpainting);
        GET_BOOL (d, "use_texture_normalization",           s.useTextureNormalization);
        GET_INT  (d, "input_filter_type",                   s.inputFilterType);
    }

    algo::IAlgorithm* ar = nullptr;
    check_ec(algo::createTexturizationAlgorithm(&ar, &s),
             "createTexturizationAlgorithm");
    return run_algo(ar, model_cap);
}

// ============================================================
// Auto Align
// ============================================================
// settings keys: scanner_type(int)

static py::capsule auto_align(py::capsule model_cap, py::object s_obj)
{
    algo::AutoAlignSettings s{};
    s.scannerType = base::ScannerType_Unknown;

    if (!s_obj.is_none())
    {
        py::dict d = s_obj.cast<py::dict>();
        GET_INT(d, "scanner_type", s.scannerType);
    }

    algo::IAlgorithm* ar = nullptr;
    check_ec(algo::createAutoalignAlgorithm(&ar, &s),
             "createAutoalignAlgorithm");
    return run_algo(ar, model_cap);
}

// ============================================================
// Loop Closure
// ============================================================
// settings keys: scanner_type(int)

static py::capsule loop_closure(py::capsule model_cap, py::object s_obj)
{
    algo::LoopClosureSettings s{};
    s.scannerType = base::ScannerType_Unknown;

    if (!s_obj.is_none())
    {
        py::dict d = s_obj.cast<py::dict>();
        GET_INT(d, "scanner_type", s.scannerType);
    }

    algo::IAlgorithm* ar = nullptr;
    check_ec(algo::createLoopClosureAlgorithm(&ar, &s),
             "createLoopClosureAlgorithm");
    return run_algo(ar, model_cap);
}

// ============================================================
// 설정 초기화 함수 (SDK 기본값 → dict)
// ============================================================

static py::dict init_fast_fusion_settings(int scanner_type_int)
{
    algo::FastFusionSettings s{};
    auto st = static_cast<base::ScannerType>(scanner_type_int);
    check_ec(algo::initializeFastFusionSettings(&s, st), "initializeFastFusionSettings");
    py::dict d;
    d["scanner_type"]     = static_cast<int>(s.scannerType);
    d["resolution"]       = s.resolution;
    d["radius"]           = s.radius;
    d["generate_normals"] = s.generateNormals;
    return d;
}

static py::dict init_poisson_fusion_settings(int scanner_type_int)
{
    algo::PoissonFusionSettings s{};
    auto st = static_cast<base::ScannerType>(scanner_type_int);
    check_ec(algo::initializePoissonFusionSettings(&s, st), "initializePoissonFusionSettings");
    py::dict d;
    d["scanner_type"]       = static_cast<int>(s.scannerType);
    d["fusion_type"]        = static_cast<int>(s.fusionType);
    d["fill_type"]          = static_cast<int>(s.fillType);
    d["resolution"]         = s.resolution;
    d["max_hole_radius"]    = s.maxHoleRadius;
    d["remove_targets"]     = s.removeTargets;
    d["target_inner_size"]  = s.targetInnerSize;
    d["target_outer_size"]  = s.targetOuterSize;
    d["generate_normals"]   = s.generateNormals;
    d["input_filter_type"]  = static_cast<int>(s.inputFilterType);
    return d;
}

static py::dict init_texturization_settings(int scanner_type_int)
{
    algo::TexturizationSettings s{};
    auto st = static_cast<base::ScannerType>(scanner_type_int);
    check_ec(algo::initializeTexturizationSettings(&s, st), "initializeTexturizationSettings");
    py::dict d;
    d["scanner_type"]                         = static_cast<int>(s.scannerType);
    d["texturize_type"]                       = static_cast<int>(s.texturizeType);
    d["texturize_resolution"]                 = static_cast<int>(s.texturizeResolution);
    d["enable_background_segmentation"]       = s.enableBackgroundSegmentation;
    d["enable_ambient_lighting_compensation"] = s.enableAmbientLightingCompensation;
    d["atlas_unfolding_polygon_limit"]        = s.atlasUnfoldingPolygonLimit;
    d["enable_texture_inpainting"]            = s.enableTextureInpainting;
    d["use_texture_normalization"]            = s.useTextureNormalization;
    d["input_filter_type"]                    = static_cast<int>(s.inputFilterType);
    return d;
}

static py::dict init_small_objects_filter_settings(int scanner_type_int)
{
    algo::SmallObjectsFilterSettings s{};
    auto st = static_cast<base::ScannerType>(scanner_type_int);
    check_ec(algo::initializeSmallObjectsFilterSettings(&s, st),
             "initializeSmallObjectsFilterSettings");
    py::dict d;
    d["scanner_type"]     = static_cast<int>(s.scannerType);
    d["filter_type"]      = static_cast<int>(s.filterType);
    d["filter_threshold"] = s.filterThreshold;
    return d;
}

static py::dict init_mesh_simplification_settings(int scanner_type_int, int simplify_type_int)
{
    algo::MeshSimplificationSettings s{};
    auto st   = static_cast<base::ScannerType>(scanner_type_int);
    auto stype = static_cast<algo::SimplifyType>(simplify_type_int);
    check_ec(algo::initializeMeshSimplificationSettings(&s, st, stype),
             "initializeMeshSimplificationSettings");
    py::dict d;
    d["scanner_type"]       = static_cast<int>(s.scannerType);
    d["simplify_type"]      = static_cast<int>(s.simplifyType);
    d["simplify_metrics"]   = static_cast<int>(s.simplifyMetrics);
    d["triangle_number"]    = s.triangleNumber;
    d["keep_boundary"]      = s.keepBoundary;
    d["angle_threshold"]    = s.angleThreshold;
    d["remesh_edge_threshold"] = s.remeshEdgeThreshold;
    d["error"]              = s.error;
    return d;
}

static py::dict init_fast_mesh_simplification_settings(int scanner_type_int)
{
    algo::FastMeshSimplificationSettings s{};
    auto st = static_cast<base::ScannerType>(scanner_type_int);
    check_ec(algo::initializeFastMeshSimplificationSettings(&s, st),
             "initializeFastMeshSimplificationSettings");
    py::dict d;
    d["scanner_type"]                  = static_cast<int>(s.scannerType);
    d["triangle_number"]               = s.triangleNumber;
    d["keep_boundary"]                 = s.keepBoundary;
    d["enable_additional_criteria"]    = s.enableAdditionalCriteria;
    d["enable_distance_threshold"]     = s.enableDistanceThreshold;
    d["distance_threshold"]            = s.distanceThreshold;
    d["enable_angle_threshold"]        = s.enableAngleThreshold;
    d["angle_threshold"]               = s.angleThreshold;
    d["enable_aspect_ratio_threshold"] = s.enableAspectRatioThreshold;
    d["aspect_ratio_threshold"]        = s.aspectRatioThreshold;
    return d;
}

static py::dict init_outliers_removal_settings(int scanner_type_int)
{
    algo::OutliersRemovalSettings s{};
    auto st = static_cast<base::ScannerType>(scanner_type_int);
    check_ec(algo::initializeOutliersRemovalSettings(&s, st),
             "initializeOutliersRemovalSettings");
    py::dict d;
    d["scanner_type"]                    = static_cast<int>(s.scannerType);
    d["standard_deviation_multiplier"]   = s.standardDeviationMultiplier;
    d["resolution"]                      = s.resolution;
    return d;
}

// ============================================================
// pybind11 module: artec_algorithm_py
// ============================================================

PYBIND11_MODULE(artec_algorithm_py, m)
{
    m.doc() =
        "Artec Algorithm SDK raw bindings (artec_algorithm_py).\n"
        "All functions are free functions operating on capsule handles.\n"
        "Python classes (Algorithms, *SettingsDTO, enums) live in artec_algorithm.py.\n"
        "\n"
        "Algorithm pattern: in == out (in-place). Returns same IModel* capsule with addRef.";

    // 권한 / 유틸
    m.def("check_permission", &check_permission,
          "Return True if algorithms are available on this machine.");
    m.def("get_scanner_type", &get_scanner_type,
          py::arg("scanner_capsule"),
          "Return ScannerType int from an IScanner* capsule.");

    // 알고리즘
    m.def("serial_registration",  &serial_registration,
          py::arg("model_cap"), py::arg("settings") = py::none());
    m.def("global_registration",  &global_registration,
          py::arg("model_cap"), py::arg("settings") = py::none());
    m.def("outliers_removal",     &outliers_removal,
          py::arg("model_cap"), py::arg("settings") = py::none());
    m.def("small_objects_filter", &small_objects_filter,
          py::arg("model_cap"), py::arg("settings") = py::none());
    m.def("fast_fusion",          &fast_fusion,
          py::arg("model_cap"), py::arg("settings") = py::none());
    m.def("poisson_fusion",       &poisson_fusion,
          py::arg("model_cap"), py::arg("settings") = py::none());
    m.def("mesh_simplify",        &mesh_simplify,
          py::arg("model_cap"), py::arg("settings") = py::none());
    m.def("fast_mesh_simplify",   &fast_mesh_simplify,
          py::arg("model_cap"), py::arg("settings") = py::none());
    m.def("texturize",            &texturize,
          py::arg("model_cap"), py::arg("settings") = py::none());
    m.def("auto_align",           &auto_align,
          py::arg("model_cap"), py::arg("settings") = py::none());
    m.def("loop_closure",         &loop_closure,
          py::arg("model_cap"), py::arg("settings") = py::none());

    // 설정 초기화
    m.def("init_fast_fusion_settings",
          &init_fast_fusion_settings,
          py::arg("scanner_type") = 0);
    m.def("init_poisson_fusion_settings",
          &init_poisson_fusion_settings,
          py::arg("scanner_type") = 0);
    m.def("init_texturization_settings",
          &init_texturization_settings,
          py::arg("scanner_type") = 0);
    m.def("init_small_objects_filter_settings",
          &init_small_objects_filter_settings,
          py::arg("scanner_type") = 0);
    m.def("init_mesh_simplification_settings",
          &init_mesh_simplification_settings,
          py::arg("scanner_type") = 0,
          py::arg("simplify_type") = 0);
    m.def("init_fast_mesh_simplification_settings",
          &init_fast_mesh_simplification_settings,
          py::arg("scanner_type") = 0);
    m.def("init_outliers_removal_settings",
          &init_outliers_removal_settings,
          py::arg("scanner_type") = 0);
}
