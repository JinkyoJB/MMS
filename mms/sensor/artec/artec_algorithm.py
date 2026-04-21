# mms/sensor/artec/artec_algorithm.py
#
# Artec Algorithm SDK Python 래퍼.
# artec_algorithm_py (C++ pybind11)는 capsule 기반 free function만 노출.
# 이 파일에서 클래스 계층(Algorithms, *SettingsDTO, enums)과
# 고수준 헬퍼(run_pipeline, visualize_model_result, verify_artec_algorithm)를
# Python으로 구현한다.
#
# Exposes
# -------
#   Enums (IntEnum / IntFlag)
#     ScannerType, SerialRegistrationType, GlobalRegistrationType,
#     PoissonFusionType, FillHolesType, SmallObjectsFilterType,
#     SimplifyType, SimplifyMetric, TexturizeType, TexturizeResolution,
#     InputFilter
#
#   DTOs (dataclass, default() classmethod)
#     SerialRegistrationSettingsDTO, GlobalRegistrationSettingsDTO,
#     OutliersRemovalSettingsDTO, SmallObjectsFilterSettingsDTO,
#     FastFusionSettingsDTO, PoissonFusionSettingsDTO,
#     MeshSimplificationSettingsDTO, FastMeshSimplificationSettingsDTO,
#     TexturizationSettingsDTO, PipelineSettingsDTO
#
#   Algorithm runner
#     Algorithms   — static methods: serial_registration, global_registration,
#                    outliers_removal, small_objects_filter, fast_fusion,
#                    poisson_fusion, mesh_simplify, fast_mesh_simplify,
#                    texturize, auto_align, loop_closure, run_pipeline,
#                    is_available()
#
#   Visualization / verify
#     visualize_model_result(model_handle)
#     verify_artec_algorithm(artec_client, visualize=True)

from __future__ import annotations

import os
import sys
import time
from dataclasses import dataclass, field
from enum import IntEnum
from pathlib import Path
from typing import Optional

import numpy as np

_ARTEC_SDK_BIN = Path(r"C:\Program Files\Artec\Artec 3D Scanning SDK\bin-x64")
_HERE = Path(__file__).parent

_algo_py = None  # lazy-loaded artec_algorithm_py module


def _load():
    """artec_algorithm_py .pyd 를 lazy-load하고 반환."""
    global _algo_py
    if _algo_py is not None:
        return _algo_py

    if _ARTEC_SDK_BIN.exists():
        os.add_dll_directory(str(_ARTEC_SDK_BIN))
        os.environ["PATH"] = str(_ARTEC_SDK_BIN) + ";" + os.environ.get("PATH", "")

    if str(_HERE) not in sys.path:
        sys.path.insert(0, str(_HERE))

    try:
        import artec_algorithm_py as _m
        _algo_py = _m
        return _algo_py
    except ImportError as e:
        raise ImportError(
            "[artec_algorithm] artec_algorithm_py 모듈 로드 실패.\n"
            "  cmake --build build --config Release 후 artec_algorithm_py.pyd 생성 확인.\n"
            f"  원인: {e}"
        ) from e


# ==============================================================
# Enums
# ==============================================================

class ScannerType(IntEnum):
    UNKNOWN            = 0
    EVA                = 1
    SPIDER             = 2
    SPACE              = 3
    HANDHELD           = 4
    MH                 = 5
    MICRO              = 6
    L                  = 7
    EVA_LITE           = 8
    LEO                = 9
    FLEX               = 10
    MICRO_II           = 11
    MICRO_II_STANDARD  = 12


class SerialRegistrationType(IntEnum):
    ROUGH           = 0
    ROUGH_TEXTURED  = 1
    FINE            = 2
    FINE_TEXTURED   = 3


class GlobalRegistrationType(IntEnum):
    GEOMETRY             = 0
    GEOMETRY_AND_TEXTURE = 1


class PoissonFusionType(IntEnum):
    SHARP  = 0
    SMOOTH = 1


class FillHolesType(IntEnum):
    ALL       = 0
    BY_RADIUS = 1


class SmallObjectsFilterType(IntEnum):
    LEAVE_BIGGEST_OBJECT = 0
    FILTER_BY_THRESHOLD  = 1


class SimplifyType(IntEnum):
    TRIANGLE_QUANTITY      = 0
    ACCURACY               = 1
    REMESH                 = 2
    TRIANGLE_QUANTITY_FAST = 3


class SimplifyMetric(IntEnum):
    EDGE_LENGTH                      = 0
    EDGE_LENGTH_AND_ANGLE            = 1
    DISTANCE_TO_SURFACE              = 2
    DISTANCE_TO_SURFACE_ITERATIVE    = 3


class TexturizeType(IntEnum):
    ADVANCED               = 0
    ATLAS                  = 1
    KEEP_ATLAS             = 2
    VERTEX_COLOR_TO_ATLAS  = 3


class TexturizeResolution(IntEnum):
    R512    = 0
    R1024   = 1
    R2048   = 2
    R4096   = 3
    R8192   = 4
    R16384  = 5


class InputFilter(IntEnum):
    USE_TEXTURE_KEYFRAMES = 0
    USE_ALL_TEXTURES      = 1


# ==============================================================
# DTOs
# ==============================================================

@dataclass
class SerialRegistrationSettingsDTO:
    scanner_type:       int = ScannerType.UNKNOWN
    registration_type:  int = SerialRegistrationType.FINE

    @classmethod
    def default(cls, scanner_type: int = ScannerType.UNKNOWN) -> "SerialRegistrationSettingsDTO":
        return cls(scanner_type=scanner_type)

    def _to_dict(self) -> dict:
        return {
            "scanner_type":      int(self.scanner_type),
            "registration_type": int(self.registration_type),
        }


@dataclass
class GlobalRegistrationSettingsDTO:
    scanner_type:       int = ScannerType.UNKNOWN
    registration_type:  int = GlobalRegistrationType.GEOMETRY

    @classmethod
    def default(cls, scanner_type: int = ScannerType.UNKNOWN) -> "GlobalRegistrationSettingsDTO":
        return cls(scanner_type=scanner_type)

    def _to_dict(self) -> dict:
        return {
            "scanner_type":      int(self.scanner_type),
            "registration_type": int(self.registration_type),
        }


@dataclass
class OutliersRemovalSettingsDTO:
    scanner_type:                   int   = ScannerType.UNKNOWN
    standard_deviation_multiplier:  float = 3.0
    resolution:                     float = 0.0

    @classmethod
    def default(cls, scanner_type: int = ScannerType.UNKNOWN) -> "OutliersRemovalSettingsDTO":
        d = _load().init_outliers_removal_settings(int(scanner_type))
        return cls(
            scanner_type=d.get("scanner_type", scanner_type),
            standard_deviation_multiplier=d.get("standard_deviation_multiplier", 3.0),
            resolution=d.get("resolution", 0.0),
        )

    def _to_dict(self) -> dict:
        return {
            "scanner_type":                  int(self.scanner_type),
            "standard_deviation_multiplier": float(self.standard_deviation_multiplier),
            "resolution":                    float(self.resolution),
        }


@dataclass
class SmallObjectsFilterSettingsDTO:
    scanner_type:     int = ScannerType.UNKNOWN
    filter_type:      int = SmallObjectsFilterType.LEAVE_BIGGEST_OBJECT
    filter_threshold: int = 0

    @classmethod
    def default(cls, scanner_type: int = ScannerType.UNKNOWN) -> "SmallObjectsFilterSettingsDTO":
        d = _load().init_small_objects_filter_settings(int(scanner_type))
        return cls(
            scanner_type=d.get("scanner_type", scanner_type),
            filter_type=d.get("filter_type", SmallObjectsFilterType.LEAVE_BIGGEST_OBJECT),
            filter_threshold=d.get("filter_threshold", 0),
        )

    def _to_dict(self) -> dict:
        return {
            "scanner_type":     int(self.scanner_type),
            "filter_type":      int(self.filter_type),
            "filter_threshold": int(self.filter_threshold),
        }


@dataclass
class FastFusionSettingsDTO:
    scanner_type:     int   = ScannerType.UNKNOWN
    resolution:       float = 0.0
    radius:           int   = 2
    generate_normals: bool  = True

    @classmethod
    def default(cls, scanner_type: int = ScannerType.UNKNOWN) -> "FastFusionSettingsDTO":
        d = _load().init_fast_fusion_settings(int(scanner_type))
        return cls(
            scanner_type=d.get("scanner_type", scanner_type),
            resolution=d.get("resolution", 0.0),
            radius=d.get("radius", 2),
            generate_normals=d.get("generate_normals", True),
        )

    def _to_dict(self) -> dict:
        return {
            "scanner_type":     int(self.scanner_type),
            "resolution":       float(self.resolution),
            "radius":           int(self.radius),
            "generate_normals": bool(self.generate_normals),
        }


@dataclass
class PoissonFusionSettingsDTO:
    scanner_type:       int   = ScannerType.UNKNOWN
    fusion_type:        int   = PoissonFusionType.SHARP
    fill_type:          int   = FillHolesType.ALL
    resolution:         float = 0.0
    max_hole_radius:    float = 0.0
    remove_targets:     bool  = True
    target_inner_size:  float = 0.0
    target_outer_size:  float = 0.0
    generate_normals:   bool  = True
    input_filter_type:  int   = InputFilter.USE_TEXTURE_KEYFRAMES

    @classmethod
    def default(cls, scanner_type: int = ScannerType.UNKNOWN) -> "PoissonFusionSettingsDTO":
        d = _load().init_poisson_fusion_settings(int(scanner_type))
        return cls(
            scanner_type=d.get("scanner_type", scanner_type),
            fusion_type=d.get("fusion_type", PoissonFusionType.SHARP),
            fill_type=d.get("fill_type", FillHolesType.ALL),
            resolution=d.get("resolution", 0.0),
            max_hole_radius=d.get("max_hole_radius", 0.0),
            remove_targets=d.get("remove_targets", True),
            target_inner_size=d.get("target_inner_size", 0.0),
            target_outer_size=d.get("target_outer_size", 0.0),
            generate_normals=d.get("generate_normals", True),
            input_filter_type=d.get("input_filter_type", InputFilter.USE_TEXTURE_KEYFRAMES),
        )

    def _to_dict(self) -> dict:
        return {
            "scanner_type":      int(self.scanner_type),
            "fusion_type":       int(self.fusion_type),
            "fill_type":         int(self.fill_type),
            "resolution":        float(self.resolution),
            "max_hole_radius":   float(self.max_hole_radius),
            "remove_targets":    bool(self.remove_targets),
            "target_inner_size": float(self.target_inner_size),
            "target_outer_size": float(self.target_outer_size),
            "generate_normals":  bool(self.generate_normals),
            "input_filter_type": int(self.input_filter_type),
        }


@dataclass
class MeshSimplificationSettingsDTO:
    scanner_type:          int   = ScannerType.UNKNOWN
    simplify_type:         int   = SimplifyType.TRIANGLE_QUANTITY
    simplify_metrics:      int   = SimplifyMetric.EDGE_LENGTH
    triangle_number:       int   = 0
    keep_boundary:         bool  = True
    angle_threshold:       float = 0.0
    remesh_edge_threshold: float = 0.0
    error:                 float = 0.0

    @classmethod
    def default(cls,
                scanner_type:  int = ScannerType.UNKNOWN,
                simplify_type: int = SimplifyType.TRIANGLE_QUANTITY,
                ) -> "MeshSimplificationSettingsDTO":
        d = _load().init_mesh_simplification_settings(int(scanner_type), int(simplify_type))
        return cls(
            scanner_type=d.get("scanner_type", scanner_type),
            simplify_type=d.get("simplify_type", simplify_type),
            simplify_metrics=d.get("simplify_metrics", SimplifyMetric.EDGE_LENGTH),
            triangle_number=d.get("triangle_number", 0),
            keep_boundary=d.get("keep_boundary", True),
            angle_threshold=d.get("angle_threshold", 0.0),
            remesh_edge_threshold=d.get("remesh_edge_threshold", 0.0),
            error=d.get("error", 0.0),
        )

    def _to_dict(self) -> dict:
        return {
            "scanner_type":          int(self.scanner_type),
            "simplify_type":         int(self.simplify_type),
            "simplify_metrics":      int(self.simplify_metrics),
            "triangle_number":       int(self.triangle_number),
            "keep_boundary":         bool(self.keep_boundary),
            "angle_threshold":       float(self.angle_threshold),
            "remesh_edge_threshold": float(self.remesh_edge_threshold),
            "error":                 float(self.error),
        }


@dataclass
class FastMeshSimplificationSettingsDTO:
    scanner_type:                   int   = ScannerType.UNKNOWN
    triangle_number:                int   = 0
    keep_boundary:                  bool  = True
    enable_additional_criteria:     bool  = False
    enable_distance_threshold:      bool  = False
    distance_threshold:             float = 0.0
    enable_angle_threshold:         bool  = False
    angle_threshold:                float = 0.0
    enable_aspect_ratio_threshold:  bool  = False
    aspect_ratio_threshold:         float = 0.0

    @classmethod
    def default(cls, scanner_type: int = ScannerType.UNKNOWN) -> "FastMeshSimplificationSettingsDTO":
        d = _load().init_fast_mesh_simplification_settings(int(scanner_type))
        return cls(
            scanner_type=d.get("scanner_type", scanner_type),
            triangle_number=d.get("triangle_number", 0),
            keep_boundary=d.get("keep_boundary", True),
            enable_additional_criteria=d.get("enable_additional_criteria", False),
            enable_distance_threshold=d.get("enable_distance_threshold", False),
            distance_threshold=d.get("distance_threshold", 0.0),
            enable_angle_threshold=d.get("enable_angle_threshold", False),
            angle_threshold=d.get("angle_threshold", 0.0),
            enable_aspect_ratio_threshold=d.get("enable_aspect_ratio_threshold", False),
            aspect_ratio_threshold=d.get("aspect_ratio_threshold", 0.0),
        )

    def _to_dict(self) -> dict:
        return {
            "scanner_type":                  int(self.scanner_type),
            "triangle_number":               int(self.triangle_number),
            "keep_boundary":                 bool(self.keep_boundary),
            "enable_additional_criteria":    bool(self.enable_additional_criteria),
            "enable_distance_threshold":     bool(self.enable_distance_threshold),
            "distance_threshold":            float(self.distance_threshold),
            "enable_angle_threshold":        bool(self.enable_angle_threshold),
            "angle_threshold":               float(self.angle_threshold),
            "enable_aspect_ratio_threshold": bool(self.enable_aspect_ratio_threshold),
            "aspect_ratio_threshold":        float(self.aspect_ratio_threshold),
        }


@dataclass
class TexturizationSettingsDTO:
    scanner_type:                         int  = ScannerType.UNKNOWN
    texturize_type:                       int  = TexturizeType.ADVANCED
    texturize_resolution:                 int  = TexturizeResolution.R2048
    enable_background_segmentation:       bool = False
    enable_ambient_lighting_compensation: bool = False
    atlas_unfolding_polygon_limit:        int  = 0
    enable_texture_inpainting:            bool = True
    use_texture_normalization:            bool = False
    input_filter_type:                    int  = InputFilter.USE_TEXTURE_KEYFRAMES

    @classmethod
    def default(cls, scanner_type: int = ScannerType.UNKNOWN) -> "TexturizationSettingsDTO":
        d = _load().init_texturization_settings(int(scanner_type))
        return cls(
            scanner_type=d.get("scanner_type", scanner_type),
            texturize_type=d.get("texturize_type", TexturizeType.ADVANCED),
            texturize_resolution=d.get("texturize_resolution", TexturizeResolution.R2048),
            enable_background_segmentation=d.get("enable_background_segmentation", False),
            enable_ambient_lighting_compensation=d.get("enable_ambient_lighting_compensation", False),
            atlas_unfolding_polygon_limit=d.get("atlas_unfolding_polygon_limit", 0),
            enable_texture_inpainting=d.get("enable_texture_inpainting", True),
            use_texture_normalization=d.get("use_texture_normalization", False),
            input_filter_type=d.get("input_filter_type", InputFilter.USE_TEXTURE_KEYFRAMES),
        )

    def _to_dict(self) -> dict:
        return {
            "scanner_type":                         int(self.scanner_type),
            "texturize_type":                       int(self.texturize_type),
            "texturize_resolution":                 int(self.texturize_resolution),
            "enable_background_segmentation":       bool(self.enable_background_segmentation),
            "enable_ambient_lighting_compensation": bool(self.enable_ambient_lighting_compensation),
            "atlas_unfolding_polygon_limit":        int(self.atlas_unfolding_polygon_limit),
            "enable_texture_inpainting":            bool(self.enable_texture_inpainting),
            "use_texture_normalization":            bool(self.use_texture_normalization),
            "input_filter_type":                    int(self.input_filter_type),
        }


@dataclass
class PipelineSettingsDTO:
    """
    자주 쓰는 전체 후처리 파이프라인 정의.

    do_* 플래그로 각 단계 활성화. 각 단계별 세부 설정은 대응 DTO로 지정.
    None 이면 SDK 기본값을 scanner_type 기반으로 자동 초기화.
    """
    scanner_type:           int  = ScannerType.UNKNOWN

    do_serial_registration: bool = True
    serial_registration:    Optional[SerialRegistrationSettingsDTO]    = None

    do_global_registration: bool = False
    global_registration:    Optional[GlobalRegistrationSettingsDTO]    = None

    do_outliers_removal:    bool = True
    outliers_removal:       Optional[OutliersRemovalSettingsDTO]       = None

    do_small_objects_filter: bool = True
    small_objects_filter:    Optional[SmallObjectsFilterSettingsDTO]   = None

    do_fusion:              bool = True
    use_fast_fusion:        bool = True
    fast_fusion:            Optional[FastFusionSettingsDTO]            = None
    poisson_fusion:         Optional[PoissonFusionSettingsDTO]         = None

    do_simplify:            bool = False
    simplify:               Optional[MeshSimplificationSettingsDTO]    = None

    do_texturize:           bool = False
    texturize:              Optional[TexturizationSettingsDTO]         = None

    @classmethod
    def default(cls, scanner_type: int = ScannerType.UNKNOWN) -> "PipelineSettingsDTO":
        return cls(scanner_type=scanner_type)


# ==============================================================
# Algorithms
# ==============================================================

class Algorithms:
    """
    Artec Algorithm SDK 래퍼.

    모든 메서드는 static.
    입력/출력: artec_base.ModelHandle (IModel* capsule 래퍼).
    알고리즘은 in-place: 반환된 ModelHandle 은 입력과 동일한 IModel 을
    가리키지만 새 addRef capsule 이다.
    """

    @staticmethod
    def is_available() -> bool:
        """Return True if algorithms are licensed on this machine."""
        return _load().check_permission()

    # ----------------------------------------------------------
    # Registration
    # ----------------------------------------------------------

    @staticmethod
    def serial_registration(model, settings: Optional[SerialRegistrationSettingsDTO] = None):
        """
        Serial Registration.

        Parameters
        ----------
        model   : artec_base.ModelHandle
        settings: SerialRegistrationSettingsDTO | None  (None → SDK 기본값)

        Returns
        -------
        artec_base.ModelHandle
        """
        from mms.sensor.artec import artec_base
        d = settings._to_dict() if settings is not None else None
        cap = _load().serial_registration(model._cap, d)
        return artec_base.ModelHandle(cap)

    @staticmethod
    def global_registration(model, settings: Optional[GlobalRegistrationSettingsDTO] = None):
        """Global Registration."""
        from mms.sensor.artec import artec_base
        d = settings._to_dict() if settings is not None else None
        cap = _load().global_registration(model._cap, d)
        return artec_base.ModelHandle(cap)

    # ----------------------------------------------------------
    # Filtering
    # ----------------------------------------------------------

    @staticmethod
    def outliers_removal(model, settings: Optional[OutliersRemovalSettingsDTO] = None):
        """Outliers Removal."""
        from mms.sensor.artec import artec_base
        d = settings._to_dict() if settings is not None else None
        cap = _load().outliers_removal(model._cap, d)
        return artec_base.ModelHandle(cap)

    @staticmethod
    def small_objects_filter(model, settings: Optional[SmallObjectsFilterSettingsDTO] = None):
        """Small Objects Filter."""
        from mms.sensor.artec import artec_base
        d = settings._to_dict() if settings is not None else None
        cap = _load().small_objects_filter(model._cap, d)
        return artec_base.ModelHandle(cap)

    # ----------------------------------------------------------
    # Fusion
    # ----------------------------------------------------------

    @staticmethod
    def fast_fusion(model, settings: Optional[FastFusionSettingsDTO] = None):
        """Fast Fusion (빠른 메시 생성)."""
        from mms.sensor.artec import artec_base
        d = settings._to_dict() if settings is not None else None
        cap = _load().fast_fusion(model._cap, d)
        return artec_base.ModelHandle(cap)

    @staticmethod
    def poisson_fusion(model, settings: Optional[PoissonFusionSettingsDTO] = None):
        """Poisson Fusion (Sharp/Smooth watertight mesh)."""
        from mms.sensor.artec import artec_base
        d = settings._to_dict() if settings is not None else None
        cap = _load().poisson_fusion(model._cap, d)
        return artec_base.ModelHandle(cap)

    # ----------------------------------------------------------
    # Simplification
    # ----------------------------------------------------------

    @staticmethod
    def mesh_simplify(model, settings: Optional[MeshSimplificationSettingsDTO] = None):
        """Mesh Simplification (고품질)."""
        from mms.sensor.artec import artec_base
        d = settings._to_dict() if settings is not None else None
        cap = _load().mesh_simplify(model._cap, d)
        return artec_base.ModelHandle(cap)

    @staticmethod
    def fast_mesh_simplify(model, settings: Optional[FastMeshSimplificationSettingsDTO] = None):
        """Fast Mesh Simplification."""
        from mms.sensor.artec import artec_base
        d = settings._to_dict() if settings is not None else None
        cap = _load().fast_mesh_simplify(model._cap, d)
        return artec_base.ModelHandle(cap)

    # ----------------------------------------------------------
    # Texturization
    # ----------------------------------------------------------

    @staticmethod
    def texturize(model, settings: Optional[TexturizationSettingsDTO] = None):
        """Texturization."""
        from mms.sensor.artec import artec_base
        d = settings._to_dict() if settings is not None else None
        cap = _load().texturize(model._cap, d)
        return artec_base.ModelHandle(cap)

    # ----------------------------------------------------------
    # Misc
    # ----------------------------------------------------------

    @staticmethod
    def auto_align(model, settings=None):
        """Auto-align (다중 scan 자동 조립)."""
        from mms.sensor.artec import artec_base
        d = {"scanner_type": int(settings.scanner_type)} if settings is not None else None
        cap = _load().auto_align(model._cap, d)
        return artec_base.ModelHandle(cap)

    @staticmethod
    def loop_closure(model, settings=None):
        """Loop Closure."""
        from mms.sensor.artec import artec_base
        d = {"scanner_type": int(settings.scanner_type)} if settings is not None else None
        cap = _load().loop_closure(model._cap, d)
        return artec_base.ModelHandle(cap)

    # ----------------------------------------------------------
    # Pipeline
    # ----------------------------------------------------------

    @staticmethod
    def run_pipeline(model, pipeline: Optional[PipelineSettingsDTO] = None):
        """
        자주 쓰는 후처리 파이프라인을 순서대로 실행.

        순서
        ----
        1. serial_registration  (do_serial_registration)
        2. global_registration  (do_global_registration)
        3. outliers_removal     (do_outliers_removal)
        4. small_objects_filter (do_small_objects_filter)
        5. fast_fusion / poisson_fusion  (do_fusion)
        6. mesh_simplify        (do_simplify)
        7. texturize            (do_texturize)

        Parameters
        ----------
        model    : artec_base.ModelHandle
        pipeline : PipelineSettingsDTO | None  (None → 기본 파이프라인)

        Returns
        -------
        artec_base.ModelHandle  (최종 결과)
        """
        if pipeline is None:
            pipeline = PipelineSettingsDTO.default()

        st = pipeline.scanner_type
        result = model

        if pipeline.do_serial_registration:
            s = pipeline.serial_registration or SerialRegistrationSettingsDTO.default(st)
            print("  [Algorithms] serial_registration ...")
            t0 = time.perf_counter()
            result = Algorithms.serial_registration(result, s)
            print(f"    done ({time.perf_counter()-t0:.2f}s)")

        if pipeline.do_global_registration:
            s = pipeline.global_registration or GlobalRegistrationSettingsDTO.default(st)
            print("  [Algorithms] global_registration ...")
            t0 = time.perf_counter()
            result = Algorithms.global_registration(result, s)
            print(f"    done ({time.perf_counter()-t0:.2f}s)")

        if pipeline.do_outliers_removal:
            s = pipeline.outliers_removal or OutliersRemovalSettingsDTO.default(st)
            print("  [Algorithms] outliers_removal ...")
            t0 = time.perf_counter()
            result = Algorithms.outliers_removal(result, s)
            print(f"    done ({time.perf_counter()-t0:.2f}s)")

        if pipeline.do_small_objects_filter:
            s = pipeline.small_objects_filter or SmallObjectsFilterSettingsDTO.default(st)
            print("  [Algorithms] small_objects_filter ...")
            t0 = time.perf_counter()
            result = Algorithms.small_objects_filter(result, s)
            print(f"    done ({time.perf_counter()-t0:.2f}s)")

        if pipeline.do_fusion:
            if pipeline.use_fast_fusion:
                s = pipeline.fast_fusion or FastFusionSettingsDTO.default(st)
                print("  [Algorithms] fast_fusion ...")
                t0 = time.perf_counter()
                result = Algorithms.fast_fusion(result, s)
                print(f"    done ({time.perf_counter()-t0:.2f}s)")
            else:
                s = pipeline.poisson_fusion or PoissonFusionSettingsDTO.default(st)
                print("  [Algorithms] poisson_fusion ...")
                t0 = time.perf_counter()
                result = Algorithms.poisson_fusion(result, s)
                print(f"    done ({time.perf_counter()-t0:.2f}s)")

        if pipeline.do_simplify:
            s = pipeline.simplify or MeshSimplificationSettingsDTO.default(st)
            print("  [Algorithms] mesh_simplify ...")
            t0 = time.perf_counter()
            result = Algorithms.mesh_simplify(result, s)
            print(f"    done ({time.perf_counter()-t0:.2f}s)")

        if pipeline.do_texturize:
            s = pipeline.texturize or TexturizationSettingsDTO.default(st)
            print("  [Algorithms] texturize ...")
            t0 = time.perf_counter()
            result = Algorithms.texturize(result, s)
            print(f"    done ({time.perf_counter()-t0:.2f}s)")

        return result


# ==============================================================
# Visualization
# ==============================================================

def visualize_model_result(model_handle, title: str = "Artec Algorithm Result") -> bool:
    """
    알고리즘 결과 ModelHandle 을 Open3D 로 시각화.

    최종 합성 메쉬(final_vertices / final_faces)를 사용.
    합성 메쉬가 없으면 모든 스캔의 프레임을 시각화.

    Returns
    -------
    bool  True if visualization was shown, False if open3d not available.
    """
    try:
        import open3d as o3d
    except ImportError:
        print("  [visualize] open3d 없음 — 시각화 건너뜀")
        return False

    verts = model_handle.final_vertices()
    faces = model_handle.final_faces()

    if verts.shape[0] == 0:
        # 최종 메쉬 없음 → 스캔 프레임에서 포인트 수집
        print("  [visualize] final mesh 없음 — scan frame 포인트 시각화")
        all_pts = []
        for scan in model_handle.scans():
            for frame in scan.frames():
                all_pts.append(frame.vertices())
        if not all_pts:
            print("  [visualize] 시각화할 데이터 없음")
            return False
        pts = np.concatenate(all_pts, axis=0).astype(np.float64) * 1e-3  # mm→m
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pts)
        geom = pcd
    else:
        pts = verts.astype(np.float64) * 1e-3  # mm → m
        mesh = o3d.geometry.TriangleMesh()
        mesh.vertices = o3d.utility.Vector3dVector(pts)
        mesh.triangles = o3d.utility.Vector3iVector(faces.astype(np.int32))
        mesh.compute_vertex_normals()
        geom = mesh

    print(f"  [visualize] {title}  verts={verts.shape[0]:,}  faces={faces.shape[0]:,}")
    o3d.visualization.draw_geometries([geom], window_name=title)
    return True


# ==============================================================
# Verify
# ==============================================================

def verify_artec_algorithm(
    artec_client,
    visualize: bool = True,
    scan_model=None,
) -> bool:
    """
    artec_algorithm_py 바인딩 및 Algorithms 클래스 검증.

    단계
    ----
    1. 모듈 로드 확인
    2. check_permission() 호출
    3. get_scanner_type() 호출
    4. 설정 초기화 함수 검증 (init_*)
    5. SettingsDTO.default() 검증
    6. 스캔 모델 준비 (scan_model 인자 우선, 없으면 capture_to_model)
    7. Algorithms.serial_registration()
    8. Algorithms.outliers_removal()
    9. Algorithms.fast_fusion()  → final mesh 확인
    10. visualize_model_result() (선택)

    Parameters
    ----------
    artec_client : ArtecClient
    visualize : bool
    scan_model : artec_base.ModelHandle | None
        다중 프레임이 있는 ModelHandle. None이면 capture_to_model()로 캡처.
        serial_registration은 프레임 수 >= 2 필요.

    Returns
    -------
    bool  True if all steps passed.
    """
    from mms.sensor.artec import artec_base

    passed = []
    failed = []

    def chk(label: str, cond: bool, detail: str = "") -> bool:
        status = "OK" if cond else "FAIL"
        msg = f"    [{status}] {label}"
        if detail:
            msg += f"  ({detail})"
        print(msg)
        (passed if cond else failed).append(label)
        return cond

    print("\n  [verify_artec_algorithm] 시작")

    # ── 1. 모듈 로드 ──────────────────────────────────────────
    try:
        m = _load()
        chk("artec_algorithm_py 로드", True)
    except Exception as e:
        chk("artec_algorithm_py 로드", False, str(e))
        return False

    # ── 2. check_permission ───────────────────────────────────
    try:
        available = Algorithms.is_available()
        chk("check_permission() 호출", True, f"available={available}")
    except Exception as e:
        chk("check_permission() 호출", False, str(e))

    # ── 3. get_scanner_type ───────────────────────────────────
    scanner_type_int = int(ScannerType.UNKNOWN)
    try:
        sc_cap = artec_client._scanner.scanner_capsule()
        scanner_type_int = m.get_scanner_type(sc_cap)
        chk("get_scanner_type() 호출", True, f"type={scanner_type_int}")
    except Exception as e:
        chk("get_scanner_type() 호출", False, str(e))

    # ── 4. 설정 초기화 함수 ───────────────────────────────────
    init_funcs = [
        ("init_fast_fusion_settings",          lambda: m.init_fast_fusion_settings(scanner_type_int)),
        ("init_poisson_fusion_settings",       lambda: m.init_poisson_fusion_settings(scanner_type_int)),
        ("init_texturization_settings",        lambda: m.init_texturization_settings(scanner_type_int)),
        ("init_small_objects_filter_settings", lambda: m.init_small_objects_filter_settings(scanner_type_int)),
        ("init_mesh_simplification_settings",  lambda: m.init_mesh_simplification_settings(scanner_type_int, 0)),
        ("init_fast_mesh_simplification_settings", lambda: m.init_fast_mesh_simplification_settings(scanner_type_int)),
        ("init_outliers_removal_settings",     lambda: m.init_outliers_removal_settings(scanner_type_int)),
    ]
    for name, fn in init_funcs:
        try:
            d = fn()
            chk(f"{name}()", isinstance(d, dict), f"keys={list(d.keys())}")
        except Exception as e:
            chk(f"{name}()", False, str(e))

    # ── 5. SettingsDTO.default() ──────────────────────────────
    dto_defaults = [
        ("SerialRegistrationSettingsDTO.default", lambda: SerialRegistrationSettingsDTO.default(scanner_type_int)),
        ("GlobalRegistrationSettingsDTO.default",  lambda: GlobalRegistrationSettingsDTO.default(scanner_type_int)),
        ("OutliersRemovalSettingsDTO.default",     lambda: OutliersRemovalSettingsDTO.default(scanner_type_int)),
        ("SmallObjectsFilterSettingsDTO.default",  lambda: SmallObjectsFilterSettingsDTO.default(scanner_type_int)),
        ("FastFusionSettingsDTO.default",          lambda: FastFusionSettingsDTO.default(scanner_type_int)),
        ("PoissonFusionSettingsDTO.default",       lambda: PoissonFusionSettingsDTO.default(scanner_type_int)),
        ("MeshSimplificationSettingsDTO.default",  lambda: MeshSimplificationSettingsDTO.default(scanner_type_int)),
        ("FastMeshSimplificationSettingsDTO.default", lambda: FastMeshSimplificationSettingsDTO.default(scanner_type_int)),
        ("TexturizationSettingsDTO.default",       lambda: TexturizationSettingsDTO.default(scanner_type_int)),
    ]
    for name, fn in dto_defaults:
        try:
            dto = fn()
            chk(f"{name}()", dto is not None)
        except Exception as e:
            chk(f"{name}()", False, str(e))

    # ── 6. 스캔 모델 준비 ────────────────────────────────────
    # scan_model이 전달되면 재사용 (다중 프레임 필요).
    # 없으면 capture_to_model()로 1-frame 캡처 (registration 불가 → 건너뜀).
    model = None
    if scan_model is not None:
        model = scan_model
        total_frames = sum(
            model.get_scan(i).frame_count()
            for i in range(model.scan_count())
        )
        chk("scan_model 수신", True,
            f"scans={model.scan_count()}  total_frames={total_frames}")
    else:
        for tex in (False, True):
            try:
                model = artec_base.capture_to_model(artec_client, capture_texture=tex)
                if model is not None:
                    chk(f"capture_to_model(texture={tex})", True,
                        f"scans={model.scan_count()}")
                    break
                else:
                    chk(f"capture_to_model(texture={tex})", False, "None 반환")
            except Exception as e:
                chk(f"capture_to_model(texture={tex})", False, str(e))

    if model is None:
        print("    ※ 스캔 데이터 없음 — 알고리즘 단계 건너뜀")
        _print_summary(passed, failed)
        return len(failed) == 0

    total_frames = sum(
        model.get_scan(i).frame_count()
        for i in range(model.scan_count())
    )
    if total_frames < 2:
        print(f"    ※ frame 수 부족 ({total_frames}개) — serial_registration 건너뜀 (최소 2개 필요)")
        _print_summary(passed, failed)
        return len(failed) == 0

    # ── 7. serial_registration ────────────────────────────────
    try:
        s = SerialRegistrationSettingsDTO.default(scanner_type_int)
        t0 = time.perf_counter()
        model = Algorithms.serial_registration(model, s)
        elapsed = time.perf_counter() - t0
        chk("serial_registration()", isinstance(model, artec_base.ModelHandle),
            f"elapsed={elapsed:.2f}s  scans={model.scan_count()}")
    except Exception as e:
        chk("serial_registration()", False, str(e))

    # ── 8. outliers_removal ───────────────────────────────────
    try:
        s = OutliersRemovalSettingsDTO.default(scanner_type_int)
        t0 = time.perf_counter()
        model = Algorithms.outliers_removal(model, s)
        elapsed = time.perf_counter() - t0
        chk("outliers_removal()", isinstance(model, artec_base.ModelHandle),
            f"elapsed={elapsed:.2f}s")
    except Exception as e:
        chk("outliers_removal()", False, str(e))

    # ── 9. fast_fusion → final mesh ───────────────────────────
    try:
        s = FastFusionSettingsDTO.default(scanner_type_int)
        t0 = time.perf_counter()
        model = Algorithms.fast_fusion(model, s)
        elapsed = time.perf_counter() - t0
        has_mesh = model.has_final_mesh()
        verts = model.final_vertices()
        faces = model.final_faces()
        chk("fast_fusion()", has_mesh,
            f"elapsed={elapsed:.2f}s  verts={verts.shape[0]:,}  faces={faces.shape[0]:,}")
        chk("final_vertices() shape (N,3) float32",
            verts.ndim == 2 and verts.shape[1] == 3 and verts.dtype == np.float32,
            f"shape={verts.shape}  dtype={verts.dtype}")
        chk("final_faces() shape (M,3) int32",
            faces.ndim == 2 and faces.shape[1] == 3 and faces.dtype == np.int32,
            f"shape={faces.shape}  dtype={faces.dtype}")
    except Exception as e:
        chk("fast_fusion()", False, str(e))
        model = None

    # ── 10. 시각화 ────────────────────────────────────────────
    if visualize and model is not None:
        try:
            shown = visualize_model_result(model, title="artec_algorithm verify — fast_fusion result")
            chk("visualize_model_result()", shown)
        except Exception as e:
            chk("visualize_model_result()", False, str(e))

    _print_summary(passed, failed)
    return len(failed) == 0


def _print_summary(passed: list, failed: list) -> None:
    total = len(passed) + len(failed)
    print(f"\n  [verify_artec_algorithm] 완료: {len(passed)}/{total} 통과"
          + (f", 실패: {failed}" if failed else ""))
