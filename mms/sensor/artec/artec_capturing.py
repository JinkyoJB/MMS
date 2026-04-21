# mms/sensor/artec/artec_capturing.py
#
# Fine-grained Artec Capturing API — Python 구현.
#
# artec_capturing_py (C++ pybind11)는 capsule 기반 free function만 노출.
# 이 파일에서 클래스 구조(ScannerManager, ScannerHandle 등)를 Python으로 구현.
#
# Exposes
# -------
#   DTOs (dataclass)
#     ScannerIdInfo, ScannerInfoDTO
#     CapturedFrameSummary, FrameProcessorSettings
#
#   Handles (Python class, _cap으로 capsule 보유)
#     CapturedFrameHandle
#     FrameProcessorHandle
#     ScannerHandle
#
#   ScannerManager
#
#   Module-level helpers
#     list_scanners(), open_scanner(), open_from_capsule()
#     capture_and_reconstruct()
#
#   Visualization
#     visualize_mesh(), visualize_texture_image()
#
#   Verification
#     verify_artec_capturing()

from __future__ import annotations

import dataclasses
import os
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional, Tuple

import numpy as np

_ARTEC_SDK_BIN = Path(r"C:\Program Files\Artec\Artec 3D Scanning SDK\bin-x64")


def _load() :
    """artec_capturing_py 모듈 로드 (lazy, cached)."""
    if "artec_capturing_py" in sys.modules:
        return sys.modules["artec_capturing_py"]

    if _ARTEC_SDK_BIN.exists():
        os.add_dll_directory(str(_ARTEC_SDK_BIN))
        os.environ["PATH"] = str(_ARTEC_SDK_BIN) + ";" + os.environ.get("PATH", "")

    _here = Path(__file__).parent
    if str(_here) not in sys.path:
        sys.path.insert(0, str(_here))

    try:
        import artec_capturing_py as _m
        return _m
    except ImportError as e:
        raise ImportError(
            "[artec_capturing] artec_capturing_py 모듈 로드 실패.\n"
            "  cmake --build build --config Release 후 artec_capturing_py.pyd 확인.\n"
            f"  원인: {e}"
        ) from e


# ──────────────────────────────────────────────────────────────────────
# DTOs
# ──────────────────────────────────────────────────────────────────────

@dataclass
class ScannerIdInfo:
    serial:             str  = ""
    name:               str  = ""
    license:            str  = ""
    main_camera_serial: str  = ""
    has_texture_camera: bool = False
    scanner_type_str:   str  = ""
    calibration_id:     int  = 0

    @classmethod
    def _from_dict(cls, d: dict) -> "ScannerIdInfo":
        return cls(
            serial             = d.get("serial",             ""),
            name               = d.get("name",               ""),
            license            = d.get("license",            ""),
            main_camera_serial = d.get("main_camera_serial", ""),
            has_texture_camera = d.get("has_texture_camera", False),
            scanner_type_str   = d.get("scanner_type_str",   ""),
            calibration_id     = d.get("calibration_id",     0),
        )

    def __repr__(self) -> str:
        return (f"ScannerIdInfo(serial={self.serial!r}, name={self.name!r}, "
                f"type={self.scanner_type_str!r})")


@dataclass
class ScannerInfoDTO:
    depth_map_size_x:        int  = 0
    depth_map_size_y:        int  = 0
    has_texture_camera:      bool = False
    texture_size_x:          int  = 0
    texture_size_y:          int  = 0
    external_sync_supported: bool = False
    gain_available:          bool = False
    scanner_buttons_mask:    int  = 0

    @classmethod
    def _from_dict(cls, d: dict) -> "ScannerInfoDTO":
        return cls(**{k: d[k] for k in dataclasses.fields(cls) if k in d})

    def __repr__(self) -> str:
        return (f"ScannerInfoDTO(depth={self.depth_map_size_x}x{self.depth_map_size_y}, "
                f"tex={self.texture_size_x}x{self.texture_size_y}, "
                f"gain={self.gain_available})")


@dataclass
class CapturedFrameSummary:
    frame_number: int  = -1
    has_texture:  bool = False

    def __repr__(self) -> str:
        return f"CapturedFrameSummary(frame={self.frame_number}, has_texture={self.has_texture})"


@dataclass
class FrameProcessorSettings:
    """
    FrameProcessor 생성 파라미터.

    FrameProcessorDesc 필드 (createFrameProcessor 에 전달):
      minimum_object_size, triangles_step, edge_length_threshold,
      interpolate, max_interpolated_length, max_angle

    IFrameProcessor 런타임 설정 (생성 후 set):
      sensitivity, range_near_mm, range_far_mm  (0이면 기본값 유지)
    """
    minimum_object_size:     int   = 0
    triangles_step:          int   = 0
    edge_length_threshold:   float = 0.0
    interpolate:             bool  = False
    max_interpolated_length: float = 0.0
    max_angle:               float = 0.0
    sensitivity:             float = 0.0
    range_near_mm:           float = 0.0
    range_far_mm:            float = 0.0

    @classmethod
    def from_scanner_defaults(cls, scanner_handle: "ScannerHandle") -> "FrameProcessorSettings":
        """스캐너에서 기본 FrameProcessorDesc 값을 읽어 FrameProcessorSettings 생성."""
        d = _load().scanner_init_processor_desc(scanner_handle._cap)
        return cls(
            minimum_object_size     = d.get("minimum_object_size",     0),
            triangles_step          = d.get("triangles_step",          0),
            edge_length_threshold   = d.get("edge_length_threshold",   0.0),
            interpolate             = d.get("interpolate",             False),
            max_interpolated_length = d.get("max_interpolated_length", 0.0),
            max_angle               = d.get("max_angle",               0.0),
        )


# ──────────────────────────────────────────────────────────────────────
# CapturedFrameHandle
# ──────────────────────────────────────────────────────────────────────

class CapturedFrameHandle:
    """
    IFrame* capsule 래퍼.
    스캐너에서 캡처한 raw 데이터(geometry + optional raw texture).
    FrameProcessorHandle.reconstruct()에 넘겨 mesh로 변환.
    """

    def __init__(self, frame_cap):
        self._cap = frame_cap  # capsule이 살아있는 한 IFrame* 유효

    def frame_number(self) -> int:
        return _load().frame_number(self._cap)

    def has_texture(self) -> bool:
        return _load().frame_has_texture(self._cap)

    def texture(self) -> Optional[np.ndarray]:
        """Raw texture (H,W,3) uint8 | None. IScanner.convertTextureFull로 보정 가능."""
        return _load().frame_texture(self._cap)

    def summary(self) -> CapturedFrameSummary:
        return CapturedFrameSummary(
            frame_number = self.frame_number(),
            has_texture  = self.has_texture(),
        )

    def __repr__(self) -> str:
        return (f"CapturedFrameHandle(frame={self.frame_number()}, "
                f"has_texture={self.has_texture()})")


# ──────────────────────────────────────────────────────────────────────
# FrameProcessorHandle
# ──────────────────────────────────────────────────────────────────────

class FrameProcessorHandle:
    """
    IFrameProcessor* capsule 래퍼.
    CapturedFrameHandle → FrameMeshHandle (artec_base) 변환 담당.
    """

    def __init__(self, proc_cap):
        self._cap = proc_cap

    def reconstruct(self, frame: CapturedFrameHandle):
        """
        geometry mesh 재구성.
        Returns artec_base.FrameMeshHandle. GIL released.
        """
        from mms.sensor.artec import artec_base
        cap = _load().processor_reconstruct(self._cap, frame._cap)
        return artec_base.FrameMeshHandle(cap)

    def reconstruct_textured(self, frame: CapturedFrameHandle):
        """
        텍스처 포함 mesh 재구성 (느림).
        Returns artec_base.FrameMeshHandle. GIL released.
        """
        from mms.sensor.artec import artec_base
        cap = _load().processor_reconstruct_textured(self._cap, frame._cap)
        return artec_base.FrameMeshHandle(cap)

    # ── 런타임 설정 ─────────────────────────────────────────────

    def sensitivity(self) -> float:
        return _load().processor_sensitivity(self._cap)

    def set_sensitivity(self, value: float) -> None:
        _load().processor_set_sensitivity(self._cap, value)

    def scanning_range(self) -> Tuple[float, float]:
        """(near_mm, far_mm)"""
        return _load().processor_scanning_range(self._cap)

    def set_scanning_range(self, near_mm: float, far_mm: float) -> None:
        _load().processor_set_scanning_range(self._cap, near_mm, far_mm)

    def settings(self) -> FrameProcessorSettings:
        """현재 런타임 설정을 FrameProcessorSettings로 반환."""
        sens = self.sensitivity()
        near, far = self.scanning_range()
        return FrameProcessorSettings(
            sensitivity   = sens,
            range_near_mm = near,
            range_far_mm  = far,
        )

    # ── 호환성 ──────────────────────────────────────────────────

    def processor_capsule(self):
        """artec_base_py / artec_sdk_py 호환용 capsule 반환."""
        return self._cap

    def __repr__(self) -> str:
        return f"FrameProcessorHandle(sens={self.sensitivity():.3f})"


# ──────────────────────────────────────────────────────────────────────
# ScannerHandle
# ──────────────────────────────────────────────────────────────────────

class ScannerHandle:
    """
    IScanner* capsule 래퍼.
    연결된 Artec 스캐너의 모든 기능을 Python 메서드로 제공.
    """

    def __init__(self, scanner_cap):
        self._cap = scanner_cap

    # ── 식별 정보 ────────────────────────────────────────────────

    def id(self) -> ScannerIdInfo:
        return ScannerIdInfo._from_dict(_load().scanner_id(self._cap))

    def info(self) -> ScannerInfoDTO:
        return ScannerInfoDTO._from_dict(_load().scanner_info(self._cap))

    def frame_number(self) -> int:
        return _load().scanner_frame_number(self._cap)

    # ── FPS ──────────────────────────────────────────────────────

    def fps(self) -> float:
        return _load().scanner_fps(self._cap)

    def set_fps(self, fps: float) -> None:
        _load().scanner_set_fps(self._cap, fps)

    def max_fps(self) -> float:
        return _load().scanner_max_fps(self._cap)

    # ── Flash ────────────────────────────────────────────────────

    def flash_enabled(self) -> bool:
        return _load().scanner_flash_enabled(self._cap)

    def enable_flash(self, enable: bool) -> None:
        _load().scanner_enable_flash(self._cap, enable)

    def texture_flash_enabled(self) -> bool:
        return _load().scanner_texture_flash_enabled(self._cap)

    def enable_texture_flash(self, enable: bool) -> None:
        _load().scanner_enable_texture_flash(self._cap, enable)

    # ── Texture 카메라 설정 ──────────────────────────────────────

    def texture_gain(self) -> float:
        return _load().scanner_texture_gain(self._cap)

    def set_texture_gain(self, gain: float) -> None:
        _load().scanner_set_texture_gain(self._cap, gain)

    def texture_shutter_speed(self) -> float:
        return _load().scanner_texture_shutter_speed(self._cap)

    def set_texture_shutter_speed(self, ms: float) -> None:
        _load().scanner_set_texture_shutter_speed(self._cap, ms)

    # ── Auto exposure / white balance ────────────────────────────

    def auto_exposure_enabled(self) -> bool:
        return _load().scanner_auto_exposure_enabled(self._cap)

    def enable_auto_exposure(self, enable: bool) -> None:
        _load().scanner_enable_auto_exposure(self._cap, enable)

    def auto_white_balance_enabled(self) -> bool:
        return _load().scanner_auto_white_balance_enabled(self._cap)

    def enable_auto_white_balance(self, enable: bool) -> None:
        _load().scanner_enable_auto_white_balance(self._cap, enable)

    # ── Hardware trigger ─────────────────────────────────────────

    def hw_trigger_enabled(self) -> bool:
        return _load().scanner_hw_trigger_enabled(self._cap)

    def set_hw_trigger(self, enable: bool) -> None:
        _load().scanner_set_hw_trigger(self._cap, enable)

    def fire_trigger(self, capture_texture: bool = False) -> None:
        """비동기 트리거 발사. retrieve_frame()으로 결과 수신."""
        _load().scanner_fire_trigger(self._cap, capture_texture)

    def retrieve_frame(self, capture_texture: bool = False) -> CapturedFrameHandle:
        """fire_trigger() 후 결과 수신. GIL released."""
        return CapturedFrameHandle(
            _load().scanner_retrieve_frame(self._cap, capture_texture)
        )

    # ── 캡처 ────────────────────────────────────────────────────

    def capture(self, capture_texture: bool = False) -> CapturedFrameHandle:
        """1프레임 raw 캡처. GIL released."""
        return CapturedFrameHandle(
            _load().scanner_capture(self._cap, capture_texture)
        )

    def capture_texture_only(self) -> Tuple[Optional[np.ndarray], int]:
        """
        텍스처만 캡처.
        Returns (image_or_None, frame_number).
        image: (H,W,3) uint8, raw (보정 전).
        """
        return _load().scanner_capture_texture(self._cap)

    # ── FrameProcessor ──────────────────────────────────────────

    def create_frame_processor(
        self,
        settings: Optional[FrameProcessorSettings] = None,
    ) -> FrameProcessorHandle:
        """
        FrameProcessorHandle 생성.
        settings: FrameProcessorSettings 또는 None (기본값).
        """
        settings_dict = dataclasses.asdict(settings) if settings is not None else None
        proc_cap = _load().scanner_create_processor(self._cap, settings_dict)
        return FrameProcessorHandle(proc_cap)

    # ── 호환성 ──────────────────────────────────────────────────

    def scanner_capsule(self):
        """artec_base_py / artec_sdk_py 호환용 capsule."""
        return self._cap

    def __repr__(self) -> str:
        d = _load().scanner_id(self._cap)
        return f"ScannerHandle({d.get('name', '?')!r} serial={d.get('serial', '?')!r})"


# ──────────────────────────────────────────────────────────────────────
# ScannerManager
# ──────────────────────────────────────────────────────────────────────

class ScannerManager:
    """연결된 Artec 스캐너 열거 및 열기."""

    def list_scanners(self) -> list:
        """list[ScannerIdInfo] — 연결된 스캐너 목록."""
        return [ScannerIdInfo._from_dict(d) for d in _load().enumerate_scanners()]

    def open(self, index: int = 0) -> ScannerHandle:
        """인덱스로 스캐너 열기."""
        return ScannerHandle(_load().create_scanner(index))

    def open_by_serial(self, serial: str) -> ScannerHandle:
        """시리얼 번호로 스캐너 열기."""
        return ScannerHandle(_load().create_scanner_by_serial(serial))

    def __repr__(self) -> str:
        return "<ScannerManager>"


# ──────────────────────────────────────────────────────────────────────
# Module-level 편의 함수
# ──────────────────────────────────────────────────────────────────────

def list_scanners() -> list:
    """list[ScannerIdInfo] — 연결된 스캐너 목록."""
    return ScannerManager().list_scanners()


def open_scanner(index: int = 0) -> ScannerHandle:
    """인덱스로 스캐너 열기."""
    return ScannerManager().open(index)


def open_from_capsule(scanner_capsule) -> ScannerHandle:
    """
    기존 IScanner* capsule (artec_sdk_py.ArtecScanner.scanner_capsule())로
    ScannerHandle 생성. 하드웨어 재연결 없음 — capsule 참조만 유지.
    """
    return ScannerHandle(scanner_capsule)


# ──────────────────────────────────────────────────────────────────────
# High-level helper
# ──────────────────────────────────────────────────────────────────────

def capture_and_reconstruct(
    scanner_handle: ScannerHandle,
    with_texture: bool = False,
    settings: Optional[FrameProcessorSettings] = None,
):
    """
    1회 캡처 → FrameMeshHandle.

    Returns
    -------
    artec_base_py.FrameMeshHandle | None  (점 없으면 None)
    """
    proc  = scanner_handle.create_frame_processor(settings)
    frame = scanner_handle.capture(capture_texture=with_texture)

    t0 = time.perf_counter()
    fmh = proc.reconstruct_textured(frame) if with_texture else proc.reconstruct(frame)
    elapsed = time.perf_counter() - t0

    if fmh is None or fmh.vertex_count() == 0:
        return None

    print(
        f"  [artec_capturing] capture_and_reconstruct  elapsed={elapsed:.3f}s"
        f"  tex={with_texture}  verts={fmh.vertex_count():,}  faces={fmh.face_count():,}"
    )
    return fmh


# ──────────────────────────────────────────────────────────────────────
# 시각화
# ──────────────────────────────────────────────────────────────────────

def visualize_mesh(fmh, title: str = "Artec Mesh", show_texture: bool = True) -> None:
    """
    FrameMeshHandle을 Open3D로 시각화.
    좌표 단위 mm → m 변환. 텍스처/UV 있으면 적용.
    """
    try:
        import open3d as o3d
    except ImportError:
        raise ImportError("[artec_capturing] pip install open3d")

    verts = np.asarray(fmh.vertices(), dtype=np.float64) / 1000.0  # mm → m
    faces = np.asarray(fmh.faces(),    dtype=np.int32)

    mesh = o3d.geometry.TriangleMesh()
    mesh.vertices  = o3d.utility.Vector3dVector(verts)
    mesh.triangles = o3d.utility.Vector3iVector(faces)

    textured = False
    if show_texture and fmh.has_image():
        img = fmh.image()    # (H,W,3) uint8 | None
        uv  = fmh.uv()       # (N,2) float32 | None
        if img is not None and uv is not None:
            uv_np   = np.asarray(uv, dtype=np.float64)
            n_tris  = faces.shape[0]
            tri_uvs = np.empty((n_tris * 3, 2), dtype=np.float64)
            tri_uvs[0::3] = uv_np[faces[:, 0]]
            tri_uvs[1::3] = uv_np[faces[:, 1]]
            tri_uvs[2::3] = uv_np[faces[:, 2]]
            mesh.triangle_uvs = o3d.utility.Vector2dVector(tri_uvs)
            mesh.textures = [o3d.geometry.Image(np.ascontiguousarray(img))]
            mesh.triangle_material_ids = o3d.utility.IntVector([0] * n_tris)
            textured = True

    if not textured:
        mesh.compute_vertex_normals()

    axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)
    print(
        f"  [artec_capturing] visualize_mesh  "
        f"verts={verts.shape[0]:,}  faces={faces.shape[0]:,}  "
        f"{'textured' if textured else 'geometry only'}"
    )
    o3d.visualization.draw_geometries(
        [mesh, axes], window_name=title, width=1280, height=720,
    )


def visualize_texture_image(fmh, title: str = "Artec Texture") -> None:
    """FrameMeshHandle의 텍스처 이미지만 Open3D로 표시."""
    try:
        import open3d as o3d
    except ImportError:
        raise ImportError("[artec_capturing] pip install open3d")

    img = fmh.image()
    if img is None:
        print("  [artec_capturing] No texture image available.")
        return
    o3d.visualization.draw_geometries(
        [o3d.geometry.Image(np.ascontiguousarray(img))],
        window_name=title,
        width=img.shape[1], height=img.shape[0],
    )


# ──────────────────────────────────────────────────────────────────────
# 바인딩 검증
# ──────────────────────────────────────────────────────────────────────

def verify_artec_capturing(
    artec_client,
    visualize: bool = True,
    with_texture: bool = False,
) -> bool:
    """
    artec_capturing_py 바인딩 + Python 클래스 동작 검증.
    ArtecClient의 기존 연결을 재사용(하드웨어 재연결 없음).

    Parameters
    ----------
    artec_client : ArtecClient  (initialize() 완료 상태)
    visualize    : Open3D 뷰어 표시 여부
    with_texture : True → 텍스처 재구성 시도

    Returns
    -------
    bool  전체 통과 여부
    """
    failures: list[str] = []

    def chk(label: str, cond: bool, detail: str = "") -> bool:
        status = "OK" if cond else "FAIL"
        line = f"  [{status}] {label}"
        if detail:
            line += f"  ({detail})"
        print(line)
        if not cond:
            failures.append(label)
        return cond

    def section(title: str) -> None:
        print(f"\n{'─' * 55}")
        print(f"  {title}")
        print(f"{'─' * 55}")

    # ── 1. C++ 모듈 로드 ──────────────────────────────────────────
    section("1. artec_capturing_py 모듈 로드")

    try:
        mod = _load()
        chk("artec_capturing_py import", mod is not None)
    except Exception as e:
        chk("artec_capturing_py import", False, str(e))
        _print_result(failures)
        return False

    # ── 2. C++ free function 존재 확인 ───────────────────────────
    section("2. C++ free function 존재 확인")

    expected_fns = [
        "enumerate_scanners", "create_scanner", "create_scanner_by_serial",
        "scanner_id", "scanner_info", "scanner_frame_number",
        "scanner_fps", "scanner_set_fps", "scanner_max_fps",
        "scanner_flash_enabled", "scanner_enable_flash",
        "scanner_texture_flash_enabled", "scanner_enable_texture_flash",
        "scanner_texture_gain", "scanner_set_texture_gain",
        "scanner_texture_shutter_speed", "scanner_set_texture_shutter_speed",
        "scanner_auto_exposure_enabled", "scanner_enable_auto_exposure",
        "scanner_auto_white_balance_enabled", "scanner_enable_auto_white_balance",
        "scanner_hw_trigger_enabled", "scanner_set_hw_trigger",
        "scanner_capture", "scanner_capture_texture",
        "scanner_fire_trigger", "scanner_retrieve_frame",
        "scanner_create_processor", "scanner_init_processor_desc",
        "frame_number", "frame_has_texture", "frame_texture",
        "processor_reconstruct", "processor_reconstruct_textured",
        "processor_sensitivity", "processor_set_sensitivity",
        "processor_scanning_range", "processor_set_scanning_range",
    ]
    for fn in expected_fns:
        chk(f"  {fn}", hasattr(mod, fn))

    # ── 3. ScannerHandle 생성 (기존 연결 재사용) ─────────────────
    section("3. open_from_capsule() — 기존 연결 재사용")

    sh: Optional[ScannerHandle] = None
    try:
        sc_cap = artec_client._scanner.scanner_capsule()
        sh = open_from_capsule(sc_cap)
        chk("open_from_capsule() → ScannerHandle",
            isinstance(sh, ScannerHandle))
    except Exception as e:
        chk("open_from_capsule() 성공", False, str(e))

    if sh is None:
        _print_result(failures)
        return len(failures) == 0

    # ── 4. ScannerHandle 속성 조회 ───────────────────────────────
    section("4. ScannerHandle 속성 조회")

    try:
        id_info = sh.id()
        chk("id() → ScannerIdInfo",         isinstance(id_info, ScannerIdInfo))
        chk("  serial 비어있지 않음",         bool(id_info.serial),           id_info.serial)
        chk("  name 비어있지 않음",           bool(id_info.name),             id_info.name)
        chk("  scanner_type_str 비어있지 않음", bool(id_info.scanner_type_str), id_info.scanner_type_str)
        print(f"    {id_info}")
    except Exception as e:
        chk("id() 호출", False, str(e))

    try:
        info_dto = sh.info()
        chk("info() → ScannerInfoDTO",      isinstance(info_dto, ScannerInfoDTO))
        # Spider 등 일부 scanner는 SDK가 depth_map_size를 0으로 반환 (정상)
        if info_dto.depth_map_size_x == 0:
            print(f"    [INFO]   depth_map_size_x == 0 — 이 scanner type은 미지원 (정상)")
        else:
            chk("  depth_map_size_x > 0",
                info_dto.depth_map_size_x > 0,
                f"{info_dto.depth_map_size_x}x{info_dto.depth_map_size_y}")
        print(f"    {info_dto}")
    except Exception as e:
        chk("info() 호출", False, str(e))

    try:
        chk("frame_number() int",              isinstance(sh.frame_number(), int))
        chk("fps() > 0",                       sh.fps() > 0,     f"{sh.fps():.1f}")
        chk("max_fps() > 0",                   sh.max_fps() > 0, f"{sh.max_fps():.1f}")
        chk("flash_enabled() bool",            isinstance(sh.flash_enabled(), bool))
        chk("texture_flash_enabled() bool",    isinstance(sh.texture_flash_enabled(), bool))
        chk("auto_exposure_enabled() bool",    isinstance(sh.auto_exposure_enabled(), bool))
        chk("auto_white_balance_enabled() bool",isinstance(sh.auto_white_balance_enabled(), bool))
        chk("hw_trigger_enabled() bool",       isinstance(sh.hw_trigger_enabled(), bool))
        chk("texture_gain() float",            isinstance(sh.texture_gain(), float))
        chk("texture_shutter_speed() float",   isinstance(sh.texture_shutter_speed(), float))
    except Exception as e:
        chk("스캐너 속성 조회", False, str(e))

    # ── 5. FrameProcessorHandle 생성 ─────────────────────────────
    section("5. create_frame_processor()")

    proc: Optional[FrameProcessorHandle] = None
    try:
        proc = sh.create_frame_processor()
        chk("create_frame_processor() → FrameProcessorHandle",
            isinstance(proc, FrameProcessorHandle))
    except Exception as e:
        chk("create_frame_processor() 성공", False, str(e))

    if proc is not None:
        try:
            s = proc.settings()
            chk("settings() → FrameProcessorSettings", isinstance(s, FrameProcessorSettings))
            chk("  sensitivity float", isinstance(s.sensitivity, float), f"{s.sensitivity:.3f}")
            near, far = proc.scanning_range()
            chk("  scanning_range() tuple", isinstance(near, float), f"near={near:.0f}mm far={far:.0f}mm")
        except Exception as e:
            chk("FrameProcessorHandle 속성", False, str(e))

    # ── 6. capture() → CapturedFrameHandle ──────────────────────
    section("6. capture() → CapturedFrameHandle")

    raw_frame: Optional[CapturedFrameHandle] = None
    try:
        raw_frame = sh.capture(capture_texture=with_texture)
        chk("capture() → CapturedFrameHandle", isinstance(raw_frame, CapturedFrameHandle))
    except Exception as e:
        chk("capture() 성공", False, str(e))

    if raw_frame is not None:
        try:
            chk("  frame_number() int",  isinstance(raw_frame.frame_number(), int))
            chk("  has_texture() bool",  isinstance(raw_frame.has_texture(), bool))
            s = raw_frame.summary()
            chk("  summary() → CapturedFrameSummary", isinstance(s, CapturedFrameSummary))
            print(f"    {s}")
        except Exception as e:
            chk("CapturedFrameHandle 속성", False, str(e))

    # ── 7. reconstruct() → FrameMeshHandle ──────────────────────
    section(f"7. reconstruct({'textured' if with_texture else 'geometry'}) → FrameMeshHandle")

    fmh = None
    if proc is not None and raw_frame is not None:
        for tex in ([with_texture] + ([] if not with_texture else [False])):
            try:
                t0 = time.perf_counter()
                fmh = proc.reconstruct_textured(raw_frame) if tex else proc.reconstruct(raw_frame)
                elapsed = time.perf_counter() - t0
                chk(f"reconstruct(textured={tex}) 성공", True, f"{elapsed:.3f}s")
                break
            except Exception as e:
                chk(f"reconstruct(textured={tex}) 성공", False, str(e))

    if fmh is None:
        print("    ※ FrameMeshHandle 없음 — 스캐너 앞에 물체 없음")
    else:
        try:
            verts = fmh.vertices()
            faces = fmh.faces()
            chk("  vertices() (N,3) float32",
                verts.ndim == 2 and verts.shape[1] == 3 and verts.dtype == np.float32,
                str(verts.shape))
            chk("  faces() (M,3) int32",
                faces.ndim == 2 and faces.shape[1] == 3 and faces.dtype == np.int32,
                str(faces.shape))
            chk("  vertex_count > 0",  fmh.vertex_count() > 0, f"{fmh.vertex_count():,}")
            chk("  face_count > 0",    fmh.face_count() > 0,   f"{fmh.face_count():,}")
            chk("  is_textured() bool",isinstance(fmh.is_textured(), bool))
            chk("  has_image() bool",  isinstance(fmh.has_image(), bool))
            uv  = fmh.uv()
            chk("  uv() (N,2) or None",
                uv is None or (uv.ndim == 2 and uv.shape[1] == 2))
            s = fmh.summary()
            chk("  summary() → FrameMeshSummary",
                type(s).__name__ == "FrameMeshSummary")
            print(f"    {s}")
        except Exception as e:
            chk("FrameMeshHandle 속성 조회", False, str(e))

        # ── 8. Open3D 시각화 ─────────────────────────────────────
        if visualize:
            section("8. Open3D 시각화")
            try:
                title = (f"artec_capturing — {sh.id().name}  "
                         f"({fmh.vertex_count():,} verts)")
                visualize_mesh(fmh, title=title, show_texture=True)
                chk("시각화 완료", True)
            except ImportError:
                print("    ※ Open3D 없음 — pip install open3d")
            except Exception as e:
                chk("시각화", False, str(e))

    _print_result(failures)
    return len(failures) == 0


def _print_result(failures: list) -> None:
    print(f"\n{'═' * 55}")
    if failures:
        print(f"  결과: {len(failures)}개 실패")
        for f in failures:
            print(f"    - {f}")
    else:
        print("  결과: 전체 통과")
    print(f"{'═' * 55}")
