# mms/sensor/phoxi_instant_meshing.py
#
# Photoneo PhoXiInstantMeshing C API → Python ctypes 래퍼.
# GPU(CUDA/OpenGL) 기반 TSDF 볼류메트릭 퓨전으로 다중 스캔 → 메쉬.
#
# 참조 헤더:
#   C:\Program Files\Photoneo\PhoXiInstantMeshing\2.2.0\API\include\
#     PhoXiInstantMeshing/C_API/InstantMeshing.h
#     PhoXiInstantMeshing/C_API/Frame.h
#     PhoXiInstantMeshing/C_API/DeviceParams.h
#     PhoXiInstantMeshing/C_API/Settings.h
#     PhoXiInstantMeshing/C_API/PointCloud.h
#     PhoXiInstantMeshing/C_API/CroppingVolume.h
#     Utils/C_Vector.h, Utils/C_Matrix.h

from __future__ import annotations

import ctypes
import logging
import os
from pathlib import Path
from typing import Optional

import numpy as np
import open3d as o3d

log = logging.getLogger(__name__)

# ------------------------------------------------------------------
# DLL 경로
# ------------------------------------------------------------------
_DLL_DIR = Path(
    r"C:\Program Files\Photoneo\PhoXiInstantMeshing\2.2.0\API\bin\Release"
)
_DLL_NAME = "PhoXiInstantMeshing_msvc141.dll"


# ==================================================================
# ctypes struct 정의 (헤더와 byte-for-byte 일치)
# ==================================================================

# --- Utils/C_Vector.h ---

class _Vec3f(ctypes.Structure):
    """utils_Vec3f : float vec[3]"""
    _fields_ = [("vec", ctypes.c_float * 3)]

class _Vec3d(ctypes.Structure):
    """utils_Vec3d : double vec[3]"""
    _fields_ = [("vec", ctypes.c_double * 3)]

class _Vec3us(ctypes.Structure):
    """utils_Vec3us : uint16_t vec[3]"""
    _fields_ = [("vec", ctypes.c_uint16 * 3)]

class _Vec3u(ctypes.Structure):
    """utils_Vec3u : uint32_t vec[3]"""
    _fields_ = [("vec", ctypes.c_uint32 * 3)]

# --- Utils/C_Matrix.h  (column-major) ---

class _Mat4f(ctypes.Structure):
    """utils_Mat4f : float mat[16]  (column-major)"""
    _fields_ = [("mat", ctypes.c_float * 16)]

# --- DeviceParams.h ---

class _CameraIntrinsics(ctypes.Structure):
    _fields_ = [
        ("fx", ctypes.c_double),
        ("fy", ctypes.c_double),
        ("cx", ctypes.c_double),
        ("cy", ctypes.c_double),
    ]

class _DistortionCoefficients(ctypes.Structure):
    _fields_ = [
        ("k1", ctypes.c_double),
        ("k2", ctypes.c_double),
        ("p1", ctypes.c_double),
        ("p2", ctypes.c_double),
        ("k3", ctypes.c_double),
    ]

class _DeviceParams(ctypes.Structure):
    _fields_ = [
        ("id",                          ctypes.c_char_p),
        ("width",                       ctypes.c_int32),
        ("height",                      ctypes.c_int32),
        ("orthogonal_projection",       ctypes.c_bool),
        ("camera_intrinsics",           _CameraIntrinsics),
        ("distortion_coefficients",     _DistortionCoefficients),
        ("ortho_width",                 ctypes.c_float),
        ("ortho_height",                ctypes.c_float),
        ("min_depth",                   ctypes.c_float),
        ("max_depth",                   ctypes.c_float),
        ("min_color_intensity",         ctypes.c_float),
        ("max_color_intensity",         ctypes.c_float),
        ("tracking_enabled",            ctypes.c_bool),
        ("texture_tracking_enabled",    ctypes.c_bool),
        ("movement_prediction_enabled", ctypes.c_bool),
        ("force_camera_space_enabled",  ctypes.c_bool),
        ("calibration_matrix",          _Mat4f),
    ]

# --- Settings.h ---

class _Settings(ctypes.Structure):
    _fields_ = [
        ("max_gpu_memory_usage",               ctypes.c_uint8),
        ("voxel_size",                         ctypes.c_float),
        ("min_voxel_consensus",                ctypes.c_uint8),
        ("streaming_enabled",                  ctypes.c_bool),
        ("rasterization_engine",               ctypes.c_uint8),
        ("color_intensity_auto_minmax_enabled", ctypes.c_bool),
        ("min_color_intensity",                ctypes.c_float),
        ("max_color_intensity",                ctypes.c_float),
        ("tracking_lost_tolerance",            ctypes.c_float),
    ]

# --- Frame.h ---

class _Frame(ctypes.Structure):
    _fields_ = [
        ("timestamp",               ctypes.c_double),
        ("current_camera_position", _Vec3d),
        ("current_camera_x_axis",   _Vec3d),
        ("current_camera_y_axis",   _Vec3d),
        ("current_camera_z_axis",   _Vec3d),
        ("width",                   ctypes.c_int32),
        ("height",                  ctypes.c_int32),
        ("point_cloud",             ctypes.POINTER(_Vec3f)),
        ("texture",                 ctypes.POINTER(ctypes.c_float)),
        ("texture_rgb",             ctypes.POINTER(_Vec3us)),   # NULL = 없음
    ]

# --- PointCloud.h ---

class _PointCloud(ctypes.Structure):
    _fields_ = [
        ("size",      ctypes.c_size_t),
        ("positions", ctypes.POINTER(_Vec3f)),
        ("normals",   ctypes.POINTER(_Vec3f)),
        ("colors",    ctypes.POINTER(_Vec3f)),
    ]

class _Mesh(ctypes.Structure):
    _fields_ = [
        ("points",     _PointCloud),
        ("faces_size", ctypes.c_size_t),
        ("faces",      ctypes.POINTER(_Vec3u)),
    ]

# --- CroppingVolume.h ---

class _CroppingAabb(ctypes.Structure):
    _fields_ = [("min", _Vec3f), ("max", _Vec3f)]

class _CroppingSphere(ctypes.Structure):
    _fields_ = [("origin", _Vec3f), ("radius", ctypes.c_float)]

class _CroppingVolumeUnion(ctypes.Union):
    _fields_ = [("aabb", _CroppingAabb), ("sphere", _CroppingSphere)]

class _CroppingVolume(ctypes.Structure):
    _fields_ = [("type", ctypes.c_int), ("volume", _CroppingVolumeUnion)]

# Crop type enum
_CROP_NONE   = 0
_CROP_AABB   = 1
_CROP_SPHERE = 2

# Rasterization engine enum
_ENGINE_OPENGL = 0
_ENGINE_CUDA   = 1
_ENGINE_VULKAN = 2

# Message callback type
_MessageCallback = ctypes.CFUNCTYPE(None, ctypes.c_int, ctypes.c_char_p)


# ------------------------------------------------------------------
# 행렬 유틸
# ------------------------------------------------------------------

def _identity_mat4f() -> _Mat4f:
    """Column-major 4x4 단위행렬."""
    m = _Mat4f()
    for i in range(4):
        m.mat[i * 4 + i] = 1.0
    return m


def _numpy_to_mat4f(T: np.ndarray) -> _Mat4f:
    """(4,4) numpy (row-major) → column-major _Mat4f."""
    flat = T.T.astype(np.float32).flatten()
    m = _Mat4f()
    for i, v in enumerate(flat):
        m.mat[i] = float(v)
    return m


def _no_crop() -> _CroppingVolume:
    cv = _CroppingVolume()
    cv.type = _CROP_NONE
    return cv


def _aabb_crop(
    min_xyz: tuple[float, float, float],
    max_xyz: tuple[float, float, float],
) -> _CroppingVolume:
    cv = _CroppingVolume()
    cv.type = _CROP_AABB
    for i in range(3):
        cv.volume.aabb.min.vec[i] = float(min_xyz[i])
        cv.volume.aabb.max.vec[i] = float(max_xyz[i])
    return cv


# ==================================================================
# PhoxiInstantMeshingWrapper
# ==================================================================

class PhoxiInstantMeshingWrapper:
    """
    Photoneo PhoXiInstantMeshing C API의 Python ctypes 래퍼.

    GPU(CUDA/OpenGL) TSDF 볼류메트릭 퓨전으로
    다중 PhoXi 스캔을 실시간으로 통합 → 메쉬 생성.

    Coordinate convention
    ---------------------
    - point_cloud (phoim_Frame) : 카메라(C) 프레임, mm 단위.
                                  GenTL Range 컴포넌트 원본 그대로.
    - T_CB (4,4)               : C→B 변환. add_scan()에서
                                  camera_position/axes를 추출.

    Usage
    -----
    wrapper = PhoxiInstantMeshingWrapper()
    # 스캔마다 (해상도는 첫 add_scan 시점에 자동 감지):
    wrapper.add_scan(points_C_flat, texture_flat, T_CB, timestamp)
    # 완료 후:
    mesh = wrapper.get_mesh()          # o3d.geometry.TriangleMesh
    wrapper.cleanup()
    """

    def __init__(
        self,
        device_id: str = "phoxi_0",
        voxel_size_mm: float = 1.0,
        min_voxel_consensus: int = 2,
        max_gpu_memory_pct: int = 85,
        min_depth_mm: float = 200.0,
        max_depth_mm: float = 2500.0,
        ortho_width_mm: float = 500.0,
        ortho_height_mm: float = 500.0,
        tracking_enabled: bool = False,
        dll_path: Optional[Path] = None,
    ) -> None:
        """
        Parameters
        ----------
        device_id           : 임의 식별자 (add_scan 호출과 일치해야 함).
        voxel_size_mm       : TSDF 복셀 크기 (mm). 작을수록 세밀/느림.
        min_voxel_consensus : 복셀 유효 판정에 필요한 최소 스캔 수.
        max_gpu_memory_pct  : GPU VRAM 사용 상한 (0-100).
        min/max_depth_mm    : 유효 깊이 범위 (mm).
        ortho_width/height_mm : PhoXi 직교 투영 스캔 볼륨 크기 (mm).
        tracking_enabled    : False → 로봇 FK 포즈 사용 (MMS 권장).
                              True  → 시각적 텍스처 트래킹.
        dll_path            : DLL 경로 오버라이드.

        Note
        ----
        해상도(width, height)는 첫 add_scan() 호출 시 입력 데이터로부터
        자동 감지되어 phoim_SetUp이 실행됩니다.
        """
        self._device_id_bytes = device_id.encode()
        self._width:  Optional[int] = None   # 첫 add_scan 시 자동 설정
        self._height: Optional[int] = None
        self._dll: Optional[ctypes.CDLL] = None
        self._active = False
        self._scan_count = 0

        # _setup에 넘길 파라미터 저장 (lazy init용)
        self._setup_kwargs = dict(
            voxel_size_mm       = voxel_size_mm,
            min_voxel_consensus = min_voxel_consensus,
            max_gpu_memory_pct  = max_gpu_memory_pct,
            min_depth_mm        = min_depth_mm,
            max_depth_mm        = max_depth_mm,
            ortho_width_mm      = ortho_width_mm,
            ortho_height_mm     = ortho_height_mm,
            tracking_enabled    = tracking_enabled,
        )

        # 콜백 객체를 GC로부터 보호
        self._callback = _MessageCallback(self._on_message)

        dll_path = dll_path or (_DLL_DIR / _DLL_NAME)
        self._load_dll(dll_path)
        # _setup은 첫 add_scan에서 호출됨 (해상도 자동 감지)

    # ------------------------------------------------------------------
    # DLL 로드 및 함수 시그니처 바인딩
    # ------------------------------------------------------------------

    def _load_dll(self, dll_path: Path) -> None:
        if not dll_path.exists():
            raise FileNotFoundError(
                f"DLL 없음: {dll_path}\n"
                "PhoXiInstantMeshing 2.2.0 설치 확인."
            )

        # 의존 DLL 경로 등록
        # - API/bin/Release : PhoXiInstantMeshing_msvc141.dll 외 8개
        # - 2.2.0/          : PhoXi_API, glbinding, vulkan-1 등 런타임 의존 DLL
        _dep_dirs = [
            dll_path.parent,
            Path(r"C:\Program Files\Photoneo\PhoXiInstantMeshing\2.2.0"),
        ]
        for d in _dep_dirs:
            if d.exists():
                os.add_dll_directory(str(d))
                log.debug(f"[InstantMeshing] DLL 경로 등록: {d}")

        # PATH에도 추가 (일부 로더는 add_dll_directory를 무시함)
        extra = ";".join(str(d) for d in _dep_dirs if d.exists())
        os.environ["PATH"] = extra + ";" + os.environ.get("PATH", "")

        dll = ctypes.CDLL(str(dll_path))

        dll.phoim_SetUp.argtypes = [
            ctypes.POINTER(_DeviceParams),
            ctypes.c_size_t,
            _Settings,
            _MessageCallback,
        ]
        dll.phoim_SetUp.restype = None

        dll.phoim_CleanUp.argtypes = []
        dll.phoim_CleanUp.restype = None

        dll.phoim_Clear.argtypes = []
        dll.phoim_Clear.restype = None

        dll.phoim_ResetTracking.argtypes = []
        dll.phoim_ResetTracking.restype = None

        # 구조체 by-value 전달/반환 (Windows x64 ABI — ctypes 자동 처리)
        dll.phoim_AddScan.argtypes = [_Frame, ctypes.c_char_p, _Mat4f]
        dll.phoim_AddScan.restype  = _Mat4f

        dll.phoim_GetMesh.argtypes = [_CroppingVolume]
        dll.phoim_GetMesh.restype  = _Mesh

        dll.phoim_FreeMesh.argtypes = [_Mesh]
        dll.phoim_FreeMesh.restype  = None

        dll.phoim_ExportMesh.argtypes = [ctypes.c_char_p, _CroppingVolume]
        dll.phoim_ExportMesh.restype  = ctypes.c_bool

        dll.phoim_SetCroppingVolume.argtypes = [_CroppingVolume]
        dll.phoim_SetCroppingVolume.restype  = None

        dll.phoim_ResetCroppingVolume.argtypes = []
        dll.phoim_ResetCroppingVolume.restype  = None

        self._dll = dll
        log.info(f"[InstantMeshing] DLL 로드 완료: {dll_path}")

    # ------------------------------------------------------------------
    # 초기화
    # ------------------------------------------------------------------

    def _setup(
        self,
        width: int,
        height: int,
        voxel_size_mm: float,
        min_voxel_consensus: int,
        max_gpu_memory_pct: int,
        min_depth_mm: float,
        max_depth_mm: float,
        ortho_width_mm: float,
        ortho_height_mm: float,
        tracking_enabled: bool,
    ) -> None:
        identity = _identity_mat4f()

        device = _DeviceParams()
        device.id                          = self._device_id_bytes
        device.width                       = width
        device.height                      = height
        device.orthogonal_projection       = True   # CalibratedABC_Grid = 직교 투영
        device.ortho_width                 = float(ortho_width_mm)
        device.ortho_height                = float(ortho_height_mm)
        device.min_depth                   = float(min_depth_mm)
        device.max_depth                   = float(max_depth_mm)
        device.min_color_intensity         = 0.0
        device.max_color_intensity         = 1.0
        device.tracking_enabled            = tracking_enabled
        device.texture_tracking_enabled    = tracking_enabled
        device.movement_prediction_enabled = False
        device.force_camera_space_enabled  = False
        device.calibration_matrix          = identity

        settings = _Settings()
        settings.max_gpu_memory_usage               = max_gpu_memory_pct
        settings.voxel_size                         = float(voxel_size_mm)
        settings.min_voxel_consensus                = min_voxel_consensus
        settings.streaming_enabled                  = True
        settings.rasterization_engine               = _ENGINE_OPENGL
        settings.color_intensity_auto_minmax_enabled = True
        settings.min_color_intensity                = 0.0
        settings.max_color_intensity                = 1.0
        settings.tracking_lost_tolerance            = 1.0

        device_arr = (_DeviceParams * 1)(device)
        self._dll.phoim_SetUp(device_arr, 1, settings, self._callback)
        self._active = True
        log.info("[InstantMeshing] 초기화 완료.")

    # ------------------------------------------------------------------
    # 콜백
    # ------------------------------------------------------------------

    @staticmethod
    def _on_message(msg_type: int, msg: bytes) -> None:
        level_map = {0: log.debug, 1: log.info, 2: log.warning, 3: log.error, 4: log.critical}
        fn = level_map.get(msg_type, log.info)
        fn(f"[InstantMeshing] {msg.decode(errors='replace')}")

    # ------------------------------------------------------------------
    # 공개 API
    # ------------------------------------------------------------------

    def add_scan(
        self,
        points_S_flat: np.ndarray,
        texture_flat: np.ndarray,
        T_CB: np.ndarray,
        timestamp: float,
        width: int = 0,
        height: int = 0,
    ) -> bool:
        """
        TSDF에 스캔 1회 추가.

        Parameters
        ----------
        points_S_flat : (H*W, 3) float32, mm 단위, 센서(S) 프레임.
                        무효점 = (0,0,0). GenTL Range 컴포넌트 원본.
        texture_flat  : (H*W,) 또는 (H*W*3,) float32. Intensity 원본 값.
                        RGB8(3채널)이면 자동으로 grayscale 변환.
        T_CB         : (4,4) float64 또는 float32. 센서→베이스 변환.
        timestamp     : 단조 시간 (초).
        width, height : 센서 해상도. 첫 호출 시 필수.

        Returns
        -------
        bool : True = 추가 성공 (tracking valid), False = tracking lost
        """
        # 첫 호출 시 phoim_SetUp (width/height는 호출자가 직접 전달)
        if not self._active:
            if width == 0 or height == 0:
                raise ValueError("첫 add_scan 호출 시 width, height 를 전달해야 합니다.")
            self._width  = width
            self._height = height
            log.info(f"[InstantMeshing] 해상도: {width}×{height} ({width*height:,} pts)")
            self._setup(width=width, height=height, **self._setup_kwargs)

        n = self._width * self._height

        pts = np.ascontiguousarray(points_S_flat.reshape(n, 3), dtype=np.float32)

        # texture: RGB8(H*W*3) → grayscale(H*W), 그 외는 그대로
        tex_raw = texture_flat.reshape(-1).astype(np.float32)
        if tex_raw.size == n * 3:
            tex = np.ascontiguousarray(tex_raw.reshape(n, 3).mean(axis=1), dtype=np.float32)
        else:
            tex = np.ascontiguousarray(tex_raw.reshape(n), dtype=np.float32)

        pc_ptr = pts.ctypes.data_as(ctypes.POINTER(_Vec3f))
        tx_ptr = tex.ctypes.data_as(ctypes.POINTER(ctypes.c_float))

        T = np.asarray(T_CB, dtype=np.float64)

        def _v3d(col: int) -> _Vec3d:
            v = _Vec3d()
            v.vec[0] = T[0, col]
            v.vec[1] = T[1, col]
            v.vec[2] = T[2, col]
            return v

        def _pos() -> _Vec3d:
            v = _Vec3d()
            v.vec[0] = T[0, 3]
            v.vec[1] = T[1, 3]
            v.vec[2] = T[2, 3]
            return v

        frame = _Frame()
        frame.timestamp               = float(timestamp)
        frame.current_camera_position = _pos()
        frame.current_camera_x_axis   = _v3d(0)
        frame.current_camera_y_axis   = _v3d(1)
        frame.current_camera_z_axis   = _v3d(2)
        frame.width                   = self._width
        frame.height                  = self._height
        frame.point_cloud             = pc_ptr
        frame.texture                 = tx_ptr
        frame.texture_rgb             = None

        result: _Mat4f = self._dll.phoim_AddScan(
            frame, self._device_id_bytes, _identity_mat4f()
        )

        # INVALID_MATRIX = 모든 원소 0
        tracking_ok = any(result.mat[i] != 0.0 for i in range(16))

        if tracking_ok:
            self._scan_count += 1
            log.debug(f"[InstantMeshing] 스캔 {self._scan_count} 추가 완료")
        else:
            log.warning("[InstantMeshing] add_scan: tracking lost")

        return tracking_ok

    def get_mesh(
        self,
        crop_min_mm: Optional[tuple[float, float, float]] = None,
        crop_max_mm: Optional[tuple[float, float, float]] = None,
    ) -> o3d.geometry.TriangleMesh:
        """
        현재까지 퓨전된 TSDF 메쉬 반환.

        Parameters
        ----------
        crop_min_mm, crop_max_mm : (x, y, z) mm 단위 AABB 클리핑.
                                   둘 다 None이면 전체 영역.

        Returns
        -------
        o3d.geometry.TriangleMesh  (B 프레임 기준, mm 단위)
        """
        if not self._active:
            raise RuntimeError("초기화되지 않음.")

        if crop_min_mm is not None and crop_max_mm is not None:
            cv = _aabb_crop(crop_min_mm, crop_max_mm)
        else:
            cv = _no_crop()

        raw: _Mesh = self._dll.phoim_GetMesh(cv)

        n_pts = raw.points.size
        n_tri = raw.faces_size

        if n_pts == 0 or n_tri == 0:
            log.warning("[InstantMeshing] 빈 메쉬 반환됨.")
            self._dll.phoim_FreeMesh(raw)
            return o3d.geometry.TriangleMesh()

        # ctypes 포인터 → numpy (zero-copy view 후 .copy()로 소유권 확보)
        float_ptr = ctypes.cast(raw.points.positions, ctypes.POINTER(ctypes.c_float))
        verts = np.ctypeslib.as_array(float_ptr, shape=(n_pts * 3,)).reshape(n_pts, 3).copy()

        uint_ptr = ctypes.cast(raw.faces, ctypes.POINTER(ctypes.c_uint32))
        tris = np.ctypeslib.as_array(uint_ptr, shape=(n_tri * 3,)).reshape(n_tri, 3).copy()

        has_normals = raw.points.normals is not None
        if has_normals:
            nrm_ptr = ctypes.cast(raw.points.normals, ctypes.POINTER(ctypes.c_float))
            norms = np.ctypeslib.as_array(nrm_ptr, shape=(n_pts * 3,)).reshape(n_pts, 3).copy()

        has_colors = raw.points.colors is not None
        if has_colors:
            col_ptr = ctypes.cast(raw.points.colors, ctypes.POINTER(ctypes.c_float))
            colors = np.ctypeslib.as_array(col_ptr, shape=(n_pts * 3,)).reshape(n_pts, 3).copy()

        # 반드시 numpy copy 이후에 Free
        self._dll.phoim_FreeMesh(raw)

        mesh = o3d.geometry.TriangleMesh()
        mesh.vertices  = o3d.utility.Vector3dVector(verts.astype(np.float64))
        mesh.triangles = o3d.utility.Vector3iVector(tris.astype(np.int32))

        if has_normals:
            mesh.vertex_normals = o3d.utility.Vector3dVector(norms.astype(np.float64))
        else:
            mesh.compute_vertex_normals()

        if has_colors:
            mesh.vertex_colors = o3d.utility.Vector3dVector(colors.astype(np.float64))

        log.info(
            f"[InstantMeshing] 메쉬 완료: "
            f"vertices={n_pts:,}  triangles={n_tri:,}"
        )
        return mesh

    def export_mesh(
        self,
        file_path: str,
        crop_min_mm: Optional[tuple[float, float, float]] = None,
        crop_max_mm: Optional[tuple[float, float, float]] = None,
    ) -> bool:
        """
        메쉬를 파일로 저장 (라이브러리 직접 출력).

        Parameters
        ----------
        file_path : str
            출력 경로. 확장자로 포맷 결정 (.ply, .obj, .stl, .fbx).

        Returns
        -------
        bool : 성공 여부
        """
        if not self._active:
            raise RuntimeError("초기화되지 않음.")

        if crop_min_mm is not None and crop_max_mm is not None:
            cv = _aabb_crop(crop_min_mm, crop_max_mm)
        else:
            cv = _no_crop()

        ok = self._dll.phoim_ExportMesh(file_path.encode(), cv)
        if ok:
            log.info(f"[InstantMeshing] 메쉬 저장 완료: {file_path}")
        else:
            log.error(f"[InstantMeshing] 메쉬 저장 실패: {file_path}")
        return bool(ok)

    def reset(self) -> None:
        """TSDF 초기화 (다음 스캔 세션 시작 전 호출)."""
        if self._active:
            self._dll.phoim_Clear()
            self._scan_count = 0
            log.info("[InstantMeshing] TSDF 초기화.")

    def reset_tracking(self) -> None:
        """트래킹만 리셋 (TSDF 데이터 유지)."""
        if self._active:
            self._dll.phoim_ResetTracking()

    def cleanup(self) -> None:
        """DLL 세션 종료 및 리소스 해제."""
        if self._active:
            self._dll.phoim_CleanUp()
            self._active = False
            log.info("[InstantMeshing] 종료.")

    def __enter__(self) -> "PhoxiInstantMeshingWrapper":
        return self

    def __exit__(self, *_) -> None:
        self.cleanup()

    @property
    def scan_count(self) -> int:
        return self._scan_count


# ==================================================================
# 기본 동작 테스트
# ==================================================================
if __name__ == "__main__":
    import sys
    import time

    logging.basicConfig(level=logging.DEBUG)

    _PROJECT_ROOT = Path(__file__).resolve().parents[2]
    sys.path.insert(0, str(_PROJECT_ROOT))

    from mms.sensor.phoxi_client import PhoxiClient, PhoxiConfig

    cfg = PhoxiConfig(
        sensor_frames_yaml=str(_PROJECT_ROOT / "config" / "sensor_frames.yaml"),
        T_EC_key="T_EC_phoxi",
        serial_number="SEA-023",
        trigger_timeout_s=15.0,
    )
    client = PhoxiClient(cfg)

    # 해상도는 첫 프레임 취득 전에 확인 필요 → 일단 PhoXi 3D M 기본값 사용
    WIDTH, HEIGHT = 2064, 1544

    mesher = PhoxiInstantMeshingWrapper(
        width=WIDTH,
        height=HEIGHT,
        voxel_size_mm=1.5,
        min_voxel_consensus=2,
        tracking_enabled=False,   # 로봇 FK 포즈 사용
    )

    try:
        client.initialize()

        print("=== 스캔 3회 취득 후 메쉬 생성 ===")
        dummy_ee = np.eye(4, dtype=np.float64)
        T_EC = client.T_EC
        T_CB = dummy_ee @ np.linalg.inv(T_EC)

        for i in range(3):
            input(f"\n[{i+1}/3] 로봇을 위치시킨 후 Enter ▶")

            # PhoxiClient 내부 버퍼에 접근하기 위해 직접 트리거
            from mms.sensor.phoxi_client import _enabled_components
            client._features.TriggerSoftware.execute()
            comp_names = _enabled_components(client._features)

            with client._ia.fetch(timeout=15.0) as buf:
                from genicam.genapi import NodeMap
                comps = dict(zip(comp_names, buf.payload.components))
                range_comp = comps["Range"]
                pts_S = range_comp.data.reshape(-1, 3).copy().astype(np.float32)
                h, w = range_comp.height, range_comp.width

                tex_comp = comps.get("Intensity")
                if tex_comp is not None:
                    tex = tex_comp.data.astype(np.float32).reshape(-1)
                else:
                    tex = np.zeros(h * w, dtype=np.float32)

            ok = mesher.add_scan(pts_S, tex, T_CB, time.perf_counter())
            print(f"  add_scan: {'OK' if ok else 'tracking lost'}")

        print(f"\n총 {mesher.scan_count}회 스캔 → 메쉬 생성 중...")
        mesh = mesher.get_mesh()

        if len(mesh.vertices) > 0:
            o3d.visualization.draw_geometries([mesh], window_name="InstantMeshing 결과")
            mesher.export_mesh("output_mesh.ply")
        else:
            print("메쉬 없음 — 스캔 수를 늘리거나 voxel_size를 키워보세요.")

    finally:
        mesher.cleanup()
        client.shutdown()
