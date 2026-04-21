# mms/sensor/artec/artec_base.py
#
# Artec SDK Base API Python 래퍼.
# artec_base_py (C++ pybind11)는 capsule 기반 free function만 노출.
# 이 파일에서 클래스 계층(FrameMeshHandle, ScanHandle, ModelHandle)과
# DTO(@dataclass)를 Python으로 구현한다.
#
# Exposes
# -------
#   DTOs (dataclass)
#     FrameMeshSummary, ScanSummary, MeshSummary
#
#   Handles (Python class, _cap 으로 capsule 보유)
#     FrameMeshHandle
#     ScanHandle
#     ModelHandle
#
#   Module-level helpers
#     create_model()
#     capture_frame_handle(artec_client, ...)  -> FrameMeshHandle
#     capture_to_model(artec_client, ...)      -> ModelHandle

from __future__ import annotations

import os
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import numpy as np

_ARTEC_SDK_BIN = Path(r"C:\Program Files\Artec\Artec 3D Scanning SDK\bin-x64")
_HERE = Path(__file__).parent

_base_py = None  # lazy-loaded artec_base_py module


def _load():
    """artec_base_py .pyd 를 lazy-load하고 반환."""
    global _base_py
    if _base_py is not None:
        return _base_py

    if _ARTEC_SDK_BIN.exists():
        os.add_dll_directory(str(_ARTEC_SDK_BIN))
        os.environ["PATH"] = str(_ARTEC_SDK_BIN) + ";" + os.environ.get("PATH", "")

    if str(_HERE) not in sys.path:
        sys.path.insert(0, str(_HERE))

    try:
        import artec_base_py as _m
        _base_py = _m
        return _base_py
    except ImportError as e:
        raise ImportError(
            "[artec_base] artec_base_py 모듈 로드 실패.\n"
            "  cmake --build build --config Release 후 artec_base_py.pyd 생성 확인.\n"
            f"  원인: {e}"
        ) from e


# 하위 호환성: artec_client.py 가 _load_artec_base_py() 이름으로 호출
def _load_artec_base_py():
    return _load()


# ==============================================================
# DTOs
# ==============================================================

@dataclass
class FrameMeshSummary:
    vertex_count:  int  = 0
    face_count:    int  = 0
    textured:      bool = False
    image_width:   int  = 0
    image_height:  int  = 0

    def __repr__(self) -> str:
        return (f"FrameMeshSummary(verts={self.vertex_count}, faces={self.face_count}, "
                f"textured={self.textured}, img={self.image_width}x{self.image_height})")


@dataclass
class ScanSummary:
    frame_count: int = 0

    def __repr__(self) -> str:
        return f"ScanSummary(frames={self.frame_count})"


@dataclass
class MeshSummary:
    vertex_count: int  = 0
    face_count:   int  = 0
    textured:     bool = False
    scan_count:   int  = 0

    def __repr__(self) -> str:
        return (f"MeshSummary(verts={self.vertex_count}, faces={self.face_count}, "
                f"textured={self.textured}, scans={self.scan_count})")


# ==============================================================
# FrameMeshHandle
# ==============================================================

class FrameMeshHandle:
    """
    IFrameMesh* capsule 래퍼.
    재구성된 단일 프레임 메쉬(정점, 삼각형, 텍스처)에 접근.
    """

    def __init__(self, cap) -> None:
        self._cap = cap

    def vertices(self) -> np.ndarray:
        """Return (N,3) float32 vertex positions in mm, sensor frame."""
        return _load().frame_mesh_vertices(self._cap)

    def faces(self) -> np.ndarray:
        """Return (M,3) int32 triangle face indices."""
        return _load().frame_mesh_faces(self._cap)

    def uv(self) -> Optional[np.ndarray]:
        """Return (N,2) float32 UV coordinates, or None."""
        return _load().frame_mesh_uv(self._cap)

    def image(self) -> Optional[np.ndarray]:
        """Return (H,W,3) uint8 RGB texture image, or None."""
        return _load().frame_mesh_image(self._cap)

    def is_textured(self) -> bool:
        return _load().frame_mesh_is_textured(self._cap)

    def has_image(self) -> bool:
        return _load().frame_mesh_has_image(self._cap)

    def vertex_count(self) -> int:
        return _load().frame_mesh_vertex_count(self._cap)

    def face_count(self) -> int:
        return _load().frame_mesh_face_count(self._cap)

    def summary(self) -> FrameMeshSummary:
        w, h = _load().frame_mesh_image_dims(self._cap)
        return FrameMeshSummary(
            vertex_count=self.vertex_count(),
            face_count=self.face_count(),
            textured=self.is_textured(),
            image_width=w,
            image_height=h,
        )

    def __repr__(self) -> str:
        return (f"<FrameMeshHandle verts={self.vertex_count()} "
                f"faces={self.face_count()} textured={self.is_textured()}>")


# ==============================================================
# ScanHandle
# ==============================================================

class ScanHandle:
    """
    IScan* capsule 래퍼.
    스캔 결과(여러 FrameMeshHandle)에 접근.
    """

    def __init__(self, cap) -> None:
        self._cap = cap

    def frame_count(self) -> int:
        return _load().scan_frame_count(self._cap)

    def is_empty(self) -> bool:
        return _load().scan_is_empty(self._cap)

    def get_frame(self, i: int) -> FrameMeshHandle:
        return FrameMeshHandle(_load().scan_get_frame(self._cap, i))

    def last_frame(self) -> FrameMeshHandle:
        n = self.frame_count()
        if n == 0:
            raise RuntimeError("scan is empty — no last frame")
        return self.get_frame(n - 1)

    def frames(self) -> list:
        """list[FrameMeshHandle] for all frames."""
        return [self.get_frame(i) for i in range(self.frame_count())]

    def summary(self) -> ScanSummary:
        return ScanSummary(frame_count=self.frame_count())

    def __repr__(self) -> str:
        return f"<ScanHandle frames={self.frame_count()}>"


# ==============================================================
# ModelHandle
# ==============================================================

class ModelHandle:
    """
    IModel* capsule 래퍼.
    스캔 집합(ScanHandle 목록)과 최종 합성 메쉬에 접근.
    """

    def __init__(self, cap) -> None:
        self._cap = cap

    def scan_count(self) -> int:
        return _load().model_scan_count(self._cap)

    def get_scan(self, i: int) -> ScanHandle:
        return ScanHandle(_load().model_get_scan(self._cap, i))

    def scans(self) -> list:
        """list[ScanHandle] for all scans."""
        return [self.get_scan(i) for i in range(self.scan_count())]

    def has_final_mesh(self) -> bool:
        return _load().model_has_final_mesh(self._cap)

    def final_vertices(self) -> np.ndarray:
        """Return (N,3) float32 vertices of the final composite mesh."""
        return _load().model_final_vertices(self._cap)

    def final_faces(self) -> np.ndarray:
        """Return (M,3) int32 faces of the final composite mesh."""
        return _load().model_final_faces(self._cap)

    def summary(self) -> MeshSummary:
        verts = self.final_vertices()
        faces = self.final_faces()
        return MeshSummary(
            vertex_count=verts.shape[0],
            face_count=faces.shape[0],
            textured=_load().model_composite_textured(self._cap),
            scan_count=self.scan_count(),
        )

    def save_obj(self, path: str) -> None:
        """Save final composite mesh as OBJ file."""
        _load().model_save_obj(self._cap, path)

    def __repr__(self) -> str:
        return f"<ModelHandle scans={self.scan_count()}>"


# ==============================================================
# Module-level helpers
# ==============================================================

def create_model() -> ModelHandle:
    """빈 ModelHandle (IModel) 생성."""
    return ModelHandle(_load().create_model())


def capture_frame_handle(
    artec_client,
    capture_texture: Optional[bool] = None,
) -> Optional[FrameMeshHandle]:
    """
    초기화된 ArtecClient로 1회 캡처 → FrameMeshHandle.

    Parameters
    ----------
    artec_client : ArtecClient  (initialize() 완료 상태)
    capture_texture : bool | None  (None → cfg.capture_texture 사용)

    Returns
    -------
    FrameMeshHandle | None  (점 없으면 None)
    """
    m = _load()
    tex      = artec_client.cfg.capture_texture if capture_texture is None else capture_texture
    sc_cap   = artec_client.scanner_capsule()
    proc_cap = artec_client.processor_capsule()

    t0  = time.perf_counter()
    cap = m.capture_frame_mesh(sc_cap, proc_cap, tex)
    elapsed = time.perf_counter() - t0

    handle = FrameMeshHandle(cap)
    if handle.vertex_count() == 0:
        return None

    print(
        f"  [ArtecBase] capture_frame_handle  elapsed={elapsed:.3f}s"
        f"  verts={handle.vertex_count():,}  faces={handle.face_count():,}"
    )
    return handle


def capture_to_model(
    artec_client,
    capture_texture: Optional[bool] = None,
) -> Optional[ModelHandle]:
    """
    초기화된 ArtecClient로 1회 캡처 → ModelHandle.

    IModel > IScan > IFrameMesh 구조로 래핑된 결과를 반환한다.

    Parameters
    ----------
    artec_client : ArtecClient  (initialize() 완료 상태)
    capture_texture : bool | None  (None → cfg.capture_texture 사용)

    Returns
    -------
    ModelHandle | None  (스캔 없으면 None)
    """
    m = _load()
    tex      = artec_client.cfg.capture_texture if capture_texture is None else capture_texture
    sc_cap   = artec_client.scanner_capsule()
    proc_cap = artec_client.processor_capsule()

    t0  = time.perf_counter()
    cap = m.capture_to_model(sc_cap, proc_cap, tex)
    elapsed = time.perf_counter() - t0

    model = ModelHandle(cap)
    if model.scan_count() == 0:
        return None

    s = model.get_scan(0).summary()
    print(
        f"  [ArtecBase] capture_to_model  elapsed={elapsed:.3f}s"
        f"  scans={model.scan_count()}  frames={s.frame_count}"
    )
    return model
