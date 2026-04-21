# mms/sensor/artec/artec_project.py
#
# Artec Project SDK Python 래퍼.
# artec_project_py (C++ pybind11)는 capsule 기반 free function만 노출.
# 이 파일에서 클래스 계층(ProjectManager, ProjectHandle, CompositeMeshHandle,
# LoadedProjectEntry 등)과 DTO(@dataclass)를 Python으로 구현한다.
#
# Exposes
# -------
#   Enums
#     EntryType
#
#   DTOs (dataclass)
#     ProjectEntryInfo
#     LoadedProjectEntry
#     CompositeMeshSummary
#
#   Handles (Python class, _cap 으로 capsule 보유)
#     CompositeMeshHandle
#     ProjectHandle
#
#   Manager
#     ProjectManager  — static: create(), open(), max_supported_version()
#
#   Visualization / verify
#     visualize_project_entries(entries)
#     verify_artec_project(artec_client, visualize=True)

from __future__ import annotations

import os
import sys
import tempfile
import time
from dataclasses import dataclass
from enum import IntEnum
from pathlib import Path
from typing import Optional

import numpy as np

_ARTEC_SDK_BIN = Path(r"C:\Program Files\Artec\Artec 3D Scanning SDK\bin-x64")
_HERE = Path(__file__).parent

_proj_py = None  # lazy-loaded artec_project_py module


def _load():
    """artec_project_py .pyd 를 lazy-load하고 반환."""
    global _proj_py
    if _proj_py is not None:
        return _proj_py

    if _ARTEC_SDK_BIN.exists():
        os.add_dll_directory(str(_ARTEC_SDK_BIN))
        os.environ["PATH"] = str(_ARTEC_SDK_BIN) + ";" + os.environ.get("PATH", "")

    if str(_HERE) not in sys.path:
        sys.path.insert(0, str(_HERE))

    try:
        import artec_project_py as _m
        _proj_py = _m
        return _proj_py
    except ImportError as e:
        raise ImportError(
            "[artec_project] artec_project_py 모듈 로드 실패.\n"
            "  cmake --build build --config Release 후 artec_project_py.pyd 생성 확인.\n"
            f"  원인: {e}"
        ) from e


# ==============================================================
# Enums
# ==============================================================

class EntryType(IntEnum):
    """
    프로젝트 엔트리 종류.

    SDK EntryType 과 값이 1:1 대응.
      EntryType_Scan          = 0
      EntryType_CompositeMesh = 1
      EntryType_Unknown       = 0xFFFFFFFF  → C++ signed int -1
    """
    SCAN           = 0
    COMPOSITE_MESH = 1
    UNKNOWN        = -1  # SDK EntryType_Unknown = 0xFFFFFFFF (signed int)


# ==============================================================
# DTOs
# ==============================================================

@dataclass
class ProjectEntryInfo:
    """
    프로젝트 엔트리 메타데이터.

    entry_id   : UUID 문자열 (예: "550e8400-e29b-41d4-a716-446655440000")
    entry_type : EntryType enum
    name       : 엔트리 이름 (빈 문자열 가능)
    size_bytes : 저장 크기 (bytes)
    """
    entry_id:   str       = ""
    entry_type: EntryType = EntryType.UNKNOWN
    name:       str       = ""
    size_bytes: int       = 0

    def __repr__(self) -> str:
        return (f"ProjectEntryInfo(id={self.entry_id[:8]}..,"
                f" type={self.entry_type.name}, name={self.name!r},"
                f" size={self.size_bytes:,})")


@dataclass
class CompositeMeshSummary:
    vertex_count: int  = 0
    face_count:   int  = 0
    textured:     bool = False

    def __repr__(self) -> str:
        return (f"CompositeMeshSummary(verts={self.vertex_count:,},"
                f" faces={self.face_count:,}, textured={self.textured})")


@dataclass
class LoadedProjectEntry:
    """
    load_entries() 로 로드된 단일 엔트리.

    entry_info : ProjectEntryInfo  — 메타데이터
    scan       : artec_base.ScanHandle | None  — SCAN 엔트리
    mesh       : CompositeMeshHandle | None    — COMPOSITE_MESH 엔트리
    """
    entry_info: ProjectEntryInfo
    scan:       Optional[object]  = None  # artec_base.ScanHandle
    mesh:       Optional[object]  = None  # CompositeMeshHandle

    def __repr__(self) -> str:
        kind = "scan" if self.scan is not None else (
               "mesh" if self.mesh is not None else "empty")
        return f"LoadedProjectEntry({self.entry_info.entry_type.name}, {kind}, {self.entry_info.name!r})"


# ==============================================================
# CompositeMeshHandle
# ==============================================================

class CompositeMeshHandle:
    """
    ICompositeMesh* capsule 래퍼.
    프로젝트에서 로드된 합성 메시(복합 텍스처 메시)에 접근.
    """

    def __init__(self, cap) -> None:
        self._cap = cap

    def uuid(self) -> str:
        """Return UUID string of this composite mesh."""
        return _load().composite_mesh_uuid(self._cap)

    def name(self) -> str:
        """Return name of this composite mesh."""
        return _load().composite_mesh_name(self._cap)

    def vertex_count(self) -> int:
        return _load().composite_mesh_vertex_count(self._cap)

    def face_count(self) -> int:
        return _load().composite_mesh_face_count(self._cap)

    def vertices(self) -> np.ndarray:
        """Return (N,3) float32 vertex positions in mm."""
        return _load().composite_mesh_vertices(self._cap)

    def faces(self) -> np.ndarray:
        """Return (M,3) int32 triangle face indices."""
        return _load().composite_mesh_faces(self._cap)

    def is_textured(self) -> bool:
        return _load().composite_mesh_is_textured(self._cap)

    def summary(self) -> CompositeMeshSummary:
        return CompositeMeshSummary(
            vertex_count=self.vertex_count(),
            face_count=self.face_count(),
            textured=self.is_textured(),
        )

    def __repr__(self) -> str:
        return (f"<CompositeMeshHandle verts={self.vertex_count():,}"
                f" faces={self.face_count():,}"
                f" textured={self.is_textured()}>")


# ==============================================================
# ProjectHandle
# ==============================================================

class ProjectHandle:
    """
    IProject* capsule 래퍼.

    전형적인 사용 패턴
    ------------------
    >>> proj = ProjectManager.open(path)
    >>> entries = proj.load_entries()          # list[LoadedProjectEntry]
    >>> for e in entries:
    ...     if e.scan:
    ...         frames = e.scan.frames()
    ...     elif e.mesh:
    ...         verts = e.mesh.vertices()
    >>> proj.save(path="/new/path.sproj", model=loaded_model, compression_level=0)
    """

    def __init__(self, proj_cap, path: Optional[str] = None) -> None:
        self._cap  = proj_cap
        self._path = path          # 원래 프로젝트 경로 (save 기본값)
        self._loaded_model_cap = None  # project_load_all 결과 저장

    # ── 속성 ──────────────────────────────────────────────────

    def version(self) -> int:
        """Return current project version."""
        return _load().project_version(self._cap)

    def entry_count(self) -> int:
        """Return number of entries in the project."""
        return _load().project_entry_count(self._cap)

    def get_entry(self, index: int) -> ProjectEntryInfo:
        """Return ProjectEntryInfo for the entry at given index."""
        d = _load().project_get_entry(self._cap, index)
        return ProjectEntryInfo(
            entry_id=d["uuid"],
            entry_type=_safe_entry_type(d["entry_type"]),
            name=d["name"],
            size_bytes=d["size_bytes"],
        )

    def entries(self) -> list:
        """Return list[ProjectEntryInfo] for all entries."""
        return [self.get_entry(i) for i in range(self.entry_count())]

    # ── 로드 ──────────────────────────────────────────────────

    def load_entries(self, settings=None) -> list:
        """
        프로젝트의 모든 엔트리를 로드해 list[LoadedProjectEntry]로 반환.

        내부적으로 project_load_all() → IModel을 사용해 스캔/합성메시를
        로드하고, 엔트리 UUID로 매핑한다.

        Parameters
        ----------
        settings : ProjectLoaderSettingsDTO | None  (현재 미사용 — 미래 확장용)

        Returns
        -------
        list[LoadedProjectEntry]
        """
        from mms.sensor.artec import artec_base

        m = _load()

        # 1. 엔트리 메타데이터 수집
        entry_infos = self.entries()

        # 2. 모든 엔트리 로드 → IModel
        model_cap = m.project_load_all(self._cap)
        self._loaded_model_cap = model_cap

        # 3. 로드된 스캔과 합성메시 수집 + UUID 인덱스 빌드
        scan_count      = artec_base._load().model_scan_count(model_cap)
        composite_count = m.model_composite_count(model_cap)

        # UUID → IScan* capsule
        scan_by_uuid: dict[str, object] = {}
        for i in range(scan_count):
            sc_cap = artec_base._load().model_get_scan(model_cap, i)
            uid    = m.scan_uuid(sc_cap)
            scan_by_uuid[uid] = sc_cap

        # UUID → ICompositeMesh* capsule
        mesh_by_uuid: dict[str, object] = {}
        for i in range(composite_count):
            cm_cap = m.model_get_composite(model_cap, i)
            uid    = m.composite_mesh_uuid(cm_cap)
            mesh_by_uuid[uid] = cm_cap

        # 4. 엔트리 메타데이터와 로드된 객체 매핑
        result: list[LoadedProjectEntry] = []
        for info in entry_infos:
            scan_handle = None
            mesh_handle = None

            if info.entry_type == EntryType.SCAN:
                sc_cap = scan_by_uuid.get(info.entry_id)
                if sc_cap is not None:
                    scan_handle = artec_base.ScanHandle(sc_cap)

            elif info.entry_type == EntryType.COMPOSITE_MESH:
                cm_cap = mesh_by_uuid.get(info.entry_id)
                if cm_cap is not None:
                    mesh_handle = CompositeMeshHandle(cm_cap)

            result.append(LoadedProjectEntry(
                entry_info=info,
                scan=scan_handle,
                mesh=mesh_handle,
            ))

        return result

    # ── 저장 ──────────────────────────────────────────────────

    def save(
        self,
        path: Optional[str] = None,
        model=None,
        compression_level: int = 0,
    ) -> None:
        """
        프로젝트를 파일로 저장.

        Parameters
        ----------
        path : str | None
            저장 경로 (.sproj 파일). None → self._path 사용.
        model : artec_base.ModelHandle | None
            저장할 모델. None → load_entries() 로 로드된 모델 사용.
        compression_level : int
            압축 수준 (0 = 기본값).
        """
        save_path = path or self._path
        if not save_path:
            raise ValueError("[ProjectHandle.save] path must be provided "
                             "if the project has no associated path.")

        # model capsule 결정
        if model is not None:
            from mms.sensor.artec import artec_base
            if isinstance(model, artec_base.ModelHandle):
                model_cap = model._cap
            else:
                raise TypeError("[ProjectHandle.save] model must be artec_base.ModelHandle.")
        elif self._loaded_model_cap is not None:
            model_cap = self._loaded_model_cap
        else:
            raise ValueError("[ProjectHandle.save] No model available. "
                             "Either call load_entries() first or pass model= explicitly.")

        _load().project_save(self._cap, model_cap, save_path, compression_level)

    # ── magic ──────────────────────────────────────────────────

    def __repr__(self) -> str:
        return (f"<ProjectHandle version={self.version()}"
                f" entries={self.entry_count()}"
                f" path={self._path!r}>")


# ==============================================================
# ProjectManager
# ==============================================================

class ProjectManager:
    """
    Artec Project 생성/열기 진입점.
    모든 메서드는 static.
    """

    @staticmethod
    def create(path: str) -> ProjectHandle:
        """
        새 Artec Studio 프로젝트를 생성.

        Parameters
        ----------
        path : str
            프로젝트 파일 경로 (예: "C:/data/my_scan.sproj")

        Returns
        -------
        ProjectHandle
        """
        cap = _load().project_create(path)
        return ProjectHandle(cap, path=path)

    @staticmethod
    def open(path: str) -> ProjectHandle:
        """
        기존 Artec Studio 프로젝트를 열기.

        Parameters
        ----------
        path : str
            기존 프로젝트 파일 경로

        Returns
        -------
        ProjectHandle
        """
        cap = _load().project_open(path)
        return ProjectHandle(cap, path=path)

    @staticmethod
    def max_supported_version() -> int:
        """Return maximum supported project version by this SDK."""
        return _load().project_max_version()


# ==============================================================
# 내부 유틸
# ==============================================================

def _safe_entry_type(int_val: int) -> EntryType:
    """C++ int → EntryType enum (unknown은 UNKNOWN으로 매핑)."""
    try:
        return EntryType(int_val)
    except ValueError:
        return EntryType.UNKNOWN


# ==============================================================
# 시각화
# ==============================================================

def visualize_project_entries(
    entries: list,
    title: str = "Artec Project Entries",
) -> bool:
    """
    LoadedProjectEntry 목록을 Open3D로 시각화.

    - SCAN 엔트리: 각 frame의 vertex를 TriangleMesh로 표시
    - COMPOSITE_MESH 엔트리: final_vertices/final_faces로 표시

    Returns True if visualization was shown.
    """
    try:
        import open3d as o3d
    except ImportError:
        print("  [visualize_project_entries] open3d 없음 — 시각화 건너뜀")
        return False

    geometries = []
    total_verts = 0

    for entry in entries:
        if entry.scan is not None:
            for frame in entry.scan.frames():
                verts = frame.vertices()
                faces = frame.faces()
                if verts.shape[0] == 0:
                    continue
                mesh = o3d.geometry.TriangleMesh()
                mesh.vertices = o3d.utility.Vector3dVector(
                    verts.astype(np.float64) * 1e-3
                )
                if faces.shape[0] > 0:
                    mesh.triangles = o3d.utility.Vector3iVector(faces)
                    mesh.compute_vertex_normals()
                geometries.append(mesh)
                total_verts += verts.shape[0]

        elif entry.mesh is not None:
            verts = entry.mesh.vertices()
            faces = entry.mesh.faces()
            if verts.shape[0] == 0:
                continue
            mesh = o3d.geometry.TriangleMesh()
            mesh.vertices = o3d.utility.Vector3dVector(
                verts.astype(np.float64) * 1e-3
            )
            if faces.shape[0] > 0:
                mesh.triangles = o3d.utility.Vector3iVector(faces)
                mesh.compute_vertex_normals()
            geometries.append(mesh)
            total_verts += verts.shape[0]

    if not geometries:
        print("  [visualize_project_entries] 시각화할 geometry 없음")
        return False

    print(f"  [visualize_project_entries] {title}"
          f"  entries={len(entries)}  total_verts={total_verts:,}")
    o3d.visualization.draw_geometries(geometries, window_name=title)
    return True


# ==============================================================
# 검증
# ==============================================================

def verify_artec_project(
    artec_client,
    visualize: bool = True,
    scan_model=None,
) -> bool:
    """
    artec_project_py 바인딩 및 ProjectManager / ProjectHandle 검증.

    단계
    ----
    1. 모듈 로드 확인
    2. 함수 존재 여부
    3. project_max_version()
    4. 스캔 모델 준비 (scan_model 인자 우선, 없으면 capture_to_model)
    5. ProjectManager.create(tmp_path) — 새 프로젝트
    6. entry_count() == 0 (빈 프로젝트)
    7. project_save() — 스캔 데이터 저장
    8. ProjectManager.open(saved_path) — 저장된 프로젝트 열기
    9. load_entries() → list[LoadedProjectEntry]
    10. 로드된 엔트리에서 scan 또는 mesh 확인
    11. CompositeMeshHandle 또는 ScanHandle 데이터 shape 검증
    12. visualize_project_entries() (선택)
    13. 임시 파일 정리

    Parameters
    ----------
    artec_client : ArtecClient
    visualize : bool
    scan_model : artec_base.ModelHandle | None
        SDK scanning session으로 생성된 ModelHandle 우선 사용.
        None이면 capture_to_model()로 캡처 (UUID 없어 save 실패 가능).

    Returns
    -------
    bool  True if all steps passed.
    """
    from mms.sensor.artec import artec_base

    passed: list[str] = []
    failed: list[str] = []

    def chk(label: str, cond: bool, detail: str = "") -> bool:
        status = "OK" if cond else "FAIL"
        msg = f"    [{status}] {label}"
        if detail:
            msg += f"  ({detail})"
        print(msg)
        (passed if cond else failed).append(label)
        return cond

    print("\n  [verify_artec_project] 시작")

    # ── 1. 모듈 로드 ──────────────────────────────────────────
    try:
        m = _load()
        chk("artec_project_py 로드", True)
    except Exception as e:
        chk("artec_project_py 로드", False, str(e))
        return False

    # ── 2. 함수 존재 여부 ─────────────────────────────────────
    fn_names = [
        "project_create", "project_open", "project_max_version",
        "project_version", "project_entry_count", "project_get_entry",
        "project_load_all", "project_save",
        "scan_uuid", "scan_name",
        "model_composite_count", "model_get_composite",
        "composite_mesh_uuid", "composite_mesh_name",
        "composite_mesh_vertex_count", "composite_mesh_face_count",
        "composite_mesh_vertices", "composite_mesh_faces",
        "composite_mesh_is_textured",
    ]
    for fn in fn_names:
        chk(f"  {fn} 존재", hasattr(m, fn))

    # ── 3. project_max_version ────────────────────────────────
    try:
        ver = ProjectManager.max_supported_version()
        chk("project_max_version() 호출", isinstance(ver, int), f"version={ver}")
    except Exception as e:
        chk("project_max_version() 호출", False, str(e))

    # ── 4. 스캔 모델 준비 ────────────────────────────────────
    # scan_model이 전달되면 재사용 (SDK scanning session 결과 → UUID 보유).
    # 없으면 capture_to_model()로 캡처 (UUID 미설정으로 save 실패 가능).
    if scan_model is not None:
        total_frames = sum(
            scan_model.get_scan(i).frame_count()
            for i in range(scan_model.scan_count())
        )
        chk("scan_model 수신", True,
            f"scans={scan_model.scan_count()}  total_frames={total_frames}")
    else:
        scan_model = None
        for tex in (False, True):
            try:
                scan_model = artec_base.capture_to_model(artec_client, capture_texture=tex)
                if scan_model is not None:
                    chk(f"capture_to_model(texture={tex})", True,
                        f"scans={scan_model.scan_count()}")
                    break
            except Exception as e:
                chk(f"capture_to_model(texture={tex})", False, str(e))

    if scan_model is None:
        print("    ※ 스캔 데이터 없음 — 저장/로드 테스트 건너뜀")
        _print_summary(passed, failed)
        return len(failed) == 0

    # ── 5. 임시 프로젝트 생성 ─────────────────────────────────
    tmp_dir  = tempfile.mkdtemp(prefix="artec_project_test_")
    tmp_path = str(Path(tmp_dir) / "test_scan.sproj")
    new_proj: Optional[ProjectHandle] = None

    try:
        new_proj = ProjectManager.create(tmp_path)
        chk("ProjectManager.create()", new_proj is not None,
            f"path={tmp_path}")
    except Exception as e:
        chk("ProjectManager.create()", False, str(e))
        _print_summary(passed, failed)
        return len(failed) == 0

    # ── 6. 빈 프로젝트 entry_count ────────────────────────────
    try:
        cnt = new_proj.entry_count()
        chk("entry_count() == 0 (새 프로젝트)", cnt == 0, f"count={cnt}")
    except Exception as e:
        chk("entry_count() == 0", False, str(e))

    # ── 7. 저장 ───────────────────────────────────────────────
    try:
        t0 = time.perf_counter()
        new_proj.save(path=tmp_path, model=scan_model)
        elapsed = time.perf_counter() - t0
        chk("project_save()", True, f"elapsed={elapsed:.2f}s  path={tmp_path}")
    except Exception as e:
        chk("project_save()", False, str(e))
        _print_summary(passed, failed)
        return len(failed) == 0

    # ── 8. 저장된 프로젝트 열기 ───────────────────────────────
    opened_proj: Optional[ProjectHandle] = None
    try:
        opened_proj = ProjectManager.open(tmp_path)
        chk("ProjectManager.open(saved)", opened_proj is not None)
        chk("  version() >= 1", opened_proj.version() >= 1,
            f"version={opened_proj.version()}")
    except Exception as e:
        chk("ProjectManager.open(saved)", False, str(e))
        _print_summary(passed, failed)
        return len(failed) == 0

    # ── 9. load_entries() ─────────────────────────────────────
    entries: list[LoadedProjectEntry] = []
    try:
        t0 = time.perf_counter()
        entries = opened_proj.load_entries()
        elapsed = time.perf_counter() - t0
        chk("load_entries() 반환", isinstance(entries, list),
            f"count={len(entries)}  elapsed={elapsed:.2f}s")
        chk("  entry 수 > 0", len(entries) > 0, f"count={len(entries)}")
    except Exception as e:
        chk("load_entries()", False, str(e))
        _print_summary(passed, failed)
        return len(failed) == 0

    # ── 10. 엔트리 타입 확인 ──────────────────────────────────
    scan_entries = [e for e in entries if e.scan is not None]
    mesh_entries = [e for e in entries if e.mesh is not None]
    print(f"    엔트리: scan={len(scan_entries)}  composite_mesh={len(mesh_entries)}")
    chk("scan 또는 mesh 엔트리 존재",
        len(scan_entries) + len(mesh_entries) > 0)

    for i, e in enumerate(entries):
        chk(f"  entries[{i}].entry_info 존재",
            isinstance(e.entry_info, ProjectEntryInfo))
        chk(f"  entries[{i}].entry_type 확인",
            e.entry_info.entry_type != EntryType.UNKNOWN,
            f"type={e.entry_info.entry_type.name}")

    # ── 11. ScanHandle / CompositeMeshHandle shape 검증 ───────
    for i, e in enumerate(entries[:3]):   # 최대 3개 검증
        if e.scan is not None:
            try:
                fc = e.scan.frame_count()
                chk(f"  scan[{i}].frame_count() >= 0", fc >= 0,
                    f"frames={fc}")
                if fc > 0:
                    frame = e.scan.get_frame(0)
                    verts = frame.vertices()
                    faces = frame.faces()
                    chk(f"  scan[{i}].frame.vertices() (N,3) float32",
                        verts.ndim == 2 and verts.shape[1] == 3
                        and verts.dtype == np.float32,
                        f"shape={verts.shape}")
                    chk(f"  scan[{i}].frame.faces() (M,3) int32",
                        faces.ndim == 2 and faces.shape[1] == 3
                        and faces.dtype == np.int32,
                        f"shape={faces.shape}")
            except Exception as e_:
                chk(f"  scan[{i}] 검증", False, str(e_))

        if e.mesh is not None:
            try:
                verts = e.mesh.vertices()
                faces = e.mesh.faces()
                chk(f"  mesh[{i}].vertices() (N,3) float32",
                    verts.ndim == 2 and verts.shape[1] == 3
                    and verts.dtype == np.float32,
                    f"shape={verts.shape}")
                chk(f"  mesh[{i}].faces() (M,3) int32",
                    faces.ndim == 2 and faces.shape[1] == 3
                    and faces.dtype == np.int32,
                    f"shape={faces.shape}")
                s = e.mesh.summary()
                chk(f"  mesh[{i}].summary() → CompositeMeshSummary",
                    isinstance(s, CompositeMeshSummary))
                print(f"    {s}")
            except Exception as e_:
                chk(f"  mesh[{i}] 검증", False, str(e_))

    # ── 12. Open3D 시각화 ─────────────────────────────────────
    if visualize and entries:
        try:
            shown = visualize_project_entries(
                entries, title="artec_project verify — loaded entries"
            )
            chk("visualize_project_entries()", shown)
        except Exception as e:
            chk("visualize_project_entries()", False, str(e))

    # ── 13. 임시 파일 정리 ────────────────────────────────────
    try:
        import shutil
        shutil.rmtree(tmp_dir, ignore_errors=True)
        print(f"    [cleanup] 임시 디렉터리 삭제: {tmp_dir}")
    except Exception:
        pass

    _print_summary(passed, failed)
    return len(failed) == 0


def _print_summary(passed: list, failed: list) -> None:
    total = len(passed) + len(failed)
    print(f"\n  [verify_artec_project] 완료: {len(passed)}/{total} 통과"
          + (f", 실패: {failed}" if failed else ""))
