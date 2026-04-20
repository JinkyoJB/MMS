# mms/sensor/artec/artec_client.py
#
# Artec 3D 스캐너 클라이언트.
# pybind11 확장 모듈 artec_sdk_py를 통해 Artec Capture SDK를 래핑.
#
# 빌드 방법
# ---------
#   pip install pybind11
#   $cmake = "...\CMake\bin\cmake.exe"
#   & $cmake -B build -G "Visual Studio 18 2026" -A x64
#   & $cmake --build build --config Release
#
# 실행 (바인딩 검증)
# ------------------
#   python mms/sensor/artec/artec_client.py

from __future__ import annotations

import os
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

if __name__ == "__main__":
    sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

import numpy as np

from mms.sensor.scan_result import ScanResult

_ARTEC_SDK_BIN = Path(r"C:\Program Files\Artec\Artec 3D Scanning SDK\bin-x64")


def _load_artec_sdk_py():
    if _ARTEC_SDK_BIN.exists():
        os.add_dll_directory(str(_ARTEC_SDK_BIN))
        os.environ["PATH"] = str(_ARTEC_SDK_BIN) + ";" + os.environ.get("PATH", "")

    _here = Path(__file__).parent
    if str(_here) not in sys.path:
        sys.path.insert(0, str(_here))

    try:
        import artec_sdk_py as _m
        return _m
    except ImportError as e:
        raise ImportError(
            "[ArtecClient] artec_sdk_py 모듈 로드 실패.\n"
            "  CMakeLists.txt로 빌드 후 artec_sdk_py.pyd를 mms/sensor/artec/에 배치.\n"
            f"  원인: {e}"
        ) from e


# ------------------------------------------------------------------
# Config
# ------------------------------------------------------------------

@dataclass
class ArtecConfig:
    """
    Artec 3D 스캐너 클라이언트 설정.

    serial_number : str | None
        스캐너 시리얼 번호. None → 첫 번째 발견된 스캐너.
    capture_texture : bool
        True → RGB 텍스처 포함 재구성. False → 지오메트리만 (빠름).
    fps : float | None
        스캐너 FPS. None → 기본값 유지.
    target_interval_s : float
        캡처 호출당 최소 대기 시간(초). 0.0 이면 대기 없음.
    """
    serial_number: Optional[str] = None
    capture_texture: bool = True
    fps: Optional[float] = None
    target_interval_s: float = 0.0


# ------------------------------------------------------------------
# ArtecClient
# ------------------------------------------------------------------

class ArtecClient:
    """
    Artec 3D 스캐너 클라이언트.

    capture() 는 센서(S) 프레임의 ScanResult를 반환한다.
    좌표 단위: points mm, 센서 프레임.
    """

    def __init__(self, cfg: ArtecConfig) -> None:
        self.cfg = cfg
        self._sdk = None
        self._scanner = None
        self._initialized: bool = False

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def initialize(self) -> None:
        self._sdk = _load_artec_sdk_py()

        serial = self.cfg.serial_number or ""
        self._scanner = self._sdk.ArtecScanner(serial)
        self._scanner.initialize()

        if self.cfg.fps is not None:
            fps = min(self.cfg.fps, self._scanner.get_max_fps())
            self._scanner.set_fps(fps)
            print(f"[ArtecClient] FPS={fps:.1f}")

        self._initialized = True
        print(
            f"[ArtecClient] Connected: {self._scanner.get_name()}"
            f"  serial={self._scanner.get_serial()}"
        )

    def shutdown(self) -> None:
        if self._scanner is not None:
            self._scanner.shutdown()
            self._scanner = None
        self._initialized = False
        print("[ArtecClient] Disconnected.")

    # ------------------------------------------------------------------
    # Capture
    # ------------------------------------------------------------------

    def capture(self, frame_id: int = 0, timestamp: Optional[float] = None) -> Optional[ScanResult]:
        """
        1회 스캔.

        Returns
        -------
        ScanResult
            points    : (N, 3) float32, mm, 센서(S) 프레임 (IFrameMesh 버텍스)
            normals   : (N, 3) float32 스무스 법선 | None
            triangles : (M, 3) int32 메쉬 face 인덱스
            img       : (H, W, 3) uint8 RGB 텍스처 | None
        """
        if not self._initialized:
            raise RuntimeError("[ArtecClient] Not initialized.")

        if timestamp is None:
            timestamp = time.perf_counter()

        _t0 = time.perf_counter()
        result = self._scanner.capture(self.cfg.capture_texture)
        _t1 = time.perf_counter()

        pts = np.asarray(result.points,    dtype=np.float32)
        if pts.shape[0] == 0:
            return None

        nrm = np.asarray(result.normals,   dtype=np.float32)
        tri = np.asarray(result.triangles, dtype=np.int32)
        tex = np.asarray(result.texture_image, dtype=np.uint8)

        scan = ScanResult(
            sensor_type="artec",
            points=pts,
            normals=nrm   if nrm.shape[0] > 0 else None,
            triangles=tri if tri.shape[0] > 0 else None,
            img=tex       if tex.shape[0]  > 0 else None,
            frame_id=frame_id,
            timestamp=timestamp,
        )

        elapsed = time.perf_counter() - _t0
        print(
            f"  [ArtecClient] capture={_t1-_t0:.3f}s  total={elapsed:.3f}s"
            f"  pts={len(scan.points):,}"
        )

        remaining = self.cfg.target_interval_s - elapsed
        if remaining > 0.0:
            time.sleep(remaining)

        return scan


# ------------------------------------------------------------------
# 바인딩 검증
# ------------------------------------------------------------------

def _check(label: str, condition: bool, detail: str = "") -> bool:
    status = "OK" if condition else "FAIL"
    msg = f"  [{status}] {label}"
    if detail:
        msg += f"  ({detail})"
    print(msg)
    return condition


def _section(title: str) -> None:
    print(f"\n{'─' * 55}")
    print(f"  {title}")
    print(f"{'─' * 55}")


if __name__ == "__main__":
    from mms.sensor.artec import artec_base
    from mms.sensor.artec import artec_capturing
    from mms.sensor.artec import artec_scanning
    from mms.sensor.artec import artec_algorithm
    from mms.sensor.artec import artec_project

    failures: list[str] = []

    def chk(label: str, condition: bool, detail: str = "") -> None:
        ok = _check(label, condition, detail)
        if not ok:
            failures.append(label)

    # ──────────────────────────────────────────────────────────────
    # 1. 모듈 로드
    # ──────────────────────────────────────────────────────────────
    _section("1. 모듈 로드")

    sdk = _load_artec_sdk_py()
    chk("artec_sdk_py import", sdk is not None)

    base_mod = artec_base._load_artec_base_py()
    chk("artec_base_py import", base_mod is not None)

    # ──────────────────────────────────────────────────────────────
    # 2. artec_sdk_py — 클래스 / 함수 존재 여부
    # ──────────────────────────────────────────────────────────────
    _section("2. artec_sdk_py 바인딩 확인")

    chk("enumerate_scanners 존재",  hasattr(sdk, "enumerate_scanners"))
    chk("ArtecScanner 존재",        hasattr(sdk, "ArtecScanner"))
    chk("CaptureResult 존재",       hasattr(sdk, "CaptureResult"))

    scanner_methods = [
        "initialize", "shutdown", "capture",
        "scanner_capsule", "processor_capsule",
        "get_fps", "set_fps", "get_max_fps",
        "get_texture_gain", "set_texture_gain",
        "get_serial", "get_name", "is_initialized",
    ]
    for m in scanner_methods:
        chk(f"  ArtecScanner.{m} 존재", hasattr(sdk.ArtecScanner, m))

    # ──────────────────────────────────────────────────────────────
    # 3. artec_base_py — 클래스 / 함수 존재 여부
    # ──────────────────────────────────────────────────────────────
    _section("3. artec_base_py 바인딩 확인")

    chk("create_model 존재",          hasattr(base_mod, "create_model"))
    chk("capture_frame_handle 존재",  hasattr(base_mod, "capture_frame_handle"))
    chk("capture_to_model 존재",      hasattr(base_mod, "capture_to_model"))
    chk("FrameMeshHandle 존재",       hasattr(base_mod, "FrameMeshHandle"))
    chk("ScanHandle 존재",            hasattr(base_mod, "ScanHandle"))
    chk("ModelHandle 존재",           hasattr(base_mod, "ModelHandle"))
    chk("FrameMeshSummary 존재",      hasattr(base_mod, "FrameMeshSummary"))
    chk("ScanSummary 존재",           hasattr(base_mod, "ScanSummary"))
    chk("MeshSummary 존재",           hasattr(base_mod, "MeshSummary"))

    fmh_methods = [
        "vertices", "faces", "is_textured", "uv",
        "image", "has_image", "vertex_count", "face_count", "summary",
    ]
    for m in fmh_methods:
        chk(f"  FrameMeshHandle.{m} 존재", hasattr(base_mod.FrameMeshHandle, m))

    scan_methods = ["frame_count", "frames", "get_frame", "last_frame", "is_empty", "summary"]
    for m in scan_methods:
        chk(f"  ScanHandle.{m} 존재", hasattr(base_mod.ScanHandle, m))

    model_methods = [
        "scan_count", "scans", "get_scan",
        "has_final_mesh", "final_vertices", "final_faces",
        "summary", "save_obj",
    ]
    for m in model_methods:
        chk(f"  ModelHandle.{m} 존재", hasattr(base_mod.ModelHandle, m))

    # ──────────────────────────────────────────────────────────────
    # 4. 스캐너 연결
    # ──────────────────────────────────────────────────────────────
    _section("4. 스캐너 연결")

    scanners = sdk.enumerate_scanners()
    chk("enumerate_scanners() 실행", True,
        f"{len(scanners)}개 발견" if scanners else "스캐너 없음")

    if not scanners:
        print("\n  스캐너가 연결되어 있지 않아 캡처 테스트를 건너뜁니다.")
        print(f"\n{'═' * 55}")
        print(f"  결과: {len(failures)} 실패" if failures else "  결과: 전체 통과")
        if failures:
            for f in failures:
                print(f"    - {f}")
        print(f"{'═' * 55}")
        sys.exit(1 if failures else 0)

    for s in scanners:
        print(f"    [{s['index']}] {s['name']}  serial={s['serial']}"
              f"  texture_cam={s['has_texture_camera']}")

    cfg = ArtecConfig(serial_number=None, capture_texture=True)
    client = ArtecClient(cfg)

    try:
        client.initialize()
        chk("ArtecClient.initialize()", client._initialized)

        chk("scanner_capsule() 반환",   client._scanner.scanner_capsule()   is not None)
        chk("processor_capsule() 반환", client._scanner.processor_capsule() is not None)

        # ──────────────────────────────────────────────────────────
        # 5. ArtecClient.capture() → ScanResult
        # ──────────────────────────────────────────────────────────
        _section("5. capture() → ScanResult")

        scan = None
        try:
            scan = client.capture(frame_id=0)
        except Exception as e:
            chk("capture() 예외 없음", False, str(e))

        has_scan_data = scan is not None and scan.points.shape[0] > 0

        if scan is None:
            print("    ※ capture() → None (스캔 데이터 없음 — 스캐너 앞에 물체 없음)")
        else:
            chk("capture() 반환값 존재", True)
            chk("points shape (N,3)",
                scan.points.ndim == 2 and scan.points.shape[1] == 3,
                f"shape={scan.points.shape}  dtype={scan.points.dtype}")
            chk("points dtype float32",
                scan.points.dtype == np.float32)
            chk("points N > 0",
                has_scan_data,
                f"N={scan.points.shape[0]:,}")
            chk("triangles shape (M,3) or None",
                scan.triangles is None or
                (scan.triangles.ndim == 2 and scan.triangles.shape[1] == 3),
                f"shape={scan.triangles.shape if scan.triangles is not None else 'None'}")
            chk("normals shape (N,3) or None",
                scan.normals is None or
                (scan.normals.ndim == 2 and scan.normals.shape[1] == 3))
            chk("img shape (H,W,3) or None",
                scan.img is None or
                (scan.img.ndim == 3 and scan.img.shape[2] == 3))
            chk("sensor_type == 'artec'",
                scan.sensor_type == "artec")

        # ──────────────────────────────────────────────────────────
        # 6. artec_base.capture_frame_handle() → FrameMeshHandle
        #    텍스처 캡처 실패 시 capture_texture=False 로 폴백
        # ──────────────────────────────────────────────────────────
        _section("6. capture_frame_handle() → FrameMeshHandle")

        fh = None
        for tex in (True, False):
            try:
                fh = artec_base.capture_frame_handle(client, capture_texture=tex)
                chk(f"capture_frame_handle(texture={tex}) 호출 성공", True)
                break
            except Exception as e:
                chk(f"capture_frame_handle(texture={tex}) 호출 성공", False, str(e))

        if fh is None:
            print("    ※ FrameMeshHandle 없음 — 스캐너 앞에 물체 없음")
        else:
            verts = fh.vertices()
            faces = fh.faces()
            chk("vertices() shape (N,3) float32",
                verts.ndim == 2 and verts.shape[1] == 3 and verts.dtype == np.float32,
                f"shape={verts.shape}  dtype={verts.dtype}")
            chk("vertices N > 0",
                verts.shape[0] > 0,
                f"N={verts.shape[0]:,}")
            chk("faces() shape (M,3) int32",
                faces.ndim == 2 and faces.shape[1] == 3 and faces.dtype == np.int32,
                f"shape={faces.shape}  dtype={faces.dtype}")
            chk("is_textured() bool",
                isinstance(fh.is_textured(), bool),
                str(fh.is_textured()))
            chk("has_image() bool",
                isinstance(fh.has_image(), bool),
                str(fh.has_image()))

            uv = fh.uv()
            chk("uv() shape (N,2) float32 or None",
                uv is None or (uv.ndim == 2 and uv.shape[1] == 2 and uv.dtype == np.float32),
                f"shape={uv.shape if uv is not None else 'None'}")

            img = fh.image()
            chk("image() shape (H,W,3) uint8 or None",
                img is None or (img.ndim == 3 and img.shape[2] == 3 and img.dtype == np.uint8),
                f"shape={img.shape if img is not None else 'None'}")

            s = fh.summary()
            chk("summary() → FrameMeshSummary",
                type(s).__name__ == "FrameMeshSummary")
            chk("  summary.vertex_count > 0",
                s.vertex_count > 0,
                f"vertex_count={s.vertex_count:,}")
            chk("  summary.face_count > 0",
                s.face_count > 0,
                f"face_count={s.face_count:,}")
            print(f"    {s}")

        # ──────────────────────────────────────────────────────────
        # 7. artec_base.capture_to_model() → ModelHandle
        #    텍스처 캡처 실패 시 capture_texture=False 로 폴백
        # ──────────────────────────────────────────────────────────
        _section("7. capture_to_model() → ModelHandle")

        model = None
        for tex in (True, False):
            try:
                model = artec_base.capture_to_model(client, capture_texture=tex)
                chk(f"capture_to_model(texture={tex}) 호출 성공", True)
                break
            except Exception as e:
                chk(f"capture_to_model(texture={tex}) 호출 성공", False, str(e))

        if model is None:
            print("    ※ ModelHandle 없음 — 스캐너 앞에 물체 없음")
        else:
            chk("scan_count() == 1",
                model.scan_count() == 1,
                f"scan_count={model.scan_count()}")

            scan_handle = model.get_scan(0)
            chk("get_scan(0) → ScanHandle",
                type(scan_handle).__name__ == "ScanHandle")
            chk("  frame_count() == 1",
                scan_handle.frame_count() == 1,
                f"frame_count={scan_handle.frame_count()}")
            chk("  is_empty() == False",
                not scan_handle.is_empty())

            ss = scan_handle.summary()
            chk("  summary() → ScanSummary",
                type(ss).__name__ == "ScanSummary",
                f"frame_count={ss.frame_count}")

            frame_handle = scan_handle.get_frame(0)
            chk("  get_frame(0) → FrameMeshHandle",
                type(frame_handle).__name__ == "FrameMeshHandle")

            last = scan_handle.last_frame()
            chk("  last_frame() → FrameMeshHandle",
                type(last).__name__ == "FrameMeshHandle")

            frame_verts = frame_handle.vertices()
            chk("  frame.vertices() shape (N,3) float32",
                frame_verts.ndim == 2 and frame_verts.shape[1] == 3
                and frame_verts.dtype == np.float32,
                f"N={frame_verts.shape[0]:,}")

            chk("has_final_mesh() == False (알고리즘 전)",
                not model.has_final_mesh())

            ms = model.summary()
            chk("summary() → MeshSummary",
                type(ms).__name__ == "MeshSummary")
            chk("  summary.scan_count == 1",
                ms.scan_count == 1,
                f"scan_count={ms.scan_count}")
            print(f"    {ms}")

            scans_list = model.scans()
            chk("scans() → list length 1",
                len(scans_list) == 1,
                f"len={len(scans_list)}")

        # ──────────────────────────────────────────────────────────
        # 8. create_model() — 빈 모델 (스캐너 상태 무관)
        # ──────────────────────────────────────────────────────────
        _section("8. create_model() — 빈 ModelHandle")

        empty_model = artec_base.create_model()
        chk("create_model() 반환값 존재",  empty_model is not None)
        chk("scan_count() == 0",           empty_model.scan_count() == 0)
        chk("has_final_mesh() == False",   not empty_model.has_final_mesh())
        final_v = empty_model.final_vertices()
        chk("final_vertices() shape (0,3)",
            final_v.shape == (0, 3),
            f"shape={final_v.shape}")

        # ──────────────────────────────────────────────────────────
        # 9. artec_capturing — 세부 Capturing API 바인딩 + 시각화
        # ──────────────────────────────────────────────────────────
        _section("9. artec_capturing_py 바인딩 검증 (+ Open3D 시각화)")

        capturing_ok = artec_capturing.verify_artec_capturing(
            client,
            visualize=True,   # Open3D 뷰어 표시 (없으면 자동 skip)
            with_texture=False,
        )
        chk("artec_capturing 전체 통과", capturing_ok)

        # ──────────────────────────────────────────────────────────
        # 10. artec_scanning — Scanning API 바인딩 + Open3D 시각화
        # ──────────────────────────────────────────────────────────
        _section("10. artec_scanning_py 바인딩 검증 (+ Open3D 시각화)")

        scanning_ok = artec_scanning.verify_artec_scanning(
            client,
            visualize=True,     # Open3D 뷰어 표시 (없으면 자동 skip)
            record_seconds=5.0, # Record 모드 지속 시간(초)
        )
        chk("artec_scanning 전체 통과", scanning_ok)

        # ──────────────────────────────────────────────────────────
        # 11. artec_algorithm — Algorithm API 바인딩 + Open3D 시각화
        # ──────────────────────────────────────────────────────────
        _section("11. artec_algorithm_py 바인딩 검증 (+ Open3D 시각화)")

        algorithm_ok = artec_algorithm.verify_artec_algorithm(
            client,
            visualize=True,  # Open3D 뷰어 표시 (없으면 자동 skip)
        )
        chk("artec_algorithm 전체 통과", algorithm_ok)

        # ──────────────────────────────────────────────────────────
        # 12. artec_project — Project API 바인딩 + Open3D 시각화
        # ──────────────────────────────────────────────────────────
        _section("12. artec_project_py 바인딩 검증 (+ Open3D 시각화)")

        project_ok = artec_project.verify_artec_project(
            client,
            visualize=True,  # Open3D 뷰어 표시 (없으면 자동 skip)
        )
        chk("artec_project 전체 통과", project_ok)

    except Exception as e:
        print(f"\n[ERROR] {e}", file=sys.stderr)
        raise
    finally:
        client.shutdown()

    # ──────────────────────────────────────────────────────────────
    # 최종 결과
    # ──────────────────────────────────────────────────────────────
    print(f"\n{'═' * 55}")
    if failures:
        print(f"  결과: {len(failures)}개 실패")
        for f in failures:
            print(f"    - {f}")
        sys.exit(1)
    else:
        print("  결과: 전체 통과")
    print(f"{'═' * 55}")
