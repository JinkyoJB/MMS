# mms_artec/sensor/binding_test.py
#
# Artec SDK 바인딩 전체 검증 스크립트.
#
# 실행
# ----
#   python mms_artec/sensor/binding_test.py

from __future__ import annotations

import sys
from pathlib import Path

if __name__ == "__main__":
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import os
import numpy as np

from mms_artec.sensor.artec_client import ArtecClient, ArtecConfig
from mms_artec.sensor import artec_base
from mms_artec.sensor import artec_capturing
from mms_artec.sensor import artec_scanning
from mms_artec.sensor import artec_algorithm
from mms_artec.sensor import artec_project

_ARTEC_SDK_BIN = Path(r"C:\Program Files\Artec\Artec 3D Scanning SDK\bin-x64")


def _load_artec_sdk_py():
    """artec_sdk_py 바인딩 검증용 — 고수준 ArtecScanner 클래스 포함 모듈."""
    if _ARTEC_SDK_BIN.exists():
        os.add_dll_directory(str(_ARTEC_SDK_BIN))
        os.environ["PATH"] = str(_ARTEC_SDK_BIN) + ";" + os.environ.get("PATH", "")
    _here = Path(__file__).parent
    if str(_here) not in sys.path:
        sys.path.insert(0, str(_here))
    import artec_sdk_py as _m
    return _m


# ------------------------------------------------------------------
# 헬퍼
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


# ------------------------------------------------------------------
# 메인
# ------------------------------------------------------------------

if __name__ == "__main__":
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

    # base_mod : artec_base_py (C++ 바인딩) — free function만 노출
    chk("create_model 존재",          hasattr(base_mod, "create_model"))
    chk("capture_frame_mesh 존재",    hasattr(base_mod, "capture_frame_mesh"))
    chk("capture_to_model 존재",      hasattr(base_mod, "capture_to_model"))

    # artec_base : Python 래퍼 — 클래스/DTO/헬퍼
    chk("FrameMeshHandle 존재",       hasattr(artec_base, "FrameMeshHandle"))
    chk("ScanHandle 존재",            hasattr(artec_base, "ScanHandle"))
    chk("ModelHandle 존재",           hasattr(artec_base, "ModelHandle"))
    chk("FrameMeshSummary 존재",      hasattr(artec_base, "FrameMeshSummary"))
    chk("ScanSummary 존재",           hasattr(artec_base, "ScanSummary"))
    chk("MeshSummary 존재",           hasattr(artec_base, "MeshSummary"))
    chk("capture_frame_handle 존재",  hasattr(artec_base, "capture_frame_handle"))

    fmh_methods = [
        "vertices", "faces", "is_textured", "uv",
        "image", "has_image", "vertex_count", "face_count", "summary",
    ]
    for m in fmh_methods:
        chk(f"  FrameMeshHandle.{m} 존재", hasattr(artec_base.FrameMeshHandle, m))

    scan_methods = ["frame_count", "frames", "get_frame", "last_frame", "is_empty", "summary"]
    for m in scan_methods:
        chk(f"  ScanHandle.{m} 존재", hasattr(artec_base.ScanHandle, m))

    model_methods = [
        "scan_count", "scans", "get_scan",
        "has_final_mesh", "final_vertices", "final_faces",
        "summary", "save_obj",
    ]
    for m in model_methods:
        chk(f"  ModelHandle.{m} 존재", hasattr(artec_base.ModelHandle, m))

    # ──────────────────────────────────────────────────────────────
    # 4. 스캐너 연결
    # ──────────────────────────────────────────────────────────────
    _section("4. 스캐너 연결")

    scanners = artec_capturing.list_scanners()
    chk("list_scanners() 실행", True,
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

    for i, s in enumerate(scanners):
        print(f"    [{i}] {s.name}  serial={s.serial}  texture_cam={s.has_texture_camera}")

    cfg = ArtecConfig(serial_number=None, capture_texture=True)
    client = ArtecClient(cfg)

    try:
        client.initialize()
        chk("ArtecClient.initialize()", client._initialized)

        chk("scanner_capsule() 반환",   client.scanner_capsule()   is not None)
        chk("processor_capsule() 반환", client.processor_capsule() is not None)

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
            visualize=True,
            with_texture=False,
        )
        chk("artec_capturing 전체 통과", capturing_ok)

        # ──────────────────────────────────────────────────────────
        # 10. artec_scanning — Scanning API 바인딩 + Open3D 시각화
        # ──────────────────────────────────────────────────────────
        _section("10. artec_scanning_py 바인딩 검증 (+ Open3D 시각화)")

        scan_model_container: list = []
        scanning_ok = artec_scanning.verify_artec_scanning(
            client,
            visualize=True,
            record_seconds=5.0,
            scan_model_out=scan_model_container,
        )
        chk("artec_scanning 전체 통과", scanning_ok)
        scan_model = scan_model_container[0] if scan_model_container else None

        # ──────────────────────────────────────────────────────────
        # 11. artec_algorithm — Algorithm API 바인딩 + Open3D 시각화
        #     scanning 결과 ModelHandle(다중 프레임)을 재사용
        # ──────────────────────────────────────────────────────────
        _section("11. artec_algorithm_py 바인딩 검증 (+ Open3D 시각화)")

        algorithm_ok = artec_algorithm.verify_artec_algorithm(
            client,
            visualize=True,
            scan_model=scan_model,
        )
        chk("artec_algorithm 전체 통과", algorithm_ok)

        # ──────────────────────────────────────────────────────────
        # 12. artec_project — Project API 바인딩 + Open3D 시각화
        #     scanning 결과 ModelHandle을 재사용 (UUID 보유)
        # ──────────────────────────────────────────────────────────
        _section("12. artec_project_py 바인딩 검증 (+ Open3D 시각화)")

        project_ok = artec_project.verify_artec_project(
            client,
            visualize=True,
            scan_model=scan_model,
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
