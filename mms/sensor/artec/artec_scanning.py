# mms/sensor/artec/artec_scanning.py
#
# Artec Scanning API Python 래퍼.
# C++ 바인딩(artec_scanning_py)은 capsule 기반 free function만 노출하며,
# 모든 클래스 계층은 이 파일에서 Python으로 구현한다.
#
# 주요 클래스
# -----------
#   ScanSessionSettings   : 스캔 파이프라인 설정 (@dataclass + 편의 setter)
#   ScanSession           : 단일 스캐너 스캔 세션 (preview/record/stop 제어)
#   FrameEvent            : onFrameScanned 이벤트 DTO (@dataclass)
#
# 열거형
# ------
#   ScanningState, RegistrationType, CaptureTextureMethod,
#   FrameState, ScanningPipelineFlags
#
# 시각화 / 검증
# -------------
#   visualize_scan_model(model_handle)
#   verify_artec_scanning(artec_client, visualize=True, record_seconds=5.0)

from __future__ import annotations

import dataclasses
import os
import sys
import time
from dataclasses import dataclass, field
from enum import IntEnum, IntFlag
from pathlib import Path
from typing import Callable, Optional

import numpy as np

_ARTEC_SDK_BIN = Path(r"C:\Program Files\Artec\Artec 3D Scanning SDK\bin-x64")
_HERE = Path(__file__).parent

_scanning_py = None  # lazy-loaded artec_scanning_py module


def _load():
    """artec_scanning_py .pyd를 lazy-load하고 반환."""
    global _scanning_py
    if _scanning_py is not None:
        return _scanning_py

    if _ARTEC_SDK_BIN.exists():
        os.add_dll_directory(str(_ARTEC_SDK_BIN))
        os.environ["PATH"] = str(_ARTEC_SDK_BIN) + ";" + os.environ.get("PATH", "")

    if str(_HERE) not in sys.path:
        sys.path.insert(0, str(_HERE))

    try:
        import artec_scanning_py as _m
        _scanning_py = _m
        return _scanning_py
    except ImportError as e:
        raise ImportError(
            "[artec_scanning] artec_scanning_py 모듈 로드 실패.\n"
            "  CMakeLists.txt로 빌드 후 artec_scanning_py.pyd를 mms/sensor/artec/에 배치.\n"
            f"  원인: {e}"
        ) from e


# ==============================================================
# 열거형
# ==============================================================

class ScanningState(IntEnum):
    """IScanningProcedure 상태 열거형."""
    PREVIEW = 0   # ScanningState_Preview
    RECORD  = 1   # ScanningState_Record
    STOP    = 3   # ScanningState_Stop


class RegistrationType(IntEnum):
    """프레임 등록 알고리즘."""
    ICP     = 0   # RegistrationAlgorithmType_ICP
    HYBRID  = 1   # RegistrationAlgorithmType_Hybrid
    TEXTURE = 2   # RegistrationAlgorithmType_Texture


class CaptureTextureMethod(IntEnum):
    """텍스처 캡처 방식."""
    NONE               = 0   # CaptureTextureMethod_NoTextures
    EVERY_N            = 1   # CaptureTextureMethod_EveryNFrame
    ON_TEXTURE_KEYFRAME = 2  # CaptureTextureMethod_OnTextureKeyFrame
    ALWAYS             = 3   # CaptureTextureMethod_Always


class FrameState(IntEnum):
    """프레임 처리 결과 상태."""
    OK                      = 0
    TRIGGER_CAPTURE_FAILED  = 1
    CAPTURE_FAILED          = 2
    RECONSTRUCTION_FAILED   = 3
    REGISTRATION_FAILED     = 4
    TEXTURE_MAPPING_FAILED  = 5
    ADD_TO_SCAN_FAILED      = 6


class ScanningPipelineFlags(IntFlag):
    """스캔 파이프라인 단계 플래그 (OR 조합 가능)."""
    CAPTURE_ONLY           = 0x00  # ScanningPipeline_CaptureOnly
    CONVERT_TEXTURES       = 0x01  # ScanningPipeline_ConvertTextures
    CALCULATE_NORMALS      = 0x02  # ScanningPipeline_CalculateNormals
    MAP_TEXTURE            = 0x04  # ScanningPipeline_MapTexture
    REGISTER_FRAME         = 0x08  # ScanningPipeline_RegisterFrame
    FIND_GEOMETRY_KEYFRAME = 0x10  # ScanningPipeline_FindGeometryKeyFrame
    FAST_CAPTURE           = 0x20  # ScanningPipeline_FastCapture


# ==============================================================
# ScanSessionSettings
# ==============================================================

@dataclass
class ScanSessionSettings:
    """
    IScanningProcedure 생성에 필요한 설정.

    사용 예
    -------
    >>> s = ScanSessionSettings.default()
    >>> s.set_registration_type(RegistrationType.ICP)
    >>> s.set_initial_state(ScanningState.PREVIEW)
    >>> s.set_max_frame_count(100)
    """

    max_frame_count:             int                  = 0
    registration_type:           RegistrationType     = RegistrationType.ICP
    pipeline:                    int                  = (
        ScanningPipelineFlags.REGISTER_FRAME |
        ScanningPipelineFlags.FIND_GEOMETRY_KEYFRAME
    )
    initial_state:               ScanningState        = ScanningState.PREVIEW
    capture_texture:             CaptureTextureMethod = CaptureTextureMethod.NONE
    capture_texture_frequency:   int                  = 0
    ignore_registration_errors:  bool                 = False
    save_empty_surfaces:         bool                 = False

    @classmethod
    def default(cls) -> "ScanSessionSettings":
        """SDK 기본값으로 초기화된 설정 반환."""
        d = _load().init_scan_settings()
        return cls(
            max_frame_count=d["max_frame_count"],
            registration_type=RegistrationType(d["registration_type"]),
            pipeline=d["pipeline"],
            initial_state=ScanningState(d["initial_state"]),
            capture_texture=CaptureTextureMethod(d["capture_texture"]),
            capture_texture_frequency=d["capture_texture_frequency"],
            ignore_registration_errors=d["ignore_registration_errors"],
            save_empty_surfaces=d["save_empty_surfaces"],
        )

    def set_registration_type(self, v: RegistrationType) -> None:
        """프레임 등록 알고리즘 설정 (ICP / HYBRID / TEXTURE)."""
        self.registration_type = v

    def set_pipeline(self, flags: int) -> None:
        """ScanningPipelineFlags OR 조합으로 파이프라인 구성."""
        self.pipeline = int(flags)

    def set_initial_state(self, state: ScanningState) -> None:
        """executeJob 시작 직후 초기 상태 (PREVIEW / RECORD)."""
        self.initial_state = state

    def set_max_frame_count(self, n: int) -> None:
        """최대 캡처 프레임 수 (0=무제한)."""
        self.max_frame_count = n

    def set_capture_texture(
        self,
        method: CaptureTextureMethod,
        frequency: Optional[int] = None,
    ) -> None:
        """텍스처 캡처 방식 설정."""
        self.capture_texture = method
        if frequency is not None:
            self.capture_texture_frequency = frequency

    def set_ignore_registration_errors(self, v: bool) -> None:
        """등록 실패 프레임 처리 정책."""
        self.ignore_registration_errors = v

    def set_save_empty_surfaces(self, v: bool) -> None:
        """재구성 실패 프레임을 빈 surface로 유지할지 여부."""
        self.save_empty_surfaces = v

    def _to_dict(self) -> dict:
        d = dataclasses.asdict(self)
        # enum → int 변환
        d["registration_type"] = int(self.registration_type)
        d["pipeline"]          = int(self.pipeline)
        d["initial_state"]     = int(self.initial_state)
        d["capture_texture"]   = int(self.capture_texture)
        return d


# ==============================================================
# FrameEvent
# ==============================================================

@dataclass
class FrameEvent:
    """
    onFrameScanned 이벤트 DTO.

    Attributes
    ----------
    frame_state  : FrameState — 프레임 처리 결과
    frame_mesh   : FrameMeshHandle | None — OK 상태일 때 mesh
    scan_index   : int | None — 번들에서 스캐너 인덱스 (0 = 단일 스캐너)
    """
    frame_state:  FrameState
    frame_mesh:   Optional[object]  # artec_base_py.FrameMeshHandle | None
    scan_index:   Optional[int] = None


# ==============================================================
# ScanSession
# ==============================================================

class ScanSession:
    """
    단일 스캐너 스캔 세션.

    전형적인 사용 패턴
    ------------------
    >>> session = ScanSession.create(scanner_handle)
    >>> session.start_preview()          # preview 모드로 executeJob 시작
    >>> time.sleep(2)                    # 물체 위치 잡기
    >>> session.start_record()           # record 모드로 전환
    >>> time.sleep(5)                    # N초 스캔
    >>> model = session.stop()           # 중지 + 결과 ModelHandle 반환
    """

    def __init__(self, sess_cap) -> None:
        self._cap = sess_cap
        self._frame_callback: Optional[Callable[[FrameEvent], None]] = None

    @classmethod
    def create(
        cls,
        scanner_handle,
        settings: Optional[ScanSessionSettings] = None,
    ) -> "ScanSession":
        """
        스캔 세션 생성.

        Parameters
        ----------
        scanner_handle : artec_capturing.ScannerHandle
            IScanner* capsule을 보유한 ScannerHandle.
        settings : ScanSessionSettings | None
            None이면 SDK 기본값 사용.
        """
        settings_dict = settings._to_dict() if settings is not None else None
        sess_cap = _load().create_session(scanner_handle._cap, settings_dict)
        return cls(sess_cap)

    # ── 실행 제어 ──────────────────────────────────────────────

    def start_preview(self) -> None:
        """Preview 모드로 executeJob 시작 (프레임 캡처만, 스캔에 추가 안 됨)."""
        m = _load()
        m.session_set_state(self._cap, int(ScanningState.PREVIEW))
        if not m.session_is_running(self._cap):
            m.session_launch(self._cap)

    def start_record(self) -> None:
        """Record 모드로 전환 (프레임을 스캔에 추가). 미실행이면 자동 launch."""
        m = _load()
        m.session_set_state(self._cap, int(ScanningState.RECORD))
        if not m.session_is_running(self._cap):
            m.session_launch(self._cap)

    def stop(self) -> Optional[object]:
        """
        스캔 중지 후 결과 ModelHandle 반환.

        Returns
        -------
        artec_base.ModelHandle | None
        """
        from mms.sensor.artec import artec_base
        m = _load()
        m.session_set_state(self._cap, int(ScanningState.STOP))
        m.session_join(self._cap)
        cap = m.session_result_model(self._cap)
        return artec_base.ModelHandle(cap) if cap is not None else None

    # ── 상태 조회 / 변경 ───────────────────────────────────────

    def state(self) -> ScanningState:
        """현재 ScanningState 반환."""
        return ScanningState(_load().session_get_state(self._cap))

    def set_state(self, state: ScanningState) -> None:
        """ScanningState를 직접 설정 (실행 중에도 호출 가능)."""
        _load().session_set_state(self._cap, int(state))

    def is_running(self) -> bool:
        """background executeJob이 실행 중이면 True."""
        return _load().session_is_running(self._cap)

    # ── 감도 ───────────────────────────────────────────────────

    def sensitivity(self) -> float:
        """현재 감도 반환 (0.0–1.0)."""
        return _load().session_sensitivity(self._cap)

    def set_sensitivity(self, v: float) -> None:
        """mesh reconstruction 감도 설정 (0.0–1.0)."""
        _load().session_set_sensitivity(self._cap, v)

    # ── 스캔 범위 ──────────────────────────────────────────────

    def scanning_range(self) -> tuple:
        """현재 스캔 범위 (near_mm, far_mm) 반환."""
        return _load().session_scanning_range(self._cap)

    def set_scanning_range(self, near: float, far: float) -> None:
        """근/원거리 스캔 범위 설정 (mm 단위)."""
        _load().session_set_scanning_range(self._cap, near, far)

    # ── ROI (2차 구현) ─────────────────────────────────────────

    def set_roi(self, x: float, y: float, w: float, h: float) -> None:
        """화면 ROI 지정 (x, y, width, height)."""
        _load().session_set_roi(self._cap, x, y, w, h)

    def clear_roi(self) -> None:
        """ROI 비활성화."""
        _load().session_clear_roi(self._cap)

    def roi(self) -> Optional[tuple]:
        """현재 ROI (x, y, w, h) 반환. 미설정이면 None."""
        return _load().session_get_roi(self._cap)

    # ── 이벤트 ────────────────────────────────────────────────

    def set_frame_callback(
        self, fn: Optional[Callable[[FrameEvent], None]]
    ) -> None:
        """
        onFrameScanned 이벤트 콜백 등록.

        fn(event: FrameEvent) → None. None이면 콜백 해제.
        poll_events()를 주기적으로 호출해 이벤트를 dispatch한다.
        """
        self._frame_callback = fn

    def poll_events(self) -> list:
        """
        onFrameScanned 이벤트를 드레인해 list[FrameEvent]로 반환.

        set_frame_callback이 등록된 경우 각 이벤트에 콜백도 호출.
        """
        from mms.sensor.artec import artec_base
        raw = _load().session_poll_events(self._cap)
        events: list[FrameEvent] = []
        for d in raw:
            cap = d.get("frame_mesh")
            fmh = artec_base.FrameMeshHandle(cap) if cap is not None else None
            ev = FrameEvent(
                frame_state=FrameState(d["frame_state"]),
                frame_mesh=fmh,
                scan_index=d.get("scanner_index"),
            )
            events.append(ev)
            if self._frame_callback is not None:
                try:
                    self._frame_callback(ev)
                except Exception:
                    pass
        return events


# ==============================================================
# 시각화
# ==============================================================

def visualize_scan_model(model_handle, title: str = "Artec Scan Result") -> None:
    """
    ModelHandle의 모든 scan → frame을 Open3D로 시각화.

    mm 단위 좌표를 m 단위로 변환해서 표시한다.
    open3d가 없으면 자동으로 skip.
    """
    try:
        import open3d as o3d
    except ImportError:
        print("  [visualize_scan_model] open3d not installed — skipping.")
        return

    geometries = []
    total_verts = 0
    total_frames = 0

    for si in range(model_handle.scan_count()):
        scan = model_handle.get_scan(si)
        for fi in range(scan.frame_count()):
            frame = scan.get_frame(fi)
            verts = frame.vertices()
            faces = frame.faces()
            if verts.shape[0] == 0:
                continue
            total_verts  += verts.shape[0]
            total_frames += 1

            mesh = o3d.geometry.TriangleMesh()
            # mm → m 단위 변환
            mesh.vertices = o3d.utility.Vector3dVector(
                verts.astype(np.float64) / 1000.0
            )
            if faces.shape[0] > 0:
                mesh.triangles = o3d.utility.Vector3iVector(faces)
                mesh.compute_vertex_normals()

            geometries.append(mesh)

    if not geometries:
        print("  [visualize_scan_model] No geometry to display.")
        return

    print(
        f"  [visualize_scan_model] {total_frames} frames, "
        f"{total_verts:,} vertices total."
    )
    o3d.visualization.draw_geometries(geometries, window_name=title)


# ==============================================================
# 바인딩 검증
# ==============================================================

def verify_artec_scanning(
    artec_client,
    visualize: bool = True,
    record_seconds: float = 5.0,
) -> bool:
    """
    artec_scanning_py 바인딩 전체를 검증한다.

    Parameters
    ----------
    artec_client : ArtecClient
        initialize() 완료된 ArtecClient 인스턴스.
    visualize : bool
        True → 스캔 결과를 Open3D로 시각화.
    record_seconds : float
        Record 모드 지속 시간(초).

    Returns
    -------
    bool
        모든 검증 통과 시 True.
    """
    from mms.sensor.artec import artec_capturing

    failures: list[str] = []

    def chk(label: str, cond: bool, detail: str = "") -> bool:
        status = "OK" if cond else "FAIL"
        msg = f"    [{status}] {label}"
        if detail:
            msg += f"  ({detail})"
        print(msg)
        if not cond:
            failures.append(label)
        return cond

    # ──────────────────────────────────────────────────────────
    # 1. 모듈 로드
    # ──────────────────────────────────────────────────────────
    print("  [artec_scanning] 1. 모듈 로드")

    try:
        m = _load()
        chk("artec_scanning_py import", m is not None)
    except Exception as e:
        chk("artec_scanning_py import", False, str(e))
        return False

    # ──────────────────────────────────────────────────────────
    # 2. 함수 존재 여부
    # ──────────────────────────────────────────────────────────
    print("  [artec_scanning] 2. 함수 존재 여부")

    fn_names = [
        "init_scan_settings",
        "create_session",
        "session_launch",
        "session_join",
        "session_is_running",
        "session_get_state",
        "session_set_state",
        "session_sensitivity",
        "session_set_sensitivity",
        "session_scanning_range",
        "session_set_scanning_range",
        "session_set_roi",
        "session_clear_roi",
        "session_get_roi",
        "session_result_model",
        "session_poll_events",
    ]
    for fn in fn_names:
        chk(f"  {fn} 존재", hasattr(m, fn))

    # ──────────────────────────────────────────────────────────
    # 3. init_scan_settings()
    # ──────────────────────────────────────────────────────────
    print("  [artec_scanning] 3. init_scan_settings()")

    defaults = None
    try:
        defaults = m.init_scan_settings()
        chk("init_scan_settings() 반환", isinstance(defaults, dict))
    except Exception as e:
        chk("init_scan_settings() 반환", False, str(e))

    if defaults:
        expected_keys = [
            "max_frame_count", "registration_type", "pipeline",
            "initial_state", "capture_texture",
            "capture_texture_frequency", "ignore_registration_errors",
            "save_empty_surfaces",
        ]
        for k in expected_keys:
            chk(f"  key '{k}' 존재", k in defaults)

    # ──────────────────────────────────────────────────────────
    # 4. ScanSessionSettings.default()
    # ──────────────────────────────────────────────────────────
    print("  [artec_scanning] 4. ScanSessionSettings.default()")

    try:
        settings = ScanSessionSettings.default()
        chk("ScanSessionSettings.default() 성공", True)
        chk("  initial_state → ScanningState",
            isinstance(settings.initial_state, ScanningState))
        chk("  registration_type → RegistrationType",
            isinstance(settings.registration_type, RegistrationType))
    except Exception as e:
        chk("ScanSessionSettings.default()", False, str(e))
        settings = ScanSessionSettings()

    # ──────────────────────────────────────────────────────────
    # 5. ScanSession.create() — 세션 생성
    # ──────────────────────────────────────────────────────────
    print("  [artec_scanning] 5. ScanSession.create()")

    # ArtecClient._scanner.scanner_capsule() → ScannerHandle
    scanner_cap = artec_client._scanner.scanner_capsule()
    scanner_handle = artec_capturing.ScannerHandle(scanner_cap)

    settings.set_initial_state(ScanningState.PREVIEW)

    session: Optional[ScanSession] = None
    try:
        session = ScanSession.create(scanner_handle, settings)
        chk("ScanSession.create() 성공", session is not None)
    except Exception as e:
        chk("ScanSession.create() 성공", False, str(e))
        return len(failures) == 0

    # ──────────────────────────────────────────────────────────
    # 6. 상태 / 감도 / range 조회 (실행 전)
    # ──────────────────────────────────────────────────────────
    print("  [artec_scanning] 6. 상태 / 감도 / range 조회")

    try:
        st = session.state()
        chk("state() → ScanningState", isinstance(st, ScanningState),
            f"state={st.name}")
    except Exception as e:
        chk("state()", False, str(e))

    try:
        sens = session.sensitivity()
        chk("sensitivity() → float", isinstance(sens, float),
            f"sens={sens:.3f}")
    except Exception as e:
        chk("sensitivity()", False, str(e))

    try:
        near_v, far_v = session.scanning_range()
        chk("scanning_range() → (near, far)",
            isinstance(near_v, float) and isinstance(far_v, float),
            f"near={near_v:.1f} far={far_v:.1f}")
    except Exception as e:
        chk("scanning_range()", False, str(e))

    # ──────────────────────────────────────────────────────────
    # 7. start_preview → record → stop
    # ──────────────────────────────────────────────────────────
    print("  [artec_scanning] 7. start_preview → start_record → stop")
    print(f"    (preview 2s → record {record_seconds}s)")

    model = None
    frame_events: list[FrameEvent] = []

    try:
        session.start_preview()
        chk("start_preview() 성공", True)
        chk("is_running() == True", session.is_running())
    except Exception as e:
        chk("start_preview()", False, str(e))

    # preview 2초
    t0 = time.perf_counter()
    while time.perf_counter() - t0 < 2.0:
        time.sleep(0.1)
        session.poll_events()  # 드레인 (preview 이벤트는 무시)

    try:
        session.start_record()
        chk("start_record() 성공", True)
    except Exception as e:
        chk("start_record()", False, str(e))

    # record N초
    t0 = time.perf_counter()
    while time.perf_counter() - t0 < record_seconds:
        time.sleep(0.2)
        evts = session.poll_events()
        frame_events.extend(evts)

    ok_frames = [e for e in frame_events if e.frame_state == FrameState.OK]
    print(f"    스캔 이벤트: {len(frame_events)}건 (OK={len(ok_frames)})")

    try:
        model = session.stop()
        chk("stop() 성공", True)
        chk("is_running() == False (stop 후)", not session.is_running())
    except Exception as e:
        chk("stop()", False, str(e))

    # ──────────────────────────────────────────────────────────
    # 8. 결과 ModelHandle 검증
    # ──────────────────────────────────────────────────────────
    print("  [artec_scanning] 8. 결과 ModelHandle 검증")

    if model is None:
        print("    ※ model == None — 스캐너 앞에 물체가 없었거나 스캔이 비어있음")
    else:
        chk("session_result_model() → ModelHandle",
            type(model).__name__ == "ModelHandle")

        sc_count = model.scan_count()
        chk("scan_count() >= 0", sc_count >= 0,
            f"scan_count={sc_count}")

        total_frames_count = 0
        for si in range(sc_count):
            scan = model.get_scan(si)
            total_frames_count += scan.frame_count()
        print(f"    총 {sc_count}개 스캔, {total_frames_count}개 프레임")

        chk("총 frame 수 > 0 (스캔 데이터 존재)",
            total_frames_count > 0,
            f"total_frames={total_frames_count}")

        ms = model.summary()
        chk("summary() → MeshSummary",
            type(ms).__name__ == "MeshSummary")

    # ──────────────────────────────────────────────────────────
    # 9. Open3D 시각화
    # ──────────────────────────────────────────────────────────
    if visualize and model is not None and model.scan_count() > 0:
        print("  [artec_scanning] 9. Open3D 시각화")
        try:
            visualize_scan_model(model, title="Artec Scanning Result")
        except Exception as e:
            print(f"    [WARN] 시각화 실패: {e}")

    passed = len(failures) == 0
    status = "전체 통과" if passed else f"{len(failures)}개 실패: {failures}"
    print(f"  [artec_scanning] 결과: {status}")
    return passed
