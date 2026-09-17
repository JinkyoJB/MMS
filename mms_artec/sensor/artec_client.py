# mms/sensor/artec/artec_client.py
#
# Artec 3D 스캐너 통합 클라이언트 (Facade).
#
# artec_capturing / artec_base / artec_scanning /
# artec_algorithm / artec_project 래퍼 모듈을 단일 인터페이스로 제공.
# system.py 는 이 파일만 import해서 모든 Artec 기능을 사용한다.
#
# 빌드 방법
# ---------
#   pip install pybind11
#   $cmake = "...\CMake\bin\cmake.exe"
#   & $cmake -B build -G "Visual Studio 18 2026" -A x64
#   & $cmake --build build --config Release

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Optional

import numpy as np

from utils.scan_result import ScanResult
from mms_artec.sensor import artec_base
from mms_artec.sensor import artec_capturing
from mms_artec.sensor import artec_scanning
from mms_artec.sensor import artec_algorithm
from mms_artec.sensor import artec_project


# ------------------------------------------------------------------
# Config
# ------------------------------------------------------------------

# ArtecConfig 는 바인딩 비의존 경량 모듈로 분리됨 (real/isaac 양쪽에서 import).
# 하위호환: 기존처럼 `from mms_artec.sensor.artec_client import ArtecConfig` 동작.
from mms_artec.sensor.artec_config import ArtecConfig  # noqa: E402,F401


# ------------------------------------------------------------------
# ArtecClient (Facade)
# ------------------------------------------------------------------

class ArtecClient:
    """
    Artec 3D 스캐너 통합 클라이언트.

    ┌──────────────────────────────────────────────────────┐
    │  Lifecycle      initialize() / shutdown()            │
    │  Single capture capture() / capture_frame()          │
    │                 capture_to_model()                   │
    │  Scanning       scan() / create_scan_session()       │
    │  Algorithms     serial_registration() …              │
    │                 run_pipeline()                       │
    │  Project I/O    save_project() / load_project()      │
    └──────────────────────────────────────────────────────┘
    """

    def __init__(self, cfg: ArtecConfig) -> None:
        self.cfg = cfg
        self._scanner: Optional[artec_capturing.ScannerHandle] = None
        self._processor: Optional[artec_capturing.FrameProcessorHandle] = None
        self._initialized: bool = False

    # ==============================================================
    # Lifecycle
    # ==============================================================

    def initialize(self) -> None:
        """스캐너에 연결하고 FrameProcessor를 초기화한다."""
        mgr = artec_capturing.ScannerManager()
        if self.cfg.serial_number:
            self._scanner = mgr.open_by_serial(self.cfg.serial_number)
        else:
            self._scanner = mgr.open(index=0)

        if self.cfg.fps is not None:
            fps = min(self.cfg.fps, self._scanner.max_fps())
            self._scanner.set_fps(fps)
            print(f"[ArtecClient] FPS={fps:.1f}")

        self._processor = self._scanner.create_frame_processor()
        self._initialized = True

        # ★ 작동거리 창 — 설정이 있으면 적용하고, **결과를 반드시 찍는다.**
        #   이 값이 `SensorModel.dof` 로 들어가 밴드 수를 정하므로, 실제로 뭐가
        #   걸렸는지 로그에 남아야 나중에 "왜 밴드가 이렇게 나왔나" 를 추적할 수 있다.
        near_cfg = getattr(self.cfg, "scan_range_near_mm", None)
        far_cfg = getattr(self.cfg, "scan_range_far_mm", None)
        if near_cfg is not None and far_cfg is not None:
            try:
                self._processor.set_scanning_range(float(near_cfg), float(far_cfg))
            except Exception as e:                       # noqa: BLE001
                print(f"[ArtecClient] ⚠ 스캔 범위 설정 실패({type(e).__name__}: {e})")
        try:
            n, f = self._processor.scanning_range()
            print(f"[ArtecClient] 작동거리 창 = {n:.0f}~{f:.0f}mm"
                  + ("" if near_cfg is not None else "  (SDK 기본값 — 설정 안 함)"))
        except Exception as e:                           # noqa: BLE001
            print(f"[ArtecClient] ⚠ 스캔 범위 조회 실패({type(e).__name__}: {e})")

        id_info = self._scanner.id()
        print(f"[ArtecClient] Connected: {id_info.name}  serial={id_info.serial}")

    def shutdown(self) -> None:
        """연결을 해제한다. 내부 핸들은 GC가 정리한다."""
        self._processor = None
        self._scanner = None
        self._initialized = False
        print("[ArtecClient] Disconnected.")

    # ==============================================================
    # Capsule accessors
    # ==============================================================

    def scanner_capsule(self):
        """IScanner* capsule (artec_base / artec_scanning 내부에서 사용)."""
        return self._scanner.scanner_capsule()

    def processor_capsule(self):
        """IFrameProcessor* capsule (artec_base 내부에서 사용)."""
        return self._processor.processor_capsule()

    # ==============================================================
    # Single-frame capture
    # ==============================================================

    # ── 스캔 범위 (작동거리 창) ─────────────────────────────────────
    #  ★ 계획기(`SensorModel.dof`)가 이 값을 쓴다. 여태 하드코딩 추측이었는데,
    #    SDK 가 그대로 알려주므로 추정하지 않는다. **sim 스캐너도 같은 이름으로
    #    같은 의미를 돌려준다**(`IsaacArtecScanner.scanning_range`) — 호출부가
    #    백엔드를 안 가리게 하려는 것이다.
    def scanning_range(self):
        """(near_mm, far_mm) — SDK `IFrameProcessor::getScanningRange`."""
        if self._processor is None:
            raise RuntimeError("scanning_range: processor 없음 (initialize 먼저)")
        return self._processor.scanning_range()

    def set_scanning_range(self, near_mm: float, far_mm: float) -> None:
        """근/원거리 스캔 범위 설정 (mm)."""
        if self._processor is None:
            raise RuntimeError("set_scanning_range: processor 없음 (initialize 먼저)")
        self._processor.set_scanning_range(float(near_mm), float(far_mm))

    def capture(self, frame_id: int = 0, timestamp: Optional[float] = None) -> Optional[ScanResult]:
        """
        1회 캡처 → ScanResult.

        Returns
        -------
        ScanResult
            points    : (N,3) float32, mm, 센서 프레임
            triangles : (M,3) int32 | None
            img       : (H,W,3) uint8 | None
            normals   : None  (IFrameMesh 는 법선 미제공)
        """
        self._require_init()
        if timestamp is None:
            timestamp = time.perf_counter()

        t0  = time.perf_counter()
        raw = self._scanner.capture(capture_texture=self.cfg.capture_texture)
        t1  = time.perf_counter()

        fmh = (self._processor.reconstruct_textured(raw)
               if self.cfg.capture_texture
               else self._processor.reconstruct(raw))

        pts = fmh.vertices()
        if pts.shape[0] == 0:
            return None

        tri = fmh.faces()
        tex = fmh.image()

        scan = ScanResult(
            sensor_type="artec",
            points=pts,
            normals=None,
            triangles=tri if tri.shape[0] > 0 else None,
            img=tex,
            frame_id=frame_id,
            timestamp=timestamp,
        )

        elapsed = time.perf_counter() - t0
        print(f"  [ArtecClient] capture={t1-t0:.3f}s  total={elapsed:.3f}s"
              f"  pts={len(scan.points):,}")

        remaining = self.cfg.target_interval_s - elapsed
        if remaining > 0.0:
            time.sleep(remaining)

        return scan

    def capture_frame(
        self,
        capture_texture: Optional[bool] = None,
    ) -> Optional[artec_base.FrameMeshHandle]:
        """
        1회 캡처 → FrameMeshHandle.

        Parameters
        ----------
        capture_texture : bool | None  (None → cfg.capture_texture)
        """
        self._require_init()
        return artec_base.capture_frame_handle(self, capture_texture=capture_texture)

    def capture_to_model(
        self,
        capture_texture: Optional[bool] = None,
    ) -> Optional[artec_base.ModelHandle]:
        """
        1회 캡처 → ModelHandle (IScan 1개, IFrameMesh 1개 구조).

        Parameters
        ----------
        capture_texture : bool | None  (None → cfg.capture_texture)
        """
        self._require_init()
        return artec_base.capture_to_model(self, capture_texture=capture_texture)

    def capture_points_base(self, robot, T_EC, **_) -> np.ndarray:
        """
        1회 단일프레임 캡처 → 점군을 **로봇 base 프레임**(m)으로 반환.

        IsaacArtecScanner.capture_points_base 와 동일 인터페이스 (sim/real 통일).

          1. capture()      → 센서 프레임 단일프레임 점군 (mm, SLAM/추적 미사용)
          2. p_base = T_CB @ p_sensor,  T_CB = compute_T_CB(robot FK, T_EC)

        ★ Artec 의 first-frame 추적 transform 은 쓰지 않는다. 카메라 위치는 로봇
          (EE FK + 손-눈 T_EC)이 알려준다.

        Parameters
        ----------
        robot : XArmInterface 호환 (get_ee_pose_mat → T_EB, 병진 m)
        T_EC  : (4,4) 손-눈 (E→C)
        """
        from utils.transforms import compute_T_CB
        res = self.capture()
        if res is None or res.points is None or len(res.points) == 0:
            return np.zeros((0, 3))
        pts = np.asarray(res.points, dtype=float) / 1000.0      # mm → m, 센서 프레임
        T_CB = compute_T_CB(robot.get_ee_pose_mat(), np.asarray(T_EC, dtype=float))
        ph = np.concatenate([pts, np.ones((len(pts), 1))], axis=1)
        return (ph @ T_CB.T)[:, :3]

    # ==============================================================
    # Scanning session
    # ==============================================================

    def scan(
        self,
        record_seconds: float = 5.0,
        preview_seconds: float = 2.0,
        settings: Optional[artec_scanning.ScanSessionSettings] = None,
    ) -> Optional[artec_base.ModelHandle]:
        """
        Preview → Record → Stop 전체 스캔 파이프라인을 실행한다.

        Parameters
        ----------
        record_seconds  : Record 모드 지속 시간(초)
        preview_seconds : Record 전 Preview 대기 시간(초)
        settings        : ScanSessionSettings | None  (None → SDK 기본값)

        Returns
        -------
        artec_base.ModelHandle | None
        """
        self._require_init()
        session = artec_scanning.ScanSession.create(self._scanner, settings)
        session.start_preview()
        t0 = time.perf_counter()
        while time.perf_counter() - t0 < preview_seconds:
            time.sleep(0.1)
            session.poll_events()

        session.start_record()
        t0 = time.perf_counter()
        while time.perf_counter() - t0 < record_seconds:
            time.sleep(0.2)
            session.poll_events()

        model = session.stop()
        if model is not None:
            total = sum(model.get_scan(i).frame_count()
                        for i in range(model.scan_count()))
            print(f"[ArtecClient] scan 완료  scans={model.scan_count()}  frames={total}")
        return model

    def create_scan_session(
        self,
        settings: Optional[artec_scanning.ScanSessionSettings] = None,
    ) -> artec_scanning.ScanSession:
        """
        ScanSession을 생성해 반환한다.
        start_preview() / start_record() / stop() 을 직접 제어할 때 사용.
        """
        self._require_init()
        return artec_scanning.ScanSession.create(self._scanner, settings)

    # ==============================================================
    # Model utilities
    # ==============================================================

    @staticmethod
    def create_model() -> artec_base.ModelHandle:
        """빈 ModelHandle 생성."""
        return artec_base.create_model()

    # ==============================================================
    # Algorithms
    # ==============================================================

    @staticmethod
    def serial_registration(
        model: artec_base.ModelHandle,
        settings: Optional[artec_algorithm.SerialRegistrationSettingsDTO] = None,
    ) -> artec_base.ModelHandle:
        """Serial Registration. 최소 2프레임 필요."""
        return artec_algorithm.Algorithms.serial_registration(model, settings)

    @staticmethod
    def global_registration(
        model: artec_base.ModelHandle,
        settings: Optional[artec_algorithm.GlobalRegistrationSettingsDTO] = None,
    ) -> artec_base.ModelHandle:
        """Global Registration."""
        return artec_algorithm.Algorithms.global_registration(model, settings)

    @staticmethod
    def outliers_removal(
        model: artec_base.ModelHandle,
        settings: Optional[artec_algorithm.OutliersRemovalSettingsDTO] = None,
    ) -> artec_base.ModelHandle:
        """Outliers Removal."""
        return artec_algorithm.Algorithms.outliers_removal(model, settings)

    @staticmethod
    def small_objects_filter(
        model: artec_base.ModelHandle,
        settings: Optional[artec_algorithm.SmallObjectsFilterSettingsDTO] = None,
    ) -> artec_base.ModelHandle:
        """Small Objects Filter."""
        return artec_algorithm.Algorithms.small_objects_filter(model, settings)

    @staticmethod
    def fast_fusion(
        model: artec_base.ModelHandle,
        settings: Optional[artec_algorithm.FastFusionSettingsDTO] = None,
    ) -> artec_base.ModelHandle:
        """Fast Fusion — 빠른 합성 메시 생성."""
        return artec_algorithm.Algorithms.fast_fusion(model, settings)

    @staticmethod
    def poisson_fusion(
        model: artec_base.ModelHandle,
        settings: Optional[artec_algorithm.PoissonFusionSettingsDTO] = None,
    ) -> artec_base.ModelHandle:
        """Poisson Fusion — watertight 합성 메시 생성."""
        return artec_algorithm.Algorithms.poisson_fusion(model, settings)

    @staticmethod
    def mesh_simplify(
        model: artec_base.ModelHandle,
        settings: Optional[artec_algorithm.MeshSimplificationSettingsDTO] = None,
    ) -> artec_base.ModelHandle:
        """Mesh Simplification (고품질)."""
        return artec_algorithm.Algorithms.mesh_simplify(model, settings)

    @staticmethod
    def texturize(
        model: artec_base.ModelHandle,
        settings: Optional[artec_algorithm.TexturizationSettingsDTO] = None,
    ) -> artec_base.ModelHandle:
        """Texturization."""
        return artec_algorithm.Algorithms.texturize(model, settings)

    @staticmethod
    def run_pipeline(
        model: artec_base.ModelHandle,
        pipeline: Optional[artec_algorithm.PipelineSettingsDTO] = None,
    ) -> artec_base.ModelHandle:
        """
        후처리 파이프라인을 순서대로 실행한다.

        기본 파이프라인: serial_registration → outliers_removal
                        → small_objects_filter → fast_fusion

        PipelineSettingsDTO 의 do_* 플래그로 각 단계를 개별 제어 가능.
        """
        return artec_algorithm.Algorithms.run_pipeline(model, pipeline)

    # ==============================================================
    # Project I/O
    # ==============================================================

    @staticmethod
    def save_project(
        model: artec_base.ModelHandle,
        path: str,
        compression_level: int = 0,
    ) -> None:
        """
        모델을 Artec Studio 프로젝트(.sproj)로 저장한다.

        Parameters
        ----------
        model             : 저장할 ModelHandle
        path              : 저장 경로 (예: "C:/data/scan.sproj")
        compression_level : 압축 수준 (0 = 기본값)
        """
        proj = artec_project.ProjectManager.create(path)
        proj.save(path=path, model=model, compression_level=compression_level)

    @staticmethod
    def open_project(path: str) -> artec_project.ProjectHandle:
        """
        저장된 프로젝트를 열어 ProjectHandle을 반환한다.

        load_entries() / save() 를 직접 제어할 때 사용.
        """
        return artec_project.ProjectManager.open(path)

    @staticmethod
    def load_project(path: str) -> list:
        """
        프로젝트를 열고 모든 엔트리를 로드해 반환한다.

        Returns
        -------
        list[artec_project.LoadedProjectEntry]
            SCAN 엔트리 → .scan (artec_base.ScanHandle)
            COMPOSITE_MESH 엔트리 → .mesh (artec_project.CompositeMeshHandle)
        """
        proj = artec_project.ProjectManager.open(path)
        return proj.load_entries()

    # ==============================================================
    # 내부
    # ==============================================================

    def _require_init(self) -> None:
        if not self._initialized:
            raise RuntimeError("[ArtecClient] Not initialized. Call initialize() first.")
