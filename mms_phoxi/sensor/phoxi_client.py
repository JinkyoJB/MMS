# mms/sensor/phoxi/phoxi_client.py
#
# Photoneo PhoXi3D 클라이언트.
# Harvesters + mvGenTLProducer.cti (GigE-V / GenTL) 기반.

from __future__ import annotations

import os
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional

import numpy as np
from genicam.genapi import NodeMap
from harvesters.core import Component2DImage, Harvester, ImageAcquirer

try:
    import open3d as o3d
    _HAS_O3D = True
except ImportError:
    o3d = None  # type: ignore
    _HAS_O3D = False

from utils.scan_result import ScanResult


# ------------------------------------------------------------------
# CTI 파일 탐색
# ------------------------------------------------------------------

def _find_cti() -> Path:
    """
    mvGenTLProducer.cti 경로 탐색 (GENICAM_GENTL64_PATH 기반).
    """
    env_val = os.getenv("GENICAM_GENTL64_PATH")
    if env_val is None:
        raise EnvironmentError(
            "[PhoxiClient] GENICAM_GENTL64_PATH 환경변수가 설정되지 않았습니다.\n"
            "  GENICAM_GENTL64_PATH를 mvGenTLProducer.cti 위치로 설정하세요."
        )
    for p in env_val.split(os.pathsep):
        cti = Path(p) / "mvGenTLProducer.cti"
        if cti.exists():
            return cti
    raise FileNotFoundError(
        f"[PhoxiClient] mvGenTLProducer.cti not found in GENICAM_GENTL64_PATH={env_val}"
    )


# ------------------------------------------------------------------
# Harvesters 내부 유틸
# ------------------------------------------------------------------

def _data_stream_reset(ia: ImageAcquirer) -> None:
    ia._release_data_streams()
    ia._setup_data_streams(file_dict=ia._file_dict)


def _sorted_components(features: NodeMap) -> List[str]:
    def id_value(comp: str) -> int:
        features.ComponentSelector.value = comp
        return features.ComponentIDValue.value
    return sorted(features.ComponentSelector.symbolics, key=id_value)


def _enable_components(features: NodeMap, component_list: List[str]) -> None:
    for comp in features.ComponentSelector.symbolics:
        features.ComponentSelector.value = comp
        features.ComponentEnable.value = (comp in component_list)


def _enabled_components(features: NodeMap) -> List[str]:
    result = []
    for comp in _sorted_components(features):
        features.ComponentSelector.value = comp
        if features.ComponentEnable.value:
            result.append(comp)
    return result


# ------------------------------------------------------------------
# Config
# ------------------------------------------------------------------

@dataclass
class PhoxiConfig:
    """
    PhoXi3D 클라이언트 설정.

    serial_number
        디바이스 시리얼 번호. None → 첫 번째 발견된 디바이스.
    trigger_timeout_s
        ia.fetch() 대기 최대 시간(초).
    target_interval_s
        캡처 호출당 최소 대기 시간(초). 0.0 이면 대기 없음.
    enable_color_camera
        True 면 `ColorCamera` GenICam 컴포넌트를 활성 — per-frame 2D RGB 이미지
        (해상도 = ColorSettings_Resolution, 기본 2576×1460) 를 fetch. Gen3 전용.
    texture_source
        "LED" (default), "Laser", "LaserEnhanced", "Color", "Computed", ...
        "Color" 로 두면 Intensity 컴포넌트가 **RGB-per-3D-point** 로 바뀜 (Gen3).
    color_resolution
        None 이면 그대로, 아니면 `ColorSettings_Resolution` 에 세팅.
        예: "Resolution_2576x1460", "Resolution_1932x1096".
    """
    serial_number: Optional[str] = None
    trigger_timeout_s: float = 15.0
    target_interval_s: float = 1.0
    enable_color_camera: bool = False
    texture_source: str = "LED"
    color_resolution: Optional[str] = None


# ------------------------------------------------------------------
# PhoxiClient
# ------------------------------------------------------------------

class PhoxiClient:
    """
    Photoneo PhoXi3D 센서 클라이언트.

    capture() 는 센서(S) 프레임의 ScanResult를 반환한다.
    좌표계 변환(Base 프레임 변환)은 시스템 레이어에서 수행.
    """

    def __init__(self, cfg: PhoxiConfig) -> None:
        self.cfg = cfg
        self._harvester: Optional[Harvester] = None
        self._ia: Optional[ImageAcquirer] = None
        self._features: Optional[NodeMap] = None
        self._comp_names: List[str] = []
        self._initialized: bool = False

        self._last_intensity: Optional[np.ndarray] = None
        self._last_organized_pts: Optional[np.ndarray] = None
        self._last_depth_mm: Optional[np.ndarray] = None          # (H, W) float32, mm
        self._last_organized_normals: Optional[np.ndarray] = None # (H, W, 3) float32
        self._last_color_image: Optional[np.ndarray] = None       # (H_c, W_c, 3) uint8 RGB
        self._last_intensity_is_rgb: bool = False                  # TextureSource=Color 일 때 True
        self._cached_intrinsic: Optional["o3d.camera.PinholeCameraIntrinsic"] = None

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def initialize(self) -> None:
        """
        GenTL Producer 로드 → 디바이스 탐색 → 연결 → 설정 → 취득 시작.
        """
        cti_path = _find_cti()
        print(f"[PhoxiClient] Using CTI: {cti_path}")

        h = Harvester()
        h.add_file(str(cti_path), check_existence=True, check_validity=True)
        h.update()

        devices = h.device_info_list
        if not devices:
            h.reset()
            raise RuntimeError("[PhoxiClient] No devices found.")

        print(f"[PhoxiClient] Found {len(devices)} device(s):")
        for i, dev in enumerate(devices):
            props = dev.property_dict
            print(f"  [{i}] serial={props.get('serial_number')}  model={props.get('model')}")

        search_key = (
            {"serial_number": self.cfg.serial_number}
            if self.cfg.serial_number else 0
        )
        ia = h.create(search_key)
        features: NodeMap = ia.remote_device.node_map

        fw_ver = features.DeviceFirmwareVersion.value
        print(f"[PhoxiClient] Connected. Firmware: {fw_ver}")

        features.UserSetSelector.value = "Default"
        features.UserSetLoad.execute()

        features.TriggerSelector.value = "FrameStart"
        features.TriggerMode.value = "On"
        features.TriggerSource.value = "Software"

        features.Scan3dOutputMode.value = "CalibratedABC_Grid"

        # TextureSource — cfg.texture_source 에서 지정한 값으로 세팅 (기본 "LED").
        # "Color" 로 두면 Gen3 의 Intensity 컴포넌트가 RGB-per-3D-point 가 된다.
        for _node_name in ["CapturingSettingsTextureSource", "TextureSource",
                           "CapturingSettings_TextureSource"]:
            try:
                node = getattr(features, _node_name, None)
                if node is not None:
                    node.value = self.cfg.texture_source
                    print(f"[PhoxiClient] TextureSource → {self.cfg.texture_source}  "
                          f"(node: {_node_name})")
                    break
            except Exception as e:
                print(f"[PhoxiClient] TextureSource 설정 실패 ({_node_name}): {e}")

        # ColorCamera 해상도 (옵션)
        if self.cfg.color_resolution is not None:
            try:
                features.ColorSettings_Resolution.value = self.cfg.color_resolution
                print(f"[PhoxiClient] ColorSettings_Resolution → "
                      f"{self.cfg.color_resolution}")
            except Exception as e:
                print(f"[PhoxiClient] ColorSettings_Resolution 설정 실패: {e}")

        # Components — ColorCamera 옵션 포함
        _components_to_enable = ["Intensity", "Range", "Normal"]
        if self.cfg.enable_color_camera:
            _components_to_enable.append("ColorCamera")
            print("[PhoxiClient] ColorCamera 컴포넌트 활성화")
        _enable_components(features, _components_to_enable)

        ia.start()

        self._harvester = h
        self._ia = ia
        self._features = features
        self._comp_names = _enabled_components(features)
        self._initialized = True
        print(f"[PhoxiClient] Acquisition started. components={self._comp_names}")

    def shutdown(self) -> None:
        if self._ia is not None:
            self._ia.stop()
            self._ia.destroy()
            self._ia = None
        if self._harvester is not None:
            self._harvester.reset()
            self._harvester = None
        self._features = None
        self._initialized = False
        print("[PhoxiClient] Disconnected.")

    # ------------------------------------------------------------------
    # Capture
    # ------------------------------------------------------------------

    def capture(self, frame_id: int = 0, timestamp: Optional[float] = None) -> Optional[ScanResult]:
        """
        소프트웨어 트리거로 1회 스캔.

        Returns
        -------
        ScanResult
            points: (H*W, 3) float32, mm, 센서(S) 프레임
            normals: (H*W, 3) float32
            img: (H, W, 3) uint8 그레이스케일→RGB
        """
        if not self._initialized:
            raise RuntimeError("[PhoxiClient] Not initialized. Call initialize() first.")

        if timestamp is None:
            timestamp = time.perf_counter()

        try:
            from _gentl import TimeoutException as _GentlTimeout
        except ImportError:
            _GentlTimeout = Exception

        _t0 = time.perf_counter()

        def _drain():
            while True:
                try:
                    with self._ia.fetch(timeout=0.05):
                        pass
                except Exception:
                    break

        def _trigger_and_fetch():
            _drain()
            self._features.TriggerSoftware.execute()
            return self._ia.fetch(timeout=self.cfg.trigger_timeout_s)

        last_exc = None
        _buf_ctx = None
        for _attempt in range(3):
            try:
                _buf_ctx = _trigger_and_fetch()
                break
            except _GentlTimeout as e:
                last_exc = e
                if _attempt < 2:
                    print(f"[PhoxiClient] fetch timeout (시도 {_attempt+1}/3), 스트림 재시작...")
                    try:
                        self._ia.stop()
                        _data_stream_reset(self._ia)
                        self._ia.start()
                        time.sleep(2.0)
                    except Exception as restart_err:
                        print(f"[PhoxiClient] 재시작 실패: {restart_err}")
                        break
            except Exception as e:
                last_exc = e
                break

        if _buf_ctx is None:
            raise last_exc

        with _buf_ctx as buffer:
            _t1 = time.perf_counter()
            components: Dict[str, Component2DImage] = dict(
                zip(self._comp_names, buffer.payload.components)
            )

            if "Range" not in components:
                print("[PhoxiClient] Range component not found.")
                return None

            range_comp = components["Range"]
            h_img = range_comp.height
            w_img = range_comp.width
            points_flat = range_comp.data.reshape(-1, 3).copy().astype(np.float32)  # mm

            normal_flat: Optional[np.ndarray] = None
            if "Normal" in components:
                nm_comp = components["Normal"]
                normal_flat = nm_comp.data.reshape(-1, 3).copy().astype(np.float32)

            tex_raw: Optional[np.ndarray] = None
            tex_fmt: Optional[str] = None
            tex_h = h_img
            tex_w = w_img
            if "Intensity" in components:
                _tc = components["Intensity"]
                tex_fmt = _tc.data_format
                tex_raw = _tc.data.copy()
                tex_h = _tc.height
                tex_w = _tc.width

            # ColorCamera 컴포넌트 (옵션) — Gen3 의 2D RGB 카메라 이미지
            color_raw: Optional[np.ndarray] = None
            color_fmt: Optional[str] = None
            color_h = 0
            color_w = 0
            if "ColorCamera" in components:
                _cc = components["ColorCamera"]
                color_raw = _cc.data.copy()
                color_fmt = _cc.data_format
                color_h = _cc.height
                color_w = _cc.width

        _t2 = time.perf_counter()

        # 유효점 필터링 (all-zero 제거)
        valid = ~np.all(points_flat == 0.0, axis=1)
        points_valid = points_flat[valid]
        normals_valid = normal_flat[valid] if normal_flat is not None else None

        # Intensity 디코드
        # - TextureSource=Color 이고 data_format 이 3채널 (RGB8 등) 이면 RGB per-point
        # - 그 외에는 기존 grayscale 디코드
        img_rgb: Optional[np.ndarray] = None
        self._last_intensity_is_rgb = False
        if tex_raw is not None:
            is_rgb_texture = (
                self.cfg.texture_source.lower() == "color"
                and tex_fmt in ("RGB8", "BGR8", "RGB8Packed")
            )
            if is_rgb_texture:
                rgb = tex_raw.astype(np.uint8).reshape(tex_h, tex_w, 3)
                if tex_fmt.startswith("BGR"):
                    rgb = rgb[:, :, ::-1].copy()
                self._last_intensity = rgb          # (H, W, 3) uint8 RGB-per-point
                self._last_intensity_is_rgb = True
                img_rgb = rgb
            else:
                gray = self._decode_intensity(tex_raw, tex_fmt, tex_h, tex_w)
                self._last_intensity = gray         # (H, W) uint8 grayscale
                img_rgb = np.stack([gray, gray, gray], axis=-1)

        # ColorCamera 2D RGB 이미지 (별도 컴포넌트)
        if color_raw is not None:
            try:
                if color_fmt in ("RGB8", "RGB8Packed"):
                    self._last_color_image = color_raw.astype(np.uint8).reshape(
                        color_h, color_w, 3,
                    )
                elif color_fmt in ("BGR8", "BGR8Packed"):
                    bgr = color_raw.astype(np.uint8).reshape(color_h, color_w, 3)
                    self._last_color_image = bgr[:, :, ::-1].copy()
                else:
                    # Mono 계열이면 grayscale → 3채널 확장
                    gray_c = self._decode_intensity(
                        color_raw, color_fmt, color_h, color_w,
                    )
                    self._last_color_image = np.stack([gray_c] * 3, axis=-1)
                print(f"  [PhoxiClient] ColorCamera image {color_w}×{color_h}  "
                      f"fmt={color_fmt}")
            except Exception as e:
                print(f"  [PhoxiClient] ColorCamera decode 실패: {e}")
                self._last_color_image = None
        else:
            self._last_color_image = None

        organized_pts = points_flat.reshape(h_img, w_img, 3)
        self._last_organized_pts = organized_pts
        # 조직적(organized) depth 이미지 = Z 성분 (mm). 0 은 invalid.
        depth_mm = organized_pts[:, :, 2].astype(np.float32)
        depth_mm[depth_mm < 0] = 0.0
        self._last_depth_mm = depth_mm
        # Organized Normal — 데이터셋 덤프에서 pixel 대응 유지하려면 (H,W,3) 필요
        self._last_organized_normals = (
            normal_flat.reshape(h_img, w_img, 3).copy()
            if normal_flat is not None else None
        )

        _t3 = time.perf_counter()
        print(
            f"  [PhoxiClient] trigger+fetch={_t1-_t0:.3f}s  "
            f"decode={_t2-_t1:.3f}s  build={_t3-_t2:.3f}s  "
            f"pts={len(points_valid):,}"
        )

        result = ScanResult(
            sensor_type="phoxi",
            points=points_valid,
            normals=normals_valid,
            img=img_rgb,
            depth=depth_mm,                    # (H, W) float32, mm — organized
            frame_id=frame_id,
            timestamp=timestamp,
        )

        # target_interval_s 대기
        elapsed = time.perf_counter() - _t0
        remaining = self.cfg.target_interval_s - elapsed
        if remaining > 0.0:
            time.sleep(remaining)

        return result

    def detect_marker_transform(self) -> Optional[np.ndarray]:
        """
        Photoneo 내장 마커 인식으로 T_M^S (Marker→Sensor) 변환 행렬 반환.

        Translation 단위: mm.

        Returns
        -------
        T_M_S : (4,4) float64  — Marker frame → Sensor frame, mm 단위
                None            — 마커 인식 실패
        """
        if not self._initialized:
            raise RuntimeError("[PhoxiClient] Not initialized.")

        features = self._features
        ia = self._ia
        T_S_M: Optional[np.ndarray] = None

        ia.stop()
        try:
            features.RecognizeMarkers.value = True
            features.CoordinateSpace.value = "MarkerSpace"
            features.ChunkModeActive.value = True
            features.ChunkSelector.value = "CurrentCameraToCoordinateSpaceTransformation"
            features.ChunkEnable.value = True
            _data_stream_reset(ia)
            ia.start()

            while True:
                try:
                    with ia.fetch(timeout=0.05):
                        pass
                except Exception:
                    break

            features.TriggerSoftware.execute()
            try:
                with ia.fetch(timeout=self.cfg.trigger_timeout_s) as _buf:
                    T_S_M = self._read_transform_chunk(
                        features, "CurrentCameraToCoordinateSpaceTransformation"
                    )
                    comp_names = _enabled_components(features)
                    components = dict(zip(comp_names, _buf.payload.components))
                    if "Range" in components:
                        rng = components["Range"]
                        h, w = rng.height, rng.width
                        self._last_organized_pts = (
                            rng.data.reshape(-1, 3).copy()
                            .astype(np.float32)
                            .reshape(h, w, 3)
                        )
                        if "Intensity" in components:
                            tc = components["Intensity"]
                            self._last_intensity = self._decode_intensity(
                                tc.data.copy(), tc.data_format, h, w
                            )
            except Exception as e:
                print(f"[PhoxiClient] 마커 인식 실패: {e}")
                T_S_M = None
        finally:
            ia.stop()
            try:
                features.RecognizeMarkers.value = False
                features.CoordinateSpace.value = "CalibratedABC_Grid"
                features.ChunkModeActive.value = False
            except Exception:
                pass
            _data_stream_reset(ia)
            ia.start()

        if T_S_M is None:
            return None

        T_M_S = np.linalg.inv(T_S_M)
        t_norm = np.linalg.norm(T_M_S[:3, 3])
        print(f"[PhoxiClient] 마커 인식 성공  t_norm={t_norm:.1f}mm")
        return T_M_S

    # ------------------------------------------------------------------
    # Intrinsic (NBV / TSDF 용)
    # ------------------------------------------------------------------

    def get_intrinsic(
        self,
        force_refit: bool = False,
    ) -> "o3d.camera.PinholeCameraIntrinsic":
        """
        O3D PinholeCameraIntrinsic 을 반환. 세션 동안 1회 피팅 후 캐시.

        방법
        ----
        `CalibratedABC_Grid` 모드의 organized range component 는 pixel (u,v) 에서의
        3D 점 (X, Y, Z) 를 제공한다. pinhole 가정:
            u = fx · X/Z + cx
            v = fy · Y/Z + cy
        유효 pixel 로부터 최소제곱으로 (fx, cx) / (fy, cy) 를 푼다.

        최소 한 번 `capture()` 가 실행돼 `_last_organized_pts` 가 채워져 있어야 한다.

        Parameters
        ----------
        force_refit : bool
            True 면 캐시 무시하고 재피팅.

        Returns
        -------
        o3d.camera.PinholeCameraIntrinsic
        """
        if not _HAS_O3D:
            raise RuntimeError("open3d 가 설치돼 있지 않습니다.")
        if not force_refit and self._cached_intrinsic is not None:
            return self._cached_intrinsic

        pts = self._last_organized_pts
        if pts is None:
            raise RuntimeError(
                "[PhoxiClient.get_intrinsic] organized points 가 없습니다. "
                "먼저 capture() 를 한 번 호출하세요."
            )
        H, W, _ = pts.shape
        X = pts[:, :, 0]
        Y = pts[:, :, 1]
        Z = pts[:, :, 2]
        u_grid, v_grid = np.meshgrid(np.arange(W), np.arange(H))

        valid = (Z > 1e-3) & np.isfinite(X) & np.isfinite(Y) & np.isfinite(Z)
        if valid.sum() < 100:
            raise RuntimeError(
                f"[PhoxiClient.get_intrinsic] 유효 픽셀 부족 ({valid.sum()}); "
                "장면이 너무 비었거나 depth 스케일 문제."
            )

        xz = (X[valid] / Z[valid]).astype(np.float64)
        yz = (Y[valid] / Z[valid]).astype(np.float64)
        u = u_grid[valid].astype(np.float64)
        v = v_grid[valid].astype(np.float64)

        # u = fx · xz + cx  —  least squares
        A_u = np.column_stack([xz, np.ones_like(xz)])
        fx, cx = np.linalg.lstsq(A_u, u, rcond=None)[0]
        A_v = np.column_stack([yz, np.ones_like(yz)])
        fy, cy = np.linalg.lstsq(A_v, v, rcond=None)[0]

        intr = o3d.camera.PinholeCameraIntrinsic(
            width=int(W), height=int(H),
            fx=float(fx), fy=float(fy),
            cx=float(cx), cy=float(cy),
        )
        self._cached_intrinsic = intr
        print(
            f"[PhoxiClient.get_intrinsic] fit {W}×{H}  "
            f"fx={fx:.1f} fy={fy:.1f} cx={cx:.1f} cy={cy:.1f}  "
            f"(valid pixels {int(valid.sum()):,})"
        )
        return intr

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------

    @staticmethod
    def _decode_intensity(raw: np.ndarray, fmt: Optional[str], h: int, w: int) -> np.ndarray:
        if fmt == "RGB8":
            return raw.astype(np.uint8).reshape(h, w, 3).mean(axis=2).astype(np.uint8)
        elif fmt == "Mono10":
            return (raw.astype(np.float32) / 1024.0 * 255).clip(0, 255).astype(np.uint8).reshape(h, w)
        elif fmt == "Mono12":
            return (raw.astype(np.float32) / 4096.0 * 255).clip(0, 255).astype(np.uint8).reshape(h, w)
        else:
            _max = float(raw.max()) or 1.0
            return (raw.astype(np.float32) / _max * 255).clip(0, 255).astype(np.uint8).reshape(h, w)

    @staticmethod
    def _read_transform_chunk(features: NodeMap, chunk_name: str) -> np.ndarray:
        selector_alias = f"Chunk{chunk_name}Selector"
        value_alias    = f"Chunk{chunk_name}Value"
        order = [
            "Rot00", "Rot01", "Rot02", "TransX",
            "Rot10", "Rot11", "Rot12", "TransY",
            "Rot20", "Rot21", "Rot22", "TransZ",
        ]
        chunk_data = {}
        for key in order:
            features.get_node(selector_alias).value = key
            chunk_data[key] = features.get_node(value_alias).value
        flat = [chunk_data[k] for k in order]
        mat_3x4 = np.array(flat, dtype=np.float64).reshape(3, 4)
        return np.vstack([mat_3x4, [0.0, 0.0, 0.0, 1.0]])


# ------------------------------------------------------------------
# 테스트
# ------------------------------------------------------------------
if __name__ == "__main__":
    if __package__ is None:
        sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

    import open3d as o3d

    cfg = PhoxiConfig(
        serial_number="SEA-023",
        trigger_timeout_s=15.0,
    )
    client = PhoxiClient(cfg)

    try:
        print("=== initialize ===")
        client.initialize()

        print("\n=== capture ===")
        result = client.capture(frame_id=0)

        if result is not None:
            print(f"  pts={len(result.points):,}  normals={result.normals is not None}  img={result.img is not None}")
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(result.points.astype(np.float64))
            o3d.visualization.draw_geometries([pcd], window_name="PhoxiClient")
        else:
            print("  capture 실패")

    finally:
        client.shutdown()
