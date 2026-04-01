# mms/sensor/phoxi_client.py
#
# Photoneo PhoXi3D 클라이언트.
# Harvesters + mvGenTLProducer.cti (GigE-V / GenTL) 기반.
# 공식 예제 참조: photoneo-python-examples/GigE-V/harvesters/

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

from mms.core.frames import Frame
from mms.core.transforms import load_transform
from mms.sensor.phoxi_instant_meshing import PhoxiInstantMeshingWrapper


# ------------------------------------------------------------------
# CTI 파일 탐색
# ------------------------------------------------------------------

def _find_cti() -> Path:
    """
    mvGenTLProducer.cti 경로 탐색 (GENICAM_GENTL64_PATH 기반).

    PhoXi3D는 motionCam과 동일하게 Matrix Vision GenTL Producer를 사용.

    환경변수 설정 예 (PowerShell)
    ------------------------------
    $env:GENICAM_GENTL64_PATH = "C:\\path\\to\\mvIMPACT_Acquire\\lib"
    """
    env_val = os.getenv("GENICAM_GENTL64_PATH")
    if env_val is None:
        raise EnvironmentError(
            "[PhoxiClient] GENICAM_GENTL64_PATH 환경변수가 설정되지 않았습니다.\n"
            "  Matrix Vision mvIMPACT Acquire 또는 PhoXi Control 설치 후\n"
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
# Harvesters 내부 유틸 (공식 예제 photoneo_genicam/utils.py 참조)
# ------------------------------------------------------------------

def _data_stream_reset(ia: ImageAcquirer) -> None:
    """
    start()/stop()을 반복 호출할 때 필요한 스트림 채널 리셋.
    Harvesters 내부 함수 사용 (API 변경 가능성 있음).
    """
    ia._release_data_streams()
    ia._setup_data_streams(file_dict=ia._file_dict)


# ------------------------------------------------------------------
# 컴포넌트 유틸 (공식 예제 photoneo_genicam/components.py 참조)
# ------------------------------------------------------------------

def _sorted_components(features: NodeMap) -> List[str]:
    """ComponentIDValue 오름차순으로 정렬된 컴포넌트 이름 목록."""
    def id_value(comp: str) -> int:
        features.ComponentSelector.value = comp
        return features.ComponentIDValue.value

    return sorted(features.ComponentSelector.symbolics, key=id_value)


def _enable_components(features: NodeMap, component_list: List[str]) -> None:
    """지정한 컴포넌트만 활성화하고 나머지는 비활성화."""
    for comp in features.ComponentSelector.symbolics:
        features.ComponentSelector.value = comp
        features.ComponentEnable.value = (comp in component_list)


def _enabled_components(features: NodeMap) -> List[str]:
    """
    현재 활성화된 컴포넌트를 ComponentIDValue 순서대로 반환.
    multipart payload의 component 순서와 일치함.
    """
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
    PhoXi3D client configuration.

    sensor_frames_yaml
        config/sensor_frames.yaml 경로.
    T_E_S_key
        sensor_frames.yaml에서 EE→Sensor transform 키. 예: 'T_E_S_phoxi'
    serial_number
        디바이스 시리얼 번호. None → 첫 번째 발견된 디바이스.
    trigger_timeout_s
        ia.fetch() 대기 최대 시간(초). 구조광 스캔은 노출 시간이 길므로 여유있게 설정.
    target_interval_s
        캡처 호출당 목표 벽시계 시간(초).
        MMS.capture_frames()가 max(0, target - elapsed) 만큼 sleep.
        0.0 이면 최대 속도로 캡처.
    """

    sensor_frames_yaml: str
    T_E_S_key: str
    serial_number: Optional[str] = None
    trigger_timeout_s: float = 15.0
    target_interval_s: float = 1.0

    def __post_init__(self) -> None:
        self.sensor_frames_yaml = str(Path(self.sensor_frames_yaml).resolve())


# ------------------------------------------------------------------
# PhoxiClient
# ------------------------------------------------------------------

class PhoxiClient:
    """
    Photoneo PhoXi3D 클라이언트. Harvesters + mvGenTLProducer.cti (GenTL/GigE-V) 기반.

    Notation
    --------
    T_A^B maps frame A to frame B:  x_B = T_A^B @ x_A

    사전 조건
    ---------
    - GENICAM_GENTL64_PATH 환경변수에 mvGenTLProducer.cti 위치 등록.
    - PhoXi Control 소프트웨어가 실행 중이어야 함.
    """

    def __init__(self, cfg: PhoxiConfig, mesher: Optional[PhoxiInstantMeshingWrapper] = None) -> None:
        self.cfg = cfg
        self.T_E_S: np.ndarray = load_transform(cfg.sensor_frames_yaml, cfg.T_E_S_key)
        self._mesher = mesher

        self._harvester: Optional[Harvester] = None
        self._ia: Optional[ImageAcquirer] = None
        self._features: Optional[NodeMap] = None
        self._initialized: bool = False

        # 최근 캡처 데이터 (hand-eye 캘리브레이션 등 외부 접근용)
        self._last_intensity: Optional[np.ndarray] = None        # (H, W) uint8
        self._last_organized_pts: Optional[np.ndarray] = None    # (H, W, 3) float32, mm, sensor frame

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def initialize(self) -> None:
        """
        GenTL Producer 로드 → 디바이스 탐색 → 연결 → 설정 → 취득 시작.

        설정 내용
        ---------
        - UserSet Default 로드
        - Software trigger 설정
        - Scan3dOutputMode = CalibratedABC_Grid  (보정된 XYZ 포인트 클라우드)
        - 활성 컴포넌트: Intensity, Range, Normal

        Raises
        ------
        EnvironmentError / FileNotFoundError
            GENICAM_GENTL64_PATH 미설정 또는 CTI 파일 없을 때.
        RuntimeError
            디바이스 없거나 연결 실패 시.
        """
        cti_path = _find_cti()
        print(f"[PhoxiClient] Using CTI: {cti_path}")

        h = Harvester()
        h.add_file(str(cti_path), check_existence=True, check_validity=True)
        h.update()

        devices = h.device_info_list
        if not devices:
            h.reset()
            raise RuntimeError(
                "[PhoxiClient] No devices found.\n"
                "  GigE-V 직접 연결 방식은 PhoXi Control이 디바이스를 점유하면 탐색 불가.\n"
                "  → PhoXi Control UI에서 디바이스를 'Disconnect' 후 재시도하세요.\n"
                "  → 컴퓨터 NIC가 카메라와 같은 서브넷(192.168.10.x)인지 확인하세요."
            )

        print(f"[PhoxiClient] Found {len(devices)} device(s):")
        for i, dev in enumerate(devices):
            props = dev.property_dict
            print(f"  [{i}] serial={props.get('serial_number')}  "
                  f"model={props.get('model')}")

        search_key = (
            {"serial_number": self.cfg.serial_number}
            if self.cfg.serial_number else 0
        )
        ia = h.create(search_key)
        features: NodeMap = ia.remote_device.node_map

        fw_ver = features.DeviceFirmwareVersion.value
        print(f"[PhoxiClient] Connected. Firmware: {fw_ver}")

        # UserSet Default 로드
        features.UserSetSelector.value = "Default"
        features.UserSetLoad.execute()

        # Software trigger
        features.TriggerSelector.value = "FrameStart"
        features.TriggerMode.value = "On"
        features.TriggerSource.value = "Software"

        # 보정된 XYZ 포인트 클라우드 모드
        features.Scan3dOutputMode.value = "CalibratedABC_Grid"

        # 활성 컴포넌트: Intensity (텍스처), Range (포인트클라우드), Normal (법선)
        _enable_components(features, ["Intensity", "Range", "Normal"])

        ia.start()

        self._harvester = h
        self._ia = ia
        self._features = features
        self._comp_names = _enabled_components(features)   # 초기화 시 1회 캐시
        self._initialized = True
        print(f"[PhoxiClient] Acquisition started.  components={self._comp_names}")

    def shutdown(self) -> None:
        """취득 중지 및 리소스 해제."""
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

    def capture_frame(
        self,
        ee_pose_mat_B: np.ndarray,
        frame_id: int,
        timestamp: float,
        get_mesh: bool = False,
    ) -> Optional[Frame]:
        """
        소프트웨어 트리거로 1회 스캔하고 base (B) 프레임의 Frame을 반환.

        Parameters
        ----------
        ee_pose_mat_B : (4,4) np.ndarray
            T_E^B — 캡처 시점의 EE→Base 변환 (xArm FK 결과).
        frame_id : int
        timestamp : float

        Returns
        -------
        Frame or None
        """
        if not self._initialized:
            raise RuntimeError("[PhoxiClient] Not initialized. Call initialize() first.")

        _t0 = time.perf_counter()

        # stale 버퍼 소진
        try:
            from _gentl import TimeoutException as _GentlTimeout
        except ImportError:
            _GentlTimeout = Exception

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

        # 최대 2회 재시도: 1회 실패 시 stop→start 후 재시도
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
                        self._ia.start()
                        time.sleep(0.5)
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
                print("[PhoxiClient] Range component not found in payload.")
                return None

            # Range (포인트 클라우드): Coord3D_ABC32f, (H*W*3,) float32, mm 단위
            range_comp = components["Range"]
            points_S_flat = range_comp.data.reshape(-1, 3).copy().astype(np.float32)

            # Normal (법선): Coord3D_ABC32f, (H*W*3,) float32
            normal_S_flat: Optional[np.ndarray] = None
            if "Normal" in components:
                nm_comp = components["Normal"]
                normal_S_flat = nm_comp.data.reshape(-1, 3).copy().astype(np.float32)

            # Intensity (텍스처): Mono10/12/16 또는 RGB8 — 버퍼 안에서 즉시 복사
            h_tex = range_comp.height
            w_tex = range_comp.width
            tex_raw: Optional[np.ndarray] = None
            tex_fmt: Optional[str] = None
            if "Intensity" in components:
                _tc = components["Intensity"]
                tex_fmt = _tc.data_format
                tex_raw = _tc.data.copy()   # 버퍼 반환 전 복사

        _t2 = time.perf_counter()

        # 강도 이미지 → uint8 그레이스케일 (hand-eye 캘리브레이션 / _build_frame 공용)
        if tex_raw is not None:
            if tex_fmt == "RGB8":
                _gray = tex_raw.astype(np.uint8).reshape(h_tex, w_tex, 3).mean(axis=2).astype(np.uint8)
                tex_flat_f32 = tex_raw.astype(np.float32).reshape(-1)
            elif tex_fmt == "Mono10":
                _gray = (tex_raw.astype(np.float32) / 1024.0 * 255).clip(0, 255).astype(np.uint8).reshape(h_tex, w_tex)
                tex_flat_f32 = tex_raw.astype(np.float32).reshape(-1)
            elif tex_fmt == "Mono12":
                _gray = (tex_raw.astype(np.float32) / 4096.0 * 255).clip(0, 255).astype(np.uint8).reshape(h_tex, w_tex)
                tex_flat_f32 = tex_raw.astype(np.float32).reshape(-1)
            else:
                _max = float(tex_raw.max()) or 1.0
                _gray = (tex_raw.astype(np.float32) / _max * 255).clip(0, 255).astype(np.uint8).reshape(h_tex, w_tex)
                tex_flat_f32 = tex_raw.astype(np.float32).reshape(-1)
            self._last_intensity = _gray
        else:
            _gray = None
            tex_flat_f32 = np.zeros(h_tex * w_tex, dtype=np.float32)
            self._last_intensity = None

        self._last_organized_pts = points_S_flat.reshape(h_tex, w_tex, 3)

        # Mesher에 원시 센서 데이터 공급 (S 프레임, mm 단위)
        if self._mesher is not None:
            T_S_B = ee_pose_mat_B @ np.linalg.inv(self.T_E_S)
            self._mesher.add_scan(points_S_flat, tex_flat_f32, T_S_B, timestamp,
                                  width=w_tex, height=h_tex)

        frame = self._build_frame(
            points_S_flat=points_S_flat,
            normal_S_flat=normal_S_flat,
            img_gray=_gray,
            h=h_tex,
            w=w_tex,
            ee_pose_mat_B=ee_pose_mat_B,
            T_E_S=self.T_E_S,
            frame_id=frame_id,
            timestamp=timestamp,
        )

        # get_mesh=True 이면 누적된 TSDF 메쉬를 frame에 저장
        if get_mesh and self._mesher is not None:
            frame.mesh = self._mesher.get_mesh()

        _t3 = time.perf_counter()

        print(
            f"  [timing] trigger+fetch={_t1-_t0:.3f}s  decode={_t2-_t1:.3f}s  "
            f"build_frame={_t3-_t2:.3f}s  total={_t3-_t0:.3f}s  pts={len(frame.points):,}"
        )
        return frame

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------

    @staticmethod
    def _build_frame(
        points_S_flat: np.ndarray,
        normal_S_flat: Optional[np.ndarray],
        img_gray: Optional[np.ndarray],
        h: int,
        w: int,
        ee_pose_mat_B: np.ndarray,
        T_E_S: np.ndarray,
        frame_id: int,
        timestamp: float,
    ) -> Frame:
        """
        PhoXi 센서 프레임 데이터를 base (B) 프레임의 Frame으로 변환.

        좌표 변환
        ---------
        T_S^B = T_E^B @ (T_E^S)^{-1}
        x_B   = T_S^B @ x_S

        Parameters
        ----------
        points_S_flat : (H*W, 3) float32, mm 단위. 무효점 = all-zero.
        normal_S_flat : (H*W, 3) float32 or None.
        img_gray      : (H, W) uint8 or None — 이미 디코딩된 강도 이미지.
        h, w          : 이미지 높이/너비.
        ee_pose_mat_B : (4,4) T_E^B
        T_E_S         : (4,4) T_E^S
        """
        # T_S^B = T_E^B @ inv(T_E^S)
        T_S_B = ee_pose_mat_B @ np.linalg.inv(T_E_S)
        R_mat = T_S_B[:3, :3].astype(np.float32)
        t_vec = T_S_B[:3, 3].astype(np.float32)

        # 유효점 마스크: all-zero 아닌 점
        valid = ~np.all(points_S_flat == 0.0, axis=1)
        pts_m = points_S_flat[valid] / 1000.0          # mm → m, (N, 3)

        # x_B = R @ x_S + t
        points_B = pts_m @ R_mat.T + t_vec              # (N, 3) float32

        # NormalMap: 회전만 적용 (방향 벡터)
        normals_B: Optional[np.ndarray] = None
        if normal_S_flat is not None:
            norms_valid = normal_S_flat[valid]
            normals_B = norms_valid @ R_mat.T
            nlen = np.linalg.norm(normals_B, axis=1, keepdims=True)
            nlen = np.where(nlen < 1e-6, 1.0, nlen)
            normals_B = (normals_B / nlen).astype(np.float32)

        # 그레이스케일 → pseudo-RGB (H, W, 3) uint8
        raw_img: Optional[np.ndarray] = None
        if img_gray is not None:
            raw_img = np.stack([img_gray, img_gray, img_gray], axis=-1)

        return Frame(
            sensor_type="phoxi",
            img=raw_img,
            depth=None,         # CalibratedABC_Grid 모드에서는 DepthMap 미사용
            points=points_B,
            normals=normals_B,
            colors=None,        # PhoXi3D는 color 없음
            frame_id=frame_id,
            timestamp=timestamp,
            ee_pose_mat_B=ee_pose_mat_B.astype(np.float64),
        )


# ------------------------------------------------------------------
# 기본 기능 테스트
# ------------------------------------------------------------------
if __name__ == "__main__":
    _PROJECT_ROOT = Path(__file__).resolve().parents[2]
    sys.path.insert(0, str(_PROJECT_ROOT))

    import open3d as o3d

    cfg = PhoxiConfig(
        sensor_frames_yaml=str(_PROJECT_ROOT / "config" / "sensor_frames.yaml"),
        T_E_S_key="T_E_S_phoxi",
        serial_number="SEA-023",  # PhoXi Control에서 확인한 ID
        trigger_timeout_s=15.0,
    )

    mesher = PhoxiInstantMeshingWrapper(
        voxel_size_mm       = 1.5,
        min_voxel_consensus = 1,    # 단일 스캔에서도 메쉬 생성
        tracking_enabled    = False,
        # 해상도는 첫 capture_frame 시 자동 감지
    )
    client = PhoxiClient(cfg, mesher=mesher)
    dummy_ee = np.eye(4, dtype=np.float64)

    try:
        print("=== [TEST 1] initialize ===")
        client.initialize()

        N_FRAMES = 10
        print(f"\n=== [TEST 2] {N_FRAMES}회 capture_frame 타이밍 ===")
        frames = []
        for i in range(N_FRAMES):
            is_last = (i == N_FRAMES - 1)
            t0 = time.perf_counter()
            frame = client.capture_frame(
                ee_pose_mat_B=dummy_ee,
                frame_id=i,
                timestamp=t0,
                get_mesh=is_last,   # 마지막 프레임에만 get_mesh
            )
            elapsed = time.perf_counter() - t0
            if frame is not None:
                frames.append(frame)
                mesh_info = (
                    f"  mesh={len(frame.mesh.vertices):,}v/{len(frame.mesh.triangles):,}t"
                    if (frame.mesh is not None and len(frame.mesh.vertices) > 0)
                    else ""
                )
                print(f"  [{i+1:2d}/{N_FRAMES}] {elapsed:.3f}s  pts={len(frame.points):,}{mesh_info}")
            else:
                print(f"  [{i+1:2d}/{N_FRAMES}] {elapsed:.3f}s  None")

        print(f"\n=== [TEST 3] open3d 시각화 ===")
        last = frames[-1] if frames else None
        if last is None:
            print("  캡처된 프레임 없음")
        else:
            geoms = []
            pcd = last.to_pcd()
            pcd.paint_uniform_color([0.6, 0.6, 0.6])
            geoms.append(pcd)

            if last.mesh is not None and len(last.mesh.vertices) > 0:
                print(f"  mesh vertices  : {len(last.mesh.vertices):,}")
                print(f"  mesh triangles : {len(last.mesh.triangles):,}")
                last.mesh.paint_uniform_color([0.2, 0.6, 1.0])
                geoms.append(last.mesh)
            else:
                print("  mesh 없음")

            o3d.visualization.draw_geometries(
                geoms,
                window_name="PhoxiClient — PointCloud & Mesh",
                width=1280,
                height=720,
            )

    except Exception as e:
        print(f"\n[ERROR] {e}", file=sys.stderr)
        raise

    finally:
        print("\n=== [TEST 4] shutdown ===")
        mesher.cleanup()
        client.shutdown()
