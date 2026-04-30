# mms/sensor/phoxi/phoxi_dataset_replay.py
#
# `scripts/phoxi_dataset_capture.py` 가 만든 데이터셋을 **PhoxiClient 와 같은 API** 로
# 재생한다. 센서 없이 기존 파이프라인 (ScanSession, PcdAccumulateVolume 등) 을
# 그대로 돌릴 수 있도록 한다.
#
# 역할
# ----
# - `initialize() / shutdown()` — 빈 구현 (호환)
# - `capture(frame_id, timestamp)` → ScanResult   (순차 재생)
# - `_last_organized_pts / _last_intensity / _last_depth_mm / _last_organized_normals`
# - `get_intrinsic() → o3d.camera.PinholeCameraIntrinsic`
# - `cfg.target_interval_s` (MMS.capture_frames 가 참조)
#
# 추가 API
# -------
# - `seek(idx)`           : 다음에 재생할 프레임 index 변경
# - `n_frames`            : 총 프레임 수
# - `get_pose(idx) → (θ_rad, T_EB)`  : 캡처 당시 턴테이블 각도 + 로봇 FK

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional

import numpy as np

try:
    import open3d as o3d
    _HAS_O3D = True
except ImportError:
    o3d = None
    _HAS_O3D = False

from utils.scan_result import ScanResult


@dataclass
class PhoxiDatasetConfig:
    dataset_dir: str
    target_interval_s: float = 0.0       # 재생 시 sleep 필요 없음 (기본 0)
    trigger_timeout_s: float = 15.0      # (호환용, 미사용)
    serial_number: Optional[str] = None  # (호환용, 미사용)


class PhoxiDatasetReplay:
    """
    PhoxiClient 호환 replay — 오프라인 데이터셋을 센서처럼 사용.
    """

    def __init__(self, cfg: PhoxiDatasetConfig) -> None:
        self.cfg = cfg
        self._root = Path(cfg.dataset_dir)
        if not self._root.exists():
            raise FileNotFoundError(f"dataset dir not found: {self._root}")

        meta_p = self._root / "session_meta.json"
        if not meta_p.exists():
            raise FileNotFoundError(f"session_meta.json not found under {self._root}")
        self._session_meta = json.loads(meta_p.read_text(encoding="utf-8"))

        self._frames_root = self._root / self._session_meta.get("frames_dir", "frames")
        self._frame_dirs: List[Path] = sorted(
            [p for p in self._frames_root.iterdir() if p.is_dir()],
            key=lambda p: p.name,
        )
        if not self._frame_dirs:
            raise RuntimeError(f"no frame_* directories under {self._frames_root}")

        self._cursor: int = 0

        # PhoxiClient 호환 버퍼
        self._last_intensity: Optional[np.ndarray] = None
        self._last_organized_pts: Optional[np.ndarray] = None
        self._last_depth_mm: Optional[np.ndarray] = None
        self._last_organized_normals: Optional[np.ndarray] = None

        self._cached_intrinsic: Optional["o3d.camera.PinholeCameraIntrinsic"] = None

    # ── lifecycle ──────────────────────────────────────────────────────

    def initialize(self) -> None:
        print(f"[PhoxiDatasetReplay] dataset = {self._root}")
        print(f"[PhoxiDatasetReplay] n_frames = {len(self._frame_dirs)}")
        im = self._session_meta.get("intrinsic")
        if im:
            print(f"[PhoxiDatasetReplay] intrinsic: {im['width']}×{im['height']}  "
                  f"fx={im['fx']:.1f} fy={im['fy']:.1f} cx={im['cx']:.1f} cy={im['cy']:.1f}")

    def shutdown(self) -> None:
        pass

    # ── 재생 제어 ──────────────────────────────────────────────────────

    @property
    def n_frames(self) -> int:
        return len(self._frame_dirs)

    def seek(self, idx: int) -> None:
        if not (0 <= idx < len(self._frame_dirs)):
            raise IndexError(f"idx {idx} out of [0, {len(self._frame_dirs)})")
        self._cursor = int(idx)

    def get_pose(self, idx: int) -> tuple:
        """
        해당 프레임의 (θ_rad, T_EB) 를 반환 — ScanSession 에서 T_CO 재계산할 때 사용.
        """
        meta = json.loads((self._frame_dirs[idx] / "meta.json").read_text(encoding="utf-8"))
        theta = float(meta["theta_actual_rad"])
        T_EB  = np.array(meta["T_EB"], dtype=np.float64)
        return theta, T_EB

    # ── capture (PhoxiClient 호환) ────────────────────────────────────

    def capture(
        self,
        frame_id: int = 0,
        timestamp: Optional[float] = None,
    ) -> Optional[ScanResult]:
        if self._cursor >= len(self._frame_dirs):
            print("[PhoxiDatasetReplay] 모든 프레임 소진 — None")
            return None
        fdir = self._frame_dirs[self._cursor]
        self._cursor += 1

        # Load arrays
        range_npy = fdir / "range.npy"
        if not range_npy.exists():
            print(f"[PhoxiDatasetReplay] range.npy missing in {fdir}")
            return None
        organized = np.load(range_npy).astype(np.float32)
        H, W, _ = organized.shape

        intensity = None
        if (fdir / "intensity.npy").exists():
            intensity = np.load(fdir / "intensity.npy").astype(np.uint8)

        normals_org = None
        if (fdir / "normals.npy").exists():
            normals_org = np.load(fdir / "normals.npy").astype(np.float32)

        depth_mm = organized[:, :, 2].astype(np.float32)
        depth_mm[depth_mm < 0] = 0.0

        # PhoxiClient 호환 필드 업데이트
        self._last_organized_pts = organized
        self._last_intensity = intensity
        self._last_depth_mm = depth_mm
        self._last_organized_normals = normals_org

        # ScanResult 구성 (points/normals 는 유효점만 필터링)
        flat = organized.reshape(-1, 3)
        valid = ~np.all(flat == 0.0, axis=1)
        points_valid = flat[valid]
        normals_valid = None
        if normals_org is not None:
            normals_valid = normals_org.reshape(-1, 3)[valid]

        img_rgb = None
        if intensity is not None:
            if intensity.ndim == 2:
                img_rgb = np.stack([intensity] * 3, axis=-1).astype(np.uint8)
            else:
                img_rgb = intensity.astype(np.uint8)

        return ScanResult(
            sensor_type="phoxi",
            points=points_valid.astype(np.float32),
            normals=normals_valid.astype(np.float32) if normals_valid is not None else None,
            img=img_rgb,
            depth=depth_mm,
            frame_id=frame_id,
            timestamp=timestamp if timestamp is not None else 0.0,
        )

    # ── intrinsic ──────────────────────────────────────────────────────

    def get_intrinsic(self, force_refit: bool = False):
        """
        session_meta 에 intrinsic 이 있으면 그걸로 구성. 없으면 organized pts 로 fit.
        """
        if not _HAS_O3D:
            raise RuntimeError("open3d 가 없습니다.")

        if not force_refit and self._cached_intrinsic is not None:
            return self._cached_intrinsic

        im = self._session_meta.get("intrinsic")
        if im is not None:
            intr = o3d.camera.PinholeCameraIntrinsic(
                width=int(im["width"]), height=int(im["height"]),
                fx=float(im["fx"]), fy=float(im["fy"]),
                cx=float(im["cx"]), cy=float(im["cy"]),
            )
            self._cached_intrinsic = intr
            return intr

        # fit from first frame's organized pts
        first = self._frame_dirs[0]
        organized = np.load(first / "range.npy")
        H, W, _ = organized.shape
        X = organized[:, :, 0]; Y = organized[:, :, 1]; Z = organized[:, :, 2]
        u_g, v_g = np.meshgrid(np.arange(W), np.arange(H))
        valid = (Z > 1e-3) & np.isfinite(X) & np.isfinite(Y) & np.isfinite(Z)
        xz = (X[valid] / Z[valid]).astype(np.float64)
        yz = (Y[valid] / Z[valid]).astype(np.float64)
        u = u_g[valid].astype(np.float64); v = v_g[valid].astype(np.float64)
        A_u = np.column_stack([xz, np.ones_like(xz)])
        fx, cx = np.linalg.lstsq(A_u, u, rcond=None)[0]
        A_v = np.column_stack([yz, np.ones_like(yz)])
        fy, cy = np.linalg.lstsq(A_v, v, rcond=None)[0]
        intr = o3d.camera.PinholeCameraIntrinsic(
            width=int(W), height=int(H),
            fx=float(fx), fy=float(fy), cx=float(cx), cy=float(cy),
        )
        self._cached_intrinsic = intr
        return intr

    @property
    def session_meta(self) -> dict:
        return dict(self._session_meta)
