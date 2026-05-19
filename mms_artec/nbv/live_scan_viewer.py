# mms_artec/nbv/live_scan_viewer.py
#
# 라이브 스캔 뷰어 **컨트롤러** (파이프라인 측). Open3D 를 전혀 만지지 않는다.
#
# 아키텍처 (검증된 유일 구성)
# ---------------------------
# 이 환경에서 라이브 포인트클라우드가 실제로 렌더된 유일한 형태는
# "사용자가 터미널에서 직접 띄운 Filament(O3DVisualizer) 프로세스" 였다
# (scripts/artec/live_scan_view.py / live_scan_viz_test.py). Popen 으로
# 자식 프로세스로 띄우면 GUI 가 반짝 떴다 죽는다 (legacy Visualizer 는
# 동적 PointCloud 자체를 못 그림 — [[project_open3d_must_use_filament_o3dvisualizer]]).
#
# 따라서 분리:
#   - 파이프라인(이 클래스): 매 OK 프레임을 SDK 정합행렬로 scan-world 누적,
#     throttle 마다 snapshot .npy 를 atomic write. Open3D / 프로세스 spawn 없음.
#   - 뷰어: 사용자가 **다른 터미널에서 직접** `python scripts/artec/
#     live_scan_view.py` 실행 → 그 스냅샷을 tail 하며 Filament 로 렌더.
#
# 누적 좌표
#   x_mm(sensor) ──T(=FrameEvent.transformation, SDK 정합)──▶ scan-world
#   ──/1000──▶ m.  θ / turntable_frame.yaml / hand-eye 의존 없음.
#
# 견고성: 스캔을 절대 느리게/깨뜨리지 않는다. 모든 공개 메서드는 예외를
# 삼키고, 한 번 죽으면 _dead=True 로 영구 no-op.
#
# Notation: T_AB : A → B (x_B = T_AB @ x_A) — CLAUDE.md 준수.

from __future__ import annotations

import os
import time
from datetime import datetime
from pathlib import Path
from typing import Optional

import numpy as np

# 사용자 뷰어와 공유하는 고정 경로 (pid 무관 — 뷰어가 인자 없이 찾도록).
_OUT = Path(__file__).resolve().parents[2] / "output"
SNAP_PATH = str(_OUT / "_live_latest.npy")
FLAG_PATH = str(_OUT / "_live_latest.flag")
STOP_PATH = str(_OUT / "_live_latest.stop")


class LiveScanViewer:
    """
    누적 컬러 포인트클라우드 스냅샷 writer (scan-world, meter).

    Parameters
    ----------
    voxel_mm : float
        누적 voxel downsample (mm). 더 세밀: 0.4 / 가벼움: 1.0.
    max_points : int
        snapshot 최대 점 수. 초과분 무작위 subsample (IPC 부하 제한).
    throttle_s : float
        snapshot write 최소 간격.
    title : str
        (호환용; 뷰어 창 제목은 뷰어 스크립트가 정함)
    """

    def __init__(
        self,
        voxel_mm: float = 0.6,
        max_points: int = 1_500_000,
        throttle_s: float = 0.5,
        title: str = "Artec Live Scan (Filament)",
    ) -> None:
        self._dead = False
        self.voxel_m = float(voxel_mm) / 1000.0
        self.max_points = int(max_points)
        self.throttle_s = float(throttle_s)
        self.title = title

        self._pts: list = []
        self._cols: list = []
        self._n_buffered = 0
        self._last_tick = 0.0
        self._dirty = False
        self._pass_label = ""
        self._frames_ingested = 0
        self._bbox_str = ""
        self._reject = ""

        try:
            _OUT.mkdir(parents=True, exist_ok=True)
            # 이전 run 잔여물 정리 — 특히 STOP 가 남아있으면 뷰어가 즉시
            # 자살한다. SNAP/FLAG 도 지워 새 run 이 옛 데이터로 시작 안 함.
            for p in (STOP_PATH, SNAP_PATH, FLAG_PATH, SNAP_PATH + ".tmp"):
                try:
                    os.remove(p)
                except OSError:
                    pass
            print("  [live] 스냅샷 writer 활성 (Open3D 미사용)")
            print("  [live] ★ 다른 터미널에서 뷰어를 실행하세요:")
            print("  [live]     conda activate mms-env")
            print("  [live]     python scripts/artec/live_scan_view.py")
            print(f"  [live]   (snapshot: {SNAP_PATH})")
        except Exception as e:
            print(f"  [live] ⚠ writer 비활성 ({type(e).__name__}: {e})")
            self._dead = True

    # ── pass 경계 ──────────────────────────────────────────────────────

    def new_pass(self, label: str = "") -> None:
        if self._dead:
            return
        self._pass_label = label
        if label:
            print(f"  [live] ── {label} ──")

    # ── 프레임 ingest ──────────────────────────────────────────────────

    def add_frame(self, frame_mesh, transformation) -> None:
        if self._dead or transformation is None or frame_mesh is None:
            if not self._reject:
                self._reject = (
                    "dead" if self._dead else
                    "transformation=None" if transformation is None else
                    "frame_mesh=None")
            return
        try:
            v_mm = frame_mesh.vertices()
            if v_mm is None or v_mm.shape[0] == 0:
                if not self._reject:
                    self._reject = "verts=empty"
                return
            v_mm = v_mm.astype(np.float64)                 # mm, sensor frame
            T = np.asarray(transformation, dtype=np.float64).reshape(4, 4)
            x_w_mm = v_mm @ T[:3, :3].T + T[:3, 3]
            x_W = (x_w_mm / 1000.0).astype(np.float32)     # m, scan-world
            col = self._sample_colors(frame_mesh, v_mm.shape[0])
            xq, cq = self._voxel_chunk(x_W, col.astype(np.float32))
            if xq.shape[0] == 0:
                if not self._reject:
                    self._reject = "voxel=empty"
                return
            self._pts.append(xq)
            self._cols.append(cq)
            self._n_buffered += xq.shape[0]
            self._frames_ingested += 1
            self._dirty = True
        except Exception as e:
            if not self._reject:
                self._reject = f"exc:{type(e).__name__}:{e}"
            return

    def _sample_colors(self, frame_mesh, n: int) -> np.ndarray:
        try:
            uv = frame_mesh.uv()
            img = frame_mesh.image()
            if uv is None or img is None or uv.shape[0] != n:
                raise ValueError
            H, W = img.shape[0], img.shape[1]
            u = np.clip(uv[:, 0], 0.0, 1.0)
            vv = np.clip(1.0 - uv[:, 1], 0.0, 1.0)
            px = np.clip((u * (W - 1)).astype(np.int32), 0, W - 1)
            py = np.clip((vv * (H - 1)).astype(np.int32), 0, H - 1)
            return img[py, px, :3].astype(np.float32) / 255.0
        except Exception:
            return np.full((n, 3), 0.6, dtype=np.float32)

    def _voxel_chunk(self, pts: np.ndarray, cols: np.ndarray):
        if pts.shape[0] == 0 or self.voxel_m <= 0:
            return pts, cols
        keys = np.floor(pts / self.voxel_m).astype(np.int64)
        _, idx = np.unique(keys, axis=0, return_index=True)
        return pts[idx], cols[idx]

    # ── snapshot write (Open3D 없음) ───────────────────────────────────

    def tick(self, scanning_flag: bool = True) -> bool:
        if self._dead:
            return False
        try:
            with open(FLAG_PATH, "w") as f:
                f.write("1" if scanning_flag else "0")
        except Exception:
            pass

        now = time.time()
        if (now - self._last_tick) < self.throttle_s:
            return True
        self._last_tick = now

        try:
            if self._dirty and self._n_buffered > 0:
                pts = np.vstack(self._pts)
                cols = np.vstack(self._cols)
                pts, cols = self._voxel_chunk(pts, cols)
                if pts.shape[0] > self.max_points:
                    sel = np.random.default_rng(0).choice(
                        pts.shape[0], self.max_points, replace=False)
                    pts, cols = pts[sel], cols[sel]
                self._pts = [pts]
                self._cols = [cols]
                self._n_buffered = pts.shape[0]

                mn, mx = pts.min(axis=0), pts.max(axis=0)
                ctr = (mn + mx) / 2.0
                cmu = float(cols.mean()) if cols.size else 0.0
                self._bbox_str = (
                    f"n={pts.shape[0]} "
                    f"c=({ctr[0]:+.2f},{ctr[1]:+.2f},{ctr[2]:+.2f})"
                    f" ext=({mx[0]-mn[0]:.2f},{mx[1]-mn[1]:.2f},"
                    f"{mx[2]-mn[2]:.2f})m colμ={cmu:.2f}")

                arr = np.concatenate(
                    [pts.astype(np.float32), cols.astype(np.float32)], axis=1)
                # np.save(path) 는 .npy 자동첨부 → 파일객체로 경로 고정 후
                # atomic replace.
                tmp = SNAP_PATH + ".tmp"
                with open(tmp, "wb") as f:
                    np.save(f, arr)
                os.replace(tmp, SNAP_PATH)
                self._dirty = False
        except Exception as e:
            print(f"  [live] ⚠ snapshot write 예외 "
                  f"({type(e).__name__}: {e}) — writer 종료")
            self._dead = True
            return False
        return True

    # ── 종료 ───────────────────────────────────────────────────────────

    def close(self) -> None:
        # 뷰어에 종료 신호
        try:
            with open(STOP_PATH, "w") as f:
                f.write("stop")
        except Exception:
            pass
        # 최종 누적 클라우드 PLY (Open3D 없이 binary PLY 직접 작성)
        try:
            if self._pts and self._n_buffered > 0:
                pts = np.vstack(self._pts).astype(np.float32)
                cols = np.clip(np.vstack(self._cols), 0.0, 1.0)
                rgb = (cols * 255.0).astype(np.uint8)
                ts = datetime.now().strftime("%Y%m%d_%H%M%S")
                ply = str(_OUT / f"live_cloud_{ts}.ply")
                self._write_ply(ply, pts, rgb)
                print(f"  [live] 누적 클라우드 PLY → {ply}\n"
                      f"  [live]   AABB {self._bbox_str} "
                      f"(CloudCompare/MeshLab 로 열기)")
        except Exception as e:
            print(f"  [live] ⚠ PLY 덤프 실패 ({type(e).__name__}: {e})")
        finally:
            self._dead = True

    @staticmethod
    def _write_ply(path: str, pts: np.ndarray, rgb: np.ndarray) -> None:
        n = pts.shape[0]
        hdr = ("ply\nformat binary_little_endian 1.0\n"
               f"element vertex {n}\n"
               "property float x\nproperty float y\nproperty float z\n"
               "property uchar red\nproperty uchar green\n"
               "property uchar blue\nend_header\n").encode("ascii")
        rec = np.zeros(n, dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"),
                                 ("r", "u1"), ("g", "u1"), ("b", "u1")])
        rec["x"], rec["y"], rec["z"] = pts[:, 0], pts[:, 1], pts[:, 2]
        rec["r"], rec["g"], rec["b"] = rgb[:, 0], rgb[:, 1], rgb[:, 2]
        with open(path, "wb") as f:
            f.write(hdr)
            f.write(rec.tobytes())
