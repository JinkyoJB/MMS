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
import threading
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

# Pass 별 색조 (annotation 만 — 정합 시각 검증용). multiplicative tint:
# 원본 텍스처를 유지하면서 어느 IScan 출신인지 구분. Pass 0 = 원본.
# [[feedback_live_viewer_must_mirror_scan]] — annotation 은 데이터 fakery 아님.
_PASS_TINT = np.array([
    [1.00, 1.00, 1.00],  # Pass 0: 원본
    [1.00, 0.55, 0.80],  # Pass 1: 핑크
    [0.55, 1.00, 0.80],  # Pass 2: 청록
    [1.00, 0.90, 0.45],  # Pass 3: 황금
    [0.65, 0.80, 1.00],  # Pass 4: 푸른
    [0.80, 1.00, 0.55],  # Pass 5: 라임
], dtype=np.float32)


def _tint_for(pass_idx: int) -> np.ndarray:
    """Pass index → tint factor (cycling palette)."""
    if pass_idx < 0:
        return _PASS_TINT[0]
    return _PASS_TINT[pass_idx % len(_PASS_TINT)]


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

        # ★ snapshot 쓰기는 **백그라운드 스레드**에서 한다. 예전에는 `tick()` 이
        #   스캔 폴링 루프에서 직접 vstack+voxel+저장을 했고, 버퍼가 커지면
        #   (rebuild 후 5.1M 점) 한 번에 2~3초를 잡아먹어 `poll_events()` 가 굶었다.
        #   그러면 콜백이 안 들어와 stall watchdog 이 "frame stall" 로 스캔을 죽인다
        #   (2026-09-22 run_182946: 첫 IScan 이후 모든 세션이 3~7초 만에 죽고
        #    윗면 밴드와 flip 이 통째로 날아갔다). 뷰어는 거울일 뿐 스캔을 막으면 안 된다.
        self._lock = threading.Lock()
        self._req = threading.Event()
        self._worker: Optional[threading.Thread] = None
        self._worker_stop = threading.Event()
        self._pts: list = []
        self._cols: list = []
        self._n_buffered = 0
        self._last_tick = 0.0
        self._dirty = False
        self._pass_label = ""
        self._pass_index = -1            # new_pass() 호출 시 0 부터 시작
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
        self._pass_index += 1
        self._pass_label = label
        tint = _tint_for(self._pass_index)
        if label:
            print(f"  [live] ── {label} ── tint=({tint[0]:.2f},"
                  f"{tint[1]:.2f},{tint[2]:.2f})")

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
            # Pass annotation tint — 어느 IScan 출신인지 시각 구분.
            col = col * _tint_for(self._pass_index)
            xq, cq = self._voxel_chunk(x_W, col.astype(np.float32))
            if xq.shape[0] == 0:
                if not self._reject:
                    self._reject = "voxel=empty"
                return
            with self._lock:
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

    # ── master_model 동기화 (pass cleanup 직후) ────────────────────────

    def rebuild_from_model(self, master_model) -> None:
        """
        Buffer 전체 비우고 master_model 의 모든 IScan/frame 으로 재구성.

        Pass 끝나고 SerialReg + OutlierRemoval 돌린 뒤 호출 — 그 결과가
        master_model 에 반영돼 있으니 viewer 가 그걸 그대로 비춰야
        viewer = scan 일관성 유지 ([[feedback_live_viewer_must_mirror_scan]]).

        Pass tint 는 master_model 안의 IScan 인덱스 = 가시 pass 순서로 적용.
        """
        if self._dead or master_model is None:
            return
        try:
            n_scans = master_model.scan_count()
            new_pts: list = []
            new_cols: list = []
            n_total = 0
            for s_i in range(n_scans):
                scan = master_model.get_scan(s_i)
                n_frames = scan.frame_count()
                tint = _tint_for(s_i)
                for f_i in range(n_frames):
                    try:
                        frame = scan.get_frame(int(f_i))
                        v_mm = frame.vertices()
                        if v_mm is None or v_mm.shape[0] == 0:
                            continue
                        v_mm = v_mm.astype(np.float64)
                        T = scan.get_frame_transformation(int(f_i))
                        x_w_mm = v_mm @ T[:3, :3].T + T[:3, 3]
                        x_W = (x_w_mm / 1000.0).astype(np.float32)
                        col = self._sample_colors(frame, v_mm.shape[0]) * tint
                        xq, cq = self._voxel_chunk(
                            x_W, col.astype(np.float32))
                        if xq.shape[0] > 0:
                            new_pts.append(xq)
                            new_cols.append(cq)
                            n_total += xq.shape[0]
                    except Exception:
                        continue
            # ★ **전역** voxel + 상한. 프레임마다 voxel 을 걸어도 프레임끼리 겹친
            #   점은 그대로 남아, 1000 프레임이면 5M 점이 된다(run_182946 실측
            #   5,121,810). 그 상태로 다음 세션에 들어가면 snapshot 한 번이 몇 초다.
            if n_total:
                P = np.vstack(new_pts)
                C = np.vstack(new_cols)
                P, C = self._voxel_chunk(P, C)
                if P.shape[0] > self.max_points:
                    sel = np.random.default_rng(0).choice(
                        P.shape[0], self.max_points, replace=False)
                    P, C = P[sel], C[sel]
                if P.shape[0] != n_total:
                    print(f"  [live] rebuild 축약 {n_total:,} → {P.shape[0]:,} 점 "
                          f"(전역 voxel {self.voxel_m*1000:.0f}mm, 상한 {self.max_points:,})")
                new_pts, new_cols, n_total = [P], [C], int(P.shape[0])
            self._pts = new_pts
            self._cols = new_cols
            self._n_buffered = n_total
            self._frames_ingested = sum(
                master_model.get_scan(i).frame_count()
                for i in range(n_scans)
            )
            self._dirty = True
            print(f"  [live] rebuild_from_model: {n_scans} scan(s), "
                  f"{n_total} pts (post-cleanup)")
        except Exception as e:
            print(f"  [live] ⚠ rebuild_from_model 실패 "
                  f"({type(e).__name__}: {e})")

    # ── snapshot write (Open3D 없음) ───────────────────────────────────

    def tick(self, scanning_flag: bool = True) -> bool:
        """스캔 루프가 부른다 — **절대 블로킹하지 않는다**. 실제 쓰기는 워커 몫."""
        if self._dead:
            return False
        try:
            with open(FLAG_PATH, "w") as f:
                f.write("1" if scanning_flag else "0")
        except Exception:
            pass
        if self._worker is None:
            self._worker = threading.Thread(
                target=self._writer_loop, name="live-snapshot", daemon=True)
            self._worker.start()
        self._req.set()
        return True

    def _writer_loop(self) -> None:
        while not self._worker_stop.is_set():
            if not self._req.wait(0.5):
                continue
            self._req.clear()
            if self._dead:
                return
            now = time.time()
            if (now - self._last_tick) < self.throttle_s:
                time.sleep(max(0.0, self.throttle_s - (now - self._last_tick)))
            self._last_tick = time.time()
            self._write_snapshot()

    def _write_snapshot(self) -> None:
        try:
            with self._lock:
                if not (self._dirty and self._n_buffered > 0):
                    return
                pts_l, cols_l = self._pts, self._cols
                # 처리하는 동안 들어오는 프레임은 빈 리스트에 쌓인다 — 아래에서 합친다.
                self._pts, self._cols, self._n_buffered = [], [], 0
                self._dirty = False
            if True:
                pts = np.vstack(pts_l)
                cols = np.vstack(cols_l)
                pts, cols = self._voxel_chunk(pts, cols)
                if pts.shape[0] > self.max_points:
                    sel = np.random.default_rng(0).choice(
                        pts.shape[0], self.max_points, replace=False)
                    pts, cols = pts[sel], cols[sel]
                with self._lock:                 # 처리 중 들어온 새 점 앞에 되돌린다
                    self._pts.insert(0, pts)
                    self._cols.insert(0, cols)
                    self._n_buffered += pts.shape[0]

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
        self._worker_stop.set()
        self._req.set()
        if self._worker is not None:
            self._worker.join(timeout=5.0)          # 쓰던 snapshot 은 끝내고 나간다
        try:
            with self._lock:
                _pl, _cl, _n = list(self._pts), list(self._cols), self._n_buffered
            if _pl and _n > 0:
                pts = np.vstack(_pl).astype(np.float32)
                cols = np.clip(np.vstack(_cl), 0.0, 1.0)
                rgb = (cols * 255.0).astype(np.uint8)
                # ★ run 폴더에 고정 이름으로 (2026-09-23). 예전엔 output/ 바로 밑에
                #   `live_cloud_<지금시각>.ply` 였다 — 그 시각은 **파일 쓴 때**(finalize)라
                #   RUN_TS 와 달라서 "왜 그 이름의 run 폴더가 없냐" 를 매번 묻게 됐다.
                #   IPC 용 `_live_latest.*` 는 외부 뷰어가 인자 없이 찾으므로 고정 경로 유지.
                from utils.run_paths import run_dir
                _rd = run_dir(); _rd.mkdir(parents=True, exist_ok=True)
                ply = str(_rd / "live_cloud.ply")
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


# ─────────────────────────────────────────────────────────────────────────────
# ★ 이 파일은 **뷰어가 아니다** — 직접 실행해도 아무 일도 일어나지 않는다.
#   그래서 실행했을 때 조용히 끝나지 않고 어디로 가야 하는지 알려준다
#   (2026-09-21: `python mms_artec/nbv/live_scan_viewer.py` 를 뷰어로 알고
#    실행했는데 아무 반응이 없어 원인을 찾느라 시간을 썼다).
if __name__ == "__main__":
    import sys
    print(__doc__ or "")
    print("=" * 66)
    print("  이 파일은 파이프라인 쪽 **스냅샷 writer** 다 (Open3D 를 안 만진다).")
    print("  화면에 띄우는 뷰어는 따로 있다:")
    print()
    print("      python scripts/artec/live_scan_view.py")
    print()
    print(f"  (스냅샷: {SNAP_PATH})")
    print("=" * 66)
    sys.exit(2)
