# scripts/artec/live_scan_viz_test.py
#
# 격리 테스트 (v2): Artec 스트리밍 → 실시간 시각화, **Open3D 신형 렌더러**.
#
# 왜 v2
# -----
# legacy o3d.visualization.Visualizer 는 이 환경에서 동적 포인트클라우드를
# 못 그린다 (메쉬는 됨, 점은 한 점으로 뭉갬 — in-loop/subprocess/단일스레드
# 모든 구성에서 재현). → Filament 기반 신형 렌더러(O3DVisualizer, Artec
# Studio 와 같은 계열)로 교체. legacy 와 완전히 다른 코드 경로.
#
# 구조 (신형 렌더러의 올바른 사용법)
# ----------------------------------
#   - GUI(렌더링) : 메인 스레드. gui.Application + O3DVisualizer.
#   - 스캔(SDK)   : 백그라운드 스레드. poll_events 누적.
#   - 스레드 간   : numpy 스냅샷을 app.post_to_main_thread 로 GUI 에 전달.
#                   SDK 는 bg 스레드만, Open3D 는 main 스레드만 → race 없음.
#
# 턴테이블/로봇/멀티패스 전부 없음. 손으로 정적 물체를 비추면 됨.
#
#   python scripts/artec/live_scan_viz_test.py          # 60s
#   python scripts/artec/live_scan_viz_test.py 30
#
# Notation: T_AB : A → B (x_B = T_AB @ x_A) — CLAUDE.md 준수.

import sys
import threading
import time
from datetime import datetime
from pathlib import Path

import numpy as np

_ROOT = Path(__file__).resolve().parents[2]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

import open3d as o3d  # noqa: E402
import open3d.visualization.gui as gui  # noqa: E402
import open3d.visualization.rendering as rendering  # noqa: E402

from mms_artec.sensor import artec_scanning  # noqa: E402
from mms_artec.sensor.artec_client import ArtecClient, ArtecConfig  # noqa: E402

DURATION_S = float(sys.argv[1]) if len(sys.argv) > 1 else 60.0
VOXEL_M = 0.0005          # 0.5mm — Spider 해상도(0.1mm) 활용. 더 디테일하게
                          # 하려면 0.0003. 성능 빡세면 0.0008.
MAX_POINTS = 4_000_000    # Filament 는 수백만 점 OK
RENDER_HZ = 8.0
COLOR_GAMMA = 1.0         # 1.0 = 텍스처 원색 그대로 (밝기 부스트 제거).
                          # Artec 텍스처가 어두우면 0.8 정도로만.
SETTLE_S = 1.5


def _voxel(pts, cols):
    if pts.shape[0] == 0:
        return pts, cols
    k = np.floor(pts / VOXEL_M).astype(np.int64)
    _, idx = np.unique(k, axis=0, return_index=True)
    return pts[idx], cols[idx]


def _colors(fm, n):
    try:
        uv = fm.uv()
        img = fm.image()
        if uv is None or img is None or uv.shape[0] != n:
            raise ValueError
        H, W = img.shape[0], img.shape[1]
        u = np.clip(uv[:, 0], 0.0, 1.0)
        v = np.clip(1.0 - uv[:, 1], 0.0, 1.0)
        px = np.clip((u * (W - 1)).astype(np.int32), 0, W - 1)
        py = np.clip((v * (H - 1)).astype(np.int32), 0, H - 1)
        return img[py, px, :3].astype(np.float32) / 255.0
    except Exception:
        return np.full((n, 3), 0.6, dtype=np.float32)


class LiveTest:
    def __init__(self):
        self.app = gui.Application.instance
        self.app.initialize()
        self.win = o3d.visualization.O3DVisualizer(
            "Artec LIVE (Filament renderer)", 1600, 900)
        self.win.show_settings = True
        # 어두운 배경 — 텍스처 원색이 또렷이 살아남 (렌더 정상화됐으니
        # 더는 밝은 배경 불필요).
        self.win.set_background([0.06, 0.06, 0.08, 1.0], None)
        self.win.show_skybox(False)
        self.app.add_window(self.win)

        self.mat = rendering.MaterialRecord()
        self.mat.shader = "defaultUnlit"
        self.mat.point_size = 3.0    # 조밀한 클라우드엔 작은 점이 더 선명

        # 원점 축 (메쉬 — 신형 렌더러에서도 기준점)
        ax = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.08)
        self.win.add_geometry("axis", ax)

        self._added = False
        self._framed = False
        self._stop = threading.Event()
        self._final = None        # (P, C) 종료 시 PLY 용
        self._t = threading.Thread(target=self._scan_loop, daemon=True)
        self._t.start()

    # ── GUI 스레드에서 실행 (post_to_main_thread) ──────────────────────
    def _apply(self, P, C):
        try:
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(P.astype(np.float64))
            Cv = np.clip(np.power(C, COLOR_GAMMA), 0.0, 1.0)
            pcd.colors = o3d.utility.Vector3dVector(Cv.astype(np.float64))
            if self._added:
                self.win.remove_geometry("cloud")
            self.win.add_geometry("cloud", pcd, self.mat)
            self._added = True
            if not self._framed:
                self._framed = True
                ctr = np.asarray(
                    pcd.get_axis_aligned_bounding_box().get_center(),
                    dtype=np.float32)
                eye = (ctr + np.array([0.5, -0.5, 0.5], np.float32)
                       ).astype(np.float32)
                up = np.array([0.0, 0.0, 1.0], np.float32)
                self.win.setup_camera(60.0, ctr, eye, up)
        except Exception as e:
            print(f"  [gui] _apply 예외: {e}")

    def _close(self):
        try:
            self.win.close()
        except Exception:
            pass

    # ── 백그라운드 스캔 스레드 ─────────────────────────────────────────
    def _scan_loop(self):
        print("=== Artec live viz test v2 (Filament) ===")
        client = ArtecClient(ArtecConfig(capture_texture=True))
        client.initialize()
        scanner = client._scanner
        try:
            scanner.set_fps(scanner.max_fps())
        except Exception:
            pass

        s = artec_scanning.ScanSessionSettings.default()
        s.set_max_frame_count(0)
        s.set_registration_type(artec_scanning.RegistrationType.HYBRID)
        s.set_pipeline(
            int(artec_scanning.ScanningPipelineFlags.REGISTER_FRAME) |
            int(artec_scanning.ScanningPipelineFlags.FIND_GEOMETRY_KEYFRAME) |
            int(artec_scanning.ScanningPipelineFlags.MAP_TEXTURE) |
            int(artec_scanning.ScanningPipelineFlags.CONVERT_TEXTURES))
        s.set_initial_state(artec_scanning.ScanningState.PREVIEW)
        s.set_capture_texture(artec_scanning.CaptureTextureMethod.ALWAYS)
        s.set_ignore_registration_errors(True)
        session = artec_scanning.ScanSession.create(scanner, s)

        print(f"  start_preview (settle {SETTLE_S}s) → start_record")
        session.start_preview()
        time.sleep(SETTLE_S)
        session.poll_events()
        session.start_record()
        t0 = time.time()
        print("  ● RECORDING — 스캐너로 물체를 비추세요. 창닫기/시간초과=종료\n")

        buf_p, buf_c = [], []
        n_frames = 0
        last_render = 0.0
        last_log = 0.0
        try:
            while not self._stop.is_set():
                now = time.time()
                elapsed = now - t0
                if elapsed > DURATION_S:
                    print("  (duration 종료)")
                    break
                for ev in session.poll_events():
                    if (ev.frame_state == artec_scanning.FrameState.OK
                            and ev.frame_mesh is not None
                            and ev.transformation is not None):
                        fm = ev.frame_mesh
                        v = fm.vertices()
                        if v is None or v.shape[0] == 0:
                            continue
                        v = v.astype(np.float64)
                        T = np.asarray(ev.transformation,
                                       dtype=np.float64).reshape(4, 4)
                        xw = (v @ T[:3, :3].T + T[:3, 3]) / 1000.0
                        c = _colors(fm, v.shape[0])
                        p, c = _voxel(xw.astype(np.float32), c)
                        if p.shape[0]:
                            buf_p.append(p)
                            buf_c.append(c)
                            n_frames += 1

                if buf_p and (now - last_render) >= (1.0 / RENDER_HZ):
                    last_render = now
                    P = np.vstack(buf_p)
                    C = np.vstack(buf_c)
                    P, C = _voxel(P, C)
                    if P.shape[0] > MAX_POINTS:
                        sel = np.random.default_rng(0).choice(
                            P.shape[0], MAX_POINTS, replace=False)
                        P, C = P[sel], C[sel]
                    buf_p, buf_c = [P], [C]
                    self._final = (P.copy(), C.copy())
                    # GUI 스레드로 전달 (복사본)
                    Pp, Cc = P.copy(), C.copy()
                    self.app.post_to_main_thread(
                        self.win, lambda Pp=Pp, Cc=Cc: self._apply(Pp, Cc))

                    if (now - last_log) >= 2.0:
                        last_log = now
                        mn, mx = P.min(0), P.max(0)
                        print(f"  [{elapsed:5.1f}s] frames={n_frames} "
                              f"pts={P.shape[0]} "
                              f"ext=({mx[0]-mn[0]:.2f},{mx[1]-mn[1]:.2f},"
                              f"{mx[2]-mn[2]:.2f})m "
                              f"colμ={float(C.mean()):.2f}")
                time.sleep(0.01)
        finally:
            print("\n  session.stop() ...")
            try:
                session.stop()
            except Exception:
                pass
            try:
                client.shutdown()
            except Exception:
                pass
            self._save_ply()
            self.app.post_to_main_thread(self.win, self._close)

    def _save_ply(self):
        if not self._final:
            return
        P, C = self._final
        rgb = (np.clip(C, 0, 1) * 255).astype(np.uint8)
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        out = _ROOT / "output" / f"live_test_{ts}.ply"
        out.parent.mkdir(parents=True, exist_ok=True)
        hdr = ("ply\nformat binary_little_endian 1.0\n"
               f"element vertex {P.shape[0]}\n"
               "property float x\nproperty float y\nproperty float z\n"
               "property uchar red\nproperty uchar green\n"
               "property uchar blue\nend_header\n").encode()
        rec = np.zeros(P.shape[0],
                       dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"),
                              ("r", "u1"), ("g", "u1"), ("b", "u1")])
        rec["x"], rec["y"], rec["z"] = P[:, 0], P[:, 1], P[:, 2]
        rec["r"], rec["g"], rec["b"] = rgb[:, 0], rgb[:, 1], rgb[:, 2]
        with open(out, "wb") as f:
            f.write(hdr)
            f.write(rec.tobytes())
        print(f"  PLY 저장 → {out}  ({P.shape[0]} pts)  "
              f"(CloudCompare/MeshLab 로 열기 — Windows 3D뷰어는 점PLY 미지원)")


def main() -> int:
    lt = LiveTest()
    lt.app.run()             # 메인스레드 블로킹 GUI 루프
    lt._stop.set()
    lt._t.join(timeout=3.0)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
