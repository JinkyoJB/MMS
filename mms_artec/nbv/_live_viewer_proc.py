# mms_artec/nbv/_live_viewer_proc.py
#
# 라이브 뷰어 **서브프로세스** — 독립 실행 전용 (직접 import 하지 말 것).
# 렌더러: Open3D 신형 **O3DVisualizer (Filament)**.
#
# 왜 이 구조인가
# --------------
# 1) 파이프라인 메인 스레드는 스캔/턴테이블 오케스트레이션이 점유 → 블로킹인
#    gui.Application.run() 을 거기 둘 수 없다. → 별도 프로세스로 분리.
# 2) 이 환경의 legacy o3d.visualization.Visualizer 는 동적 PointCloud 를 못
#    그린다 (메쉬는 됨). 신형 O3DVisualizer(Filament, Artec Studio 와 같은
#    계열) 는 정상 — scripts/artec/live_scan_viz_test.py 로 검증됨.
#
# 이 프로세스 내부 구조 (신형 렌더러의 올바른 사용법)
#   - GUI(렌더)  : 이 프로세스의 메인 스레드. app.run() 블로킹.
#   - watcher    : 백그라운드 스레드. 부모가 atomic write 한 snapshot .npy
#                  (N,6: xyz+rgb, m, scan-world) 의 mtime 을 폴링.
#   - 스레드 간  : app.post_to_main_thread 로 numpy 스냅샷 전달
#                  (Open3D 는 메인 스레드만 만짐 → race 없음).
#
# IPC (부모 = live_scan_viewer.LiveScanViewer)
#   snap : (N,6) float32  [x,y,z, r,g,b]   (m, scan-world; rgb 0..1)
#   flag : '1'/'0'  scanning_flag (정합 끊기면 배경 빨강; best-effort)
#   stop : 존재하면 종료
#
# argv: snap_path flag_path stop_path title

import os
import sys
import threading
import time

import numpy as np

GAMMA = 1.0          # 텍스처 원색. 어두우면 0.8 정도.
POINT_SIZE = 3.0
BG_NORM = [0.06, 0.06, 0.08, 1.0]
BG_LOST = [0.30, 0.06, 0.06, 1.0]


def main() -> int:
    if len(sys.argv) < 5:
        return 2
    snap_path, flag_path, stop_path, title = sys.argv[1:5]

    def log(m):
        print(f"[live-proc] {m}", flush=True)

    try:
        import open3d as o3d
        import open3d.visualization.gui as gui
        import open3d.visualization.rendering as rendering
    except Exception as e:
        log(f"open3d import 실패: {e}")
        return 3
    log(f"open3d {o3d.__version__} (Filament/O3DVisualizer)")

    app = gui.Application.instance
    app.initialize()
    win = o3d.visualization.O3DVisualizer(title, 1600, 900)
    win.show_settings = True
    win.set_background(BG_NORM, None)
    win.show_skybox(False)
    app.add_window(win)

    mat = rendering.MaterialRecord()
    mat.shader = "defaultUnlit"
    mat.point_size = POINT_SIZE

    axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.08)
    win.add_geometry("axis", axis)

    st = {"added": False, "framed": False, "stop": False, "bg": "norm"}

    # ── GUI 스레드에서 실행 (post_to_main_thread) ──────────────────────
    def apply(P, C):
        try:
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(P.astype(np.float64))
            Cv = np.clip(np.power(C, GAMMA), 0.0, 1.0)
            pcd.colors = o3d.utility.Vector3dVector(Cv.astype(np.float64))
            if st["added"]:
                win.remove_geometry("cloud")
            win.add_geometry("cloud", pcd, mat)
            st["added"] = True
            if not st["framed"]:
                st["framed"] = True
                bb = pcd.get_axis_aligned_bounding_box()
                ctr = np.asarray(bb.get_center(), dtype=np.float32)
                eye = (ctr + np.array([0.5, -0.5, 0.5], np.float32)
                       ).astype(np.float32)
                win.setup_camera(60.0, ctr, eye,
                                 np.array([0.0, 0.0, 1.0], np.float32))
                log(f"첫 클라우드 표시 n={P.shape[0]} center={np.round(ctr,3)}")
        except Exception as e:
            log(f"apply 예외: {e}")

    def set_bg(lost):
        try:
            win.set_background(BG_LOST if lost else BG_NORM, None)
        except Exception:
            pass

    # ── 백그라운드 watcher (snapshot 폴링) ─────────────────────────────
    def watcher():
        last_mtime = -1
        while not st["stop"]:
            try:
                if os.path.exists(stop_path):
                    log("stop 신호 감지")
                    break
                # scanning_flag → 배경 (변할 때만 post)
                try:
                    with open(flag_path, "r") as f:
                        lost = (f.read(1) == "0")
                    want = "lost" if lost else "norm"
                    if want != st["bg"]:
                        st["bg"] = want
                        app.post_to_main_thread(
                            win, lambda l=lost: set_bg(l))
                except Exception:
                    pass

                st_ = os.stat(snap_path)
                if st_.st_mtime_ns != last_mtime:
                    last_mtime = st_.st_mtime_ns
                    try:
                        arr = np.load(snap_path)        # (N,6) float32
                    except Exception:
                        time.sleep(0.1)
                        continue
                    if (arr.ndim == 2 and arr.shape[1] == 6
                            and arr.shape[0] > 0):
                        P = arr[:, :3].copy()
                        C = arr[:, 3:].copy()
                        app.post_to_main_thread(
                            win, lambda P=P, C=C: apply(P, C))
            except FileNotFoundError:
                pass                                    # snap 아직 없음
            except Exception as e:
                log(f"watcher 예외: {e}")
            time.sleep(0.15)
        # 종료 — GUI 닫기 (app.run() 반환시킴)
        try:
            app.post_to_main_thread(win, win.close)
        except Exception:
            pass

    t = threading.Thread(target=watcher, daemon=True)
    t.start()
    log("window OK — app.run() 진입")
    try:
        app.run()                 # 블로킹: 창 닫힘 또는 watcher가 win.close
    except Exception as e:
        log(f"app.run 예외: {e}")
    st["stop"] = True
    t.join(timeout=2.0)
    log("종료")
    return 0


if __name__ == "__main__":
    sys.exit(main())
