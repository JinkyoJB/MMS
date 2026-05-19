# scripts/artec/live_scan_view.py
#
# 라이브 스캔 뷰어 (사용자가 직접 실행). main_artec.py 가 쓰는 스냅샷
# (output/_live_latest.npy) 을 tail 하며 **Filament(O3DVisualizer)** 로 렌더.
#
# 왜 별도 수동 실행
# -----------------
# 이 환경에서 라이브 PointCloud 가 실제로 렌더된 유일한 형태가
# "사용자가 터미널에서 직접 띄운 Filament 프로세스" 였다 (Popen 자식은
# GUI 가 반짝 떴다 죽음). 그 검증된 형태 그대로 둔다.
# 구조: 렌더=메인 스레드 app.run(), watcher=백그라운드 스레드(snapshot
# mtime 폴링) → post_to_main_thread 로 GUI 갱신.
#
# 사용
#   터미널 A:  python main_artec.py
#   터미널 B:  conda activate mms-env
#              python scripts/artec/live_scan_view.py
#   (B 를 먼저 띄워두고 A 를 시작해도 됨 — 스냅샷 생기면 자동 표시)
#
#   인자로 다른 .npy 경로 지정 가능:
#     python scripts/artec/live_scan_view.py path\to\snap.npy
#
# 종료: 창 닫기, 또는 main_artec 종료 시 자동(stop 신호).

import os
import sys
import threading
import time
from pathlib import Path

import numpy as np

_ROOT = Path(__file__).resolve().parents[2]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

import open3d as o3d  # noqa: E402
import open3d.visualization.gui as gui  # noqa: E402
import open3d.visualization.rendering as rendering  # noqa: E402

_OUT = _ROOT / "output"
SNAP = sys.argv[1] if len(sys.argv) > 1 else str(_OUT / "_live_latest.npy")
FLAG = str(_OUT / "_live_latest.flag")
STOP = str(_OUT / "_live_latest.stop")

GAMMA = 1.0          # 텍스처 원색. 어두우면 0.8
POINT_SIZE = 3.0
BG_NORM = [0.06, 0.06, 0.08, 1.0]
BG_LOST = [0.30, 0.06, 0.06, 1.0]


def main() -> int:
    print("=== Artec live scan viewer (Filament) ===")
    print(f"  snapshot: {SNAP}")
    print("  main_artec.py 를 시작하면 스캔이 여기 실시간 표시됩니다.")
    print("  창 닫기 / main_artec 종료 시 자동 종료.\n")

    # ★ 이전 run 이 남긴 stale stop 제거 + 뷰어 시작 시각 기록.
    #   이게 없으면 워처가 옛 stop 을 보고 창이 뜨기 전에 닫힌다.
    try:
        os.remove(STOP)
    except OSError:
        pass
    t_start = time.time()

    app = gui.Application.instance
    app.initialize()
    win = o3d.visualization.O3DVisualizer("Artec LIVE scan", 1600, 900)
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
                print(f"  ● 첫 클라우드 표시 n={P.shape[0]}")
        except Exception as e:
            print(f"  apply 예외: {e}")

    def set_bg(lost):
        try:
            win.set_background(BG_LOST if lost else BG_NORM, None)
        except Exception:
            pass

    def watcher():
        last_mtime = -1
        waited = False
        while not st["stop"]:
            try:
                # 뷰어 시작 이후에 생긴 stop 만 종료 신호로 인정
                # (시작 전 stale stop 에 자살하지 않도록).
                if os.path.exists(STOP) and os.path.getmtime(STOP) >= t_start:
                    print("  (main_artec 종료 신호 — 뷰어 종료)")
                    break
                try:
                    with open(FLAG, "r") as f:
                        lost = (f.read(1) == "0")
                    want = "lost" if lost else "norm"
                    if want != st["bg"]:
                        st["bg"] = want
                        app.post_to_main_thread(win, lambda l=lost: set_bg(l))
                except Exception:
                    pass

                stt = os.stat(SNAP)
                if stt.st_mtime_ns != last_mtime:
                    last_mtime = stt.st_mtime_ns
                    try:
                        arr = np.load(SNAP)
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
                if not waited:
                    waited = True
                    print("  스냅샷 대기 중… (main_artec.py 시작하세요)")
            except Exception as e:
                print(f"  watcher 예외: {e}")
            time.sleep(0.15)
        try:
            app.post_to_main_thread(win, win.close)
        except Exception:
            pass

    t = threading.Thread(target=watcher, daemon=True)
    t.start()
    app.run()                       # 메인스레드 블로킹 GUI 루프
    st["stop"] = True
    t.join(timeout=2.0)
    print("  종료.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
