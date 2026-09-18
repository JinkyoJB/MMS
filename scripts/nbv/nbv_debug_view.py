#!/usr/bin/env python
"""nbv_debug_view.py — nbv 디버그 뷰어 (사용자가 **다른 터미널에서 직접** 실행).

`utils/nbv/nbv_debug_dump.py` 가 반복마다 남기는 `output/debug/nbv/nbv_*.npz` 를 tail 하며
Filament(O3DVisualizer) 로 그린다. sim·real 공용 — 파이프라인이 무엇을 겨냥해 무엇을
얻었는지 한 그림에:

    회색   master(정합 대상 누적 점군)      주황 구   gap 후보(크기 = L)      빨강 구  고른 gap
    파랑   카메라 프러스텀(창 200~300mm)·광축   초록 호  턴테이블 스윕 구간(θ±span/2)
    노랑   찍힌 점(기구학 자리)              연두      정합 후 점            제목     정합 결과

키:  [ / ]  이전/다음 반복     r  카메라 리셋     q  종료
    python scripts/nbv/nbv_debug_view.py [디렉터리]        (기본 output/debug/nbv)
    python scripts/nbv/nbv_debug_view.py --png [디렉터리]  (창 없이 PNG 만 다시 생성)

왜 별도 실행인가 — 이 환경에서 Open3D 창은 파이프라인의 자식 프로세스로 띄우면
반짝 떴다 죽는다(`scripts/artec/live_scan_view.py` 와 같은 이유). 검증된 형태 그대로.
"""
import glob
import json
import math
import os
import sys
import threading
import time

import numpy as np

HFOV_DEG, VFOV_DEG, NEAR, FAR = 21.58, 28.58, 0.20, 0.30


def load(path):
    """npz → dict. 쓰는 중이거나 깨진 파일이면 None (호출자가 건너뛴다)."""
    try:
        with np.load(path, allow_pickle=False) as d:
            rec = {k: d[k] for k in d.files}
    except Exception as e:                                       # noqa: BLE001
        print(f"[view] {os.path.basename(path)} 못 읽음({type(e).__name__}) — 건너뜀")
        return None
    rec["result"] = json.loads(str(rec["result"])) if "result" in rec else {}
    return rec


def list_files(d):
    """완성된 기록만 — .part(쓰는 중) 제외, 최소 크기 확인."""
    out = []
    for f in sorted(glob.glob(os.path.join(d, "nbv_*.npz"))):
        try:
            if os.path.getsize(f) > 1000:
                out.append(f)
        except OSError:
            pass
    return out


def frustum_lines(eye, target, up):
    """카메라 프러스텀(작동거리 창 200~300mm) 선분 — (points, lines)."""
    f = target - eye; f /= np.linalg.norm(f) + 1e-12
    r = np.cross(f, up); r /= np.linalg.norm(r) + 1e-12
    u = np.cross(r, f)
    th, tv = math.tan(math.radians(HFOV_DEG) / 2), math.tan(math.radians(VFOV_DEG) / 2)
    P = [eye]
    for d in (NEAR, FAR):
        c = eye + f * d
        for sx, sy in ((-1, -1), (1, -1), (1, 1), (-1, 1)):
            P.append(c + r * (sx * d * th) + u * (sy * d * tv))
    L = [(1, 2), (2, 3), (3, 4), (4, 1), (5, 6), (6, 7), (7, 8), (8, 5),
         (1, 5), (2, 6), (3, 7), (4, 8), (0, 5), (0, 6), (0, 7), (0, 8)]
    return np.array(P), np.array(L)


def sweep_arc(axis, radius, theta, span_deg, up, n=24):
    """턴테이블 스윕 구간 호 — 축 둘레 반지름 radius, θ±span/2."""
    s = math.radians(span_deg) / 2
    P = []
    for a in np.linspace(theta - s, theta + s, n):
        P.append(axis + np.array([radius * math.cos(a), radius * math.sin(a), 0.0]))
    L = [(i, i + 1) for i in range(n - 1)]
    return np.array(P), np.array(L)


def build_geoms(rec):
    import open3d as o3d
    g = {}
    def pcd(P, color):
        p = o3d.geometry.PointCloud(); p.points = o3d.utility.Vector3dVector(np.asarray(P, float))
        p.paint_uniform_color(color); return p
    def lines(P, L, color):
        ls = o3d.geometry.LineSet(); ls.points = o3d.utility.Vector3dVector(P)
        ls.lines = o3d.utility.Vector2iVector(L); ls.paint_uniform_color(color); return ls
    up = np.array([0, 0, float(rec["up_sign"])])
    if len(rec["master"]): g["master"] = pcd(rec["master"], (0.7, 0.7, 0.7))
    if len(rec["patch_raw"]): g["patch_raw"] = pcd(rec["patch_raw"], (1.0, 0.85, 0.0))
    if len(rec["patch_aligned"]): g["patch_aligned"] = pcd(rec["patch_aligned"], (0.2, 0.9, 0.2))
    for i, (p, L) in enumerate(zip(rec["gaps_p"], rec["gaps_L"])):
        s = o3d.geometry.TriangleMesh.create_sphere(radius=max(0.003, min(0.012, float(L) * 0.15)))
        s.translate(p); s.paint_uniform_color((1.0, 0.1, 0.1) if i == int(rec["chosen"]) else (1.0, 0.55, 0.0))
        g[f"gap{i:02d}"] = s
    P, L = frustum_lines(rec["eye"], rec["target"], up); g["frustum"] = lines(P, L, (0.1, 0.3, 1.0))
    r = float(np.linalg.norm((rec["target"] - rec["axis"])[:2])) or 0.1
    P, L = sweep_arc(rec["axis"], r + 0.02, float(rec["theta"]), float(rec["span_deg"]), up); g["sweep"] = lines(P, L, (0.1, 0.7, 0.3))
    g["axis"] = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.05, origin=rec["axis"])
    return g


def title(rec, name):
    res = rec["result"]
    t = f"{name}  {rec['mode']}  gap#{int(rec['chosen'])}  θ={math.degrees(float(rec['theta'])) % 360:.0f}° ±{float(rec['span_deg'])/2:.0f}°  patch {len(rec['patch_raw']):,}pt"
    if res:
        t += f"  | icp {'OK' if res['ok'] else 'REJECT'} fit={res['fitness']:.2f} rmse={res['rmse_m']*1000:.1f}mm Δt={res['dt_m']*1000:.1f}mm  {res['reason']}"
    return t


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    d = args[0] if args else "output/debug/nbv"
    if "--png" in sys.argv:
        from utils.nbv.nbv_debug_dump import NbvDebugDump
        for f in list_files(d):
            rec = load(f)
            if rec is not None:
                NbvDebugDump._png(f[:-4] + ".png", rec, rec["result"]); print("png", f)
        return 0
    import open3d as o3d
    from open3d.visualization import gui
    app = gui.Application.instance; app.initialize()
    win = o3d.visualization.O3DVisualizer("NBV debug", 1500, 900); win.show_settings = True
    state = {"files": [], "i": -1, "names": []}
    mat_pt = o3d.visualization.rendering.MaterialRecord(); mat_pt.shader = "defaultUnlit"; mat_pt.point_size = 2.0
    mat_ln = o3d.visualization.rendering.MaterialRecord(); mat_ln.shader = "unlitLine"; mat_ln.line_width = 2.0

    def show(i):
        try:
            _show(i)
        except Exception as e:                                   # noqa: BLE001
            print(f"[view] 표시 실패({type(e).__name__}: {e}) — 창은 유지")

    def _show(i):
        if not state["files"]: return
        i = max(0, min(i, len(state["files"]) - 1)); state["i"] = i
        f = state["files"][i]; rec = load(f)
        if rec is None:
            return
        for n in state["names"]:
            try: win.remove_geometry(n)
            except Exception: pass
        state["names"] = []
        for n, geo in build_geoms(rec).items():
            m = mat_ln if isinstance(geo, o3d.geometry.LineSet) else mat_pt
            win.add_geometry(n, geo, m); state["names"].append(n)
        win.title = title(rec, os.path.basename(f))
        if i == 0 or "first" not in state:
            win.reset_camera_to_default(); state["first"] = True
        win.post_redraw()

    def on_key(e):
        if e.type != gui.KeyEvent.Type.DOWN: return False
        if e.key == ord("]"): show(state["i"] + 1); return True
        if e.key == ord("["): show(state["i"] - 1); return True
        if e.key == ord("r"): win.reset_camera_to_default(); return True
        if e.key == ord("q"): app.quit(); return True
        return False
    win.set_on_key(on_key) if hasattr(win, "set_on_key") else None

    def watcher():
        seen = 0
        while True:
            files = list_files(d)
            if len(files) != seen:
                seen = len(files); state["files"] = files
                app.post_to_main_thread(win, lambda: show(len(files) - 1))
            time.sleep(1.0)
    threading.Thread(target=watcher, daemon=True).start()
    app.add_window(win); app.run(); return 0


if __name__ == "__main__":
    sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))
    sys.exit(main())
