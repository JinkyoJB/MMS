"""
probe: master.sproj 구조 확인 + 세션별 점군 추출 검증.
ascii-only. hardware 불필요.
"""
from __future__ import annotations
import sys
from pathlib import Path
import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT_ROOT))

SPROJ = PROJECT_ROOT / "output" / "scan_raw" / "20260520_153357" / "master.sproj"


def scan_to_points(scan):
    """모든 frame 을 frame_transformation 으로 scan 좌표계에 누적 (mm)."""
    pts = []
    cols = []
    n = scan.frame_count()
    n_tex = 0
    for i in range(n):
        f = scan.get_frame(i)
        v = np.asarray(f.vertices(), dtype=np.float64)  # (N,3) mm
        if v.size == 0:
            continue
        T = np.asarray(scan.get_frame_transformation(i), float)
        vw = v @ T[:3, :3].T + T[:3, 3]
        pts.append(vw)
        if f.is_textured():
            n_tex += 1
    P = np.vstack(pts) if pts else np.zeros((0, 3))
    return P, n, n_tex


def main():
    from mms_artec.sensor.artec_client import ArtecClient
    print(f"[probe] open {SPROJ}  exists={SPROJ.exists()}")
    entries = ArtecClient.load_project(str(SPROJ))
    print(f"[probe] entries={len(entries)}")
    si = 0
    for e in entries:
        scan = getattr(e, "scan", None)
        if scan is None:
            print("  (non-scan entry)")
            continue
        P, nf, ntex = scan_to_points(scan)
        if len(P):
            mn = P.min(0); mx = P.max(0); ext = mx - mn
            print(f"  scan[{si}] frames={nf} textured_frames={ntex} "
                  f"pts={len(P):,}")
            print(f"           bbox_mm min={np.round(mn,1)} max={np.round(mx,1)} "
                  f"ext={np.round(ext,1)}")
        else:
            print(f"  scan[{si}] frames={nf} EMPTY")
        si += 1
    print(f"[probe] total scan sessions = {si}")

    # meta.npz
    meta_p = SPROJ.parent / "meta.npz"
    if meta_p.exists():
        m = np.load(meta_p, allow_pickle=False)
        print(f"[probe] meta keys={list(m.keys())} "
              f"n_scans={int(m['n_scans'])} has_T_BC={bool(m['has_T_BC'])} "
              f"hints={len(m['scan_indices'])}")


if __name__ == "__main__":
    main()
