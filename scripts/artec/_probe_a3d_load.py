"""
de-risk probe: Artec Studio .a3d project loadable by SDK binding?

ascii-only prints (Korean Windows console = cp949).
hardware not required (no scanner connect).
"""
from __future__ import annotations

import sys
import traceback
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT_ROOT))

DS = PROJECT_ROOT / "datasets" / "noHint_20260520_142405"
CANDIDATES = [
    DS / "noHint_20260520_142405.a3d",
    DS,
    DS / "data" / "miscellaneous" / "project.conf",
]


def main() -> None:
    from mms_artec.sensor import artec_project  # noqa
    from mms_artec.sensor.artec_client import ArtecClient

    try:
        print(f"[probe] SDK max_supported_project_version = "
              f"{artec_project.ProjectManager.max_supported_version()}")
    except Exception as e:
        print(f"[probe] max_version query failed: {type(e).__name__}: {e}")

    for p in CANDIDATES:
        print(f"\n[probe] try open: {p}  (exists={p.exists()})")
        try:
            entries = ArtecClient.load_project(str(p))
        except Exception as e:
            print(f"  -> FAIL {type(e).__name__}: {e}")
            continue
        print(f"  -> OK entries={len(entries)}")
        n_scan = 0
        for i, e in enumerate(entries):
            scan = getattr(e, "scan", None)
            if scan is not None:
                n_scan += 1
                fc = scan.frame_count()
                vc = ""
                if fc > 0:
                    try:
                        vc = f" frame0_verts={len(scan.get_frame(0).vertices())}"
                    except Exception as ex:
                        vc = f" (frame0 fail: {ex})"
                print(f"     entry[{i}] SCAN frames={fc}{vc}")
        print(f"  -> total SCAN entries = {n_scan}")
        return
    print("\n[probe] all candidates failed.")


if __name__ == "__main__":
    try:
        main()
    except Exception:
        traceback.print_exc()
