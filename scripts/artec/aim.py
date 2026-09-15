#!/usr/bin/env python
"""aim.py — 캘리브 **조준 한 방에**: 로봇 웹 UI + 스캐너 실시간 뷰어.

조준은 두 개를 같이 봐야 한다 —
  · 로봇을 **손으로 끄는 수단**  = 웹 UI 수동 모드 (`http://<robot>:18333`)
  · 지금 **뭐가 찍히는지**       = `live_view.py` (텍스처 카메라 + ChArUco 판정)

둘을 따로 띄우다 보면 순서를 빠뜨린다. 특히 **끝나고 웹 UI 탭을 닫는 것** —
안 닫으면 이후 SDK 명령이 통째로 무시된다(`robot_control.md` §1).
이 스크립트가 순서와 뒷정리 안내를 담당한다.

    python scripts/artec/aim.py
    python scripts/artec/aim.py --no-browser     # 브라우저는 직접 열겠다

절차는 `docs/calibration_runbook.md` §2.
"""
from __future__ import annotations

import argparse
import runpy
import sys
import webbrowser
from pathlib import Path

_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(_ROOT))

ROBOT_IP = "192.168.1.210"
BAR = "=" * 68


def _studio_running() -> list[str]:
    """Artec Studio 가 스캐너를 물고 있으면 live_view 가 못 뜬다."""
    names = []
    try:
        import subprocess
        out = subprocess.run(
            ["tasklist", "/FI", "IMAGENAME eq astudio_pro.exe"],
            capture_output=True, text=True, timeout=10).stdout
        if "astudio_pro" in out:
            names.append("astudio_pro")
    except Exception:                                          # noqa: BLE001
        pass
    return names


def main() -> int:
    ap = argparse.ArgumentParser(description="조준 헬퍼 (웹 UI + 실시간 뷰어)")
    ap.add_argument("--ip", default=ROBOT_IP)
    ap.add_argument("--no-browser", action="store_true")
    args, rest = ap.parse_known_args()

    url = f"http://{args.ip}:18333"

    busy = _studio_running()
    if busy:
        print(f"\n⚠ {', '.join(busy)} 가 실행 중이다 — 스캐너를 독점한다.")
        print("  먼저 닫을 것:  Stop-Process -Name astudio_pro -Force\n")
        return 1

    print(BAR)
    print("  캘리브 조준")
    print(BAR)
    print("  1. 브라우저의 xArm Studio 에서 **Manual Mode** 를 켠다")
    print("     → 중력보상이 걸려 팔을 손으로 밀 수 있다")
    print("  2. 스캐너를 잡고 보드가 **화면 중심 십자**에 오도록 맞춘다")
    print("     → 뷰어가 OK / 잘림 / 일부만 을 그 자리에서 알려준다")
    print("  3. `OK  markers 7/8` 이 뜨면 손을 뗀다 (툴 무게로 처질 수 있으니 확인)")
    print("  4. **Manual Mode 를 끈다**")
    print("  5. 뷰어를 q 로 닫는다 (스캐너를 놔줘야 다음 단계가 잡는다)")
    print(BAR)
    print(f"  웹 UI : {url}")
    if not args.no_browser:
        try:
            webbrowser.open(url)
            print("          (브라우저를 열었다)")
        except Exception as e:                                 # noqa: BLE001
            print(f"          (브라우저 자동 실행 실패: {e} — 직접 열 것)")
    print(BAR + "\n")

    sys.argv = [str(_ROOT / "scripts" / "artec" / "live_view.py")] + rest
    try:
        runpy.run_path(sys.argv[0], run_name="__main__")
    except SystemExit:
        pass

    print("\n" + BAR)
    print("  ★ 끝내기 전에 두 가지")
    print("     · xArm Studio 의 **Manual Mode 를 껐는지**")
    print("     · **브라우저 탭을 닫았는지** — 열려 있으면 컨트롤러 mode/state 를")
    print("       웹이 계속 바꿔서 이후 SDK 명령이 무시된다 (robot_control.md §1)")
    print(BAR)
    print("\n  다음:  python scripts\\artec\\gen_calib_poses.py --from-view")
    return 0


if __name__ == "__main__":
    sys.exit(main())
