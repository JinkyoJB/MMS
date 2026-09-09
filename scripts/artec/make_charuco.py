#!/usr/bin/env python3
"""
scripts/artec/make_charuco.py

ChArUco 보드 PNG 생성 (Artec hand-eye 캘리브용).

프리셋은 `hand_eye_calib.BOARD_PRESETS` 를 그대로 쓴다. 기본은 `spider`.

  spider        5×3, square 20mm, marker 15mm, DICT_4X4_50  → 100×60mm  (권장)
  spider_small  5×3, square 16mm, marker 12mm, DICT_4X4_50  →  80×48mm
  a4            7×5, square 30mm, marker 22mm, DICT_5X5_100 → 210×150mm (PhoXi/광각용)

★ Spider 는 FOV 가 좁아(30°×21°, 작동거리 0.2~0.3m) a4 보드는 화면 밖으로 나간다.

인쇄
----
  배율 **100%** 로 출력한다. "용지에 맞춤(fit to page)" 을 반드시 끌 것 —
  이 옵션이 켜져 있으면 칸 크기가 달라져 캘리브 결과가 전부 틀어진다.
  인쇄 후 자로 한 칸을 실측하고, 20mm 가 아니면 캘리브 실행 시 `--square-mm` 으로 보정한다.

Usage
-----
  python scripts/artec/make_charuco.py
  python scripts/artec/make_charuco.py --board spider_small
  python scripts/artec/make_charuco.py --out custom.png --pixels-per-mm 5
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import cv2

_PROJECT_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(_PROJECT_ROOT))

from mms_artec.utils.calibration.artec_charuco_detector import CharucoBoardSpec


def main() -> None:
    # 캘리브 스크립트와 동일한 프리셋 사용
    from scripts.artec.hand_eye_calib import BOARD_PRESETS
    parser = argparse.ArgumentParser(description="ChArUco 보드 PNG 생성")
    parser.add_argument("--out", type=str, default=None,
                        help="출력 경로 (기본: debug_calib/charuco_<sx>x<sy>_<sq>_<mk>.png)")
    parser.add_argument("--board", type=str, default="spider",
                        choices=list(BOARD_PRESETS.keys()),
                        help="보드 프리셋 — Artec Spider 는 'spider' 권장")
    parser.add_argument("--squares-x", type=int, default=None)
    parser.add_argument("--squares-y", type=int, default=None)
    parser.add_argument("--square-mm", type=float, default=None)
    parser.add_argument("--marker-mm", type=float, default=None)
    parser.add_argument("--pixels-per-mm", type=float, default=10.0,
                        help="해상도 (10 px/mm = 300 DPI)")
    args = parser.parse_args()

    base = BOARD_PRESETS[args.board]
    spec = CharucoBoardSpec(
        squares_x=int(args.squares_x) if args.squares_x is not None else base.squares_x,
        squares_y=int(args.squares_y) if args.squares_y is not None else base.squares_y,
        square_length_mm=float(args.square_mm) if args.square_mm is not None else base.square_length_mm,
        marker_length_mm=float(args.marker_mm) if args.marker_mm is not None else base.marker_length_mm,
        aruco_dict=base.aruco_dict,
    )
    board = spec.make_board()

    W = int(round(spec.squares_x * spec.square_length_mm * args.pixels_per_mm))
    H = int(round(spec.squares_y * spec.square_length_mm * args.pixels_per_mm))

    # 신/구 API 모두 지원
    try:
        img = board.generateImage((W, H), marginSize=20, borderBits=1)
    except AttributeError:
        img = board.draw((W, H), marginSize=20, borderBits=1)

    if args.out:
        out_path = Path(args.out)
    else:
        out_path = (_PROJECT_ROOT / "debug_calib" /
                    f"charuco_{spec.squares_x}x{spec.squares_y}_"
                    f"{int(spec.square_length_mm)}_{int(spec.marker_length_mm)}.png")
    out_path.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(out_path), img)

    board_w_mm = spec.squares_x * spec.square_length_mm
    board_h_mm = spec.squares_y * spec.square_length_mm
    print(f"[artec_make_charuco] saved → {out_path}")
    print(f"  preset: {args.board}  spec: {spec.squares_x}×{spec.squares_y}  "
          f"sq={spec.square_length_mm}mm  mk={spec.marker_length_mm}mm  "
          f"({board_w_mm:.0f}×{board_h_mm:.0f}mm)")
    print(f"  image: {W}×{H} px ({args.pixels_per_mm} px/mm)")
    print("  ⚠ 인쇄 후 한 칸 길이를 자로 실측해서 hand-eye script 의 "
          "`--square-mm` 으로 보정하세요.")


if __name__ == "__main__":
    main()
