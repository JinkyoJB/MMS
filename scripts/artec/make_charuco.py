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


def _write_a4_pdf(board_img, pdf_path: Path, spec, ppmm: float) -> None:
    """보드를 **A4 페이지 안에 실측 크기로** 박은 PDF 를 만든다.

    PNG 를 그대로 인쇄하면 뷰어·드라이버마다 배율 해석이 달라 칸 크기가 흔들린다.
    PDF 는 페이지 크기(A4)와 그 안의 오브젝트 크기가 문서에 박혀 있어서,
    "실제 크기/100%" 로만 인쇄하면 배율이 확정된다.

    검증용 **100mm 자 눈금**을 같이 그린다 — 인쇄 후 이 선이 100mm 가 아니면
    배율이 먹은 것이므로 그 비율로 `--square-mm` 을 보정하면 된다.
    """
    import numpy as np

    A4_W_MM, A4_H_MM = 210.0, 297.0
    W = int(round(A4_W_MM * ppmm))
    H = int(round(A4_H_MM * ppmm))
    page = np.full((H, W), 255, np.uint8)

    bh, bw = board_img.shape[:2]
    x0 = (W - bw) // 2
    y0 = int(round(40.0 * ppmm))
    page[y0:y0 + bh, x0:x0 + bw] = board_img

    def mm(v):     # mm → px
        return int(round(v * ppmm))

    # ── 100mm 검증 자 ──────────────────────────────────────────────────
    ry = y0 + bh + mm(30)
    rx0 = (W - mm(100)) // 2
    cv2.line(page, (rx0, ry), (rx0 + mm(100), ry), 0, max(1, mm(0.4)))
    for t in range(0, 101, 10):
        h = mm(4.0) if t % 50 == 0 else mm(2.5)
        x = rx0 + mm(t)
        cv2.line(page, (x, ry - h), (x, ry), 0, max(1, mm(0.3)))

    # ── 안내 문구 (ASCII only — cv2 는 한글 폰트가 없다) ──────────────
    fs = ppmm / 9.0
    th = max(1, int(round(ppmm / 8.0)))
    lines = [
        (mm(20), f"ChArUco {spec.squares_x}x{spec.squares_y}  "
                 f"square={spec.square_length_mm:.1f}mm  "
                 f"marker={spec.marker_length_mm:.1f}mm  "
                 f"board={spec.squares_x*spec.square_length_mm:.0f}x"
                 f"{spec.squares_y*spec.square_length_mm:.0f}mm"),
        (mm(28), "PRINT AT 100% / ACTUAL SIZE - turn OFF 'fit to page'"),
        (ry + mm(10), "100 mm reference - measure after printing."),
        (ry + mm(18), "If it is not 100mm, pass the measured square size via --square-mm."),
    ]
    for y, txt in lines:
        cv2.putText(page, txt, (mm(15), y), cv2.FONT_HERSHEY_SIMPLEX, fs, 0, th, cv2.LINE_AA)

    from PIL import Image
    dpi = ppmm * 25.4
    Image.fromarray(page).save(pdf_path, "PDF", resolution=dpi)
    print(f"[artec_make_charuco] saved → {pdf_path}")
    print(f"  A4 {A4_W_MM:.0f}×{A4_H_MM:.0f}mm @ {dpi:.0f} DPI  "
          f"— '실제 크기/100%' 로 인쇄할 것")


def main() -> None:
    # 캘리브 스크립트와 동일한 프리셋 사용
    from mms_artec.utils.calibration.artec_charuco_detector import (
        BOARD_PRESETS, DEFAULT_BOARD_NAME)
    parser = argparse.ArgumentParser(description="ChArUco 보드 PNG 생성")
    parser.add_argument("--out", type=str, default=None,
                        help="출력 경로 (기본: debug_calib/charuco_<sx>x<sy>_<sq>_<mk>.png)")
    parser.add_argument("--board", type=str, default=DEFAULT_BOARD_NAME,
                        choices=list(BOARD_PRESETS.keys()),
                        help="보드 프리셋 — Artec Spider 는 'spider_dense' 권장")
    parser.add_argument("--squares-x", type=int, default=None)
    parser.add_argument("--squares-y", type=int, default=None)
    parser.add_argument("--square-mm", type=float, default=None)
    parser.add_argument("--marker-mm", type=float, default=None)
    parser.add_argument("--pixels-per-mm", type=float, default=10.0,
                        help="해상도 (10 px/mm = 254 DPI)")
    parser.add_argument("--pdf", action="store_true",
                        help="A4 PDF 도 같이 만든다 (실측 크기 고정 — 인쇄용 권장)")
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

    # ★ 여백을 **더해서** 출력 크기를 잡는다. `generateImage` 의 marginSize 는
    #   바깥에 덧붙는 게 아니라 outSize **안쪽을** 깎는다. 게다가 OpenCV 는 칸을
    #   정사각으로 유지하려고 두 축 중 **더 빡빡한 쪽**에 칸 크기를 맞춘다.
    #   그래서 outSize 를 보드 크기 그대로 주면 인쇄된 칸이 공칭보다 작아진다
    #   — 5×3/20mm/10px_per_mm 에서 18.67mm (−6.7%), 7×5/12mm/20px_per_mm 에서
    #   11.60mm (−3.3%) 였다. 캘리브 스케일에 그대로 들어가는 계통 오차다.
    MARGIN_PX = 20
    W = int(round(spec.squares_x * spec.square_length_mm * args.pixels_per_mm)) + 2 * MARGIN_PX
    H = int(round(spec.squares_y * spec.square_length_mm * args.pixels_per_mm)) + 2 * MARGIN_PX

    # 신/구 API 모두 지원
    try:
        img = board.generateImage((W, H), marginSize=MARGIN_PX, borderBits=1)
    except AttributeError:
        img = board.draw((W, H), marginSize=MARGIN_PX, borderBits=1)

    if args.out:
        out_path = Path(args.out)
    else:
        out_path = (_PROJECT_ROOT / "debug_calib" /
                    f"charuco_{spec.squares_x}x{spec.squares_y}_"
                    f"{int(spec.square_length_mm)}_{int(spec.marker_length_mm)}.png")
    out_path.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(out_path), img)
    # ★ PNG 에 물리 크기(pHYs)를 박아 둔다. `cv2.imwrite` 는 이 청크를 안 쓴다 —
    #   그러면 인쇄 경로가 기본 96 DPI 로 가정해서 1720px 짜리 보드를 455mm 로
    #   해석하고, A4 를 넘으니 "용지에 맞춤" 으로 줄여 **페이지를 꽉 채운다**.
    #   실측에서 칸이 12mm 대신 ~29mm 로 나왔다.
    _dpi = args.pixels_per_mm * 25.4
    try:
        from PIL import Image
        Image.open(out_path).save(out_path, dpi=(_dpi, _dpi))
    except Exception as e:      # Pillow 없으면 PNG 는 그대로 두고 PDF 를 권한다
        print(f"  [warn] DPI 메타데이터 기록 실패({e}) — --pdf 로 인쇄할 것")

    if args.pdf:
        _write_a4_pdf(img, out_path.with_suffix(".pdf"), spec, args.pixels_per_mm)

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
