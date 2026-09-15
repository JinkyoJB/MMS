#!/usr/bin/env python
"""live_view.py — 스캐너 **텍스처 카메라를 실시간으로 본다** (조준용).

Artec Studio 는 스캔 모드 미리보기만 주고, 리포의 다른 뷰어
(`live_scan_view.py`, `live_scan_viz_test.py`)는 전부 **점군**이다.
캘리브 조준에는 "보드가 화면 안에 다 들어왔나"를 봐야 하는데 그건 텍스처
이미지라야 보인다 — 그래서 이 도구가 따로 있다.

ChArUco 를 같이 검출해 **지금 자세가 캘리브에 쓸 만한지**를 그 자리에서 알려준다.

    python scripts/artec/live_view.py
    python scripts/artec/live_view.py --no-charuco     # 이미지만

    q / ESC : 종료        s : 스냅샷 저장 (output/live_view/)

⚠ 스캐너는 SDK 와 Artec Studio 중 하나만 잡는다 — Studio 를 닫고 실행할 것.
  이 도구가 스캐너를 물고 있으므로 `calibrate.py` 와 동시에 못 돌린다.

조준 요령 — 로봇은 **웹 UI 수동 모드**로 손으로 끌면서 이 화면을 본다
(`docs/calibration_runbook.md` §2). 끝나면 수동 모드를 끄고 탭을 닫는다.
"""
from __future__ import annotations

import argparse
import sys
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
import yaml

_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(_ROOT))

from mms_artec.sensor.artec_client import ArtecClient, ArtecConfig     # noqa: E402

INTRINSIC = _ROOT / "config" / "calibration" / "artec_intrinsic.yaml"
SNAP_DIR = _ROOT / "output" / "live_view"
MARKER_MM = 15.0                      # spider 프리셋 marker 한 변
N_MARKERS = 8                         # 5x3 보드의 마커 수 (전부 보이면 완벽)
WIN = "Artec live  [q=quit  s=snapshot]"


def _fx() -> float | None:
    """거리 추정용 초점거리. 낡은 값이어도 대략 거리를 보는 데는 쓸 만하다."""
    if not INTRINSIC.exists():
        return None
    try:
        return float(np.asarray(
            yaml.safe_load(INTRINSIC.read_text(encoding="utf-8"))["K"], float)[0, 0])
    except Exception:                                          # noqa: BLE001
        return None


def main() -> int:
    ap = argparse.ArgumentParser(description="Artec 텍스처 카메라 실시간 뷰어")
    ap.add_argument("--no-charuco", action="store_true", help="검출 오버레이 끄기")
    ap.add_argument("--max-h", type=int, default=900, help="표시 최대 높이 px")
    args = ap.parse_args()

    fx = _fx()
    adict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    adet = cv2.aruco.ArucoDetector(adict, cv2.aruco.DetectorParameters())

    sensor = ArtecClient(ArtecConfig(serial_number=None, capture_texture=True))
    print("[live] 스캐너 연결 중 ...")
    sensor.initialize()
    # ★ auto exposure 를 안 켜면 capture_frame 이 계속 None 을 돌려준다(실측).
    #   hand_eye_calib.py 도 같은 설정을 하고 시작한다.
    try:
        sensor._scanner.enable_auto_exposure(True)
    except Exception as e:                                     # noqa: BLE001
        print(f"[live] ⚠ auto exposure 설정 실패: {e}")
    print("[live] q 로 종료. 로봇은 웹 UI 수동 모드로 끌면서 이 화면을 볼 것.")

    n, t0, fps, n_none = 0, time.time(), 0.0, 0
    try:
        while True:
            try:
                fmh = sensor.capture_frame(capture_texture=True)
            except RuntimeError as e:                          # SDK 일시 실패는 넘긴다
                print(f"  [warn] capture: {e}")
                time.sleep(0.2)
                continue
            if fmh is None or not fmh.has_image() or not fmh.is_textured():
                # 조용히 도는 대신 알린다 — 빈 창만 보고 원인을 못 찾는 일을 막는다.
                n_none += 1
                if n_none in (10, 50) or n_none % 200 == 0:
                    print(f"[live] ⚠ 텍스처 프레임을 {n_none}회 연속 못 받았다. "
                          f"스캐너 앞에 물체가 있는지 · Artec Studio 가 떠 있지 않은지 확인")
                time.sleep(0.1)
                continue
            n_none = 0

            img = fmh.image()
            vis = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            h, w = vis.shape[:2]
            msg, color = "", (200, 200, 200)

            if not args.no_charuco:
                corners, ids, _ = adet.detectMarkers(cv2.cvtColor(img, cv2.COLOR_RGB2GRAY))
                n_ids = 0 if ids is None else len(ids)
                if n_ids:
                    cv2.aruco.drawDetectedMarkers(vis, corners, ids)
                    pts = np.concatenate([c.reshape(4, 2) for c in corners])
                    x0, y0, x1, y1 = (pts[:, 0].min(), pts[:, 1].min(),
                                      pts[:, 0].max(), pts[:, 1].max())
                    cv2.rectangle(vis, (int(x0), int(y0)), (int(x1), int(y1)),
                                  (0, 200, 255), 3)
                    # 프레임 가장자리에 닿으면 보드가 잘리고 있다는 뜻
                    pad = 0.03
                    clipped = (x0 < w * pad or x1 > w * (1 - pad)
                               or y0 < h * pad or y1 > h * (1 - pad))
                    dist = ""
                    if fx:
                        side = np.mean([np.linalg.norm(c.reshape(4, 2)[k]
                                                       - c.reshape(4, 2)[(k + 1) % 4])
                                        for c in corners for k in range(4)])
                        dist = f"  ~{fx * MARKER_MM / side:.0f}mm"
                    if n_ids >= N_MARKERS - 1 and not clipped:
                        msg, color = f"OK  markers {n_ids}/{N_MARKERS}{dist}", (0, 230, 0)
                    elif clipped:
                        msg, color = (f"잘림 - 뒤로/중앙으로  markers {n_ids}/{N_MARKERS}{dist}",
                                      (0, 165, 255))
                    else:
                        msg, color = (f"일부만  markers {n_ids}/{N_MARKERS}{dist}",
                                      (0, 165, 255))
                else:
                    msg, color = "보드 없음", (0, 0, 255)

            # 화면 중심 십자 — 보드를 여기 맞춘다
            cv2.drawMarker(vis, (w // 2, h // 2), (255, 0, 255),
                           cv2.MARKER_CROSS, 60, 2)
            n += 1
            if time.time() - t0 > 1.0:
                fps, n, t0 = n / (time.time() - t0), 0, time.time()
            cv2.rectangle(vis, (0, 0), (w, 64), (0, 0, 0), -1)
            cv2.putText(vis, msg, (12, 44), cv2.FONT_HERSHEY_SIMPLEX, 1.1, color, 3)
            cv2.putText(vis, f"{fps:.1f} fps", (w - 190, 44),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (180, 180, 180), 2)

            if h > args.max_h:
                s = args.max_h / h
                vis = cv2.resize(vis, (int(w * s), args.max_h))
            cv2.imshow(WIN, vis)
            k = cv2.waitKey(1) & 0xFF
            if k in (ord("q"), 27):
                break
            if k == ord("s"):
                SNAP_DIR.mkdir(parents=True, exist_ok=True)
                p = SNAP_DIR / f"{datetime.now():%Y%m%d_%H%M%S}.png"
                cv2.imwrite(str(p), cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
                print(f"  저장: {p}")
    except KeyboardInterrupt:
        print("\n[live] Ctrl+C")
    finally:
        cv2.destroyAllWindows()
        try:
            sensor.shutdown()
        except Exception:                                      # noqa: BLE001
            pass
        print("[live] 종료")
    return 0


if __name__ == "__main__":
    sys.exit(main())
