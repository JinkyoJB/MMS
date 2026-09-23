# scripts/artec/live_range_view.py
#
# 거리추종·nbv **디버그 이미지 라이브 뷰어 + 녹화**. main_artec.py 가 판정마다 떨구는
#   output/<RUN_TS>/debug/lookaround/s01_band1_live_f0012.png
#   output/<RUN_TS>/debug/nbv/nbv01_step03_th300.png
# 를 tail 하며 가장 최근 장을 창에 띄우고, 같은 순서로 동영상에도 쌓는다
#   output/<RUN_TS>/debug/range.avi          (규칙: utils/run_paths.py)
#
# 왜 이건 자식 프로세스로 띄워도 되나
# ---------------------------------
# `live_scan_view.py`(점군)는 Filament/Open3D 라 Popen 자식에서 GUI 가 죽었고
# 그래서 수동 2터미널로 남겨 뒀다. 이쪽은 **OpenCV HighGUI + 이미 저장된 PNG**
# 라 그 제약이 없다. main_artec.py 가 자동으로 띄운다(`--no-range-view` 로 끔).
#
# 조작
#   q / ESC     종료
#   SPACE       LIVE ↔ 정지(멈춰서 들여다보기)
#   ← / →       정지 상태에서 앞뒤 장 이동 (이동 전/후 비교용)
#   HOME / END  처음 / 마지막 장
#   (정지·되감기 중에도 녹화는 새 이미지를 순서대로 계속 담는다)
#
# 수동 실행
#   python scripts/artec/live_range_view.py                    # 가장 최근 run, 창만
#   python scripts/artec/live_range_view.py --record           # 창 + 녹화(avi)
#   python scripts/artec/live_range_view.py --run 20260922_170931 --make-video
#                                                              # 창 없이 지난 run 을 동영상으로

from __future__ import annotations

import argparse
import atexit
import re
import signal
import sys
import time
from pathlib import Path

import cv2
import numpy as np

# ★ 콘솔이 아닌 곳(파이프·파일)으로 출력이 가면 Python 은 **로케일 인코딩**(이 PC 는
#   cp949)을 쓴다. 안내문의 em-dash(—)가 cp949 에 없어서, main_artec 이 자동 실행한
#   뷰어가 첫 print 에서 UnicodeEncodeError 로 **즉시 죽었다**(2026-09-23 run_101718).
#   손으로 띄울 때는 PYTHONIOENCODING=utf-8 이 설정돼 있어 안 드러났다.
for _std in (sys.stdout, sys.stderr):
    try:
        _std.reconfigure(encoding="utf-8", errors="replace")
    except Exception:                                   # noqa: BLE001
        pass

_ROOT = Path(__file__).resolve().parents[2]
#: 창 제목에 RUN_TS 를 넣는다 — run 마다 창이 하나씩 생기므로 이게 없으면 **지난 run 의
#  창**(그 run 이 끝났으니 마지막 장에서 멈춰 있다)을 이번 것으로 착각한다(2026-09-22).
WIN_FMT = "MMS range debug [{label}]  (SPACE=pause  arrows=step  q=quit)"
POLL_S = 0.15
#: 새 이미지가 이만큼 안 오면 '그 run 은 끝났다' 로 보고 배너에 표시한다.
IDLE_FINISH_S = 45.0
#: 녹화 캔버스 — 이미지 크기가 장마다 다르다(텍스처가 붙으면 넓어지고, 없으면 좁다).
#  동영상은 한 크기로 고정해야 하므로 이 캔버스에 비율 유지로 넣고 남는 곳은 검게 둔다.
CANVAS_W, CANVAS_H = 980, 430


#: 뷰어가 보는 단계 폴더 — output/<RUN>/debug/<단계>/ (거리 이미지만. cam/·nbv_plan/ 은 형식이 달라 제외)
_STAGES = ("preview", "lookaround", "nbv", "flip")
_RUN_RE = re.compile(r"^\d{8}_\d{6}$")


def _debug_root(run: str) -> Path:
    return _ROOT / "output" / run / "debug"


def _pick_run(run: str | None) -> str | None:
    """RUN_TS 결정 — 지정 없으면 debug/ 가 있는 run 폴더 중 이름(=실행 일시)이 가장 늦은 것.
    (폴더 mtime 은 정리 스크립트·하위 폴더 생성으로 바뀌므로 믿지 않는다.)"""
    if run:
        return run
    cands = [p.name for p in (_ROOT / "output").glob("*")
             if p.is_dir() and _RUN_RE.match(p.name) and (p / "debug").is_dir()]
    return max(cands) if cands else None


def _dirs_for(run: str | None, explicit: Path | None) -> list[Path]:
    """그 run 의 **모든 단계 폴더** — preview·lookaround·nbv·flip. 아직 없는 단계는
    나중에 생기므로 폴더 목록은 매 폴링마다 다시 만든다(`_scan` 호출부 참조)."""
    if explicit is not None:
        return [explicit]
    if not run:
        return []
    return [d for d in (_debug_root(run) / s for s in _STAGES) if d.is_dir()]


def _scan(dirs: list[Path]) -> list[Path]:
    """관찰 대상 PNG 목록 (오래된 것부터). 저장 중인 파일은 건너뛴다."""
    out: list[tuple[float, Path]] = []
    for d in dirs:
        if not d.is_dir():
            continue
        for p in d.glob("*.png"):
            try:
                st = p.stat()
            except OSError:
                continue
            if st.st_size < 1024:          # 아직 쓰는 중
                continue
            out.append((st.st_mtime, p))
    out.sort(key=lambda t: t[0])
    return [p for _, p in out]


def _placeholder(msg: str, sub: str = "") -> np.ndarray:
    img = np.full((360, 750, 3), 24, np.uint8)
    cv2.putText(img, msg, (24, 170), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 200, 200), 1, cv2.LINE_AA)
    if sub:
        cv2.putText(img, sub, (24, 200), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (140, 140, 140), 1, cv2.LINE_AA)
    return img


def _banner(img: np.ndarray, idx: int, n: int, name: str, live: bool, rec: bool = False,
            idle_s: float = 0.0) -> np.ndarray:
    """아래에 상태 줄을 덧붙인다 — 원본 이미지를 가리지 않게."""
    bar = np.full((22, img.shape[1], 3), 40, np.uint8)
    done = live and idle_s >= IDLE_FINISH_S
    tag = ("FINISHED" if done else "LIVE") if live else "PAUSED"
    col = ((150, 150, 150) if done else (90, 220, 90)) if live else (90, 180, 255)
    # ★ 배너는 **ASCII 만** — OpenCV putText 는 한글을 그리지 못하고 '?' 로 찍는다.
    if done:
        tag += f" (no new img {idle_s / 60:.0f}m)"
    left = f"{tag}  {idx + 1}/{n}"
    cv2.putText(bar, left, (6, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.45, col, 1, cv2.LINE_AA)
    # 파일명은 왼쪽 글자 폭을 재서 그 뒤에 — 겹치지 않게.
    (tw, _), _b = cv2.getTextSize(left, cv2.FONT_HERSHEY_SIMPLEX, 0.45, 1)
    cv2.putText(bar, name, (16 + tw, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.42,
                (210, 210, 210), 1, cv2.LINE_AA)
    if rec:
        cv2.circle(bar, (img.shape[1] - 46, 11), 5, (60, 60, 240), -1)
        cv2.putText(bar, "REC", (img.shape[1] - 36, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.42,
                    (60, 60, 240), 1, cv2.LINE_AA)
    return np.vstack([img, bar])


def _fit(img: np.ndarray, w: int = CANVAS_W, h: int = CANVAS_H) -> np.ndarray:
    """비율 유지로 캔버스 안에 넣는다 (동영상은 크기가 고정이어야 한다)."""
    ih, iw = img.shape[:2]
    s = min(w / iw, h / ih)
    if s < 1.0:
        img = cv2.resize(img, (max(1, int(iw * s)), max(1, int(ih * s))), interpolation=cv2.INTER_AREA)
        ih, iw = img.shape[:2]
    out = np.zeros((h, w, 3), np.uint8)
    y, x = (h - ih) // 2, (w - iw) // 2
    out[y:y + ih, x:x + iw] = img
    return out


class _Recorder:
    """PNG 시퀀스를 동영상으로.

    `live=True`(run 과 함께 녹화)면 **AVI/MJPG 를 먼저** 고른다. 라이브 녹화는
    창을 닫지 않고 강제 종료될 수 있는데, mp4 는 그때 moov atom 이 안 쓰여
    **통째로 재생 불가**가 된다(실측 44바이트 / "moov atom not found").
    AVI/MJPG 는 마무리 없이 잘려도 쓴 프레임이 전부 읽힌다(실측 20/20).
    `--make-video`(사후 제작)는 끝까지 도니 공유하기 좋은 mp4 를 먼저 쓴다.
    """

    def __init__(self, path: Path, fps: float, live: bool = False):
        self.path = path
        self.fps = float(fps)
        self.live = bool(live)
        self.w = None
        self.n = 0
        self._err = None
        path.parent.mkdir(parents=True, exist_ok=True)

    def _open(self):
        order = ((("MJPG", ".avi"), ("mp4v", ".mp4")) if self.live
                 else (("mp4v", ".mp4"), ("MJPG", ".avi")))
        for fourcc, suffix in order:
            p = self.path.with_suffix(suffix)
            w = cv2.VideoWriter(str(p), cv2.VideoWriter_fourcc(*fourcc),
                                self.fps, (CANVAS_W, CANVAS_H))
            if w.isOpened():
                self.path = p
                return w
            w.release()
        return None

    def add(self, img: np.ndarray) -> None:
        if self._err:
            return
        if self.w is None:
            self.w = self._open()
            if self.w is None:
                self._err = "VideoWriter 를 열 수 없음 (코덱 없음)"
                print(f"[range-view] ⚠ 녹화 불가: {self._err}")
                return
            print(f"[range-view] 녹화 시작 → {self.path}")
        self.w.write(_fit(img))
        self.n += 1

    def close(self, *_a) -> None:
        # ★ 강제 종료(TerminateProcess)면 이게 안 불리고 mp4 가 **마무리되지 않아
        #   재생 불가**가 된다(moov atom 미기록, 실측 44바이트). atexit·시그널로
        #   최대한 붙잡되, 그래도 놓치면 PNG 는 그대로 남아 있으니
        #   `--make-video` 로 언제든 다시 만들면 된다.
        if self.w is not None:
            self.w.release()
            self.w = None
            secs = self.n / max(self.fps, 1e-6)
            print(f"[range-view] 녹화 저장 — {self.n} 장 / {secs:.0f}s → {self.path}")


def _stamp(img: np.ndarray, name: str, i: int, n: int) -> np.ndarray:
    """녹화용 자막 — 동영상만 봐도 어느 파일인지 알 수 있게."""
    return _banner(img, i, n, name, live=True)


def main() -> int:
    ap = argparse.ArgumentParser(description="MMS 거리추종/nbv 디버그 이미지 라이브 뷰어 + 녹화")
    ap.add_argument("--run", default=None, help="RUN_TS (없으면 가장 최근)")
    ap.add_argument("--dir", default=None, help="이 폴더만 본다 (RUN_TS 무시)")
    ap.add_argument("--wait", type=float, default=0.0,
                    help="폴더가 아직 없을 때 이만큼(초) 기다린다. 0=무한")
    ap.add_argument("--record", action="store_true",
                    help="본 순서대로 동영상으로도 저장 (output/<RUN_TS>/debug/range.avi). "
                         "run 이 끝난 뒤 --make-video 로 다시 만들어도 된다 — PNG 가 원본이다")
    ap.add_argument("--record-path", default=None, help="동영상 경로를 직접 지정")
    ap.add_argument("--video-fps", type=float, default=4.0, help="동영상 fps (기본 4)")
    ap.add_argument("--make-video", action="store_true",
                    help="창 없이 **이미 있는** 이미지들만 동영상으로 만들고 끝낸다 (지난 run 용)")
    ap.add_argument("--idle-exit", type=float, default=0.0, metavar="SEC",
                    help="새 이미지가 이만큼 안 오면 창을 닫는다 (0=안 닫음). "
                         "run 마다 창이 쌓이는 게 싫을 때")
    a = ap.parse_args()

    explicit = Path(a.dir).resolve() if a.dir else None
    run = None if explicit else _pick_run(a.run)
    dirs = _dirs_for(run, explicit)
    label = explicit.name if explicit else (run or "?")
    if dirs:
        print("[range-view] 감시: " + " · ".join(str(d) for d in dirs))
    else:
        print("[range-view] 감시 대상 없음 — run 이 시작되면 잡는다")

    rec = None
    if a.record or a.make_video or a.record_path:
        rp = Path(a.record_path) if a.record_path else (_debug_root(label) / "range.mp4")
        rec = _Recorder(rp, a.video_fps, live=not a.make_video)
        atexit.register(rec.close)
        for _sig in ("SIGTERM", "SIGINT", "SIGBREAK"):
            _s = getattr(signal, _sig, None)
            if _s is not None:
                try:
                    signal.signal(_s, lambda *_a: sys.exit(0))
                except (ValueError, OSError):        # 메인 스레드가 아니거나 미지원
                    pass

    # ── 오프라인: 지난 run 을 동영상으로만 ────────────────────────────────
    if a.make_video:
        files = _scan(dirs)
        if not files:
            print("[range-view] ✘ 이미지가 없다 — RUN_TS 를 확인할 것")
            return 1
        for i, p in enumerate(files):
            img = cv2.imread(str(p))
            if img is not None:
                rec.add(_stamp(img, p.name, i, len(files)))
        rec.close()
        return 0

    win = WIN_FMT.format(label=label)
    cv2.namedWindow(win, cv2.WINDOW_AUTOSIZE)
    files: list[Path] = []
    cur = -1                      # -1 = LIVE(마지막 장)
    rec_n = 0                     # 녹화에 이미 담은 장 수 (표시와 무관하게 순서대로)
    shown: Path | None = None
    t_start = time.time()
    t_new = time.time()           # 마지막으로 **새 이미지**를 본 시각
    while True:
        if not dirs:              # run 을 아직 못 정했으면 계속 찾아본다
            run = _pick_run(a.run)
            dirs = _dirs_for(run, explicit)
            if run and run != label:
                label = run
                _w2 = WIN_FMT.format(label=label)     # run 을 이제 알았으니 제목 갱신
                try:
                    cv2.destroyWindow(win)
                except cv2.error:
                    pass
                win = _w2
                cv2.namedWindow(win, cv2.WINDOW_AUTOSIZE)
            if run:
                if rec is not None and rec.n == 0 and a.record_path is None:
                    rec.path = _debug_root(label) / "range.mp4"
        if run and explicit is None:
            dirs = _dirs_for(run, None)      # 단계가 넘어가면 폴더가 새로 생긴다
        _n_before = len(files)
        files = _scan(dirs)
        if len(files) > _n_before:
            t_new = time.time()
        idle = time.time() - t_new
        # 녹화는 **표시와 독립**으로 새 장을 순서대로 담는다 (멈춰 봐도 기록은 이어진다).
        if rec is not None and len(files) > rec_n:
            for i in range(rec_n, len(files)):
                img = cv2.imread(str(files[i]))
                if img is not None:
                    rec.add(_stamp(img, files[i].name, i, len(files)))
            rec_n = len(files)
        if files:
            idx = len(files) - 1 if cur < 0 else max(0, min(cur, len(files) - 1))
            path = files[idx]
            if path != shown or cur < 0:
                img = cv2.imread(str(path))
                if img is not None:
                    cv2.imshow(win, _banner(img, idx, len(files), path.name, cur < 0,
                                            rec=rec is not None, idle_s=idle))
                    shown = path
            elif cur < 0 and idle >= IDLE_FINISH_S and int(idle) % 5 == 0:
                img = cv2.imread(str(path))           # 배너만 갱신 (FINISHED 표시)
                if img is not None:
                    cv2.imshow(win, _banner(img, idx, len(files), path.name, True,
                                            rec=rec is not None, idle_s=idle))
        else:
            waited = time.time() - t_start
            cv2.imshow(win, _placeholder(
                f"{label}: 디버그 이미지 대기 중 ... ({waited:.0f}s)",
                "main_artec.py 가 lookaround 밴드에 들어가면 자동으로 뜬다"))
            if a.wait and waited > a.wait:
                print("[range-view] 대기 시간 초과 — 종료")
                break
        if a.idle_exit and files and idle > a.idle_exit:
            print(f"[range-view] 새 이미지 {idle:.0f}s 없음 — 종료 (--idle-exit)")
            break

        k = cv2.waitKeyEx(int(POLL_S * 1000))
        if k in (27, ord("q"), ord("Q")):
            break
        if k == ord(" "):
            cur = (len(files) - 1) if cur < 0 else -1
        elif k in (2424832, 65361, ord("a")):          # ←
            cur = (len(files) - 1 if cur < 0 else cur) - 1
            cur = max(0, cur)
        elif k in (2555904, 65363, ord("d")):          # →
            cur = (len(files) - 1 if cur < 0 else cur) + 1
            cur = min(len(files) - 1, cur) if files else -1
        elif k in (2359296, 65360):                    # HOME
            cur = 0
        elif k in (2293760, 65367):                    # END
            cur = -1
        try:                                            # 창을 X 로 닫았나
            if cv2.getWindowProperty(win, cv2.WND_PROP_VISIBLE) < 1:
                break
        except cv2.error:
            break
    cv2.destroyAllWindows()
    if rec is not None:
        rec.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
