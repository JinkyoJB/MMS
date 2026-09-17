"""debug_view.py — lookaround 디버깅용 **스캐너 시점 스냅샷** 저장 (sim·real 공용).

왜 필요한가
-----------
"왜 점이 안 들어왔나" 를 숫자로만 쫓으면 오래 걸린다. 2026-09-17 하루에만
`crop=0` 하나를 두고 IK·충돌·프레임 변환·관절 리밋을 차례로 배제하다가, 결국
원인은 **카메라가 딴 데를 보고 있었던 것**이었다. 그 한 장을 봤으면 바로 끝났다.

그래서 **스캐너가 그 순간 실제로 본 그림**을 남긴다. 저장 위치는 `output/debug/<단계>/`:

  output/debug/preview/      거리 탐색이 어디를 봤는가 (이동마다)
  output/debug/lookaround/   밴드 처음·중간·끝

단계마다 폴더를 나누는 이유 — preview 는 "왜 이 밴드 계획이 나왔나" 를 되짚을 때
제일 먼저 보는 그림인데, 밴드 스냅샷과 한 폴더에 섞이면 묻힌다.
파일명에는 언제·어디서 찍었는지가 들어간다.

sim·real 공용: 백엔드는 **이미지 배열만** 준다(`save`). 경로 규칙·파일명·오버레이는
여기서 정해서 두 쪽 그림이 같은 규칙으로 쌓이게 한다.

  sim  : Isaac 카메라 `get_rgba()`
  real : Artec preview 의 intensity (`capture_organized`)

끄려면 `MMS_DEBUG_VIEW=0`. 기본은 켬 — 디버깅 중이고, 한 장이 수십 KB 다.
"""
from __future__ import annotations

import os
import time
from typing import Optional

import numpy as np


def _enabled() -> bool:
    return os.environ.get("MMS_DEBUG_VIEW", "1") != "0"


class DebugViewSaver:
    """스캐너 시점 스냅샷을 순번 붙여 저장한다.

    `output/debug/<단계>/` 아래에 쌓는다(단계 미지정이면 lookaround). 실행마다
    지우지 않는다 — 직전 실행과 비교하는 것이 디버깅의 핵심이라서다. 대신 파일명 앞에 **실행
    타임스탬프**를 붙여 섞이지 않게 한다.
    """

    def __init__(self, root: str = None, run_ts: str = None, log=print):
        from utils import PROJECT_ROOT
        #: 단계별 폴더의 부모. `output/debug/<단계>/` 로 갈라진다.
        #  `MMS_DEBUG_DIR` 로 바꿀 수 있다 — testset 여러 종을 연달아 돌릴 때
        #  물체별로 폴더를 나누지 않으면 한 곳에 섞여 눈으로 못 본다.
        self._base = str(root or os.environ.get("MMS_DEBUG_DIR")
                         or (PROJECT_ROOT / "output" / "debug"))
        self.root = os.path.join(self._base, "lookaround")   # 단계 미지정 기본
        self.run_ts = run_ts or time.strftime("%H%M%S")
        self.log = log
        self.n = {}                       # 폴더마다 따로 센다 (파일명 순서 유지)
        self.enabled = _enabled()

    def _dir_for(self, stage):
        """이 단계의 저장 폴더. 만들지 못하면 None(그 저장만 건너뛴다)."""
        d = os.path.join(self._base, str(stage)) if stage else self.root
        try:
            os.makedirs(d, exist_ok=True)
        except OSError as e:                                  # noqa: BLE001
            self.log(f"[dbgview] ⚠ 폴더 생성 실패({d}: {e}) — 이 저장 건너뜀")
            return None
        return d

    # ── 내부 ────────────────────────────────────────────────────────────
    @staticmethod
    def _to_u8(img) -> Optional[np.ndarray]:
        """(H,W) / (H,W,3) / (H,W,4) 아무거나 → uint8 RGB."""
        a = np.asarray(img)
        if a.ndim == 2:
            a = np.stack([a] * 3, axis=-1)
        elif a.ndim == 3 and a.shape[2] == 4:
            a = a[:, :, :3]
        elif a.ndim != 3 or a.shape[2] != 3:
            return None
        if a.dtype != np.uint8:                     # float(0~1) 또는 0~255 둘 다 받는다
            m = float(np.nanmax(a)) if a.size else 1.0
            a = np.clip(a * (255.0 if m <= 1.0 + 1e-6 else 1.0), 0, 255)
            a = a.astype(np.uint8)
        return a

    # ── 공개 ────────────────────────────────────────────────────────────
    def save(self, img, tag: str, note: str = "",
             stage: str = None) -> Optional[str]:
        """스냅샷 저장. 반환 = 파일 경로(실패/비활성이면 None).

        `tag`   파일명에 들어가는 짧은 식별자 (`preview_d300_az0`, `band1_mid` 등)
        `note`  이미지 위에 얹는 한 줄 (점 수·거리 같은 그 순간의 숫자)
        `stage` 저장 위치 — `output/debug/<stage>/` (미지정이면 lookaround)
        """
        if not self.enabled:
            return None
        a = self._to_u8(img)
        if a is None:
            self.log(f"[dbgview] ⚠ 이미지 형식 알 수 없음 {np.asarray(img).shape} — 건너뜀")
            return None
        d = self._dir_for(stage)
        if d is None:
            return None
        n = self.n.get(d, 0) + 1
        self.n[d] = n
        safe = "".join(c if (c.isalnum() or c in "._-+") else "_" for c in tag)
        path = os.path.join(d, f"{self.run_ts}_{n:03d}_{safe}.png")
        try:
            from PIL import Image, ImageDraw
            im = Image.fromarray(a)
            if note:
                # 글자를 읽으려면 배경이 있어야 한다 — 어두운 띠 위에 흰 글씨.
                d = ImageDraw.Draw(im)
                d.rectangle([0, 0, im.width, 14], fill=(0, 0, 0))
                d.text((3, 2), note[:120], fill=(255, 255, 255))
            im.save(path)
        except Exception as e:                                # noqa: BLE001
            self.log(f"[dbgview] ⚠ 저장 실패({type(e).__name__}: {e})")
            return None
        return path

    def band_frames(self, n_frames: int):
        """밴드 안에서 **처음·중간·끝** 프레임 번호 집합. 캡처 루프가 이걸로 거른다."""
        if n_frames <= 0:
            return set()
        return {0, max(0, n_frames // 2), max(0, n_frames - 1)}
