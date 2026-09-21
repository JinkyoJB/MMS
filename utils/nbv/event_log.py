"""event_log.py — run 이벤트(JSONL) 기록. tracking-lost 분석용.

왜
--
콘솔 로그는 사람이 읽는 것이라 "언제 잃었고, 뭘 했고, 됐나" 를 run 여러 개에 걸쳐
세려면 매번 grep 을 다시 짜야 했다(2026-09-21 여섯 run 을 그렇게 봤다). 그래서
판단이 일어나는 자리마다 **구조화된 한 줄**을 남긴다 — `output/events_<RUN_TS>.jsonl`.
집계는 `scripts/artec/lost_report.py`.

이벤트 종류 (`kind`)
    lost            추적 상실 판정          stage, band, n_bands, theta_deg, frames_ok, reason
    band_move       밴드 전환 시작/결과     from_band, to_band, steps, step_mm, outcome, lost_alpha…
    band_partial    밴드 부분 성공 처리     done, total, next
    recovery        자동 복구 시도/결과     attempt, ok, pose_changed
    rotation_ok     회전 정상 완료          stage, band…
    merge           master 병합             stage, pose_idx, method(hint|greg|camera), n_scans
    user            사용자 입력             prompt, key

절대 파이프라인을 깨지 않는다 — 기록 실패는 조용히 무시.
"""
from __future__ import annotations

import json
import os
import time
from pathlib import Path

_T0 = time.perf_counter()
_PATH = None


def _path() -> Path:
    global _PATH
    if _PATH is None:
        root = Path(__file__).resolve().parents[2]
        tag = os.environ.get("MMS_RUN_TS") or time.strftime("%Y%m%d_%H%M%S")
        _PATH = root / "output" / f"events_{tag}.jsonl"
    return _PATH


def _clean(v):
    try:
        import numpy as np
        if isinstance(v, np.generic):
            return v.item()
        if isinstance(v, np.ndarray):
            return v.tolist()
    except Exception:                                   # noqa: BLE001
        pass
    if isinstance(v, (list, tuple)):
        return [_clean(x) for x in v]
    if isinstance(v, dict):
        return {str(k): _clean(x) for k, x in v.items()}
    if isinstance(v, float):
        return round(v, 4)
    return v


def log_event(kind: str, **fields) -> None:
    """한 줄 기록. 예: `log_event("lost", stage="lookaround", band=2, reason=...)`."""
    try:
        rec = {"t": round(time.perf_counter() - _T0, 2),
               "wall": time.strftime("%H:%M:%S"), "kind": str(kind)}
        rec.update({k: _clean(v) for k, v in fields.items()})
        p = _path()
        p.parent.mkdir(parents=True, exist_ok=True)
        with open(p, "a", encoding="utf-8") as fh:
            fh.write(json.dumps(rec, ensure_ascii=False) + "\n")
    except Exception:                                   # noqa: BLE001
        pass
