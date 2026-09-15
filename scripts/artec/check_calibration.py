#!/usr/bin/env python
"""check_calibration.py — 캘리브레이션 결과가 말이 되는지 **읽기 전용** 검산.

장비도 로봇도 건드리지 않는다. yaml 세 개와 충돌 캐시만 읽는다.
캘리브를 돌린 뒤 "이 값을 믿어도 되나"를 혼자 판단할 수 있게 하는 것이 목적이다.

    python scripts/artec/check_calibration.py

판정: [OK] 통과 · [WARN] 의심스러움 · [FAIL] 쓰면 안 됨
"""
from __future__ import annotations

import sys
from datetime import date
from pathlib import Path

import numpy as np
import yaml

_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(_ROOT))

CFG = _ROOT / "config"
OK, WARN, FAIL = "[OK]  ", "[WARN]", "[FAIL]"
_n = {"ok": 0, "warn": 0, "fail": 0}


def say(level: str, msg: str, detail: str = "") -> None:
    _n["ok" if level is OK else "warn" if level is WARN else "fail"] += 1
    print(f"  {level} {msg}")
    if detail:
        print(f"         {detail}")


def _load(path: Path):
    try:
        return yaml.safe_load(path.read_text(encoding="utf-8"))
    except Exception as e:                                      # noqa: BLE001
        return {"__error__": f"{type(e).__name__}: {e}"}


def check_scanner_serial() -> None:
    """현재 연결된 스캐너와 캘리브 당시 스캐너가 같은가 — 다르면 T_EC 무효."""
    print("\n== 스캐너 동일성 ==")
    try:
        from mms_artec.sensor import artec_base
        artec_base._load()
        import artec_sdk_py
        found = artec_sdk_py.enumerate_scanners()
    except Exception as e:                                      # noqa: BLE001
        say(WARN, f"스캐너 조회 불가 ({type(e).__name__}) — 직접 확인할 것")
        return
    if not found:
        say(WARN, "스캐너 미연결 — 시리얼 대조를 건너뛴다")
        return
    serials = [s["serial"] for s in found]
    say(OK, f"연결된 스캐너: {', '.join(serials)}")
    print("         ※ T_EC 는 개체 종속이다. 캘리브 당시와 다른 시리얼이면 재캘리브.")


def check_hand_eye() -> np.ndarray | None:
    print("\n== T_EC (hand-eye) ==")
    d = _load(CFG / "sensor_frames.yaml")
    if "__error__" in d:
        say(FAIL, "sensor_frames.yaml 읽기 실패", d["__error__"]); return None
    e = d.get("T_EC_artec")
    if not e:
        say(FAIL, "T_EC_artec 없음"); return None
    t = np.asarray(e["translation"], float)
    n = float(np.linalg.norm(t))
    # 손목에 붙은 스캐너 — 플랜지에서 수 cm~20cm 안이어야 한다.
    if 0.03 <= n <= 0.35:
        say(OK, f"|t| = {n*1000:.1f} mm  (손목 장착으로 타당)")
    else:
        say(FAIL, f"|t| = {n*1000:.1f} mm — 손목 장착 스캐너로는 비현실적",
            "단위(m/mm) 혼동이나 잘못된 해를 의심")
    q = np.asarray(e.get("rotation_quat", [1, 0, 0, 0]), float)
    dq = abs(np.linalg.norm(q) - 1.0)
    say(OK if dq < 1e-3 else FAIL, f"quat norm 편차 {dq:.2e}")
    return t


def check_turntable() -> None:
    print("\n== T_B_F0 (턴테이블 축) ==")
    p = CFG / "calibration" / "turntable_frame.yaml"
    d = _load(p)
    if "__error__" in d:
        say(FAIL, "turntable_frame.yaml 읽기 실패", d["__error__"]); return

    npts = int(d.get("n_rim_points", 0))
    if npts >= 6:
        say(OK, f"rim 점 {npts}개")
    elif npts >= 4:
        say(WARN, f"rim 점 {npts}개 — 6개 이상 권장")
    else:
        say(FAIL, f"rim 점 {npts}개 — 3점은 원이 **항상** 정확히 지나간다",
            "residual 이 0 이어도 정확하다는 뜻이 아니다(미결정 피팅). 다시 잡을 것")

    res = d.get("rim_residual_mm")
    if res is None:
        say(WARN, "rim_residual_mm 없음")
    elif npts <= 3:
        say(WARN, f"residual {res} mm — 점이 3개라 의미 없는 값")
    elif res < 2.0:
        say(OK, f"residual {res} mm")
    elif res < 5.0:
        say(WARN, f"residual {res} mm — 5mm 넘으면 재작업")
    else:
        say(FAIL, f"residual {res} mm — 너무 크다")

    r = d.get("rim_radius_mm")
    if r is not None:
        say(OK if 100 <= r <= 135 else WARN,
            f"rim 반경 {r} mm  (실측 원판 반경 119mm 근처여야 한다)")

    when = str(d.get("date", ""))
    if when:
        try:
            age = (date.today() - date.fromisoformat(when)).days
            say(OK if age < 120 else WARN, f"측정일 {when} ({age}일 전)")
        except ValueError:
            say(WARN, f"측정일 파싱 실패: {when}")

    t = np.asarray(d["T_B_F0"]["translation"], float)
    print(f"\n  원점(base) = [{t[0]:+.3f}, {t[1]:+.3f}, {t[2]:+.3f}] m   "
          f"거리 {np.linalg.norm(t):.3f} m")
    _cross_check_cell(t)


def _cross_check_cell(t_bf0: np.ndarray) -> None:
    """★ 충돌 캐시(실측 셀)에 정말 그 자리에 턴테이블이 있는가.

    캘리브가 엉뚱한 곳을 가리키면 여기서 걸린다 — 두 독립 출처의 교차검증이다.
    """
    npz = _ROOT / "utils" / "collision" / "data" / "cell_env.npz"
    if not npz.exists():
        say(WARN, "충돌 캐시 없음 — 교차검증 생략"); return
    active = (_ROOT / "utils" / "collision" / "data" / "ACTIVE_LAYOUT.txt")
    tag = active.read_text(encoding="utf-8").strip() if active.exists() else "?"
    E = np.asarray(np.load(npz)["env"], float)
    d = np.linalg.norm(E - t_bf0, axis=1)
    near = int((d < 0.15).sum())
    print(f"  교차검증 — 활성 레이아웃 '{tag}' 에서 이 좌표 15cm 안의 셀 점: {near:,}개")
    if near > 5000:
        say(OK, "그 자리에 구조물이 있다 — 캘리브와 셀 모델이 일치")
    elif near > 0:
        say(WARN, f"점이 {near}개뿐 — 위치가 조금 어긋났거나 원판이 캐시에 없다")
    else:
        say(FAIL, "그 자리에 아무것도 없다",
            "T_B_F0 와 셀 모델 중 하나가 틀렸다. docs/4_collision.md §6 · "
            "docs/calibration_runbook.md §5 참고")


def main() -> int:
    print("=" * 62)
    print("  캘리브레이션 검산 — 읽기 전용 (장비 무동작)")
    print("=" * 62)
    check_scanner_serial()
    check_hand_eye()
    check_turntable()
    print("\n" + "=" * 62)
    print(f"  OK {_n['ok']}  ·  WARN {_n['warn']}  ·  FAIL {_n['fail']}")
    if _n["fail"]:
        print("  → FAIL 이 있으면 그 값으로 스캔하지 말 것.")
    print("=" * 62)
    return 1 if _n["fail"] else 0


if __name__ == "__main__":
    sys.exit(main())
