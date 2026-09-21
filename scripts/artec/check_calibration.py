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

    # ★ 저장 쪽(`turntable_frame.save_turntable_frame_yaml`)이 쓰는 키는
    #   **`n_points`** 다. 여기서 `n_rim_points` 만 보던 탓에 항상 0 으로 읽혀
    #   점을 15개 찍어도 "rim 점 0개 — 다시 잡을 것" FAIL 이 났다(2026-09-16).
    #   구 파일 호환을 위해 옛 키도 같이 본다.
    npts = int(d.get("n_points", d.get("n_rim_points", 0)))
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

    # ★ `T_B_F0` 는 **B→F** 규약이다 (x_F = R·x_B + t). 따라서 `translation` 은
    #   턴테이블의 base 좌표가 **아니다.** F 원점(x_F=0)의 base 좌표는 −Rᵀ·t 다.
    #   2026-09-16 까지 이 값을 그대로 원점으로 써서, 실제로는 맞는 캘리브에도
    #   "그 자리에 아무것도 없다" FAIL 이 항상 났다 — 셀 모델을 의심하게 만든
    #   오진이었다(실측: translation [-0.862,0.080,-0.603] → 셀 점 0개,
    #   −Rᵀ·t [0.799,0.005,0.688] → 셀 점 25,097개).
    M = np.asarray(d["T_B_F0"]["matrix"], float)
    R_bf, t_bf = M[:3, :3], M[:3, 3]
    o = -R_bf.T @ t_bf                      # F 원점을 base 로
    axis = R_bf[2] / (np.linalg.norm(R_bf[2]) + 1e-12)      # F 의 +z 를 base 로
    print(f"\n  원점(base) = [{o[0]:+.3f}, {o[1]:+.3f}, {o[2]:+.3f}] m   "
          f"거리 {np.linalg.norm(o):.3f} m")
    _cross_check_cell(o, axis, float(d.get("rim_radius_mm", 121.5)) / 1000.0)


#: 충돌 여유 (m) — `CollisionModel(env_margin_m=0.025)` 와 같은 값.
ENV_MARGIN_M = 0.025
#: **게이트 사각 비율** 임계 (%). 캘리브된 원판면 중 충돌 캐시에 여유 안쪽
#  물체가 없는 면적 비율 = "실제 턴테이블인데 게이트가 못 보는 부분".
#
#  왜 거리(mm)가 아니라 이 비율인가 — 원판은 지름 243mm 평면이라 옆으로 50mm
#  밀려도 대부분의 점은 여전히 원판 위 어딘가에 가깝다. 거리 중앙값·p90 은
#  거의 안 변해서 옆이동을 못 잡는다(실측: 50mm 이동에 p90 13→17mm).
#  사각 비율은 밀린 쪽 초승달 부분을 직접 세므로 단조 증가한다
#  (0mm→0.0% · 30mm→0.1% · 50mm→4.0% · 80mm→15.8% · 120mm→35.9%).
CELL_BLIND_OK, CELL_BLIND_WARN = 1.0, 8.0


def _cross_check_cell(o: np.ndarray, axis: np.ndarray, r_rim: float) -> None:
    """★ 충돌 캐시가 **캘리브된 자리에** 턴테이블을 두고 있는가.

    캘리브가 말하는 원판면(상면 + 측벽)을 해석적으로 샘플링해, 그 점들이 캐시
    점군에서 얼마나 떨어져 있는지 잰다. "근처에 점이 몇 개 있나" 세는 것보다
    **어긋난 양을 mm 로** 주므로, 재빌드가 필요한지 바로 판단할 수 있다.

    왜 중요한가 — 캘리브는 yaml 만 쓰고 캐시는 USD 를 구워서 만든다. 턴테이블을
    옮기거나 재캘리브하면 둘이 갈라지는데, 로봇이 실제로 피하는 것은 **캐시** 다.
    """
    npz = _ROOT / "utils" / "collision" / "data" / "cell_env.npz"
    if not npz.exists():
        say(WARN, "충돌 캐시 없음 — 교차검증 생략"); return
    active = (_ROOT / "utils" / "collision" / "data" / "ACTIVE_LAYOUT.txt")
    tag = active.read_text(encoding="utf-8").strip() if active.exists() else "?"

    # 축에 수직인 정규직교 기저
    a = np.array([1.0, 0.0, 0.0])
    if abs(float(a @ axis)) > 0.9:
        a = np.array([0.0, 1.0, 0.0])
    u = np.cross(a, axis); u /= np.linalg.norm(u)
    v = np.cross(axis, u)

    rng = np.random.default_rng(0)
    th = rng.uniform(0, 2 * np.pi, 4000)
    rr = r_rim * np.sqrt(rng.uniform(0, 1, 4000))           # 상면 (균일 면적)
    top = o + np.outer(rr * np.cos(th), u) + np.outer(rr * np.sin(th), v)
    th = rng.uniform(0, 2 * np.pi, 2000)
    ss = rng.uniform(0.0, 0.02, 2000)                        # 측벽 20mm
    side = (o + r_rim * (np.outer(np.cos(th), u) + np.outer(np.sin(th), v))
            + np.outer(ss, axis))
    S = np.vstack([top, side])

    E = np.asarray(np.load(npz)["env"], float)
    near = E[np.linalg.norm(E - o, axis=1) < 0.45]
    if len(near) < 100:
        say(FAIL, f"활성 레이아웃 '{tag}' 에 그 자리 구조물이 없다 ({len(near)}점)",
            "T_B_F0 와 셀 모델 중 하나가 틀렸다 — "
            "scripts/collision/rebuild_from_calib.py")
        return
    from scipy.spatial import cKDTree
    dist = cKDTree(near).query(S)[0]
    blind = float((dist > ENV_MARGIN_M).mean() * 100.0)
    p99 = float(np.percentile(dist, 99) * 1000.0)
    print(f"  교차검증 — 활성 레이아웃 '{tag}' 기준 "
          f"게이트 사각 {blind:.1f}% · 원판면 어긋남 p99 {p99:.0f}mm")
    if blind < CELL_BLIND_OK:
        say(OK, "충돌 캐시가 캘리브된 턴테이블을 덮고 있다")
    elif blind < CELL_BLIND_WARN:
        say(WARN, f"턴테이블 {blind:.0f}% 가 게이트에 안 보인다 — 캐시 재빌드 권장",
            "scripts/collision/rebuild_from_calib.py")
    else:
        say(FAIL, f"턴테이블 {blind:.0f}% 가 게이트에 안 보인다 — 그쪽으로 지나가도 안 막힌다",
            "로봇을 움직이기 전에: scripts/collision/rebuild_from_calib.py")

    # 재빌드를 못 돌린 표식이 남아 있으면 반드시 짚는다.
    mark = _ROOT / "utils" / "collision" / "data" / "STALE_COLLISION.txt"
    if mark.exists():
        first = mark.read_text(encoding="utf-8").strip().splitlines()
        say(WARN, "충돌 모델 재빌드 미완료 표식이 남아 있다",
            " / ".join(first[:2]))


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
