"""preview 콜백 계약 검사 — **하드웨어·Isaac 없이 0.3초**에 끝난다.

왜 상설인가
-----------
같은 결함이 2026-09-17 하루에 **세 번** 나왔다:

  ① real `_preview_object_points_B` 의 return 5곳이 전부 맨 배열
     → real 은 적응 preview 가 **한 번도 안 돌았다**(늘 고정 격자).
  ② sim `preview_at` 의 빈 캡처·도달실패 갈래가 맨 배열 / `cam=None`
     → 적응이 **가장 필요한 순간에** 꺼졌다.
  ③ `_unpack_preview` 가 `cam is None` 하나로 "옛 계약"과 "이번엔 못 읽음"을
     겸함 → 한 프레임 실패가 **실행 전체**의 적응을 죽였고, 그 판정이
     `blind_probe` 앞이라 빈 캡처가 탐침에 도달조차 못 했다.
  ④ 거리격자 `d_steps` 가 sim (300,380) / real (240,300) 로 하드코딩돼 갈라짐
     → real 은 반경 40mm 만 넘으면 첫 스텝이 **근접한계 안쪽**이었다.
  ⑤ `min_pts` 가 선언만 되고 안 쓰임 → **1점만 들어와도 "유효"** 라서 노이즈
     중앙값으로 거리를 정하고, 탐침까지 건너뛰었다.
  ⑥ `blind_probe` 가 한쪽 한계에서 `None` → **양쪽 탐색이 같이 끝났다.**
     실효 범위가 220~380mm 로 잘려 상한 480mm 는 이름만 남았다.

셋 다 **조용히** 나빠진다 — 스캔은 끝나고 결과도 나오는데 거리만 안 맞는다.
그래서 눈으로는 못 잡고, 실물 세션을 태워야만 드러났다. 정적으로 잡는다.

    env -u PYTHONPATH python scripts/nbv/verify_preview_contract.py
"""
from __future__ import annotations

import ast
import io
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))

import numpy as np                                            # noqa: E402

from utils.nbv import lookaround as p1                        # noqa: E402
from utils.nbv.standoff import PREVIEW_RADIUS_MAX_M as PREVIEW_R_MAX  # noqa: E402

#: (파일, 함수) — `collect_planning_points` 에 넘기는 preview 콜백들.
CALLBACKS = [
    ("mms_artec/nbv/artec_multipass_scan_session.py", "_preview_object_points_B"),
    ("mms_artec/backends/isaac/isaac_scan_session.py", "preview_at"),
]


def check_returns() -> int:
    """모든 return 이 `(점군, 카메라위치)` 2-tuple 인가 (AST)."""
    bad = 0
    root = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..")
    for rel, fname in CALLBACKS:
        path = os.path.join(root, rel)
        fn = next((n for n in ast.walk(ast.parse(io.open(path, encoding="utf-8").read()))
                   if isinstance(n, ast.FunctionDef) and n.name == fname), None)
        if fn is None:
            print(f"  ✘ {fname} 를 못 찾음 ({rel}) — 이름이 바뀌었으면 이 목록도 고칠 것")
            bad += 1
            continue
        rets = [r for r in ast.walk(fn) if isinstance(r, ast.Return)]
        ok = [r for r in rets if isinstance(r.value, ast.Tuple) and len(r.value.elts) == 2]
        for r in rets:
            if r not in ok:
                print(f"  ✘ {rel}:{r.lineno} — 2-tuple 아님 (맨 배열이면 적응이 꺼진다)")
                bad += 1
        print(f"  {'✓' if len(ok) == len(rets) else '✘'} {fname}: "
              f"return {len(ok)}/{len(rets)} 곳이 (pts, cam)")
    return bad


def check_sentinel() -> int:
    """`cam=None` 이 '옛 계약'으로 오인되지 않는가 (동작)."""
    dof = (0.20, 0.30)
    axis, adir = np.zeros(3), np.array([0.0, 0.0, 1.0])
    r = 0.110              # 시작 290mm 가 표면 180mm — 근접한계 안쪽이라 반환 0

    def body(d, tz):
        if not (dof[0] <= d - r <= dof[1]):
            return np.zeros((0, 3))
        return (np.tile(np.array([r, 0.0, tz]), (300, 1))
                + np.random.default_rng(0).normal(0, 0.002, (300, 3)))

    seen = {"n": 0}

    def flaky(tz, d):      # 첫 프레임만 카메라 읽기 실패 (일시적 예외 모사)
        seen["n"] += 1
        b = body(d, tz)
        return (b, None) if seen["n"] == 1 else (b, np.array([d, 0.0, tz]))

    cases = [
        ("정상 (pts, cam)", lambda tz, d: (body(d, tz), np.array([d, 0.0, tz])), False),
        ("빈 캡처 때 cam=None",
         lambda tz, d: ((lambda b: (b, np.array([d, 0.0, tz]) if len(b) else None))(
             body(d, tz))), False),
        ("첫 프레임 cam=None", flaky, False),
        ("옛 계약 (맨 배열)", lambda tz, d: body(d, tz), True),
    ]
    bad = 0
    for name, cb, want_fixed in cases:
        seen["n"] = 0
        logs: list = []
        pts = p1.collect_planning_points(
            cb, lambda _th: True, axis, adir, sensor=p1.SensorModel(dof=dof),
            max_heights=1, thetas=(0.0,), log=logs.append)
        fixed = any("고정 거리격자" in L for L in logs)
        ok = (fixed == want_fixed) and (want_fixed or len(pts) > 0)
        bad += 0 if ok else 1
        print(f"  {'✓' if ok else '✘'} {name:22s} → "
              f"{'고정격자' if fixed else '적응'}, {len(pts)}점")
    return bad


def check_grid() -> int:
    """거리격자가 **창에서 유도**되는가 — 백엔드 하드코딩이 없는가."""
    from utils.nbv.standoff import preview_grid, probe_bounds
    bad = 0
    root = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..")
    # sim·real 이 같은 창을 주면 같은 격자가 나와야 한다 (공용 유도).
    for dof in [(0.20, 0.30), (0.17, 0.35)]:
        g = preview_grid(dof)
        lo, hi = probe_bounds(dof)
        near = float(dof[0])
        inside = [d for d in g if (d - PREVIEW_R_MAX) < near - 1e-9]
        ok = all(d >= near for d in g) and lo >= near - 1e-9
        bad += 0 if ok else 1
        print(f"  {'✓' if ok else '✘'} 창 {near*1000:.0f}~{dof[1]*1000:.0f}mm → "
              f"격자 {[round(d*1000) for d in g]}mm · 탐색 {lo*1000:.0f}~{hi*1000:.0f}mm")
        _ = inside
    # 백엔드에 거리격자가 다시 박히는 것을 막는다.
    for rel in ["mms_artec/backends/isaac/isaac_scan_session.py",
                "mms_artec/nbv/artec_multipass_scan_session.py"]:
        src = io.open(os.path.join(root, rel), encoding="utf-8").read()
        for node in ast.walk(ast.parse(src)):
            if not (isinstance(node, ast.keyword) and node.arg == "d_steps"):
                continue
            if isinstance(node.value, ast.Tuple):
                print(f"  ✘ {rel}:{node.value.lineno} — d_steps 가 하드코딩됐다 "
                      f"(창에서 유도되게 둘 것)")
                bad += 1
    if not bad:
        print("  ✓ 백엔드에 하드코딩된 거리격자 없음")
    return bad


def check_probe_sequence() -> int:
    """한쪽이 막혀도 **살아 있는 쪽으로 계속** 벌리는가."""
    from utils.nbv.standoff import blind_probe
    bad = 0
    for d0, lo, hi, want in [
        # 좁은 쪽(하한)이 먼저 소진 — 먼 쪽으로 계속 가야 한다
        (0.30, 0.20, 0.48, [260, 340, 220, 380, 420, 460]),
        # 양쪽 다 좁다 — 두 번이면 끝
        (0.30, 0.20, 0.30, [260, 220]),
        # 시작이 하한에 붙어 있다 — 사실상 먼 쪽만
        (0.25, 0.20, 0.48, [210, 290, 330, 370, 410, 450]),
    ]:
        seq, k = [], 1
        while k <= 40:
            v = blind_probe(d0, k, lo, hi)
            if v is None:
                break
            seq.append(round(v * 1000))
            k += 1
        ok = seq == want
        bad += 0 if ok else 1
        print(f"  {'✓' if ok else '✘'} 시작 {d0*1000:.0f}mm "
              f"한계 {lo*1000:.0f}~{hi*1000:.0f}mm → {seq}"
              + ("" if ok else f"   (기대 {want})"))
    return bad


def check_min_pts() -> int:
    """노이즈 몇 점이 '유효 관측'으로 오인되지 않는가."""
    dof = (0.20, 0.30)
    axis, adir = np.zeros(3), np.array([0.0, 0.0, 1.0])
    rng = np.random.default_rng(0)

    def noisy(tz, d):
        """어느 거리에서나 **자기점 3개만** 돌려준다 (물체는 안 보인다).

        그 3점은 카메라 바로 앞(80mm)에 있어, 믿으면 카메라가 뒤로 크게 물러난다.
        """
        cam = np.array([d, 0.0, tz])
        pts = np.tile(cam + np.array([-0.08, 0.0, 0.0]), (3, 1))
        return pts + rng.normal(0, 0.001, (3, 3)), cam

    logs: list = []
    pts = p1.collect_planning_points(
        noisy, lambda _th: True, axis, adir, sensor=p1.SensorModel(dof=dof),
        max_heights=1, thetas=(0.0,), log=logs.append)
    probed = sum(1 for L in logs if "탐침" in L or "소진" in L)
    ok = (len(pts) == 0) and probed > 0
    print(f"  {'✓' if ok else '✘'} 3점짜리 노이즈 캡처 → "
          f"누적 {len(pts)}점, 탐침 {probed}회 "
          f"{'(버리고 탐색 계속)' if ok else '(유효로 오인 — min_pts 미배선)'}")
    for L in logs:
        print(f"      {L}")
    return 0 if ok else 1


def main() -> None:
    print("preview 콜백 계약 (AST)")
    bad = check_returns()
    print("\n거리격자 유도 (sim·real 공용)")
    bad += check_grid()
    print("\n센티널 과적재 회귀 (동작)")
    bad += check_sentinel()
    print("\n노이즈 문턱 min_pts (동작)")
    bad += check_min_pts()
    print("\n반환 0 탐침 수열 (양쪽 브래킷)")
    bad += check_probe_sequence()
    print("\n" + ("✓ 전부 통과" if not bad else f"✘ 실패 {bad}건"))
    raise SystemExit(1 if bad else 0)


if __name__ == "__main__":
    main()
