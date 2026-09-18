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


def check_obstacle_envelope() -> int:
    """대상물 장애물이 **회전 불변**이고 자세를 안 막는가."""
    import math
    from utils.nbv.lookaround import revolution_envelope
    rng = np.random.default_rng(0)
    ax, ad = np.zeros(3), np.array([0.0, 0.0, 1.0])
    # 손잡이 달린 비대칭 물체 (몸통 r=34mm + 한쪽 돌출 r=90mm)
    t = rng.uniform(0, 2 * np.pi, 3000)
    P = np.vstack([
        np.c_[0.034 * np.cos(t), 0.034 * np.sin(t), rng.uniform(0, 0.20, 3000)],
        np.c_[rng.uniform(0.04, 0.09, 300), rng.uniform(-0.01, 0.01, 300),
              rng.uniform(0.10, 0.13, 300)]])
    E = revolution_envelope(P, ax, ad)
    bad = 0

    # ① 회전 불변 — 물체를 θ 만큼 돌려도 외피 안에 들어가야 한다
    worst = 0.0
    for deg in (17, 53, 91, 137, 210, 300):
        a = math.radians(deg)
        R = np.array([[math.cos(a), -math.sin(a), 0], [math.sin(a), math.cos(a), 0],
                      [0, 0, 1.0]])
        Q = P @ R.T
        # 각 점이 그 높이의 외피 반경 안에 있는가
        for z0, z1 in [(0.10, 0.13), (0.0, 0.20)]:
            m = (Q[:, 2] > z0) & (Q[:, 2] < z1)
            e = (E[:, 2] > z0) & (E[:, 2] < z1)
            if not m.any() or not e.any():
                continue
            rq = np.linalg.norm(Q[m, :2], axis=1).max()
            re = np.linalg.norm(E[e, :2], axis=1).max()
            worst = max(worst, rq - re)
    ok = worst <= 1e-6
    bad += 0 if ok else 1
    print(f"  {'✓' if ok else '✘'} 회전 불변 — 6개 각도에서 외피 밖으로 "
          f"{worst*1000:+.2f}mm (0 이하여야 함)")

    # ② 실제 쓰이는 자세를 기각하지 않는가
    #    ⚠ **3D 로 본다.** 수평거리만 비교하면 안 된다 — el 이 크면 카메라가
    #      물체 **위**에 있어서 수평거리가 반경보다 작아도 충돌이 아니다.
    #      충돌 게이트(SDF)도 3D 거리로 판정하므로 여기도 같아야 한다.
    from utils.collision.collision_model import SCAN_OBSTACLE_MARGIN_M as M
    z_top = float(P[:, 2].max())
    for name, el, d, tz in [("lookaround", 30, 0.284, 0.5 * z_top),
                            ("nbv", 50, 0.259, 0.5 * z_top),
                            ("flip/뚜껑", 70, 0.259, 0.9 * z_top)]:
        a = math.radians(el)
        tgt = np.array([0.0, 0.0, tz])
        eye = tgt + d * np.array([math.cos(a), 0.0, math.sin(a)])
        clear = float(np.linalg.norm(E - eye, axis=1).min())
        good = clear > M
        bad += 0 if good else 1
        print(f"  {'✓' if good else '✘'} {name:10s} el={el}° "
              f"카메라↔외피 {clear*1000:.0f}mm  (여유 {M*1000:.0f}mm 필요)")
    return bad


def check_evidence() -> int:
    """플래너가 "못 본 방위" 를 "없는 면" 으로 읽지 않는가 (2026-09-18 세제 회귀).

    원통을 하나는 전 방위, 하나는 카메라 쪽 반원만 남긴 점군으로 채점한다.
    반쪽 점군에서 minfill 이 0 으로 무너지면 회귀다 — 그것이 아랫밴드 전부를
    기각시킨 원인이었다. 전 방위 점군은 근거 없는 각도가 없어야 한다.
    """
    import math
    from utils.nbv import lookaround as p1
    th = np.linspace(0, 2 * math.pi, 720, endpoint=False)
    z = np.linspace(0.0, 0.10, 40)
    T, Z = np.meshgrid(th, z)
    r = 0.06
    full = np.column_stack([r * np.cos(T).ravel(), r * np.sin(T).ravel(), Z.ravel()])
    half = full[np.cos(T).ravel() > 0]            # 카메라(+X) 쪽 반원만
    S = p1.SensorModel()
    pose = p1.make_view_pose((0.0, 0.0), 0.05, 30.0, 0.0, 0.22 + r, +1.0)
    bad = 0
    for name, P in (("전 방위", full), ("반원만", half)):
        n = p1.estimate_outward_normals(P, (0.0, 0.0))
        ev = p1.evaluate_viewpoint(P, n, (0.0, 0.0), pose, S, n_theta=36)
        ok = ev.min_fill_cm2 > p1.FILL_HARD_MIN_CM2
        if name == "전 방위":
            ok = ok and ev.unknown_frac == 0.0
        else:
            ok = ok and 0.25 < ev.unknown_frac < 0.75   # 진실 50%, 섹터 폭만큼 과신 허용
        bad += 0 if ok else 1
        print(f"  {'✓' if ok else '✘'} {name}: minfill={ev.min_fill_cm2:.1f}cm² "
              f"근거없는 방위={ev.unknown_frac*100:.0f}%")
    return bad


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
    print("\npreview 가 못 본 방위 (없는 면으로 읽지 않는가)")
    bad += check_evidence()
    print("\n대상물 장애물 회전체 외피")
    bad += check_obstacle_envelope()
    print("\n" + ("✓ 전부 통과" if not bad else f"✘ 실패 {bad}건"))
    raise SystemExit(1 if bad else 0)


if __name__ == "__main__":
    main()
