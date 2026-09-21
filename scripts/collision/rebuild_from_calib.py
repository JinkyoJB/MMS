#!/usr/bin/env python
"""rebuild_from_calib.py — 캘리브 결과(`T_B_F0`)를 **충돌 모델까지** 반영한다.
`calibrate.py` 의 4단계.

왜 필요한가
-----------
`calibrate.py` 1~3 단계는 yaml 만 쓴다. 그런데 로봇이 실제로 무엇을 피할지는
**충돌 캐시(`utils/collision/data/cell_env.npz`)** 가 정한다. 턴테이블을
재캘리브해도 캐시는 옛 자리를 그대로 믿으므로, 게이트가 엉뚱한 자리를 검사한다.

  2026-09-19 에 로봇 시스템을 옮겼다. 턴테이블이 틀어졌다면 `T_B_F0` 는 새 값이
  되는데 캐시는 옛 값 그대로다 — 그 캐시가 바로 `home.py`·NBV 이동이 "부딪히나"를
  묻는 상대다.

두 갈래
-------
  A. **미세조정** (기본, 장비·Isaac 불요)
     캐시 안의 **턴테이블 점 뭉치만** 옛 자리 → 새 자리로 강체변환한다.
     턴테이블만 움직인 경우에 정확하다. 테이블·벽은 그대로 두는 것이 맞다.

  B. **전체 재빌드** (`--full`, Isaac Sim + USD 자산 필요)
     USD 씬을 `T_B_F0` 자리로 다시 만들어 통째로 굽는다. 셀 형상 자체가 바뀌었으면
     이쪽이다 — 다만 그 경우는 `bake_layout.py` 가 본래 도구다.

  A 가 가능하려면 "캐시가 **어느** 턴테이블 자리로 구워졌나"를 알아야 한다.
  `cell_env.meta.yaml` 의 `T_B_F0_at_bake` 가 그 값이고, `bake_layout.py` 가 적는다.
  meta 가 없으면 캐시에서 원판을 찾아 추정한다(원 피팅).

    python scripts/collision/rebuild_from_calib.py            # 미세조정
    python scripts/collision/rebuild_from_calib.py --full     # USD 재빌드
    python scripts/collision/rebuild_from_calib.py --check    # 확인만
"""
from __future__ import annotations

import argparse
import datetime as _dt
import os
import shutil
import subprocess
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO))

DATA = REPO / "utils" / "collision" / "data"
ACTIVE = DATA / "cell_env.npz"
META = DATA / "cell_env.meta.yaml"
STALE_MARK = DATA / "STALE_COLLISION.txt"

#: 캐시에서 **턴테이블 뭉치**로 볼 영역 (축 기준 원기둥, m).
#  실측 프로파일 근거: 원판 상면이 s≈−0.02, 하우징이 s≈+0.07 까지이고 반경은
#  ~0.19m 에서 끝난다. 그 아래(s>0.07)는 반경이 0.28m 로 튀는 **테이블**이라
#  같이 옮기면 안 된다.
TT_R_MAX, TT_S_LO, TT_S_HI = 0.20, -0.05, 0.075
#: 이 이상 옮겨야 할 때만 실제로 파일을 고친다 (m).
MIN_SHIFT_M = 0.002


# ── 축 읽기 ──────────────────────────────────────────────────────────────────
def calib_axis() -> tuple[np.ndarray, np.ndarray]:
    from utils.transforms import load_transform, TurntableTransformConfig
    tt = TurntableTransformConfig(load_transform(
        str(REPO / "config" / "calibration" / "turntable_frame.yaml"), "T_B_F0"))
    return (np.asarray(tt.axis_point_B, float).copy(),
            np.asarray(tt.axis_dir_B, float).copy())


def meta_axis():
    """캐시를 구울 때의 턴테이블 축. 없으면 None."""
    if not META.exists():
        return None
    import yaml
    m = yaml.safe_load(META.read_text(encoding="utf-8")) or {}
    a = m.get("T_B_F0_at_bake")
    if not a:
        return None
    return np.asarray(a["axis_point_B"], float), np.asarray(a["axis_dir_B"], float)


def estimate_axis_from_cache(E: np.ndarray, hint_c: np.ndarray,
                             hint_d: np.ndarray, r_rim: float):
    """meta 가 없을 때 — 캐시 점군에서 원판 rim 을 찾아 중심을 추정한다.

    ⚠ 이건 **추정**이다. rim 점을 고르는 고리(annulus)가 가정 중심을 따라가므로
      한 번만 피팅하면 힌트 쪽으로 끌린다 — 실측: 캐시가 60mm 떨어져 있는데
      7.8mm 만 떨어진 것으로 추정했다(그대로 쓰면 거의 못 고친다). 그래서
      **수렴할 때까지 재선택-재피팅** 하고, 그래도 호출부가 사후 검증으로
      확인한다. 정확한 값은 `bake_layout.py` 가 적는 meta 쪽이다.
    """
    a = np.array([1.0, 0.0, 0.0])
    if abs(float(a @ hint_d)) > 0.9:
        a = np.array([0.0, 1.0, 0.0])
    u = np.cross(a, hint_d); u /= np.linalg.norm(u)
    v = np.cross(hint_d, u)
    # 넓게 한 번만 잘라두고(원판 주변), 그 안에서 중심만 반복 갱신한다.
    P = E[np.linalg.norm(E - hint_c, axis=1) < 0.45]
    w = P - hint_c
    s = w @ hint_d
    band = (s > TT_S_LO) & (s < 0.04)
    X0, Y0 = (w @ u)[band], (w @ v)[band]
    if len(X0) < 200:
        return None
    cx, cy = 0.0, 0.0
    for _ in range(12):
        r = np.hypot(X0 - cx, Y0 - cy)
        m = (r > r_rim - 0.030) & (r < r_rim + 0.030)
        if int(m.sum()) < 200:
            return None
        X, Y = X0[m], Y0[m]
        A = np.column_stack([X, Y, np.ones(len(X))])
        sol, *_ = np.linalg.lstsq(A, X ** 2 + Y ** 2, rcond=None)
        nx, ny = sol[0] / 2.0, sol[1] / 2.0
        if np.hypot(nx - cx, ny - cy) < 1e-4:
            cx, cy = nx, ny
            break
        cx, cy = nx, ny
    return hint_c + cx * u + cy * v, hint_d.copy()


def blind_fraction(E: np.ndarray, c: np.ndarray, d: np.ndarray,
                   r_rim: float = 0.1215, margin: float = 0.025) -> float:
    """캘리브된 원판면 중 캐시에 여유 안쪽 물체가 없는 면적 비율 (%).

    `check_calibration.py` 와 같은 지표다 — 고치기 전후를 같은 잣대로 비교해
    **정말 나아졌는지** 확인하려고 여기에도 둔다.
    """
    from scipy.spatial import cKDTree
    a = np.array([1.0, 0.0, 0.0])
    if abs(float(a @ d)) > 0.9:
        a = np.array([0.0, 1.0, 0.0])
    u = np.cross(a, d); u /= np.linalg.norm(u)
    v = np.cross(d, u)
    rng = np.random.default_rng(0)
    th = rng.uniform(0, 2 * np.pi, 4000)
    rr = r_rim * np.sqrt(rng.uniform(0, 1, 4000))
    S = c + np.outer(rr * np.cos(th), u) + np.outer(rr * np.sin(th), v)
    near = E[np.linalg.norm(E - c, axis=1) < 0.45]
    if len(near) < 100:
        return 100.0
    return float((cKDTree(near).query(S)[0] > margin).mean() * 100.0)


# ── 미세조정 ─────────────────────────────────────────────────────────────────
def rot_between(d_from: np.ndarray, d_to: np.ndarray) -> np.ndarray:
    """두 단위벡터를 잇는 최소 회전 (Rodrigues)."""
    a = d_from / np.linalg.norm(d_from)
    b = d_to / np.linalg.norm(d_to)
    v = np.cross(a, b)
    c = float(a @ b)
    n = float(np.linalg.norm(v))
    if n < 1e-12:
        return np.eye(3) if c > 0 else -np.eye(3)
    K = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + K + K @ K * ((1 - c) / (n * n))


def fine_adjust(check_only: bool) -> int:
    if not ACTIVE.exists():
        print("  ✘ 충돌 캐시가 없다 — 먼저 scripts/collision/bake_layout.py")
        return 1
    c_new, d_new = calib_axis()
    E = np.asarray(np.load(ACTIVE)["env"], float)

    src = "meta"
    old = meta_axis()
    if old is None:
        import yaml
        r_rim = 0.1215
        try:
            y = yaml.safe_load((REPO / "config/calibration/turntable_frame.yaml")
                               .read_text(encoding="utf-8"))
            r_rim = float(y.get("rim_radius_mm", 121.5)) / 1000.0
        except Exception:                                    # noqa: BLE001
            pass
        old = estimate_axis_from_cache(E, c_new, d_new, r_rim)
        src = "캐시 추정(원 피팅)"
        if old is None:
            print("  ✘ 캐시에서 턴테이블을 못 찾았다 — 미세조정 불가")
            print("     전체 재빌드가 필요하다: --full  또는  bake_layout.py")
            return 1
    c_old, d_old = old

    dc = c_new - c_old
    ang = float(np.degrees(np.arccos(np.clip(d_old @ d_new, -1, 1))))
    print(f"  캐시 기준 축 ({src}) : 원점 {c_old.round(4)}")
    print(f"  캘리브  축            : 원점 {c_new.round(4)}")
    print(f"  차이                  : {np.linalg.norm(dc)*1000:.1f} mm · 축 {ang:.2f}°")

    if np.linalg.norm(dc) < MIN_SHIFT_M and ang < 0.5:
        print("  ✓ 이미 일치한다 — 고칠 것 없음")
        if not check_only:
            _record_meta(c_new, d_new)
            _clear_stale()
        return 0
    if check_only:
        print("  (--check 이므로 고치지 않는다)")
        return 0

    # 턴테이블 뭉치 선택 — **옛 축 기준**이다. 새 축으로 고르면 아직 안 옮긴
    # 점을 놓친다.
    w = E - c_old
    s = w @ d_old
    r = np.linalg.norm(w - np.outer(s, d_old), axis=1)
    sel = (r < TT_R_MAX) & (s > TT_S_LO) & (s < TT_S_HI)
    n = int(sel.sum())
    print(f"  턴테이블 점 {n:,}개 선택 (r<{TT_R_MAX*1000:.0f}mm, "
          f"s {TT_S_LO*1000:+.0f}~{TT_S_HI*1000:+.0f}mm)")
    if n < 500:
        print("  ✘ 선택된 점이 너무 적다 — 영역 가정이 안 맞는다. --full 을 쓸 것")
        return 1

    R = rot_between(d_old, d_new)
    out = E.copy()
    out[sel] = (E[sel] - c_old) @ R.T + c_new

    # ★ **사후 검증.** 옛 축을 meta 가 아니라 추정으로 얻었으면 틀릴 수 있다
    #   (실측: 60mm 어긋난 캐시를 7.8mm 로 추정 → 고쳐도 거의 그대로였다).
    #   고친 결과가 실제로 나아졌는지 같은 잣대로 확인하고, 아니면 **되돌린다.**
    #   충돌 캐시에서 "고쳤다고 믿었는데 아니었다" 가 가장 위험하다.
    before = blind_fraction(E, c_new, d_new)
    after = blind_fraction(out, c_new, d_new)
    print(f"  게이트 사각: {before:.1f}% → {after:.1f}%")
    if after > 1.0 and after > before - 1.0:
        print("  ✘ 충분히 나아지지 않았다 — 캐시를 고치지 않고 되돌린다")
        if src != "meta":
            print("     옛 축 추정이 빗나간 것으로 보인다. 정확히 하려면:")
            print("       · USD 자산 있는 PC 에서  rebuild_from_calib.py --full")
            print("       · 또는  bake_layout.py --scene <씬.usd>  로 다시 굽기")
        return 1

    bak = ACTIVE.with_suffix(".npz.bak")
    shutil.copy2(ACTIVE, bak)
    np.savez_compressed(ACTIVE, env=out.astype(np.float32))
    print(f"  ✓ 캐시 갱신 (백업 → {bak.name})")
    _record_meta(c_new, d_new)
    _clear_stale()
    return 0


def _record_meta(c: np.ndarray, d: np.ndarray) -> None:
    import yaml
    m = {}
    if META.exists():
        m = yaml.safe_load(META.read_text(encoding="utf-8")) or {}
    m["T_B_F0_at_bake"] = {"axis_point_B": [float(x) for x in c],
                           "axis_dir_B": [float(x) for x in d]}
    m["adjusted"] = _dt.date.today().isoformat()
    META.write_text(yaml.safe_dump(m, allow_unicode=True, sort_keys=False),
                    encoding="utf-8")


def _clear_stale() -> None:
    if STALE_MARK.exists():
        STALE_MARK.unlink()


def mark_stale(reason: str) -> None:
    """고치지 못했다는 사실을 **파일로** 남긴다 — 경고 한 줄은 스크롤에 묻힌다."""
    STALE_MARK.write_text(
        f"{_dt.date.today().isoformat()}\n{reason}\n"
        "해소: scripts/collision/rebuild_from_calib.py\n", encoding="utf-8")


# ── 전체 재빌드 ──────────────────────────────────────────────────────────────
def full_rebuild(alias: str, tilt: str) -> int:
    from scripts.collision.bake_layout import isaac_python
    py = isaac_python()
    if py is None:
        print("  ✘ Isaac Sim 파이썬이 없다 — --full 불가 (미세조정을 쓸 것)")
        return 2
    try:
        from mms_paths import asset
        src = Path(asset("frame_xarm7_spider_turntable/v2.usd"))
    except Exception as e:                                   # noqa: BLE001
        print(f"  ✘ 자산 경로 해석 실패: {e}")
        return 2
    if not src.exists():
        print(f"  ✘ USD 자산 없음: {src}")
        return 2

    usd_out = asset(f"frame_xarm7_spider_turntable/{alias}.usd")
    env = {**os.environ}; env.pop("PYTHONPATH", None)
    print("\n  [1/2] 씬 빌드 — T_B_F0 자리에 턴테이블 배치")
    if subprocess.run([str(py), str(REPO / "scripts/sim/build_scene_v2_real.py"),
                       "--out", str(usd_out), "--tilt", tilt],
                      cwd=REPO, env=env).returncode != 0:
        print("✘ 씬 빌드 실패"); return 1
    print("\n  [2/2] 베이크 + 활성화")
    return subprocess.run([sys.executable, str(REPO / "scripts/collision/bake_layout.py"),
                           "--scene", str(usd_out), "--alias", alias],
                          cwd=REPO).returncode


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--check", action="store_true", help="어긋남만 보고 고치지 않는다")
    ap.add_argument("--full", action="store_true",
                    help="USD 씬부터 전체 재빌드 (Isaac Sim + 자산 필요)")
    ap.add_argument("--alias", default=None, help="--full 의 레이아웃 별칭")
    ap.add_argument("--tilt", default="measured", choices=("measured", "level"))
    a = ap.parse_args()

    print("=" * 62)
    print("  [4] 충돌 모델 — 캘리브된 턴테이블 자리 반영")
    print("=" * 62)

    if a.full:
        alias = a.alias or f"v2_real_{_dt.date.today():%y%m%d}"
        rc = full_rebuild(alias, a.tilt)
        if rc == 2:
            mark_stale("--full 전제조건 없음 (Isaac/자산)")
        return rc

    rc = fine_adjust(a.check)
    if rc != 0 and not a.check:
        mark_stale("미세조정 실패")
    return rc


if __name__ == "__main__":
    raise SystemExit(main())
