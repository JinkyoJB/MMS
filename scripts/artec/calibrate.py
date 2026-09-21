#!/usr/bin/env python3
"""calibrate.py — 실물 캘리브레이션 **단일 진입점**. 순서를 강제한다.

실행 순서가 왜 이런가
--------------------
`turntable_calib.py` 는 rim 점을 **base 로 변환한 뒤** 원을 피팅한다.
그 변환 `T_CB = T_EB · inv(T_EC)` 에 **`T_EC` 가 들어가므로** hand-eye 가 먼저다.

  ※ 이는 **현재 구현 기준**이다. 클릭 자체는 카메라 프레임 점을 주므로, 카메라
    프레임에서 피팅해 턴테이블을 hand-eye 타깃으로 쓰면 둘을 동시에 풀 수도 있다
    (5-DOF 타깃, 미구현 — docs/1_calibration.md T10).

전체 순서
--------
  0. 연결 확인 + **사람이 수동으로 조준** (보드·턴테이블이 카메라에 들어오게)
  1. intrinsic  — 카메라 K (최초 1회, 렌즈/센서 바뀌면 다시)
  2. hand-eye   — T_EC  → config/sensor_frames.yaml
  3. turntable  — T_B_F0 → config/calibration/turntable_frame.yaml
  4. 충돌 모델  — T_B_F0 를 셀 캐시까지 반영 → utils/collision/data/cell_env.npz

  ※ 보드를 **턴테이블 원판 위에** 올려두면 2→3 을 같은 조준 자세에서 이어서 할 수 있다.
  ※ 4 단계가 왜 캘리브에 포함되나 — 로봇이 실제로 무엇을 피할지는 yaml 이 아니라
    **충돌 캐시**가 정한다. 3 단계만 하고 멈추면 캐시는 옛 턴테이블 자리를 믿는다.
  ※ 셀 형상(테이블·벽·로봇 마운트)이 바뀐 경우는 4 단계로 안 된다 →
    scripts/collision/bake_layout.py (USD 를 다시 굽는다)

사용
----
  conda activate mms-env
  env -u PYTHONPATH python scripts/artec/calibrate.py              # 전체
  env -u PYTHONPATH python scripts/artec/calibrate.py --from 2     # 2단계부터
  env -u PYTHONPATH python scripts/artec/calibrate.py --only 3     # 3단계만

  각 단계는 기존 스크립트를 그대로 호출한다(로직 중복 없음):
    scripts/artec/intrinsic_calib.py / hand_eye_calib.py / turntable_calib.py

sim 에서 미리 확인하려면 → `scripts/sim/calib_handeye_sim.py`,
`scripts/sim/calib_rim_sim.py` (docs/1_calibration.md §4)
"""
from __future__ import annotations

import argparse
import os
import subprocess
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO))

from scripts.artec._step_guard import ENV_FLAG as STEP_GUARD_ENV

STEPS = [
    (1, "intrinsic  — 카메라 K",      "scripts/artec/intrinsic_calib.py",
     "config/calibration/artec_intrinsic.yaml"),
    (2, "hand-eye   — T_EC",          "scripts/artec/hand_eye_calib.py",
     "config/sensor_frames.yaml::T_EC_artec  (직접 갱신)"),
    (3, "turntable  — T_B_F0",        "scripts/artec/turntable_calib.py",
     "config/calibration/turntable_frame.yaml"),
    # ★ 4 단계가 없으면 사슬이 여기서 끊긴다. yaml 은 새 값인데 로봇이 실제로
    #   피하는 **충돌 캐시**는 옛 턴테이블 자리를 그대로 믿는다 — 그 상태로
    #   움직이면 게이트가 엉뚱한 자리를 검사한다. 캐시의 **턴테이블 점만** 새
    #   자리로 옮기므로 장비도 Isaac 도 필요 없다. 못 고치면 말없이 넘어가지 않고
    #   어긋남 표식을 남긴다. 셀 형상 자체가 바뀐 경우는 bake_layout.py.
    (4, "충돌 모델   — cell_env.npz", "scripts/collision/rebuild_from_calib.py",
     "utils/collision/data/cell_env.npz  (활성 레이아웃)"),
]

AIM_GUIDE = """
──────────────────────────────────────────────────────────────
 0. 수동 조준  (사람이 하는 단계)
──────────────────────────────────────────────────────────────
  · ChArUco 보드를 **턴테이블 원판 위**에 평평하게 올린다
  · 로봇을 움직여 보드와 원판 rim 이 스캐너 카메라에 **함께** 들어오게 한다
  · 이 자세에서 2~3 단계를 이어서 수행한다

  로봇 이동:  scripts/artec/go_home.py 로 홈 복귀 후 수동 조작
  연결 확인:  ping 192.168.1.210
──────────────────────────────────────────────────────────────
"""


POSES_YAML = REPO / "config" / "calibration" / "artec_calibration_poses.yaml"


def _poses_path_checked() -> bool:
    """자세 목록이 **자세↔자세 경로 검사**를 거쳤나 (`gen_calib_poses.py` 표식).

    통과했으면 home 을 경유하지 않고 이어서 가도 안전하다. 표식이 없으면
    옛 목록이거나 손으로 만든 것이라 경로가 보장되지 않는다 — 그때는 느려도
    home 을 경유한다. 레이아웃이 바뀌었으면 검사 결과도 무효이므로 같이 본다.
    """
    try:
        import yaml
        d = yaml.safe_load(POSES_YAML.read_text(encoding="utf-8")) or {}
    except Exception as e:                                   # noqa: BLE001
        print(f"  [poses] 읽기 실패({type(e).__name__}) — home 경유")
        return False
    if not d.get("path_checked"):
        print(f"  [poses] 경로 검사 표식 없음 — home 을 경유한다 (느림)")
        print(f"          빠르게 하려면: python scripts/artec/gen_calib_poses.py "
              f"--from-view --write")
        return False
    tag = _active_layout()
    if d.get("layout") not in (None, "?", tag):
        print(f"  [poses] 검사 당시 레이아웃 '{d.get('layout')}' ≠ 현재 '{tag}' "
              f"— home 을 경유한다")
        return False
    print(f"  [poses] 경로 검사 완료 ({d.get('generated','?')}, "
          f"자세간 {d.get('max_step_deg','?')}°) — home 경유 생략")
    return True


def _active_layout() -> str:
    try:
        return (REPO / "utils" / "collision" / "data" / "ACTIVE_LAYOUT.txt"
                ).read_text(encoding="utf-8").strip() or "?"
    except OSError:
        return "?"


def run(step_no: int, title: str, rel: str, out: str, extra: list[str]) -> bool:
    print(f"\n{'='*66}\n  [{step_no}/{len(STEPS)}] {title}\n  → {out}\n{'='*66}")
    cmd = [sys.executable, str(REPO / rel), *extra]
    print(f"  $ {' '.join(cmd)}\n")
    # 자식에게 "드라이버가 부른 것" 표식을 준다 — 개별 스크립트의 순서 안내
    # (`_step_guard.warn_if_direct`)가 여기서는 뜨지 않게.
    env = {**os.environ, STEP_GUARD_ENV: "1"}
    r = subprocess.run(cmd, cwd=REPO, env=env)
    # ★ 4 단계만 exit 2 = "전제조건 없음" 을 따로 취급한다. 캘리브 값 자체는
    #   이미 제대로 나왔으므로 전체를 실패로 만들 이유가 없다. 대신 표식이
    #   남고 `check_calibration.py` 가 계속 짚으므로 조용히 묻히지 않는다.
    if step_no == 4 and r.returncode == 2:
        print(f"\n⚠ [{step_no}] 건너뜀 — 충돌 모델이 **옛 턴테이블 자리**로 남았다.")
        print("   자산 있는 PC 에서: python scripts/collision/rebuild_from_calib.py")
        return True
    if r.returncode != 0:
        print(f"\n✘ [{step_no}] 실패 (exit {r.returncode}) — 여기서 중단한다.")
        return False
    print(f"\n✓ [{step_no}] 완료")
    return True


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--from", dest="start", type=int, default=1, choices=(1, 2, 3, 4),
                    help="이 단계부터 실행 (기본 1)")
    ap.add_argument("--only", type=int, choices=(1, 2, 3, 4), help="이 단계만 실행")
    ap.add_argument("--skip-aim", action="store_true", help="수동 조준 안내 생략")
    ap.add_argument("--via-home", action="store_true",
                    help="자세마다 home 을 경유한다 (느리다). 기본은 경유하지 않는 "
                         "것이고, 자세 목록에 경로 검사 표식이 없으면 자동으로 "
                         "경유하므로 보통 줄 필요가 없다")
    ap.add_argument("rest", nargs=argparse.REMAINDER,
                    help="-- 뒤의 인자는 해당 스크립트로 전달 (--only 와 함께 쓸 것)")
    args = ap.parse_args()

    extra = [a for a in args.rest if a != "--"]
    if extra and args.only is None:
        print("✘ 추가 인자는 --only 와 함께만 쓸 수 있다."); return 2

    todo = [s for s in STEPS if (s[0] == args.only if args.only else s[0] >= args.start)]

    if not args.skip_aim and (args.only is None or args.only >= 2):
        print(AIM_GUIDE)
        if input("  조준을 마쳤으면 Enter (중단은 q): ").strip().lower() == "q":
            return 1

    for no, title, rel, out in todo:
        step_extra = list(extra) if args.only == no else []
        # 3단계(turntable)는 rim 클릭이라 자세 순회가 없다 — 플래그가 없다.
        # ★ home 경유는 **기본 끔**이다 — 1·2단계가 3배 이상 빨라진다.
        #   단, 자세 목록이 경로 검사를 거쳤을 때만 안전하다. `gen_calib_poses.py`
        #   가 남긴 `path_checked` 표식을 보고 자동으로 정한다. 표식이 없으면
        #   (옛 목록·손으로 만든 목록) 조용히 빠르게 가지 않고 home 을 경유한다.
        if no in (1, 2) and (args.via_home or not _poses_path_checked()):
            step_extra.append("--via-home")
        if not run(no, title, rel, out, step_extra):
            return 1

    print(f"\n{'='*66}\n  캘리브레이션 완료\n{'='*66}")
    print("  확인:  config/sensor_frames.yaml               (T_EC_artec)")
    print("        config/calibration/turntable_frame.yaml  (T_B_F0)")
    print("        utils/collision/data/cell_env.npz        (충돌 모델)")
    print("  검증:  python scripts/artec/check_calibration.py\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
