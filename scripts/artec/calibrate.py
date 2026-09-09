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

  ※ 보드를 **턴테이블 원판 위에** 올려두면 2→3 을 같은 조준 자세에서 이어서 할 수 있다.

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
import subprocess
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]

STEPS = [
    (1, "intrinsic  — 카메라 K",      "scripts/artec/intrinsic_calib.py",
     "config/calibration/artec_intrinsic.yaml"),
    (2, "hand-eye   — T_EC",          "scripts/artec/hand_eye_calib.py",
     "config/calibration/hand_eye_artec.yaml → sensor_frames.yaml::T_EC_artec"),
    (3, "turntable  — T_B_F0",        "scripts/artec/turntable_calib.py",
     "config/calibration/turntable_frame.yaml"),
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


def run(step_no: int, title: str, rel: str, out: str, extra: list[str]) -> bool:
    print(f"\n{'='*66}\n  [{step_no}/3] {title}\n  → {out}\n{'='*66}")
    cmd = [sys.executable, str(REPO / rel), *extra]
    print(f"  $ {' '.join(cmd)}\n")
    r = subprocess.run(cmd, cwd=REPO)
    if r.returncode != 0:
        print(f"\n✘ [{step_no}] 실패 (exit {r.returncode}) — 여기서 중단한다.")
        return False
    print(f"\n✓ [{step_no}] 완료")
    return True


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--from", dest="start", type=int, default=1, choices=(1, 2, 3),
                    help="이 단계부터 실행 (기본 1)")
    ap.add_argument("--only", type=int, choices=(1, 2, 3), help="이 단계만 실행")
    ap.add_argument("--skip-aim", action="store_true", help="수동 조준 안내 생략")
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
        if not run(no, title, rel, out, extra if args.only == no else []):
            return 1

    print(f"\n{'='*66}\n  캘리브레이션 완료\n{'='*66}")
    print("  확인:  config/sensor_frames.yaml            (T_EC_artec)")
    print("        config/calibration/turntable_frame.yaml (T_B_F0)")
    print("  검증:  docs/1_calibration.md 부록 T5\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
