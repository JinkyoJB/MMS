"""캘리브 단계 스크립트를 **직접** 실행했을 때 순서를 환기시킨다.

왜 필요한가
----------
`intrinsic_calib.py` · `hand_eye_calib.py` · `turntable_calib.py` 는 각각 단독 실행이
가능하지만, 셋 사이에는 **의존 순서**가 있다:

    intrinsic(K) → hand-eye(T_EC) → turntable(T_B_F0)

turntable 은 rim 점을 `T_CB = T_EB · inv(T_EC)` 로 base 에 옮겨 원을 피팅하므로
**T_EC 가 먼저** 나와야 하고, hand-eye 는 K 로 solvePnP 를 풀어 `T_MC` 를 구하므로
**K 가 먼저** 나와야 한다. 순서를 건너뛰면 조용히 낡은 값을 쓴 결과가 나온다.

문서 여러 곳이 개별 실행을 나란히 안내해 와서 어느 게 정본인지 헷갈렸다
(2026-09-16). 스크립트를 지우는 대신 **여기서 한 번 짚어준다** —
`calibrate.py` 가 부를 때는 조용히 넘어간다.
"""
from __future__ import annotations

import os
import sys

#: `calibrate.py` 가 자식 프로세스에 세우는 표식. 이게 있으면 경고하지 않는다.
ENV_FLAG = "MMS_CALIB_DRIVER"

_TITLES = {
    1: "intrinsic  — 카메라 K",
    2: "hand-eye   — T_EC",
    3: "turntable  — T_B_F0",
}


def warn_if_direct(step_no: int) -> None:
    """`calibrate.py` 를 거치지 않은 직접 실행이면 안내하고 확인을 받는다."""
    if os.environ.get(ENV_FLAG) == "1":
        return                      # calibrate.py 가 부른 것 — 조용히 진행

    title = _TITLES.get(step_no, "?")
    print()
    print("  " + "─" * 64)
    print(f"  ⚠ 이건 캘리브레이션 {step_no}/3 단계다 — {title}")
    print("  " + "─" * 64)
    print("    순서가 있다:  1 intrinsic(K) → 2 hand-eye(T_EC) → 3 turntable(T_B_F0)")
    print("    뒷단계가 앞단계 결과를 쓰므로, 건너뛰면 낡은 값이 그대로 들어간다.")
    print()
    print("      전체    :  python scripts/artec/calibrate.py")
    print(f"      이것만  :  python scripts/artec/calibrate.py --only {step_no}")
    print(f"      여기부터:  python scripts/artec/calibrate.py --from {step_no}")
    print("  " + "─" * 64)

    # 비대화형(파이프·CI)에서는 멈추지 않는다 — 안내만 남기고 진행.
    #
    # ⚠ `isatty()` 만으로는 부족하다. tty 라고 보고하면서도 실제로는 읽을 게 없어
    #   `input()` 이 EOFError 로 죽는 환경이 있다(에이전트 셸에서 실측). 안내용
    #   경고가 본 작업을 죽이면 안 되므로 EOF 는 '진행' 으로 본다.
    try:
        if not sys.stdin.isatty():
            print("  (비대화형 — 그대로 진행한다)\n")
            return
        ans = input("  그래도 이 단계만 실행하려면 Enter (중단은 q): ")
    except (EOFError, OSError):
        print("  (입력을 받을 수 없다 — 그대로 진행한다)\n")
        return
    if ans.strip().lower() == "q":
        raise SystemExit(1)
