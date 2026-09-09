#!/usr/bin/env python3
"""calib_handeye_sim.py — hand-eye(`T_E_C`) sim 검증을 **standalone** 으로 실행.

`sim_harness/MMS_ext_calibration.py` 는 Isaac GUI 안에서만 도는 Script Editor
스크립트라 터미널에서 못 돌린다. 이 스크립트가 그 로직을 **그대로 재사용**하면서
SimulationApp 을 직접 띄워 헤드리스로도 돌아가게 한다.

  ※ 검증 로직(보드 생성·자세 순회·검출·solvePnP·calibrateHandEye·GT 비교)은
    복제하지 않는다. 하니스 모듈을 로드해 그 함수를 호출한다 — 두 벌로 갈라지면
    sim 검증의 의미가 없어지기 때문이다.

실행
----
  # 헤드리스 (수치만, 권장)
  env -u PYTHONPATH ~/miniconda3/envs/env_isaacsim/bin/python scripts/sim/calib_handeye_sim.py

  # GUI 로 보면서
  env -u PYTHONPATH ~/miniconda3/envs/env_isaacsim/bin/python scripts/sim/calib_handeye_sim.py --gui

옵션
----
  --gui            GUI 표시 (기본: 헤드리스)
  --max-steps N    물리 스텝 상한 (기본 20000). 넘으면 중단하고 현황 출력
  --out DIR        산출물 위치 (기본: MMS_HARNESS_OUT 또는 scripts/sim/log/handeye)

결과
----
  로그에 `t_err` / `r_err` 과 판정(PASS/CHECK). `handeye_result.npz` 저장.
  기준: t_err < 5mm, r_err < 2°  (실물 2026-04-29 기준 3.55mm / 1.30°)
"""
from __future__ import annotations

import argparse
import importlib.util
import os
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO))

HARNESS = REPO / "sim_harness" / "MMS_ext_calibration.py"


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--gui", action="store_true", help="GUI 표시 (기본 헤드리스)")
    ap.add_argument("--max-steps", type=int, default=20000, help="물리 스텝 상한")
    ap.add_argument("--out", default=None, help="산출물 디렉터리")
    args = ap.parse_args()

    if not HARNESS.is_file():
        print(f"✘ 하니스를 찾을 수 없다: {HARNESS}")
        return 1

    if args.out:
        os.environ["MMS_HARNESS_OUT"] = args.out
    elif "MMS_HARNESS_OUT" not in os.environ:
        os.environ["MMS_HARNESS_OUT"] = str(Path(__file__).resolve().parent / "log" / "handeye")
    os.environ.setdefault("MMS_ROOT", str(REPO))

    # ── 1. SimulationApp 먼저 — 이 뒤에야 omni.* / isaacsim.* import 가 된다 ──
    from isaacsim import SimulationApp
    app = SimulationApp({"headless": not args.gui})

    try:
        # ── 2. 하니스 로드 (omni 가 준비된 뒤여야 한다) ──
        spec = importlib.util.spec_from_file_location("mms_ext_calibration", HARNESS)
        H = importlib.util.module_from_spec(spec)
        sys.modules["mms_ext_calibration"] = H       # @dataclass 등이 참조
        spec.loader.exec_module(H)

        # ── 3. 실행 ──
        #   setup_async 는 "셋업만" 하는 게 아니다. 물리 콜백을 등록하고
        #   play_async 를 await 한 채 남아, **실제 캘리브는 물리 스텝에서** 돈다.
        #   따라서 setup 완료를 기다리면 안 되고, `_ctx["phase"] == "DONE"` 을 본다.
        #
        #   Isaac(omni.kit)이 자체 asyncio 루프를 돌리므로 run_until_complete 로
        #   직접 돌리면 "Cannot enter into task ..." 로 깨진다 → app.update() 로 편다.
        import asyncio
        task = asyncio.ensure_future(H.setup_async())

        print(f"[standalone] 실행 시작 (update 상한 {args.max_steps})", flush=True)
        t0 = time.time()
        for i in range(args.max_steps):
            app.update()

            if task.done() and task.exception() is not None:
                import traceback
                traceback.print_exception(task.exception())
                return 1

            if H._ctx.get("phase") == "DONE":
                print(f"[standalone] 완료 — {i} update / {time.time()-t0:.1f}s", flush=True)
                break

            if i and i % 1000 == 0:
                cal = H._ctx.get("calibrator")
                print(f"[standalone]   {i} update  phase={H._ctx.get('phase')} "
                      f"pose={H._ctx.get('pose_idx')}/{len(H._ctx.get('poses') or [])} "
                      f"samples={getattr(cal, 'n_samples', 0)}", flush=True)
        else:
            print(f"⚠ update 상한({args.max_steps}) 도달 — phase={H._ctx.get('phase')}. "
                  f"--max-steps 를 늘릴 것", flush=True)
            return 2

        out = os.environ["MMS_HARNESS_OUT"]
        print(f"[standalone] 산출물: {out}", flush=True)
        return 0
    finally:
        try:
            app.close()
        except Exception:
            pass


if __name__ == "__main__":
    raise SystemExit(main())
