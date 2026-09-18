#!/bin/bash
# run_e2e_gui.sh — lookaround 플래너 E2E 를 GUI 로 직접 관찰.
#
# 사용:  ./scripts/sim/run_e2e_gui.sh [물체] [stage_until]
#        stage_until = preview | lookaround | nbv | flip   (기본 lookaround)
#
# ⚠ 이 러너는 **v3 씬 전용**이다. 실물 셀은 v2 이고 기본 씬도
#   v2_real_260917.usd 다 — 실물과 맞춘 실행은 docs/3_lookaround.md §1 참조.
#   여기는 v3 로 남겨둔 회귀용이다.
#   ./scripts/sim/run_e2e_gui.sh                      # 기본 씬(v3_scene.usd), lookaround
#   ./scripts/sim/run_e2e_gui.sh spray_can            # 물체별 v3 씬 (부분이름 OK)
#   ./scripts/sim/run_e2e_gui.sh detergent lookaround # 밴드 분할이 걸리는 사례
#   ./scripts/sim/run_e2e_gui.sh mug nbv              # nbv 보강까지
#
# 씬은 **v3 만** 쓴다. 구 v2.usd / testset/composed/*_on_turntable.usd 는 카메라·
# 턴테이블 prim 경로가 달라 스캔 없이 30초 만에 끝난다(실측 2026-08-19).
# 물체별 v3 씬이 없으면 만든다:
#   build_scene_v3.py --out v3_ts_<이름>.usd --object "$(mms_testset_dir)/<이름>.usd"
OBJ=${1:-}; PM=${2:-lookaround}
cd "$(dirname "$0")/../.."
source scripts/sim/_paths.sh

# 씬 선택 — 인자 없으면 기본 v3 씬, 있으면 물체별 v3 씬
if [ -z "$OBJ" ]; then
  scene=$(mms_v3_scene) || exit 1
  objprim=""
else
  scene=$(mms_v3_ts "$OBJ") || exit 1
  objprim=/World/ScanTarget/TestObject
fi

ENVV=(MMS_SIM_STAGE_UNTIL=$PM
      MMS_SIM_NTHETA=24 MMS_SIM_DRIVE_STEPS=20
      MMS_SIM_USD="$scene")
[ -n "$objprim" ] && ENVV+=(MMS_SIM_OBJECT_PRIM=$objprim)

echo "[run] scene=$(basename $scene)  stage_until=$PM"
env "${ENVV[@]}" "$(mms_python)" -u main_artec.py
