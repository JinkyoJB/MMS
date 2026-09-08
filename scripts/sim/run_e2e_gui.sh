#!/bin/bash
# run_e2e_gui.sh — Phase1 플래너 E2E 를 GUI 로 직접 관찰 (+베이스 높이 조건부).
#
# 사용:  ./scripts/sim/run_e2e_gui.sh [물체] [phase_mode] [planner|legacy] [ΔH cm]
#   ./scripts/sim/run_e2e_gui.sh                       # marble, Phase1, 플래너, ΔH=0
#   ./scripts/sim/run_e2e_gui.sh mug                   # testset 물체 (부분이름 OK)
#   ./scripts/sim/run_e2e_gui.sh marble 2 planner 10   # ★ 베이스 +10cm 로 Phase1→2
#   ./scripts/sim/run_e2e_gui.sh detergent 1 planner 15
#   ./scripts/sim/run_e2e_gui.sh marble 1 legacy       # 기존 방식 A/B
#
# ΔH>0 이면 오버레이 씬(hibase/*_dh<cm>.usd)을 자동 생성(1회, 이후 캐시).
# 원본 v2.usd/composed 는 절대 수정하지 않음 — real 개조 전 sim 선행 검증용.
OBJ=${1:-marble}; PM=${2:-1}; MODE=${3:-planner}; DH=${4:-0}
cd "$(dirname "$0")/../.."
source scripts/sim/_paths.sh
ASSET_ROOT=$(mms_asset_root) || exit 1

# 씬 선택
if [[ "$OBJ" == marble* || "$OBJ" == solid* ]]; then
  scene=$ASSET_ROOT/frame_xarm7_spider_turntable/v2.usd
  objprim=""
else
  scene=$(ls "$(mms_testset_dir)"/composed/*${OBJ}*_on_turntable.usd 2>/dev/null | head -1)
  if [ -z "$scene" ]; then echo "✘ testset 에서 '$OBJ' 못 찾음"; exit 1; fi
  objprim=/World/ScanTarget/TestObject
fi

# 베이스 높이 오버레이 (캐시)
if [ "$DH" != "0" ]; then
  stem=$(basename "$scene" .usd)
  hb="$(dirname "$scene")/hibase/${stem}_dh${DH}.usd"
  if [ ! -f "$hb" ]; then
    echo "[run] ΔH=+${DH}cm 오버레이 생성 중 (1회, ~30s)..."
    ~/isaacsim/python.sh scripts/sim/make_hibase_scene.py --dh-cm "$DH" --scene "$scene" > /dev/null 2>&1
  fi
  if [ ! -f "$hb" ]; then echo "✘ 오버레이 생성 실패"; exit 1; fi
  scene="$hb"
fi

ENVV=(MMS_SIM_PHASE_MODE=$PM MMS_SIM_P1_MODE=$MODE
      MMS_SIM_NTHETA=24 MMS_SIM_DRIVE_STEPS=20
      MMS_SIM_USD="$scene")
[ -n "$objprim" ] && ENVV+=(MMS_SIM_OBJECT_PRIM=$objprim)

echo "[run] scene=$(basename $scene)  phase_mode=$PM  mode=$MODE  ΔH=+${DH}cm"
env "${ENVV[@]}" "$(mms_python)" -u main_artec.py
