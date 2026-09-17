#!/bin/bash
# 9종 testset E2E 순회 (lookaround 만, 가볍게): v3 씬 → 플래너(preview→크롭→밴드) → export
# 전체 lookaround→2→3 을 운용 설정으로 돌리려면 testset_sweep.sh 를 쓴다.
#
# ★ 구 testset/composed/*_on_turntable.usd 는 **v2 레이아웃**이라 카메라·턴테이블
#   prim 을 못 찾고 스캔 없이 끝난다(실측 2026-08-19). v3_ts_* 씬만 쓴다.
cd "$(dirname "$0")/../.."
source scripts/sim/_paths.sh
OUT=scripts/sim/log/e2e_sweep; mkdir -p $OUT
V3DIR=$(mms_v3_dir) || exit 1
for usd in "$V3DIR"/v3_ts_*.usd; do
  name=$(basename "$usd" .usd); name=${name#v3_ts_}
  echo "=== SWEEP $name start $(date +%H:%M:%S)"
  PYTHONUNBUFFERED=1 MMS_ISAAC_HEADLESS=1 MMS_SIM_NO_VIZ=1 \
  MMS_SIM_USD="$usd" MMS_SIM_OBJECT_PRIM=/World/ScanTarget/TestObject \
  MMS_SIM_STAGE_UNTIL=1 MMS_SIM_NTHETA=12 MMS_SIM_DRIVE_STEPS=10 \
  timeout 900 "$(mms_python)" -u main_artec.py \
    > "$OUT/$name.log" 2>&1
  code=$?
  plan=$(grep -a "P1 플랜" "$OUT/$name.log" | head -1 | sed 's/.*P1 플랜: //')
  fin=$(grep -a "완료 — 누적" "$OUT/$name.log" | head -1 | sed 's/.*완료 — //')
  obj=$(grep -a "saved →" "$OUT/$name.log" | head -1 | sed 's/.*saved → //')
  echo "=== SWEEP $name exit=$code | plan=$plan | $fin | obj=$obj"
done
echo "=== SWEEP ALL DONE"
