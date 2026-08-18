#!/bin/bash
# 9종 testset E2E 순회: 합성 씬 → Phase1 플래너(preview→크롭→밴드) → export
cd "$(dirname "$0")/../.."
OUT=scripts/sim/log/e2e_sweep; mkdir -p $OUT
for usd in /home/keti/isaacsim/standalone_examples/play/MMS/testset/composed/*_on_turntable.usd; do
  name=$(basename "$usd" _on_turntable.usd)
  echo "=== SWEEP $name start $(date +%H:%M:%S)"
  PYTHONUNBUFFERED=1 MMS_ISAAC_HEADLESS=1 MMS_SIM_NO_VIZ=1 \
  MMS_SIM_USD="$usd" MMS_SIM_OBJECT_PRIM=/World/ScanTarget/TestObject \
  MMS_SIM_PHASE_MODE=1 MMS_SIM_NTHETA=12 MMS_SIM_DRIVE_STEPS=10 \
  timeout 900 /home/keti/miniconda3/envs/env_isaacsim/bin/python -u main_artec.py \
    > "$OUT/$name.log" 2>&1
  code=$?
  plan=$(grep -a "P1 플랜" "$OUT/$name.log" | head -1 | sed 's/.*P1 플랜: //')
  fin=$(grep -a "완료 — 누적" "$OUT/$name.log" | head -1 | sed 's/.*완료 — //')
  obj=$(grep -a "saved →" "$OUT/$name.log" | head -1 | sed 's/.*saved → //')
  echo "=== SWEEP $name exit=$code | plan=$plan | $fin | obj=$obj"
done
echo "=== SWEEP ALL DONE"
