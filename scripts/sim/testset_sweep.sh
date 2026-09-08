#!/bin/bash
# testset_sweep.sh — 9종 testset 에 대해 **전체 Phase 1→2→3** 을 돌려 일반성 검증.
#
# 기존 e2e_sweep.sh 는 Phase1 만 NTHETA=12 로 도는 가벼운 점검이다. 이 스크립트는
# 실제 운용 설정(120프레임/rev)으로 끝까지 돌려 물체별 품질·시간을 표로 남긴다.
#
#   scripts/sim/testset_sweep.sh              # 9종 전부
#   scripts/sim/testset_sweep.sh 0146_mug     # 특정 물체만
#
# ⚠ Isaac 은 인스턴스를 **동시에 띄우면 물리엔진이 깨진다**. 반드시 순차 실행.
set -u
cd "$(dirname "$0")/../.."
OUT=${OUT:-scripts/sim/log/testset_sweep}; mkdir -p "$OUT"
FRAMES=${MMS_SIM_FRAMES_PER_REV:-120}
TIMEOUT=${TIMEOUT:-1500}
PY=/home/keti/miniconda3/envs/env_isaacsim/bin/python
# ★ v3 기준 씬을 쓴다. composed/*.usd 는 **구 v2 레이아웃**이라 턴테이블/카메라
#   prim 을 못 찾고 스캔 없이 30초만에 끝난다(실측 2026-08-19).
#   씬 생성: build_scene_v3.py --out v3_ts_<name>.usd --object <testset>.usd
ASSET=/home/keti/workspace/sync/2_Rapid_Digital_Twin/1_MMS/2_3Dassets/frame_xarm7_spider_turntable_v2

if [ $# -gt 0 ]; then LIST=(); for n in "$@"; do LIST+=("$ASSET/v3_ts_${n}.usd"); done
else LIST=("$ASSET"/v3_ts_*.usd); fi

if pgrep -f "python -u main_artec" > /dev/null; then
  echo "⚠ main_artec 이 이미 실행 중이다. 동시 실행은 물리엔진을 깨뜨린다. 중단."; exit 1
fi

SUM="$OUT/summary.tsv"
printf "물체\t종료\t밴드\tP1_bnd\tP1_gap\t패치\t수렴\tP2_bnd\tP2_gap\tflip각\tflip정합\t누적점\t초\n" > "$SUM"

for usd in "${LIST[@]}"; do
  name=$(basename "$usd" .usd); name=${name#v3_ts_}
  [ -f "$usd" ] || { echo "!! 없음: $usd"; continue; }
  echo "=== $name 시작 $(date +%H:%M:%S)"
  t0=$(date +%s)
  MMS_ISAAC_HEADLESS=1 MMS_SIM_NO_VIZ=1 MMS_SIM_VIZ=0 \
  MMS_SIM_USD="$usd" MMS_SIM_OBJECT_PRIM=/World/ScanTarget/TestObject \
  MMS_SIM_FRAMES_PER_REV="$FRAMES" MMS_SIM_PROFILE_EVERY=0 \
  timeout "$TIMEOUT" env -u PYTHONPATH $PY -u main_artec.py > "$OUT/$name.log" 2>&1
  code=$?; dt=$(( $(date +%s) - t0 ))

  # Phase1 = 첫 boundary, Phase2 = 마지막 boundary (flip 전까지)
  mapfile -t B < <(grep -a "boundary=" "$OUT/$name.log" | sed -E 's/.*boundary=([0-9]+)mm.*gaps=([0-9]+).*/\1 \2/')
  p1=${B[0]:-"-"}; p2=${B[${#B[@]}-1]:-"-"}
  flip=$(grep -a "flip 정합 채택\|flip 국소정합 적용" "$OUT/$name.log" | head -1 | sed 's/.*] *//' | cut -c1-28)
  [ -z "$flip" ] && flip=$(grep -aq "hint 그대로" "$OUT/$name.log" && echo "hint폴백" || echo "-")
  pts=$(grep -a "완료 — 누적" "$OUT/$name.log" | tail -1 | sed -E 's/.*누적 ([0-9]+)점.*/\1/')
  # 새 신호들 — 밴드 수 / NBV 패치 수 / 종료 사유 / 적용된 flip 각
  bands=$(grep -a "P1 플랜:" "$OUT/$name.log" | head -1 | sed -E 's/.*플랜: ([0-9]+) bands.*/\1/;t;s/.*플랜: single.*/1/')
  npatch=$(grep -ac "=== patch: Phase 2 NBV" "$OUT/$name.log")
  if grep -aq "수렴 — 새로 보이는 곳이 없다" "$OUT/$name.log"; then conv="조기(복셀)"
  elif grep -aq "gap 겨냥 실패" "$OUT/$name.log"; then conv="후보소진"
  else conv="상한"; fi
  fang=$(grep -a "Phase 3: 물체" "$OUT/$name.log" | sed -E 's/.*축 ([0-9]+)°.*/\1/' | paste -sd+ -)
  [ -z "$fang" ] && fang="-"
  printf "%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n" \
    "$name" "$code" "${bands:-1}" "${p1% *}" "${p1#* }" "$npatch" "$conv" \
    "${p2% *}" "${p2#* }" "$fang" "$flip" "${pts:--}" "$dt" >> "$SUM"
  [ "${#B[@]}" -eq 0 ] && echo "  !! boundary 지표 없음 — 스캔이 돌지 않았다. 로그 확인: $OUT/$name.log"
  echo "=== $name 종료(code=$code, ${dt}s)  P1=$p1  P2=$p2"
done
echo; echo "=== 요약 ==="; column -t -s$'\t' "$SUM"
