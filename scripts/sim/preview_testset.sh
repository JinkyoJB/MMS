#!/usr/bin/env bash
# preview_testset.sh — testset 9종을 preview 단계만 연달아 돌린다.
#
#   기본은 **GUI** 다 — 로봇이 어디로 가는지 눈으로 본다. 창을 닫으면 다음 물체.
#   `--headless` 를 주면 창 없이 돌고 스냅샷·요약표만 남는다(훨씬 빠르다).
#
#   물체마다:
#     · 스냅샷  output/debug/testset/<물체>/preview/   (이동마다 1장)
#     · 로그    output/debug/testset/_logs/<물체>.log
#   마지막에 요약표를 찍는다.
#
#   사용:
#     bash scripts/sim/preview_testset.sh                 # 9종, GUI
#     bash scripts/sim/preview_testset.sh --headless      # 9종, 창 없이(빠름)
#     bash scripts/sim/preview_testset.sh mug drill       # 이름에 그 글자가 든 것만
#     bash scripts/sim/preview_testset.sh --headless mug  # 섞어 써도 된다
#
#   ⚠ GUI 는 한 종당 Isaac 창을 새로 띄웠다 닫는다(기동만 ~40초). 9종이면
#     10분쯤 걸린다. 숫자만 볼 거면 --headless 가 낫다.
set -u
cd "$(dirname "$0")/../.."
ROOT=$PWD
ISAAC=~/miniconda3/envs/env_isaacsim/bin/python
MMSPY=~/miniconda3/envs/mms-env/bin/python
ASSET=$(env -u PYTHONPATH $MMSPY -c "import mms_paths;print(mms_paths.asset_root())")
SCENES="$ASSET/frame_xarm7_spider_turntable/testset_scenes"
OUT="$ROOT/output/debug/testset"
LOGS="$OUT/_logs"
mkdir -p "$LOGS"

[ -d "$SCENES" ] || { echo "✘ 씬이 없다: $SCENES"; echo "  먼저 씬을 구울 것 (README 참조)"; exit 1; }

# ── GUI/headless ────────────────────────────────────────────────────────
HEADLESS=0
ARGS=()
for a in "$@"; do
  case "$a" in
    --headless|-H) HEADLESS=1 ;;
    --gui|-g)      HEADLESS=0 ;;
    *)             ARGS+=("$a") ;;
  esac
done
set -- "${ARGS[@]+"${ARGS[@]}"}"
if [ "$HEADLESS" = "1" ]; then
  echo "모드: headless (창 없음 — 스냅샷·요약표만)"
else
  echo "모드: GUI — 물체마다 Isaac 창이 뜬다. 기동만 ~40초씩 걸린다"
  echo "      (숫자만 볼 거면 --headless)"
fi

shopt -s nullglob
for f in "$SCENES"/v2r_*.usd; do
  n=$(basename "$f" .usd); n=${n#v2r_}
  if [ $# -gt 0 ]; then
    hit=0; for pat in "$@"; do case "$n" in *"$pat"*) hit=1;; esac; done
    [ $hit -eq 1 ] || continue
  fi
  echo "── $n ──────────────────────────────────────────"
  env -u PYTHONPATH \
    MMS_BACKEND=isaac MMS_ISAAC_HEADLESS=$HEADLESS \
    MMS_SIM_STAGE_UNTIL=preview \
    MMS_DEBUG_VIEW=1 MMS_DEBUG_DIR="$OUT/$n" \
    MMS_COLLISION_LAYOUT=v2_real_260917 \
    MMS_SIM_USD="$f" \
    MMS_SIM_OBJECT_PRIM=/World/ScanTarget/TestObject \
    $ISAAC -u main_artec.py --no-prompt > "$LOGS/$n.log" 2>&1 < /dev/null
  grep -hE "객체=|계획용 preview|밴드 [0-9]개|단일|자세 el=" "$LOGS/$n.log" \
    | sed 's/^\[[a-z_]*\] */    /' | head -8
done

echo
echo "════════ 요약 ════════"
printf "%-24s %7s %6s %5s %s\n" 물체 preview점 이동 밴드 "band_h/겹침"
for L in "$LOGS"/*.log; do
  n=$(basename "$L" .log)
  p=$(grep -oE "계획용 preview [0-9]+pt" "$L" | grep -oE "[0-9]+" | head -1)
  mv=$(grep -oE "\([0-9]+ 유효 / [0-9]+ 이동" "$L" | grep -oE "[0-9]+ 이동" | grep -oE "[0-9]+")
  b=$(grep -oE "밴드 [0-9]+개" "$L" | grep -oE "[0-9]+" | head -1)
  bh=$(grep -oE "band_h=[0-9]+mm 센터간격=[0-9]+mm" "$L" | head -1)
  ov=$(grep -oE "측정 인접겹침 최소=[0-9]+%" "$L" | head -1 | grep -oE "[0-9]+%")
  [ -z "$b" ] && b=$(grep -qE "단일" "$L" && echo "단일" || echo "-")
  printf "%-24s %7s %6s %5s %s %s\n" "$n" "${p:--}" "${mv:--}" "$b" "${bh:--}" "${ov:-}"
done
echo
echo "스냅샷: $OUT/<물체>/preview/   로그: $LOGS/"
