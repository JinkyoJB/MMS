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
#   GUI 에서는 preview 가 끝나고 **장애물 등록 결과를 보여주며 잠깐 멈춘다**:
#     파랑 = preview 동안 걸어둔 보호 원기둥(가정치, r=원판반경)
#     주황 = 실제로 등록된 회전체 — 충돌 게이트가 보는 바로 그 점군
#
#   사용:
#     bash scripts/sim/preview_testset.sh                 # 9종, GUI, 10초 멈춤
#     bash scripts/sim/preview_testset.sh --headless      # 9종, 창 없이(빠름)
#     bash scripts/sim/preview_testset.sh mug drill       # 이름에 그 글자가 든 것만
#     bash scripts/sim/preview_testset.sh --pause 30      # 더 오래 보기
#     bash scripts/sim/preview_testset.sh --pause 0       # 안 멈추고 쭉
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
PAUSE=10            # GUI 에서 장애물 등록 결과를 보여주며 멈추는 초
ARGS=()
want_pause=0
for a in "$@"; do
  if [ $want_pause -eq 1 ]; then PAUSE=$a; want_pause=0; continue; fi
  case "$a" in
    --headless|-H) HEADLESS=1 ;;
    --gui|-g)      HEADLESS=0 ;;
    --pause|-p)    want_pause=1 ;;
    *)             ARGS+=("$a") ;;
  esac
done
set -- "${ARGS[@]+"${ARGS[@]}"}"
[ "$HEADLESS" = "1" ] && PAUSE=0            # 볼 창이 없다
VIZ=$([ "$HEADLESS" = "1" ] && echo 0 || echo 1)
if [ "$HEADLESS" = "1" ]; then
  echo "모드: headless (창 없음 — 스냅샷·요약표만)"
else
  echo "모드: GUI — 물체마다 Isaac 창이 뜬다. 기동만 ~40초씩 걸린다"
  echo "      preview 후 ${PAUSE}초 멈춰 장애물 등록 결과를 보여준다"
  echo "        파랑 = 보호 원기둥(가정치) · 주황 = 실제 등록된 회전체"
  echo "      (숫자만 볼 거면 --headless, 시간 조절은 --pause N)"
fi

shopt -s nullglob

# 콘솔에 **실시간**으로 흘릴 줄. Isaac 기동 로그(수백 줄)는 버리고 진행 단계만
# 남긴다. 전체 로그는 파일에 그대로 쌓이므로 나중에 다 볼 수 있다.
#   ⚠ 예전에는 출력을 통째로 파일로만 보내서(`> log 2>&1`) 물체당 ~75초 동안
#     콘솔이 완전히 조용했다 — 지금 어느 단계인지, 멎은 건지 알 수가 없었다.
KEEP='^\[(stage|main|lookaround|collision|IsaacWorld)\]|^\[isaac_scan\]|^\[p1plan\]|Traceback|Error:|✘'
DROP='Warning|\[ext:|deprecat|Carpet_Cream'

# 대상 목록 먼저 세어 둔다 (진행 n/N 표시용)
TARGETS=()
for f in "$SCENES"/v2r_*.usd; do
  n=$(basename "$f" .usd); n=${n#v2r_}
  if [ $# -gt 0 ]; then
    hit=0; for pat in "$@"; do case "$n" in *"$pat"*) hit=1;; esac; done
    [ $hit -eq 1 ] || continue
  fi
  TARGETS+=("$f")
done
N=${#TARGETS[@]}
[ "$N" -gt 0 ] || { echo "✘ 대상 없음 (이름 필터를 확인할 것)"; exit 1; }
echo "대상 $N 종"

i=0
for f in "${TARGETS[@]}"; do
  i=$((i+1))
  n=$(basename "$f" .usd); n=${n#v2r_}
  t0=$SECONDS
  echo
  echo "══════ [$i/$N] $n ══════════════════════════════════════"
  echo "   Isaac 기동 중… (창이 뜰 때까지 ~40초, 로그는 $LOGS/$n.log)"
  env -u PYTHONPATH \
    MMS_BACKEND=isaac MMS_ISAAC_HEADLESS=$HEADLESS \
    MMS_SIM_STAGE_UNTIL=preview \
    MMS_DEBUG_VIEW=1 MMS_DEBUG_DIR="$OUT/$n" \
    MMS_SIM_VIZ=$VIZ MMS_SIM_PAUSE=$PAUSE \
    MMS_COLLISION_LAYOUT=v2_real_260917 \
    MMS_SIM_USD="$f" \
    $ISAAC -u main_artec.py --no-prompt 2>&1 < /dev/null \
    | tee "$LOGS/$n.log" \
    | grep --line-buffered -aE "$KEEP" \
    | grep --line-buffered -avE "$DROP"
  rc=${PIPESTATUS[0]}
  echo "   └ [$i/$N] $n  $((SECONDS-t0))초  (종료코드 $rc)"
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
