# _paths.sh — 셸 스크립트용 경로 해석. `source scripts/sim/_paths.sh` 로 쓴다.
#
# 파이썬 쪽 mms_paths.py 와 같은 규칙:
#   MMS_ASSET_ROOT > 인수인계 배치 > 원본 개발 배치 > 리포 내부 > legacy 절대경로
# 호출 전에 리포 루트로 cd 되어 있다고 가정한다 (각 스크립트가 `cd "$(dirname "$0")/../.."`).

mms_asset_root() {
  local r
  for r in "${MMS_ASSET_ROOT:-}" \
           "$PWD/../../2_데이터/2_3Dassets" \
           "$PWD/../2_3Dassets" \
           "$PWD/2_3Dassets" \
           "/home/keti/workspace/sync/2_Rapid_Digital_Twin/1_MMS/2_3Dassets"; do
    [ -n "$r" ] && [ -d "$r" ] && { (cd "$r" && pwd); return 0; }
  done
  echo "✘ 자산 루트(2_3Dassets)를 찾지 못했다." >&2
  echo "  → export MMS_ASSET_ROOT=/경로/2_3Dassets" >&2
  return 1
}

# v3 씬 디렉터리 (v3_scene.usd + 물체별 v3_ts_*.usd)
mms_v3_dir() {
  local r; r=$(mms_asset_root) || return 1
  echo "$r/frame_xarm7_spider_turntable_v2"
}

# 기본 씬 = v3_scene.usd
mms_v3_scene() {
  local d; d=$(mms_v3_dir) || return 1
  echo "$d/v3_scene.usd"
}

# 물체별 v3 씬. 부분이름 매칭 (mms_v3_ts spray_can → v3_ts_0101_spray_can.usd)
#
# ★ 구 `testset/composed/*_on_turntable.usd` 는 **v2 레이아웃**이라 카메라·턴테이블
#   prim 을 못 찾고 스캔 없이 30초 만에 끝난다(실측 2026-08-19). v3 씬만 쓸 것.
#   없으면 만든다:
#     build_scene_v3.py --out v3_ts_<이름>.usd --object "$(mms_testset_dir)/<이름>.usd"
mms_v3_ts() {
  local d hit; d=$(mms_v3_dir) || return 1
  hit=$(ls "$d"/v3_ts_*"$1"*.usd 2>/dev/null | head -1)
  [ -n "$hit" ] && { echo "$hit"; return 0; }
  echo "✘ v3 씬을 못 찾음: $d/v3_ts_*$1*.usd" >&2
  echo "  → build_scene_v3.py --out v3_ts_<이름>.usd --object <testset>/<이름>.usd" >&2
  return 1
}

# 대상물 USD 트리 (v3 씬을 만들 때의 --object 원본)
mms_testset_dir() {
  local r d
  if [ -n "${MMS_TESTSET_DIR:-}" ]; then echo "$MMS_TESTSET_DIR"; return 0; fi
  r=$(mms_asset_root) || return 1
  for d in "$(dirname "$r")/testset" "$r/testset" \
           "$HOME/isaacsim/standalone_examples/play/MMS/testset"; do
    [ -d "$d" ] && { echo "$d"; return 0; }
  done
  echo "$(dirname "$r")/testset"
}

# Isaac Sim 파이썬
mms_python() {
  echo "${MMS_PYTHON:-$HOME/miniconda3/envs/env_isaacsim/bin/python}"
}
