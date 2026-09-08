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

# testset USD 트리 (Isaac standalone 아래)
mms_testset_dir() {
  echo "${MMS_TESTSET_DIR:-$HOME/isaacsim/standalone_examples/play/MMS/testset}"
}

# Isaac Sim 파이썬
mms_python() {
  echo "${MMS_PYTHON:-$HOME/miniconda3/envs/env_isaacsim/bin/python}"
}
