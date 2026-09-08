#!/usr/bin/env bash
# setup_envs.sh — MMS 개발환경 3종을 한 번에 만든다.
#
#   bash setup/setup_envs.sh            # 3개 전부
#   bash setup/setup_envs.sh mms-env    # 하나만
#
# 왜 3개인가
#   mms-env       실물(real) 백엔드 + 오프라인 스크립트(캘리브·분석). numpy 2.x
#   env_isaacsim  Isaac Sim(sim) 백엔드.                              numpy 1.x ← 섞으면 깨짐
#   step2usd      STEP→USD 변환 전용. Isaac 불필요라 가볍고 빠름
#
# ⚠ env 를 섞지 말 것. 특히 numpy 메이저 버전이 달라 한 env 로 합칠 수 없다.

set -euo pipefail
cd "$(dirname "$0")/.."          # 리포 루트
TARGET="${1:-all}"
PY=3.11

command -v conda >/dev/null || { echo "✘ conda 를 찾을 수 없다."; exit 1; }
eval "$(conda shell.bash hook)"

have() { conda env list | awk '{print $1}' | grep -qx "$1"; }

make_env() {
  local name=$1
  if have "$name"; then
    echo "▸ $name — 이미 존재. 건너뜀 (다시 만들려면: conda env remove -n $name)"
    return 1
  fi
  echo "▸ $name 생성 중 (python $PY)..."
  conda create -y -n "$name" "python=$PY" >/dev/null
  return 0
}

# ── 1. mms-env — 실물 백엔드 ──────────────────────────────────────────
if [[ "$TARGET" == all || "$TARGET" == mms-env ]]; then
  if make_env mms-env; then
    conda activate mms-env
    pip install -q -r requirements.txt
    conda deactivate
    echo "  ✓ mms-env 완료"
    echo "    ※ Artec SDK python 바인딩은 별도 빌드 필요"
    echo "      → docs/artec_SDK/artec0_build_guide.md"
  fi
fi

# ── 2. env_isaacsim — Isaac Sim 백엔드 ────────────────────────────────
if [[ "$TARGET" == all || "$TARGET" == env_isaacsim ]]; then
  if make_env env_isaacsim; then
    conda activate env_isaacsim
    # isaacsim 은 NVIDIA 인덱스에서 받는다
    pip install -q -r setup/requirements-isaac.txt --extra-index-url https://pypi.nvidia.com
    conda deactivate
    echo "  ✓ env_isaacsim 완료"
    echo "    ※ 첫 실행 시 Isaac 이 셰이더를 캐싱하느라 수 분 걸린다"
  fi
fi

# ── 3. step2usd — STEP→USD 변환 ───────────────────────────────────────
if [[ "$TARGET" == all || "$TARGET" == step2usd ]]; then
  if make_env step2usd; then
    conda activate step2usd
    # pythonocc-core 는 pip 에 없다 → conda-forge
    conda install -y -q -c conda-forge "pythonocc-core=7.9" >/dev/null
    pip install -q -r setup/requirements-step2usd.txt
    conda deactivate
    echo "  ✓ step2usd 완료"
  fi
fi

# ── 검증 ──────────────────────────────────────────────────────────────
echo
echo "── 설치 확인 ──────────────────────────────────────────"
check() {  # name  import문  설명
  local p=~/miniconda3/envs/$1/bin/python
  printf "  %-14s " "$1"
  [ -x "$p" ] || { echo "✘ 없음"; return; }
  env -u PYTHONPATH "$p" -c "$2" 2>/dev/null || echo "✘ $3 import 실패"
}
check mms-env      "import numpy,cv2,open3d;print(f'✓ numpy {numpy.__version__} / cv2 {cv2.__version__} / open3d {open3d.__version__}')" "core"
check env_isaacsim "import numpy,open3d;print(f'✓ numpy {numpy.__version__} / open3d {open3d.__version__}  (isaacsim 은 실행 시 확인)')" "core"
check step2usd     "import OCC,pxr;print('✓ pythonocc + usd-core')" "OCC/pxr"

echo
echo "── 실행 예시 ──────────────────────────────────────────"
cat <<'USAGE'
  # sim (권장 진입점 — 씬·인자 처리 포함)
  ./scripts/sim/run_e2e_gui.sh mug

  # real
  conda activate mms-env
  env -u PYTHONPATH python main_artec.py

  ⚠ 모든 실행에 `env -u PYTHONPATH` 를 붙인다 (ROS python3.10 경로 오염 방지)
USAGE
