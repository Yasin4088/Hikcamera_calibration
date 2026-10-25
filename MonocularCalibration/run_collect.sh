#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="${SCRIPT_DIR}"
BUILD_DIR="${BUILD_DIR:-${PROJECT_DIR}/build}"
BIN="${BUILD_DIR}/demarcate"
CONFIG="${CONFIG:-${PROJECT_DIR}/config/dart.yaml}"
MVS_ROOT="${MVS_ROOT:-/opt/MVS}"

INPUT_DIR_DEFAULT="${PROJECT_DIR}/sub_module/Calibration/Input/camera_$(date +%Y%m%d_%H%M%S)"

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  cat <<'EOF'
Usage:
  ./run_collect.sh [input_dir] [--demarcate-options]

Examples:
  ./run_collect.sh
  ./run_collect.sh ./sub_module/Calibration/Input/camera_test --interval=0.8 --auto=true

Environment:
  BUILD_DIR   Build directory (default: ./build)
  CONFIG      Config file path (default: ./config/dart.yaml)
  MVS_ROOT    Hik MVS SDK root (default: /opt/MVS)
EOF
  exit 0
fi

INPUT_DIR="${INPUT_DIR_DEFAULT}"
if [[ $# -gt 0 && "${1}" != --* ]]; then
  INPUT_DIR="${1}"
  shift
fi

if [[ ! -x "${BIN}" ]]; then
  echo "[run_collect] demarcate not found, building target..."
  cmake -S "${PROJECT_DIR}" -B "${BUILD_DIR}" -DMVS_ROOT="${MVS_ROOT}"
  cmake --build "${BUILD_DIR}" --target demarcate -j"$(nproc)"
fi

echo "[run_collect] input directory: ${INPUT_DIR}"
echo "[run_collect] config: ${CONFIG}"
echo "[run_collect] extra args: $*"

exec "${BIN}" --collect --config="${CONFIG}" --input="${INPUT_DIR}" "$@"
