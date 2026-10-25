#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="${SCRIPT_DIR}"
BUILD_DIR="${BUILD_DIR:-${PROJECT_DIR}/build}"
BIN="${BUILD_DIR}/demarcate"
MVS_ROOT="${MVS_ROOT:-/opt/MVS}"

INPUT_DIR_DEFAULT="${PROJECT_DIR}/sub_module/Calibration/Input/camera"
OUTPUT_DIR_DEFAULT="${PROJECT_DIR}/sub_module/Calibration/Results/camera_$(date +%Y%m%d_%H%M%S)"

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  cat <<'EOF'
Usage:
  ./run_calibration.sh [input_dir] [output_dir] [--demarcate-options]

Examples:
  ./run_calibration.sh
  ./run_calibration.sh ./sub_module/Calibration/Input/camera_test
  ./run_calibration.sh ./sub_module/Calibration/Input/camera_test ./sub_module/Calibration/Results/my_result --model=full --min-views=12 --max-views=40

Environment:
  BUILD_DIR   Build directory (default: ./build)
  MVS_ROOT    Hik MVS SDK root (default: /opt/MVS)
EOF
  exit 0
fi

INPUT_DIR="${INPUT_DIR_DEFAULT}"
if [[ $# -gt 0 && "${1}" != --* ]]; then
  INPUT_DIR="${1}"
  shift
fi

OUTPUT_DIR="${OUTPUT_DIR_DEFAULT}"
if [[ $# -gt 0 && "${1}" != --* ]]; then
  OUTPUT_DIR="${1}"
  shift
fi

if [[ ! -x "${BIN}" ]]; then
  echo "[run_calibration] demarcate not found, building target..."
  cmake -S "${PROJECT_DIR}" -B "${BUILD_DIR}" -DMVS_ROOT="${MVS_ROOT}"
  cmake --build "${BUILD_DIR}" --target demarcate -j"$(nproc)"
fi

echo "[run_calibration] input directory: ${INPUT_DIR}"
echo "[run_calibration] output directory: ${OUTPUT_DIR}"
echo "[run_calibration] extra args: $*"

exec "${BIN}" --calibration --input="${INPUT_DIR}" --output="${OUTPUT_DIR}" "$@"
