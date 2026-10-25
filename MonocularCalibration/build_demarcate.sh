#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="${SCRIPT_DIR}"

BUILD_DIR="${BUILD_DIR:-${PROJECT_DIR}/build}"
BUILD_TYPE="${BUILD_TYPE:-Release}"
MVS_ROOT="${MVS_ROOT:-/opt/MVS}"
TARGET="${TARGET:-demarcate}"
JOBS="${JOBS:-$(nproc)}"
CLEAN=0

usage() {
  cat <<'EOF'
Usage:
  ./build_demarcate.sh [options]

Options:
  -d, --build-dir <dir>      Build directory (default: ./build or $BUILD_DIR)
  -t, --build-type <type>    CMake build type: Release/Debug/... (default: Release or $BUILD_TYPE)
  -m, --mvs-root <path>      Hik MVS SDK root (default: /opt/MVS or $MVS_ROOT)
  -j, --jobs <n>             Parallel build jobs (default: nproc or $JOBS)
  --target <name>            Build target (default: demarcate or $TARGET)
  --clean                    Remove build directory before configure/build
  -h, --help                 Show this help

Examples:
  ./build_demarcate.sh
  ./build_demarcate.sh --build-type Debug
  ./build_demarcate.sh --clean -j 8
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    -d|--build-dir)
      BUILD_DIR="$2"
      shift 2
      ;;
    -t|--build-type)
      BUILD_TYPE="$2"
      shift 2
      ;;
    -m|--mvs-root)
      MVS_ROOT="$2"
      shift 2
      ;;
    -j|--jobs)
      JOBS="$2"
      shift 2
      ;;
    --target)
      TARGET="$2"
      shift 2
      ;;
    --clean)
      CLEAN=1
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "[build_demarcate] unknown option: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

if [[ "${CLEAN}" -eq 1 && -d "${BUILD_DIR}" ]]; then
  echo "[build_demarcate] removing build directory: ${BUILD_DIR}"
  rm -rf "${BUILD_DIR}"
fi

echo "[build_demarcate] project:    ${PROJECT_DIR}"
echo "[build_demarcate] build dir:  ${BUILD_DIR}"
echo "[build_demarcate] build type: ${BUILD_TYPE}"
echo "[build_demarcate] mvs root:   ${MVS_ROOT}"
echo "[build_demarcate] target:     ${TARGET}"
echo "[build_demarcate] jobs:       ${JOBS}"

cmake -S "${PROJECT_DIR}" -B "${BUILD_DIR}" \
  -DCMAKE_BUILD_TYPE="${BUILD_TYPE}" \
  -DMVS_ROOT="${MVS_ROOT}"

cmake --build "${BUILD_DIR}" --target "${TARGET}" -j"${JOBS}"

echo "[build_demarcate] build completed: ${BUILD_DIR}/${TARGET}"
