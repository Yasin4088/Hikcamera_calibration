#!/usr/bin/env bash
set -eo pipefail

ROOT_DIR="$(cd "$(dirname "$0")" && pwd)"
RVIZ_CONFIG="$ROOT_DIR/rviz/mid70_pointcloud.rviz"

if [ ! -f "$ROOT_DIR/install/setup.bash" ]; then
  echo "[visualize] install/setup.bash not found. Please build first:"
  echo "  cd $ROOT_DIR && colcon build --symlink-install"
  exit 1
fi

source "$ROOT_DIR/install/setup.bash"
set -u

find_latest_bag() {
  local latest=""
  while IFS= read -r dir; do
    if [ -f "$ROOT_DIR/$dir/metadata.yaml" ]; then
      latest="$ROOT_DIR/$dir"
      break
    fi
  done < <(cd "$ROOT_DIR" && ls -dt mid70_* 2>/dev/null || true)
  echo "$latest"
}

start_live_mode() {
  echo "[visualize] Live mode: no bag playback, opening RViz only."
  echo "[visualize] Ensure driver is running and publishing /pc_raw."
  rviz2 -d "$RVIZ_CONFIG"
}

start_bag_mode() {
  local bag_path="$1"
  local loop_play="${2:-true}"

  if [ ! -d "$bag_path" ]; then
    echo "[visualize] Bag path does not exist: $bag_path"
    exit 1
  fi
  if [ ! -f "$bag_path/metadata.yaml" ]; then
    echo "[visualize] Invalid bag directory (metadata.yaml not found): $bag_path"
    exit 1
  fi

  echo "[visualize] Playing bag: $bag_path"
  if [ "$loop_play" = "true" ]; then
    ros2 bag play "$bag_path" --loop >/tmp/mid70_bag_play.log 2>&1 &
  else
    ros2 bag play "$bag_path" >/tmp/mid70_bag_play.log 2>&1 &
  fi
  local bag_pid=$!
  echo "[visualize] bag play pid: $bag_pid"

  cleanup() {
    if kill -0 "$bag_pid" >/dev/null 2>&1; then
      kill "$bag_pid" >/dev/null 2>&1 || true
    fi
  }
  trap cleanup EXIT INT TERM

  sleep 1
  rviz2 -d "$RVIZ_CONFIG"
}

bag_arg="${1:-}"
loop_play="true"

if [ "${2:-}" = "--no-loop" ] || [ "${1:-}" = "--no-loop" ]; then
  loop_play="false"
fi

if [ "$bag_arg" = "--live" ]; then
  start_live_mode
  exit 0
fi

if [ -z "$bag_arg" ] || [ "$bag_arg" = "--no-loop" ]; then
  bag_arg="$(find_latest_bag)"
  if [ -z "$bag_arg" ]; then
    echo "Usage:"
    echo "  $0 <bag_dir> [--no-loop]   # play bag then open RViz"
    echo "  $0 --live                  # open RViz for live /pc_raw"
    exit 1
  fi
fi

if [[ "$bag_arg" != /* ]]; then
  bag_arg="$ROOT_DIR/$bag_arg"
fi

start_bag_mode "$bag_arg" "$loop_play"
