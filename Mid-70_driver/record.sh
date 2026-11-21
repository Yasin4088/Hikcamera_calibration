#!/usr/bin/env bash
set -e

if [ $# -lt 1 ]; then
  echo "Usage: $0 <bag_name_or_path>"
  echo "Example: $0 mid70_$(date +%Y%m%d_%H%M%S)"
  exit 1
fi

cd "$(dirname "$0")"
source ./install/setup.bash

bag_path="$1"
shift || true

# 默认录制驱动输出与 TF，可在命令后继续追加自定义 topic。
ros2 bag record -o "$bag_path" /pc_raw /tf /tf_static "$@"
