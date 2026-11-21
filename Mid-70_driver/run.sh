#!/usr/bin/env bash
set -e

cd "$(dirname "$0")"
source ./install/setup.bash
ros2 launch ./launch/mid70_driver_launch.py
