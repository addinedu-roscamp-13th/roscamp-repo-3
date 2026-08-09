#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="${WASAB_WS_ROOT:-$(cd "$SCRIPT_DIR/../../../../../.." && pwd)}"

cd "$WS_ROOT"
colcon build --base-paths "$SCRIPT_DIR" --packages-select pinky_yolo
source "$WS_ROOT/install/setup.bash"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-51}" ros2 run pinky_yolo ai_node \
  --robot-id "${WASAB_ROBOT_ID:-50}" \
  --console-domain "${WASAB_CONSOLE_DOMAIN:-50}"
