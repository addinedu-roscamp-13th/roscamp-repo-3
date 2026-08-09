#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PYTHON_BIN="${PYTHON_BIN:-python3}"

source /opt/ros/jazzy/setup.bash
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-69}"

exec "${PYTHON_BIN}" "${SCRIPT_DIR}/intruder_event_node.py" \
  --console-domain "${WASAB_CONSOLE_DOMAIN:-50}" \
  --pinky-domain "${WASAB_PINKY_DOMAIN:-51}" \
  "$@"
