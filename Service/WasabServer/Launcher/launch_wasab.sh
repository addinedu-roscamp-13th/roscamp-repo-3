#!/usr/bin/env bash
set -u

PROJECT_ROOT="/home/ane/dev_ws/src/roscamp-repo-3"
LOG_ROOT="/tmp/wasab-launcher"

mkdir -p "$LOG_ROOT"
cd "$PROJECT_ROOT" || exit 1
exec /usr/bin/python3 \
  "$PROJECT_ROOT/Service/WasabServer/Launcher/wasab_launcher.py" \
  >>"$LOG_ROOT/startup.log" 2>&1
