#!/usr/bin/env bash
# run_fire_detect.sh — (PC) fire_detect_node.py 실행. run_fire_search.sh(JetCobot)와 도메인 통일 필요.
set -uo pipefail

set +u
source /opt/ros/jazzy/setup.bash
set -u

export ROS_DOMAIN_ID=69   # run_fire_search.sh(JetCobot)와 동일해야 /wasab/k3/fire 가 보임

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT"
exec python3 fire_detect_node.py --console-domain 50 --show
