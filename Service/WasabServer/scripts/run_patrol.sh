#!/usr/bin/env bash
# run_patrol.sh — WaSaB 순찰 실행: PC에서 patrol_node 를 로봇 도메인으로 띄워 그 로봇 자율 순회.
#
# Usage: run_patrol.sh [--robot <id>] [--domain <id>] [--peer <ip>] [--console-domain <id>]
#   --robot <id>           순찰 로봇 id (robot_id, 기본 50)
#   --domain <id>          로봇 로컬 ROS_DOMAIN_ID (기본 51)
#   --peer <ip>            유니캐스트 discovery 피어 = 로봇 IP (ROS_STATIC_PEERS, 기본 192.168.2.9)
#   --console-domain <id>  heartbeat(타 로봇 회피) 도메인 (기본 50)
#   -h, --help             도움말
# 기본값 = 로봇50(검증됨). 버튼 연동 시 robots.yaml(id->domain->ip)에서 값 주입.
#   예) 다른 로봇:  run_patrol.sh --robot 87 --domain 52 --peer 192.168.2.13
# 포그라운드 실행 — Ctrl-C 로 정지.
set -eu
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
PROJ="${WASAB_WS_ROOT:-$(cd "$REPO_ROOT/../.." && pwd)}"

ROBOT=50                  # robot_id (patrol_node -p robot_id)
DOMAIN=51                 # 로봇 로컬 ROS_DOMAIN_ID (patrol_node robot_ctx 가 붙는 도메인)
PEER=192.168.2.9          # ROS_STATIC_PEERS (WiFi 멀티캐스트 불안정 → 로봇 IP 유니캐스트 지정)
CONSOLE_DOMAIN=50         # heartbeat(타 로봇 pose) 수신 도메인

while [ $# -gt 0 ]; do
  case "$1" in
    --robot) ROBOT="$2"; shift 2 ;;
    --domain) DOMAIN="$2"; shift 2 ;;
    --peer) PEER="$2"; shift 2 ;;
    --console-domain) CONSOLE_DOMAIN="$2"; shift 2 ;;
    -h|--help) sed -n '2,12p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; exit 0 ;;
    *) echo "unknown option: $1" >&2; exit 2 ;;
  esac
done

export ROS_DOMAIN_ID="$DOMAIN"
export ROS_STATIC_PEERS="$PEER"

set +eu
if [ -n "${ROS_DISTRO:-}" ] && [ -f "/opt/ros/$ROS_DISTRO/setup.bash" ]; then
  source "/opt/ros/$ROS_DISTRO/setup.bash"
elif [ -f /opt/ros/jazzy/setup.bash ]; then
  source /opt/ros/jazzy/setup.bash
fi
[ -f "$PROJ/install/setup.bash" ] && source "$PROJ/install/setup.bash"   # ros2 run wasab_patrol
set -eu

cd "$PROJ"
echo "[patrol] robot_id=$ROBOT  ROS_DOMAIN_ID=$ROS_DOMAIN_ID  peer=$ROS_STATIC_PEERS  console_domain=$CONSOLE_DOMAIN"
echo "[patrol] Ctrl-C 로 정지"
exec ros2 run wasab_patrol patrol_node --ros-args -p robot_id:="$ROBOT" -p console_domain:="$CONSOLE_DOMAIN"
