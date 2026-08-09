#!/usr/bin/env bash
# start_agent.sh — WaSaB 로봇 heartbeat agent 기동 (로봇측에서 실행).
# 로봇 도메인(ROS_DOMAIN_ID)에서 map->base TF/battery를 읽고, 공유 콘솔 도메인
# (WASAB_CONSOLE_DOMAIN)으로 /robots/heartbeat 발행 + /robot_<id>/cmd_vel·/wasab/cmd_mode 브리지.
#
# Usage: start_agent.sh [--robot-domain <id>] [--console-domain <id>] [--home <dir>] [--foreground] [-h]
#   --robot-domain <id>    로봇 로컬 ROS 도메인(미지정 시 env ROS_DOMAIN_ID 사용)
#   --console-domain <id>  공유 콘솔 도메인(기본 env WASAB_CONSOLE_DOMAIN 또는 50)
#   --home <dir>           agent 패키지 경로(기본 Device/.../wasab_robot_agent)
#   --foreground           포그라운드 실행(기본: 백그라운드 + 로그 ~/agent.log)
#   -h, --help             도움말
set -eu

AGENT_HOME="${AGENT_HOME:-$HOME/wasab/Device/WasabBot/WasabMoveController/wasab_robot_agent}"
LOG="${AGENT_LOG:-$HOME/agent.log}"
CONSOLE_DOMAIN="${WASAB_CONSOLE_DOMAIN:-50}"
ROBOT_DOMAIN=""
FG=0
PAT='[w]asab_robot_agent.agent_node'

while [ $# -gt 0 ]; do
  case "$1" in
    --robot-domain) ROBOT_DOMAIN="$2"; shift 2 ;;
    --console-domain) CONSOLE_DOMAIN="$2"; shift 2 ;;
    --home) AGENT_HOME="$2"; shift 2 ;;
    --foreground|-f) FG=1; shift ;;
    -h|--help) sed -n '2,11p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; exit 0 ;;
    *) echo "unknown option: $1" >&2; exit 2 ;;
  esac
done

# 중복 방지(과거 .87 이중 노드 사례) — 이미 떠있으면 중단
if pgrep -f "$PAT" >/dev/null 2>&1; then
  echo "[agent] 이미 실행 중(pid $(pgrep -f "$PAT" | tr '\n' ' ')). 먼저 stop_agent.sh." >&2
  exit 1
fi

# ROS 환경 (비interactive 셸이라 .bashrc 의존 금지 — 명시적 source)
set +eu
if [ -n "${ROS_DISTRO:-}" ] && [ -f "/opt/ros/$ROS_DISTRO/setup.bash" ]; then
  source "/opt/ros/$ROS_DISTRO/setup.bash"
elif [ -f /opt/ros/jazzy/setup.bash ]; then
  source /opt/ros/jazzy/setup.bash
fi
# 워크스페이스 오버레이 — agent_node가 wasab_docking을 import하므로 필수
WS_SETUP="${WASAB_WS_SETUP:-$HOME/wasab/install/setup.bash}"
[ -f "$WS_SETUP" ] && source "$WS_SETUP"
set -eu

# PYTHONPATH는 append(ROS 경로 보존 — clobber 시 rclpy 사라짐)
export PYTHONPATH="$AGENT_HOME:${PYTHONPATH:-}"
[ -n "$ROBOT_DOMAIN" ] && export ROS_DOMAIN_ID="$ROBOT_DOMAIN"
export WASAB_CONSOLE_DOMAIN="$CONSOLE_DOMAIN"

if [ -z "${ROS_DOMAIN_ID:-}" ]; then
  echo "[agent] 경고: ROS_DOMAIN_ID 미설정 — 로봇 고유 도메인을 설정해야 함(.bashrc/export)." >&2
fi

# rclpy 사전 확인(명확한 에러)
if ! python3 -c "import rclpy" 2>/dev/null; then
  echo "[agent] 오류: rclpy import 실패. ROS2 source/PYTHONPATH 확인." >&2
  exit 3
fi

echo "[agent] ROS_DOMAIN_ID(로봇)=${ROS_DOMAIN_ID:-unset}  WASAB_CONSOLE_DOMAIN(공유)=$WASAB_CONSOLE_DOMAIN  home=$AGENT_HOME"

if [ "$FG" = "1" ]; then
  exec python3 -u -m wasab_robot_agent.agent_node
else
  nohup python3 -u -m wasab_robot_agent.agent_node > "$LOG" 2>&1 < /dev/null &
  sleep 1.5
  if pgrep -f "$PAT" >/dev/null 2>&1; then
    echo "[agent] 백그라운드 기동(pid $(pgrep -f "$PAT" | tr '\n' ' ')). 로그: $LOG"
  else
    echo "[agent] 기동 실패 — 로그:" >&2; tail -n 15 "$LOG" >&2; exit 4
  fi
fi
