#!/usr/bin/env bash
# run_overhead_pose.sh — 오버헤드 로봇 pose 발행 노드 (노트북, ROS2 필요)
#
# Usage: run_overhead_pose.sh [-c|--camera <i>] [--width <px>] [--height <px>] [--config <path>] [--rotate180|--no-rotate180] [--topic <t>] [--rate <hz>]
#   -c, --camera <i>     카메라 인덱스 (config 덮어씀)
#       --width/--height 해상도 override
#       --config <path>  overhead.yaml 경로
#       --rotate180 / --no-rotate180   프레임 180도 회전 토글
#       --topic <t>      발행 토픽 (기본 /wasab/overhead/poses)
#       --rate <hz>      발행 주기 (기본 10)
#   -h, --help           도움말
set -eu
PROJ="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"

# ROS2 source (nounset 회피)
set +eu
if [ -n "${ROS_DISTRO:-}" ] && [ -f "/opt/ros/$ROS_DISTRO/setup.bash" ]; then
  source "/opt/ros/$ROS_DISTRO/setup.bash"
elif [ -f /opt/ros/jazzy/setup.bash ]; then
  source /opt/ros/jazzy/setup.bash
fi
set -eu

ARGS=()
while [ $# -gt 0 ]; do
  case "$1" in
    -c|--camera) ARGS+=(--camera "$2"); shift 2 ;;
    --width)     ARGS+=(--width "$2");  shift 2 ;;
    --height)    ARGS+=(--height "$2"); shift 2 ;;
    --config)    ARGS+=(--config "$2"); shift 2 ;;
    --topic)     ARGS+=(--topic "$2");  shift 2 ;;
    --rate)      ARGS+=(--rate "$2");   shift 2 ;;
    --rotate180) ARGS+=(--rotate180);   shift ;;
    --no-rotate180) ARGS+=(--no-rotate180); shift ;;
    -h|--help)   sed -n '2,11p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; exit 0 ;;
    *) echo "unknown option: $1" >&2; exit 2 ;;
  esac
done

cd "$PROJ"
exec python3 -m wasab_overhead.pose_node "${ARGS[@]}"
