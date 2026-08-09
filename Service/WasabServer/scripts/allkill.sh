#!/usr/bin/env bash
# allkill.sh — 떠 있는 ROS2 관련 프로세스를 모두 종료(클린 슬레이트).
#
# Usage: allkill.sh [-n|--dry-run] [-y|--yes] [-h|--help]
#   -n, --dry-run   종료 대상만 출력, 실제로 죽이지 않음
#   -y, --yes       확인 프롬프트 없이 즉시 종료
#   -h, --help      이 도움말
#
# 종료 대상: ros2 cli/launch/daemon, rviz2, slam_toolbox, nav2 노드,
#   robot_state_publisher, component_container, sllidar, teleop, pinky_*.
# 이 스크립트 자신과 부모 셸 PID 는 항상 제외(pkill 자기-매칭 함정 방어).
set -u

PATTERNS=(
  "ros2 launch" "ros2 run" "ros2 daemon" "_ros2_daemon" "ros2cli"
  "rviz2" "slam_toolbox"
  "controller_server" "planner_server" "bt_navigator" "behavior_server"
  "smoother_server" "velocity_smoother" "waypoint_follower"
  "amcl" "map_server" "map_saver" "lifecycle_manager" "nav2_"
  "robot_state_publisher" "component_container" "static_transform_publisher"
  "sllidar" "teleop_twist_keyboard" "pinky_"
)

dry=0; yes=0
while [ $# -gt 0 ]; do
  case "$1" in
    -n|--dry-run) dry=1; shift ;;
    -y|--yes)     yes=1; shift ;;
    -h|--help)    sed -n '2,11p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
    *) echo "unknown option: $1" >&2; exit 2 ;;
  esac
done

self=$$; parent=$PPID
# 패턴 매칭 PID 수집 — 자기/부모 셸 제외
pids=$(for p in "${PATTERNS[@]}"; do pgrep -f -- "$p"; done \
         | sort -un | grep -vxE "$self|$parent")

if [ -z "$pids" ]; then
  echo "ROS2 관련 프로세스 없음."
  exit 0
fi

echo "== 종료 대상 =="
for pid in $pids; do
  ps -p "$pid" -o pid=,cmd= 2>/dev/null | cut -c1-110
done

[ "$dry" -eq 1 ] && { echo "(dry-run: 종료 안 함)"; exit 0; }

if [ "$yes" -eq 0 ]; then
  printf "위 프로세스를 종료할까요? [y/N] "
  read -r ans
  case "$ans" in y|Y) ;; *) echo "취소."; exit 0 ;; esac
fi

kill $pids 2>/dev/null            # SIGTERM
sleep 1.5
for pid in $pids; do              # 잔존분 SIGKILL
  kill -0 "$pid" 2>/dev/null && kill -9 "$pid" 2>/dev/null
done

command -v ros2 >/dev/null 2>&1 && ros2 daemon stop >/dev/null 2>&1
echo "완료."
