#!/usr/bin/env bash
# stop_laptop.sh — (노트북 에서 실행) motion_mimic + identity_publisher 종료.
# 사용법: scripts/stop_laptop.sh [옵션]
#   -h, --help   이 도움말
set -uo pipefail
case "${1:-}" in
  -h|--help) grep -E '^# ' "${BASH_SOURCE[0]}" | sed 's/^# \?//'; exit 0;;
esac

PAT='[p]erception_node.py|[m]otion_mimic|[i]dentity_publisher.py'
pids=$(pgrep -f "$PAT" | grep -vw "$$" | tr '\n' ' ')
if [ -n "${pids// /}" ]; then
  kill $pids 2>/dev/null; sleep 1
  rem=$(pgrep -f "$PAT" | grep -vw "$$" | tr '\n' ' ')
  [ -n "${rem// /}" ] && kill -9 $rem 2>/dev/null
  echo "[laptop] 종료: ${pids}${rem}"
else
  echo "[laptop] 실행 중인 K3 프로세스 없음"
fi
