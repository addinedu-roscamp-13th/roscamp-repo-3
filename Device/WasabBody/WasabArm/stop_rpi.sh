#!/usr/bin/env bash
# stop_rpi.sh — (RPi 에서 실행) arm_search + cam_server 종료 + 카메라 잠금 해제.
# 사용법: scripts/stop_rpi.sh [옵션]
#       --keep-cam   cam_server 는 두고 arm_search 만 종료
#   -h, --help       이 도움말
set -uo pipefail
KEEP_CAM=0
usage() { grep -E '^# ' "${BASH_SOURCE[0]}" | sed 's/^# \?//'; }

while [ $# -gt 0 ]; do
  case "$1" in
    --keep-cam) KEEP_CAM=1; shift;;
    -h|--help)  usage; exit 0;;
    *) echo "알 수 없는 옵션: $1" >&2; usage; exit 1;;
  esac
done

_kill_pat() { # $1=설명 $2=pgrep패턴
  local pids; pids=$(pgrep -f "$2" | grep -vw "$$" | tr '\n' ' ')
  if [ -n "${pids// /}" ]; then
    kill $pids 2>/dev/null; sleep 1
    local rem; rem=$(pgrep -f "$2" | grep -vw "$$" | tr '\n' ' ')
    [ -n "${rem// /}" ] && kill -9 $rem 2>/dev/null
    echo "[rpi] $1 종료: ${pids}${rem}"
  else
    echo "[rpi] $1 없음"
  fi
}

echo "[rpi] arm_search 종료..."
_kill_pat "arm_search" 'search.launch.py|[a]rm_search'

if [ "$KEEP_CAM" = 0 ]; then
  echo "[rpi] cam_server 종료 + 카메라 해제..."
  _kill_pat "cam_server" '[c]am_server.py'
  for dev in /dev/video0; do
    [ -e "$dev" ] || continue
    h=$(fuser "$dev" 2>/dev/null) || true
    if [ -n "${h// /}" ]; then fuser -k "$dev" 2>/dev/null; echo "[rpi] $dev 잠금해제:$h"; fi
  done
fi
echo "[rpi] 완료"
