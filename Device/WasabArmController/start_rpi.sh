#!/usr/bin/env bash
# start_rpi.sh — (RPi 에서 실행) K3 카메라 스트림 + 팔 상태머신 기동.
# cam_server(MJPEG) 를 백그라운드로 보장 기동하고, arm_search(search.launch.py) 를
# 포그라운드로 실행한다(Ctrl+C 로 팔 정지). 종료는 stop_rpi.sh.
# 사용법: scripts/start_rpi.sh [옵션]
#   -c, --camera <idx>   카메라 인덱스      (기본 0)
#   -P, --port   <port>  cam_server 포트    (기본 8090)
#       --cam-only       cam_server 만 기동(팔 미실행)
#       --no-cam         cam_server 건너뛰고 팔만 실행
#   -h, --help           이 도움말
# ⚠ arm_bridge.launch.py 는 실행 금지(search_node 와 팔 제어 충돌).
set -uo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CAM=0; PORT=8090; CAM_ONLY=0; NO_CAM=0
LOG_DIR="$ROOT/log"; CAM_LOG="$LOG_DIR/cam_server.log"
usage() { grep -E '^# ' "${BASH_SOURCE[0]}" | sed 's/^# \?//'; }

while [ $# -gt 0 ]; do
  case "$1" in
    -c|--camera) CAM="$2"; shift 2;;
    -P|--port)   PORT="$2"; shift 2;;
    --cam-only)  CAM_ONLY=1; shift;;
    --no-cam)    NO_CAM=1; shift;;
    -h|--help)   usage; exit 0;;
    *) echo "알 수 없는 옵션: $1" >&2; usage; exit 1;;
  esac
done
mkdir -p "$LOG_DIR"
export ROS_DOMAIN_ID=69   # 노트북·RPi 통일(터미널 잔존 도메인 무시)

# 1) cam_server 보장 기동 (이미 떠있으면 스킵)
if [ "$NO_CAM" = 0 ]; then
  if pgrep -f "[c]am_server.py" >/dev/null; then
    echo "[rpi] cam_server 이미 실행 중"
  else
    echo "[rpi] cam_server 기동 (camera=$CAM port=$PORT) → $CAM_LOG"
    nohup python3 "$ROOT/cam_server.py" --camera "$CAM" --port "$PORT" >"$CAM_LOG" 2>&1 &
    sleep 2
    if pgrep -f "[c]am_server.py" >/dev/null; then
      echo "[rpi] cam_server OK (http://0.0.0.0:$PORT/stream)"
    else
      echo "[rpi] ⚠ cam_server 기동 실패 — $CAM_LOG 확인" >&2
    fi
  fi
fi
[ "$CAM_ONLY" = 1 ] && { echo "[rpi] --cam-only 완료"; exit 0; }

# 2) 팔 상태머신 (포그라운드, Ctrl+C 로 정지)
# ROS setup.bash 는 nounset 에서 죽으므로 source 구간만 -e/-u 끔.
set +eu
# shellcheck source=/dev/null
source "$ROOT/install/setup.bash"
set -u
echo "[rpi] arm_search 실행 (Ctrl+C 정지) — 팔 주변 공간/카메라 케이블 확인"
exec ros2 launch wasab_k3_mimic search.launch.py
