#!/usr/bin/env bash
# run_webapp.sh — Service/WasabServer/WebService 백엔드 + UI/mobile/user_gui 실행.
#
# Usage: run_webapp.sh [--no-ros] [--domain <id>] [--host <host>] [--port <port>]
#   --no-ros       ROS 미연결 GUI/로봇팔 테스트 모드
#   --domain <id>  통합 콘솔 ROS domain (기본 50)
#   --host <host>  바인드 주소 (기본 127.0.0.1)
#   --port <port>  GUI 포트 (기본 8100)
#   -h, --help     도움말
set -eu

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
SERVICE_ROOT="$PROJECT_ROOT/Service/WasabServer/WebService"
PYTHON_BIN="${WASAB_WEBSERVICE_PYTHON:-/home/ane/dev_ws/.venv-server/bin/python}"
WANT_ROS=1
DOMAIN=50
HOST=127.0.0.1
PORT=8100

while [ $# -gt 0 ]; do
  case "$1" in
    --no-ros) WANT_ROS=0; shift ;;
    --domain) DOMAIN="$2"; shift 2 ;;
    --host) HOST="$2"; shift 2 ;;
    --port) PORT="$2"; shift 2 ;;
    -h|--help) sed -n '2,9p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; exit 0 ;;
    *) echo "unknown option: $1" >&2; exit 2 ;;
  esac
done

if [ ! -x "$PYTHON_BIN" ]; then
  echo "[webapp] Python 환경 없음: $PYTHON_BIN" >&2
  echo "[webapp] WASAB_WEBSERVICE_PYTHON으로 FastAPI 환경을 지정하세요." >&2
  exit 1
fi

export PYTHONPATH="$SERVICE_ROOT${PYTHONPATH:+:$PYTHONPATH}"
export WASAB_WEBSERVICE_ROS="$WANT_ROS"
export WASAB_CONSOLE_DOMAIN="$DOMAIN"
export ROS_DOMAIN_ID="$DOMAIN"
export ROS_STATIC_PEERS="${ROS_STATIC_PEERS:-192.168.2.15;192.168.2.9;192.168.2.13;192.168.2.11}"
export WASAB_WEBAPP_SECURE_COOKIES=0
export WASAB_WEBAPP_ORIGIN="http://$HOST:$PORT,http://127.0.0.1:$PORT,http://localhost:$PORT"
export WASAB_ARM_API_URL="${WASAB_ARM_API_URL:-http://192.168.2.8:8000}"

if [ "$WANT_ROS" = "1" ]; then
  set +eu
  [ -f /opt/ros/jazzy/setup.bash ] && source /opt/ros/jazzy/setup.bash
  set -eu
fi

cd "$SERVICE_ROOT"
echo "[webapp] UI: http://$HOST:$PORT/"
echo "[webapp] arm API: $WASAB_ARM_API_URL"
exec "$PYTHON_BIN" -m uvicorn wasab_web_service.main:app \
  --host "$HOST" --port "$PORT" --log-level warning
