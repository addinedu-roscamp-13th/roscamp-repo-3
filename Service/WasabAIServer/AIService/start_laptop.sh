#!/usr/bin/env bash
# start_laptop.sh — (노트북) 얼굴+손목 통합 인지 perception_node 실행 (.venv).
# insightface(얼굴)+MediaPipe Hands(손목)를 한 프로세스에서 같은 프레임으로 처리해
# /identity, /wrist 를 동시 발행한다(얼굴/손 딜레이 없음). 종료는 stop_laptop.sh.
# 사용법: scripts/start_laptop.sh [옵션]
#   -H, --host <ip>    RPi(cam_server) 주소  (기본 192.168.0.86)
#   -P, --port <port>  스트림 포트           (기본 8090)
#       --show-view    얼굴 bbox + 손목 점 오버레이 창
#   -h, --help         이 도움말
set -uo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
HOST=192.168.0.86; PORT=8090; SHOW=0
FR_DIR="$ROOT/face-recog"; VENV_PY="${FACE_VENV:-$HOME/face-recog/.venv}/bin/python"
usage() { grep -E '^# ' "${BASH_SOURCE[0]}" | sed 's/^# \?//'; }

while [ $# -gt 0 ]; do
  case "$1" in
    -H|--host) HOST="$2"; shift 2;;
    -P|--port) PORT="$2"; shift 2;;
    --show-view) SHOW=1; shift;;
    -h|--help)   usage; exit 0;;
    *) echo "알 수 없는 옵션: $1" >&2; usage; exit 1;;
  esac
done
STREAM="http://$HOST:$PORT/stream"
export ROS_DOMAIN_ID=69   # 노트북·RPi 통일(터미널 잔존 도메인 무시)

if [ ! -x "$VENV_PY" ]; then
  echo "[laptop] ⚠ face-recog .venv 없음: $VENV_PY" >&2
  echo "  cd face-recog && python3 -m venv .venv && pip install -r requirements.txt" >&2
  exit 1
fi

# rclpy 환경 (perception_node 는 .venv python 이지만 ROS PYTHONPATH 필요).
# perception_node.py가 wasab_k3_mimic(제스처 로직) 패키지를 직접 import 하므로
# 시스템 ROS만으론 부족 — colcon으로 빌드된 워크스페이스 install이 필요함.
# 이 파일은 <워크스페이스>/src/roscamp-repo-3/Service/WasabAIServer/ 에 있다고 가정하고
# 4단계 위(<워크스페이스>/install/setup.bash)를 소싱한다. 다른 위치에 두는 경우
# WS_INSTALL 환경변수로 override.
WS_INSTALL="${WS_INSTALL:-$ROOT/../../../../install/setup.bash}"
set +eu
# shellcheck source=/dev/null
source "$WS_INSTALL"
set -u

ARGS=(--source "$STREAM")
[ "$SHOW" = 1 ] && ARGS+=(--show)
cd "$FR_DIR"
echo "[laptop] perception_node (얼굴+손목 통합) 실행 (Ctrl+C 종료)"
exec "$VENV_PY" perception_node.py "${ARGS[@]}"
