#!/usr/bin/env bash
# register.sh — Pi cam_server 기동 + 얼굴 등록(register.py).
# 사용법: scripts/register.sh <이름> [옵션]
#   <이름>               등록자 이름 (예: stephen) — 첫 인자(위치)
#   -H, --host <ip>      Pi 주소        (기본 192.168.0.86)
#   -P, --port <port>    UDP 포트       (기본 8090)
#   -r, --res  <WxH>     해상도         (기본 640x480, 표준 모드만)
#       --scale <f>      등록 화면 표시 배율(기본 1.0, 예: 1.5 또는 2.0)
#       --local [idx]    Pi cam_server 대신 로컬 카메라(노트북 내장/USB, 기본 index 0) 직접 사용
#   -h, --help           이 도움말
# 환경변수: PI_PASS (기본 1), FACE_VENV (기본 ~/face-recog/.venv — perception 과 동일)
# 예: scripts/register.sh stephen            (JetCobot 캠)
#     scripts/register.sh stephen --local    (노트북 내장/USB 캠)
set -uo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VENV_PY="${FACE_VENV:-$HOME/face-recog/.venv}/bin/python"
# shellcheck source=/dev/null
source "$ROOT/scripts/_camlib.sh"

if [ ! -x "$VENV_PY" ]; then
  echo "[register] ⚠ insightface venv 없음: $VENV_PY" >&2
  echo "  FACE_VENV 로 경로 지정하거나 perception 과 같은 venv 를 쓰세요." >&2
  exit 1
fi

HOST=192.168.0.86; PORT=8090; RES=640x480; NAME=""; USE_LOCAL=0; LOCAL_IDX=0; SCALE=1.0
usage() { grep -E '^# ' "${BASH_SOURCE[0]}" | sed 's/^# \?//'; }

while [ $# -gt 0 ]; do
  case "$1" in
    -H|--host) HOST="$2"; shift 2;;
    -P|--port) PORT="$2"; shift 2;;
    -r|--res)  RES="$2"; shift 2;;
    --scale) SCALE="$2"; shift 2;;
    --local) USE_LOCAL=1; if [[ "${2:-}" =~ ^[0-9]+$ ]]; then LOCAL_IDX="$2"; shift 2; else shift; fi;;
    -h|--help) usage; exit 0;;
    -*) echo "알 수 없는 옵션: $1" >&2; usage; exit 1;;
    *)  NAME="$1"; shift;;
  esac
done
[ -z "$NAME" ] && { echo "이름이 필요합니다. 예: scripts/register.sh stephen" >&2; usage; exit 1; }

# 로컬 카메라(노트북 내장/USB) 모드 — cam_server·Pi 건너뜀
if [ "$USE_LOCAL" = 1 ]; then
  echo "[register] '$NAME' 등록 — 로컬 카메라(index $LOCAL_IDX), SPACE 캡처 / q 종료"
  cd "$ROOT"
  exec "$VENV_PY" register.py --name "$NAME" --source "$LOCAL_IDX" --display-scale "$SCALE"
fi

W="${RES%x*}"; H="${RES#*x}"
ensure_cam "$HOST" "$PORT" "$W" "$H" || exit 1

echo "[register] '$NAME' 등록 — SPACE 캡처 / q 종료"
cd "$ROOT"
exec "$VENV_PY" register.py --name "$NAME" --udp-port "$PORT" --display-scale "$SCALE"
