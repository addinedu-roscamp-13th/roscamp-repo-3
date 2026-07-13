#!/usr/bin/env bash
# start.sh — Pi cam_server 기동 + 라이브 얼굴 인식(run.py).
# 사용법: scripts/start.sh [옵션]
#   -H, --host <ip>      Pi(cam_server) 주소  (기본 192.168.0.86)
#   -P, --port <port>    cam_server 포트      (기본 8090)
#   -r, --res  <WxH>     카메라 해상도        (기본 640x480, 표준 모드만)
#       --no-display     run.py 헤드리스(콘솔 이벤트만)
#       --reload         등록 얼굴 재임베딩
#   -t, --tolerance <f>  cosine 임계값 override
#   -m, --model <name>   buffalo_sc | buffalo_l
#       --cam-only       cam_server 만 기동(run.py 미실행)
#       --local [idx]    Pi cam_server 대신 로컬 카메라(노트북 내장/USB, 기본 index 0) 직접 사용
#   -h, --help           이 도움말
# 환경변수: PI_PASS (Pi SSH 비밀번호, 기본 1)
set -uo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VENV_PY="$ROOT/.venv/bin/python"
# shellcheck source=/dev/null
source "$ROOT/scripts/_camlib.sh"

HOST=192.168.0.86; PORT=8090; RES=640x480; CAM_ONLY=0; USE_LOCAL=0; LOCAL_IDX=0; EXTRA=()
usage() { grep -E '^# ' "${BASH_SOURCE[0]}" | sed 's/^# \?//'; }

while [ $# -gt 0 ]; do
  case "$1" in
    -H|--host) HOST="$2"; shift 2;;
    -P|--port) PORT="$2"; shift 2;;
    -r|--res)  RES="$2"; shift 2;;
    --no-display) EXTRA+=(--no-display); shift;;
    --reload)     EXTRA+=(--reload); shift;;
    -t|--tolerance) EXTRA+=(--tolerance "$2"); shift 2;;
    -m|--model)     EXTRA+=(--model "$2"); shift 2;;
    --cam-only) CAM_ONLY=1; shift;;
    --local) USE_LOCAL=1; if [[ "${2:-}" =~ ^[0-9]+$ ]]; then LOCAL_IDX="$2"; shift 2; else shift; fi;;
    -h|--help)  usage; exit 0;;
    *) echo "알 수 없는 옵션: $1" >&2; usage; exit 1;;
  esac
done

# 로컬 카메라(노트북 내장/USB) 모드 — cam_server·Pi 건너뜀
if [ "$USE_LOCAL" = 1 ]; then
  echo "[start] 로컬 카메라(index $LOCAL_IDX) 사용 — cam_server 건너뜀, q 종료"
  cd "$ROOT"
  exec "$VENV_PY" run.py --source "$LOCAL_IDX" ${EXTRA[@]+"${EXTRA[@]}"}
fi

W="${RES%x*}"; H="${RES#*x}"; STREAM="http://$HOST:$PORT/stream"
ensure_cam "$HOST" "$PORT" "$W" "$H" || exit 1
[ "$CAM_ONLY" = 1 ] && { echo "[start] --cam-only 완료 ($STREAM)"; exit 0; }

echo "[start] run.py 실행 — q 종료"
cd "$ROOT"
exec "$VENV_PY" run.py --source "$STREAM" ${EXTRA[@]+"${EXTRA[@]}"}
