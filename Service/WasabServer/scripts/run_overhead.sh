#!/usr/bin/env bash
# run_overhead.sh — 오버헤드 ArUco 맵 캘리브레이션 실행 (노트북)
#
# Usage: run_overhead.sh [-c|--camera <index>] [--width <px>] [--height <px>] [--config <path>] [--rotate180|--no-rotate180]
#   -c, --camera <index>   카메라 디바이스 인덱스 (config 덮어씀)
#       --width  <px>      프레임 폭 override
#       --height <px>      프레임 높이 override
#       --config <path>    overhead.yaml 경로
#       --rotate180        프레임 180도 회전(카메라가 맵과 반대로 장착)
#       --no-rotate180     180도 회전 끄기
#   -h, --help             도움말
#
# 의존: cv2(opencv-contrib)+numpy+pyyaml 가 있는 python.
#   WASAB_OVERHEAD_VENV 로 venv 지정 가능(없으면 시스템 python3 사용 — numpy1.26/cv2 4.9 보유).
set -eu
PROJ="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"

PY=python3
if [ -n "${WASAB_OVERHEAD_VENV:-}" ] && [ -f "$WASAB_OVERHEAD_VENV/bin/activate" ]; then
  # shellcheck disable=SC1091
  source "$WASAB_OVERHEAD_VENV/bin/activate"
  PY=python
fi

ARGS=()
while [ $# -gt 0 ]; do
  case "$1" in
    -c|--camera) ARGS+=(--camera "$2"); shift 2 ;;
    --width)     ARGS+=(--width "$2");  shift 2 ;;
    --height)    ARGS+=(--height "$2"); shift 2 ;;
    --config)    ARGS+=(--config "$2"); shift 2 ;;
    --rotate180) ARGS+=(--rotate180); shift ;;
    --no-rotate180) ARGS+=(--no-rotate180); shift ;;
    -h|--help)   sed -n '2,14p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; exit 0 ;;
    *) echo "unknown option: $1" >&2; exit 2 ;;
  esac
done

cd "$PROJ"
exec "$PY" -m wasab_overhead.calibrate "${ARGS[@]}"
