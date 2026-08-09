#!/usr/bin/env bash
# run_sensorpose_calib.sh — pinky_pro 오도메트리 휠 캘리브/검증 런처
#
# Usage: ./scripts/run_sensorpose_calib.sh <command>
#   calibrate   직진+회전 측정으로 wheel_radius/separation 산출 → calib.yaml
#   verify      현재 파라미터로 재측정, 잔차 게이트 통과 여부 출력
#   -h, --help  이 도움말
set -euo pipefail
SELF="$(cd "$(dirname "$0")" && pwd)/$(basename "$0")"
cd "$(dirname "$0")/.."   # repo root (wasab_sensorpose 패키지 import 가능)

cmd="${1:-}"
case "$cmd" in
  calibrate|verify)
    exec python3 -m wasab_sensorpose.calibrate "$@"
    ;;
  -h|--help|"")
    sed -n '4,7p' "$SELF" | sed 's/^# \{0,1\}//'
    ;;
  *)
    echo "알 수 없는 명령: $cmd (calibrate|verify)" >&2
    exit 1
    ;;
esac
