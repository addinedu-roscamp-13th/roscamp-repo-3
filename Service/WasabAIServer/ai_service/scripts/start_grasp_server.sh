#!/usr/bin/env bash
# 이전 스크립트명을 유지한 호환용 래퍼입니다.
# 원격 딥러닝 서버/SSH 터널 없이 현재 노트북에서 실행합니다.
set -euo pipefail
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"
PYTHON_BIN="${PYTHON_BIN:-python3}"
exec "$PYTHON_BIN" run_server.py
