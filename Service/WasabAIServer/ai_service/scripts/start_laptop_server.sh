#!/usr/bin/env bash
# Linux/macOS 노트북용 실행 스크립트. 가상환경을 활성화한 뒤 사용합니다.
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"
PYTHON_BIN="${PYTHON_BIN:-python3}"
exec "$PYTHON_BIN" run_server.py
