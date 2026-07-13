#!/usr/bin/env bash
# Direct Pi -> laptop-local service health check. No SSH tunnel is used.
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"
exec python3 check_server_connection.py
