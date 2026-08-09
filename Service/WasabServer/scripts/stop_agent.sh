#!/usr/bin/env bash
# stop_agent.sh — WaSaB 로봇 heartbeat agent(start_agent.sh) 확실 종료 (로봇측).
# TERM 후 짧게 폴링, 남으면 KILL. 중복 프로세스도 모두 정리.
# 사용법: stop_agent.sh [옵션]
#   -h, --help   이 도움말
set -uo pipefail
case "${1:-}" in
  -h|--help) grep -E '^# ' "${BASH_SOURCE[0]}" | sed 's/^# \?//'; exit 0;;
esac

PAT='[w]asab_robot_agent.agent_node'
_pids() { pgrep -f "$PAT" | grep -vw "$$"; }

pids=$(_pids | tr '\n' ' ')
if [ -z "${pids// /}" ]; then
  echo "[agent] 실행 중인 agent 없음"
  exit 0
fi

echo "[agent] 종료 시도(TERM): $pids"
kill -TERM $pids 2>/dev/null || true

for _ in $(seq 1 8); do          # 최대 ~2.4s 폴링
  [ -z "$(_pids | tr -d '\n ')" ] && { echo "[agent] 종료 완료"; exit 0; }
  sleep 0.3
done

rem=$(_pids | tr '\n' ' ')
if [ -n "${rem// /}" ]; then
  echo "[agent] TERM 미응답 → 강제 종료(KILL): $rem"
  kill -9 $rem 2>/dev/null || true
  sleep 0.5
fi

if [ -z "$(_pids | tr -d '\n ')" ]; then
  echo "[agent] 종료 완료"
else
  echo "[agent] 경고: 잔존: $(_pids | tr '\n' ' ')" >&2
  exit 1
fi
