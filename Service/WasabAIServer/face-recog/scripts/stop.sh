#!/usr/bin/env bash
# stop.sh — Pi cam_server 종료 + 로컬 run.py/register.py/preview 정리.
# 사용법: scripts/stop.sh [옵션]
#   -H, --host <ip>   Pi 주소 (기본 192.168.0.86)
#       --keep-cam    cam_server 는 두고 로컬 프로세스만 정리
#   -h, --help        이 도움말
# 환경변수: PI_PASS (기본 1)
set -uo pipefail
HOST=192.168.0.86; PI_USER=jetcobot; PI_PASS="${PI_PASS:-1}"; KEEP_CAM=0
usage() { grep -E '^# ' "${BASH_SOURCE[0]}" | sed 's/^# \?//'; }

while [ $# -gt 0 ]; do
  case "$1" in
    -H|--host) HOST="$2"; shift 2;;
    --keep-cam) KEEP_CAM=1; shift;;
    -h|--help) usage; exit 0;;
    *) echo "알 수 없는 옵션: $1" >&2; usage; exit 1;;
  esac
done

if [ "$KEEP_CAM" = 0 ]; then
  echo "[stop] Pi cam_server 종료 + 카메라 잠금 해제..."
  # 브래킷 패턴만 사용 → self-match 회피. PID 로 종료 후 카메라 디바이스 점유까지 해제.
  sshpass -p "$PI_PASS" ssh -o ConnectTimeout=10 -o StrictHostKeyChecking=accept-new "$PI_USER@$HOST" '
    # 1) cam_server 종료 (TERM → 잔존 시 KILL)
    pids=$(pgrep -f "[c]am_server.py")
    if [ -n "$pids" ]; then
      kill $pids 2>/dev/null; sleep 1
      rem=$(pgrep -f "[c]am_server.py"); [ -n "$rem" ] && kill -9 $rem 2>/dev/null
      echo "[pi] cam_server killed: $pids"
    else
      echo "[pi] cam_server 없음"
    fi
    # 2) 카메라 디바이스 잠금(점유 프로세스) 해제 — 다음 start 가 깨끗이 열도록
    for dev in /dev/jetcocam0 /dev/video0; do
      [ -e "$dev" ] || { echo "[pi] $dev 없음(스킵)"; continue; }
      h=$(fuser "$dev" 2>/dev/null)
      if [ -n "$h" ]; then fuser -k "$dev" 2>/dev/null; echo "[pi] $dev 잠금해제:$h"; else echo "[pi] $dev 잠금 없음"; fi
    done
  ' || echo "[stop] ⚠ Pi 접속 실패"
fi

echo "[stop] 로컬 face-recog 프로세스 정리..."
# 프로젝트 .venv python(run.py/register.py/임시 스크립트) + 프리뷰 = face-recog 관련 전부.
# pgrep 은 자기 자신 제외 + 패턴이 stop.sh(bash) 명령줄과 불일치 → self-match 회피. $$ 도 제외.
PAT='face-recog/\.venv|[c]am_preview\.py|[r]un\.py|[r]egister\.py'
pids=$(pgrep -f "$PAT" 2>/dev/null | grep -vw "$$" | tr '\n' ' ')
if [ -n "${pids// /}" ]; then
  kill $pids 2>/dev/null; sleep 1
  rem=$(pgrep -f "$PAT" 2>/dev/null | grep -vw "$$" | tr '\n' ' ')
  [ -n "${rem// /}" ] && kill -9 $rem 2>/dev/null
  echo "[local] 종료: ${pids}${rem}"
else
  echo "[local] 실행 중인 face-recog 프로세스 없음"
fi
echo "[stop] 완료"
