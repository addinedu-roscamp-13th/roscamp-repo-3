#!/usr/bin/env bash
# run_follow_all.sh — 추종(Follow) A안(제스처 전용) 4개 프로세스를 한 번에 백그라운드로
# 띄우고, Ctrl+C 한 번으로 전부 정리 종료한다.
#
# 대상 4개: ①JetCobot start_rpi.sh ②PC start_laptop.sh ③PinkyPro pinky_node.py ④PC run_ai.sh
#
# 사용법: Service/WasabServer/scripts/run_follow_all.sh
#   (env) PC_IP        이 PC의 IP, JetCobot cam_server 타겟으로 씀 (기본 192.168.2.6)
#   (env) JETCOBOT_IP  JetCobot IP (기본 192.168.2.12)
#   (env) PINKY_IP     PinkyPro IP (기본 192.168.2.9)
#
# 로그는 Service/WasabServer/scripts/logs/*.log 에 남는다. 종료: Ctrl+C.
set -u

PC_IP="${PC_IP:-192.168.2.6}"
JETCOBOT_IP="${JETCOBOT_IP:-192.168.2.12}"
PINKY_IP="${PINKY_IP:-192.168.2.9}"
JETCOBOT_HOST="jetcobot@$JETCOBOT_IP"
PINKY_HOST="pinky@$PINKY_IP"
SSH="sshpass -p 1 ssh -o StrictHostKeyChecking=no"

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
LOG_DIR="$ROOT/Service/WasabServer/scripts/logs"
mkdir -p "$LOG_DIR"

PIDS=()

cleanup() {
  echo ""
  echo "[run_follow_all] 종료 중..."
  for pid in "${PIDS[@]:-}"; do
    kill "$pid" 2>/dev/null
  done
  # JetCobot의 cam_server.py는 start_rpi.sh 안에서 nohup으로 뜨기 때문에
  # ssh 세션이 끊겨도 안 죽는다 — 명시적으로 원격 pkill 필요(오늘 실제로 겪은 문제).
  $SSH "$JETCOBOT_HOST" 'pkill -f cam_server.py; pkill -f search.launch' >/dev/null 2>&1
  $SSH "$PINKY_HOST" 'pkill -f pinky_node.py' >/dev/null 2>&1
  echo "[run_follow_all] 전부 종료됨."
  exit 0
}
trap cleanup INT TERM

echo "[1/4] JetCobot start_rpi.sh (target=$PC_IP)"
$SSH "$JETCOBOT_HOST" "cd ~/wasab/scripts && ./start_rpi.sh -t $PC_IP" \
  > "$LOG_DIR/jetcobot.log" 2>&1 &
PIDS+=("$!")
sleep 3

echo "[2/4] PC start_laptop.sh"
( cd "$ROOT/Service/WasabAIServer/AIService" && ./start_laptop.sh ) \
  > "$LOG_DIR/start_laptop.log" 2>&1 &
PIDS+=("$!")
sleep 2

echo "[3/4] PinkyPro pinky_node.py"
$SSH "$PINKY_HOST" "python3 ~/wasab/pinky_node.py" \
  > "$LOG_DIR/pinky.log" 2>&1 &
PIDS+=("$!")
sleep 2

echo "[4/4] PC run_ai.sh"
( cd "$ROOT/Service/WasabAIServer/AIService/pinky_yolo" && ./run_ai.sh ) \
  > "$LOG_DIR/run_ai.log" 2>&1 &
PIDS+=("$!")

echo ""
echo "[run_follow_all] 4개 프로세스 실행 중 — 로그: $LOG_DIR/*.log"
echo "[run_follow_all] 종료하려면 이 터미널에서 Ctrl+C"
wait
