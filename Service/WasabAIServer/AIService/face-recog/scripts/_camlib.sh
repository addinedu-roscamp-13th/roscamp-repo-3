# _camlib.sh — start.sh/register.sh 공용 함수. (실행용 아님 — source 전용)
# 호출 전 설정 필요: VENV_PY(=.venv/bin/python)
# 환경변수: PI_PASS(기본 1), TARGET_IP(이 PC의 IP, 미지정시 자동감지)
PI_USER=jetcobot
PI_PASS="${PI_PASS:-1}"
TARGET_IP="${TARGET_IP:-$(hostname -I | awk '{print $1}')}"

# UDP 포트로 실제 프레임 청크가 들어오는지 검사(3초 이내 최소 1개). 0=LIVE, 1=아님.
# timeout 6: 바인드 실패/응답 없음으로 무한 대기하는 것을 차단(→ 1 반환).
stream_live() {
  timeout 6 "$VENV_PY" - "$1" <<'PY' 2>/dev/null
import sys, socket
port = int(sys.argv[1])
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.bind(("0.0.0.0", port))
s.settimeout(3.0)
try:
    s.recvfrom(65536)
    sys.exit(0)
except socket.timeout:
    sys.exit(1)
finally:
    s.close()
PY
}

# cam_server 가 LIVE 가 되도록 보장. 이미 LIVE 면 그대로, 아니면 (재)기동.
# 인자: $1=host $2=port $3=W $4=H  (TARGET_IP 전역 사용 — cam_server가 UDP를 쏠 이 PC의 IP)
ensure_cam() {
  local host="$1" port="$2" w="$3" h="$4" i
  if stream_live "$port"; then echo "[cam] 이미 LIVE"; return 0; fi
  echo "[cam] (재)기동 @ ${host}:${port} → target=$TARGET_IP ${w}x${h}"
  # (1) kill — 브래킷 패턴만 사용(명령 문자열에 'cam_server.py' 리터럴 없음 → self-match 회피).
  sshpass -p "$PI_PASS" ssh -o ConnectTimeout=10 -o StrictHostKeyChecking=accept-new "$PI_USER@$host" \
    'pids=$(pgrep -f "[c]am_server.py"); [ -n "$pids" ] && kill $pids && sleep 1; exit 0' || true
  # (2) launch — kill 없음(리터럴 'cam_server.py' 있어도 self-match 무관). nohup+setsid 로 분리.
  sshpass -p "$PI_PASS" ssh -o ConnectTimeout=10 "$PI_USER@$host" \
    "cd ~/wasab && nohup setsid python3 cam_server.py --camera 0 --target $TARGET_IP --port $port --width $w --height $h >cam_server.log 2>&1 </dev/null & sleep 2; exit 0" || true
  # (3) verify — LIVE 될 때까지 폴링.
  for i in $(seq 1 15); do stream_live "$port" && { echo "[cam] LIVE"; return 0; }; sleep 1; done
  echo "[cam] ❌ 스트림 미수신 — Pi/카메라 확인" >&2; return 1
}
