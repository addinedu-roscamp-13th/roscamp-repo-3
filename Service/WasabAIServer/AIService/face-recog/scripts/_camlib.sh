# _camlib.sh — start.sh/register.sh 공용 함수. (실행용 아님 — source 전용)
# 호출 전 설정 필요: VENV_PY(=.venv/bin/python), STREAM(=http://host:port/stream)
# 환경변수: PI_PASS(기본 1)
PI_USER=jetcobot
PI_PASS="${PI_PASS:-1}"

# 스트림이 실제로 흐르는지(2프레임 diff>1.0) 검사. 0=LIVE, 1=아님.
# timeout 6: frozen/기동중 서버에서 read() 가 무한 대기하는 것을 차단(→ 1 반환).
stream_live() {
  timeout 6 "$VENV_PY" - "$1" <<'PY' 2>/dev/null
import sys, cv2, time, numpy as np
c = cv2.VideoCapture(sys.argv[1])
ok1, f1 = c.read(); time.sleep(0.8); ok2, f2 = c.read(); c.release()
ok = ok1 and ok2 and np.abs(f1.astype(int) - f2.astype(int)).mean() > 1.0
sys.exit(0 if ok else 1)
PY
}

# cam_server 가 LIVE 가 되도록 보장. 이미 LIVE 면 그대로, 아니면 (재)기동.
# 인자: $1=host $2=port $3=W $4=H  (STREAM 전역 사용)
ensure_cam() {
  local host="$1" port="$2" w="$3" h="$4" i
  if stream_live "$STREAM"; then echo "[cam] 이미 LIVE"; return 0; fi
  echo "[cam] (재)기동 @ ${host}:${port} ${w}x${h}"
  # (1) kill — 브래킷 패턴만 사용(명령 문자열에 'cam_server.py' 리터럴 없음 → self-match 회피).
  sshpass -p "$PI_PASS" ssh -o ConnectTimeout=10 -o StrictHostKeyChecking=accept-new "$PI_USER@$host" \
    'pids=$(pgrep -f "[c]am_server.py"); [ -n "$pids" ] && kill $pids && sleep 1; exit 0' || true
  # (2) launch — kill 없음(리터럴 'cam_server.py' 있어도 self-match 무관). nohup+setsid 로 분리.
  sshpass -p "$PI_PASS" ssh -o ConnectTimeout=10 "$PI_USER@$host" \
    "cd ~/wasab && nohup setsid python3 cam_server.py --camera 0 --port $port --width $w --height $h >cam_server.log 2>&1 </dev/null & sleep 2; exit 0" || true
  # (3) verify — LIVE 될 때까지 폴링.
  for i in $(seq 1 15); do stream_live "$STREAM" && { echo "[cam] LIVE"; return 0; }; sleep 1; done
  echo "[cam] ❌ 스트림 미수신 — Pi/카메라 확인" >&2; return 1
}
