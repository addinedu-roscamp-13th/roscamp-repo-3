# WaSaB scripts bash completion
#
# Bootstrap: ~/.bashrc 에 아래 한 줄 추가 (노트북·RPi 양쪽)
#   [ -f "$HOME/dev_ws/wasab/scripts/completion.bash" ] && \
#     source "$HOME/dev_ws/wasab/scripts/completion.bash"
#   (RPi 는 경로가 $HOME/wasab/scripts/completion.bash)

# --- run_camview.sh (카메라 뷰, 노트북) ---
_run_camview() {
  local cur prev opts
  COMPREPLY=()
  cur="${COMP_WORDS[COMP_CWORD]}"; prev="${COMP_WORDS[COMP_CWORD-1]}"
  opts="-u --url -c --conf -r --raw -h --help"
  case "$prev" in
    -u|--url) return ;;
    -c|--conf) COMPREPLY=( $(compgen -W "0.3 0.4 0.5 0.6 0.7" -- "$cur") ); return ;;
  esac
  COMPREPLY=( $(compgen -W "$opts" -- "$cur") )
}
complete -F _run_camview run_camview.sh ./run_camview.sh

# --- start_rpi.sh (RPi: 스트림+팔) ---
_start_rpi() {
  local cur prev opts
  COMPREPLY=()
  cur="${COMP_WORDS[COMP_CWORD]}"; prev="${COMP_WORDS[COMP_CWORD-1]}"
  opts="-c --camera -P --port --cam-only --no-cam -h --help"
  case "$prev" in
    -c|--camera) COMPREPLY=( $(compgen -W "0 1 2" -- "$cur") ); return ;;
    -P|--port) return ;;
  esac
  COMPREPLY=( $(compgen -W "$opts" -- "$cur") )
}
complete -F _start_rpi start_rpi.sh ./start_rpi.sh

# --- stop_rpi.sh (RPi) ---
_stop_rpi() {
  local cur; COMPREPLY=()
  cur="${COMP_WORDS[COMP_CWORD]}"
  COMPREPLY=( $(compgen -W "--keep-cam -h --help" -- "$cur") )
}
complete -F _stop_rpi stop_rpi.sh ./stop_rpi.sh

# --- start_laptop.sh (노트북: 손목+신원) ---
_start_laptop() {
  local cur prev opts
  COMPREPLY=()
  cur="${COMP_WORDS[COMP_CWORD]}"; prev="${COMP_WORDS[COMP_CWORD-1]}"
  opts="-H --host -P --port --show-view --no-face --face-only -h --help"
  case "$prev" in
    -H|--host|-P|--port) return ;;
  esac
  COMPREPLY=( $(compgen -W "$opts" -- "$cur") )
}
complete -F _start_laptop start_laptop.sh ./start_laptop.sh

# --- stop_laptop.sh (노트북) ---
_stop_laptop() {
  local cur; COMPREPLY=()
  cur="${COMP_WORDS[COMP_CWORD]}"
  COMPREPLY=( $(compgen -W "-h --help" -- "$cur") )
}
complete -F _stop_laptop stop_laptop.sh ./stop_laptop.sh

# --- selftest_arm.py (로봇 셀프테스트, RPi) — argcomplete ---
if command -v register-python-argcomplete >/dev/null 2>&1; then
  eval "$(register-python-argcomplete selftest_arm.py)" 2>/dev/null || true
fi

# --- run_overhead.sh (오버헤드 ArUco 캘리브레이션, 노트북) ---
_run_overhead() {
  local cur prev opts
  COMPREPLY=()
  cur="${COMP_WORDS[COMP_CWORD]}"; prev="${COMP_WORDS[COMP_CWORD-1]}"
  opts="-c --camera --width --height --config --rotate180 --no-rotate180 -h --help"
  case "$prev" in
    -c|--camera) COMPREPLY=( $(compgen -W "$(ls /dev/video* 2>/dev/null | sed 's#/dev/video##')" -- "$cur") ); return ;;
    --width)  COMPREPLY=( $(compgen -W "640 1280 1920" -- "$cur") ); return ;;
    --height) COMPREPLY=( $(compgen -W "480 720 1080" -- "$cur") ); return ;;
    --config) COMPREPLY=( $(compgen -f -- "$cur") ); return ;;
  esac
  COMPREPLY=( $(compgen -W "$opts" -- "$cur") )
}
complete -F _run_overhead run_overhead.sh ./run_overhead.sh

# --- run_overhead_pose.sh (오버헤드 로봇 pose 발행, 노트북) ---
_run_overhead_pose() {
  local cur prev opts
  COMPREPLY=()
  cur="${COMP_WORDS[COMP_CWORD]}"; prev="${COMP_WORDS[COMP_CWORD-1]}"
  opts="-c --camera --width --height --config --topic --rate --rotate180 --no-rotate180 -h --help"
  case "$prev" in
    -c|--camera) COMPREPLY=( $(compgen -W "$(ls /dev/video* 2>/dev/null | sed 's#/dev/video##')" -- "$cur") ); return ;;
    --width)  COMPREPLY=( $(compgen -W "640 1280 1920" -- "$cur") ); return ;;
    --height) COMPREPLY=( $(compgen -W "480 720 1080" -- "$cur") ); return ;;
    --rate)   COMPREPLY=( $(compgen -W "5 10 15 30" -- "$cur") ); return ;;
    --config) COMPREPLY=( $(compgen -f -- "$cur") ); return ;;
    --topic)  return ;;
  esac
  COMPREPLY=( $(compgen -W "$opts" -- "$cur") )
}
complete -F _run_overhead_pose run_overhead_pose.sh ./run_overhead_pose.sh

# --- run_gui_map.sh (GUI 맵 뷰, 노트북) ---
_run_gui_map() {
  local cur prev opts
  COMPREPLY=()
  cur="${COMP_WORDS[COMP_CWORD]}"; prev="${COMP_WORDS[COMP_CWORD-1]}"
  opts="--mock --ros --topic -h --help"
  case "$prev" in
    --topic) return ;;
  esac
  COMPREPLY=( $(compgen -W "$opts" -- "$cur") )
}
complete -F _run_gui_map run_gui_map.sh ./run_gui_map.sh

# --- run_console.sh (통합 제어·모니터링 콘솔, 노트북) ---
_run_console() {
  local cur prev opts
  COMPREPLY=()
  cur="${COMP_WORDS[COMP_CWORD]}"; prev="${COMP_WORDS[COMP_CWORD-1]}"
  opts="--no-ros --domain --config --roles -h --help"
  case "$prev" in
    --config|--roles) COMPREPLY=( $(compgen -f -- "$cur") ); return ;;
    --domain) COMPREPLY=( $(compgen -W "50" -- "$cur") ); return ;;
  esac
  COMPREPLY=( $(compgen -W "$opts" -- "$cur") )
}
complete -F _run_console run_console.sh ./run_console.sh

# --- run_patrol.sh (순찰 실행: patrol_node 로봇 도메인, 노트북/PC) ---
_run_patrol() {
  local cur prev opts
  COMPREPLY=()
  cur="${COMP_WORDS[COMP_CWORD]}"; prev="${COMP_WORDS[COMP_CWORD-1]}"
  opts="--robot --domain --peer --console-domain -h --help"
  case "$prev" in
    --robot)          COMPREPLY=( $(compgen -W "50 87 44 31" -- "$cur") ); return ;;
    --domain)         COMPREPLY=( $(compgen -W "51 52 53 54" -- "$cur") ); return ;;
    --console-domain) COMPREPLY=( $(compgen -W "50" -- "$cur") ); return ;;
    --peer)           COMPREPLY=( $(compgen -W "192.168.2.9 192.168.2.13 192.168.2.11 192.168.2.15" -- "$cur") ); return ;;
  esac
  COMPREPLY=( $(compgen -W "$opts" -- "$cur") )
}
complete -F _run_patrol run_patrol.sh ./run_patrol.sh
complete -F _run_patrol scripts/run_patrol.sh ./scripts/run_patrol.sh

# --- stop_console.sh (통합 제어·모니터링 콘솔 종료) ---
_stop_console() {
  local cur; COMPREPLY=()
  cur="${COMP_WORDS[COMP_CWORD]}"
  COMPREPLY=( $(compgen -W "-h --help" -- "$cur") )
}
complete -F _stop_console stop_console.sh ./stop_console.sh

# --- run_webapp.sh (사용자 웹앱 FastAPI 서버, 노트북/PC) ---
_run_webapp() {
  local cur prev opts
  COMPREPLY=()
  cur="${COMP_WORDS[COMP_CWORD]}"; prev="${COMP_WORDS[COMP_CWORD-1]}"
  opts="--no-ros --domain --host --port -h --help"
  case "$prev" in
    --domain) COMPREPLY=( $(compgen -W "50" -- "$cur") ); return ;;
    --host)   COMPREPLY=( $(compgen -W "0.0.0.0 127.0.0.1" -- "$cur") ); return ;;
    --port)   COMPREPLY=( $(compgen -W "8100" -- "$cur") ); return ;;
  esac
  COMPREPLY=( $(compgen -W "$opts" -- "$cur") )
}
complete -F _run_webapp run_webapp.sh ./run_webapp.sh
complete -F _run_webapp scripts/run_webapp.sh ./scripts/run_webapp.sh

# --- stop_webapp.sh (사용자 웹앱 서버 종료) ---
_stop_webapp() {
  local cur; COMPREPLY=()
  cur="${COMP_WORDS[COMP_CWORD]}"
  COMPREPLY=( $(compgen -W "-h --help" -- "$cur") )
}
complete -F _stop_webapp stop_webapp.sh ./stop_webapp.sh
complete -F _stop_webapp scripts/stop_webapp.sh ./scripts/stop_webapp.sh

# --- allkill.sh (ROS2 프로세스 일괄 종료) ---
_allkill() {
  local cur; COMPREPLY=()
  cur="${COMP_WORDS[COMP_CWORD]}"
  COMPREPLY=( $(compgen -W "-n --dry-run -y --yes -h --help" -- "$cur") )
}
complete -F _allkill allkill.sh ./allkill.sh

# --- run_sensorpose_calib.sh (오도메트리 휠 캘리브/검증, 노트북) ---
_run_sensorpose_calib() {
  local cur
  cur="${COMP_WORDS[COMP_CWORD]}"
  COMPREPLY=()
  if [ "$COMP_CWORD" -eq 1 ]; then
    COMPREPLY=( $(compgen -W "calibrate verify -h --help" -- "$cur") )
  fi
}
complete -F _run_sensorpose_calib run_sensorpose_calib.sh ./run_sensorpose_calib.sh
complete -F _run_sensorpose_calib scripts/run_sensorpose_calib.sh ./scripts/run_sensorpose_calib.sh

# --- start_agent.sh (로봇 heartbeat agent 기동, RPi) ---
_start_agent() {
  local cur prev opts
  COMPREPLY=()
  cur="${COMP_WORDS[COMP_CWORD]}"; prev="${COMP_WORDS[COMP_CWORD-1]}"
  opts="--robot-domain --console-domain --home --foreground -f -h --help"
  case "$prev" in
    --robot-domain)   COMPREPLY=( $(compgen -W "51 52 53 54" -- "$cur") ); return ;;
    --console-domain) COMPREPLY=( $(compgen -W "50" -- "$cur") ); return ;;
    --home)           COMPREPLY=( $(compgen -d -- "$cur") ); return ;;
  esac
  COMPREPLY=( $(compgen -W "$opts" -- "$cur") )
}
complete -F _start_agent start_agent.sh ./start_agent.sh

# --- stop_agent.sh (로봇 heartbeat agent 종료, RPi) ---
_stop_agent() {
  local cur; COMPREPLY=()
  cur="${COMP_WORDS[COMP_CWORD]}"
  COMPREPLY=( $(compgen -W "-h --help" -- "$cur") )
}
complete -F _stop_agent stop_agent.sh ./stop_agent.sh

# --- start_camera_stream.sh (로봇 전방 카메라 MJPEG 서빙, RPi) ---
_start_camera_stream() {
  local cur prev
  COMPREPLY=()
  cur="${COMP_WORDS[COMP_CWORD]}"; prev="${COMP_WORDS[COMP_CWORD-1]}"
  case "$prev" in
    --port|--domain|--topic) return 0 ;;   # 자유 입력
  esac
  COMPREPLY=( $(compgen -W "--port --topic --domain --foreground -h --help" -- "$cur") )
}
complete -F _start_camera_stream start_camera_stream.sh ./start_camera_stream.sh

# --- start_picamera_stream.sh (Pi camera picamera2 MJPEG 서빙, ROS 불필요, RPi) ---
_start_picamera_stream() {
  local cur prev
  COMPREPLY=()
  cur="${COMP_WORDS[COMP_CWORD]}"; prev="${COMP_WORDS[COMP_CWORD-1]}"
  case "$prev" in
    --port|--width|--height|--quality) return 0 ;;   # 자유 입력
  esac
  COMPREPLY=( $(compgen -W "--port --width --height --quality --no-flip --flip-horizontal --no-swap-rb --foreground -h --help" -- "$cur") )
}
complete -F _start_picamera_stream start_picamera_stream.sh ./start_picamera_stream.sh

# --- preflight_robot.sh (bringup 전 사전점검, 로봇) ---
_preflight_robot() {
  local cur prev
  COMPREPLY=()
  cur="${COMP_WORDS[COMP_CWORD]}"; prev="${COMP_WORDS[COMP_CWORD-1]}"
  case "$prev" in
    --robot) COMPREPLY=( $(compgen -W "31 44 50 87" -- "$cur") ); return 0 ;;
    --map) COMPREPLY=( $(compgen -W "wasab_map5 wasab_map9 wasab_map10 wasab_map11" -- "$cur") ); return 0 ;;
    --params) COMPREPLY=( $(compgen -f -- "$cur") ); return 0 ;;
  esac
  COMPREPLY=( $(compgen -W "--robot --map --params --allow-uncalibrated -h --help" -- "$cur") )
}
complete -F _preflight_robot preflight_robot.sh ./preflight_robot.sh

# --- verify_lidar_scale.py (라이다 편향 검증, 로봇) ---
_verify_lidar_scale() {
  local cur prev
  COMPREPLY=()
  cur="${COMP_WORDS[COMP_CWORD]}"; prev="${COMP_WORDS[COMP_CWORD-1]}"
  case "$prev" in
    --axis1) COMPREPLY=( $(compgen -W "2.00 1.95" -- "$cur") ); return 0 ;;
    --axis2) COMPREPLY=( $(compgen -W "1.00 0.95" -- "$cur") ); return 0 ;;
    --tol) COMPREPLY=( $(compgen -W "0.01 0.02 0.03" -- "$cur") ); return 0 ;;
    --window) COMPREPLY=( $(compgen -W "1.0 2.0 3.0" -- "$cur") ); return 0 ;;
  esac
  COMPREPLY=( $(compgen -W "--axis1 --axis2 --tol --window -h --help" -- "$cur") )
}
complete -F _verify_lidar_scale verify_lidar_scale.py ./verify_lidar_scale.py
