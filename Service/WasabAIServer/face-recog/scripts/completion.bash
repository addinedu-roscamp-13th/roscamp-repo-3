# face-recog 런처 탭 자동완성 (start.sh / register.sh / stop.sh).
# 부트스트랩: ~/.bashrc 에 아래 한 줄 추가 후 새 셸:
#   [ -f "$HOME/face-recog/scripts/completion.bash" ] && source "$HOME/face-recog/scripts/completion.bash"

_start() {
  local cur prev; cur="${COMP_WORDS[COMP_CWORD]}"; prev="${COMP_WORDS[COMP_CWORD-1]}"
  case "$prev" in
    -m|--model)     COMPREPLY=($(compgen -W "buffalo_sc buffalo_l" -- "$cur")); return;;
    -t|--tolerance) COMPREPLY=($(compgen -W "0.30 0.35 0.40 0.45 0.50" -- "$cur")); return;;
    -r|--res)       COMPREPLY=($(compgen -W "320x240 640x480" -- "$cur")); return;;
    -H|--host|-P|--port) return;;
  esac
  COMPREPLY=($(compgen -W "-H --host -P --port -r --res --no-display --reload -t --tolerance -m --model --cam-only --local -h --help" -- "$cur"))
}
complete -F _start start.sh ./start.sh scripts/start.sh ./scripts/start.sh

_register() {
  local cur prev; cur="${COMP_WORDS[COMP_CWORD]}"; prev="${COMP_WORDS[COMP_CWORD-1]}"
  case "$prev" in
    -r|--res) COMPREPLY=($(compgen -W "320x240 640x480" -- "$cur")); return;;
    --scale) COMPREPLY=($(compgen -W "1.0 1.5 2.0" -- "$cur")); return;;
    -H|--host|-P|--port) return;;
  esac
  COMPREPLY=($(compgen -W "-H --host -P --port -r --res --scale --local -h --help" -- "$cur"))
}
complete -F _register register.sh ./register.sh scripts/register.sh ./scripts/register.sh

_stop() {
  local cur prev; cur="${COMP_WORDS[COMP_CWORD]}"; prev="${COMP_WORDS[COMP_CWORD-1]}"
  case "$prev" in -H|--host) return;; esac
  COMPREPLY=($(compgen -W "-H --host --keep-cam -h --help" -- "$cur"))
}
complete -F _stop stop.sh ./stop.sh scripts/stop.sh ./scripts/stop.sh
