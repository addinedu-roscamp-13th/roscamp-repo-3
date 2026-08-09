#!/usr/bin/env bash
set -euo pipefail

arm_id="${1:-}"
case "$arm_id" in
  left)
    robot_host="192.168.2.10"
    ;;
  right)
    robot_host="192.168.2.12"
    ;;
  *)
    echo "usage: $0 left|right" >&2
    exit 2
    ;;
esac

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
profile="${script_dir}/config/client_config.ini"
remote_root="/home/jetcobot/wasab/roscamp-repo-3/Device/WasabBot/WasabArmController"

test -f "$profile"
scp "$profile" "jetcobot@${robot_host}:${remote_root}/config/client_config.ini"
ssh "jetcobot@${robot_host}" \
  "printf '%s\n' '${arm_id}' > '${remote_root}/config/arm_identity'"
echo "deployed unified config + identity=${arm_id} -> ${robot_host}"
