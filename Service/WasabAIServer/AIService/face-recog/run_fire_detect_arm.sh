#!/usr/bin/env bash
# run_fire_detect_arm.sh — (JetCobot) fire_detect_arm.py 실행. 카메라를 로컬로 직접 읽어 감지까지 끝내고
# 판정 결과만 ROS로 발행한다. 화면은 --target으로 PC에 UDP 중계(기존 cam_server.py와 동일 프로토콜).
#
# 주의: cam_server.py와 같은 --camera 인덱스를 동시에 실행하지 말 것 — 카메라 장치는 한 프로세스만
# 열 수 있어서 충돌한다. 화재감지 중엔 이 스크립트가 cam_server.py를 대신한다.
#
# run_fire_search.sh(추종/서보)와 도메인 통일 필요 — /wasab/k3/fire 를 못 보면 이 값부터 확인.
set -uo pipefail

set +u
source /opt/ros/jazzy/setup.bash
set -u

export ROS_DOMAIN_ID=69   # run_fire_search.sh와 동일해야 /wasab/k3/fire 가 보임
# 2026-07-28: WiFi 멀티캐스트 불안정으로 콘솔 도메인(fire_response 등)이 가끔 유실되던 문제 대응 —
# 웹앱/콘솔/순찰 스크립트와 동일하게 유니캐스트 피어 강제 지정.
export ROS_STATIC_PEERS="${ROS_STATIC_PEERS:-192.168.2.15;192.168.2.9;192.168.2.13;192.168.2.11;192.168.2.6}"

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT"

TARGET="${1:-}"
if [ -z "$TARGET" ]; then
  echo "[run_fire_detect_arm] 사용법: ./run_fire_detect_arm.sh <PC IP> [추가 옵션...]"
  echo "[run_fire_detect_arm] PC IP 없이 실행하면 화면 중계 없이 감지만 수행합니다."
  exec python3 fire_detect_arm.py --console-domain 50 --flip "${@:2}"
else
  exec python3 fire_detect_arm.py --target "$TARGET" --console-domain 50 --flip "${@:2}"
fi
