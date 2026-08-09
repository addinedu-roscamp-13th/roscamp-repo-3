#!/usr/bin/env bash
# run_fire_search.sh — 순찰(화재감지)용 fire_arm_search 실행. JetCobot에서 실행.
# 값 바꾸고 싶으면 아래 -p 옵션들만 수정하면 됨. fire_search_node.py는 search_node.py(추종용)와
# 별개 파일 — 여기서 뭘 바꿔도 얼굴 추종엔 전혀 영향 없음.
set -uo pipefail

set +u
source /opt/ros/jazzy/setup.bash
source ~/wasab/install/setup.bash
set -u

export ROS_DOMAIN_ID=69   # fire_detect_node.py(PC)와 도메인 통일 — 안 맞으면 토픽이 전혀 안 보임

# face_timeout: 2026-07-28 1.0→2.0. 실측 보정 간격이 ~1.68~1.69초(mycobot 명령처리 하드웨어
# 한계로 추정)라, 1.0초로는 다음 보정 오기 전에 매번 "놓침" 판정→SEARCH 재진입을 반복해
# 움직임이 끊겨 보였음.
exec ros2 run wasab_k3_mimic fire_arm_search --ros-args \
  -p face_topic:=/wasab/k3/fire \
  -p face_timeout:=2.0 \
  -p track_speed:=10 \
  -p search_speed:=10 \
  -p kx:=-8.0 \
  -p ky:=6.0 \
  -p deadzone:=0.05 \
  -p track_rate_hz:=10.0 \
  -p search_dwell_sec:=2.0 \
  -p search_yaw_step:=14.0 \
  -p search_pitch_step:=6.0
