#!/bin/bash
cd ~/dev_ws/wasab
colcon build --packages-select pinky_yolo
source install/setup.bash
ROS_DOMAIN_ID=51 ros2 run pinky_yolo ai_node
