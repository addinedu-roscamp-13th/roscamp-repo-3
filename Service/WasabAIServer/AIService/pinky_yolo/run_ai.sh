#!/bin/bash
cd ~/dev_ws/wasab/src/roscamp-repo-3/Service/WasabAIServer/AIService/pinky_yolo
colcon build --packages-select pinky_yolo
source install/setup.bash
ROS_DOMAIN_ID=53 ros2 run pinky_yolo ai_node
