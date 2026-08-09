# 정밀 주차 launch (Phase 1): precision_parking controller.
# detector는 precision_parking 노드가 SEARCH_TAG에서 자체 spawn(2026-07-13 지연기동).
# tag_id는 detector가 쓰던 값(precision_parking.yaml apriltag_detector.tag_id=8)과 맞춘다.
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory("wasab_docking"), "config", "precision_parking.yaml")
    return LaunchDescription([
        Node(package="wasab_docking", executable="precision_parking",
             name="precision_parking", parameters=[cfg, {"tag_id": 8}], output="screen"),
    ])
