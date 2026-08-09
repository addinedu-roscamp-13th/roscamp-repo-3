import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("wasab_docking")
    det_cfg = os.path.join(share, "config", "precision_parking.yaml")
    reloc_cfg = os.path.join(share, "config", "relocalizer.yaml")
    # tag_map_poses_path는 노드 기본값(~/.wasab/tag_map_poses.yaml, writable)을 그대로 사용 —
    # install/share(read-only, 재빌드 덮임)로 override하지 않는다.
    return LaunchDescription([
        Node(package="wasab_docking", executable="apriltag_detector",
             name="apriltag_detector", parameters=[det_cfg]),
        Node(package="wasab_docking", executable="apriltag_relocalizer",
             name="apriltag_relocalizer", parameters=[reloc_cfg]),
    ])
