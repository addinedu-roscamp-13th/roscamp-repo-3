"""콘솔 태그 docking(mode A): precision_parking에 launch args로 파라미터 주입.
detector는 co-launch하지 않음 — precision_parking이 SEARCH_TAG 진입 시 tag_id로 직접 spawn(Task 1).
approach_pose_set은 내부에서 true 고정(mode A 전용)."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory("wasab_docking"), "config", "precision_parking.yaml")
    args = {
        "tag_id": "8", "approach_pose_x": "0.0", "approach_pose_y": "0.0",
        "approach_pose_yaw": "0.0", "nav_enabled": "true", "cmd_vel_enabled": "true",
        "tag_goal_x": "0.15",   # 정면 15cm 밀착(2026-07-11). 콘솔 도킹(agent 미오버라이드) 기본값.
    }
    decls = [DeclareLaunchArgument(k, default_value=v) for k, v in args.items()]
    lc = {k: LaunchConfiguration(k) for k in args}
    return LaunchDescription(decls + [
        Node(package="wasab_docking", executable="precision_parking",
             name="precision_parking", output="screen",
             parameters=[cfg, {
                 "tag_id": ParameterValue(lc["tag_id"], value_type=int),   # detector spawn용
                 "nav_enabled": ParameterValue(lc["nav_enabled"], value_type=bool),
                 "cmd_vel_enabled": ParameterValue(lc["cmd_vel_enabled"], value_type=bool),
                 "approach_pose_set": True,          # mode A 전용 → 항상 true
                 "approach_pose_x": ParameterValue(lc["approach_pose_x"], value_type=float),
                 "approach_pose_y": ParameterValue(lc["approach_pose_y"], value_type=float),
                 "approach_pose_yaw": ParameterValue(lc["approach_pose_yaw"], value_type=float),
                 "tag_goal_x": ParameterValue(lc["tag_goal_x"], value_type=float),
             }]),
    ])
