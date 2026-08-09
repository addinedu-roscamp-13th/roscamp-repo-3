import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    share = get_package_share_directory("wasab_patrol")
    cm_yaml = os.path.join(share, "config", "collision_monitor.yaml")
    wp_yaml = os.path.join(share, "config", "waypoints.yaml")

    # PATROL_ROBOT_ID 미설정/비정상이면 collision_monitor를 띄우기 전에 명확히 실패.
    # (0으로 조용히 돌면 patrol_node가 자기 heartbeat를 타 로봇으로 오인→영구 YIELD.)
    raw = os.environ.get("PATROL_ROBOT_ID", "")
    try:
        robot_id = int(raw)
    except ValueError:
        robot_id = 0
    if robot_id <= 0:
        raise RuntimeError(
            "PATROL_ROBOT_ID must be set to the patrol robot id (>0); got %r" % (raw,))

    return LaunchDescription([
        Node(package="nav2_collision_monitor", executable="collision_monitor",
             name="collision_monitor", parameters=[cm_yaml], output="screen"),
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
             name="lifecycle_manager_collision", output="screen",
             parameters=[{"autostart": True, "node_names": ["collision_monitor"]}]),
        Node(package="wasab_patrol", executable="patrol_node", name="wasab_patrol",
             parameters=[{"robot_id": robot_id, "console_domain": 50, "config": wp_yaml}],
             output="screen"),
    ])
