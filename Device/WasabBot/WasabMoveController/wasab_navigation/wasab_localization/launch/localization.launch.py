import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('wasab_localization')
    ekf_yaml = os.path.join(pkg, 'config', 'ekf.yaml')
    relay_yaml = os.path.join(pkg, 'config', 'relay.yaml')

    sllidar_share = get_package_share_directory('sllidar_ros2')
    pinky_nav_share = get_package_share_directory('pinky_navigation')
    pinky_desc_share = get_package_share_directory('pinky_description')

    args = [
        DeclareLaunchArgument('map', description='맵 yaml 경로'),
        DeclareLaunchArgument('wheel_radius', default_value='0.027'),
        DeclareLaunchArgument('wheel_separation', default_value='0.0961'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'amcl_params_file',
            default_value=os.path.join(pinky_nav_share, 'params', 'nav2_params.yaml'),
            description='AMCL params (WaSaB 튜닝 시 wasab_nav2/params/nav2_params.yaml 경로로 override)'),
    ]

    upload_robot = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(pinky_desc_share, 'launch', 'upload_robot.launch.py')),
        launch_arguments={'is_sim': LaunchConfiguration('use_sim_time')}.items())

    imu = Node(
        package='pinky_imu_bno055', executable='main_node',
        name='pinky_imu_bno055', output='screen')

    # bringup: /tf remap으로 odom→base TF 억제 (upstream 무수정, 실행파일만 사용)
    bringup = Node(
        package='pinky_bringup', executable='bringup', name='pinky_bringup',
        parameters=[{
            'wheel_radius': LaunchConfiguration('wheel_radius'),
            'wheel_separation': LaunchConfiguration('wheel_separation'),
        }],
        remappings=[('/tf', '/tf_suppressed')],
        output='screen')

    sllidar = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(sllidar_share, 'launch', 'sllidar_c1_launch.py')),
        launch_arguments={
            'serial_port': '/dev/ttyS0', 'frame_id': 'rplidar_link',
            'inverted': 'false', 'angle_compensate': 'true',
            'scan_mode': 'DenseBoost',
        }.items())

    relay = Node(
        package='wasab_localization', executable='odom_cov_relay',
        name='odom_cov_relay', parameters=[relay_yaml], output='screen')

    ekf = Node(
        package='robot_localization', executable='ekf_node', name='ekf_filter_node',
        parameters=[ekf_yaml, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen')

    amcl = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(pinky_nav_share, 'launch', 'localization_launch.xml')),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'params_file': LaunchConfiguration('amcl_params_file'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items())

    return LaunchDescription(args + [upload_robot, imu, bringup, sllidar, relay, ekf, amcl])
