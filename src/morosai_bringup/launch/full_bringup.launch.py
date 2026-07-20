#!/usr/bin/env python3
"""
MOROSAI AGV Full Bringup Launch File

Launches all systems for the MOROSAI AGV:
    - Sensors (TOF, LIDARs, Optical Head) via sensors_up.launch.py
    - Robot State Publisher (URDF)
    - NAV2 Navigation Stack
    - Navigation Topic Publisher (for /agv_* topics)
    - RViz2 visualization

Usage:
    ros2 launch morosai_bringup full_bringup.launch.py
    ros2 launch morosai_bringup full_bringup.launch.py map:=/path/to/map.yaml
    ros2 launch morosai_bringup full_bringup.launch.py use_rviz:=false
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node, LifecycleNode
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # ============================================================
    # Package directories
    # ============================================================
    pkg_morosai_sensors = get_package_share_directory('morosai_sensors')
    pkg_morosai_navigation = get_package_share_directory('morosai_navigation')
    pkg_morosai_description = get_package_share_directory('morosai_description')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    # ============================================================
    # Launch Arguments
    # ============================================================
    declare_map_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(pkg_morosai_navigation, 'maps', 'fema_map2.yaml'),
        description='Full path to map yaml file'
    )

    # declare_nav_params_arg = DeclareLaunchArgument(
    #     'nav_params_file',
    #     default_value=os.path.join(pkg_morosai_navigation, 'config', 'nav2_bringup_final.yaml'),
    #     description='Full path to NAV2 params file'
    # )

    declare_nav_params_arg = DeclareLaunchArgument(
        'nav_params_file',
        default_value=os.path.join(pkg_morosai_navigation, 'config','DWB_FINAL.yaml'),
        description='Full path to NAV2 params file'
    )

    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    declare_use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz2'
    )

    declare_autostart_arg = DeclareLaunchArgument(
        'autostart',
        default_value='True',
        description='Automatically start NAV2 lifecycle nodes'
    )

    # ============================================================
    # Launch Configuration
    # ============================================================
    map_file = LaunchConfiguration('map')
    nav_params_file = LaunchConfiguration('nav_params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    autostart = LaunchConfiguration('autostart')

    # ============================================================
    # 1. SENSORS - sensors_up.launch.py
    # Includes: TOF, LIDARs (front+rear), Optical Head, Robot State Publisher
    # ============================================================
    sensors_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_morosai_sensors, 'launch', 'sensors_up.launch.py')
        )
    )

    # ============================================================
    # 2. NAV2 Navigation Stack
    # ============================================================
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_file,
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'params_file': nav_params_file,
            'use_composition': 'False'
        }.items()
    )

    # ============================================================
    # 3. Navigation Topic Publisher
    # Publishes: /agv_pose, /agv_path, /agv_v, /agv_qr, /agv_detect, /wheel_v, /agv_act, /agv_op
    # ============================================================
    nav_topic_publisher_node = Node(
        package='morosai_navigation',
        executable='mqtt_bridge_publisher.py',
        name='mqtt_bridge_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )

    # ============================================================
    # 4. Nav2 Collision Monitor (Safety Filter)
    # ============================================================
    collision_monitor_node = LifecycleNode(
        package='nav2_collision_monitor',
        executable='collision_monitor',
        name='collision_monitor',
        namespace='',
        output='screen',
        emulate_tty=True,
        parameters=[nav_params_file]
    )

    # ============================================================
    # 4b. AprilTag Node
    # ============================================================
    apriltag_node = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag_node',
        remappings=[
            ('image_rect', '/gordon_tof/bgr'),
            ('camera_info', '/gordon_tof/camera_info'),
        ],
        parameters=[{
            'family': 'Standard52h13',
            'size': 0.088,
            'detector.threads': 4,
            'detector.decimate': 2.0
        }],
        output='screen'
    )

    # ============================================================
    # 4c. Lifecycle Manager Dedicato per la Sicurezza
    # ============================================================
    lifecycle_manager_safety = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_safety',
        output='screen',
        parameters=[{
            'autostart': True,
            'node_names': ['collision_monitor'],
            'bond_timeout': 10.0
        }]
    )

    # ============================================================
    # 4d. Map Updater Node
    # ============================================================
    map_updater_node = Node(
        package='morosai_navigation',
        executable='map_updater_node.py',
        name='map_updater_node',
        output='screen'
    )

    # ============================================================
    # 5. RViz2 Visualization
    # ============================================================
    rviz_config_file = os.path.join(pkg_morosai_description, 'rviz', 'display_NAV2.rviz')
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen',
        respawn=True,
        condition=IfCondition(use_rviz)
    )

    # ============================================================
    # Build Launch Description
    # ============================================================
    return LaunchDescription([
        # Declare arguments
        declare_map_arg,
        declare_nav_params_arg,
        declare_use_sim_time_arg,
        declare_use_rviz_arg,
        declare_autostart_arg,

        # Launch components
        sensors_launch,
        nav2_bringup_launch,
        nav_topic_publisher_node,
        collision_monitor_node,
        lifecycle_manager_safety,
        apriltag_node,
        map_updater_node
        # rviz_node
    ])
