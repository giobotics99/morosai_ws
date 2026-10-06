#!/usr/bin/env python3
"""
MOROSAI AGV Full Bringup Launch File
Ottimizzato per architettura custom su NVIDIA Jetson Orin AGX.

Questo script lancia i nodi NAV2 singolarmente, evitando di caricare
processi inutili (docking_server, route_server, smoother_server) che 
nav2_bringup caricherebbe di default in ROS 2 Jazzy.
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

    # ============================================================
    # Launch Arguments
    # ============================================================
    declare_map_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(pkg_morosai_navigation, 'maps', 'map_fema_28_09_26.yaml'),
        description='Full path to map yaml file'
    )

    declare_nav_params_arg = DeclareLaunchArgument(
        'nav_params_file',
        default_value=os.path.join(pkg_morosai_navigation, 'config','DWB_jazzy.yaml'),
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
        default_value='true',
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

    # Configurazione comune per i nodi NAV2
    nav2_node_config = {
        'namespace': '',
        'output': 'screen',
        'emulate_tty': True,  # Molto utile su Jetson per mantenere l'ordine dei log nei TTY
        'parameters': [nav_params_file],
        'respawn': True,
        'respawn_delay': 2.0
    }
    
    # Nodi Lifecycle principali per Nav2 (ordine logico)
    lifecycle_nodes = [
        'map_server',
        'amcl',
        'controller_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
        'velocity_smoother'
    ]

    # ============================================================
    # 1. SENSORS & FILTERING
    # ============================================================
    sensors_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_morosai_sensors, 'launch', 'sensors_up.launch.py')
        )
    )

    laser_filter_node = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        name='scan_to_scan_filter_chain',
        namespace='',
        output='screen',
        parameters=[os.path.join(pkg_morosai_navigation, 'config', 'laser_filters.yaml')],
        remappings=[
            ('scan', '/merged'),
            ('scan_filtered', '/merged_filtered'),
        ]
    )

    # ============================================================
    # 2. NAV2 EXPLICIT LIFECYCLE NODES
    # ============================================================
    map_server = LifecycleNode(
        package='nav2_map_server', executable='map_server', name='map_server',
        namespace='', parameters=[nav_params_file, {'yaml_filename': map_file}], output='screen'
    )
    amcl = LifecycleNode(package='nav2_amcl', executable='amcl', name='amcl', **nav2_node_config)
    controller_server = LifecycleNode(package='nav2_controller', executable='controller_server', name='controller_server', **nav2_node_config)
    planner_server = LifecycleNode(package='nav2_planner', executable='planner_server', name='planner_server', **nav2_node_config)
    behavior_server = LifecycleNode(package='nav2_behaviors', executable='behavior_server', name='behavior_server', **nav2_node_config)
    bt_navigator = LifecycleNode(package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator', **nav2_node_config)
    waypoint_follower = LifecycleNode(package='nav2_waypoint_follower', executable='waypoint_follower', name='waypoint_follower', **nav2_node_config)
    
    # Il velocity_smoother in standard Jazzy mappa l'output su cmd_vel_smoothed. 
    # Assicurati che il tuo collision_monitor (che ascolta /cmd_vel_raw) legga dal topic giusto
    velocity_smoother = LifecycleNode(
        package='nav2_velocity_smoother', executable='velocity_smoother', name='velocity_smoother', 
        namespace='',
        remappings=[('cmd_vel_smoothed', '/cmd_vel_raw')],
        output='screen',
        emulate_tty=True,
        parameters=[nav_params_file]
    )

    # Lifecycle Manager per lo stack di navigazione (niente docking_server o route_server!)
    nav2_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        namespace='',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': lifecycle_nodes
        }]
    )

    # ============================================================
    # 3. SAFETY & COLLISION MONITOR (Gestito dal proprio lifecycle)
    # ============================================================
    collision_monitor_node = LifecycleNode(
        package='nav2_collision_monitor',
        executable='collision_monitor',
        name='collision_monitor',
        **nav2_node_config
    )

    lifecycle_manager_safety = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_safety',
        namespace='',
        output='screen',
        parameters=[{
            'autostart': autostart,
            'node_names': ['collision_monitor'],
            'bond_timeout': 10.0
        }]
    )

    # ============================================================
    # 4. EXTRAS & UTILITIES
    # ============================================================
    nav_topic_publisher_node = Node(
        package='morosai_navigation',
        executable='mqtt_bridge_publisher.py',
        name='mqtt_bridge_publisher',
        namespace='',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    apriltag_node = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag_node',
        namespace='',
        remappings=[
            ('image_rect', '/gordon_tof/bgr'),
            ('camera_info', '/gordon_tof/camera_info'),
        ],
        parameters=[{
            'family': 'Standard52h13',
            'size': 0.088,
            'detector.threads': 8, # Jetson A78AE ha 12 core, dedichiamone 8 ad apriltag!
            'detector.decimate': 1.0, 
            'qos_profile': "sensor_data"
        }],
        output='screen'
    )

    map_updater_node = Node(
        package='morosai_navigation',
        executable='map_updater_node.py',
        name='map_updater_node',
        namespace='',
        output='screen'
    )

    ekf_config_file = os.path.join(pkg_morosai_navigation, 'config', 'ekf.yaml')
    ekf_filter_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        namespace='',
        output='screen',
        parameters=[ekf_config_file, {'use_sim_time': use_sim_time}],
        remappings=[('/odometry/filtered', '/odometry/filtered')]
    )

    rviz_config_file = os.path.join(pkg_morosai_description, 'rviz', 'display_NAV2.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        namespace='',
        arguments=['-d', rviz_config_file],
        output='screen',
        respawn=True,
        condition=IfCondition(use_rviz)
    )

    # ============================================================
    # Build Launch Description
    # ============================================================
    return LaunchDescription([
        declare_map_arg,
        declare_nav_params_arg,
        declare_use_sim_time_arg,
        declare_use_rviz_arg,
        declare_autostart_arg,

        sensors_launch,
        laser_filter_node,
        # ekf_filter_node,

        # NAV2 EXPLICIT
        map_server,
        amcl,
        controller_server,
        planner_server,
        behavior_server,
        bt_navigator,
        waypoint_follower,
        velocity_smoother,
        nav2_lifecycle_manager,

        # SAFETY
        collision_monitor_node,
        lifecycle_manager_safety,

        # EXTRAS
        nav_topic_publisher_node,
        apriltag_node,
        map_updater_node,
        rviz_node
    ])
