#!/usr/bin/env python3
"""
MOROSAI AGV SLAM Localization Bringup Launch File

Launches all systems for the MOROSAI AGV with SLAM Toolbox acting as a LOCALIZER on a serialized map:
    - Sensors (TOF, LIDARs, Optical Head) via sensors_up.launch.py
    - Robot State Publisher (URDF)
    - NAV2 SLAM Toolbox in localization mode with slam_toolbox_localization.yaml
    - NAV2 Navigation Stack with nav2_slam.yaml
    - Navigation Topic Publisher (for /agv_* topics)
    - RViz2 visualization

===================================================================================
⚠️ IMPORTANT: SLAM TOOLBOX LOCALIZATION USES SERIALIZED MAPS (NOT .yaml and .pgm)
===================================================================================

If you want to run this launch file, you MUST have saved the map as a serialized map 
(.posegraph file). SLAM Toolbox Localization does NOT use the standard ROS2 map.yaml 
and .pgm image maps like AMCL does.

HOW TO SERIALIZE A MAP:
1. First, run the robot in mapping mode (e.g., using slam_bringup.launch.py which is 
   configured for mapper_params_online_async.yaml).
2. Drive the robot around until you mapped the entire environment.
3. Call the SLAM Toolbox serialize map service via terminal:
   ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '/home/orion/morosai_mini_ws/src/morosai_navigation/maps/my_map_name'}"
   (This creates your_map_name.posegraph and your_map_name.data in the maps folder)
4. Update src/morosai_navigation/config/slam_toolbox_localization.yaml map_file_name
   to point to the absolute path of your map (omitting the .posegraph extension!).
5. Now you can run THIS launch file!
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
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
    # Using nav2_slam.yaml as standard, but you can swap to nav2_bringup_final.yaml if needed
    declare_nav_params_arg = DeclareLaunchArgument(
        'nav_params_file',
        default_value=os.path.join(pkg_morosai_navigation, 'config', 'nav2_slam.yaml'),
        description='Full path to NAV2 navigation params file'
    )

    declare_slam_params_arg = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(pkg_morosai_navigation, 'config', 'slam_toolbox_localization.yaml'),
        description='Full path to SLAM toolbox localization params file'
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
    nav_params_file = LaunchConfiguration('nav_params_file')
    slam_params_file = LaunchConfiguration('slam_params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    autostart = LaunchConfiguration('autostart')

    # ============================================================
    # 1. SENSORS - sensors_up.launch.py
    # ============================================================
    sensors_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_morosai_sensors, 'launch', 'sensors_up.launch.py')
        )
    )

    # ============================================================
    # 2. SLAM - slam_launch.py
    # Launches slam_toolbox in localization mode
    # ============================================================
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'slam_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'params_file': slam_params_file,
        }.items()
    )

    # ============================================================
    # 3. NAVIGATION - navigation_launch.py
    # Launches controller, planner, behavior server, etc.
    # ============================================================
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'params_file': nav_params_file,
            'use_composition': 'False',
        }.items()
    )

    # ============================================================
    # 4. Navigation Topic Publisher
    # ============================================================
    nav_topic_publisher_node = Node(
        package='morosai_navigation',
        executable='nav_topic_publisher.py',
        name='nav_topic_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }]
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
        condition=IfCondition(use_rviz)
    )

    # ============================================================
    # Build Launch Description
    # ============================================================
    return LaunchDescription([
        # Declare arguments
        declare_nav_params_arg,
        declare_slam_params_arg,
        declare_use_sim_time_arg,
        declare_use_rviz_arg,
        declare_autostart_arg,

        # Launch components
        sensors_launch,
        slam_launch,
        navigation_launch,
        nav_topic_publisher_node,
        rviz_node
    ])
