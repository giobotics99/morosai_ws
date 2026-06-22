from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetRemap
import os

def generate_launch_description():
    # === Nodo throttle per /odom ===
    odom_throttle_node = Node(
        package='topic_tools',
        executable='throttle',
        name='odom_throttle',
        arguments=['messages', '/odom', '10.0', '/odom_throttle'],  
        output='screen'
    )

    # === Include Nav2 bringup ===
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            '/opt/ros/humble/share/nav2_bringup/launch/bringup_launch.py'
        ]),
        launch_arguments={
            'map': '/home/orion/morosai_mini_ws/src/morosai_navigation/maps/corridor.yaml',
            'use_sim_time': 'false',
            'autostart': 'False',
            'use_composition': 'False',
            'params_file': '/home/orion/morosai_mini_ws/src/morosai_navigation/config/nav2_bringup_real3.yaml'
        }.items(),
    )

    # === Remap globale /odom → /odom_throttle ===
    remap_group = GroupAction([
        SetRemap(src='/odom', dst='/odom_throttle'),
        nav2_bringup
    ])

    return LaunchDescription([
        nav2_bringup,
        # odom_throttle_node,
        # remap_group
    ])

# After you run this launch file, to test if the throttling is working, you can publish to /odom and check /odom_throttle:
# ros2 topic hz /odom_throttle

# ros2 node info /controller_server | grep odom
# ros2 node info /bt_navigator | grep odom
# ros2 node info /amcl | grep odom
# ros2 node info /planner_server | grep odom



# ros2 topic pub /odom nav_msgs/msg/Odometry "{header: {frame_id: 'odom'}, child_frame_id: 'base_footprint', pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}, twist: {twist: {linear: {x: 0.0}, angular: {z: 0.0}}}}" -r 50
# ros2 run topic_tools throttle messages /odom 10




# Se vedi ancora i topic di odom, uccidi il nodo topic_tools:
# ps aux | grep topic_tools
# kill -9 12345 # sostituisci 12345 con il PID effettivo 



 ## PER KILLARE SE VEDI ANCORA I NODI ATTIVI DOPO AVER CHIUSO IL LAUNCH
#  pkill -f "nav2_"
# pkill -f odom_throttle
#  pkill -f ros2 (ULTIMA SPIAGGIA!!!!)
#  ros2 daemon stop
#  ros2 daemon start

# rviz2 -d /home/orion/morosai_mini_ws/src/morosai_description/rviz/display_NAV2.rviz

