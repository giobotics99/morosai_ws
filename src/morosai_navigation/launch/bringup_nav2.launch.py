from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetRemap
import os

def generate_launch_description():
    # === Include Nav2 bringup ===
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            '/opt/ros/humble/share/nav2_bringup/launch/bringup_launch.py'
        ]),
        launch_arguments={
            'map': '/home/orion/morosai_mini_ws/src/morosai_navigation/maps/map_ingel_final.yaml',
            'use_sim_time': 'false',
            'autostart': 'False',
            'use_composition': 'False',
            'params_file': '/home/orion/morosai_mini_ws/src/morosai_navigation/config/nav2_bringup_final.yaml'
        }.items(),
    )

    return LaunchDescription([
        nav2_bringup
    ])

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

