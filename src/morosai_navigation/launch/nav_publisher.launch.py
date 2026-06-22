"""
Launch file for the Navigation Topic Publisher node.

This launch file starts the nav_topic_publisher node which republishes
NAV2 topics to custom /agv_* topic names.

Usage:
  ros2 launch morosai_navigation nav_publisher.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Navigation Topic Publisher Node
    nav_topic_publisher_node = Node(
        package='morosai_navigation',
        executable='nav_topic_publisher.py',
        name='nav_topic_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': False
        }]
    )

    return LaunchDescription([
        nav_topic_publisher_node
    ])
