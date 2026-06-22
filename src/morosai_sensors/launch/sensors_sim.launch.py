from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory
import os

def generate_launch_description():
    # Bridge configurato via YAML per mappare i topic GZ -> ROS con i nomi "reali"
    bridge_params = os.path.join(
        get_package_share_directory('morosai_sensors'), 'config', 'gz_bridge.yaml'
    )

    return LaunchDescription([
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='gz_bridge',
            output='screen',
            arguments=['--ros-args', '--params-file', bridge_params]
        ),
        # Se in sim generi l'odom lato ROS (shim), aggiungi qui il nodo odom_sim.
        # Altrimenti, se il plugin GZ già pubblica odom, basta il bridge (vedi YAML).
    ])
