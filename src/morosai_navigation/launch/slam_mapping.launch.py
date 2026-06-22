from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import ThisLaunchFileDir, LaunchConfiguration
import os

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Percorsi ai pacchetti
    morosai_sensors_dir = get_package_share_directory('morosai_sensors')
    morosai_navigation_dir = get_package_share_directory('morosai_navigation')
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')

    # Include sensors.launch.py (LiDAR + ToF)
    sensors_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(morosai_sensors_dir, 'launch', 'sensors_up.launch.py')
        )
    )

    # Include slam_toolbox online_async_launch.py
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_dir, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={'slam_params_file': os.path.join(morosai_navigation_dir, 'config', 'mapper_params_online_async.yaml'),
                            'use_sim_time': 'false'}.items()
    )

    # # SLAM Toolbox con timer di 2 secondi
    # slam_launch = TimerAction(
    #     period=2.0,  # aspetta 2 secondi
    #     actions=[
    #         IncludeLaunchDescription(
    #             PythonLaunchDescriptionSource(
    #                 os.path.join(slam_toolbox_dir, 'launch', 'online_async_launch.py')
    #             ),
    #             launch_arguments={'scan_topic': '/merged', 'use_sim_time': 'false'}.items()
    #         )
    #     ]
    # )

    return LaunchDescription([
        sensors_launch,
        slam_launch
    ])


# ros2 launch morosai_navigation slam_mapping.launch.py
# ros2 run teleop_twist_keyboard teleop_twist_keyboard
# ros2 run nav2_map_server map_saver_cli -f ~/my_map


# Per avviare la localizzazione con AMCL dopo aver salvato la mappa:
# ros2 launch nav2_bringup localization_launch.py \
# map:=/home/orion/morosai_mini_ws/src/morosai_navigation/maps/my_map2.yaml \
# use_sim_time:=false \
# params_file:=/home/orion/morosai_mini_ws/src/morosai_navigation/config/amcl.yaml




# ⚠️ Important: SLAM Toolbox Localization uses Serialized Maps
# Wait! Before you use SLAM Toolbox Localization, you need to know a very critical difference between AMCL and SLAM Toolbox. Standard ROS 2 Navigation (AMCL) uses a simple map.yaml and .pgm image map. 
# SLAM Toolbox Localization does NOT use these image files. Instead, it needs a serialized map (a .posegraph file) which contains the complete pose graph and laser scans of the environment.

# to serialize a map while in mapping mode:
# ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '/home/orion/morosai_mini_ws/src/morosai_navigation/maps/map_blabla'}"
