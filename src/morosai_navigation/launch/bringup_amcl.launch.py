from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Percorsi ai pacchetti
    morosai_sensors_dir = get_package_share_directory('morosai_sensors')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # Include sensors.launch.py (LiDAR + ToF + fusion)
    sensors_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(morosai_sensors_dir, 'launch', 'sensors.launch.py')
        )
    )

    # Include localization_launch.py di Nav2 con path espliciti
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'localization_launch.py')
        ),
        launch_arguments={
            'map': '/home/orion/morosai_mini_ws/src/morosai_navigation/maps/my_map2.yaml',
            'use_sim_time': 'false',
            'params_file': '/home/orion/morosai_mini_ws/src/morosai_navigation/config/amcl.yaml'
        }.items()
    )

    return LaunchDescription([
        # sensors_launch,
        localization_launch
    ])

    
# ros2 launch morosai_navigation bringup_amcl.launch.py