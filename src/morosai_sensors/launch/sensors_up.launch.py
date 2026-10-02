#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
import os
import yaml


def load_yaml(package, path):
    pkg_dir = get_package_share_directory(package)
    full = os.path.join(pkg_dir, path)
    with open(full, 'r') as f:
        return yaml.safe_load(f)


def generate_launch_description():

    # === Path packages ===
    pkg_sllidar = get_package_share_directory('sllidar_ros2')
    pkg_morosai_desc = get_package_share_directory('morosai_description')
    pkg_morosai_sensors = get_package_share_directory('morosai_sensors')

    # === External Launch Files ===
    sllidar_launch_path = os.path.join(pkg_sllidar, 'launch', 'sllidar_a3_launch_multiple.py')
    dual_laser_merger_launch_path = os.path.join(pkg_morosai_sensors, 'launch', 'dual_laser_merger_custom.launch.py')

    # === Load Sensor Config ===
    # Switch to 'config/sensors.yaml' for the standard robot (camera upside-down).
    # Switch to 'config/sensors_cnr.yaml' for the CNR robot (camera upright).
    config = load_yaml('morosai_sensors', 'config/sensors_cnr.yaml')

    tof_cfg          = config['tof_camera']
    lidar_front_cfg  = config['lidar_front']
    lidar_rear_cfg   = config['lidar_rear']
    optical_head_cfg = config['optical_head']

    return LaunchDescription([

        # === TOF CAMERA Node ===
        Node(
            package='kea_camera',
            executable='tof_camera_calib_opt_node',
            namespace=tof_cfg["namespace"],
            name=tof_cfg["node_name"],
            output='screen',
            respawn=True,
            parameters=[{
                'serial_number': tof_cfg["serial_number"],
                'frame_id':      tof_cfg["frame_id"],
                'vertical_flip': tof_cfg.get("vertical_flip", False),
            }]
        ),

        # === OPTICAL HEAD Node ===
        Node(
            package='optical_head',
            executable='optical_head_converted',
            name=optical_head_cfg['node_name'],
            namespace=optical_head_cfg['namespace'],
            output='screen',
            # respawn=True,
            parameters=[{
                'frame_id': optical_head_cfg['frame_id'],
                'serial_port': optical_head_cfg['serial_port']
            }]
        ),

        # # === Virtual Corridor Node ===
        # Node(
        #     package='morosai_sensors',
        #     executable='virtual_corridor_updated.py',
        #     name='virtual_corridor_node_updated',
        #     output='screen',
        #     parameters=[{
        #         'corridor_width': 0.4,
        #         'corridor_length': 2.0
        #     }]
        # ),

        # === Include SLLIDAR MULTIPLE ===
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sllidar_launch_path),
            launch_arguments={
                'serial_port1': lidar_front_cfg['serial_port'],
                'serial_port2': lidar_rear_cfg['serial_port'],
                'name1': lidar_front_cfg['node_name'],
                'name2': lidar_rear_cfg['node_name'],
                'frame_id1': lidar_front_cfg['frame_id'],
                'frame_id2': lidar_rear_cfg['frame_id'],
                'serial_baudrate1': str(lidar_front_cfg['serial_baudrate']),
                'serial_baudrate2': str(lidar_rear_cfg['serial_baudrate']),
            }.items()
        ),

        # === Include Dual Laser Merger ===
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(dual_laser_merger_launch_path)
        ),

        # === Sensor Sync Python Node ===
        # Node(
        #     package='morosai_sensors',
        #     executable='sensor_sync.py',
        #     name='sensor_sync_node',
        #     output='screen'
        # ),

        # === Robot State Publisher (URDF) ===
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': ParameterValue(
                    Command([
                        'xacro ',
                        os.path.join(pkg_morosai_desc, 'urdf', 'morosai_agv.urdf.xacro'),
                        ' is_ignition:=false'
                    ]),
                    value_type=str
                )
            }]
        ),

        # === TOF Intensity Filter Node ===
        Node(
            package='morosai_sensors',
            executable='pointcloud_filter_tof.py',
            name='tof_intensity_filter_node',
            output='screen',
            parameters=[{
                'intensity_threshold_down': 10
            }]
        ),

        # === LiDAR Watchdog Node ===
        Node(
            package='sllidar_ros2',
            executable='watchdog_lidar.py',
            name='lidar_watchdog_node',
            output='screen',
            parameters=[{
                'timeout_sec': 3.0
            }]
        ),

        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='camera',
            namespace='camera',
            output='screen',
            parameters=[{
                'enable_pose': True,
                'device_type': 't265'
            }]
        )

    ])
