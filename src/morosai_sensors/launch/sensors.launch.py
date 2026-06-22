from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
import os

def generate_launch_description():
    pkg_sllidar = get_package_share_directory('sllidar_ros2')
    pkg_morosai_desc = get_package_share_directory('morosai_description')
    pkg_morosai_sensors = get_package_share_directory('morosai_sensors')

    sllidar_launch_path = os.path.join(pkg_sllidar, 'launch', 'sllidar_a3_launch_multiple.py')
    dual_laser_merger_launch_path = os.path.join(pkg_morosai_sensors, 'launch', 'dual_laser_merger_custom.launch.py')

    return LaunchDescription([
        # === TOF Camera Node ===
        Node(
            package='kea_camera',
            executable='tof_camera_node',
            name='tof_camera_node',
            output='screen'
        ),

        Node(
            package='optical_head',
            executable='optical_head_converted',
            name='optical_head_converted',
            output='screen'
        ),

        # === Virtual Corridor Node ===
        Node(
            package='morosai_sensors',
            executable='virtual_corridor_updated.py',
            name='virtual_corridor_node_updated',
            output='screen',
            parameters=[{
                'corridor_width': 0.4,
                'corridor_length': 2.0
            }]
        ),

        # === Include SLLidar Launch ===
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sllidar_launch_path)
        ),

        # === Include Dual Laser Merger Launch ===
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(dual_laser_merger_launch_path)
        ),

        # === Sensor Sync Python Node ===
        Node(
            package='morosai_sensors',
            executable='sensor_sync.py',
            name='sensor_sync_node',
            output='screen'
        ),

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
        )

        # Node(
        #     package='joint_state_publisher',
        #     executable='joint_state_publisher',
        #     name='joint_state_publisher',
        #     parameters=[{'use_sim_time': 0}],
        # )

        # Node(
        #     package='morosai_sensors',
        #     executable='odom_to_base_footprint.py',
        #     name='odom_to_base_footprint_node',
        #     output='screen'
        # ),

        # Node(
        #     package="joint_state_publisher_gui",
        #     executable="joint_state_publisher_gui",
        #     name="joint_state_publisher_gui"
        # ),

        # # === RViz ===
        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     arguments=['-d', os.path.join(pkg_morosai_desc, 'rviz', 'display_Ingel.rviz')],
        #     output='screen'
        # )
    ])


# ros2 bag record /scan /tof_scan /tf /tf_static /odom /joint_states
# ros2 bag play <nome_rosbag>
