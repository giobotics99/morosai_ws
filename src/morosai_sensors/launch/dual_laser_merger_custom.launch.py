from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():

    container = ComposableNodeContainer(
        name='dual_laser_merger_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='dual_laser_merger',
                plugin='merger_node::MergerNode',
                name='dual_laser_merger',
                parameters=[{
                    'laser_1_topic': '/lidar_front/scan',
                    'laser_2_topic': '/lidar_rear/scan',
                    'merged_topic': '/merged',
                    'publish_rate': '100',
                    'target_frame': 'base_footprint',  # usa il TF già presente
                    'queue_size': 40,
                    'angle_increment': 0.001,
                    'scan_time': 0.067,
                    'range_min': 0.01,
                    'range_max': 20.0
                }]
            )
        ],
        output='screen',
    )

    return LaunchDescription([
        container
    ])
