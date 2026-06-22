from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchSubstitution, LaunchArgument

def generate_launch_description():
    bag_file = LaunchArgument('bag', description='Percorso del file rosbag')
    calib_file = LaunchArgument('calib', default_value='tof_calib.yaml', description='Percorso del file YAML di calibrazione')

    return LaunchDescription([
        bag_file,
        calib_file,
        
        # Riproduce il bag
        ExecuteProcess(
            cmd=['ros2', 'bag', 'play', LaunchSubstitution('bag')],
            output='screen'
        ),

        # Pubblica le CameraInfo statiche
        Node(
            package='camera_info_manager', # Assicurati che sia installato: sudo apt install ros-humble-camera-info-manager
            executable='camera_info_publisher', # Nota: potrebbe servire un pacchetto specifico o un piccolo script
            name='camera_info_publisher',
            parameters=[{'camera_info_url': f"file://{os.path.abspath(LaunchSubstitution('calib'))}"}],
            remappings=[('/camera_info', '/gordon_tof/camera_info')]
        ),
    ])

# In alternativa, uno script ROS 2 più semplice per il publisher
