import os
from pathlib import Path
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution



def generate_launch_description():
    
    warehouse_world = "/home/orion/morosai_ws/src/morosai_description/worlds/warehouse.sdf"

    # Determine the ROS distribution and if Ignition simulation is used
    morosai_description_dir = get_package_share_directory("morosai_description")
    rviz_config_path = os.path.join(morosai_description_dir, "rviz", "display.rviz")
    ros_distro = os.environ["ROS_DISTRO"]
    is_ignition = "True" if ros_distro == "humble" else "False"


    # Declare path to the xacro model
    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(morosai_description_dir, "urdf", "morosai_agv.urdf.xacro"),
        description="Absolute path to robot URDF file"
    )

    # Process xacro and pass it to robot_state_publisher
    robot_description = ParameterValue(
        Command([
            "xacro ",
            LaunchConfiguration("model"),
            " is_ignition:=",
            is_ignition
        ]),
        value_type=str
    )

    # Robot State Publisher node
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}]
    )

    # Joint State Publisher GUI (optional if not using real hardware)
    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui"
    )

    # RViz visualization
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_path]
    )

    # Gazebo environment resource path
    gazebo_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[
            str(Path(morosai_description_dir).parent.resolve())
        ]
    )

    # Gazebo launch file
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")
        ]),
        launch_arguments=[
            ("gz_args", f"-r -v 4 {warehouse_world}")
        ]
    )

    # Spawn the robot into Gazebo
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-topic", "robot_description", "-name", "morosai_agv"]
    )

    return LaunchDescription([
        model_arg,
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz_node,
        gazebo_resource_path,
        gazebo,
        gz_spawn_entity
    ])
