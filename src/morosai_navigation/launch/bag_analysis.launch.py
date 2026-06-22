"""
Launch file for bag analysis.

Launches the bag_analysis_node that subscribes to Nav2 topics.
You can either play the bag manually or use the `bag_path` argument to automate it.

Usage Option 1 — Launch node + play bag manually:
    Terminal 1:  ros2 launch morosai_navigation bag_analysis.launch.py
    Terminal 2:  ros2 bag play /path/to/your/bag_folder

Usage Option 2 — Automated playback (Recommended):
    ros2 launch morosai_navigation bag_analysis.launch.py bag_path:=/path/to/bag_folder

When the bag finishes (in Option 2), the node automatically triggers analysis and exits.
In Option 1, it will trigger after `auto_shutdown_delay` seconds of silence or via Ctrl+C.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, Shutdown
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch_ros.actions import Node


def generate_launch_description():

    # ── Declare arguments ─────────────────────────────────────────
    turn_threshold_arg = DeclareLaunchArgument(
        'turn_threshold',
        default_value='0.4',
        description='Angular velocity threshold (rad/s) for turn detection'
    )

    csv_output_arg = DeclareLaunchArgument(
        'csv_output',
        default_value='',
        description='Path to save CSV output (empty = no CSV)'
    )

    auto_shutdown_delay_arg = DeclareLaunchArgument(
        'auto_shutdown_delay',
        default_value='10.0',
        description='Seconds of silence before auto-running analysis (fallback)'
    )

    bag_path_arg = DeclareLaunchArgument(
        'bag_path',
        default_value='',
        description='Path to rosbag2 folder. If set, bag play is launched automatically.'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time (recommended for bag analysis)'
    )

    # ── Configuration Substitutions ───────────────────────────────
    bag_path = LaunchConfiguration('bag_path')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Enable watchdog only if bag_path is NOT set (manual mode)
    # If bag_path is set, we use the process exit event instead.
    enable_watchdog = PythonExpression([
        "'false' if '", bag_path, "' != '' else 'true'"
    ])

    # ── Analysis Node ─────────────────────────────────────────────
    analysis_node = Node(
        package='morosai_navigation',
        executable='bag_analysis_node.py',
        name='bag_analysis_node',
        output='screen',
        parameters=[{
            'turn_threshold': LaunchConfiguration('turn_threshold'),
            'csv_output': LaunchConfiguration('csv_output'),
            'auto_shutdown_delay': LaunchConfiguration('auto_shutdown_delay'),
            'enable_watchdog': enable_watchdog,
            'use_sim_time': use_sim_time,
        }],
    )

    # ── Bag Player ────────────────────────────────────────────────
    # Only launched if bag_path is provided
    bag_player = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', bag_path],
        output='screen',
        condition=IfCondition(PythonExpression(["'", bag_path, "' != ''"]))
    )

    # ── Event Handlers ──────────────────────────────────────────
    # When bag player finishes, shutdown the entire launch (triggers analysis node report)
    shutdown_on_bag_exit = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=bag_player,
            on_exit=[Shutdown(reason='Rosbag playback finished.')]
        ),
        condition=IfCondition(PythonExpression(["'", bag_path, "' != ''"]))
    )

    return LaunchDescription([
        turn_threshold_arg,
        csv_output_arg,
        auto_shutdown_delay_arg,
        bag_path_arg,
        use_sim_time_arg,
        analysis_node,
        bag_player,
        shutdown_on_bag_exit,
    ])
