# morosai_bringup

The `morosai_bringup` package is the main entry point for starting the MOROSAI AGV system. It orchestrates the launching of multiple subsystems (sensors, navigation, description) into a cohesive robot operation.

## Key Launch Files

- **`full_bringup.launch.py`**: Launches all core systems for standard autonomous operation, including sensor drivers and the base navigation stack.
- **`slam_bringup.launch.py`**: Specifically configured for asynchronous SLAM operations using the `slam_toolbox`. This is the default for exploring and mapping new environments.
- **`slam_sync_bringup.launch.py`**: A specialized launch file for **Synchronous SLAM**. Every sensor scan is processed sequentially to ensure maximum mapping accuracy, including the built-in health monitor.

## Dependencies
This package depends on:
- `morosai_sensors` (for hardware interface)
- `morosai_navigation` (for path planning and maps)
- `morosai_description` (for robot model and TF)
- `nav2_bringup` (standard ROS 2 navigation)

## Usage
To start the robot with synchronous mapping:
```bash
ros2 launch morosai_bringup slam_sync_bringup.launch.py
```
