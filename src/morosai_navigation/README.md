# MOROSAI Navigation Package
The `morosai_navigation` package provides the "intelligence" for the MOROSAI AGV. It manages the Nav2 stack configuration, environment maps, and the bridge between external mission commands and internal ROS 2 actions.

## Package Components
- **`nav2_bringup_final.yaml`**: The primary configuration for the Nav2 controllers, planners, and costmaps.
- **`nav_topic_publisher.py`**: An integration node that republishes standard ROS 2 topics into custom JSON formats (`/agv_pose`, `/agv_act`, etc.) for external monitoring and MQTT communication.
- **`maps/`**: Storage for `.yaml` and `.pgm` map files generated during SLAM.

This package contains the configuration, maps, and logic for the MOROSAI AGV autonomous navigation.

For scheduler integration details and terminal/MQTT examples, see
[SCHEDULER_TUTORIAL.md](SCHEDULER_TUTORIAL.md).

### `nav_topic_publisher.py` Topic Mappings

The `nav_topic_publisher.py` node serves as a bridge between internal, standard ROS 2/Nav2 topics and custom JSON-formatted topics that make external communication (like the MQTT bridge) much simpler.

| ROS 2 Standard Topic | Custom AGV Topic | Purpose / Description |
|----------------------|------------------|-----------------------|
| `/amcl_pose` | `/agv_pose` | Robot pose from localization (exported as JSON) |
| `/plan` | `/agv_path` | Planned global navigation path (array of poses in JSON) |
| `/local_plan` | `/agv_local_path` | Dynamic local path computed by the DWB planner |
| *Calculated Internally* | `/agv_deviation` | Minimum distance from the robot's pose to the DWB local path |
| `/cmd_vel` | `/agv_v` | Current linear velocity magnitude in m/s |
| `/optical_head/pgv100_scan` | `/agv_qr` | QR/tag detection events from the optical head |
| `/local_costmap/costmap` | `/agv_detect` | Processed obstacle detection information |
| `/joint_states` | `/wheel_v` | Individual wheel velocities and positions |
| `/goal_pose` (Nav2 Action) | `/agv_act` | Active waypoint array and current navigation status |
| `/agv_op` *(Input)* | `/agv_op` | Operator commands and system operational state |

## Headless Autonomous Navigation

In a production environment, the robot operates without a graphical interface (RViz). Navigation commands are sent via ROS 2 Actions or through the MQTT bridge.

### 1. Sending Goals via Terminal (CLI)

To send a navigation goal directly from the command line:

#### Using ROS 2 Actions (Recommended)
This method allows you to track the progress and receive feedback.
```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 2.5, y: -1.0}, orientation: {w: 1.0}}}}"
```

#### Using ROS 2 Topics (Fire-and-Forget)
```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped "{
  header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'map'},
  pose: {
    position: {x: 2.0, y: 1.0, z: 0.0},
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  }
}"
```

### 2. MQTT Bridge Architecture

Goal commands arrive from the **Cloud Scheduler** via MQTT.
- **MQTT Topic**: `morosai/cmd/goal`
- **JSON Format**: `{"target": "station_a", "type": "navigate"}`
- **Processing**: The `MQTT Wrapper` translates this into a ROS 2 message on `/agv_act`.
- **Execution**: The `Navigation` node (described in [nav_topic_publisher.py](scripts/nav_topic_publisher.py)) executes the task.

### 3. Waypoint Management (POI)

To navigate correctly, you must pre-save "Points of Interest" (POI).
1.  Map the environment using SLAM.
2.  Save the robot's coordinates for each station into a configuration file.
3.  The `SYSTEM` node will use these coordinates when the Scheduler requests a specific station name.

---

## Production Launching with Tmux

To manage multiple ROS 2 launch files and scripts in a headless environment, we use **tmux**. This allows you to:
- Run everything in one terminal session.
- Keep processes running even if you disconnect from SSH.
- Monitor logs for each component in separate panes.