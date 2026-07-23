# morosai_sensors

The `morosai_sensors` package is responsible for interfacing with all hardware sensors on the MOROSAI AGV and providing synchronization monitoring.

## Included Components

- **`sensors_up.launch.py`**: A unified launch file that starts the Lidar, TOF Camera, and PGV100 drivers simultaneously.
- **`sensor_sync.py` (The Health Monitor)**: 
    - An advanced diagnostic tool that monitors the arrival frequency (Hz) and latency of all sensors.
    - Categorizes sensors into **Critical (Lidar/TOF)** and **Optional (PGV)**.
    - Publishes real-time health reports to `/sensor_sync/health_report`.

## Configuration
- Device ports (e.g., `/dev/ttyUSB0`, `/dev/pgv100`) and IP addresses for the TOF camera are managed in the `config/` directory.

## Monitoring
To check sensor health from the terminal:
```bash
ros2 topic echo /sensor_sync/health_report
```
Look for the 🟢 OK status and an `Inter-sensor Drift` below 0.1s for optimal SLAM performance.

## PGV floor docking

`pgv_floor_dock_node.py` watches `/pgv100_scan`. When a valid PGV tag is detected
between `x=200` and `x=800` mm, it requests cancellation of active Nav2 goals,
simultaneously corrects its heading and centers the robot on `y=0`, then drives
in the positive robot X direction. The forward velocity decreases linearly and
reaches zero at `x=800` mm.

Start it after Nav2 and the optical-head node:

```bash
ros2 run morosai_sensors pgv_floor_dock_node.py
```

The default command path is `/cmd_vel_raw`, which is the configured input to
the workspace collision monitor. The lateral correction assumes a positive
PGV `y_pos` requires negative robot `linear.y`; invert it with
`--ros-args -p lateral_sign:=1.0` if the first alignment motion is reversed.
The docking node is controlled by the scheduler through `/agv_dock`
(`std_msgs/Bool`):

- `true` / `1`: arm or reset the node for a new docking mission;
- `false` / `0`: stop and disarm the node.

After `SUCCESS`, the node disarms itself and will not start docking again when
the same tag is seen during a later navigation goal. The scheduler should arm
it again only when the robot must dock at the next station:

```bash
ros2 topic pub --once /agv_dock std_msgs/msg/Bool "{data: true}"
```

Useful parameters are `max_forward_speed`, `max_lateral_speed`,
`y_tolerance_mm`, `x_start_mm`, `x_stop_mm`, and `scan_timeout`. Heading
alignment uses `target_angle_deg`, `angle_tolerance_deg`, `angle_kp`,
`max_angular_speed`, and `angle_sign`. Set `target_angle_deg` to the PGV angle
that corresponds to the robot rear pointing toward increasing PGV X; the
default `180.0` degrees must be verified against the physical sensor mounting.
