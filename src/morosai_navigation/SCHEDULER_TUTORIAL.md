# MOROSAI Scheduler ROS 2 Tutorial

This document describes the ROS 2 topics that the scheduler uses to command the
MOROSAI AGV:

- `/agv_act`: send a navigation goal to Nav2;
- `/agv_align`: align with an AprilTag using the TOF camera;
- `/agv_dock`: arm or disarm PGV floor-tag docking.

The scheduler can send these messages directly through ROS 2 or through the
MQTT bridge. The MQTT bridge keeps the same topic names and serializes message
fields to JSON.

## Recommended Mission Sequence

For a normal navigation and docking mission:

1. Send a navigation goal on `/agv_act`.
2. If an AprilTag alignment is required, send its frame name on `/agv_align`.
3. If PGV floor docking is required, send `true` on `/agv_dock` before the
   robot reaches the PGV tag.
4. The PGV node cancels active Nav2 goals when it detects a valid tag and then
   performs the final alignment and approach.
5. After docking, the PGV node disables itself automatically. It will not
   reactivate when the same tag remains under the sensor.
6. Send `false` on `/agv_dock` when the scheduler wants to explicitly disable
   docking, or send `true` again before the next docking mission.

The scheduler should send the docking-enable command before the robot enters
the PGV tag detection region. Sending it after the tag has already passed may
require a new approach or a new mission.

## `/agv_dock`: PGV Floor Docking

### ROS 2 message type

```text
std_msgs/msg/Bool
```

### Meaning of the flag

| Value | Meaning |
|---|---|
| `true` / `1` | Arm or re-arm the PGV docking node for a new mission. |
| `false` / `0` | Stop the docking controller and disarm it. |

The PGV docking node starts disabled. A tag crossing by itself does not start
motion. After receiving `true`, the node waits for a valid PGV scan:

- `tag_detected != 0`;
- `error == false`;
- `200 <= x_pos <= 800` mm.

When a valid tag is detected, the node cancels active Nav2 goals and publishes
slow velocity commands on `/cmd_vel_raw`. The collision monitor remains in the
command path. The robot first aligns `y_pos` to zero and then moves in positive
robot X. The approach speed decreases as `x_pos` approaches `800` mm.

The node stops and disarms if the PGV scan becomes stale, reports an error, or
exceeds the hard `x_pos <= 800` limit. It also disarms after `SUCCESS`. A new
`true` command is required before another docking attempt.

### Terminal examples

Arm docking:

```bash
ros2 topic pub --once /agv_dock std_msgs/msg/Bool "{data: true}"
```

Disarm docking:

```bash
ros2 topic pub --once /agv_dock std_msgs/msg/Bool "{data: false}"
```

Watch the docking status:

```bash
ros2 topic echo /pgv_floor_dock/status
```

Possible status values include:

```text
ARMED
TAG_DETECTED_CANCELING_NAV
APPROACHING
SUCCESS
FAILED_TAG_LOST
DISABLED
```

### MQTT examples

The MQTT command uses the same topic and field name:

```json
{
  "topic": "/agv_dock",
  "data": true
}
```

To disarm:

```json
{
  "topic": "/agv_dock",
  "data": false
}
```

The exact outer MQTT envelope depends on the MQTT wrapper configuration. The
ROS message payload must contain the Bool field `data`.

## `/agv_align`: AprilTag Alignment

### ROS 2 message type

```text
std_msgs/msg/String
```

The string must be the complete TF frame name of the AprilTag. For the standard
AprilTag configuration this is normally:

```text
tagStandard52h13:<tag_id>
```

The node accepts only tags listed in its `allowed_tags` parameter. It then uses
the TOF camera transform to align with the selected tag.

### Terminal example

```bash
ros2 topic pub --once /agv_align std_msgs/msg/String \
  "{data: 'tagStandard52h13:108'}"
```

Watch the result:

```bash
ros2 topic echo /align_result
```

Typical result values are:

```text
SUCCESS
REJECTED_NOT_ALLOWED
REJECTED_NOT_VISIBLE
FAILED_TIMEOUT
```

### MQTT example

```json
{
  "topic": "/agv_align",
  "data": "tagStandard52h13:108"
}
```

The AprilTag frame name must match the configured tag exactly. Sending only the
numeric ID, for example `108`, is not sufficient for the current node.

## `/agv_act`: Nav2 Navigation Goal

### ROS 2 message type used by `mqtt_bridge_publisher.py`

```text
morosai_navigation/msg/Pose
```

Message fields:

```text
float32 x
float32 y
float32 angle
```

The goal is expressed in the `map` frame. `x` and `y` are metres. `angle` is
the target yaw in radians.

### Terminal example

```bash
ros2 topic pub --once /agv_act morosai_navigation/msg/Pose \
  "{x: 2.5, y: -1.0, angle: 0.0}"
```

The bridge receives this message, creates a Nav2 `NavigateToPose` goal, and
sends it to the `navigate_to_pose` action server.

Check the topic type before publishing:

```bash
ros2 topic type /agv_act
```

Expected output for the `mqtt_bridge_publisher.py` path:

```text
morosai_navigation/msg/Pose
```

### MQTT example

```json
{
  "topic": "/agv_act",
  "x": 2.5,
  "y": -1.0,
  "angle": 0.0
}
```

The scheduler should publish a new goal only when at least one of `x`, `y`, or
`angle` changes. The current bridge ignores an identical goal if it is received
again.

## Useful Inspection Commands

List active topics:

```bash
ros2 topic list
```

Inspect a topic's type:

```bash
ros2 topic type /agv_dock
ros2 topic type /agv_align
ros2 topic type /agv_act
```

Inspect live messages:

```bash
ros2 topic echo /agv_dock
ros2 topic echo /agv_align
ros2 topic echo /agv_act
```

## `/map_new`: Updating the Navigation Map

The `map_updater_node.py` node allows the system to replace the active Nav2
map when a new map is generated or received. The scheduler or mapping process
publishes a complete `nav_msgs/msg/OccupancyGrid` on `/map_new`.

The node then:

1. writes the received grid to `map.pgm` and `map.yaml`;
2. atomically replaces the previous files;
3. calls `/map_server/load_map` to reload the new YAML map;
4. optionally clears the global and local costmaps;
5. optionally reinitializes AMCL.

### ROS 2 message type

```text
nav_msgs/msg/OccupancyGrid
```

The message must contain a complete map, including:

- `info.width` and `info.height`;
- `info.resolution` in metres per cell;
- `info.origin`, including its position and orientation;
- `data`, with one occupancy value per cell.

Occupancy values use the standard ROS convention:

- `0`: free;
- `100`: occupied;
- `-1`: unknown.

The default node configuration writes the files to `/tmp/nav2_maps/map.pgm`
and `/tmp/nav2_maps/map.yaml`. These paths can be changed with ROS parameters.

### Starting the node

The node must be running together with Nav2 and a map server:

```bash
ros2 run morosai_navigation map_updater_node.py
```

If the executable is installed under another package, use that package name in
the `ros2 run` command. The required services are normally provided by Nav2:

```text
/map_server/load_map
/global_costmap/clear_entirely_global_costmap
/local_costmap/clear_entirely_local_costmap
```

### Publishing a test map from the terminal

For a small synthetic test map, publish an `OccupancyGrid` directly:

```bash
ros2 topic pub --once /map_new nav_msgs/msg/OccupancyGrid "{
  header: {frame_id: map},
  info: {
    map_load_time: {sec: 0, nanosec: 0},
    resolution: 0.05,
    width: 4,
    height: 3,
    origin: {
      position: {x: 0.0, y: 0.0, z: 0.0},
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
    }
  },
  data: [0, 0, 0, 0, 0, 100, 100, 0, 0, 0, -1, 0]
}"
```

The `data` array must contain exactly `width * height` values. For a real map,
the scheduler should publish the generated `OccupancyGrid` instead of building
the array manually in the shell.

### Useful parameters

```bash
ros2 run morosai_navigation map_updater_node.py --ros-args \
  -p map_new_topic:=/map_new \
  -p map_directory:=/tmp/nav2_maps \
  -p map_name:=map \
  -p update_period_sec:=2.0 \
  -p clear_costmaps_on_update:=true \
  -p reinitialize_amcl_on_update:=false
```

`update_period_sec` limits how frequently the node writes and reloads maps.
When a new map arrives while the node is busy, the most recent pending map is
kept and processed after the current reload finishes.

### Important operational note

Reloading the map changes the map used by Nav2. Clearing costmaps is normally
recommended after the reload. Reinitializing AMCL is disabled by default
because it is disruptive: the robot may lose its current localization and
need to be localized again. Use `reinitialize_amcl_on_update:=true` only when
the new map is not aligned with the current localization or when the scheduler
is prepared to provide a new initial pose.

Check the update with:

```bash
ros2 topic echo /map_new --once
ros2 service call /map_server/load_map nav2_msgs/srv/LoadMap \
  "{map_url: /tmp/nav2_maps/map.yaml}"
```

Check the Nav2 action server:

```bash
ros2 action list
ros2 action info /navigate_to_pose
```
