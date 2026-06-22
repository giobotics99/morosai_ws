# MOROSAI AGV: Synchronization & Alignment Tutorial

This tutorial explains how to verify that your robot's sensors, transforms (TF), and SLAM are correctly synchronized and aligned. This is critical for high-quality mapping and robust navigation, especially when using **Synchronous SLAM**.

## 1. Verifying Coordinate Frames (TF)
The most common cause of mapping "smearing" is a lag or misalignment in the TF tree.

### Check TF Tree
Run the following command to see if all frames are connected and have low latency:
```bash
ros2 run tf2_tools view_frames
```
*   **What to look for**: `map -> odom -> base_footprint -> [sensor_frames]`.
*   **Consistency**: The "broadcaster" for each link should have a consistent update rate (e.g., 20Hz-50Hz).

### RViz Visualization
1. Set **Fixed Frame** to `map`.
2. Add a `TF` display.
3. Observe the LIDAR data (Pointcloud or LaserScan).
4. **The Test**: Rotate the robot in place. The LIDAR points representing walls should stay in the same position in the `map` frame even as the robot turns. If they move with the robot, your `odom -> base_footprint` transform or LIDAR transform is incorrect.

## 2. Checking Message Timestamps
All sensors must use the same clock. If you are using physical hardware, ensure they use **System Time**.

### How to ensure System Time Consistency
When using multiple sensors (e.g., a LIDAR on USB and a TOF camera on Ethernet):

1. **Host Synchronization**: If you have multiple computers (e.g., an OAK-D camera on its own RPi and the Jetson Orin), use **Chrony** to sync their clocks.
   - **Check Chrony Status**: On the Jetson, run:
     ```bash
     chronyc sources -v
     chronyc tracking
     ```
   - **What to look for**: The "System time" offset should be very small (e.g., < 0.001s). If `tracking` shows a large offset, your time sync service is not working.

2. **Driver Configuration**: I have checked your specific workspace drivers (`sllidar_ros2` and `kea_camera`) and confirmed their behavior:
   - **`sllidar_ros2`**: The code (`sllidar_node.cpp`) uses `this->now()` to stamp every scan. It automatically uses your Jetson's system time.
   - **`kea_camera`**: The code (`tof_camera_node.cpp`) also uses `this->now()` for all pointclouds and images.
   - **Conclusion**: You do **not** need to add any special parameters to your YAML or launch files; they are already hardcoded to use the best possible clock (the host system clock).

3. **Check ALL Sensors**: You asked if you should check only `/merged`. **No**, you should check every sensor that contributes to Nav2 (LIDAR, TOF, etc.).
   - **LIDAR & TOF (Critical)**: These must be perfectly synced for costmaps and SLAM.
   - **PGV100 (Optional)**: This is monitored independently because it is a "background" sensor. If it lags, it won't stop the LIDAR-TOF monitor from working.

### Monitoring with Advanced Sync Tool
Run the monitor included in your workspace:
```bash
ros2 run morosai_sensors sensor_sync.py
```
Check the health report:
```bash
ros2 topic echo /sensor_sync/health_report
```
*   **What to look for**: 
    - `LIDAR` and `TOF` should show `🟢 OK`.
    - `Inter-sensor Drift` should be `< 0.1s`.
    - `PGV` is monitored as `(OPT)` and will show its own frequency and latency independently.

## 3. LIDAR Synchronization
For SLAM to work perfectly, the scan data must be correctly associated with the robot's pose at the *exact moment* the scan was taken.

### scan_topic consistency
In `mapper_params_online_sync.yaml`, ensure `scan_topic` matches your actual topic:
```yaml
scan_topic: /merged
```

### Update rates
- **LIDAR**: Typically 10-15Hz.
- **Odometry**: 20-50Hz.
- **SLAM Update**: 5Hz (default in `map_update_interval`).

If your LIDAR is too fast and your CPU is slow, **Synchronous SLAM** will start to lag behind real-time. You can monitor this by checking if the `/map` update frequency slows down significantly during fast movements.

## 4. Troubleshooting "Ghost" Walls
If you see multiple sets of walls appearing:
1. **Time Offset**: Your LIDAR and Odometry timestamps might be out of sync.
2. **Transform Precision**: Check `morosai_description` URDF to ensure the LIDAR position (`xyz`) and orientation (`rpy`) are measured precisely.
3. **Scan Matching**: In `mapper_params_online_sync.yaml`, try increasing `minimum_travel_distance` if the robot is creating "blobs" while standing still.
