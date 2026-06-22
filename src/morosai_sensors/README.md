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
