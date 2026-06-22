# tof_camera

The `tof_camera` package is the ROS 2 driver interface for the **Kea / Chronoptics TOF (Time-of-Flight) Camera**. It provides dense 3D point cloud data for high-fidelity obstacle avoidance and environment sensing.

## Driver Functionality
- Interfaces with the proprietary Chronoptics SDK to capture depth maps and images.
- Publishes `sensor_msgs/msg/PointCloud2` and `sensor_msgs/msg/Image`.
- Real-time conversion from raw sensor phase data to calibrated 3D space.

## Integration Details
The driver is implemented as a ROS 2 node that stamps every message with the **Jetson System Time** to ensure perfect alignment with the Lidar data. It is a critical component for detecting small or low-lying obstacles that the planar Lidar might miss.


# ToF Ros node
This is a node wrapper around the tof library and kea camera class showing off some of the advanced features of the tof library for optimal performance. The camera starts streaming as soon as one of it's publishers is subscribed to. 

The node accepts the following private parameters
- serial: (string) The serial of the camera you want to connect to. Default empty
- log: (bool) Whether to output log messages from the tof library. Default true
- queue_size: (int) The amount of user pointers to queue up. Default 10

The node has been tested with ros 2 humble in a docker container

## Normal

Copy the folder this readme is in into your ros workspace.

    cp -R examples/ros ~/ros_ws/src/kea_camera

Build like usual:

    colcon build 

Source the setup.bash

    source install/setup.bash

Run the ros node

    ros2 run kea_camera kea_camera_node

## Docker

To build image:

    docker build -t ros-tof .

To run image:

    docker run -it --name ros-develop --network=host -v $(pwd):/workspace/src/kea_camera/ ros-tof

Afterwards you can follow the normal instructions starting from build like usual.
