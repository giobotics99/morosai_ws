# morosai_description

The `morosai_description` package contains the physical and visual representation of the MOROSAI AGV. It provides the URDF (Unified Robot Description Format) models used for coordinate transformations (TF), simulation, and visualization in RViz.

## Package Structure

- **`urdf/`**: Contains the `.xacro` and `.urdf` files defining the robot's links (body, wheels, sensor mounts) and joints.
- **`meshes/`**: Storage for 3D visual and collision models (e.g., `.stl` or `.dae` files) representing the actual robot components.
- **`rviz/`**: Pre-configured RViz visualization settings (e.g., `display_NAV2.rviz`) to quickly monitor navigation and sensory data.
- **`launch/`**:
    - `description.launch.py`: Publishes the robot state to the `/robot_description` topic and manages the static transforms between sensor frames.

## Coordinate Frames (TF)
The package defines the essential transform tree:
`map` -> `odom` -> `base_footprint` -> `base_link` -> `lidar_link`, `gordon_tof`, etc.
