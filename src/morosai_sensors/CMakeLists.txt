cmake_minimum_required(VERSION 3.8)
project(morosai_sensors)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rclpy REQUIRED)
find_package(action_msgs REQUIRED)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}/
)

install(
  PROGRAMS
    scripts/sensor_sync.py
    scripts/odom_to_base_footprint.py
    scripts/virtual_corridor.py
    scripts/virtual_corridor_updated.py
    scripts/dummy_odom_publisher.py
    scripts/yolo_tof_detector.py
    scripts/tag_aligner_node.py
    scripts/tag_align_service.py
    scripts/tag_dock_align_service.py
    scripts/pgv_floor_dock_node.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
