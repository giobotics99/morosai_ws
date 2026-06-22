cmake_minimum_required(VERSION 3.8)
project(morosai_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(dwb_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate custom messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "morosai_msgs/Pose.msg"
  "morosai_msgs/Path.msg"
  "morosai_msgs/Detect.msg"
  "morosai_msgs/NavStatus.msg"
  "morosai_msgs/LocalPath.msg"
  "morosai_msgs/Tag.msg"
  "morosai_msgs/FinalPath.msg"
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

install(
  DIRECTORY maps
  DESTINATION share/${PROJECT_NAME}/
)

# Install Python scripts
install(
  PROGRAMS
    scripts/nav_topic_publisher.py
    scripts/mqtt_bridge_publisher.py
    scripts/bag_analysis_node.py
    scripts/extract_metrics.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
