cmake_minimum_required(VERSION 3.8)
project(realsense_t265_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

# Trova librealsense2 installata sul sistema
find_package(realsense2 REQUIRED)

add_executable(realsense_t265_node src/realsense_t265_node.cpp)
ament_target_dependencies(realsense_t265_node rclcpp geometry_msgs)
target_link_libraries(realsense_t265_node realsense2::realsense2)

install(TARGETS
  realsense_t265_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()