cmake_minimum_required(VERSION 3.5)
project(kea_camera)

# ============================================================
# TOF Library Configuration
# Set TOF_LIB_DIR environment variable or modify default path
# ============================================================
set(TOF_LIB_DIR "/home/robouser/tof_lib")

message(STATUS "Using TOF library from: ${TOF_LIB_DIR}")

# Add TOF library to CMake search path
list(APPEND CMAKE_PREFIX_PATH "${TOF_LIB_DIR}/lib/cmake/tof")

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tof REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(Eigen3 REQUIRED)

# Include directories
include_directories(
  ${TOF_LIB_DIR}/include
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(
  ${TOF_LIB_DIR}/lib
)

# Add executables
add_executable(image_topics src/image_topics.cpp)
add_executable(pointcloud_topics src/pointcloud_topics.cpp)
add_executable(tof_camera_node src/tof_camera_node.cpp)
add_executable(tof_camera_calib_node src/tof_camera_calib_node.cpp)
add_executable(tof_camera_calib_opt_node src/tof_camera_calib_opt_node.cpp)
add_executable(slim_code src/slim_code.cpp)
add_executable(original src/original.cpp)

# Add dependencies
ament_target_dependencies(image_topics rclcpp std_msgs sensor_msgs tof cv_bridge)
ament_target_dependencies(pointcloud_topics rclcpp std_msgs sensor_msgs tof cv_bridge image_transport pcl_conversions PCL) 
ament_target_dependencies(tof_camera_node rclcpp std_msgs sensor_msgs tof cv_bridge image_transport pcl_conversions PCL) 
ament_target_dependencies(tof_camera_calib_node rclcpp std_msgs sensor_msgs tof cv_bridge image_transport pcl_conversions PCL) 
ament_target_dependencies(tof_camera_calib_opt_node rclcpp std_msgs sensor_msgs tof cv_bridge image_transport pcl_conversions PCL) 
ament_target_dependencies(slim_code rclcpp std_msgs sensor_msgs tof cv_bridge image_transport pcl_conversions PCL) 
ament_target_dependencies(original rclcpp std_msgs sensor_msgs tof cv_bridge image_transport pcl_conversions PCL) 

# Link libraries
target_link_libraries(image_topics tof::tof ${OpenCV_LIBS})
target_link_libraries(pointcloud_topics tof::tof ${OpenCV_LIBS})
target_link_libraries(tof_camera_node tof::tof ${OpenCV_LIBS})
target_link_libraries(tof_camera_calib_node tof::tof ${OpenCV_LIBS})
target_link_libraries(tof_camera_calib_opt_node tof::tof ${OpenCV_LIBS})
target_link_libraries(slim_code tof::tof ${OpenCV_LIBS})
target_link_libraries(original tof::tof ${OpenCV_LIBS})

#Set RPATH so that tof library can be found at runtime
set_target_properties(image_topics pointcloud_topics tof_camera_node tof_camera_calib_node tof_camera_calib_opt_node slim_code original
  PROPERTIES 
    INSTALL_RPATH "${TOF_LIB_DIR}/lib"
    BUILD_RPATH "${TOF_LIB_DIR}/lib"
    INSTALL_RPATH_USE_LINK_PATH TRUE
)

# Install executables
install(TARGETS 
  image_topics 
  pointcloud_topics 
  tof_camera_node 
  tof_camera_calib_node
  tof_camera_calib_opt_node
  slim_code 
  original
  DESTINATION lib/${PROJECT_NAME}
)

# Install config and rviz directories
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY rviz/ DESTINATION share/${PROJECT_NAME}/rviz)

ament_package()



# [INFO] [tof_camera_node-1]: process started with pid [14834]
# [ERROR] [tof_camera_node-1]: process has died [pid 14834, exit code 127, cmd '/home/orion/morosai_mini_ws/install/kea_camera/lib/kea_camera/tof_camera_node --ros-args -r __node:=tof_camera_node -r __ns:=/tof --params-file /tmp/launch_params_2wnq98n0'].
# Solved with Set RPATH above