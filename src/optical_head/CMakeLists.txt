cmake_minimum_required(VERSION 3.8)
project(optical_head)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Genera i messaggi ROS 2
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PgvScanData.msg"
  "msg/PgvDirMsg.msg"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

# Definisci gli eseguibili
add_executable(optical_head_converted src/optical_head_converted.cpp)

# Includi le directory con i file generati e header pubblici
foreach(target IN ITEMS optical_head_converted)
# foreach(target IN ITEMS optical_head_converted)
  target_include_directories(${target} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>
  )
  target_compile_features(${target} PUBLIC cxx_std_17)
  ament_target_dependencies(${target} rclcpp std_msgs)
endforeach()

# Linka le librerie generate dai messaggi
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

foreach(target IN ITEMS optical_head_converted)
# foreach(target IN ITEMS optical_head_converted)
  target_link_libraries(${target} ${cpp_typesupport_target})
endforeach()

# target_link_libraries(optical_head_node ${cpp_typesupport_target})

# Installazione messaggi e binari
install(DIRECTORY msg
  DESTINATION share/${PROJECT_NAME}/msg
)

install(TARGETS
  optical_head_converted
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
