cmake_minimum_required(VERSION 3.5)
project(vive_ros2)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)


# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)


rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/VivePathCapture.srv"
  "srv/VivePathReset.srv"
  DEPENDENCIES nav_msgs # Add packages that above messages depend on
)

install(PROGRAMS
        scripts/vive_tracker_node.py
        scripts/vive_tracker_client.py
        scripts/vive_controller_node.py
        scripts/vive_controller_client.py
        DESTINATION lib/${PROJECT_NAME}
        )

add_executable(vive_capture_service_server src/vive_capture_service_server.cpp)
add_executable(vive_capture_service_listener src/vive_capture_service_listener.cpp)

ament_target_dependencies(
    vive_capture_service_server
    geometry_msgs
    visualization_msgs
    nav_msgs
    rclcpp
    tf2
    tf2_ros
)
ament_target_dependencies(
    vive_capture_service_listener
    sensor_msgs
    rclcpp
    geometry_msgs
    nav_msgs
)

rosidl_target_interfaces(vive_capture_service_server
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

rosidl_target_interfaces(vive_capture_service_listener
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS
  vive_capture_service_server
  vive_capture_service_listener
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
