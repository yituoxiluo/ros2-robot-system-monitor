cmake_minimum_required(VERSION 3.8)
project(robot_system_monitor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)

add_executable(device_driver_node src/device_driver_node.cpp)
add_executable(communication_manager_node src/communication_manager_node.cpp)
add_executable(diagnostic_monitor_node src/diagnostic_monitor_node.cpp)
add_executable(logger_node src/logger_node.cpp)

ament_target_dependencies(device_driver_node
  rclcpp
  std_msgs
  sensor_msgs
  diagnostic_msgs
)

ament_target_dependencies(communication_manager_node
  rclcpp
  std_msgs
  sensor_msgs
  diagnostic_msgs
)

ament_target_dependencies(diagnostic_monitor_node
  rclcpp
  std_msgs
  sensor_msgs
  diagnostic_msgs
)

ament_target_dependencies(logger_node
  rclcpp
  std_msgs
  sensor_msgs
)

install(TARGETS
  device_driver_node
  communication_manager_node
  diagnostic_monitor_node
  logger_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
