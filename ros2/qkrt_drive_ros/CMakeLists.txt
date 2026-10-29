cmake_minimum_required(VERSION 3.16)
project(qkrt_drive_ros LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(qkrt_core REQUIRED)

set(NODES sim_node drive_controller_node localization_node planner_node teleop_bridge_node)
foreach(node ${NODES})
  add_executable(${node} src/${node}.cpp)
  target_link_libraries(${node} qkrt::qkrt_core)
  ament_target_dependencies(${node} rclcpp geometry_msgs nav_msgs sensor_msgs std_msgs)
  install(TARGETS ${node} DESTINATION lib/${PROJECT_NAME})
endforeach()

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
