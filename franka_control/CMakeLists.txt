cmake_minimum_required(VERSION 3.8)
project(franka_control)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(controller_manager REQUIRED)
find_package(franka_hw REQUIRED)
find_package(franka_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(Franka 0.7.0 REQUIRED)

add_library(franka_state_controller SHARED
  src/franka_state_controller.cpp
)
target_link_libraries(franka_state_controller PUBLIC Franka::Franka franka_hw)
target_include_directories(franka_state_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(franka_state_controller
  controller_interface
  franka_hw
  franka_msgs
  geometry_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  sensor_msgs
  tf2
  tf2_geometry_msgs
  tf2_msgs
)

add_executable(franka_control_node src/franka_control_node.cpp)
ament_target_dependencies(franka_control_node controller_manager rclcpp)

pluginlib_export_plugin_description_file(controller_interface franka_controller_plugins.xml)

install(TARGETS franka_state_controller franka_control_node
  EXPORT export_franka_control
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(franka_state_controller)
ament_export_dependencies(controller_interface franka_hw franka_msgs geometry_msgs pluginlib rclcpp realtime_tools sensor_msgs tf2 tf2_msgs)
ament_export_targets(export_franka_control HAS_LIBRARY_TARGET)

ament_package()
