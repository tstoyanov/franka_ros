cmake_minimum_required(VERSION 3.8)
project(franka_hw)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(franka_msgs REQUIRED)
find_package(Franka 0.7.0 REQUIRED)

add_library(franka_control_services SHARED
  src/services.cpp
)
target_link_libraries(franka_control_services Franka::Franka)
target_include_directories(franka_control_services PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(franka_control_services
  rclcpp
  franka_msgs
)

add_library(franka_hw SHARED
  src/control_mode.cpp
  src/franka_hw.cpp
  src/franka_combinable_hw.cpp
  src/trigger_rate.cpp
)
target_link_libraries(franka_hw Franka::Franka franka_control_services)
target_include_directories(franka_hw PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(franka_hw
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  std_msgs
  franka_msgs
)

pluginlib_export_plugin_description_file(hardware_interface franka_combinable_hw_plugin.xml)

install(TARGETS franka_hw franka_control_services
  EXPORT export_franka_hw
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(franka_hw franka_control_services)
ament_export_dependencies(hardware_interface pluginlib rclcpp rclcpp_action rclcpp_lifecycle std_msgs franka_msgs)
ament_export_targets(export_franka_hw HAS_LIBRARY_TARGET)

if(BUILD_TESTING)
  add_subdirectory(test)
endif()

ament_package()
