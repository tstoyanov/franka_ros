cmake_minimum_required(VERSION 3.8)
project(franka_example_controllers)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(franka_hw REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Franka 0.7.0 REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/JointTorqueComparison.msg
)

add_library(franka_example_controllers_lib SHARED
  src/elbow_example_controller.cpp
  src/cartesian_pose_example_controller.cpp
  src/cartesian_velocity_example_controller.cpp
  src/joint_position_example_controller.cpp
  src/joint_velocity_example_controller.cpp
  src/model_example_controller.cpp
  src/joint_impedance_example_controller.cpp
  src/cartesian_impedance_example_controller.cpp
  src/cartesian_impedance_mod_controller.cpp
  src/force_example_controller.cpp
  src/dual_arm_cartesian_impedance_example_controller.cpp
)
target_link_libraries(franka_example_controllers_lib Franka::Franka Eigen3::Eigen)
ament_target_dependencies(franka_example_controllers_lib franka_hw)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(franka_example_controllers_lib ${cpp_typesupport_target})
target_include_directories(franka_example_controllers_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(franka_example_controllers_lib
  controller_interface
  franka_hw
  geometry_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  tf2
  tf2_eigen
  tf2_geometry_msgs
  tf2_ros
)

pluginlib_export_plugin_description_file(controller_interface franka_example_controllers_plugin.xml)

install(TARGETS franka_example_controllers_lib
  EXPORT export_franka_example_controllers
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(franka_example_controllers_lib)
ament_export_dependencies(controller_interface franka_hw geometry_msgs pluginlib rclcpp realtime_tools tf2 tf2_eigen tf2_geometry_msgs tf2_ros rosidl_default_runtime)
ament_export_targets(export_franka_example_controllers HAS_LIBRARY_TARGET)

ament_package()
