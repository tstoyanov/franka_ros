cmake_minimum_required(VERSION 3.8)
project(franka_gripper)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(actionlib_msgs REQUIRED)
find_package(Franka 0.7.0 REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  action/Grasp.action
  action/Homing.action
  action/Stop.action
  action/Move.action
  msg/GraspEpsilon.msg
  DEPENDENCIES actionlib_msgs
)

add_library(franka_gripper SHARED src/franka_gripper.cpp)
target_link_libraries(franka_gripper PUBLIC Franka::Franka)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(franka_gripper PUBLIC ${cpp_typesupport_target})
target_include_directories(franka_gripper PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(franka_gripper rclcpp rclcpp_action control_msgs)

add_executable(franka_gripper_node src/franka_gripper_node.cpp)
target_link_libraries(franka_gripper_node PRIVATE franka_gripper ${cpp_typesupport_target})
ament_target_dependencies(franka_gripper_node rclcpp rclcpp_action sensor_msgs control_msgs)

install(TARGETS franka_gripper franka_gripper_node
  EXPORT export_franka_gripper
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(franka_gripper)
ament_export_dependencies(rclcpp rclcpp_action control_msgs rosidl_default_runtime)
ament_export_targets(export_franka_gripper HAS_LIBRARY_TARGET)

ament_package()
