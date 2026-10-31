cmake_minimum_required(VERSION 3.8)
project(franka_visualization)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Franka 0.7.0 REQUIRED)

add_executable(gripper_joint_state_publisher src/gripper_joint_state_publisher.cpp)
target_link_libraries(gripper_joint_state_publisher PUBLIC Franka::Franka)
ament_target_dependencies(gripper_joint_state_publisher rclcpp sensor_msgs)

add_executable(robot_joint_state_publisher src/robot_joint_state_publisher.cpp)
target_link_libraries(robot_joint_state_publisher PUBLIC Franka::Franka)
ament_target_dependencies(robot_joint_state_publisher rclcpp sensor_msgs)

install(TARGETS gripper_joint_state_publisher robot_joint_state_publisher
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
