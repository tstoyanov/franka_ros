find_package(ament_cmake_gtest REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rclcpp REQUIRED)

ament_add_gtest(example_controllers_test
  example_controllers_test.cpp
)
target_link_libraries(example_controllers_test franka_example_controllers_lib Franka::Franka)
target_include_directories(example_controllers_test PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/../include
)
ament_target_dependencies(example_controllers_test
  controller_interface
  hardware_interface
  lifecycle_msgs
  rclcpp
  franka_hw
)
