find_package(ament_cmake_gtest REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(rclcpp REQUIRED)

ament_add_gtest(franka_hw_test
  main.cpp
  franka_hw_interfaces_test.cpp
  franka_hw_controller_switching_test.cpp
  franka_combinable_hw_controller_switching_test.cpp
)
target_link_libraries(franka_hw_test franka_hw Franka::Franka)
target_include_directories(franka_hw_test PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/../include
)
ament_target_dependencies(franka_hw_test
  hardware_interface
  rclcpp
  franka_msgs
)
