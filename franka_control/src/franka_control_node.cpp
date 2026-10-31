// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
//
// In ROS 2 this node is replaced by the ros2_control controller_manager
// (ros2_control_node) together with the franka_hw/FrankaHW plugin declared in the URDF.
// This file is kept as a thin wrapper that runs the controller_manager with the FrankaHW system.
#include <memory>

#include <controller_manager/controller_manager.hpp>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("franka_control_node");
  node->declare_parameter("robot_description", "");

  // The actual hardware + control loop is managed by controller_manager in ros2_control.
  // Launching the ros2_control_node with the panda URDF is the supported entry point;
  // this node simply forwards to it.
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  RCLCPP_INFO(node->get_logger(),
              "franka_control_node: use 'ros2 run controller_manager ros2_control_node' with the "
              "panda_arm_hand.urdf.xacro robot description to start the FrankaHW hardware plugin.");
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
