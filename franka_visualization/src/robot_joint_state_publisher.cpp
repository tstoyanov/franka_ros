// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <memory>
#include <string>
#include <vector>

#include <franka/exception.h>
#include <franka/robot.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("robot_joint_state_publisher");
  node->declare_parameter("joint_names", std::vector<std::string>());
  node->declare_parameter("robot_ip", std::string());
  node->declare_parameter("publish_rate", 30.0);

  std::vector<std::string> joint_names = node->get_parameter("joint_names").as_string_array();
  std::string robot_ip = node->get_parameter("robot_ip").as_string();
  double publish_rate = node->get_parameter("publish_rate").as_double();

  rclcpp::Rate rate(publish_rate);

  sensor_msgs::msg::JointState states;
  states.effort.resize(joint_names.size());
  states.name.resize(joint_names.size());
  states.position.resize(joint_names.size());
  states.velocity.resize(joint_names.size());

  for (size_t i = 0; i < joint_names.size(); i++) {
    states.name[i] = joint_names[i];
  }

  auto publisher = node->create_publisher<sensor_msgs::msg::JointState>("joint_states", 1);

  try {
    franka::Robot robot(robot_ip);

    robot.read([&](const franka::RobotState& robot_state) {
      states.header.stamp = node->now();
      for (size_t i = 0; i < joint_names.size(); i++) {
        states.position[i] = robot_state.q[i];
        states.velocity[i] = robot_state.dq[i];
        states.effort[i] = robot_state.tau_J[i];
      }
      publisher->publish(states);
      rclcpp::spin_some(node);
      rate.sleep();
      return rclcpp::ok();
    });

  } catch (const franka::Exception& e) {
    RCLCPP_ERROR_STREAM(node->get_logger(), "Exception: " << e.what());
    return -1;
  }

  rclcpp::shutdown();
  return 0;
}
