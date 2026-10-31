// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <controller_interface/controller_interface.hpp>
#include <franka_hw/franka_state_interface.h>
#include <franka_hw/trigger_rate.h>
#include <franka_msgs/msg/franka_state.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <realtime_tools/realtime_publisher.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

namespace franka_control {

/**
 * Controller to publish the robot state as ROS topics.
 */
class FrankaStateController : public controller_interface::ControllerInterface {
 public:
  /**
   * Creates an instance of a FrankaStateController.
   */
  FrankaStateController() = default;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::return_type update(const rclcpp::Time& time,
                                           const rclcpp::Duration& period) override;
  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

 private:
  void publishFrankaStates(const rclcpp::Time& time);
  void publishJointStates(const rclcpp::Time& time);
  void publishTransforms(const rclcpp::Time& time);
  void publishExternalWrench(const rclcpp::Time& time);

  std::string arm_id_;

  realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage> publisher_transforms_;
  realtime_tools::RealtimePublisher<franka_msgs::msg::FrankaState> publisher_franka_states_;
  realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState> publisher_joint_states_;
  realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState> publisher_joint_states_desired_;
  realtime_tools::RealtimePublisher<geometry_msgs::msg::WrenchStamped> publisher_external_wrench_;
  franka_hw::TriggerRate trigger_publish_;
  franka::RobotState robot_state_;
  uint64_t sequence_number_ = 0;
  std::vector<std::string> joint_names_;
};

}  // namespace franka_control
