// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_example_controllers/joint_impedance_example_controller.h>

#include <cmath>
#include <memory>

#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

#include <franka/robot_state.h>

#include <franka_example_controllers/example_controllers_common.h>

namespace franka_example_controllers {

CallbackReturn JointImpedanceExampleController::on_init() {
  auto_declare("arm_id", std::string());
  auto_declare("joint_names", std::vector<std::string>());
  auto_declare("radius", 0.1);
  auto_declare("vel_max", 0.05);
  auto_declare("acceleration_time", 2.0);
  auto_declare("k_gains", std::vector<double>(7, 0.0));
  auto_declare("d_gains", std::vector<double>(7, 0.0));
  auto_declare("publish_rate", 30.0);
  auto_declare("coriolis_factor", 1.0);
  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
JointImpedanceExampleController::command_interface_configuration() const {
  // The controller commands joint torques and additionally claims the cartesian pose
  // command interfaces to feed libfranka's internal motion generator (the ROS 1 version
  // used the FrankaCartesianPoseHandle for the same purpose).
  std::vector<std::string> interfaces =
      jointCommandInterfaces(joint_names_, hardware_interface::HW_IF_EFFORT);
  for (const auto& iface : cartesianPoseCommandInterfaces(arm_id_)) {
    interfaces.push_back(iface);
  }
  return {controller_interface::interface_configuration_type::INDIVIDUAL, interfaces};
}

controller_interface::InterfaceConfiguration
JointImpedanceExampleController::state_interface_configuration() const {
  return {controller_interface::interface_configuration_type::INDIVIDUAL,
          {frankaStateInterface(arm_id_), frankaModelInterface(arm_id_)}};
}

CallbackReturn JointImpedanceExampleController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  auto node = get_node();
  arm_id_ = node->get_parameter("arm_id").as_string();
  radius_ = node->get_parameter("radius").as_double();
  if (std::fabs(radius_) < 0.005) {
    RCLCPP_INFO(node->get_logger(), "JointImpedanceExampleController: Set radius to small, defaulting to: 0.1");
    radius_ = 0.1;
  }
  vel_max_ = node->get_parameter("vel_max").as_double();
  acceleration_time_ = node->get_parameter("acceleration_time").as_double();
  joint_names_ = node->get_parameter("joint_names").as_string_array();
  if (joint_names_.size() != 7) {
    RCLCPP_ERROR(node->get_logger(),
                 "JointImpedanceExampleController: Invalid or no joint_names parameters provided, "
                 "aborting controller init!");
    return CallbackReturn::ERROR;
  }
  k_gains_ = node->get_parameter("k_gains").as_double_array();
  d_gains_ = node->get_parameter("d_gains").as_double_array();
  if (k_gains_.size() != 7 || d_gains_.size() != 7) {
    RCLCPP_ERROR(node->get_logger(),
                 "JointImpedanceExampleController: Invalid or no k_gains/d_gains parameters "
                 "provided, aborting controller init!");
    return CallbackReturn::ERROR;
  }
  rate_trigger_ = franka_hw::TriggerRate(node->get_parameter("publish_rate").as_double());
  coriolis_factor_ = node->get_parameter("coriolis_factor").as_double();

    torques_publisher_ =
      std::make_shared<realtime_tools::RealtimePublisher<franka_example_controllers::msg::JointTorqueComparison>>(
          node->create_publisher<franka_example_controllers::msg::JointTorqueComparison>(
              "torque_comparison", 1));
  std::fill(dq_filtered_.begin(), dq_filtered_.end(), 0);
  return CallbackReturn::SUCCESS;
}

CallbackReturn JointImpedanceExampleController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  // The state interfaces (and thus the robot state pointer) are only loaned to the
  // controller at activation time, so the model handle is created here.
  auto* robot_state = decodePointer<franka::RobotState>(state_interfaces_[0].get_value());
  auto* model = decodePointer<franka::Model>(state_interfaces_[1].get_value());
  model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(arm_id_ + "_model", *model,
                                                                  *robot_state);
  auto& robot_state_ref = robotStateFromInterface(state_interfaces_[0]);
  initial_pose_ = robot_state_ref.O_T_EE_d;
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type JointImpedanceExampleController::update(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& period) {
  if (vel_current_ < vel_max_) {
    vel_current_ += period.seconds() * std::fabs(vel_max_ / acceleration_time_);
  }
  vel_current_ = std::fmin(vel_current_, vel_max_);

  angle_ += period.seconds() * vel_current_ / std::fabs(radius_);
  if (angle_ > 2 * M_PI) {
    angle_ -= 2 * M_PI;
  }

  double delta_y = radius_ * (1 - std::cos(angle_));
  double delta_z = radius_ * std::sin(angle_);

  std::array<double, 16> pose_desired = initial_pose_;
  pose_desired[13] += delta_y;
  pose_desired[14] += delta_z;
  // command_interfaces_[0..6] are the joint effort interfaces, the cartesian pose
  // command interfaces follow at offset 7.
  setCartesianPoseOffsetCommand(command_interfaces_, 7, pose_desired);

  auto& robot_state = robotStateFromInterface(state_interfaces_[0]);
  std::array<double, 7> coriolis = model_handle_->getCoriolis();
  std::array<double, 7> gravity = model_handle_->getGravity();

  double alpha = 0.99;
  for (size_t i = 0; i < 7; i++) {
    dq_filtered_[i] = (1 - alpha) * dq_filtered_[i] + alpha * robot_state.dq[i];
  }

  std::array<double, 7> tau_d_calculated;
  for (size_t i = 0; i < 7; ++i) {
    tau_d_calculated[i] = coriolis_factor_ * coriolis[i] +
                          k_gains_[i] * (robot_state.q_d[i] - robot_state.q[i]) +
                          d_gains_[i] * (robot_state.dq_d[i] - dq_filtered_[i]);
  }

  // Maximum torque difference with a sampling rate of 1 kHz. The maximum torque rate is
  // 1000 * (1 / sampling_time).
  std::array<double, 7> tau_d_saturated = saturateTorqueRate(tau_d_calculated, robot_state.tau_J_d);

  for (size_t i = 0; i < 7; ++i) {
    command_interfaces_[i].set_value(tau_d_saturated[i]);
  }

  if (rate_trigger_() && torques_publisher_->trylock()) {
    std::array<double, 7> tau_j = robot_state.tau_J;
    std::array<double, 7> tau_error;
    double error_rms(0.0);
    for (size_t i = 0; i < 7; ++i) {
      tau_error[i] = last_tau_d_[i] - tau_j[i];
      error_rms += std::sqrt(std::pow(tau_error[i], 2.0)) / 7.0;
    }
    torques_publisher_->msg_.root_mean_square_error = error_rms;
    for (size_t i = 0; i < 7; ++i) {
      torques_publisher_->msg_.tau_commanded[i] = last_tau_d_[i];
      torques_publisher_->msg_.tau_error[i] = tau_error[i];
      torques_publisher_->msg_.tau_measured[i] = tau_j[i];
    }
    torques_publisher_->unlockAndPublish();
  }

  for (size_t i = 0; i < 7; ++i) {
    last_tau_d_[i] = tau_d_saturated[i] + gravity[i];
  }
  return controller_interface::return_type::OK;
}

std::array<double, 7> JointImpedanceExampleController::saturateTorqueRate(
    const std::array<double, 7>& tau_d_calculated,
    const std::array<double, 7>& tau_j_d) {  // NOLINT (readability-identifier-naming)
  std::array<double, 7> tau_d_saturated{};
  for (size_t i = 0; i < 7; i++) {
    double difference = tau_d_calculated[i] - tau_j_d[i];
    tau_d_saturated[i] = tau_j_d[i] + std::max(std::min(difference, kDeltaTauMax), -kDeltaTauMax);
  }
  return tau_d_saturated;
}

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::JointImpedanceExampleController,
                       controller_interface::ControllerInterface)
