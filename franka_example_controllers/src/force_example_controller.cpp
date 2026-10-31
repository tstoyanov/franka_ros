// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_example_controllers/force_example_controller.h>

#include <cmath>
#include <memory>

#include <franka/robot_state.h>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

#include <franka_example_controllers/example_controllers_common.h>

namespace franka_example_controllers {

CallbackReturn ForceExampleController::on_init() {
  auto_declare("arm_id", std::string());
  auto_declare("joint_names", std::vector<std::string>());
  auto_declare("desired_mass", 0.0);
  auto_declare("k_p", 0.0);
  auto_declare("k_i", 0.0);
  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration ForceExampleController::command_interface_configuration()
    const {
  return {controller_interface::interface_configuration_type::INDIVIDUAL,
          jointCommandInterfaces(joint_names_, hardware_interface::HW_IF_EFFORT)};
}

controller_interface::InterfaceConfiguration ForceExampleController::state_interface_configuration()
    const {
  return {controller_interface::interface_configuration_type::INDIVIDUAL,
          {frankaStateInterface(arm_id_), frankaModelInterface(arm_id_)}};
}

CallbackReturn ForceExampleController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  auto node = get_node();
  RCLCPP_WARN(node->get_logger(),
              "ForceExampleController: Make sure your robot's endeffector is in contact "
              "with a horizontal surface before starting the controller!");
  arm_id_ = node->get_parameter("arm_id").as_string();
  joint_names_ = node->get_parameter("joint_names").as_string_array();
  if (joint_names_.size() != 7) {
    RCLCPP_ERROR(node->get_logger(),
                 "ForceExampleController: Invalid or no joint_names parameters provided, aborting "
                 "controller init!");
    return CallbackReturn::ERROR;
  }

  auto* robot_state = decodePointer<franka::RobotState>(state_interfaces_[0].get_value());
  auto* model = decodePointer<franka::Model>(state_interfaces_[1].get_value());
  model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(arm_id_ + "_model", *model,
                                                                  *robot_state);

  target_mass_ = node->get_parameter("desired_mass").as_double();
  target_k_p_ = node->get_parameter("k_p").as_double();
  target_k_i_ = node->get_parameter("k_i").as_double();
  param_callback_handle_ = node->add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter>& parameters) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        std::lock_guard<std::mutex> lock(target_mutex_);
        for (const auto& param : parameters) {
          if (param.get_name() == "desired_mass") {
            target_mass_ = param.as_double();
          } else if (param.get_name() == "k_p") {
            target_k_p_ = param.as_double();
          } else if (param.get_name() == "k_i") {
            target_k_i_ = param.as_double();
          }
        }
        return result;
      });
  return CallbackReturn::SUCCESS;
}

CallbackReturn ForceExampleController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  auto& robot_state = robotStateFromInterface(state_interfaces_[0]);
  std::array<double, 7> gravity_array = model_handle_->getGravity();
  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_measured(robot_state.tau_J.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> gravity(gravity_array.data());
  // Bias correction for the current external torque
  tau_ext_initial_ = tau_measured - gravity;
  tau_error_.setZero();
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type ForceExampleController::update(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& period) {
  auto& robot_state = robotStateFromInterface(state_interfaces_[0]);
  std::array<double, 42> jacobian_array =
      model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
  std::array<double, 7> gravity_array = model_handle_->getGravity();
  Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_measured(robot_state.tau_J.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_j_d(  // NOLINT (readability-identifier-naming)
      robot_state.tau_J_d.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> gravity(gravity_array.data());

  Eigen::VectorXd tau_d(7), desired_force_torque(6), tau_cmd(7), tau_ext(7);
  desired_force_torque.setZero();
  desired_force_torque(2) = desired_mass_ * -9.81;
  tau_ext = tau_measured - gravity - tau_ext_initial_;
  tau_d << jacobian.transpose() * desired_force_torque;
  tau_error_ = tau_error_ + period.seconds() * (tau_d - tau_ext);
  // FF + PI control (PI gains are initially all 0)
  tau_cmd = tau_d + k_p_ * (tau_d - tau_ext) + k_i_ * tau_error_;
  tau_cmd << saturateTorqueRate(tau_cmd, tau_j_d);

  for (size_t i = 0; i < 7; ++i) {
    command_interfaces_[i].set_value(tau_cmd(i));
  }

  // Update signals changed online through parameters
  {
    std::lock_guard<std::mutex> lock(target_mutex_);
    desired_mass_ = filter_gain_ * target_mass_ + (1 - filter_gain_) * desired_mass_;
    k_p_ = filter_gain_ * target_k_p_ + (1 - filter_gain_) * k_p_;
    k_i_ = filter_gain_ * target_k_i_ + (1 - filter_gain_) * k_i_;
  }
  return controller_interface::return_type::OK;
}

Eigen::Matrix<double, 7, 1> ForceExampleController::saturateTorqueRate(
    const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
    const Eigen::Matrix<double, 7, 1>& tau_j_d) {  // NOLINT (readability-identifier-naming)
  Eigen::Matrix<double, 7, 1> tau_d_saturated{};
  for (size_t i = 0; i < 7; i++) {
    double difference = tau_d_calculated[i] - tau_j_d[i];
    tau_d_saturated[i] = tau_j_d[i] + std::max(std::min(difference, kDeltaTauMax), -kDeltaTauMax);
  }
  return tau_d_saturated;
}

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::ForceExampleController,
                       controller_interface::ControllerInterface)
