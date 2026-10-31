// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_example_controllers/model_example_controller.h>

#include <sstream>
#include <string>

#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

#include <franka_example_controllers/example_controllers_common.h>

namespace franka_example_controllers {

CallbackReturn ModelExampleController::on_init() {
  auto_declare("arm_id", std::string());
  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration ModelExampleController::command_interface_configuration()
    const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::NONE;
  return config;
}

controller_interface::InterfaceConfiguration ModelExampleController::state_interface_configuration()
    const {
  return {controller_interface::interface_configuration_type::INDIVIDUAL,
          {frankaStateInterface(arm_id_), frankaModelInterface(arm_id_)}};
}

CallbackReturn ModelExampleController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  arm_id_ = get_node()->get_parameter("arm_id").as_string();
  return CallbackReturn::SUCCESS;
}

CallbackReturn ModelExampleController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  // The state interfaces (and thus the robot state pointer) are only loaned to the
  // controller at activation time, so the model handle is created here.
  // state_interfaces_[0] holds the franka::RobotState pointer, state_interfaces_[1] the model.
  auto* robot_state = decodePointer<franka::RobotState>(state_interfaces_[0].get_value());
  auto* model = decodePointer<franka::Model>(state_interfaces_[1].get_value());
  model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(arm_id_ + "_model", *model,
                                                                  *robot_state);
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type ModelExampleController::update(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  if (rate_trigger_()) {
    std::array<double, 49> mass = model_handle_->getMass();
    std::array<double, 7> coriolis = model_handle_->getCoriolis();
    std::array<double, 7> gravity = model_handle_->getGravity();
    std::array<double, 16> pose = model_handle_->getPose(franka::Frame::kJoint4);
    std::array<double, 42> joint4_body_jacobian =
        model_handle_->getBodyJacobian(franka::Frame::kJoint4);
    std::array<double, 42> endeffector_zero_jacobian =
        model_handle_->getZeroJacobian(franka::Frame::kEndEffector);

    auto logger = get_node()->get_logger();
    RCLCPP_INFO(logger, "--------------------------------------------------");
    std::ostringstream mass_str, coriolis_str, gravity_str, pose_str, j1_str, j2_str;
    for (auto v : mass) mass_str << v << " ";
    for (auto v : coriolis) coriolis_str << v << " ";
    for (auto v : gravity) gravity_str << v << " ";
    for (auto v : pose) pose_str << v << " ";
    for (auto v : joint4_body_jacobian) j1_str << v << " ";
    for (auto v : endeffector_zero_jacobian) j2_str << v << " ";
    RCLCPP_INFO_STREAM(logger, "mass :" << mass_str.str());
    RCLCPP_INFO_STREAM(logger, "coriolis: " << coriolis_str.str());
    RCLCPP_INFO_STREAM(logger, "gravity :" << gravity_str.str());
    RCLCPP_INFO_STREAM(logger, "joint_pose :" << pose_str.str());
    RCLCPP_INFO_STREAM(logger, "joint4_body_jacobian :" << j1_str.str());
    RCLCPP_INFO_STREAM(logger, "joint_zero_jacobian :" << j2_str.str());
  }
  return controller_interface::return_type::OK;
}

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::ModelExampleController,
                       controller_interface::ControllerInterface)
