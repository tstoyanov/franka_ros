// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_example_controllers/joint_position_example_controller.h>

#include <cmath>

#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

#include <franka_example_controllers/example_controllers_common.h>

namespace franka_example_controllers {

CallbackReturn JointPositionExampleController::on_init() {
  auto_declare("joint_names", std::vector<std::string>());
  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
JointPositionExampleController::command_interface_configuration() const {
  return {controller_interface::interface_configuration_type::INDIVIDUAL,
          jointCommandInterfaces(joint_names_, hardware_interface::HW_IF_POSITION)};
}

controller_interface::InterfaceConfiguration
JointPositionExampleController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto& joint : joint_names_) {
    config.names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
  }
  return config;
}

CallbackReturn JointPositionExampleController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  joint_names_ = get_node()->get_parameter("joint_names").as_string_array();
  if (joint_names_.size() != 7) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "JointPositionExampleController: Wrong number of joint names, got %zu instead of "
                 "7 names!",
                 joint_names_.size());
    return CallbackReturn::ERROR;
  }

  std::array<double, 7> q_start{{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
  for (size_t i = 0; i < q_start.size(); i++) {
    if (std::abs(state_interfaces_[i].get_value() - q_start[i]) > 0.1) {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "JointPositionExampleController: Robot is not in the expected starting position "
                   "for running this example. Run `ros2 launch franka_example_controllers "
                   "move_to_start.launch.py robot_ip:=<robot-ip> load_gripper:=<has-attached-gripper>` "
                   "first.");
      return CallbackReturn::ERROR;
    }
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn JointPositionExampleController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  for (size_t i = 0; i < 7; ++i) {
    initial_pose_[i] = state_interfaces_[i].get_value();
  }
  elapsed_time_ = rclcpp::Duration(0, 0);
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type JointPositionExampleController::update(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& period) {
  elapsed_time_ = elapsed_time_ + period;

  double delta_angle = M_PI / 16 * (1 - std::cos(M_PI / 5.0 * elapsed_time_.seconds())) * 0.2;
  for (size_t i = 0; i < 7; ++i) {
    if (i == 4) {
      command_interfaces_[i].set_value(initial_pose_[i] - delta_angle);
    } else {
      command_interfaces_[i].set_value(initial_pose_[i] + delta_angle);
    }
  }
  return controller_interface::return_type::OK;
}

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::JointPositionExampleController,
                       controller_interface::ControllerInterface)
