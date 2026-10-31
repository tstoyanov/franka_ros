// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_example_controllers/joint_velocity_example_controller.h>

#include <cmath>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

#include <franka_example_controllers/example_controllers_common.h>

namespace franka_example_controllers {

CallbackReturn JointVelocityExampleController::on_init() {
  auto_declare("arm_id", std::string());
  auto_declare("joint_names", std::vector<std::string>());
  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
JointVelocityExampleController::command_interface_configuration() const {
  return {controller_interface::interface_configuration_type::INDIVIDUAL,
          jointCommandInterfaces(joint_names_, hardware_interface::HW_IF_VELOCITY)};
}

controller_interface::InterfaceConfiguration
JointVelocityExampleController::state_interface_configuration() const {
  return {controller_interface::interface_configuration_type::INDIVIDUAL,
          {frankaStateInterface(arm_id_)}};
}

CallbackReturn JointVelocityExampleController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  arm_id_ = get_node()->get_parameter("arm_id").as_string();
  joint_names_ = get_node()->get_parameter("joint_names").as_string_array();
  if (joint_names_.size() != 7) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "JointVelocityExampleController: Invalid or no joint_names parameters provided!");
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn JointVelocityExampleController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  // The state interfaces are only loaned to the controller at activation time.
  const auto& state = robotStateFromInterface(state_interfaces_[0]);
  std::array<double, 7> q_start{{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
  for (size_t i = 0; i < q_start.size(); i++) {
    if (std::abs(state.q_d[i] - q_start[i]) > 0.1) {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "JointVelocityExampleController: Robot is not in the expected starting position "
                   "for running this example. Run move_to_start.launch.py first.");
      return CallbackReturn::ERROR;
    }
  }
  elapsed_time_ = rclcpp::Duration(0, 0);
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type JointVelocityExampleController::update(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& period) {
  elapsed_time_ = elapsed_time_ + period;

  double time_max = 8.0;
  double omega_max = 0.1;
  double cycle = std::floor(std::pow(-1.0, (elapsed_time_.seconds() -
                                             std::fmod(elapsed_time_.seconds(), time_max)) /
                                            time_max));
  double omega = cycle * omega_max / 2.0 *
                 (1.0 - std::cos(2.0 * M_PI / time_max * elapsed_time_.seconds()));

  for (auto& command_interface : command_interfaces_) {
    command_interface.set_value(omega);
  }
  return controller_interface::return_type::OK;
}

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::JointVelocityExampleController,
                       controller_interface::ControllerInterface)
