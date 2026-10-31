// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_example_controllers/cartesian_velocity_example_controller.h>

#include <array>
#include <cmath>
#include <string>

#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

#include <franka_example_controllers/example_controllers_common.h>

namespace franka_example_controllers {

CallbackReturn CartesianVelocityExampleController::on_init() {
  auto_declare("arm_id", std::string());
  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
CartesianVelocityExampleController::command_interface_configuration() const {
  return {controller_interface::interface_configuration_type::INDIVIDUAL,
          cartesianVelocityCommandInterfaces(arm_id_)};
}

controller_interface::InterfaceConfiguration
CartesianVelocityExampleController::state_interface_configuration() const {
  return {controller_interface::interface_configuration_type::INDIVIDUAL,
          {frankaStateInterface(arm_id_)}};
}

CallbackReturn CartesianVelocityExampleController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  arm_id_ = get_node()->get_parameter("arm_id").as_string();
  const auto& state = robotStateFromInterface(state_interfaces_[0]);
  std::array<double, 7> q_start{{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
  for (size_t i = 0; i < q_start.size(); i++) {
    if (std::abs(state.q_d[i] - q_start[i]) > 0.1) {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "CartesianVelocityExampleController: Robot is not in the expected starting "
                   "position for running this example. Run move_to_start.launch.py first.");
      return CallbackReturn::ERROR;
    }
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn CartesianVelocityExampleController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  elapsed_time_ = rclcpp::Duration(0, 0);
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type CartesianVelocityExampleController::update(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& period) {
  elapsed_time_ = elapsed_time_ + period;

  double time_max = 4.0;
  double v_max = 0.05;
  double angle = M_PI / 4.0;
  double cycle = std::floor(std::pow(-1.0, (elapsed_time_.seconds() -
                                             std::fmod(elapsed_time_.seconds(), time_max)) /
                                            time_max));
  double v = cycle * v_max / 2.0 * (1.0 - std::cos(2.0 * M_PI / time_max * elapsed_time_.seconds()));
  double v_x = std::cos(angle) * v;
  double v_z = -std::sin(angle) * v;
  std::array<double, 6> command = {{v_x, 0.0, v_z, 0.0, 0.0, 0.0}};
  setCartesianVelocityCommand(command_interfaces_, command);
  return controller_interface::return_type::OK;
}

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::CartesianVelocityExampleController,
                       controller_interface::ControllerInterface)
