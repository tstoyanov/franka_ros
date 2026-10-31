// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_example_controllers/elbow_example_controller.h>

#include <cmath>
#include <string>

#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

#include <franka_example_controllers/example_controllers_common.h>

namespace franka_example_controllers {

CallbackReturn ElbowExampleController::on_init() {
  auto_declare("arm_id", std::string());
  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration ElbowExampleController::command_interface_configuration()
    const {
  return {controller_interface::interface_configuration_type::INDIVIDUAL,
          cartesianPoseCommandInterfaces(arm_id_)};
}

controller_interface::InterfaceConfiguration ElbowExampleController::state_interface_configuration()
    const {
  return {controller_interface::interface_configuration_type::INDIVIDUAL,
          {frankaStateInterface(arm_id_)}};
}

CallbackReturn ElbowExampleController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  arm_id_ = get_node()->get_parameter("arm_id").as_string();
  return CallbackReturn::SUCCESS;
}

CallbackReturn ElbowExampleController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  // The state interfaces are only loaned to the controller at activation time.
  const auto& state = robotStateFromInterface(state_interfaces_[0]);
  std::array<double, 7> q_start{{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
  for (size_t i = 0; i < q_start.size(); i++) {
    if (std::abs(state.q_d[i] - q_start[i]) > 0.1) {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "ElbowExampleController: Robot is not in the expected starting position for "
                   "running this example. Run move_to_start.launch.py first.");
      return CallbackReturn::ERROR;
    }
  }
  initial_pose_ = state.O_T_EE_d;
  initial_elbow_ = state.elbow_d;
  elapsed_time_ = rclcpp::Duration(0, 0);
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type ElbowExampleController::update(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& period) {
  elapsed_time_ = elapsed_time_ + period;

  double angle = M_PI / 10.0 * (1.0 - std::cos(M_PI / 5.0 * elapsed_time_.seconds()));
  auto elbow = initial_elbow_;
  elbow[0] += angle;

  setCartesianPoseCommand(command_interfaces_, initial_pose_, elbow);
  return controller_interface::return_type::OK;
}

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::ElbowExampleController,
                       controller_interface::ControllerInterface)
