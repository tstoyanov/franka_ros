// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_example_controllers/cartesian_pose_example_controller.h>

#include <cmath>
#include <string>

#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

#include <franka_example_controllers/example_controllers_common.h>

namespace franka_example_controllers {

CallbackReturn CartesianPoseExampleController::on_init() {
  auto_declare("arm_id", std::string());
  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
CartesianPoseExampleController::command_interface_configuration() const {
  return {controller_interface::interface_configuration_type::INDIVIDUAL,
          cartesianPoseCommandInterfaces(arm_id_)};
}

controller_interface::InterfaceConfiguration
CartesianPoseExampleController::state_interface_configuration() const {
  return {controller_interface::interface_configuration_type::INDIVIDUAL,
          {frankaStateInterface(arm_id_)}};
}

CallbackReturn CartesianPoseExampleController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  arm_id_ = get_node()->get_parameter("arm_id").as_string();
  return CallbackReturn::SUCCESS;
}

CallbackReturn CartesianPoseExampleController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  // The state interfaces are only loaned to the controller at activation time.
  const auto& state = robotStateFromInterface(state_interfaces_[0]);
  std::array<double, 7> q_start{{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
  for (size_t i = 0; i < q_start.size(); i++) {
    if (std::abs(state.q_d[i] - q_start[i]) > 0.1) {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "CartesianPoseExampleController: Robot is not in the expected starting position "
                   "for running this example. Run move_to_start.launch.py first.");
      return CallbackReturn::ERROR;
    }
  }
  initial_pose_ = state.O_T_EE_d;
  elapsed_time_ = rclcpp::Duration(0, 0);
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type CartesianPoseExampleController::update(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& period) {
  elapsed_time_ = elapsed_time_ + period;

  double radius = 0.3;
  double angle = M_PI / 4 * (1 - std::cos(M_PI / 5.0 * elapsed_time_.seconds()));
  double delta_x = radius * std::sin(angle);
  double delta_z = radius * (std::cos(angle) - 1);
  std::array<double, 16> new_pose = initial_pose_;
  new_pose[12] -= delta_x;
  new_pose[14] -= delta_z;
  setCartesianPoseCommand(command_interfaces_, new_pose);
  return controller_interface::return_type::OK;
}

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::CartesianPoseExampleController,
                       controller_interface::ControllerInterface)
