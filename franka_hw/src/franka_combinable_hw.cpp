// Copyright (c) 2019 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_hw/franka_combinable_hw.h>

#include <functional>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <franka_hw/services.h>

namespace franka_hw {

CallbackReturn FrankaCombinableHW::on_init(const hardware_interface::HardwareInfo& info) {
  if (FrankaHW::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type FrankaCombinableHW::read(const rclcpp::Time& time,
                                                          const rclcpp::Duration& period) {
  return FrankaHW::read(time, period);
}

hardware_interface::return_type FrankaCombinableHW::write(const rclcpp::Time& time,
                                                           const rclcpp::Duration& period) {
  return FrankaHW::write(time, period);
}

bool FrankaCombinableHW::setRunFunction(const ControlMode& requested_control_mode,
                                        const bool limit_rate,
                                        const double cutoff_frequency,
                                        const franka::ControllerMode /*internal_controller*/) {
  using Callback = std::function<bool(const franka::RobotState&, franka::Duration)>;

  if (requested_control_mode == ControlMode::None) {
    return true;
  }
  if (requested_control_mode == ControlMode::JointTorque) {
    run_function_ = [this, limit_rate, cutoff_frequency](franka::Robot& robot,
                                                         Callback /*callback*/) {
      robot.control(std::bind(&FrankaCombinableHW::libfrankaUpdateCallback<franka::Torques>, this,
                              std::cref(effort_joint_command_libfranka_), std::placeholders::_1,
                              std::placeholders::_2),
                    limit_rate, cutoff_frequency);
    };
    return true;
  }

  RCLCPP_ERROR(get_node()->get_logger(),
               "FrankaCombinableHW: No valid control mode selected; cannot set run function.");
  return false;
}

}  // namespace franka_hw

PLUGINLIB_EXPORT_CLASS(franka_hw::FrankaCombinableHW, hardware_interface::SystemInterface)
