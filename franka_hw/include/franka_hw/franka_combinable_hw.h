// Copyright (c) 2019 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <functional>
#include <mutex>
#include <stdexcept>
#include <string>

#include <franka/exception.h>
#include <franka/robot_state.h>

#include <franka_hw/franka_hw.h>

namespace franka_hw {

/**
 * A hardware class for a Panda robot based on the ros2_control framework.
 * This class is ready to be combined with other hardware classes e.g. to
 * control multiple robots from a single controller.
 * Note: This class allows for torque (effort) control only due to the lack of synchronization
 * between master controllers of different robots.
 */
class FrankaCombinableHW : public FrankaHW {
 public:
  /**
   * Creates an instance of FrankaCombinableHW.
   */
  FrankaCombinableHW() = default;

  CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;

  /**
   * Reads data from the franka robot.
   */
  hardware_interface::return_type read(const rclcpp::Time& time,
                                       const rclcpp::Duration& period) override;

  /**
   * Writes data to the franka robot.
   */
  hardware_interface::return_type write(const rclcpp::Time& time,
                                        const rclcpp::Duration& period) override;

 private:
  template <typename T>
  T libfrankaUpdateCallback(const T& command,
                            const franka::RobotState& robot_state,
                            franka::Duration /*time_step*/) {
    if (commandHasNaN(command)) {
      std::string error_message = "FrankaCombinableHW: Got NaN value in command!";
      RCLCPP_FATAL(rclcpp::get_logger("FrankaCombinableHW"), "%s", error_message.c_str());
      throw std::invalid_argument(error_message);
    }
    checkJointLimits();
    {
      std::lock_guard<std::mutex> state_lock(libfranka_state_mutex_);
      robot_state_libfranka_ = robot_state;
    }

    std::lock_guard<std::mutex> command_lock(libfranka_cmd_mutex_);
    T current_cmd = command;
    if (has_error_ || !controller_active_) {
      return franka::MotionFinished(current_cmd);
    }
    return current_cmd;
  }

  bool setRunFunction(const ControlMode& requested_control_mode,
                      bool limit_rate,
                      double cutoff_frequency,
                      franka::ControllerMode internal_controller) override;
};

}  // namespace franka_hw
