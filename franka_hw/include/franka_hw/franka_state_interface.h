// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <string>

#include <franka/robot_state.h>

namespace franka_hw {

constexpr char kFrankaStateInterface[] = "franka_hw/FrankaStateInterface";

/**
 * Handle to read the complete state of a robot.
 *
 * In ros2_control this is exported as a custom state interface named
 * `<arm_id>/franka_hw/FrankaStateInterface/robot_state`.
 */
class FrankaStateHandle {
 public:
  FrankaStateHandle() = delete;

  /**
   * Creates an instance of a FrankaStateHandle.
   *
   * @param[in] name The name of the state handle.
   * @param[in] robot_state A reference to the robot state wrapped by this handle.
   */
  FrankaStateHandle(const std::string& name, franka::RobotState& robot_state)
      : name_(name), robot_state_(&robot_state) {}

  /**
   * Gets the name of the state handle.
   *
   * @return Name of the state handle.
   */
  const std::string& getName() const noexcept { return name_; }

  /**
   * Gets the current robot state.
   *
   * @return Current robot state.
   */
  const franka::RobotState& getRobotState() const noexcept { return *robot_state_; }

 private:
  std::string name_;
  const franka::RobotState* robot_state_;
};

}  // namespace franka_hw
