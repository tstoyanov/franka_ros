// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <array>

#include <franka_hw/franka_state_interface.h>

namespace franka_hw {

constexpr char kFrankaPoseCartesianInterface[] = "franka_hw/FrankaPoseCartesianInterface";
constexpr char kFrankaVelocityCartesianInterface[] = "franka_hw/FrankaVelocityCartesianInterface";

/**
 * Handle to read and command a Cartesian pose.
 *
 * In ros2_control the pose command is exported as 16 command interfaces
 * `<arm_id>/franka_hw/FrankaPoseCartesianInterface/O_T_EE_<ij>` plus a 2-dof elbow command.
 */
class FrankaCartesianPoseHandle : public FrankaStateHandle {
 public:
  FrankaCartesianPoseHandle() = delete;

  /**
   * Creates an instance of a FrankaCartesianPoseHandle.
   *
   * @param[in] franka_state_handle Robot state handle.
   * @param[in] command A reference to the Cartesian pose command wrapped by this handle.
   * @param[in] elbow A reference to the elbow wrapped by this handle.
   */
  FrankaCartesianPoseHandle(const FrankaStateHandle& franka_state_handle,
                            std::array<double, 16>& command,
                            std::array<double, 2>& elbow)
      : FrankaStateHandle(franka_state_handle), command_(&command), elbow_(&elbow) {}

  /**
   * Sets the given command.
   *
   * @param[in] command Command to set.
   */
  void setCommand(const std::array<double, 16>& command) noexcept {
    *command_ = command;
    *elbow_ = {};
  }

  /**
   * Sets the given command.
   *
   * @param[in] command Command to set.
   * @param[in] elbow Elbow to set.
   */
  void setCommand(const std::array<double, 16>& command,
                  const std::array<double, 2>& elbow) noexcept {
    *command_ = command;
    *elbow_ = elbow;
  }

  /**
   * Gets the current command.
   *
   * @return Current command.
   */
  const std::array<double, 16>& getCommand() const noexcept { return *command_; }

  /**
   * Gets the current elbow command.
   *
   * @return Current elbow command.
   */
  const std::array<double, 2>& getElbow() const noexcept { return *elbow_; }

 private:
  std::array<double, 16>* command_;
  std::array<double, 2>* elbow_;
};

/**
 * Handle to read and command a Cartesian velocity.
 *
 * In ros2_control the velocity command is exported as 6 command interfaces
 * `<arm_id>/franka_hw/FrankaVelocityCartesianInterface/O_dP_EE_<i>` plus a 2-dof elbow command.
 */
class FrankaCartesianVelocityHandle : public FrankaStateHandle {
 public:
  FrankaCartesianVelocityHandle() = delete;

  /**
   * @param[in] franka_state_handle Robot state handle.
   * @param[in] command A reference to the Cartesian velocity command wrapped by this handle.
   * @param[in] elbow A reference to the elbow wrapped by this handle.
   */
  FrankaCartesianVelocityHandle(const FrankaStateHandle& franka_state_handle,
                                std::array<double, 6>& command,
                                std::array<double, 2>& elbow)
      : FrankaStateHandle(franka_state_handle), command_(&command), elbow_(&elbow) {}

  /**
   * Sets the given command.
   *
   * @param[in] command Command to set.
   */
  void setCommand(const std::array<double, 6>& command) noexcept {
    *command_ = command;
    *elbow_ = {};
  }

  /**
   * Sets the given command.
   *
   * @param[in] command Command to set.
   * @param[in] elbow Elbow to set.
   */
  void setCommand(const std::array<double, 6>& command,
                  const std::array<double, 2>& elbow) noexcept {
    *command_ = command;
    *elbow_ = elbow;
  }

  /**
   * Gets the current command.
   *
   * @return Current command.
   */
  const std::array<double, 6>& getCommand() const noexcept { return *command_; }

  /**
   * Gets the current elbow command.
   *
   * @return Current elbow command.
   */
  const std::array<double, 2>& getElbow() const noexcept { return *elbow_; }

 private:
  std::array<double, 6>* command_;
  std::array<double, 2>* elbow_;
};

}  // namespace franka_hw
