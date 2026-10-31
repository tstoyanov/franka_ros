// Copyright (c) 2024 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <array>
#include <cstring>
#include <string>
#include <vector>

#include <controller_interface/controller_interface.hpp>
#include <franka_hw/franka_cartesian_command_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <franka_hw/franka_model_interface.h>
#include <hardware_interface/loaned_state_interface.hpp>
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>

namespace franka_example_controllers {

// Interface names exported by franka_hw::FrankaHW.
inline std::string frankaStateInterface(const std::string& arm_id) {
  return arm_id + "/" + franka_hw::kFrankaStateInterface;
}
inline std::string frankaModelInterface(const std::string& arm_id) {
  return arm_id + "/" + franka_hw::kFrankaModelInterface;
}
inline std::vector<std::string> cartesianPoseCommandInterfaces(const std::string& arm_id) {
  static const std::array<std::string, 16> pose_names = {
      "00", "01", "02", "03", "04", "05", "06", "07",
      "08", "09", "10", "11", "12", "13", "14", "15"};
  std::vector<std::string> interfaces;
  for (const auto& n : pose_names) {
    interfaces.push_back(arm_id + "/" + franka_hw::kFrankaPoseCartesianInterface + "/O_T_EE_" + n);
  }
  interfaces.push_back(arm_id + "/" + franka_hw::kFrankaPoseCartesianInterface + "/elbow_0");
  interfaces.push_back(arm_id + "/" + franka_hw::kFrankaPoseCartesianInterface + "/elbow_1");
  return interfaces;
}
inline std::vector<std::string> cartesianVelocityCommandInterfaces(const std::string& arm_id) {
  std::vector<std::string> interfaces;
  for (size_t i = 0; i < 6; i++) {
    interfaces.push_back(arm_id + "/" + franka_hw::kFrankaVelocityCartesianInterface + "/O_dP_EE_" +
                         std::to_string(i));
  }
  interfaces.push_back(arm_id + "/" + franka_hw::kFrankaVelocityCartesianInterface + "/elbow_0");
  interfaces.push_back(arm_id + "/" + franka_hw::kFrankaVelocityCartesianInterface + "/elbow_1");
  return interfaces;
}
inline std::vector<std::string> jointCommandInterfaces(const std::vector<std::string>& joint_names,
                                                       const std::string& type) {
  std::vector<std::string> interfaces;
  for (const auto& joint : joint_names) {
    interfaces.push_back(joint + "/" + type);
  }
  return interfaces;
}


// In ROS 2 jazzy, LoanedStateInterface::get_value() returns a double. Our franka state/model
// interfaces store the raw pointer bits in that double, so decode via memcpy.
template <typename T>
inline T* decodePointer(double value) {
  T* ptr = nullptr;
  static_assert(sizeof(ptr) == sizeof(value));
  std::memcpy(&ptr, &value, sizeof(value));
  return ptr;
}

// Reads the franka::RobotState pointer exported on the franka state interface.
inline const franka::RobotState& robotStateFromInterface(
    const hardware_interface::LoanedStateInterface& state_interface) {
  return *decodePointer<franka::RobotState>(state_interface.get_value());
}

// Reads/writes the 16-dof cartesian pose command interfaces into an array.
inline std::array<double, 16> getCartesianPoseCommand(
    const std::vector<hardware_interface::LoanedCommandInterface>& command_interfaces) {
  std::array<double, 16> pose{};
  for (size_t i = 0; i < 16; i++) {
    pose[i] = command_interfaces[i].get_value();
  }
  return pose;
}
inline void setCartesianPoseCommand(
    std::vector<hardware_interface::LoanedCommandInterface>& command_interfaces,
    const std::array<double, 16>& pose,
    const std::array<double, 2>& elbow = {0.0, 0.0}) {
  for (size_t i = 0; i < 16; i++) {
    command_interfaces[i].set_value(pose[i]);
  }
  command_interfaces[16].set_value(elbow[0]);
  command_interfaces[17].set_value(elbow[1]);
}
inline std::array<double, 6> getCartesianVelocityCommand(
    const std::vector<hardware_interface::LoanedCommandInterface>& command_interfaces) {
  std::array<double, 6> vel{};
  for (size_t i = 0; i < 6; i++) {
    vel[i] = command_interfaces[i].get_value();
  }
  return vel;
}
inline void setCartesianVelocityCommand(
    std::vector<hardware_interface::LoanedCommandInterface>& command_interfaces,
    const std::array<double, 6>& vel,
    const std::array<double, 2>& elbow = {0.0, 0.0}) {
  for (size_t i = 0; i < 6; i++) {
    command_interfaces[i].set_value(vel[i]);
  }
  command_interfaces[6].set_value(elbow[0]);
  command_interfaces[7].set_value(elbow[1]);
}

}  // namespace franka_example_controllers
