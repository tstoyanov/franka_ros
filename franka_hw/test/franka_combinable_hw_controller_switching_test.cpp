// Copyright (c) 2019 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <array>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>

#include <franka_hw/franka_cartesian_command_interface.h>
#include <franka_hw/franka_combinable_hw.h>

extern std::string arm_id;
extern std::array<std::string, 7> joint_names;

namespace franka_hw {

namespace {

hardware_interface::HardwareInfo makeTestHardwareInfo() {
  hardware_interface::HardwareInfo info;
  info.name = "FrankaCombinableHW";
  for (const auto& joint : joint_names) {
    hardware_interface::ComponentInfo joint_info;
    joint_info.name = joint;
    info.joints.push_back(joint_info);
  }
  info.hardware_parameters["arm_id"] = arm_id;
  info.hardware_parameters["robot_ip"] = "unused_dummy_ip";
  info.hardware_parameters["rate_limiting"] = "true";
  info.hardware_parameters["cutoff_frequency"] = "1000";
  info.hardware_parameters["internal_controller"] = "joint_impedance";
  info.hardware_parameters["realtime_config"] = "enforce";
  return info;
}

std::vector<std::string> jointEffortInterfaces() {
  std::vector<std::string> interfaces;
  for (const auto& joint : joint_names) {
    interfaces.push_back(joint + "/" + hardware_interface::HW_IF_EFFORT);
  }
  return interfaces;
}

}  // namespace

TEST(FrankaCombinableHWControllerSwitching, OnlyTorqueModeIsAdmissible) {
  FrankaCombinableHW robot;
  ASSERT_EQ(robot.initParameters(makeTestHardwareInfo()), CallbackReturn::SUCCESS);

  // torque claim is admissible
  EXPECT_EQ(robot.prepare_command_mode_switch(jointEffortInterfaces(), {}),
            hardware_interface::return_type::OK);

  // position claim is not supported by the combinable hardware
  std::vector<std::string> position_interfaces;
  for (const auto& joint : joint_names) {
    position_interfaces.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
  }
  EXPECT_EQ(robot.prepare_command_mode_switch(position_interfaces, {}),
            hardware_interface::return_type::ERROR);
}

}  // namespace franka_hw
