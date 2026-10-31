// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <array>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>

#include <franka_hw/franka_cartesian_command_interface.h>
#include <franka_hw/franka_hw.h>

using std::string;

std::string arm_id("panda");
std::array<std::string, 7> joint_names = {
    arm_id + "_joint1", arm_id + "_joint2", arm_id + "_joint3", arm_id + "_joint4",
    arm_id + "_joint5", arm_id + "_joint6", arm_id + "_joint7"};

namespace franka_hw {

namespace {

hardware_interface::HardwareInfo makeTestHardwareInfo() {
  hardware_interface::HardwareInfo info;
  info.name = "FrankaHW";
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

std::vector<std::string> jointInterfaces(const std::string& iface) {
  std::vector<std::string> interfaces;
  for (const auto& joint : joint_names) {
    interfaces.push_back(joint + "/" + iface);
  }
  return interfaces;
}

}  // namespace

TEST(FrankaHWControllerSwitching, AdmissibleRequests) {
  FrankaHW robot;
  ASSERT_EQ(robot.initParameters(makeTestHardwareInfo()), CallbackReturn::SUCCESS);

  // single joint position claim
  EXPECT_EQ(robot.prepare_command_mode_switch(jointInterfaces(hardware_interface::HW_IF_POSITION), {}),
            hardware_interface::return_type::OK);
  // torque + position combination is admissible
  EXPECT_EQ(robot.prepare_command_mode_switch(jointInterfaces(hardware_interface::HW_IF_EFFORT),
                                              jointInterfaces(hardware_interface::HW_IF_POSITION)),
            hardware_interface::return_type::OK);
  // cartesian pose claim on its own
  FrankaHW robot2;
  ASSERT_EQ(robot2.initParameters(makeTestHardwareInfo()), CallbackReturn::SUCCESS);
  std::vector<std::string> pose_interfaces;
  for (size_t i = 0; i < 16; i++) {
    pose_interfaces.push_back(arm_id + "/" + kFrankaPoseCartesianInterface + "/O_T_EE_0" +
                              std::to_string(i));
  }
  EXPECT_EQ(robot2.prepare_command_mode_switch(pose_interfaces, {}),
            hardware_interface::return_type::OK);
  // stopping everything again
  EXPECT_EQ(robot.prepare_command_mode_switch(
                {}, jointInterfaces(hardware_interface::HW_IF_POSITION)),
            hardware_interface::return_type::OK);
}

TEST(FrankaHWControllerSwitching, PerformSwitchActivatesController) {
  FrankaHW robot;
  ASSERT_EQ(robot.initParameters(makeTestHardwareInfo()), CallbackReturn::SUCCESS);
  ASSERT_FALSE(robot.controllerActive());
  ASSERT_EQ(robot.prepare_command_mode_switch(jointInterfaces(hardware_interface::HW_IF_EFFORT), {}),
            hardware_interface::return_type::OK);
  EXPECT_EQ(robot.perform_command_mode_switch(jointInterfaces(hardware_interface::HW_IF_EFFORT), {}),
            hardware_interface::return_type::OK);
  EXPECT_TRUE(robot.controllerActive());
}

}  // namespace franka_hw
