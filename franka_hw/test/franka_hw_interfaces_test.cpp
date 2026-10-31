// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <array>
#include <random>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>

#include <franka_hw/franka_cartesian_command_interface.h>
#include <franka_hw/franka_hw.h>
#include <franka_hw/franka_model_interface.h>

extern std::string arm_id;
extern std::array<std::string, 7> joint_names;

namespace franka_hw {

namespace {

hardware_interface::HardwareInfo makeTestHardwareInfo() {
  hardware_interface::HardwareInfo info;
  info.name = "FrankaHW";
  for (const auto& joint : joint_names) {
    hardware_interface::ComponentInfo joint_info;
    joint_info.name = joint;
    joint_info.type = "joint";
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

}  // namespace

TEST(FrankaHWTests, ExportsInterfaces) {
  FrankaHW robot;
  auto info = makeTestHardwareInfo();
  // Skip connecting to a real robot: only parameter parsing is tested here.
  ASSERT_EQ(robot.initParameters(info), CallbackReturn::SUCCESS);

  auto state_interfaces = robot.export_state_interfaces();
  auto command_interfaces = robot.export_command_interfaces();

  // 7 joints x 3 state interfaces + franka state interface
  EXPECT_EQ(state_interfaces.size(), 7 * 3 + 1);
  // 7 joints x 3 command interfaces + 16 pose + 2 pose elbow + 6 velocity + 2 velocity elbow
  EXPECT_EQ(command_interfaces.size(), 7 * 3 + 16 + 2 + 6 + 2);

  bool has_robot_state = false;
  for (const auto& si : state_interfaces) {
    if (si.get_name() == arm_id &&
        si.get_interface_name() == kFrankaStateInterface) {
      has_robot_state = true;
    }
  }
  EXPECT_TRUE(has_robot_state);
}

TEST(FrankaHWTests, HandlesWorkForReadAndCommand) {
  FrankaHW robot;
  auto info = makeTestHardwareInfo();
  ASSERT_EQ(robot.initParameters(info), CallbackReturn::SUCCESS);

  franka::RobotState robot_state{};
  FrankaStateHandle state_handle(arm_id + "_robot", robot_state);
  EXPECT_EQ(state_handle.getName(), arm_id + "_robot");
  EXPECT_EQ(&state_handle.getRobotState(), &robot_state);

  std::array<double, 16> pose_command = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
                                         1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
  std::array<double, 2> elbow_command = {0.5, -0.5};
  FrankaCartesianPoseHandle pose_handle(state_handle, pose_command, elbow_command);
  pose_handle.setCommand(pose_command);
  EXPECT_EQ(pose_command, pose_handle.getCommand());
  pose_handle.setCommand(pose_command, elbow_command);
  EXPECT_EQ(elbow_command, pose_handle.getElbow());

  std::array<double, 6> vel_command = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
  std::array<double, 2> vel_elbow = {0.1, 0.2};
  FrankaCartesianVelocityHandle vel_handle(state_handle, vel_command, vel_elbow);
  vel_handle.setCommand(vel_command);
  EXPECT_EQ(vel_command, vel_handle.getCommand());
  vel_handle.setCommand(vel_command, vel_elbow);
  EXPECT_EQ(vel_elbow, vel_handle.getElbow());
}

}  // namespace franka_hw
