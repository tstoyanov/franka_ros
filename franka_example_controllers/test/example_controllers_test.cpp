// Copyright (c) 2024 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <memory>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include <controller_interface/controller_interface_base.hpp>
#include <controller_interface/controller_interface_params.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/rclcpp.hpp>

#include <franka_example_controllers/example_controllers_common.h>
#include <franka_example_controllers/joint_impedance_example_controller.h>

namespace {

const std::string kArmId = "panda";

std::vector<std::string> jointNames() {
  return {kArmId + "_joint1", kArmId + "_joint2", kArmId + "_joint3", kArmId + "_joint4",
          kArmId + "_joint5", kArmId + "_joint6", kArmId + "_joint7"};
}

bool initController(franka_example_controllers::JointImpedanceExampleController& controller,
                    const std::string& name) {
  controller_interface::ControllerInterfaceParams params;
  params.controller_name = name;
  params.update_rate = 100;
  params.controller_manager_update_rate = 100;
  return controller.init(params) == controller_interface::return_type::OK;
}

void declareControllerParameters(franka_example_controllers::JointImpedanceExampleController&
                                    controller) {
  auto node = controller.get_node();
  node->set_parameters({rclcpp::Parameter("arm_id", kArmId),
                        rclcpp::Parameter("joint_names", jointNames()),
                        rclcpp::Parameter("k_gains", std::vector<double>(7, 10.0)),
                        rclcpp::Parameter("d_gains", std::vector<double>(7, 0.1)),
                        rclcpp::Parameter("publish_rate", 30.0),
                        rclcpp::Parameter("coriolis_factor", 1.0)});
}

}  // namespace

// The controller must claim the joint effort command interfaces AND the cartesian
// pose command interfaces: update() writes both torques and the cartesian pose
// (the ROS 1 version used the FrankaCartesianPoseHandle). Regression test for the
// out-of-bounds command_interfaces_ access.
TEST(FrankaExampleControllersTest, JointImpedanceClaimsTorqueAndCartesianPoseInterfaces) {
  franka_example_controllers::JointImpedanceExampleController controller;
  ASSERT_TRUE(initController(controller, "joint_impedance_example_controller"));
  declareControllerParameters(controller);
  ASSERT_EQ(controller.configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  // 7 joint effort + 18 cartesian pose command interfaces.
  const auto cmd_config = controller.command_interface_configuration();
  ASSERT_EQ(cmd_config.type, controller_interface::interface_configuration_type::INDIVIDUAL);
  ASSERT_EQ(cmd_config.names.size(), 7u + 18u);
  for (size_t i = 0; i < 7; i++) {
    EXPECT_EQ(cmd_config.names[i], jointNames()[i] + "/effort");
  }
  const auto pose_ifaces = franka_example_controllers::cartesianPoseCommandInterfaces(kArmId);
  for (size_t i = 0; i < 18; i++) {
    EXPECT_EQ(cmd_config.names[7 + i], pose_ifaces[i]);
  }

  // franka state + model state interfaces.
  const auto state_config = controller.state_interface_configuration();
  ASSERT_EQ(state_config.type, controller_interface::interface_configuration_type::INDIVIDUAL);
  EXPECT_EQ(state_config.names.size(), 2u);
  EXPECT_EQ(state_config.names[0], franka_example_controllers::frankaStateInterface(kArmId));
  EXPECT_EQ(state_config.names[1], franka_example_controllers::frankaModelInterface(kArmId));
}

// on_configure must not touch the state interfaces: they are only loaned to the
// controller at activation time by the controller_manager. Regression test for the
// crash on configure() with empty state_interfaces_.
TEST(FrankaExampleControllersTest, ConfigureSucceedsWithoutStateInterfaces) {
  franka_example_controllers::JointImpedanceExampleController controller;
  ASSERT_TRUE(initController(controller, "joint_impedance_example_controller"));
  declareControllerParameters(controller);

  // No interfaces assigned here: state_interfaces_ is empty at configure time.
  ASSERT_EQ(controller.configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
}

// The franka state interface transports the franka::RobotState pointer in the double
// value (as FrankaHW::export_state_interfaces does with the boxed pointer storage).
// Verify the roundtrip through a LoanedStateInterface returns the same robot state.
TEST(FrankaExampleControllersTest, RobotStatePointerRoundtrip) {
  franka::RobotState robot_state{};
  robot_state.q = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7};
  franka::RobotState* state_box_ptr = &robot_state;

  auto state_interface = std::make_shared<hardware_interface::StateInterface>(
      hardware_interface::StateInterface(
          kArmId, franka_hw::kFrankaStateInterface,
          reinterpret_cast<double*>(&state_box_ptr)));
  hardware_interface::LoanedStateInterface loaned(state_interface);

  auto& decoded = franka_example_controllers::robotStateFromInterface(loaned);
  EXPECT_EQ(&decoded, &robot_state);
  for (size_t i = 0; i < 7; i++) {
    EXPECT_DOUBLE_EQ(decoded.q[i], robot_state.q[i]);
  }
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
