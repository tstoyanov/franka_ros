// Copyright (c) 2019 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <array>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <hardware_interface/component_parser.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include <franka_hw/franka_cartesian_command_interface.h>
#include <franka_hw/franka_hw.h>

namespace {

// Runs xacro on the given xacro file and returns the generated URDF.
std::string runXacro(const std::string& xacro_file) {
  const std::string share = ament_index_cpp::get_package_share_directory("franka_description");
  const std::string command = "xacro " + share + "/robots/" + xacro_file +
                              " arm_id:=panda robot_ip:=1.2.3.4 > /tmp/franka_urdf_test.xml";
  if (std::system(command.c_str()) != 0) {
    return "";
  }
  std::ifstream file("/tmp/franka_urdf_test.xml");
  std::stringstream buffer;
  buffer << file.rdbuf();
  return buffer.str();
}

}  // namespace

TEST(FrankaUrdfTest, PandaArmHandDeclaresRos2ControlHardware) {
  const std::string urdf = runXacro("panda_arm_hand.urdf.xacro");
  ASSERT_FALSE(urdf.empty()) << "xacro failed to generate the URDF";

  std::vector<hardware_interface::HardwareInfo> hardware_info;
  try {
    hardware_info = hardware_interface::parse_control_resources_from_urdf(urdf);
  } catch (const std::exception& e) {
    FAIL() << "ros2_control tag could not be parsed: " << e.what();
  }
  ASSERT_EQ(hardware_info.size(), 1u);

  const auto& info = hardware_info[0];
  EXPECT_EQ(info.type, "system");
  EXPECT_EQ(info.hardware_plugin_name, "franka_hw/FrankaCombinableHW");

  // The hardware parameters the FrankaHW::initParameters expects from the URDF.
  EXPECT_NE(info.hardware_parameters.find("arm_id"), info.hardware_parameters.end());
  EXPECT_NE(info.hardware_parameters.find("robot_ip"), info.hardware_parameters.end());
  EXPECT_NE(info.hardware_parameters.find("rate_limiting"), info.hardware_parameters.end());
  EXPECT_NE(info.hardware_parameters.find("cutoff_frequency"), info.hardware_parameters.end());
  EXPECT_NE(info.hardware_parameters.find("internal_controller"), info.hardware_parameters.end());
  EXPECT_NE(info.hardware_parameters.find("realtime_config"), info.hardware_parameters.end());

  // 7 joints with position/velocity/effort command and state interfaces.
  ASSERT_EQ(info.joints.size(), 7u);
  for (const auto& joint : info.joints) {
    ASSERT_EQ(joint.command_interfaces.size(), 3u) << joint.name;
    ASSERT_EQ(joint.state_interfaces.size(), 3u) << joint.name;
  }
  EXPECT_EQ(info.joints[0].name, "panda_joint1");
  EXPECT_EQ(info.joints[6].name, "panda_joint7");

  // One gpio carrying the franka state/model and cartesian command interfaces.
  ASSERT_EQ(info.gpios.size(), 1u);
  const auto& gpio = info.gpios[0];
  EXPECT_EQ(gpio.name, "panda");

  // The exported interfaces must match FrankaHW::export_state_interfaces and
  // export_command_interfaces exactly, otherwise resource_manager::validate_storage
  // rejects the URDF at startup.
  const std::array<std::string, 2> gpio_state_interfaces = {
      "franka_hw/FrankaStateInterface", "franka_hw/FrankaModelInterface"};
  ASSERT_EQ(gpio.state_interfaces.size(), gpio_state_interfaces.size());
  for (size_t i = 0; i < gpio_state_interfaces.size(); i++) {
    EXPECT_EQ(gpio.state_interfaces[i].name, gpio_state_interfaces[i]);
  }

  // 16 pose + 2 elbow + 6 velocity + 2 elbow = 26 command interfaces.
  ASSERT_EQ(gpio.command_interfaces.size(), 26u);
  EXPECT_EQ(gpio.command_interfaces[0].name, "franka_hw/FrankaPoseCartesianInterface/O_T_EE_00");
  EXPECT_EQ(gpio.command_interfaces[15].name, "franka_hw/FrankaPoseCartesianInterface/O_T_EE_15");
  EXPECT_EQ(gpio.command_interfaces[16].name, "franka_hw/FrankaPoseCartesianInterface/elbow_0");
  EXPECT_EQ(gpio.command_interfaces[17].name, "franka_hw/FrankaPoseCartesianInterface/elbow_1");
  EXPECT_EQ(gpio.command_interfaces[18].name,
            "franka_hw/FrankaVelocityCartesianInterface/O_dP_EE_0");
  EXPECT_EQ(gpio.command_interfaces[23].name,
            "franka_hw/FrankaVelocityCartesianInterface/O_dP_EE_5");
  EXPECT_EQ(gpio.command_interfaces[24].name,
            "franka_hw/FrankaVelocityCartesianInterface/elbow_0");
  EXPECT_EQ(gpio.command_interfaces[25].name,
            "franka_hw/FrankaVelocityCartesianInterface/elbow_1");
}

TEST(FrankaUrdfTest, DeclaredInterfacesMatchExportedInterfaces) {
  // The interfaces declared in the URDF must exactly match the ones exported by
  // FrankaHW, otherwise resource_manager::validate_storage fails the startup.
  const std::string urdf = runXacro("panda_arm_hand.urdf.xacro");
  ASSERT_FALSE(urdf.empty());

  hardware_interface::HardwareInfo info;
  ASSERT_NO_THROW(info = hardware_interface::parse_control_resources_from_urdf(urdf)[0]);

  franka_hw::FrankaHW hardware;
  // initParameters only (do not connect to a robot).
  ASSERT_EQ(hardware.initParameters(info), CallbackReturn::SUCCESS);

  // Sanity-check the NaN command validation while we have the hardware here (this also
  // anchors a direct libfranka symbol reference in this test binary).
  EXPECT_TRUE(franka_hw::FrankaHW::commandHasNaN(
      franka::JointPositions({std::numeric_limits<double>::quiet_NaN(), 0, 0, 0, 0, 0, 0})));
  EXPECT_FALSE(franka_hw::FrankaHW::commandHasNaN(
      franka::JointPositions({0, 0, 0, 0, 0, 0, 0})));

  // Collect the interface names exported by the hardware. The model interface is
  // only exported once a robot connection loaded the franka::Model, so it is added
  // manually here.
  std::set<std::string> exported_state;
  std::set<std::string> exported_command;
  for (const auto& si : hardware.export_state_interfaces()) {
    exported_state.insert(si.get_name());
  }
  exported_state.insert("panda/" + std::string(franka_hw::kFrankaModelInterface));
  for (const auto& ci : hardware.export_command_interfaces()) {
    exported_command.insert(ci.get_name());
  }

  // Collect the interface names declared in the URDF.
  std::set<std::string> declared_state;
  std::set<std::string> declared_command;
  for (const auto& joint : info.joints) {
    for (const auto& s : joint.state_interfaces) {
      declared_state.insert(joint.name + "/" + s.name);
    }
    for (const auto& c : joint.command_interfaces) {
      declared_command.insert(joint.name + "/" + c.name);
    }
  }
  for (const auto& gpio : info.gpios) {
    for (const auto& s : gpio.state_interfaces) {
      declared_state.insert(gpio.name + "/" + s.name);
    }
    for (const auto& c : gpio.command_interfaces) {
      declared_command.insert(gpio.name + "/" + c.name);
    }
  }

  // Every declared interface must be exported...
  for (const auto& name : declared_state) {
    EXPECT_TRUE(exported_state.count(name) > 0)
        << "state interface declared but not exported: " << name;
  }
  for (const auto& name : declared_command) {
    EXPECT_TRUE(exported_command.count(name) > 0)
        << "command interface declared but not exported: " << name;
  }
  // ... and vice versa.
  for (const auto& name : exported_state) {
    EXPECT_TRUE(declared_state.count(name) > 0)
        << "state interface exported but not declared: " << name;
  }
  for (const auto& name : exported_command) {
    EXPECT_TRUE(declared_command.count(name) > 0)
        << "command interface exported but not declared: " << name;
  }
}
