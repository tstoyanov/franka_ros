// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <array>
#include <memory>
#include <string>
#include <vector>

#include <controller_interface/controller_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

#include <realtime_tools/realtime_publisher.hpp>

#include <franka_example_controllers/msg/joint_torque_comparison.hpp>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/trigger_rate.h>

namespace franka_example_controllers {

class JointImpedanceExampleController : public controller_interface::ControllerInterface {
 public:
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::return_type update(const rclcpp::Time& time,
                                           const rclcpp::Duration& period) override;
  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

 private:
  // Saturation
  std::array<double, 7> saturateTorqueRate(
      const std::array<double, 7>& tau_d_calculated,
      const std::array<double, 7>& tau_j_d);  // NOLINT (readability-identifier-naming)

  std::string arm_id_;
  std::vector<std::string> joint_names_;
  std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;

  static constexpr double kDeltaTauMax{1.0};
  double radius_{0.1};
  double acceleration_time_{2.0};
  double vel_max_{0.05};
  double angle_{0.0};
  double vel_current_{0.0};

  std::vector<double> k_gains_;
  std::vector<double> d_gains_;
  double coriolis_factor_{1.0};
  std::array<double, 7> dq_filtered_;
  std::array<double, 16> initial_pose_;

  franka_hw::TriggerRate rate_trigger_{1.0};
  std::array<double, 7> last_tau_d_{};
  std::shared_ptr<realtime_tools::RealtimePublisher<franka_example_controllers::msg::JointTorqueComparison>>
      torques_publisher_;
};

}  // namespace franka_example_controllers
