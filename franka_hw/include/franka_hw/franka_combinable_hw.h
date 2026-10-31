// Copyright (c) 2019 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <atomic>
#include <functional>
#include <memory>
#include <string>
#include <thread>

#include <franka/exception.h>
#include <franka/robot_state.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/bool.hpp>

#include <franka_hw/franka_hw.h>
#include <franka_hw/services.h>
#include <franka_msgs/action/error_recovery.hpp>

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
  FrankaCombinableHW();

  CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;

  /**
   * Runs the currently active controller in a realtime loop. If no controller is active, the
   * function immediately exits.
   *
   * @param[in] ros_callback A callback function that is executed at each time step.
   *
   * @throw franka::ControlException if an error related to torque control occurred.
   */
  void control(const std::function<bool(const rclcpp::Time&, const rclcpp::Duration&)>&
                   ros_callback =
               [](const rclcpp::Time&, const rclcpp::Duration&) {
                 return true;
               }) const override;

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

  /**
   * Getter method for the arm_id which is used to distinguish between multiple
   * instances of FrankaCombinableHW.
   *
   * @return A copy of the arm_id string identifying the class instance.
   */
  std::string getArmID() const noexcept;

  /**
   * Triggers a stop of the controlLoop. This interface is used to stop all combined
   * robots together when one robot encounters an error.
   */
  void triggerError();

  /**
   * Getter for the error flag of the class.
   *
   * @return True in case of an error, false otherwise.
   */
  bool hasError() const noexcept;

  /**
   * Recovers the libfranka robot, resets the error flag and publishes the error state.
   */
  void resetError();

  /**
   * Returns whether the controller needs to be reset e.g. after error recovery.
   *
   * @return A copy of the controller_needs_reset flag.
   */
  bool controllerNeedsReset() const noexcept;

 private:
  template <typename T>
  T libfrankaUpdateCallback(const T& command,
                            const franka::RobotState& robot_state,
                            franka::Duration time_step) {
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

  void publishErrorState(bool error);

  void setupServicesAndActionServers(rclcpp::Node::SharedPtr node);

  void initRobot() override;

  bool setRunFunction(const ControlMode& requested_control_mode,
                      bool limit_rate,
                      double cutoff_frequency,
                      franka::ControllerMode internal_controller) override;

  void controlLoop();

  std::unique_ptr<std::thread> control_loop_thread_;
  ServiceContainer services_;
  rclcpp_action::Server<franka_msgs::action::ErrorRecovery>::SharedPtr recovery_action_server_;
  std::atomic_bool has_error_{false};
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr has_error_pub_;
  std::atomic_bool error_recovered_{false};
  mutable std::atomic_bool controller_needs_reset_{false};
};

}  // namespace franka_hw
