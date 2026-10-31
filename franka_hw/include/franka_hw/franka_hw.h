// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <array>
#include <atomic>
#include <exception>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <franka/control_types.h>
#include <franka/duration.h>
#include <franka/model.h>
#include <franka/rate_limiting.h>
#include <franka/robot.h>
#include <franka/robot_state.h>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <franka_hw/control_mode.h>
#include <franka_hw/franka_cartesian_command_interface.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <franka_hw/services.h>

#include <franka_msgs/action/error_recovery.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/bool.hpp>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace franka_hw {

/**
 * This class wraps the functionality of libfranka (0.7.0) for controlling Panda robots into the
 * ros2_control framework as a hardware_interface::SystemInterface plugin.
 */
class FrankaHW : public hardware_interface::SystemInterface {
 public:
  /**
   * Default constructor.
   * Note: Be sure to call on_init() before operation.
   */
  FrankaHW();

  ~FrankaHW() override;

  /**
   * Initializes the hardware interface from the URDF/hardware info, connects to the robot via
   * libfranka and loads the model.
   *
   * @param[in] info The hardware info parsed from the URDF <ros2_control> tag.
   * @return CallbackReturn::SUCCESS if successful.
   */
  CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;

  /**
   * Reads the parameterization of the hardware class from the hardware info parameters
   * (e.g. arm_id, robot_ip, joint_names etc.)
   *
   * @param[in] info The hardware info.
   * @return CallbackReturn::SUCCESS if successful, ERROR otherwise.
   */
  virtual CallbackReturn initParameters(const hardware_interface::HardwareInfo& info);

  /**
   * Exports all state interfaces (joint states, franka robot state, model).
   */
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  /**
   * Exports all command interfaces (joint position/velocity/effort, cartesian pose/velocity).
   */
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  /**
   * Determines the control mode from the interfaces to start/stop and configures the libfranka
   * control callback accordingly.
   */
  hardware_interface::return_type prepare_command_mode_switch(
      const std::vector<std::string>& start_interfaces,
      const std::vector<std::string>& stop_interfaces) override;

  /**
   * Activates the control mode chosen in prepare_command_mode_switch (real-time capable).
   */
  hardware_interface::return_type perform_command_mode_switch(
      const std::vector<std::string>& start_interfaces,
      const std::vector<std::string>& stop_interfaces) override;

  /**
   * Runs the currently active controller in a realtime loop.
   *
   * If no controller is active, the function immediately exits. When running a controller,
   * the function only exits when ros_callback returns false.
   *
   * @param[in] ros_callback A callback function that is executed at each time step and runs
   * all ROS-side functionality of the hardware. Execution is stopped if it returns false.
   *
   * @throw franka::ControlException if an error related to torque control or motion generation
   * occurred.
   * @throw franka::InvalidOperationException if a conflicting operation is already running.
   * @throw franka::NetworkException if the connection is lost, e.g. after a timeout.
   * @throw franka::RealtimeException if realtime priority cannot be set for the current thread.
   */
  virtual void control(
      const std::function<bool(const rclcpp::Time&, const rclcpp::Duration&)>& ros_callback =
          [](const rclcpp::Time&, const rclcpp::Duration&) {
            return true;
          }) const;

  /**
   * Updates the robot state published through the state interfaces.
   *
   * @param[in] robot_state Current robot state.
   */
  virtual void update(const franka::RobotState& robot_state);

  /**
   * Indicates whether there is an active controller.
   *
   * @return True if a controller is currently active, false otherwise.
   */
  virtual bool controllerActive() const noexcept;

  /**
   * Gets the current joint position command.
   *
   * @return The current joint position command.
   */
  virtual std::array<double, 7> getJointPositionCommand() const noexcept;

  /**
   * Gets the current joint velocity command.
   *
   * @return The current joint velocity command.
   */
  virtual std::array<double, 7> getJointVelocityCommand() const noexcept;

  /**
   * Gets the current joint torque command.
   *
   * @return The current joint torque command.
   */
  virtual std::array<double, 7> getJointEffortCommand() const noexcept;

  /**
   * Resets the limit interfaces.
   */
  virtual void reset();

  /**
   * Checks the proximity of each joint to its joint position limits and logs
   * a warning whenever a joint is close to a limit.
   */
  virtual void checkJointLimits();

  /**
   * Reads data from the franka robot.
   *
   * @param[in] time The current time.
   * @param[in] period The time passed since the last call to \ref read.
   */
  hardware_interface::return_type read(const rclcpp::Time& time,
                                       const rclcpp::Duration& period) override;

  /**
   * Writes data to the franka robot.
   *
   * @param[in] time The current time.
   * @param[in] period The time passed since the last call to \ref write.
   */
  hardware_interface::return_type write(const rclcpp::Time& time,
                                        const rclcpp::Duration& period) override;

  /**
   * Getter for the libfranka robot instance.
   */
  virtual franka::Robot& robot() const;

  /**
   * Getter for the libfranka model instance.
   */
  virtual franka::Model& model() const;

  /**
   * Getter for the current robot state (ROS-side copy).
   */
  const franka::RobotState& robotStateRos() const;

  /**
   * Getter for the arm_id which identifies this hardware instance.
   */
  const std::string& armId() const noexcept { return arm_id_; }

  /**
   * Getter method for the arm_id which is used to distinguish between multiple
   * instances of FrankaHW. Returns a copy of the arm_id string.
   */
  std::string getArmID() const noexcept;

  /**
   * Triggers a stop of the controlLoop. This interface is used to stop all
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

  /**
   * Checks a command for NaN values.
   *
   * @param[in] command  The command to check.
   *
   * @return True if the command contains NaN, false otherwise.
   */
  static bool commandHasNaN(const franka::Torques& command);

  /**
   * Checks a command for NaN values.
   *
   * @param[in] command The command to check.
   *
   * @return True if the command contains NaN, false otherwise.
   */
  static bool commandHasNaN(const franka::JointPositions& command);

  /**
   * Checks a command for NaN values.
   *
   * @param[in] command The command to check.
   *
   * @return True if the command contains NaN, false otherwise.
   */
  static bool commandHasNaN(const franka::JointVelocities& command);

  /**
   * Checks a command for NaN values.
   *
   * @param[in] command  The command to check.
   *
   * @return True if the command contains NaN, false otherwise.
   */
  static bool commandHasNaN(const franka::CartesianPose& command);

  /**
   * Checks a command for NaN values.
   *
   * @param[in] command  The command to check.
   *
   * @return True if the command contains NaN, false otherwise.
   */
  static bool commandHasNaN(const franka::CartesianVelocities& command);

 protected:
  /**
   * Checks whether an array of doubles contains NaN values.
   *
   * @param[in] command  array The array to check.
   *
   * @return True if the array contains NaN values, false otherwise.
   */
  template <size_t size>
  static bool arrayHasNaN(const std::array<double, size>& array) {
    return std::any_of(array.begin(), array.end(), [](const double& e) { return std::isnan(e); });
  }

  using Callback = std::function<bool(const franka::RobotState&, franka::Duration)>;

  /**
   * Background loop that runs the libfranka control (run_function_) whenever a controller is
   * active. While no controller is active (or after a control error), it keeps the state
   * interfaces updated with robot_->readOnce(). This replaces the ROS 1
   * franka_control_node main loop: in ros2_control the controller_manager update cycle
   * only calls read()/write(), so the blocking libfranka control loop must run in its own
   * thread.
   */
  void controlLoop();

  /**
   * Publishes the error state on the "has_error" topic.
   */
  void publishErrorState(bool error);

  /**
   * Sets up the libfranka parameter services and the error recovery action server on the
   * hardware node.
   */
  void setupServicesAndActionServers();

  /**
   * Callback for the libfranka control loop. This method is designed to incorporate a
   * second callback named ros_callback that will be called on each iteration of the callback.
   *
   * @param[in] command The datafield containing the command to send to the robot.
   * @param[in] ros_callback An additional callback that is executed every time step.
   * @param[in] robot_state The current robot state to compute commands with.
   * @param[in] time_step Time since last call to the callback.
   * @throw std::invalid_argument When a command contains NaN values.
   *
   * @return The command to be sent to the robot via libfranka.
   */
  template <typename T>
  T controlCallback(const T& command,
                    Callback ros_callback,
                    const franka::RobotState& robot_state,
                    franka::Duration time_step) {
    robot_state_libfranka_ = robot_state;
    rclcpp::Time now = rclcpp::Clock(RCL_SYSTEM_TIME).now();
    read(now, rclcpp::Duration::from_seconds(time_step.toSec()));

    if (!controller_active_ || (ros_callback && !ros_callback(robot_state, time_step))) {
      return franka::MotionFinished(command);
    }

    write(now, rclcpp::Duration::from_seconds(time_step.toSec()));
    if (commandHasNaN(command)) {
      std::string error_message = "FrankaHW::controlCallback: Got NaN command!";
      RCLCPP_FATAL(rclcpp::get_logger("FrankaHW"), "%s", error_message.c_str());
      throw std::invalid_argument(error_message);
    }

    return command;
  }

  /**
   * Configures the run function which is used as libfranka control callback based on the requested
   * control mode.
   *
   * @param[in] requested_control_mode The control mode to configure (e.g. torque/position/velocity
   * etc.)
   * @param[in] limit_rate Flag to enable/disable rate limiting to smoothen the commands.
   * @param[in] cutoff_frequency The cutoff frequency applied for command smoothing.
   * @param[in] internal_controller The internal controller to use when using position or velocity
   * modes.
   *
   * @return True if successful, false otherwise.
   */
  virtual bool setRunFunction(const ControlMode& requested_control_mode,
                              bool limit_rate,
                              double cutoff_frequency,
                              franka::ControllerMode internal_controller);

  /**
   * Uses the robot_ip_ to connect to the robot via libfranka and loads the libfranka model.
   */
  virtual void initRobot();

  /**
   * Parses a set of collision thresholds from the hardware info parameters. The method returns
   * the default values if no parameter was found or the size of the array did not match
   * the defaults dimension.
   *
   * @param[in] name The name of the parameter to look for.
   * @param[in] info The hardware info holding the parameters.
   * @param[in] defaults A set of default values that also specify the size the parameter must have
   * to be valid.
   * @return A set parsed parameters if valid parameters where found, the default values otherwise.
   */
  static std::vector<double> getCollisionThresholds(const std::string& name,
                                                    const hardware_interface::HardwareInfo& info,
                                                    const std::vector<double>& defaults);

  /**
   * Derives the ControlMode bitmask from the claimed interface names.
   */
  ControlMode controlModeFromInterfaces(const std::vector<std::string>& start_interfaces,
                                        const std::vector<std::string>& stop_interfaces) const;

  struct CollisionConfig {
    std::array<double, 7> lower_torque_thresholds_acceleration;
    std::array<double, 7> upper_torque_thresholds_acceleration;
    std::array<double, 7> lower_torque_thresholds_nominal;
    std::array<double, 7> upper_torque_thresholds_nominal;
    std::array<double, 6> lower_force_thresholds_acceleration;
    std::array<double, 6> upper_force_thresholds_acceleration;
    std::array<double, 6> lower_force_thresholds_nominal;
    std::array<double, 6> upper_force_thresholds_nominal;
  };

  CollisionConfig collision_config_;

  std::mutex libfranka_state_mutex_;
  std::mutex ros_state_mutex_;
  franka::RobotState robot_state_libfranka_{};
  franka::RobotState robot_state_ros_{};

  // Boxed pointers whose bits are transported on the franka state/model interfaces
  // (see export_state_interfaces). They must always alias robot_state_ros_ / model_.
  franka::RobotState* state_box_ptr_{nullptr};
  franka::Model* model_box_ptr_{nullptr};

  std::mutex libfranka_cmd_mutex_;
  franka::JointPositions position_joint_command_libfranka_;
  franka::JointVelocities velocity_joint_command_libfranka_;
  franka::Torques effort_joint_command_libfranka_;
  franka::CartesianPose pose_cartesian_command_libfranka_;
  franka::CartesianVelocities velocity_cartesian_command_libfranka_;

  std::mutex ros_cmd_mutex_;
  franka::JointPositions position_joint_command_ros_;
  franka::JointVelocities velocity_joint_command_ros_;
  franka::Torques effort_joint_command_ros_;
  franka::CartesianPose pose_cartesian_command_ros_;
  franka::CartesianVelocities velocity_cartesian_command_ros_;

  std::unique_ptr<franka::Robot> robot_;
  std::unique_ptr<franka::Model> model_;

  std::array<std::string, 7> joint_names_;
  std::string arm_id_;
  std::string robot_ip_;
  double joint_limit_warning_threshold_{0.1};
  franka::RealtimeConfig realtime_config_;

  bool initialized_{false};
  std::atomic_bool controller_active_{false};
  ControlMode current_control_mode_ = ControlMode::None;

  bool rate_limiting_{true};
  double cutoff_frequency_{1000.0};
  franka::ControllerMode internal_controller_{franka::ControllerMode::kJointImpedance};

  std::function<void(franka::Robot&, Callback)> run_function_;

  std::unique_ptr<std::thread> control_loop_thread_;
  ServiceContainer services_;
  rclcpp_action::Server<franka_msgs::action::ErrorRecovery>::SharedPtr recovery_action_server_;
  std::atomic_bool has_error_{false};
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr has_error_pub_;
  std::atomic_bool error_recovered_{false};
  mutable std::atomic_bool controller_needs_reset_{false};

  rclcpp::Logger logger_{rclcpp::get_logger("FrankaHW")};
};

}  // namespace franka_hw
