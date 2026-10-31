// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_hw/franka_hw.h>

#include <array>
#include <cstdint>
#include <exception>
#include <functional>
#include <ostream>
#include <sstream>
#include <string>
#include <utility>

#include <franka/control_types.h>
#include <franka/rate_limiting.h>
#include <franka/robot.h>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>

namespace franka_hw {

namespace {
std::ostream& operator<<(std::ostream& ostream, franka::ControllerMode mode) {
  if (mode == franka::ControllerMode::kJointImpedance) {
    ostream << "joint_impedance";
  } else if (mode == franka::ControllerMode::kCartesianImpedance) {
    ostream << "cartesian_impedance";
  } else {
    ostream << "<unknown>";
  }
  return ostream;
}

std::string toStringWithPrecision(const double value, const size_t precision = 6) {
  std::ostringstream out;
  out.precision(precision);
  out << std::fixed << value;
  return out.str();
}

}  // namespace franka_hw

FrankaHW::FrankaHW()
    : position_joint_command_ros_({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}),
      position_joint_command_libfranka_({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}),
      velocity_joint_command_ros_({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}),
      velocity_joint_command_libfranka_({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}),
      effort_joint_command_ros_({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}),
      effort_joint_command_libfranka_({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}),
      pose_cartesian_command_ros_(
          {1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0}),
      pose_cartesian_command_libfranka_(
          {1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0}),
      velocity_cartesian_command_ros_({0.0, 0.0, 0.0, 0.0, 0.0, 0.0}),
      velocity_cartesian_command_libfranka_({0.0, 0.0, 0.0, 0.0, 0.0, 0.0}) {}

CallbackReturn FrankaHW::on_init(const hardware_interface::HardwareInfo& info) {
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  logger_ = get_node()->get_logger();
  if (initParameters(info) != CallbackReturn::SUCCESS) {
    RCLCPP_ERROR(logger_, "FrankaHW: Failed to parse all required parameters.");
    return CallbackReturn::ERROR;
  }
  try {
    initRobot();
  } catch (const std::runtime_error& error) {
    RCLCPP_ERROR(logger_, "FrankaHW: Failed to initialize libfranka robot. %s", error.what());
    return CallbackReturn::ERROR;
  }

  initialized_ = true;
  return CallbackReturn::SUCCESS;
}

CallbackReturn FrankaHW::initParameters(const hardware_interface::HardwareInfo& info) {
  const auto& params = info.hardware_parameters;
  auto get_param = [&params](const std::string& name, auto& value) {
    auto it = params.find(name);
    if (it == params.end()) {
      return false;
    }
    if constexpr (std::is_same_v<std::decay_t<decltype(value)>, std::string>) {
      value = it->second;
    } else if constexpr (std::is_same_v<std::decay_t<decltype(value)>, bool>) {
      value = (it->second == "true" || it->second == "1");
    } else if constexpr (std::is_same_v<std::decay_t<decltype(value)>, double>) {
      value = std::stod(it->second);
    }
    return true;
  };

  std::vector<std::string> joint_names_vector;
  for (const auto& joint : info.joints) {
    joint_names_vector.push_back(joint.name);
  }
  if (joint_names_vector.size() != 7) {
    RCLCPP_ERROR(logger_, "Invalid or no joint_names provided (need 7 joints from URDF)");
    return CallbackReturn::ERROR;
  }
  std::copy(joint_names_vector.cbegin(), joint_names_vector.cend(), joint_names_.begin());

  if (!get_param("rate_limiting", rate_limiting_)) {
    RCLCPP_ERROR(logger_, "Invalid or no rate_limiting parameter provided");
    return CallbackReturn::ERROR;
  }

  if (!get_param("cutoff_frequency", cutoff_frequency_)) {
    RCLCPP_ERROR(logger_, "Invalid or no cutoff_frequency parameter provided");
    return CallbackReturn::ERROR;
  }

  std::string internal_controller;
  if (!get_param("internal_controller", internal_controller)) {
    RCLCPP_ERROR(logger_, "No internal_controller parameter provided");
    return CallbackReturn::ERROR;
  }
  if (internal_controller == "joint_impedance") {
    internal_controller_ = franka::ControllerMode::kJointImpedance;
  } else if (internal_controller == "cartesian_impedance") {
    internal_controller_ = franka::ControllerMode::kCartesianImpedance;
  } else {
    RCLCPP_ERROR(logger_, "Invalid internal_controller parameter provided");
    return CallbackReturn::ERROR;
  }

  if (!get_param("arm_id", arm_id_)) {
    RCLCPP_ERROR(logger_, "Invalid or no arm_id parameter provided");
    return CallbackReturn::ERROR;
  }

  if (!get_param("robot_ip", robot_ip_)) {
    RCLCPP_ERROR(logger_, "Invalid or no robot_ip parameter provided");
    return CallbackReturn::ERROR;
  }

  if (!get_param("joint_limit_warning_threshold", joint_limit_warning_threshold_)) {
    RCLCPP_INFO(logger_,
                "No parameter joint_limit_warning_threshold is found, using default value %f",
                joint_limit_warning_threshold_);
  }

  std::string realtime_config_param = "enforce";
  get_param("realtime_config", realtime_config_param);
  if (realtime_config_param == "enforce") {
    realtime_config_ = franka::RealtimeConfig::kEnforce;
  } else if (realtime_config_param == "ignore") {
    realtime_config_ = franka::RealtimeConfig::kIgnore;
  } else {
    RCLCPP_ERROR(logger_,
                 "Invalid realtime_config parameter provided. Valid values are 'enforce', 'ignore'.");
    return CallbackReturn::ERROR;
  }

  // Get full collision behavior config from the parameters.
  std::vector<double> thresholds =
      getCollisionThresholds("lower_torque_thresholds_acceleration", info,
                             {20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0});
  std::copy(thresholds.begin(), thresholds.end(),
            collision_config_.lower_torque_thresholds_acceleration.begin());
  thresholds = getCollisionThresholds("upper_torque_thresholds_acceleration", info,
                                      {20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0});
  std::copy(thresholds.begin(), thresholds.end(),
            collision_config_.upper_torque_thresholds_acceleration.begin());
  thresholds = getCollisionThresholds("lower_torque_thresholds_nominal", info,
                                      {20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0});
  std::copy(thresholds.begin(), thresholds.end(),
            collision_config_.lower_torque_thresholds_nominal.begin());
  thresholds = getCollisionThresholds("upper_torque_thresholds_nominal", info,
                                      {20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0});
  std::copy(thresholds.begin(), thresholds.end(),
            collision_config_.upper_torque_thresholds_nominal.begin());
  thresholds = getCollisionThresholds("lower_force_thresholds_acceleration", info,
                                      {20.0, 20.0, 20.0, 25.0, 25.0, 25.0});
  std::copy(thresholds.begin(), thresholds.end(),
            collision_config_.lower_force_thresholds_acceleration.begin());
  thresholds = getCollisionThresholds("upper_force_thresholds_acceleration", info,
                                      {20.0, 20.0, 20.0, 25.0, 25.0, 25.0});
  std::copy(thresholds.begin(), thresholds.end(),
            collision_config_.upper_force_thresholds_acceleration.begin());
  thresholds = getCollisionThresholds("lower_force_thresholds_nominal", info,
                                      {20.0, 20.0, 20.0, 25.0, 25.0, 25.0});
  std::copy(thresholds.begin(), thresholds.end(),
            collision_config_.lower_force_thresholds_nominal.begin());
  thresholds = getCollisionThresholds("upper_force_thresholds_nominal", info,
                                      {20.0, 20.0, 20.0, 25.0, 25.0, 25.0});
  std::copy(thresholds.begin(), thresholds.end(),
            collision_config_.upper_force_thresholds_nominal.begin());

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> FrankaHW::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < joint_names_.size(); i++) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        joint_names_[i], hardware_interface::HW_IF_POSITION, &robot_state_ros_.q[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        joint_names_[i], hardware_interface::HW_IF_VELOCITY, &robot_state_ros_.dq[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        joint_names_[i], hardware_interface::HW_IF_EFFORT, &robot_state_ros_.tau_J[i]));
  }
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      arm_id_, kFrankaStateInterface, "robot_state", nullptr));
  if (model_) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        arm_id_, kFrankaModelInterface, "robot_model", nullptr));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> FrankaHW::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < joint_names_.size(); i++) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        joint_names_[i], hardware_interface::HW_IF_POSITION, &position_joint_command_ros_.q[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        joint_names_[i], hardware_interface::HW_IF_VELOCITY, &velocity_joint_command_ros_.dq[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        joint_names_[i], hardware_interface::HW_IF_EFFORT, &effort_joint_command_ros_.tau_J[i]));
  }
  static const std::array<std::string, 16> pose_names = {
      "00", "01", "02", "03", "04", "05", "06", "07",
      "08", "09", "10", "11", "12", "13", "14", "15"};
  for (size_t i = 0; i < 16; i++) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        arm_id_, std::string(kFrankaPoseCartesianInterface) + "/O_T_EE_" + pose_names[i],
        &pose_cartesian_command_ros_.O_T_EE[i]));
  }
  for (size_t i = 0; i < 2; i++) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        arm_id_, std::string(kFrankaPoseCartesianInterface) + "/elbow_" + std::to_string(i),
        &pose_cartesian_command_ros_.elbow[i]));
  }
  static const std::array<std::string, 6> vel_names = {"0", "1", "2", "3", "4", "5"};
  for (size_t i = 0; i < 6; i++) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        arm_id_, std::string(kFrankaVelocityCartesianInterface) + "/O_dP_EE_" + vel_names[i],
        &velocity_cartesian_command_ros_.O_dP_EE[i]));
  }
  for (size_t i = 0; i < 2; i++) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        arm_id_, std::string(kFrankaVelocityCartesianInterface) + "/elbow_" + std::to_string(i),
        &velocity_cartesian_command_ros_.elbow[i]));
  }
  return command_interfaces;
}

ControlMode FrankaHW::controlModeFromInterfaces(
    const std::vector<std::string>& start_interfaces,
    const std::vector<std::string>& stop_interfaces) const {
  ControlMode mode = current_control_mode_;
  auto remove = [&mode](ControlMode m) { mode &= ~m; };
  auto add = [&mode](ControlMode m) { mode |= m; };
  for (const auto& iface : stop_interfaces) {
    if (iface.find(hardware_interface::HW_IF_POSITION) != std::string::npos &&
        iface.find(kFrankaPoseCartesianInterface) == std::string::npos &&
        iface.find(kFrankaVelocityCartesianInterface) == std::string::npos) {
      remove(ControlMode::JointPosition);
    } else if (iface.find(hardware_interface::HW_IF_VELOCITY) != std::string::npos &&
               iface.find(kFrankaVelocityCartesianInterface) == std::string::npos) {
      remove(ControlMode::JointVelocity);
    } else if (iface.find(hardware_interface::HW_IF_EFFORT) != std::string::npos) {
      remove(ControlMode::JointTorque);
    } else if (iface.find(kFrankaPoseCartesianInterface) != std::string::npos) {
      remove(ControlMode::CartesianPose);
    } else if (iface.find(kFrankaVelocityCartesianInterface) != std::string::npos) {
      remove(ControlMode::CartesianVelocity);
    }
  }
  for (const auto& iface : start_interfaces) {
    if (iface.find(kFrankaPoseCartesianInterface) != std::string::npos) {
      add(ControlMode::CartesianPose);
    } else if (iface.find(kFrankaVelocityCartesianInterface) != std::string::npos) {
      add(ControlMode::CartesianVelocity);
    } else if (iface.find(hardware_interface::HW_IF_POSITION) != std::string::npos) {
      add(ControlMode::JointPosition);
    } else if (iface.find(hardware_interface::HW_IF_VELOCITY) != std::string::npos) {
      add(ControlMode::JointVelocity);
    } else if (iface.find(hardware_interface::HW_IF_EFFORT) != std::string::npos) {
      add(ControlMode::JointTorque);
    }
  }
  return mode;
}

hardware_interface::return_type FrankaHW::prepare_command_mode_switch(
    const std::vector<std::string>& start_interfaces,
    const std::vector<std::string>& stop_interfaces) {
  ControlMode requested_control_mode = controlModeFromInterfaces(start_interfaces, stop_interfaces);
  if (!setRunFunction(requested_control_mode, rate_limiting_, cutoff_frequency_,
                      internal_controller_)) {
    return hardware_interface::return_type::ERROR;
  }
  if (current_control_mode_ != requested_control_mode) {
    RCLCPP_INFO_STREAM(logger_, "FrankaHW: Prepared switching controllers to "
                                   << requested_control_mode << " with parameters "
                                   << "limit_rate=" << rate_limiting_
                                   << ", cutoff_frequency=" << cutoff_frequency_
                                   << ", internal_controller=" << internal_controller_);
    current_control_mode_ = requested_control_mode;
    controller_active_ = false;
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type FrankaHW::perform_command_mode_switch(
    const std::vector<std::string>& /*start_interfaces*/,
    const std::vector<std::string>& /*stop_interfaces*/) {
  if (current_control_mode_ != ControlMode::None) {
    reset();
    controller_active_ = true;
  }
  return hardware_interface::return_type::OK;
}

void FrankaHW::update(const franka::RobotState& robot_state) {
  std::lock_guard<std::mutex> ros_lock(ros_state_mutex_);
  robot_state_ros_ = robot_state;
}

bool FrankaHW::controllerActive() const noexcept {
  return controller_active_;
}

void FrankaHW::control(
    const std::function<bool(const rclcpp::Time&, const rclcpp::Duration&)>& ros_callback) const {
  if (!initialized_) {
    RCLCPP_ERROR(logger_, "FrankaHW: Call to control before initialization!");
    return;
  }
  if (!controller_active_) {
    return;
  }

  franka::Duration last_time = robot_state_ros_.time;

  run_function_(*robot_, [this, ros_callback, &last_time](const franka::RobotState& robot_state,
                                                          franka::Duration time_step) {
    if (last_time != robot_state.time) {
      last_time = robot_state.time;

      return ros_callback(rclcpp::Clock(RCL_SYSTEM_TIME).now(),
                          rclcpp::Duration::from_seconds(time_step.toSec()));
    }
    return true;
  });
}

std::array<double, 7> FrankaHW::getJointPositionCommand() const noexcept {
  return position_joint_command_ros_.q;
}

std::array<double, 7> FrankaHW::getJointVelocityCommand() const noexcept {
  return velocity_joint_command_ros_.dq;
}

std::array<double, 7> FrankaHW::getJointEffortCommand() const noexcept {
  return effort_joint_command_ros_.tau_J;
}

void FrankaHW::reset() {
  std::lock_guard<std::mutex> ros_lock(ros_cmd_mutex_);
  position_joint_command_ros_.q = robot_state_ros_.q_d;
  velocity_joint_command_ros_.dq = robot_state_ros_.dq_d;
  effort_joint_command_ros_.tau_J = robot_state_ros_.tau_J_d;
}

void FrankaHW::checkJointLimits() {
  std::string joint_limits_warning;
  for (const auto& k_joint_name : joint_names_) {
    auto it = info_.joints_map_.find(k_joint_name);
    if (it == info_.joints_map_.end() || !it->second.joint) {
      continue;
    }
    const auto& limits = it->second.joint->limits;
    if (!limits) {
      continue;
    }
    double joint_position = 0.0;
    {
      std::lock_guard<std::mutex> ros_lock(ros_state_mutex_);
      for (size_t i = 0; i < joint_names_.size(); i++) {
        if (joint_names_[i] == k_joint_name) {
          joint_position = robot_state_ros_.q[i];
        }
      }
    }
    double dist = std::min(std::fabs(joint_position - limits->lower),
                           std::fabs(joint_position - limits->upper));
    if (dist < joint_limit_warning_threshold_) {
      joint_limits_warning +=
          "\n\t" + k_joint_name + ": " + toStringWithPrecision(dist * 180 / M_PI) +
          " degrees to joint limits (limits: [" + toStringWithPrecision(limits->lower) + ", " +
          toStringWithPrecision(limits->upper) + "]" +
          " q: " + toStringWithPrecision(joint_position) + ") ";
    }
  }
  if (!joint_limits_warning.empty()) {
    RCLCPP_WARN_THROTTLE(logger_, *get_node()->get_clock(), 5000, "FrankaHW: %s",
                         joint_limits_warning.c_str());
  }
}

franka::Robot& FrankaHW::robot() const {
  if (!initialized_ || !robot_) {
    std::string error_message = "FrankaHW: Attempt to access robot before initialization!";
    RCLCPP_ERROR(logger_, "%s", error_message.c_str());
    throw std::logic_error(error_message);
  }
  return *robot_;
}

franka::Model& FrankaHW::model() const {
  if (!initialized_ || !model_) {
    std::string error_message = "FrankaHW: Attempt to access model before initialization!";
    RCLCPP_ERROR(logger_, "%s", error_message.c_str());
    throw std::logic_error(error_message);
  }
  return *model_;
}

const franka::RobotState& FrankaHW::robotStateRos() const {
  return robot_state_ros_;
}

hardware_interface::return_type FrankaHW::read(const rclcpp::Time& /*time*/,
                                               const rclcpp::Duration& /*period*/) {
  std::lock_guard<std::mutex> ros_lock(ros_state_mutex_);
  std::lock_guard<std::mutex> libfranka_lock(libfranka_state_mutex_);
  robot_state_ros_ = robot_state_libfranka_;
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type FrankaHW::write(const rclcpp::Time& /*time*/,
                                                const rclcpp::Duration& /*period*/) {
  std::lock_guard<std::mutex> ros_lock(ros_cmd_mutex_);
  std::lock_guard<std::mutex> libfranka_lock(libfranka_cmd_mutex_);
  pose_cartesian_command_libfranka_ = pose_cartesian_command_ros_;
  velocity_cartesian_command_libfranka_ = velocity_cartesian_command_ros_;
  effort_joint_command_libfranka_ = effort_joint_command_ros_;
  position_joint_command_libfranka_ = position_joint_command_ros_;
  velocity_joint_command_libfranka_ = velocity_joint_command_ros_;
  return hardware_interface::return_type::OK;
}

bool FrankaHW::setRunFunction(const ControlMode& requested_control_mode,
                              const bool limit_rate,
                              const double cutoff_frequency,
                              const franka::ControllerMode internal_controller) {
  using std::placeholders::_1;
  using std::placeholders::_2;
  using Callback = std::function<bool(const franka::RobotState&, franka::Duration)>;

  switch (requested_control_mode) {
    case ControlMode::None:
      break;
    case ControlMode::JointTorque:
      run_function_ = [=](franka::Robot& robot, Callback ros_callback) {
        robot.control(std::bind(&FrankaHW::controlCallback<franka::Torques>, this,
                                std::cref(effort_joint_command_libfranka_), ros_callback, _1, _2),
                      limit_rate, cutoff_frequency);
      };
      break;
    case ControlMode::JointPosition:
      run_function_ = [=](franka::Robot& robot, Callback ros_callback) {
        robot.control(std::bind(&FrankaHW::controlCallback<franka::JointPositions>, this,
                                std::cref(position_joint_command_libfranka_), ros_callback, _1, _2),
                      internal_controller, limit_rate, cutoff_frequency);
      };
      break;
    case ControlMode::JointVelocity:
      run_function_ = [=](franka::Robot& robot, Callback ros_callback) {
        robot.control(std::bind(&FrankaHW::controlCallback<franka::JointVelocities>, this,
                                std::cref(velocity_joint_command_libfranka_), ros_callback, _1, _2),
                      internal_controller, limit_rate, cutoff_frequency);
      };
      break;
    case ControlMode::CartesianPose:
      run_function_ = [=](franka::Robot& robot, Callback ros_callback) {
        robot.control(std::bind(&FrankaHW::controlCallback<franka::CartesianPose>, this,
                                std::cref(pose_cartesian_command_libfranka_), ros_callback, _1, _2),
                      internal_controller, limit_rate, cutoff_frequency);
      };
      break;
    case ControlMode::CartesianVelocity:
      run_function_ = [=](franka::Robot& robot, Callback ros_callback) {
        robot.control(
            std::bind(&FrankaHW::controlCallback<franka::CartesianVelocities>, this,
                      std::cref(velocity_cartesian_command_libfranka_), ros_callback, _1, _2),
            internal_controller, limit_rate, cutoff_frequency);
      };
      break;
    case (ControlMode::JointTorque | ControlMode::JointPosition):
      run_function_ = [=](franka::Robot& robot, Callback ros_callback) {
        robot.control(std::bind(&FrankaHW::controlCallback<franka::Torques>, this,
                                std::cref(effort_joint_command_libfranka_), ros_callback, _1, _2),
                      std::bind(&FrankaHW::controlCallback<franka::JointPositions>, this,
                                std::cref(position_joint_command_libfranka_), ros_callback, _1, _2),
                      limit_rate, cutoff_frequency);
      };
      break;
    case (ControlMode::JointTorque | ControlMode::JointVelocity):
      run_function_ = [=](franka::Robot& robot, Callback ros_callback) {
        robot.control(std::bind(&FrankaHW::controlCallback<franka::Torques>, this,
                                std::cref(effort_joint_command_libfranka_), ros_callback, _1, _2),
                      std::bind(&FrankaHW::controlCallback<franka::JointVelocities>, this,
                                std::cref(velocity_joint_command_libfranka_), ros_callback, _1, _2),
                      limit_rate, cutoff_frequency);
      };
      break;
    case (ControlMode::JointTorque | ControlMode::CartesianPose):
      run_function_ = [=](franka::Robot& robot, Callback ros_callback) {
        robot.control(std::bind(&FrankaHW::controlCallback<franka::Torques>, this,
                                std::cref(effort_joint_command_libfranka_), ros_callback, _1, _2),
                      std::bind(&FrankaHW::controlCallback<franka::CartesianPose>, this,
                                std::cref(pose_cartesian_command_libfranka_), ros_callback, _1, _2),
                      limit_rate, cutoff_frequency);
      };
      break;
    case (ControlMode::JointTorque | ControlMode::CartesianVelocity):
      run_function_ = [=](franka::Robot& robot, Callback ros_callback) {
        robot.control(
            std::bind(&FrankaHW::controlCallback<franka::Torques>, this,
                      std::cref(effort_joint_command_libfranka_), ros_callback, _1, _2),
            std::bind(&FrankaHW::controlCallback<franka::CartesianVelocities>, this,
                      std::cref(velocity_cartesian_command_libfranka_), ros_callback, _1, _2),
            limit_rate, cutoff_frequency);
      };
      break;
    default:
      RCLCPP_WARN(logger_, "FrankaHW: No valid control mode selected; cannot switch controllers.");
      return false;
  }
  return true;
}

void FrankaHW::initRobot() {
  robot_ = std::make_unique<franka::Robot>(robot_ip_, realtime_config_);
  model_ = std::make_unique<franka::Model>(robot_->loadModel());
  robot_->setCollisionBehavior(collision_config_.lower_torque_thresholds_acceleration,
                               collision_config_.upper_torque_thresholds_acceleration,
                               collision_config_.lower_torque_thresholds_nominal,
                               collision_config_.upper_torque_thresholds_nominal,
                               collision_config_.lower_force_thresholds_acceleration,
                               collision_config_.upper_force_thresholds_acceleration,
                               collision_config_.lower_force_thresholds_nominal,
                               collision_config_.upper_force_thresholds_nominal);
  update(robot_->readOnce());
}

bool FrankaHW::commandHasNaN(const franka::Torques& command) {
  return arrayHasNaN(command.tau_J);
}

bool FrankaHW::commandHasNaN(const franka::JointPositions& command) {
  return arrayHasNaN(command.q);
}

bool FrankaHW::commandHasNaN(const franka::JointVelocities& command) {
  return arrayHasNaN(command.dq);
}

bool FrankaHW::commandHasNaN(const franka::CartesianPose& command) {
  return arrayHasNaN(command.elbow) || arrayHasNaN(command.O_T_EE);
}

bool FrankaHW::commandHasNaN(const franka::CartesianVelocities& command) {
  return arrayHasNaN(command.elbow) || arrayHasNaN(command.O_dP_EE);
}

std::vector<double> FrankaHW::getCollisionThresholds(const std::string& name,
                                                     const hardware_interface::HardwareInfo& info,
                                                     const std::vector<double>& defaults) {
  std::vector<double> thresholds;
  auto it = info.hardware_parameters.find("collision_config/" + name);
  if (it == info.hardware_parameters.end()) {
    RCLCPP_INFO(rclcpp::get_logger("FrankaHW"), "No parameter %s found, using default values",
                name.c_str());
    return defaults;
  }
  std::stringstream ss(it->second);
  double value;
  while (ss >> value) {
    thresholds.push_back(value);
    if (ss.peek() == ',' || ss.peek() == ' ') {
      ss.ignore();
    }
  }
  if (thresholds.size() != defaults.size()) {
    RCLCPP_INFO(rclcpp::get_logger("FrankaHW"), "Parameter %s has wrong size, using defaults",
                name.c_str());
    return defaults;
  }
  return thresholds;
}

}  // namespace franka_hw
