// Copyright (c) 2019 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_hw/franka_combinable_hw.h>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <std_msgs/msg/bool.hpp>

#include <franka_hw/services.h>

namespace franka_hw {

FrankaCombinableHW::FrankaCombinableHW() : has_error_(false), error_recovered_(false) {}

void FrankaCombinableHW::initRobot() {
  FrankaHW::initRobot();
  control_loop_thread_ = std::make_unique<std::thread>(&FrankaCombinableHW::controlLoop, this);
}

void FrankaCombinableHW::publishErrorState(const bool error) {
  if (has_error_pub_) {
    std_msgs::msg::Bool msg;
    msg.data = static_cast<bool>(error);
    has_error_pub_->publish(msg);
  }
}

void FrankaCombinableHW::controlLoop() {
  rclcpp::Clock clock(RCL_SYSTEM_TIME);
  while (rclcpp::ok()) {
    rclcpp::Time last_time = clock.now();

    // Wait until controller has been activated or error has been recovered
    while (!controllerActive() || has_error_) {
      if (!controllerActive()) {
        RCLCPP_DEBUG_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                              "FrankaCombinableHW::%s::control_loop(): controller is not active.",
                              arm_id_.c_str());
      }
      if (has_error_) {
        RCLCPP_DEBUG_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                              "FrankaCombinableHW::%s::control_loop(): an error has occured.",
                              arm_id_.c_str());
      }

      checkJointLimits();

      {
        std::lock_guard<std::mutex> ros_state_lock(ros_state_mutex_);
        std::lock_guard<std::mutex> libfranka_state_lock(libfranka_state_mutex_);
        robot_state_libfranka_ = robot_->readOnce();
        robot_state_ros_ = robot_state_libfranka_;
      }

      if (!rclcpp::ok()) {
        return;
      }
    }
    RCLCPP_INFO(get_node()->get_logger(), "FrankaCombinableHW::%s::control_loop(): controller is active.",
                arm_id_.c_str());

    // Reset commands
    {
      std::lock_guard<std::mutex> command_lock(libfranka_cmd_mutex_);
      effort_joint_command_libfranka_ = franka::Torques({0., 0., 0., 0., 0., 0., 0.});
    }

    try {
      control();
    } catch (const franka::ControlException& e) {
      // Reflex could be caught and it needs to wait for automatic error recovery
      RCLCPP_ERROR(get_node()->get_logger(), "%s: %s", arm_id_.c_str(), e.what());
      has_error_ = true;
      publishErrorState(has_error_);
    }
  }
}

void FrankaCombinableHW::setupServicesAndActionServers(rclcpp::Node::SharedPtr node) {
  setupServices(*robot_, node, services_);

  auto handle_goal = [this](const rclcpp_action::GoalUUID&,
                            std::shared_ptr<const franka_msgs::action::ErrorRecovery::Goal>) {
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  };
  auto handle_cancel = [](std::shared_ptr<rclcpp_action::ServerGoalHandle<
                          franka_msgs::action::ErrorRecovery>>) {
    return rclcpp_action::CancelResponse::ACCEPT;
  };
  auto handle_accepted = [this](std::shared_ptr<rclcpp_action::ServerGoalHandle<
                                franka_msgs::action::ErrorRecovery>> goal_handle) {
    std::thread([this, goal_handle]() {
      auto result = std::make_shared<franka_msgs::action::ErrorRecovery::Result>();
      try {
        robot_->automaticErrorRecovery();
        // error recovered => reset controller
        if (has_error_) {
          error_recovered_ = true;
        }
        has_error_ = false;
        publishErrorState(has_error_);
        goal_handle->succeed(result);
      } catch (const franka::Exception& ex) {
        RCLCPP_ERROR(rclcpp::get_logger("FrankaCombinableHW"), "error recovery failed: %s", ex.what());
        goal_handle->abort(result);
      }
    }).detach();
  };
  recovery_action_server_ = rclcpp_action::create_server<franka_msgs::action::ErrorRecovery>(
      node, "error_recovery", handle_goal, handle_cancel, handle_accepted);
}

CallbackReturn FrankaCombinableHW::on_init(const hardware_interface::HardwareInfo& info) {
  if (FrankaHW::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  has_error_pub_ = get_node()->create_publisher<std_msgs::msg::Bool>("has_error", 1);
  publishErrorState(has_error_);
  setupServicesAndActionServers(get_node());
  return CallbackReturn::SUCCESS;
}

void FrankaCombinableHW::control(
    const std::function<bool(const rclcpp::Time&, const rclcpp::Duration&)>& /*ros_callback*/) const {
  if (!controller_active_) {
    return;
  }
  auto empty_method = [](const franka::RobotState&, franka::Duration) { return true; };
  run_function_(*robot_, empty_method);
}

hardware_interface::return_type FrankaCombinableHW::read(const rclcpp::Time& time,
                                                          const rclcpp::Duration& period) {
  return FrankaHW::read(time, period);
}

hardware_interface::return_type FrankaCombinableHW::write(const rclcpp::Time& time,
                                                           const rclcpp::Duration& period) {
  return FrankaHW::write(time, period);
}

std::string FrankaCombinableHW::getArmID() const noexcept {
  return arm_id_;
}

void FrankaCombinableHW::triggerError() {
  has_error_ = true;
  publishErrorState(has_error_);
}

bool FrankaCombinableHW::hasError() const noexcept {
  return has_error_;
}

void FrankaCombinableHW::resetError() {
  try {
    robot_->automaticErrorRecovery();
    if (has_error_) {
      error_recovered_ = true;
      controller_needs_reset_ = true;
    }
    has_error_ = false;
    publishErrorState(has_error_);
  } catch (const franka::Exception& ex) {
    RCLCPP_ERROR(get_node()->get_logger(), "FrankaCombinableHW: error recovery failed: %s",
                 ex.what());
    throw;
  }
}

bool FrankaCombinableHW::controllerNeedsReset() const noexcept {
  return controller_needs_reset_.exchange(false);
}

bool FrankaCombinableHW::setRunFunction(const ControlMode& requested_control_mode,
                                        const bool limit_rate,
                                        const double cutoff_frequency,
                                        const franka::ControllerMode /*internal_controller*/) {
  using Callback = std::function<bool(const franka::RobotState&, franka::Duration)>;

  if (requested_control_mode == ControlMode::None) {
    return true;
  }
  if (requested_control_mode == ControlMode::JointTorque) {
    run_function_ = [this, limit_rate, cutoff_frequency](franka::Robot& robot,
                                                         Callback /*callback*/) {
      robot.control(std::bind(&FrankaCombinableHW::libfrankaUpdateCallback<franka::Torques>, this,
                              std::cref(effort_joint_command_libfranka_), std::placeholders::_1,
                              std::placeholders::_2),
                    limit_rate, cutoff_frequency);
    };
    return true;
  }

  RCLCPP_ERROR(get_node()->get_logger(),
               "FrankaCombinableHW: No valid control mode selected; cannot set run function.");
  return false;
}

}  // namespace franka_hw

PLUGINLIB_EXPORT_CLASS(franka_hw::FrankaCombinableHW, hardware_interface::SystemInterface)
