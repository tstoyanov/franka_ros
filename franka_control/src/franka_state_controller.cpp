// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_control/franka_state_controller.h>

#include <cmath>
#include <memory>
#include <mutex>
#include <string>

#include <franka/errors.h>
#include <franka_hw/franka_cartesian_command_interface.h>
#include <franka_msgs/msg/errors.hpp>

#include <pluginlib/class_list_macros.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>


namespace {

tf2::Transform convertArrayToTf(const std::array<double, 16>& transform) {
  tf2::Matrix3x3 rotation(transform[0], transform[4], transform[8], transform[1], transform[5],
                          transform[9], transform[2], transform[6], transform[10]);
  tf2::Vector3 translation(transform[12], transform[13], transform[14]);
  return tf2::Transform(rotation, translation);
}

geometry_msgs::msg::TransformStamped toTransformStampedMsg(const tf2::Transform& transform,
                                                           const rclcpp::Time& stamp,
                                                           const std::string& parent_frame,
                                                           const std::string& child_frame) {
  geometry_msgs::msg::TransformStamped msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = parent_frame;
  msg.child_frame_id = child_frame;
  msg.transform = tf2::toMsg(transform);
  return msg;
}

franka_msgs::msg::Errors errorsToMessage(const franka::Errors& error) {
  franka_msgs::msg::Errors message;
  message.joint_position_limits_violation =
      static_cast<decltype(message.joint_position_limits_violation)>(
          error.joint_position_limits_violation);
  message.cartesian_position_limits_violation =
      static_cast<decltype(message.cartesian_position_limits_violation)>(
          error.cartesian_position_limits_violation);
  message.self_collision_avoidance_violation =
      static_cast<decltype(message.self_collision_avoidance_violation)>(
          error.self_collision_avoidance_violation);
  message.joint_velocity_violation =
      static_cast<decltype(message.joint_velocity_violation)>(error.joint_velocity_violation);
  message.cartesian_velocity_violation =
      static_cast<decltype(message.cartesian_velocity_violation)>(
          error.cartesian_velocity_violation);
  message.force_control_safety_violation =
      static_cast<decltype(message.force_control_safety_violation)>(
          error.force_control_safety_violation);
  message.joint_reflex = static_cast<decltype(message.joint_reflex)>(error.joint_reflex);
  message.cartesian_reflex =
      static_cast<decltype(message.cartesian_reflex)>(error.cartesian_reflex);
  message.max_goal_pose_deviation_violation =
      static_cast<decltype(message.max_goal_pose_deviation_violation)>(
          error.max_goal_pose_deviation_violation);
  message.max_path_pose_deviation_violation =
      static_cast<decltype(message.max_path_pose_deviation_violation)>(
          error.max_path_pose_deviation_violation);
  message.cartesian_velocity_profile_safety_violation =
      static_cast<decltype(message.cartesian_velocity_profile_safety_violation)>(
          error.cartesian_velocity_profile_safety_violation);
  message.joint_position_motion_generator_start_pose_invalid =
      static_cast<decltype(message.joint_position_motion_generator_start_pose_invalid)>(
          error.joint_position_motion_generator_start_pose_invalid);
  message.joint_motion_generator_position_limits_violation =
      static_cast<decltype(message.joint_motion_generator_position_limits_violation)>(
          error.joint_motion_generator_position_limits_violation);
  message.joint_motion_generator_velocity_limits_violation =
      static_cast<decltype(message.joint_motion_generator_velocity_limits_violation)>(
          error.joint_motion_generator_velocity_limits_violation);
  message.joint_motion_generator_velocity_discontinuity =
      static_cast<decltype(message.joint_motion_generator_velocity_discontinuity)>(
          error.joint_motion_generator_velocity_discontinuity);
  message.joint_motion_generator_acceleration_discontinuity =
      static_cast<decltype(message.joint_motion_generator_acceleration_discontinuity)>(
          error.joint_motion_generator_acceleration_discontinuity);
  message.cartesian_position_motion_generator_start_pose_invalid =
      static_cast<decltype(message.cartesian_position_motion_generator_start_pose_invalid)>(
          error.cartesian_position_motion_generator_start_pose_invalid);
  message.cartesian_motion_generator_elbow_limit_violation =
      static_cast<decltype(message.cartesian_motion_generator_elbow_limit_violation)>(
          error.cartesian_motion_generator_elbow_limit_violation);
  message.cartesian_motion_generator_velocity_limits_violation =
      static_cast<decltype(message.cartesian_motion_generator_velocity_limits_violation)>(
          error.cartesian_motion_generator_velocity_limits_violation);
  message.cartesian_motion_generator_velocity_discontinuity =
      static_cast<decltype(message.cartesian_motion_generator_velocity_discontinuity)>(
          error.cartesian_motion_generator_velocity_discontinuity);
  message.cartesian_motion_generator_acceleration_discontinuity =
      static_cast<decltype(message.cartesian_motion_generator_acceleration_discontinuity)>(
          error.cartesian_motion_generator_acceleration_discontinuity);
  message.cartesian_motion_generator_elbow_sign_inconsistent =
      static_cast<decltype(message.cartesian_motion_generator_elbow_sign_inconsistent)>(
          error.cartesian_motion_generator_elbow_sign_inconsistent);
  message.cartesian_motion_generator_start_elbow_invalid =
      static_cast<decltype(message.cartesian_motion_generator_start_elbow_invalid)>(
          error.cartesian_motion_generator_start_elbow_invalid);
  message.cartesian_motion_generator_joint_position_limits_violation =
      static_cast<decltype(message.cartesian_motion_generator_joint_position_limits_violation)>(
          error.cartesian_motion_generator_joint_position_limits_violation);
  message.cartesian_motion_generator_joint_velocity_limits_violation =
      static_cast<decltype(message.cartesian_motion_generator_joint_velocity_limits_violation)>(
          error.cartesian_motion_generator_joint_velocity_limits_violation);
  message.cartesian_motion_generator_joint_velocity_discontinuity =
      static_cast<decltype(message.cartesian_motion_generator_joint_velocity_discontinuity)>(
          error.cartesian_motion_generator_joint_velocity_discontinuity);
  message.cartesian_motion_generator_joint_acceleration_discontinuity =
      static_cast<decltype(message.cartesian_motion_generator_joint_acceleration_discontinuity)>(
          error.cartesian_motion_generator_joint_acceleration_discontinuity);
  message.cartesian_position_motion_generator_invalid_frame =
      static_cast<decltype(message.cartesian_position_motion_generator_invalid_frame)>(
          error.cartesian_position_motion_generator_invalid_frame);
  message.force_controller_desired_force_tolerance_violation =
      static_cast<decltype(message.force_controller_desired_force_tolerance_violation)>(
          error.force_controller_desired_force_tolerance_violation);
  message.controller_torque_discontinuity =
      static_cast<decltype(message.controller_torque_discontinuity)>(
          error.controller_torque_discontinuity);
  message.start_elbow_sign_inconsistent =
      static_cast<decltype(message.start_elbow_sign_inconsistent)>(
          error.start_elbow_sign_inconsistent);
  message.communication_constraints_violation =
      static_cast<decltype(message.communication_constraints_violation)>(
          error.communication_constraints_violation);
  message.power_limit_violation =
      static_cast<decltype(message.power_limit_violation)>(error.power_limit_violation);
  message.joint_p2p_insufficient_torque_for_planning =
      static_cast<decltype(message.joint_p2p_insufficient_torque_for_planning)>(
          error.joint_p2p_insufficient_torque_for_planning);
  message.tau_j_range_violation =
      static_cast<decltype(message.tau_j_range_violation)>(error.tau_j_range_violation);
  message.instability_detected =
      static_cast<decltype(message.instability_detected)>(error.instability_detected);
  return message;
}

}  // anonymous namespace

namespace franka_control {

CallbackReturn FrankaStateController::on_init() {
  try {
    auto_declare("arm_id", std::string());
    auto_declare("publish_rate", 30.0);
    auto_declare("joint_names", std::vector<std::string>());
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception during init: %s", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
FrankaStateController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::NONE;
  return config;
}

controller_interface::InterfaceConfiguration
FrankaStateController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names.push_back(arm_id_ + "/" + franka_hw::kFrankaStateInterface + "/robot_state");
  return config;
}

CallbackReturn FrankaStateController::on_configure(const rclcpp_lifecycle::State& /*previous_state*/) {
  arm_id_ = get_node()->get_parameter("arm_id").as_string();
  if (arm_id_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "FrankaStateController: Could not get parameter arm_id");
    return CallbackReturn::ERROR;
  }
  double publish_rate = get_node()->get_parameter("publish_rate").as_double();
  trigger_publish_ = franka_hw::TriggerRate(publish_rate);

  joint_names_ = get_node()->get_parameter("joint_names").as_string_array();
  if (joint_names_.size() != robot_state_.q.size()) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "FrankaStateController: Invalid or no joint_names parameters provided, aborting "
                 "controller init!");
    return CallbackReturn::ERROR;
  }

  auto node = get_node();
  publisher_transforms_.init(node, "/tf", 1);
  publisher_franka_states_.init(node, "franka_states", 1);
  publisher_joint_states_.init(node, "joint_states", 1);
  publisher_joint_states_desired_.init(node, "joint_states_desired", 1);
  publisher_external_wrench_.init(node, "F_ext", 1);

  {
    std::lock_guard<realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>> lock(
        publisher_joint_states_);
    publisher_joint_states_.msg_.name.resize(joint_names_.size());
    publisher_joint_states_.msg_.position.resize(robot_state_.q.size());
    publisher_joint_states_.msg_.velocity.resize(robot_state_.dq.size());
    publisher_joint_states_.msg_.effort.resize(robot_state_.tau_J.size());
  }
  {
    std::lock_guard<realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>> lock(
        publisher_joint_states_desired_);
    publisher_joint_states_desired_.msg_.name.resize(joint_names_.size());
    publisher_joint_states_desired_.msg_.position.resize(robot_state_.q_d.size());
    publisher_joint_states_desired_.msg_.velocity.resize(robot_state_.dq_d.size());
    publisher_joint_states_desired_.msg_.effort.resize(robot_state_.tau_J_d.size());
  }
  {
    std::lock_guard<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>> lock(
        publisher_transforms_);
    publisher_transforms_.msg_.transforms.resize(2);
    tf2::Quaternion quaternion(0.0, 0.0, 0.0, 1.0);
    tf2::Vector3 translation(0.0, 0.0, 0.05);
    tf2::Transform transform(quaternion, translation);
    publisher_transforms_.msg_.transforms[0] =
        toTransformStampedMsg(transform, node->now(), arm_id_ + "_link8", arm_id_ + "_EE");
    translation = tf2::Vector3(0.0, 0.0, 0.0);
    transform = tf2::Transform(quaternion, translation);
    publisher_transforms_.msg_.transforms[1] =
        toTransformStampedMsg(transform, node->now(), arm_id_ + "_EE", arm_id_ + "_K");
  }
  {
    std::lock_guard<realtime_tools::RealtimePublisher<geometry_msgs::msg::WrenchStamped>> lock(
        publisher_external_wrench_);
    publisher_external_wrench_.msg_.header.frame_id = arm_id_ + "_K";
    publisher_external_wrench_.msg_.wrench.force.x = 0.0;
    publisher_external_wrench_.msg_.wrench.force.y = 0.0;
    publisher_external_wrench_.msg_.wrench.force.z = 0.0;
    publisher_external_wrench_.msg_.wrench.torque.x = 0.0;
    publisher_external_wrench_.msg_.wrench.torque.y = 0.0;
    publisher_external_wrench_.msg_.wrench.torque.z = 0.0;
  }
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type FrankaStateController::update(const rclcpp::Time& time,
                                                                 const rclcpp::Duration& /*period*/) {
  if (trigger_publish_()) {
    robot_state_ = *static_cast<franka::RobotState*>(state_interfaces_[0].get_value());
    publishFrankaStates(time);
    publishTransforms(time);
    publishExternalWrench(time);
    publishJointStates(time);
    sequence_number_++;
  }
  return controller_interface::return_type::OK;
}

void FrankaStateController::publishFrankaStates(const rclcpp::Time& time) {
  if (publisher_franka_states_.trylock()) {
    static_assert(
        sizeof(robot_state_.cartesian_collision) == sizeof(robot_state_.cartesian_contact),
        "Robot state Cartesian members do not have same size");
    static_assert(sizeof(robot_state_.cartesian_collision) == sizeof(robot_state_.K_F_ext_hat_K),
                  "Robot state Cartesian members do not have same size");
    static_assert(sizeof(robot_state_.cartesian_collision) == sizeof(robot_state_.O_F_ext_hat_K),
                  "Robot state Cartesian members do not have same size");
    static_assert(sizeof(robot_state_.cartesian_collision) == sizeof(robot_state_.O_dP_EE_d),
                  "Robot state Cartesian members do not have same size");
    static_assert(sizeof(robot_state_.cartesian_collision) == sizeof(robot_state_.O_dP_EE_c),
                  "Robot state Cartesian members do not have same size");
    static_assert(sizeof(robot_state_.cartesian_collision) == sizeof(robot_state_.O_ddP_EE_c),
                  "Robot state Cartesian members do not have same size");
    for (size_t i = 0; i < robot_state_.cartesian_collision.size(); i++) {
      publisher_franka_states_.msg_.cartesian_collision[i] = robot_state_.cartesian_collision[i];
      publisher_franka_states_.msg_.cartesian_contact[i] = robot_state_.cartesian_contact[i];
      publisher_franka_states_.msg_.K_F_ext_hat_K[i] = robot_state_.K_F_ext_hat_K[i];
      publisher_franka_states_.msg_.O_F_ext_hat_K[i] = robot_state_.O_F_ext_hat_K[i];
      publisher_franka_states_.msg_.O_dP_EE_d[i] = robot_state_.O_dP_EE_d[i];
      publisher_franka_states_.msg_.O_dP_EE_c[i] = robot_state_.O_dP_EE_c[i];
      publisher_franka_states_.msg_.O_ddP_EE_c[i] = robot_state_.O_ddP_EE_c[i];
    }

    static_assert(sizeof(robot_state_.q) == sizeof(robot_state_.q_d),
                  "Robot state joint members do not have same size");
    static_assert(sizeof(robot_state_.q) == sizeof(robot_state_.dq),
                  "Robot state joint members do not have same size");
    static_assert(sizeof(robot_state_.q) == sizeof(robot_state_.dq_d),
                  "Robot state joint members do not have same size");
    static_assert(sizeof(robot_state_.q) == sizeof(robot_state_.ddq_d),
                  "Robot state joint members do not have same size");
    static_assert(sizeof(robot_state_.q) == sizeof(robot_state_.tau_J),
                  "Robot state joint members do not have same size");
    static_assert(sizeof(robot_state_.q) == sizeof(robot_state_.dtau_J),
                  "Robot state joint members do not have same size");
    static_assert(sizeof(robot_state_.q) == sizeof(robot_state_.tau_J_d),
                  "Robot state joint members do not have same size");
    static_assert(sizeof(robot_state_.q) == sizeof(robot_state_.theta),
                  "Robot state joint members do not have same size");
    static_assert(sizeof(robot_state_.q) == sizeof(robot_state_.dtheta),
                  "Robot state joint members do not have same size");
    static_assert(sizeof(robot_state_.q) == sizeof(robot_state_.joint_collision),
                  "Robot state joint members do not have same size");
    static_assert(sizeof(robot_state_.q) == sizeof(robot_state_.joint_contact),
                  "Robot state joint members do not have same size");
    static_assert(sizeof(robot_state_.q) == sizeof(robot_state_.tau_ext_hat_filtered),
                  "Robot state joint members do not have same size");
    for (size_t i = 0; i < robot_state_.q.size(); i++) {
      publisher_franka_states_.msg_.q[i] = robot_state_.q[i];
      publisher_franka_states_.msg_.q_d[i] = robot_state_.q_d[i];
      publisher_franka_states_.msg_.dq[i] = robot_state_.dq[i];
      publisher_franka_states_.msg_.dq_d[i] = robot_state_.dq_d[i];
      publisher_franka_states_.msg_.ddq_d[i] = robot_state_.ddq_d[i];
      publisher_franka_states_.msg_.tau_J[i] = robot_state_.tau_J[i];
      publisher_franka_states_.msg_.dtau_J[i] = robot_state_.dtau_J[i];
      publisher_franka_states_.msg_.tau_J_d[i] = robot_state_.tau_J_d[i];
      publisher_franka_states_.msg_.theta[i] = robot_state_.theta[i];
      publisher_franka_states_.msg_.dtheta[i] = robot_state_.dtheta[i];
      publisher_franka_states_.msg_.joint_collision[i] = robot_state_.joint_collision[i];
      publisher_franka_states_.msg_.joint_contact[i] = robot_state_.joint_contact[i];
      publisher_franka_states_.msg_.tau_ext_hat_filtered[i] = robot_state_.tau_ext_hat_filtered[i];
    }

    static_assert(sizeof(robot_state_.elbow) == sizeof(robot_state_.elbow_d),
                  "Robot state elbow configuration members do not have same size");
    static_assert(sizeof(robot_state_.elbow) == sizeof(robot_state_.elbow_c),
                  "Robot state elbow configuration members do not have same size");
    static_assert(sizeof(robot_state_.elbow) == sizeof(robot_state_.delbow_c),
                  "Robot state elbow configuration members do not have same size");
    static_assert(sizeof(robot_state_.elbow) == sizeof(robot_state_.ddelbow_c),
                  "Robot state elbow configuration members do not have same size");

    for (size_t i = 0; i < robot_state_.elbow.size(); i++) {
      publisher_franka_states_.msg_.elbow[i] = robot_state_.elbow[i];
      publisher_franka_states_.msg_.elbow_d[i] = robot_state_.elbow_d[i];
      publisher_franka_states_.msg_.elbow_c[i] = robot_state_.elbow_c[i];
      publisher_franka_states_.msg_.delbow_c[i] = robot_state_.delbow_c[i];
      publisher_franka_states_.msg_.ddelbow_c[i] = robot_state_.ddelbow_c[i];
    }

    static_assert(sizeof(robot_state_.O_T_EE) == sizeof(robot_state_.F_T_EE),
                  "Robot state transforms do not have same size");
    static_assert(sizeof(robot_state_.O_T_EE) == sizeof(robot_state_.EE_T_K),
                  "Robot state transforms do not have same size");
    static_assert(sizeof(robot_state_.O_T_EE) == sizeof(robot_state_.O_T_EE_d),
                  "Robot state transforms do not have same size");
    static_assert(sizeof(robot_state_.O_T_EE) == sizeof(robot_state_.O_T_EE_c),
                  "Robot state transforms do not have same size");
    for (size_t i = 0; i < robot_state_.O_T_EE.size(); i++) {
      publisher_franka_states_.msg_.O_T_EE[i] = robot_state_.O_T_EE[i];
      publisher_franka_states_.msg_.F_T_EE[i] = robot_state_.F_T_EE[i];
      publisher_franka_states_.msg_.EE_T_K[i] = robot_state_.EE_T_K[i];
      publisher_franka_states_.msg_.O_T_EE_d[i] = robot_state_.O_T_EE_d[i];
      publisher_franka_states_.msg_.O_T_EE_c[i] = robot_state_.O_T_EE_c[i];
    }
    publisher_franka_states_.msg_.m_ee = robot_state_.m_ee;
    publisher_franka_states_.msg_.m_load = robot_state_.m_load;
    publisher_franka_states_.msg_.m_total = robot_state_.m_total;

    for (size_t i = 0; i < robot_state_.I_load.size(); i++) {
      publisher_franka_states_.msg_.I_ee[i] = robot_state_.I_ee[i];
      publisher_franka_states_.msg_.I_load[i] = robot_state_.I_load[i];
      publisher_franka_states_.msg_.I_total[i] = robot_state_.I_total[i];
    }

    for (size_t i = 0; i < robot_state_.F_x_Cload.size(); i++) {
      publisher_franka_states_.msg_.F_x_Cee[i] = robot_state_.F_x_Cee[i];
      publisher_franka_states_.msg_.F_x_Cload[i] = robot_state_.F_x_Cload[i];
      publisher_franka_states_.msg_.F_x_Ctotal[i] = robot_state_.F_x_Ctotal[i];
    }

    publisher_franka_states_.msg_.time = robot_state_.time.toSec();
    publisher_franka_states_.msg_.control_command_success_rate =
        robot_state_.control_command_success_rate;
    publisher_franka_states_.msg_.current_errors = errorsToMessage(robot_state_.current_errors);
    publisher_franka_states_.msg_.last_motion_errors =
        errorsToMessage(robot_state_.last_motion_errors);

    switch (robot_state_.robot_mode) {
      case franka::RobotMode::kOther:
        publisher_franka_states_.msg_.robot_mode = franka_msgs::msg::FrankaState::ROBOT_MODE_OTHER;
        break;

      case franka::RobotMode::kIdle:
        publisher_franka_states_.msg_.robot_mode = franka_msgs::msg::FrankaState::ROBOT_MODE_IDLE;
        break;

      case franka::RobotMode::kMove:
        publisher_franka_states_.msg_.robot_mode = franka_msgs::msg::FrankaState::ROBOT_MODE_MOVE;
        break;

      case franka::RobotMode::kGuiding:
        publisher_franka_states_.msg_.robot_mode = franka_msgs::msg::FrankaState::ROBOT_MODE_GUIDING;
        break;

      case franka::RobotMode::kReflex:
        publisher_franka_states_.msg_.robot_mode = franka_msgs::msg::FrankaState::ROBOT_MODE_REFLEX;
        break;

      case franka::RobotMode::kUserStopped:
        publisher_franka_states_.msg_.robot_mode =
            franka_msgs::msg::FrankaState::ROBOT_MODE_USER_STOPPED;
        break;

      case franka::RobotMode::kAutomaticErrorRecovery:
        publisher_franka_states_.msg_.robot_mode =
            franka_msgs::msg::FrankaState::ROBOT_MODE_AUTOMATIC_ERROR_RECOVERY;
        break;
    }

    publisher_franka_states_.msg_.header.stamp = time;
    publisher_franka_states_.msg_.header.stamp = time;
    publisher_franka_states_.unlockAndPublish();
  }
}

void FrankaStateController::publishJointStates(const rclcpp::Time& time) {
  if (publisher_joint_states_.trylock()) {
    static_assert(sizeof(robot_state_.q) == sizeof(robot_state_.dq),
                  "Robot state joint members do not have same size");
    static_assert(sizeof(robot_state_.q) == sizeof(robot_state_.tau_J),
                  "Robot state joint members do not have same size");
    for (size_t i = 0; i < robot_state_.q.size(); i++) {
      publisher_joint_states_.msg_.name[i] = joint_names_[i];
      publisher_joint_states_.msg_.position[i] = robot_state_.q[i];
      publisher_joint_states_.msg_.velocity[i] = robot_state_.dq[i];
      publisher_joint_states_.msg_.effort[i] = robot_state_.tau_J[i];
    }
    publisher_joint_states_.msg_.header.stamp = time;
    
    publisher_joint_states_.unlockAndPublish();
  }
  if (publisher_joint_states_desired_.trylock()) {
    static_assert(sizeof(robot_state_.q_d) == sizeof(robot_state_.dq_d),
                  "Robot state joint members do not have same size");
    static_assert(sizeof(robot_state_.q_d) == sizeof(robot_state_.tau_J_d),
                  "Robot state joint members do not have same size");
    for (size_t i = 0; i < robot_state_.q_d.size(); i++) {
      publisher_joint_states_desired_.msg_.name[i] = joint_names_[i];
      publisher_joint_states_desired_.msg_.position[i] = robot_state_.q_d[i];
      publisher_joint_states_desired_.msg_.velocity[i] = robot_state_.dq_d[i];
      publisher_joint_states_desired_.msg_.effort[i] = robot_state_.tau_J_d[i];
    }
    publisher_joint_states_desired_.msg_.header.stamp = time;
    
    publisher_joint_states_desired_.unlockAndPublish();
  }
}

void FrankaStateController::publishTransforms(const rclcpp::Time& time) {
  if (publisher_transforms_.trylock()) {
    publisher_transforms_.msg_.transforms[0] =
        toTransformStampedMsg(convertArrayToTf(robot_state_.F_T_EE), time, arm_id_ + "_link8",
                              arm_id_ + "_EE");
    publisher_transforms_.msg_.transforms[1] =
        toTransformStampedMsg(convertArrayToTf(robot_state_.EE_T_K), time, arm_id_ + "_EE",
                              arm_id_ + "_K");
    publisher_transforms_.unlockAndPublish();
  }
}

void FrankaStateController::publishExternalWrench(const rclcpp::Time& time) {
  if (publisher_external_wrench_.trylock()) {
    publisher_external_wrench_.msg_.header.frame_id = arm_id_ + "_K";
    publisher_external_wrench_.msg_.header.stamp = time;
    publisher_external_wrench_.msg_.wrench.force.x = robot_state_.K_F_ext_hat_K[0];
    publisher_external_wrench_.msg_.wrench.force.y = robot_state_.K_F_ext_hat_K[1];
    publisher_external_wrench_.msg_.wrench.force.z = robot_state_.K_F_ext_hat_K[2];
    publisher_external_wrench_.msg_.wrench.torque.x = robot_state_.K_F_ext_hat_K[3];
    publisher_external_wrench_.msg_.wrench.torque.y = robot_state_.K_F_ext_hat_K[4];
    publisher_external_wrench_.msg_.wrench.torque.z = robot_state_.K_F_ext_hat_K[5];
    publisher_external_wrench_.unlockAndPublish();
  }
}

}  // namespace franka_control

PLUGINLIB_EXPORT_CLASS(franka_control::FrankaStateController, controller_interface::ControllerInterface)
