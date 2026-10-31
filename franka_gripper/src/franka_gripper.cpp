// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_gripper/franka_gripper.h>

#include <cmath>
#include <functional>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include <franka/exception.h>
#include <franka/gripper_state.h>

namespace franka_gripper {

bool updateGripperState(const franka::Gripper& gripper, franka::GripperState* state) {
  try {
    *state = gripper.readOnce();
  } catch (const franka::Exception& ex) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("GripperServer"),
                        "GripperServer: Exception reading gripper state: " << ex.what());
    return false;
  }
  return true;
}

void gripperCommandExecuteCallback(
    const franka::Gripper& gripper,
    const GraspEpsilon& grasp_epsilon,
    double default_speed,
    std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::GripperCommand>>
        goal_handle,
    const std::shared_ptr<const control_msgs::action::GripperCommand::Goal>& goal) {
  auto gripper_command_handler = [&goal, &grasp_epsilon, default_speed, &gripper]() {
    // HACK: As one gripper finger is <mimic>, MoveIt!'s trajectory execution manager
    // only sends us the width of one finger. Multiply by 2 to get the intended width.
    double target_width = 2 * goal->command.position;

    franka::GripperState state = gripper.readOnce();
    if (target_width > state.max_width || target_width < 0.0) {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("GripperServer"),
                          "GripperServer: Commanding out of range width! max_width = "
                              << state.max_width << " command = " << target_width);
      return false;
    }
    constexpr double kSamePositionThreshold = 1e-4;
    if (std::abs(target_width - state.width) < kSamePositionThreshold) {
      return true;
    }
    if (target_width >= state.width) {
      return gripper.move(target_width, default_speed);
    }
    return gripper.grasp(target_width, default_speed, goal->command.max_effort,
                         grasp_epsilon.inner, grasp_epsilon.outer);
  };

  auto result = std::make_shared<control_msgs::action::GripperCommand::Result>();
  try {
    if (gripper_command_handler()) {
      franka::GripperState state;
      if (updateGripperState(gripper, &state)) {
        result->effort = 0.0;
        result->position = state.width;
        result->reached_goal = true;
        result->stalled = false;
        goal_handle->succeed(result);
        return;
      }
    }
  } catch (const franka::Exception& ex) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("GripperServer"), "" << ex.what());
  }
  goal_handle->abort(result);
}

bool move(const franka::Gripper& gripper, const MoveGoal& goal) {
  return gripper.move(goal.width, goal.speed);
}

bool homing(const franka::Gripper& gripper, const HomingGoal& /*goal*/) {
  return gripper.homing();
}

bool stop(const franka::Gripper& gripper, const StopGoal& /*goal*/) {
  return gripper.stop();
}

bool grasp(const franka::Gripper& gripper, const GraspGoal& goal) {
  return gripper.grasp(goal.width, goal.speed, goal.force, goal.epsilon.inner,
                       goal.epsilon.outer);
}

}  // namespace franka_gripper
