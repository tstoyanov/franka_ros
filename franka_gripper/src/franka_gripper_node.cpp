// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <franka/gripper_state.h>
#include <franka_gripper/franka_gripper.h>

namespace {

template <typename T_action>
void handleErrors(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<T_action>> goal_handle,
    std::function<bool(const typename T_action::Goal&)> handler,
    const typename T_action::Goal& goal) {
  auto result = std::make_shared<typename T_action::Result>();
  try {
    result->success = handler(goal);
    goal_handle->succeed(result);
  } catch (const franka::Exception& ex) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("franka_gripper_node"), "" << ex.what());
    result->success = false;
    result->error = ex.what();
    goal_handle->abort(result);
  }
}

template <typename T_action>
rclcpp_action::Server<T_action>::SharedPtr makeActionServer(
    rclcpp::Node::SharedPtr node,
    const std::string& name,
    std::function<bool(const typename T_action::Goal&)> handler) {
  auto handle_goal = [](const rclcpp_action::GoalUUID&,
                        std::shared_ptr<const typename T_action::Goal>) {
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  };
  auto handle_cancel = [](std::shared_ptr<rclcpp_action::ServerGoalHandle<T_action>>) {
    return rclcpp_action::CancelResponse::ACCEPT;
  };
  auto handle_accepted = [handler](std::shared_ptr<rclcpp_action::ServerGoalHandle<T_action>> goal_handle) {
    std::thread([goal_handle, handler, goal = goal_handle->get_goal()]() {
      handleErrors<T_action>(goal_handle, handler, *goal);
    }).detach();
  };
  return rclcpp_action::create_server<T_action>(node, name, handle_goal, handle_cancel,
                                                 handle_accepted);
}

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("franka_gripper_node");
  node->declare_parameter("robot_ip", std::string());
  node->declare_parameter("default_speed", 0.1);
  node->declare_parameter("default_grasp_epsilon.inner", 0.005);
  node->declare_parameter("default_grasp_epsilon.outer", 0.005);
  node->declare_parameter("publish_rate", 30.0);
  node->declare_parameter("joint_names", std::vector<std::string>());

  std::string robot_ip = node->get_parameter("robot_ip").as_string();
  if (robot_ip.empty()) {
    RCLCPP_ERROR(node->get_logger(), "franka_gripper_node: Could not parse robot_ip parameter");
    return -1;
  }

  double default_speed = node->get_parameter("default_speed").as_double();
  RCLCPP_INFO_STREAM(node->get_logger(), "franka_gripper_node: Found default_speed " << default_speed);

  franka_gripper::GraspEpsilon default_grasp_epsilon;
  default_grasp_epsilon.inner = node->get_parameter("default_grasp_epsilon.inner").as_double();
  default_grasp_epsilon.outer = node->get_parameter("default_grasp_epsilon.outer").as_double();

  franka::Gripper gripper(robot_ip);

  auto homing_handler = [&gripper](const franka_gripper::HomingGoal& goal) {
    return franka_gripper::homing(gripper, goal);
  };
  auto stop_handler = [&gripper](const franka_gripper::StopGoal& goal) {
    return franka_gripper::stop(gripper, goal);
  };
  auto grasp_handler = [&gripper](const franka_gripper::GraspGoal& goal) {
    return franka_gripper::grasp(gripper, goal);
  };
  auto move_handler = [&gripper](const franka_gripper::MoveGoal& goal) {
    return franka_gripper::move(gripper, goal);
  };

  auto homing_server = makeActionServer<franka_gripper::action::Homing>(node, "homing", homing_handler);
  auto stop_server = makeActionServer<franka_gripper::action::Stop>(node, "stop", stop_handler);
  auto move_server = makeActionServer<franka_gripper::action::Move>(node, "move", move_handler);
  auto grasp_server = makeActionServer<franka_gripper::action::Grasp>(node, "grasp", grasp_handler);

  auto handle_gripper_goal = [](const rclcpp_action::GoalUUID&,
                                std::shared_ptr<const control_msgs::action::GripperCommand::Goal>) {
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  };
  auto handle_gripper_cancel = [](std::shared_ptr<
                                  rclcpp_action::ServerGoalHandle<control_msgs::action::GripperCommand>>) {
    return rclcpp_action::CancelResponse::ACCEPT;
  };
  auto handle_gripper_accepted =
      [&gripper, default_grasp_epsilon, default_speed](
          std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::GripperCommand>>
              goal_handle) {
        std::thread([&gripper, default_grasp_epsilon, default_speed, goal_handle]() {
          franka_gripper::gripperCommandExecuteCallback(gripper, default_grasp_epsilon, default_speed,
                                                        goal_handle, goal_handle->get_goal());
        }).detach();
      };
  auto gripper_command_server = rclcpp_action::create_server<control_msgs::action::GripperCommand>(
      node, "gripper_action", handle_gripper_goal, handle_gripper_cancel, handle_gripper_accepted);

  double publish_rate = node->get_parameter("publish_rate").as_double();
  std::vector<std::string> joint_names = node->get_parameter("joint_names").as_string_array();
  if (joint_names.size() != 2) {
    RCLCPP_ERROR(node->get_logger(), "franka_gripper_node: Got wrong number of joint_names!");
    return -1;
  }

  franka::GripperState gripper_state;
  std::mutex gripper_state_mutex;
  std::thread read_thread([&gripper_state, &gripper, &gripper_state_mutex]() {
    rclcpp::Rate read_rate(10);
    while (rclcpp::ok()) {
      {
        std::lock_guard<std::mutex> _(gripper_state_mutex);
        franka_gripper::updateGripperState(gripper, &gripper_state);
      }
      read_rate.sleep();
    }
  });

  auto gripper_state_publisher = node->create_publisher<sensor_msgs::msg::JointState>("joint_states", 1);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  rclcpp::Rate rate(publish_rate);
  std::thread spin_thread([&executor]() { executor.spin(); });
  while (rclcpp::ok()) {
    if (gripper_state_mutex.try_lock()) {
      sensor_msgs::msg::JointState joint_states;
      joint_states.header.stamp = node->now();
      joint_states.name.push_back(joint_names[0]);
      joint_states.name.push_back(joint_names[1]);
      joint_states.position.push_back(gripper_state.width * 0.5);
      joint_states.position.push_back(gripper_state.width * 0.5);
      joint_states.velocity.push_back(0.0);
      joint_states.velocity.push_back(0.0);
      joint_states.effort.push_back(0.0);
      joint_states.effort.push_back(0.0);
      gripper_state_publisher->publish(joint_states);
      gripper_state_mutex.unlock();
    }
    rate.sleep();
  }
  executor.cancel();
  spin_thread.join();
  read_thread.join();
  rclcpp::shutdown();
  return 0;
}
