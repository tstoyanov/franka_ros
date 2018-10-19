// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_example_controllers/first_pose_ctrl.h>

#include <cmath>
#include <stdexcept>
#include <string>
#include <eigen3/Eigen/Eigen>

#include <controller_interface/controller_base.h>
#include <franka_hw/franka_cartesian_command_interface.h>
#include <hardware_interface/hardware_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <geometry_msgs/Pose.h>

namespace franka_example_controllers {

Eigen::Vector3d     pos;
Eigen::Quaterniond  ori_quat;


void posture__Callback(const geometry_msgs::Pose::ConstPtr& msg)
{

  pos << msg->position.x, msg->position.y, msg->position.z;
  ori_quat.x() = msg->orientation.x;
  ori_quat.y() = msg->orientation.y;
  ori_quat.z() = msg->orientation.z;
  ori_quat.w() = msg->orientation.w;

}

bool FirstPoseCtrl::init(hardware_interface::RobotHW* robot_hardware,
                                          ros::NodeHandle& node_handle) {
  cartesian_pose_interface_ = robot_hardware->get<franka_hw::FrankaPoseCartesianInterface>();
  if (cartesian_pose_interface_ == nullptr) {
    ROS_ERROR(
        "FirstPoseCtrl: Could not get Cartesian Pose "
        "interface from hardware");
    return false;
  }

  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR("FirstPoseCtrl: Could not get parameter arm_id");
    return false;
  }

  try {
    cartesian_pose_handle_.reset(new franka_hw::FrankaCartesianPoseHandle(
        cartesian_pose_interface_->getHandle(arm_id + "_robot")));
  } catch (const hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR_STREAM(
        "FirstPoseCtrl: Exception getting Cartesian handle: " << e.what());
    return false;
  }

  auto state_interface = robot_hardware->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR("FirstPoseCtrl: Could not get state interface from hardware");
    return false;
  }

  try {
    auto state_handle = state_interface->getHandle(arm_id + "_robot");

    std::array<double, 7> q_start{{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
    for (size_t i = 0; i < q_start.size(); i++) {
      if (std::abs(state_handle.getRobotState().q_d[i] - q_start[i]) > 0.1) {
        ROS_ERROR_STREAM(
            "FirstPoseCtrl: Robot is not in the expected starting position for "
            "running this example. Run `roslaunch franka_example_controllers move_to_start.launch "
            "robot_ip:=<robot-ip> load_gripper:=<has-attached-gripper>` first.");
        return false;
      }
    }
  } catch (const hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR_STREAM(
        "FirstPoseCtrl: Exception getting state handle: " << e.what());
    return false;
  }

    // ------------------------------------------------------------------------------------- Subscribe to topics 
    ros::Subscriber   sub_posture   = node_handle.subscribe("desired_posture", 1000, posture__Callback);


  return true;
}

void FirstPoseCtrl::starting(const ros::Time& /* time */) {
  initial_pose_ = cartesian_pose_handle_->getRobotState().O_T_EE_d;
  elapsed_time_ = ros::Duration(0.0);
}

void FirstPoseCtrl::update(const ros::Time& /* time */,
                                            const ros::Duration& period) {
  std::array<double, 16> new_pose = initial_pose_;
  new_pose[12] = pos(0);
  new_pose[13] = pos(1);
  new_pose[14] = pos(2);
  cartesian_pose_handle_->setCommand(new_pose);

  std::cout << new_pose[12] << new_pose[13] << new_pose[14] << std::endl;
}

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::FirstPoseCtrl,
                       controller_interface::ControllerBase)
