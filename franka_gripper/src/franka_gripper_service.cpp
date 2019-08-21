// #include <ros/ros.h>
#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/spinner.h>
// #include "ros/platform.h"
#include <franka_gripper/franka_gripper.h>

#include "franka_gripper/gripper_service.h"

std::string robot_ip;

bool move(franka_gripper::gripper_service::Request  &req,
         franka_gripper::gripper_service::Response &res)
{
  franka::Gripper gripper(robot_ip);
  // res.success = franka_gripper::service_move(gripper, (double)req.width, (double)req.speed);
  res.success = franka_gripper::service_move(gripper, 0.08, 0.05);
  res.error = "";
  gripper.~Gripper();
  ROS_INFO("request: width=%f, speed=%f", (double)req.width, (double)req.speed);
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "franka_gripper_move_service_node");
  ros::NodeHandle node_handle("~");
  if (!node_handle.getParam("robot_ip", robot_ip)) {
    ROS_ERROR("franka_gripper_node: Could not parse robot_ip parameter");
    return -1;
  }

  ros::ServiceServer service = node_handle.advertiseService("franka_gripper_move_service", move);
  ROS_INFO("Ready to move the gripper.");
  ros::spin();

  return 0;
}