// #include <ros/ros.h>
#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/spinner.h>
#include "ros/platform.h"
#include "franka_gripper/gripper_service.h"

bool move(franka_gripper::gripper_service::Request  &req,
         franka_gripper::gripper_service::Response &res)
{
  res.success = true;
  res.error = "";
  ROS_INFO("request: width=%d, speed=%d", (double)req.width, (double)req.speed);
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "franka_gripper_move_service_node");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("franka_gripper_move_service", move);
  ROS_INFO("Ready to move the gripper.");
  ros::spin();

  return 0;
}