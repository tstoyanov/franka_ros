// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_hw/services.h>

namespace franka_hw {

void setupServices(franka::Robot& robot, rclcpp::Node::SharedPtr node, ServiceContainer& services) {
  services
      .advertiseService<franka_msgs::srv::SetJointImpedance>(
          node, "set_joint_impedance",
          [&robot](auto&& req, auto&& res) { return franka_hw::setJointImpedance(robot, req, res); })
      .advertiseService<franka_msgs::srv::SetCartesianImpedance>(
          node, "set_cartesian_impedance",
          [&robot](auto&& req, auto&& res) {
            return franka_hw::setCartesianImpedance(robot, req, res);
          })
      .advertiseService<franka_msgs::srv::SetEEFrame>(
          node, "set_EE_frame",
          [&robot](auto&& req, auto&& res) { return franka_hw::setEEFrame(robot, req, res); })
      .advertiseService<franka_msgs::srv::SetKFrame>(
          node, "set_K_frame",
          [&robot](auto&& req, auto&& res) { return franka_hw::setKFrame(robot, req, res); })
      .advertiseService<franka_msgs::srv::SetForceTorqueCollisionBehavior>(
          node, "set_force_torque_collision_behavior",
          [&robot](auto&& req, auto&& res) {
            return franka_hw::setForceTorqueCollisionBehavior(robot, req, res);
          })
      .advertiseService<franka_msgs::srv::SetFullCollisionBehavior>(
          node, "set_full_collision_behavior",
          [&robot](auto&& req, auto&& res) {
            return franka_hw::setFullCollisionBehavior(robot, req, res);
          })
      .advertiseService<franka_msgs::srv::SetLoad>(
          node, "set_load",
          [&robot](auto&& req, auto&& res) { return franka_hw::setLoad(robot, req, res); });
}

void setCartesianImpedance(franka::Robot& robot,
                           const franka_msgs::srv::SetCartesianImpedance::Request::SharedPtr req,
                           franka_msgs::srv::SetCartesianImpedance::Response::SharedPtr /*res*/) {
  std::array<double, 6> cartesian_stiffness;
  std::copy(req->cartesian_stiffness.cbegin(), req->cartesian_stiffness.cend(),
            cartesian_stiffness.begin());
  robot.setCartesianImpedance(cartesian_stiffness);
}

void setJointImpedance(franka::Robot& robot,
                       const franka_msgs::srv::SetJointImpedance::Request::SharedPtr req,
                       franka_msgs::srv::SetJointImpedance::Response::SharedPtr /*res*/) {
  std::array<double, 7> joint_stiffness;
  std::copy(req->joint_stiffness.cbegin(), req->joint_stiffness.cend(), joint_stiffness.begin());
  robot.setJointImpedance(joint_stiffness);
}

void setEEFrame(franka::Robot& robot,
                const franka_msgs::srv::SetEEFrame::Request::SharedPtr req,
                franka_msgs::srv::SetEEFrame::Response::SharedPtr /*res*/) {
  std::array<double, 16> f_t_ee;  // NOLINT [readability-identifier-naming]
  std::copy(req->f_t_ee.cbegin(), req->f_t_ee.cend(), f_t_ee.begin());
  robot.setEE(f_t_ee);
}

void setKFrame(franka::Robot& robot,
               const franka_msgs::srv::SetKFrame::Request::SharedPtr req,
               franka_msgs::srv::SetKFrame::Response::SharedPtr /*res*/) {
  std::array<double, 16> ee_t_k;  // NOLINT [readability-identifier-naming]
  std::copy(req->ee_t_k.cbegin(), req->ee_t_k.cend(), ee_t_k.begin());
  robot.setK(ee_t_k);
}

void setForceTorqueCollisionBehavior(
    franka::Robot& robot,
    const franka_msgs::srv::SetForceTorqueCollisionBehavior::Request::SharedPtr req,
    const franka_msgs::srv::SetForceTorqueCollisionBehavior::Response::SharedPtr /*res*/) {
  std::array<double, 7> lower_torque_thresholds_nominal;
  std::copy(req->lower_torque_thresholds_nominal.cbegin(),
            req->lower_torque_thresholds_nominal.cend(), lower_torque_thresholds_nominal.begin());
  std::array<double, 7> upper_torque_thresholds_nominal;
  std::copy(req->upper_torque_thresholds_nominal.cbegin(),
            req->upper_torque_thresholds_nominal.cend(), upper_torque_thresholds_nominal.begin());
  std::array<double, 6> lower_force_thresholds_nominal;
  std::copy(req->lower_force_thresholds_nominal.cbegin(), req->lower_force_thresholds_nominal.cend(),
            lower_force_thresholds_nominal.begin());
  std::array<double, 6> upper_force_thresholds_nominal;
  std::copy(req->upper_force_thresholds_nominal.cbegin(), req->upper_force_thresholds_nominal.cend(),
            upper_force_thresholds_nominal.begin());

  robot.setCollisionBehavior(lower_torque_thresholds_nominal, upper_torque_thresholds_nominal,
                             lower_force_thresholds_nominal, upper_force_thresholds_nominal);
}

void setFullCollisionBehavior(
    franka::Robot& robot,
    const franka_msgs::srv::SetFullCollisionBehavior::Request::SharedPtr req,
    const franka_msgs::srv::SetFullCollisionBehavior::Response::SharedPtr /*res*/) {
  std::array<double, 7> lower_torque_thresholds_acceleration;
  std::copy(req->lower_torque_thresholds_acceleration.cbegin(),
            req->lower_torque_thresholds_acceleration.cend(),
            lower_torque_thresholds_acceleration.begin());
  std::array<double, 7> upper_torque_thresholds_acceleration;
  std::copy(req->upper_torque_thresholds_acceleration.cbegin(),
            req->upper_torque_thresholds_acceleration.cend(),
            upper_torque_thresholds_acceleration.begin());
  std::array<double, 7> lower_torque_thresholds_nominal;
  std::copy(req->lower_torque_thresholds_nominal.cbegin(),
            req->lower_torque_thresholds_nominal.cend(), lower_torque_thresholds_nominal.begin());
  std::array<double, 7> upper_torque_thresholds_nominal;
  std::copy(req->upper_torque_thresholds_nominal.cbegin(),
            req->upper_torque_thresholds_nominal.cend(), upper_torque_thresholds_nominal.begin());
  std::array<double, 6> lower_force_thresholds_acceleration;
  std::copy(req->lower_force_thresholds_acceleration.cbegin(),
            req->lower_force_thresholds_acceleration.cend(),
            lower_force_thresholds_acceleration.begin());
  std::array<double, 6> upper_force_thresholds_acceleration;
  std::copy(req->upper_force_thresholds_acceleration.cbegin(),
            req->upper_force_thresholds_acceleration.cend(),
            upper_force_thresholds_acceleration.begin());
  std::array<double, 6> lower_force_thresholds_nominal;
  std::copy(req->lower_force_thresholds_nominal.cbegin(), req->lower_force_thresholds_nominal.cend(),
            lower_force_thresholds_nominal.begin());
  std::array<double, 6> upper_force_thresholds_nominal;
  std::copy(req->upper_force_thresholds_nominal.cbegin(), req->upper_force_thresholds_nominal.cend(),
            upper_force_thresholds_nominal.begin());
  robot.setCollisionBehavior(lower_torque_thresholds_acceleration,
                             upper_torque_thresholds_acceleration, lower_torque_thresholds_nominal,
                             upper_torque_thresholds_nominal, lower_force_thresholds_acceleration,
                             upper_force_thresholds_acceleration, lower_force_thresholds_nominal,
                             upper_force_thresholds_nominal);
}

void setLoad(franka::Robot& robot,
             const franka_msgs::srv::SetLoad::Request::SharedPtr req,
             const franka_msgs::srv::SetLoad::Response::SharedPtr /*res*/) {
  double mass(req->mass);
  std::array<double, 3> f_x_center_load;  // NOLINT [readability-identifier-naming]
  std::copy(req->f_x_center_load.cbegin(), req->f_x_center_load.cend(), f_x_center_load.begin());
  std::array<double, 9> load_inertia;
  std::copy(req->load_inertia.cbegin(), req->load_inertia.cend(), load_inertia.begin());

  robot.setLoad(mass, f_x_center_load, load_inertia);
}

}  // namespace franka_hw
