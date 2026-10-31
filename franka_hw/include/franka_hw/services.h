// Copyright (c) 2019 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once
#include <memory>
#include <string>
#include <vector>

#include <franka/exception.h>
#include <franka/robot.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/service.hpp>

#include <franka_msgs/srv/set_cartesian_impedance.hpp>
#include <franka_msgs/srv/set_ee_frame.hpp>
#include <franka_msgs/srv/set_force_torque_collision_behavior.hpp>
#include <franka_msgs/srv/set_full_collision_behavior.hpp>
#include <franka_msgs/srv/set_joint_impedance.hpp>
#include <franka_msgs/srv/set_k_frame.hpp>
#include <franka_msgs/srv/set_load.hpp>

namespace franka_hw {

/**
 * Advertises a service that acts according to the handler function which is used in the service
 * callback.
 *
 * @param[in] node The node in whose namespace to advertise the service.
 * @param[in] name The name of the service.
 * @param[in] handler The callback method for the service.
 * @return The service server.
 */
template <typename T>
typename rclcpp::Service<T>::SharedPtr advertiseService(
    rclcpp::Node::SharedPtr node,
    const std::string& name,
    std::function<void(std::shared_ptr<typename T::Request>, std::shared_ptr<typename T::Response>)>
        handler) {
  return node->create_service<T>(
      name,
      [node, name, handler](std::shared_ptr<typename T::Request> request,
                      std::shared_ptr<typename T::Response> response) {
        try {
          handler(request, response);
          response->success = true;
          RCLCPP_DEBUG(node->get_logger(), "%s succeeded.", name.c_str());
        } catch (const franka::Exception& ex) {
          RCLCPP_ERROR(node->get_logger(), "%s failed: %s", name.c_str(), ex.what());
          response->success = false;
          response->error = ex.what();
        }
      });
}

/**
 * This class serves as container that gathers all possible service interfaces to a libfranka robot
 * instance.
 */
class ServiceContainer {
 public:
  /**
   * Advertises and adds a service to the container class.
   *
   * @return A reference to the service container to allow a fluent API.
   */
  template <typename T, typename... TArgs>
  ServiceContainer& advertiseService(TArgs&&... args) {
    services_.push_back(franka_hw::advertiseService<T>(std::forward<TArgs>(args)...));
    return *this;
  }

 private:
  std::vector<std::shared_ptr<rclcpp::ServiceBase>> services_;
};

/**
 * Sets up all services relevant for a libfranka robot inside a service container.
 *
 * @param[in] robot The libfranka robot for which to set up services interfaces.
 * @param[in] node The node in whose namespace to advertise the services.
 * @param[in] services The container to store the service servers.
 */
void setupServices(franka::Robot& robot, rclcpp::Node::SharedPtr node, ServiceContainer& services);

/**
 * Callback for the service interface to franka::robot::setCartesianImpedance.
 */
void setCartesianImpedance(franka::Robot& robot,
                           const franka_msgs::srv::SetCartesianImpedance::Request::SharedPtr req,
                           franka_msgs::srv::SetCartesianImpedance::Response::SharedPtr res);

/**
 * Callback for the service interface to franka::robot::setJointImpedance.
 */
void setJointImpedance(franka::Robot& robot,
                       const franka_msgs::srv::SetJointImpedance::Request::SharedPtr req,
                       franka_msgs::srv::SetJointImpedance::Response::SharedPtr res);

/**
 * Callback for the service interface to franka::robot::setEEFrame.
 */
void setEEFrame(franka::Robot& robot,
                const franka_msgs::srv::SetEEFrame::Request::SharedPtr req,
                franka_msgs::srv::SetEEFrame::Response::SharedPtr res);

/**
 * Callback for the service interface to franka::robot::setKFrame.
 */
void setKFrame(franka::Robot& robot,
               const franka_msgs::srv::SetKFrame::Request::SharedPtr req,
               franka_msgs::srv::SetKFrame::Response::SharedPtr res);

/**
 * Callback for the service interface to franka::robot::setForceTorqueCollisionBehavior.
 */
void setForceTorqueCollisionBehavior(
    franka::Robot& robot,
    const franka_msgs::srv::SetForceTorqueCollisionBehavior::Request::SharedPtr req,
    franka_msgs::srv::SetForceTorqueCollisionBehavior::Response::SharedPtr res);

/**
 * Callback for the service interface to franka::robot::setFullCollisionBehavior.
 */
void setFullCollisionBehavior(
    franka::Robot& robot,
    const franka_msgs::srv::SetFullCollisionBehavior::Request::SharedPtr req,
    franka_msgs::srv::SetFullCollisionBehavior::Response::SharedPtr res);

/**
 * Callback for the service interface to franka::robot::setLoad.
 */
void setLoad(franka::Robot& robot,
             const franka_msgs::srv::SetLoad::Request::SharedPtr req,
             franka_msgs::srv::SetLoad::Response::SharedPtr res);

}  // namespace franka_hw
