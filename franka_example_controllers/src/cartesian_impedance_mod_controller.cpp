// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_example_controllers/cartesian_impedance_mod_controller.h>

#include <cmath>
#include <memory>

#include <franka/robot_state.h>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

#include <franka_example_controllers/example_controllers_common.h>
#include "pseudo_inversion.h"

namespace franka_example_controllers {

CallbackReturn CartesianImpedanceModController::on_init() {
  auto_declare("arm_id", std::string());
  auto_declare("joint_names", std::vector<std::string>());
  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
CartesianImpedanceModController::command_interface_configuration() const {
  return {controller_interface::interface_configuration_type::INDIVIDUAL,
          jointCommandInterfaces(joint_names_, hardware_interface::HW_IF_EFFORT)};
}

controller_interface::InterfaceConfiguration
CartesianImpedanceModController::state_interface_configuration() const {
  return {controller_interface::interface_configuration_type::INDIVIDUAL,
          {frankaStateInterface(arm_id_), frankaModelInterface(arm_id_)}};
}

CallbackReturn CartesianImpedanceModController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  auto node = get_node();
  arm_id_ = node->get_parameter("arm_id").as_string();
  joint_names_ = node->get_parameter("joint_names").as_string_array();
  if (joint_names_.size() != 7) {
    RCLCPP_ERROR(node->get_logger(),
                 "CartesianImpedanceModController: Invalid or no joint_names parameters provided, "
                 "aborting controller init!");
    return CallbackReturn::ERROR;
  }

  auto* robot_state = static_cast<franka::RobotState*>(state_interfaces_[0].get_value());
  auto* model = static_cast<franka::Model*>(state_interfaces_[1].get_value());
  model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(arm_id_ + "_model", *model,
                                                                  *robot_state);

  sub_equilibrium_pose_ = node->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/equilibrium_pose", 20,
      std::bind(&CartesianImpedanceModController::equilibriumPoseCallback, this,
                std::placeholders::_1));

  sub_desired_stiffness_ = node->create_subscription<geometry_msgs::msg::TwistStamped>(
      "/desired_stiffness", 20,
      std::bind(&CartesianImpedanceModController::desiredStiffnessCallback, this,
                std::placeholders::_1));

  position_d_.setZero();
  orientation_d_.coeffs() << 0.0, 0.0, 0.0, 1.0;
  position_d_target_.setZero();
  orientation_d_target_.coeffs() << 0.0, 0.0, 0.0, 1.0;

  double initial_stiffness = 200;
  cartesian_stiffness_target_ = initial_stiffness * Eigen::Matrix<double, 6, 6>::Identity();
  cartesian_damping_target_ = 2.0 * std::sqrt(initial_stiffness) * Eigen::Matrix<double, 6, 6>::Identity();
  cartesian_stiffness_.setZero();
  cartesian_damping_.setZero();

  return CallbackReturn::SUCCESS;
}

CallbackReturn CartesianImpedanceModController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  // compute initial velocity with jacobian and set x_attractor and q_d_nullspace
  // to initial configuration
  const auto& initial_state = robotStateFromInterface(state_interfaces_[0]);
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q_initial(initial_state.q.data());
  Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));
  position_d_ = initial_transform.translation();
  orientation_d_ = Eigen::Quaterniond(initial_transform.linear());
  position_d_target_ = initial_transform.translation();
  orientation_d_target_ = Eigen::Quaterniond(initial_transform.linear());

  // set nullspace equilibrium configuration to initial q
  q_d_nullspace_ = q_initial;
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type CartesianImpedanceModController::update(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  // get state variables
  const auto& robot_state = robotStateFromInterface(state_interfaces_[0]);
  std::array<double, 7> coriolis_array = model_handle_->getCoriolis();
  std::array<double, 42> jacobian_array =
      model_handle_->getZeroJacobian(franka::Frame::kEndEffector);

  // convert to Eigen
  Eigen::Map<Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
  Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_j_d(  // NOLINT (readability-identifier-naming)
      robot_state.tau_J_d.data());
  Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  Eigen::Vector3d position(transform.translation());
  Eigen::Quaterniond orientation(transform.linear());

  // compute error to desired pose
  // position error
  Eigen::Matrix<double, 6, 1> error;
  error.head(3) << position - position_d_;

  // orientation error
  if (orientation_d_.coeffs().dot(orientation.coeffs()) < 0.0) {
    orientation.coeffs() << -orientation.coeffs();
  }
  // "difference" quaternion
  Eigen::Quaterniond error_quaternion(orientation * orientation_d_.inverse());
  // convert to axis angle
  Eigen::AngleAxisd error_quaternion_angle_axis(error_quaternion);
  // compute "orientation error"
  error.tail(3) << error_quaternion_angle_axis.axis() * error_quaternion_angle_axis.angle();

  // compute control
  // allocate variables
  Eigen::VectorXd tau_task(7), tau_nullspace(7), tau_d(7);

  // pseudoinverse for nullspace handling
  // kinematic pseuoinverse
  Eigen::MatrixXd jacobian_transpose_pinv;
  pseudoInverse(jacobian.transpose(), jacobian_transpose_pinv);

  Eigen::Matrix<double, 6, 6> stiffness;
  Eigen::Matrix<double, 6, 6> damping;
  double nullspace_stiffness;
  {
    std::lock_guard<std::mutex> lock(target_mutex_);
    stiffness = cartesian_stiffness_;
    damping = cartesian_damping_;
    nullspace_stiffness = nullspace_stiffness_;
  }

  // Cartesian PD control with damping ratio = 1
  tau_task << jacobian.transpose() * (-stiffness * error - damping * (jacobian * dq));
  // nullspace PD control with damping ratio = 1
  tau_nullspace << (Eigen::MatrixXd::Identity(7, 7) -
                    jacobian.transpose() * jacobian_transpose_pinv) *
                       (nullspace_stiffness * (q_d_nullspace_ - q) -
                        (2.0 * std::sqrt(nullspace_stiffness)) * dq);
  // Desired torque
  tau_d << tau_task + tau_nullspace + coriolis;
  // Saturate torque rate to avoid discontinuities
  tau_d << saturateTorqueRate(tau_d, tau_j_d);
  for (size_t i = 0; i < 7; ++i) {
    command_interfaces_[i].set_value(tau_d(i));
  }

  // update parameters changed online through topics by filtering
  {
    std::lock_guard<std::mutex> lock(target_mutex_);
    cartesian_stiffness_ =
        filter_params_ * cartesian_stiffness_target_ + (1.0 - filter_params_) * cartesian_stiffness_;
    cartesian_damping_ =
        filter_params_ * cartesian_damping_target_ + (1.0 - filter_params_) * cartesian_damping_;
    nullspace_stiffness_ =
        filter_params_ * nullspace_stiffness_target_ + (1.0 - filter_params_) * nullspace_stiffness_;
    position_d_ = filter_params_ * position_d_target_ + (1.0 - filter_params_) * position_d_;
    Eigen::AngleAxisd aa_orientation_d(orientation_d_);
    Eigen::AngleAxisd aa_orientation_d_target(orientation_d_target_);
    aa_orientation_d.axis() = filter_params_ * aa_orientation_d_target.axis() +
                              (1.0 - filter_params_) * aa_orientation_d.axis();
    aa_orientation_d.angle() = filter_params_ * aa_orientation_d_target.angle() +
                               (1.0 - filter_params_) * aa_orientation_d.angle();
    orientation_d_ = Eigen::Quaterniond(aa_orientation_d);
  }
  return controller_interface::return_type::OK;
}

Eigen::Matrix<double, 7, 1> CartesianImpedanceModController::saturateTorqueRate(
    const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
    const Eigen::Matrix<double, 7, 1>& tau_j_d) {  // NOLINT (readability-identifier-naming)
  Eigen::Matrix<double, 7, 1> tau_d_saturated{};
  for (size_t i = 0; i < 7; i++) {
    double difference = tau_d_calculated[i] - tau_j_d[i];
    tau_d_saturated[i] =
        tau_j_d[i] + std::max(std::min(difference, delta_tau_max_), -delta_tau_max_);
  }
  return tau_d_saturated;
}

void CartesianImpedanceModController::equilibriumPoseCallback(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(target_mutex_);
  position_d_target_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
  Eigen::Quaterniond last_orientation_d_target(orientation_d_target_);
  orientation_d_target_.coeffs() << msg->pose.orientation.x, msg->pose.orientation.y,
      msg->pose.orientation.z, msg->pose.orientation.w;
  if (last_orientation_d_target.coeffs().dot(orientation_d_target_.coeffs()) < 0.0) {
    orientation_d_target_.coeffs() << -orientation_d_target_.coeffs();
  }
}

void CartesianImpedanceModController::desiredStiffnessCallback(
    const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(target_mutex_);
  // Stiffness
  cartesian_stiffness_target_(0, 0) = msg->twist.linear.x;
  cartesian_stiffness_target_(1, 1) = msg->twist.linear.y;
  cartesian_stiffness_target_(2, 2) = msg->twist.linear.z;
  cartesian_stiffness_target_(3, 3) = msg->twist.angular.x;
  cartesian_stiffness_target_(4, 4) = msg->twist.angular.y;
  cartesian_stiffness_target_(5, 5) = msg->twist.angular.z;

  // Damping
  cartesian_damping_target_(0, 0) = 2.0 * std::sqrt(cartesian_stiffness_target_(0, 0));
  cartesian_damping_target_(1, 1) = 2.0 * std::sqrt(cartesian_stiffness_target_(1, 1));
  cartesian_damping_target_(2, 2) = 2.0 * std::sqrt(cartesian_stiffness_target_(2, 2));
  cartesian_damping_target_(3, 3) = 2.0 * std::sqrt(cartesian_stiffness_target_(3, 3));
  cartesian_damping_target_(4, 4) = 2.0 * std::sqrt(cartesian_stiffness_target_(4, 4));
  cartesian_damping_target_(5, 5) = 2.0 * std::sqrt(cartesian_stiffness_target_(5, 5));
}

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::CartesianImpedanceModController,
                       controller_interface::ControllerInterface)
