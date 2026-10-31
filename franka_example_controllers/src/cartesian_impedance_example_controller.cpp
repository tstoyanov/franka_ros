// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_example_controllers/cartesian_impedance_example_controller.h>

#include <cmath>
#include <memory>

#include <franka/robot_state.h>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

#include <franka_example_controllers/example_controllers_common.h>
#include <franka_example_controllers/pseudo_inversion.h>

namespace franka_example_controllers {

CallbackReturn CartesianImpedanceExampleController::on_init() {
  auto_declare("arm_id", std::string());
  auto_declare("joint_names", std::vector<std::string>());
  auto_declare("translational_stiffness", 200.0);
  auto_declare("rotational_stiffness", 10.0);
  auto_declare("nullspace_stiffness", 0.0);
  return CallbackReturn::SUCCESS;
}

void CartesianImpedanceExampleController::declareAndWatchParameters() {
  auto node = get_node();
  param_callback_handle_ = node->add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter>& parameters) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        std::lock_guard<std::mutex> lock(target_mutex_);
        for (const auto& param : parameters) {
          if (param.get_name() == "translational_stiffness" ||
              param.get_name() == "rotational_stiffness" ||
              param.get_name() == "nullspace_stiffness") {
            double translational = param.get_name() == "translational_stiffness"
                                       ? param.as_double()
                                       : cartesian_stiffness_target_.topLeftCorner(3, 3)(0, 0);
            double rotational = param.get_name() == "rotational_stiffness"
                                      ? param.as_double()
                                      : cartesian_stiffness_target_.bottomRightCorner(3, 3)(0, 0);
            double nullspace = param.get_name() == "nullspace_stiffness"
                                    ? param.as_double()
                                    : nullspace_stiffness_target_;
            cartesian_stiffness_target_.setIdentity();
            cartesian_stiffness_target_.topLeftCorner(3, 3) << translational * Eigen::Matrix3d::Identity();
            cartesian_stiffness_target_.bottomRightCorner(3, 3) << rotational * Eigen::Matrix3d::Identity();
            cartesian_damping_target_.setIdentity();
            // Damping ratio = 1
            cartesian_damping_target_.topLeftCorner(3, 3) << 2.0 * std::sqrt(translational) * Eigen::Matrix3d::Identity();
            cartesian_damping_target_.bottomRightCorner(3, 3) << 2.0 * std::sqrt(rotational) * Eigen::Matrix3d::Identity();
            nullspace_stiffness_target_ = nullspace;
          }
        }
        return result;
      });
}

controller_interface::InterfaceConfiguration
CartesianImpedanceExampleController::command_interface_configuration() const {
  return {controller_interface::interface_configuration_type::INDIVIDUAL,
          jointCommandInterfaces(joint_names_, hardware_interface::HW_IF_EFFORT)};
}

controller_interface::InterfaceConfiguration
CartesianImpedanceExampleController::state_interface_configuration() const {
  return {controller_interface::interface_configuration_type::INDIVIDUAL,
          {frankaStateInterface(arm_id_), frankaModelInterface(arm_id_)}};
}

CallbackReturn CartesianImpedanceExampleController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  auto node = get_node();
  arm_id_ = node->get_parameter("arm_id").as_string();
  joint_names_ = node->get_parameter("joint_names").as_string_array();
  if (joint_names_.size() != 7) {
    RCLCPP_ERROR(node->get_logger(),
                 "CartesianImpedanceExampleController: Invalid or no joint_names parameters "
                 "provided, aborting controller init!");
    return CallbackReturn::ERROR;
  }

  sub_equilibrium_pose_ = node->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/equilibrium_pose", 20,
      std::bind(&CartesianImpedanceExampleController::equilibriumPoseCallback, this,
                std::placeholders::_1));

  declareAndWatchParameters();
  {
    std::lock_guard<std::mutex> lock(target_mutex_);
    double translational = node->get_parameter("translational_stiffness").as_double();
    double rotational = node->get_parameter("rotational_stiffness").as_double();
    nullspace_stiffness_target_ = node->get_parameter("nullspace_stiffness").as_double();
    cartesian_stiffness_target_.setIdentity();
    cartesian_stiffness_target_.topLeftCorner(3, 3) << translational * Eigen::Matrix3d::Identity();
    cartesian_stiffness_target_.bottomRightCorner(3, 3) << rotational * Eigen::Matrix3d::Identity();
    cartesian_damping_target_.setIdentity();
    cartesian_damping_target_.topLeftCorner(3, 3) << 2.0 * std::sqrt(translational) * Eigen::Matrix3d::Identity();
    cartesian_damping_target_.bottomRightCorner(3, 3) << 2.0 * std::sqrt(rotational) * Eigen::Matrix3d::Identity();
  }

  position_d_.setZero();
  orientation_d_.coeffs() << 0.0, 0.0, 0.0, 1.0;
  position_d_target_.setZero();
  orientation_d_target_.coeffs() << 0.0, 0.0, 0.0, 1.0;

  cartesian_stiffness_.setZero();
  cartesian_damping_.setZero();

  return CallbackReturn::SUCCESS;
}

CallbackReturn CartesianImpedanceExampleController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  // The state interfaces (and thus the robot state pointer) are only loaned to the
  // controller at activation time, so the model handle is created here.
  auto* robot_state = decodePointer<franka::RobotState>(state_interfaces_[0].get_value());
  auto* model = decodePointer<franka::Model>(state_interfaces_[1].get_value());
  model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(arm_id_ + "_model", *model,
                                                                  *robot_state);

  // compute initial velocity with jacobian and set x_attractor and q_d_nullspace
  // to initial configuration
  auto& initial_state = robotStateFromInterface(state_interfaces_[0]);
  // get jacobian
  std::array<double, 42> jacobian_array =
      model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
  // convert to eigen
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q_initial(initial_state.q.data());
  Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));

  // set equilibrium point to current state
  position_d_ = initial_transform.translation();
  orientation_d_ = Eigen::Quaterniond(initial_transform.linear());
  position_d_target_ = initial_transform.translation();
  orientation_d_target_ = Eigen::Quaterniond(initial_transform.linear());

  // set nullspace equilibrium configuration to initial q
  q_d_nullspace_ = q_initial;
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type CartesianImpedanceExampleController::update(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  // get state variables
  auto& robot_state = robotStateFromInterface(state_interfaces_[0]);
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
  Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d_);
  error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
  // Transform to base frame
  error.tail(3) << -transform.linear() * error.tail(3);

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

  // update parameters changed online either through ROS parameters or through the interactive
  // target by filtering
  {
    std::lock_guard<std::mutex> lock(target_mutex_);
    cartesian_stiffness_ =
        filter_params_ * cartesian_stiffness_target_ + (1.0 - filter_params_) * cartesian_stiffness_;
    cartesian_damping_ =
        filter_params_ * cartesian_damping_target_ + (1.0 - filter_params_) * cartesian_damping_;
    nullspace_stiffness_ =
        filter_params_ * nullspace_stiffness_target_ + (1.0 - filter_params_) * nullspace_stiffness_;
    position_d_ = filter_params_ * position_d_target_ + (1.0 - filter_params_) * position_d_;
    orientation_d_ = orientation_d_.slerp(filter_params_, orientation_d_target_);
  }
  return controller_interface::return_type::OK;
}

Eigen::Matrix<double, 7, 1> CartesianImpedanceExampleController::saturateTorqueRate(
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

void CartesianImpedanceExampleController::equilibriumPoseCallback(
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

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::CartesianImpedanceExampleController,
                       controller_interface::ControllerInterface)
