// Copyright (c) 2019 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_example_controllers/dual_arm_cartesian_impedance_example_controller.h>

#include <cmath>
#include <functional>
#include <memory>

#include <franka/robot_state.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <franka_example_controllers/example_controllers_common.h>
#include <franka_example_controllers/pseudo_inversion.h>

namespace franka_example_controllers {

bool DualArmCartesianImpedanceExampleController::initArm(const std::string& arm_id,
                                                         const std::vector<std::string>& joint_names,
                                                         size_t state_offset,
                                                         size_t command_offset) {
  FrankaDataContainer arm_data;
  arm_data.robot_state_ = decodePointer<franka::RobotState>(
      state_interfaces_[state_offset].get_value());
  auto* model = decodePointer<franka::Model>(state_interfaces_[state_offset + 1].get_value());
  arm_data.model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
      arm_id + "_model", *model, *arm_data.robot_state_);
  arm_data.command_offset_ = command_offset;

  arm_data.position_d_.setZero();
  arm_data.orientation_d_.coeffs() << 0.0, 0.0, 0.0, 1.0;
  arm_data.position_d_target_.setZero();
  arm_data.orientation_d_target_.coeffs() << 0.0, 0.0, 0.0, 1.0;

  arm_data.cartesian_stiffness_.setZero();
  arm_data.cartesian_damping_.setZero();

  arms_data_.emplace(std::make_pair(arm_id, std::move(arm_data)));
  return true;
}

CallbackReturn DualArmCartesianImpedanceExampleController::on_init() {
  auto_declare("left.arm_id", std::string());
  auto_declare("left.joint_names", std::vector<std::string>());
  auto_declare("right.arm_id", std::string());
  auto_declare("right.joint_names", std::vector<std::string>());
  auto_declare("left.translational_stiffness", 200.0);
  auto_declare("left.rotational_stiffness", 10.0);
  auto_declare("left.nullspace_stiffness", 0.0);
  auto_declare("right.translational_stiffness", 200.0);
  auto_declare("right.rotational_stiffness", 10.0);
  auto_declare("right.nullspace_stiffness", 0.0);
  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
DualArmCartesianImpedanceExampleController::command_interface_configuration() const {
  std::vector<std::string> interfaces;
  for (const auto& joint : left_joint_names_) {
    interfaces.push_back(joint + "/" + hardware_interface::HW_IF_EFFORT);
  }
  for (const auto& joint : right_joint_names_) {
    interfaces.push_back(joint + "/" + hardware_interface::HW_IF_EFFORT);
  }
  return {controller_interface::interface_configuration_type::INDIVIDUAL, interfaces};
}

controller_interface::InterfaceConfiguration
DualArmCartesianImpedanceExampleController::state_interface_configuration() const {
  return {controller_interface::interface_configuration_type::INDIVIDUAL,
          {frankaStateInterface(left_arm_id_), frankaModelInterface(left_arm_id_),
           frankaStateInterface(right_arm_id_), frankaModelInterface(right_arm_id_)}};
}

void DualArmCartesianImpedanceExampleController::applyComplianceParameters() {
  auto node = get_node();
  auto set_arm = [&](const std::string& prefix, FrankaDataContainer& arm_data) {
    double translational = node->get_parameter(prefix + ".translational_stiffness").as_double();
    double rotational = node->get_parameter(prefix + ".rotational_stiffness").as_double();
    arm_data.cartesian_stiffness_target_.setIdentity();
    arm_data.cartesian_stiffness_target_.topLeftCorner(3, 3) << translational * Eigen::Matrix3d::Identity();
    arm_data.cartesian_stiffness_target_.bottomRightCorner(3, 3) << rotational * Eigen::Matrix3d::Identity();
    arm_data.cartesian_damping_target_.setIdentity();
    arm_data.cartesian_damping_target_.topLeftCorner(3, 3) << 2.0 * std::sqrt(translational) * Eigen::Matrix3d::Identity();
    arm_data.cartesian_damping_target_.bottomRightCorner(3, 3) << 2.0 * std::sqrt(rotational) * Eigen::Matrix3d::Identity();
    arm_data.nullspace_stiffness_target_ = node->get_parameter(prefix + ".nullspace_stiffness").as_double();
  };
  std::lock_guard<std::mutex> lock(target_mutex_);
  if (arms_data_.find(left_arm_id_) != arms_data_.end()) {
    set_arm("left", arms_data_.at(left_arm_id_));
  }
  if (arms_data_.find(right_arm_id_) != arms_data_.end()) {
    set_arm("right", arms_data_.at(right_arm_id_));
  }
}

CallbackReturn DualArmCartesianImpedanceExampleController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  auto node = get_node();
  left_arm_id_ = node->get_parameter("left.arm_id").as_string();
  left_joint_names_ = node->get_parameter("left.joint_names").as_string_array();
  right_arm_id_ = node->get_parameter("right.arm_id").as_string();
  right_joint_names_ = node->get_parameter("right.joint_names").as_string_array();
  if (left_joint_names_.size() != 7 || right_joint_names_.size() != 7) {
    RCLCPP_ERROR(node->get_logger(),
                 "DualArmCartesianImpedanceExampleController: Invalid or no joint_names parameters "
                 "provided, aborting controller init!");
    return CallbackReturn::ERROR;
  }

  sub_target_pose_left_ = node->create_subscription<geometry_msgs::msg::PoseStamped>(
      "centering_frame_target_pose", 1,
      std::bind(&DualArmCartesianImpedanceExampleController::targetPoseCallback, this,
                std::placeholders::_1));

  param_callback_handle_ = node->add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter>&) {
        applyComplianceParameters();
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        return result;
      });
  applyComplianceParameters();

  // Get the transformation from right_O_frame to left_O_frame
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  try {
    geometry_msgs::msg::TransformStamped transform_msg = tf_buffer_->lookupTransform(
        left_arm_id_ + "_link0", right_arm_id_ + "_link0", tf2::TimePointZero,
        std::chrono::seconds(4));
    Ol_T_Or_ = tf2::transformToEigen(transform_msg);
  } catch (const tf2::TransformException& ex) {
    RCLCPP_ERROR(node->get_logger(),
                 "DualArmCartesianImpedanceExampleController: Failed to read transform from %s to "
                 "%s: %s. Aborting init!",
                 (right_arm_id_ + "_link0").c_str(), (left_arm_id_ + "_link0").c_str(), ex.what());
    return CallbackReturn::ERROR;
  }

  // Setup publisher for the centering frame.
    center_frame_pub_ =
      std::make_shared<realtime_tools::RealtimePublisher<geometry_msgs::msg::PoseStamped>>(
          node->create_publisher<geometry_msgs::msg::PoseStamped>("centering_frame", 1));

  return CallbackReturn::SUCCESS;
}

void DualArmCartesianImpedanceExampleController::startingArm(FrankaDataContainer& arm_data) {
  // compute initial velocity with jacobian and set x_attractor and q_d_nullspace
  // to initial configuration
  auto& initial_state = *arm_data.robot_state_;
  // get jacobian
  std::array<double, 42> jacobian_array =
      arm_data.model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
  // convert to eigen
  Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q_initial(initial_state.q.data());
  Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));

  // set target point to current state
  arm_data.position_d_ = initial_transform.translation();
  arm_data.orientation_d_ = Eigen::Quaterniond(initial_transform.linear());
  arm_data.position_d_target_ = initial_transform.translation();
  arm_data.orientation_d_target_ = Eigen::Quaterniond(initial_transform.linear());

  // set nullspace target configuration to initial q
  arm_data.q_d_nullspace_ = q_initial;
}

CallbackReturn DualArmCartesianImpedanceExampleController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  // The state interfaces (and thus the robot state pointers) are only loaned to the
  // controller at activation time, so the per-arm handles are created here.
  arms_data_.clear();
  bool left_success = initArm(left_arm_id_, left_joint_names_, 0, 0);
  bool right_success = initArm(right_arm_id_, right_joint_names_, 2, 7);
  if (!left_success || !right_success) {
    return CallbackReturn::ERROR;
  }
  applyComplianceParameters();
  for (auto& arm_data : arms_data_) {
    startingArm(arm_data.second);
  }
  auto& robot_state_right = *arms_data_.at(right_arm_id_).robot_state_;
  auto& robot_state_left = *arms_data_.at(left_arm_id_).robot_state_;
  Eigen::Affine3d Ol_T_EEl(Eigen::Matrix4d::Map(  // NOLINT (readability-identifier-naming)
      robot_state_left.O_T_EE.data()));           // NOLINT (readability-identifier-naming)
  Eigen::Affine3d Or_T_EEr(Eigen::Matrix4d::Map(  // NOLINT (readability-identifier-naming)
      robot_state_right.O_T_EE.data()));          // NOLINT (readability-identifier-naming)
  EEr_T_EEl_ =
      Or_T_EEr.inverse() * Ol_T_Or_.inverse() * Ol_T_EEl;  // NOLINT (readability-identifier-naming)
  EEl_T_C_.setIdentity();
  Eigen::Vector3d EEr_r_EEr_EEl =  // NOLINT (readability-identifier-naming)
      EEr_T_EEl_.translation();    // NOLINT (readability-identifier-naming)
  EEl_T_C_.translation() = -0.5 * EEr_T_EEl_.inverse().rotation() * EEr_r_EEr_EEl;
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type DualArmCartesianImpedanceExampleController::update(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  for (auto& arm_data : arms_data_) {
    updateArm(arm_data.second);
  }
  if (publish_rate_()) {
    publishCenteringPose();
  }
  return controller_interface::return_type::OK;
}

void DualArmCartesianImpedanceExampleController::updateArm(FrankaDataContainer& arm_data) {
  // get state variables
  auto& robot_state = *arm_data.robot_state_;
  std::array<double, 49> inertia_array = arm_data.model_handle_->getMass();
  std::array<double, 7> coriolis_array = arm_data.model_handle_->getCoriolis();
  std::array<double, 42> jacobian_array =
      arm_data.model_handle_->getZeroJacobian(franka::Frame::kEndEffector);

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
  error.head(3) << position - arm_data.position_d_;

  // orientation error
  if (arm_data.orientation_d_.coeffs().dot(orientation.coeffs()) < 0.0) {
    orientation.coeffs() << -orientation.coeffs();
  }
  // "difference" quaternion
  Eigen::Quaterniond error_quaternion(orientation * arm_data.orientation_d_.inverse());
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
  franka_example_controllers::pseudoInverse(jacobian.transpose(), jacobian_transpose_pinv);

  Eigen::Matrix<double, 6, 6> stiffness;
  Eigen::Matrix<double, 6, 6> damping;
  double nullspace_stiffness;
  {
    std::lock_guard<std::mutex> lock(target_mutex_);
    stiffness = arm_data.cartesian_stiffness_;
    damping = arm_data.cartesian_damping_;
    nullspace_stiffness = arm_data.nullspace_stiffness_;
  }

  // Cartesian PD control with damping ratio = 1
  tau_task << jacobian.transpose() * (-stiffness * error - damping * (jacobian * dq));
  // nullspace PD control with damping ratio = 1
  tau_nullspace << (Eigen::MatrixXd::Identity(7, 7) -
                    jacobian.transpose() * jacobian_transpose_pinv) *
                       (nullspace_stiffness * (arm_data.q_d_nullspace_ - q) -
                        (2.0 * std::sqrt(nullspace_stiffness)) * dq);
  // Desired torque
  tau_d << tau_task + tau_nullspace + coriolis;
  // Saturate torque rate to avoid discontinuities
  tau_d << saturateTorqueRate(arm_data, tau_d, tau_j_d);
  for (size_t i = 0; i < 7; ++i) {
    command_interfaces_[arm_data.command_offset_ + i].set_value(tau_d(i));
  }

  // update parameters changed online either through ROS parameters or through the interactive
  // target by filtering
  std::lock_guard<std::mutex> lock(target_mutex_);
  arm_data.cartesian_stiffness_ = arm_data.filter_params_ * arm_data.cartesian_stiffness_target_ +
                                  (1.0 - arm_data.filter_params_) * arm_data.cartesian_stiffness_;
  arm_data.cartesian_damping_ = arm_data.filter_params_ * arm_data.cartesian_damping_target_ +
                                (1.0 - arm_data.filter_params_) * arm_data.cartesian_damping_;
  arm_data.nullspace_stiffness_ = arm_data.filter_params_ * arm_data.nullspace_stiffness_target_ +
                                  (1.0 - arm_data.filter_params_) * arm_data.nullspace_stiffness_;
  arm_data.position_d_ = arm_data.filter_params_ * arm_data.position_d_target_ +
                         (1.0 - arm_data.filter_params_) * arm_data.position_d_;
  arm_data.orientation_d_ =
      arm_data.orientation_d_.slerp(arm_data.filter_params_, arm_data.orientation_d_target_);
}

Eigen::Matrix<double, 7, 1> DualArmCartesianImpedanceExampleController::saturateTorqueRate(
    const FrankaDataContainer& arm_data,
    const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
    const Eigen::Matrix<double, 7, 1>& tau_j_d) {  // NOLINT (readability-identifier-naming)
  Eigen::Matrix<double, 7, 1> tau_d_saturated{};
  for (size_t i = 0; i < 7; i++) {
    double difference = tau_d_calculated[i] - tau_j_d[i];
    tau_d_saturated[i] = tau_j_d[i] + std::max(std::min(difference, arm_data.delta_tau_max_),
                                               -arm_data.delta_tau_max_);
  }
  return tau_d_saturated;
}

void DualArmCartesianImpedanceExampleController::targetPoseCallback(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  try {
    if (msg->header.frame_id != left_arm_id_ + "_link0") {
      RCLCPP_ERROR_STREAM(get_node()->get_logger(),
                          "DualArmCartesianImpedanceExampleController: Got pose target with invalid"
                          " frame_id "
                          << msg->header.frame_id << ". Expected " << left_arm_id_ + "_link0");
      return;
    }

    std::lock_guard<std::mutex> lock(target_mutex_);
    // Set target for the left robot.
    auto& left_arm_data = arms_data_.at(left_arm_id_);
    Eigen::Affine3d Ol_T_C;
    tf2::fromMsg(msg->pose, Ol_T_C);
    Eigen::Affine3d Ol_T_EEl_d =      // NOLINT (readability-identifier-naming)
        Ol_T_C * EEl_T_C_.inverse();  // NOLINT (readability-identifier-naming)
    left_arm_data.position_d_target_ = Ol_T_EEl_d.translation();
    Eigen::Quaterniond last_orientation_d_target(left_arm_data.orientation_d_target_);
    Eigen::Quaterniond new_orientation_target(Ol_T_EEl_d.linear());
    if (last_orientation_d_target.coeffs().dot(new_orientation_target.coeffs()) < 0.0) {
      new_orientation_target.coeffs() << -new_orientation_target.coeffs();
    }
    Ol_T_EEl_d.linear() = new_orientation_target.matrix();
    left_arm_data.orientation_d_target_ = Ol_T_EEl_d.linear();

    // Compute target for the right endeffector given the static desired transform from left to
    // right endeffector.
    Eigen::Affine3d Or_T_EEr_d = Ol_T_Or_.inverse()     // NOLINT (readability-identifier-naming)
                                 * Ol_T_EEl_d *         // NOLINT (readability-identifier-naming)
                                 EEr_T_EEl_.inverse();  // NOLINT (readability-identifier-naming)
    auto& right_arm_data = arms_data_.at(right_arm_id_);
    right_arm_data.position_d_target_ =
        Or_T_EEr_d.translation();  // NOLINT (readability-identifier-naming)
    last_orientation_d_target = Eigen::Quaterniond(right_arm_data.orientation_d_target_);
    right_arm_data.orientation_d_target_ =
        Or_T_EEr_d.linear();  // NOLINT (readability-identifier-naming)
    if (last_orientation_d_target.coeffs().dot(right_arm_data.orientation_d_target_.coeffs()) <
        0.0) {
      right_arm_data.orientation_d_target_.coeffs()
          << -right_arm_data.orientation_d_target_.coeffs();
    }

  } catch (std::out_of_range& ex) {
    RCLCPP_ERROR_STREAM(get_node()->get_logger(),
                        "DualArmCartesianImpedanceExampleController: Exception setting target poses.");
  }
}

void DualArmCartesianImpedanceExampleController::publishCenteringPose() {
  if (center_frame_pub_->trylock()) {
    auto& robot_state_left = *arms_data_.at(left_arm_id_).robot_state_;
    Eigen::Affine3d Ol_T_EEl(Eigen::Matrix4d::Map(  // NOLINT (readability-identifier-naming)
        robot_state_left.O_T_EE.data()));           // NOLINT (readability-identifier-naming)
    Eigen::Affine3d Ol_T_C = Ol_T_EEl * EEl_T_C_;   // NOLINT (readability-identifier-naming)
    center_frame_pub_->msg_.pose = tf2::toMsg(Ol_T_C);
    center_frame_pub_->msg_.header.frame_id = left_arm_id_ + "_link0";
    center_frame_pub_->msg_.header.stamp = get_node()->now();
    center_frame_pub_->unlockAndPublish();
  }
}

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::DualArmCartesianImpedanceExampleController,
                       controller_interface::ControllerInterface)
