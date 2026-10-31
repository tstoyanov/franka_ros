// Copyright (c) 2019 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <controller_interface/controller_interface.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <realtime_tools/realtime_publisher.hpp>
#include <Eigen/Dense>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <franka_hw/franka_model_interface.h>
#include <franka_hw/trigger_rate.h>

namespace franka_example_controllers {

/**
 * This container holds all data and parameters used to control one panda arm with a Cartesian
 * impedance control law tracking a desired target pose.
 */
struct FrankaDataContainer {
  franka::RobotState* robot_state_{nullptr};  ///< Pointer to the robot state of the arm.
  std::unique_ptr<franka_hw::FrankaModelHandle>
      model_handle_;  ///< To have access to e.g. jacobians.
  size_t command_offset_{0};  ///< Offset of this arm's effort command interfaces.
  double filter_params_{0.005};       ///< [-] PT1-Filter constant to smooth target values set
                                      ///< by parameter updates (stiffness/damping)
                                      ///< or interactive markers for the target poses.
  double nullspace_stiffness_{20.0};  ///< [Nm/rad] To track the initial joint configuration in
                                      ///< the nullspace of the Cartesian motion.
  double nullspace_stiffness_target_{20.0};  ///< [Nm/rad] Unfiltered raw value.
  const double delta_tau_max_{1.0};          ///< [Nm/ms] Maximum difference in joint-torque per
                                             ///< timestep. Used to saturated torque rates to ensure
                                             ///< feasible commands.
  Eigen::Matrix<double, 6, 6> cartesian_stiffness_;         ///< To track the target pose.
  Eigen::Matrix<double, 6, 6> cartesian_stiffness_target_;  ///< Unfiltered raw value.
  Eigen::Matrix<double, 6, 6> cartesian_damping_;           ///< To damp cartesian motions.
  Eigen::Matrix<double, 6, 6> cartesian_damping_target_;    ///< Unfiltered raw value.
  Eigen::Matrix<double, 7, 1> q_d_nullspace_;               ///< Target joint pose for nullspace
                                                            ///< motion. For now we track the
                                                            ///< initial joint pose.
  Eigen::Vector3d position_d_;               ///< Target position of the end effector.
  Eigen::Quaterniond orientation_d_;         ///< Target orientation of the end effector.
  Eigen::Vector3d position_d_target_;        ///< Unfiltered raw value.
  Eigen::Quaterniond orientation_d_target_;  ///< Unfiltered raw value.
};

/**
 * Controller class for ros2_control that renders two decoupled Cartesian impedances for the
 * endeffectors of two panda arms.
 */
class DualArmCartesianImpedanceExampleController : public controller_interface::ControllerInterface {
 public:
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::return_type update(const rclcpp::Time& time,
                                           const rclcpp::Duration& period) override;
  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

 private:
  /**
   * Saturates torque commands to ensure feasibility.
   */
  Eigen::Matrix<double, 7, 1> saturateTorqueRate(
      const FrankaDataContainer& arm_data,
      const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
      const Eigen::Matrix<double, 7, 1>& tau_J_d);  // NOLINT (readability-identifier-naming)

  /**
   * Initializes a single Panda robot arm.
   */
  bool initArm(const std::string& arm_id, const std::vector<std::string>& joint_names,
               size_t state_offset, size_t command_offset);

  /**
   * Computes the decoupled controller update for a single arm.
   */
  void updateArm(FrankaDataContainer& arm_data);

  /**
   * Prepares all internal states to be ready to run the real-time control for one arm.
   */
  void startingArm(FrankaDataContainer& arm_data);

  ///< Dynamic parameters callback (replacing dynamic_reconfigure)
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
  void applyComplianceParameters();

  ///< Target pose subscriber
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_target_pose_left_;
  void targetPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  /**
   * Publishes a Pose Stamped for visualization of the current centering pose.
   */
  void publishCenteringPose();

  std::map<std::string, FrankaDataContainer> arms_data_;  ///< Holds all relevant data for both arms.
  std::mutex target_mutex_;
  std::string left_arm_id_;
  std::string right_arm_id_;
  std::vector<std::string> left_joint_names_;
  std::vector<std::string> right_joint_names_;

  ///< Transformation between base frames of the robots.
  Eigen::Affine3d Ol_T_Or_;  // NOLINT (readability-identifier-naming)
  ///< Target transformation between the two endeffectors.
  Eigen::Affine3d EEr_T_EEl_;  // NOLINT (readability-identifier-naming)
  ///< Transformation from the centering frame to the left end effector.
  Eigen::Affine3d EEl_T_C_;

  ///< Publisher for the centering tracking frame of the coordinated motion.
  realtime_tools::RealtimePublisher<geometry_msgs::msg::PoseStamped> center_frame_pub_;
  ///< Rate to trigger publishing the current pose of the centering frame.
  franka_hw::TriggerRate publish_rate_{30.0};

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace franka_example_controllers
