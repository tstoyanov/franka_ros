# Migration procedure: ROS 1 (kinetic-devel) → ROS 2 (jazzy)

Target: port `franka_ros` (this repo, branch `kinetic-devel`) to ROS 2 Jazzy while keeping
**libfranka 0.7.0** as the pinned FCI client library for hardware compatibility.
The CI image `tstoyanov/jazzy_libfranka:latest` (based on `ros:jazzy`) ships a working
libfranka 0.7.0, so all packages keep `find_package(Franka 0.7.0 REQUIRED)`.
The `tstoyanov/franka_arm_ros2` drivers are used **as reference only** — they target a newer
libfranka that is incompatible with our hardware; API calls must follow the 0.7.0 headers
(`franka/robot.h`, `franka/model.h`, `franka/gripper.h`, `franka/robot_state.h`,
`franka/control_types.h`, `franka/duration.h`, `franka/rate_limiting.h`) exactly as used on
`kinetic-devel`.

## 1. Build system: catkin → ament_cmake

Per package, replace `CMakeLists.txt` and `package.xml`:

- `package.xml`: `<buildtool_depend>catkin</buildtool_depend>` →
  `<buildtool_depend>ament_cmake</buildtool_depend>`; convert `message_generation` /
  `message_runtime` → `rosidl_default_generators` / `rosidl_default_runtime`;
  ROS 1 deps → ROS 2 equivalents (`roscpp`→`rclcpp`, `rospy`→`rclpy`, `tf`→`tf2_ros` +
  `tf2_geometry_msgs`, `control_msgs`/`sensor_msgs`/`geometry_msgs` keep the same names but
  are `rosidl` packages, `dynamic_reconfigure`→ dropped, `pluginlib` stays,
  `hardware_interface`/`controller_interface`/`controller_manager` → `ros2_control` packages
  of the same names). Format 3 `package.xml` is recommended.
- `CMakeLists.txt`: `find_package(catkin ...)` + `catkin_package()` →
  `find_package(ament_cmake REQUIRED)` + per-dependency `find_package(...)` +
  `ament_package()`. Export targets with `ament_export_targets`, install with
  `install(TARGETS ... EXPORT ...)` + `ament_export_dependencies(...)`.
- `franka_msgs`: `add_message_files/add_service_files/add_action_files/generate_messages` →
  `rosidl_generate_interfaces(${PROJECT_NAME} msg/... srv/... action/... DEPENDENCIES std_msgs actionlib_msgs)`.
  Message/action/srv definitions themselves are unchanged (plain ROS IDL).
- Workspace layout: `catkin_init_workspace`/`devel` → plain `src/` + `colcon build/test`.

## 2. Core ROS API: roscpp → rclcpp

- `ros::init` → `rclcpp::init`; `ros::NodeHandle` → `std::shared_ptr<rclcpp::Node>`;
  `ros::AsyncSpinner` → `rclcpp::executors::MultiThreadedExecutor` (or
  `rclcpp::spin_some` in a thread). Private params `nh("~")` → node parameters
  (`declare_parameter`/`get_parameter`, or YAML via launch).
- Logging macros `ROS_INFO_STREAM` etc. → `RCLCPP_INFO_STREAM(node->get_logger(), ...)`.
- `ros::Time::now()` → `node->now()`; `ros::Duration` → `rclcpp::Duration`.
- `ros::Rate` → `rclcpp::Rate`.

## 3. ros_control → ros2_control (the largest change)

The ROS 1 `hardware_interface::RobotHW` + `controller_manager` + `pluginlib` controller
loading model maps onto ros2_control as follows:

- `franka_hw::FrankaHW : hardware_interface::RobotHW` →
  `hardware_interface::SystemInterface` (ros2_control) with
  `on_init(const hardware_interface::HardwareInfo&)`, `export_state_interfaces()`,
  `export_command_interfaces()`, `read(rclcpp::Time, rclcpp::Duration)`,
  `write(...)`. `FrankaCombinableHW`/`FrankaCombinedHW` → combine multiple
  `SystemInterface`s via `composite` hardware components (or keep a single system handling
  both arms, as `franka_combined_hw` does today).
- Custom interfaces (`FrankaStateInterface`, `FrankaModelInterface`,
  `FrankaCartesianCommandInterface` in `franka_hw/include/franka_hw`) → ros2_control
  `StateInterface`/`CommandInterface` with custom interface names registered through the
  URDF `<ros2_control>` tags instead of `hardware_interface` claim maps.
- Controllers (`franka_control::FrankaStateController`, all
  `franka_example_controllers::*` `MultiInterfaceController`s) →
  `controller_interface::ControllerInterface` (or `ChainableControllerInterface` where
  chaining matters). `PLUGINLIB_EXPORT_CLASS` → `PLUGINLIB_EXPORT_CLASS` still works but
  plugin XML base class becomes `controller_interface::ControllerInterface`;
  loading via `controller_manager` (ros2_control) with `spawner` nodes.
- `joint_limits_interface` → `joint_limits` package of ros2_control (URDF parsing via
  `hardware_interface::HardwareInfo`).
- Node structure: `franka_control_node` → a ros2_control `controller_manager` running with a
  `FrankaHW` system plugin, configured by URDF + `controllers.yaml`; the separate
  `franka_combined_control_node` merges into the combined-hardware variant.
- `realtime_tools::RealtimePublisher` → `realtime_tools::RealtimePublisher` (available for
  ROS 2) or `rclcpp` publishers with realtime-safe handling in the `update()` loop.

## 4. tf → tf2

- `franka_state_controller` uses `tf::Transform`, `tf::StampedTransform`, publishes
  `tf2_msgs::TFMessage` → switch to `tf2::Transform`, `tf2_ros::TransformBroadcaster`,
  `geometry_msgs::TransformStamped`; `convertArrayToTf` becomes a
  `std::array<double,16>` → `tf2::Matrix3x3`/`tf2::Transform` conversion.
- `tf_conversions` / `eigen_conversions` in example controllers → `tf2_eigen` /
  `Eigen` geometry directly.

## 5. Actionlib → rclcpp_action

- `franka_gripper` uses `actionlib::SimpleActionServer<control_msgs::GripperCommandAction>`
  plus `actionlib_msgs`/`control_msgs` → `rclcpp_action::ActionServer<control_msgs::action::GripperCommand>`,
  feedback published in the execute callback; the gripper node also advertises
  `sensor_msgs/JointState` (→ `rclcpp::Publisher<sensor_msgs::msg::JointState>`).
- `franka_msgs/ErrorRecovery.action` → generated by `rosidl_generate_interfaces`;
  the service in `franka_control` (`ErrorRecovery`) becomes an `rclcpp::Service` with
  `std_srvs`-style empty request/response semantics kept.

## 6. Services

- `franka_hw::advertiseService` template (`ros::ServiceServer`,
  `franka_msgs::Set*::Request/Response`) → `rclcpp::Service<T>::SharedPtr` via
  `node->create_service<T>(name, callback)`; the `success`/`error` fields stay in the srv
  definitions. `xmlrpcpp` dependency drops out.

## 7. dynamic_reconfigure → ROS 2 parameters

- `franka_example_controllers/cfg/*.cfg` (compliance params, desired mass, dual-arm
  compliance) → `dynamic_reconfigure` has no direct ROS 2 port; migrate to
  `rclcpp` dynamic parameters with `add_on_set_parameters_callback`, updating the
  impedance/mass setpoints in the realtime loop guarded by a mutex/atomic, keeping the
  same parameter names and ranges as the `.cfg` files.

## 8. Launch files & tests

- `*.launch` (XML, `$(find pkg)`) → Python launch (`launch_ros` actions), `$(find ...)` →
  `get_package_share_directory`, `param`/`rosparam` → `parameters=[...]`.
- `franka_hw/test` rostest (`add_rostest_gtest`, `.test` files) → `ament_cmake_gtest` +
  `launch_testing` (`launch_testing_ros` for node tests); the dummy-IP tests
  (`robot_ip=unused_dummy_ip`) keep their structure with `launch_testing`.
- `.ci/debug.sh` (catkin) → colcon equivalent (already wired in
  `.github/workflows/jazzy-ci.yaml`): `colcon build` + `colcon test`.

## 9. Package-by-package checklist

| Package | Main work |
|---|---|
| franka_msgs | rosidl_generate_interfaces; msgs/srvs/action unchanged |
| franka_description | ament_cmake install of urdf/meshes; xacro unchanged |
| franka_hw | SystemInterface port, custom interfaces, services.h → rclcpp services |
| franka_control | controller_manager + FrankaStateController port, tf2 broadcaster, launch → py |
| franka_gripper | rclcpp_action GripperCommand, rclcpp node, JointState publisher |
| franka_example_controllers | ControllerInterface ports, dyn-reconf → parameters, tf2_eigen |
| franka_visualization | two simple JointState publisher nodes → rclcpp |
| franka_ros | metapackage (ament_cmake, `<metapackage/>` export kept) |

## 10. Suggested order of work

1. CI + branch (this change): `jazzy` branch, `.github/workflows/jazzy-ci.yaml` building in
   `tstoyanov/jazzy_libfranka:latest` with `find_package(Franka 0.7.0 REQUIRED)`.
2. `franka_msgs` + `franka_description` (no logic, validates the toolchain and libfranka
   0.7.0 linkage).
3. `franka_hw` (SystemInterface + interfaces + services) — the foundation everything else
   depends on.
4. `franka_control` + `franka_visualization` (nodes, state controller, tf2).
5. `franka_gripper` (action server).
6. `franka_example_controllers` (controllers + parameter migration), largest package.
7. Launch files, tests, docs; full `colcon test` green in CI.

Throughout: keep every libfranka call site byte-compatible with 0.7.0 semantics — do not adopt
newer-libfranka APIs seen in `franka_arm_ros2`; where ros2_control forces an API change
(e.g. `read`/`write` signatures), change only the ROS-facing glue, never the FCI calls.

## 11. Progress checklist (jazzy branch)

- [x] **Stage 1** — `jazzy` branch, GitHub Actions CI (`.github/workflows/jazzy-ci.yaml`, image
  `tstoyanov/jazzy_libfranka:latest`), migration doc, README badge.
- [x] **Stage 2** — `franka_msgs` (rosidl_generate_interfaces, msg/srv/action unchanged),
  `franka_description` (ament_cmake).
- [x] **Stage 3** — `franka_hw`: `SystemInterface` port of `FrankaHW`/`FrankaCombinableHW`;
  custom interfaces exported as named state/command interfaces
  (`<arm_id>/franka_hw/FrankaStateInterface/robot_state`, `.../FrankaModelInterface/robot_model`,
  cartesian pose/velocity command interfaces); services -> rclcpp services; error recovery ->
  rclcpp_action; `combined_robot_hw` dropped (multi-system ros2_control); tests ->
  ament_cmake_gtest with synthetic HardwareInfo (no robot needed).
- [x] **Stage 4** — `franka_control` (FrankaStateController as ControllerInterface, tf2,
  ros2_control_node entry point, python launch) + `franka_visualization` (rclcpp nodes).
- [x] **Stage 5** — `franka_gripper`: rclcpp_action servers (homing/stop/move/grasp +
  GripperCommand), own actions via rosidl, rclcpp node with parameters.
- [x] **Stage 6** — `franka_example_controllers`: all 11 controllers as ControllerInterface;
  dynamic_reconfigure -> rclcpp dynamic parameters (same names/defaults/ranges);
  `/desired_stiffness` topic kept in the mod controller; tf -> tf2_ros/tf2_eigen.
- [x] **Stage 7** — metapackage, legacy CI files (Jenkinsfile/.travis.yml/.ci/cmake) removed,
  all launch files -> python launch, README/docs updated.

libfranka stays pinned at `find_package(Franka 0.7.0 REQUIRED)` in every C++ package.

## 12. CI status

`jazzy-ci.yaml` is green on the `jazzy` branch (run 41203935559): **Build and test (colcon)**
(8 packages built, `franka_hw_test` gtest suite passing) and **Lint (ament linters)** both
succeed in `tstoyanov/jazzy_libfranka:latest`.

## 13. Code review findings (ros1_control -> ros2_control gotchas) and fixes

A deep review of the port against ros2_control (jazzy) semantics found the following
issues, all fixed on the `jazzy` branch:

1. **No `<ros2_control>` tag in the URDF** (blocking): `FrankaHW::initParameters` reads
   `info.hardware_parameters`, which ros2_control only populates from the URDF
   `<ros2_control><hardware><param>` tags — not from a YAML file like ROS 1's
   `franka_control_node.yaml`. The `ros2_control_node` could therefore never load the
   plugin. Fixed by adding `franka_description/robots/panda.ros2_control.xacro`
   (hardware params + 7 joints + one gpio with the franka state/model and cartesian
   command interfaces) and including it from `panda_arm_hand.urdf.xacro` and
   `dual_panda_example.urdf.xacro`. A new gtest (`franka_hw_urdf_test`) parses the
   generated URDF and asserts the declared interfaces exactly match the exported ones
   (what `resource_manager::validate_storage` checks at startup).

2. **`state_interfaces_` used in `on_configure`** (crash): in ros2_control the loaned
   state/command interfaces are assigned by `controller_manager`'s
   `activate_controllers` — after `configure()` — so `state_interfaces_` is **empty**
   during `on_configure`. All example controllers dereferenced it there (decoding the
   robot-state pointer from element 0). Fixed by moving every state-interface access
   (model-handle creation, start-position checks, initial poses) into `on_activate`.

3. **Commands never reached the robot on `franka_hw/FrankaHW`** (blocking): the
   background libfranka control loop only existed in `FrankaCombinableHW`. The plain
   `FrankaHW` plugin exported command interfaces and `write()` copied them, but nothing
   ever called `run_function_`, so `robot.control(...)` never started. Fixed by moving
   the control-loop thread, error state/`has_error` topic, services and the
   error-recovery action server into `FrankaHW` (shared by both plugins).
   `FrankaCombinableHW` keeps only its torque-only `setRunFunction`.

4. **Out-of-bounds `command_interfaces_` writes** (memory corruption):
   `JointImpedanceExampleController` claimed only the 7 joint-effort command interfaces
   but called `setCartesianPoseCommand(command_interfaces_, ...)`, which writes indices
   0..17 of a 7-element vector. Fixed by claiming the 18 cartesian pose command
   interfaces as well and writing them at offset 7
   (`setCartesianPoseOffsetCommand`); a regression test asserts the claim list.

5. **Joint limits in `checkJointLimits` read from the wrong place**: the ros2_control
   URDF parser stores `<command_interface name="position"><param name="min">` in
   `InterfaceInfo::min/max`, not `ComponentInfo::parameters["lower"/"upper"]` (that was
   the ROS 1 `joint_limits_interface` URDF layout). Fixed to read `min`/`max` from the
   position command interface.

6. **State/model interfaces carried garbage instead of the pointer** (crash): the
   franka state/model interfaces must transport the *address* of the
   `franka::RobotState` / `franka::Model` in the double value, but the port pointed the
   interface at the objects themselves, so `get_value()` returned the first 8 bytes of
   `O_T_EE` and the controllers' `decodePointer` yielded a garbage pointer. Fixed with
   dedicated box-pointer storage (`state_box_ptr_` / `model_box_ptr_` aliasing
   `robot_state_ros_` / `model_`), exactly like upstream `franka_ros2`'s
   `robot_state_box_ptr_`. The `RobotStatePointerRoundtrip` gtest covers the roundtrip.

7. **CI**: `rosdep install --from-paths src` pointed at a nonexistent `src/` directory
   (the repo root holds the packages) — fixed to `--from-paths .`; added gtest coverage
   for the URDF/ros2_control integration and the example controllers to the CI
   `colcon test` run.
