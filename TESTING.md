# Testing the ROS 2 Jazzy port on a real robot

This document describes how to validate the ROS 2 (jazzy / ros2_control) port of
`franka_ros` on real Franka hardware. The port keeps **libfranka 0.7.0** pinned
(`find_package(Franka 0.7.0 REQUIRED)`), so the FCI behavior is identical to the
ROS 1 kinetic stack; everything that changed is ROS-facing glue
(ros2_control hardware plugin, controller lifecycle, launch files).

CI (`jazzy-ci.yaml`) proves the port *builds* and the robot-independent unit tests
pass (`franka_hw_test`, `franka_hw_urdf_test`, `example_controllers_test`). It cannot
prove the robot actually moves — that is what this checklist is for. Work through the
stages in order; **do not skip stage 0 or stage 1**, they catch the failure modes the
review found (missing URDF `<ros2_control>` tag, interface-name mismatches, garbage
state pointers) without any motion risk.

## Safety notes (read first)

- Have someone near the robot's e-stop at all times during the first runs of each stage.
- Start every motion stage from the "move_to_start" pose (`0, -π/4, 0, -3π/4, 0, π/2, π/4`)
  used by the example controllers.
- The desk must be in "Active mode" with the FCI enabled and the joints unlocked before
  launching any controller that claims effort interfaces.
- First motion tests should use the **joint position** example (stage 5) — position
  commands are internally impedance-controlled and are the most forgiving. Do not start
  with torque control.
- `realtime_config: enforce` is the default in the URDF xacro. If the control thread
  cannot get realtime priority on your kernel, libfranka will throw; switch the URDF
  parameter to `ignore` for bench testing, but never ship it that way.

## 0. Pre-flight (no robot needed)

Run on the workstation against the merged branch:

```bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
colcon test && colcon test-result --verbose
```

Then generate the URDF and verify the `ros2_control` tag is present and complete:

```bash
source install/setup.bash
xacro $(ros2 pkg prefix --share franka_description)/robots/panda_arm_hand.urdf.xacro \
      arm_id:=panda robot_ip:=<ROBOT_IP> > /tmp/panda.urdf
grep -A 30 "<ros2_control" /tmp/panda.urdf
```

Checklist:

- [ ] `colcon test` green (all three gtest suites).
- [ ] The `<ros2_control>` block shows `franka_hw/FrankaCombinableHW`, the
      `arm_id`/`robot_ip` params and all 7 joints.
- [ ] `libfranka` in use is 0.7.0 (`ldd install/franka_hw/lib/libfranka_hw.so | grep franka`).

## 1. Hardware bring-up, no controller (robot connect + state only)

```bash
ros2 launch franka_control franka_control.launch.py robot_ip:=<ROBOT_IP> load_gripper:=true arm_id:=panda
```

What happens: the `controller_manager` loads the `FrankaCombinableHW` system, which
connects via libfranka, sets the collision behavior from the URDF params and starts the
background loop (state only, no motion). The launch file also spawns the
`franka_state_controller`.

Verify:

- [ ] No `Discrepancy between robot description file (urdf) and actually exported HW
      interfaces` error — this is `resource_manager::validate_storage` rejecting a URDF
      that does not match the exported interfaces (the port's #1 failure mode).
- [ ] `ros2 control list_hardware_interfaces` shows all 21 joint interfaces, the
      `panda/franka_hw/FrankaStateInterface`, `panda/franka_hw/FrankaModelInterface` and
      the 26 cartesian command interfaces, all unclaimed (command ones).
- [ ] `ros2 topic hz /franka_states` ≈ 30 Hz and the joint values match the desk.
- [ ] `ros2 topic echo /joint_states --once` shows plausible q/dq/tau for the 7 joints.
- [ ] `ros2 topic echo /franka_states --once` shows the full `FrankaState` (including
      `cartesian_contact`, `o_f_ext_hat_k`, `robot_time`-style fields) matching the desk.
- [ ] TF tree present: `ros2 run tf2_tools view_frames` — `panda_link0 … panda_link8`,
      `panda_EE`, `panda_K`.
- [ ] Push the robot lightly in guiding mode: `ros2 topic echo /joint_states` joint
      positions track the motion (validates the state interface pointer plumbing).
- [ ] The services exist: `ros2 service list | grep set_` shows `/set_joint_impedance`,
      `/set_cartesian_impedance`, `/set_EE_frame`, `/set_K_frame`, `/set_load`,
      `/set_force_torque_collision_behavior`, `/set_full_collision_behavior`.
- [ ] The error-recovery action server is advertised: `ros2 action list` shows
      `/error_recovery` (plus the `/franka_gripper/*` actions if the gripper node is
      running).

## 2. Collision behavior and services

With the robot still in the state-only setup:

- [ ] `ros2 service call /set_joint_impedance franka_msgs/srv/SetJointImpedance
      "{joint_stiffness: [1500.0, 1500.0, 1000.0, 1000.0, 500.0, 300.0, 100.0]}"` returns
      `success: true` and the desk shows the impedance change.
- [ ] `ros2 service call /set_full_collision_behavior ...` with the same defaults as the
      URDF succeeds.
- [ ] Trigger a mild reflex (tap a link gently during a later motion stage) and confirm
      `ros2 topic echo /has_error` flips to `true` and the robot stops (the `has_error`
      path moved into the base `FrankaHW` during review — this is its live check).

## 3. Gripper (if `load_gripper:=true`)

The `franka_gripper_node` exposes `franka_gripper/action/Homing`, `Stop`, `Move`,
`Grasp`, plus the `control_msgs/action/GripperCommand` server under `gripper_action`.

- [ ] `ros2 action send_goal /franka_gripper/homing franka_gripper/action/Homing "{}"`
      — the gripper homes (fingers fully open).
- [ ] `ros2 action send_goal /franka_gripper/grasp franka_gripper/action/Grasp
      "{width: 0.0, epsilon: {inner: 0.005, outer: 0.005}, speed: 0.1, force: 10.0}"`
      grasps and reports `success: true`.
- [ ] `ros2 action send_goal /franka_gripper/move franka_gripper/action/Move
      "{width: 0.08, speed: 0.1}"` opens to 8 cm.
- [ ] `ros2 action send_goal /franka_gripper/stop franka_gripper/action/Stop "{}"` stops
      an ongoing motion.
- [ ] `ros2 action send_goal /franka_gripper/gripper_action control_msgs/action/GripperCommand
      "{command: {position: 0.0, max_effort: 10.0}}"` works (MoveIt-style interface).
- [ ] `/franka_gripper/joint_states` publishes the two finger joints.

## 4. Error recovery round-trip

- [ ] Trigger a reflex (e.g. joint-position-limit violation or a gentle collision during
      the stage-5 motion).
- [ ] Confirm the active controller is deactivated by the error and `has_error` is true.
- [ ] `ros2 action send_goal /error_recovery franka_msgs/action/ErrorRecovery` clears
      the desk error; the goal succeeds and `has_error` flips back to false.
- [ ] Re-activating the controller works without restarting the stack.

## 5. First motion — joint position example

From the move-to-start pose:

```bash
ros2 launch franka_example_controllers joint_position_example_controller.launch.py \
    robot_ip:=<ROBOT_IP> load_gripper:=false
```

Verify:

- [ ] The spawner activates `joint_position_example_controller`; `ros2 control list_controllers`
      shows it `[active]`.
- [ ] The robot performs the small sinusoidal joint motion around the start pose —
      smooth, no jerk at activation (validates `on_activate` reading the joint position
      state interfaces at the right lifecycle point).
- [ ] `ros2 control list_hardware_interfaces` shows the 7 position command interfaces
      claimed by the controller.
- [ ] Deactivate (`ros2 service call /controller_manager/deactivate_controller ...`) and
      re-activate — the motion restarts cleanly (regression for the lifecycle changes).

## 6. Cartesian examples (pose, velocity, elbow)

```bash
ros2 launch franka_example_controllers cartesian_pose_example_controller.launch.py robot_ip:=<ROBOT_IP>
ros2 launch franka_example_controllers cartesian_velocity_example_controller.launch.py robot_ip:=<ROBOT_IP>
ros2 launch franka_example_controllers elbow_example_controller.launch.py robot_ip:=<ROBOT_IP>
```

- [ ] Each motion follows the expected small circle/segment; deactivating the controller
      stops motion immediately (cartesian command interfaces released).
- [ ] While a cartesian controller is active, `ros2 control list_hardware_interfaces`
      shows the `franka_hw/FrankaPoseCartesianInterface/*` (or velocity) command
      interfaces claimed — this proves the GPIO cartesian command path through
      ros2_control, which ROS 1 handled via `FrankaCartesianPoseHandle`.

## 7. Torque and impedance examples

```bash
ros2 launch franka_example_controllers joint_impedance_example_controller.launch.py robot_ip:=<ROBOT_IP>
ros2 launch franka_example_controllers cartesian_impedance_example_controller.launch.py robot_ip:=<ROBOT_IP>
```

- [ ] Joint impedance: the robot tracks the circular path; `ros2 topic echo /torque_comparison`
      shows commanded vs measured torques, RMS error small.
- [ ] Cartesian impedance: the robot is compliant — push the end effector gently and it
      yields; `ros2 topic pub /equilibrium_pose ...` moves the setpoint. Tune stiffness
      online via `ros2 param set ...` and feel the difference (dynamic-parameter path).
- [ ] These are the first stages where torque commands actually reach the robot through
      the background libfranka loop; watch for `FrankaHW::control_loop(): controller is
      active.` in the logs at activation — that message is the confirmation the loop fix
      works.

## 8. Controller switching (regression for `prepare/perform_command_mode_switch`)

```bash
ros2 control switch_controllers --activate cartesian_impedance_example_controller --deactivate joint_position_example_controller
```

- [ ] Switching position↔torque controllers does not throw a libfranka
      `InvalidOperationException` and the robot does not jerk between the two.
- [ ] `ros2 control list_hardware_interfaces` reflects the claimed-interface changes.
- [ ] Switching back and forth repeatedly (10+ times) stays stable.

## 9. Dual-arm (if applicable)

```bash
# generate the dual-arm URDF with both ros2_control blocks and launch two
# controller_managers or one with two FrankaCombinableHW systems
```

- [ ] Both `FrankaCombinableHW` instances load (two `ros2_control` blocks in the
      dual-panda URDF, `arm_id:=panda_1` / `panda_2`).
- [ ] `dual_arm_cartesian_impedance_example_controller` activates and both arms render
      the centering-frame compliance; the `/centering_frame` topic publishes.
- [ ] An error on one arm stops both (the `triggerError` cross-link).

## 10. Endurance and shutdown

- [ ] Run the cartesian impedance example for ≥ 30 minutes: no state dropouts
      (`ros2 topic hz /franka_states` stays ≈ 30), no reflex, stable memory.
- [ ] `Ctrl-C` on the launch file: controllers deactivate, the control thread joins
      cleanly (no hang on shutdown, no segfault) — validates the `FrankaHW` destructor.
- [ ] Power-cycling the robot mid-run and relaunching recovers without a reboot of the
      workstation.

## Known-good reference behavior (ROS 1 parity)

Wherever behavior feels different from the kinetic stack, compare against
`kinetic-devel`: the libfranka layer is identical, so any difference should come from
the ros2_control glue — controller activation timing, interface claiming, or the
lifecycle. File regressions against the `jazzy` branch (which stays open for
hardware-validation fixes).
