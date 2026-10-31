# ROS integration for Franka Emika research robots (ROS 2 Jazzy port)

[![Jazzy CI](https://github.com/tstoyanov/franka_ros/actions/workflows/jazzy-ci.yaml/badge.svg?branch=jazzy)](https://github.com/tstoyanov/franka_ros/actions/workflows/jazzy-ci.yaml?query=branch%3Ajazzy)

See the [Franka Control Interface (FCI) documentation][fci-docs] for more information.

This branch (`jazzy`) ports the original ROS 1 `kinetic-devel` code to ROS 2 Jazzy while
keeping **libfranka 0.7.0** pinned for hardware compatibility. See
[docs/jazzy_migration.md](docs/jazzy_migration.md) for the migration procedure and progress
checklist. CI builds and tests with `colcon` in
[`tstoyanov/jazzy_libfranka:latest`](https://hub.docker.com/r/tstoyanov/jazzy_libfranka)
(ros:jazzy + libfranka 0.7.0).

## Quick start

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to franka_ros
source install/setup.bash
ros2 launch franka_control franka_control.launch.py robot_ip:=<robot-ip>
ros2 launch franka_example_controllers joint_position_example_controller.launch.py robot_ip:=<robot-ip>
```

## License

All packages of `franka_ros` are licensed under the [Apache 2.0 license][apache-2.0].

[apache-2.0]: https://www.apache.org/licenses/LICENSE-2.0.html
[fci-docs]: https://frankaemika.github.io/docs
