# Copyright (c) 2017 Franka Emika GmbH
# Use of this source code is governed by the Apache-2.0 license, see LICENSE
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_ip = LaunchConfiguration("robot_ip")
    load_gripper = LaunchConfiguration("load_gripper")

    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_ip"),
            DeclareLaunchArgument("load_gripper", default_value="true"),
            IncludeLaunchDescription(
                PathJoinSubstitution(
                    [FindPackageShare("franka_control"), "launch", "franka_control.launch.py"]
                ),
                launch_arguments={"robot_ip": robot_ip, "load_gripper": load_gripper}.items(),
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["dual_arm_cartesian_impedance_example_controller", "-c", "/controller_manager"],
            ),
        ]
    )
