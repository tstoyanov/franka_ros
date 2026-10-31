# Copyright (c) 2017 Franka Emika GmbH
# Use of this source code is governed by the Apache-2.0 license, see LICENSE
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_ip = LaunchConfiguration("robot_ip")
    arm_id = LaunchConfiguration("arm_id")

    config = PathJoinSubstitution(
        [FindPackageShare("franka_gripper"), "config", "franka_gripper_node.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_ip"),
            DeclareLaunchArgument("arm_id", default_value="panda"),
            Node(
                package="franka_gripper",
                executable="franka_gripper_node",
                name="franka_gripper",
                parameters=[
                    config,
                    {
                        "robot_ip": robot_ip,
                        "joint_names": [
                            arm_id + "_finger_joint1",
                            arm_id + "_finger_joint2",
                        ],
                    },
                ],
                output="screen",
            ),
        ]
    )
