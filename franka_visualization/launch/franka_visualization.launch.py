# Copyright (c) 2019 Franka Emika GmbH
# Use of this source code is governed by the Apache-2.0 license, see LICENSE
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    robot_ip = LaunchConfiguration("robot_ip")
    arm_id = LaunchConfiguration("arm_id")

    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_ip"),
            DeclareLaunchArgument("arm_id", default_value="panda"),
            Node(
                package="franka_visualization",
                executable="robot_joint_state_publisher",
                parameters=[
                    {
                        "robot_ip": robot_ip,
                        "publish_rate": 30.0,
                        "joint_names": [
                            arm_id + "_joint" + str(i) for i in range(1, 8)
                        ],
                    }
                ],
                output="screen",
            ),
            Node(
                package="franka_visualization",
                executable="gripper_joint_state_publisher",
                parameters=[
                    {
                        "robot_ip": robot_ip,
                        "publish_rate": 30.0,
                        "joint_names": [
                            arm_id + "_finger_joint1",
                            arm_id + "_finger_joint2",
                        ],
                    }
                ],
                output="screen",
            ),
        ]
    )
