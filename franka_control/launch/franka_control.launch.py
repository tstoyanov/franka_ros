# Copyright (c) 2017 Franka Emika GmbH
# Use of this source code is governed by the Apache-2.0 license, see LICENSE
"""Launch the ros2_control controller manager with the FrankaHW hardware plugin."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_ip = LaunchConfiguration("robot_ip")
    load_gripper = LaunchConfiguration("load_gripper")
    arm_id = LaunchConfiguration("arm_id")

    robot_description_content = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("franka_description"), "robots", "panda_arm_hand.urdf.xacro"]
            ),
            " arm_id:=",
            arm_id,
            " robot_ip:=",
            robot_ip,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    controllers_file = PathJoinSubstitution(
        [FindPackageShare("franka_control"), "config", "default_controllers.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_ip", description="IP of the robot"),
            DeclareLaunchArgument("load_gripper", default_value="true"),
            DeclareLaunchArgument("arm_id", default_value="panda"),
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                parameters=[robot_description, controllers_file],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["franka_state_controller", "-c", "/controller_manager"],
                output="screen",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[robot_description],
                output="screen",
            ),
        ]
    )
