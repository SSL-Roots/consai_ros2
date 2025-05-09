# Copyright 2021 Roots
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import launch
from launch.actions import DeclareLaunchArgument
from launch.actions.shutdown_action import Shutdown
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """launchを生成する関数"""
    declare_arg_gui = DeclareLaunchArgument(
        "gui",
        default_value="true",
        description=('Set "true" to run consai_visualizer.'),
    )

    declare_arg_invert = DeclareLaunchArgument(
        "invert",
        default_value="false",
        description=('Set "true" to invert detection_tracked data.'),
    )

    declare_arg_yellow = DeclareLaunchArgument(
        "yellow",
        default_value="false",
        description=('Set "true" to control yellow team robots.'),
    )

    declare_arg_vision_addr = DeclareLaunchArgument(
        "vision_addr",
        default_value="224.5.23.2",
        description=("Set multicast address to connect SSL-Vision."),
    )

    declare_arg_vision_port = DeclareLaunchArgument(
        "vision_port",
        default_value="10006",
        description=("Set multicast port to connect SSL-Vision."),
    )

    declare_arg_robot_control_ip = DeclareLaunchArgument(
        "robot_control_ip",
        default_value="127.0.0.1",
        description=("Set GrSim control address."),
    )

    declare_arg_robot_control_port = DeclareLaunchArgument(
        "robot_control_port",
        default_value="20011",
        description=("Set GrSim control port."),
    )

    container = ComposableNodeContainer(
        name="test_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            ComposableNode(
                package="consai_robot_controller",
                plugin="consai_robot_controller::Controller",
                name="controller",
                parameters=[
                    {"team_is_yellow": LaunchConfiguration("yellow")},
                    {"invert": LaunchConfiguration("invert")},
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            ComposableNode(
                package="consai_robot_controller",
                plugin="consai_robot_controller::GrSimCommandConverter",
                name="command_converter",
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            ComposableNode(
                package="consai_vision_tracker",
                plugin="consai_vision_tracker::Tracker",
                name="tracker",
                parameters=[{"invert": LaunchConfiguration("invert")}],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            ComposableNode(
                package="robocup_ssl_comm",
                plugin="robocup_ssl_comm::Vision",
                name="vision",
                extra_arguments=[{"use_intra_process_comms": True}],
                parameters=[
                    {
                        "multicast_address": LaunchConfiguration("vision_addr"),
                        "multicast_port": LaunchConfiguration("vision_port"),
                    }
                ],
            ),
            ComposableNode(
                package="robocup_ssl_comm",
                plugin="robocup_ssl_comm::GrSim",
                name="grsim",
                parameters=[
                    {
                        "robot_control_ip": LaunchConfiguration("robot_control_ip"),
                        "robot_control_port": LaunchConfiguration("robot_control_port"),
                    }
                ],
            ),
        ],
        output="screen",
        on_exit=Shutdown(),
    )

    visualizer = Node(
        package="consai_visualizer",
        executable="consai_visualizer",
        output="screen",
        parameters=[{"invert": LaunchConfiguration("invert")}],
        condition=IfCondition(LaunchConfiguration("gui")),
    )

    return launch.LaunchDescription(
        [
            declare_arg_gui,
            declare_arg_invert,
            declare_arg_yellow,
            declare_arg_vision_addr,
            declare_arg_vision_port,
            declare_arg_robot_control_ip,
            declare_arg_robot_control_port,
            container,
            visualizer,
        ]
    )
