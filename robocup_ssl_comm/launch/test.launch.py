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
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    declare_arg_vision_addr = DeclareLaunchArgument(
        'vision_addr', default_value='224.5.23.2',
        description=('Set multicast address to connect SSL-Vision.')
    )

    declare_arg_vision_port = DeclareLaunchArgument(
        'vision_port', default_value='10006',
        description=('Set multicast port to connect SSL-Vision.')
    )

    declare_arg_referee_addr = DeclareLaunchArgument(
        'referee_addr', default_value='224.5.23.1',
        description=('Set multicast address to connect Game Controller.')
    )

    declare_arg_referee_port = DeclareLaunchArgument(
        'referee_port', default_value='10003',
        description=('Set multicast port to connect Game Controller.')
    )

    container = ComposableNodeContainer(
            name='test_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container_mt',
            composable_node_descriptions=[
                ComposableNode(
                    package='robocup_ssl_comm',
                    plugin='robocup_ssl_comm::Vision',
                    name='vision',
                    parameters=[{
                        'multicast_address': LaunchConfiguration('vision_addr'),
                        'multicast_port': LaunchConfiguration('vision_port'),
                        }],
                    ),
                ComposableNode(
                    package='robocup_ssl_comm',
                    plugin='robocup_ssl_comm::GameController',
                    name='game_controller',
                    parameters=[{
                        'multicast_address': LaunchConfiguration('referee_addr'),
                        'multicast_port': LaunchConfiguration('referee_port'),
                        }],
                    ),
                ComposableNode(
                    package='robocup_ssl_comm',
                    plugin='robocup_ssl_comm::GrSim',
                    name='grsim'),
            ],
            output='screen',
    )

    return launch.LaunchDescription([
        declare_arg_vision_addr,
        declare_arg_vision_port,
        declare_arg_referee_addr,
        declare_arg_referee_port,
        container
    ])
