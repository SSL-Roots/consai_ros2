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

from ament_index_python.packages import get_package_share_directory
import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import Node
import os

def generate_launch_description():
    pid_gains = os.path.join(
        get_package_share_directory('consai_robot_controller'),
        'config',
        'pid_gains.yaml'
        )

    team = os.path.join(
        get_package_share_directory('consai_robot_controller'),
        'config',
        'team.yaml'
        )


    container = ComposableNodeContainer(
            name='test_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container_mt',
            composable_node_descriptions=[
                ComposableNode(
                    package='consai_robot_controller',
                    plugin='consai_robot_controller::Controller',
                    name='controller',
                    parameters=[pid_gains, team],
                    extra_arguments=[{'use_intra_process_comms': True}],
                    ),
                ComposableNode(
                    package='consai_robot_controller',
                    plugin='consai_robot_controller::GrSimCommandConverter',
                    name='command_converter',
                    parameters=[team],
                    extra_arguments=[{'use_intra_process_comms': True}],
                    ),
                ComposableNode(
                    package='consai_vision_tracker',
                    plugin='consai_vision_tracker::Tracker',
                    name='tracker',
                    extra_arguments=[{'use_intra_process_comms': True}],
                    ),
                ComposableNode(
                    package='robocup_ssl_comm',
                    plugin='robocup_ssl_comm::Vision',
                    name='vision',
                    extra_arguments=[{'use_intra_process_comms': True}],
                    ),
                ComposableNode(
                    package='robocup_ssl_comm',
                    plugin='robocup_ssl_comm::GrSim',
                    name='grsim'),
            ],
            output='screen',
    )

    visualizer = Node(
        package='consai_visualizer',
        executable='consai_visualizer',
        output='screen',
    )

    return launch.LaunchDescription([container, visualizer])
