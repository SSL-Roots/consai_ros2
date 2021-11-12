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
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    declare_arg_invert = DeclareLaunchArgument(
        'invert', default_value='false',
        description=('Set "true" to invert detection_tracked data.')
    )

    declare_arg_yellow = DeclareLaunchArgument(
        'yellow', default_value='false',
        description=('Set "true" to control yellow team robots.')
    )

    controller = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('consai_robot_controller'),
                '/launch/test.launch.py']),
            launch_arguments={'invert': LaunchConfiguration('invert'),
                              'yellow': LaunchConfiguration('yellow')}.items(),
        )

    container = ComposableNodeContainer(
            name='referee_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container_mt',
            composable_node_descriptions=[
                ComposableNode(
                    package='robocup_ssl_comm',
                    plugin='robocup_ssl_comm::GameController',
                    name='game_controller')
            ])

    return launch.LaunchDescription([
        declare_arg_invert,
        declare_arg_yellow,
        controller,
        container
    ])
