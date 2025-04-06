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
from launch.actions.shutdown_action import Shutdown
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PythonExpression
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import Node
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

    declare_arg_game = DeclareLaunchArgument(
        'game', default_value='false',
        description=('Set "true" to run game script.')
    )

    declare_arg_goalie = DeclareLaunchArgument(
        'goalie', default_value='0',
        description=('Set goalie id for game script.')
    )

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

    declare_arg_robot_control_ip = DeclareLaunchArgument(
        'robot_control_ip', default_value='127.0.0.1',
        description=('Set GrSim control address.')
    )

    declare_arg_robot_control_port = DeclareLaunchArgument(
        'robot_control_port', default_value='20011',
        description=('Set GrSim control port.')
    )

    param_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('consai_description'),
            '/launch/parameter_publisher.launch.py'])
    )

    controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('consai_robot_controller'),
            '/launch/controller.launch.py']),
        launch_arguments={'invert': LaunchConfiguration('invert'),
                          'yellow': LaunchConfiguration('yellow'),
                          'vision_addr': LaunchConfiguration('vision_addr'),
                          'vision_port': LaunchConfiguration('vision_port'),
                          'robot_control_ip': LaunchConfiguration('robot_control_ip'),
                          'robot_control_port': LaunchConfiguration('robot_control_port'),
                          }.items(),
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
                    parameters=[{
                        'multicast_address': LaunchConfiguration('referee_addr'),
                        'multicast_port': LaunchConfiguration('referee_port'),
                    }],
                    name='game_controller')
        ])

    cmd_arg_yellow = ['"--yellow" if "true" == "', LaunchConfiguration('yellow'), '" else ""']
    cmd_arg_invert = ['"--invert" if "true" == "', LaunchConfiguration('invert'), '" else ""']
    game_node = Node(package='consai_examples',
                     executable='game.py',
                     output='screen',
                     arguments=[
                         PythonExpression(cmd_arg_yellow),
                         PythonExpression(cmd_arg_invert),
                         '--goalie', LaunchConfiguration('goalie')
                     ],
                     condition=IfCondition(LaunchConfiguration('game')),
                     on_exit=Shutdown(),
                     )

    set_colorized_output = launch.actions.SetEnvironmentVariable(
        name='RCUTILS_COLORIZED_OUTPUT', value='1')

    return launch.LaunchDescription([
        set_colorized_output,
        declare_arg_invert,
        declare_arg_yellow,
        declare_arg_game,
        declare_arg_goalie,
        declare_arg_vision_addr,
        declare_arg_vision_port,
        declare_arg_referee_addr,
        declare_arg_referee_port,
        declare_arg_robot_control_ip,
        declare_arg_robot_control_port,
        param_publisher,
        controller,
        container,
        game_node
    ])
