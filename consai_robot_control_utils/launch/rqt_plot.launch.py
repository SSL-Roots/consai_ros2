# Copyright 2023 Roots
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

from distutils.util import strtobool
import launch
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def make_rqt_plot_node(context):
    robot_id = int(LaunchConfiguration('robot_id').perform(context))
    yellow = strtobool(LaunchConfiguration('yellow').perform(context))

    local_vel_index = robot_id
    if yellow:
        local_vel_index += 16

    command_prefix = '/robot{}/command'.format(robot_id)
    local_vel_prefix = '/robot_local_velocities/velocities[{}]/velocity'.format(local_vel_index)

    node = Node(
        package='rqt_plot',
        executable='rqt_plot',
        output='screen',
        arguments=['topics',
                   command_prefix + '/velocity_x',
                   command_prefix + '/velocity_y',
                   command_prefix + '/velocity_theta',
                   local_vel_prefix + '/x',
                   local_vel_prefix + '/y',
                   local_vel_prefix + '/theta',
                   ]
    )

    return [node]


def generate_launch_description():
    declare_arg_yellow = DeclareLaunchArgument(
        'yellow', default_value='false',
        description=('Set "false" to visualize blue team robot information.')
    )

    declare_arg_robot_id = DeclareLaunchArgument(
        'robot_id', default_value='0',
        description=('Set robot ID to visualize.')
    )

    rqt_plot = OpaqueFunction(function=make_rqt_plot_node)

    return launch.LaunchDescription([
        declare_arg_yellow,
        declare_arg_robot_id,
        rqt_plot,
    ])
