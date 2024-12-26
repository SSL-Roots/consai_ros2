# Copyright 2024 Roots
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
from launch_ros.actions import Node
import os


def generate_launch_description():

    config_rule = os.path.join(get_package_share_directory('consai_description'), 'config', 'rule', 'division_a.yaml')

    publisher_node = Node(
        package='consai_description',
        executable='parameter_publisher.py',
        output='screen',
        arguments=[
            '--config_rule', config_rule,
            ],
        # parameters=[
        #     config_rule_file_path
        # ]
        )

    ld = launch.LaunchDescription()
    ld.add_action(publisher_node)

    return ld
