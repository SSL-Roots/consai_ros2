# coding: UTF-8

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


from consai_examples.observer.pos_vel import PosVel
from consai_visualizer_msgs.msg import Objects
from consai_visualizer_msgs.msg import ShapeText


def to_visualize_msg(role_dict: dict[int, str], our_robots: dict[int, PosVel]) -> Objects:
    vis_objects = Objects()

    vis_objects.layer = 'game'
    vis_objects.sub_layer = 'role'
    vis_objects.z_order = 5

    for robot_id, role in role_dict.items():
        if robot_id not in our_robots:
            continue
        robot_pos = our_robots[robot_id].pos()

        shape = ShapeText()
        shape.x = robot_pos.x + 0.1
        shape.y = robot_pos.y + 0.1
        shape.text = role
        shape.size = 8
        shape.color.name = 'white'
        vis_objects.texts.append(shape)

    return vis_objects
