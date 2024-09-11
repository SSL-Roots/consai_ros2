#!/usr/bin/env python3
# coding: UTF-8

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

from consai_msgs.msg import State2D
import copy


class Field:
    # Division A仕様をデフォルトとして設定
    _our_goal_dict = {
        'upper': State2D(x=-0.55, y=0.09),
        'center': State2D(x=-0.55, y=0.0),
        'lower': State2D(x=-0.55, y=-0.09)}
    _their_goal_dict = {
        'upper': State2D(x=0.55, y=0.09),
        'center': State2D(x=0.55, y=0.0),
        'lower': State2D(x=0.55, y=-0.09)}
    _goal_pose = {
        'our': copy.deepcopy(_our_goal_dict),
        'their': copy.deepcopy(_their_goal_dict)}
    _our_penalty_dict = {
        'upper_front': State2D(x=-0.4675, y=0.12),
        'upper_back': State2D(x=-0.55, y=0.12),
        'lower_front': State2D(x=-0.4675, y=-0.12),
        'lower_back': State2D(x=-0.55, y=-0.12)}
    _their_penalty_dict = {
        'upper_front': State2D(x=0.4675, y=0.12),
        'upper_back': State2D(x=0.55, y=0.08),
        'lower_front': State2D(x=0.4675, y=-0.12),
        'lower_back': State2D(x=0.55, y=-0.08)}
    _penalty_pose = {
        'our': copy.deepcopy(_our_penalty_dict),
        'their': copy.deepcopy(_their_penalty_dict)}
    _field = {
        'length':         1.1,
        'width':          0.6,
        'half_length':    0.55,
        'quarter_length': 0.275,
        'half_width':     0.3,
        'quarter_width':  0.15}
    _defense_area = {
        'length':      0.165,
        'width':       0.24,
        'helf_length': 0.0825,
        'helf_width':  0.12}
    _geometry_field_lines = {}

    # _penalty_poseとfield_lines紐付ける辞書
    _field_lines_to_penalty_pose = {
        'our_upper_front': ["LeftPenaltyStretch", 1],
        'our_upper_back': ["LeftFieldLeftPenaltyStretch", 0],
        'our_lower_front': ["LeftPenaltyStretch", 0],
        'our_lower_back': ["LeftFieldRightPenaltyStretch", 0],
        'their_upper_front': ["RightPenaltyStretch", 1],
        'their_upper_back': ["RightFieldRightPenaltyStretch", 0],
        'their_lower_front': ["RightPenaltyStretch", 0],
        'their_lower_back': ["RightFieldLeftPenaltyStretch", 0],
    }

    @classmethod
    def goal_pose(cls, team='our', position='center'):
        return Field._goal_pose[team][position]

    @classmethod
    def penalty_pose(cls, team='our', position='upper_front'):
        return Field._penalty_pose[team][position]

    @classmethod
    def field(cls, param='length'):
        return Field._field[param]

    @classmethod
    def defense_area(cls, param='length'):
        return Field._defense_area[param]
