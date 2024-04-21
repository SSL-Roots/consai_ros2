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
        'upper': State2D(x=-6.0, y=0.9),
        'center': State2D(x=-6.0, y=0.0),
        'lower': State2D(x=-6.0, y=-0.9)}
    _their_goal_dict = {
        'upper': State2D(x=6.0, y=0.9),
        'center': State2D(x=6.0, y=0.0),
        'lower': State2D(x=6.0, y=-0.9)}
    _goal_pose = {
        'our': copy.deepcopy(_our_goal_dict),
        'their': copy.deepcopy(_their_goal_dict)}
    _our_penalty_dict = {
        'upper_front': State2D(x=-4.2, y=1.8),
        'upper_back': State2D(x=-6.0, y=1.8),
        'lower_front': State2D(x=-4.2, y=-1.8),
        'lower_back': State2D(x=-6.0, y=-1.8)}
    _their_penalty_dict = {
        'upper_front': State2D(x=4.2, y=1.8),
        'upper_back': State2D(x=6.0, y=1.2),
        'lower_front': State2D(x=4.2, y=-1.8),
        'lower_back': State2D(x=6.0, y=-1.2)}
    _penalty_pose = {
        'our': copy.deepcopy(_our_penalty_dict),
        'their': copy.deepcopy(_their_penalty_dict)}
    _field = {
        'length':         12.0,
        'width':          9.0,
        'half_length':    6.0,
        'quarter_length': 3.0,
        'half_width':     4.5,
        'quarter_width':  2.25}
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
