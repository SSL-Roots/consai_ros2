#!/usr/bin/env python3

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

"""フィールドの位置データを管理し, ゴールやペナルティの位置を提供するモジュール."""

import copy

from consai_msgs.msg import State2D


class FieldPositions:
    """フィールド上の位置を管理するクラス."""

    def __init__(self):
        """フィールドの初期位置を設定する関数."""
        self._set_positions()

    def set_field_size(self, length=12.0, goal_width=9.0, penalty_depth=1.8, penalty_width=3.6):
        """フィールドのサイズを設定する関数."""
        self._set_positions(
            length * 0.5,
            goal_width * 0.5,
            penalty_depth,
            penalty_width * 0.5)

    def _set_positions(
            self, half_length=6.0, half_goal_width=0.9, penalty_depth=1.8, half_penalty_width=1.8):
        """フィールド上のゴールとペナルティの位置を設定する関数."""
        _our_goal_dict = {
            'upper': State2D(x=-half_length, y=half_goal_width),
            'center': State2D(x=-half_length, y=0.0),
            'lower': State2D(x=-half_length, y=-half_goal_width)}
        _their_goal_dict = {
            'upper': State2D(x=half_length, y=half_goal_width),
            'center': State2D(x=half_length, y=0.0),
            'lower': State2D(x=half_length, y=-half_goal_width)}
        self._goal_pose = {
            'our': copy.deepcopy(_our_goal_dict),
            'their': copy.deepcopy(_their_goal_dict)}

        penalty_x = half_length - penalty_depth

        _our_penalty_dict = {
            'upper_front': State2D(x=-penalty_x, y=half_penalty_width),
            'upper_back': State2D(x=-half_length, y=half_penalty_width),
            'lower_front': State2D(x=-penalty_x, y=-half_penalty_width),
            'lower_back': State2D(x=-half_length, y=-half_penalty_width)}
        _their_penalty_dict = {
            'upper_front': State2D(x=penalty_x, y=half_penalty_width),
            'upper_back': State2D(x=half_length, y=half_penalty_width),
            'lower_front': State2D(x=penalty_x, y=-half_penalty_width),
            'lower_back': State2D(x=half_length, y=-half_penalty_width)}
        self._penalty_pose = {
            'our': copy.deepcopy(_our_penalty_dict),
            'their': copy.deepcopy(_their_penalty_dict)}

    def goal_pose(self, team='our', position='center'):
        """指定したチームの指定位置のゴール位置を取得する関数."""
        return self._goal_pose[team][position]

    def penalty_pose(self, team='our', position='upper_front'):
        """指定したチームの指定位置のペナルティ位置を取得する関数."""
        return self._penalty_pose[team][position]
