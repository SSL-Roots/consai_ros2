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

"""ロボットの意思決定を行うためのクラス群を提供するモジュール."""

from consai_examples.field_observer import FieldObserver
from consai_examples.operation import Operation


class DecisionBase(object):
    """ロボットの意思決定を行うクラス."""

    def __init__(self, robot_operator, field_observer: FieldObserver):
        """DecisionBaseクラスの初期化処理,ロボットオペレーターとフィールドオブザーバーを設定する関数."""
        self._operator = robot_operator
        self._field_observer = field_observer

        self._div_a_x = self._field_observer.on_div_a_x
        self._div_a_y = self._field_observer.on_div_a_y
        self._div_a_dia = self._field_observer.on_div_a_robot_diameter
        self._field_pos = self._field_observer.field_pos

        self._num_of_center_back_roles = 0
        self._num_of_side_back_roles = 0
        self._num_of_zone_roles = 0
        self.command_elapsed_time = 0.0

    def _penalty_wait_x(self):
        """ペナルティキック待機位置のX座標を返す関数."""
        return self._div_a_x(4.1)

    def enable_stop_game_velocity(self, robot_id):
        """ロボットのストップゲーム速度を有効にする関数."""
        self._operator.enable_stop_game_velocity(robot_id)

    def disable_stop_game_velocity(self, robot_id):
        """ロボットのストップゲーム速度を無効にする関数."""
        self._operator.disable_stop_game_velocity(robot_id)

    def enable_avoid_obstacles(self, robot_id):
        """ロボットの障害物回避機能を有効にする関数."""
        self._operator.enable_avoid_obstacles(robot_id)

    def disable_avoid_obstacles(self, robot_id):
        """ロボットの障害物回避機能を無効にする関数."""
        self._operator.disable_avoid_obstacles(robot_id)

    def set_num_of_center_back_roles(self, num_of_center_back_roles):
        """センターバック役割の数を設定する関数."""
        self._num_of_center_back_roles = num_of_center_back_roles

    def set_num_of_side_back_roles(self, num_of_side_back_roles):
        """サイドバック役割の数を設定する関数."""
        self._num_of_side_back_roles = num_of_side_back_roles

    def set_num_of_zone_roles(self, num_of_zone_roles):
        """ゾーン役割の数を設定する関数."""
        self._num_of_zone_roles = num_of_zone_roles

    def reset_operation(self, robot_id: int) -> None:
        """ロボットの操作をリセットする関数."""
        self._operator.reset_operation(robot_id)

    def our_ball_placement(self, robot_id, placement_pos):
        """ボールを自チームの配置位置にセットする関数."""
        print("OUR BALL PLACEMENT:{} to x:{}, y:{}".format(
            robot_id, placement_pos.x, placement_pos.y))
        operation_halt = Operation().halt()
        self._operator.operate(robot_id, operation_halt)

    def their_ball_placement(self, robot_id, placement_pos):
        """ボールを相手チームの配置位置にセットする関数."""
        print("THEIR BALL PLACEMENT:{} to x:{}, y:{}".format(
            robot_id, placement_pos.x, placement_pos.y))
        operation_halt = Operation().halt()
        self._operator.operate(robot_id, operation_halt)

    def set_command_elapsed_time(self, command_elapsed_time):
        """コマンドの経過時間を設定する関数."""
        self._command_elapsed_time = command_elapsed_time


def generate_function():
    """決定を行う関数を生成する関数."""
    def function(self, robot_id):
        operation_halt = Operation().halt()
        self._operator.operate(robot_id, operation_halt)
    return function


function_names = [
    'halt', 'stop', 'inplay',
    'our_pre_kickoff', 'our_kickoff', 'their_pre_kickoff', 'their_kickoff',
    'our_pre_penalty', 'our_penalty', 'their_pre_penalty', 'their_penalty',
    'our_penalty_inplay', 'their_penalty_inplay',
    'our_direct', 'their_direct', 'our_indirect', 'their_indirect',
    'our_timeout', 'their_timeout'
]
for name in function_names:
    setattr(DecisionBase, name, generate_function())
