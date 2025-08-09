#!/usr/bin/env python3
# coding: UTF-8

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

"""サイドウィングの意思決定を行うモジュール."""

from enum import Enum

from consai_examples.decisions.decision_base import DecisionBase
from consai_examples.operation import Operation
from consai_examples.operation import TargetTheta
from consai_examples.operation import TargetXY


class WingID(Enum):
    """WingIDを定義するEnumクラス."""

    LEFT = 0
    RIGHT = 1


class SideWingDecision(DecisionBase):
    """サイドウィングの意思決定を行うクラス."""

    def __init__(self, robot_operator, field_observer, wing_id: WingID):
        """SideWingDecisionのインスタンスを初期化する関数."""
        super().__init__(robot_operator, field_observer)
        self._wing_id = wing_id
        self._our_penalty_pos_x = -self._penalty_wait_x()
        self._our_penalty_pos_y = 4.5 - 0.3 * (3.0 + self._wing_id.value)
        self._their_penalty_pos_x = self._penalty_wait_x()
        self._their_penalty_pos_y = 4.5 - 0.3 * (3.0 + self._wing_id.value)
        self._ball_placement_pos_x = -6.0 + 2.0
        self._ball_placement_pos_y = 1.8 - 0.3 * (8.0 + self._wing_id.value)

    def _defend_upper_defense_area(self, robot_id):
        """ディフェンスエリアの外側を守る動作を行う関数."""
        p1_x = -6.0 + 0.3
        p1_y = (1.8 + 0.5) * (-1 if self._wing_id.value > 0 else 1)
        p2_x = -6.0 + 1.8 + 0.3
        p2_y = (1.8 + 0.5) * (-1 if self._wing_id.value > 0 else 1)
        operation = Operation().move_to_intersection(
            TargetXY.value(p1_x, p1_y), TargetXY.value(p2_x, p2_y),
            TargetXY.our_goal(), TargetXY.ball(), TargetTheta.look_ball())
        operation = operation.with_ball_receiving()
        self._operator.operate(robot_id, operation)

    def _defend_our_half_way_operation(self):
        """自陣のHalf-wayラインで待機する動作を返す関数."""
        target_x = -1.0
        target_y = 4.0 * (-1 if self._wing_id.value > 0 else 1)
        operation = Operation().move_to_pose(
            TargetXY.value(target_x, target_y), TargetTheta.look_ball())
        operation = operation.with_reflecting_to(TargetXY.their_goal())
        return operation

    def _offend_upper_defense_area_operation(self):
        """相手フィールドで待機する動作を返す関数."""
        p1_x = 2.0
        p1_y = 4.0 * (-1 if self._wing_id.value > 0 else 1)
        p2_x = 5.0
        p2_y = 4.0 * (-1 if self._wing_id.value > 0 else 1)
        operation = Operation().move_to_intersection(
            TargetXY.value(p1_x, p1_y), TargetXY.value(p2_x, p2_y),
            TargetXY.our_side_center(), TargetXY.ball(), TargetTheta.look_ball())
        operation = operation.with_ball_receiving()
        return operation

    def stop(self, robot_id):
        """ロボットの動作を停止する関数."""
        operation = self._offend_upper_defense_area_operation()
        operation = operation.enable_avoid_ball()
        operation = operation.enable_avoid_pushing_robots()
        self._operator.operate(robot_id, operation)

    def inplay(self, robot_id):
        """ゲーム中の動作を実行する関数."""
        operation = self._offend_upper_defense_area_operation()
        # シュート可能なIDリストを取得
        shoot_pos_list = self._field_observer.pass_shoot().get_shoot_pos_list()
        # シュートできる場合はシュートする
        if len(shoot_pos_list) > 0:
            operation = operation.with_reflecting_to(
                TargetXY.value(shoot_pos_list[0].x, shoot_pos_list[0].y))
        self._operator.operate(robot_id, operation)

    def our_pre_kickoff(self, robot_id):
        """キックオフ前の自陣での動作を実行する関数."""
        operation = self._defend_our_half_way_operation()
        operation = operation.enable_avoid_ball()
        self._operator.operate(robot_id, operation)

    def our_kickoff(self, robot_id):
        """キックオフ時の自陣での動作を実行する関数."""
        operation = self._defend_our_half_way_operation()
        self._operator.operate(robot_id, operation)

    def their_pre_kickoff(self, robot_id):
        """相手のキックオフ前の動作を実行する関数."""
        operation = self._defend_our_half_way_operation()
        operation = operation.enable_avoid_ball()
        self._operator.operate(robot_id, operation)

    def their_kickoff(self, robot_id):
        """相手のキックオフ時の動作を実行する関数."""
        operation = self._defend_our_half_way_operation()
        self._operator.operate(robot_id, operation)

    def our_direct(self, robot_id):
        """自陣の直接フリーキック時の動作を実行する関数."""
        operation = self._defend_our_half_way_operation()
        operation = operation.enable_avoid_ball()
        self._operator.operate(robot_id, operation)

    def their_direct(self, robot_id):
        """相手の直接フリーキック時の動作を実行する関数."""
        operation = self._defend_our_half_way_operation()
        operation = operation.enable_avoid_ball()
        self._operator.operate(robot_id, operation)

    def our_indirect(self, robot_id):
        """自陣の間接フリーキック時の動作を実行する関数."""
        operation = self._defend_our_half_way_operation()
        operation = operation.enable_avoid_ball()
        self._operator.operate(robot_id, operation)

    def their_indirect(self, robot_id):
        """相手の間接フリーキック時の動作を実行する関数."""
        operation = self._defend_our_half_way_operation()
        operation = operation.enable_avoid_ball()
        self._operator.operate(robot_id, operation)

    def _our_penalty_operation(self):
        """自陣のペナルティ時の動作を返す関数."""
        return Operation().move_to_pose(
            TargetXY.value(self._our_penalty_pos_x, self._our_penalty_pos_y),
            TargetTheta.look_ball())

    def _their_penalty_operation(self):
        """相手のペナルティ時の動作を返す関数."""
        return Operation().move_to_pose(
            TargetXY.value(self._their_penalty_pos_x, self._their_penalty_pos_y),
            TargetTheta.look_ball())

    def _ball_placement_operation(self):
        """ボール配置時の動作を返す関数."""
        return Operation().move_to_pose(
            TargetXY.value(
                self._ball_placement_pos_x, self._ball_placement_pos_y),
            TargetTheta.look_ball())


def gen_our_penalty_function():
    """自陣ペナルティエリアでの操作を生成する関数."""
    def function(self, robot_id):
        """ロボットに自陣ペナルティエリアでの操作を実行させる関数."""
        operation = self._our_penalty_operation()
        operation = operation.enable_avoid_ball()
        self._operator.operate(robot_id, operation)
    return function


def gen_their_penalty_function():
    """相手陣ペナルティエリアでの操作を生成する関数."""
    def function(self, robot_id):
        """ロボットに相手陣ペナルティエリアでの操作を実行させる関数."""
        operation = self._their_penalty_operation()
        operation = operation.enable_avoid_ball()
        self._operator.operate(robot_id, operation)
    return function


def gen_ball_placement_function():
    """ボール配置操作を生成する関数."""
    def function(self, robot_id, placement_pos=None):
        """ロボットにボール配置操作を実行させる関数."""
        operation = self._ball_placement_operation()
        operation = operation.enable_avoid_placement_area(placement_pos)
        operation = operation.enable_avoid_pushing_robots()
        self._operator.operate(robot_id, operation)
    return function


for name in ['our_pre_penalty', 'our_penalty', 'our_penalty_inplay']:
    setattr(SideWingDecision, name, gen_our_penalty_function())

for name in ['their_pre_penalty', 'their_penalty', 'their_penalty_inplay']:
    setattr(SideWingDecision, name, gen_their_penalty_function())

for name in ['our_ball_placement', 'their_ball_placement']:
    setattr(SideWingDecision, name, gen_ball_placement_function())
