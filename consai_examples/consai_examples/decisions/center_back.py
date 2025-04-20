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

"""センターバックの意思決定に関するモジュール."""

from enum import Enum

from consai_examples.decisions.decision_base import DecisionBase
from consai_examples.operation import Operation
from consai_examples.operation import TargetTheta
from consai_examples.operation import TargetXY

from consai_tools.geometry import geometry_tools as tool


class CenterBackID(Enum):
    """センターバックのIDを定義するクラス."""

    # センターバックは最大2台まで
    CENTER_BACK1 = 0
    CENTER_BACK2 = 1


class CenterBackDecision(DecisionBase):
    """センターバックの意思決定を行うクラス."""

    def __init__(self, robot_operator, field_observer, center_back_id: CenterBackID):
        """CenterBackDecisionクラスの初期化処理,センターバックIDとフィールドオブザーバーを設定する関数."""
        super().__init__(robot_operator, field_observer)
        self._center_back_id = center_back_id

    def _margin_line(self) -> float:
        """ペナルティエリアからどれだけ離れるかを計算する関数."""
        return self._div_a_dia(0.3)

    def _get_offset(self) -> float:
        """センターバックIDに応じたオフセットを計算する関数."""
        MARGIN_ROBOT = self._div_a_dia(0.1)
        offset = 0.0
        if self._num_of_center_back_roles > 1:
            if self._center_back_id == CenterBackID.CENTER_BACK1:
                offset = -MARGIN_ROBOT
            else:
                offset = MARGIN_ROBOT
        return offset

    def _defend_front_operation(self, offset: float):
        """ディフェンスエリアの前側を守るための操作を行う関数."""
        p1_x = self._field_pos().penalty_pose('our', 'upper_front').x + self._margin_line()
        p1_y = self._field_pos().penalty_pose('our', 'upper_front').y
        p2_x = self._field_pos().penalty_pose('our', 'lower_front').x + self._margin_line()
        p2_y = self._field_pos().penalty_pose('our', 'lower_front').y
        return Operation().move_to_intersection(
            TargetXY.value(p1_x, p1_y), TargetXY.value(p2_x, p2_y),
            TargetXY.our_goal(), TargetXY.ball(), TargetTheta.look_ball(), offset)

    def _defend_upper_operation(self, offset: float):
        """ディフェンスエリアの上側を守るための操作を行う関数."""
        p1_x = self._field_pos().penalty_pose('our', 'upper_back').x + self._margin_line()
        p1_y = self._field_pos().penalty_pose('our', 'upper_back').y + self._margin_line()
        p2_x = self._field_pos().penalty_pose('our', 'upper_front').x + self._margin_line()
        p2_y = self._field_pos().penalty_pose('our', 'upper_front').y + self._margin_line()
        return Operation().move_to_intersection(
            TargetXY.value(p1_x, p1_y), TargetXY.value(p2_x, p2_y),
            TargetXY.our_goal(), TargetXY.ball(), TargetTheta.look_ball(), offset)

    def _defend_lower_operation(self, offset: float):
        """ディフェンスエリアの下側を守るための操作を行う関数."""
        p1_x = self._field_pos().penalty_pose('our', 'lower_front').x + self._margin_line()
        p1_y = self._field_pos().penalty_pose('our', 'lower_front').y - self._margin_line()
        p2_x = self._field_pos().penalty_pose('our', 'lower_back').x + self._margin_line()
        p2_y = self._field_pos().penalty_pose('our', 'lower_back').y - self._margin_line()
        return Operation().move_to_intersection(
            TargetXY.value(p1_x, p1_y), TargetXY.value(p2_x, p2_y),
            TargetXY.our_goal(), TargetXY.ball(), TargetTheta.look_ball(), offset)

    def _defend_defense_area_operation(self, robot_id) -> Operation:
        """ボール位置に基づいてディフェンスエリアの操作を変更する関数."""
        offset = self._get_offset()
        # ボール位置で行動を変更
        if self._field_observer.zone().ball_is_in_left_top():
            operation = self._defend_upper_operation(offset)
        elif self._field_observer.zone().ball_is_in_left_bottom():
            operation = self._defend_lower_operation(offset)
        else:
            operation = self._defend_front_operation(offset)
        # レシービングしない
        # operation = operation.with_ball_receiving()
        operation = operation.with_reflecting_to(TargetXY.their_goal())
        return operation

    def _modify_our_robots_avoidance(self, robot_id: int, operation: Operation) -> Operation:
        """隣同士の衝突回避をやめるため, ディフェンスエリアに近づいたとき, our robot回避を無効にする関数."""
        my_pos = self._field_observer.detection().our_robots()[robot_id].pos()
        our_goal_center = self._field_pos().goal_pose('our', 'center')
        if tool.get_distance(my_pos, our_goal_center) < self._div_a_x(3.0):
            operation = operation.disable_avoid_our_robots()
        return operation

    def stop(self, robot_id):
        """ロボットを停止させるための操作を行う関数."""
        offset = self._get_offset()
        operation = self._defend_front_operation(offset)
        operation = operation.enable_avoid_ball()
        operation = operation.enable_avoid_pushing_robots()
        operation = self._modify_our_robots_avoidance(robot_id, operation)
        self._operator.operate(robot_id, operation)

    def inplay(self, robot_id):
        """ディフェンスエリア内でボールに基づいてプレイを再開するための操作を行う関数."""
        operation = self._defend_defense_area_operation(robot_id)
        operation = self._modify_our_robots_avoidance(robot_id, operation)
        self._operator.operate(robot_id, operation)

    def _our_penalty_operation(self):
        """自チームのペナルティエリア内での操作を行う関数."""
        our_penalty_pos_y = self._div_a_y(4.5 - 0.3 * (1.0 + self._center_back_id.value))
        return Operation().move_to_pose(
            TargetXY.value(-self._penalty_wait_x(), our_penalty_pos_y),
            TargetTheta.look_ball())

    def _their_penalty_operation(self):
        """相手チームのペナルティエリア内での操作を行う関数."""
        their_penalty_pos_y = self._div_a_y(4.5 - 0.3 * (1.0 + self._center_back_id.value))
        return Operation().move_to_pose(
            TargetXY.value(self._penalty_wait_x(), their_penalty_pos_y),
            TargetTheta.look_ball())

    def _ball_placement_operation(self):
        """ボールの配置操作を行う関数."""
        ball_placement_pos_x = self._div_a_x(-6.0 + 2.0)
        ball_placement_pos_y = self._div_a_y(1.8 - 0.3 * (1.0 + self._center_back_id.value))
        return Operation().move_to_pose(
            TargetXY.value(ball_placement_pos_x, ball_placement_pos_y),
            TargetTheta.look_ball())


def gen_kickoff_function():
    """キックオフ処理を生成する関数."""
    def function(self, robot_id):
        """キックオフ処理を実行する内部関数."""
        offset = self._get_offset()
        operation = self._defend_front_operation(offset)
        operation = operation.enable_avoid_ball()
        self._operator.operate(robot_id, operation)
    return function


def gen_setplay_function():
    """セットプレイ処理を生成する関数."""
    def function(self, robot_id):
        """セットプレイ処理を実行する内部関数."""
        offset = self._get_offset()
        operation = self._defend_front_operation(offset)
        operation = operation.with_ball_receiving()
        operation = operation.enable_avoid_ball()
        self._operator.operate(robot_id, operation)
    return function


def gen_our_penalty_function():
    """自チームのペナルティ処理を生成する関数."""
    def function(self, robot_id):
        """自チームのペナルティ処理を実行する内部関数."""
        operation = self._our_penalty_operation()
        operation = operation.enable_avoid_ball()
        self._operator.operate(robot_id, operation)
    return function


def gen_their_penalty_function():
    """相手チームのペナルティ処理を生成する関数."""
    def function(self, robot_id):
        """相手チームのペナルティ処理を実行する内部関数."""
        operation = self._their_penalty_operation()
        operation = operation.enable_avoid_ball()
        self._operator.operate(robot_id, operation)
    return function


def gen_ball_placement_function():
    """ボール配置処理を生成する関数."""
    def function(self, robot_id, placement_pos):
        """ボール配置処理を実行する内部関数."""
        offset = self._get_offset()
        operation = self._defend_front_operation(offset)
        operation = operation.enable_avoid_ball()
        operation = operation.enable_avoid_placement_area(placement_pos)
        operation = operation.enable_avoid_pushing_robots()
        self._operator.operate(robot_id, operation)
    return function


for name in ['our_pre_kickoff', 'our_kickoff', 'their_pre_kickoff']:
    setattr(CenterBackDecision, name, gen_kickoff_function())

for name in ['their_kickoff', 'our_direct', 'their_direct', 'our_indirect', 'their_indirect']:
    setattr(CenterBackDecision, name, gen_setplay_function())

for name in ['our_pre_penalty', 'our_penalty', 'our_penalty_inplay']:
    setattr(CenterBackDecision, name, gen_our_penalty_function())

for name in ['their_pre_penalty', 'their_penalty', 'their_penalty_inplay']:
    setattr(CenterBackDecision, name, gen_their_penalty_function())

for name in ['our_ball_placement', 'their_ball_placement']:
    setattr(CenterBackDecision, name, gen_ball_placement_function())
