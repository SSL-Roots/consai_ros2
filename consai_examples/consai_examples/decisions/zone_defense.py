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

"""ゾーンディフェンスの意思決定に関するモジュール."""

from enum import Enum

from consai_examples.decisions.decision_base import DecisionBase
from consai_examples.operation import Operation
from consai_examples.operation import TargetTheta
from consai_examples.operation import TargetXY


class ZoneDefenseID(Enum):
    """ゾーンディフェンスにおけるゾーンIDの列挙型."""

    # ゾーンディフェンスは最大4台まで
    ZONE1 = 0
    ZONE2 = 1
    ZONE3 = 2
    ZONE4 = 3


class ZoneDefenseDecision(DecisionBase):
    """ゾーンディフェンスの意思決定クラス."""

    def __init__(self, robot_operator, field_observer, zone_id: ZoneDefenseID):
        """ゾーンディフェンスの意思決定クラスの初期化."""
        super().__init__(robot_operator, field_observer)
        self._zone_id = zone_id

    def _zone_defense_operation(self, robot_id, without_mark=False):
        """ゾーンディフェンスの操作を生成する関数."""
        # ゾーンディフェンスの担当者数に合わせて、待機位置を変更する
        DISTANCE_FROM = self._div_a_x(0.4)

        # ZONE_ID = self._zone_id.value
        target_id = self._field_observer.man_mark().get_mark_robot_id(robot_id)

        # ゾーン内にボールがあるか判定
        # ボールを追いかける処理は行ってない（アタッカーと取り合いになるため）
        # FIXME: アタッカーと取り合いにならない程度にボールは追いかけてほしい
        # ball_is_in_my_zone = self._ball_is_in_zone()

        # ゾーン内の相手ロボットがいる、ボールとロボットの間に移動する
        # if self._field_observer.zone_target().has_zone_target(ZONE_ID) and without_mark is False:
        if target_id != -1 and without_mark is False:
            operation = Operation().move_on_line(
                TargetXY.their_robot(target_id), TargetXY.ball(),
                distance_from_p1=DISTANCE_FROM, target_theta=TargetTheta.look_ball())
            operation = operation.with_ball_receiving()
        else:
            # ゾーン内で待機する
            target_x, target_y = self._get_zone_target_xy()
            operation = Operation().move_to_pose(
                TargetXY.value(target_x, target_y), TargetTheta.look_ball())
            operation = operation.with_reflecting_to(TargetXY.their_goal())
        return operation

    def _get_zone_target_xy(self):
        """ゾーンごとのターゲット位置を取得する関数."""
        # 待機場所をそれぞれの関数で算出する
        target_x = -2.0
        target_y = 0
        if self._zone_id == ZoneDefenseID.ZONE1:
            return self._get_zone1_target_xy()
        elif self._zone_id == ZoneDefenseID.ZONE2:
            return self._get_zone2_target_xy()
        elif self._zone_id == ZoneDefenseID.ZONE3:
            return self._get_zone3_target_xy()
        elif self._zone_id == ZoneDefenseID.ZONE4:
            return self._get_zone4_target_xy()
        return target_x, target_y

    def _get_zone1_target_xy(self):
        """ゾーン1のターゲット位置を計算する関数."""
        target_x = -2.0
        target_y = 0
        if self._num_of_zone_roles == 2:
            target_y = 4.5 * 0.5
        elif self._num_of_zone_roles >= 3:
            target_y = 4.5 * 0.75
        return target_x, target_y

    def _get_zone2_target_xy(self):
        """ゾーン2のターゲット位置を計算する関数."""
        target_x = -2.0
        target_y = -4.5 * 0.5
        if self._num_of_zone_roles == 3:
            target_y = 0.0
        elif self._num_of_zone_roles == 4:
            target_y = 4.5 * 0.25
        return target_x, target_y

    def _get_zone3_target_xy(self):
        """ゾーン3のターゲット位置を計算する関数."""
        target_x = -2.0
        target_y = -4.5 * 0.75
        if self._num_of_zone_roles == 4:
            target_y = -4.5 * 0.25
        return target_x, target_y

    def _get_zone4_target_xy(self):
        """ゾーン4のターゲット位置を計算する関数."""
        target_x = -2.0
        target_y = -4.5 * 0.75
        return target_x, target_y

    def _ball_is_in_zone(self):
        """
        自分のゾーン内にボールがあるか判定する関数.

        自分のzone_idと味方ゾーンDFの数によって自分の担当ゾーンを決定して, ボールがあるか判定する.
        """
        if self._zone_id == ZoneDefenseID.ZONE1:
            return self._ball_is_in_zone1()
        elif self._zone_id == ZoneDefenseID.ZONE2:
            return self._ball_is_in_zone2()
        elif self._zone_id == ZoneDefenseID.ZONE3:
            return self._ball_is_in_zone3()
        elif self._zone_id == ZoneDefenseID.ZONE4:
            return self._ball_is_in_zone4()
        return False

    def _ball_is_in_zone1(self):
        """ゾーン1にボールがあるか判定する関数."""
        if self._field_observer.ball_position().is_in_our_side():
            if self._num_of_zone_roles == 1:
                return True
            elif self._num_of_zone_roles == 2:
                if self._field_observer.zone().ball_is_in_left_top() or \
                        self._field_observer.zone().ball_is_in_left_mid_top():
                    return True
            elif self._num_of_zone_roles == 3 or self._num_of_zone_roles == 4:
                if self._field_observer.zone().ball_is_in_left_top():
                    return True
        return False

    def _ball_is_in_zone2(self):
        """ゾーン2にボールがあるか判定する関数."""
        if self._field_observer.ball_position().is_in_our_side():
            if self._num_of_zone_roles == 2:
                if self._field_observer.zone().ball_is_in_left_bottom() or \
                        self._field_observer.zone().ball_is_in_left_mid_bottom():
                    return True
            elif self._num_of_zone_roles == 3:
                if self._field_observer.zone().ball_is_in_left_mid_top() or \
                        self._field_observer.zone().ball_is_in_left_mid_bottom():
                    return True
            else:
                if self._field_observer.zone().ball_is_in_left_mid_top():
                    return True
        return False

    def _ball_is_in_zone3(self):
        """ゾーン3にボールがあるか判定する関数."""
        if self._field_observer.ball_position().is_in_our_side():
            if self._num_of_zone_roles == 3:
                if self._field_observer.zone().ball_is_in_left_bottom():
                    return True
            else:
                if self._field_observer.zone().ball_is_in_left_mid_bottom():
                    return True
        return False

    def _ball_is_in_zone4(self):
        """ゾーン4にボールがあるか判定する関数."""
        if self._field_observer.ball_position().is_in_our_side():
            if self._field_observer.zone().ball_is_in_left_bottom():
                return True
        return False

    def stop(self, robot_id):
        """ロボットの停止操作を行う関数."""
        operation = self._zone_defense_operation(robot_id, without_mark=True)
        operation = operation.enable_avoid_ball()
        operation = operation.enable_avoid_pushing_robots()
        self._operator.operate(robot_id, operation)

    def inplay(self, robot_id):
        """ロボットがプレイ中に行う操作を実行する関数."""
        operation = self._zone_defense_operation(robot_id)
        self._operator.operate(robot_id, operation)

    def our_pre_kickoff(self, robot_id):
        """ロボットがキックオフ前に行う操作を実行する関数."""
        operation = self._zone_defense_operation(robot_id, without_mark=True)
        operation = operation.enable_avoid_ball()
        self._operator.operate(robot_id, operation)

    def our_kickoff(self, robot_id):
        """ロボットが自分のキックオフ時に行う操作を実行する関数."""
        operation = self._zone_defense_operation(robot_id, without_mark=True)
        self._operator.operate(robot_id, operation)

    def their_pre_kickoff(self, robot_id):
        """ロボットが相手のキックオフ前に行う操作を実行する関数."""
        operation = self._zone_defense_operation(robot_id, without_mark=True)
        operation = operation.enable_avoid_ball()
        self._operator.operate(robot_id, operation)

    def their_kickoff(self, robot_id):
        """ロボットが相手のキックオフ時に行う操作を実行する関数."""
        operation = self._zone_defense_operation(robot_id, without_mark=True)
        self._operator.operate(robot_id, operation)

    def our_direct(self, robot_id):
        """自分の直接フリーキック時の操作を行う関数."""
        operation = self._zone_defense_operation(robot_id)
        operation = operation.enable_avoid_ball()
        self._operator.operate(robot_id, operation)

    def their_direct(self, robot_id):
        """相手の直接フリーキック時の操作を行う関数."""
        operation = self._zone_defense_operation(robot_id)
        operation = operation.enable_avoid_ball()
        self._operator.operate(robot_id, operation)

    def our_indirect(self, robot_id):
        """自分の間接フリーキック時の操作を行う関数."""
        operation = self._zone_defense_operation(robot_id)
        operation = operation.enable_avoid_ball()
        self._operator.operate(robot_id, operation)

    def their_indirect(self, robot_id):
        """相手の間接フリーキック時の操作を行う関数."""
        operation = self._zone_defense_operation(robot_id)
        operation = operation.enable_avoid_ball()
        self._operator.operate(robot_id, operation)

    def _our_penalty_operation(self):
        """自分のペナルティエリア内の操作を行う関数."""
        self._our_penalty_pos_x = -self._penalty_wait_x()
        self._our_penalty_pos_y = self._div_a_y(4.5 - 0.3 * (7.0 + self._zone_id.value))
        return Operation().move_to_pose(
            TargetXY.value(self._our_penalty_pos_x, self._our_penalty_pos_y),
            TargetTheta.look_ball())

    def _their_penalty_operation(self):
        """相手のペナルティエリア内の操作を行う関数."""
        self._their_penalty_pos_x = self._penalty_wait_x()
        self._their_penalty_pos_y = self._div_a_y(4.5 - 0.3 * (7.0 + self._zone_id.value))
        return Operation().move_to_pose(
            TargetXY.value(self._their_penalty_pos_x, self._their_penalty_pos_y),
            TargetTheta.look_ball())

    def _ball_placement_operation(self):
        """ボールの配置位置に移動する操作を行う関数."""
        self._ball_placement_pos_x = self._div_a_x(-6.0 + 2.0)
        self._ball_placement_pos_y = self._div_a_y(1.8 - 0.3 * (4.0 + self._zone_id.value))
        return Operation().move_to_pose(
            TargetXY.value(
                self._ball_placement_pos_x, self._ball_placement_pos_y),
            TargetTheta.look_ball())


def gen_our_penalty_function():
    """自分のペナルティ時に実行される関数を生成する関数."""
    def function(self, robot_id):
        """自分のペナルティ操作を行う関数."""
        operation = self._our_penalty_operation()
        operation = operation.enable_avoid_ball()
        self._operator.operate(robot_id, operation)
    return function


def gen_their_penalty_function():
    """相手のペナルティ時に実行される関数を生成する関数."""
    def function(self, robot_id):
        """相手のペナルティ操作を行う関数."""
        operation = self._their_penalty_operation()
        operation = operation.enable_avoid_ball()
        self._operator.operate(robot_id, operation)
    return function


def gen_ball_placement_function():
    """ボール配置時に実行される関数を生成する関数."""
    def function(self, robot_id, placement_pos=None):
        """ボール配置操作を行う関数."""
        operation = self._zone_defense_operation(robot_id, without_mark=True)
        operation = operation.enable_avoid_ball()
        operation = operation.enable_avoid_placement_area(placement_pos)
        operation = operation.enable_avoid_pushing_robots()
        self._operator.operate(robot_id, operation)
    return function


for name in ['our_pre_penalty', 'our_penalty', 'our_penalty_inplay']:
    setattr(ZoneDefenseDecision, name, gen_our_penalty_function())

for name in ['their_pre_penalty', 'their_penalty', 'their_penalty_inplay']:
    setattr(ZoneDefenseDecision, name, gen_their_penalty_function())

for name in ['our_ball_placement', 'their_ball_placement']:
    setattr(ZoneDefenseDecision, name, gen_ball_placement_function())
