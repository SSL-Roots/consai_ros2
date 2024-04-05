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


from enum import Enum

from decisions.decision_base import DecisionBase
from field_observer import FieldObserver
from operation import Operation
from operation import TargetXY
from operation import TargetTheta


class ZoneDefenseID(Enum):
    # ゾーンディフェンスは最大4台まで
    ZONE1 = 0
    ZONE2 = 1
    ZONE3 = 2
    ZONE4 = 3


class ZoneDefenseDecision(DecisionBase):

    def __init__(self, robot_operator, field_observer, zone_id: ZoneDefenseID):
        super().__init__(robot_operator, field_observer)
        self._zone_id = zone_id
        self._our_penalty_pos_x = -self._PENALTY_WAIT_X
        self._our_penalty_pos_y = 4.5 - 0.3 * (6.0 + self._zone_id.value)
        self._their_penalty_pos_x = self._PENALTY_WAIT_X
        self._their_penalty_pos_y = 4.5 - 0.3 * (6.0 + self._zone_id.value)
        self._ball_placement_pos_x = -6.0 + 2.0
        self._ball_placement_pos_y = 1.8 - 0.3 * (4.0 + self._zone_id.value)

    def _zone_defense_operation(self, without_mark=False):
        # ゾーンディフェンスの担当者数に合わせて、待機位置を変更する
        ZONE_ID = self._zone_id.value

        # ゾーン内にボールがあるか判定
        # ボールを追いかける処理は行ってない（アタッカーと取り合いになるため）
        # FIXME: アタッカーと取り合いにならない程度にボールは追いかけてほしい
        # ball_is_in_my_zone = self._ball_is_in_zone()

        # ゾーン内の相手ロボットがいる、ボールとロボットの間に移動する
        if self._field_observer.zone_target().has_zone_target(ZONE_ID) and without_mark is False:
            target_id = self._field_observer.zone_target().get_zone_target_id(ZONE_ID)
            operation = Operation().move_on_line(
                TargetXY.their_robot(target_id), TargetXY.ball(),
                distance_from_p1=0.5, target_theta=TargetTheta.look_ball())
            operation = operation.with_ball_receiving()
        else:
            # ゾーン内で待機する
            target_x, target_y = self._get_zone_target_xy()
            operation = Operation().move_to_pose(
                TargetXY.value(target_x, target_y), TargetTheta.look_ball())
            operation = operation.with_reflecting_to(TargetXY.their_goal())
        return operation

    def _get_zone_target_xy(self):
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
        target_x = -2.0
        target_y = 0
        if self._num_of_zone_roles == 2:
            target_y = 4.5 * 0.5
        elif self._num_of_zone_roles >= 3:
            target_y = 4.5 * 0.75
        return target_x, target_y

    def _get_zone2_target_xy(self):
        target_x = -2.0
        target_y = -4.5 * 0.5
        if self._num_of_zone_roles == 3:
            target_y = 0.0
        elif self._num_of_zone_roles == 4:
            target_y = 4.5 * 0.25
        return target_x, target_y

    def _get_zone3_target_xy(self):
        target_x = -2.0
        target_y = -4.5 * 0.75
        if self._num_of_zone_roles == 4:
            target_y = -4.5 * 0.25
        return target_x, target_y

    def _get_zone4_target_xy(self):
        target_x = -2.0
        target_y = -4.5 * 0.75
        return target_x, target_y

    def _ball_is_in_zone(self):
        # 自分のzone_idと味方ゾーンDFの数によって
        # 自分の担当ゾーンを決定して、ボールがあるか判定
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
        if self._field_observer.ball_position().is_in_our_side():
            if self._num_of_zone_roles == 3:
                if self._field_observer.zone().ball_is_in_left_bottom():
                    return True
            else:
                if self._field_observer.zone().ball_is_in_left_mid_bottom():
                    return True
        return False

    def _ball_is_in_zone4(self):
        if self._field_observer.ball_position().is_in_our_side():
            if self._field_observer.zone().ball_is_in_left_bottom():
                return True
        return False

    def stop(self, robot_id):
        operation = self._zone_defense_operation(without_mark=True)
        operation = operation.enable_avoid_ball()
        operation = operation.enable_avoid_pushing_robots()
        self._operator.operate(robot_id, operation)

    def inplay(self, robot_id):
        operation = self._zone_defense_operation()
        self._operator.operate(robot_id, operation)

    def our_pre_kickoff(self, robot_id):
        operation = self._zone_defense_operation(without_mark=True)
        operation = operation.enable_avoid_ball()
        self._operator.operate(robot_id, operation)

    def our_kickoff(self, robot_id):
        operation = self._zone_defense_operation(without_mark=True)
        self._operator.operate(robot_id, operation)

    def their_pre_kickoff(self, robot_id):
        operation = self._zone_defense_operation(without_mark=True)
        operation = operation.enable_avoid_ball()
        self._operator.operate(robot_id, operation)

    def their_kickoff(self, robot_id):
        operation = self._zone_defense_operation(without_mark=True)
        self._operator.operate(robot_id, operation)

    def our_direct(self, robot_id):
        operation = self._zone_defense_operation()
        operation = operation.enable_avoid_ball()
        self._operator.operate(robot_id, operation)

    def their_direct(self, robot_id):
        operation = self._zone_defense_operation()
        operation = operation.enable_avoid_ball()
        self._operator.operate(robot_id, operation)

    def our_indirect(self, robot_id):
        operation = self._zone_defense_operation()
        operation = operation.enable_avoid_ball()
        self._operator.operate(robot_id, operation)

    def their_indirect(self, robot_id):
        operation = self._zone_defense_operation()
        operation = operation.enable_avoid_ball()
        self._operator.operate(robot_id, operation)

    def _our_penalty_operation(self):
        return Operation().move_to_pose(
            TargetXY.value(self._our_penalty_pos_x, self._our_penalty_pos_y),
            TargetTheta.look_ball())

    def _their_penalty_operation(self):
        return Operation().move_to_pose(
            TargetXY.value(self._their_penalty_pos_x, self._their_penalty_pos_y),
            TargetTheta.look_ball())

    def _ball_placement_operation(self):
        return Operation().move_to_pose(
            TargetXY.value(
                self._ball_placement_pos_x, self._ball_placement_pos_y),
            TargetTheta.look_ball())


def gen_our_penalty_function():
    def function(self, robot_id):
        operation = self._our_penalty_operation()
        operation = operation.enable_avoid_ball()
        self._operator.operate(robot_id, operation)
    return function


def gen_their_penalty_function():
    def function(self, robot_id):
        operation = self._their_penalty_operation()
        operation = operation.enable_avoid_ball()
        self._operator.operate(robot_id, operation)
    return function


def gen_ball_placement_function():
    def function(self, robot_id, placement_pos=None):
        operation = self._ball_placement_operation()
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
