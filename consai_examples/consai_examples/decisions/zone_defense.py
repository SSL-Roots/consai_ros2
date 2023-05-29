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

    def _zone_defense(self, robot_id, base_id, without_mark=False):
        # ゾーンディフェンスの担当者数に合わせて、待機位置を変更する
        # ID_DEFEND_BALL = base_id + self._num_of_zone_roles
        ID_IN_ZONE = base_id + self._num_of_zone_roles + 100
        ID_MAN_MARK = base_id + self._num_of_zone_roles + 200
        ID_IN_OUR_DEFENSE = base_id + self._num_of_zone_roles + 300

        ZONE_TARGET = self._zone_id.value

        # ボールが自分ディフェンスエリアにあるときは、ゾーン初期位置で待機
        if self._ball_state == FieldObserver.BALL_IS_IN_OUR_DEFENSE_AREA:
            if self._act_id != ID_IN_OUR_DEFENSE:
                target_x, target_y = self._get_zone_trget_xy()
                self._operator.move_to_reflect_shoot_to_their_goal(
                    robot_id, target_x, target_y)
                self._act_id = ID_IN_OUR_DEFENSE
            return

        # ゾーン内にボールがあるか判定
        # ボールを追いかける処理は行ってない（アタッカーと取り合いになるため）
        # FIXME: アタッカーと取り合いにならない程度にボールは追いかけてほしい
        # ball_is_in_my_zone = self._ball_is_in_zone()

        # ゾーン内の相手ロボットがいる、かつボールが自分サイドになければ、ボールとロボットの間に移動する
        # if self._zone_targets[ZONE_TARGET] is not None \
        #    and without_mark is False \
        #    and ball_is_in_my_zone is False:
        # ゾーン内の相手ロボットがいる、ボールとロボットの間に移動する
        if self._zone_targets[ZONE_TARGET] is not None and without_mark is False:
            if self._act_id != ID_MAN_MARK:
                self._operator.man_mark(
                    robot_id, self._zone_targets[ZONE_TARGET], 0.5)
                self._act_id = ID_MAN_MARK
            return

        # ゾーン内で待機する
        if self._act_id != ID_IN_ZONE:
            target_x, target_y = self._get_zone_trget_xy()
            # self._operator.move_to_receive(robot_id, target_x, target_y)
            self._operator.move_to_reflect_shoot_to_their_goal(
                robot_id, target_x, target_y)
            self._act_id = ID_IN_ZONE
        return

    def _get_zone_trget_xy(self):
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
        if self._ball_state == FieldObserver.BALL_IS_IN_OUR_SIDE:
            if self._num_of_zone_roles == 1:
                return True
            elif self._num_of_zone_roles == 2:
                if self._ball_zone_state in [FieldObserver.BALL_ZONE_LEFT_TOP,
                                             FieldObserver.BALL_ZONE_LEFT_MID_TOP]:
                    return True
            elif self._num_of_zone_roles == 3 or self._num_of_zone_roles == 4:
                if self._ball_zone_state == FieldObserver.BALL_ZONE_LEFT_TOP:
                    return True
        return False

    def _ball_is_in_zone2(self):
        if self._ball_state == FieldObserver.BALL_IS_IN_OUR_SIDE:
            if self._num_of_zone_roles == 2:
                if self._ball_zone_state in [FieldObserver.BALL_ZONE_LEFT_MID_BOTTOM,
                                             FieldObserver.BALL_ZONE_LEFT_BOTTOM]:
                    return True
            elif self._num_of_zone_roles == 3:
                if self._ball_zone_state in [FieldObserver.BALL_ZONE_LEFT_MID_TOP,
                                             FieldObserver.BALL_ZONE_LEFT_MID_BOTTOM]:
                    return True
            else:
                if self._ball_zone_state in [FieldObserver.BALL_ZONE_LEFT_MID_TOP]:
                    return True
        return False

    def _ball_is_in_zone3(self):
        if self._ball_state == FieldObserver.BALL_IS_IN_OUR_SIDE:
            if self._num_of_zone_roles == 3:
                if self._ball_zone_state in [FieldObserver.BALL_ZONE_LEFT_BOTTOM]:
                    return True
            else:
                if self._ball_zone_state in [FieldObserver.BALL_ZONE_LEFT_MID_BOTTOM]:
                    return True
        return False

    def _ball_is_in_zone4(self):
        if self._ball_state == FieldObserver.BALL_IS_IN_OUR_SIDE:
            if self._ball_zone_state in [FieldObserver.BALL_ZONE_LEFT_BOTTOM]:
                return True
        return False

    def stop(self, robot_id):
        self._zone_defense(robot_id, self.ACT_ID_STOP, without_mark=True)

    def inplay(self, robot_id):
        self._zone_defense(robot_id, self.ACT_ID_INPLAY)

    def our_pre_kickoff(self, robot_id):
        self._zone_defense(robot_id, self.ACT_ID_PRE_KICKOFF, without_mark=True)

    def our_kickoff(self, robot_id):
        self._zone_defense(robot_id, self.ACT_ID_KICKOFF, without_mark=True)

    def their_pre_kickoff(self, robot_id):
        self._zone_defense(robot_id, self.ACT_ID_PRE_KICKOFF, without_mark=True)

    def their_kickoff(self, robot_id):
        self._zone_defense(robot_id, self.ACT_ID_KICKOFF, without_mark=True)

    def our_pre_penalty(self, robot_id):
        if self._act_id != self.ACT_ID_PRE_PENALTY:
            self._operator.move_to_look_ball(
                robot_id, self._our_penalty_pos_x, self._our_penalty_pos_y)
            self._act_id = self.ACT_ID_PRE_PENALTY

    def our_penalty(self, robot_id):
        if self._act_id != self.ACT_ID_PRE_PENALTY:
            self._operator.move_to_look_ball(
                robot_id, self._our_penalty_pos_x, self._our_penalty_pos_y)
            self._act_id = self.ACT_ID_PRE_PENALTY

    def their_pre_penalty(self, robot_id):
        if self._act_id != self.ACT_ID_PRE_PENALTY:
            self._operator.move_to_look_ball(
                robot_id, self._their_penalty_pos_x, self._their_penalty_pos_y)
            self._act_id = self.ACT_ID_PRE_PENALTY

    def their_penalty(self, robot_id):
        if self._act_id != self.ACT_ID_PRE_PENALTY:
            self._operator.move_to_look_ball(
                robot_id, self._their_penalty_pos_x, self._their_penalty_pos_y)
            self._act_id = self.ACT_ID_PRE_PENALTY

    def our_penalty_inplay(self, robot_id):
        if self._act_id != self.ACT_ID_INPLAY:
            self._operator.stop(robot_id)
            self._act_id = self.ACT_ID_INPLAY

    def their_penalty_inplay(self, robot_id):
        if self._act_id != self.ACT_ID_INPLAY:
            self._operator.stop(robot_id)
            self._act_id = self.ACT_ID_INPLAY

    def our_direct(self, robot_id):
        self._zone_defense(robot_id, self.ACT_ID_DIRECT, without_mark=False)

    def their_direct(self, robot_id):
        self._zone_defense(robot_id, self.ACT_ID_DIRECT, without_mark=False)

    def our_indirect(self, robot_id):
        self._zone_defense(robot_id, self.ACT_ID_INDIRECT)

    def their_indirect(self, robot_id):
        self._zone_defense(robot_id, self.ACT_ID_INDIRECT)

    def our_ball_placement(self, robot_id, placement_pos):
        if self._act_id != self.ACT_ID_OUR_PLACEMENT:
            self._operator.move_to_look_ball(
                robot_id, self._ball_placement_pos_x, self._ball_placement_pos_y)
            self._act_id = self.ACT_ID_OUR_PLACEMENT

    def their_ball_placement(self, robot_id, placement_pos):
        if self._act_id != self.ACT_ID_THEIR_PLACEMENT:
            self._operator.move_to_look_ball(
                robot_id, self._ball_placement_pos_x, self._ball_placement_pos_y)
            self._act_id = self.ACT_ID_THEIR_PLACEMENT
