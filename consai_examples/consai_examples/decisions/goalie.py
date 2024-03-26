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

from decisions.decision_base import DecisionBase

from field_observer import FieldObserver
from operation import Operation
from operation import TargetXY
from operation import TargetTheta


class GoaleDecision(DecisionBase):

    def __init__(self, robot_operator, field_observer):
        super().__init__(robot_operator, field_observer)

    def _defend_goal_operation(self):
        p1_x = -6.0 + 0.3
        p1_y = 0.9
        p2_x = -6.0 + 0.3
        p2_y = -0.9
        defend_goal = Operation().move_to_intersection(
            TargetXY.value(p1_x, p1_y), TargetXY.value(p2_x, p2_y),
            TargetXY.our_goal(), TargetXY.ball(), TargetTheta.look_ball())
        defend_goal = defend_goal.with_ball_receiving()
        return defend_goal

    def _penalty_defend_operation(self):
        p1_x = -6.0 + 0.05
        p1_y = 0.9
        p2_x = -6.0 + 0.05
        p2_y = -0.9
        defend_goal = Operation().move_to_intersection(
            TargetXY.value(p1_x, p1_y), TargetXY.value(p2_x, p2_y),
            TargetXY.our_goal(), TargetXY.ball(), TargetTheta.look_ball())
        defend_goal = defend_goal.with_ball_receiving()
        return defend_goal

    def stop(self, robot_id):
        defend_our_goal = self._defend_goal_operation()
        self._operator.operate(robot_id, defend_our_goal)

    def inplay(self, robot_id):
        # ボールがディフェンスエリアにあるときは、ボールを蹴る
        if self._ball_state == FieldObserver.BALL_IS_IN_OUR_DEFENSE_AREA \
           and not self._field_observer.ball_is_moving():
            # レシーバ候補のロボットIDリストを取得
            can_pass_id_list, can_pass_pos_list, can_shoot_id_list, \
                can_shoot_pos_list = self._field_observer.get_open_path_id_list(robot_id)

            move_to_behind_ball = Operation().move_on_line(
                TargetXY.ball(), TargetXY.our_goal(), 0.3, TargetTheta.look_ball())
            move_to_behind_ball = move_to_behind_ball.with_ball_receiving()

            # リストが空でない場合
            if 0 < len(can_pass_id_list):
                # リストの先頭のロボットにパス
                passing = move_to_behind_ball.with_passing_to(
                    TargetXY.our_robot(can_pass_id_list[0]))
                self._operator.operate(robot_id, passing)
            # リストが空の場合
            else:
                # ボールがフィールド上側にあるときは、上側コーナを狙って蹴る
                if self._ball_zone_state in [FieldObserver.BALL_ZONE_LEFT_TOP,
                                             FieldObserver.BALL_ZONE_LEFT_MID_TOP]:
                    clear_ball = move_to_behind_ball.with_shooting_to(
                        TargetXY.their_top_corner())
                else:
                    clear_ball = move_to_behind_ball.with_shooting_to(
                        TargetXY.their_bottom_corner())
                self._operator.operate(robot_id, clear_ball)
            return

        # ボールとゴールを結ぶ直線上を守る
        defend_our_goal = self._defend_goal_operation()
        self._operator.operate(robot_id, defend_our_goal)

    def our_pre_kickoff(self, robot_id):
        if self._act_id != self.ACT_ID_PRE_KICKOFF:
            defend_our_goal = self._defend_goal_operation()
            self._operator.operate(robot_id, defend_our_goal)
            self._act_id = self.ACT_ID_PRE_KICKOFF

    def our_kickoff(self, robot_id):
        if self._act_id != self.ACT_ID_KICKOFF:
            defend_our_goal = self._defend_goal_operation()
            self._operator.operate(robot_id, defend_our_goal)
            self._act_id = self.ACT_ID_KICKOFF

    def their_pre_kickoff(self, robot_id):
        if self._act_id != self.ACT_ID_PRE_KICKOFF:
            defend_our_goal = self._defend_goal_operation()
            self._operator.operate(robot_id, defend_our_goal)
            self._act_id = self.ACT_ID_PRE_KICKOFF

    def their_kickoff(self, robot_id):
        if self._act_id != self.ACT_ID_KICKOFF:
            defend_our_goal = self._defend_goal_operation()
            self._operator.operate(robot_id, defend_our_goal)
            self._act_id = self.ACT_ID_KICKOFF

    def our_pre_penalty(self, robot_id):
        if self._act_id != self.ACT_ID_PRE_PENALTY:
            defend_our_goal = self._defend_goal_operation()
            self._operator.operate(robot_id, defend_our_goal)
            self._act_id = self.ACT_ID_PRE_PENALTY

    def our_penalty(self, robot_id):
        if self._act_id != self.ACT_ID_PENALTY:
            defend_our_goal = self._defend_goal_operation()
            self._operator.operate(robot_id, defend_our_goal)
            self._act_id = self.ACT_ID_PENALTY

    def their_pre_penalty(self, robot_id):
        if self._act_id != self.ACT_ID_PRE_PENALTY:
            penalty_defend = self._penalty_defend_operation()
            self._operator.operate(robot_id, penalty_defend)
            self._act_id = self.ACT_ID_PRE_PENALTY

    def their_penalty(self, robot_id):
        if self._act_id != self.ACT_ID_PENALTY:
            penalty_defend = self._penalty_defend_operation()
            self._operator.operate(robot_id, penalty_defend)
            self._act_id = self.ACT_ID_PENALTY

    def our_penalty_inplay(self, robot_id):
        if self._act_id != self.ACT_ID_INPLAY:
            defend_our_goal = self._defend_goal_operation()
            self._operator.operate(robot_id, defend_our_goal)
            self._act_id = self.ACT_ID_INPLAY

    def their_penalty_inplay(self, robot_id):
        if self._act_id != self.ACT_ID_INPLAY:
            defend_our_goal = self._defend_goal_operation()
            self._operator.operate(robot_id, defend_our_goal)
            self._act_id = self.ACT_ID_INPLAY

    def our_direct(self, robot_id):
        if self._act_id != self.ACT_ID_DIRECT:
            defend_our_goal = self._defend_goal_operation()
            self._operator.operate(robot_id, defend_our_goal)
            self._act_id = self.ACT_ID_DIRECT

    def their_direct(self, robot_id):
        if self._act_id != self.ACT_ID_DIRECT:
            defend_our_goal = self._defend_goal_operation()
            self._operator.operate(robot_id, defend_our_goal)
            self._act_id = self.ACT_ID_DIRECT

    def our_indirect(self, robot_id):
        if self._act_id != self.ACT_ID_INDIRECT:
            defend_our_goal = self._defend_goal_operation()
            self._operator.operate(robot_id, defend_our_goal)
            self._act_id = self.ACT_ID_INDIRECT

    def their_indirect(self, robot_id):
        if self._act_id != self.ACT_ID_INDIRECT:
            defend_our_goal = self._defend_goal_operation()
            self._operator.operate(robot_id, defend_our_goal)
            self._act_id = self.ACT_ID_INDIRECT

    def our_ball_placement(self, robot_id, placement_pos):
        if self._act_id != self.ACT_ID_OUR_PLACEMENT:
            defend_our_goal = self._defend_goal_operation()
            self._operator.operate(robot_id, defend_our_goal)
            self._act_id = self.ACT_ID_OUR_PLACEMENT

    def their_ball_placement(self, robot_id, placement_pos):
        if self._act_id != self.ACT_ID_THEIR_PLACEMENT:
            defend_our_goal = self._defend_goal_operation()
            self._operator.operate(robot_id, defend_our_goal)
            self._act_id = self.ACT_ID_THEIR_PLACEMENT
