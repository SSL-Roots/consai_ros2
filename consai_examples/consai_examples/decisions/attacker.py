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

class AttackerDecision(DecisionBase):

    def __init__(self, robot_operator, field_observer):
        super().__init__(robot_operator, field_observer)

    def stop(self, robot_id):
        ID_CHASE = self.ACT_ID_STOP + 0
        ID_IN_OUR_DEFENSE = self.ACT_ID_STOP + 1
        ID_IN_THEIR_DEFENSE = self.ACT_ID_STOP + 2

        # ボールが自分ディフェンスエリアにあるときは、ボールと同じ軸上に移動する
        if self._ball_state == FieldObserver.BALL_IS_IN_OUR_DEFENSE_AREA or self._ball_state == FieldObserver.BALL_IS_OUTSIDE_BACK_X:
            if self._act_id != ID_IN_OUR_DEFENSE:
                self._operator.move_to_ball_y(robot_id, -3.0)
                self._act_id = ID_IN_OUR_DEFENSE
            return

        # ボールが相手ディフェンスエリアにあるときは、ボールと同じ軸上に移動する
        if self._ball_state == FieldObserver.BALL_IS_IN_THEIR_DEFENSE_AREA or self._ball_state == FieldObserver.BALL_IS_OUTSIDE_FRONT_X:
            if self._act_id != ID_IN_THEIR_DEFENSE:
                self._operator.move_to_ball_y(robot_id, 3.0)
                self._act_id = ID_IN_THEIR_DEFENSE
            return

        # ボールを追いかける
        if self._act_id != ID_CHASE:
            self._operator.chase_ball(robot_id, -0.6, 0.0, 0.0, look_from=False, keep=True)
            self._act_id = ID_CHASE

    def inplay(self, robot_id):
        ID_INPLAY = self.ACT_ID_INPLAY + 0
        ID_IN_OUR_DEFENSE = self.ACT_ID_INPLAY + 1
        ID_IN_THEIR_DEFENSE = self.ACT_ID_INPLAY + 2
        ID_SHOOT_PASS = self.ACT_ID_INPLAY + 3
        ID_NO_SHOOT_PASS = self.ACT_ID_INPLAY + 10

        # ボールが自分ディフェンスエリアにあるときは、ボールと同じ軸上に移動する
        if self._ball_state == FieldObserver.BALL_IS_IN_OUR_DEFENSE_AREA or self._ball_state == FieldObserver.BALL_IS_OUTSIDE_BACK_X:
            if self._act_id != ID_IN_OUR_DEFENSE:
                self._operator.move_to_ball_y(robot_id, -3.0)
                self._act_id = ID_IN_OUR_DEFENSE
            return

        # ボールが相手ディフェンスエリアにあるときは、ボールと同じ軸上に移動する
        if self._ball_state == FieldObserver.BALL_IS_IN_THEIR_DEFENSE_AREA or self._ball_state == FieldObserver.BALL_IS_OUTSIDE_FRONT_X:
            if self._act_id != ID_IN_THEIR_DEFENSE:
                self._operator.move_to_ball_y(robot_id, 3.0)
                self._act_id = ID_IN_THEIR_DEFENSE
            return

        # 指定座標に向けてシュートまたはパスをする
        if self._ball_state == FieldObserver.BALL_IS_IN_THEIR_SIDE and self._act_id != ID_NO_SHOOT_PASS:
            if self._act_id != ID_SHOOT_PASS:
                shoot_point = FieldObserver.get_shoot_point(robot_id)
                receiver_robots_id = FieldObserver.get_receiver_robots_id(robot_id)
                # 指定座標に向けてシュートする
                if len(shoot_point) > 0:
                    self._operator.shoot_to(robot_id, FieldObserver.SHOOTS_POS[shoot_point][0], FieldObserver.SHOOTS_POS[shoot_point][1])
                    self._act_id = ID_SHOOT_PASS
                # 前方のパスが出せるロボットにパスを出す
                elif len(receiver_robots_id) > 0:
                    # レシーバ候補のロボットIDリストを取得
                    self._operator.pass_to_our_robot(robot_id, receiver_robots_id[0])
                # シュートもパスもできないときはこの条件に入らないように設定
                else:
                    self._act_id = ID_NO_SHOOT_PASS
                    return
                self._act_id = ID_SHOOT_PASS
            return

        # ゴールに向かってシュートする
        if self._act_id != ID_INPLAY:
            self._operator.shoot_to_their_goal_with_reflect(robot_id)
            self._act_id = ID_INPLAY

    def our_pre_kickoff(self, robot_id):
        if self._act_id != self.ACT_ID_PRE_KICKOFF:
            self._operator.chase_ball(robot_id, -0.6, 0.0, 0.0, look_from=False, keep=True)
            self._act_id = self.ACT_ID_PRE_KICKOFF

    def our_kickoff(self, robot_id):
        if self._act_id != self.ACT_ID_KICKOFF:
            self._operator.setplay_shoot_to_their_goal(robot_id)
            self._act_id = self.ACT_ID_KICKOFF

    def their_pre_kickoff(self, robot_id):
        if self._act_id != self.ACT_ID_PRE_KICKOFF:
            self._operator.chase_ball(robot_id, -0.6, 0.0, 0.0, look_from=False, keep=True)
            self._act_id = self.ACT_ID_PRE_KICKOFF

    def their_kickoff(self, robot_id):
        if self._act_id != self.ACT_ID_KICKOFF:
            self._operator.chase_ball(robot_id, -0.6, 0.0, 0.0, look_from=False, keep=True)
            self._act_id = self.ACT_ID_KICKOFF

    def our_pre_penalty(self, robot_id):
        if self._act_id != self.ACT_ID_PRE_PENALTY:
            self._operator.chase_ball(robot_id, -0.6, 0.0, 0.0, look_from=False, keep=True)
            self._act_id = self.ACT_ID_PRE_PENALTY

    def our_penalty(self, robot_id):
        if self._act_id != self.ACT_ID_PENALTY:
            self._operator.setplay_shoot_to_their_goal(robot_id)
            self._act_id = self.ACT_ID_PENALTY

    def their_pre_penalty(self, robot_id):
        if self._act_id != self.ACT_ID_PRE_PENALTY:
            self._operator.move_to_look_ball(robot_id, 6.0 - 0.5, 4.5 - 0.3 * 0.0)
            self._act_id = self.ACT_ID_PRE_PENALTY

    def their_penalty(self, robot_id):
        if self._act_id != self.ACT_ID_PENALTY:
            self._operator.move_to_look_ball(robot_id, 6.0 - 0.5, 4.5 - 0.3 * 0.0)
            self._act_id = self.ACT_ID_PENALTY

    def their_penalty_inplay(self, robot_id):
        if self._act_id != self.ACT_ID_INPLAY:
            self._operator.move_to_look_ball(robot_id, 6.0 - 0.5, 4.5 - 0.3 * 0.0)
            self._act_id = self.ACT_ID_INPLAY

    def our_penalty_inplay(self, robot_id):
        if self._act_id != self.ACT_ID_INPLAY:
            self._operator.setplay_shoot_to_their_goal(robot_id)
            self._act_id = self.ACT_ID_INPLAY

    def our_direct(self, robot_id):
        if self._act_id != self.ACT_ID_DIRECT:
            self._operator.setplay_shoot_to_their_goal(robot_id)
            self._act_id = self.ACT_ID_DIRECT

    def their_direct(self, robot_id):
        if self._act_id != self.ACT_ID_DIRECT:
            self._operator.move_to_defend_our_goal_from_ball(robot_id, 0.9)
            # self._operator.chase_ball(robot_id, -0.6, 0.0, 0.0, look_from=False, keep=True)
            self._act_id = self.ACT_ID_DIRECT

    def our_indirect(self, robot_id):
        if self._act_id != self.ACT_ID_INDIRECT:
            self._operator.setplay_shoot_to_their_goal(robot_id)
            self._act_id = self.ACT_ID_INDIRECT

    def their_indirect(self, robot_id):
        if self._act_id != self.ACT_ID_INDIRECT:
            self._operator.chase_ball(robot_id, -0.6, 0.0, 0.0, look_from=False, keep=True)
            self._act_id = self.ACT_ID_INDIRECT

    def their_ball_placement(self, robot_id, placement_pos):
        if self._act_id != self.ACT_ID_THEIR_PLACEMENT:
            self._operator.move_to_look_ball(robot_id, -6.0 + 2.0, 1.8)
            self._act_id = self.ACT_ID_THEIR_PLACEMENT

    def our_ball_placement(self, robot_id, placement_pos):
        ID_FAR_FROM = self.ACT_ID_OUR_PLACEMENT + 0
        ID_NEAR = self.ACT_ID_OUR_PLACEMENT + 1
        ID_ARRIVED = self.ACT_ID_OUR_PLACEMENT + 2

        # プレースメントを回避しない
        self._operator.disable_avoid_placement(robot_id)

        if self._ball_placement_state == FieldObserver.BALL_PLACEMENT_FAR_FROM_TARGET:
            # ボール位置が配置目標位置から離れているときはパスする
            if self._act_id != ID_FAR_FROM:
                self._operator.pass_to(robot_id, placement_pos.x, placement_pos.y)
                self._act_id = ID_FAR_FROM
            return
        
        if self._ball_placement_state == FieldObserver.BALL_PLACEMENT_NEAR_TARGET:
            # ボール位置が配置目標位置に近づいたときはドリブルする
            if self._act_id != ID_NEAR:
                self._operator.dribble_to(robot_id, placement_pos.x, placement_pos.y)
                self._act_id = ID_NEAR
            return

        if self._ball_placement_state == FieldObserver.BALL_PLACEMENT_ARRIVED_AT_TARGET:
            # ボール位置が配置目標位置に到着したらボールから離れる
            if self._act_id != ID_ARRIVED:
                self._operator.approach_to_ball(robot_id, 0.6)
                self._act_id = ID_ARRIVED
