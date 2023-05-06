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

        self.goal_pos_list = self._field_observer.get_goal_pos_list()
        self._old_reciever_robot_id = 0

    def stop(self, robot_id):
        ID_CHASE = self.ACT_ID_STOP + 0
        ID_IN_OUR_DEFENSE = self.ACT_ID_STOP + 1
        ID_IN_THEIR_DEFENSE = self.ACT_ID_STOP + 2

        # ボールが自分ディフェンスエリアにあるときは、ボールと同じ軸上に移動する
        if self._ball_state in [FieldObserver.BALL_IS_IN_OUR_DEFENSE_AREA, FieldObserver.BALL_IS_OUTSIDE_BACK_X, FieldObserver.BALL_IS_NEAR_OUTSIDE_BACK_X]:
        # if self._ball_state == FieldObserver.BALL_IS_IN_OUR_DEFENSE_AREA or self._ball_state == FieldObserver.BALL_IS_OUTSIDE_BACK_X:
            if self._act_id != ID_IN_OUR_DEFENSE:
                self._operator.move_to_ball_y(robot_id, -3.0)
                self._act_id = ID_IN_OUR_DEFENSE
            return

        # ボールが相手ディフェンスエリアにあるときは、ボールと同じ軸上に移動する
        if self._ball_state in [FieldObserver.BALL_IS_IN_THEIR_DEFENSE_AREA, FieldObserver.BALL_IS_OUTSIDE_FRONT_X, FieldObserver.BALL_IS_NEAR_OUTSIDE_FRONT_X]:
        # if self._ball_state == FieldObserver.BALL_IS_IN_THEIR_DEFENSE_AREA or self._ball_state == FieldObserver.BALL_IS_OUTSIDE_FRONT_X:
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
        ID_SHOOT_THEIR_AREA = self.ACT_ID_INPLAY + 1000
        ID_PASS = self.ACT_ID_INPLAY + 2000
        ID_SHOOT = self.ACT_ID_INPLAY + 3000

        # ボールが自分ディフェンスエリアにあるときは、ボールと同じ軸上に移動する
        if self._ball_state in [FieldObserver.BALL_IS_IN_OUR_DEFENSE_AREA, FieldObserver.BALL_IS_OUTSIDE_BACK_X, FieldObserver.BALL_IS_NEAR_OUTSIDE_BACK_X]:
            if self._act_id != ID_IN_OUR_DEFENSE:
                self._operator.move_to_ball_y(robot_id, -3.0)
                self._act_id = ID_IN_OUR_DEFENSE
            return

        # ボールが相手ディフェンスエリアにあるときは、ボールと同じ軸上に移動する
        if self._ball_state in [FieldObserver.BALL_IS_IN_THEIR_DEFENSE_AREA, FieldObserver.BALL_IS_OUTSIDE_FRONT_X, FieldObserver.BALL_IS_NEAR_OUTSIDE_FRONT_X]:
            if self._act_id != ID_IN_THEIR_DEFENSE:
                self._operator.move_to_ball_y(robot_id, 3.0)
                self._act_id = ID_IN_THEIR_DEFENSE
            return

        # パス・シュート可能なIDのリストと位置を取得
        receiver_robots_id, receiver_robots_pos, shoot_point_list, shoot_pos_list = self._field_observer.get_open_path_id_list(robot_id)

        # NamedTargetを更新
        if len(shoot_point_list) > 0:
            shoot_point = shoot_point_list[0]
            shoot_pos_x = self.goal_pos_list[shoot_point].x
            shoot_pos_y = self.goal_pos_list[shoot_point].y
            self._operator.set_named_target("shoot", shoot_pos_x, shoot_pos_y)
        else:
            self._operator.set_named_target("shoot", 6.0, 0.0)
        self._operator.publish_named_targets()

        # シュート可能かつ相手エリアにいる場合
        if len(shoot_point_list) > 0 and self._ball_state == FieldObserver.BALL_IS_IN_THEIR_SIDE:
            if self._act_id != ID_SHOOT_THEIR_AREA:
                # 指定座標に向けてシュートする
                self._operator.pass_to_named_target(robot_id, "shoot")
                self._act_id = ID_SHOOT_THEIR_AREA

        # パス可能な場合
        elif len(receiver_robots_id) > 0:
            if self._act_id != ID_PASS:
                # NamedTargetを更新
                _pos = receiver_robots_pos[0]
                pass_pos_x = _pos.x
                pass_pos_y = _pos.y
                self._operator.set_named_target("pass", pass_pos_x, pass_pos_y)
                self._operator.publish_named_targets()
                # 前方のパスが出せるロボットにパスを出す
                self._operator.pass_to_named_target(robot_id, "pass")
                self._act_id = ID_PASS

        # シュート可能かつ相手エリアにいる場合
        elif len(shoot_point_list) > 0:
            if self._act_id != ID_SHOOT:
                # 指定座標に向けてシュートする
                self._operator.pass_to_named_target(robot_id, "shoot")
                self._act_id != ID_SHOOT
        else:
            if self._act_id != ID_INPLAY:
                # 相手ゴールの中心に向かってシュートを打つ
                self._operator.pass_to_named_target(robot_id, "shoot")
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
            self._operator.move_to_look_ball(robot_id, self._PENALTY_WAIT_X, 4.5 - 0.3 * 0.0)
            self._act_id = self.ACT_ID_PRE_PENALTY

    def their_penalty(self, robot_id):
        if self._act_id != self.ACT_ID_PENALTY:
            self._operator.move_to_look_ball(robot_id, self._PENALTY_WAIT_X, 4.5 - 0.3 * 0.0)
            self._act_id = self.ACT_ID_PENALTY

    def their_penalty_inplay(self, robot_id):
        if self._act_id != self.ACT_ID_INPLAY:
            self._operator.move_to_look_ball(robot_id, self._PENALTY_WAIT_X, 4.5 - 0.3 * 0.0)
            self._act_id = self.ACT_ID_INPLAY

    def our_penalty_inplay(self, robot_id):
        ID_INPLAY_BALL_IN_DEFENSE_AREA = self.ACT_ID_INPLAY + 1
        # ボールがディフェンスエリアに入ったら、ボールと同じ軸上に移動する
        if self._ball_state in [FieldObserver.BALL_IS_IN_THEIR_DEFENSE_AREA, FieldObserver.BALL_IS_OUTSIDE_FRONT_X, FieldObserver.BALL_IS_NEAR_OUTSIDE_FRONT_X]:
            if self._act_id != ID_INPLAY_BALL_IN_DEFENSE_AREA:
                self._operator.move_to_ball_y(robot_id, 3.0)
                self._act_id = ID_INPLAY_BALL_IN_DEFENSE_AREA
        else:
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
        ID_NEAR_OUTSIDE = self.ACT_ID_OUR_PLACEMENT + 3

        # プレースメントを回避しない
        self._operator.disable_avoid_placement(robot_id)

        ball_state = self._field_observer.get_ball_state()
        # 壁際にあると判定した場合
        if ball_state in [FieldObserver.BALL_IS_NEAR_OUTSIDE_FRONT_X, FieldObserver.BALL_IS_NEAR_OUTSIDE_BACK_X,
                FieldObserver.BALL_IS_NEAR_OUTSIDE_LEFT_Y, FieldObserver.BALL_IS_NEAR_OUTSIDE_RIGHT_Y]:
            if self._act_id != ID_NEAR_OUTSIDE:
                # 壁際に蹴る位置を取得
                outside_kick_pos = self._field_observer.get_near_outside_ball_placement(ball_state, placement_pos)
                # 壁際に蹴る
                self._operator.pass_to(robot_id, outside_kick_pos.x, outside_kick_pos.y)
                self._act_id = ID_NEAR_OUTSIDE
            return

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
