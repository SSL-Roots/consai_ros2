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

from consai_msgs.msg import State2D
from decisions.decision_base import DecisionBase
from operation import Operation
from operation import TargetXY
from operation import TargetTheta


class AttackerDecision(DecisionBase):

    def __init__(self, robot_operator, field_observer):
        super().__init__(robot_operator, field_observer)

    def stop(self, robot_id):
        move_to_ball = Operation().move_to_pose(TargetXY.ball(), TargetTheta.look_ball())
        move_to_ball = move_to_ball.enable_avoid_ball()
        move_to_ball = move_to_ball.enable_avoid_pushing_robots()

        # ボールを追いかける
        chase_ball = move_to_ball.offset_pose_x(-0.6)
        self._operator.operate(robot_id, chase_ball)

    def inplay(self, robot_id):
        # 何メートル後ろの味方ロボットまでパス対象に含めるかオフセットをかける
        search_offset = 0.5
        move_to_ball = Operation().move_on_line(
            TargetXY.ball(), TargetXY.our_robot(robot_id), 0.3, TargetTheta.look_ball())
        move_to_ball = move_to_ball.with_ball_receiving()
        move_to_ball = move_to_ball.with_reflecting_to(TargetXY.their_goal())

        # シュートできる場合はシュートする
        shoot_pos_list = self._field_observer.pass_shoot().get_shoot_pos_list()
        if len(shoot_pos_list) > 0:
            shooting = move_to_ball.with_shooting_to(
                TargetXY.value(shoot_pos_list[0].x, shoot_pos_list[0].y))
            self._operator.operate(robot_id, shooting)
            return

        # パス可能なIDのリストを取得
        receivers_id_list = self._field_observer.pass_shoot().search_receivers_list(
            robot_id, search_offset)

        # パス可能な場合
        if len(receivers_id_list) > 0:
            passing = move_to_ball.with_passing_to(
                TargetXY.our_robot(receivers_id_list[0]))
            self._operator.operate(robot_id, passing)
            return

        # TODO(Roots):シュートもパスもできない場合の動きを考える
        # 後ろにドリブルとかしたいね
        dribbling = move_to_ball.with_dribbling_to(
            TargetXY.their_goal())
        self._operator.operate(robot_id, dribbling)
        return

    def our_direct(self, robot_id):
        # 何メートル後ろの味方ロボットまでパス対象に含めるかオフセットをかける
        search_offset = 0.5
        move_to_ball = Operation().move_to_pose(TargetXY.ball(), TargetTheta.look_ball())
        move_to_ball.with_ball_receiving()
        move_to_ball = move_to_ball.with_reflecting_to(TargetXY.their_goal())

        # シュート可能なIDリストを取得
        shoot_pos_list = self._field_observer.pass_shoot().get_shoot_pos_list()
        # シュートできる場合はシュートする
        if len(shoot_pos_list) > 0:
            shooting = move_to_ball.with_shooting_for_setplay_to(
                TargetXY.value(shoot_pos_list[0].x, shoot_pos_list[0].y))
            self._operator.operate(robot_id, shooting)
            return

        # パス可能なIDのリストを取得
        receivers_id_list = self._field_observer.pass_shoot().search_receivers_list(
            robot_id, search_offset)
        # パスできる味方ロボットがいる場合はパスする
        if len(receivers_id_list) > 0:
            passing = move_to_ball.with_passing_for_setplay_to(
                TargetXY.our_robot(receivers_id_list[0]))
            self._operator.operate(robot_id, passing)
            return

        # パスとシュートができない場合はゴール中央に向けてシュートする
        shooting_center = move_to_ball.with_shooting_for_setplay_to(
            TargetXY.their_goal())
        self._operator.operate(robot_id, shooting_center)
        return

    def their_direct(self, robot_id):
        prevent_direct_shooting = Operation().move_on_line(
            TargetXY.ball(), TargetXY.our_goal(), 0.9, TargetTheta.look_ball())
        prevent_direct_shooting = prevent_direct_shooting.with_ball_receiving()
        prevent_direct_shooting = prevent_direct_shooting.enable_avoid_ball()
        self._operator.operate(robot_id, prevent_direct_shooting)

    def their_indirect(self, robot_id):
        prevent_direct_shooting = Operation().move_on_line(
            TargetXY.ball(), TargetXY.our_goal(), 0.9, TargetTheta.look_ball())
        prevent_direct_shooting = prevent_direct_shooting.with_ball_receiving()
        prevent_direct_shooting = prevent_direct_shooting.enable_avoid_ball()
        self._operator.operate(robot_id, prevent_direct_shooting)

    def their_ball_placement(self, robot_id, placement_pos):
        operation = Operation().move_to_pose(TargetXY.ball(), TargetTheta.look_ball())
        operation = operation.offset_pose_x(-0.6)
        operation = operation.enable_avoid_ball()
        operation = operation.enable_avoid_placement_area(placement_pos)
        operation = operation.enable_avoid_pushing_robots()
        self._operator.operate(robot_id, operation)

    def _kick_pos_to_reflect_on_wall(self, placement_pos: State2D) -> State2D:
        # 壁に向かってボールを蹴り、反射させるためのシュート目標位置を生成する
        ball_pos = self._field_observer.detection().ball().pos()
        wall_x = self._field_observer.field_half_length() \
            + self._field_observer.field_margin_to_wall()
        wall_y = self._field_observer.field_half_width() \
            + self._field_observer.field_margin_to_wall()
        offset = 0.2  # ボールからのオフセット距離。値が大きいほどキック角度が浅くなる

        if self._field_observer.ball_position().is_outside_of_left_with_margin():
            if ball_pos.y > placement_pos.y:
                return State2D(x=-wall_x, y=ball_pos.y - offset)
            else:
                return State2D(x=-wall_x, y=ball_pos.y + offset)
        elif self._field_observer.ball_position().is_outside_of_right_with_margin():
            if ball_pos.y > placement_pos.y:
                return State2D(x=wall_x, y=ball_pos.y - offset)
            else:
                return State2D(x=wall_x, y=ball_pos.y + offset)
        elif self._field_observer.ball_position().is_outside_of_top_with_margin():
            if ball_pos.x > placement_pos.x:
                return State2D(x=ball_pos.x - offset, y=wall_y)
            else:
                return State2D(x=ball_pos.x + offset, y=wall_y)
        else:
            if ball_pos.x > placement_pos.x:
                return State2D(x=ball_pos.x - offset, y=-wall_y)
            else:
                return State2D(x=ball_pos.x + offset, y=-wall_y)

    def our_ball_placement(self, robot_id, placement_pos):
        # 壁際にあると判定した場合
        # ボールがフィールド外にある場合は、バックドリブルでボールをフィールド内に戻す
        if self._field_observer.ball_position().is_outside_with_margin():
            move_to_ball = Operation().move_on_line(
                TargetXY.ball(), TargetXY.our_robot(robot_id), 0.5, TargetTheta.look_ball())
            move_to_ball = move_to_ball.with_ball_receiving()
            back_dribble = move_to_ball.with_back_dribbling_to(TargetXY.value(0.0, 0.0))
            back_dribble = back_dribble.disable_avoid_defense_area()
            self._operator.operate(robot_id, back_dribble)
            return

        if self._field_observer.ball_placement().is_far_from(placement_pos):
            # ボール位置が配置目標位置から離れているときはパスする
            move_to_ball = Operation().move_on_line(
                TargetXY.ball(), TargetXY.our_robot(robot_id), 0.5, TargetTheta.look_ball())
            move_to_ball = move_to_ball.with_ball_receiving()
            passing = move_to_ball.with_passing_for_setplay_to(TargetXY.value(
                placement_pos.x, placement_pos.y))
            passing = passing.disable_avoid_defense_area()
            self._operator.operate(robot_id, passing)
            return

        if not self._field_observer.ball_placement().is_arrived_at(placement_pos):
            # ボール位置が配置目標位置に近づいたときはドリブルする
            move_to_behind_ball = Operation().move_on_line(
                TargetXY.ball(), TargetXY.value(placement_pos.x, placement_pos.y), -0.3,
                TargetTheta.look_ball())
            move_to_behind_ball = move_to_behind_ball.with_ball_receiving()
            dribbling = move_to_behind_ball.with_dribbling_to(
                TargetXY.value(placement_pos.x, placement_pos.y))
            dribbling = dribbling.disable_avoid_defense_area()
            self._operator.operate(robot_id, dribbling)
            return

        # ボール位置が配置目標位置に到着したらボールから離れる
        avoid_ball = Operation().move_on_line(
            TargetXY.ball(), TargetXY.our_robot(robot_id), 0.6, TargetTheta.look_ball())
        self._operator.operate(robot_id, avoid_ball)


def gen_chase_ball_function():
    def function(self, robot_id):
        move_to_ball = Operation().move_to_pose(TargetXY.ball(), TargetTheta.look_ball())
        chase_ball = move_to_ball.offset_pose_x(-0.6)
        chase_ball = chase_ball.enable_avoid_ball()
        self._operator.operate(robot_id, chase_ball)
    return function


def gen_setplay_shoot_function():
    def function(self, robot_id):
        move_to_ball = Operation().move_on_line(
            TargetXY.ball(), TargetXY.our_robot(robot_id), 0.05, TargetTheta.look_ball())
        setplay_shoot = move_to_ball.with_shooting_for_setplay_to(TargetXY.their_goal())
        self._operator.operate(robot_id, setplay_shoot)
    return function


def gen_their_penalty_function():
    def function(self, robot_id):
        operation = Operation().move_to_pose(
            TargetXY.value(self._PENALTY_WAIT_X, 4.5 - 0.3 * 0.0),
            TargetTheta.look_ball())
        operation = operation.enable_avoid_ball()
        self._operator.operate(robot_id, operation)
    return function


def gen_our_penalty_function():
    def function(self, robot_id):
        ball_pos = self._field_observer.detection().ball().pos()
        dribble_pos = State2D(x=ball_pos.x + 2.0, y=ball_pos.y)

        move_to_ball = Operation().move_to_pose(TargetXY.ball(), TargetTheta.look_ball())

        # ボールが相手ゴールに近づくまでは、ボールを弱く蹴って前進する
        if ball_pos.x < self._field_observer.field_half_length() * 0.25:
            dribbling = move_to_ball.with_passing_to(
                TargetXY.value(dribble_pos.x, dribble_pos.y))
            self._operator.operate(robot_id, dribbling)
            return
        else:
            # 空いているところを狙ってシュート
            shoot_pos_list = self._field_observer.pass_shoot().get_shoot_pos_list()
            if len(shoot_pos_list) > 0:
                shooting = move_to_ball.with_shooting_to(
                    TargetXY.value(shoot_pos_list[0].x, shoot_pos_list[0].y))
                self._operator.operate(robot_id, shooting)
                return
            shooting = move_to_ball.with_shooting_to(TargetXY.their_goal())
            self._operator.operate(robot_id, shooting)
    return function


for name in ['our_pre_kickoff', 'their_pre_kickoff', 'their_kickoff', 'our_pre_penalty']:
    setattr(AttackerDecision, name, gen_chase_ball_function())

for name in ['our_kickoff', 'our_penalty', 'our_indirect']:
    setattr(AttackerDecision, name, gen_setplay_shoot_function())

for name in ['their_pre_penalty', 'their_penalty', 'their_penalty_inplay']:
    setattr(AttackerDecision, name, gen_their_penalty_function())

for name in ['our_penalty', 'our_penalty_inplay']:
    setattr(AttackerDecision, name, gen_our_penalty_function())
