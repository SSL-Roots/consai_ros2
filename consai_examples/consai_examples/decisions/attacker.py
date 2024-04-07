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
from field_observer import FieldObserver
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

        # ボールが自分ディフェンスエリアにあるときは、ボールと同じ軸上に移動する
        if self._field_observer.ball_position().is_in_our_defense_area() or \
           self._field_observer.ball_position().is_outside_of_left():
            move_to_ball = move_to_ball.overwrite_pose_x(-3.0)
            self._operator.operate(robot_id, move_to_ball)
            return

        # ボールが相手ディフェンスエリアにあるときは、ボールと同じ軸上に移動する
        if self._field_observer.ball_position().is_in_their_defense_area() or \
           self._field_observer.ball_position().is_outside_of_right():
            move_to_ball = move_to_ball.overwrite_pose_x(3.0)
            self._operator.operate(robot_id, move_to_ball)
            return

        # ボールを追いかける
        chase_ball = move_to_ball.offset_pose_x(-0.6)
        self._operator.operate(robot_id, chase_ball)

    def inplay(self, robot_id):
        move_to_ball = Operation().move_to_pose(TargetXY.ball(), TargetTheta.look_ball())
        move_to_ball = move_to_ball.with_ball_receiving()
        move_to_ball = move_to_ball.with_reflecting_to(TargetXY.their_goal())
        # ボールが自分ディフェンスエリアにあるときは、ボールと同じ軸上に移動する
        if self._field_observer.ball_position().is_in_our_defense_area() or \
           self._field_observer.ball_position().is_outside_of_left():
            move_to_ball = move_to_ball.overwrite_pose_x(-3.0)
            self._operator.operate(robot_id, move_to_ball)
            return

        # ボールが相手ディフェンスエリアにあるときは、ボールと同じ軸上に移動する
        if self._field_observer.ball_position().is_in_their_defense_area() or \
           self._field_observer.ball_position().is_outside_of_right():
            move_to_ball = move_to_ball.overwrite_pose_x(3.0)
            self._operator.operate(robot_id, move_to_ball)
            return

        # パス可能なIDのリストを取得
        receivers_id_list = self._field_observer.pass_shoot().search_receivers_list(robot_id)

        # パス可能な場合
        if len(receivers_id_list) > 0:
            passing = move_to_ball.with_passing_to(
                TargetXY.our_robot(receivers_id_list[0]))
            self._operator.operate(robot_id, passing)
            return
        else:
            shoot_pos = self._field_observer.pass_shoot().search_shoot_pos(robot_id)
            shooting = move_to_ball.with_shooting_to(
                TargetXY.value(shoot_pos.x, shoot_pos.y))
            self._operator.operate(robot_id, shooting)
            return

    def our_penalty_inplay(self, robot_id):
        # ボールがディフェンスエリアに入ったら、ボールと同じ軸上に移動する
        if self._field_observer.ball_position().is_in_their_defense_area() or \
           self._field_observer.ball_position().is_outside_of_right():
            move_to_ball = Operation().move_to_pose(TargetXY.ball(), TargetTheta.look_ball())
            move_to_ball = move_to_ball.overwrite_pose_x(3.0)
            self._operator.operate(robot_id, move_to_ball)
        else:
            move_to_ball = Operation().move_to_pose(TargetXY.ball(), TargetTheta.look_ball())
            setplay_shoot = move_to_ball.with_shooting_for_setplay_to(TargetXY.their_goal())
            self._operator.operate(robot_id, setplay_shoot)

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
        operation = Operation().move_to_pose(
            TargetXY.value(-6.0 + 2.0, 1.8),
            TargetTheta.look_ball())
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

        if self._field_observer.ball_position().is_outside_of_left():
            if ball_pos.y > placement_pos.y:
                return State2D(x=-wall_x, y=ball_pos.y - offset)
            else:
                return State2D(x=-wall_x, y=ball_pos.y + offset)
        elif self._field_observer.ball_position().is_outside_of_right():
            if ball_pos.y > placement_pos.y:
                return State2D(x=wall_x, y=ball_pos.y - offset)
            else:
                return State2D(x=wall_x, y=ball_pos.y + offset)
        elif self._field_observer.ball_position().is_outside_of_top():
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
        # ボールがフィールド外にある場合は、壁に向かってボールを蹴る
        if self._field_observer.ball_position().is_outside():
            kick_pos = self._kick_pos_to_reflect_on_wall(placement_pos)
            move_to_ball = Operation().move_to_pose(TargetXY.ball(), TargetTheta.look_ball())
            put_ball_back = move_to_ball.with_shooting_for_setplay_to(
                TargetXY.value(kick_pos.x, kick_pos.y))
            self._operator.operate(robot_id, put_ball_back)
            return

        if self._field_observer.ball_placement().is_far_from(placement_pos):
            # ボール位置が配置目標位置から離れているときはパスする
            move_to_ball = Operation().move_to_pose(TargetXY.ball(), TargetTheta.look_ball())
            passing = move_to_ball.with_passing_to(TargetXY.value(
                placement_pos.x, placement_pos.y))
            self._operator.operate(robot_id, passing)
            return

        if not self._field_observer.ball_placement().is_arrived_at(placement_pos):
            # ボール位置が配置目標位置に近づいたときはドリブルする
            move_to_behind_ball = Operation().move_on_line(
                TargetXY.ball(), TargetXY.value(placement_pos.x, placement_pos.y), -0.3,
                TargetTheta.look_ball())
            dribbling = move_to_behind_ball.with_dribbling_to(
                TargetXY.value(placement_pos.x, placement_pos.y))
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
        move_to_ball = Operation().move_to_pose(TargetXY.ball(), TargetTheta.look_ball())
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


for name in ['our_pre_kickoff', 'their_pre_kickoff', 'their_kickoff', 'our_pre_penalty']:
    setattr(AttackerDecision, name, gen_chase_ball_function())

for name in ['our_kickoff', 'our_penalty', 'our_direct', 'our_indirect']:
    setattr(AttackerDecision, name, gen_setplay_shoot_function())

for name in ['their_pre_penalty', 'their_penalty', 'their_penalty_inplay']:
    setattr(AttackerDecision, name, gen_their_penalty_function())
