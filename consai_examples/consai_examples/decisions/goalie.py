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
        defend_goal = defend_goal.disable_avoid_defense_area()
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
        defend_goal = defend_goal.disable_avoid_defense_area()
        return defend_goal

    def stop(self, robot_id):
        defend_our_goal = self._defend_goal_operation()
        defend_our_goal = defend_our_goal.enable_avoid_ball()
        defend_our_goal = defend_our_goal.enable_avoid_pushing_robots()
        defend_our_goal = defend_our_goal.disable_avoid_defense_area()
        self._operator.operate(robot_id, defend_our_goal)

    def inplay(self, robot_id):
        # ボールがディフェンスエリアにあるときは、ボールを蹴る
        if self._field_observer.ball_position().is_in_our_defense_area() \
           and not self._field_observer.ball_motion().is_moving():

            # パス可能なIDのリストを取得
            receivers_id_list = self._field_observer.pass_shoot().search_receivers_list(robot_id)

            move_to_behind_ball = Operation().move_on_line(
                TargetXY.ball(), TargetXY.our_goal(), 0.3, TargetTheta.look_ball())
            move_to_behind_ball = move_to_behind_ball.with_ball_receiving()
            move_to_behind_ball = move_to_behind_ball.disable_avoid_defense_area()

            if len(receivers_id_list) > 0:
                passing = move_to_behind_ball.with_passing_to(
                    TargetXY.our_robot(receivers_id_list[0]))
                self._operator.operate(robot_id, passing)
                return
            else:
                # ボールがフィールド上側にあるときは、上側コーナを狙って蹴る
                if self._field_observer.zone().ball_is_in_left_top() or \
                     self._field_observer.zone().ball_is_in_left_mid_top():
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

    def their_pre_penalty(self, robot_id):
        penalty_defend = self._penalty_defend_operation()
        self._operator.operate(robot_id, penalty_defend)

    def their_penalty(self, robot_id):
        penalty_defend = self._penalty_defend_operation()
        self._operator.operate(robot_id, penalty_defend)

    def our_ball_placement(self, robot_id, placement_pos):
        defend_our_goal = self._defend_goal_operation()
        defend_our_goal = defend_our_goal.enable_avoid_placement_area(placement_pos)
        defend_our_goal = defend_our_goal.enable_avoid_pushing_robots()
        self._operator.operate(robot_id, defend_our_goal)

    def their_ball_placement(self, robot_id, placement_pos):
        defend_our_goal = self._defend_goal_operation()
        defend_our_goal = defend_our_goal.enable_avoid_placement_area(placement_pos)
        defend_our_goal = defend_our_goal.enable_avoid_pushing_robots()
        self._operator.operate(robot_id, defend_our_goal)


def generate_defend_function():
    def function(self, robot_id):
        defend_our_goal = self._defend_goal_operation()
        self._operator.operate(robot_id, defend_our_goal)
    return function


defend_func_names = [
    'our_pre_kickoff', 'our_kickoff', 'their_pre_kickoff', 'their_kickoff',
    'our_pre_penalty', 'our_penalty',
    'our_penalty_inplay', 'their_penalty_inplay',
    'our_direct', 'their_direct', 'our_indirect', 'their_indirect',
]
for name in defend_func_names:
    setattr(GoaleDecision, name, generate_defend_function())
