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

from consai_tools.geometry import geometry_tools
from consai_msgs.msg import State2D
from field import Field
import math


class GoaleDecision(DecisionBase):

    def __init__(self, robot_operator, field_observer):
        super().__init__(robot_operator, field_observer)
        self.our_goal_upper_pos = Field()._our_goal_dict['upper']
        self.our_goal_center_pos = Field()._our_goal_dict['center']
        self.our_goal_lower_pos = Field()._our_goal_dict['lower']
        # ゴール前を守る位置のマージン[m]
        self.margin_x = 0.2
        self.margin_y = 0.35
        self._in_flag = False

    def _defend_goal_operation(self):
        p1_x = self.our_goal_upper_pos.x + self.margin_x
        p1_y = self.our_goal_upper_pos.y - self.margin_y
        p2_x = self.our_goal_lower_pos.x + self.margin_x
        p2_y = self.our_goal_lower_pos.y + self.margin_y

        # ボールの座標を取得
        ball_pos = self._field_observer.detection().ball().pos()
        # ボールの速度を取得
        ball_vel = self._field_observer.detection().ball().vel()

        # ボールと敵ロボットの距離を取得
        distance_ball_to_their_robots = self._field_observer.distance().ball_to_their_robots()
        # ボールが味方側エリアにあるか取得
        is_in_our_side = self._field_observer.ball_position().is_in_our_side()

        # ボールと敵ロボットの状況を見てディフェンス座標を変更するフラグを生成
        flag = 1
        # 味方エリアにボールが存在かつボールに近い敵ロボットが存在する場合
        if is_in_our_side and len(distance_ball_to_their_robots.keys()):
            # ロボットの位置を取得
            robots = self._field_observer.detection().their_robots()
            for _ in range(len(distance_ball_to_their_robots)):
                # ボールに一番近い敵ロボットのIDを取得
                i = min(distance_ball_to_their_robots, key=distance_ball_to_their_robots.get)
                distance = distance_ball_to_their_robots.pop(i)
                # キーが存在している場合
                if i in robots:
                    # 一番近いロボットの位置を取得
                    robot_pos = robots[i].pos()

                    # 敵ロボットの距離が近いかつ距離の近い敵ロボットよりボールがゴール側にある場合
                    if distance < 0.15 and ball_pos.x < robot_pos.x:
                        # 2点を結ぶ直線の傾きと切片を取得
                        slope, intercept, _ = geometry_tools.get_line_parameter(
                            ball_pos, robot_pos)
                        # ゴール前との交点(y座標)を算出
                        y = slope * p1_x + intercept
                        if abs(y) < p1_y:
                            x = p1_x
                            defend_pose = TargetXY.value(x, y)
                            flag = 2
                    break

        # ボールがゴールに向かって来る場合
        elif 0.2 < abs(ball_vel.x) and 0.1 < math.hypot(ball_vel.x, ball_vel.y):
            # 2点を結ぶ直線の傾きと切片を取得
            slope, intercept, _ = geometry_tools.get_line_parameter(ball_pos, ball_vel)
            # ゴール前との交点(y座標)を算出
            y = slope * p1_x + intercept
            if abs(y) < p1_y + self.margin_y:
                x = p1_x
                defend_pose = TargetXY.value(x, y)
                flag = 2

        if flag == 1:
            slope, intercept, _ = geometry_tools.get_line_parameter(
                ball_pos, State2D(x=-6.75, y=0.0))
            y = slope * p1_x + intercept
            defend_goal = Operation().move_to_intersection(
                TargetXY.value(p1_x, p1_y), TargetXY.value(p2_x, p2_y),
                TargetXY.value(p1_x, y), TargetXY.ball(), TargetTheta.look_ball())
        if flag == 2:
            defend_goal = Operation().move_to_pose(
                defend_pose, TargetTheta.look_ball())

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

            move_to_behind_ball = Operation().move_on_line(
                TargetXY.ball(), TargetXY.our_goal(), 0.3, TargetTheta.look_ball())
            move_to_behind_ball = move_to_behind_ball.with_ball_receiving()
            move_to_behind_ball = move_to_behind_ball.disable_avoid_defense_area()

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
