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

from consai_examples.decisions.decision_base import DecisionBase
from consai_examples.field import Field
from consai_examples.operation import Operation
from consai_examples.operation import TargetXY
from consai_examples.operation import TargetTheta
from consai_msgs.msg import State2D
from consai_tools.geometry import geometry_tools
import math


class GoaleDecision(DecisionBase):

    def __init__(self, robot_operator, field_observer):
        super().__init__(robot_operator, field_observer)
        self.our_goal_upper_pos = Field()._our_goal_dict['upper']
        self.our_goal_center_pos = Field()._our_goal_dict['center']
        self.our_goal_lower_pos = Field()._our_goal_dict['lower']
        # ゴール前を守る位置のマージン[m]
        self.margin_x = 0.3
        self.margin_y = 0.3
        self._in_flag = 0

    def _defend_goal_operation(self):
        p1_x = self.our_goal_upper_pos.x + self.margin_x
        p1_y = self.our_goal_upper_pos.y - self.margin_y
        p2_x = self.our_goal_lower_pos.x + self.margin_x
        p2_y = self.our_goal_lower_pos.y + self.margin_y

        # ボールの座標を取得
        ball_pos = self._field_observer.detection().ball().pos()
        # ボールの速度を取得
        ball_vel = self._field_observer.detection().ball().vel()

        # ボールと敵ロボットの状況を見てディフェンス座標を変更するフラグを生成
        flag = 1
        # ボールがゴールに向かって来る場合
        # if ball_vel.x < -0.2 and 0.1 < math.hypot(ball_vel.x, ball_vel.y):
        if 0.2 < abs(ball_vel.x) and 0.1 < math.hypot(ball_vel.x, ball_vel.y):
            # 2点を結ぶ直線の傾きと切片を取得
            next_ball_pos = State2D(x=ball_pos.x + ball_vel.x, y=ball_pos.y + ball_vel.y)
            slope, intercept, _ = geometry_tools.get_line_parameter(ball_pos, next_ball_pos)
            # ゴール前との交点を算出
            x = p2_x
            y = slope * x + intercept
            x1 = self.our_goal_upper_pos.x + 0.0
            y1 = self.our_goal_upper_pos.y + 0.1
            x2 = p1_x
            y2 = p1_y
            # ゴール際にくるボールの処理
            if p2_y < abs(y):
                # フィールドの上下それぞれで処理
                for _i in range(2):
                    # 2回目のループでは座標を反転しておく
                    if _i == 1:
                        y1 = -y1
                        y2 = -y2
                    # 交点を算出
                    slope2, intercept2, _ = geometry_tools.get_line_parameter(
                        State2D(x=x1, y=y1), State2D(x=x2, y=y2))
                    x = (intercept2 - intercept) / (slope - slope2)
                    y = (slope * intercept2 - slope2 * intercept) / (slope - slope2)
                    # ゴールを守るべきか判定
                    if x1 <= x and abs(y) < abs(y1):
                        if x < x1 + 0.1:
                            x = x1 + 0.1
                        defend_pose = TargetXY.value(x, y)
                        flag = 2
                        break
            else:
                defend_pose = TargetXY.value(x, y)
                flag = 2

        if flag == 1:
            slope, intercept, _ = geometry_tools.get_line_parameter(
                ball_pos, State2D(x=self.our_goal_upper_pos.x - 0.75, y=0.0))
            y = slope * p1_x + intercept
            defend_goal = Operation().move_to_intersection(
                TargetXY.value(p1_x, p1_y), TargetXY.value(p2_x, p2_y),
                TargetXY.value(p1_x, y), TargetXY.ball(), TargetTheta.look_ball())
        elif flag == 2:
            defend_goal = Operation().move_to_pose(
                defend_pose, TargetTheta.look_ball())

        # defend_goal = defend_goal.with_ball_receiving()
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
        # ボールがディフェンスエリアにあり停止しているときはボールを蹴る
        if self._field_observer.ball_position().is_in_our_defense_area() \
           and not self._field_observer.ball_motion().is_moving():

            move_to_behind_ball = Operation().move_on_line(
                TargetXY.ball(), TargetXY.our_goal(), 0.05, TargetTheta.look_ball())
            # move_to_behind_ball = move_to_behind_ball.with_ball_receiving()
            move_to_behind_ball = move_to_behind_ball.disable_avoid_defense_area()

            clear_pos_list = self._field_observer.pass_shoot().get_clear_pos_list()
            if self._in_flag == 0:
                if len(clear_pos_list) > 0:
                    self._in_flag = 1
                # ボールがフィールド上側にあるときは、上側コーナを狙って蹴る
                elif self._field_observer.zone().ball_is_in_left_top() or \
                        self._field_observer.zone().ball_is_in_left_mid_top():
                    self._in_flag = 2
                else:
                    self._in_flag = 3

            # フラグによる動作切り替え
            if self._in_flag == 1:
                move_to_behind_ball = move_to_behind_ball.with_shooting_to(
                    TargetXY.value(clear_pos_list[0].x, clear_pos_list[0].y))
                # self._operator.operate(robot_id, shooting)
            elif self._in_flag == 2:
                move_to_behind_ball = move_to_behind_ball.with_shooting_to(
                    TargetXY.their_top_corner())
            else:
                move_to_behind_ball = move_to_behind_ball.with_shooting_to(
                    TargetXY.their_bottom_corner())
            self._operator.operate(robot_id, move_to_behind_ball)
            return
        else:
            self._in_flag = 0

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
