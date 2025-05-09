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

"""ロボットがゴール前で防御を行うための操作を提供するモジュール."""

import math

from consai_examples.decisions.decision_base import DecisionBase
from consai_examples.operation import Operation
from consai_examples.operation import TargetTheta
from consai_examples.operation import TargetXY

from consai_msgs.msg import State2D

from consai_tools.geometry import geometry_tools


class GoaleDecision(DecisionBase):
    """ゴール前で防御を行うクラス."""

    def __init__(self, robot_operator, field_observer):
        """GoaleDecisionクラスの初期化関数."""
        super().__init__(robot_operator, field_observer)
        self._in_flag = 0

    def _defend_goal_operation(self):
        """ゴール前を守るための操作を行う関数."""
        # ゴール前を守る位置のマージン[m]
        MARGIN_X = self._div_a_x(0.2)
        MARGIN_Y = self._div_a_y(0.35)
        p1_x = self._field_pos().goal_pose('our', 'upper').x + MARGIN_X
        p1_y = self._field_pos().goal_pose('our', 'upper').y - MARGIN_Y
        p2_x = self._field_pos().goal_pose('our', 'lower').x + MARGIN_X
        p2_y = self._field_pos().goal_pose('our', 'lower').y + MARGIN_Y

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
                    if distance < self._div_a_dia(0.15) and ball_pos.x < robot_pos.x:
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
        elif self._div_a_x(0.2) < abs(ball_vel.x) and \
                self._div_a_x(0.1) < math.hypot(ball_vel.x, ball_vel.y):
            # 2点を結ぶ直線の傾きと切片を取得
            slope, intercept, _ = geometry_tools.get_line_parameter(ball_pos, ball_vel)
            # ゴール前との交点(y座標)を算出
            y = slope * p1_x + intercept
            if abs(y) < p1_y + MARGIN_Y:
                x = p1_x
                defend_pose = TargetXY.value(x, y)
                flag = 2

        if flag == 1:
            slope, intercept, _ = geometry_tools.get_line_parameter(
                ball_pos, State2D(x=self._div_a_x(-6.75), y=0.0))
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
        """ペナルティエリアを守るための操作を行う関数."""
        p1_x = self._div_a_x(-6.0 + 0.05)
        p1_y = self._div_a_y(0.9)
        p2_x = self._div_a_x(-6.0 + 0.05)
        p2_y = self._div_a_y(-0.9)
        defend_goal = Operation().move_to_intersection(
            TargetXY.value(p1_x, p1_y), TargetXY.value(p2_x, p2_y),
            TargetXY.our_goal(), TargetXY.ball(), TargetTheta.look_ball())
        defend_goal = defend_goal.with_ball_receiving()
        defend_goal = defend_goal.disable_avoid_defense_area()
        return defend_goal

    def stop(self, robot_id):
        """ロボットがゴール前で防御を行う関数."""
        defend_our_goal = self._defend_goal_operation()
        defend_our_goal = defend_our_goal.enable_avoid_ball()
        defend_our_goal = defend_our_goal.enable_avoid_pushing_robots()
        defend_our_goal = defend_our_goal.disable_avoid_defense_area()
        self._operator.operate(robot_id, defend_our_goal)

    def inplay(self, robot_id):
        """試合中にボールがディフェンスエリアにある場合の動作を行う関数."""
        # ボールがディフェンスエリアにあるときは、ボールを蹴る
        if self._field_observer.ball_position().is_in_our_defense_area() \
           and not self._field_observer.ball_motion().is_moving():

            move_to_behind_ball = Operation().move_on_line(
                TargetXY.ball(), TargetXY.our_goal(), self._div_a_x(0.05), TargetTheta.look_ball())
            move_to_behind_ball = move_to_behind_ball.with_ball_receiving()
            move_to_behind_ball = move_to_behind_ball.disable_avoid_defense_area()

            clear_pos_list = self._field_observer.pass_shoot().get_clear_pos_list()
            if len(clear_pos_list) > 0 and self._in_flag == 0:
                self._in_flag = 1
            # ボールがフィールド上側にあるときは、上側コーナを狙って蹴る
            elif self._field_observer.zone().ball_is_in_left_top() or \
                    self._field_observer.zone().ball_is_in_left_mid_top() and \
                    self._in_flag == 0:
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
        """相手のペナルティエリア前で防御を行う関数."""
        penalty_defend = self._penalty_defend_operation()
        self._operator.operate(robot_id, penalty_defend)

    def their_penalty(self, robot_id):
        """相手がペナルティキックを行う際に防御を行う関数."""
        penalty_defend = self._penalty_defend_operation()
        self._operator.operate(robot_id, penalty_defend)

    def our_ball_placement(self, robot_id, placement_pos):
        """味方のボール配置時に防御を行う関数."""
        defend_our_goal = self._defend_goal_operation()
        defend_our_goal = defend_our_goal.enable_avoid_placement_area(placement_pos)
        defend_our_goal = defend_our_goal.enable_avoid_pushing_robots()
        self._operator.operate(robot_id, defend_our_goal)

    def their_ball_placement(self, robot_id, placement_pos):
        """相手のボール配置時に防御を行う関数."""
        defend_our_goal = self._defend_goal_operation()
        defend_our_goal = defend_our_goal.enable_avoid_placement_area(placement_pos)
        defend_our_goal = defend_our_goal.enable_avoid_pushing_robots()
        self._operator.operate(robot_id, defend_our_goal)


def generate_defend_function():
    """ディフェンス用の関数を生成する関数."""
    def function(self, robot_id):
        """ロボットにディフェンスゴールを操作させる関数."""
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
