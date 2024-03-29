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

import math
import copy

from rclpy.node import Node
from robocup_ssl_msgs.msg import TrackedBall
from robocup_ssl_msgs.msg import TrackedFrame
from robocup_ssl_msgs.msg import RobotId

from robocup_ssl_msgs.msg import TrackedRobot

from consai_msgs.msg import State2D
import consai_examples.geometry_tools as tool

# フィールド状況を観察し、ボールの位置を判断したり
# ロボットに一番近いロボットを判定する


class FieldObserver(Node):
    BALL_NONE = 0
    BALL_IS_OUTSIDE_FRONT_X = 1
    BALL_IS_OUTSIDE_BACK_X = 2
    BALL_IS_OUTSIDE_RIGHT_Y = 3
    BALL_IS_OUTSIDE_LEFT_Y = 4
    BALL_IS_IN_OUR_DEFENSE_AREA = 5
    BALL_IS_IN_OUR_SIDE = 6
    BALL_IS_IN_THEIR_SIDE = 7
    BALL_IS_IN_THEIR_DEFENSE_AREA = 8

    # 壁際に近いか判定
    BALL_IS_NEAR_OUTSIDE_FRONT_X = 21
    BALL_IS_NEAR_OUTSIDE_BACK_X = 22
    BALL_IS_NEAR_OUTSIDE_RIGHT_Y = 23
    BALL_IS_NEAR_OUTSIDE_LEFT_Y = 24

    BALL_PLACEMENT_NONE = 0
    BALL_PLACEMENT_FAR_FROM_TARGET = 1
    BALL_PLACEMENT_NEAR_TARGET = 2
    BALL_PLACEMENT_ARRIVED_AT_TARGET = 3

    BALL_ZONE_NONE = 0
    BALL_ZONE_LEFT_TOP = 1
    BALL_ZONE_LEFT_MID_TOP = 2
    BALL_ZONE_LEFT_MID_BOTTOM = 3
    BALL_ZONE_LEFT_BOTTOM = 4
    BALL_ZONE_RIGHT_TOP = 5
    BALL_ZONE_RIGHT_MID_TOP = 6
    BALL_ZONE_RIGHT_MID_BOTTOM = 7
    BALL_ZONE_RIGHT_BOTTOM = 8

    THRESHOLD_MARGIN = 0.05  # meters. 状態変化のしきい値にヒステリシスをもたせる
    MAX_ROBOT_NUM = 16

    GOAL_POST_Y = 0.9  # meters
    GOAL_POINT = 5  # ゴール候補のポイント
    GOAL_POST_TOP = [6, 0.9]  # meters
    GOAL_POST_BOTTOM = [6, -0.9]  # meters
    GOAL_CENTER = [6, 0]  # meters
    GOAL_TOP_CENTER = [6, 0.45]  # meters
    GOAL_CENTER_BOTTOM = [6, -0.45]  # meters
    GOAL_POST_WIDTH = 1.8  # meters
    ROBOT_RADIUS = 0.1  # meters
    GOAL_POST_TOP_NUM = 0
    GOAL_TOP_CENTER_NUM = 1
    GOAL_CENTER_NUM = 2
    GOAL_CENTER_BOTTOM_NUM = 3
    GOAL_POST_BOTTOM_NUM = 4

    def __init__(self, our_team_is_yellow=False):
        super().__init__('field_observer')

        self._our_team_is_yellow = our_team_is_yellow
        self._ball_state = self.BALL_NONE
        self._ball_placement_state = self.BALL_PLACEMENT_NONE
        self._ball_zone_state = self.BALL_ZONE_NONE
        self._ball_is_moving = False
        self._zone_targets = {0: None, 1: None, 2: None, 3: None}
        self._ball = TrackedBall()
        self._detection = TrackedFrame()

        self._our_robot = TrackedRobot()
        self._their_robot = TrackedRobot()
        self.robots = TrackedRobot()

        # ロボットのIDリストを初期化(存在するIDのみリストに格納)
        self.our_robot_id_list = []
        self.their_robot_id_list = []
        # 味方ロボットの位置と速度を初期化
        self.our_robots_pos = [None] * self.MAX_ROBOT_NUM
        self.our_robots_vel = [None] * self.MAX_ROBOT_NUM
        # 敵ロボットの位置と速度を初期化
        self.their_robots_pos = [None] * self.MAX_ROBOT_NUM
        self.their_robots_vel = [None] * self.MAX_ROBOT_NUM

        self._field_x = 12.0  # meters
        self._field_half_x = self._field_x * 0.5
        self._field_y = 9.0  # meters
        self._field_half_y = self._field_y * 0.5
        self._field_quarter_y = self._field_half_y * 0.5
        self._field_defense_x = 1.8  # meters
        self._field_defense_y = 3.6  # meters
        self._field_defense_half_y = self._field_defense_y * 0.5  # meters
        self._sub_detection_traced = self.create_subscription(
            TrackedFrame, 'detection_tracked', self._detection_tracked_callback, 10)

        self.goal_id_list = [0, 1, 2]
        self.goal_pos_list = [
            State2D(x=self._field_half_x, y=0.45),
            State2D(x=self._field_half_x, y=0.0),
            State2D(x=self._field_half_x, y=-0.45),
        ]
        self.goal_vel_list = [None] * 5

    def _detection_tracked_callback(self, msg):
        self._detection = msg
        if len(msg.balls) > 0:
            self._update_ball_state(msg.balls[0])
            self._update_ball_moving_state(msg.balls[0])
            self._update_ball_zone_state(msg.balls[0].pos)
            self._ball = msg.balls[0]

        # 位置・速度・IDのリストを初期化
        self.our_robot_id_list = []
        self.their_robot_id_list = []
        def set_none(a_list): return [None] * len(a_list)
        self.our_robots_pos = set_none(self.our_robots_pos)
        self.our_robots_vel = set_none(self.our_robots_vel)
        self.their_robots_pos = set_none(self.their_robots_pos)
        self.their_robots_vel = set_none(self.their_robots_vel)

        for robot in msg.robots:
            # visibilityが小さいときはロボットが消えたと判断する
            if len(robot.visibility) <= 0:
                continue
            if robot.visibility[0] <= 0.2:
                continue
            robot_id = robot.robot_id.id
            robot_is_yellow = robot.robot_id.team_color == RobotId.TEAM_COLOR_YELLOW
            team_str = "our" if self._our_team_is_yellow == robot_is_yellow else "their"

            # 位置・速度・IDを更新
            # TODO: robotとrobotsが混ざっているので直したい
            object_str = "self." + team_str + "_robots_"
            eval(object_str + "pos")[robot_id] = State2D(x=robot.pos.x,
                                                         y=robot.pos.y, theta=robot.orientation)
            if robot.vel and robot.vel_angular:
                eval(object_str + "vel")[robot_id] = State2D(x=robot.vel[0].x,
                                                             y=robot.vel[0].y,
                                                             theta=robot.vel_angular[0])
            eval("self." + team_str + "_robot_id_list").append(robot_id)

    def _update_ball_state(self, ball):
        # ball placement時に壁際かどうか判定
        _is_near_outside = self._check_near_outside(ball.pos)
        if _is_near_outside:
            self._ball_state = _is_near_outside
            return

        # フィールド場外判定(Goal-to-Goal方向)
        if self._check_is_ball_outside_front_x(ball.pos):
            self._ball_state = self.BALL_IS_OUTSIDE_FRONT_X
            return
        if self._check_is_ball_outside_back_x(ball.pos):
            self._ball_state = self.BALL_IS_OUTSIDE_BACK_X
            return

        # フィールド場外判定(Halfway方向)
        if self._check_is_ball_outside_right_y(ball.pos):
            self._ball_state = self.BALL_IS_OUTSIDE_RIGHT_Y
            return
        if self._check_is_ball_outside_left_y(ball.pos):
            self._ball_state = self.BALL_IS_OUTSIDE_LEFT_Y
            return

        # 自チームディフェンスエリア侵入判定
        if self._check_is_ball_in_defense_area(ball.pos, our_area=True):
            self._ball_state = self.BALL_IS_IN_OUR_DEFENSE_AREA
            return

        # 相手チームディフェンスエリア侵入判定
        if self._check_is_ball_in_defense_area(ball.pos, our_area=False):
            self._ball_state = self.BALL_IS_IN_THEIR_DEFENSE_AREA
            return

        # 自チームエリア侵入判定
        if self._check_is_ball_in_our_side(ball.pos):
            self._ball_state = self.BALL_IS_IN_OUR_SIDE
            return

        # 条件に入らなければ、相手チームエリアに侵入したと判定
        self._ball_state = self.BALL_IS_IN_THEIR_SIDE

    def _check_is_ball_outside_front_x(self, ball_pos):
        # ボールがフィールド外に出たか判定
        # xはGoal-to-Goalの方向
        # 相手ゴール方向
        threshold_x = self._field_half_x
        if self.ball_is_outside_front_x():
            threshold_x -= self.THRESHOLD_MARGIN

        if ball_pos.x > threshold_x:
            return True
        return False

    def _check_is_ball_outside_back_x(self, ball_pos):
        # ボールがフィールド外に出たか判定
        # xはGoal-to-Goalの方向
        # 自ゴール方向
        threshold_x = self._field_half_x
        if self.ball_is_outside_back_x():
            threshold_x += self.THRESHOLD_MARGIN

        if ball_pos.x < -threshold_x:
            return True
        return False

    def _check_is_ball_outside_right_y(self, ball_pos):
        # ボールがフィールド外に出たか判定
        # yはHalfwayの方向
        # 相手ゴールに対して右側
        threshold_y = self._field_half_y
        if self.ball_is_outside_right_y():
            threshold_y -= self.THRESHOLD_MARGIN

        if ball_pos.y > threshold_y:
            return True
        return False

    def _check_is_ball_outside_left_y(self, ball_pos):
        # ボールがフィールド外に出たか判定
        # yはHalfwayの方向
        # 相手ゴールに対して左側
        threshold_y = self._field_half_y
        if self.ball_is_outside_left_y():
            threshold_y += self.THRESHOLD_MARGIN

        if ball_pos.y < -threshold_y:
            return True
        return False

    def _check_is_ball_in_defense_area(self, ball_pos, our_area=True):
        # ボールがディフェンスエリアに入ったか判定
        threshold_x = self._field_half_x - self._field_defense_x
        threshold_y = self._field_defense_half_y + 0.1  # ロボットの半径分マージンをとる
        if self.ball_is_in_our_defense_area() or self.ball_is_in_their_defense_area():
            threshold_x -= self.THRESHOLD_MARGIN
            threshold_y += self.THRESHOLD_MARGIN

        if our_area and ball_pos.x < -threshold_x and math.fabs(ball_pos.y) < threshold_y:
            return True
        elif not our_area and ball_pos.x > threshold_x and math.fabs(ball_pos.y) < threshold_y:
            return True
        return False

    def _check_is_ball_in_our_side(self, ball_pos):
        # ボールがディフェンスエリアに入ったか判定
        threshold_x = 0.0
        if self.ball_is_in_our_side():
            threshold_x += self.THRESHOLD_MARGIN

        if ball_pos.x < threshold_x:
            return True
        return False

    def _check_near_outside(self, ball_pos):
        # ボールが壁際に近いか判定
        flag = 0
        # 壁際に近いと判定する距離
        threshold = 0.0

        if ball_pos.x < -self._field_half_x + threshold:
            flag = self.BALL_IS_NEAR_OUTSIDE_BACK_X
        elif self._field_half_x - threshold < ball_pos.x:
            flag = self.BALL_IS_NEAR_OUTSIDE_FRONT_X
        elif ball_pos.y < -self._field_half_y + threshold:
            flag = self.BALL_IS_NEAR_OUTSIDE_RIGHT_Y
        elif self._field_half_y - threshold < ball_pos.y:
            flag = self.BALL_IS_NEAR_OUTSIDE_LEFT_Y
        return flag

    def _update_ball_moving_state(self, ball):
        BALL_MOVING_THRESHOLD = 1.0  # m/s
        BALL_MOVING_HYSTERESIS = 0.3  # m/s

        # ボールが動いているか判定する
        if len(ball.vel) == 0:
            self._ball_is_moving = False
            return
        velocity_norm = math.hypot(ball.vel[0].x, ball.vel[0].y)

        # ボール速度がしきい値付近で揺れても、判定が切り替わらないようにヒステリシスを設ける
        threshold = BALL_MOVING_THRESHOLD
        if self._ball_is_moving:
            threshold = BALL_MOVING_THRESHOLD - BALL_MOVING_HYSTERESIS

        if velocity_norm > threshold:
            self._ball_is_moving = True
        else:
            self._ball_is_moving = False

        if ball.vel[0].x < 0:
            self._ball_to_our_field = True
        else:
            self._ball_to_our_field = False

    def _update_ball_zone_state(self, ball_pos):
        ZONE_THRESHOLD = 0.2  # meters
        # ボールがどのZONEに存在するのかを判定する
        threshold_x = 0.0
        if self.ball_is_in_our_side():
            threshold_x += ZONE_THRESHOLD

        threshold_y_top = self._field_quarter_y
        if self.ball_is_in_right_top_zone() or self.ball_is_in_left_top_zone():
            threshold_y_top -= ZONE_THRESHOLD

        threshold_y_mid_top = 0.0
        if self.ball_is_in_right_mid_top_zone() or self.ball_is_in_left_mid_top_zone():
            threshold_y_mid_top -= ZONE_THRESHOLD

        threshold_y_mid_bottom = -self._field_quarter_y
        if self.ball_is_in_right_mid_bottom_zone() or self.ball_is_in_left_mid_bottom_zone():
            threshold_y_mid_bottom -= ZONE_THRESHOLD

        if ball_pos.x > threshold_x:
            if ball_pos.y > threshold_y_top:
                self._ball_zone_state = self.BALL_ZONE_RIGHT_TOP
            elif ball_pos.y > threshold_y_mid_top:
                self._ball_zone_state = self.BALL_ZONE_RIGHT_MID_TOP
            elif ball_pos.y > threshold_y_mid_bottom:
                self._ball_zone_state = self.BALL_ZONE_RIGHT_MID_BOTTOM
            else:
                self._ball_zone_state = self.BALL_ZONE_RIGHT_BOTTOM
        else:
            if ball_pos.y > threshold_y_top:
                self._ball_zone_state = self.BALL_ZONE_LEFT_TOP
            elif ball_pos.y > threshold_y_mid_top:
                self._ball_zone_state = self.BALL_ZONE_LEFT_MID_TOP
            elif ball_pos.y > threshold_y_mid_bottom:
                self._ball_zone_state = self.BALL_ZONE_LEFT_MID_BOTTOM
            else:
                self._ball_zone_state = self.BALL_ZONE_LEFT_BOTTOM

    def update_zone_targets(self, number_of_zone_roles):
        # ゾーンディフェンス担当者に合わせて、マンマークする相手ロボットを検出する
        # 存在しない場合はNoneをセットする
        their_robots = []
        for robot in self._detection.robots:
            # 相手チームのロボットを抽出
            target_is_blue = self._our_team_is_yellow \
                and robot.robot_id.team_color == RobotId.TEAM_COLOR_BLUE
            target_is_yellow = not self._our_team_is_yellow \
                and robot.robot_id.team_color == RobotId.TEAM_COLOR_YELLOW
            if target_is_blue or target_is_yellow:
                their_robots.append(robot)

        self._reset_zone_targets()
        if number_of_zone_roles == 1:
            self._update_zone1_targets(their_robots)
        elif number_of_zone_roles == 2:
            self._update_zone2_targets(their_robots)
        elif number_of_zone_roles == 3:
            self._update_zone3_targets(their_robots)
        elif number_of_zone_roles == 4:
            self._update_zone4_targets(their_robots)
        return self._zone_targets

    def _reset_zone_targets(self):
        # ZONEターゲットを初期化する
        self._zone_targets = {0: None, 1: None, 2: None, 3: None}

    def _update_zone1_targets(self, their_robots):
        # ZONE1に属するターゲットを抽出する
        nearest_x = 10.0
        nearest_id = None
        for robot in their_robots:
            if self._is_in_zone1_0(robot.pos):
                if robot.pos.x < nearest_x:
                    nearest_x = robot.pos.x
                    nearest_id = robot.robot_id.id

        self._zone_targets[0] = nearest_id

    def _update_zone2_targets(self, their_robots):
        # ZONE2に属するターゲットを抽出する
        nearest_x0 = 10.0
        nearest_x1 = 10.0
        nearest_id0 = None
        nearest_id1 = None
        for robot in their_robots:
            if self._is_in_zone2_0(robot.pos):
                if robot.pos.x < nearest_x0:
                    nearest_x0 = robot.pos.x
                    nearest_id0 = robot.robot_id.id
                continue

            if self._is_in_zone2_1(robot.pos):
                if robot.pos.x < nearest_x1:
                    nearest_x1 = robot.pos.x
                    nearest_id1 = robot.robot_id.id
                continue

        self._zone_targets[0] = nearest_id0
        self._zone_targets[1] = nearest_id1

    def _update_zone3_targets(self, their_robots):
        # ZONE3に属するターゲットを抽出する
        nearest_x0 = 10.0
        nearest_x1 = 10.0
        nearest_x2 = 10.0
        nearest_id0 = None
        nearest_id1 = None
        nearest_id2 = None
        for robot in their_robots:
            if self._is_in_zone3_0(robot.pos):
                if robot.pos.x < nearest_x0:
                    nearest_x0 = robot.pos.x
                    nearest_id0 = robot.robot_id.id
                continue

            if self._is_in_zone3_1(robot.pos):
                if robot.pos.x < nearest_x1:
                    nearest_x1 = robot.pos.x
                    nearest_id1 = robot.robot_id.id
                continue

            if self._is_in_zone3_2(robot.pos):
                if robot.pos.x < nearest_x2:
                    nearest_x2 = robot.pos.x
                    nearest_id2 = robot.robot_id.id
                continue

        self._zone_targets[0] = nearest_id0
        self._zone_targets[1] = nearest_id1
        self._zone_targets[2] = nearest_id2

    def _update_zone4_targets(self, their_robots):
        # ZONE4に属するターゲットを抽出する
        nearest_x0 = 10.0
        nearest_x1 = 10.0
        nearest_x2 = 10.0
        nearest_x3 = 10.0
        nearest_id0 = None
        nearest_id1 = None
        nearest_id2 = None
        nearest_id3 = None
        for robot in their_robots:
            if self._is_in_zone4_0(robot.pos):
                if robot.pos.x < nearest_x0:
                    nearest_x0 = robot.pos.x
                    nearest_id0 = robot.robot_id.id
                continue

            if self._is_in_zone4_1(robot.pos):
                if robot.pos.x < nearest_x1:
                    nearest_x1 = robot.pos.x
                    nearest_id1 = robot.robot_id.id
                continue

            if self._is_in_zone4_2(robot.pos):
                if robot.pos.x < nearest_x2:
                    nearest_x2 = robot.pos.x
                    nearest_id2 = robot.robot_id.id
                continue

            if self._is_in_zone4_3(robot.pos):
                if robot.pos.x < nearest_x3:
                    nearest_x3 = robot.pos.x
                    nearest_id3 = robot.robot_id.id
                continue

        self._zone_targets[0] = nearest_id0
        self._zone_targets[1] = nearest_id1
        self._zone_targets[2] = nearest_id2
        self._zone_targets[3] = nearest_id3

    def _is_in_defence_area(self, pos):
        # ディフェンスエリアに入ってたらtrue
        defense_x = -6.0 + 1.8 + 0.6  # 0.4はロボットの直径x2
        defense_y = 1.8 + 0.6  # 0.4はロボットの直径x2
        if pos.x < defense_x and math.fabs(pos.y) < defense_y:
            return True
        return False

    def _is_in_zone1_0(self, pos):
        # ZONE1 (左サイドの全部)にロボットがいればtrue
        if self._is_in_defence_area(pos):
            return False

        if pos.x < 0.0:
            return True
        return False

    def _is_in_zone2_0(self, pos):
        # ZONE2 (左サイドの上半分)にロボットがいればtrue
        if self._is_in_defence_area(pos):
            return False
        if pos.x < 0.0 and pos.y > 0.0:
            return True
        return False

    def _is_in_zone2_1(self, pos):
        # ZONE2 (左サイドの下半分)にロボットがいればtrue
        if self._is_in_defence_area(pos):
            return False
        if pos.x < 0.0 and pos.y <= 0.0:
            return True
        return False

    def _is_in_zone3_0(self, pos):
        # ZONE3 (左サイドの上半分の上)にロボットがいればtrue
        if self._is_in_defence_area(pos):
            return False
        if pos.x < 0.0 and pos.y > 4.5 * 0.5:
            return True
        return False

    def _is_in_zone3_1(self, pos):
        # ZONE3 (左サイドの真ん中)にロボットがいればtrue
        if self._is_in_defence_area(pos):
            return False
        if pos.x < 0.0 and math.fabs(pos.y) <= 4.5 * 0.5:
            return True
        return False

    def _is_in_zone3_2(self, pos):
        # ZONE3 (左サイドの下半分の下)にロボットがいればtrue
        if self._is_in_defence_area(pos):
            return False
        if pos.x < 0.0 and pos.y < -4.5 * 0.5:
            return True
        return False

    def _is_in_zone4_0(self, pos):
        # ZONE4 (左サイドの上半分の上)にロボットがいればtrue
        if self._is_in_defence_area(pos):
            return False
        if pos.x < 0.0 and pos.y > 4.5 * 0.5:
            return True
        return False

    def _is_in_zone4_1(self, pos):
        # ZONE4 (左サイドの上半分の上)にロボットがいればtrue
        if self._is_in_defence_area(pos):
            return False
        if pos.x < 0.0 and pos.y > 0.0 and pos.y <= 4.5 * 0.5:
            return True
        return False

    def _is_in_zone4_2(self, pos):
        # ZONE4 (左サイドの下半分の上)にロボットがいればtrue
        if self._is_in_defence_area(pos):
            return False
        if pos.x < 0.0 and pos.y <= 0.0 and pos.y > -4.5 * 0.5:
            return True
        return False

    def _is_in_zone4_3(self, pos):
        # ZONE4 (左サイドの下半分の下)にロボットがいればtrue
        if self._is_in_defence_area(pos):
            return False
        if pos.x < 0.0 and pos.y <= -4.5 * 0.5:
            return True
        return False

    def get_ball_state(self):
        return self._ball_state

    def ball_is_outside_front_x(self):
        return self._ball_state == self.BALL_IS_OUTSIDE_FRONT_X

    def ball_is_outside_back_x(self):
        return self._ball_state == self.BALL_IS_OUTSIDE_BACK_X

    def ball_is_outside_right_y(self):
        return self._ball_state == self.BALL_IS_OUTSIDE_RIGHT_Y

    def ball_is_outside_left_y(self):
        return self._ball_state == self.BALL_IS_OUTSIDE_LEFT_Y

    def ball_is_in_our_defense_area(self):
        return self._ball_state == self.BALL_IS_IN_OUR_DEFENSE_AREA

    def ball_is_in_their_defense_area(self):
        return self._ball_state == self.BALL_IS_IN_THEIR_DEFENSE_AREA

    def ball_is_in_our_side(self):
        return self._ball_state == self.BALL_IS_IN_OUR_SIDE

    def ball_is_moving(self):
        return self._ball_is_moving

    def ball_to_our_field(self):
        return self._ball_to_our_field

    def get_ball_zone_state(self):
        return self._ball_zone_state

    def ball_is_in_left_top_zone(self):
        return self._ball_zone_state == self.BALL_ZONE_LEFT_TOP

    def ball_is_in_left_mid_top_zone(self):
        return self._ball_zone_state == self.BALL_ZONE_LEFT_MID_TOP

    def ball_is_in_left_mid_bottom_zone(self):
        return self._ball_zone_state == self.BALL_ZONE_LEFT_MID_BOTTOM

    def ball_is_in_left_bottom_zone(self):
        return self._ball_zone_state == self.BALL_ZONE_LEFT_BOTTOM

    def ball_is_in_right_top_zone(self):
        return self._ball_zone_state == self.BALL_ZONE_RIGHT_TOP

    def ball_is_in_right_mid_top_zone(self):
        return self._ball_zone_state == self.BALL_ZONE_RIGHT_MID_TOP

    def ball_is_in_right_mid_bottom_zone(self):
        return self._ball_zone_state == self.BALL_ZONE_RIGHT_MID_BOTTOM

    def ball_is_in_right_bottom_zone(self):
        return self._ball_zone_state == self.BALL_ZONE_RIGHT_BOTTOM

    def _update_ball_placement_state(self, placement_position):
        ARRIVED_THRESHOLD = 0.13  # meters
        ARRIVED_VELOCTY_THRESHOLD = 0.2  # m/s
        NEAR_THRESHOLD = 3.0
        THRESHOLD_MARGIN = 0.02
        diff_x = placement_position.x - self._ball.pos.x
        diff_y = placement_position.y - self._ball.pos.y
        distance = math.hypot(diff_x, diff_y)
        # ボール速度が小さければ、目標位置にたどり着いたと判定する
        velocity_norm = 0.0
        if len(self._ball.vel) > 0:
            velocity_norm = math.hypot(
                self._ball.vel[0].x, self._ball.vel[0].y)

        arrived_threshold = ARRIVED_THRESHOLD
        near_threshold = NEAR_THRESHOLD
        if self._ball_placement_state == self.BALL_PLACEMENT_ARRIVED_AT_TARGET:
            arrived_threshold += THRESHOLD_MARGIN
        elif self._ball_placement_state == self.BALL_PLACEMENT_NEAR_TARGET:
            near_threshold += THRESHOLD_MARGIN

        if distance < arrived_threshold and velocity_norm < ARRIVED_VELOCTY_THRESHOLD:
            self._ball_placement_state = self.BALL_PLACEMENT_ARRIVED_AT_TARGET
        elif distance < near_threshold:
            self._ball_placement_state = self.BALL_PLACEMENT_NEAR_TARGET
        else:
            self._ball_placement_state = self.BALL_PLACEMENT_FAR_FROM_TARGET

    def get_ball_placement_state(self, placement_position):
        self._update_ball_placement_state(placement_position)
        return self._ball_placement_state

    def get_near_outside_ball_placement(self, ball_state, placement_position):

        margin = 0.5
        ball_pos = self._ball.pos

        if ball_state == self.BALL_IS_NEAR_OUTSIDE_FRONT_X:
            x = self._field_half_x + margin
            if ball_pos.y < placement_position.y:
                y = ball_pos.y + margin
            else:
                y = ball_pos.y - margin

        elif ball_state == self.BALL_IS_NEAR_OUTSIDE_BACK_X:
            x = -self._field_half_x - margin
            if ball_pos.y < placement_position.y:
                y = ball_pos.y + margin
            else:
                y = ball_pos.y - margin

        elif ball_state == self.BALL_IS_NEAR_OUTSIDE_LEFT_Y:
            y = self._field_half_y + margin
            if ball_pos.x < placement_position.x:
                x = ball_pos.x + margin
            else:
                x = ball_pos.x - margin

        elif ball_state == self.BALL_IS_NEAR_OUTSIDE_RIGHT_Y:
            y = -self._field_half_y - margin
            if ball_pos.x < placement_position.x:
                x = ball_pos.x + margin
            else:
                x = ball_pos.x - margin

        outside_kick_pos = State2D(x=x, y=y)

        return outside_kick_pos

    def get_ball_pos(self):
        return self._ball.pos

    def get_goal_pos_list(self):
        return self.goal_pos_list

    def get_open_path_id_list(self, my_robot_id, select_forward_between=1):

        try:
            can_pass_id_list, can_pass_pos_list = self.get_receiver_robots_id(
                my_robot_id, need_pass=True)
        except ValueError:  # Lint対策のため、bare exceptを使用しない
            can_pass_id_list = []
            can_pass_pos_list = []

        try:
            can_shoot_id_list, can_shoot_pos_list = self.get_receiver_robots_id(
                my_robot_id, need_shoot=True)
        except ValueError:  # Lint対策のため、bare exceptを使用しない
            can_shoot_id_list = []
            can_shoot_pos_list = []

        return can_pass_id_list, can_pass_pos_list, can_shoot_id_list, can_shoot_pos_list

    def get_receiver_robots_id(
            self, my_robot_id, need_pass=False, need_shoot=False, select_forward_between=1):
        # パス可能なロボットIDのリストを返す関数

        # 計算対象にする相手ロボットIDを格納するリスト
        target_their_robot_id_list = []
        # 計算上の相手ロボットの半径（通常の倍の半径（直径）に設定）
        robot_r = 0.4
        # 前方にいる相手ロボットの数と比較する用の変数
        check_count = 0
        # ロボットの位置座標取得から実際にパスを出すまでの想定時間
        dt = 0.5
        # 相手ロボットのゴーリーのx座標
        # their_goalie_x = 0.0
        # # 相手ロボットのゴーリーのy座標
        # their_goalie_y = 0.0
        # # 相手ゴーリーのID
        # their_goalie_id = 99

        # 味方ロボットの位置と速度を取得
        our_robot_id_list = copy.deepcopy(self.our_robot_id_list)
        our_robots_pos = copy.deepcopy(self.our_robots_pos)
        our_robots_vel = copy.deepcopy(self.our_robots_vel)

        # 敵ロボットの位置と速度を取得
        their_robot_id_list = copy.deepcopy(self.their_robot_id_list)
        their_robots_pos = copy.deepcopy(self.their_robots_pos)
        their_robots_vel = copy.deepcopy(self.their_robots_vel)

        # パサーがフィールドにいない場合
        if my_robot_id not in our_robot_id_list:
            return []
        # パサーの位置が検出されていない場合
        my_robot_pos = our_robots_pos[my_robot_id]
        if my_robot_pos is None:
            return []

        # ID数と位置情報の数が不一致の場合
        if len(our_robot_id_list) != self.MAX_ROBOT_NUM - our_robots_pos.count(None):
            return []

        skip_my_id = True
        if need_shoot:
            # 計算上の相手ロボットの半径（通常の倍の半径（直径）に設定）
            # robot_r = 0.1 #meters
            # # ロボットの位置座標取得から実際にパスを出すまでの想定時間
            # dt = 0.0 #sec
            skip_my_id = False
            # ゴールのIDと位置
            # シュートのときは味方位置をゴールの位置と見立てる
            our_robot_id_list = copy.deepcopy(self.goal_id_list)
            our_robots_pos = copy.deepcopy(self.goal_pos_list)
            our_robots_vel = copy.deepcopy(self.goal_vel_list)

        # パサーよりも前にいる味方ロボットIDのリストを取得
        forward_our_robot_id = self._forward_robots_id(
            my_robot_id, my_robot_pos, our_robot_id_list, our_robots_pos,
            our_robots_vel, robot_r, dt, skip_my_id=skip_my_id)
        # パサーよりも前にいる敵ロボットIDリストを取得
        forward_their_robot_id = self._forward_robots_id(
            my_robot_id, my_robot_pos, their_robot_id_list, their_robots_pos,
            their_robots_vel, robot_r, dt)

        # パサーよりも前にいるロボットがいなければ空のリストを返す
        if len(forward_our_robot_id) == 0:
            return forward_our_robot_id

        # 自分と各ロボットまでの距離を基にロボットIDをソート
        our_robot_id_list = self._sort_by_from_robot_distance(
            my_robot_pos, forward_our_robot_id, our_robots_pos)

        # パス可能なロボットIDを格納するリスト
        robots_to_pass = []
        pass_robots_pos_list = []

        # 各レシーバー候補ロボットに対してパス可能か判定
        for our_robot_id in our_robot_id_list:
            our_robot_pos = our_robots_pos[our_robot_id]
            trans = tool.Trans(my_robot_pos, tool.get_angle(my_robot_pos, our_robot_pos))

            # パサーとレシーバー候補ロボットの間にいる相手ロボットを計算対象とするときの処理
            if select_forward_between == 1:
                target_their_robot_id_list = [
                    _id for _id in forward_their_robot_id
                    if our_robot_pos.x > their_robots_pos[_id].x]
            # パサーより前方にいる相手ロボットを計算対象とするときの処理
            else:
                target_their_robot_id_list = forward_their_robot_id

            # 相手ロボットが存在するときの処理
            if 0 < len(target_their_robot_id_list):
                # 対象となる相手ロボット全てに対してパスコースを妨げるような動きをしているか計算
                for their_robot_id in target_their_robot_id_list:
                    their_robot_pos = their_robots_pos[their_robot_id]
                    their_robot_pos_trans = trans.transform(their_robot_pos)
                    their_robot_vel = their_robots_vel[their_robot_id]

                    # TODO(ShotaAk): ソフト構造を変更し、問題の根本解決をすべき
                    if their_robot_vel is None:
                        continue

                    # 共有点を持つか判定
                    common_point = -1
                    if abs(their_robot_pos_trans.y) < \
                       robot_r + math.hypot(their_robot_vel.x, their_robot_vel.y) * dt:
                        common_point = 1

                    # 共有点を持たないときの処理
                    if common_point < 0:
                        # 対象としている相手ロボットすべてにパスコースが妨害されないときの処理
                        if check_count >= len(target_their_robot_id_list) - 1:
                            # 何台の相手ロボットに妨害されないかをカウントする変数をリセット
                            check_count = 0
                            # パスできる味方ロボットとしてリストに格納
                            robots_to_pass.append(our_robot_id)
                            pass_robots_pos_list.append(our_robot_pos)
                        # まだすべてのロボットに対して計算を行っていない場合の処理
                        else:
                            # 何台の相手ロボットに妨害されないかをカウントする変数をインクリメント
                            check_count += 1
                    # 共有点を持つときの処理
                    else:
                        # 何台の相手ロボットに妨害されないかをカウントする変数をリセット
                        check_count = 0
                        break

                    # if need_shoot and their_goalie_x < their_robot_pos_trans.x \
                    #    and -self._field_defense_half_y < their_goalie_y \
                    #    and their_goalie_y < self._field_defense_half_y:
                    #     their_goalie_id = their_robot_id
                    #     their_goalie_x = their_robot_pos_trans.x
                    #     their_goalie_y = their_robot_pos_trans.y
            # 計算対象とする相手ロボットが存在しないとき（邪魔する相手ロボットがいないとき）
            else:
                # パスができる味方ロボットとしてリストに格納
                robots_to_pass.append(our_robot_id)
                pass_robots_pos_list.append(our_robot_pos)

            # if their_goalie_id != 99 and their_robot_pos_trans.y > 0:
            #     robots_to_pass = sorted(robots_to_pass, reverse=True)

        return robots_to_pass, pass_robots_pos_list

    def _sort_by_from_robot_distance(self, my_robot_pos, robots_id, robots_pos):
        # 自ロボットから各ロボットまでの距離を計算し近い順にソートする関数

        # 自ロボットから各ロボットまでの距離、IDをリストに格納
        robots_dist_and_id = [
            [tool.get_distance(robots_pos[_id], my_robot_pos), _id] for _id in robots_id]

        # 距離が近い順にロボットIDをソート
        sorted_robots_id = [_id for _, _id in sorted(robots_dist_and_id)]

        return sorted_robots_id

    def _forward_robots_id(
            self, my_robot_id, my_robot_pos, robots_id, robots_pos,
            robots_vel, robot_r, dt, skip_my_id=False):

        # パサーより前に存在するロボットIDをリスト化
        # TODO: ここ、geometry_toolsのTrans使えばきれいに書けそう
        # TODO: x座標でしか評価されていないので、自チーム側へのパスに対応できない
        forward_robots_id = []
        for robot_id in robots_id:
            # 自身のIDをスキップする場合
            if skip_my_id and my_robot_id == robot_id:
                # 自身のIDをスキップ
                continue

            target_robot_pos = robots_pos[robot_id]
            target_robot_vel = robots_vel[robot_id]

            # TODO(ShotaAk): ソフト構造を変更し、問題の根本解決をすべき
            if target_robot_pos is None:
                continue

            estimated_displacement = 0.0
            vel_norm = 0.0
            if target_robot_vel is not None:
                vel_norm = math.hypot(target_robot_vel.x, target_robot_vel.y)

                # dt時間後の移動距離を計算
                if not math.isclose(vel_norm, 0.0, abs_tol=0.000001):  # ゼロ除算回避
                    estimated_displacement = (abs(target_robot_vel.x) *
                                              dt + robot_r) * target_robot_vel.x / vel_norm

            if my_robot_pos.x < target_robot_pos.x + estimated_displacement:
                forward_robots_id.append(robot_id)

        # forward_robots_id = [
        #     _id for _id in robots_id
        #     if my_robot_pos[0] < robots_pos[_id][0] \
        #       + (abs(robots_vel[_id][0]) * dt + robot_r) * \
        #       robots_vel[_id][0] / math.sqrt(robots_vel[_id][0] ** 2 \
        #       + robots_vel[_id][1] ** 2)]

        return forward_robots_id
