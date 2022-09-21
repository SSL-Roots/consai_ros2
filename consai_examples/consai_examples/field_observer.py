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
from turtle import pos

from rclpy.node import Node
from robocup_ssl_msgs.msg import TrackedBall
from robocup_ssl_msgs.msg import TrackedFrame
from robocup_ssl_msgs.msg import RobotId

from robocup_ssl_msgs.msg import DetectionRobot
from robocup_ssl_msgs.msg import TrackedRobot

# フィールド状況を観察し、ボールの位置を判断したり
# ロボットに一番近いロボットを判定する


class FieldObserver(Node):
    BALL_NONE = 0
    BALL_IS_OUTSIDE = 1
    BALL_IS_IN_OUR_DEFENSE_AREA = 2
    BALL_IS_IN_OUR_SIDE = 3
    BALL_IS_IN_THEIR_SIDE = 4
    BALL_IS_IN_THEIR_DEFENSE_AREA = 5

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
        self._enemy_robot = TrackedRobot()
        # self.robots = []
        self.robots = TrackedRobot()

        # self.our_robots = TrackedRobot()

        # ------------------------------------------------------------------------------

        self.our_robots_pos = []
        self.our_robots_angle = []
        self.our_robots_vel = []
        self.our_robots_speed = []

        self.enemy_robots_pos = []
        self.enemy_robots_angle = []
        self.enemy_robots_vel = []
        self.enemy_robots_speed = []

        # ------------------------------------------------------------------------------

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

    def _detection_tracked_callback(self, msg):
        self._detection = msg
        if len(msg.balls) > 0:
            self._update_ball_state(msg.balls[0])
            self._update_ball_moving_state(msg.balls[0])
            self._update_ball_zone_state(msg.balls[0].pos)
            self._ball = msg.balls[0]
        if len(msg.robots) > 0:
            self.robots = msg.robots
            if self._our_team_is_yellow:
                self._update_our_robots_pos(
                    [msg.robots[our_robot] for our_robot in range(16, 32)])
                self._update_enemy_robots_pos(
                    [msg.robots[enemy_robot] for enemy_robot in range(16)])
                self._update_our_robots_vel(
                    [msg.robots[our_robot] for our_robot in range(16, 32)])
                self._update_enemy_robots_vel(
                    [msg.robots[enemy_robot] for enemy_robot in range(16)])
                self._our_robot = [msg.robots[robot]
                                   for robot in range(16, 32)]
                self._enemy_robot = [msg.robots[robot] for robot in range(16)]
            elif not self._our_team_is_yellow:
                self._update_our_robots_pos(
                    [msg.robots[our_robot] for our_robot in range(16)])
                self._update_enemy_robots_pos(
                    [msg.robots[enemy_robot] for enemy_robot in range(16, 32)])
                self._update_our_robots_vel(
                    [msg.robots[our_robot] for our_robot in range(16)])
                self._update_enemy_robots_vel(
                    [msg.robots[enemy_robot] for enemy_robot in range(16, 32)])
                self._our_robot = [msg.robots[robot] for robot in range(16)]
                self._enemy_robot = [msg.robots[robot]
                                     for robot in range(16, 32)]

    def update_robot_state(self):
        # robot_state = []
        # robot_state.append([self._robot.x, self._robot.y])
        # return robot_state

        # robot_id = []
        # for robot in self._robot.robot_id:
        #     robot_id.append(robot)
        # return robot_id
        return self.robots

        # robocup_ssl_msgs.msg.TrackedRobot(robot_id=robocup_ssl_msgs.msg.RobotId(id=15, team_color=1),
        # pos=robocup_ssl_msgs.msg.Vector2(x=0.0, y=0.0), orientation=0.0,
        # vel=[robocup_ssl_msgs.msg.Vector2(x=0.0, y=0.0)], vel_angular=[0.0], visibility=[0.0])

    # 味方ロボットの位置座標 [x,y] 取得
    def _update_our_robots_pos(self, our_robots):
        self.our_robots_pos = [
            "None" if our_robots[robot].visibility[0] <= 0.2 else [our_robots[robot].pos.x, our_robots[robot].pos.y] for robot in range(len(our_robots))]

    def get_our_robots_pos(self):
        return self.our_robots_pos

    def get_our_robot_pos(self, our_robot_id):
        return self.our_robots_pos[our_robot_id]

    # 相手ロボットの位置座標 [x,y] 取得
    def _update_enemy_robots_pos(self, enemy_robots):
        self.enemy_robots_pos = [
            "None" if enemy_robots[robot].visibility[0] <= 0.2 else [enemy_robots[robot].pos.x, enemy_robots[robot].pos.y] for robot in range(len(enemy_robots))]

    def get_enemy_robots_pos(self):
        return self.enemy_robots_pos

    def get_enemy_robot_pos(self, enemy_robot_id):
        return self.enemy_robots_pos[enemy_robot_id]

    # 味方ロボットの速度 [x,y] 取得
    def _update_our_robots_vel(self, our_robots):
        self.our_robots_vel = [
            our_robots[robot].vel for robot in range(len(our_robots))]
        self.our_robots_vel = sum(self.our_robots_vel, [])
        self.our_robots_vel = ["None" if our_robots[robot].visibility[0] <= 0.2 else [
            self.our_robots_vel[robot].x, self.our_robots_vel[robot].y] for robot in range(len(self.our_robots_vel))]

    def get_our_robots_vel(self):
        return self.our_robots_vel

    # def get_our_robots_speed(self):
    #     self.our_robots_speed = [math.sqrt()]

    # 相手ロボットの速度 [x,y] 取得

    def _update_enemy_robots_vel(self, enemy_robots):
        self.enemy_robots_vel = [
            enemy_robots[robot].vel for robot in range(len(enemy_robots))]
        self.enemy_robots_vel = sum(self.enemy_robots_vel, [])
        self.enemy_robots_vel = ["None" if enemy_robots[robot].visibility[0] <= 0.2 else [
            self.enemy_robots_vel[robot].x, self.enemy_robots_vel[robot].y] for robot in range(len(self.enemy_robots_vel))]

    def get_enemy_robots_vel(self):
        return self.enemy_robots_vel

    def update_our_robots_thita(self):
        our_robots_thita = [
            self._our_robot[robot].orientation for robot in range(len(self._our_robot))]
        return our_robots_thita

    # 味方ロボットの角速度取得
    def update_our_robots_vel_angle(self):
        our_robots_vel_angle = [
            self._our_robot[robot].vel_angular[0] for robot in range(len(self._our_robot))]
        return our_robots_vel_angle

    def _update_ball_state(self, ball):
        # フィールド場外判定
        if self._check_is_ball_outside(ball.pos):
            self._ball_state = self.BALL_IS_OUTSIDE
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

    def _check_is_ball_outside(self, ball_pos):
        # ボールがフィールド外に出たか判定
        threshold_x = self._field_half_x
        threshold_y = self._field_half_y
        if self.ball_is_outside():
            threshold_x -= self.THRESHOLD_MARGIN
            threshold_y -= self.THRESHOLD_MARGIN

        if math.fabs(ball_pos.x) > threshold_x or math.fabs(ball_pos.y) > threshold_y:
            return True
        return False

    def _check_is_ball_in_defense_area(self, ball_pos, our_area=True):
        # ボールがディフェンスエリアに入ったか判定
        threshold_x = self._field_half_x - self._field_defense_x
        threshold_y = self._field_defense_half_y
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
            target_is_blue = self._our_team_is_yellow and robot.robot_id.team_color == RobotId.TEAM_COLOR_BLUE
            target_is_yellow = not self._our_team_is_yellow and robot.robot_id.team_color == RobotId.TEAM_COLOR_YELLOW
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

    def ball_is_outside(self):
        return self._ball_state == self.BALL_IS_OUTSIDE

    def ball_is_in_our_defense_area(self):
        return self._ball_state == self.BALL_IS_IN_OUR_DEFENSE_AREA

    def ball_is_in_their_defense_area(self):
        return self._ball_state == self.BALL_IS_IN_THEIR_DEFENSE_AREA

    def ball_is_in_our_side(self):
        return self._ball_state == self.BALL_IS_IN_OUR_SIDE

    def ball_is_moving(self):
        return self._ball_is_moving

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

    def get_ball_pos(self):
        return self._ball
