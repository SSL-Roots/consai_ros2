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
        self._their_robot = TrackedRobot()
        self.robots = TrackedRobot()

        self.our_robots_pos = []
        self.our_robots_angle = []
        self.our_robots_vel = []
        self.our_robots_speed = []
        self.our_robots_vel_angle = []

        self.their_robots_pos = []
        self.their_robots_angle = []
        self.their_robots_vel = []
        self.their_robots_speed = []
        self.their_robots_vel_angle = []

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
                self._update_our_robots_pos([msg.robots[our_robot] for our_robot in range(16, 32)])
                self._update_their_robots_pos([msg.robots[their_robot] for their_robot in range(16)])
                self._update_our_robots_vel([msg.robots[our_robot] for our_robot in range(16, 32)])
                self._update_their_robots_vel([msg.robots[their_robot] for their_robot in range(16)])
                self._update_our_robots_thita([msg.robots[our_robot] for our_robot in range(16, 32)])
                self._update_their_robots_thita([msg.robots[their_robot] for their_robot in range(16)])
                self._update_our_robots_vel_angle([msg.robots[our_robot] for our_robot in range(16, 32)])
                self._update_their_robots_vel_angle([msg.robots[their_robot] for their_robot in range(16)])
                self._our_robot = [msg.robots[robot] for robot in range(16, 32)]
                self._their_robot = [msg.robots[robot] for robot in range(16)]
            elif not self._our_team_is_yellow:
                self._update_our_robots_pos([msg.robots[our_robot] for our_robot in range(16)])
                self._update_their_robots_pos([msg.robots[their_robot] for their_robot in range(16, 32)])
                self._update_our_robots_vel([msg.robots[our_robot] for our_robot in range(16)])
                self._update_their_robots_vel([msg.robots[their_robot] for their_robot in range(16, 32)])
                self._update_our_robots_thita([msg.robots[our_robot] for our_robot in range(16)])
                self._update_their_robots_thita([msg.robots[their_robot] for their_robot in range(16, 32)])
                self._update_our_robots_vel_angle([msg.robots[our_robot] for our_robot in range(16)])
                self._update_their_robots_vel_angle([msg.robots[their_robot] for their_robot in range(16, 32)])
                self._our_robot = [msg.robots[robot] for robot in range(16)]
                self._their_robot = [msg.robots[robot] for robot in range(16, 32)]

    def update_robot_state(self):
        return self.robots
    
    # 味方ロボットの位置座標 [x,y] 取得
    def _update_our_robots_pos(self, our_robots):
        self.our_robots_pos = [
            "None" if our_robots[robot].visibility[0] <= 0.2 else [our_robots[robot].pos.x, our_robots[robot].pos.y] for robot in range(len(our_robots))]

    def get_our_robots_pos(self):
        return self.our_robots_pos

    def get_our_robot_pos(self, our_robot_id):
        return self.our_robots_pos[our_robot_id]

    # 相手ロボットの位置座標 [x,y] 取得
    def _update_their_robots_pos(self, their_robots):
        self.their_robots_pos = [
            "None" if their_robots[robot].visibility[0] <= 0.2 else [their_robots[robot].pos.x, their_robots[robot].pos.y] for robot in range(len(their_robots))]

    def get_their_robots_pos(self):
        return self.their_robots_pos

    def get_their_robot_pos(self, their_robot_id):
        return self.their_robots_pos[their_robot_id]

    # 味方ロボットの速度 [x,y] 取得
    def _update_our_robots_vel(self, our_robots):
        self.our_robots_vel = [our_robots[robot].vel for robot in range(len(our_robots))]
        self.our_robots_vel = sum(self.our_robots_vel, [])
        self.our_robots_vel = ["None" if our_robots[robot].visibility[0] <= 0.2 else [self.our_robots_vel[robot].x, self.our_robots_vel[robot].y] for robot in range(len(self.our_robots_vel))]

    def get_our_robots_vel(self):
        return self.our_robots_vel

    # 相手ロボットの速度 [x,y] 取得
    def _update_their_robots_vel(self, their_robots):
        self.their_robots_vel = [their_robots[robot].vel for robot in range(len(their_robots))]
        self.their_robots_vel = sum(self.their_robots_vel, [])
        self.their_robots_vel = ["None" if their_robots[robot].visibility[0] <= 0.2 else [self.their_robots_vel[robot].x, self.their_robots_vel[robot].y] for robot in range(len(self.their_robots_vel))]

    def get_their_robots_vel(self):
        return self.their_robots_vel

    # 味方ロボットの向いている角度（rad）取得
    def _update_our_robots_thita(self, our_robots):
        self.our_robots_angle = [our_robots[robot].orientation for robot in range(len(our_robots))]
    
    def get_our_robots_thita(self):
        return self.our_robots_angle
    
    # 相手ロボットの向いている角度（rad）取得
    def _update_their_robots_thita(self, their_robots):
        self.our_robots_angle = [their_robots[robot].orientation for robot in range(len(their_robots))]
    
    def get_their_robots_thita(self):
        return self.their_robots_angle

    # 味方ロボットの角速度（rad/s）取得
    def _update_our_robots_vel_angle(self, our_robots):
        self.our_robots_vel_angle = [our_robots[robot].vel_angular[0] for robot in range(len(our_robots))]

    def get_our_robots_vel_angle(self):
        return self.our_robots_vel_angle
    
    # 相手ロボットの角速度（rad/s）取得
    def _update_their_robots_vel_angle(self, their_robots):
        self.their_robots_vel_angle = [their_robots[robot].vel_angular[0] for robot in range(len(their_robots))]

    def get_their_robots_vel_angle(self):
        return self.their_robots_vel_angle

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
        ball_pos = [self._ball.pos.x, self._ball.pos.y]
        return ball_pos

    def get_receiver_robots_id(self, my_robot_id, select_forward_between=1):
        # パス可能なロボットIDのリストを返す関数

        # パス可能なロボットIDを格納するリスト
        robots_to_pass = []
        # 計算対象にする相手ロボットIDを格納するリスト
        target_their_robots_id = []
        # 計算上の相手ロボットの半径（通常の倍の半径（直径）に設定）
        robot_r = 0.4
        # 前方にいる相手ロボットの数と比較する用の変数
        check_count = 0
        # ロボットの位置座標取得から実際にパスを出すまでの想定時間
        dt = 0.5

        # 各ロボットの位置と速度を取得
        our_robots_pos = self.our_robots_pos
        their_robots_pos = self.their_robots_pos
        our_robots_vel = self.our_robots_vel
        their_robots_vel = self.their_robots_vel

        # エラー対策
        # TODO: 時々Vector2形式でデータが混入するので対策が必要
        if type(our_robots_vel[0]) is not list:
            return []

        # パサーよりも前にいる味方ロボットIDをリストにまとめる
        forward_our_robots_id = self._forward_our_robots_id(my_robot_id, our_robots_pos, our_robots_vel, robot_r, dt)
        # パサーよりも前にいる敵ロボットIDをリストにまとめる
        forward_their_robots_id = self._forward_their_robots_id(my_robot_id, our_robots_pos, their_robots_pos, their_robots_vel, robot_r, dt)

        # パサーよりも前にいるロボットがいなければ空のリストを返す
        if len(forward_our_robots_id) == 0:
            return forward_our_robots_id

        # 自分と各ロボットまでの距離を計測し近い順にソート
        dist_our_robot = self._sort_passer_from_our_robot_distance(my_robot_id, forward_our_robots_id, our_robots_pos)

        # ロボットの位置と長半径（移動距離）をロボットごとに格納
        their_robot_state = [[their_robots_pos[i][0], their_robots_pos[i][1], dt * abs(their_robots_vel[i][0]) + robot_r] for i in forward_their_robots_id]

        # 各レシーバー候補ロボットに対してパス可能か判定
        for i in range(len(dist_our_robot)):
            # パサーと味方ロボットの位置の差分
            dx = our_robots_pos[dist_our_robot[i][1]][0] - our_robots_pos[my_robot_id][0]
            dy = our_robots_pos[dist_our_robot[i][1]][1] - our_robots_pos[my_robot_id][1]

            # パサーとレシーバー候補ロボットを結ぶ直線の傾き
            if dx == 0:
                slope_from_passer_to_our_robot = dy
            else:
                slope_from_passer_to_our_robot = dy / dx

            # パサーとレシーバー候補となる味方ロボットを結ぶ直線の切片
            intercept_from_passer_to_our_robot = our_robots_pos[my_robot_id][1] - slope_from_passer_to_our_robot * our_robots_pos[my_robot_id][0]

            # パサーとレシーバー候補ロボットの間にいる相手ロボットを計算対象とするときの処理
            if select_forward_between == 1:
                target_their_robots_id = [robot_id for robot_id in forward_their_robots_id if our_robots_pos[dist_our_robot[i][1]][0] > their_robots_pos[robot_id][0]]
            # パサーより前方にいる相手ロボットを計算対象とするときの処理
            else:
                target_their_robots_id = forward_their_robots_id

            # 相手ロボットが存在するときの処理
            if len(target_their_robots_id) != 0:
                # 対象となる相手ロボット全てに対してパスコースを妨げるような動きをしているか計算
                for robot_id in target_their_robots_id:
                    # 判別式を解くための変数
                    a_kai = slope_from_passer_to_our_robot ** 2 + 1
                    b_kai = 2 * ((intercept_from_passer_to_our_robot - their_robot_state[robot_id][1]) * slope_from_passer_to_our_robot - their_robot_state[robot_id][0])
                    c_kai = their_robot_state[robot_id][0] ** 2 + (intercept_from_passer_to_our_robot - their_robot_state[robot_id][1]) ** 2 - their_robot_state[robot_id][2] ** 2

                    # 共有点を持つか判定
                    common_point = b_kai ** 2 - 4 * a_kai * c_kai

                    # 共有点を持たないときの処理
                    if common_point < 0:
                        # 対象としている相手ロボットすべてにパスコースが妨害されないときの処理
                        if check_count >= len(target_their_robots_id) - 1:
                            # 何台の相手ロボットに妨害されないかをカウントする変数をリセット
                            check_count = 0
                            # パスできる味方ロボットとしてリストに格納
                            robots_to_pass.append(dist_our_robot[i][1])
                        # まだすべてのロボットに対して計算を行っていない場合の処理
                        else:
                            # 何台の相手ロボットに妨害されないかをカウントする変数をインクリメント
                            check_count += 1
                    # 共有点を持つときの処理
                    else:
                        # 何台の相手ロボットに妨害されないかをカウントする変数をリセット
                        check_count = 0
                        # どの相手ロボットがパスコースに影響していたか（1台のみ出力）
                        #     このときの味方ロボットはパスの候補から除外する
                        # print(dist_our_robot[i][1], "番の味方ロボットへのパスは", robot_id, "番の相手ロボットに防がれる可能性あり")
                        break
            # 計算対象とする相手ロボットが存在しないとき（邪魔する相手ロボットがいないとき）
            else:
                # パスができる味方ロボットとしてリストに格納
                robots_to_pass.append(dist_our_robot[i][1])
        
        return robots_to_pass

    def _sort_passer_from_our_robot_distance(self, my_robot_id, our_robot_id, our_robots_pos):
        # パサーからレシーバー候補となる味方ロボットまでの距離を計算し近い順にソートする関数

        # パサーからレシーバー候補となる味方ロボットまでの距離、レシーバー候補となる味方ロボットのIDをリストに格納
        diff_passer_to_our_robot = [[our_robots_pos[i][0] - our_robots_pos[my_robot_id][0], our_robots_pos[i][1] - our_robots_pos[my_robot_id][1]] for i in our_robot_id]
        receive_dist_our_robot = [[math.sqrt((diff_passer_to_our_robot[i][0]) ** 2 + (diff_passer_to_our_robot[i][1]) ** 2), our_robot_id[i]] for i in range(len(our_robot_id))]

        # パサーに近い順に味方ロボットをソート
        receive_dist_our_robot = sorted(receive_dist_our_robot)

        return receive_dist_our_robot

    def _get_robots_in_field(self, robots_pos):
        # フィールド上にいるロボットのIDのみリスト化
        robots_in_field = [robot_id for robot_id in range(len(robots_pos)) if robots_pos[robot_id] != "None"]
        return robots_in_field

    def _forward_our_robots_id(self, robot_id_has_ball, our_robots_pos, our_robots_vel, robot_r, dt):
        # フィールド上にいる味方ロボットのIDのみリスト化
        our_robots_in_field = self._get_robots_in_field(our_robots_pos)
        # パサーよりも前にいる味方ロボットのIDをリスト化
        forward_our_robots_id = [robot_id for robot_id in our_robots_in_field 
                                if robot_id != robot_id_has_ball and our_robots_pos[robot_id_has_ball][0] < our_robots_pos[robot_id][0] + (abs(our_robots_vel[robot_id][0]) * dt + robot_r) * 
                                our_robots_vel[robot_id][0] / math.sqrt(our_robots_vel[robot_id][0] ** 2 + our_robots_vel[robot_id][1] ** 2)]
        return forward_our_robots_id

    def _forward_their_robots_id(self, robot_id_has_ball, our_robots_pos, their_robots_pos, their_robots_vel, robot_r, dt):
        # フィールド上にいる相手ロボットのidのみリスト化
        their_robots_in_field = self._get_robots_in_field(their_robots_pos)
        # パサーよりも前にいる相手ロボットのIDをリスト化
        forward_their_robots_id = [robot_id for robot_id in their_robots_in_field 
                                  if our_robots_pos[robot_id_has_ball][0] < their_robots_pos[robot_id][0] + (abs(their_robots_vel[robot_id][0]) * dt + robot_r) * 
                                  their_robots_vel[robot_id][0] / math.sqrt(their_robots_vel[robot_id][0] ** 2 + their_robots_vel[robot_id][1] ** 2)]
        return forward_their_robots_id