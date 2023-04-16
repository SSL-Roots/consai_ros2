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

import copy
from enum import Enum
import math

from rclpy.node import Node
from robocup_ssl_msgs.msg import Point
from robocup_ssl_msgs.msg import Referee
from robocup_ssl_msgs.msg import RobotId
from robocup_ssl_msgs.msg import TrackedBall
from robocup_ssl_msgs.msg import TrackedFrame
from robocup_ssl_msgs.msg import TrackedRobot
from robocup_ssl_msgs.msg import Vector3

class RoleName(Enum):
    GOALIE = 0
    ATTACKER = 1
    CENTER_BACK1 = 2
    CENTER_BACK2 = 3
    SUB_ATTACKER = 4
    ZONE1 = 5
    ZONE2 = 6
    ZONE3 = 7
    ZONE4 = 8
    SIDE_BACK1 = 9
    SIDE_BACK2 = 10


# フィールド状況を見て、ロボットの役割を決めるノード
# ロボットの役割が頻繁に変わらないように調整する
class RoleAssignment(Node):
    # フィールドに出せるロボットの数は11台
    # Ref: https://robocup-ssl.github.io/ssl-rules/sslrules.html#_number_of_robots
    ROLE_NUM = 11
    # ロボット、ボールの消失判定しきい値
    VISIBILITY_THRESHOLD = 0.01

    def __init__(self, goalie_id, our_team_is_yellow=False):
        super().__init__('assignor')

        self._our_team_is_yellow = our_team_is_yellow

        if self._our_team_is_yellow:
            self.get_logger().info('ourteamはyellowです')
        else:
            self.get_logger().info('ourteamはblueです')

        self._role = [None] * self.ROLE_NUM
        self._goalie_id = goalie_id
        self.get_logger().info('goalie IDは{}です'.format(self._goalie_id))

        self._detection = TrackedFrame()
        self._sub_detection_tracked = self.create_subscription(
            TrackedFrame, 'detection_tracked', self._detection_tracked_callback, 10)

    def get_robot_id(self, role_index):
        # 指定された役割を担当するロボットIDを返す
        if role_index < len(self._role):
            return self._role[role_index]
        else:
            return None

    def update_role(self):
        # 役割リストを更新する
        # 担当が変わった役割のインデックスのリストを返す
        # 更新前後でリストに変化がなければ空のリストを返す
        prev_role_list = copy.deepcopy(self._role)
        self._update_role_list()
        changed_index = []
        for i in range(len(self._role)):
            if self._role[i] is not None and self._role[i] != prev_role_list[i]:
                changed_index.append(i)

        return changed_index

    def get_active_roles(self):
        # IDが割り当てられているroleのリストを返す
        active_index = []
        for i in range(len(self._role)):
            if self._role[i] is not None:
                active_index.append(i)
        return active_index

    def _detection_tracked_callback(self, msg):
        self._detection = msg

    def _update_role_list(self):
        # 役割リストを更新する
        our_active_robots, their_active_robots = self._extract_active_robots(self._detection.robots)
        our_active_ids = [robot.robot_id.id for robot in our_active_robots]

        self._reset_inactive_id(our_active_ids)    
        self._set_avtive_id_to_empty_role(our_active_ids)
        self._sort_empty_role()

        # ボールが存在する場合、ボール情報を用いて役割を変更する
        if len(self._detection.balls) == 0:
            return
        ball = self._detection.balls[0]

        # visibilityがセットされていなければ消失と判定する
        if len(ball.visibility) == 0:
            return
        
        if ball.visibility[0] < self.VISIBILITY_THRESHOLD:
            return

        next_attacker_id = self._determine_next_attacker_id(our_active_robots, ball.pos)
        # アタッカーのIDを役割リストにセットする
        if next_attacker_id in self._role:
            next_attacker_index = self._role.index(next_attacker_id)
            self._swap_role(RoleName.ATTACKER.value, next_attacker_index)

    def _reset_inactive_id(self, our_active_ids):
        # 役割リストにあるIDが非アクティブな場合、リストの枠にNoneをセットする
        for index, robot_id in enumerate(self._role):
            if robot_id is None:
                continue

            if robot_id not in our_active_ids:
                self._role[index] = None

    def _set_avtive_id_to_empty_role(self, our_active_ids):
        # アクティブなIDを役割リストの空きスペース(None)にセットする

        for active_id in our_active_ids:
            # 空きスペースが無くなれば終了
            if None not in self._role:
                break

            # すでに役割が与えられていたらスキップする
            if active_id in self._role:
                continue

            # アクティブなIDがgoalieであれば、指定されたスペースにセットする
            if active_id == self._goalie_id:
                self._role[RoleName.GOALIE.value] = self._goalie_id
                continue

            # 空きスペースにIDをセットする
            # ただし、goalieのスペースは空けておく
            for index, robot_id in enumerate(self._role):
                if index == RoleName.GOALIE.value:
                    continue

                if robot_id is None:
                    self._role[index] = active_id
                    break

    def _sort_empty_role(self):
        # 役割リストのIDの並び途中にNoneが来ないように、
        # 一番後半のIDをNoneにセットしていく

        # 空きスペースがなければ終了
        if None not in self._role:
            return

        for index, robot_id in enumerate(self._role):
            # goalieのスペースは空けておく
            if index == RoleName.GOALIE.value:
                continue

            if robot_id is None:
                last_active_index = self._last_active_role_index()
                # アクティブな役割がなければ終了
                if last_active_index is None:
                    break
                # 空きスペースの後にアクティブなIDがあればスワップする
                if index < last_active_index:
                    self._swap_role(index, last_active_index)
        
    def _last_active_role_index(self):
        # 役割リストを逆順に検索し、アクティブなroleのindexを返す
        for robot_id in reversed(self._role):
            if robot_id is not None:
                return self._role.index(robot_id)
        return None
            
    def _swap_role(self, index1, index2):
        # 役割リストの指定されたindexをスワップする
        self._role[index1], self._role[index2] = self._role[index2], self._role[index1]

    def _extract_active_robots(self, robots_msg):
        # TrackedFrame.robots からアクティブなロボットを抽出する
        our_active_robots = []
        their_active_robots = []
        for robot in robots_msg:
            # visibilityがなければアクティブではない
            if len(robot.visibility) == 0:
                continue

            # visibilityが小さければ、フィールドに存在しないと判断
            if robot.visibility[0] < self.VISIBILITY_THRESHOLD:
                continue

            # ロボットIDを自チームと相手チームに分ける
            is_yellow = robot.robot_id.team_color == RobotId.TEAM_COLOR_YELLOW
            is_blue = robot.robot_id.team_color == RobotId.TEAM_COLOR_BLUE
            if (self._our_team_is_yellow and is_yellow) or \
               (not self._our_team_is_yellow and is_blue):
                our_active_robots.append(robot)
            else:
                their_active_robots.append(robot)

        return our_active_robots, their_active_robots

    def _determine_next_attacker_id(self, our_active_robots, ball_pos):
        # ボールに最も近いロボットをアタッカー候補として出力する
        # アタッカーIDが頻繁に変わらないヒステリシスをもたせる
        NEAREST_THRESHOLD = 0.5  # meter

        nearest_id = None
        nearest_distance = 1000  # 適当な巨大な距離を初期値とする
        attacker_id = None
        attacker_distance = 1000  # 適当な巨大な距離を初期値とする
        for robot in our_active_robots:
            distance_x = ball_pos.x - robot.pos.x
            distance_y = ball_pos.y - robot.pos.y
            distance = math.hypot(distance_x, distance_y)

            # アタッカーの距離とIDをセット
            # NoneなIDを検出しない
            if self._role[RoleName.ATTACKER.value] and \
               robot.robot_id.id == self._role[RoleName.ATTACKER.value]:
                attacker_id = robot.robot_id.id
                attacker_distance = distance
                continue

            # Goalieはスキップ
            if robot.robot_id.id == self._goalie_id:
                continue

            # 最もボールに近いロボットの距離とIDを更新
            if distance < nearest_distance:
                nearest_distance = distance
                nearest_id = robot.robot_id.id

        # アタッカー以外のロボットが十分にボールに近ければ、
        # アタッカー候補としてIDを返す
        if attacker_distance - nearest_distance > NEAREST_THRESHOLD:
            return nearest_id
        else:
            return attacker_id
