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

# 設定可能なロボット役割一覧
# ロボットの台数より多く定義してもOK
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
    SUBSTITUTE = 11


# フィールド状況を見て、ロボットの役割を決めるノード
# ロボットの役割が頻繁に変わらないように調整する
class RoleAssignment(Node):
    # ロボット、ボールの消失判定しきい値
    VISIBILITY_THRESHOLD = 0.01

    def __init__(self, goalie_id, our_team_is_yellow=False):
        super().__init__('assignor')

        self._our_team_is_yellow = our_team_is_yellow

        if self._our_team_is_yellow:
            self.get_logger().info('ourteamはyellowです')
        else:
            self.get_logger().info('ourteamはblueです')
        
        # フィールド上のロボット役割一覧
        # フィールドに出せるロボットの数は11台
        # Ref: https://robocup-ssl.github.io/ssl-rules/sslrules.html#_number_of_robots
        self._active_role_list = [
            RoleName.GOALIE,
            RoleName.ATTACKER,
            RoleName.CENTER_BACK1,
            RoleName.CENTER_BACK2,
            RoleName.SUB_ATTACKER,
            RoleName.ZONE1,
            RoleName.ZONE2,
            RoleName.ZONE3,
            RoleName.ZONE4,
            RoleName.SIDE_BACK1,
            RoleName.SIDE_BACK2,
        ]
        # 実際に運用するroleのリスト
        # イエローカードや交代指示などで役割が変更されます
        self._present_role_list = copy.deepcopy(self._active_role_list)
        self._ROLE_NUM = len(self._active_role_list)

        # role優先度とロボットIDを結びつけるリスト
        # indexの数値が小さいほど優先度が高い
        self._robot_id_of_role_priority = [None] * self._ROLE_NUM

        self._goalie_id = goalie_id
        self.get_logger().info('goalie IDは{}です'.format(self._goalie_id))

        self._detection = TrackedFrame()
        self._sub_detection_tracked = self.create_subscription(
            TrackedFrame, 'detection_tracked', self._detection_tracked_callback, 10)

    def update_role(self, update_attacker_by_ball_pos=True, allowed_robot_num=11):
        # 役割リストを更新する
        # 担当が変わった役割のリストを返す
        # 更新前後でリストに変化がなければ空のリストを返す
        prev_robot_id_list = copy.deepcopy(self._robot_id_of_role_priority)

        self._update_role_list(update_attacker_by_ball_pos)

        changed_roles = []
        for i in range(len(self._robot_id_of_role_priority)):
            if self._robot_id_of_role_priority[i] is not None \
               and self._robot_id_of_role_priority[i] != prev_robot_id_list[i]:

                changed_roles.append(self._active_role_list[i])

        # イエローカードなどでallowed_robot_numが減った場合、SUBSTITUTEを割り当てる
        self._present_role_list = copy.deepcopy(self._active_role_list)
        num_of_substitute = self._ROLE_NUM - allowed_robot_num
        if num_of_substitute > 0:
            self._overwrite_substite_to_present_role_list(num_of_substitute)
            changed_roles.append(RoleName.SUBSTITUTE)

        return changed_roles

    def get_assigned_roles(self):
        # IDが割り当てられているroleのリストを返す
        active_index = []
        for i in range(len(self._robot_id_of_role_priority)):
            if self._robot_id_of_role_priority[i] is not None:
                active_index.append(i)
        return active_index

    def get_assigned_roles_and_ids(self):
        # IDが割り当てられているroleとIDのペアのリストを返す
        active_index = []
        role_and_id = []
        for i in range(len(self._robot_id_of_role_priority)):
            role = self._present_role_list[i]
            robot_id = self._robot_id_of_role_priority[i]
            if robot_id is not None:
                role_and_id.append((role, robot_id))
        return role_and_id

    def get_role_from_robot_id(self, robot_id):
        # 指定したIDが割り当てられているroleを返す
        # 割り当てていなければNoneを返す
        if robot_id in self._robot_id_of_role_priority:
            role_index = self._robot_id_of_role_priority.index(robot_id)
            return self._present_role_list[role_index]
        return None

    def _detection_tracked_callback(self, msg):
        self._detection = msg

    def _get_priority_of_role(self, role):
        # roleの優先度を返す
        # 重複するroleが設定されている場合、高いほうの優先度を返す
        return self._active_role_list.index(role)

    def _update_role_list(self, update_attacker_by_ball_pos=True):
        # 役割リストを更新する
        our_active_robots, their_active_robots = self._extract_active_robots(self._detection.robots)
        our_active_ids = [robot.robot_id.id for robot in our_active_robots]

        self._reset_inactive_id(our_active_ids)
        self._set_avtive_id_to_empty_role(our_active_ids)
        self._sort_empty_role()

        # アタッカーを更新しない場合は終了
        if update_attacker_by_ball_pos is False:
            return

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
        if next_attacker_id in self._robot_id_of_role_priority:
            next_attacker_priority = self._robot_id_of_role_priority.index(next_attacker_id)
            attacker_role_priority = self._get_priority_of_role(RoleName.ATTACKER)
            self._swap_robot_id_of_priority(next_attacker_priority, attacker_role_priority)

    def _reset_inactive_id(self, our_active_ids):
        # 役割リストにあるIDが非アクティブな場合、リストの枠にNoneをセットする
        for index, robot_id in enumerate(self._robot_id_of_role_priority):
            if robot_id is None:
                continue

            if robot_id not in our_active_ids:
                self._robot_id_of_role_priority[index] = None

    def _set_avtive_id_to_empty_role(self, our_active_ids):
        # アクティブなIDを役割リストの空きスペース(None)にセットする

        for active_id in our_active_ids:
            # 空きスペースが無くなれば終了
            if None not in self._robot_id_of_role_priority:
                break

            # すでに役割が与えられていたらスキップする
            if active_id in self._robot_id_of_role_priority:
                continue

            # アクティブなIDがgoalieであれば、指定されたスペースにセットする
            if active_id == self._goalie_id:
                goalie_priority = self._get_priority_of_role(RoleName.GOALIE)
                self._robot_id_of_role_priority[goalie_priority] = self._goalie_id
                continue

            # 空きスペースにIDをセットする
            # ただし、goalieのスペースは空けておく
            priority = self._find_highest_prioiry_from_free_roles(ignore_goalie=True)
            if priority is not None:
                self._robot_id_of_role_priority[priority] = active_id

    def _sort_empty_role(self):
        # 優先度の高いroleに空きが出ないように、
        # 一番優先度の低いroleの担当を、優先度の高い空きroleに割り当てる

        # 空いているroleがなければ終了
        if None not in self._robot_id_of_role_priority:
            return

        for priority, robot_id in enumerate(self._robot_id_of_role_priority):
            # goalieの担当は空けておく
            if priority == self._get_priority_of_role(RoleName.GOALIE):
                continue

            # 担当がいればスキップ
            if robot_id is not None:
                continue

            lowest_priority = self._find_lowest_prioiry_from_assigned_roles()
            if lowest_priority is None:
                break

            # 空きスペースの後にアクティブなIDがあればスワップする
            if priority < lowest_priority:
                self._swap_robot_id_of_priority(priority, lowest_priority)
        
    def _find_lowest_prioiry_from_assigned_roles(self):
        # 担当がいるroleの中で最も優先度が低いものを探し、優先度を返す
        # 担当が一人もいなければNoneを返す
        for robot_id in reversed(self._robot_id_of_role_priority):
            if robot_id is not None:
                return self._robot_id_of_role_priority.index(robot_id)
        return None

    def _find_highest_prioiry_from_free_roles(self, ignore_goalie=True):
        # 担当がいないroleの中で、最も優先度が高いものを探し、優先度を返す
        # 空きroleがなければNoneを返す
        # ignore_goalieがTrueの場合、goalieのroleを除外する
        for priority, robot_id in enumerate(self._robot_id_of_role_priority):
            if priority == self._get_priority_of_role(RoleName.GOALIE) and ignore_goalie:
                continue

            if robot_id is None:
                return priority
        return None
            
    def _swap_robot_id_of_priority(self, priority1, priority2):
        # robot_id_of_priorityの指定されたpriorityの要素を入れ替える
        self._robot_id_of_role_priority[priority1], self._robot_id_of_role_priority[priority2] \
            = self._robot_id_of_role_priority[priority2], self._robot_id_of_role_priority[priority1]

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
        present_attacker_distance = 1000  # 適当な巨大な距離を初期値とする

        attacker_role_priority = self._get_priority_of_role(RoleName.ATTACKER)
        present_attacker_id = self._robot_id_of_role_priority[attacker_role_priority]
        for robot in our_active_robots:
            target_id = robot.robot_id.id
            distance_x = ball_pos.x - robot.pos.x
            distance_y = ball_pos.y - robot.pos.y
            distance = math.hypot(distance_x, distance_y)

            # 現在のアタッカーの距離をセット
            if target_id == present_attacker_id:
                present_attacker_distance = distance
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
        if present_attacker_distance - nearest_distance > NEAREST_THRESHOLD:
            return nearest_id
        else:
            return present_attacker_id

    def _overwrite_substite_to_present_role_list(self, num_of_substitute):
        # 指定した数だけ、presetn_role_listの後ろからSUBSTITUEロールを上書きする
        if num_of_substitute <= 0:
            return

        self._present_role_list[-num_of_substitute:] = \
            [RoleName.SUBSTITUTE] * min(num_of_substitute, len(self._present_role_list))
        