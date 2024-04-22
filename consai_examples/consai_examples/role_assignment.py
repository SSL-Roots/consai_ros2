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

from consai_examples.observer.pos_vel import PosVel
from consai_examples.role_to_visualize_msg import to_visualize_msg
from consai_msgs.msg import State2D
from consai_tools.geometry import geometry_tools as tool
from consai_visualizer_msgs.msg import Objects

import math

from rclpy import qos
from rclpy.node import Node

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
    LEFT_WING = 11
    RIGHT_WING = 12
    SIDE_BACK1 = 13
    SIDE_BACK2 = 14
    SUBSTITUTE = 15


class BallState(Enum):
    NONE = 0
    STOP = 1
    MOVE = 2


# フィールド状況を見て、ロボットの役割を決めるノード
# ロボットの役割が頻繁に変わらないように調整する
class RoleAssignment(Node):
    # フィールド上のロボット役割一覧
    # フィールドに出せるロボットの数は11台
    # Ref: https://robocup-ssl.github.io/ssl-rules/sslrules.html#_number_of_robots
    ACTIVE_ROLE_LIST = [
        RoleName.GOALIE,
        RoleName.ATTACKER,
        RoleName.SUB_ATTACKER,
        RoleName.CENTER_BACK1,
        RoleName.CENTER_BACK2,
        RoleName.SIDE_BACK1,
        RoleName.SIDE_BACK2,
        RoleName.ZONE1,
        RoleName.ZONE2,
        RoleName.ZONE3,
        RoleName.ZONE4,
        # RoleName.LEFT_WING,
        # RoleName.RIGHT_WING,
    ]

    def __init__(self, goalie_id: int):
        super().__init__('assignor')
        # 実際に運用するroleのリスト
        # イエローカードや交代指示などで役割が変更されます
        self._present_role_list = copy.deepcopy(self.ACTIVE_ROLE_LIST)
        self._ROLE_NUM = len(self.ACTIVE_ROLE_LIST)

        self._present_active_ids: list[int] = []
        self._present_ball_state = BallState.NONE

        # role優先度とロボットIDを結びつけるリスト
        # indexの数値が小さいほど優先度が高い
        self._id_list_ordered_by_role_priority = [None] * self._ROLE_NUM

        self._goalie_id = goalie_id
        self.get_logger().info('goalie IDは{}です'.format(self._goalie_id))

        self._pub_visualizer_objects = self.create_publisher(
            Objects, 'visualizer_objects', qos.qos_profile_sensor_data)

    def update_role(
            self, ball: PosVel, our_robots: dict[int, PosVel],
            update_attacker_by_ball_pos=True, allowed_robot_num=11):
        # roleへのロボットの割り当てを更新する

        # 優先度が高い順にロボットIDを並べる
        prev_id_list_ordered_by_role_priority = copy.deepcopy(
            self._id_list_ordered_by_role_priority)
        if self._update_role_list(
              ball, our_robots, update_attacker_by_ball_pos):
            self.get_logger().info('Role Updated')

        # 役割リストを更新する
        prev_role_list = copy.deepcopy(self._present_role_list)
        self._present_role_list = copy.deepcopy(self.ACTIVE_ROLE_LIST)
        # SUBSTITUTEを更新する
        num_of_substitute = self._ROLE_NUM - allowed_robot_num
        if num_of_substitute > 0:
            self._overwrite_substite_to_present_role_list(num_of_substitute)

        # 担当が変わったロボットIDのリストを返す
        changed_robot_ids = []
        for priority, robot_id in enumerate(self._id_list_ordered_by_role_priority):
            # 役割にロボットIDがセットされていなければスキップ
            if robot_id is None:
                continue

            # 前回と同じroleを担当していればスキップ
            if robot_id in prev_id_list_ordered_by_role_priority:
                prev_priority = prev_id_list_ordered_by_role_priority.index(robot_id)
                if self._present_role_list[priority] == prev_role_list[prev_priority]:
                    continue

            # 変更リストにIDを追加する
            changed_robot_ids.append(robot_id)

        return changed_robot_ids

    def get_assigned_roles_and_ids(self):
        # IDが割り当てられているroleとIDのペアのリストを返す
        role_and_id = []
        for i in range(len(self._id_list_ordered_by_role_priority)):
            role = self._present_role_list[i]
            robot_id = self._id_list_ordered_by_role_priority[i]
            if robot_id is not None:
                role_and_id.append((role, robot_id))
        return role_and_id

    def get_role_from_robot_id(self, robot_id):
        # 指定したIDが割り当てられているroleを返す
        # 割り当てていなければNoneを返す
        if robot_id in self._id_list_ordered_by_role_priority:
            role_index = self._id_list_ordered_by_role_priority.index(robot_id)
            return self._present_role_list[role_index]
        return None

    def publish_role_for_visualizer(self, our_robots: dict[int, PosVel]):
        role_dict = {}
        for robot_id in self._id_list_ordered_by_role_priority:
            if robot_id is not None:
                role = self.get_role_from_robot_id(robot_id)
                role_dict[robot_id] = role.name
        self._pub_visualizer_objects.publish(to_visualize_msg(role_dict, our_robots))

    def _get_priority_of_role(self, role):
        # roleの優先度を返す
        # 重複するroleが設定されている場合、高いほうの優先度を返す
        return self.ACTIVE_ROLE_LIST.index(role)

    def _update_role_list(
            self, ball: PosVel, our_robots: dict[int, PosVel],
            update_attacker_by_ball_pos=True):
        # イベントが発生したらroleを更新する
        updated = False

        our_active_ids = [robot_id for robot_id in our_robots.keys()]

        # イベント：フィールド上のロボット台数が変わる
        if our_active_ids != self._present_active_ids:
            # 役割リストを更新する
            self._reset_inactive_id(our_active_ids)
            self._set_avtive_id_to_empty_role(our_active_ids)
            self._sort_empty_role()
            updated = True
        self._present_active_ids = copy.deepcopy(our_active_ids)

        # イベント：ボールの状況が変わったらアタッカーを更新する
        if self._update_ball_state(ball) and update_attacker_by_ball_pos:
            if self._present_ball_state == BallState.STOP:
                # ボールが止まっている場合は、位置情報をもとにアタッカーを更新する
                next_attacker_id = self._determine_attacker_via_ball_pos(our_robots, ball)
            elif self._present_ball_state == BallState.MOVE:
                # ボールが動いている場合は、ボールの軌道をもとにアタッカーを更新する
                next_attacker_id = self._determine_attacker_via_ball_motion(our_robots, ball)

            if next_attacker_id is not None \
               and next_attacker_id in self._id_list_ordered_by_role_priority:
                next_attacker_priority = self._id_list_ordered_by_role_priority.index(next_attacker_id)
                attacker_role_priority = self._get_priority_of_role(RoleName.ATTACKER)
                self._swap_robot_id_of_priority(next_attacker_priority, attacker_role_priority)
                updated = True

        return updated

    def _update_ball_state(self, ball: PosVel) -> bool:
        # ボール状態を更新する
        # 状態が変わったらTrueを返す
        MOVE_THRESHOLD = 2.0  # m/s
        ball_vel_norm = math.hypot(ball.vel().x, ball.vel().y)

        prev_state = copy.deepcopy(self._present_ball_state)
        if ball_vel_norm < MOVE_THRESHOLD:
            self._present_ball_state = BallState.STOP
        else:
            self._present_ball_state = BallState.MOVE
        return prev_state != self._present_ball_state

    def _reset_inactive_id(self, our_active_ids):
        # 役割リストにあるIDが非アクティブな場合、リストの枠にNoneをセットする
        for index, robot_id in enumerate(self._id_list_ordered_by_role_priority):
            if robot_id is None:
                continue

            if robot_id not in our_active_ids:
                self._id_list_ordered_by_role_priority[index] = None

    def _set_avtive_id_to_empty_role(self, our_active_ids):
        # アクティブなIDを役割リストの空きスペース(None)にセットする

        for active_id in our_active_ids:
            # 空きスペースが無くなれば終了
            if None not in self._id_list_ordered_by_role_priority:
                break

            # すでに役割が与えられていたらスキップする
            if active_id in self._id_list_ordered_by_role_priority:
                continue

            # アクティブなIDがgoalieであれば、指定されたスペースにセットする
            if active_id == self._goalie_id:
                goalie_priority = self._get_priority_of_role(RoleName.GOALIE)
                self._id_list_ordered_by_role_priority[goalie_priority] = self._goalie_id
                continue

            # 空きスペースにIDをセットする
            # ただし、goalieのスペースは空けておく
            priority = self._find_highest_prioiry_from_free_roles(ignore_goalie=True)
            if priority is not None:
                self._id_list_ordered_by_role_priority[priority] = active_id

    def _sort_empty_role(self):
        # 優先度の高いroleに空きが出ないように、
        # 一番優先度の低いroleの担当を、優先度の高い空きroleに割り当てる

        # 空いているroleがなければ終了
        if None not in self._id_list_ordered_by_role_priority:
            return

        for priority, robot_id in enumerate(self._id_list_ordered_by_role_priority):
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
        for robot_id in reversed(self._id_list_ordered_by_role_priority):
            if robot_id is not None:
                return self._id_list_ordered_by_role_priority.index(robot_id)
        return None

    def _find_highest_prioiry_from_free_roles(self, ignore_goalie=True):
        # 担当がいないroleの中で、最も優先度が高いものを探し、優先度を返す
        # 空きroleがなければNoneを返す
        # ignore_goalieがTrueの場合、goalieのroleを除外する
        for priority, robot_id in enumerate(self._id_list_ordered_by_role_priority):
            if priority == self._get_priority_of_role(RoleName.GOALIE) and ignore_goalie:
                continue

            if robot_id is None:
                return priority
        return None

    def _swap_robot_id_of_priority(self, priority1, priority2):
        # robot_id_of_priorityの指定されたpriorityの要素を入れ替える
        self._id_list_ordered_by_role_priority[priority1], \
            self._id_list_ordered_by_role_priority[priority2] \
            = self._id_list_ordered_by_role_priority[priority2], \
            self._id_list_ordered_by_role_priority[priority1]

    def _determine_attacker_via_ball_pos(self, our_robots: dict[int, PosVel], ball: PosVel):
        # ボール位置に一番近いロボットをアタッカー候補とする
        def nearest_robot_id_by_ball_position():
            # ボールに最も近いロボットを返す
            next_id = None
            nearest_distance = 1000  # 適当な巨大な距離を初期値とする
            for robot_id, robot in our_robots.items():
                # Goalieはスキップ
                if robot_id == self._goalie_id:
                    continue

                distance = tool.get_distance(ball.pos(), robot.pos())

                # 最もボールに近いロボットの距離とIDを更新
                if distance < nearest_distance:
                    nearest_distance = distance
                    next_id = robot_id
            return next_id

        return nearest_robot_id_by_ball_position()

    def _determine_attacker_via_ball_motion(self, our_robots: dict[int, PosVel], ball: PosVel):
        # ボール軌道に一番近いロボットをアタッカー候補とする

        def nearest_robot_id_by_ball_motion():
            # ボールの軌道に一番近いロボットを返す
            VEL_NORM_GAIN = 2.0
            next_id = None
            nearest_distance = 1000  # 適当な巨大な距離を初期値とする

            ball_vel_norm = math.hypot(ball.vel().x, ball.vel().y)
            # ボールを中心にしたボール軌道をX軸とする座標系を生成
            trans = tool.Trans(ball.pos(), math.atan2(ball.vel().y, ball.vel().x))
            for robot_id, robot in our_robots.items():
                # Goalieはスキップ
                if robot_id == self._goalie_id:
                    continue

                tr_robot_pos = trans.transform(robot.pos())
                # ボール軌道の後ろにいるロボットはスキップ
                if tr_robot_pos.x < 0:
                    continue

                # ボール速度を考慮して、ボールに近すぎるロボットはスキップ
                # distance_to_ball = math.hypot(tr_robot_pos.x, tr_robot_pos.y)
                # if distance_to_ball < ball_vel_norm * VEL_NORM_GAIN:
                #     continue

                distance_to_trajectory = abs(tr_robot_pos.y)
                # 最もボールに近いロボットの距離とIDを更新
                if distance_to_trajectory < nearest_distance:
                    nearest_distance = distance_to_trajectory
                    next_id = robot_id
            return next_id

        return nearest_robot_id_by_ball_motion()

    # def _determine_next_attacker_id(self, our_robots: dict[int, PosVel], ball: PosVel):
    #     # ボールを受け取れるロボットをアタッカー候補として出力する
    #     # ボールが転がっている場合は、その軌道に一番近いロボットをアタッカー候補とする
    #     # ボールが止まっている場合は、ボールに最も近いロボットをアタッカー候補とする

    #     def nearest_robot_id_by_ball_position():
    #         # ボールに最も近いロボットを返す
    #         next_id = None
    #         nearest_distance = 1000  # 適当な巨大な距離を初期値とする
    #         for robot_id, robot in our_robots.items():
    #             # Goalieはスキップ
    #             if robot_id == self._goalie_id:
    #                 continue

    #             distance = tool.get_distance(ball.pos(), robot.pos())

    #             # 最もボールに近いロボットの距離とIDを更新
    #             if distance < nearest_distance:
    #                 nearest_distance = distance
    #                 next_id = robot_id
    #         return next_id

    #     # def nearest_robot_id_by_ball_motion():
    #     #     # ボールの軌道に一番近いロボットを返す
    #     #     VEL_NORM_GAIN = 2.0
    #     #     next_id = None
    #     #     nearest_distance = 1000  # 適当な巨大な距離を初期値とする

    #     #     ball_vel_norm = math.hypot(ball.vel().x, ball.vel().y)
    #     #     trans = tool.trans(ball.pos(), math.atan2(ball.vel().y, ball.vel().x))
    #     #     for robot_id, robot in our_robots.items():
    #     #         # Goalieはスキップ
    #     #         if robot_id == self._goalie_id:
    #     #             continue

    #     #         tr_robot_pos = trans.transform(robot.pos())
    #     #         # ボール軌道の後ろにいるロボットはスキップ
    #     #         if tr_robot_pos.x < 0:
    #     #             continue

    #     #         # ボール速度を考慮して、ボールに近すぎるロボットはスキップ
    #     #         distance = math.hypot(tr_robot_pos.x, tr_robot_pos.y)
    #     #         if distance < ball_vel_norm * VEL_NORM_GAIN:
    #     #             continue

    #     #         # 最もボールに近いロボットの距離とIDを更新
    #     #         if distance < nearest_distance:
    #     #             nearest_distance = distance
    #     #             next_id = robot_id
    #     #     return next_id

    #     def select_attacker_id_by_ball_position(present_id: int, next_id: int) -> int:
    #         NEAREST_DIFF_THRESHOLD = 0.5  # meter
    #         if present_id is None:
    #             return None
    #         if next_id is None:
    #             return present_id

    #         present_distance = tool.get_distance(
    #             ball.pos(), our_robots[present_id].pos())
    #         next_distance = tool.get_distance(
    #             ball.pos(), our_robots[next_id].pos())

    #         # ヒステリシスをもたせ、アタッカー候補が頻繁に変わらないようにする
    #         if present_distance - next_distance > NEAREST_DIFF_THRESHOLD:
    #             return next_id
    #         return present_id

    #     present_attacker_id = self._id_list_ordered_by_role_priority[
    #         self._get_priority_of_role(RoleName.ATTACKER)]

    #     next_attacker_id = nearest_robot_id_by_ball_position()

    #     # 現在アタッカーがいない場合は、アタッカー候補をそのまま返す
    #     if present_attacker_id is None:
    #         return next_attacker_id

    #     selected_id = select_attacker_id_by_ball_position(
    #         present_attacker_id, next_attacker_id)

    #     return selected_id

    def _overwrite_substite_to_present_role_list(self, num_of_substitute):
        # 指定した数だけ、presetn_role_listの後ろからSUBSTITUEロールを上書きする
        if num_of_substitute <= 0:
            return

        self._present_role_list[-num_of_substitute:] = \
            [RoleName.SUBSTITUTE] * min(num_of_substitute, len(self._present_role_list))
