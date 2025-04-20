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

"""ロボットの役割を割り当て、更新する処理を実装したモジュール."""

import copy
import math
from enum import Enum

from consai_examples.observer.pos_vel import PosVel
from consai_examples.role_to_visualize_msg import to_visualize_msg

from consai_tools.geometry import geometry_tools as tool

from consai_visualizer_msgs.msg import Objects

from rclpy import qos
from rclpy.node import Node

# 設定可能なロボット役割一覧
# ロボットの台数より多く定義してもOK


class RoleName(Enum):
    """ロボットの役割を定義する列挙型."""

    GOALIE = 0
    ATTACKER = 1
    CENTER_BACK1 = 2
    CENTER_BACK2 = 3
    SUB_ATTACKER1 = 4
    ZONE1 = 5
    ZONE2 = 6
    ZONE3 = 7
    SUB_ATTACKER2 = 8
    LEFT_WING = 11
    RIGHT_WING = 12
    SIDE_BACK1 = 13
    SIDE_BACK2 = 14
    SUBSTITUTE = 15


class BallState(Enum):
    """ボールの状態を定義する列挙型."""

    NONE = 0
    STOP = 1
    MOVE = 2


class RoleAssignment(Node):
    """
    フィールド状況を見て, ロボットの役割を決めるノード.

    ロボットの役割が頻繁に変わらないように調整する.
    """

    # フィールド上のロボット役割一覧
    # フィールドに出せるロボットの数は11台
    # Ref: https://robocup-ssl.github.io/ssl-rules/sslrules.html#_number_of_robots
    ACTIVE_ROLE_LIST = [
        RoleName.GOALIE,
        RoleName.ATTACKER,
        RoleName.SUB_ATTACKER1,
        RoleName.CENTER_BACK1,
        RoleName.CENTER_BACK2,
        RoleName.SIDE_BACK1,
        RoleName.SIDE_BACK2,
        RoleName.SUB_ATTACKER2,
        RoleName.ZONE1,
        RoleName.ZONE2,
        RoleName.ZONE3,
        # RoleName.LEFT_WING,
        # RoleName.RIGHT_WING,
    ]

    def __init__(self, goalie_id: int):
        """
        RoleAssignmentクラスの初期化処理を行う関数.

        Args:
            goalie_id (int): ゴールキーパーとして割り当てるロボットのID.
        """
        super().__init__("assignor")
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
        self.get_logger().info("goalie IDは{}です".format(self._goalie_id))

        self._pub_visualizer_objects = self.create_publisher(Objects, "visualizer_objects", qos.qos_profile_sensor_data)

    def update_role(self, ball: PosVel, our_robots: dict[int, PosVel], allowed_robot_num=11):
        # roleへのロボットの割り当てを更新する
        """
        ボールとロボットの情報に基づき, ロール割り当てを更新する関数.

        Args:
            ball (PosVel): ボールの位置と速度.
            our_robots (dict[int, PosVel]): ロボットIDをキーとした位置と速度の辞書.
            allowed_robot_num (int): フィールドに出られる最大ロボット数.

        Returns:
            list[int]: 担当が変更されたロボットIDのリスト.
        """
        # 優先度が高い順にロボットIDを並べる
        prev_id_list_ordered_by_role_priority = copy.deepcopy(self._id_list_ordered_by_role_priority)
        self._update_role_list(ball, our_robots)

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
        """
        現在割り当てられているロールとロボットIDのペアを取得する.

        Returns:
            list[tuple[RoleName, int]]: ロールとロボットIDのペアのリスト.
        """
        role_and_id = []
        for i in range(len(self._id_list_ordered_by_role_priority)):
            role = self._present_role_list[i]
            robot_id = self._id_list_ordered_by_role_priority[i]
            if robot_id is not None:
                role_and_id.append((role, robot_id))
        return role_and_id

    def get_role_from_robot_id(self, robot_id):
        """
        指定されたロボットIDに割り当てられているロールを取得する関数.

        Args:
            robot_id (int): ロボットID.

        Returns:
            RoleName or None: 割り当てられているロール, ない場合はNone.
        """
        if robot_id in self._id_list_ordered_by_role_priority:
            role_index = self._id_list_ordered_by_role_priority.index(robot_id)
            return self._present_role_list[role_index]
        return None

    def publish_role_for_visualizer(self, our_robots: dict[int, PosVel]):
        """
        可視化用にロールとIDの対応をパブリッシュする関数.

        Args:
            our_robots (dict[int, PosVel]): ロボットの情報.
        """
        role_dict = {}
        for robot_id in self._id_list_ordered_by_role_priority:
            if robot_id is not None:
                role = self.get_role_from_robot_id(robot_id)
                role_dict[robot_id] = role.name
        self._pub_visualizer_objects.publish(to_visualize_msg(role_dict, our_robots))

    def _get_priority_of_role(self, role):
        """
        指定したロールの優先度を取得する関数.

        重複するroleが設定されている場合、高いほうの優先度を返す.

        Args:
            role (RoleName): ロール.

        Returns:
            int: 優先度（小さいほど高優先）.
        """
        return self.ACTIVE_ROLE_LIST.index(role)

    def _update_role_list(self, ball: PosVel, our_robots: dict[int, PosVel]):
        """
        イベントが発生したらroleを更新する関数.

        Args:
            ball (PosVel): ボールの位置と速度.
            our_robots (dict[int, PosVel]): ロボットの情報.

        Returns:
            bool: 更新された場合はTrue.
        """
        updated = False

        our_active_ids = list(our_robots.keys())

        # イベント：フィールド上のロボット台数が変わる
        if our_active_ids != self._present_active_ids:
            # 役割リストを更新する
            self._reset_inactive_id(our_active_ids)
            self._set_avtive_id_to_empty_role(our_active_ids)
            self._sort_empty_role()
            updated = True
        self._present_active_ids = copy.deepcopy(our_active_ids)

        # イベント：ボールの状況が変わったらアタッカーを更新する
        if self._update_ball_state(ball):
            if self._present_ball_state == BallState.STOP:
                # ボールが止まっている場合は、位置情報をもとにアタッカーを更新する
                next_attacker_id = self._determine_attacker_via_ball_pos(our_robots, ball)
            elif self._present_ball_state == BallState.MOVE:
                # ボールが動いている場合は、ボールの軌道をもとにアタッカーを更新する
                next_attacker_id = self._determine_attacker_via_ball_motion(our_robots, ball)

            if next_attacker_id is not None and next_attacker_id in self._id_list_ordered_by_role_priority:
                next_attacker_priority = self._id_list_ordered_by_role_priority.index(next_attacker_id)
                attacker_role_priority = self._get_priority_of_role(RoleName.ATTACKER)
                self._swap_robot_id_of_priority(next_attacker_priority, attacker_role_priority)
                updated = True

        return updated

    def _update_ball_state(self, ball: PosVel) -> bool:
        """
        ボールの状態（停止または移動）を更新する関数.

        Args:
            ball (PosVel): ボールの位置と速度.

        Returns:
            bool: 状態が変わったらTrue.
        """
        MOVE_THRESHOLD = 1.0  # m/s
        ball_vel_norm = math.hypot(ball.vel().x, ball.vel().y)

        prev_state = copy.deepcopy(self._present_ball_state)
        if ball_vel_norm < MOVE_THRESHOLD:
            self._present_ball_state = BallState.STOP
        else:
            self._present_ball_state = BallState.MOVE
        return prev_state != self._present_ball_state

    def _reset_inactive_id(self, our_active_ids):
        """
        役割リストにあるIDが非アクティブな場合, リストの枠にNoneをセットする関数.

        Args:
            our_active_ids (list[int]): 現在アクティブなロボットIDのリスト.
        """
        for index, robot_id in enumerate(self._id_list_ordered_by_role_priority):
            if robot_id is None:
                continue

            if robot_id not in our_active_ids:
                self._id_list_ordered_by_role_priority[index] = None

    def _set_avtive_id_to_empty_role(self, our_active_ids):
        """
        アクティブなIDを役割リストの空きスペース(None)にセットする関数.

        Args:
            our_active_ids (list[int]): 現在アクティブなロボットIDのリスト.
        """
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
        """優先度の高いroleに空きが出ないように, 一番優先度の低いroleの担当を優先度の高い空きroleに割り当てる関数."""
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
        """
        割り当てられているロールの中で, 最も優先度が低いロールを探す.

        Returns:
            int or None: 優先度のインデックス, なければNone.
        """
        for robot_id in reversed(self._id_list_ordered_by_role_priority):
            if robot_id is not None:
                return self._id_list_ordered_by_role_priority.index(robot_id)
        return None

    def _find_highest_prioiry_from_free_roles(self, ignore_goalie=True):
        """
        割り当てのないロールの中で最も優先度の高いものを探す関数.

        ignore_goalieがTrueの場合、goalieのroleを除外する.

        Returns:
            int or None: 優先度のインデックス, 空きロールがなければNone.
        """
        for priority, robot_id in enumerate(self._id_list_ordered_by_role_priority):
            if priority == self._get_priority_of_role(RoleName.GOALIE) and ignore_goalie:
                continue

            if robot_id is None:
                return priority
        return None

    def _swap_robot_id_of_priority(self, priority1, priority2):
        """
        優先度の位置を指定して, ロボットIDを入れ替える.

        robot_id_of_priorityの指定されたpriorityの要素を入れ替える関数.

        Args:
            priority1 (int): 一方の優先度.
            priority2 (int): もう一方の優先度.
        """
        self._id_list_ordered_by_role_priority[priority1], self._id_list_ordered_by_role_priority[priority2] = (
            self._id_list_ordered_by_role_priority[priority2],
            self._id_list_ordered_by_role_priority[priority1],
        )

    def _determine_attacker_via_ball_pos(self, our_robots: dict[int, PosVel], ball: PosVel):
        """
        ボール位置に最も近いロボットをアタッカーとして決定する関数.

        Args:
            our_robots (dict[int, PosVel]): ロボットの情報.
            ball (PosVel): ボールの位置と速度.

        Returns:
            int or None: 選ばれたアタッカーのロボットID.
        """

        def nearest_robot_id_by_ball_position():
            """
            ボールの動きに最も近いロボットをアタッカーとして決定する関数.

            Args:
                our_robots (dict[int, PosVel]): ロボットの情報.
                ball (PosVel): ボールの位置と速度.

            Returns:
                int or None: 選ばれたアタッカーのロボットID.
            """
            next_id = None
            nearest_distance = 1000  # 適当な巨大な距離を初期値とする
            for robot_id, robot in our_robots.items():
                # Goalieはスキップ
                if robot_id == self._goalie_id:
                    continue

                distance = tool.get_distance(ball.pos(), robot.pos())

                # 最もボールに近いロボットの距離とIDを更新
                if distance < nearest_distance:
                    # nearest_distarobot_id_of_priorityの指定されたpriorityの要素を入れ替える
                    nearest_distance = 1000  # 適当な巨大な距離を初期値とする

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

                distance_to_trajectory = abs(tr_robot_pos.y)
                # 最もボールに近いロボットの距離とIDを更新
                if distance_to_trajectory < nearest_distance:
                    nearest_distance = distance_to_trajectory
                    next_id = robot_id
            return next_id

        return nearest_robot_id_by_ball_position()

    def _overwrite_substite_to_present_role_list(self, num_of_substitute):
        """指定した数だけ, presetn_role_listの後ろからSUBSTITUEロールを上書きする関数."""
        if num_of_substitute <= 0:
            return

        self._present_role_list[-num_of_substitute:] = [RoleName.SUBSTITUTE] * min(
            num_of_substitute, len(self._present_role_list)
        )
