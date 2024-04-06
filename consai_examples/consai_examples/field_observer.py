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
from consai_tools.geometry import geometry_tools as tool
from consai_examples.observer.detection_wrapper import DetectionWrapper
from consai_examples.observer.pos_vel import PosVel
from consai_examples.observer.ball_position_observer import BallPositionObserver
from consai_examples.observer.ball_placement_observer import BallPlacementObserver
from consai_examples.observer.zone_ball_observer import ZoneBallObserver
from consai_examples.observer.zone_target_observer import ZoneTargetObserver
from consai_examples.observer.ball_motion_observer import BallMotionObserver

# フィールド状況を観察し、ボールの位置を判断したり
# ロボットに一番近いロボットを判定する


class FieldObserver(Node):
    BALL_NONE = 0

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

        self._detection_wrapper = DetectionWrapper(our_team_is_yellow)
        self._ball_position_state_observer = BallPositionObserver()
        self._ball_placement_observer = BallPlacementObserver()
        self._zone_ball_observer = ZoneBallObserver()
        self._zone_target_observer = ZoneTargetObserver()
        self._ball_motion_observer = BallMotionObserver()

        self._num_of_zone_roles = 0

    def detection(self) -> DetectionWrapper:
        return self._detection_wrapper

    def ball_position(self) -> BallPositionObserver:
        return self._ball_position_state_observer

    def ball_placement(self) -> BallPlacementObserver:
        return self._ball_placement_observer

    def zone(self) -> ZoneBallObserver:
        return self._zone_ball_observer

    def zone_target(self) -> ZoneTargetObserver:
        return self._zone_target_observer

    def ball_motion(self) -> BallMotionObserver:
        return self._ball_motion_observer

    def set_num_of_zone_roles(self, num_of_zone_roles: int) -> None:
        self._num_of_zone_roles = num_of_zone_roles

    def field_half_length(self) -> float:
        return self._field_half_x

    def field_half_width(self) -> float:
        return self._field_half_y

    def field_margin_to_wall(self) -> float:
        return 0.3

    def _detection_tracked_callback(self, msg):
        self._detection = msg

        self._detection_wrapper.update(msg)
        self._ball_position_state_observer.update(
            self._detection_wrapper.ball().pos())
        self._ball_placement_observer.update(self._detection_wrapper.ball())
        self._zone_ball_observer.update(
            self._detection_wrapper.ball().pos(),
            self.ball_position().is_in_our_side())
        self._zone_target_observer.update(
            self._detection_wrapper.their_robots(), self._num_of_zone_roles)
        self._ball_motion_observer.update(self._detection_wrapper.ball())

        if len(msg.balls) > 0:
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

    def get_ball_state(self):
        return self._ball_state

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
