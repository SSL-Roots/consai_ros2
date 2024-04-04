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

from consai_examples.operation import Operation
from consai_msgs.msg import NamedTargets
from consai_msgs.msg import RobotControlMsg
from consai_msgs.msg import State2D
from rclpy.node import Node
import time


# consai_robot_controllerに指令を送るノード
class RobotOperator(Node):

    STOP_GAME_VELOCITY = 0.8  # m/s

    def __init__(self, target_is_yellow=False):
        super().__init__('operator')

        ROBOT_NUM = 16
        self._action_clients = []
        self._pub_robot_control = []
        team_color = 'blue'
        if target_is_yellow:
            team_color = 'yellow'
        for i in range(ROBOT_NUM):
            self._pub_robot_control.append(
                self.create_publisher(RobotControlMsg, team_color + str(i) + '/control', 1))

        self._robot_is_free = [True] * ROBOT_NUM
        self._target_is_yellow = target_is_yellow
        self._stop_game_velocity_has_enabled = [False] * ROBOT_NUM
        self._prev_operation_timestamp = [0] * ROBOT_NUM
        self._prev_operation_hash = [None] * ROBOT_NUM

        # 名前付きターゲット格納用の辞書
        # データを扱いやすくするため、NamedTargets型ではなく辞書型を使用する
        self._named_targets = {}
        self._pub_named_targets = self.create_publisher(NamedTargets, 'named_targets', 1)

        if self._target_is_yellow:
            self.get_logger().info('yellowロボットを動かします')
        else:
            self.get_logger().info('blueロボットを動かします')

    def enable_stop_game_velocity(self, robot_id):
        self._stop_game_velocity_has_enabled[robot_id] = True

    def disable_stop_game_velocity(self, robot_id):
        self._stop_game_velocity_has_enabled[robot_id] = False

    def target_is_yellow(self):
        # 操作するロボットのチームカラーがyellowならtrue、blueならfalseを返す
        return self._target_is_yellow

    def robot_is_free(self, robot_id):
        # 指定されたIDのロボットが行動完了していたらtrue
        return self._robot_is_free[robot_id]

    def all_robots_are_free(self):
        # チームの全てのロボットが行動を完了していたらtrue
        return all(self._robot_is_free)

    def set_named_target(self, name, x, y, theta=0.0):
        # 名前付きターゲットをセットする
        # すでに同じ名前のターゲットが用意されていても上書きする
        self._named_targets[name] = State2D(x=x, y=y, theta=theta)

    def remove_named_target(self, name):
        # 指定した名前付きターゲットを削除する
        if name in self._named_targets:
            self._named_targets.pop(name)

    def clear_named_targets(self):
        # 名前付きターゲットを初期化する
        self._named_targets.clear()

    def publish_named_targets(self):
        # 名前付きターゲットを送信する
        msg = NamedTargets()
        for name, pose in self._named_targets.items():
            msg.name.append(name)
            msg.pose.append(pose)
        self._pub_named_targets.publish(msg)

    def reset_operation(self, robot_id: int) -> None:
        self._prev_operation_hash[robot_id] = None
        self._prev_operation_timestamp[robot_id] = 0.0

    def operate(self, robot_id: int, operation: Operation) -> None:
        # 前回と違うOperationが来たときのみ、Goalを設定する
        hash_goal = operation.get_hash()
        if self._prev_operation_hash[robot_id] == hash_goal:
            return
        self.get_logger().debug(
            'New operation for Robot {}'.format(robot_id))

        if self._stop_game_velocity_has_enabled[robot_id]:
            operation = operation.restrict_velocity_xy(self.STOP_GAME_VELOCITY)

        self._pub_robot_control[robot_id].publish(operation.get_goal())
        self._prev_operation_hash[robot_id] = hash_goal

        # 短い期間でoperateする場合は警告を出す
        THRESHOLD = 0.1  # seconds
        present_time = time.time()
        if present_time - self._prev_operation_timestamp[robot_id] < THRESHOLD:
            self.get_logger().warn(
                'Robot {} is operated too frequently'.format(robot_id))
        self._prev_operation_timestamp[robot_id] = present_time
