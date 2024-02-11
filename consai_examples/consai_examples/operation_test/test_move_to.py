#!/usr/bin/env python3
# coding: UTF-8

# Copyright 2023 Roots
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

import argparse
import threading

import rclpy
import math
from rclpy.executors import MultiThreadedExecutor
from consai_examples.robot_operator import RobotOperator

from consai_examples.operation import OneShotOperation
from consai_examples.operation import TargetXY
from consai_examples.operation import TargetTheta


def test_move_to(target_x: float, target_y: float):
    # フィールド上の全ロボットが、フィールドを上下(y軸)に往復する
    ROBOT_NUM = 16
    OFFSET_X = 0.5
    TARGET_THETA = math.pi * 0.5

    for i in range(ROBOT_NUM):
        operation = OneShotOperation().move_to_pose(
            TargetXY.value(target_x + OFFSET_X * i, target_y),
            TargetTheta.value(TARGET_THETA))
        operator_node.operate(i, operation)

    # 全てのロボットがフリー（目的地に到着 or 常時制御中）になるまで待機
    while operator_node.all_robots_are_free() is False:
        pass

    # 制御継続フラグがTrueなので、ロボットはすぐにフリー状態になる
    for i in range(ROBOT_NUM):
        operation = OneShotOperation().move_to_pose(
            TargetXY.value(target_x + OFFSET_X * i, -target_y),
            TargetTheta.value(-TARGET_THETA))
        operator_node.operate(i, operation)

    while operator_node.all_robots_are_free() is False:
        pass


if __name__ == '__main__':
    arg_parser = argparse.ArgumentParser()
    arg_parser.add_argument('--yellow',
                            default=False,
                            action='store_true',
                            help='yellowロボットを動かす場合にセットする')
    arg_parser.add_argument('--invert',
                            default=False,
                            action='store_true',
                            help='ball placementの目標座標を反転する場合にセットする')
    arg_parser.add_argument('-x', default=-5.0)
    arg_parser.add_argument('-y', default=4.0)
    args = arg_parser.parse_args()

    rclpy.init(args=None)

    operator_node = RobotOperator(target_is_yellow=args.yellow)

    # すべてのロボットの衝突回避を解除
    for i in range(16):
        operator_node.disable_avoid_obstacles(i)

    executor = MultiThreadedExecutor()
    executor.add_node(operator_node)

    # エグゼキュータは別スレッドでspinさせ続ける
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    try:
        test_move_to(float(args.x), float(args.y))
    except KeyboardInterrupt:
        pass

    executor.shutdown()
    rclpy.shutdown()
