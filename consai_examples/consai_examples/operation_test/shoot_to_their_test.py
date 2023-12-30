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


def shoot_to_their_test(robot_id: int):
    operation = OneShotOperation().move_to_pose(TargetXY.ball(), TargetTheta.look_ball())
    operation = operation.with_shooting_to(TargetXY.their_goal())
    operator_node.operate(robot_id, operation)


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
    arg_parser.add_argument('-robot_id', default=0)
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
        shoot_to_their_test(int(args.robot_id))
    except KeyboardInterrupt:
        pass

    executor.shutdown()
    rclpy.shutdown()
