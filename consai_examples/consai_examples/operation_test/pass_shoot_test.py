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
from rclpy.executors import MultiThreadedExecutor
from consai_examples.robot_operator import RobotOperator

from consai_examples.operation import Operation
from consai_examples.operation import TargetXY
from consai_examples.operation import TargetTheta


def pass_shoot(robot_id0: int, x0: float, y0: float, robot_id1: int, x1: float, y1: float):
    # ロボットを目標位置に動かしてリフレクトシュートかパスをパス相手の味方ロボットに行う

    # robot_id0に設定しているロボットを指定した目標位置に移動
    pass_shoot0 = Operation().move_to_pose(TargetXY.value(x0, y0), TargetTheta.look_ball())
    # robot_id1に指定したロボットに対してリフレクトシュートをする
    pass_shoot0 = pass_shoot0.with_reflecting_to(TargetXY.our_robot(robot_id1))
    # robot_id1に指定したロボットに対してパスをする
    pass_shoot0 = pass_shoot0.with_passing_to(TargetXY.our_robot(robot_id1))
    operator_node.operate(robot_id0, pass_shoot0)

    # robot_id1に指定しているロボットを指定した目標位置に移動
    pass_shoot1 = Operation().move_to_pose(TargetXY.value(x1, y1), TargetTheta.look_ball())
    # robot_id0に指定したロボットに対してリフレクトシュートをする
    pass_shoot1 = pass_shoot1.with_reflecting_to(TargetXY.our_robot(robot_id0))
    # robot_id0に指定したロボットに対してパスをする
    pass_shoot1 = pass_shoot1.with_passing_to(TargetXY.our_robot(robot_id0))
    operator_node.operate(robot_id1, pass_shoot1)

    while True:
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
    arg_parser.add_argument('--robot_id0', '-id0', type=int, default=0)
    arg_parser.add_argument('-x0', type=float, default=3.0)
    arg_parser.add_argument('-y0', type=float, default=-1.0)
    arg_parser.add_argument('--robot_id1', '-id1', type=int, default=1)
    arg_parser.add_argument('-x1', type=float, default=-3.0)
    arg_parser.add_argument('-y1', type=float, default=1.0)
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
        pass_shoot(int(args.robot_id0), float(args.x0), float(args.y0),
                   int(args.robot_id1), float(args.x1), float(args.y1))
    except KeyboardInterrupt:
        pass

    executor.shutdown()
    rclpy.shutdown()
