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

import argparse
from consai_examples.operation import Operation
from consai_examples.operation import TargetXY
from consai_examples.operation import TargetTheta
from consai_examples.referee_parser import RefereeParser
from consai_examples.robot_operator import RobotOperator
import rclpy
from rclpy.executors import MultiThreadedExecutor
import threading
import time


def print_command():
    # refereeのcommandを表示する
    if referee_parser.halt():
        print('halt')
    elif referee_parser.stop():
        print('stop')
    elif referee_parser.inplay():
        print('inplay')
    elif referee_parser.our_pre_kickoff():
        print('our pre kickoff')
    elif referee_parser.our_kickoff():
        print('our kickoff')
    elif referee_parser.their_pre_kickoff():
        print('their pre kickoff')
    elif referee_parser.their_kickoff():
        print('their kickoff')
    elif referee_parser.our_pre_penalty():
        print('our pre penalty')
    elif referee_parser.our_penalty():
        print('our penalty kick')
    elif referee_parser.their_pre_penalty():
        print('their pre penalty')
    elif referee_parser.their_penalty():
        print('their penalty kick')
    elif referee_parser.our_direct():
        print('our direct free kick')
    elif referee_parser.their_direct():
        print('their direct free kick')
    elif referee_parser.our_indirect():
        print('our indirect free kick')
    elif referee_parser.their_indirect():
        print('their indirect free kick')
    elif referee_parser.our_timeout():
        print('our timeout')
    elif referee_parser.their_timeout():
        print('their timeout')
    elif referee_parser.our_ball_placement():
        print('our ball placement x:{}, y:{}'.format(
            referee_parser.placement_position().x,
            referee_parser.placement_position().y))
    elif referee_parser.their_ball_placement():
        print('their ball placement x:{}, y:{}'.format(
            referee_parser.placement_position().x,
            referee_parser.placement_position().y))
    else:
        print('none')


def stop_operation(args: argparse.Namespace) -> None:
    # RefereeコマンドがSTOPのときの動作
    # 0番のロボットをボール近くへ動かす

    # Ref: https://robocup-ssl.github.io/ssl-rules/sslrules.html#_stop

    print('STOPでは、全てのロボットは1.5 m/s未満に減速しなければなりません')
    print('全てのロボットはボールから0.5メートルの距離を取らなければなりません')
    print('ボールを蹴ったりドリブルしたりしてはいけません')
    operation = Operation().move_to_pose(
        TargetXY.ball(), TargetTheta.look_ball())
    operation = operation.offset_pose_x(-0.6)
    operator_node.operate(args.id, operation)

    while referee_parser.stop():
        time.sleep(1)  # コマンドが変わるまで待機


def halt_operation():
    # RefereeコマンドがHALTのときの動作
    # 全てのロボットを停止させる

    # Ref: https://robocup-ssl.github.io/ssl-rules/sslrules.html#_halt
    print('HALTではロボットを動かしてはいけません')
    print('ロボットが止まるまでに2秒の猶予があります')
    for i in range(16):
        operator_node.stop(i)

    while referee_parser.halt():
        time.sleep(1)  # コマンドが変わるまで待機


def default_operation():
    # 全てのロボットを停止させて1秒待機する

    for i in range(16):
        operator_node.stop(i)
    time.sleep(1)


def main():
    while rclpy.ok():
        print_command()
        if referee_parser.halt():
            halt_operation()
        elif referee_parser.stop():
            stop_operation(args)
        else:
            default_operation()


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
    arg_parser.add_argument('--id',
                            default=0,
                            type=int,
                            help='ロボットID')
    args = arg_parser.parse_args()

    rclpy.init(args=None)

    our_team_is_yellow = False

    operator_node = RobotOperator(args.yellow)
    referee_parser = RefereeParser(args.yellow, args.invert)

    executor = MultiThreadedExecutor()
    executor.add_node(operator_node)
    executor.add_node(referee_parser)

    # エグゼキュータは別スレッドでspinさせ続ける
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    try:
        main()
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
    executor.shutdown()
