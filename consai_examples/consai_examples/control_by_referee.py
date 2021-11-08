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
from robot_operator import RobotOperator
from referee_parser import RefereeParser
import math
import rclpy
from rclpy.executors import SingleThreadedExecutor


def main():
    # 実行したい関数のコメントを外してください
    while rclpy.ok():
        if referee_parser.halt():
            print("halt")
        elif referee_parser.stop():
            print("stop")
        elif referee_parser.our_pre_kickoff():
            print("our pre kickoff")
        elif referee_parser.our_kickoff():
            print("our kickoff")
        elif referee_parser.their_pre_kickoff():
            print("their pre kickoff")
        elif referee_parser.their_kickoff():
            print("their kickoff")
        elif referee_parser.our_pre_penalty():
            print("our pre penalty")
        elif referee_parser.our_penalty():
            print("our penalty kick")
        elif referee_parser.their_pre_penalty():
            print("their pre penalty")
        elif referee_parser.their_penalty():
            print("their penalty kick")
        elif referee_parser.our_direct():
            print("our direct free kick")
        elif referee_parser.their_direct():
            print("their direct free kick")
        elif referee_parser.our_indirect():
            print("our indirect free kick")
        elif referee_parser.their_indirect():
            print("their indirect free kick")
        elif referee_parser.our_timeout():
            print("our timeout")
        elif referee_parser.their_timeout():
            print("their timeout")
        elif referee_parser.our_ball_placement():
            print("our ball placement")
        elif referee_parser.their_ball_placement():
            print("their ball placement")
        else:
            print("none")
        executor.spin_once(1)

if __name__ == '__main__':
    arg_parser = argparse.ArgumentParser()
    arg_parser.add_argument('--yellow',
        default=False,
        action='store_true',
        help='yellowロボットを動かす場合にセットする')
    args = arg_parser.parse_args()

    rclpy.init(args=None)

    our_team_is_yellow = False

    operator_node = RobotOperator(args.yellow)
    referee_parser = RefereeParser(args.yellow)

    executor = SingleThreadedExecutor()
    executor.add_node(operator_node)
    executor.add_node(referee_parser)

    try:
        main()
    except KeyboardInterrupt:
        pass

    executor.shutdown()
    rclpy.shutdown()
