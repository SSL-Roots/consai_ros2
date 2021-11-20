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
import threading
import time

from decisions.decision_base import DecisionBase
from decisions.goalie import GoaleDecition
import rclpy
from rclpy.executors import MultiThreadedExecutor
from referee_parser import RefereeParser
from robot_operator import RobotOperator
from role_assignment import RoleAssignment

ROLE_GOALIE = 0
ROLE_ATTACKER = 1
ROLE_DEFENSE1 = 2
ROLE_DEFENSE2 = 3
ROLE_DEFENSE3 = 4
ROLE_DEFENSE4 = 5
ROLE_OFFENSE1 = 6
ROLE_OFFENSE2 = 7
ROLE_OFFENSE3 = 8
ROLE_OFFENSE4 = 9
ROLE_CENTER = 10

def main():
    prev_command_count = -1  # レフェリーコマンド更新判定用の変数
    while rclpy.ok():
        # ロボットの役割の更新
        updated_roles = assignment_node.update_role()
        if prev_command_count != referee_parser.get_command_count():
            prev_command_count = referee_parser.get_command_count()
            print(decisions.keys())

        # 役割が変わったロボットのみ、行動をアップデートする
        for role in updated_roles:
            robot_id = assignment_node.get_robot_id(role)
            # レフェリーコマンドに合わせて行動を決定する
            if referee_parser.halt():
                decisions[role].halt(robot_id)
            elif referee_parser.stop():
                decisions[role].stop(robot_id)
            else:
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
    arg_parser.add_argument('--goalie',
                            default=0,
                            type=int,
                            help='ゴールキーパのIDをセットする')
    args = arg_parser.parse_args()

    rclpy.init(args=None)

    operator_node = RobotOperator(args.yellow)
    assignment_node = RoleAssignment(args.goalie, args.yellow)
    referee_parser = RefereeParser(args.yellow, args.invert)

    executor = MultiThreadedExecutor()
    executor.add_node(operator_node)
    executor.add_node(assignment_node)
    executor.add_node(referee_parser)

    # エグゼキュータは別スレッドでspinさせ続ける
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    decisions = {
        ROLE_GOALIE: GoaleDecition(operator_node),
        ROLE_ATTACKER: DecisionBase(operator_node),
        ROLE_DEFENSE1: DecisionBase(operator_node),
        ROLE_DEFENSE2: DecisionBase(operator_node),
        ROLE_DEFENSE3: DecisionBase(operator_node),
        ROLE_DEFENSE4: DecisionBase(operator_node),
        ROLE_OFFENSE1: DecisionBase(operator_node),
        ROLE_OFFENSE2: DecisionBase(operator_node),
        ROLE_OFFENSE3: DecisionBase(operator_node),
        ROLE_OFFENSE4: DecisionBase(operator_node),
        ROLE_CENTER: DecisionBase(operator_node),
    }

    try:
        main()
    except KeyboardInterrupt:
        pass

    print("ロボットを停止します")

    # ロボットを停止
    for i in range(16):
        operator_node.stop(i)
    time.sleep(5.0)

    rclpy.shutdown()
    executor.shutdown()
