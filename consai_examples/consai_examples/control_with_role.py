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

import rclpy
from rclpy.executors import MultiThreadedExecutor
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

def action_goalie(robot_id):
    # goalieの行動
    print("goalie {} はゴール前に移動します".format(robot_id))
    operator_node.move_to_normalized(robot_id, -0.9, 0.0, 0.0, False)

def action_attacker(robot_id):
    # attackerの行動
    print("attacker {} はボールを追いかけます".format(robot_id))
    operator_node.chase_ball(robot_id, -0.6, 0.0, 0.0, False, True)

def action_defense1(robot_id):
    # defense1の行動
    print("defense1 {} は自チームサイドの左上に移動します".format(robot_id))
    operator_node.move_to_normalized(robot_id, -0.8, 0.8, 0.0, True)

def action_defense2(robot_id):
    # defense2の行動
    print("defense2 {} は自チームサイドの右上に移動します".format(robot_id))
    operator_node.move_to_normalized(robot_id, -0.2, 0.8, 0.0, True)

def action_defense3(robot_id):
    # defense3の行動
    print("defense3 {} は自チームサイドの右下に移動します".format(robot_id))
    operator_node.move_to_normalized(robot_id, -0.2, -0.8, 0.0, True)

def action_defense4(robot_id):
    # defense4の行動
    print("defense4 {} は自チームサイドの左下に移動します".format(robot_id))
    operator_node.move_to_normalized(robot_id, -0.8, -0.8, 0.0, True)

def action_offense1(robot_id):
    # offense1の行動
    print("offense1 {} は相手チームサイドの左上に移動します".format(robot_id))
    operator_node.move_to_normalized(robot_id, 0.2, 0.8, 0.0, True)

def action_offense2(robot_id):
    # offense2の行動
    print("offense1 {} は相手チームサイドの右上に移動します".format(robot_id))
    operator_node.move_to_normalized(robot_id, 0.8, 0.8, 0.0, True)

def action_offense3(robot_id):
    # offense3の行動
    print("offense3 {} は相手チームサイドの右下に移動します".format(robot_id))
    operator_node.move_to_normalized(robot_id, 0.8, -0.8, 0.0, True)

def action_offense4(robot_id):
    # offense4の行動
    print("offense4 {} は相手チームサイドの左下に移動します".format(robot_id))
    operator_node.move_to_normalized(robot_id, 0.2, -0.8, 0.0, True)

def action_center(robot_id):
    # centerの行動
    print("center {} はフィールド中央に移動します".format(robot_id))
    operator_node.move_to_normalized(robot_id, 0.0, 0.0, 0.0, True)

actions = {
    ROLE_GOALIE: action_goalie,
    ROLE_ATTACKER: action_attacker,
    ROLE_DEFENSE1: action_defense1,
    ROLE_DEFENSE2: action_defense2,
    ROLE_DEFENSE3: action_defense3,
    ROLE_DEFENSE4: action_defense4,
    ROLE_OFFENSE1: action_offense1,
    ROLE_OFFENSE2: action_offense2,
    ROLE_OFFENSE3: action_offense3,
    ROLE_OFFENSE4: action_offense4,
    ROLE_CENTER: action_center,
}

def main():
    while rclpy.ok():
        updated_roles = assignment_node.update_role()
        for role in updated_roles:
            robot_id = assignment_node.get_robot_id(role)
            actions[role](robot_id)

        time.sleep(0.1)

if __name__ == '__main__':
    arg_parser = argparse.ArgumentParser()
    arg_parser.add_argument('--yellow',
                            default=False,
                            action='store_true',
                            help='yellowロボットを動かす場合にセットする')
    arg_parser.add_argument('--goalie',
                            default=0,
                            type=int,
                            help='ゴールキーパのIDをセットする')
    args = arg_parser.parse_args()

    rclpy.init(args=None)

    operator_node = RobotOperator(args.yellow)
    assignment_node = RoleAssignment(args.goalie, args.yellow)

    executor = MultiThreadedExecutor()
    executor.add_node(operator_node)
    executor.add_node(assignment_node)

    # エグゼキュータは別スレッドでspinさせ続ける
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    try:
        main()
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
    executor.shutdown()
