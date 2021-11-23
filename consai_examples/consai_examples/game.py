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

from decisions.attacker import AttackerDecision
from decisions.decision_base import DecisionBase
from decisions.goalie import GoaleDecition
from decisions.sub_attacker import SubAttackerDecision
from field_observer import FieldObserver
import rclpy
from rclpy.executors import MultiThreadedExecutor
from referee_parser import RefereeParser
from robot_operator import RobotOperator
from role_assignment import RoleAssignment

ROLE_GOALIE = 0
ROLE_ATTACKER = 1
ROLE_CENTER_BACK1 = 2
ROLE_CENTER_BACK2 = 3
ROLE_SUB_ATTACKER = 4
ROLE_ZONE1 = 5
ROLE_ZONE2 = 6
ROLE_ZONE3 = 7
ROLE_ZONE4 = 8
ROLE_SIDE_BACK1 = 9
ROLE_SIDE_BACK2 = 10

def main():
    prev_referee_command = -1  # レフェリーコマンド更新判定用の変数
    prev_ball_state = -1  # ボール状態更新判定用の変数
    prev_ball_placement_state = -1  # ボール配置状態更新判定用の変数
    while rclpy.ok():
        updated_roles = []
        referee_has_changed = prev_referee_command != referee.get_command()
        ball_state_has_changed = prev_ball_state != observer.get_ball_state()

        # レフェリーコマンドが変化した場合は、全てのロボットの行動を更新する
        if referee_has_changed:
            print("referee command has changed")
            prev_referee_command = referee.get_command()
            updated_roles = assignor.get_active_roles()

        # ボール状態が変化した場合は、すべてのロボットの行動を更新する
        if ball_state_has_changed:
            print("ball state has changed")
            prev_ball_state = observer.get_ball_state()
            updated_roles = assignor.get_active_roles()

        # ボール配置状態が変化した場合は、すべてのロボットの行動を更新する
        if referee.our_ball_placement() or referee.their_ball_placement():
            current_state = observer.get_ball_placement_state(referee.placement_position())
            if prev_ball_placement_state != current_state:
                print("ball placement state has changed")
                prev_ball_placement_state = current_state
                updated_roles = assignor.get_active_roles()

        # アタッカーの切り替わりを防ぐため、
        # ボールが動いてたり、ディフェンスエリアにあるときは役割を更新しない
        if not referee_has_changed and \
           not ball_state_has_changed and \
           not observer.ball_is_in_our_defense_area() and \
           not observer.ball_is_moving():
            # ロボットの役割の更新し、
            # 役割が変わったロボットのみ、行動を更新する
            updated_roles = assignor.update_role()

        for role in updated_roles:
            robot_id = assignor.get_robot_id(role)
            # ボール状態をセットする
            decisions[role].set_ball_state(prev_ball_state)
            # ボール配置状態をセットする
            decisions[role].set_ball_placement_state(prev_ball_placement_state)

            # レフェリーコマンドに合わせて行動を決定する
            if observer.ball_is_outside():
                # ボールが場外に出たらロボットを停止する
                decisions[role].halt(robot_id)
                print("ボールが外に出た")
                continue

            if referee.halt():
                decisions[role].halt(robot_id)
            elif referee.stop():
                decisions[role].stop(robot_id)
            elif referee.inplay():
                decisions[role].inplay(robot_id)
            elif referee.our_pre_kickoff():
                decisions[role].our_pre_kickoff(robot_id)
            elif referee.our_kickoff():
                decisions[role].our_kickoff(robot_id)
            elif referee.their_pre_kickoff():
                decisions[role].their_pre_kickoff(robot_id)
            elif referee.their_kickoff():
                decisions[role].their_kickoff(robot_id)
            elif referee.our_pre_penalty():
                decisions[role].our_pre_penalty(robot_id)
            elif referee.our_penalty():
                decisions[role].our_penalty(robot_id)
            elif referee.their_pre_penalty():
                decisions[role].their_pre_penalty(robot_id)
            elif referee.their_penalty():
                decisions[role].their_penalty(robot_id)
            elif referee.our_direct():
                decisions[role].our_direct(robot_id)
            elif referee.their_direct():
                decisions[role].their_direct(robot_id)
            elif referee.our_indirect():
                decisions[role].our_indirect(robot_id)
            elif referee.their_indirect():
                decisions[role].their_indirect(robot_id)
            elif referee.our_timeout():
                decisions[role].our_timeout(robot_id)
            elif referee.their_timeout():
                decisions[role].their_timeout(robot_id)
            elif referee.our_ball_placement():
                decisions[role].our_ball_placement(
                    robot_id, referee.placement_position())
            elif referee.their_ball_placement():
                decisions[role].their_ball_placement(
                    robot_id, referee.placement_position())
            else:
                print("UNDEFINED REFEREE COMMAND!!!")

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

    operator = RobotOperator(args.yellow)
    assignor = RoleAssignment(args.goalie, args.yellow)
    referee = RefereeParser(args.yellow, args.invert)
    observer = FieldObserver()

    executor = MultiThreadedExecutor()
    executor.add_node(operator)
    executor.add_node(assignor)
    executor.add_node(referee)
    executor.add_node(observer)

    # エグゼキュータは別スレッドでspinさせ続ける
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    decisions = {
        ROLE_GOALIE: GoaleDecition(operator),
        ROLE_ATTACKER: AttackerDecision(operator),
        ROLE_CENTER_BACK1: DecisionBase(operator),
        ROLE_CENTER_BACK2: DecisionBase(operator),
        ROLE_SUB_ATTACKER: SubAttackerDecision(operator),
        ROLE_ZONE1: DecisionBase(operator),
        ROLE_ZONE2: DecisionBase(operator),
        ROLE_ZONE3: DecisionBase(operator),
        ROLE_ZONE4: DecisionBase(operator),
        ROLE_SIDE_BACK1: DecisionBase(operator),
        ROLE_SIDE_BACK2: DecisionBase(operator),
    }

    try:
        main()
    except KeyboardInterrupt:
        pass

    print("ロボットを停止します")
    # ロボットを停止
    for i in range(16):
        operator.stop(i)
    time.sleep(1.0)

    rclpy.shutdown()
    executor.shutdown()
