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
from decisions.attacker import AttackerDecision
from decisions.center_back1 import CenterBack1Decision
from decisions.center_back2 import CenterBack2Decision
from decisions.goalie import GoaleDecision
from decisions.side_back1 import SideBack1Decision
from decisions.side_back2 import SideBack2Decision
from decisions.side_wing import SideWingDecision, WingID
from decisions.sub_attacker import SubAttackerDecision
from decisions.substitute import SubstituteDecision
from decisions.zone_defense import ZoneDefenseDecision, ZoneDefenseID
from field_observer import FieldObserver
from rclpy.executors import MultiThreadedExecutor
from referee_parser import RefereeParser
from robot_operator import RobotOperator
from role_assignment import RoleAssignment, RoleName


def num_of_active_zone_roles(active_roles):
    # アクティブなゾーンディフェンス担当者の数を返す
    role_zone_list = [
        RoleName.ZONE1,
        RoleName.ZONE2,
        RoleName.ZONE3,
        RoleName.ZONE4]
    return len(set(role_zone_list) & set(active_roles))


def enable_update_attacker_by_ball_pos():
    # アタッカーの切り替わりを防ぐため、
    # ボールが動いてたり、ディフェンスエリアや自ゴール側場外にあるときは役割を更新しない
    return not observer.ball_is_in_our_defense_area() and \
        not observer.ball_is_outside_back_x() and \
        not observer.ball_is_outside_front_x() and \
        not observer.ball_is_outside_right_y() and \
        not observer.ball_is_outside_left_y() and \
        not observer.ball_is_moving() and \
        not referee.our_ball_placement() and \
        not referee.our_pre_penalty() and \
        not referee.our_penalty() and \
        not referee.their_pre_penalty() and \
        not referee.their_penalty() and \
        not referee.their_ball_placement()


def main():
    while rclpy.ok():

        if referee.halt():
            # 各decisionsがセットしたNamedTargetを消去する
            operator.clear_named_targets()
            operator.publish_named_targets()

        ball_state = observer.get_ball_state()
        ball_placement_state = observer.get_ball_placement_state(referee.placement_position())
        ball_zone_state = observer.get_ball_zone_state()

        # ロボットの役割の更新する
        changed_ids = assignor.update_role(
            enable_update_attacker_by_ball_pos(),
            referee.max_allowed_our_bots())

        # 担当者がいるroleの中から、ゾーンディフェンスの数を抽出する
        assigned_roles = [t[0] for t in assignor.get_assigned_roles_and_ids()]
        num_of_zone_roles = num_of_active_zone_roles(assigned_roles)
        zone_targets = observer.update_zone_targets(num_of_zone_roles)

        for role, robot_id in assignor.get_assigned_roles_and_ids():
            # 役割が変わったロボットのみ、行動を更新する
            # 頻繁に行動を更新すると、controllerの負荷が高まり制御に遅延が発生します
            if robot_id in changed_ids:
                decisions[role].reset_act_id()

            # ボール状態をセットする
            decisions[role].set_ball_state(ball_state)
            # ボール配置状態をセットする
            decisions[role].set_ball_placement_state(ball_placement_state)
            # ボールゾーン状態をセットする
            decisions[role].set_ball_zone_state(ball_zone_state)
            # ゾーンディフェンスの担当者数をセットする
            decisions[role].set_num_of_zone_roles(num_of_zone_roles)
            # ゾーンディフェンスのターゲットをセットする
            decisions[role].set_zone_targets(zone_targets)
            # プレースメントを回避する
            decisions[role].enable_avoid_placement(robot_id)
            # 障害物を回避する
            decisions[role].enable_avoid_obstacles(robot_id)

            # レフェリーコマンドに合わせて行動を決定する
            if referee.halt():
                decisions[role].halt(robot_id)
            elif referee.stop():
                decisions[role].enable_stop_game_velocity(robot_id)
                decisions[role].stop(robot_id)
                decisions[role].disable_stop_game_velocity(robot_id)
            elif referee.inplay() or referee.force_start():
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
                decisions[role].enable_stop_game_velocity(robot_id)
                decisions[role].our_pre_penalty(robot_id)
                decisions[role].disable_stop_game_velocity(robot_id)

            elif referee.our_penalty():
                decisions[role].enable_stop_game_velocity(robot_id)
                decisions[role].our_penalty(robot_id)
                decisions[role].disable_stop_game_velocity(robot_id)

            elif referee.their_pre_penalty():
                decisions[role].enable_stop_game_velocity(robot_id)
                decisions[role].their_pre_penalty(robot_id)
                decisions[role].disable_stop_game_velocity(robot_id)

            elif referee.their_penalty():
                decisions[role].enable_stop_game_velocity(robot_id)
                decisions[role].their_penalty(robot_id)
                decisions[role].disable_stop_game_velocity(robot_id)

            elif referee.our_penalty_inplay():
                decisions[role].our_penalty_inplay(robot_id)

            elif referee.their_penalty_inplay():
                decisions[role].their_penalty_inplay(robot_id)

            elif referee.our_direct():
                decisions[role].enable_stop_game_velocity(robot_id)
                decisions[role].our_direct(robot_id)
                decisions[role].disable_stop_game_velocity(robot_id)
            elif referee.their_direct():
                decisions[role].enable_stop_game_velocity(robot_id)
                decisions[role].their_direct(robot_id)
                decisions[role].disable_stop_game_velocity(robot_id)
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
                print("UNDEFINED REFEREE COMMAND!!! : {}".format(referee.get_command()))


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
    args, other_args = arg_parser.parse_known_args()

    rclpy.init(args=other_args)

    operator = RobotOperator(args.yellow)
    assignor = RoleAssignment(args.goalie, args.yellow)
    referee = RefereeParser(args.yellow, args.invert)
    observer = FieldObserver(args.yellow)

    executor = MultiThreadedExecutor()
    executor.add_node(operator)
    executor.add_node(assignor)
    executor.add_node(referee)
    executor.add_node(observer)

    # エグゼキュータは別スレッドでspinさせ続ける
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    decisions = {
        RoleName.GOALIE: GoaleDecision(operator, observer),
        RoleName.ATTACKER: AttackerDecision(operator, observer),
        RoleName.CENTER_BACK1: CenterBack1Decision(operator, observer),
        RoleName.CENTER_BACK2: CenterBack2Decision(operator, observer),
        RoleName.SUB_ATTACKER: SubAttackerDecision(operator, observer),
        RoleName.ZONE1: ZoneDefenseDecision(operator, observer, ZoneDefenseID.ZONE1),
        RoleName.ZONE2: ZoneDefenseDecision(operator, observer, ZoneDefenseID.ZONE2),
        RoleName.ZONE3: ZoneDefenseDecision(operator, observer, ZoneDefenseID.ZONE3),
        RoleName.ZONE4: ZoneDefenseDecision(operator, observer, ZoneDefenseID.ZONE4),
        RoleName.SIDE_BACK1: SideBack1Decision(operator, observer),
        RoleName.SIDE_BACK2: SideBack2Decision(operator, observer),
        RoleName.LEFT_WING: SideWingDecision(operator, observer, WingID.LEFT),
        RoleName.RIGHT_WING: SideWingDecision(operator, observer, WingID.RIGHT),
        RoleName.SUBSTITUTE: SubstituteDecision(operator, observer, args.invert),
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
