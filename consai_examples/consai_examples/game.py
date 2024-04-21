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
from decisions.center_back import CenterBackDecision, CenterBackID
from decisions.goalie import GoaleDecision
from decisions.side_back import SideBackDecision, SideBackID
from decisions.side_wing import SideWingDecision, WingID
from decisions.sub_attacker import SubAttackerDecision
from decisions.substitute import SubstituteDecision
from decisions.zone_defense import ZoneDefenseDecision, ZoneDefenseID
from field_observer import FieldObserver
from rclpy.executors import MultiThreadedExecutor
from referee_parser import RefereeParser
from robot_operator import RobotOperator
from role_assignment import RoleAssignment, RoleName


def num_of_active_center_back_roles(active_roles):
    # アクティブなセンターバック担当者の数を返す
    role_zone_list = [
        RoleName.CENTER_BACK1,
        RoleName.CENTER_BACK2]
    return len(set(role_zone_list) & set(active_roles))


def num_of_active_side_back_roles(active_roles):
    # アクティブなサイドバック担当者の数を返す
    role_zone_list = [
        RoleName.SIDE_BACK1,
        RoleName.SIDE_BACK2]
    return len(set(role_zone_list) & set(active_roles))


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
    return not observer.ball_position().is_in_our_defense_area() and \
        not observer.ball_position().is_outside() and \
        not referee.our_ball_placement() and \
        not referee.our_pre_penalty() and \
        not referee.our_penalty() and \
        not referee.their_pre_penalty() and \
        not referee.their_penalty() and \
        not referee.their_ball_placement()
    # not observer.ball_motion().is_moving() and \


def update_decisions(changed_ids: list[int],
                     num_of_center_back_roles: int,
                     num_of_side_back_roles: int,
                     num_of_zone_roles: int):
    for role, robot_id in assignor.get_assigned_roles_and_ids():
        # 役割が変わったロボットのみ、行動を更新する
        if robot_id in changed_ids:
            decisions[role].reset_operation(robot_id)

        # センターバックの担当者数をセットする
        decisions[role].set_num_of_center_back_roles(num_of_center_back_roles)
        # サイドバックの担当者数をセットする
        decisions[role].set_num_of_side_back_roles(num_of_side_back_roles)
        # ゾーンディフェンスの担当者数をセットする
        decisions[role].set_num_of_zone_roles(num_of_zone_roles)

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
            decisions[role].our_pre_penalty(robot_id)
        elif referee.our_penalty():
            decisions[role].our_penalty(robot_id)
        elif referee.their_pre_penalty():
            decisions[role].their_pre_penalty(robot_id)
        elif referee.their_penalty():
            decisions[role].their_penalty(robot_id)
        elif referee.our_penalty_inplay():
            decisions[role].our_penalty_inplay(robot_id)
        elif referee.their_penalty_inplay():
            decisions[role].their_penalty_inplay(robot_id)
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
        elif referee.goal_blue():
            decisions[role].halt(robot_id)
        elif referee.goal_yellow():
            decisions[role].halt(robot_id)
        else:
            print("UNDEFINED REFEREE COMMAND!!! : {}".format(referee.get_command()))


def main():
    TARGET_PERIOD = 0.01  # 100Hz

    while rclpy.ok():
        start_time = time.time()

        if referee.halt():
            # 各decisionsがセットしたNamedTargetを消去する
            operator.clear_named_targets()
            operator.publish_named_targets()

        # ロボットの役割の更新する
        changed_ids = assignor.update_role(
            observer.detection().ball(),
            observer.detection().our_robots(),
            enable_update_attacker_by_ball_pos(),
            referee.max_allowed_our_bots())

        # 役割の描画情報を出力する
        assignor.publish_role_for_visualizer(observer.detection().our_robots())

        # 担当者がいるroleの中から、ゾーンディフェンスの数を抽出する
        assigned_roles = [t[0] for t in assignor.get_assigned_roles_and_ids()]
        num_of_center_back_roles = num_of_active_center_back_roles(assigned_roles)
        num_of_side_back_roles = num_of_active_side_back_roles(assigned_roles)
        num_of_zone_roles = num_of_active_zone_roles(assigned_roles)
        observer.set_num_of_zone_roles(num_of_zone_roles)

        update_decisions(changed_ids, num_of_center_back_roles,
                         num_of_side_back_roles, num_of_zone_roles)

        elapsed_time = time.time() - start_time
        if elapsed_time < TARGET_PERIOD:
            time.sleep(TARGET_PERIOD - elapsed_time)
        else:
            rclpy.logging.get_logger("game.py").warn(
                "Update took too long: {}".format(elapsed_time))


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
        RoleName.CENTER_BACK1: CenterBackDecision(operator, observer, CenterBackID.CENTER_BACK1),
        RoleName.CENTER_BACK2: CenterBackDecision(operator, observer, CenterBackID.CENTER_BACK2),
        RoleName.SUB_ATTACKER: SubAttackerDecision(operator, observer),
        RoleName.ZONE1: ZoneDefenseDecision(operator, observer, ZoneDefenseID.ZONE1),
        RoleName.ZONE2: ZoneDefenseDecision(operator, observer, ZoneDefenseID.ZONE2),
        RoleName.ZONE3: ZoneDefenseDecision(operator, observer, ZoneDefenseID.ZONE3),
        RoleName.ZONE4: ZoneDefenseDecision(operator, observer, ZoneDefenseID.ZONE4),
        RoleName.LEFT_WING: SideWingDecision(operator, observer, WingID.LEFT),
        RoleName.RIGHT_WING: SideWingDecision(operator, observer, WingID.RIGHT),
        RoleName.SIDE_BACK1: SideBackDecision(operator, observer, SideBackID.SIDE1),
        RoleName.SIDE_BACK2: SideBackDecision(operator, observer, SideBackID.SIDE2),
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
