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

"""レフェリーの指示に基づきロボットの役割を更新し, 各ロボットの動作を決定するプログラム."""

import argparse
import threading
import time

from consai_examples.decisions.attacker import AttackerDecision
from consai_examples.decisions.center_back import CenterBackDecision, CenterBackID
from consai_examples.decisions.goalie import GoaleDecision
from consai_examples.decisions.side_back import SideBackDecision, SideBackID
from consai_examples.decisions.side_wing import SideWingDecision, WingID
from consai_examples.decisions.sub_attacker import SubAttackerDecision, SubAttackerID
from consai_examples.decisions.substitute import SubstituteDecision
from consai_examples.decisions.zone_defense import ZoneDefenseDecision, ZoneDefenseID
from consai_examples.field_observer import FieldObserver
from consai_examples.referee_parser import RefereeParser
from consai_examples.robot_operator import RobotOperator
from consai_examples.role_assignment import RoleAssignment, RoleName

import rclpy
from rclpy.executors import MultiThreadedExecutor


def num_of_active_center_back_roles(active_roles):
    """アクティブなセンターバック担当者の数を返す関数."""
    role_zone_list = [RoleName.CENTER_BACK1, RoleName.CENTER_BACK2]
    return len(set(role_zone_list) & set(active_roles))


def num_of_active_side_back_roles(active_roles):
    """アクティブなサイドバック担当者の数を返す関数."""
    role_zone_list = [RoleName.SIDE_BACK1, RoleName.SIDE_BACK2]
    return len(set(role_zone_list) & set(active_roles))


def num_of_active_zone_roles(active_roles):
    """アクティブなゾーンディフェンス担当者の数を返す関数."""
    role_zone_list = [RoleName.ZONE1, RoleName.ZONE2, RoleName.ZONE3]
    return len(set(role_zone_list) & set(active_roles))


def zone_role_ids(aVctive_roles) -> list[int]:
    """ゾーンディンフェンスを担当するロボットIDのリストを返す関数."""
    ids = []
    for role, robot_id in assignor.get_assigned_roles_and_ids():
        if role in [RoleName.ZONE1, RoleName.ZONE2, RoleName.ZONE3]:
            ids.append(robot_id)
    return ids


def enable_role_update():
    """ロールの更新を許可する条件を返す関数."""
    return (
        not referee.our_pre_penalty()
        and not referee.our_penalty()
        and not referee.our_penalty_inplay()
        and not referee.our_ball_placement()
        and not referee.their_pre_penalty()
        and not referee.their_penalty()
        and not referee.their_penalty_inplay()
    )


def update_decisions(num_of_center_back_roles: int, num_of_side_back_roles: int, num_of_zone_roles: int):
    """各役割に対応する決定を更新する関数."""
    for role, robot_id in assignor.get_assigned_roles_and_ids():
        # センターバックの担当者数をセットする
        decisions[role].set_num_of_center_back_roles(num_of_center_back_roles)
        # サイドバックの担当者数をセットする
        decisions[role].set_num_of_side_back_roles(num_of_side_back_roles)
        # ゾーンディフェンスの担当者数をセットする
        decisions[role].set_num_of_zone_roles(num_of_zone_roles)
        # 現在のレフリーの経過時間をセットする
        decisions[role].set_command_elapsed_time(referee.command_elapsed_time())

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
            decisions[role].our_ball_placement(robot_id, referee.placement_position())
        elif referee.their_ball_placement():
            decisions[role].their_ball_placement(robot_id, referee.placement_position())
        elif referee.goal_blue():
            decisions[role].halt(robot_id)
        elif referee.goal_yellow():
            decisions[role].halt(robot_id)
        else:
            print("UNDEFINED REFEREE COMMAND!!! : {}".format(referee.get_command()))


def main():
    """メイン処理を開始する関数."""
    TARGET_PERIOD = 0.01  # 100Hz

    while rclpy.ok():
        start_time = time.time()

        if referee.halt():
            # 各decisionsがセットしたNamedTargetを消去する
            operator.clear_named_targets()
            operator.publish_named_targets()

        # ロボットの役割の更新する
        if enable_role_update():
            assignor.update_role(
                observer.detection().ball(), observer.detection().our_robots(), referee.max_allowed_our_bots()
            )

        # 担当者がいるroleの中から、ゾーンディフェンスの数を抽出する
        assigned_roles = [t[0] for t in assignor.get_assigned_roles_and_ids()]
        num_of_center_back_roles = num_of_active_center_back_roles(assigned_roles)
        num_of_side_back_roles = num_of_active_side_back_roles(assigned_roles)
        num_of_zone_roles = num_of_active_zone_roles(assigned_roles)
        observer.set_num_of_zone_roles(num_of_zone_roles)
        observer.man_mark().set_our_active_bot_ids(zone_role_ids(assigned_roles))

        update_decisions(num_of_center_back_roles, num_of_side_back_roles, num_of_zone_roles)

        # 役割情報を出力する
        assignor.publish_role_for_visualizer(observer.detection().our_robots())

        elapsed_time = time.time() - start_time
        if elapsed_time < TARGET_PERIOD:
            time.sleep(TARGET_PERIOD - elapsed_time)
        else:
            rclpy.logging.get_logger("game.py").warn("Update took too long: {}".format(elapsed_time))


if __name__ == "__main__":
    arg_parser = argparse.ArgumentParser()
    arg_parser.add_argument("--yellow", default=False, action="store_true", help="yellowロボットを動かす場合にセットする")
    arg_parser.add_argument("--invert", default=False, action="store_true", help="ball placementの目標座標を反転する場合にセットする")
    arg_parser.add_argument("--goalie", default=0, type=int, help="ゴールキーパのIDをセットする")
    args, other_args = arg_parser.parse_known_args()

    rclpy.init(args=other_args)

    operator = RobotOperator(args.yellow)
    assignor = RoleAssignment(args.goalie)
    referee = RefereeParser(args.yellow, args.invert)
    observer = FieldObserver(args.goalie, args.yellow)

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
        RoleName.SUB_ATTACKER1: SubAttackerDecision(operator, observer, SubAttackerID.SUBATTACK1),
        RoleName.SUB_ATTACKER2: SubAttackerDecision(operator, observer, SubAttackerID.SUBATTACK2),
        RoleName.ZONE1: ZoneDefenseDecision(operator, observer, ZoneDefenseID.ZONE1),
        RoleName.ZONE2: ZoneDefenseDecision(operator, observer, ZoneDefenseID.ZONE2),
        RoleName.ZONE3: ZoneDefenseDecision(operator, observer, ZoneDefenseID.ZONE3),
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
