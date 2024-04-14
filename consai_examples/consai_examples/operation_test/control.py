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
from consai_examples.field_observer import FieldObserver
from consai_examples.operation import OneShotOperation
from consai_examples.operation import Operation
from consai_examples.operation import TargetXY
from consai_examples.operation import TargetTheta
from consai_examples.referee_parser import RefereeParser
from consai_examples.robot_operator import RobotOperator
import rclpy
from rclpy.executors import MultiThreadedExecutor
import threading
import time


def move_to(args: argparse.Namespace) -> None:
    CYCLE_TIME = 5.0

    while True:
        operation = OneShotOperation().move_to_pose(
            TargetXY.value(args.x, args.y), TargetTheta.value(args.theta)
        )
        operator_node.operate(args.id, operation)
        time.sleep(CYCLE_TIME)

        operation = OneShotOperation().move_to_pose(
            TargetXY.value(-args.x, args.y), TargetTheta.value(args.theta)
        )
        operator_node.operate(args.id, operation)
        time.sleep(CYCLE_TIME)


def chase_ball(args: argparse.Namespace) -> None:
    operation = OneShotOperation().move_to_pose(
        TargetXY.ball(), TargetTheta.look_ball()
    )
    operation = operation.offset_pose_x(-0.2)
    operator_node.operate(args.id, operation)

    while operator_node.robot_is_free(args.id) is False:
        logger.info("Robot {} is running".format(args.id))
        time.sleep(1.0)


def wait_ball_and_shoot(args: argparse.Namespace) -> None:
    operation = Operation().move_to_pose(
        TargetXY.value(args.x, args.y), TargetTheta.look_their_goal()
    )
    operation = operation.with_shooting_to(TargetXY.their_goal())
    operator_node.operate(args.id, operation)

    logger.info("Robot {} keep moving after this script shutdown".format(args.id))


def receive_ball_and_shoot(args: argparse.Namespace) -> None:
    operation = Operation().move_to_pose(
        TargetXY.value(args.x, args.y), TargetTheta.look_their_goal()
    )
    operation = operation.with_shooting_to(TargetXY.their_goal())
    operation = operation.with_reflecting_to(TargetXY.their_goal())
    operator_node.operate(args.id, operation)

    logger.info("Robot {} keep moving after this script shutdown".format(args.id))


def pass_ball_between_robots(args: argparse.Namespace) -> None:
    ope1 = Operation().move_to_pose(TargetXY.value(-2.0, -1.0), TargetTheta.look_ball())
    ope1 = ope1.with_passing_to(TargetXY.our_robot(args.sub_id))
    operator_node.operate(args.id, ope1)

    ope2 = Operation().move_to_pose(TargetXY.value(2.0, 1.0), TargetTheta.look_ball())
    ope2 = ope2.with_passing_to(TargetXY.our_robot(args.id))
    operator_node.operate(args.sub_id, ope2)

    logger.info(
        "Robots {}, {} keep moving after this script shutdown".format(
            args.id, args.sub_id
        )
    )


def move_to_between_ball_and_center(args: argparse.Namespace) -> None:
    operation = OneShotOperation().move_on_line(
        TargetXY.ball(), TargetXY.value(0, 0), 0.5, TargetTheta.look_ball()
    )
    operator_node.operate(args.id, operation)

    while operator_node.robot_is_free(args.id) is False:
        logger.info("Robot {} is running".format(args.id))
        time.sleep(1.0)


def defend_our_goal(args: argparse.Namespace) -> None:
    p1_x = -5.5
    p1_y = 1.8
    p2_x = -5.5
    p2_y = -1.8
    operation = Operation().move_to_intersection(
        TargetXY.value(p1_x, p1_y),
        TargetXY.value(p2_x, p2_y),
        TargetXY.our_goal(),
        TargetXY.ball(),
        TargetTheta.look_ball(),
    )
    operation = operation.with_ball_receiving()
    operator_node.operate(args.id, operation)

    logger.info(
        "Robots {}, {} keep moving after this script shutdown".format(
            args.id, args.sub_id
        )
    )


def main():
    pass


if __name__ == "__main__":
    example_functions = {
        0: move_to,
        1: chase_ball,
        2: wait_ball_and_shoot,
        3: receive_ball_and_shoot,
        4: pass_ball_between_robots,
        5: move_to_between_ball_and_center,
        6: defend_our_goal,
    }
    examples_text = ""
    for key, value in example_functions.items():
        examples_text += "{}: {}\n".format(key, value.__name__)

    arg_parser = argparse.ArgumentParser(
        formatter_class=argparse.RawTextHelpFormatter,
    )
    arg_parser.add_argument(
        "--yellow",
        default=False,
        action="store_true",
        help="yellowロボットを動かす場合にセットする",
    )
    arg_parser.add_argument(
        "--invert",
        default=False,
        action="store_true",
        help="ball placementの目標座標を反転する場合にセットする",
    )
    arg_parser.add_argument("--id", default=0, type=int, help="ロボットID")
    arg_parser.add_argument("--sub_id", default=1, type=int, help="サブロボットID")
    arg_parser.add_argument("--x", default=0.0, type=float, help="目標座標x")
    arg_parser.add_argument("--y", default=0.0, type=float, help="目標座標y")
    arg_parser.add_argument("--theta", default=0.0, type=float, help="目標角度")
    arg_parser.add_argument(
        "--example", default=0, type=int, help="実行したい関数の番号\n" + examples_text
    )
    args = arg_parser.parse_args()

    rclpy.init(args=None)

    logger = rclpy.logging.get_logger("control.py")
    operator_node = RobotOperator(target_is_yellow=args.yellow)
    observer_node = FieldObserver(args.yellow)
    referee_node = RefereeParser(args.yellow, args.invert)

    executor = MultiThreadedExecutor()
    executor.add_node(operator_node)
    executor.add_node(observer_node)
    executor.add_node(referee_node)

    # エグゼキュータは別スレッドでspinさせ続ける
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    try:
        example_functions[args.example](args)
        main()
    except KeyboardInterrupt:
        pass

    executor.shutdown()
    rclpy.shutdown()
