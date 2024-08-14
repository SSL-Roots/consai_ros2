#!/usr/bin/env python3
# coding: UTF-8

import argparse
import threading

import rclpy
from rclpy.executors import MultiThreadedExecutor
from consai_examples.robot_operator import RobotOperator

from consai_examples.operation import OneShotOperation
from consai_examples.operation import TargetXY
from consai_examples.operation import TargetTheta

import math

def aligment(initial_x: float, initial_y:float, final_x: float, final_y: float):
    robots = range(5)
    target_point = []

    between_bots_x = 0.3    # meter
    between_bots_y = 0.3    # meter

    dist_x = final_x - initial_x
    dist_y = final_y - initial_y

    theta_target = math.degrees(math.atan2(dist_y, dist_x))

    next_target_x = [initial_x - between_bots_x * i * math.cos(theta_target) for i in range(len(robots))]
    next_target_y = [initial_y - between_bots_y * i * math.sin(theta_target) for i in range(len(robots))]

    for robot in range(len(robots)):
        target_point.append(OneShotOperation().move_to_pose(
            TargetXY.value(next_target_x[robot], next_target_y[robot]), 
            TargetTheta.value(0)))
        operator_node.operate(robots[robot], target_point[robot])

    span = 15

    delta_x = dist_x / span
    delta_y = dist_y / span

    next_target_x = [initial_x + 1 * delta_x - between_bots_x * i * math.cos(theta_target) for i in range(len(robots))]
    next_target_y = [initial_y + 1 * delta_y - between_bots_y * i * math.sin(theta_target) for i in range(len(robots))]

    for n in range(span + 1):
        next_target_x = [initial_x - n * delta_x - between_bots_x * i * math.cos(theta_target) for i in range(len(robots))]
        next_target_y = [initial_y - n * delta_y - between_bots_y * i * math.sin(theta_target) for i in range(len(robots))]

        for robot in range(len(robots)):
            target_point[robot] = (OneShotOperation().move_to_pose(
                TargetXY.value(next_target_x[robot], next_target_y[robot]), 
                TargetTheta.value(0)))
            operator_node.operate(robots[robot], target_point[robot])


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
    arg_parser.add_argument('-initial_x', type=float, default=-3.0) 
    arg_parser.add_argument('-initial_y', type=float, default=0.0)
    arg_parser.add_argument('-final_x', type=float, default=3.0)
    arg_parser.add_argument('-final_y', type=float, default=-1.0)
    args = arg_parser.parse_args()

    rclpy.init(args=None)

    operator_node = RobotOperator(target_is_yellow=args.yellow)

    # すべてのロボットの衝突回避を解除
    # for i in range(16):
    #     operator_node.disable_avoid_obstacles(i)

    executor = MultiThreadedExecutor()
    executor.add_node(operator_node)

    # エグゼキュータは別スレッドでspinさせ続ける
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    try:
        aligment(args.initial_x, args.initial_y, args.final_x, args.final_y)
    except KeyboardInterrupt:
        pass

    executor.shutdown()
    rclpy.shutdown()
