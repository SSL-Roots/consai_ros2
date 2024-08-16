#!/usr/bin/env python3
# coding: UTF-8

import argparse
import threading

import rclpy
from rclpy.executors import MultiThreadedExecutor
from consai_examples.robot_operator import RobotOperator

from consai_examples.operation import Operation
from consai_examples.operation import TargetXY
from consai_examples.operation import TargetTheta
from consai_msgs.msg import State2D

import numpy as np
import copy
import time

def alignment(initial_x: float, initial_y:float):
    FIELD_MIN_X = -3.0
    FIELD_MAX_X = 3.0
    FIELD_MIN_Y = -1.0
    FIELD_MAX_Y = 1.0
    robots = [4,8,9]
    theta = 45

    between_bots_dist = 0.3    # meter

    target_hist = [State2D() for i in range(len(robots))]

    for i in range(len(robots)):
        target_hist[i].x = initial_x - (i * between_bots_dist)
        target_hist[i].y = initial_y

    while True:
        for i in range(len(robots)):
            operation = Operation().disable_avoid_defense_area()
            operation = operation.disable_avoid_our_robots()
            operation = operation.move_to_pose(TargetXY.value(target_hist[i].x, target_hist[i].y),
                                            TargetTheta.value(0))
            operator_node.operate(robots[i], operation)
        start_time = time.time()
        while time.time() - start_time < 0.5:
            pass
        
        front_pos = [copy.deepcopy(target_hist[i]) for i in range(len(robots) - 1)]
        for i in range(1, len(robots)):
            dx = front_pos[i-1].x - target_hist[i].x
            dy = front_pos[i-1].y - target_hist[i].y
            target_hist[i].x += dx
            target_hist[i].y += dy
        if ((FIELD_MIN_X <= target_hist[0].x <= FIELD_MAX_X) == False 
            or (FIELD_MIN_Y <= target_hist[0].y <= FIELD_MAX_Y) == False):
            theta += 120

        target_hist[0].x = target_hist[0].x + between_bots_dist * np.cos(theta)
        target_hist[0].y = target_hist[0].y + between_bots_dist * np.sin(theta)

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
    args = arg_parser.parse_args()

    rclpy.init(args=None)

    operator_node = RobotOperator(target_is_yellow=args.yellow)

    executor = MultiThreadedExecutor()
    executor.add_node(operator_node)

    # エグゼキュータは別スレッドでspinさせ続ける
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    try:
        alignment(args.initial_x, args.initial_y)
    except KeyboardInterrupt:
        pass

    executor.shutdown()
    rclpy.shutdown()
