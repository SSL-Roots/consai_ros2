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

from consai_msgs.action import RobotControl
from functools import partial
import math
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.node import Node


class ControlTest(Node):

    def __init__(self):
        super().__init__('control_test_node')

        ROBOT_NUM = 16
        self._action_clients = []
        for i in range(ROBOT_NUM):
            action_name = 'blue' + str(i) + '/control'
            self._action_clients.append(ActionClient(self, RobotControl, action_name))
        self._robot_is_free = [True] * ROBOT_NUM
        self._send_goal_future = [None] * ROBOT_NUM
        self._get_result_future = [None] * ROBOT_NUM

    def robot_is_free(self, robot_id):
        return self._robot_is_free[robot_id]

    def all_robots_are_free(self):
        return all(self._robot_is_free)

    def move_robot(self, robot_id, x, y, theta):
        self.get_logger().info("move_robot start :" + str(robot_id))
        goal_msg = RobotControl.Goal()
        goal_msg.x.value = x
        goal_msg.y.value = y
        goal_msg.theta.value = theta

        if not self._action_clients[robot_id].wait_for_server(5):
            self.get_logger().error("TIMEOUT: wait_for_server")
            return False

        self._send_goal_future[robot_id] = self._action_clients[robot_id].send_goal_async(
            goal_msg, feedback_callback=partial(self._feedback_callback, robot_id=robot_id))

        self._send_goal_future[robot_id].add_done_callback(partial(self._goal_response_callback, robot_id=robot_id))
        self._robot_is_free[robot_id] = False

    def _goal_response_callback(self, future, robot_id):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        self._get_result_future[robot_id] = goal_handle.get_result_async()
        self._get_result_future[robot_id].add_done_callback(partial(self._get_result_callback, robot_id=robot_id))

    def _feedback_callback(self, feedback_msg, robot_id):
        feedback = feedback_msg.feedback

    def _get_result_callback(self, future, robot_id):
        result = future.result().result
        self.get_logger().info('RobotId: {0}, Result: {0}, Message: {0}'.format(robot_id, result.success, result.message))
        self._robot_is_free[robot_id] = True

def main():
    robot_id = 0
    for i in range(3):
        for i in range(16):
            test_node.move_robot(i, -4.0 + 0.2 * i, 2.0, math.pi * 0.5)

        while test_node.all_robots_are_free() is False:
            executor.spin_once()  # タイムアウト入れないとフリーズする

        for i in range(16):
            test_node.move_robot(i, -4.0 + 0.2 * i, -2.0, math.pi * 0.5)

        while test_node.all_robots_are_free() is False:
            executor.spin_once()  # タイムアウト入れないとフリーズする

if __name__ == '__main__':
    rclpy.init(args=None)

    test_node = ControlTest()

    executor = SingleThreadedExecutor()
    executor.add_node(test_node)

    try:
        main()
    except KeyboardInterrupt:
        pass

    executor.shutdown()
    rclpy.shutdown()
