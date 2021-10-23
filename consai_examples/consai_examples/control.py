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
import math
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.node import Node


class ControlTest(Node):

    def __init__(self):
        super().__init__('control_test_node')

        self._action_client = ActionClient(self, RobotControl, '/blue0/control')
        self._robot_is_free = True

    def robot_is_free(self):
        return self._robot_is_free

    def _goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self._get_result_callback)

    def _feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        # self.get_logger().info('remaining x:{}, y:{}, theta:{}'.format(
        #     feedback.remaining_x,
        #     feedback.remaining_y,
        #     feedback.remaining_theta,
        # ))

    def _get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}, Message: {0}'.format(result.success, result.message))
        self._robot_is_free = True

    def move_robot(self, x, y, theta):
        self.get_logger().info("move_robot start")
        goal_msg = RobotControl.Goal()
        goal_msg.x.value = x
        goal_msg.y.value = y
        goal_msg.theta.value = theta

        if not self._action_client.wait_for_server(5):
            self.get_logger().error("TIMEOUT: wait_for_server")
            return False

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self._feedback_callback)

        self._send_goal_future.add_done_callback(self._goal_response_callback)
        self._robot_is_free = False
        self.get_logger().info("move_robot stop")

def main():
    test_node.move_robot(2.0, 1.0, math.pi * 0.5)
    while test_node.robot_is_free() is False:
        executor.spin_once()  # タイムアウト入れないとフリーズする

    test_node.move_robot(2.0, -1.0, math.pi * 0.5)
    while test_node.robot_is_free() is False:
        executor.spin_once()

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
