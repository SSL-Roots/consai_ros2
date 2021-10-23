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
from consai_msgs.msg import ConstraintTarget
from functools import partial
import math
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.node import Node


class ControlTest(Node):

    def __init__(self, target_is_yellow=False):
        super().__init__('control_test_node')

        ROBOT_NUM = 16
        self._action_clients = []
        team_color = 'blue'
        if target_is_yellow:
            team_color = 'yellow'
        for i in range(ROBOT_NUM):
            action_name = team_color + str(i) + '/control'
            self._action_clients.append(ActionClient(self, RobotControl, action_name))
        self._robot_is_free = [True] * ROBOT_NUM
        self._send_goal_future = [None] * ROBOT_NUM
        self._get_result_future = [None] * ROBOT_NUM

    def robot_is_free(self, robot_id):
        return self._robot_is_free[robot_id]

    def all_robots_are_free(self):
        return all(self._robot_is_free)

    def move_robot(self, robot_id, x, y, theta, keep=False):
        goal_msg = RobotControl.Goal()
        goal_msg.x.value.append(x)
        goal_msg.y.value.append(y)
        goal_msg.theta.value.append(theta)
        goal_msg.keep_control = keep

        return self._set_goal(robot_id, goal_msg)

    def chase_ball(self, robot_id, offset_x, offset_y, look_from=False, keep=False):
        goal_msg = RobotControl.Goal()
        goal_msg.x.target.append(ConstraintTarget.TARGET_BALL)
        goal_msg.x.target_parameter.append(ConstraintTarget.PARAMETER_X)
        goal_msg.y.target.append(ConstraintTarget.TARGET_BALL)
        goal_msg.y.target_parameter.append(ConstraintTarget.PARAMETER_Y)
        goal_msg.theta.target.append(ConstraintTarget.TARGET_BALL)
        if look_from:
            goal_msg.theta.target_parameter.append(ConstraintTarget.PARAMETER_LOOK_FROM)
        else:
            goal_msg.theta.target_parameter.append(ConstraintTarget.PARAMETER_LOOK_TO)
        goal_msg.offset_x = offset_x
        goal_msg.offset_y = offset_y
        goal_msg.keep_control = keep

        return self._set_goal(robot_id, goal_msg)

    def chase_robot(self, robot_id, target_is_yellow, target_id, offset_x, offset_y, look_from=False, keep=False):
        goal_msg = RobotControl.Goal()
        if target_is_yellow:
            goal_msg.x.target.append(ConstraintTarget.TARGET_YELLOW_ROBOT)
            goal_msg.y.target.append(ConstraintTarget.TARGET_YELLOW_ROBOT)
            goal_msg.theta.target.append(ConstraintTarget.TARGET_YELLOW_ROBOT)
        else:
            goal_msg.x.target.append(ConstraintTarget.TARGET_BLUE_ROBOT)
            goal_msg.y.target.append(ConstraintTarget.TARGET_BLUE_ROBOT)
            goal_msg.theta.target.append(ConstraintTarget.TARGET_BLUE_ROBOT)
        goal_msg.x.target_id.append(target_id)
        goal_msg.y.target_id.append(target_id)
        goal_msg.theta.target_id.append(target_id)

        goal_msg.x.target_parameter.append(ConstraintTarget.PARAMETER_X)
        goal_msg.y.target_parameter.append(ConstraintTarget.PARAMETER_Y)
        if look_from:
            goal_msg.theta.target_parameter.append(ConstraintTarget.PARAMETER_LOOK_FROM)
        else:
            goal_msg.theta.target_parameter.append(ConstraintTarget.PARAMETER_LOOK_TO)
        goal_msg.offset_x = offset_x
        goal_msg.offset_y = offset_y
        goal_msg.keep_control = keep

        return self._set_goal(robot_id, goal_msg)

    def _set_goal(self, robot_id, goal_msg):
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
            self._robot_is_free[robot_id] = True
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
    for i in range(2):
        for i in range(16):
            test_node.chase_robot(i, True, i, 0.0, 0.5, False, True)
            # test_node.chase_ball(i, 0.4*(i+1), 0.4*(i+1), False, True)

        while test_node.all_robots_are_free() is False:
            executor.spin_once(1)  # タイムアウト入れないとフリーズする

if __name__ == '__main__':
    rclpy.init(args=None)

    test_node = ControlTest(False)

    executor = SingleThreadedExecutor()
    executor.add_node(test_node)

    try:
        main()
    except KeyboardInterrupt:
        pass

    executor.shutdown()
    rclpy.shutdown()
