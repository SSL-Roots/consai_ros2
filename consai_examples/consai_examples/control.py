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
from consai_msgs.msg import ConstraintObject 
from consai_msgs.msg import ConstraintPose
from consai_msgs.msg import ConstraintTheta
from consai_msgs.msg import ConstraintXY
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

    def move_to(self, robot_id, x, y, theta, keep=False):
        pose = ConstraintPose()

        pose.xy.value_x.append(x)
        pose.xy.value_y.append(y)
        pose.theta.value_theta.append(theta)

        goal_msg = RobotControl.Goal()
        goal_msg.pose.append(pose)
        goal_msg.keep_control = keep

        return self._set_goal(robot_id, goal_msg)

    def move_to_normalized(self, robot_id, x, y, theta, keep=False):
        pose = ConstraintPose()

        pose.xy.normalized = True
        pose.xy.value_x.append(x)
        pose.xy.value_y.append(y)
        pose.theta.value_theta.append(theta)

        goal_msg = RobotControl.Goal()
        goal_msg.pose.append(pose)
        goal_msg.keep_control = keep

        return self._set_goal(robot_id, goal_msg)

    def chase_ball(self, robot_id, offset_x, offset_y, offset_theta, look_from=False, keep=False):
        constraint_obj = ConstraintObject()
        constraint_obj.type = ConstraintObject.BALL

        pose = ConstraintPose()

        pose.xy.object.append(constraint_obj)
        pose.theta.object.append(constraint_obj)
        pose.theta.param = ConstraintTheta.PARAM_LOOK_TO
        if look_from:
            pose.theta.param = ConstraintTheta.PARAM_LOOK_FROM

        pose.offset.x = offset_x
        pose.offset.y = offset_y
        pose.offset.theta = offset_theta

        goal_msg = RobotControl.Goal()
        goal_msg.pose.append(pose)
        goal_msg.keep_control = keep

        return self._set_goal(robot_id, goal_msg)

    def chase_robot(self, robot_id, object_is_yellow, object_id, offset_x, offset_y, offset_theta, look_from=False, keep=False):
        constraint_obj = ConstraintObject()
        constraint_obj.robot_id = object_id
        constraint_obj.type = ConstraintObject.BLUE_ROBOT
        if object_is_yellow:
            constraint_obj.type = ConstraintObject.YELLOW_ROBOT

        pose = ConstraintPose()

        pose.xy.object.append(constraint_obj)
        pose.theta.object.append(constraint_obj)
        pose.theta.param = ConstraintTheta.PARAM_LOOK_TO
        if look_from:
            pose.theta.param = ConstraintTheta.PARAM_LOOK_FROM

        pose.offset.x = offset_x
        pose.offset.y = offset_y
        pose.offset.theta = offset_theta

        goal_msg = RobotControl.Goal()
        goal_msg.pose.append(pose)
        goal_msg.keep_control = keep

        return self._set_goal(robot_id, goal_msg)

    def shoot(self, robot_id, target_x, target_y):
        pose = ConstraintPose()

        pose.xy.value_x.append(0.0)
        pose.xy.value_y.append(0.0)

        constraint_obj = ConstraintObject()
        constraint_obj.type = ConstraintObject.BALL
        pose.theta.object.append(constraint_obj)
        pose.theta.param = ConstraintTheta.PARAM_LOOK_TO

        goal_msg = RobotControl.Goal()
        goal_msg.pose.append(pose)
        goal_msg.keep_control = True

        goal_msg.kick_shoot = True

        goal_msg.kick_target.normalized = True
        goal_msg.kick_target.value_x.append(target_x)
        goal_msg.kick_target.value_y.append(target_y)

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

def test_move_to():
    for i in range(16):
        test_node.move_to(i, -5.0 + 0.5 * i, 4.0, math.pi * 0.5, False)

    while test_node.all_robots_are_free() is False:
        executor.spin_once(1)  # タイムアウト入れないとフリーズする

    for i in range(16):
        test_node.move_to(i, -5.0 + 0.5 * i, -4.0, -math.pi * 0.5, True)

    while test_node.all_robots_are_free() is False:
        executor.spin_once(1)  # タイムアウト入れないとフリーズする

def test_move_to_normalized(divide_n=3):

    # フィールド端まで広がる
    size = 1.0
    # 0番はセンターに配置
    test_node.move_to_normalized(0, 0.0, 0.0, 0.0, False)
    # 1 ~ 8は時計回りに配置
    test_node.move_to_normalized(1, 0.0, size, 0.0, False)
    test_node.move_to_normalized(2, size, size, 0.0, False)
    test_node.move_to_normalized(3, size, 0.0, 0.0, False)
    test_node.move_to_normalized(4, size, -size, 0.0, False)
    test_node.move_to_normalized(5, 0.0, -size, 0.0, False)
    test_node.move_to_normalized(6, -size, -size, 0.0, False)
    test_node.move_to_normalized(7, -size, 0.0, 0.0, False)
    test_node.move_to_normalized(8, -size, size, 0.0, False)

    while test_node.all_robots_are_free() is False:
        executor.spin_once(1)  # タイムアウト入れないとフリーズする

    for i in range(1, divide_n):
        size = 1.0 / (i + 1)
        # 0番はセンターに配置
        test_node.move_to_normalized(0, 0.0, 0.0, 0.0, False)
        # 1 ~ 8は時計回りに配置
        test_node.move_to_normalized(1, 0.0, size, 0.0, False)
        test_node.move_to_normalized(2, size, size, 0.0, False)
        test_node.move_to_normalized(3, size, 0.0, 0.0, False)
        test_node.move_to_normalized(4, size, -size, 0.0, False)
        test_node.move_to_normalized(5, 0.0, -size, 0.0, False)
        test_node.move_to_normalized(6, -size, -size, 0.0, False)
        test_node.move_to_normalized(7, -size, 0.0, 0.0, False)
        test_node.move_to_normalized(8, -size, size, 0.0, False)
        while test_node.all_robots_are_free() is False:
            executor.spin_once(1)  # タイムアウト入れないとフリーズする

        # 番号をずらして回転
        test_node.move_to_normalized(8, 0.0, size, 0.0, False)
        test_node.move_to_normalized(1, size, size, 0.0, False)
        test_node.move_to_normalized(2, size, 0.0, 0.0, False)
        test_node.move_to_normalized(3, size, -size, 0.0, False)
        test_node.move_to_normalized(4, 0.0, -size, 0.0, False)
        test_node.move_to_normalized(5, -size, -size, 0.0, False)
        test_node.move_to_normalized(6, -size, 0.0, 0.0, False)
        test_node.move_to_normalized(7, -size, size, 0.0, False)
        while test_node.all_robots_are_free() is False:
            executor.spin_once(1)  # タイムアウト入れないとフリーズする

def test_chase_ball():
    # ボールの右側に、2次関数のように並ぶ
    for i in range(16):
        test_node.chase_ball(i, 0.2 + 0.2*i, 0.05 * i*i, 0.1 * math.pi * i, True, False)

    while test_node.all_robots_are_free() is False:
        executor.spin_once(1)  # タイムアウト入れないとフリーズする

    # ボールを見る
    for i in range(16):
        test_node.chase_ball(i, 0.2 + 0.2*i, 0.05 * i*i, 0.0, False, True)

    while test_node.all_robots_are_free() is False:
        executor.spin_once(1)  # タイムアウト入れないとフリーズする

def test_chase_robot():
    # 同じIDの黄色ロボットの左上に移動する
    for i in range(16):
        test_node.chase_robot(i, True, i, -0.2, 0.2, 0.0, True, False)

    while test_node.all_robots_are_free() is False:
        executor.spin_once(1)  # タイムアウト入れないとフリーズする

    # 左横に移動する
    for i in range(16):
        test_node.chase_robot(i, True, i, -0.2, 0.0, 0.0, False, True)

    while test_node.all_robots_are_free() is False:
        executor.spin_once(1)  # タイムアウト入れないとフリーズする

def test_for_config_pid(test_x=False, test_y=False, test_theta=False):
    # PID調整用
    # この関数を実行すると、ロボットが繰り返し動作するので、その裏でPIDゲインを調整すること
    # 左右
    if test_x:
        for x in [0.1, 0.3, 0.5, 0.7, 0.9]:
            for i in range(16):
                test_node.move_to_normalized(i, -x, 1.0 - 2.0 * i / 16.0, 0.0, False)

            while test_node.all_robots_are_free() is False:
                executor.spin_once(1)  # タイムアウト入れないとフリーズする

            for i in range(16):
                test_node.move_to_normalized(i, x, 1.0 - 2.0 * i / 16.0, 0.0, False)

            while test_node.all_robots_are_free() is False:
                executor.spin_once(1)  # タイムアウト入れないとフリーズする

    # 上下
    if test_y:
        for y in [0.1, 0.3, 0.5, 0.7, 0.9]:
            for i in range(16):
                test_node.move_to_normalized(i, -0.9 + 2.0 * i / 16.0, y, math.pi * 0.5, False)

            while test_node.all_robots_are_free() is False:
                executor.spin_once(1)  # タイムアウト入れないとフリーズする

            for i in range(16):
                test_node.move_to_normalized(i, -0.9 + 2.0 * i / 16.0, -y, math.pi * 0.5, False)

            while test_node.all_robots_are_free() is False:
                executor.spin_once(1)  # タイムアウト入れないとフリーズする

    if test_theta:
        for theta in [0.1, 0.3, 0.5, 0.7, 0.9]:
            for i in range(16):
                test_node.move_to_normalized(i, -0.9 + 2.0 * i / 16.0, 0.5, math.pi * theta, False)

            while test_node.all_robots_are_free() is False:
                executor.spin_once(1)  # タイムアウト入れないとフリーズする

            for i in range(16):
                test_node.move_to_normalized(i, -0.9 + 2.0 * i / 16.0, 0.5, -math.pi * theta, False)

            while test_node.all_robots_are_free() is False:
                executor.spin_once(1)  # タイムアウト入れないとフリーズする

def test_shoot(x, y):
    test_node.shoot(0, x, y)

    while test_node.all_robots_are_free() is False:
        executor.spin_once(1)  # タイムアウト入れないとフリーズする

def main():
    # test_move_to()
    # test_move_to_normalized(3)
    # test_chase_ball()
    # test_chase_robot()
    # test_for_config_pid(test_x=True)
    test_shoot(1.0, 0.0)

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
