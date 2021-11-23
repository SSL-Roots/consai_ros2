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

from functools import partial

from consai_msgs.action import RobotControl
from consai_msgs.msg import ConstraintLine
from consai_msgs.msg import ConstraintObject
from consai_msgs.msg import ConstraintPose
from consai_msgs.msg import ConstraintTheta
from consai_msgs.msg import ConstraintXY
from rclpy.action import ActionClient
from rclpy.node import Node


# consai_robot_controllerに指令を送るノード
class RobotOperator(Node):

    def __init__(self, target_is_yellow=False):
        super().__init__('operator')

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
        self._target_is_yellow = target_is_yellow

        if self._target_is_yellow:
            self.get_logger().info('yellowロボットを動かします')
        else:
            self.get_logger().info('blueロボットを動かします')

    def target_is_yellow(self):
        # 操作するロボットのチームカラーがyellowならtrue、blueならfalseを返す
        return self._target_is_yellow

    def robot_is_free(self, robot_id):
        # 指定されたIDのロボットが行動完了していたらtrue
        return self._robot_is_free[robot_id]

    def all_robots_are_free(self):
        # チームの全てのロボットが行動を完了していたらtrue
        return all(self._robot_is_free)

    def stop(self, robot_id):
        # 指定されたIDの制御を停止する
        goal_msg = RobotControl.Goal()
        goal_msg.stop = True
        return self._set_goal(robot_id, goal_msg)

    def move_to(self, robot_id, x, y, theta, keep=False, max_velocity_xy=None):
        # 指定したIDのロボットを目的地（x, y, theta）へ移動させる
        # 目的地に到達しても制御を維持する場合はkeepをtrue
        pose = ConstraintPose()

        pose.xy.value_x.append(x)
        pose.xy.value_y.append(y)
        pose.theta.value_theta.append(theta)

        goal_msg = RobotControl.Goal()
        goal_msg.pose.append(pose)
        goal_msg.keep_control = keep

        # 最大走行速度を制限する
        if max_velocity_xy:
            goal_msg.max_velocity_xy.append(max_velocity_xy)

        return self._set_goal(robot_id, goal_msg)

    def move_to_line(self, robot_id, p1_x, p1_y, p2_x, p2_y, distance, theta, keep=False):
        # 指定したIDのロボットを直線上へ移動させる
        line = ConstraintLine()

        line.p1.value_x.append(p1_x)
        line.p1.value_y.append(p1_y)
        line.p2.value_x.append(p2_x)
        line.p2.value_y.append(p2_y)
        line.distance = distance
        line.theta.value_theta.append(theta)

        goal_msg = RobotControl.Goal()
        goal_msg.line.append(line)
        goal_msg.keep_control = keep

        return self._set_goal(robot_id, goal_msg)

    def move_to_line_to_defend_our_goal(self, robot_id, p1_x, p1_y, p2_x, p2_y, keep=False):
        # 自チームのゴールをボールから守るように、直線p1->p2に移動する
        line = ConstraintLine()

        # 直線p1->p2を作成
        line.p1.value_x.append(p1_x)
        line.p1.value_y.append(p1_y)
        line.p2.value_x.append(p2_x)
        line.p2.value_y.append(p2_y)

        # 自チームのゴールとボールを結ぶ直線p3->p4を作成
        p3 = ConstraintXY()
        p3.normalized = True
        p3.value_x.append(-1.0)
        p3.value_y.append(0.0)

        p4 = ConstraintXY()
        obj_ball = ConstraintObject()
        obj_ball.type = ConstraintObject.BALL
        p4.object.append(obj_ball)

        line.p3.append(p3)
        line.p4.append(p4)
        line.theta.object.append(obj_ball)
        line.theta.param = ConstraintTheta.PARAM_LOOK_TO

        goal_msg = RobotControl.Goal()
        goal_msg.line.append(line)
        goal_msg.keep_control = keep

        return self._set_goal(robot_id, goal_msg)

    def move_to_defend_our_goal_from_ball(self, robot_id, distance):
        # 自チームとボールを結び、ボールからdistanceだけ離れた位置に移動する
        line = ConstraintLine()
        line.p1.object.append(self._object_ball())
        line.p2 = self._xy_our_goal()
        line.distance = distance
        line.theta = self._theta_look_ball()
        return self._set_line_goal(robot_id, line)

    def move_to_normalized(self, robot_id, x, y, theta, keep=False):
        # 指定したIDのロボットを目的地（x, y, theta）へ移動させる
        # 目的地の座標(x, y)は-1.0 ~ 1.0に正規化されている
        # フィールドサイズを気にせずにロボットを動かせるので便利
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
        # 指定したIDのロボットをボール直近へ移動させる
        # 目標角度は、ボールを見る向きになる
        # offset_x, offset_yで、ボールからどれだけ離れるのかを指定できる
        # offset_thetaは角度のオフセット値
        # look_fromがtrueで、目標角度がボールからロボットを見る向きになる
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

    def chase_robot(self, robot_id, object_is_yellow, object_id, offset_x, offset_y,
                    offset_theta, look_from=False, keep=False):
        # 指定したIDのロボットをロボット直近へ移動させる
        # 追跡対象のロボットはobject_is_yellowでチームカラーが、object_idでIDが指定される
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

    def shoot(self, robot_id, x, y, target_x, target_y):
        # 指定したIDのロボットにボールを蹴らせる
        # 座標(x, y)に移動した後、ロボット付近にボールが近づいたら
        # ターゲット座標（target_x, target_y）に向かってボールを蹴る
        # 目標座標に到達しても制御を維持する
        pose = ConstraintPose()

        pose.xy.value_x.append(x)
        pose.xy.value_y.append(y)

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

    def kick_pass(self, robot_id, target_id, x, y):
        # 指定したIDのロボットが、目標ロボットに対してボールをパスする
        # 座標(x, y)に移動した後、ロボット付近にボールが近づいたら
        # 同じチームの指定されたIDのロボットに向かってボールを蹴る
        # 目標座標に到達しても制御を維持する
        constraint_ball = ConstraintObject()
        constraint_ball.type = ConstraintObject.BALL

        pose = ConstraintPose()
        pose.xy.value_x.append(x)
        pose.xy.value_y.append(y)
        pose.theta.object.append(constraint_ball)  # ボールを見る
        pose.theta.param = ConstraintTheta.PARAM_LOOK_TO

        goal_msg = RobotControl.Goal()
        goal_msg.pose.append(pose)
        goal_msg.keep_control = True

        goal_msg.receive_ball = True

        # targetロボットを狙ってキックする
        goal_msg.kick_shoot = True
        constraint_robot = ConstraintObject()
        constraint_robot.robot_id = target_id
        constraint_robot.type = ConstraintObject.BLUE_ROBOT
        if self._target_is_yellow:
            constraint_robot.type = ConstraintObject.YELLOW_ROBOT
        goal_msg.kick_target.object.append(constraint_robot)

        return self._set_goal(robot_id, goal_msg)

    def _object_ball(self):
        # ConstraintObjectのBallを返す
        obj_ball = ConstraintObject()
        obj_ball.type = ConstraintObject.BALL
        return obj_ball

    def _xy_our_goal(self):
        # ConstraintXYの自チームゴール座標を返す
        our_goal = ConstraintXY()
        our_goal.normalized = True
        our_goal.value_x.append(-1.0)
        our_goal.value_y.append(0.0)
        return our_goal

    def _theta_look_ball(self):
        # ConstraintThetaでボールを見る角度を返す
        look_ball = ConstraintTheta()
        look_ball.object.append(self._object_ball())
        look_ball.param = ConstraintTheta.PARAM_LOOK_TO
        return look_ball

    def _set_line_goal(self, robot_id, line, keep=True):
        # ConstraintLineのアクションを設定する
        goal_msg = RobotControl.Goal()
        goal_msg.line.append(line)
        goal_msg.keep_control = keep
        return self._set_goal(robot_id, goal_msg)

    def _set_goal(self, robot_id, goal_msg):
        # アクションのゴールを設定する
        if not self._action_clients[robot_id].wait_for_server(5):
            self.get_logger().error('TIMEOUT: wait_for_server')
            return False

        self._send_goal_future[robot_id] = self._action_clients[robot_id].send_goal_async(
            goal_msg, feedback_callback=partial(self._feedback_callback, robot_id=robot_id))

        self._send_goal_future[robot_id].add_done_callback(
            partial(self._goal_response_callback, robot_id=robot_id))
        self._robot_is_free[robot_id] = False

    def _goal_response_callback(self, future, robot_id):
        # アクションサーバへゴールを送信した後の応答を受信したら実行される関数
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().debug('Goal rejected')
            self._robot_is_free[robot_id] = True
            return

        self.get_logger().debug('Goal accepted')
        self._get_result_future[robot_id] = goal_handle.get_result_async()
        self._get_result_future[robot_id].add_done_callback(
            partial(self._get_result_callback, robot_id=robot_id))

    def _feedback_callback(self, feedback_msg, robot_id):
        # アクションサーバからのフィードバックを受信したら実行される関数
        feedback = feedback_msg.feedback
        (feedback)

    def _get_result_callback(self, future, robot_id):
        # アクションサーバからの行動完了通知を受信したら実行される関数
        result = future.result().result
        self.get_logger().debug('RobotId: {}, Result: {}, Message: {}'.format(
            robot_id, result.success, result.message))
        self._robot_is_free[robot_id] = True
