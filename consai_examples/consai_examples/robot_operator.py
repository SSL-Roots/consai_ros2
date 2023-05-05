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
import math

from consai_msgs.action import RobotControl
from consai_msgs.msg import ConstraintLine
from consai_msgs.msg import ConstraintObject
from consai_msgs.msg import ConstraintPose
from consai_msgs.msg import ConstraintTheta
from consai_msgs.msg import ConstraintXY
from consai_msgs.msg import NamedTargets
from consai_msgs.msg import State2D
from rclpy.action import ActionClient
from rclpy.node import Node


# consai_robot_controllerに指令を送るノード
class RobotOperator(Node):

    STOP_GAME_VELOCITY = 0.8  # m/s
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
        self._stop_game_velocity_has_enabled = [False] * ROBOT_NUM
        self._avoid_obstacles_enabled = [True] * ROBOT_NUM
        self._avoid_placement_enabled = [True] * ROBOT_NUM

        # 名前付きターゲット格納用の辞書
        # データを扱いやすくするため、NamedTargets型ではなく辞書型を使用する
        self._named_targets = {}
        self._pub_named_targets = self.create_publisher(NamedTargets, 'named_targets', 1)

        if self._target_is_yellow:
            self.get_logger().info('yellowロボットを動かします')
        else:
            self.get_logger().info('blueロボットを動かします')

    def enable_stop_game_velocity(self, robot_id):
        self._stop_game_velocity_has_enabled[robot_id] = True

    def disable_stop_game_velocity(self, robot_id):
        self._stop_game_velocity_has_enabled[robot_id] = False

    def enable_avoid_obstacles(self, robot_id):
        self._avoid_obstacles_enabled[robot_id] = True

    def disable_avoid_obstacles(self, robot_id):
        self._avoid_obstacles_enabled[robot_id] = False

    def enable_avoid_placement(self, robot_id):
        self._avoid_placement_enabled[robot_id] = True

    def disable_avoid_placement(self, robot_id):
        self._avoid_placement_enabled[robot_id] = False

    def target_is_yellow(self):
        # 操作するロボットのチームカラーがyellowならtrue、blueならfalseを返す
        return self._target_is_yellow

    def robot_is_free(self, robot_id):
        # 指定されたIDのロボットが行動完了していたらtrue
        return self._robot_is_free[robot_id]

    def all_robots_are_free(self):
        # チームの全てのロボットが行動を完了していたらtrue
        return all(self._robot_is_free)

    def set_named_target(self, name, x, y, theta=0.0):
        # 名前付きターゲットをセットする
        # すでに同じ名前のターゲットが用意されていても上書きする
        self._named_targets[name] = State2D(x=x, y=y, theta=theta)

    def remove_named_target(self, name):
        # 指定した名前付きターゲットを削除する
        if name in self._named_targets:
            self._named_targets.pop(name)

    def clear_named_targets(self):
        # 名前付きターゲットを初期化する
        self._named_targets.clear()

    def publish_named_targets(self):
        # 名前付きターゲットを送信する
        msg = NamedTargets()
        for name, pose in self._named_targets.items():
            msg.name.append(name)
            msg.pose.append(pose)
        self._pub_named_targets.publish(msg)

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

    def move_to_look_ball(self, robot_id, x, y):
        # x, y座標に移動し、ボールを眺める
        pose = ConstraintPose()
        pose.xy.value_x.append(x)
        pose.xy.value_y.append(y)
        pose.theta = self._theta_look_ball()
        return self._set_goal(robot_id, self._pose_goal(pose, keep=True))

    def move_to_receive(self, robot_id, x, y):
        # x, y座標に移動する
        # ボールが来たら受け取りに移動する
        pose = ConstraintPose()
        pose.xy.value_x.append(x)
        pose.xy.value_y.append(y)
        pose.theta = self._theta_look_ball()
        return self._set_goal(robot_id, self._with_receive(self._pose_goal(pose, keep=True)))

    def move_to_reflect_shoot_to_their_goal(self, robot_id, x, y):
        # x, y座標に移動する
        # ボールが来たら相手ゴールに向かってリフレクトシュート
        pose = ConstraintPose()
        pose.xy.value_x.append(x)
        pose.xy.value_y.append(y)
        pose.theta = self._theta_look_ball()

        target = self._xy_their_goal()
        return self._set_goal(
            robot_id, self._with_reflect_kick(
                self._pose_goal(pose, keep=True), target, kick_pass=False))

    def move_to_reflect_shoot_to_our_robot(self, robot_id, target_id, x, y):
        # x, y座標に移動する
        # ボールが来たら相手ゴールに向かってリフレクトシュート
        pose = ConstraintPose()
        pose.xy.value_x.append(x)
        pose.xy.value_y.append(y)
        pose.theta = self._theta_look_ball()

        target = ConstraintXY()
        target.object.append(self._object_our_robot(target_id))
        return self._set_goal(
            robot_id, self._with_reflect_kick(
                self._pose_goal(pose, keep=True), target, kick_pass=False))

    def move_to_ball_x(self, robot_id, y, offset_x=0.0):
        # ボールと同じx軸上でyの位置に移動する
        pose = ConstraintPose()
        pose.xy.object.append(self._object_ball())
        pose.xy.value_x.append(offset_x)
        pose.xy.value_y.append(y)
        pose.theta = self._theta_look_ball()
        return self._set_goal(robot_id, self._with_receive(self._pose_goal(pose, keep=True)))

    def move_to_ball_x_with_reflect(self, robot_id, y, offset_x=0.0):
        # ボールと同じx軸上でyの位置に移動する
        pose = ConstraintPose()
        pose.xy.object.append(self._object_ball())
        pose.xy.value_x.append(offset_x)
        pose.xy.value_y.append(y)
        pose.theta = self._theta_look_ball()

        target = self._xy_their_goal()
        return self._set_goal(robot_id, self._with_reflect_kick(self._pose_goal(pose, keep=True), target, kick_pass=False))

    def move_to_ball_y(self, robot_id, x):
        # ボールと同じy軸上でxの位置に移動する
        pose = ConstraintPose()
        pose.xy.object.append(self._object_ball())
        pose.xy.value_x.append(x)
        pose.theta = self._theta_look_ball()
        return self._set_goal(robot_id, self._with_receive(self._pose_goal(pose, keep=True)))

    def move_to_line_to_defend_our_goal(self, robot_id, p1_x, p1_y, p2_x, p2_y):
        # 自チームのゴールをボールから守るように、直線p1->p2に移動する
        line = ConstraintLine()

        # 直線p1->p2を作成
        line.p1 = self._xy(p1_x, p1_y)
        line.p2 = self._xy(p2_x, p2_y)

        # 自チームのゴールとボールを結ぶ直線p3->p4を作成
        line.p3.append(self._xy_our_goal())
        line.p4.append(self._xy_object_ball())
        line.theta = self._theta_look_ball()
        return self._set_goal(robot_id, self._with_receive(self._line_goal(line, keep=True)))

    def move_to_line_to_defend_our_goal_with_reflect(self, robot_id, p1_x, p1_y, p2_x, p2_y):
        # 自チームのゴールをボールから守るように、直線p1->p2に移動する
        line = ConstraintLine()

        # 直線p1->p2を作成
        line.p1 = self._xy(p1_x, p1_y)
        line.p2 = self._xy(p2_x, p2_y)

        # 自チームのゴールとボールを結ぶ直線p3->p4を作成
        line.p3.append(self._xy_our_goal())
        line.p4.append(self._xy_object_ball())
        line.theta = self._theta_look_ball()

        target = self._xy_their_goal()
        return self._set_goal(robot_id, self._with_reflect_kick(self._line_goal(line, keep=True), target, kick_pass=False))

    def move_to_cross_line_their_center_and_ball(self, robot_id, p1_x, p1_y, p2_x, p2_y):
        # 直線p1->p2と
        # 相手サイドの中心とボールを結ぶ直線が交差する点で、ボールを見る
        line = ConstraintLine()

        # 直線p1->p2を作成
        line.p1 = self._xy(p1_x, p1_y)
        line.p2 = self._xy(p2_x, p2_y)

        line.p3.append(self._xy_their_side_center())
        line.p4.append(self._xy_object_ball())
        line.theta = self._theta_look_ball()
        return self._set_goal(robot_id, self._with_receive(self._line_goal(line, keep=True)))

    def move_to_cross_line_their_center_and_ball_with_reflect(self, robot_id, p1_x, p1_y, p2_x, p2_y):
        # 直線p1->p2と
        # 相手サイドの中心とボールを結ぶ直線が交差する点で、ボールを見る
        line = ConstraintLine()

        # 直線p1->p2を作成
        line.p1 = self._xy(p1_x, p1_y)
        line.p2 = self._xy(p2_x, p2_y)

        line.p3.append(self._xy_their_side_center())
        line.p4.append(self._xy_object_ball())
        line.theta = self._theta_look_ball()
        target = self._xy_their_goal()
        return self._set_goal(robot_id, self._with_reflect_kick(self._line_goal(line, keep=True), target, kick_pass=False))
    
    def move_to_cross_line_our_center_and_ball(self, robot_id, p1_x, p1_y, p2_x, p2_y):
        # 直線p1->p2と
        # 自分サイドの中心とボールを結ぶ直線が交差する点で、ボールを見る
        line = ConstraintLine()

        # 直線p1->p2を作成
        line.p1 = self._xy(p1_x, p1_y)
        line.p2 = self._xy(p2_x, p2_y)

        line.p3.append(self._xy_our_side_center())
        line.p4.append(self._xy_object_ball())
        line.theta = self._theta_look_ball()
        return self._set_goal(robot_id, self._with_receive(self._line_goal(line, keep=True)))

    def move_to_cross_line_our_center_and_ball_with_reflect(self, robot_id, p1_x, p1_y, p2_x, p2_y):
        # 直線p1->p2と
        # 自分サイドの中心とボールを結ぶ直線が交差する点で、ボールを見る
        line = ConstraintLine()

        # 直線p1->p2を作成
        line.p1 = self._xy(p1_x, p1_y)
        line.p2 = self._xy(p2_x, p2_y)

        line.p3.append(self._xy_our_side_center())
        line.p4.append(self._xy_object_ball())
        line.theta = self._theta_look_ball()
        target = self._xy_their_goal()
        return self._set_goal(robot_id, self._with_reflect_kick(self._line_goal(line, keep=True), target, kick_pass=False))

    def move_to_defend_our_goal_from_ball(self, robot_id, distance):
        # 自チームのゴールとボールを結び、ボールからdistanceだけ離れた位置に移動する
        line = ConstraintLine()
        line.p1.object.append(self._object_ball())
        line.p2 = self._xy_our_goal()
        line.distance = distance
        line.theta = self._theta_look_ball()
        return self._set_goal(robot_id, self._with_receive(self._line_goal(line, keep=True)))

    def man_mark(self, robot_id, their_id, distance):
        # ボールと敵チームロボットの間に入る
        # 敵ロボットからdistanceだけ離れた位置に移動する
        line = ConstraintLine()
        line.p1.object.append(self._object_their_robot(their_id))
        line.p2.object.append(self._object_ball())
        line.distance = distance
        line.theta = self._theta_look_ball()
        return self._set_goal(robot_id, self._with_receive(self._line_goal(line, keep=True)))

    def move_to_named_target(self, robot_id, name, keep=False):
        # 指定したIDのロボットをnameで指定した名前付きターゲットへ移動させる
        pose = ConstraintPose()
        pose.xy = self._xy_object_named_target(name)
        pose.theta = self._theta_object_named_target(name)

        goal_msg = RobotControl.Goal()
        goal_msg.pose.append(pose)
        goal_msg.keep_control = keep

        return self._set_goal(robot_id, goal_msg)

    def move_to_named_target_with_reflect_pass_to_named_target(
        self, robot_id, name_to_move, name_to_pass):
        # 指定したIDのロボットをnameで指定した名前付きターゲットへ移動させる
        # ボールが来たらリフレクトパスを実施する

        pose = ConstraintPose()
        pose.xy = self._xy_object_named_target(name_to_move)
        pose.theta = self._theta_object_named_target(name_to_move)

        goal_msg = RobotControl.Goal()
        goal_msg.pose.append(pose)
        goal_msg.keep_control = True

        goal_msg = self._with_receive(
            self._with_reflect_and_normal_kick(
                goal_msg, self._xy_object_named_target(name_to_pass), kick_pass=True))

        return self._set_goal(robot_id, goal_msg)

    def shoot_to(self, robot_id, x, y):
        # ボールの後ろに移動し、指定位置に向かってシュートする
        return self._act_to_ball(robot_id, self._xy(x, y), do_shoot=True)

    def shoot_to_their_goal(self, robot_id):
        # ボールの後ろに移動し、相手ゴールに向かってシュートする
        return self._act_to_ball(robot_id, self._xy_their_goal(), do_shoot=True)

    def shoot_to_named_target(self, robot_id, name):
        # ボールの後ろに移動し、名前付きターゲットに向かってシュートする
        return self._act_to_ball(robot_id, self._xy_object_named_target(name), do_shoot=True)

    def shoot_to_their_goal_with_reflect(self, robot_id):
        # ボールの後ろに移動し、相手ゴールに向かってシュートする
        # ボールが自分に向かって転がってきた場合はリフレクトシュートする
        return self._act_to_ball(robot_id, self._xy_their_goal(), do_reflect_shoot=True)

    def shoot_to_their_corner(self, robot_id, target_is_top_corner=True, set_play=False):
        # ボールの後ろに移動し、相手コーナーに向かってシュートする
        xy_target = self._xy_their_bottom_corner()
        if target_is_top_corner:
            xy_target = self._xy_their_top_corner()

        return self._act_to_ball(robot_id, xy_target, do_shoot=True, as_setplay=set_play)

    def setplay_shoot_to_their_goal(self, robot_id):
        # ボールの後ろに移動し、セットプレイのように慎重に、相手ゴールに向かってシュートする
        return self._act_to_ball(robot_id, self._xy_their_goal(), do_shoot=True, as_setplay=True)

    def dribble_to(self, robot_id, x, y):
        # ボールの後ろに移動し、指定位置に向かってドリブルする
        return self._act_to_ball(robot_id, self._xy(x, y), do_dribble=True)

    def receive_from(self, robot_id, x, y, offset, dynamic_receive=True):
        # ボールと指定位置(x, y)を結ぶ直線上で、指定位置からoffsetだけ後ろに下がり、
        # ボールを受け取る
        line = ConstraintLine()
        line.p1 = self._xy(x, y)
        line.p2.object.append(self._object_ball())
        line.distance = -offset
        line.theta = self._theta_look_ball()
        if dynamic_receive:
            return self._set_goal(robot_id, self._with_receive(
                self._line_goal(line, keep=True)))
        else:
            return self._set_goal(robot_id, self._line_goal(line, keep=True))

    def approach_to_ball(self, robot_id, distance):
        # 自分とボールを結ぶ直線上で、ボールからdistanceだけ離れた位置に移動する
        # 最終到達位置は、自分の初期位置に依存する
        line = ConstraintLine()
        line.p1.object.append(self._object_ball())
        line.p2.object.append(self._object_our_robot(robot_id))
        line.distance = distance
        line.theta = self._theta_look_ball()
        return self._set_goal(robot_id, self._line_goal(line, keep=True))

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

        goal_msg.kick_enable = True

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
        goal_msg.kick_enable = True
        constraint_robot = ConstraintObject()
        constraint_robot.robot_id = target_id
        constraint_robot.type = ConstraintObject.BLUE_ROBOT
        if self._target_is_yellow:
            constraint_robot.type = ConstraintObject.YELLOW_ROBOT
        goal_msg.kick_target.object.append(constraint_robot)

        return self._set_goal(robot_id, goal_msg)

    def pass_to(self, robot_id, x, y):
        # ボールの後ろに移動し、指定位置に向かってパスする
        return self._act_to_ball(robot_id, self._xy(x, y), do_pass=True)

    def pass_to_our_robot(self, robot_id, target_id):
        # ボールの後ろに移動し、指定したIDのロボットが、指定したIDの目標ロボットに対してボールをパスする
        return self._act_to_ball(robot_id, self._xy_object_our_robot(target_id), do_pass=True)

    def _object_ball(self):
        # ConstraintObjectのBallを返す
        obj_ball = ConstraintObject()
        obj_ball.type = ConstraintObject.BALL
        return obj_ball

    def _xy_object_ball(self):
        # ConstraintObjectのBallを返す
        xy = ConstraintXY()
        xy.object.append(self._object_ball())
        return xy

    def _xy_object_our_robot(self, robot_id):
        # ConstraintObjectのBallを返す
        xy = ConstraintXY()
        xy.object.append(self._object_our_robot(robot_id))
        return xy

    def _object_named_target(self, name):
        # ConstraintObjectのNamedTargetを返す
        obj = ConstraintObject()
        obj.type = ConstraintObject.NAMED_TARGET
        obj.name = name
        return obj

    def _xy_object_named_target(self, name):
        # NamedTargetを適用したConstraintXYを返す
        xy = ConstraintXY()
        xy.object.append(self._object_named_target(name))
        return xy

    def _theta_object_named_target(self, name):
        # NamedTargetを適用したConstraintThetaを返す
        theta = ConstraintTheta()
        theta.object.append(self._object_named_target(name))
        theta.param = ConstraintTheta.PARAM_THETA
        return theta

    def _object_our_robot(self, robot_id):
        # ConstraintObjectの自チームのRobotを返す
        obj_robot = ConstraintObject()
        obj_robot.robot_id = robot_id
        obj_robot.type = ConstraintObject.BLUE_ROBOT
        if self._target_is_yellow:
            obj_robot.type = ConstraintObject.YELLOW_ROBOT
        return obj_robot

    def _object_their_robot(self, robot_id):
        # ConstraintObjectの相手チームのRobotを返す
        obj_robot = ConstraintObject()
        obj_robot.robot_id = robot_id
        obj_robot.type = ConstraintObject.BLUE_ROBOT
        if not self._target_is_yellow:
            obj_robot.type = ConstraintObject.YELLOW_ROBOT
        return obj_robot

    def _xy(self, x, y, normalized=False):
        # x, y座標を指定したConstraintXYを返す
        xy = ConstraintXY()
        xy.value_x.append(x)
        xy.value_y.append(y)
        xy.normalized = normalized
        return xy

    def _xy_our_goal(self):
        # ConstraintXYの自チームゴール座標を返す
        our_goal = ConstraintXY()
        our_goal.normalized = True
        our_goal.value_x.append(-1.0)
        our_goal.value_y.append(0.0)
        return our_goal

    def _xy_their_goal(self):
        # ConstraintXYの相手チームゴール座標を返す
        our_goal = ConstraintXY()
        our_goal.normalized = True
        our_goal.value_x.append(1.0)
        our_goal.value_y.append(0.0)
        return our_goal

    def _xy_their_top_corner(self):
        # ConstraintXYの相手サイドのコーナー上側座標を返す
        our_goal = ConstraintXY()
        our_goal.normalized = True
        our_goal.value_x.append(1.0)
        our_goal.value_y.append(1.0)
        return our_goal

    def _xy_their_bottom_corner(self):
        # ConstraintXYの相手サイドのコーナー上側座標を返す
        our_goal = ConstraintXY()
        our_goal.normalized = True
        our_goal.value_x.append(1.0)
        our_goal.value_y.append(-1.0)
        return our_goal

    def _xy_their_side_center(self):
        # ConstraintXYの相手サイドの中央を返す
        their_center = ConstraintXY()
        their_center.normalized = True
        their_centeposer.value_x.append(0.5)
        their_center.value_y.append(0.0)
        return their_center
    
    def _xy_our_side_center(self):
        # ConstraintXYの自分サイドの中央を返す
        their_center = ConstraintXY()
        their_center.normalized = True
        their_center.value_x.append(-0.5)
        their_center.value_y.append(0.0)
        return their_center

    def _theta_look_ball(self):
        # ConstraintThetaでボールを見る角度を返す
        look_ball = ConstraintTheta()
        look_ball.object.append(self._object_ball())
        look_ball.param = ConstraintTheta.PARAM_LOOK_TO
        return look_ball

    def _pose_goal(self, pose, keep=True):
        goal_msg = RobotControl.Goal()
        goal_msg.pose.append(pose)
        goal_msg.keep_control = keep
        return goal_msg

    def _line_goal(self, line, keep=True):
        goal_msg = RobotControl.Goal()
        goal_msg.line.append(line)
        goal_msg.keep_control = keep
        return goal_msg

    def _with_kick(self, goal_msg, target, kick_pass=False, kick_setplay=False):
        goal_msg.kick_enable = True
        goal_msg.kick_pass = kick_pass
        goal_msg.kick_target = target
        goal_msg.kick_setplay = kick_setplay
        return goal_msg

    def _with_reflect_kick(self, goal_msg, target, kick_pass=False):
        goal_msg.reflect_shoot = True
        goal_msg.kick_pass = kick_pass
        goal_msg.kick_target = target
        goal_msg.kick_setplay = False
        return goal_msg

    def _with_reflect_and_normal_kick(self, goal_msg, target, kick_pass=False):
        goal_msg.kick_enable = True
        goal_msg.reflect_shoot = True
        goal_msg.kick_pass = kick_pass
        goal_msg.kick_target = target
        goal_msg.kick_setplay = False
        return goal_msg

    def _with_dribble(self, goal_msg, target):
        goal_msg.dribble_enable = True
        goal_msg.dribble_target = target
        return goal_msg

    def _with_receive(self, goal_msg):
        goal_msg.receive_ball = True
        return goal_msg

    def _act_to_ball(self, robot_id, target_xy_object,
                     do_shoot=False, do_pass=False, do_dribble=False,
                     do_reflect_shoot=False,
                     as_setplay=False):
        # ボールとobjectを結ぶ直線上で、ボールの後ろに移動し、
        # objectに向かってshoot/pass/dribbleする
        if not any([do_shoot, do_pass, do_dribble, do_reflect_shoot]):
            self.get_logger().warn('do_***のいずれかをセットしてください')
            return

        line = ConstraintLine()
        line.p1.object.append(self._object_ball())
        line.p2 = target_xy_object
        line.distance = -0.3
        line.theta = self._theta_look_ball()
        target = target_xy_object

        goal = None
        if do_shoot:
            goal = self._with_kick(
                self._line_goal(line, keep=True), target, kick_pass=False, kick_setplay=as_setplay)
        elif do_pass:
            goal = self._with_kick(
                self._line_goal(line, keep=True), target, kick_pass=True)
        elif do_dribble:
            goal = self._with_dribble(
                self._line_goal(line, keep=True), target)
        elif do_reflect_shoot:
            goal = self._with_receive(
                self._with_reflect_and_normal_kick(
                    self._line_goal(line, keep=True), target, kick_pass=False))

        self._set_goal(robot_id, goal)

    def _set_goal(self, robot_id, goal_msg):
        # アクションのゴールを設定する
        if not self._action_clients[robot_id].wait_for_server(5):
            self.get_logger().error('TIMEOUT: wait_for_server')
            return False

        if self._stop_game_velocity_has_enabled[robot_id]:
            goal_msg.max_velocity_xy.append(self.STOP_GAME_VELOCITY)

        goal_msg.avoid_obstacles = self._avoid_obstacles_enabled[robot_id]
        goal_msg.avoid_placement = self._avoid_placement_enabled[robot_id]

        self._send_goal_future[robot_id] = self._action_clients[robot_id].send_goal_async(
            goal_msg, feedback_callback=partial(self._feedback_callback, robot_id=robot_id))

        self._send_goal_future[robot_id].add_done_callback(
            partial(self._goal_response_callback, robot_id=robot_id))
        self._robot_is_free[robot_id] = False

    def _goal_response_callback(self, future, robot_id):
        # アクションサーバへゴールを送信した後の応答を受信したら実行される関数
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
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
        self.get_logger().debug('RobotId: {}, Result: {}, Message: {}'.format(robot_id, result.success, result.message))
        self._robot_is_free[robot_id] = True

