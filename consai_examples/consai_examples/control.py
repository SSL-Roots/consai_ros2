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
from cmath import sqrt
import math
import threading
import time
import random
from unittest.mock import patch
from matplotlib import patches
import numpy as np
import statistics
import matplotlib.pyplot as plt

import rclpy
from rclpy.executors import MultiThreadedExecutor
from robot_operator import RobotOperator


def test_move_to(max_velocity_xy=None):
    # フィールド上の全ロボットが、フィールドを上下(y軸)に往復する
    for i in range(16):
        operator_node.move_to(i, -5.0 + 0.5 * i, 4.0,
                              math.pi * 0.5, False, max_velocity_xy)

    # 全てのロボットがフリー（目的地に到着 or 常時制御中）になるまで待機
    while operator_node.all_robots_are_free() is False:
        pass

    # 制御継続フラグがTrueなので、ロボットはすぐにフリー状態になる
    for i in range(16):
        operator_node.move_to(i, -5.0 + 0.5 * i, -4.0, -
                              math.pi * 0.5, True, max_velocity_xy)

    while operator_node.all_robots_are_free() is False:
        pass


def test_move_to_normalized(divide_n=3):
    # ID0のロボットはフィールド中央に、
    # 8台のロボットはフィールドの上下左右斜めの方向から中心に向かって移動する
    # divide_nはフィールドの大きさをn等分して移動することを意味する

    # フィールド端まで広がる
    size = 1.0
    # 0番はセンターに配置
    operator_node.move_to_normalized(0, 0.0, 0.0, 0.0, False)
    # 1 ~ 8は時計回りに配置
    operator_node.move_to_normalized(1, 0.0, size, 0.0, False)
    operator_node.move_to_normalized(2, size, size, 0.0, False)
    operator_node.move_to_normalized(3, size, 0.0, 0.0, False)
    operator_node.move_to_normalized(4, size, -size, 0.0, False)
    operator_node.move_to_normalized(5, 0.0, -size, 0.0, False)
    operator_node.move_to_normalized(6, -size, -size, 0.0, False)
    operator_node.move_to_normalized(7, -size, 0.0, 0.0, False)
    operator_node.move_to_normalized(8, -size, size, 0.0, False)

    while operator_node.all_robots_are_free() is False:
        pass

    for i in range(1, divide_n):
        size = 1.0 / (i + 1)
        # 0番はセンターに配置
        operator_node.move_to_normalized(0, 0.0, 0.0, 0.0, False)
        # 1 ~ 8は時計回りに配置
        operator_node.move_to_normalized(1, 0.0, size, 0.0, False)
        operator_node.move_to_normalized(2, size, size, 0.0, False)
        operator_node.move_to_normalized(3, size, 0.0, 0.0, False)
        operator_node.move_to_normalized(4, size, -size, 0.0, False)
        operator_node.move_to_normalized(5, 0.0, -size, 0.0, False)
        operator_node.move_to_normalized(6, -size, -size, 0.0, False)
        operator_node.move_to_normalized(7, -size, 0.0, 0.0, False)
        operator_node.move_to_normalized(8, -size, size, 0.0, False)
        while operator_node.all_robots_are_free() is False:
            pass

        # 番号をずらして回転
        operator_node.move_to_normalized(8, 0.0, size, 0.0, False)
        operator_node.move_to_normalized(1, size, size, 0.0, False)
        operator_node.move_to_normalized(2, size, 0.0, 0.0, False)
        operator_node.move_to_normalized(3, size, -size, 0.0, False)
        operator_node.move_to_normalized(4, 0.0, -size, 0.0, False)
        operator_node.move_to_normalized(5, -size, -size, 0.0, False)
        operator_node.move_to_normalized(6, -size, 0.0, 0.0, False)
        operator_node.move_to_normalized(7, -size, size, 0.0, False)
        while operator_node.all_robots_are_free() is False:
            pass


def test_chase_ball():
    # ボールの右側に、2次関数のように並ぶ
    for i in range(16):
        operator_node.chase_ball(
            i, 0.2 + 0.2*i, 0.05 * i*i, 0.1 * math.pi * i, True, False)

    while operator_node.all_robots_are_free() is False:
        pass

    # ボールを見る
    for i in range(16):
        operator_node.chase_ball(i, 0.2 + 0.2*i, 0.05 * i*i, 0.0, False, True)

    while operator_node.all_robots_are_free() is False:
        pass


def test_chase_robot():
    # 全ロボットが、別のチームカラーの同じIDのロボットの左上に移動する
    for i in range(16):
        operator_node.chase_robot(
            i, not operator_node.target_is_yellow(), i, -0.2, 0.2, 0.0, True, False)

    while operator_node.all_robots_are_free() is False:
        pass

    # 左横に移動する
    for i in range(16):
        operator_node.chase_robot(
            i, not operator_node.target_is_yellow(), i, -0.2, 0.0, 0.0, False, True)

    while operator_node.all_robots_are_free() is False:
        pass


def test_for_config_pid(pattern=[0.1, 0.3, 0.5, 0.7, 0.9], test_x=False, test_y=False,
                        test_theta=False):
    # PID調整用の関数
    # この関数を実行している裏で、consai_robot_controllerのPIDゲインを調整することを推奨する
    # X, Y, thetaの目標値が往復するように変化する
    # 例：test_xがtrueで、X方向に往復移動する
    # patternには往復する距離（角度）の正規化された値の配列を設定する

    # 左右
    if test_x:
        for x in pattern:
            for i in range(16):
                operator_node.move_to_normalized(
                    i, -x, 1.0 - 2.0 * i / 16.0, 0.0, False)

            while operator_node.all_robots_are_free() is False:
                pass

            for i in range(16):
                operator_node.move_to_normalized(
                    i, x, 1.0 - 2.0 * i / 16.0, 0.0, False)

            while operator_node.all_robots_are_free() is False:
                pass

    # 上下
    if test_y:
        for y in pattern:
            for i in range(16):
                operator_node.move_to_normalized(
                    i, -0.9 + 2.0 * i / 16.0, y, math.pi * 0.5, False)

            while operator_node.all_robots_are_free() is False:
                pass

            for i in range(16):
                operator_node.move_to_normalized(
                    i, -0.9 + 2.0 * i / 16.0, -y, math.pi * 0.5, False)

            while operator_node.all_robots_are_free() is False:
                pass

    if test_theta:
        for theta in pattern:
            for i in range(16):
                operator_node.move_to_normalized(
                    i, -0.9 + 2.0 * i / 16.0, 0.5, math.pi * theta, False)

            while operator_node.all_robots_are_free() is False:
                pass

            for i in range(16):
                operator_node.move_to_normalized(
                    i, -0.9 + 2.0 * i / 16.0, 0.5, -math.pi * theta, False)

            while operator_node.all_robots_are_free() is False:
                pass


def test_shoot(target_x, target_y):
    # ID0ロボットがフィールド中央に移動して、ターゲット座標に向かってボールを蹴る
    pos_x = 0.0
    pos_y = 0.0
    operator_node.shoot(0, pos_x, pos_y, target_x, target_y)

    while operator_node.all_robots_are_free() is False:
        pass


def test_shoot_to_their(robot_id):
    operator_node.shoot_to_their_goal(robot_id)


def test_pass_two_robots():
    # 2台のロボットでパスし合う
    operator_node.kick_pass(0, 1, -3.0, 0.0)
    operator_node.kick_pass(1, 0, 3.0, 0.0)

    while operator_node.all_robots_are_free() is False:
        pass


def test_pass_four_robots():
    # 4台のロボットでパスし合う

    operator_node.kick_pass(0, 1, 3.0, 3.0)
    operator_node.kick_pass(1, 2, 3.0, -3.0)
    operator_node.kick_pass(2, 3, -3.0, 3.0)
    operator_node.kick_pass(3, 0, -3.0, -3.0)

    while operator_node.all_robots_are_free() is False:
        pass


def test_stop_robots():
    # フィールド上の全ロボットが、フィールドを上下(y軸)に往復する
    # 動作の途中でロボットを停止させる

    # y軸上方向へ移動
    for i in range(16):
        operator_node.move_to(i, -5.0 + 0.5 * i, 4.0, math.pi * 0.5, False)
    start_time = time.time()

    # 数秒間待機
    while time.time() - start_time < 2.0:
        pass

    # 5台のロボットを停止
    print('ロボットの動作停止！')
    for i in range(5):
        operator_node.stop(i)
    # 動作完了まで待機
    while operator_node.all_robots_are_free() is False:
        pass

    # y軸下方向へ移動
    for i in range(16):
        operator_node.move_to(i, -5.0 + 0.5 * i, -4.0, math.pi * 0.5, False)
    start_time = time.time()

    # 数秒間待機
    while time.time() - start_time < 2.0:
        pass

    # 5台のロボットを停止
    print('ロボットの動作停止！')
    for i in range(5):
        operator_node.stop(i)
    # 動作完了まで待機
    while operator_node.all_robots_are_free() is False:
        pass


def test_move_to_line():
    # フィールド上の全ロボットが、フィールドを上下(y軸)に往復する
    p1_x = 0.0
    p1_y = 0.0
    STEP = 6
    LENGTH = 5.0
    for i in range(STEP):
        angle = 2.0 * math.pi * i / float(STEP)
        p2_x = LENGTH * math.cos(angle)
        p2_y = LENGTH * math.sin(angle)
        for i in range(16):
            operator_node.move_to_line(
                i, p1_x, p1_y, p2_x, p2_y, i*0.4, 0, False)

        # 全てのロボットがフリー（目的地に到着 or 常時制御中）になるまで待機
        while operator_node.all_robots_are_free() is False:
            pass


def test_defend_goal_on_line(p1_x, p1_y, p2_x, p2_y):
    robot_harf_width = 0.09
    harf_width = 6.0
    defence_harf_width = 0.9
    defence_harf_height = 1.8
    goal_harf_height = 0.9
    # 0番はキーパの位置に
    x = harf_width - robot_harf_width
    y = goal_harf_height
    operator_node.move_to_line_to_defend_our_goal(0, -x, y, -x, -y, True)
    # 1番はディフェンスエリアの上左側に
    # 2番はディフェンスエリアの上右側に
    y = defence_harf_height + robot_harf_width
    x = harf_width - robot_harf_width
    end_x = harf_width - defence_harf_width
    middle_x = harf_width - defence_harf_width - robot_harf_width * 2.0
    middle_end_x = harf_width - defence_harf_width * 2.0 - robot_harf_width
    operator_node.move_to_line_to_defend_our_goal(1, -x, y, -end_x, y, True)
    operator_node.move_to_line_to_defend_our_goal(
        2, -middle_x, y, -middle_end_x, y, True)

    # 3番はディフェンスエリアの下左側に
    # 4番はディフェンスエリアの下右側に
    operator_node.move_to_line_to_defend_our_goal(3, -x, -y, -end_x, -y, True)
    operator_node.move_to_line_to_defend_our_goal(
        4, -middle_x, -y, -middle_end_x, -y, True)

    # 5番はディフェンスエリアの正面上側に
    # 6番はディフェンスエリアの正面上側に
    # 7番はディフェンスエリアの正面下側に
    # 8番はディフェンスエリアの正面下側に
    x = harf_width - defence_harf_width * 2.0 - robot_harf_width
    y = defence_harf_height - robot_harf_width
    end_y = defence_harf_height * 0.5
    middle_y = defence_harf_height * 0.5 - robot_harf_width * 2.0
    middle_end_y = robot_harf_width
    operator_node.move_to_line_to_defend_our_goal(5, -x, y, -x, end_y, True)
    operator_node.move_to_line_to_defend_our_goal(
        6, -x, middle_y, -x, middle_end_y, True)
    operator_node.move_to_line_to_defend_our_goal(7, -x, -y, -x, -end_y, True)
    operator_node.move_to_line_to_defend_our_goal(
        8, -x, -middle_y, -x, -middle_end_y, True)


def test_reflect_shoot(robot_id, x, y):
    operator_node.move_to_reflect_shoot_to_their_goal(robot_id, x, y)


def test_refelect_shoot_four_robots(id1, id2, id3, id4):
    # 4台のロボットでパスし合う
    dist = 2.5
    operator_node.move_to_reflect_shoot_to_our_robot(id1, id2, dist, dist)
    operator_node.move_to_reflect_shoot_to_our_robot(id2, id3, dist, -dist)
    operator_node.move_to_reflect_shoot_to_our_robot(id3, id4, -dist, dist)
    operator_node.move_to_reflect_shoot_to_our_robot(id4, id1, -dist, -dist)
    while operator_node.all_robots_are_free() is False:
        pass


def try_test_move_to(max_velocity_xy=None):
    # フィールド上の全ロボットが、フィールドを上下(y軸)に往復する
    while(1):
        for i in range(11):
            operator_node.move_to(i, -5.0 + 0.5 * i, 4.0,
                                  math.pi * 0.5, False, max_velocity_xy)

        # 全てのロボットがフリー（目的地に到着 or 常時制御中）になるまで待機
        while operator_node.all_robots_are_free() is False:
            pass

        # 制御継続フラグがTrueなので、ロボットはすぐにフリー状態になる
        for i in range(11):
            operator_node.move_to(i, -5.0 + 0.5 * i, -4.0, -
                                  math.pi * 0.5, False, max_velocity_xy)

        while operator_node.all_robots_are_free() is False:
            pass

        #  制御継続フラグがTrueなので、ロボットはすぐにフリー状態になる
        for i in range(11):
            operator_node.move_to(i, -5.0, -4.0 + 0.8 * i, -
                                  math.pi * 0.5, False, max_velocity_xy)

        while operator_node.all_robots_are_free() is False:
            pass

        # 制御継続フラグがTrueなので、ロボットはすぐにフリー状態になる
        for i in range(11):
            operator_node.move_to(i, 5.0, -4.0 + 0.8 * i, -
                                  math.pi * 0.5, False, max_velocity_xy)

        while operator_node.all_robots_are_free() is False:
            pass


def try_test_chase_ball():
    # # 配置設定
    robot_dist = 3.0
    for i in range(2):
        operator_node.chase_ball(
            i, 0.2 + robot_dist*i, 0.0, 0.1 * math.pi * i, True, False)

    while operator_node.all_robots_are_free() is False:
        pass

    # ボールを見る
    for i in range(2):
        operator_node.chase_ball(i, 0.2 + robot_dist*i, 0.0, 0.0, False, True)

    while operator_node.all_robots_are_free() is False:
        pass


def try_test_shoot_ball(target_x, target_y, robot_id):
    # ID0ロボットがターゲット座標に向かってボールを蹴る
    pos_x = 1.0
    pos_y = 0.0
    operator_node.shoot(robot_id, pos_x, pos_y, target_x, target_y)

    while operator_node.all_robots_are_free() is False:
        pass


def pass_two_robots_and_shoot(id1, id2):

    # 2台のロボットでパスしてシュートする
    # operator_node.move_to_normalized(id2, 0.5, 0.5, theta, False)
    operator_node.chase_ball(id1, -0.07, -0.07, 3.14, True, False)
    operator_node.move_to_look_ball(id2, 3, 3)
    # operator_node.dribble_to(id1,0.0, 0.0)
    while operator_node.all_robots_are_free() is False:
        pass

    operator_node.kick_pass(id1, id2, 0.0, 0.0)
    # operator_node.pass_to(id1,2.5,1.5)
    # operator_node.move_to_receive(id2, 3.0, 3.0)
    while operator_node.all_robots_are_free() is False:
        pass

    operator_node.setplay_shoot_to_their_goal(id2)
    while operator_node.all_robots_are_free() is False:
        pass


def pass_three_robots_and_shoot(id1, id2, id3):
    # 1台のロボットが2台のロボットのどちらかにパスしてシュートする
    operator_node.chase_ball(id1, -0.1, 0.0, 3.14, True, False)
    operator_node.move_to_look_ball(id2, 2, 4)
    operator_node.move_to_look_ball(id3, 4, -2)
    while operator_node.all_robots_are_free() is False:
        pass

    select_robot = random.randint(1, 2)
    print(select_robot)

    if select_robot == 1:
        operator_node.chase_ball(id1, -0.07, -0.07, 3.14, True, False)
        # while operator_node.all_robots_are_free() is False:
        #     pass

        operator_node.kick_pass(id1, id2, 0.0, 0.0)
        while operator_node.all_robots_are_free() is False:
            pass

        operator_node.setplay_shoot_to_their_goal(id2)
        while operator_node.all_robots_are_free() is False:
            pass

    elif select_robot == 2:
        operator_node.chase_ball(id1, 0.07, -0.07, 3.14, True, False)
        # while operator_node.all_robots_are_free() is False:
        #     pass

        operator_node.kick_pass(id1, id3, 0.0, 0.0)
        while operator_node.all_robots_are_free() is False:
            pass

        operator_node.setplay_shoot_to_their_goal(id3)
        while operator_node.all_robots_are_free() is False:
            pass

    else:
        for i in range(3):
            operator_node.stop(i)
        while operator_node.all_robots_are_free() is False:
            pass


def pass_three_robots_and_shoot_ex(id1, id2, id3):
    # 1台のロボットが2台のロボットのどちらかにパスしてシュートする
    operator_node.chase_ball(id1, -0.1, 0.0, 3.14, True, False)
    operator_node.move_to_look_ball(id2, 2, 3)
    operator_node.move_to_look_ball(id3, 4, -2)
    while operator_node.all_robots_are_free() is False:
        pass

    pass_point = eval_easy_to_pass()

    print(pass_point)

    operator_node.chase_ball(id1, -0.1, 0.0, 3.14, True, False)
    # start_time = time.time()

    if pass_point[1] >= 0:
        # operator_node.receive_from(id2, pass_point[0] * 3, pass_point[1] * 3, 2.5, True)
        operator_node.move_to_receive(
            id2, pass_point[0] * 2, pass_point[1] * 2)
        # operator_node.move_to_reflect_shoot_to_their_goal(id2, pass_point[0]*2, pass_point[1]*2)
        while operator_node.all_robots_are_free() is False:
            pass

        operator_node.kick_pass(id1, id2, 0.0, 0.0)
        # operator_node.shoot_to(id1,pass_point[0], pass_point[1])
        start_time = time.time()
        while time.time() - start_time < 1:
            pass

        operator_node.shoot_to_their_goal(id2)

    elif pass_point[1] < 0:
        # operator_node.receive_from(id3, pass_point[0] * 3, pass_point[1] * 3, 2.5, True)
        operator_node.move_to_receive(
            id3, pass_point[0] * 2, pass_point[1] * 2)
        # operator_node.move_to_reflect_shoot_to_their_goal(id3, pass_point[0]*2, pass_point[1]*2)
        while operator_node.all_robots_are_free() is False:
            pass

        operator_node.kick_pass(id1, id3, 0.0, 0.0)
        start_time = time.time()
        while time.time() - start_time < 1:
            pass

        operator_node.shoot_to_their_goal(id3)

    else:
        for i in range(3):
            operator_node.stop(i)

    while operator_node.all_robots_are_free() is False:
        pass


def pass_and_shoot(enemy_num, ally_id1, ally_id2, ally_id3, ally_id4):
    # ball_pos = position_random_generation(1)
    ball_pos = [0, 0]
    enemy_robot_pos = position_random_generation(enemy_num)
    for i in range(4, 11):
        operator_node.move_to(
            i, enemy_robot_pos[i-4][0], enemy_robot_pos[i-4][1], 0, False)

    ally_robots_pos = [[2, 3], [4, -2], [5, 1]]
    operator_node.move_to_look_ball(ally_id1, ball_pos[0]-1, ball_pos[1])
    operator_node.move_to_look_ball(
        ally_id2, ally_robots_pos[0][0], ally_robots_pos[0][1])
    operator_node.move_to_look_ball(
        ally_id3, ally_robots_pos[1][0], ally_robots_pos[1][1])
    operator_node.move_to_look_ball(
        ally_id4, ally_robots_pos[2][0], ally_robots_pos[2][1])
    while operator_node.all_robots_are_free() is False:
        pass

    pass_point = eval_easy_to_pass_radius_ex2(
        enemy_robot_pos, ball_pos, ally_robots_pos)
    print(pass_point)

    # receive_dist_ally_robot_id2 = math.sqrt((pass_point[0] - 2) ** 2 + (pass_point[1] - 3) ** 2)
    # receive_dist_ally_robot_id3 = math.sqrt((pass_point[0] - 4) ** 2 + (pass_point[1] + 2) ** 2)
    # receive_dist_ally_robot_id4 = math.sqrt((pass_point[0] - 5) ** 2 + (pass_point[1] - 1) ** 2)

    receive_dist_ally_robot = [[math.sqrt((pass_point[0] - 2) ** 2 + (pass_point[1] - 3) ** 2), 2],
                               [math.sqrt((pass_point[0] - 4) ** 2 +
                                          (pass_point[1] + 2) ** 2), 3],
                               [math.sqrt((pass_point[0] - 5) ** 2 + (pass_point[1] - 1) ** 2), 4]]
    receive_dist_ally_robot = sorted(receive_dist_ally_robot)

    operator_node.chase_ball(ally_id1, -0.1, 0.0, 3.14, True, False)

    if receive_dist_ally_robot[0][1] == 2:
        print("pass from 0 to 1")
        # operator_node.receive_from(id2, pass_point[0] * 3, pass_point[1] * 3, 2.5, True)
        operator_node.move_to_receive(ally_id2, pass_point[0], pass_point[1])
        # operator_node.move_to_reflect_shoot_to_their_goal(id2, pass_point[0]*2, pass_point[1]*2)
        while operator_node.all_robots_are_free() is False:
            pass

        # var = input("Please input variable : ")
        # while 1:
        #     if var == 'OK':
        #         break

        # operator_node.shoot_to(id1,pass_point[0], pass_point[1])
        operator_node.kick_pass(ally_id1, ally_id2, ball_pos[0], ball_pos[1])
        start_time = time.time()
        while time.time() - start_time < 1:
            pass

        operator_node.shoot_to_their_goal(ally_id2)

    elif receive_dist_ally_robot[0][1] == 3:
        print("pass to 0 to 2")
        # operator_node.receive_from(id3, pass_point[0] * 3, pass_point[1] * 3, 2.5, True)
        operator_node.move_to_receive(ally_id3, pass_point[0], pass_point[1])
        # operator_node.move_to_reflect_shoot_to_their_goal(id3, pass_point[0]*2, pass_point[1]*2)
        while operator_node.all_robots_are_free() is False:
            pass

        # var = input("Please input variable : ")
        # while 1:
        #     if var == 'OK':
        #         break

        operator_node.kick_pass(ally_id1, ally_id3, ball_pos[0], ball_pos[1])
        start_time = time.time()
        while time.time() - start_time < 1:
            pass

        operator_node.shoot_to_their_goal(ally_id3)

    elif receive_dist_ally_robot[0][1] == 4:
        print("pass to 0 to 3")
        # operator_node.receive_from(id3, pass_point[0] * 3, pass_point[1] * 3, 2.5, True)
        operator_node.move_to_receive(ally_id4, pass_point[0], pass_point[1])
        # operator_node.move_to_reflect_shoot_to_their_goal(id3, pass_point[0]*2, pass_point[1]*2)
        while operator_node.all_robots_are_free() is False:
            pass

        # var = input("Please input variable : ")
        # while 1:
        #     if var == 'OK':
        #         break
        operator_node.kick_pass(ally_id1, ally_id4, ball_pos[0], ball_pos[1])
        start_time = time.time()
        while time.time() - start_time < 1:
            pass

        operator_node.shoot_to_their_goal(ally_id4)

    else:
        for i in range(4):
            operator_node.stop(i)

    while operator_node.all_robots_are_free() is False:
        pass


def pass_and_shoot_ex(enemy_num, ally_id1, ally_id2, ally_id3, ally_id4):
    # ball_pos = position_random_generation(1)
    ball_pos = [0, 0]
    enemy_robot_pos = position_random_generation(enemy_num)
    for i in range(4, 11):
        operator_node.move_to(
            i, enemy_robot_pos[i-4][0], enemy_robot_pos[i-4][1], 0, False)

    ally_robots_pos = [[4, -2], [2, 3], [5, 1]]
    operator_node.move_to_look_ball(ally_id1, ball_pos[0]-1, ball_pos[1])
    operator_node.move_to_look_ball(
        ally_id2, ally_robots_pos[0][0], ally_robots_pos[0][1])
    operator_node.move_to_look_ball(
        ally_id3, ally_robots_pos[1][0], ally_robots_pos[1][1])
    operator_node.move_to_look_ball(
        ally_id4, ally_robots_pos[2][0], ally_robots_pos[2][1])
    while operator_node.all_robots_are_free() is False:
        pass

    pass_to_robot = eval_easy_to_pass_radius_ex2(
        enemy_robot_pos, ball_pos, ally_robots_pos)
    print(pass_to_robot)

    while operator_node.all_robots_are_free() is False:
        pass

    operator_node.kick_pass(ally_id1, pass_to_robot +
                            1, ball_pos[0], ball_pos[1])
    # operator_node.shoot_to(
    #     ally_id1, ally_robots_pos[pass_to_robot][0], ally_robots_pos[pass_to_robot][1])
    start_time = time.time()
    while time.time() - start_time < 1:
        pass
    # while operator_node.all_robots_are_free() is False:
    #     pass

    operator_node.move_to_receive(
        pass_to_robot + 1, ally_robots_pos[pass_to_robot][0], ally_robots_pos[pass_to_robot][1])

    # start_time = time.time()
    # while time.time() - start_time < 1:
    #     pass
    while operator_node.all_robots_are_free() is False:
        pass

    operator_node.shoot_to_their_goal(pass_to_robot + 1)


def test_prototyping(conb):
    enemy_robot_pos = position_random_generation(conb)
    ball_pos = position_random_generation(1)

    start = time.time()
    print(enemy_robot_pos)  # 敵ロボットの位置座標
    print(ball_pos)  # ボールの位置座標

    result = eval_easy_to_pass_radius_ex(enemy_robot_pos, ball_pos)
    print(result)
    necessary_time = time.time() - start
    print(necessary_time)

# 評価関数部 ------------------------------------------------------------------------------------


def eval_easy_to_pass():
    # 相手のロボットのx軸、y軸をそれぞれ格納する
    # 一定速度で移動すると考えてどこにパスを出せばよいかを評価する
    enemy_robot_pos0 = [1, 2.5]
    enemy_robot_pos1 = [2, 0]
    enemy_robot_pos2 = [1, -2]

    # 中点の算出
    midpoint_x01 = (enemy_robot_pos0[0] + enemy_robot_pos1[0]) / 2
    midpoint_x02 = (enemy_robot_pos0[0] + enemy_robot_pos2[0]) / 2
    midpoint_x12 = (enemy_robot_pos1[0] + enemy_robot_pos2[0]) / 2
    midpoint_y01 = (enemy_robot_pos0[1] + enemy_robot_pos1[1]) / 2
    midpoint_y02 = (enemy_robot_pos0[1] + enemy_robot_pos2[1]) / 2
    midpoint_y12 = (enemy_robot_pos1[1] + enemy_robot_pos2[1]) / 2

    midpoint_xy01 = []
    midpoint_xy01.append(midpoint_x01)
    midpoint_xy01.append(midpoint_y01)
    midpoint_xy02 = []
    midpoint_xy02.append(midpoint_x02)
    midpoint_xy02.append(midpoint_y02)
    midpoint_xy12 = []
    midpoint_xy12.append(midpoint_x12)
    midpoint_xy12.append(midpoint_y12)

    print(midpoint_xy01)
    print(midpoint_xy02)
    print(midpoint_xy12)

    # 2点間の距離の算出
    ene_robot_0 = np.array(enemy_robot_pos0)
    ene_robot_1 = np.array(enemy_robot_pos1)
    ene_robot_2 = np.array(enemy_robot_pos2)

    distance_01 = np.linalg.norm(ene_robot_0 - ene_robot_1)
    distance_02 = np.linalg.norm(ene_robot_0 - ene_robot_2)
    distance_12 = np.linalg.norm(ene_robot_1 - ene_robot_2)

    # 2点間の距離の中央値の算出
    distances = []
    distances.append(distance_01)
    distances.append(distance_02)
    distances.append(distance_12)
    pass_course = statistics.median(distances)

    if pass_course == distance_01:
        return midpoint_xy01
    elif pass_course == distance_02:
        return midpoint_xy02
    elif pass_course == distance_12:
        return midpoint_xy12


def eval_easy_to_pass_ex(conb: int):
    # 敵のロボットの座標
    enemy_robot_pos = position_random_generation(conb)

    # 2点間の距離の算出
    for i in range(conb - 1):
        for j in range(i + 1, conb):
            target_num0 = np.array(enemy_robot_pos[i])
            target_num1 = np.array(enemy_robot_pos[j])

            distance = np.linalg.norm(enemy_robot_pos[i] - enemy_robot_pos[j])
            distances = []
            distances.append(distance)

    # 最適なパスコース(複数も要検討)

    # 最適なパスコースの中点の算出


def eval_easy_to_pass_radius(conb: int):
    # 敵のロボットの座標
    # enemy_robot_pos = position_random_generation(conb)
    # print(enemy_robot_pos)  # 敵ロボットの位置座標
    # enemy_robot_pos = np.array(enemy_robot_pos)
    # ball_pos = position_random_generation(1)
    # print(ball_pos)  # ボールの位置座標
    # ball_pos = np.array(ball_pos)

    enemy_robot_pos = [
        [random.randint(-6, 6), random.randint(-4, 4)] for i in range(conb)]
    ball_pos = [random.randint(-6, 6), random.randint(-4, 4)]

    print(enemy_robot_pos)  # 敵ロボットの位置座標
    print(ball_pos)  # ボールの位置座標

    enemy_robot_pos = np.array(enemy_robot_pos)
    ball_pos = np.array(ball_pos)

    pass_possible = []
    r = 1
    count = 0

    # midpoint_xy = []
    # sorted_enemy_robot_pos = []
    # pass_possible = []

    # 障害になるロボットの選別
    # for i in range(conb):
    #     if not ball_pos[0] > enemy_robot_pos[i][0]:
    #         sorted_enemy_robot_pos.append(enemy_robot_pos[i])

    sorted_enemy_robot_pos = [enemy_robot_pos[i] for i in range(
        conb) if not ball_pos[0] > enemy_robot_pos[i][0]]
    print(sorted_enemy_robot_pos)

    # すべてのロボット間の中点の算出
    # for i in range(len(sorted_enemy_robot_pos) - 1):
    #     for j in range(i + 1, len(sorted_enemy_robot_pos)):
    #         midpoint_x = (
    #             sorted_enemy_robot_pos[i][0] + sorted_enemy_robot_pos[j][0]) / 2
    #         midpoint_y = (
    #             sorted_enemy_robot_pos[i][1] + sorted_enemy_robot_pos[j][1]) / 2
    #         midpoint_xy.append([midpoint_x, midpoint_y])

    midpoint_xy = [[(sorted_enemy_robot_pos[i][0] + sorted_enemy_robot_pos[j][0]) / 2, (sorted_enemy_robot_pos[i][1] + sorted_enemy_robot_pos[j][1]) / 2]
                   for i in range(len(sorted_enemy_robot_pos) - 1) for j in range(i + 1, len(sorted_enemy_robot_pos))]

    print(midpoint_xy)  # すべてのロボットの中点

    # 相手ロボットの推定動作範囲の半径を設定と候補点をまとめる

    for i in range(len(sorted_enemy_robot_pos)):
        x_diff = midpoint_xy[i][0] - ball_pos[0]
        y_diff = midpoint_xy[i][1] - ball_pos[1]
        dist_ball_to_enemy_x = ball_pos[0] - sorted_enemy_robot_pos[i][0]
        dist_ball_to_enemy_y = ball_pos[1] - sorted_enemy_robot_pos[i][1]

        # 解を求める
        a = x_diff ** 2 + y_diff ** 2
        b = x_diff * dist_ball_to_enemy_x + y_diff * dist_ball_to_enemy_y
        c = dist_ball_to_enemy_x ** 2 + dist_ball_to_enemy_y ** 2 - r ** 2
        # D = b ** 2 - a * c
        # if D < 0:
        #     pass_possible.append(midpoint_xy[i])

        factor1 = c
        factor2 = a + 2 * b + c

        if (factor1 < 0 and factor2 > 0) or (factor1 > 0 and factor2 < 0):
            pass_possible.append(midpoint_xy[i])
            return pass_possible

    print(len(pass_possible))
    # for i in range(len(pass_possible)):
    #     plt.scatter(pass_possible[i][0], pass_possible[i][1])
    # plt.scatter(ball_pos[0], ball_pos[1])
    # plt.show()
    return pass_possible


def eval_easy_to_pass_radius_ex(enemy_robot_pos, ball_pos):
    enemy_robot_pos = np.array(enemy_robot_pos)
    ball_pos = np.array(ball_pos)

    # pass_possible = []
    r = 1

    # ボールより敵のゴール側にいるロボットのみに絞る
    sorted_enemy_robot_pos = [enemy_robot_pos[i] for i in range(
        len(enemy_robot_pos)) if not ball_pos[0] > enemy_robot_pos[i][0]]
    print(sorted_enemy_robot_pos)
    sorted_enemy_robot_pos = np.array(sorted_enemy_robot_pos)

    # ボールより敵ゴール側にいるロボットの中点をすべて算出
    midpoint_xy = [[(sorted_enemy_robot_pos[i][0] + sorted_enemy_robot_pos[j][0]) / 2, (sorted_enemy_robot_pos[i][1] + sorted_enemy_robot_pos[j][1]) / 2, i, j]
                   for i in range(len(sorted_enemy_robot_pos) - 1) for j in range(i + 1, len(sorted_enemy_robot_pos))]
    print(midpoint_xy)
    # midpoint_xy = np.array(midpoint_xy)

    # 解を求めるための要素
    midpoint_from_ball_xy = [[math.sqrt((midpoint_xy[i][0] - ball_pos[0]) ** 2 + (midpoint_xy[i][1] - ball_pos[1]) ** 2),
                              midpoint_xy[i][0] - ball_pos[0], midpoint_xy[i][1] - ball_pos[1], midpoint_xy[i]] for i in range(len(midpoint_xy))]
    midpoint_from_ball_xy = sorted(midpoint_from_ball_xy, reverse=True)

    print(midpoint_from_ball_xy)
    midpoint_from_ball_xy = np.array(midpoint_from_ball_xy)

    dist_ball_to_enemy_xy = [[ball_pos[0] - sorted_enemy_robot_pos[midpoint_xy[i][2]], ball_pos[1] -
                              sorted_enemy_robot_pos[midpoint_xy[i][3]]] for i in range(len(midpoint_xy))]
    print(dist_ball_to_enemy_xy)
    dist_ball_to_enemy_xy = np.array(midpoint_from_ball_xy)

    # 解を求める
    for i in range(len(midpoint_from_ball_xy)):
        for j in range(len(dist_ball_to_enemy_xy)):
            a = midpoint_from_ball_xy[i][1] ** 2 + \
                midpoint_from_ball_xy[i][2] ** 2
            b = midpoint_from_ball_xy[i][1] * dist_ball_to_enemy_xy[j][0] + \
                midpoint_from_ball_xy[i][2] * dist_ball_to_enemy_xy[j][1]
            c = dist_ball_to_enemy_xy[j][0] ** 2 + \
                dist_ball_to_enemy_xy[j][1] ** 2 - r ** 2

            # D = b ** 2 - a * c
            # if D < 0:
            # pass_possible.append(midpoint_from_ball_xy[i][3])
            # return midpoint_from_ball_xy[i][3]

            factor1 = c
            factor2 = a + 2 * b + c

            if (factor1 >= 0 and factor2 <= 0) or (factor1 <= 0 and factor2 >= 0):
                return midpoint_from_ball_xy[i][3]


def eval_easy_to_pass_radius_ex2(enemy_robot_pos, ball_pos, ally_robot_pos):
    enemy_robot_pos = np.array(enemy_robot_pos)
    ally_robot_pos = np.array(ally_robot_pos)
    ball_pos = np.array(ball_pos)

    r = 1
    count = 0

    # ボールより敵のゴール側にいるロボットのみに絞る
    forward_enemy_robot_pos = [enemy_robot_pos[i] for i in range(
        len(enemy_robot_pos)) if not ball_pos[0] > enemy_robot_pos[i][0]]
    print(forward_enemy_robot_pos)
    forward_enemy_robot_pos = np.array(forward_enemy_robot_pos)

    diff_ball_to_robot = [[ball_pos[0] - ally_robot_pos[i][0],
                           ball_pos[1] - ally_robot_pos[i][1]] for i in range(len(ally_robot_pos))]
    print(diff_ball_to_robot)
    receive_dist_ally_robot = [[math.sqrt((diff_ball_to_robot[i][0]) ** 2 + (
        diff_ball_to_robot[i][1]) ** 2), i] for i in range(len(ally_robot_pos))]
    receive_dist_ally_robot = sorted(receive_dist_ally_robot)

    diff_enemy_to_ball = [[ball_pos[0] - enemy_robot_pos[i][0], ball_pos[1] -
                           enemy_robot_pos[i][1]] for i in range(len(enemy_robot_pos))]

    print(receive_dist_ally_robot)
    # 解を求めよう
    for i in range(len(ally_robot_pos)):
        a = diff_ball_to_robot[receive_dist_ally_robot[i][1]][0] ** 2 + \
            diff_ball_to_robot[receive_dist_ally_robot[i][1]][1] ** 2
        for j in range(len(forward_enemy_robot_pos)):
            b = diff_ball_to_robot[receive_dist_ally_robot[i][1]][0]*diff_enemy_to_ball[j][0] + \
                diff_ball_to_robot[receive_dist_ally_robot[i][1]
                                   ][1]*diff_enemy_to_ball[j][1]
            c = diff_enemy_to_ball[j][0]**2+diff_enemy_to_ball[j][1]**2-r**2

            factor1 = c
            factor2 = a + 2 * b + c

            if (factor1 >= 0 and factor2 <= 0) or (factor1 <= 0 and factor2 >= 0):
                count = 0
                print('くそ')
                break
            else:
                count += 1
                if count == len(forward_enemy_robot_pos):
                    print('入った')
                    # return ally_robot_pos[receive_dist_ally_robot[i][1]]
                    return receive_dist_ally_robot[i][1]

# -----------------------------------------------------------------------------------------------

# 仮想敵ランダム生成関数-------------------------------------------------------------------------


def position_random_generation(how_many: int):
    generating_pos = []
    if how_many > 1:
        for i in range(how_many):
            generating_pos_x = random.randint(-6, 6)
            generating_pos_y = random.randint(-4, 4)
            generating_pos.append([generating_pos_x, generating_pos_y])
    elif how_many == 1:
        generating_pos_x = random.randint(-6, 6)
        generating_pos_y = random.randint(-4, 4)
        generating_pos.append(generating_pos_x)
        generating_pos.append(generating_pos_y)

    return generating_pos

# -----------------------------------------------------------------------------------------------


def main():
    # 実行したい関数のコメントを外してください
    # test_move_to()
    # test_move_to(1.5)  # 走行速度を1.0 m/sに制限
    # test_move_to_normalized(3)
    # test_chase_ball()
    # test_chase_robot()
    # test_for_config_pid(test_x=True)
    # test_shoot(1.0, 0.0)
    # test_shoot_to_their(6)
    # test_pass_two_robots()
    # test_pass_four_robots()
    # test_stop_robots()
    # test_move_to_line()
    # test_defend_goal_on_line(-1.0, 1.0, -1.0, -1.0)
    # test_reflect_shoot(0, 0, 0)
    # test_refelect_shoot_four_robots(0, 1, 2, 3)
    # try_test_move_to()
    # try_test_chase_ball()
    # try_test_shoot_ball(1.0, 1.0, 6)
    # pass_two_robots_and_shoot(0, 1)
    # pass_three_robots_and_shoot(0, 1, 2)
    # pass_three_robots_and_shoot_ex(0, 1, 2)
    # test_prototyping(11)
    # pass_and_shoot(7, 0, 1, 2, 3)
    pass_and_shoot_ex(7, 0, 1, 2, 3)


if __name__ == '__main__':
    arg_parser = argparse.ArgumentParser()
    arg_parser.add_argument('--yellow',
                            default=False,
                            action='store_true',
                            help='yellowロボットを動かす場合にセットする')
    args = arg_parser.parse_args()

    rclpy.init(args=None)

    operator_node = RobotOperator(target_is_yellow=args.yellow)

    # すべてのロボットの衝突回避を解除
    for i in range(16):
        operator_node.disable_avoid_obstacles(i)

    executor = MultiThreadedExecutor()
    executor.add_node(operator_node)

    # エグゼキュータは別スレッドでspinさせ続ける
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    try:
        main()
    except KeyboardInterrupt:
        pass

    executor.shutdown()
    rclpy.shutdown()
