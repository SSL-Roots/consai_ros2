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
from lib2to3.pytree import Node
import math
import threading
import time
import random
from unittest.mock import patch
from matplotlib import patches
import numpy as np
import statistics
import matplotlib.pyplot as plt

from field_observer import FieldObserver
import rclpy
from rclpy.executors import MultiThreadedExecutor
from robot_operator import RobotOperator
from referee_parser import RefereeParser
from developer_pass_course import PassCourceGenerate


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
            operator_node.move_to(i, 2 * (-1) ** i, 2 * (-1) ** i, 0, False)
        while operator_node.all_robots_are_free() is False:
            pass


def test_prototyping():
    while rclpy.ok():
        robot_state = observer_node.update_robot_state()

        our_robots_pos = observer_node.get_our_robots_pos()
        enemy_robots_pos = observer_node.get_enemy_robots_pos()
        our_robots_vel = observer_node.get_our_robots_vel()

        # our_robots_angle = observer_node.update_our_robots_vel_angle()
        # our_robots_thita = observer_node.update_our_robots_thita()

        # our_robot_pos = observer_node.get_our_robot_pos(3)
        # enemy_robot_pos = observer_node.get_enemy_robot_pos(3)

        # print(robot_state)

        print(our_robots_pos)
        # print(enemy_robots_pos)
        # print(our_robots_vel)
        # print(our_robots_angle)
        # print(our_robots_thita)

        # print(our_robot_pos)
        # print(enemy_robot_pos)


def static_pass_and_shoot(enemy_num, ally_id1, ally_id2, ally_id3, ally_id4):
    # 静的な相手がいる状態でパスを出すロボットが味方ロボットの3台農地いずれかにパスを出すプログラム

    # ボールの位置座標を指定
    ball_pos = [0, 0]

    # 味方のロボットのうち，使用していないロボットを仮想相手として配置するために配置する位置座標を生成する
    enemy_robot_pos = pass_course_node.position_random_generation(
        enemy_num)
    for i in range(4, 11):
        operator_node.move_to(
            i, enemy_robot_pos[i-4][0], enemy_robot_pos[i-4][1], 0, False)

    # 味方のロボットを配置させる位置座標
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

    # 処理開始時間を記憶
    start_time = time.time()

    # どのロボットにパスを出せばいいかの結果を格納
    pass_to_robot = pass_course_node.eval_easy_to_pass_radius(
        enemy_robot_pos, ball_pos, ally_robots_pos)
    print(pass_to_robot)

    # パスを出すロボットに向けてパスをする
    operator_node.kick_pass(ally_id1, pass_to_robot +
                            1, ball_pos[0], ball_pos[1])

    # パスを成功させる都合上必要な遅延時間
    delay_time = time.time()
    while time.time() - delay_time < 1:
        pass

    # パス目標の位置からずれた位置にパスが出るのでその補正用
    operator_node.move_to_look_ball(
        pass_to_robot + 1, ally_robots_pos[pass_to_robot][0] + 1, ally_robots_pos[pass_to_robot][1])

    # パスを成功させる都合上必要な遅延時間
    delay_time = time.time()
    while time.time() - delay_time < 0.5:
        pass

    # パスを受けるロボットは，ボールをレシーブした後，相手ゴールに向けてシュートする
    operator_node.move_to_receive(
        pass_to_robot + 1, ally_robots_pos[pass_to_robot][0] + 1, ally_robots_pos[pass_to_robot][1])

    while operator_node.all_robots_are_free() is False:
        pass

    # シュートさせる
    operator_node.shoot_to_their_goal(pass_to_robot + 1)

    # 処理時間の計算
    necessary_time = time.time() - start_time
    print(necessary_time)


def dynamic_pass_and_shoot(enemy_num):
    # 動的な相手がいる状態でパスを出すロボットがランダムに配置された味方ロボットの3台農地いずれかにパスを出すプログラム

    # ボールの位置座標を指定
    ball_pos = [0, 0]

    # 相手ロボットの位置座標をランダムで生成
    enemy_robot_pos = pass_course_node.position_random_generation(
        enemy_num)

    # 相手ロボットの移動速度と移動角度をランダムで生成
    enemy_robot_state = pass_course_node.robot_vvector_generation(
        enemy_num)

    # 味方のロボットを配置させる位置座標
    ally_robot_pos = pass_course_node.forward_position_random_generation(
        10 - enemy_num)

    # 相手ロボットを配置する
    for i in range(11 - enemy_num, 11):
        operator_node.move_to(
            i, enemy_robot_pos[i-4][0], enemy_robot_pos[i-4][1], 0, False)

    # パスロボットと味方ロボットを配置する
    operator_node.move_to_look_ball(0, ball_pos[0]-0.3, ball_pos[1])
    for i in range(1, 11-enemy_num):
        operator_node.move_to_look_ball(
            i, ally_robot_pos[i-1][0], ally_robot_pos[i-1][1])
    while operator_node.all_robots_are_free() is False:
        pass

    # 相手ロボットがt病後（設定としては２秒後）に到達しているであろう予測位置座標を計算
    enemy_robot_move_point = [[enemy_robot_pos[i][0] + enemy_robot_state[i][0] * 2 * math.cos(
        enemy_robot_state[i][1]), enemy_robot_pos[i][1] + enemy_robot_state[i][0] * 2 * math.sin(
        enemy_robot_state[i][1])] for i in range(7)]

    # 処理時間計測用の変数
    start_time = time.perf_counter()

    # どのロボットにパスを出せばいいかの結果を格納
    pass_to_robot = pass_course_node.dynamic_enemy_for_pass_to(
        enemy_robot_pos, enemy_robot_state, ball_pos, ally_robot_pos)

    # 計算した場所に移動させる
    for i in range(4, 11):
        operator_node.move_to(
            i, enemy_robot_move_point[i-4][0], enemy_robot_move_point[i-4][1], 0, False)

    # もし戻り値がなし（パスできるロボットがいない）ときの処理
    if pass_to_robot == None:
        print("I can't pass.")
        print("全ロボットの動作停止")
        # 全ロボットを停止させる
        for i in range(11):
            operator_node.stop(i)
    # 何らかの戻り値があった（パスを出すロボットのIDが帰ってきた）とき

    # 戻り値がrobot_idのときの処理
    # else:
    #     print("パスするロボットの番号", pass_to_robot + 1)
    #     # 処理にかかった時間を計算
    #     necessary_time = time.perf_counter() - start_time
    #     print("処理にかかった時間", necessary_time * 1000, "[ms]")

    #     # パスを出してからシュートまでの流れ
    #     # 未だにパスがつながらないため改善する必要あり

    #     # 戻り値で与えられたロボットIDに向けてパスをする
    #     operator_node.kick_pass(0, pass_to_robot +
    #                             1, ball_pos[0], ball_pos[1])

    #     # パスの誤差を考えて受け取るために移動
    #     operator_node.move_to_look_ball(
    #         pass_to_robot + 1, ally_robot_pos[pass_to_robot][0] + 1, ally_robot_pos[pass_to_robot][1])

    #     # 相手ゴールに向かってシュートする
    #     operator_node.shoot_to_their_goal(pass_to_robot + 1)

    # 戻り値がパスをするロボットの位置座標とrobot_idのとき
    else:
        print("戻り値", pass_to_robot)
        # 処理にかかった時間を計算
        necessary_time = time.perf_counter() - start_time
        print("処理にかかった時間", necessary_time * 1000, "[ms]")

        # 戻り値で与えられた座標を向いてドリブル
        operator_node.dribble_to(0, pass_to_robot[0][0], pass_to_robot[0][1])

        delay_time = time.time()
        while time.time() - delay_time < 1:
            pass

        # 味方ロボットがいる座標にパス
        operator_node.shoot_to(0, pass_to_robot[0][0], pass_to_robot[0][1])

        # while operator_node.all_robots_are_free() is False:
        #     pass

        # operator_node.stop(0)
        # 相手ゴールに向かってシュートする
        operator_node.move_to_receive(
            pass_to_robot[1]+1, pass_to_robot[0][0], pass_to_robot[0][1])
        # operator_node.shoot_to_their_goal(pass_to_robot[1] + 1)


def goalie_save(goalie_robot_id):
    # goalie（ゴールキーパー）がセーブするためのプログラム

    # 味方のゴールキーパーを自ゴールの中心地点に配置する
    operator_node.move_to_look_ball(goalie_robot_id, -6, 3)

    # 相手ロボットからゴールの両端までの範囲をシュート可能ゾーンに設定する

    # それ以外の範囲にボールが進んでいたらその進行先にあるロボットに対象を移す
    # シュート可能ゾーンに入っている場合は相手ロボットの存在次第で挙動を変える
    # いなければシュートとして扱う
    # いれば関係性を見つける


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
    test_prototyping()

    # static_pass_and_shoot(7, 0, 1, 2, 3)
    # dynamic_pass_and_shoot(7)
    # goalie_save(10)


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
    args = arg_parser.parse_args()

    rclpy.init(args=None)

    operator_node = RobotOperator(target_is_yellow=args.yellow)
    observer_node = FieldObserver(args.yellow)
    referee_node = RefereeParser(args.yellow, args.invert)
    pass_course_node = PassCourceGenerate()

    # すべてのロボットの衝突回避を解除
    for i in range(16):
        operator_node.disable_avoid_obstacles(i)

    executor = MultiThreadedExecutor()
    executor.add_node(operator_node)
    executor.add_node(observer_node)
    executor.add_node(referee_node)

    # エグゼキュータは別スレッドでspinさせ続ける
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    try:
        main()
    except KeyboardInterrupt:
        pass

    executor.shutdown()
    rclpy.shutdown()
