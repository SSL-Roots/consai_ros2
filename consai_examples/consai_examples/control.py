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
import math
import threading
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor
from robot_operator import RobotOperator


def test_move_to(max_velocity_xy=None):
    # フィールド上の全ロボットが、フィールドを上下(y軸)に往復する
    for i in range(16):
        operator_node.move_to(i, -5.0 + 0.5 * i, 4.0, math.pi * 0.5, False, max_velocity_xy)

    # 全てのロボットがフリー（目的地に到着 or 常時制御中）になるまで待機
    while operator_node.all_robots_are_free() is False:
        pass

    # 制御継続フラグがTrueなので、ロボットはすぐにフリー状態になる
    for i in range(16):
        operator_node.move_to(i, -5.0 + 0.5 * i, -4.0, -math.pi * 0.5, True, max_velocity_xy)

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
        operator_node.chase_ball(i, 0.2 + 0.2*i, 0.05 * i*i, 0.1 * math.pi * i, True, False)

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
                operator_node.move_to_normalized(i, -x, 1.0 - 2.0 * i / 16.0, 0.0, False)

            while operator_node.all_robots_are_free() is False:
                pass

            for i in range(16):
                operator_node.move_to_normalized(i, x, 1.0 - 2.0 * i / 16.0, 0.0, False)

            while operator_node.all_robots_are_free() is False:
                pass

    # 上下
    if test_y:
        for y in pattern:
            for i in range(16):
                operator_node.move_to_normalized(i, -0.9 + 2.0 * i / 16.0, y, math.pi * 0.5, False)

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
            operator_node.move_to_line(i, p1_x, p1_y, p2_x, p2_y, i*0.4, 0, False)

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
    operator_node.move_to_line_to_defend_our_goal(2, -middle_x, y, -middle_end_x, y, True)

    # 3番はディフェンスエリアの下左側に
    # 4番はディフェンスエリアの下右側に
    operator_node.move_to_line_to_defend_our_goal(3, -x, -y, -end_x, -y, True)
    operator_node.move_to_line_to_defend_our_goal(4, -middle_x, -y, -middle_end_x, -y, True)

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
    operator_node.move_to_line_to_defend_our_goal(6, -x, middle_y, -x, middle_end_y, True)
    operator_node.move_to_line_to_defend_our_goal(7, -x, -y, -x, -end_y, True)
    operator_node.move_to_line_to_defend_our_goal(8, -x, -middle_y, -x, -middle_end_y, True)

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

def main():
    # 実行したい関数のコメントを外してください
    test_move_to()
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
