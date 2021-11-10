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
from robot_operator import RobotOperator
import math
import rclpy
from rclpy.executors import SingleThreadedExecutor
import time

def test_move_to():
    # フィールド上の全ロボットが、フィールドを上下(y軸)に往復する
    for i in range(16):
        operator_node.move_to(i, -5.0 + 0.5 * i, 4.0, math.pi * 0.5, False)

    while operator_node.all_robots_are_free() is False:
        executor.spin_once(1)  # タイムアウト入れないとフリーズする

    for i in range(16):
        operator_node.move_to(i, -5.0 + 0.5 * i, -4.0, -math.pi * 0.5, True)

    while operator_node.all_robots_are_free() is False:
        executor.spin_once(1)  # タイムアウト入れないとフリーズする

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
        executor.spin_once(1)  # タイムアウト入れないとフリーズする

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
            executor.spin_once(1)  # タイムアウト入れないとフリーズする

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
            executor.spin_once(1)  # タイムアウト入れないとフリーズする

def test_chase_ball():
    # ボールの右側に、2次関数のように並ぶ
    for i in range(16):
        operator_node.chase_ball(i, 0.2 + 0.2*i, 0.05 * i*i, 0.1 * math.pi * i, True, False)

    while operator_node.all_robots_are_free() is False:
        executor.spin_once(1)  # タイムアウト入れないとフリーズする

    # ボールを見る
    for i in range(16):
        operator_node.chase_ball(i, 0.2 + 0.2*i, 0.05 * i*i, 0.0, False, True)

    while operator_node.all_robots_are_free() is False:
        executor.spin_once(1)  # タイムアウト入れないとフリーズする

def test_chase_robot():
    # 全ロボットが、別のチームカラーの同じIDのロボットの左上に移動する
    for i in range(16):
        operator_node.chase_robot(
            i, not operator_node.target_is_yellow(), i, -0.2, 0.2, 0.0, True, False)

    while operator_node.all_robots_are_free() is False:
        executor.spin_once(1)  # タイムアウト入れないとフリーズする

    # 左横に移動する
    for i in range(16):
        operator_node.chase_robot(
            i, not operator_node.target_is_yellow(), i, -0.2, 0.0, 0.0, False, True)

    while operator_node.all_robots_are_free() is False:
        executor.spin_once(1)  # タイムアウト入れないとフリーズする

def test_for_config_pid(pattern=[0.1, 0.3, 0.5, 0.7, 0.9], test_x=False, test_y=False, test_theta=False):
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
                executor.spin_once(1)  # タイムアウト入れないとフリーズする

            for i in range(16):
                operator_node.move_to_normalized(i, x, 1.0 - 2.0 * i / 16.0, 0.0, False)

            while operator_node.all_robots_are_free() is False:
                executor.spin_once(1)  # タイムアウト入れないとフリーズする

    # 上下
    if test_y:
        for y in pattern:
            for i in range(16):
                operator_node.move_to_normalized(i, -0.9 + 2.0 * i / 16.0, y, math.pi * 0.5, False)

            while operator_node.all_robots_are_free() is False:
                executor.spin_once(1)  # タイムアウト入れないとフリーズする

            for i in range(16):
                operator_node.move_to_normalized(i, -0.9 + 2.0 * i / 16.0, -y, math.pi * 0.5, False)

            while operator_node.all_robots_are_free() is False:
                executor.spin_once(1)  # タイムアウト入れないとフリーズする

    if test_theta:
        for theta in pattern:
            for i in range(16):
                operator_node.move_to_normalized(i, -0.9 + 2.0 * i / 16.0, 0.5, math.pi * theta, False)

            while operator_node.all_robots_are_free() is False:
                executor.spin_once(1)  # タイムアウト入れないとフリーズする

            for i in range(16):
                operator_node.move_to_normalized(i, -0.9 + 2.0 * i / 16.0, 0.5, -math.pi * theta, False)

            while operator_node.all_robots_are_free() is False:
                executor.spin_once(1)  # タイムアウト入れないとフリーズする

def test_shoot(target_x, target_y):
    # ID0ロボットがフィールド中央に移動して、ターゲット座標に向かってボールを蹴る
    pos_x = 0.0
    pos_y = 0.0
    operator_node.shoot(0, pos_x, pos_y, target_x, target_y)

    while operator_node.all_robots_are_free() is False:
        executor.spin_once(1)  # タイムアウト入れないとフリーズする

def test_pass_two_robots():
    # 2台のロボットでパスし合う
    operator_node.kick_pass(0, 1, -3.0, 0.0)
    operator_node.kick_pass(1, 0, 3.0, 0.0)

    while operator_node.all_robots_are_free() is False:
        executor.spin_once(1)  # タイムアウト入れないとフリーズする

def test_pass_four_robots():
    # 4台のロボットでパスし合う

    operator_node.kick_pass(0, 1, 3.0, 3.0)
    operator_node.kick_pass(1, 2, 3.0, -3.0)
    operator_node.kick_pass(2, 3, -3.0, 3.0)
    operator_node.kick_pass(3, 0, -3.0, -3.0)

    while operator_node.all_robots_are_free() is False:
        executor.spin_once(1)  # タイムアウト入れないとフリーズする

def test_stop_robots():
    # フィールド上の全ロボットが、フィールドを上下(y軸)に往復する
    # 動作の途中でロボットを停止させる

    # y軸上方向へ移動
    for i in range(16):
        operator_node.move_to(i, -5.0 + 0.5 * i, 4.0, math.pi * 0.5, False)
    start_time = time.time()

    # 数秒間待機
    while time.time() - start_time < 2.0:
        executor.spin_once(1)  # タイムアウト入れないとフリーズする

    # 5台のロボットを停止
    print("ロボットの動作停止！")
    for i in range(5):
        operator_node.stop(i)
    # 動作完了まで待機
    while operator_node.all_robots_are_free() is False:
        executor.spin_once(1)  # タイムアウト入れないとフリーズする

    # y軸下方向へ移動
    for i in range(16):
        operator_node.move_to(i, -5.0 + 0.5 * i, -4.0, math.pi * 0.5, False)
    start_time = time.time()

    # 数秒間待機
    while time.time() - start_time < 2.0:
        executor.spin_once(1)  # タイムアウト入れないとフリーズする

    # 5台のロボットを停止
    print("ロボットの動作停止！")
    for i in range(5):
        operator_node.stop(i)
    # 動作完了まで待機
    while operator_node.all_robots_are_free() is False:
        executor.spin_once(1)  # タイムアウト入れないとフリーズする

def main():
    # 実行したい関数のコメントを外してください
    # test_move_to()
    # test_move_to_normalized(3)
    # test_chase_ball()
    # test_chase_robot()
    # test_for_config_pid(test_x=True)
    # test_shoot(1.0, 0.0)
    # test_pass_two_robots()
    # test_pass_four_robots()
    test_stop_robots()

if __name__ == '__main__':
    arg_parser = argparse.ArgumentParser()
    arg_parser.add_argument('--yellow',
        default=False,
        action='store_true',
        help='yellowロボットを動かす場合にセットする')
    args = arg_parser.parse_args()

    rclpy.init(args=None)

    operator_node = RobotOperator(target_is_yellow=args.yellow)

    executor = SingleThreadedExecutor()
    executor.add_node(operator_node)

    try:
        main()
    except KeyboardInterrupt:
        pass

    executor.shutdown()
    rclpy.shutdown()
