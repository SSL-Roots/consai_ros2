#!/usr/bin/env python3
# coding: UTF-8

import math
import random
import numpy as np


def eval_easy_to_pass_radius(enemy_robot_pos, ball_pos, ally_robot_pos):
    # どのロボットにパスを出すべきかの評価をする関数
    enemy_robot_pos = np.array(enemy_robot_pos)
    ally_robot_pos = np.array(ally_robot_pos)
    ball_pos = np.array(ball_pos)

    # 敵ロボットの動作予想範囲の半径を1に設定
    r = 1
    count = 0

    # ボールより敵のゴール側にいるロボットのみに絞る
    forward_enemy_robot_pos = [enemy_robot_pos[i] for i in range(
        len(enemy_robot_pos)) if not ball_pos[0] > enemy_robot_pos[i][0]]
    print(forward_enemy_robot_pos)
    forward_enemy_robot_pos = np.array(forward_enemy_robot_pos)

    # ボールから味方のロボットまでの距離を計測し，近い順にソートする
    diff_ball_to_robot = [[ball_pos[0] - ally_robot_pos[i][0],
                           ball_pos[1] - ally_robot_pos[i][1]] for i in range(len(ally_robot_pos))]
    print(diff_ball_to_robot)
    receive_dist_ally_robot = [[math.sqrt((diff_ball_to_robot[i][0]) ** 2 + (
        diff_ball_to_robot[i][1]) ** 2), i] for i in range(len(ally_robot_pos))]
    receive_dist_ally_robot = sorted(receive_dist_ally_robot)

    # ボールから敵ロボットの距離を計測する
    # diff_enemy_to_ball = [[ball_pos[0] - enemy_robot_pos[i][0], ball_pos[1] -
    #                        enemy_robot_pos[i][1]] for i in range(len(enemy_robot_pos))]

    print(receive_dist_ally_robot)

    # 解を求めよう
    # for i in range(len(ally_robot_pos)):
    #     a = diff_ball_to_robot[receive_dist_ally_robot[i][1]][0] ** 2 + \
    #         diff_ball_to_robot[receive_dist_ally_robot[i][1]][1] ** 2
    #     for j in range(len(forward_enemy_robot_pos)):
    #         b = diff_ball_to_robot[receive_dist_ally_robot[i][1]][0]*diff_enemy_to_ball[j][0] + \
    #             diff_ball_to_robot[receive_dist_ally_robot[i][1]
    #                                ][1]*diff_enemy_to_ball[j][1]
    #         c = diff_enemy_to_ball[j][0]**2 + \
    #             diff_enemy_to_ball[j][1]**2-r**2

    #         factor1 = c
    #         factor2 = a + 2 * b + c

    #         if (factor1 >= 0 and factor2 <= 0) or (factor1 <= 0 and factor2 >= 0):
    #             # 敵ロボットの動作予想範囲にどれか一つにでもパスコースが被ったときの処理
    #             count = 0
    #             print('だめポ')
    #             break
    #         else:
    #             # 敵ロボットの影響を受けなければ以下の処理へ
    #             count += 1
    #             if count == len(forward_enemy_robot_pos):
    #                 print('いけそう')
    #                 # return ally_robot_pos[receive_dist_ally_robot[i][1]]
    #                 return receive_dist_ally_robot[i][1]

    # 解を求める前のとこから↓
    # 味方ロボットとボールまでの直線の方程式
    for i in range(len(ally_robot_pos)):
        # x差分
        x = ally_robot_pos[receive_dist_ally_robot[i][1]][0]-ball_pos[0]
        # y差分
        y = ally_robot_pos[receive_dist_ally_robot[i][1]][1]-ball_pos[1]

        a = y/x
        b = y

        # 解の公式
        for j in range(len(forward_enemy_robot_pos)):
            a_kai = a ** 2 + 1
            b_kai = 2 * \
                ((b - forward_enemy_robot_pos[j][1])
                 * a - forward_enemy_robot_pos[j][0])
            c_kai = ball_pos[0] ** 2 + (b - ball_pos[1]) ** 2

            D = b_kai ** 2 - 4 * a_kai * c_kai

            print(D)
            if D < 0:
                print('いけそ')
                return receive_dist_ally_robot[i][1]
            else:
                print('だめぽ')


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
