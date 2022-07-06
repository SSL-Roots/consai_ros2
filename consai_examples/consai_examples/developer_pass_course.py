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
    r = 0.4
    cnt = 0

    # ボールより敵のゴール側にいるロボットのみに絞る
    after_sort_enemy_robot = _forward_enemy_robot(enemy_robot_pos, ball_pos)

    # ボールから味方のロボットまでの距離を計測し，近い順にソートする
    dist_ally_robot = _sort_ball_from_ally_robot_distance(
        ball_pos, ally_robot_pos)

    enemy_robot_cnt = len(after_sort_enemy_robot)
    print(enemy_robot_cnt)

    # 解を求めよう
    # 味方ロボットとボールまでの直線の方程式
    for i in range(len(ally_robot_pos)):
        # x差分
        x = ally_robot_pos[dist_ally_robot[i][1]][0]-ball_pos[0]
        # y差分
        y = ally_robot_pos[dist_ally_robot[i][1]][1]-ball_pos[1]

        a = y/x
        b = y

        # 判別式
        for j in range(enemy_robot_cnt):
            a_kai = a ** 2 + 1
            b_kai = 2 * \
                ((b - after_sort_enemy_robot[j][1])
                 * a - after_sort_enemy_robot[j][0])
            c_kai = after_sort_enemy_robot[j][0] ** 2 + \
                (b - after_sort_enemy_robot[j][1]) ** 2 - r ** 2

            D = b_kai ** 2 - 4 * a_kai * c_kai

            print(D)
            if D < 0 and cnt >= enemy_robot_cnt - 1:
                print('いけそ')
                cnt = 0
                return dist_ally_robot[i][1]
            elif D < 0 and cnt < enemy_robot_cnt - 1:
                cnt += 1
                print(cnt)
            else:
                print('だめぽ')
                cnt = 0
                print(after_sort_enemy_robot[j])
                break


def dynamic_enemy_for_pass_to(now_enemy_robot_pos, enemy_robot_vvector, ball_pos, ally_robot_pos):
    # 動的な敵に対してどの味方ロボットにパスを出すべきかの評価をする関数
    # now_enemy_robot_pos = np.array(now_enemy_robot_pos)
    # enemy_robot_vvector = np.array(enemy_robot_vvector)
    # ally_robot_pos = np.array(ally_robot_pos)
    # ball_pos = np.array(ball_pos)

    # 楕円（敵ロボットの移動範囲）の短半径を設定
    short_r = 0.4

    cnt = 0
    dt = 2.0

    # 長半径を求めるコード
    long_r = [dt * enemy_robot_vvector[i][0] +
              short_r for i in range(len(now_enemy_robot_pos))]
    print(long_r)

    enemy_robot_state = [[now_enemy_robot_pos[i], long_r[i]]
                         for i in range(len(now_enemy_robot_pos))]

    print(enemy_robot_state)

    after_sort_enemy_robot = _dynamic_forward_enemy_robot(
        enemy_robot_state, ball_pos, enemy_robot_vvector)
    print(after_sort_enemy_robot)

    # ボールから味方のロボットまでの距離を計測し，近い順にソートする
    dist_ally_robot = _sort_ball_from_ally_robot_distance(
        ball_pos, ally_robot_pos)

    enemy_robot_cnt = len(after_sort_enemy_robot)
    print(enemy_robot_cnt)

    # 解を求める
    # 味方ロボットとボールまでの直線の方程式
    for i in range(len(ally_robot_pos)):
        # x差分
        x = ally_robot_pos[dist_ally_robot[i][1]][0]-ball_pos[0]
        # y差分
        y = ally_robot_pos[dist_ally_robot[i][1]][1]-ball_pos[1]

        a = y/x
        b = y

        # 判別式
        for j in range(enemy_robot_cnt):
            a_kai = a ** 2 + 1
            b_kai = 2 * \
                ((b - after_sort_enemy_robot[j][0][1])
                 * a - after_sort_enemy_robot[j][0][0])
            c_kai = after_sort_enemy_robot[j][0][0] ** 2 + \
                (b - after_sort_enemy_robot[j][0][1]) ** 2 - \
                after_sort_enemy_robot[j][1] ** 2

            D = b_kai ** 2 - 4 * a_kai * c_kai

            print(D)
            if D < 0 and cnt >= enemy_robot_cnt - 1:
                print('いけそ')
                cnt = 0
                return dist_ally_robot[i][1]
            elif D < 0 and cnt < enemy_robot_cnt - 1:
                cnt += 1
                print(cnt)
            else:
                print('だめぽ')
                cnt = 0
                print(after_sort_enemy_robot[j])
                break


def position_random_generation(how_many: int):
    generating_pos = []
    if how_many > 1:
        while len(generating_pos) < how_many:
            generating_pos_x = random.randint(-6, 6)
            generating_pos_y = random.randint(-4, 4)
            generating_pos.append([generating_pos_x, generating_pos_y])
            if _has_duplicates2(generating_pos) == 1:
                generating_pos.pop(-1)

    elif how_many == 1:
        generating_pos_x = random.randint(-6, 6)
        generating_pos_y = random.randint(-4, 4)
        generating_pos.append(generating_pos_x)
        generating_pos.append(generating_pos_y)

    return generating_pos


def _has_duplicates2(seq):
    seen = []
    unique_list = [x for x in seq if x not in seen and not seen.append(x)]
    return len(seq) != len(unique_list)


def _forward_enemy_robot(enemy_robot_pos, ball_pos):
    # ボールより敵のゴール側にいるロボットのみに絞る関数
    forward_enemy_robot_pos = [enemy_robot_pos[i] for i in range(
        len(enemy_robot_pos)) if not ball_pos[0] > enemy_robot_pos[i][0]]
    print(forward_enemy_robot_pos)
    forward_enemy_robot_pos = np.array(forward_enemy_robot_pos)
    return forward_enemy_robot_pos


def _dynamic_forward_enemy_robot(enemy_robot_state, ball_pos, enemy_robot_vvector):
    # ボールより動いている敵のゴール側にいるロボットのみに絞る関数
    forward_enemy_robot_pos = [enemy_robot_state[i] for i in range(
        len(enemy_robot_state)) if not ball_pos[0] > (enemy_robot_state[i][0][0] + enemy_robot_state[i][1] * math.cos(enemy_robot_vvector[i][1]))]
    print(forward_enemy_robot_pos)
    forward_enemy_robot_pos = np.array(forward_enemy_robot_pos)
    return forward_enemy_robot_pos


def _sort_ball_from_ally_robot_distance(ball_pos, ally_robot_pos):
    # ボールから味方のロボットまでの距離を計算し，近い順にソートする関数
    diff_ball_to_robot = [[ball_pos[0] - ally_robot_pos[i][0],
                           ball_pos[1] - ally_robot_pos[i][1]] for i in range(len(ally_robot_pos))]
    print(diff_ball_to_robot)
    receive_dist_ally_robot = [[math.sqrt((diff_ball_to_robot[i][0]) ** 2 + (
        diff_ball_to_robot[i][1]) ** 2), i] for i in range(len(ally_robot_pos))]
    receive_dist_ally_robot = sorted(receive_dist_ally_robot)
    print(receive_dist_ally_robot)
    return receive_dist_ally_robot


def robot_vvector_generation(robot_cnt: int):
    generating_vvec = []
    for i in range(robot_cnt):
        generating_vvec_speed = random.uniform(0, 2)
        generating_vvec_theta = random.uniform(0, 360)
        generating_vvec.append([generating_vvec_speed, generating_vvec_theta])

    return generating_vvec
