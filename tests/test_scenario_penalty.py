# Copyright 2024 Roots
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

import math
import time

from rcst.communication import Communication
from rcst import calc
from rcst.ball import Ball
from rcst.robot import RobotDict

from utils_for_placement import init_placement

# TODO: Implement a motion dribbling the ball to the opponent's side.
# Issue: https://github.com/SSL-Roots/consai_ros2/issues/186

# def test_our_penalty_shoot(rcst_comm: Communication):
#     rcst_comm.send_empty_world()
#     rcst_comm.send_ball(-2, 0)
#     rcst_comm.send_blue_robot(1, -2.5, 0.0, math.radians(0))

#     rcst_comm.observer.reset()
#     rcst_comm.change_referee_command('STOP', 3.0)
#     rcst_comm.change_referee_command('PREPARE_PENALTY_BLUE', 3.0)
#     rcst_comm.change_referee_command('NORMAL_START', 5.0)

#     assert rcst_comm.observer.goal().ball_has_been_in_positive_goal() is True


def test_their_penalty_defend(rcst_comm: Communication):
    rcst_comm.send_empty_world()
    rcst_comm.send_ball(2, 0)
    rcst_comm.send_blue_robot(0, -5.5, 0.0, math.radians(0))
    rcst_comm.send_yellow_robot(0, 2.1, 0.0, math.radians(180))

    rcst_comm.observer.reset()
    rcst_comm.change_referee_command('STOP', 3.0)
    rcst_comm.change_referee_command('PREPARE_PENALTY_YELLOW', 3.0)
    rcst_comm.change_referee_command('NORMAL_START', 1.0)

    # Shoot to our goal.
    rcst_comm.send_ball(0, 0, -6.0, 0.5)
    time.sleep(5)

    assert rcst_comm.observer.goal().ball_has_been_in_negative_goal() is False


# def blue_robot_are_behind_ball(
#         ball: Ball, blue_robots: RobotDict, yellow_robots: RobotDict) -> bool:
#     DISTANCE = 1.0  # meters
#     KICKER_ID = 1

#     for robot in blue_robots.values():
#         if robot.id == KICKER_ID:
#             continue

#         if robot.x > ball.x - DISTANCE:
#             return False

#     return True

# TODO: Fix the issue: https://github.com/SSL-Roots/consai_ros2/issues/185
# def test_robots_be_behind_ball(rcst_comm: Communication):
#     rcst_comm.send_empty_world()
#     for i in range(11):
#         rcst_comm.send_blue_robot(i, -1.0, 3.0 - i * 0.5, math.radians(0))
#     time.sleep(3)  # Wait for the robots to be placed.

#     # Kicker robot.
#     rcst_comm.send_blue_robot(1, -2.2, 0.0, math.radians(0))
#     rcst_comm.send_ball(-2, 0)

#     rcst_comm.change_referee_command('STOP', 3.0)
#     rcst_comm.change_referee_command('PREPARE_PENALTY_BLUE', 0.0)

#     rcst_comm.observer.customized().register_sticky_true_callback(
#         "test", blue_robot_are_behind_ball)

#     success = False
#     for _ in range(10):
#         if rcst_comm.observer.customized().get_result("test"):
#             success = True
#             break
#         time.sleep(1)
#     assert success is True


def test_robots_be_behind_ball_for_their_penalty(rcst_comm: Communication):
    rcst_comm.send_empty_world()
    for i in range(11):
        rcst_comm.send_blue_robot(i, 1.0, 3.0 - i * 0.5, math.radians(0))

    rcst_comm.send_ball(2, 0)

    rcst_comm.change_referee_command('STOP', 3.0)
    rcst_comm.change_referee_command('PREPARE_PENALTY_YELLOW', 0.0)

    def blue_robot_are_behind_ball(
            ball: Ball, blue_robots: RobotDict, yellow_robots: RobotDict) -> bool:
        DISTANCE = 1.0  # meters
        GOALIE_ID = 0

        for robot in blue_robots.values():
            if robot.id == GOALIE_ID:
                continue

            if robot.x < ball.x + DISTANCE:
                return False
        return True

    rcst_comm.observer.customized().register_sticky_true_callback(
        "test", blue_robot_are_behind_ball)

    success = False
    for _ in range(10):
        if rcst_comm.observer.customized().get_result("test"):
            success = True
            break
        time.sleep(1)
    assert success is True


def test_goalie_on_the_goal_line(rcst_comm: Communication):
    rcst_comm.send_empty_world()
    rcst_comm.send_blue_robot(0, -4, 0.0, math.radians(0))

    rcst_comm.send_ball(2, 0)

    rcst_comm.change_referee_command('STOP', 3.0)
    rcst_comm.change_referee_command('PREPARE_PENALTY_YELLOW', 0.0)

    def goalie_on_the_goal_line(
            ball: Ball, blue_robots: RobotDict, yellow_robots: RobotDict) -> bool:
        GOALIE_ID = 0
        HALF_FIELD_LENGTH = -6.0  # meters
        HALF_GOAL_WIDTH = 0.9  # meters
        ROBOT_RADIUS = 0.09  # meters

        for robot in blue_robots.values():
            if robot.id != GOALIE_ID:
                continue

            if abs(robot.x - HALF_FIELD_LENGTH) < ROBOT_RADIUS and \
                    abs(robot.y) < HALF_GOAL_WIDTH:
                return True
        return False

    rcst_comm.observer.customized().register_sticky_true_callback(
        "test", goalie_on_the_goal_line)

    success = False
    for _ in range(10):
        if rcst_comm.observer.customized().get_result("test"):
            success = True
            break
        time.sleep(1)
    assert success is True

def test_ボールプレースメント時に進入禁止エリアに侵入しないこと(rcst_comm: Communication):
    target_x = 0.0  # meters
    target_y = 0.0  # meters
    ball_x = -4.0  # meters
    ball_y = 0.0  # meters
    init_placement(
        rcst_comm, color="yellow", target_x=target_x, target_y=target_y,num_of_robots=11, ball_x=ball_x, ball_y=ball_y
    )
    time.sleep(3)

    def blue_robots_are_out_of_placement_position(
            ball: Ball, blue_robots: RobotDict, yellow_robots: RobotDict) -> bool:
        DISTANCE = 0.5  # meters
        GOALIE_ID = 0

        for robot in blue_robots.values():
            if robot.id == GOALIE_ID:
                continue

            distance = calc.distance_point_c_to_line_ab(
                target_x, target_y, ball.x, ball.y, robot.x, robot.y
            )

            if distance < DISTANCE:
                return False
        return True

    rcst_comm.observer.customized().register_sticky_true_callback(
        "test", blue_robots_are_out_of_placement_position)

    success = False
    for _ in range(10):
        if rcst_comm.observer.customized().get_result("test"):
            success = True
            break
        time.sleep(1)
    assert success is True


# def test_インプレイ中にディフェンスエリアに侵入しないこと(rcst_comm: Communication):
#     rcst_comm.send_empty_world()
#     robot_num = 11
#     for i in range(robot_num):
#         rcst_comm.send_blue_robot(i, -1.0, 3.0 - 0.5*i, 0.0)
#     rcst_comm.send_ball(4.5, 0)

#     rcst_comm.change_referee_command('STOP', 3.0)
#     rcst_comm.change_referee_command('FORCE_START', 0.0)

#     def robot_is_in_the_defense_area(
#             ball: Ball, blue_robots: RobotDict, yellow_robots: RobotDict) -> bool:
#         GOALIE_ID = 0
#         HALF_FIELD_LENGTH = 6.0  # meters
#         DEFENSE_WIDTH = 1.8  # meters
#         DEFENSE_HALF_HEIGHT = 1.8  # meters
#         ROBOT_RADIUS = 0.09  # meters

#         for robot in blue_robots.values():
#             if robot.id == GOALIE_ID:
#                 continue

#             if (abs(robot.y) - ROBOT_RADIUS) < DEFENSE_HALF_HEIGHT and \
#                     (abs(robot.x) + ROBOT_RADIUS) > (HALF_FIELD_LENGTH - DEFENSE_WIDTH):
#                 return True
#         return False

#     rcst_comm.observer.customized().register_sticky_true_callback(
#         "test", robot_is_in_the_defense_area)

#     success = True
#     for _ in range(5):
#         if rcst_comm.observer.customized().get_result("test"):
#             success = False
#             break
#         time.sleep(1)
#     assert success is True
    
#     rcst_comm.send_ball(4.5, 2.0)
#     for _ in range(5):
#         if rcst_comm.observer.customized().get_result("test"):
#             success = False
#             break
#         time.sleep(1)
#     assert success is True

#     rcst_comm.send_ball(5.5, -2.0)
#     for _ in range(5):
#         if rcst_comm.observer.customized().get_result("test"):
#             success = False
#             break
#         time.sleep(1)
#     assert success is True
